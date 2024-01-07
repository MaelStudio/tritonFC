/*
* Capture ESP32 Cam JPEG images into a AVI file and store on SD
* matches file writes to the SD card sector size.
* AVI files stored on the SD card can also be selected and streamed to a browser.
*
* s60sc 2020, 2022
*/

#include "appGlobals.h"

bool forceRecord = false; // Recording enabled by rec button

// status & control fields
int maxFrames = 20000; // maximum number of frames in video before auto close 
uint8_t FPS = 0;
uint8_t fsizePtr; // index to frameData[]
uint8_t minSeconds = 5; // default min video length (includes POST_MOTION_TIME)
bool doRecording = true; // whether to capture to SD or not 
uint8_t xclkMhz = 20; // camera clock rate MHz

// header and reporting info
static uint32_t vidSize; // total video size
static uint16_t frameCnt;
static uint32_t startTime; // total overall time
static uint32_t dTimeTot; // total frame decode/monitor time
static uint32_t fTimeTot; // total frame buffering time
static uint32_t wTimeTot; // total SD write time
static uint32_t oTime; // file opening time
static uint32_t cTime; // file closing time
static uint32_t sTime; // file streaming time

uint8_t frameDataRows = 14; // number of frame sizes
static uint16_t frameInterval; // units of 0.1ms between frames

// SD card storage
uint8_t iSDbuffer[(RAMSIZE + CHUNK_HDR) * 2];
static size_t highPoint;
static File aviFile;
static char aviFileName[FILE_NAME_LEN];

// SD playback
static File playbackFile;
static char partName[FILE_NAME_LEN];
static size_t readLen;
static uint8_t recFPS;
static uint32_t recDuration;
static uint8_t saveFPS = 99;
bool doPlayback = false; // controls playback

// task control
TaskHandle_t captureHandle = NULL;
TaskHandle_t playbackHandle = NULL;
static SemaphoreHandle_t readSemaphore;
static SemaphoreHandle_t playbackSemaphore;
SemaphoreHandle_t frameSemaphore[MAX_STREAMS] = {NULL};
SemaphoreHandle_t aviMutex = NULL;
static volatile bool isPlaying = false; // controls playback on app
bool isCapturing = false;
bool stopPlayback = false; // controls if playback allowed

/**************** timers & ISRs ************************/

static void IRAM_ATTR frameISR() {
  // interrupt at current frame rate
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (isPlaying) xSemaphoreGiveFromISR (playbackSemaphore, &xHigherPriorityTaskWoken ); // notify playback to send frame
  vTaskNotifyGiveFromISR(captureHandle, &xHigherPriorityTaskWoken); // wake capture task to process frame
  if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

void controlFrameTimer(bool restartTimer) {
  // frame timer control, timer 3 so dont conflict with cam
  static hw_timer_t* frameTimer = NULL;
  // stop current timer
  if (frameTimer) {
    timerAlarmDisable(frameTimer);   
    timerDetachInterrupt(frameTimer); 
    timerEnd(frameTimer);
  }
  if (restartTimer) {
    // (re)start timer 3 interrupt per required framerate
    frameTimer = timerBegin(3, 8000, true); // 0.1ms tick
    frameInterval = 10000 / FPS; // in units of 0.1ms 
    LOG_DBG("Frame timer interval %ums for FPS %u", frameInterval/10, FPS); 
    timerAlarmWrite(frameTimer, frameInterval, true); 
    timerAlarmEnable(frameTimer);
    timerAttachInterrupt(frameTimer, &frameISR, true);
  }
}

/**************** capture AVI  ************************/

static void openAvi() {
  // derive filename from date & time, store in date folder
  // time to open a new file on SD increases with the number of files already present
  oTime = millis();
  dateFormat(partName, sizeof(partName), true);
  STORAGE.mkdir(partName); // make date folder if not present
  dateFormat(partName, sizeof(partName), false);
  // open avi file with temporary name 
  aviFile = STORAGE.open(AVITEMP, FILE_WRITE);
  oTime = millis() - oTime;
  LOG_DBG("File opening time: %ums", oTime);
  startAudio();
  startTelemetry();
  // initialisation of counters
  startTime = millis();
  frameCnt = fTimeTot = wTimeTot = dTimeTot = vidSize = 0;
  highPoint = AVI_HEADER_LEN; // allot space for AVI header
  prepAviIndex();
}

static void saveFrame(camera_fb_t* fb) {
  // save frame on SD card
  uint32_t fTime = millis();
  // align end of jpeg on 4 byte boundary for AVI
  uint16_t filler = (4 - (fb->len & 0x00000003)) & 0x00000003; 
  size_t jpegSize = fb->len + filler;
  // add avi frame header
  memcpy(iSDbuffer+highPoint, dcBuf, 4); 
  memcpy(iSDbuffer+highPoint+4, &jpegSize, 4);
  highPoint += CHUNK_HDR;
  if (highPoint >= RAMSIZE) {
    // marker overflows buffer
    highPoint -= RAMSIZE;
    aviFile.write(iSDbuffer, RAMSIZE);
    // push overflow to buffer start
    memcpy(iSDbuffer, iSDbuffer+RAMSIZE, highPoint);
  }
  // add frame content
  size_t jpegRemain = jpegSize;
  uint32_t wTime = millis();
  while (jpegRemain >= RAMSIZE - highPoint) {
    // write to SD when RAMSIZE is filled in buffer
    memcpy(iSDbuffer+highPoint, fb->buf + jpegSize - jpegRemain, RAMSIZE - highPoint);
    aviFile.write(iSDbuffer, RAMSIZE);
    jpegRemain -= RAMSIZE - highPoint;
    highPoint = 0;
  } 
  wTime = millis() - wTime;
  wTimeTot += wTime;
  LOG_DBG("SD storage time %u ms", wTime); 
  // whats left or small frame
  memcpy(iSDbuffer+highPoint, fb->buf + jpegSize - jpegRemain, jpegRemain);
  highPoint += jpegRemain;
  
  buildAviIdx(jpegSize); // save avi index for frame
  vidSize += jpegSize + CHUNK_HDR;
  frameCnt++; 
  fTime = millis() - fTime - wTime;
  fTimeTot += fTime;
  LOG_DBG("Frame processing time %u ms", fTime);
  LOG_DBG("============================");
}

static bool closeAvi() {
  // closes the recorded file
  uint32_t vidDuration = millis() - startTime;
  uint32_t vidDurationSecs = lround(vidDuration/1000.0);
  logLine();
  LOG_DBG("Capture time %u, min seconds: %u ", vidDurationSecs, minSeconds);

  cTime = millis();
  // write remaining frame content to SD
  aviFile.write(iSDbuffer, highPoint); 
  size_t readLen = 0;
  // add wav file if exists
  finishAudio(true);
  bool haveWav = haveWavFile();
  if (haveWav) {
    do {
      readLen = writeWavFile(iSDbuffer, RAMSIZE);
      aviFile.write(iSDbuffer, readLen);
    } while (readLen > 0);
  }
  // save avi index
  finalizeAviIndex(frameCnt);
  do {
    readLen = writeAviIndex(iSDbuffer, RAMSIZE);
    if (readLen) aviFile.write(iSDbuffer, readLen);
  } while (readLen > 0);
  // save avi header at start of file
  float actualFPS = (1000.0f * (float)frameCnt) / ((float)vidDuration);
  uint8_t actualFPSint = (uint8_t)(lround(actualFPS));  
  xSemaphoreTake(aviMutex, portMAX_DELAY);
  buildAviHdr(actualFPSint, fsizePtr, frameCnt);
  xSemaphoreGive(aviMutex); 
  aviFile.seek(0, SeekSet); // start of file
  aviFile.write(aviHeader, AVI_HEADER_LEN); 
  aviFile.close();
  LOG_DBG("Final SD storage time %lu ms", millis() - cTime);
  uint32_t hTime = millis();
  if (vidDurationSecs >= minSeconds) {
    // name file to include actual dateTime, FPS, duration, and frame count
    int alen = snprintf(aviFileName, FILE_NAME_LEN - 1, "%s_%s_%u_%u_%u%s.%s", 
      partName, frameData[fsizePtr].frameSizeStr, actualFPSint, vidDurationSecs, frameCnt, haveWav ? "_S" : "", AVI_EXT);
    if (alen > FILE_NAME_LEN - 1) LOG_WRN("file name truncated");
    STORAGE.rename(AVITEMP, aviFileName);
    LOG_DBG("AVI close time %lu ms", millis() - hTime); 
    cTime = millis() - cTime;
    stopTelemetry(aviFileName);
    
    // AVI stats
    LOG_INF("******** AVI recording stats ********");
    LOG_ALT("Recorded %s", aviFileName);
    LOG_INF("AVI duration: %u secs", vidDurationSecs);
    LOG_INF("Number of frames: %u", frameCnt);
    LOG_INF("Required FPS: %u", FPS);
    LOG_INF("Actual FPS: %0.1f", actualFPS);
    LOG_INF("File size: %s", fmtSize(vidSize));
    if (frameCnt) {
      LOG_INF("Average frame length: %u bytes", vidSize / frameCnt);
      LOG_INF("Average frame monitoring time: %u ms", dTimeTot / frameCnt);
      LOG_INF("Average frame buffering time: %u ms", fTimeTot / frameCnt);
      LOG_INF("Average frame storage time: %u ms", wTimeTot / frameCnt);
    }
    LOG_INF("Average SD write speed: %u kB/s", ((vidSize / wTimeTot) * 1000) / 1024);
    LOG_INF("File open / completion times: %u ms / %u ms", oTime, cTime);
    LOG_INF("Busy: %u%%", std::min(100 * (wTimeTot + fTimeTot + dTimeTot + oTime + cTime) / vidDuration, (uint32_t)100));
    checkMemory();
    LOG_INF("*************************************");
    checkFreeStorage();
    return true; 
  } else {
    // delete too small files if exist
    STORAGE.remove(AVITEMP);
    LOG_INF("Insufficient capture duration: %u secs", vidDurationSecs); 
    return false;
  }
}

static boolean processFrame() {

  // get camera frame
  static bool wasCapturing = false;
  static bool wasRecording = false;
  bool res = true;
  uint32_t dTime = millis();
  bool finishRecording = false;

  camera_fb_t* fb = esp_camera_fb_get();
  if (fb == NULL || !fb->len || fb->len > MAX_JPEG) return false;
  for (int i = 0; i < numStreams; i++) {
    if (!streamBufferSize[i] && streamBuffer[i] != NULL) {
      memcpy(streamBuffer[i], fb->buf, fb->len);
      streamBufferSize[i] = fb->len;   
      xSemaphoreGive(frameSemaphore[i]); // signal frame ready for stream
    }
  }
  
  // force start button will start capture,
  isCapturing = forceRecord;
  if (forceRecord || wasRecording || doRecording) {
    if (forceRecord && !wasRecording) wasRecording = true;
    else if (!forceRecord && wasRecording) wasRecording = false;
    
    if (isCapturing && !wasCapturing) {
      stopPlaying(); // terminate any playback
      stopPlayback = true; // stop any subsequent playback
      LOG_ALT("Capture started by Button");
      openAvi();
      wasCapturing = true;
    }
    if (isCapturing && wasCapturing) {
      // capture is ongoing
      dTimeTot += millis() - dTime;
      saveFrame(fb);
      showProgress();
      if (frameCnt >= maxFrames) {
        logLine();
        LOG_INF("Auto closed recording after %u frames", maxFrames);
        forceRecord = false;
      }
    }
    if (!isCapturing && wasCapturing) {
      // movement stopped
      finishRecording = true;
    }
    wasCapturing = isCapturing;
  }

  esp_camera_fb_return(fb);
  if (finishRecording) {
    // cleanly finish recording (normal or forced)
    if (stopPlayback) closeAvi();
    finishRecording = isCapturing = wasCapturing = stopPlayback = false; // allow for playbacks
  }
  return res;
}

static void captureTask(void* parameter) {
  // woken by frame timer when time to capture frame
  uint32_t ulNotifiedValue;
  while (true) {
    ulNotifiedValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (ulNotifiedValue > FB_BUFFERS) ulNotifiedValue = FB_BUFFERS; // prevent too big queue if FPS excessive
    // may be more than one isr outstanding if the task delayed by SD write or jpeg decode
    while (ulNotifiedValue-- > 0) processFrame();
  }
  vTaskDelete(NULL);
}

uint8_t setFPS(uint8_t val) {
  // change or retrieve FPS value
  if (val) {
    FPS = val;
    // change frame timer which drives the task
    controlFrameTimer(true);
    saveFPS = FPS; // used to reset FPS after playback
  }
  return FPS;
}

uint8_t setFPSlookup(uint8_t val) {
  // set FPS from framesize lookup
  fsizePtr = val;
  return setFPS(frameData[fsizePtr].defaultFPS);
}

/********************** plackback AVI as MJPEG ***********************/

static fnameStruct extractMeta(const char* fname) {
  // extract FPS, duration, and frame count from avi filename
  fnameStruct fnameMeta;
  char fnameStr[FILE_NAME_LEN];
  strcpy(fnameStr, fname);
  // replace all '_' with space for sscanf
  replaceChar(fnameStr, '_', ' ');
  int items = sscanf(fnameStr, "%*s %*s %*s %hhu %u %hu", &fnameMeta.recFPS, &fnameMeta.recDuration, &fnameMeta.frameCnt);
  if (items != 3) LOG_ERR("failed to parse %s, items %u", fname, items);
  return fnameMeta;
}

static void playbackFPS(const char* fname) {
  // extract meta data from filename to commence playback
  fnameStruct fnameMeta = extractMeta(fname);
  recFPS = fnameMeta.recFPS;
  if (recFPS < 1) recFPS = 1;
  recDuration = fnameMeta.recDuration;
  // temp change framerate to recorded framerate
  FPS = recFPS;
  controlFrameTimer(true); // set frametimer
}

static void readSD() {
  // read next cluster from SD for playback
  uint32_t rTime = millis();
  // read to interim dram before copying to psram
  readLen = 0;
  if (!stopPlayback) {
    readLen = playbackFile.read(iSDbuffer+RAMSIZE+CHUNK_HDR, RAMSIZE);
    LOG_DBG("SD read time %lu ms", millis() - rTime);
  }
  wTimeTot += millis() - rTime;
  xSemaphoreGive(readSemaphore); // signal that ready     
  delay(10);
}


void openSDfile(const char* streamFile) {
  // open selected file on SD for streaming
  if (stopPlayback) LOG_WRN("Playback refused - capture in progress");
  else {
    stopPlaying(); // in case already running
    strcpy(aviFileName, streamFile);
    LOG_INF("Playing %s", aviFileName);
    playbackFile = STORAGE.open(aviFileName, FILE_READ);
    playbackFile.seek(AVI_HEADER_LEN, SeekSet); // skip over header
    playbackFPS(aviFileName);
    isPlaying = true; //playback status
    doPlayback = true; // control playback
    readSD(); // prime playback task
  }
}

mjpegStruct getNextFrame(bool firstCall) {
  // get next cluster on demand when ready for opened avi
  mjpegStruct mjpegData;
  static bool remainingBuff;
  static bool completedPlayback;
  static size_t buffOffset;
  static uint32_t hTimeTot;
  static uint32_t tTimeTot;
  static uint32_t hTime;
  static size_t remainingFrame;
  static size_t buffLen;
  const uint32_t dcVal = 0x63643030; // value of 00dc marker
  if (firstCall) {
    sTime = millis();
    hTime = millis();  
    remainingBuff = completedPlayback = false;
    frameCnt = remainingFrame = vidSize =  buffOffset = 0;
    wTimeTot = fTimeTot = hTimeTot = tTimeTot = 1; // avoid divide by 0
  }  
  LOG_DBG("http send time %lu ms", millis() - hTime);
  hTimeTot += millis() - hTime;
  uint32_t mTime = millis();
  if (!stopPlayback) {
    // continue sending out frames
    if (!remainingBuff) {
      // load more data from SD
      mTime = millis();
      // move final bytes to buffer start in case jpeg marker at end of buffer
      memcpy(iSDbuffer, iSDbuffer+RAMSIZE, CHUNK_HDR);
      xSemaphoreTake(readSemaphore, portMAX_DELAY); // wait for read from SD card completed
      buffLen = readLen;
      LOG_DBG("SD wait time %lu ms", millis()-mTime);
      wTimeTot += millis()-mTime;
      mTime = millis();  
      // overlap buffer by CHUNK_HDR to prevent jpeg marker being split between buffers
      memcpy(iSDbuffer+CHUNK_HDR, iSDbuffer+RAMSIZE+CHUNK_HDR, buffLen); // load new cluster from double buffer
      LOG_DBG("memcpy took %lu ms for %u bytes", millis()-mTime, buffLen);
      fTimeTot += millis() - mTime;
      remainingBuff = true;
      if (buffOffset > RAMSIZE) buffOffset = 4; // special case, marker overlaps end of buffer 
      else buffOffset = frameCnt ? 0 : CHUNK_HDR; // only before 1st frame
      xTaskNotifyGive(playbackHandle); // wake up task to get next cluster - sets readLen
    }
    mTime = millis();
    if (!remainingFrame) {
      // at start of jpeg frame marker
      uint32_t inVal;
      memcpy(&inVal, iSDbuffer + buffOffset, 4);
      if (inVal != dcVal) {
        // reached end of frames to stream
        mjpegData.buffLen = buffOffset; // remainder of final jpeg
        mjpegData.buffOffset = 0; // from start of buff
        mjpegData.jpegSize = 0; 
        stopPlayback = completedPlayback = true;
        return mjpegData;
      } else {
        // get jpeg frame size
        uint32_t jpegSize;
        memcpy(&jpegSize, iSDbuffer + buffOffset + 4, 4);
        remainingFrame = jpegSize;
        vidSize += jpegSize;
        buffOffset += CHUNK_HDR; // skip over marker 
        mjpegData.jpegSize = jpegSize; // signal start of jpeg to webServer
        mTime = millis();
        // wait on playbackSemaphore for rate control
        xSemaphoreTake(playbackSemaphore, portMAX_DELAY);
        LOG_DBG("frame timer wait %lu ms", millis()-mTime);
        tTimeTot += millis()-mTime;
        frameCnt++;
        showProgress();
      }
    } else mjpegData.jpegSize = 0; // within frame,    
    // determine amount of data to send to webServer
    if (buffOffset > RAMSIZE) mjpegData.buffLen = 0; // special case 
    else mjpegData.buffLen = (remainingFrame > buffLen - buffOffset) ? buffLen - buffOffset : remainingFrame;
    mjpegData.buffOffset = buffOffset; // from here    
    remainingFrame -= mjpegData.buffLen;
    buffOffset += mjpegData.buffLen;
    if (buffOffset >= buffLen) remainingBuff = false;
  } else {
    // finished, close SD file used for streaming
    playbackFile.close();
    logLine();
    if (!completedPlayback) LOG_INF("Force close playback");
    uint32_t playDuration = (millis() - sTime) / 1000;
    uint32_t totBusy = wTimeTot + fTimeTot + hTimeTot;
    LOG_INF("******** AVI playback stats ********");
    LOG_INF("Playback %s", aviFileName);
    LOG_INF("Recorded FPS %u, duration %u secs", recFPS, recDuration);
    LOG_INF("Playback FPS %0.1f, duration %u secs", (float)frameCnt / playDuration, playDuration);
    LOG_INF("Number of frames: %u", frameCnt);
    if (frameCnt) {
      LOG_INF("Average SD read speed: %u kB/s", ((vidSize / wTimeTot) * 1000) / 1024);
      LOG_INF("Average frame SD read time: %u ms", wTimeTot / frameCnt);
      LOG_INF("Average frame processing time: %u ms", fTimeTot / frameCnt);
      LOG_INF("Average frame delay time: %u ms", tTimeTot / frameCnt);
      LOG_INF("Average http send time: %u ms", hTimeTot / frameCnt);
      LOG_INF("Busy: %u%%", min(100 * totBusy / (totBusy + tTimeTot), (uint32_t)100));
    }
    checkMemory();
    LOG_INF("*************************************\n");
    setFPS(saveFPS); // realign with browser
    stopPlayback = isPlaying = false;
    mjpegData.buffLen = mjpegData.buffOffset = 0; // signal end of jpeg
  }
  hTime = millis();
  delay(1);
  return mjpegData;
}

void stopPlaying() {
  if (isPlaying) {
    // force stop any currently running playback
    stopPlayback = true;
    // wait till stopped cleanly, but prevent infinite loop
    uint32_t timeOut = millis();
    while (doPlayback && millis() - timeOut < MAX_FRAME_WAIT) delay(10);
    if (doPlayback) {
      // not yet closed, so force close
      logLine();
      LOG_WRN("Force closed playback");
      doPlayback = false; // stop webserver playback
      setFPS(saveFPS);
      xSemaphoreGive(playbackSemaphore);
      xSemaphoreGive(readSemaphore);
      delay(200);
    } 
    stopPlayback = false;
    isPlaying = false;
  }
}

static void playbackTask(void* parameter) {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    readSD();
  }
  vTaskDelete(NULL);
}

/******************* Startup ********************/

static void startSDtasks() {
  // tasks to manage SD card operation
  xTaskCreate(&captureTask, "captureTask", CAPTURE_STACK_SIZE, NULL, 5, &captureHandle);
  
  sensor_t * s = esp_camera_sensor_get();
  fsizePtr = s->status.framesize;
  setFPS(frameData[fsizePtr].defaultFPS);

  debugMemory("startSDtasks");
}

bool prepRecording() {
  // initialisation & prep for AVI capture
  readSemaphore = xSemaphoreCreateBinary();
  playbackSemaphore = xSemaphoreCreateBinary();
  aviMutex = xSemaphoreCreateMutex();
  for (int i = 0; i < numStreams; i++) frameSemaphore[i] = xSemaphoreCreateBinary();
  camera_fb_t* fb = esp_camera_fb_get();
  if (fb == NULL) LOG_WRN("failed to get camera frame");
  else {
    esp_camera_fb_return(fb);
    fb = NULL;
  }
  startSDtasks();
#if INCLUDE_TINYML
  LOG_INF("%sUsing TinyML", mlUse ? "" : "Not ");
#endif
  LOG_INF("Ready to record new AVI !");
  logLine();
  debugMemory("prepRecording");
  return true;
}

static void deleteTask(TaskHandle_t thisTaskHandle) {
  if (thisTaskHandle != NULL) vTaskDelete(thisTaskHandle);
  thisTaskHandle = NULL;
}

void endTasks() {
  for (int i = 0; i < numStreams; i++) deleteTask(sustainHandle[i]);
  deleteTask(captureHandle);
  deleteTask(playbackHandle);
  deleteTask(DS18B20handle);
  deleteTask(telemetryHandle);
  deleteTask(servoHandle);
  deleteTask(emailHandle);
  deleteTask(fsHandle);
  deleteTask(uartClientHandle);
  deleteTask(stickHandle);
}

void OTAprereq() {
  // stop timer isrs, and free up heap space, or crashes esp32
  doPlayback = forceRecord = false;
  controlFrameTimer(false);
  stickTimer(false);
  stopPing();
  endTasks();
  esp_camera_deinit();
  delay(100);
}

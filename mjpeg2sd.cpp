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
#define OneMHz 1000000

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
static char partName[FILE_NAME_LEN];

// task control
TaskHandle_t captureHandle = NULL;
static SemaphoreHandle_t readSemaphore;
SemaphoreHandle_t frameSemaphore[MAX_STREAMS] = {NULL};
SemaphoreHandle_t aviMutex = NULL;
bool isCapturing = false;

/**************** timers & ISRs ************************/

static void IRAM_ATTR frameISR() {
  // interrupt at current frame rate
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(captureHandle, &xHigherPriorityTaskWoken); // wake capture task to process frame
  if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

void controlFrameTimer(bool restartTimer) {
  // frame timer control
  static hw_timer_t* frameTimer = NULL;
  // stop current timer
  if (frameTimer) {
    timerDetachInterrupt(frameTimer); 
    timerEnd(frameTimer);
    frameTimer = NULL;
  }
  if (restartTimer) {
    // (re)start timer interrupt for required framerate
    frameTimer = timerBegin(OneMHz);
    if (frameTimer) {
      frameInterval = OneMHz / FPS; // in units of us
      timerAttachInterrupt(frameTimer, &frameISR);
      timerAlarm(frameTimer, frameInterval, true, 0); // micro seconds
    } else LOG_ERR("Failed to setup frameTimer");
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
  
  // force start button will start capture,
  isCapturing = forceRecord;
  if (forceRecord || wasRecording || doRecording) {
    if (forceRecord && !wasRecording) wasRecording = true;
    else if (!forceRecord && wasRecording) wasRecording = false;
    
    if (isCapturing && !wasCapturing) {
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
    closeAvi();
    finishRecording = isCapturing = wasCapturing = false;
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
  }
  return FPS;
}

uint8_t setFPSlookup(uint8_t val) {
  // set FPS from framesize lookup
  fsizePtr = val;
  return setFPS(frameData[fsizePtr].defaultFPS);
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
  aviMutex = xSemaphoreCreateMutex();
  camera_fb_t* fb = esp_camera_fb_get();
  if (fb == NULL) LOG_WRN("failed to get camera frame");
  else {
    esp_camera_fb_return(fb);
    fb = NULL;
  }
  startSDtasks();
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
  deleteTask(captureHandle);
}

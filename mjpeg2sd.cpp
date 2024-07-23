#include "globals.h"
#include <esp_camera.h>
#include <SD_MMC.h>
#include <FS.h>

// SD card storage
uint8_t iSDbuffer[(RAMSIZE + CHUNK_HDR) * 2];
static size_t highPoint;
static File aviFile;
static char aviFileName[FILE_NAME_LEN];
static char dirName[FILE_NAME_LEN] = "/recordings";

// status & control fields
bool forceRecord = false; // Recording enabled by rec button
static uint16_t frameInterval; // units of 0.1ms between frames
uint8_t FPS = 0;
uint8_t fsizePtr; // index to frameData[]
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

// task control
TaskHandle_t captureHandle = NULL;
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
    } else Serial.println("[!] Failed to setup frameTimer");
  }
}

/**************** capture AVI  ************************/

static void openAvi() {
  // time to open a new file on SD increases with the number of files already present
  oTime = millis();
  SD_MMC.mkdir(dirName);
  // open avi file with temporary name 
  aviFile = SD_MMC.open(AVITEMP, FILE_WRITE);
  oTime = millis() - oTime;
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
  // whats left or small frame
  memcpy(iSDbuffer+highPoint, fb->buf + jpegSize - jpegRemain, jpegRemain);
  highPoint += jpegRemain;
  
  buildAviIdx(jpegSize); // save avi index for frame
  vidSize += jpegSize + CHUNK_HDR;
  frameCnt++; 
  fTime = millis() - fTime - wTime;
  fTimeTot += fTime;
}

static bool closeAvi() {
  // closes the recorded file
  uint32_t vidDuration = millis() - startTime;
  uint32_t vidDurationSecs = lround(vidDuration/1000.0);

  cTime = millis();
  // write remaining frame content to SD
  size_t readLen = 0;
  aviFile.write(iSDbuffer, highPoint);
  // save avi index
  finalizeAviIndex(frameCnt);
  do {
    readLen = writeAviIndex(iSDbuffer, RAMSIZE);
    if (readLen) aviFile.write(iSDbuffer, readLen);
  } while (readLen > 0);
  // save avi header at start of file
  float actualFPS = (1000.0f * (float)frameCnt) / ((float)vidDuration);
  uint8_t actualFPSint = (uint8_t)(lround(actualFPS));
  buildAviHdr(actualFPSint, fsizePtr, frameCnt);
  aviFile.seek(0, SeekSet); // start of file
  aviFile.write(aviHeader, AVI_HEADER_LEN); 
  aviFile.close();
  // name file
  int alen = snprintf(aviFileName, FILE_NAME_LEN - 1, "%s/%s_%u.%s", dirName, frameData[fsizePtr].frameSizeStr, frameCnt, AVI_EXT);
  if (alen > FILE_NAME_LEN - 1) Serial.println("File name truncated");
  SD_MMC.rename(AVITEMP, aviFileName);
  cTime = millis() - cTime;
  
  // AVI stats
  Serial.println("");
  Serial.println("******** AVI recording stats ********");
  Serial.printf("Recorded %s\n", aviFileName);
  Serial.printf("AVI duration: %u secs\n", vidDurationSecs);
  Serial.printf("Number of frames: %u\n", frameCnt);
  Serial.printf("Required FPS: %u\n", FPS);
  Serial.printf("Actual FPS: %0.1f\n", actualFPS);
  Serial.printf("File size: %s\n", fmtSize(vidSize));
  if (frameCnt) {
    Serial.printf("Average frame length: %u bytes\n", vidSize / frameCnt);
    Serial.printf("Average frame monitoring time: %u ms\n", dTimeTot / frameCnt);
    Serial.printf("Average frame buffering time: %u ms\n", fTimeTot / frameCnt);
    Serial.printf("Average frame storage time: %u ms\n", wTimeTot / frameCnt);
  }
  Serial.printf("Average SD write speed: %u kB/s\n", ((vidSize / wTimeTot) * 1000) / 1024);
  Serial.printf("File open / completion times: %u ms / %u ms\n", oTime, cTime);
  Serial.printf("Busy: %u%%\n", std::min(100 * (wTimeTot + fTimeTot + dTimeTot + oTime + cTime) / vidDuration, (uint32_t)100));
  Serial.println("*************************************");
  return true;
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
  if (forceRecord || wasRecording) {
    if (forceRecord && !wasRecording) wasRecording = true;
    else if (!forceRecord && wasRecording) wasRecording = false;
    
    if (isCapturing && !wasCapturing) {
      Serial.println("Capture started");
      openAvi();
      wasCapturing = true;
    }
    if (isCapturing && wasCapturing) {
      // capture is ongoing
      dTimeTot += millis() - dTime;
      saveFrame(fb);
      showProgress();
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

bool startStorage() {
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  return SD_MMC.begin("/sdcard", true, true);
}

bool startCam() {
  // configure camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = xclkMhz * 1000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_count = FB_BUFFERS;

  config.frame_size = FRAMESIZE_VGA; // image res
  config.jpeg_quality = 4; //0-63, lower number = higher quality

  // camera init
  esp_err_t err = esp_camera_init(&config);
  return err == ESP_OK;

}

static void startSDtasks() {
  // tasks to manage SD card operation
  xTaskCreate(&captureTask, "captureTask", CAPTURE_STACK_SIZE, NULL, 1, &captureHandle);
  
  sensor_t * s = esp_camera_sensor_get();
  fsizePtr = s->status.framesize;
  setFPS(frameData[fsizePtr].defaultFPS);
}

bool prepRecording() {
  // initialisation & prep for AVI capture
  camera_fb_t* fb = esp_camera_fb_get();
  if (fb == NULL) return false;

  esp_camera_fb_return(fb);
  fb = NULL;
  startSDtasks();
  return true;
}

/****** misc ******/

static void deleteTask(TaskHandle_t thisTaskHandle) {
  if (thisTaskHandle != NULL) vTaskDelete(thisTaskHandle);
  thisTaskHandle = NULL;
}

void endTasks() {
  deleteTask(captureHandle);
}

void showProgress(const char* marker) {
  // show progess as dots 
  static uint8_t dotCnt = 0;
  Serial.print(marker); // progress marker
  if (++dotCnt >= 30) {
    dotCnt = 0;
    Serial.println("");
  }
}

char* fmtSize (uint64_t sizeVal) {
  // format size according to magnitude
  // only one call per format string
  static char returnStr[20];
  if (sizeVal < 50 * 1024) sprintf(returnStr, "%llu bytes", sizeVal);
  else if (sizeVal < ONEMEG) sprintf(returnStr, "%lluKB", sizeVal / 1024);
  else if (sizeVal < ONEMEG * 1024) sprintf(returnStr, "%0.1fMB", (double)(sizeVal) / ONEMEG);
  else sprintf(returnStr, "%0.1fGB", (double)(sizeVal) / (ONEMEG * 1024));
  return returnStr;
}
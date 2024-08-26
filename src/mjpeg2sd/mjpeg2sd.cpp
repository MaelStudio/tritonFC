// Libraries
#include <SD_MMC.h>
#include <FS.h>
#include <esp_camera.h>

#include "src/mjpeg2sd/mjpeg2sd.h"

// Store video res for flight stats
int frameWidth = 0;
int frameHeight = 0;

// SD card storage
uint8_t iSDbuffer[(RAMSIZE + CHUNK_HDR) * 2];
static size_t highPoint;

// status & control fields
int FPS = 0;
static uint16_t frameInterval; // units of 0.1ms between frames
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
bool isRecording = false;

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

static void openAvi(char* fileName) {
  // time to open a new file on SD increases with the number of files already present
  oTime = millis();
  aviFile = SD_MMC.open(fileName, FILE_WRITE);
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

static float closeAvi() {
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
  cTime = millis() - cTime;
  
  // AVI stats
  Serial.println("");
  Serial.println("******** AVI recording stats ********");
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
  return actualFPS;
}

void startVideo(char* fileName) {
  openAvi(fileName);
  isRecording = true;
}

float stopVideo() {
  isRecording = false;
  return closeAvi();
}

static boolean processFrame() {

  if (isRecording) {
    uint32_t dTime = millis();
    camera_fb_t* fb = esp_camera_fb_get(); // get camera frame
    if (fb == NULL || !fb->len || fb->len > MAX_JPEG) return false;
    dTimeTot += millis() - dTime;

    saveFrame(fb);
    esp_camera_fb_return(fb);
  }

  return true;
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

/******************* Startup ********************/

bool startCam(char vidRes[10]) {
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
  config.jpeg_quality = 4; //0-63, lower number = higher quality

  bool validFrameSize = false;
  framesize_t frameSize;
  for (int i = 0; i < 14; i++) {
    if (strcmp(frameData[i].frameSizeStr, vidRes) == 0) {
      frameSize = (framesize_t)i;  // Found resolution index in frameData
      validFrameSize = true;
      break;
    }
  }
  if (!validFrameSize) {
    Serial.printf("Invalid video frame size: %s\n", vidRes);
    return false;
  }

  config.frame_size = frameSize; // image res
  
  // save frame res
  frameWidth = frameData[frameSize].frameWidth;
  frameHeight = frameData[frameSize].frameHeight;
  
  // camera init
  esp_err_t err = esp_camera_init(&config);
  return err == ESP_OK;

}

static void startSDtasks(int setFPS) {
  // tasks to manage SD card operation
  xTaskCreate(&captureTask, "captureTask", CAPTURE_STACK_SIZE, NULL, 1, &captureHandle);
  
  sensor_t * s = esp_camera_sensor_get();
  fsizePtr = s->status.framesize;

  FPS = setFPS;
  controlFrameTimer(true); // frame timer which drives the task
}

bool prepRecording(int setFPS) {
  // initialisation & prep for AVI capture
  camera_fb_t* fb = esp_camera_fb_get();
  if (fb == NULL) return false;

  esp_camera_fb_return(fb);
  fb = NULL;
  startSDtasks(setFPS);
  return true;
}

char* fmtSize (uint64_t sizeVal) {
  // format size according to magnitude
  // only one call per format string
  static char returnStr[20];
  if (sizeVal < 50 * 1024) sprintf(returnStr, "%llu bytes", sizeVal);
  else if (sizeVal < ONEMEG) sprintf(returnStr, "%llu KB", sizeVal / 1024);
  else if (sizeVal < ONEMEG * 1024) sprintf(returnStr, "%0.1f MB", (double)(sizeVal) / ONEMEG);
  else sprintf(returnStr, "%0.1f GB", (double)(sizeVal) / (ONEMEG * 1024));
  return returnStr;
}
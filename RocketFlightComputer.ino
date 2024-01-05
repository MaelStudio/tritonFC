#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "esp_timer.h"

#include "camera_pins.h"

#define SD_PIN_CS 21
#define BUTTON_PIN D0
#define ONEMEG (1024 * 1024)
#define FILE_NAME_LEN 64
#define CHUNK_HDR 8 // bytes per jpeg hdr in AVI 
#define RAMSIZE (1024 * 8) // set this to multiple of SD card sector size (512 or 1024 bytes)
#define AVI_HEADER_LEN 310 // AVI header length
#define IDX_ENTRY 16 // bytes per index entry
#define MAX_JPEG ONEMEG/2 // UXGA jpeg frame buffer at highest quality 375kB rounded up
#define AVINAME "/recording.avi"
#define WAVTEMP "/current.wav"

// status & control fields
uint8_t FPS;
uint8_t fsizePtr; // index to frameData[]
bool doRecording = true; // whether to capture to SD or not 
uint8_t xclkMhz = 20; // camera clock rate MHz
uint8_t minSeconds = 5; // default min video length

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

uint8_t frameDataRows = 14; 
static uint16_t frameInterval; // units of 0.1ms between frames

// SD card storage
int maxFrames = 3000; // maximum number of frames in video before auto close
#define MAX_JPEG ONEMEG/2 // UXGA jpeg frame buffer at highest quality 375kB rounded up
uint8_t iSDbuffer[(RAMSIZE + CHUNK_HDR) * 2];
static size_t highPoint;
static File aviFile;
static char folderName[FILE_NAME_LEN];

// avi header data
const uint8_t dcBuf[4] = {0x30, 0x30, 0x64, 0x63};   // 00dc
const uint8_t wbBuf[4] = {0x30, 0x31, 0x77, 0x62};   // 01wb
static const uint8_t idx1Buf[4] = {0x69, 0x64, 0x78, 0x31}; // idx1
static const uint8_t zeroBuf[4] = {0x00, 0x00, 0x00, 0x00}; // 0000
static uint8_t* idxBuf[2] = {NULL, NULL};

uint8_t aviHeader[AVI_HEADER_LEN] = { // AVI header template
  0x52, 0x49, 0x46, 0x46, 0x00, 0x00, 0x00, 0x00, 0x41, 0x56, 0x49, 0x20, 0x4C, 0x49, 0x53, 0x54,
  0x16, 0x01, 0x00, 0x00, 0x68, 0x64, 0x72, 0x6C, 0x61, 0x76, 0x69, 0x68, 0x38, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x49, 0x53, 0x54, 0x6C, 0x00, 0x00, 0x00,
  0x73, 0x74, 0x72, 0x6C, 0x73, 0x74, 0x72, 0x68, 0x30, 0x00, 0x00, 0x00, 0x76, 0x69, 0x64, 0x73,
  0x4D, 0x4A, 0x50, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x74, 0x72, 0x66,
  0x28, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x18, 0x00, 0x4D, 0x4A, 0x50, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x4C, 0x49, 0x53, 0x54, 0x56, 0x00, 0x00, 0x00, 
  0x73, 0x74, 0x72, 0x6C, 0x73, 0x74, 0x72, 0x68, 0x30, 0x00, 0x00, 0x00, 0x61, 0x75, 0x64, 0x73,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x00, 0x11, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x11, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x73, 0x74, 0x72, 0x66,
  0x12, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x11, 0x2B, 0x00, 0x00, 0x11, 0x2B, 0x00, 0x00,
  0x02, 0x00, 0x10, 0x00, 0x00, 0x00, 
  0x4C, 0x49, 0x53, 0x54, 0x00, 0x00, 0x00, 0x00, 0x6D, 0x6F, 0x76, 0x69,
};

struct frameSizeStruct {
  uint8_t frameWidth[2];
  uint8_t frameHeight[2];
};
// indexed by frame type - needs to be consistent with sensor.h framesize_t enum
static const frameSizeStruct frameSizeData[] = {
  {{0x60, 0x00}, {0x60, 0x00}}, // 96X96
  {{0xA0, 0x00}, {0x78, 0x00}}, // qqvga 
  {{0xB0, 0x00}, {0x90, 0x00}}, // qcif 
  {{0xF0, 0x00}, {0xB0, 0x00}}, // hqvga 
  {{0xF0, 0x00}, {0xF0, 0x00}}, // 240X240
  {{0x40, 0x01}, {0xF0, 0x00}}, // qvga 
  {{0x90, 0x01}, {0x28, 0x01}}, // cif 
  {{0xE0, 0x01}, {0x40, 0x01}}, // hvga 
  {{0x80, 0x02}, {0xE0, 0x01}}, // vga 
  {{0x20, 0x03}, {0x58, 0x02}}, // svga 
  {{0x00, 0x04}, {0x00, 0x03}}, // xga 
  {{0x00, 0x05}, {0xD0, 0x02}}, // hd
  {{0x00, 0x05}, {0x00, 0x04}}, // sxga
  {{0x40, 0x06}, {0xB0, 0x04}}  // uxga 
};

// separate index for motion capture and timelapse
static size_t idxPtr[2];
static size_t idxOffset[2];
static size_t moviSize[2];
static size_t audSize;
static size_t indexLen[2];
static File wavFile;
bool haveSoundFile = false;

const uint32_t WAV_HEADER_LEN = 44; // WAV header length
static uint8_t wavHeader[WAV_HEADER_LEN] = { // WAV header template
  0x52, 0x49, 0x46, 0x46, 0x00, 0x00, 0x00, 0x00, 0x57, 0x41, 0x56, 0x45, 0x66, 0x6D, 0x74, 0x20,
  0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x11, 0x2B, 0x00, 0x00, 0x11, 0x2B, 0x00, 0x00,
  0x02, 0x00, 0x10, 0x00, 0x64, 0x61, 0x74, 0x61, 0x00, 0x00, 0x00, 0x00,
};
const uint32_t SAMPLE_RATE = 16000; // sample rate used

// task control
TaskHandle_t captureHandle = NULL;
SemaphoreHandle_t aviMutex = NULL;
bool wasCapturing = false;
bool isCapturing = false;

struct frameStruct {
  const char* frameSizeStr;
  const uint16_t frameWidth;
  const uint16_t frameHeight;
  const uint16_t defaultFPS;
  const uint8_t scaleFactor; // (0..4)
  const uint8_t sampleRate; // (1..N)
};

// indexed by frame size - needs to be consistent with sensor.h framesize_t enum
const frameStruct frameData[] = {
  {"96X96", 96, 96, 30, 1, 1},   // 2MP sensors
  {"QQVGA", 160, 120, 30, 1, 1},
  {"QCIF", 176, 144, 30, 1, 1}, 
  {"HQVGA", 240, 176, 30, 2, 1}, 
  {"240X240", 240, 240, 30, 2, 1}, 
  {"QVGA", 320, 240, 30, 2, 1}, 
  {"CIF", 400, 296, 30, 2, 1},  
  {"HVGA", 480, 320, 30, 2, 1}, 
  {"VGA", 640, 480, 20, 3, 1}, 
  {"SVGA", 800, 600, 20, 3, 1}, 
  {"XGA", 1024, 768, 5, 3, 1},   
  {"HD", 1280, 720, 5, 3, 1}, 
  {"SXGA", 1280, 1024, 5, 3, 1}, 
  {"UXGA", 1600, 1200, 5, 3, 1},  
  {"FHD", 920, 1080, 5, 3, 1},    // 3MP Sensors
  {"P_HD", 720, 1280, 5, 3, 1},
  {"P_3MP", 864, 1536, 5, 3, 1},
  {"QXGA", 2048, 1536, 5, 4, 1},
  {"QHD", 2560, 1440, 5, 4, 1},   // 5MP Sensors
  {"WQXGA", 2560, 1600, 5, 4, 1},
  {"P_FHD", 1080, 1920, 5, 3, 1},
  {"QSXGA", 2560, 1920, 4, 4, 1}
};

char* fmtSize (uint64_t sizeVal) {
  // format size according to magnitude
  // only one call per format string
  static char returnStr[20];
  if (sizeVal < 100 * 1024) sprintf(returnStr, "%llu", sizeVal);
  else if (sizeVal < ONEMEG) sprintf(returnStr, "%llukB", sizeVal / 1024);
  else if (sizeVal < ONEMEG * 1024) sprintf(returnStr, "%0.1fMB", (double)(sizeVal) / ONEMEG);
  else sprintf(returnStr, "%0.1fGB", (double)(sizeVal) / (ONEMEG * 1024));
  return returnStr;
}

// AVI INDEX
void prepAviIndex(bool isTL) {
  // prep buffer to store index data, gets appended to end of file
  if (idxBuf[isTL] == NULL) idxBuf[isTL] = (uint8_t*)ps_malloc((maxFrames+1)*IDX_ENTRY); // include some space for audio index
  memcpy(idxBuf[isTL], idx1Buf, 4); // index header
  idxPtr[isTL] = CHUNK_HDR;  // leave 4 bytes for index size
  moviSize[isTL] = indexLen[isTL] = 0;
}

void buildAviIdx(size_t dataSize, bool isVid, bool isTL) {

  Serial.println("IM ALIVE");

  // build AVI video index into buffer - 16 bytes per frame
  // called from saveFrame() for each frame
  moviSize[isTL] += dataSize;
  if (isVid) memcpy(idxBuf[isTL]+idxPtr[isTL], dcBuf, 4);
  else memcpy(idxBuf[isTL]+idxPtr[isTL], wbBuf, 4);
  memcpy(idxBuf[isTL]+idxPtr[isTL]+4, zeroBuf, 4);
  memcpy(idxBuf[isTL]+idxPtr[isTL]+8, &idxOffset[isTL], 4); 
  memcpy(idxBuf[isTL]+idxPtr[isTL]+12, &dataSize, 4); 
  idxOffset[isTL] += dataSize + CHUNK_HDR;
  idxPtr[isTL] += IDX_ENTRY; 
}

void finalizeAviIndex(uint16_t frameCnt, bool isTL) {
  // update index with size
  uint32_t sizeOfIndex = (frameCnt+(haveSoundFile?1:0))*IDX_ENTRY;
  memcpy(idxBuf[isTL]+4, &sizeOfIndex, 4); // size of index 
  indexLen[isTL] = sizeOfIndex + CHUNK_HDR;
  idxPtr[isTL] = 0; // pointer to index buffer
}

size_t writeAviIndex(byte* clientBuf, size_t buffSize, bool isTL) {
  // write completed index to avi file
  // called repeatedly from closeAvi() until return 0
  if (idxPtr[isTL] < indexLen[isTL]) {
    if (indexLen[isTL]-idxPtr[isTL] > buffSize) {
      memcpy(clientBuf, idxBuf[isTL]+idxPtr[isTL], buffSize);
      idxPtr[isTL] += buffSize;
      return buffSize;
    } else {
      // final part of index
      size_t final = indexLen[isTL]-idxPtr[isTL];
      memcpy(clientBuf, idxBuf[isTL]+idxPtr[isTL], final);
      idxPtr[isTL] = indexLen[isTL];
      return final;    
    }
  }
  return idxPtr[isTL] = 0;
}

void buildAviHdr(uint8_t FPS, uint8_t frameType, uint16_t frameCnt, bool isTL) {
  // update AVI header template with file specific details
  size_t aviSize = moviSize[isTL] + AVI_HEADER_LEN + ((CHUNK_HDR+IDX_ENTRY) * (frameCnt+(haveSoundFile?1:0))); // AVI content size 
  // update aviHeader with relevant stats
  memcpy(aviHeader+4, &aviSize, 4);
  uint32_t usecs = (uint32_t)round(1000000.0f / FPS); // usecs_per_frame 
  memcpy(aviHeader+0x20, &usecs, 4); 
  memcpy(aviHeader+0x30, &frameCnt, 2);
  memcpy(aviHeader+0x8C, &frameCnt, 2);
  memcpy(aviHeader+0x84, &FPS, 1);
  uint32_t dataSize = moviSize[isTL] + ((frameCnt+(haveSoundFile?1:0)) * CHUNK_HDR) + 4; 
  memcpy(aviHeader+0x12E, &dataSize, 4); // data size 
  uint8_t withAudio = 2; // increase number of streams for audio
  if (isTL) memcpy(aviHeader+0x100, zeroBuf, 4); // no audio for timelapse
  else {
    if (haveSoundFile) memcpy(aviHeader+0x38, &withAudio, 1); 
    memcpy(aviHeader+0x100, &audSize, 4); // audio data size
  }
  // apply video framesize to avi header
  memcpy(aviHeader+0x40, frameSizeData[frameType].frameWidth, 2);
  memcpy(aviHeader+0xA8, frameSizeData[frameType].frameWidth, 2);
  memcpy(aviHeader+0x44, frameSizeData[frameType].frameHeight, 2);
  memcpy(aviHeader+0xAC, frameSizeData[frameType].frameHeight, 2);
  // apply audio details to avi header
  memcpy(aviHeader+0xF8, &SAMPLE_RATE, 4);
  uint32_t bytesPerSec = SAMPLE_RATE * 2;
  memcpy(aviHeader+0x104, &bytesPerSec, 4); // suggested buffer size
  memcpy(aviHeader+0x11C, &SAMPLE_RATE, 4);
  memcpy(aviHeader+0x120, &bytesPerSec, 4); // bytes per sec

  // reset state for next recording
  moviSize[isTL] = idxOffset[isTL] = idxPtr[isTL] = 0;
}

// AVI VIDEO
static bool openAvi() {
  // time to open a new file on SD increases with the number of files already present
  oTime = millis();
  SD.mkdir(folderName); // make folder if not present
  // open avi file with temporary name 
  aviFile = SD.open(AVINAME, FILE_WRITE);

  if (!aviFile) {
    Serial.println("[!] Error opening AVI file.");
    return false;
  }
  oTime = millis() - oTime;

  Serial.print("File opening time: ");
  Serial.print(oTime);
  Serial.println(" ms");

  // wavFile = SD.open(WAVTEMP, FILE_WRITE);
  // wavFile.write(wavHeader, WAV_HEADER_LEN);

  // initialisation of counters
  startTime = millis();
  frameCnt = fTimeTot = wTimeTot = dTimeTot = vidSize = 0;
  highPoint = AVI_HEADER_LEN; // allot space for AVI header

  return true;
}


static bool closeAvi() {
  // closes the recorded file
  uint32_t vidDuration = millis() - startTime;
  uint32_t vidDurationSecs = lround(vidDuration/1000.0);
  Serial.print("Capture time: ");
  Serial.print(vidDurationSecs);
  Serial.print(" s");

  cTime = millis();
  // write remaining frame content to SD
  aviFile.write(iSDbuffer, highPoint);
  size_t readLen = 0;

  // add wav file if exists
  // finishAudio(true);
  // bool haveWav = haveWavFile();
  // if (haveWav) {
  //   do {
  //     readLen = writeWavFile(iSDbuffer, RAMSIZE);
  //     aviFile.write(iSDbuffer, readLen);
  //   } while (readLen > 0);
  // }

  // save avi index
  finalizeAviIndex(frameCnt, false);
  do {
    readLen = writeAviIndex(iSDbuffer, RAMSIZE, false);
    if (readLen) aviFile.write(iSDbuffer, readLen);
  } while (readLen > 0);

  // save avi header at start of file
  float actualFPS = (1000.0f * (float)frameCnt) / ((float)vidDuration);
  uint8_t actualFPSint = (uint8_t)(lround(actualFPS));

  xSemaphoreTake(aviMutex, portMAX_DELAY);
  buildAviHdr(actualFPSint, fsizePtr, frameCnt, false);
  xSemaphoreGive(aviMutex);

  aviFile.seek(0, SeekSet); // start of file
  aviFile.write(aviHeader, AVI_HEADER_LEN); 
  aviFile.close();

  cTime = millis() - cTime;

  Serial.print("Final SD storage time ");
  Serial.print(cTime);
  Serial.println(" ms");

  if (vidDurationSecs < minSeconds) {
    // delete too small files if exist
    SD.remove(AVINAME);
    Serial.println("[!] Insufficient capture duration!");
    return false;
  }

  // AVI stats
  Serial.println("******** AVI recording stats ********");
  Serial.print("Saved to SD card as ");
  Serial.println(AVINAME);
  Serial.print("AVI duration: ");
  Serial.println(vidDurationSecs);
  Serial.print("Number of frames: ");
  Serial.println(frameCnt);
  Serial.print("Required FPS: ");
  Serial.println(FPS);
  Serial.print("Actual FPS: ");
  Serial.println(actualFPS, 1);
  Serial.print("File size: ");
  Serial.println(fmtSize(vidSize));
  Serial.print("Average frame length: ");
  Serial.println(vidSize / frameCnt);
  Serial.print("Average frame monitoring time: ");
  Serial.println(dTimeTot / frameCnt);
  Serial.print("Average frame buffering time: ");
  Serial.println(fTimeTot / frameCnt);
  Serial.print("Average frame storage time: ");
  Serial.println(wTimeTot / frameCnt);
  Serial.print("Average SD write speed: ");
  Serial.println(((vidSize / wTimeTot) * 1000) / 1024);
  Serial.print("File open / completion times: ");
  Serial.print(oTime);
  Serial.print(" ms / ");
  Serial.println(cTime);
  Serial.print("Busy: ");
  Serial.print(std::min(100 * (wTimeTot + fTimeTot + dTimeTot + oTime + cTime) / vidDuration, (uint32_t)100));
  Serial.println("%");
  Serial.println("*************************************");
  return true;
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
  Serial.print("SD storage time ");
  Serial.print(wTime);
  Serial.println(" ms");

  // whats left or small frame
  memcpy(iSDbuffer+highPoint, fb->buf + jpegSize - jpegRemain, jpegRemain);
  highPoint += jpegRemain;
  
  buildAviIdx(jpegSize, true, false); // save avi index for frame
  vidSize += jpegSize + CHUNK_HDR;
  frameCnt++; 
  fTime = millis() - fTime - wTime;
  fTimeTot += fTime;

  Serial.print("Frame processing time ");
  Serial.print(fTime);
  Serial.println(" ms");
}

static boolean getFrame() { // originally processFrame()

  Serial.print(".");

  uint32_t dTime = millis();
  camera_fb_t* fb = esp_camera_fb_get();
  if (fb == NULL) return false;

  if (isCapturing) {

    if (!wasCapturing) {
      openAvi(); // start new recording
    } else {
      dTimeTot += millis() - dTime;
      saveFrame(fb);

      if (((millis() - startTime) >= 15000) || (frameCnt >= maxFrames)) {
        closeAvi(); // end current recording
        isCapturing = false;
      }
    }

  }

  if (fb != NULL) esp_camera_fb_return(fb);
  fb = NULL;

  wasCapturing = isCapturing;
  
  return true;
}

// Tasks and ISR

static void IRAM_ATTR frameISR() {
  // interrupt at current frame rate
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(captureHandle, &xHigherPriorityTaskWoken); // wake capture task to process frame
  if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}


void controlFrameTimer(bool restartTimer) {
  // frame timer control, timer3 so dont conflict with cam
  static hw_timer_t* timer3 = NULL;
  // stop current timer
  if (timer3) {
    timerAlarmDisable(timer3);   
    timerDetachInterrupt(timer3); 
    timerEnd(timer3);
  }
  if (restartTimer) {
    // (re)start timer 3 interrupt per required framerate
    timer3 = timerBegin(3, 8000, true); // 0.1ms tick
    frameInterval = 10000 / FPS; // in units of 0.1ms

    Serial.print("Frame timer interval ");
    Serial.print(frameInterval/10);
    Serial.print(" ms for FPS ");
    Serial.println(FPS); 

    timerAlarmWrite(timer3, frameInterval, true); 
    timerAlarmEnable(timer3);
    timerAttachInterrupt(timer3, &frameISR, true);
  }
}

static void captureTask(void* parameter) {
  // woken by frame timer when time to capture frame
  uint32_t ulNotifiedValue;
  while (true) {
    ulNotifiedValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (ulNotifiedValue > 5) ulNotifiedValue = 5; // prevent too big queue if FPS excessive
    // may be more than one isr outstanding if the task delayed by SD write or jpeg decode
    while (ulNotifiedValue-- > 0) getFrame();
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

static void startSDtasks() {
  // tasks to manage SD card operation
  aviMutex = xSemaphoreCreateMutex();
  xTaskCreate(&captureTask, "captureTask", 1024 * 4, NULL, 5, &captureHandle);
  sensor_t * s = esp_camera_sensor_get();
  fsizePtr = s->status.framesize; 
  //setFPS(frameData[fsizePtr].defaultFPS); // initial frames per second
  setFPS(10);
}

void startRecording() {
  isCapturing = true;
}

void stopRecording() {
  isCapturing = false;
}

void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // initialise camera
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_HVGA;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[!] Camera init failed with error 0x%x", err);
    while(true) {}
  }

  Serial.println("[+] Camera initialised!");

  // Initialize the SD card
  if (!SD.begin(SD_PIN_CS)) {
    Serial.println("[!] SD card initialization failed.");
    while(true) {}
  }
  
  uint8_t cardType = SD.cardType();

  // Determine if the type of SD card is available
  if(cardType == CARD_NONE){
    Serial.println("[!] No SD card attached.");
    while(true) {}
  }
  Serial.println("[+] SD card initialised!");

  Serial.println("[*] Setup complete!");

  strcpy(folderName, "videos");

  startSDtasks();

}

void loop() {
  while(!digitalRead(BUTTON_PIN)) {
    delay(50);
  }
  while(digitalRead(BUTTON_PIN)) {
    delay(50);
  }

  startRecording();

  while(!digitalRead(BUTTON_PIN)) {
    delay(50);
  }
  while(digitalRead(BUTTON_PIN)) {
    delay(50);
  }

  stopRecording();
}
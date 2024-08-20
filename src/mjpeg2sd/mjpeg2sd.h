#include <Arduino.h>
#include <FS.h>

/******************** Pins *******************/

// Camera
#define CAM_BOARD "CAMERA_MODEL_XIAO_ESP32S3"
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

/******************** Constants *******************/

#define RAMSIZE (1024 * 32) // set this to multiple of SD card sector size (512 or 1024 bytes)
#define MAX_FRAMES 30000
#define FB_BUFFERS 12 // 1 being processed, rest being filled
#define ONEMEG (1024 * 1024)
#define MAX_JPEG (ONEMEG / 2) // UXGA jpeg frame buffer at highest quality 375kB rounded up
#define CAPTURE_STACK_SIZE (1024 * 4)
#define AVI_HEADER_LEN 310 // AVI header length
#define CHUNK_HDR 8 // bytes per jpeg hdr in AVI 

/******************** Frame Size *******************/

// indexed by frame size - needs to be consistent with sensor.h framesize_t enum

struct frameStruct {
  const char* frameSizeStr;
  const int frameWidth;
  const int frameHeight;
};

const frameStruct frameData[14] = {
  {"96X96", 96, 96},   // 2MP sensors
  {"QQVGA", 160, 120},
  {"QCIF", 176, 144},
  {"HQVGA", 240, 176},
  {"240X240", 240, 240},
  {"QVGA", 320, 240},
  {"CIF", 400, 296},
  {"HVGA", 480, 320},
  {"VGA", 640, 480},
  {"SVGA", 800, 600},
  {"XGA", 1024, 768},
  {"HD", 1280, 720},
  {"SXGA", 1280, 1024},
  {"UXGA", 1600, 1200}
};

/******************** Variables *******************/

static File aviFile;
extern uint8_t aviHeader[];
extern const uint8_t dcBuf[]; // 00dc
extern int frameWidth;
extern int frameHeight;

/******************** Function declarations *******************/

void startVideo(char* fileName);
float stopVideo();

void showProgress(const char* marker = ".");
char* fmtSize (uint64_t sizeVal);
bool startCam(char vidRes[10]);
void buildAviHdr(uint8_t FPS, uint8_t frameType, uint16_t frameCnt);
void buildAviIdx(size_t dataSize, bool isVid = true);
void finalizeAviIndex(uint16_t frameCnt);
void prepAviIndex();
bool prepRecording(int setFPS);
size_t writeAviIndex(byte* clientBuf, size_t buffSize);
#include <Arduino.h>

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
#define FILE_NAME_LEN 64
#define PATH_NAME_LEN (FILE_NAME_LEN + 1 + FILE_NAME_LEN) // dir name + slash + file name
#define FB_BUFFERS 12 // 1 being processed, rest being filled
#define ONEMEG (1024 * 1024)
#define MAX_JPEG (ONEMEG / 2) // UXGA jpeg frame buffer at highest quality 375kB rounded up
#define CAPTURE_STACK_SIZE (1024 * 4)
#define AVI_HEADER_LEN 310 // AVI header length
#define CHUNK_HDR 8 // bytes per jpeg hdr in AVI 

/******************** Variables *******************/

extern int maxFrames;
extern uint8_t aviHeader[];
extern const uint8_t dcBuf[]; // 00dc

/******************** Function declarations *******************/

void startVideo(char* fileName);
void stopVideo();

void showProgress(const char* marker = ".");
char* fmtSize (uint64_t sizeVal);
bool startCam();
void buildAviHdr(uint8_t FPS, uint8_t frameType, uint16_t frameCnt, bool isTL = false);
void buildAviIdx(size_t dataSize, bool isVid = true, bool isTL = false);
void finalizeAviIndex(uint16_t frameCnt, bool isTL = false);
void prepAviIndex(bool isTL = false);
bool prepRecording();
uint8_t setFPS(uint8_t val);
size_t writeAviIndex(byte* clientBuf, size_t buffSize, bool isTL = false);

/******************** Structures *******************/

// indexed by frame size - needs to be consistent with sensor.h framesize_t enum

struct frameStruct {
  const char* frameSizeStr;
  const uint16_t frameWidth;
  const uint16_t frameHeight;
  const uint16_t defaultFPS;
  const uint8_t scaleFactor; // (0..4)
  const uint8_t sampleRate; // (1..N)
};

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
  {"UXGA", 1600, 1200, 5, 4, 1},  
  {"FHD", 920, 1080, 5, 3, 1},    // 3MP Sensors
  {"P_HD", 720, 1280, 5, 3, 1},
  {"P_3MP", 864, 1536, 5, 3, 1},
  {"QXGA", 2048, 1536, 5, 4, 1},
  {"QHD", 2560, 1440, 5, 4, 1},   // 5MP Sensors
  {"WQXGA", 2560, 1600, 5, 4, 1},
  {"P_FHD", 1080, 1920, 5, 4, 1},
  {"QSXGA", 2560, 1920, 4, 4, 1}
};
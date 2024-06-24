// Global MJPEG2SD declarations
//
// s60sc 2021, 2022

#pragma once
#include "globals.h"

/*********************** Fixed defines leave as is ***********************/ 
/** Do not change anything below here unless you know what you are doing **/

#define FILE_NAME_LEN 64
#define IN_FILE_NAME_LEN (FILE_NAME_LEN * 2)
#define FB_BUFFERS 12 // 1 being processed, rest being filled
#define MAX_JPEG (ONEMEG / 2) // UXGA jpeg frame buffer at highest quality 375kB rounded up

#define STORAGE SD_MMC
#define RAMSIZE (1024 * 8) // set this to multiple of SD card sector size (512 or 1024 bytes)

#define AVI_EXT "avi"
#define CSV_EXT "csv"
#define SRT_EXT "srt"
#define AVI_HEADER_LEN 310 // AVI header length
#define CHUNK_HDR 8 // bytes per jpeg hdr in AVI 
#define WAVTEMP "/current.wav"
#define AVITEMP "/current.avi"

#define SD_MMC_CLK 7
#define SD_MMC_CMD 9
#define SD_MMC_D0 8

#define CAPTURE_STACK_SIZE (1024 * 4)


/******************** Function declarations *******************/


// global app specific functions

void buildAviHdr(uint8_t FPS, uint8_t frameType, uint16_t frameCnt, bool isTL = false);
void buildAviIdx(size_t dataSize, bool isVid = true, bool isTL = false);
void finalizeAviIndex(uint16_t frameCnt, bool isTL = false);
void finishAudio(bool isValid);
void prepAviIndex(bool isTL = false);
bool prepRecording();
uint8_t setFPS(uint8_t val);
uint8_t setFPSlookup(uint8_t val);
size_t writeAviIndex(byte* clientBuf, size_t buffSize, bool isTL = false);

/******************** Global app declarations *******************/

// motion detection parameters
extern int moveStartChecks; // checks per second for start motion
extern int moveStopSecs; // secs between each check for stop, also determines post motion time
extern int maxFrames; // maximum number of frames in video before auto close 

// motion recording parameters
extern int detectMotionFrames; // min sequence of changed frames to confirm motion 
extern int detectNightFrames; // frames of sequential darkness to avoid spurious day / night switching
extern int detectNumBands;
extern int detectStartBand;
extern int detectEndBand; // inclusive
extern int detectChangeThreshold; // min difference in pixel comparison to indicate a change
extern bool mlUse; // whether to use ML for motion detection, requires INCLUDE_TINYML to be true
extern float mlProbability; // minimum probability (0.0 - 1.0) for positive classification

// record timelapse avi independently of motion capture, file name has same format as avi except ends with T
extern int tlSecsBetweenFrames; // too short interval will interfere with other activities
extern int tlDurationMins; // a new file starts when previous ends
extern int tlPlaybackFPS;  // rate to playback the timelapse, min 1 

// status & control fields 
extern bool autoUpload;
extern bool dbgMotion;
extern bool doPlayback;
extern bool doRecording; // whether to capture to SD or not
extern bool forceRecord; // Recording enabled by rec button
extern bool forcePlayback; // playback enabled by user
extern uint8_t FPS;
extern uint8_t fsizePtr; // index to frameData[] for record
extern bool isCapturing;
extern uint8_t lightLevel;  
extern uint8_t lampLevel;  
extern int micGain;
extern uint8_t minSeconds; // default min video length (includes moveStopSecs time)
extern float motionVal;  // motion sensitivity setting - min percentage of changed pixels that constitute a movement
extern uint8_t nightSwitch; // initial white level % for night/day switching
extern bool nightTime; 
extern bool stopPlayback;
extern bool useMotion; // whether to use camera for motion detection (with motionDetect.cpp)  
extern uint8_t colorDepth;
extern bool timeLapseOn; // enable time lapse recording
extern int maxFrames;
extern uint8_t xclkMhz;
extern char camModel[];
extern bool doKeepFrame;
extern int alertMax; // too many could cause account suspension (daily emails)
extern bool nvrStream;
extern uint8_t numStreams;

// buffers
extern uint8_t iSDbuffer[];
extern uint8_t aviHeader[];
extern const uint8_t dcBuf[]; // 00dc
extern const uint8_t wbBuf[]; // 01wb
extern byte* uartData;
extern size_t streamBufferSize[];
extern byte* streamBuffer[]; // buffer for stream frame
extern size_t motionJpegLen;
extern uint8_t* motionJpeg;

// peripherals

// IO Extender use
extern bool useIOextender; // true to use IO Extender, otherwise false
extern bool useUART0;
extern int uartTxdPin;
extern int uartRxdPin;
// peripherals used
extern bool pirUse; // true to use PIR or radar sensor (RCWL-0516) for motion detection
extern bool lampUse; // true to use lamp
extern bool lampAuto; // if true in conjunction with usePir & useLamp, switch on lamp when PIR activated
extern bool lampNight;
extern int lampType;
extern bool servoUse; // true to use pan / tilt servo control
extern bool voltUse; // true to report on ADC pin eg for for battery
// microphone cannot be used on IO Extender
extern bool micUse; // true to use external I2S microphone 
extern bool wakeUse;

// task handling
extern TaskHandle_t battHandle;
extern TaskHandle_t captureHandle;


/************************** structures ********************************/

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

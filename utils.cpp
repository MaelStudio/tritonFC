#include "appGlobals.h"

bool dbgVerbose = false;
bool timeSynchronized = false;
bool monitorOpen = true;
bool dataFilesChecked = false;
// allow any startup failures to be reported via browser for remote devices
char startupFailure[SF_LEN] = {0};

/********************** misc functions ************************/

bool changeExtension(char* fileName, const char* newExt) {
  // replace original file extension with supplied extension (buffer must be large enough)
  size_t inNamePtr = strlen(fileName);
  // find '.' before extension text
  while (inNamePtr > 0 && fileName[inNamePtr] != '.') inNamePtr--;
  inNamePtr++;
  size_t extLen = strlen(newExt);
  memcpy(fileName + inNamePtr, newExt, extLen);
  fileName[inNamePtr + extLen] = 0;
  return (inNamePtr > 1) ? true : false;
}

void showProgress(const char* marker) {
  // show progess as dots 
  static uint8_t dotCnt = 0;
  logPrint(marker); // progress marker
  if (++dotCnt >= DOT_MAX) {
    dotCnt = 0;
    logLine();
  }
}

bool calcProgress(int progressVal, int totalVal, int percentReport, uint8_t &pcProgress) {
  // calculate percentage progress, only report back on percentReport boundary
  uint8_t percentage = (progressVal * 100) / totalVal;
  if (percentage >= pcProgress + percentReport) {
    pcProgress = percentage;
    return true;
  } else return false;
}

void listBuff (const uint8_t* b, size_t len) {
  // output buffer content as hex, 16 bytes per line
  if (!len || !b) LOG_WRN("Nothing to print");
  else {
    for (size_t i = 0; i < len; i += 16) {
      int linelen = (len - i) < 16 ? (len - i) : 16;
      for (size_t k = 0; k < linelen; k++) logPrint(" %02x", b[i+k]);
      puts(" ");
    }
  }
}

size_t isSubArray(uint8_t* haystack, uint8_t* needle, size_t hSize, size_t nSize) {
  // find a subarray (needle) in another array (haystack)
  size_t h = 0, n = 0; // Two pointers to traverse the arrays
  // Traverse both arrays simultaneously
  while (h < hSize && n < nSize) {
    // If element matches, increment both pointers
    if (haystack[h] == needle[n]) {
      h++;
      n++;
      // If needle is completely traversed
      if (n == nSize) return h; // position of end of needle
    } else {
      // if not, increment h and reset n
      h = h - n + 1;
      n = 0;
    }
  }
  return 0; // not found
}

void removeChar(char* s, char c) {
  // remove specified character from string
  int writer = 0, reader = 0;
  while (s[reader]) {
    if (s[reader] != c) s[writer++] = s[reader];
    reader++;       
  }
  s[writer] = 0;
}

void replaceChar(char* s, char c, char r) {
  // replace specified character in string
  int reader = 0;
  while (s[reader]) {
    if (s[reader] == c) s[reader] = r;
    reader++;       
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

void checkMemory(const char* source ) {
  LOG_INF("%s Free: heap %u, block: %u, min: %u, pSRAM %u", source, ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getMinFreeHeap(), ESP.getFreePsram());
  if (ESP.getFreeHeap() < WARN_HEAP) LOG_WRN("Free heap only %u, min %u", ESP.getFreeHeap(), ESP.getMinFreeHeap());
  if (ESP.getMaxAllocHeap() < WARN_ALLOC) LOG_WRN("Max allocatable heap block is only %u", ESP.getMaxAllocHeap());
}

uint32_t checkStackUse(TaskHandle_t thisTask, int taskIdx) {
  // get minimum free stack size for task since started
  static uint32_t minStack[20]; 
  uint32_t freeStack = 0;
  if (thisTask != NULL) {
    freeStack = (uint32_t)uxTaskGetStackHighWaterMark(thisTask);
    if (!minStack[taskIdx]) minStack[taskIdx] = freeStack; // initialise
    if (freeStack < minStack[taskIdx]) {
      minStack[taskIdx] = freeStack;
      if (freeStack < MIN_STACK_FREE) LOG_WRN("Task %s stack space only: %u", pcTaskGetTaskName(thisTask), freeStack);
      else LOG_INF("Task %s stack space reduced to %u", pcTaskGetTaskName(thisTask), freeStack);
    }
  }
  return freeStack;
}

void debugMemory(const char* caller) {
  if (DEBUG_MEM) {
    logPrint("%s > Free: heap %u, block: %u, min: %u, pSRAM %u\n", caller, ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getMinFreeHeap(), ESP.getFreePsram());
    delay(FLUSH_DELAY);
  }
}

void doRestart(const char* restartStr) {
  LOG_ALT("Controlled restart: %s", restartStr);
  flush_log(true);
  delay(2000);
  ESP.restart();
}

uint16_t smoothAnalog(int analogPin, int samples) {
  // get averaged analog pin value 
  uint32_t level = 0; 
  if (analogPin > 0) {
    for (int j = 0; j < samples; j++) level += analogRead(analogPin); 
    level /= samples;
  }
  return level;
}

void setupADC() {
  analogSetAttenuation(ADC_ATTEN);
  analogReadResolution(ADC_BITS);
}

float smoothSensor(float latestVal, float smoothedVal, float alpha) {
  // simple Exponential Moving Average filter 
  // where alpha between 0.0 (max smooth) and 1.0 (no smooth)
  return (latestVal * alpha) + smoothedVal * (1.0 - alpha);
}

/*********************** Remote loggging ***********************/
/*
 * Log mode selection in user interface: 
 * false : log to serial / web monitor only
 * true  : also saves log on SD card. To download the log generated, either:
 *  - To view the log, press Show Log button on the browser
 * - To clear the log file contents, on log web page press Clear Log link
 */
 
#define MAX_OUT 200
static va_list arglist;
static char fmtBuf[MAX_OUT];
static char outBuf[MAX_OUT];
char alertMsg[MAX_OUT];
TaskHandle_t logHandle = NULL;
static SemaphoreHandle_t logSemaphore = NULL;
static SemaphoreHandle_t logMutex = NULL;
static int logWait = 100; // ms
bool useLogColors = false;  // true to colorise log messages (eg if using idf.py, but not arduino)
bool wsLog = false;

#define WRITE_CACHE_CYCLE 5
bool sdLog = false; // log to SD
int logType = 0; // which log contents to display (0 : ram, 1 : sd, 2 : ws)
static FILE* log_remote_fp = NULL;
static uint32_t counter_write = 0;

// RAM memory based logging in RTC slow memory
RTC_NOINIT_ATTR bool ramLog; // log to RAM
RTC_NOINIT_ATTR uint16_t mlogEnd; // cannot init here
RTC_NOINIT_ATTR char messageLog[RAM_LOG_LEN];

static void ramLogClear() {
  mlogEnd = 0;
  memset(messageLog, 0, RAM_LOG_LEN);
}
  
static void ramLogStore(size_t msgLen) {
  // save log entry in ram buffer
  if (mlogEnd + msgLen >= RAM_LOG_LEN) {
    // log needs to roll around cyclic buffer
    uint16_t firstPart = RAM_LOG_LEN - mlogEnd;
    memcpy(messageLog + mlogEnd, outBuf, firstPart);
    msgLen -= firstPart;
    memcpy(messageLog, outBuf + firstPart, msgLen);
    mlogEnd = 0;
  } else memcpy(messageLog + mlogEnd, outBuf, msgLen);
  mlogEnd += msgLen;
}

void flush_log(bool andClose) {
  if (log_remote_fp != NULL) {
    fsync(fileno(log_remote_fp));  
    fflush(log_remote_fp);
    if (andClose) {
      LOG_INF("Closed SD file for logging");
      fclose(log_remote_fp);
      log_remote_fp = NULL;
    } else delay(1000);
  }  
}

static void remote_log_init_SD() {
#if !CONFIG_IDF_TARGET_ESP32C3
  STORAGE.mkdir(DATA_DIR);
  // Open remote file
  log_remote_fp = NULL;
  log_remote_fp = fopen("/sdcard" LOG_FILE_PATH, "a");
  if (log_remote_fp == NULL) {LOG_ERR("Failed to open SD log file %s", LOG_FILE_PATH);}
  else {
    logPrint(" \n\n");
    LOG_INF("Opened SD file for logging");
  }
#endif
}

void reset_log() {
  if (logType == 0) ramLogClear();
  if (logType == 2) {
    if (log_remote_fp != NULL) flush_log(true); // Close log file
    STORAGE.remove(LOG_FILE_PATH);
    remote_log_init_SD();
  }
  if (logType != 1) LOG_INF("Cleared %s log file", logType == 0 ? "RAM" : "SD"); 
}

void remote_log_init() {
  // setup required log mode
  if (sdLog) {
    flush_log(false);
    remote_log_init_SD(); // store log on sd card
  } else flush_log(true);
}

static void logTask(void *arg) {
  // separate task to reduce stack size in other tasks
  while(true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    vsnprintf(outBuf, MAX_OUT, fmtBuf, arglist);
    va_end(arglist);
    xSemaphoreGive(logSemaphore);
  }
}

void logPrint(const char *format, ...) {
  // feeds logTask to format message, then outputs as required
  if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(logWait)) == pdTRUE) {
    strncpy(fmtBuf, format, MAX_OUT);
    va_start(arglist, format); 
    vTaskPrioritySet(logHandle, uxTaskPriorityGet(NULL) + 1);
    xTaskNotifyGive(logHandle);
    outBuf[MAX_OUT - 2] = '\n'; 
    outBuf[MAX_OUT - 1] = 0; // ensure always have ending newline
    xSemaphoreTake(logSemaphore, portMAX_DELAY); // wait for logTask to complete        
    // output to monitor console if attached
    size_t msgLen = strlen(outBuf);
    if (outBuf[msgLen - 2] == '~') {
      // set up alert message for browser
      outBuf[msgLen - 2] = ' ';
      strncpy(alertMsg, outBuf, MAX_OUT - 1);
      alertMsg[msgLen - 2] = 0;
    }
    if (monitorOpen) Serial.print(outBuf); 
    else delay(10); // allow time for other tasks
    if (sdLog) {
      if (log_remote_fp != NULL) {
        // output to SD if file opened
        fwrite(outBuf, sizeof(char), msgLen, log_remote_fp); // log.txt
        // periodic sync to SD
        if (counter_write++ % WRITE_CACHE_CYCLE == 0) fsync(fileno(log_remote_fp));
      } 
    }
    if (ramLog) ramLogStore(msgLen); // store in ram 
    // output to web socket if open
    if (msgLen > 1) {
      outBuf[msgLen - 1] = 0; // lose final '/n'
      if (wsLog) wsAsyncSend(outBuf);
    }
    xSemaphoreGive(logMutex);
  } 
}

void formatHex(const char* inData, size_t inLen) {
  // format data as hex bytes for output
  char formatted[(inLen * 3) + 1];
  for (int i=0; i<inLen; i++) sprintf(formatted + (i*3), "%02x ", inData[i]);
  formatted[(inLen * 3)] = 0; // terminator
  LOG_INF("Hex: %s", formatted);
}

const char* espErrMsg(esp_err_t errCode) {
  // convert esp error code to text
  static char errText[100];
  esp_err_to_name_r(errCode, errText, 100);
  return errText;
}

void forceCrash() {
  // force crash for testing purposes
  delay(5000);
#pragma GCC diagnostic ignored "-Wdiv-by-zero"
  printf("%u\n", 1/0);
#pragma GCC diagnostic warning "-Wdiv-by-zero"
}

/****************** base 64 ******************/

#define BASE64 "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"

const uint8_t* encode64chunk(const uint8_t* inp, int rem) {
  // receive 3 byte input buffer and return 4 byte base64 buffer
  rem = 3 - rem; // last chunk may be less than 3 bytes 
  uint32_t buff = 0; // hold 3 bytes as shifted 24 bits
  static uint8_t b64[4];
  // shift input into buffer
  for (int i = 0; i < 3 - rem; i++) buff |= inp[i] << (8*(2-i)); 
  // shift 6 bit output from buffer and encode
  for (int i = 0; i < 4 - rem; i++) b64[i] = BASE64[buff >> (6*(3-i)) & 0x3F]; 
  // filler for last chunk if less than 3 bytes
  for (int i = 0; i < rem; i++) b64[3-i] = '='; 
  return b64;
}

const char* encode64(const char* inp) {
  // helper to base64 encode strings up to 90 chars long
  static char encoded[121]; // space for 4/3 expansion + terminator
  encoded[0] = 0;
  int len = strlen(inp);
  if (len > 90) {
    LOG_WRN("Input string too long: %u chars", len);
    len = 90;
  }
  for (int i = 0; i < len; i += 3) 
    strncat(encoded, (char*)encode64chunk((uint8_t*)inp + i, min(len - i, 3)), 4);
  return encoded;
}
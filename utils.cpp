#include "appGlobals.h"

bool dbgVerbose = false;
bool monitorOpen = true;
char startupFailure[SF_LEN] = {0};

/********************** misc functions ************************/

void dateFormat(char* inBuff, size_t inBuffLen, bool isFolder) {
  // construct filename from date/time
  time_t currEpoch = getEpoch();
  if (isFolder) strftime(inBuff, inBuffLen, "/%Y%m%d", localtime(&currEpoch));
  else strftime(inBuff, inBuffLen, "/%Y%m%d/%Y%m%d_%H%M%S", localtime(&currEpoch));
}

time_t getEpoch() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec;
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

void debugMemory(const char* caller) {
  if (DEBUG_MEM) {
    logPrint("%s > Free: heap %u, block: %u, min: %u, pSRAM %u\n", caller, ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getMinFreeHeap(), ESP.getFreePsram());
    delay(FLUSH_DELAY);
  }
}



/*********************** loggging ***********************/

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
    if (ramLog) ramLogStore(msgLen); // store in ram
    xSemaphoreGive(logMutex);
  } 
}

void logLine() {
  logPrint(" \n");
}

void logSetup() {
  // prep logging environment
  Serial.begin(115200);
  Serial.setDebugOutput(DBG_ON);
  printf("\n\n=============== %s %s ===============\n", APP_NAME, APP_VER);
  if (DEBUG_MEM) printf("init > Free: heap %u\n", ESP.getFreeHeap()); 
  logSemaphore = xSemaphoreCreateBinary(); // flag that log message formatted
  logMutex = xSemaphoreCreateMutex(); // control access to log formatter
  xSemaphoreGive(logSemaphore);
  xSemaphoreGive(logMutex);
  xTaskCreate(logTask, "logTask", LOG_STACK_SIZE, NULL, 1, &logHandle);
  if (mlogEnd >= RAM_LOG_LEN) ramLogClear(); // init
  LOG_INF("Setup RAM based log, size %u, starting from %u\n\n", RAM_LOG_LEN, mlogEnd);
  wakeupResetReason();
  debugMemory("logSetup"); 
}

/****************** send device to sleep (light or deep) & watchdog ******************/

static esp_sleep_wakeup_cause_t printWakeupReason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 : LOG_INF("Wakeup by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : LOG_INF("Wakeup by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : LOG_INF("Wakeup by internal timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : LOG_INF("Wakeup by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : LOG_INF("Wakeup by ULP program"); break;
    case ESP_SLEEP_WAKEUP_GPIO: LOG_INF("Wakeup by GPIO"); break;    
    case ESP_SLEEP_WAKEUP_UART: LOG_INF("Wakeup by UART"); break; 
    default : LOG_INF("Wakeup by reset"); break;
  }
  return wakeup_reason;
}

static esp_reset_reason_t printResetReason() {
  esp_reset_reason_t bootReason = esp_reset_reason();
  switch (bootReason) {
    case ESP_RST_UNKNOWN: LOG_INF("Reset for unknown reason"); break;
    case ESP_RST_POWERON: LOG_INF("Power on reset"); break;
    case ESP_RST_EXT: LOG_INF("Reset from external pin"); break;
    case ESP_RST_SW: LOG_INF("Software reset via esp_restart"); break;
    case ESP_RST_PANIC: LOG_INF("Software reset due to exception/panic"); break;
    case ESP_RST_INT_WDT: LOG_INF("Reset due to interrupt watchdog"); break;
    case ESP_RST_TASK_WDT: LOG_INF("Reset due to task watchdog"); break;
    case ESP_RST_WDT: LOG_INF("Reset due to other watchdogs"); break;
    case ESP_RST_DEEPSLEEP: LOG_INF("Reset after exiting deep sleep mode"); break;
    case ESP_RST_BROWNOUT: LOG_INF("Brownout reset (software or hardware)"); break;
    case ESP_RST_SDIO: LOG_INF("Reset over SDIO"); break;
    default: LOG_WRN("Unhandled reset reason"); break;
  }
  return bootReason;
}

esp_sleep_wakeup_cause_t wakeupResetReason() {
  printResetReason();
  esp_sleep_wakeup_cause_t wakeupReason = printWakeupReason();
  return wakeupReason;
}
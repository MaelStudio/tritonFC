// Global generic declarations
//
// s60sc 2021, 2022

/******************** Libraries *******************/

#include "Arduino.h"
#include <driver/i2s.h>
#include <vector>
#include <regex>
#include <SD_MMC.h>
#include <LittleFS.h>

#define FILLSTAR "****************************************************************"
#define DELIM '~'
#define ONEMEG (1024 * 1024)
#define SF_LEN 100
#define RAM_LOG_LEN (1024 * 7) // size of system message log in bytes stored in slow RTC ram (max 8KB - vars)
#define MIN_STACK_FREE 512

// global general utility functions
time_t getEpoch();
void logLine();
void logPrint(const char *fmtStr, ...);
void logSetup();
void showProgress(const char* marker = ".");
esp_sleep_wakeup_cause_t wakeupResetReason();

/******************** Global utility declarations *******************/


void dateFormat(char* inBuff, size_t inBuffLen, bool isFolder);
char* fmtSize (uint64_t sizeVal);

// app status
extern char timezone[];
extern char ntpServer[];
extern uint8_t alarmHour;
extern char* jsonBuff;
extern bool dbgVerbose;
extern bool sdLog;
extern char alertMsg[];
extern bool ramLog;
extern int logType;
extern char messageLog[];
extern uint16_t mlogEnd;
extern bool timeSynchronized;
extern bool monitorOpen; 
extern const char* setupPage_html;
extern const char* otaPage_html;
extern SemaphoreHandle_t wsSendMutex;
extern char startupFailure[];
extern time_t currEpoch;

extern UBaseType_t uxHighWaterMarkArr[];

// SD storage
extern int sdMinCardFreeSpace; // Minimum amount of card free Megabytes before freeSpaceMode action is enabled
extern int sdFreeSpaceMode; // 0 - No Check, 1 - Delete oldest dir, 2 - Upload to ftp and then delete folder on SD 
extern bool formatIfMountFailed ; // Auto format the file system if mount failed. Set to false to not auto format.

/*********************** Log formatting ************************/

#define LOG_COLOR_ERR
#define LOG_COLOR_WRN
#define LOG_COLOR_DBG
#define LOG_NO_COLOR

#define INF_FORMAT(format) "[%s %s] " format "\n", esp_log_system_timestamp(), __FUNCTION__
#define LOG_INF(format, ...) logPrint(INF_FORMAT(format), ##__VA_ARGS__)
#define LOG_ALT(format, ...) logPrint(INF_FORMAT(format "~"), ##__VA_ARGS__)
#define WRN_FORMAT(format) LOG_COLOR_WRN "[%s WARN %s] " format LOG_NO_COLOR "\n", esp_log_system_timestamp(), __FUNCTION__
#define LOG_WRN(format, ...) logPrint(WRN_FORMAT(format "~"), ##__VA_ARGS__)
#define ERR_FORMAT(format) LOG_COLOR_ERR "[%s ERROR @ %s:%u] " format LOG_NO_COLOR "\n", esp_log_system_timestamp(), pathToFileName(__FILE__), __LINE__
#define LOG_ERR(format, ...) logPrint(ERR_FORMAT(format "~"), ##__VA_ARGS__)
#define DBG_FORMAT(format) LOG_COLOR_DBG "[%s DEBUG @ %s:%u] " format LOG_NO_COLOR "\n", esp_log_system_timestamp(), pathToFileName(__FILE__), __LINE__
#define LOG_DBG(format, ...) if (dbgVerbose) logPrint(DBG_FORMAT(format), ##__VA_ARGS__)
#define CHK_FORMAT(format) LOG_COLOR_ERR "[###### CHECK @ %s:%u] " format LOG_NO_COLOR "\n", pathToFileName(__FILE__), __LINE__
#define LOG_CHK(format, ...) do { logPrint(CHK_FORMAT(format), ##__VA_ARGS__); delay(FLUSH_DELAY); } while (0)
#define LOG_PRT(buff, bufflen) log_print_buf((const uint8_t*)buff, bufflen)

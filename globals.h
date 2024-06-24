// Global generic declarations
//
// s60sc 2021, 2022

#pragma once
// to compile with -Wall -Werror=all -Wextra
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
//#pragma GCC diagnostic ignored "-Wunused-variable"
//#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
//#pragma GCC diagnostic ignored "-Wignored-qualifiers"
//#pragma GCC diagnostic ignored "-Wclass-memaccess"

/******************** Libraries *******************/

#include "Arduino.h"
#include <driver/i2s.h>
#include <vector>
#include <regex>
#include <SD_MMC.h>
#include <LittleFS.h>

// ADC
#define ADC_ATTEN ADC_11db
#define ADC_SAMPLES 16
#if CONFIG_IDF_TARGET_ESP32S3
#define ADC_BITS 13
#define MAX_ADC 8191 // maximum ADC value at given resolution
#else
#define ADC_BITS 12
#define MAX_ADC 4095 // maximum ADC value at given resolution
#endif
#define CENTER_ADC (MAX_ADC / 2) 

// data folder defs
#define DATA_DIR "/data"
#define HTML_EXT ".htm"
#define TEXT_EXT ".txt"
#define JS_EXT ".js"
#define CSS_EXT ".css"
#define ICO_EXT ".ico"
#define SVG_EXT ".svg"
#define CONFIG_FILE_PATH DATA_DIR "/configs" TEXT_EXT
#define LOG_FILE_PATH DATA_DIR "/log" TEXT_EXT
#define OTA_FILE_PATH DATA_DIR "/OTA" HTML_EXT
#define COMMON_JS_PATH DATA_DIR "/common" JS_EXT 
#define GITHUB_HOST "raw.githubusercontent.com"

#define FILLSTAR "****************************************************************"
#define DELIM '~'
#define ONEMEG (1024 * 1024)
#define MAX_PWD_LEN 64
#define MAX_HOST_LEN 32
#define MAX_IP_LEN 16
#define BOUNDARY_VAL "123456789000000000000987654321"
#define SF_LEN 100
#define RAM_LOG_LEN (1024 * 7) // size of system message log in bytes stored in slow RTC ram (max 8KB - vars)
#define MIN_STACK_FREE 512

// global general utility functions in utils.cpp / utilsFS.cpp / peripherals.cpp    
void buildJsonString(uint8_t filter);
bool calcProgress(int progressVal, int totalVal, int percentReport, uint8_t &pcProgress);
bool changeExtension(char* fileName, const char* newExt);
bool checkAlarm();
bool checkDataFiles();
bool checkFreeStorage();
void checkMemory(const char* source = "");
uint32_t checkStackUse(TaskHandle_t thisTask, int taskIdx);
void debugMemory(const char* caller);
void dateFormat(char* inBuff, size_t inBuffLen, bool isFolder);
void deleteFolderOrFile(const char* deleteThis);
void devSetup();
void doAppPing();
void doRestart(const char* restartStr);
void emailAlert(const char* _subject, const char* _message);
const char* encode64(const char* inp);
const uint8_t* encode64chunk(const uint8_t* inp, int rem);
const char* espErrMsg(esp_err_t errCode);
void externalAlert(const char* subject, const char* message);
bool externalPeripheral(byte pinNum, uint32_t outputData = 0);
void flush_log(bool andClose = false);
char* fmtSize (uint64_t sizeVal);
void forceCrash();
void formatElapsedTime(char* timeStr, uint32_t timeVal, bool noDays = false);
void formatHex(const char* inData, size_t inLen);
bool fsFileOrFolder(const char* fileFolder);
const char* getEncType(int ssidIndex);
void getExtIP();
time_t getEpoch();
size_t getFreeStorage();
bool getLocalNTP();
float getNTCcelsius(uint16_t resistance, float oldTemp);
void getOldestDir(char* oldestDir);
void goToSleep(int wakeupPin, bool deepSleep);
void initStatus(int cfgGroup, int delayVal);
void killWebSocket();
void listBuff(const uint8_t* b, size_t len); 
bool listDir(const char* fname, char* jsonBuff, size_t jsonBuffLen, const char* extension);
bool loadConfig();
void logLine();
void logPrint(const char *fmtStr, ...);
void logSetup();
void OTAprereq();
bool parseJson(int rxSize);
void prepPeripherals();
void prepSMTP();
bool prepTelegram();
void prepTemperature();
void prepUart();
void prepUpload();
void reloadConfigs();
float readTemperature(bool isCelsius);
float readVoltage();
void remote_log_init();
void removeChar(char* s, char c);
void replaceChar(char* s, char c, char r);
void reset_log();
void resetWatchDog();
bool retrieveConfigVal(const char* variable, char* value);
void setFolderName(const char* fname, char* fileName);
void setPeripheralResponse(const byte pinNum, const uint32_t responseData);
void setupADC();
void showProgress(const char* marker = ".");
uint16_t smoothAnalog(int analogPin, int samples = ADC_SAMPLES);
float smoothSensor(float latestVal, float smoothedVal, float alpha);
void startOTAtask();
void startSecTimer(bool startTimer);
bool startStorage();
void startWebServer();
bool startWifi(bool firstcall = true);
void stopPing();
void syncToBrowser(uint32_t browserUTC);
bool updateConfigVect(const char* variable, const char* value);
void updateStatus(const char* variable, const char* _value);
void urlDecode(char* inVal);
uint32_t usePeripheral(const byte pinNum, const uint32_t receivedData);
esp_sleep_wakeup_cause_t wakeupResetReason();
void wsAsyncSend(const char* wsData);
// mqtt.cpp
void startMqttClient();  
void stopMqttClient();  
void mqttPublish(const char* payload);
// telegram.cpp
bool getTgramUpdate(char* response);
bool sendTgramMessage(const char* info, const char* item, const char* parseMode);
bool sendTgramPhoto(uint8_t* photoData, size_t photoSize, const char* caption);
bool sendTgramFile(const char* fileName, const char* contentType, const char* caption);
void tgramAlert(const char* subject, const char* message);

/******************** Global utility declarations *******************/

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

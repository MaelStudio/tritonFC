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
#include "esp_camera.h"
#include "camera_pins.h"

#define FILLSTAR "****************************************************************"
#define DELIM '~'
#define ONEMEG (1024 * 1024)
#define SF_LEN 100
#define MIN_STACK_FREE 512

// global general utility functions
time_t getEpoch();
void showProgress(const char* marker = ".");
esp_sleep_wakeup_cause_t wakeupResetReason();

/******************** Global utility declarations *******************/


void dateFormat(char* inBuff, size_t inBuffLen, bool isFolder);
char* fmtSize (uint64_t sizeVal);

extern char startupFailure[];
extern time_t currEpoch;

extern UBaseType_t uxHighWaterMarkArr[];

// SD storage
extern int sdMinCardFreeSpace; // Minimum amount of card free Megabytes before freeSpaceMode action is enabled
extern int sdFreeSpaceMode; // 0 - No Check, 1 - Delete oldest dir, 2 - Upload to ftp and then delete folder on SD 
extern bool formatIfMountFailed ; // Auto format the file system if mount failed. Set to false to not auto format.
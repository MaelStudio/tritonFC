#include "appGlobals.h"

char camModel[10];

static void startStorage() {

  // if (psramFound()) heap_caps_malloc_extmem_enable(MIN_RAM); // small number to force vector into psram
  // fileVec.reserve(1000);
  // if (psramFound()) heap_caps_malloc_extmem_enable(MAX_RAM);

  SD_MMC.begin("/sdcard", true, formatIfMountFailed);

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) LOG_WRN("No SD card attached");
  else {
    char typeStr[8] = "UNKNOWN";
    if (cardType == CARD_MMC) strcpy(typeStr, "MMC");
    else if (cardType == CARD_SD) strcpy(typeStr, "SDSC");
    else if (cardType == CARD_SDHC) strcpy(typeStr, "SDHC");
    LOG_INF("SD card type %s, Size: %s", typeStr, fmtSize(SD_MMC.cardSize()));
  }
}

static void prepCam() {
  // initialise camera depending on model and board
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
  // init with high specs to pre-allocate larger buffers
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.frame_size = FRAMESIZE_HVGA; // 8M
  config.jpeg_quality = 10;
  config.fb_count = FB_BUFFERS;

  // camera init
  if (psramFound()) {
    esp_err_t err = ESP_FAIL;
    uint8_t retries = 2;
    while (retries && err != ESP_OK) {
      err = esp_camera_init(&config);
      if (err != ESP_OK) {
        // power cycle the camera, provided pin is connected
        digitalWrite(PWDN_GPIO_NUM, 1);
        delay(100);
        digitalWrite(PWDN_GPIO_NUM, 0); 
        delay(100);
        retries--;
      }
    } 
    if (err != ESP_OK) snprintf(startupFailure, SF_LEN, "Startup Failure: Camera init error 0x%x", err);
    else {
      sensor_t * s = esp_camera_sensor_get();
      switch (s->id.PID) {
        case (OV2640_PID):
          strcpy(camModel, "OV2640");
        break;
        case (OV3660_PID):
          strcpy(camModel, "OV3660");
        break;
        case (OV5640_PID):
          strcpy(camModel, "OV5640");
        break;
        default:
          strcpy(camModel, "Other");
        break;
      }
      LOG_INF("Camera init OK for model %s on board %s", camModel, CAM_BOARD);

      s->set_framesize(s, FRAMESIZE_HVGA);
    }
  }
  debugMemory("prepCam");
}

void setup() {
  pinMode(D0, INPUT_PULLUP);

  logSetup();
  startStorage();
  prepCam();
  prepMic();
  prepRecording();
  
  LOG_INF("Camera model %s on board %s ready @ %uMHz", camModel, CAM_BOARD, xclkMhz);
  checkMemory();
}

void loop() {
  while (!digitalRead(D0)) {
    delay(10);
  }
  while (digitalRead(D0)) {
    delay(10);
  }

  forceRecord = true;

  while (!digitalRead(D0)) {
    delay(10);
  }
  while (digitalRead(D0)) {
    delay(10);
  }

  forceRecord = false;
}

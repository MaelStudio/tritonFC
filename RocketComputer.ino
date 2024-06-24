#include "appGlobals.h"

char camModel[10];

bool startStorage() {

  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  bool res = SD_MMC.begin("/sdcard", true, true);

  if (!res) {
    Serial.println("[!] SD card mount failed.");
    return false;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("[!] No SD card attached");
    return false;
  }
  else {
    char typeStr[8] = "UNKNOWN";
    if (cardType == CARD_MMC) strcpy(typeStr, "MMC");
    else if (cardType == CARD_SD) strcpy(typeStr, "SDSC");
    else if (cardType == CARD_SDHC) strcpy(typeStr, "SDHC");
    Serial.printf("SD card mounted. Type: %s / Size: %s\n", typeStr, fmtSize(SD_MMC.cardSize()));
  }

  return res;
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
  config.frame_size = FRAMESIZE_HVGA;
  config.jpeg_quality = 10;
  config.fb_count = FB_BUFFERS;

  // camera init
  if (psramFound()) {
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("[!] Camera init failed with error 0x%x\n", err);
      while(true) {}
    }
    
    Serial.println("Camera initialized.");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(D0, INPUT_PULLUP);

  startStorage();
  prepCam();
  prepRecording();
  
  Serial.printf("Board ready @ %uMHz\n", xclkMhz);
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

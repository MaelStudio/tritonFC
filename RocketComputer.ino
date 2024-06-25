#include "globals.h"

void setup() {
  Serial.begin(115200);
  pinMode(D0, INPUT_PULLUP);

  if (startStorage()) Serial.printf("SD card mounted. Size: %s\n", fmtSize(SD_MMC.cardSize()));
  else Serial.println("[!] SD card initialization failed");

  if (startCam()) Serial.println("Camera initialized");
  else Serial.println("[!] Camera init failed");

  prepRecording();
  
  Serial.println("Setup complete");
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

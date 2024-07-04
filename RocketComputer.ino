#include "globals.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <SD_MMC.h>

Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;

#define SEA_LEVEL_HPA 1005.00
#define APOGEE_ALTITUDE_DIFF 1 // in meters, difference between highest recorded altitude and current altitude to trigger apogee detection

float highestAltitude = 0;
bool liftoff = false;
bool apogee = false;

void setup() {
  Serial.begin(115200);

  pinMode(D0, INPUT_PULLUP);

  if (startStorage()) Serial.printf("SD card mounted. Size: %s\n", fmtSize(SD_MMC.cardSize()));
  else Serial.println("[!] SD card initialization failed");

  if (!psramFound()) Serial.println("[!] PSRAM is disabled");

  if (startCam()) Serial.println("Camera initialized");
  else Serial.println("[!] Camera init failed");

  if (prepRecording()) Serial.println("Ready to record");
  else Serial.println("[!] Failed to get camera frame");

  if (mpu.begin()) Serial.println("MPU6050 initialized");
  else Serial.println("[!] MPU6050 init failed");

  if (bmp.begin(0x76)) Serial.println("BMP280 initialized");
  else Serial.println("[!] BMP280 init failed");

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1);   /* Standby time. */
  
  Serial.println("Setup complete");

}

void loop() {

  if (!liftoff && !digitalRead(D0)) { // with the actual launch detect cable: 0 = on launch pad / 1 = in the air
    liftoff = true;
    Serial.println("[*] Liftoff!");
  }

  // get IMU data
  sensors_event_t accel, gyro, t;
  mpu.getEvent(&accel, &gyro, &t);

  // get barometer data
  float pressure = bmp.readPressure();
  float altitude = bmp.readAltitude(SEA_LEVEL_HPA);
  float temp = bmp.readTemperature();

  if (liftoff && !apogee) {
    if (altitude > highestAltitude) {
      highestAltitude = altitude;
    }else if (highestAltitude - altitude > APOGEE_ALTITUDE_DIFF) {
      apogee = true;
      Serial.println("[*] Apogee!");
    }
  }

}

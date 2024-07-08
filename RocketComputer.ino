#include "globals.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <SD_MMC.h>
#include <ESP32Servo.h>


// pins
#define LAUNCH_SW_PIN D0
#define SERVO_PIN D1
#define BUZZER_PIN D2

// constants
#define SEA_LEVEL_HPA 1005.00
#define APOGEE_ALTITUDE_DIFF 1 // in meters, difference between highest recorded altitude and current altitude to trigger apogee detection
#define SERVO_HOME 0
#define SERVO_DEPLOY 180

// variables
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
Servo servo;

float highestAltitude = 0;
bool liftoff = false;
bool apogee = false;

bool initAll() {

  if (startStorage()) Serial.printf("SD card mounted. Size: %s\n", fmtSize(SD_MMC.cardSize()));
  else {
    Serial.println("[!] SD card initialization failed");
    return false;
  }

  if (startCam()) Serial.println("Camera initialized");
  else {
    Serial.println("[!] Camera init failed");
    return false;
  }

  if (prepRecording()) Serial.println("Ready to record");
  else {
    Serial.println("[!] Failed to get camera frame");
    return false;
  }

  if (mpu.begin()) Serial.println("MPU6050 initialized");
  else {
    Serial.println("[!] MPU6050 init failed");
    return false;
  }

  if (bmp.begin(0x76)) Serial.println("BMP280 initialized");
  else {
    Serial.println("[!] BMP280 init failed");
    return false;
  }

  return true;
}

void setup() {
  Serial.begin(115200);

  pinMode(LAUNCH_SW_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  servo.attach(SERVO_PIN);

  // alarm if initialization fails
  if (!initAll()) { 
    while (1) {
      tone(BUZZER_PIN, 2000, 120);
      delay(150);
    }
  }

  // set MPU6050 range
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // set BMP280 sampling mode
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1);   /* Standby time. */
  
  // set servo orientation
  servo.write(SERVO_HOME);
  delay(500);

  Serial.println("Setup complete");

  // play startup melody
  const int melody[] = {
    880, 1040, 1320, 1560
  };

  for (int i=0; i<4; i++) {
    tone(BUZZER_PIN, melody[i], 120);
  }
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
      servo.write(SERVO_DEPLOY);
      Serial.println("[*] Apogee!");
    }
  }

}

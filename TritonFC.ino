#include "globals.h"

// Libraries
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <SD_MMC.h>
#include <ESP32Servo.h>
#include <CircularBuffer.h>

// Pins
#define SERVO_PIN D0
#define BUZZER_PIN D1

// Constants
#define ACCEL_BUFFER_SIZE 50 // Size of the buffer used to calculate the average acceleration for launch detection
#define LAUNCH_DETECT_THRESHOLD 11.0 // In m/s^2 the vertical acceleration required to trigger launch detection
#define APOGEE_DETECT_THRESHOLD 1.0 // In meters, difference between highest recorded altitude and current altitude required to trigger apogee detection
#define SEA_LEVEL_HPA 1005.00
#define SERVO_HOME 0
#define SERVO_DEPLOY 180

// Sensors and servo
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
Servo servo;

// Variables
float highestAltitude = 0;
bool launch = false;
bool apogee = false;
unsigned long lastIdleBeep = 0;
CircularBuffer<float, ACCEL_BUFFER_SIZE> aYBuffer; // Define circular buffer for vertical acceleration readings

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

  Serial.println("\n====== Welcome to Triton FC! ======");

  pinMode(BUZZER_PIN, OUTPUT);

  // Alarm if initialization fails
  if (!initAll()) { 
    while (1) {
      tone(BUZZER_PIN, 2000, 120);
      delay(150);
    }
  }

  // Set MPU6050 range
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Set BMP280 sampling mode
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1);   /* Standby time. */
  
  // Set servo to home orientation
  servo.attach(SERVO_PIN);
  servo.write(SERVO_HOME);
  delay(800);
  servo.detach(); // free up timer to prevent conflicts with tone()

  // Play startup melody
  const int melody[] = {
    880, 1040, 1320, 1560
  };

  for (int i=0; i<4; i++) {
    tone(BUZZER_PIN, melody[i], 120);
    delay(120);
  }

  Serial.println("Setup complete");
  delay(2000);
}

void loop() {

  // Get time
  unsigned long now = millis();

  // Get barometer data
  float pressure = bmp.readPressure();
  float altitude = bmp.readAltitude(SEA_LEVEL_HPA);
  float temperature = bmp.readTemperature();

  // Get accelerometer and gyroscope data
  sensors_event_t accel, gyro, t;
  mpu.getEvent(&accel, &gyro, &t);

  float aX = accel.acceleration.x;
  float aY = accel.acceleration.y; // aY = vertical acceleration
  float aZ = accel.acceleration.z;
  float gX = gyro.gyro.x;
  float gY = gyro.gyro.y;
  float gZ = gyro.gyro.z;

  aYBuffer.push(aY); // Update the circular buffer with the new aY reading
  
  // Launch detection
  if (!launch) { // This runs while the rocket is idle on the pad, until launch

    // Beep every second while idle
    if(now - lastIdleBeep >= 1000) {
      tone(BUZZER_PIN, 2000, 60);
      lastIdleBeep = now;
    }
    
    // Calculate the average of the vertical acceleration buffer
    float sum = 0.0;
    for (int i = 0; i < ACCEL_BUFFER_SIZE; i++) {
      sum += aYBuffer[i];
    }
    float avgAY = sum / ACCEL_BUFFER_SIZE;

    if(avgAY >= LAUNCH_DETECT_THRESHOLD) {  // Compare average vertical acceleration to LAUNCH_DETECT_THRESHOLD
      launch = true;
      Serial.println("[*] Launch!");
    }
  }

  // Apogee detection
  if (launch && !apogee) { // This runs while the rocket has been launched, until apogee is reached
    if (altitude > highestAltitude) { // Keep track of highest recorded altitude
      highestAltitude = altitude;
    } else if (highestAltitude - altitude >= APOGEE_DETECT_THRESHOLD) { // Compare difference between highest recorded altitude and current altitude with APOGEE_DETECT_THRESHOLD
      apogee = true;
      servo.attach(SERVO_PIN);
      servo.write(SERVO_DEPLOY);
      Serial.println("[*] Apogee!");
    }
  }

}

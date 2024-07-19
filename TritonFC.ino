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
#define ALTITUDE_BUFFER_SIZE 50 // Size of the buffer used to calculate the average altitude for apogee detection
#define LAUNCH_DETECT_THRESHOLD 1.5 // In G's, the vertical acceleration required to trigger launch detection
#define APOGEE_DETECT_THRESHOLD 1.0 // In meters, difference between highest recorded altitude and current altitude required to trigger apogee detection
#define SERVO_HOME 0
#define SERVO_DEPLOY 180
#define ALPHA 0.98 // Complementary filter coefficient. 1
#define IMU_CALIBRATION_SAMPLES 500
#define BEEP_FREQ 2000

// Environment constants
#define SEA_LEVEL_HPA 1005.00
#define GRAVITY 9.80665

// Sensors and servo
Adafruit_MPU6050 imu;
Adafruit_BMP280 barometer;
Servo servo;

// Variables
bool apogee = false;
float highestAltitude = 0;
sensors_event_t accel, gyro, temp;
CircularBuffer<float, ACCEL_BUFFER_SIZE> aYBuffer; // Define circular buffer for vertical acceleration readings
CircularBuffer<float, ALTITUDE_BUFFER_SIZE> altitudeBuffer; // Define circular buffer for altitude readings
float yaw = 0;
float pitch = 0;
float roll = 0;
float accelVel = 0;

// imu offsets
float aX_offset = 0;
float aY_offset = 0;
float aZ_offset = 0;
float gX_offset = 0;
float gY_offset = 0;
float gZ_offset = 0;

void setup() {
  Serial.begin(115200);

  Serial.println("\n====== Welcome to Triton FC! ======");

  pinMode(BUZZER_PIN, OUTPUT);

  // Alarm if initialization fails
  if (!initAll()) { 
    while (1) {
      tone(BUZZER_PIN, BEEP_FREQ, 120);
      delay(150);
    }
  }

  // Set MPU6050 range
  imu.setAccelerometerRange(MPU6050_RANGE_16_G);
  imu.setGyroRange(MPU6050_RANGE_500_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Set BMP280 sampling mode
  barometer.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1);   /* Standby time. */
  
  // Set servo to home orientation
  servo.attach(SERVO_PIN);
  servo.write(SERVO_HOME);

  Serial.println("Calibrating IMU... Stay still!");
  calibrateIMU(IMU_CALIBRATION_SAMPLES); // IMU calibration also acts as a delay while servo is homing

  servo.detach(); // Free up timer to prevent conflicts with tone()

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

  // Detect launch
  bool launch = false;
  while(!launch) { // While the rocket is idle on the pad, until launch is detected
    // Get time
    unsigned long now = millis();
    static unsigned long lastIdleBeep = now;

    // Beep every second while idle
    if (now - lastIdleBeep >= 1000) {
      tone(BUZZER_PIN, 2000, 60);
      lastIdleBeep = now;
    }

    // Get accelerometer data
    imu.getEvent(&accel, &gyro, &temp);
    float aY = accel.acceleration.y - aY_offset; // aY = vertical acceleration
    aY /= GRAVITY; // Convert from m/s2 to G's
    aYBuffer.push(aY); // Update the circular buffer with the new aY reading

    // Calculate the average of the vertical acceleration buffer
    float sum = 0.0;
    for (int i = 0; i < ACCEL_BUFFER_SIZE; i++) {
      sum += aYBuffer[i];
    }
    float avgAY = sum / ACCEL_BUFFER_SIZE;

    if (avgAY >= LAUNCH_DETECT_THRESHOLD) {  // Compare average vertical acceleration to LAUNCH_DETECT_THRESHOLD
      launch = true;
      break; // Go to loop to start recording live flight data
    }
  }

  tone(BUZZER_PIN, BEEP_FREQ, 1000); // Beep for 1 second at launch
  Serial.println("[*] Launch!");
}

void loop() {

  // Get time
  unsigned long now = millis();
  static unsigned long previousTime = now;
  float dt = (now - previousTime) / 1000.0; // Get delta time
  previousTime = now;

  // Get barometer data
  float pressure = barometer.readPressure();
  float altitude = barometer.readAltitude(SEA_LEVEL_HPA);
  float temperature = barometer.readTemperature();

  altitudeBuffer.push(altitude); // Update the circular buffer with the new altitude reading

  // Get accelerometer and gyroscope data
  imu.getEvent(&accel, &gyro, &temp);

  float aX = accel.acceleration.x - aX_offset;
  float aY = accel.acceleration.y - aY_offset; // aY = vertical acceleration
  float aZ = accel.acceleration.z - aZ_offset;
  float gX = gyro.gyro.x * RAD_TO_DEG - gX_offset;
  float gY = gyro.gyro.y * RAD_TO_DEG - gY_offset;
  float gZ = gyro.gyro.z * RAD_TO_DEG - gZ_offset;

  float acceleration = aY / GRAVITY; // Get acceleration in G's

  // Calculate yaw and pitch from the accelerometer data
  float accelYaw = atan2(aX, sqrt(aY*aY + aZ*aZ)) * RAD_TO_DEG;
  float accelPitch = atan2(aY, aZ) * RAD_TO_DEG;
  accelPitch -= 90; // Adjust pitch to be 0 degrees when upright

  // Integrate the gyroscope data
  yaw += gZ * dt;
  pitch += gX * dt;
  roll += gY * dt;

  // Apply complementary filter: high-pass filter for the gyroscope and a low-pass filter for the accelerometer
  yaw = ALPHA * yaw + (1.0 - ALPHA) * accelYaw;
  pitch = ALPHA * pitch + (1.0 - ALPHA) * accelPitch;
  
  // Integrate vertical accel to get an estimation of velocity
  accelVel += (aY - GRAVITY) * dt;

  // Calculate vertical velocity from barometer
  static float lastAltitude = altitude;
  float baroVel = (altitude - lastAltitude) / dt; // Vertical velocity in m/s
  lastAltitude = altitude;

  // Apogee detection
  if (!apogee) { // This runs until apogee is reached
    // Calculate the average of the altitude buffer
    float sum = 0.0;
    for (int i = 0; i < ALTITUDE_BUFFER_SIZE; i++) {
      sum += altitudeBuffer[i];
    }
    float avgAltitude = sum / ALTITUDE_BUFFER_SIZE;

    if (avgAltitude > highestAltitude) { // Keep track of highest recorded altitude
      highestAltitude = avgAltitude;
    } else if (highestAltitude - avgAltitude >= APOGEE_DETECT_THRESHOLD) { // Compare difference between highest recorded altitude and current altitude with APOGEE_DETECT_THRESHOLD
      apogee = true;
      servo.attach(SERVO_PIN);
      servo.write(SERVO_DEPLOY);
      Serial.println("[*] Apogee!");
    }
  }
}

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

  if (imu.begin()) Serial.println("MPU6050 initialized");
  else {
    Serial.println("[!] MPU6050 init failed");
    return false;
  }

  if (barometer.begin(0x76)) Serial.println("BMP280 initialized");
  else {
    Serial.println("[!] BMP280 init failed");
    return false;
  }

  return true;
}

void calibrateIMU(int samples) {
  aX_offset = 0;
  aY_offset = 0;
  aZ_offset = 0;
  gX_offset = 0;
  gY_offset = 0;
  gZ_offset = 0;

  for (int i=0; i<samples; i++) {
    imu.getEvent(&accel, &gyro, &temp);
    aX_offset += accel.acceleration.x;
    aY_offset += accel.acceleration.y;
    aZ_offset += accel.acceleration.z;
    gX_offset += gyro.gyro.x * RAD_TO_DEG;
    gY_offset += gyro.gyro.y * RAD_TO_DEG;
    gZ_offset += gyro.gyro.z * RAD_TO_DEG;
    delay(1);
  }

  // Calculate averages
  aX_offset /= samples;
  aY_offset /= samples; aY_offset -= GRAVITY; // Gravity compensation for Y-axis
  aZ_offset /= samples;
  gX_offset /= samples;
  gY_offset /= samples;
  gZ_offset /= samples;

  Serial.printf("IMU calibrated with offsets: %.6f %.6f %.6f %.6f %.6f %.6f\n", aX_offset, aY_offset, aZ_offset, gX_offset, gY_offset, gZ_offset);
}
#include "globals.h"

// Libraries
#include <SD_MMC.h>
#include <FS.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>
#include <CircularBuffer.h>

// Pins
#define SERVO_PIN D0
#define BUZZER_PIN D1

// SD pins
#define SD_MMC_CLK 7
#define SD_MMC_CMD 9
#define SD_MMC_D0 8

// Constants
#define LOG_DIR_NAME "/flight_%u" // %u will be replaced by the flight number
#define LOG_FILE_NAME "/flight_%u_logs.csv"
#define AVI_FILE_NAME "/flight_%u.avi"
#define ACCEL_BUFFER_SIZE 50 // Size of the buffer used to calculate the average acceleration for launch detection
#define ALTITUDE_BUFFER_SIZE 50 // Size of the buffer used to calculate the average altitude for apogee detection
#define TIME_BUFFER_SIZE 10 // Must be <= ALTITUDE_BUFFER_SIZE. Size of the buffer used to calculate the vertical velocity based on altitude (baroVel2)
#define LAUNCH_DETECT_THRESHOLD 1.5 // In G's, the vertical acceleration required to trigger launch detection
#define APOGEE_DETECT_THRESHOLD 1.0 // In meters, difference between highest recorded altitude and current altitude required to trigger apogee detection
#define SERVO_HOME 0
#define SERVO_DEPLOY 180
#define ALPHA 0.98 // Complementary filter coefficient. 1
#define SENSORS_CALIBRATION_SAMPLES 500
#define BEEP_FREQ 2000

// Environment constants
#define SEA_LEVEL_HPA 1005.00
#define GRAVITY 9.80665

// Sensors and servo
Adafruit_MPU6050 imu;
Adafruit_BMP280 barometer;
Servo servo;

// SD
File logFile;
char logDir[FILE_NAME_LEN];
char logFilePath[PATH_NAME_LEN];
char aviFilePath[PATH_NAME_LEN];

// Variables
float launchTime = 0;
bool apogee = false;
float highestAltitude = 0;
sensors_event_t accel, gyro, temp;
CircularBuffer<float, ACCEL_BUFFER_SIZE> aYBuffer; // Define circular buffer for vertical acceleration readings
CircularBuffer<float, ALTITUDE_BUFFER_SIZE> altitudeBuffer; // Define circular buffer for altitude readings
CircularBuffer<float, ALTITUDE_BUFFER_SIZE> timeBuffer;
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

// altitude offset
float altitude_offset = 0;

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

  // Play startup melody
  const int melody[] = {
    880, 1040, 1320, 1560
  };

  for (int i=0; i<4; i++) {
    tone(BUZZER_PIN, melody[i], 120);
    delay(120);
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

  Serial.println("Calibrating sensors... Stay still!");
  calibrateSensors(SENSORS_CALIBRATION_SAMPLES); // IMU and barometer calibration also acts as a delay while servo is homing

  servo.detach(); // Free up timer to prevent conflicts with tone()

  // Find flight number
  int i = 1;
  snprintf(logDir, sizeof(logDir), LOG_DIR_NAME, i);
  snprintf(logFilePath, sizeof(logFilePath), "%s%s", logDir, LOG_FILE_NAME); // Add dir
  snprintf(logFilePath, sizeof(logFilePath), logFilePath, i); // Replace %u by i
  while(SD_MMC.exists(logFilePath)) {
    i++;
    snprintf(logDir, sizeof(logDir), LOG_DIR_NAME, i);
  snprintf(logFilePath, sizeof(logFilePath), "%s%s", logDir, LOG_FILE_NAME); // Add dir
  snprintf(logFilePath, sizeof(logFilePath), logFilePath, i); // Replace %u by i
  }

  // Create log dir
  SD_MMC.mkdir(logDir);

  // Add dir to avi file name
  snprintf(aviFilePath, sizeof(aviFilePath), "%s%s", logDir, AVI_FILE_NAME); // Add dir
  snprintf(aviFilePath, sizeof(aviFilePath), aviFilePath, i); // Replace %u by i

  Serial.println("Setup complete");

  // Detect launch
  bool launch = false;
  while(!launch) { // While the rocket is idle on the pad, until launch is detected
    // Get time
    float now = micros() / 1000000.0;
    static float lastIdleBeep = now;

    // Beep every second while idle
    if (now - lastIdleBeep >= 1) {
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

  launchTime = micros() / 1000000.0;

  tone(BUZZER_PIN, BEEP_FREQ, 1000); // Beep for 1 second at launch
  Serial.println("[*] Launch!");

  // Create CSV log file
  logFile = SD_MMC.open(logFilePath, FILE_WRITE);
  logFile.println("t,altitude,accel,baroVel,baroVel2,accelVel,yaw,pitch,roll,aX,aY,aZ,gX,gY,gZ,T,P,apogee"); // Write header line
  logFile.close();

  // Create avi file and start video recording
  startVideo(aviFilePath);
}

void loop() {
  /******************** Data collection *******************/

  // Get time in seconds
  float now = (micros() / 1000000.0) - launchTime;
  static float previousTime = now;
  float dt = now - previousTime; // Get delta time
  previousTime = now;
  timeBuffer.push(now);

  // Get barometer data
  float pressure = barometer.readPressure() / 100.0;
  float altitude = barometer.readAltitude(SEA_LEVEL_HPA) - altitude_offset;
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
  float baroVel2 = (altitude - altitudeBuffer[altitudeBuffer.size() - timeBuffer.size()]) / (now - timeBuffer[0]); // Vertical velocity in m/s, averaged from previous buffered altitude readings
  
  lastAltitude = altitude;

  /******************** Apogee detection *******************/

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
      stopVideo();
    }
  }

  /******************** Data logging to SD *******************/
  
  // t,altitude,accel,baroVel,baroVel2,accelVel,yaw,pitch,roll,aX,aY,aZ,gX,gY,gZ,T,P,apogee
  logFile = SD_MMC.open(logFilePath, FILE_APPEND);
  logFile.printf("%.3f", now);
  logFile.print(",");
  logFile.print(altitude);
  logFile.print(",");
  logFile.print(acceleration);
  logFile.print(",");
  logFile.print(baroVel);
  logFile.print(",");
  logFile.print(baroVel2);
  logFile.print(",");
  logFile.print(accelVel);
  logFile.print(",");
  logFile.print(yaw);
  logFile.print(",");
  logFile.print(pitch);
  logFile.print(",");
  logFile.print(roll);
  logFile.print(",");
  logFile.print(aX);
  logFile.print(",");
  logFile.print(aY);
  logFile.print(",");
  logFile.print(aZ);
  logFile.print(",");
  logFile.print(gX);
  logFile.print(",");
  logFile.print(gY);
  logFile.print(",");
  logFile.print(gZ);
  logFile.print(",");
  logFile.print(temperature);
  logFile.print(",");
  logFile.print(pressure);
  logFile.print(",");
  logFile.print(apogee);
  logFile.print("\n");
  logFile.close();

  // Calculate memory usage in percentages
  // float heapUsage = (float)(ESP.getHeapSize() - ESP.getFreeHeap()) / ESP.getHeapSize() * 100;
  // float psramUsage = (float)(ESP.getPsramSize() - ESP.getFreePsram()) / ESP.getPsramSize() * 100;
}

bool startStorage() {
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  return SD_MMC.begin("/sdcard", true, true);
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

// IMU and barometer calibration
// Barometer calibration sets ground altitude
void calibrateSensors(int samples) {
  aX_offset = 0;
  aY_offset = 0;
  aZ_offset = 0;
  gX_offset = 0;
  gY_offset = 0;
  gZ_offset = 0;
  altitude_offset = 0;

  for (int i=0; i<samples; i++) {
    imu.getEvent(&accel, &gyro, &temp);
    aX_offset += accel.acceleration.x;
    aY_offset += accel.acceleration.y;
    aZ_offset += accel.acceleration.z;
    gX_offset += gyro.gyro.x * RAD_TO_DEG;
    gY_offset += gyro.gyro.y * RAD_TO_DEG;
    gZ_offset += gyro.gyro.z * RAD_TO_DEG;
    altitude_offset += barometer.readAltitude(SEA_LEVEL_HPA);
    delay(1);
  }

  // Calculate averages
  aX_offset /= samples;
  aY_offset /= samples; aY_offset -= GRAVITY; // Gravity compensation for Y-axis
  aZ_offset /= samples;
  gX_offset /= samples;
  gY_offset /= samples;
  gZ_offset /= samples;
  altitude_offset /= samples;

  Serial.printf("IMU calibrated with offsets: %.6f %.6f %.6f %.6f %.6f %.6f\n", aX_offset, aY_offset, aZ_offset, gX_offset, gY_offset, gZ_offset);
}
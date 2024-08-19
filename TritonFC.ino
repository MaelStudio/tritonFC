// Libraries
#include <SD_MMC.h>
#include <FS.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <CircularBuffer.hpp>

#include "globals.h"

// Pins
#define BAT_PIN D0
#define BUZZER_PIN D1
#define LED_PIN D2
#define SERVO_PIN D3

// SD pins
#define SD_MMC_CLK 7
#define SD_MMC_CMD 9
#define SD_MMC_D0 8

// SD files
#define CONFIG_FILE "/config.cfg"
#define FILE_NAME_LEN 64
#define PATH_NAME_LEN (FILE_NAME_LEN + 1 + FILE_NAME_LEN) // dir name + slash + file name

// Battery
#define LOW_VOLTAGE_ALARM 7.4 // Battery voltage is checked at startup, the alarm will ring if the voltage is below this value (On a 2S lipo, 7.4V is 3.7V per cell)
#define BAT_DETECT_VOLTAGE 5.0 // Assume that the device is powered via USB if voltage is below this value (battery is unplugged)

// Sensor buffers
#define SENSORS_CALIBRATION_SAMPLES 500
#define ACCEL_BUFFER_SIZE 20 // Size of the buffer used to calculate the average acceleration for launch detection
#define ALTITUDE_BUFFER_SIZE 20 // Size of the buffer used to calculate the average altitude for apogee detection
#define TIME_BUFFER_SIZE 10 // Must be <= ALTITUDE_BUFFER_SIZE. Size of the buffer used to calculate the vertical velocity based on altitude

// Launch/apogee/landing detect parameters
#define LAUNCH_DETECT_THRESHOLD 1.5 // In G's, the vertical acceleration required to trigger launch detection
#define APOGEE_DETECT_THRESHOLD 1.0 // In meters, difference between highest recorded altitude and current altitude required to trigger apogee detection
#define LANDING_DETECT_TRESHOLD 0.5 // In meters, maximum velocity allowed to consider the rocket to be stable
#define LANDING_DETECT_DURATION 5.0 // In seconds, duration over which the velocity must stay below LANDING_DETECT_TRESHOLD to trigger landing detection
#define LANDING_APOGEE_DELAY 5.0 // In seconds, minimum delay between apogee and landing detection

// Servo positions
#define SERVO_HOME 0
#define SERVO_DEPLOY 180

// LED colors
#define LED_BRIGHTNESS 255 // LED Brightness from 0 to 255
#define COLOR_LOW_BAT 255, 0, 0 // Red
#define COLOR_SD 255, 255, 200 // White
#define COLOR_CAMERA 255, 40, 0 // Orange
#define COLOR_GET_FB 255, 150, 0 // Yellow
#define COLOR_IMU 0, 0, 255 // Blue
#define COLOR_BAROMETER 255, 0, 50 // Pink
#define COLOR_OK 0, 255, 0 // Green
#define COLOR_PAD_IDLE 0, 255, 0 // Green
#define COLOR_PAD_IDLE_FLASH 0, 50, 255 // Blue
#define COLOR_AIR_0 0, 0, 255 // Blue
#define COLOR_AIR_1 255, 0, 0 // Red
#define COLOR_ALTITUDE_FLASH 0, 0, 255 // Blue

// Misc constants
#define ALPHA 0.98 // Complementary filter coefficient. 1
#define BEEP_FREQ 2000 // Buzzer beep frequency, 2000 Hz is loudest

// Environment constants
#define SEA_LEVEL_HPA 1005.00
#define GRAVITY 9.80665

// Sensors and actuators
Adafruit_MPU6050 imu;
Adafruit_BMP280 barometer;
Servo servo;
Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);
byte lastColor[3];

// SD
static File logFile;
char logDir[FILE_NAME_LEN];
char logFilePath[PATH_NAME_LEN];
char aviFilePath[PATH_NAME_LEN];
char statsFilePath[PATH_NAME_LEN];

// Define default config
struct Config {
  bool buzzer = true;
  char vidRes[10] = "VGA";
  
  char logTemp[FILE_NAME_LEN] = "/current.csv";
  char aviTemp[FILE_NAME_LEN] = "/current.avi";

  // In file names, %i will be replaced by the flight number
  char logDir[FILE_NAME_LEN] = "/flight_%i";
  char statsFile[PATH_NAME_LEN] = "/flight_%i.csv";
  char logFile[PATH_NAME_LEN] = "/flight_%i_logs.csv";
  char aviFile[PATH_NAME_LEN] = "/flight_%i.avi";
};

Config config;

// Variables

// Buffers
CircularBuffer<float, ACCEL_BUFFER_SIZE> aYBuffer; // Define circular buffer for vertical acceleration readings
CircularBuffer<float, ALTITUDE_BUFFER_SIZE> altitudeBuffer; // Define circular buffer for altitude readings
CircularBuffer<float, ALTITUDE_BUFFER_SIZE> timeBuffer;

// Sensor data
sensors_event_t accel, gyro, temp;
float yaw = 0;
float pitch = 0;
float roll = 0;
float accelVel = 0;
bool stable;

// State
bool launch = false;
bool apogee = false;
bool landed = false;
bool parachute = false;

// Time
float launchTime;
float apogeeTime;
float deployTime;
float flightTime;
float stableStartTime;

// Stats
int logCount = 0;
float startupVoltage;
float highestAltitude = 0;
float maxVel = 0;
float maxAccel = 0;
float deployVel;
float vidFPS;

// imu offsets
float aX_offset;
float aY_offset;
float aZ_offset;
float gX_offset;
float gY_offset;
float gZ_offset;

// altitude offset
float altitude_offset;

void setup() {
  Serial.begin(115200);
  Serial.println("\n====== Welcome to Triton FC! ======");

  pinMode(BAT_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  led.begin();
  led.setBrightness(LED_BRIGHTNESS);

  // Alarm if initialization fails
  if (!initAll()) {

    while (1) {
      beep(BEEP_FREQ, 120);
      ledOn();
      delay(120);
      ledOff();
      delay(50);
    }
  }

  delay(300);

  // Play startup melody
  const int melody[] = {
    880, 1040, 1320, 1560
  };

  for (int i=0; i<4; i++) {
    beep(melody[i], 120);
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

  Serial.println("Setup complete");

  ledColor(COLOR_PAD_IDLE);

  // Detect launch
  while(!launch) { // While the rocket is idle on the pad, until launch is detected
    // Get time
    float now = micros() / 1000000.0;
    static float lastIdleBeep = now;

    // Beep and flash every second while idle
    if (now - lastIdleBeep >= 1) {
      beep(BEEP_FREQ, 60);
      ledColor(COLOR_PAD_IDLE_FLASH);
      lastIdleBeep = now;
    } else if (now - lastIdleBeep >= 0.1) {
      ledColor(COLOR_PAD_IDLE);
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

  launchTime = micros() / 1000000.0; // Save launch time
  
  Serial.println("[*] Launch!");

  ledColor(COLOR_AIR_0);

  // Create CSV log file
  logFile = SD_MMC.open(config.logTemp, FILE_WRITE);
  logFile.println("t,altitude,vel,accel,accelVel,yaw,pitch,roll,aX,aY,aZ,gX,gY,gZ,T,P,voltage,parachute"); // Write header line
  logFile.close();

  startVideo(config.aviTemp); // Create avi file and start video recording
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
  
  accelVel += (aY - GRAVITY) * dt; // Integrate vertical accel to get an estimation of velocity
  float baroVel = (altitude - altitudeBuffer[altitudeBuffer.size() - timeBuffer.size()]) / (now - timeBuffer[0]); // Vertical velocity in m/s, calculated from previous buffered altitude readings

  // Update flight stats
  if (baroVel > maxVel) maxVel = baroVel;
  if (acceleration > maxAccel) maxAccel = acceleration;

  // Get battery voltage
  float voltage;
  if (detectBattery()) voltage = batteryVoltage();
  else voltage = 0.0;

  /******************** RGB Led *******************/

  static float ledAirColorChangeTime = now;
  static bool ledAirColor = 0;

  if (now - ledAirColorChangeTime >= 0.06) {
    if (ledAirColor) ledColor(COLOR_AIR_0);
    else ledColor(COLOR_AIR_1);
    ledAirColor = !ledAirColor;
    ledAirColorChangeTime = now;
  }

  /******************** Apogee detection & Parachute deploy *******************/

  if (!apogee) { // This runs until apogee is detected
    // Calculate the average of the altitude buffer
    float sum = 0.0;
    for (int i = 0; i < ALTITUDE_BUFFER_SIZE; i++) {
      sum += altitudeBuffer[i];
    }
    float avgAltitude = sum / ALTITUDE_BUFFER_SIZE;

    if (avgAltitude > highestAltitude) { // Keep track of highest recorded altitude
      highestAltitude = avgAltitude;
      apogeeTime = now;
    } else if (highestAltitude - avgAltitude >= APOGEE_DETECT_THRESHOLD) { // Compare difference between highest recorded altitude and current altitude with APOGEE_DETECT_THRESHOLD
      apogee = true;
      servo.attach(SERVO_PIN);
      servo.write(SERVO_DEPLOY); // deploy parachute
      parachute = true;
      deployTime = now;
      deployVel = abs(baroVel);

      Serial.println("[*] Apogee!");
    }
  }

  /******************** Landing detection *******************/

  if (!landed && apogee && (now - apogeeTime) >= LANDING_APOGEE_DELAY) {
    if (abs(baroVel) <= LANDING_DETECT_TRESHOLD) {
      if (!stable) {
        stable = true;
        stableStartTime = now;
      } else if (now - stableStartTime >= LANDING_DETECT_DURATION) {
        landed = true;
        flightTime = now;

        Serial.println("[*] Landing!");

        vidFPS = stopVideo(); // close and save video file
        saveFlightData(); // save files in flight folder

        servo.detach(); // Free up timer to prevent conflicts with tone()
        while (1) {
          beepAltitude(highestAltitude); // beep out apogee
          delay(2000);
        }
      }
    } else {
      stable = false;
    }
  }

  /******************** Data logging to SD *******************/
  
  // t,altitude,vel,accel,accelVel,yaw,pitch,roll,aX,aY,aZ,gX,gY,gZ,T,P,voltage,parachute
  logFile = SD_MMC.open(config.logTemp, FILE_APPEND);
  logFile.printf("%.3f", now);
  logFile.print(',');
  logFile.print(altitude);
  logFile.print(',');
  logFile.print(baroVel);
  logFile.print(',');
  logFile.print(acceleration);
  logFile.print(',');
  logFile.print(accelVel);
  logFile.print(',');
  logFile.print(yaw);
  logFile.print(',');
  logFile.print(pitch);
  logFile.print(',');
  logFile.print(roll);
  logFile.print(',');
  logFile.print(aX);
  logFile.print(',');
  logFile.print(aY);
  logFile.print(',');
  logFile.print(aZ);
  logFile.print(',');
  logFile.print(gX);
  logFile.print(',');
  logFile.print(gY);
  logFile.print(',');
  logFile.print(gZ);
  logFile.print(',');
  logFile.print(temperature);
  logFile.print(',');
  logFile.print(pressure);
  logFile.print(',');
  logFile.print(voltage);
  logFile.print(',');
  logFile.print(parachute);
  logFile.print('\n');
  logFile.close();

  logCount++; // Keep count of the number of times new data is logged to SD card
}

bool startStorage() {
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  return SD_MMC.begin("/sdcard", true, true);
}

bool initAll() {
  ledColor(COLOR_SD);
  if (startStorage()) Serial.printf("SD card mounted. Size: %s\n", fmtSize(SD_MMC.cardSize()));
  else {
    Serial.println("[!] SD card initialization failed");
    return false;
  }

  // Load config
  if (SD_MMC.exists(CONFIG_FILE)) {
    loadConfig();
    Serial.printf("Loaded config from %s\n", CONFIG_FILE);
  } else {
    createDefaultConfig();
    Serial.printf("Created %s to store default config\n", CONFIG_FILE);
  }

  ledColor(COLOR_LOW_BAT);
  if (detectBattery()) {
    startupVoltage = batteryVoltage();
    if (startupVoltage > LOW_VOLTAGE_ALARM) Serial.printf("Battery is at %.2fV\n", startupVoltage);
    else {
      Serial.printf("[!] Low battery: %.2fV\n", startupVoltage);
      return false;
    }
  } else {
    Serial.println("No battery detected");
    startupVoltage = 0.0;
  }
  
  ledColor(COLOR_CAMERA);
  if (startCam(config.vidRes)) Serial.println("Camera initialized");
  else {
    Serial.println("[!] Camera init failed");
    return false;
  }

  ledColor(COLOR_GET_FB);
  if (prepRecording()) Serial.println("Ready to record");
  else {
    Serial.println("[!] Failed to get camera frame");
    return false;
  }

  ledColor(COLOR_IMU);
  if (imu.begin()) Serial.println("MPU6050 initialized");
  else {
    Serial.println("[!] MPU6050 init failed");
    return false;
  }

  ledColor(COLOR_BAROMETER);
  if (barometer.begin(0x76)) Serial.println("BMP280 initialized");
  else {
    Serial.println("[!] BMP280 init failed");
    return false;
  }

  ledColor(COLOR_OK);
  return true;
}

void createDefaultConfig() {
  File configFile = SD_MMC.open(CONFIG_FILE, FILE_WRITE);

  configFile.print("buzzer=");
  configFile.println(config.buzzer ? "true" : "false");
  configFile.print("vidRes=");
  configFile.println(config.vidRes);
  configFile.print("logTemp=");
  configFile.println(config.logTemp);
  configFile.print("aviTemp=");
  configFile.println(config.aviTemp);
  configFile.print("logDir=");
  configFile.println(config.logDir);
  configFile.print("statsFile=");
  configFile.println(config.statsFile);
  configFile.print("logFile=");
  configFile.println(config.logFile);
  configFile.print("aviFile=");
  configFile.println(config.aviFile);

  configFile.close();
}

void loadConfig() {
  File configFile = SD_MMC.open(CONFIG_FILE);

  while (configFile.available()) {
    String line = configFile.readStringUntil('\n');
    line.trim(); // Remove leading and trailing whitespace

    // Skip empty lines or comments
    if (line.length() == 0 || line.startsWith("#")) {
      continue;
    }

    int delimiterIndex = line.indexOf('=');
    if (delimiterIndex == -1) continue;

    String key = line.substring(0, delimiterIndex);
    String value = line.substring(delimiterIndex + 1);
    key.trim();
    value.trim();

    // Update the Config struct based on key-value pairs
    if (key == "buzzer") {
      config.buzzer = (value.equalsIgnoreCase("true")) ? true : false;
    } else if (key == "vidRes") {
      value.toCharArray(config.vidRes, sizeof(config.vidRes));
    } else if (key == "logTemp") {
      value.toCharArray(config.logTemp, sizeof(config.logTemp));
    } else if (key == "aviTemp") {
      value.toCharArray(config.aviTemp, sizeof(config.aviTemp));
    } else if (key == "logDir") {
      value.toCharArray(config.logDir, sizeof(config.logDir));
    } else if (key == "statsFile") {
      value.toCharArray(config.statsFile, sizeof(config.statsFile));
    } else if (key == "logFile") {
      value.toCharArray(config.logFile, sizeof(config.logFile));
    } else if (key == "aviFile") {
      value.toCharArray(config.aviFile, sizeof(config.aviFile));
    } else {
      Serial.printf("Unknown config key: %s\n", key);
    }
  }
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

void saveFlightData() {
  // Find flight number
  int i = 1;
  snprintf(logDir, sizeof(logDir), config.logDir, i);
  while(SD_MMC.exists(logDir)) {
    i++;
    snprintf(logDir, sizeof(logDir), config.logDir, i);
  }
  // Create log dir
  SD_MMC.mkdir(logDir);

  // Make file paths
  snprintf(logFilePath, sizeof(logFilePath), "%s%s", logDir, config.logFile); // Add dir
  snprintf(logFilePath, sizeof(logFilePath), logFilePath, i); // Replace %i by i
  snprintf(aviFilePath, sizeof(aviFilePath), "%s%s", logDir, config.aviFile); // Add dir
  snprintf(aviFilePath, sizeof(aviFilePath), aviFilePath, i); // Replace %i by i
  snprintf(statsFilePath, sizeof(statsFilePath), "%s%s", logDir, config.statsFile); // Add dir
  snprintf(statsFilePath, sizeof(statsFilePath), statsFilePath, i); // Replace %i by i

  // Rename files
  SD_MMC.rename(config.logTemp, logFilePath);
  SD_MMC.rename(config.aviTemp, aviFilePath);

  // Calculate files size
  unsigned long sdUsage = 0;
  logFile = SD_MMC.open(logFilePath);
  aviFile = SD_MMC.open(aviFilePath);
  sdUsage += logFile.size();
  sdUsage += aviFile.size();
  logFile.close();
  aviFile.close();

  int logHz = round(logCount / flightTime); // Calculate log frequency

  // Create stats file
  File statsFile = SD_MMC.open(statsFilePath, FILE_WRITE);
  statsFile.println("flightNum,apogee (m),maxVel (m/s),maxAccel (G),apogeeTime (s),deployTime (s),flightTime (s),deployVel (m/s),startupVoltage (V),sdUsage,videoRes,videoFPS,logFreq (Hz)"); // Write header line
  statsFile.print(i);
  statsFile.print(',');
  statsFile.print(highestAltitude);
  statsFile.print(',');
  statsFile.print(maxVel);
  statsFile.print(',');
  statsFile.print(maxAccel);
  statsFile.print(',');
  statsFile.print(apogeeTime);
  statsFile.print(',');
  statsFile.print(deployTime);
  statsFile.print(',');
  statsFile.print(flightTime);
  statsFile.print(',');
  statsFile.print(deployVel);
  statsFile.print(',');
  statsFile.print(startupVoltage);
  statsFile.print(',');
  statsFile.print(fmtSize(sdUsage));
  statsFile.print(',');
  statsFile.printf("%ix%i", frameWidth, frameHeight);
  statsFile.print(',');
  statsFile.printf("%.1f", vidFPS);
  statsFile.print(',');
  statsFile.print(logHz);
  statsFile.print('\n');
  statsFile.close();
}

float batteryVoltage() {
  return analogReadMilliVolts(BAT_PIN) / 1000.0 * (7.86/7.70) / 3.3 * 8.4;
}

bool detectBattery() {
  return batteryVoltage() > BAT_DETECT_VOLTAGE;
}

void ledColor(byte r, byte g, byte b) {
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();

  // update last color
  lastColor[0] = r;
  lastColor[1] = g;
  lastColor[2] = b;
}

void ledOff() {
  led.setPixelColor(0, led.Color(0, 0, 0));
  led.show();
}

void ledOn() {
  ledColor(lastColor[0], lastColor[1], lastColor[2]); // set led to last color
}

void beep(int hz, int duration) {
  if (!config.buzzer) return;
  tone(BUZZER_PIN, hz, duration);
}

void beepDigit(int n) {
  if (n == 0) n = 10; // Beep 10 times for a 0

  for (int i=0; i<n; i++) {
    beep(BEEP_FREQ, 80);
    ledColor(COLOR_ALTITUDE_FLASH); // Also flash LED
    delay(80);
    ledOff();
    delay(120);
  }
}

void beepAltitude(float alt) {

  // Round altitude and convert to a string
  int rounded = round(alt);
  String altStr = String(rounded);

  // Iterate through each digit characters in the string
  for (int i = 0; i < altStr.length(); i++) {
    char c = altStr[i];
    int digit = c - '0'; // Convert digit character to integer
    beepDigit(digit);
    if (i < altStr.length() - 1) delay(600);
  }
}
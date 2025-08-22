#include <Arduino.h>
#include <LittleFS.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_pm.h>
#include <driver/rtc_io.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/sens_reg.h>
#include <driver/adc.h>
#include <esp32/ulp.h>



#ifdef ESP32
extern HardwareSerial Serial;
#endif

// —————— Function Declarations ——————
void performFullCalibration();
void performGyroCalibration();
void recenterYaw();
void sendSteeringBin(float bin);
float applyYawDriftCompensation(float currentYaw);
void resetYawDrift();
void filterGyroReadings(float gx, float gy, float gz);
bool calibrationExists();
void performAutoCalibration();
void performAutoRecenter();
void initializeMahonyFilter();
void quickRecenterYaw();
bool detectMotion(float gx, float gy, float gz, float ax, float ay, float az);
bool hasSignificantAccumulatedMotion();
void updatePowerManagement(float gx, float gy, float gz, float ax, float ay, float az);
void updateLEDBreathing();
void configurePowerManagement();
void setupIMUWakeup();
void handleWakeupFromSleep();
void enterScreenOffMode();
void exitScreenOffMode();
void enterULPSleep();
void setupULPProgram();
void wakePulse();
void executeCentering();

// —————— NEW IMU CALIBRATION SYSTEM ——————
void startIMUCalibration();
void updateIMUCalibration(uint32_t countdown, bool clear = false);
void stopIMUCalibration();
bool loadIMUCalibration();
void saveIMUCalibration();

void enterBLEWaitingMode();
void enterLowPowerMode();
void enterBLEActiveMode();
void stopBLE();
void startBLE();

// —————— Pin & BLE UUIDs ——————
static const int LED_PIN = 19;    // onboard LED on GPIO19

#define STERZO_SERVICE_UUID "347b0001-7635-408b-8918-8ff3949ce592"
#define CHAR14_UUID         "347b0014-7635-408b-8918-8ff3949ce592"
#define CHAR30_UUID         "347b0030-7635-408b-8918-8ff3949ce592"
#define CHAR31_UUID         "347b0031-7635-408b-8918-8ff3949ce592"
#define CHAR32_UUID         "347b0032-7635-408b-8918-8ff3949ce592"

// —————— Mahony AHRS Implementation ——————
#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.0f) // 2 * integral gain

float twoKp = twoKpDef;    // 2 * proportional gain (Kp)
float twoKi = twoKiDef;    // 2 * integral gain (Ki)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;  // integral error terms scaled by Ki
float invSampleFreq;
char anglesComputed;

// —————— Globals ——————
Preferences    prefs;

// BLE objects
BLEServer*        pServer;
BLEService*       pSvc;
BLECharacteristic* pChar14;
BLECharacteristic* pChar30;
BLECharacteristic* pChar31;
BLECharacteristic* pChar32;
BLE2902*          p2902_14;
BLE2902*          p2902_30;
BLE2902*          p2902_32;

bool deviceConnected = false;
bool ind32On        = false;
bool challengeOK    = false;
bool zwiftConnected  = false; // Track if actively connected to Zwift

// IMU calibration & steering state
float gyroBiasX=0, gyroBiasY=0, gyroBiasZ=0;
float accelBiasX=0, accelBiasY=0, accelBiasZ=0;
float yawOffset   = 0;
float lastSentBin = NAN;
bool isCalibrating = false;

// —————— NEW IMU CALIBRATION SYSTEM ——————
// Calibration strength: 0=disabled, 1=weakest, 255=strongest
static constexpr const uint8_t IMU_CALIB_STRENGTH = 128;
static uint8_t imuCalibCountdown = 0;
static bool imuCalibrationActive = false;

// Debug configuration
const bool DEBUG_MODE = false; // Set to true for verbose debug output

// Yaw drift compensation
const float YAW_DRIFT_RATE = 0.2f; // degrees per second drift back to center
float lastYawTime = 0;      // last time yaw was processed
float accumulatedYawDrift = 0.0f;
bool yawDriftEnabled = true; // enable/disable drift compensation

// Gyro drift monitoring - 60-second trailing average
const int DRIFT_SAMPLES_COUNT = 60; // 60 samples for 60 seconds (1 sample per second)
float gyroDriftSamples[DRIFT_SAMPLES_COUNT];
int gyroDriftIndex = 0;
unsigned long lastDriftSampleTime = 0;
bool driftArrayFilled = false;

// Gyro filtering
float gyroFilterAlpha = 0.1f; // Low-pass filter coefficient (0.0 = no filtering, 1.0 = no change)
float filteredGx = 0, filteredGy = 0, filteredGz = 0;

// Timing for dynamic sample frequency
float dt, preTime;
float loopfreq;

// —————— NEW POWER MANAGEMENT SYSTEM ——————
// Power mode states
enum PowerMode {
  POWER_BLE_ACTIVE,    // BLE active, 80MHz CPU, screen on/off based on timer
  POWER_BLE_WAITING,   // Waiting for BLE connection, 80MHz CPU, reduced activity
  POWER_LOW_POWER,     // No BLE, 10MHz CPU, IMU/button monitoring only
  POWER_ULP_SLEEP      // ULP coprocessor monitoring, main CPU off
};
PowerMode currentPowerMode = POWER_BLE_WAITING;

// Timing constants
const unsigned long SCREEN_ON_TIMEOUT = 60000;    // 60 seconds screen on after button
const unsigned long BLE_WAIT_TIMEOUT = 300000;     // 5 minutes waiting for BLE connection
const unsigned long LOW_POWER_TIMEOUT = 120000;    // 2 minutes in low power before ULP sleep
const unsigned long NO_MOTION_TIMEOUT_CONNECTED = 300000; // 5 minutes (300s) of no motion while connected
// Timer wake removed - only button wake now

// Activity tracking
unsigned long lastButtonTime = 0;
unsigned long lastMotionTime = 0;
unsigned long lastBLEActivityTime = 0;
bool screenOn = false;

// IMU frequency management
const float NORMAL_IMU_FREQUENCY = 25.0f;     // 25Hz for normal operation
const float CALIBRATION_IMU_FREQUENCY = 25.0f; // 25Hz during calibration
float currentIMUFrequency = NORMAL_IMU_FREQUENCY;

// CPU frequency management
const int BLE_CPU_FREQ = 80;         // 80MHz minimum for BLE operation
const int LOW_POWER_CPU_FREQ = 80;   // 80MHz for I2C/IMU/button checks (no BLE)
const int DEEP_SLEEP_CPU_FREQ = 80;  // 80MHz for ULP wake checks

// LED breathing pattern
unsigned long ledBreathingStartTime = 0;
bool ledBreathingEnabled = true;

// Motion detection with noise filtering
const float MOTION_THRESHOLD = 40.0f; // degrees/second for gyro motion detection (increased for less sensitivity)
const float ACCEL_MOTION_THRESHOLD = 0.8f; // g-force for accelerometer motion detection (increased for less sensitivity)
float lastAccelMagnitude = 1.0f; // Initialize to 1g (gravity)
float lastGyroMagnitude = 0.0f;
float motionAccumulator = 0.0f;
unsigned long lastMotionAccumulatorReset = 0;

// Store the last measured raw yaw and timestamp from recentering
float lastMeasuredRawYaw = 0.0f;
unsigned long lastYawMeasurementTime = 0;
bool hasValidYawMeasurement = false;

// ULP coprocessor variables
RTC_DATA_ATTR int ulp_wake_count = 0;
RTC_DATA_ATTR int ulp_motion_detected = 0;
bool ulpProgramLoaded = false;
unsigned long bleStartTime = 0;

// —————— MODERN DISPLAY SYSTEM ——————
static LGFX_Sprite displaySprite(&M5.Display);
static unsigned long lastDisplayUpdate = 0;
static const unsigned long DISPLAY_UPDATE_INTERVAL = 100; // 10 FPS for smooth updates

// Display state tracking for efficient updates
static float lastDisplayedYaw = 999.0f;
static bool lastDisplayedConnected = false;
static PowerMode lastDisplayedPowerMode = POWER_ULP_SLEEP;
static float lastDisplayedBattery = 0.0f;
static bool lastDisplayedMotionActive = false;
static unsigned long overlayEndTime = 0;
static String overlayText = "";
static float smoothedBatteryVoltage = -1.0f; // For battery smoothing

// Helper function to map battery voltage to percentage
int getBatteryPercentage(float voltage) {
  // Simple linear mapping from 3.2V (empty) to 4.2V (full)
  float percentage = (voltage - 3.2f) / (4.2f - 3.2f) * 100.0f;
  return constrain((int)percentage, 0, 100);
}

// —————— Utility: 32-bit rotate + hash ——————
static uint32_t rotate_left32(uint32_t value, uint32_t count) {
  const uint32_t mask = (CHAR_BIT * sizeof(value)) - 1;
  count &= mask;
  return (value << count) | (value >> (-count & mask));
}

static uint32_t hashed(uint64_t seed) {
  uint32_t ret = (uint32_t)(seed + 0x16fa5717);
  uint64_t rax = seed * 0xba2e8ba3ULL;
  uint64_t eax = (rax >> 35) * 0xb;
  uint64_t ecx = seed - eax;
  uint32_t edx = rotate_left32((uint32_t)seed, (uint32_t)(ecx & 0x0F));
  return ret ^ edx;
}

// —————— Mahony AHRS Functions ——————
void initMahonyData() {
  twoKp = twoKpDef;  // 2 * proportional gain (Kp)
  twoKi = twoKiDef; // 2 * integral gain (Ki)
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
  integralFBx = 0.0f;
  integralFBy = 0.0f;
  integralFBz = 0.0f;
  anglesComputed = 0;
}

float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float samplefrequency) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Apply proportional feedback
    gx += 2.0f * halfex;
    gy += 2.0f * halfey;
    gz += 2.0f * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / samplefrequency));    // pre-multiply common factors
  gy *= (0.5f * (1.0f / samplefrequency));
  gz *= (0.5f * (1.0f / samplefrequency));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  anglesComputed = 0;
}

float getYaw() {
  // Calculate yaw directly from quaternion components
  // This is the correct formula for yaw (heading) from quaternion
  float yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
  
  // Convert to degrees
  yaw *= 57.29578f;
  
  // Normalize to [-180, 180] range
  if (yaw > 180) yaw -= 360;
  else if (yaw < -180) yaw += 360;
  
  // Invert yaw direction: clockwise rotation = positive, counter-clockwise = negative
  return -yaw;
}

float getRoll() {
  // Calculate roll directly from quaternion components
  float roll = atan2f(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
  return roll * 57.29578f;
}

float getPitch() {
  // Calculate pitch directly from quaternion components
  float pitch = asinf(-2.0f * (q1 * q3 - q0 * q2));
  return pitch * 57.29578f;
}

// —————— Power Management Functions ——————

void configurePowerManagement() {
  Serial.println("Configuring ESP32 power management...");
  
  // Configure automatic light sleep with BLE-compatible frequencies
  esp_pm_config_esp32_t pm_config;
  pm_config.max_freq_mhz = BLE_CPU_FREQ;  // 80MHz for BLE compatibility
  pm_config.min_freq_mhz = LOW_POWER_CPU_FREQ; // 10MHz minimum for I2C/GPIO
  pm_config.light_sleep_enable = true; // Enable automatic light sleep
  
  esp_err_t ret = esp_pm_configure(&pm_config);
  if (ret == ESP_OK) {
    Serial.println("Power management configured successfully");
  } else {
    Serial.printf("Power management configuration failed: %d\n", ret);
  }
  
  Serial.println("Power management configured for BLE compatibility");
}

void setupULPProgram() {
  Serial.println("Setting up ULP coprocessor program...");
  
  // Configure RTC GPIO for button wake-up only
  // M5StickCPlus2 buttons are active LOW (normally HIGH, go LOW when pressed)
  
  rtc_gpio_init(GPIO_NUM_37); // Button A
  rtc_gpio_set_direction(GPIO_NUM_37, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en(GPIO_NUM_37);
  rtc_gpio_pulldown_dis(GPIO_NUM_37);
  
  rtc_gpio_init(GPIO_NUM_39); // Button B
  rtc_gpio_set_direction(GPIO_NUM_39, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en(GPIO_NUM_39);
  rtc_gpio_pulldown_dis(GPIO_NUM_39);
  
  rtc_gpio_init(GPIO_NUM_35); // Button C
  rtc_gpio_set_direction(GPIO_NUM_35, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en(GPIO_NUM_35);
  rtc_gpio_pulldown_dis(GPIO_NUM_35);
  
  // Enable wake on Button A (ext0) and Button C (ext1)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_37, 0); // Button A: wake on LOW (button press)
  
  // Configure ext1 for Button A wake-up
  uint64_t ext1_mask = (1ULL << GPIO_NUM_37); // Button A only
  esp_sleep_enable_ext1_wakeup(ext1_mask, ESP_EXT1_WAKEUP_ALL_LOW); // Wake on Button A LOW
  
  ulpProgramLoaded = true;
  Serial.println("ULP program configured with button-only wake sources");
}

void setupIMUWakeup() {
  // For M5StickCPlus2, we'll use the ULP coprocessor approach
  // The ULP can periodically wake the main CPU to check IMU
  setupULPProgram();
  Serial.println("IMU wake-on-motion configured via ULP coprocessor");
}

void updateLEDBreathing() {
  if (!ledBreathingEnabled) {
    ledcWrite(0, 0); // Turn off LED
    return;
  }
  
  unsigned long currentTime = millis();
  
  // Different breathing patterns based on connection state
  float breathingPeriod;
  float maxBrightness = 12.75f; // 5% of 255 (5% brightness)
  
  if (deviceConnected) {
    // Quick flash every 1 second when connected
    if ((currentTime % 1000) < 100) { // On for 100ms
      ledcWrite(0, (int)maxBrightness);
    } else {
      ledcWrite(0, 0); // Off for 900ms
    }
  } else {
    breathingPeriod = 3000.0f; // 3 second breathing when not connected
    
    // Calculate breathing phase (0.0 to 1.0)
    float phase = fmod((currentTime - ledBreathingStartTime), breathingPeriod) / breathingPeriod;
    
    // Create smooth breathing pattern using sine wave
    float brightness = (sin(phase * 2.0f * PI - PI/2.0f) + 1.0f) / 2.0f; // 0.0 to 1.0
    brightness = brightness * maxBrightness; // Scale to max brightness
    
    ledcWrite(0, (int)brightness);
  }
}

void wakePulse() {
  // Quick 50ms pulse at 10% brightness to indicate wake check
  ledcWrite(0, 25); // 10% brightness
  delay(50);
  ledcWrite(0, 0);  // Turn off
}

// —————— INACTIVITY COUNTDOWN LOGGING ——————

void logInactivityStatus() {
  static unsigned long lastLogTime = 0;
  static unsigned long lastActivityTime = 0;
  static const char* lastActivityType = "none";
  
  unsigned long currentTime = millis();
  
  // Only log every 10 seconds to avoid spam
  if (currentTime - lastLogTime < 10000) {
    return;
  }
  
  // Check for new activity
  bool newActivity = false;
  const char* activityType = "none";
  
  // Check button activity
  if (currentTime - lastButtonTime < 5000) { // Recent button press
    if (currentTime - lastButtonTime < 1000) { // Very recent
      newActivity = true;
      activityType = "button_press";
    }
  }
  
  // Check Zwift connection
  if (zwiftConnected) {
    newActivity = true;
    activityType = "zwift_connected";
  }
  
  // Check motion activity
  if (currentTime - lastMotionTime < 5000) { // Recent motion
    newActivity = true;
    activityType = "motion_detected";
  }
  
  // Update activity tracking
  if (newActivity && strcmp(activityType, lastActivityType) != 0) {
    Serial.printf("ACTIVITY: %s detected - resetting timers\n", activityType);
    lastActivityType = activityType;
    lastActivityTime = currentTime;
  }
  
  // Calculate countdowns
  unsigned long timeSinceButton = currentTime - lastButtonTime;
  unsigned long timeSinceMotion = currentTime - lastMotionTime;
  unsigned long timeSinceBLEStart = currentTime - bleStartTime;
  
  // Screen off countdown
  unsigned long screenOffCountdown = 0;
  if (screenOn) {
    if (timeSinceButton <= SCREEN_ON_TIMEOUT) {
      screenOffCountdown = (SCREEN_ON_TIMEOUT - timeSinceButton) / 1000;
    } else if (zwiftConnected) {
      screenOffCountdown = 999; // Indefinite
    } else if (timeSinceMotion <= 30000) {
      screenOffCountdown = (30000 - timeSinceMotion) / 1000;
    } else {
      screenOffCountdown = 0; // Should be off
    }
  }
  
  // Deep sleep countdown
  unsigned long sleepCountdown = 0;
  switch (currentPowerMode) {
    case POWER_BLE_WAITING:
      if (timeSinceBLEStart > BLE_WAIT_TIMEOUT) {
        sleepCountdown = 0; // Should be in low power
      } else {
        sleepCountdown = (BLE_WAIT_TIMEOUT - timeSinceBLEStart) / 1000;
      }
      break;
    case POWER_LOW_POWER:
      {
        unsigned long timeSinceActivity = min(timeSinceMotion, timeSinceButton);
        if (timeSinceActivity > LOW_POWER_TIMEOUT) {
          sleepCountdown = 0; // Should be sleeping
        } else {
          sleepCountdown = (LOW_POWER_TIMEOUT - timeSinceActivity) / 1000;
        }
      }
      break;
    case POWER_BLE_ACTIVE:
      sleepCountdown = 999; // No sleep when connected
      break;
    default:
      sleepCountdown = 0;
      break;
  }
  
  // Store status for consolidated logging (will be printed elsewhere)
  // Serial.printf("INACTIVITY: Screen=%ds Sleep=%ds BLE=%s Motion=%s Zwift=%s\n",
  //               screenOffCountdown, sleepCountdown,
  //               deviceConnected ? "connected" : "waiting",
  //               (currentTime - lastMotionTime < 5000) ? "active" : "idle",
  //               zwiftConnected ? "active" : "idle");
  
  lastLogTime = currentTime;
}

// —————— MODERN DISPLAY FUNCTIONS ——————

void initializeDisplayBuffer() {
  // Create the sprite for double-buffering
  displaySprite.setColorDepth(8); // 8-bit color for performance
  displaySprite.createSprite(M5.Display.width(), M5.Display.height());
  displaySprite.setPaletteColor(1, TFT_WHITE);
  displaySprite.setPaletteColor(2, TFT_GREEN);
  displaySprite.setPaletteColor(3, TFT_RED);
  displaySprite.setPaletteColor(4, 0x39E7); // A nice blue for the background
}

void showOverlay(String text, int duration_ms) {
  overlayText = text;
  overlayEndTime = millis() + duration_ms;
}

// Splash overlay display for startup/wake with configurable top text, center image, and bottom text
void showSplashOverlay(String topText = "", String imagePath = "", String bottomText = "", int duration_ms = 2000) {
  if (!screenOn) {
    M5.Display.wakeup();
    M5.Display.setBrightness(13);
  }
  
  // Clear and setup display
  M5.Display.clear();
  M5.Display.setRotation(0);
  M5.Display.setTextColor(WHITE);
  M5.Display.setTextSize(2);
  M5.Display.setTextDatum(MC_DATUM);
  
  int screenCenterX = M5.Display.width() / 2;
  int screenHeight = M5.Display.height();
  
  // Display top text (center-top justified)
  if (topText.length() > 0) {
    M5.Display.setTextDatum(TC_DATUM); // Top-Center
    M5.Display.drawString(topText, screenCenterX, 10);
  }
  
  // Display center image from LittleFS
  if (imagePath.length() > 0) {
    String fullPath = "/" + imagePath;
    if (LittleFS.exists(fullPath)) {
      // M5StickC Plus2 display has non-square pixels:
      // Pixel width: 0.1101 mm, Pixel height: 0.1038 mm
      // Aspect ratio: 1.061 (pixels are wider than tall)
      // To display a square image correctly, we need to stretch vertically by this ratio
      
      int imageSize = 135; // Original square image size
      int imageX = screenCenterX - (imageSize / 2);
      int imageY = (screenHeight / 2) - (imageSize / 2);
      
      // Draw the image with aspect ratio correction using scale_x and scale_y
      // scale_x = 1.0 (no horizontal scaling)
      // scale_y = 1.061 (stretch vertically to compensate for non-square pixels)
      M5.Display.drawJpgFile(LittleFS, fullPath.c_str(), imageX, imageY, 0, 0, 0, 0, 1.0f, 1.061f);
    } else {
      // If image doesn't exist, show placeholder text
      M5.Display.setTextDatum(MC_DATUM); // Middle-Center
      M5.Display.setTextSize(1);
      M5.Display.drawString("Image not found:", screenCenterX, screenHeight / 2 - 10);
      M5.Display.drawString(imagePath, screenCenterX, screenHeight / 2 + 10);
      M5.Display.setTextSize(2); // Reset text size
    }
  }
  
  // Display bottom text (center-bottom justified)
  if (bottomText.length() > 0) {
    M5.Display.setTextDatum(BC_DATUM); // Bottom-Center
    M5.Display.drawString(bottomText, screenCenterX, screenHeight - 10);
  }
  
  if (duration_ms > 0) {
    delay(duration_ms);
  }
}

void clearOverlay() {
  overlayEndTime = 0;
  overlayText = "";
}

void drawModernDisplay(float yaw, bool connected, PowerMode powerMode, float battery, int freq) {
  // Update at consistent 10 FPS when screen is on
  if (!screenOn) {
    return;
  }
  
  if (millis() - lastDisplayUpdate < DISPLAY_UPDATE_INTERVAL) {
    return;
  }
  
  int yawInt = round(yaw);
  
  // Smooth battery voltage reading
  if (smoothedBatteryVoltage < 0) {
    smoothedBatteryVoltage = battery;
  } else {
    smoothedBatteryVoltage = smoothedBatteryVoltage * 0.9f + battery * 0.1f;
  }
  int batteryPercentage = getBatteryPercentage(smoothedBatteryVoltage);

  bool motionIsActive = (millis() - lastMotionTime <= 30000);

  // Check overlay status and detect when it just expired
  bool overlayActive = (millis() < overlayEndTime);
  static bool lastOverlayActive = false;
  bool overlayJustExpired = (lastOverlayActive && !overlayActive);
  
  // Check if anything has changed to avoid unnecessary redraws
  bool needsUpdate = false;
  if (abs(yawInt - round(lastDisplayedYaw)) > 0 ||
      connected != lastDisplayedConnected ||
      powerMode != lastDisplayedPowerMode ||
      motionIsActive != lastDisplayedMotionActive ||
      abs(batteryPercentage - round(lastDisplayedBattery)) > 0 ||
      overlayActive ||          // Update when overlay is active
      overlayJustExpired) {     // Update when overlay just expired (to clear it)
    needsUpdate = true;
  }
  
  // Update overlay state tracking
  lastOverlayActive = overlayActive;
  
  if (!needsUpdate) {
    return;
  }

  // --- Start Drawing to Off-Screen Buffer ---
  
  displaySprite.fillSprite(TFT_BLACK); // Clear background

  // Header - Row 1: Title
  displaySprite.setTextColor(TFT_WHITE);
  displaySprite.setTextDatum(TC_DATUM); // Top-Center
  displaySprite.setFont(&fonts::Font2);
  displaySprite.drawString("ZTEERStick", displaySprite.width()/2, 3);

  // Header - Row 2: Status Icons
  displaySprite.setTextDatum(TL_DATUM); // Top-Left
  displaySprite.setFont(&fonts::Font2);
  // BLE Status
  if (connected) {
    displaySprite.setTextColor(TFT_GREEN);
    displaySprite.drawString("BLE", 8, 22); 
  } else {
    displaySprite.setTextColor(TFT_RED);
    displaySprite.drawString("BLE", 8, 22);
  }

  // --- Battery Icon and Percentage ---
  displaySprite.setTextDatum(TR_DATUM); // Top-Right
  displaySprite.setTextColor(TFT_WHITE);
  String battStr = String(batteryPercentage) + "%";
  displaySprite.drawString(battStr, 110, 22);
  
  // Draw battery icon outline
  displaySprite.drawRect(115, 20, 18, 10, TFT_WHITE);
  displaySprite.fillRect(133, 23, 2, 4, TFT_WHITE);

  // Draw battery level
  int batteryLevelWidth = map(batteryPercentage, 0, 100, 0, 14);
  displaySprite.fillRect(117, 22, batteryLevelWidth, 6, (batteryPercentage < 20) ? TFT_RED : TFT_GREEN);

  // Main Yaw Value
  displaySprite.setFont(&fonts::Font7);
  displaySprite.setTextDatum(MC_DATUM); // Middle-Center

  // Custom centering for sign
  String yawNumStr = String(abs(yawInt));
  int textWidth = displaySprite.textWidth(yawNumStr);
  int signWidth = displaySprite.textWidth("-");
  int x_center = M5.Display.width() / 2;
  int x_pos_num = x_center + (yawInt < 0 ? signWidth / 2 : 0);

  bool atLimit = (yawInt < -39 || yawInt > 39);
  bool flashOn = (millis() / 500) % 2 == 0;
  
  if (atLimit && flashOn) {
      displaySprite.setTextColor(TFT_RED);
  } else {
      displaySprite.setTextColor(TFT_WHITE);
  }

  displaySprite.drawString(yawNumStr, x_pos_num, M5.Display.height() / 2);
  if (yawInt < 0) {
      displaySprite.drawString("-", x_pos_num - textWidth/2 - signWidth/2, M5.Display.height() / 2);
  }

  // --- Steering Gauge ---
  int gaugeWidth = 100;
  int gaugeX = (displaySprite.width() - gaugeWidth) / 2;
  int gaugeY = M5.Display.height() - 40;
  displaySprite.drawRect(gaugeX, gaugeY, gaugeWidth, 10, TFT_WHITE); // Outline
  
  // Calculate center position and indicator position
  int centerX = gaugeX + gaugeWidth / 2;
  int indicatorPos = map(yawInt, -40, 40, 0, gaugeWidth - 4);
  int indicatorX = gaugeX + 2 + indicatorPos;
  
  // Fill from center to indicator position
  if (yawInt != 0) {
    int fillStart, fillWidth;
    uint16_t fillColor;
    
    if (yawInt > 0) { // Right turn (positive yaw)
      fillStart = centerX;
      fillWidth = indicatorX - centerX;
      fillColor = TFT_BLUE; // Blue  for right
    } else { // Left turn (negative yaw)
      fillStart = indicatorX;
      fillWidth = centerX - indicatorX;
      fillColor = TFT_ORANGE; // Orange for left
    }
    
    // Only fill if there's actual width to fill
    if (fillWidth > 0) {
      displaySprite.fillRect(fillStart, gaugeY + 2, fillWidth, 6, fillColor);
    }
  }
  
  // Center line (draw after fill so it's always visible)
  displaySprite.drawFastVLine(centerX, gaugeY, 10, TFT_WHITE);

  // Footer
  displaySprite.setTextDatum(BC_DATUM); // Bottom-Center
  displaySprite.setFont(&fonts::Font2);
  displaySprite.setTextColor(TFT_WHITE);
  
  String powerStatus = "";
   if (zwiftConnected) {
    powerStatus = "CONNECTED";
  } else if (motionIsActive) {
    powerStatus = "ACTIVE";
  } else {
      switch(powerMode) {
          case POWER_BLE_ACTIVE: powerStatus = "CONNECTED"; break;
          case POWER_BLE_WAITING: powerStatus = "WAITING"; break;
          case POWER_LOW_POWER: powerStatus = "STANDBY"; break;
          default: powerStatus = "STANDBY"; break;
      }
  }
  displaySprite.drawString(powerStatus, x_center, M5.Display.height() - 8);

  // --- Draw Overlay if active ---
  if (millis() < overlayEndTime) {
    // Adjust overlay size based on text length for better readability
    int overlayWidth = max(80, min(120, (int)(overlayText.length() * 8 + 20)));
    int overlayX = (displaySprite.width() - overlayWidth) / 2;
    
    displaySprite.fillRoundRect(overlayX, 40, overlayWidth, 40, 8, TFT_DARKGREY);
    displaySprite.drawRoundRect(overlayX, 40, overlayWidth, 40, 8, TFT_WHITE); // Add border
    displaySprite.setTextDatum(MC_DATUM);
    displaySprite.setFont(&fonts::Font2);
    displaySprite.setTextColor(TFT_WHITE);
    displaySprite.drawString(overlayText, displaySprite.width() / 2, 60);
  }

  // --- Push Buffer to Screen ---
  displaySprite.pushSprite(0, 0);

  // Update tracking variables
  lastDisplayedYaw = yaw;
  lastDisplayedConnected = connected;
  lastDisplayedPowerMode = powerMode;
  lastDisplayedBattery = batteryPercentage;
  lastDisplayedMotionActive = motionIsActive;
  lastDisplayUpdate = millis();
}

bool detectMotion(float gx, float gy, float gz, float ax, float ay, float az) {
  // Calculate gyro magnitude (degrees/second)
  float gyroMagnitude = sqrt(gx*gx + gy*gy + gz*gz);
  
  // Calculate accelerometer magnitude (g-force)
  float accelMagnitude = sqrt(ax*ax + ay*ay + az*az);
  float accelDelta = abs(accelMagnitude - lastAccelMagnitude);
  
  // Calculate motion deltas with noise filtering
  float gyroDelta = abs(gyroMagnitude - lastGyroMagnitude);
  
  // Apply deadzone/noise filtering
  bool gyroMotion = gyroDelta > MOTION_THRESHOLD;
  bool accelMotion = accelDelta > ACCEL_MOTION_THRESHOLD;

  if (gyroMotion) {
    Serial.printf("MOTION: Gyroscope event. Value: %.2f dps > Threshold: %.2f dps\n", gyroDelta, MOTION_THRESHOLD);
  }
  if (accelMotion) {
    Serial.printf("MOTION: Accelerometer event. Value: %.2f g > Threshold: %.2f g\n", accelDelta, ACCEL_MOTION_THRESHOLD);
  }
  
  // Update last values with filtering
  float alpha = 0.7f;
  lastGyroMagnitude = alpha * lastGyroMagnitude + (1.0f - alpha) * gyroMagnitude;
  lastAccelMagnitude = alpha * lastAccelMagnitude + (1.0f - alpha) * accelMagnitude;
  
  // Accumulate significant motion for deep sleep decisions
  if (gyroMotion || accelMotion) {
    motionAccumulator += gyroDelta + (accelDelta * 5.0f);
  }
  
  // Reset accumulator every 30 seconds
  unsigned long currentTime = millis();
  if (currentTime - lastMotionAccumulatorReset > 30000) {
    motionAccumulator = 0;
    lastMotionAccumulatorReset = currentTime;
  }
  
  return gyroMotion || accelMotion;
}

bool hasSignificantAccumulatedMotion() {
  return motionAccumulator > 3.0f; // Threshold for significant movement
}

void startBLE() {
  Serial.println("Starting BLE...");
  
  // Configure BLE for low power
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  bt_cfg.mode = ESP_BT_MODE_BLE; // BLE only mode
  esp_bt_controller_init(&bt_cfg);
  esp_bt_controller_enable(ESP_BT_MODE_BLE);
  
  // Ensure CPU is at 80MHz for BLE
  setCpuFrequencyMhz(BLE_CPU_FREQ);
  
  Serial.println("BLE started, CPU at 80MHz");
}

void stopBLE() {
  Serial.println("Stopping BLE to save power...");
  
  // Stop BLE advertising and disable controller
  BLEDevice::deinit(false);
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  
  Serial.println("BLE stopped");
}

void enterBLEWaitingMode() {
  if (currentPowerMode != POWER_BLE_WAITING) {
    Serial.println("Entering BLE waiting mode");
    
    // Ensure BLE is running and CPU at 80MHz
    startBLE();
    setCpuFrequencyMhz(BLE_CPU_FREQ);
    
    // Turn off screen to save power while waiting
    if (screenOn) {
      M5.Display.clear();
      M5.Display.sleep();
      screenOn = false;
    }
    
    currentPowerMode = POWER_BLE_WAITING;
    bleStartTime = millis();
    
    Serial.printf("BLE waiting mode - CPU: %d MHz, Screen: OFF\n", BLE_CPU_FREQ);
  }
}

void enterBLEActiveMode() {
  if (currentPowerMode != POWER_BLE_ACTIVE) {
    Serial.println("Entering BLE active mode");
    
    // Ensure CPU at 80MHz for BLE
    setCpuFrequencyMhz(BLE_CPU_FREQ);
    
    currentPowerMode = POWER_BLE_ACTIVE;
    
    Serial.printf("BLE active mode - CPU: %d MHz\n", BLE_CPU_FREQ);
  }
}

void enterLowPowerMode() {
  if (currentPowerMode != POWER_LOW_POWER) {
    Serial.println("Entering low power mode (no BLE)");
    
    // Stop BLE to save power
    stopBLE();
    
    // Reduce CPU to 10MHz for I2C/GPIO operations
    setCpuFrequencyMhz(LOW_POWER_CPU_FREQ);
    
    // Turn off screen
    if (screenOn) {
      M5.Display.clear();
      M5.Display.sleep();
      screenOn = false;
    }
    
    // Reduce LED brightness further
    ledcWrite(0, 2); // Very dim breathing
    
    currentPowerMode = POWER_LOW_POWER;
    
    Serial.printf("Low power mode - CPU: %d MHz, BLE: OFF\n", LOW_POWER_CPU_FREQ);
  }
}

void enterScreenOffMode() {
  if (screenOn) {
    Serial.println("Turning off screen");
    
    // Turn off display
    M5.Display.clear();
    M5.Display.sleep();
    screenOn = false;
    
    Serial.println("Screen off");
  }
}

void exitScreenOffMode() {
  if (!screenOn) {
    Serial.println("Turning on screen");
    
    // Wake up display
    M5.Display.wakeup();
    M5.Display.setBrightness(13); // 5% brightness (13/255 ≈ 5%)
    screenOn = true;
    
    Serial.println("Screen on at 5% brightness");
  }
}

void enterULPSleep() {
  Serial.println("=== ENTERING ULP SLEEP MODE ===");
  
  // Show sleep message on screen briefly
  M5.Display.wakeup();
  M5.Display.setBrightness(50);
  M5.Display.clear();
  M5.Display.setRotation(0);
  M5.Display.setTextColor(ORANGE);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(10, 20);
  M5.Display.print("GOING TO");
  M5.Display.setCursor(10, 50);
  M5.Display.print("DEEP SLEEP");

  delay(3000); // Show for 3 seconds
  
  // Turn off everything
  M5.Display.clear();
  M5.Display.sleep();
  ledcWrite(0, 0);
  screenOn = false;
  
  // Stop BLE to save maximum power
  stopBLE();
  
  // Save current state
  prefs.begin("sterzo", false);
  prefs.putBool("wasAsleep", true);
  prefs.putULong("sleepTime", millis());
  prefs.end();
  
  currentPowerMode = POWER_ULP_SLEEP;
  
  // Configure ULP wake sources if not already done
  if (!ulpProgramLoaded) {
    setupULPProgram();
  }
  
  // Debug: Show current wake-up configuration
  Serial.printf("Wake-up sources configured:\n");
  Serial.printf("- ext0: GPIO37 (Button A) on LOW\n");
  Serial.printf("- ext1: GPIO35 (Button C) on LOW\n");
  Serial.printf("- No timer wake - button only\n");
  
  Serial.printf("Going to ULP sleep (button wake only)...\n");
  Serial.flush(); // Ensure all serial output is sent
  
  // Enter deep sleep with ULP monitoring
  esp_deep_sleep_start();
}

void handleWakeupFromSleep() {
  // CRITICAL: Set HOLD pin HIGH immediately to maintain power after wake-up
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  rtc_gpio_hold_en(GPIO_NUM_4);   // keep HOLD=1 while sleeping

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  
  // Debug: Show which button woke us up
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("WAKE-UP: Button A pressed (ext0)");
  } else if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
    Serial.println("WAKE-UP: Button C pressed (ext1)");
  } else {
    Serial.printf("WAKE-UP: Unknown source (%d)\n", wakeup_reason);
  }
  
  // Always turn on screen and show wake reason prominently
  M5.Display.wakeup();
  M5.Display.setBrightness(50); // Higher brightness for wake display
  M5.Display.clear();
  M5.Display.setRotation(0);
  //showSplashOverlay("Starting up", "", "", 1500);
  
  enterBLEWaitingMode();
  }
  
void updatePowerManagement(float gx, float gy, float gz, float ax, float ay, float az) {
  unsigned long currentTime = millis();
  
  // Check for motion
  bool motionDetected = detectMotion(gx, gy, gz, ax, ay, az);
  
  if (motionDetected) {
    lastMotionTime = currentTime;
    // Debug: Show when motion keeps screen on
    if (!screenOn && currentTime - lastMotionTime <= 30000) {
      Serial.println("Motion detected - keeping screen on");
    } else if (motionDetected) {
      Serial.println("Motion detected - resetting activity timer");
    }
  }
  
  // Update BLE activity time if connected
  if (deviceConnected) {
    lastBLEActivityTime = currentTime;
    // Switch to BLE active mode when connected
    if (currentPowerMode != POWER_BLE_ACTIVE) {
      enterBLEActiveMode();
    }
  }
  
  // Screen management - keep screen on when:
  // 1. Recently pressed button (60s)
  // 2. Connected to Zwift (active connection)
  // 3. Recent motion detected (30s)
  // 4. BLE connected and recent motion (<300s)
  bool shouldKeepScreenOn = false;
  
  if (currentTime - lastButtonTime <= SCREEN_ON_TIMEOUT) {
    shouldKeepScreenOn = true; // Button recently pressed
  } else if (zwiftConnected) {
    shouldKeepScreenOn = true; // Connected to Zwift
  } else if (deviceConnected && (currentTime - lastMotionTime <= NO_MOTION_TIMEOUT_CONNECTED)) {
    shouldKeepScreenOn = true; // BLE connected and recent motion
  }
  else if (!deviceConnected && (currentTime - lastMotionTime <= 30000)) { // 30s after motion when not connected
    shouldKeepScreenOn = true; // Recent motion detected
  }
  
  if (shouldKeepScreenOn) {
    if (!screenOn) {
      exitScreenOffMode();
    }
  } else {
    if (screenOn) {
      enterScreenOffMode();
    }
  }
  
  // Power mode transitions based on activity and connection state
  if (!isCalibrating) {
    if (deviceConnected) {
      // Stay in BLE active mode when connected
      if (currentPowerMode != POWER_BLE_ACTIVE) {
        enterBLEActiveMode();
      }
    } else {
      // Not connected - manage power modes based on time and activity
      unsigned long timeSinceActivity = min(currentTime - lastMotionTime, currentTime - lastButtonTime);
      unsigned long timeSinceBLEStart = currentTime - bleStartTime;
      
      switch (currentPowerMode) {
        case POWER_BLE_WAITING: {
          // After 5 minutes of no connection, enter low power mode
          if (timeSinceBLEStart > BLE_WAIT_TIMEOUT) {
            enterLowPowerMode();
          }
          break;
        }
          
        case POWER_BLE_ACTIVE: {
          // If disconnected, go back to waiting mode
          enterBLEWaitingMode();
          break;
        }
          
        case POWER_LOW_POWER: {
          // After 2 minutes in low power with no activity, enter ULP sleep
          if (timeSinceActivity > LOW_POWER_TIMEOUT) {
            enterULPSleep();
          }
          break;
        }
          
        case POWER_ULP_SLEEP: {
          // Already in ULP sleep - wake-up handling will manage transitions
          break;
        }
      }
    }
  }
}

// —————— BLE Callbacks ——————
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect (BLEServer* s) override {
    deviceConnected = true;
    lastBLEActivityTime = millis();
    Serial.println("BLE client connected");
    
    // Turn on screen for 1 minute to show connection success
    if (!screenOn) {
      exitScreenOffMode();
    }
    lastButtonTime = millis(); // Reset screen timer for 1 minute display
    
    // Show connection success message
    M5.Display.clear();
    M5.Display.setRotation(0);
    M5.Display.setTextColor(0x07E0); // GREEN
    M5.Display.setTextSize(2);
    M5.Display.setCursor(10, 20);
    M5.Display.print("BLE");
    M5.Display.setCursor(10, 45);
    M5.Display.print("CONNECTED");
    M5.Display.setTextColor(0xFFFF); // WHITE
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 80);
    M5.Display.print("Screen on for 1min");
    
    showOverlay("CONNECTED", 1500);
    
    // Success beep pattern
    M5.Speaker.tone(800, 100);
    delay(150);
    M5.Speaker.tone(1000, 100);
    delay(150);
    M5.Speaker.tone(1200, 150);
  }
  void onDisconnect (BLEServer* s) override {
    deviceConnected = false;
    zwiftConnected = false; // Reset Zwift connection status
    Serial.println("BLE client disconnected");
    
    // Show disconnection message if screen is on
    if (screenOn) {
      M5.Display.clear();
      M5.Display.setRotation(0);
      M5.Display.setTextColor(0xF800); // RED
      M5.Display.setTextSize(2);
      M5.Display.setCursor(10, 20);
      M5.Display.print("BLE");
      M5.Display.setCursor(10, 45);
      M5.Display.print("DISCONNECTED");
      M5.Display.setTextColor(0xFFFF); // WHITE
      M5.Display.setTextSize(1);
      M5.Display.setCursor(10, 80);
      M5.Display.print("Advertising...");
      
      showOverlay("DISCONNECTED", 1500);
      
      // Disconnection beep
      M5.Speaker.tone(600, 200);
      delay(300);
      M5.Speaker.tone(400, 200);
    }
  }
};

class char31Callbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    lastBLEActivityTime = millis(); // Track BLE activity for power management
    
    auto val = c->getValue();
    Serial.print("char31 onWrite: ");
    for (auto &b: val) { Serial.printf("%02X ", b); }
    Serial.println();

    // Zwift handshake
    if (val.size()>=2 && val[0]==0x03 && val[1]==0x10) {
      Serial.println("→ got 0x310, sending initial challenge");
      uint8_t chal[4] = {0x03,0x10,0x12,0x34};
      pChar32->setValue(chal,4);
      pChar32->indicate();
      zwiftConnected = false; // Reset on new handshake attempt
    }
    else if (val.size()>=2 && val[0]==0x03 && val[1]==0x11) {
      Serial.println("→ got 0x311, marking challengeOK");
      challengeOK = true;
      uint8_t resp[4] = {0x03,0x11,0xFF,0xFF};
      pChar32->setValue(resp,4);
      pChar32->indicate();
    }
    else if (val.size()>=6 && val[0]==0x03 && val[1]==0x12) {
      // client has sent seed bytes in val[2..5]
      uint32_t seed = ( (uint32_t)val[5]<<24 ) |
                      ( (uint32_t)val[4]<<16 ) |
                      ( (uint32_t)val[3]<<8  ) |
                      ( (uint32_t)val[2]     );
      uint32_t pwd  = hashed(seed);
      Serial.printf("→ got 0x312, seed=0x%08X, pwd=0x%08X\n", seed, pwd);
      uint8_t res[6];
      res[0]=0x03; res[1]=0x12;
      res[2]= pwd        & 0xFF;
      res[3]=(pwd >>  8) & 0xFF;
      res[4]=(pwd >> 16) & 0xFF;
      res[5]=(pwd >> 24) & 0xFF;
      pChar32->setValue(res,6);
      pChar32->indicate();
    }
    else if (val.size()>=2 && val[0]==0x03 && val[1]==0x13) {
      Serial.println("→ got 0x313, final ACK");
      uint8_t r[3] = {0x03,0x13,0xFF};
      pChar32->setValue(r,3);
      pChar32->indicate();
      challengeOK = true;
      zwiftConnected = true; // Mark as actively connected to Zwift
      Serial.println("Zwift connection established - keeping screen on indefinitely");
    }
  }
};

class char32Callbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override { Serial.println("char32 onWrite"); }
  void onRead (BLECharacteristic* c) override { Serial.println("char32 onRead");  }
};

class char32Desc2902Callbacks : public BLEDescriptorCallbacks {
  void onWrite(BLEDescriptor* d) override {
    ind32On = true;
    Serial.println("Client ENABLED indications (char32)");
  }
};

// —————— Helpers ——————
void sendSteeringBin(float bin) {
  pChar30->setValue((uint8_t*)&bin, sizeof(bin));
  pChar30->notify();
  Serial.printf("NOTIFY bin: %.0f°\n", bin);
}

// —————— Calibration Functions ——————

// Apply low-pass filter to gyro readings
void filterGyroReadings(float gx, float gy, float gz) {
  filteredGx = gyroFilterAlpha * gx + (1.0f - gyroFilterAlpha) * filteredGx;
  filteredGy = gyroFilterAlpha * gy + (1.0f - gyroFilterAlpha) * filteredGy;
  filteredGz = gyroFilterAlpha * gz + (1.0f - gyroFilterAlpha) * filteredGz;
}

// Initialize drift monitoring array
void initGyroDriftMonitoring() {
  for (int i = 0; i < DRIFT_SAMPLES_COUNT; i++) {
    gyroDriftSamples[i] = 0.0f;
  }
  gyroDriftIndex = 0;
  driftArrayFilled = false;
  lastDriftSampleTime = millis();
  Serial.println("Gyro drift monitoring initialized");
}

// Add a new gyro drift sample (call every second)
void addGyroDriftSample(float gx, float gy, float gz) {
  // Calculate gyro magnitude as drift indicator
  float gyroMagnitude = sqrt(gx*gx + gy*gy + gz*gz);
  
  // Add to circular buffer
  gyroDriftSamples[gyroDriftIndex] = gyroMagnitude;
  gyroDriftIndex = (gyroDriftIndex + 1) % DRIFT_SAMPLES_COUNT;
  
  // Mark array as filled once we've wrapped around
  if (gyroDriftIndex == 0 && !driftArrayFilled) {
    driftArrayFilled = true;
    Serial.println("Gyro drift buffer filled - starting 60s trailing average reports");
  }
}

// Calculate and report 60-second trailing average
void reportGyroDriftAverage() {
  if (!driftArrayFilled && gyroDriftIndex < 10) {
    return; // Need at least 10 samples before reporting anything meaningful
  }
  
  // Calculate average over available samples
  int sampleCount = driftArrayFilled ? DRIFT_SAMPLES_COUNT : gyroDriftIndex;
  float sum = 0.0f;
  float minDrift = 999.0f, maxDrift = -999.0f;
  
  for (int i = 0; i < sampleCount; i++) {
    sum += gyroDriftSamples[i];
    if (gyroDriftSamples[i] < minDrift) minDrift = gyroDriftSamples[i];
    if (gyroDriftSamples[i] > maxDrift) maxDrift = gyroDriftSamples[i];
  }
  
  float avgDrift = sum / sampleCount;
  
  // Calculate standard deviation for drift stability assessment
  float variance = 0.0f;
  for (int i = 0; i < sampleCount; i++) {
    float diff = gyroDriftSamples[i] - avgDrift;
    variance += diff * diff;
  }
  float stdDev = sqrt(variance / sampleCount);
  
  // Report comprehensive drift statistics
  Serial.println("=== 60-SECOND GYRO DRIFT REPORT ===");
  Serial.printf("Samples: %d | Avg: %.3f°/s | Min: %.3f°/s | Max: %.3f°/s | StdDev: %.3f°/s\n", 
                sampleCount, avgDrift, minDrift, maxDrift, stdDev);
  
   
  Serial.println("=====================================");
}

// Check if calibration data exists
bool calibrationExists() {
  // Try to load IMU calibration from NVS
  return loadIMUCalibration();
}

// Non-blocking centering state variables
static bool centeringActive = false;
static unsigned long centeringStartTime = 0;
static int centeringCountdown = 0;
static unsigned long lastCountdownTime = 0;

// Start the centering process (non-blocking)
void startCentering() {
  Serial.println("Starting centering process...");
  
  // Distinctive beep pattern
  M5.Speaker.tone(800, 50);
  
  centeringActive = true;
  centeringStartTime = millis();
  centeringCountdown = 3;
  lastCountdownTime = millis();
  
  // Show initial countdown
  showOverlay("CENTER IN 3", 1100);
  M5.Speaker.tone(600, 100);
}

// Update centering process (called from main loop)
void updateCentering() {
  if (!centeringActive) return;
  
  unsigned long currentTime = millis();
  
  // Handle countdown phase
  if (centeringCountdown > 0) {
    if (currentTime - lastCountdownTime >= 1000) {
      centeringCountdown--;
      lastCountdownTime = currentTime;
      
      if (centeringCountdown > 0) {
        showOverlay("CENTER IN " + String(centeringCountdown), 1100);
        M5.Speaker.tone(600, 100);
      } else {
        // Countdown finished, perform centering
        showOverlay("CENTERING...", 1000);
        executeCentering();
      }
    }
  }
}

// Execute the actual centering (simple, single reading)
void executeCentering() {
  // Get current raw yaw (already being updated in main loop)
  float currentRawYaw = getYaw();
  
  if (!isnan(currentRawYaw) && !isinf(currentRawYaw)) {
    // Set offset to make current position = 0
    yawOffset = -currentRawYaw;
    
    // Normalize offset to [-180, 180] range
    while (yawOffset > 180) yawOffset -= 360;
    while (yawOffset < -180) yawOffset += 360;
    
    prefs.putFloat("yoff", yawOffset);
    Serial.printf("Centering complete - Current raw yaw: %.1f°, New offset: %.1f°\n", currentRawYaw, yawOffset);
    
    // Store the current raw yaw for drift compensation
    lastMeasuredRawYaw = currentRawYaw;
    lastYawMeasurementTime = millis();
    hasValidYawMeasurement = true;
    
    
    //showOverlay("CENTERED", 1500);
    
  } else {
    Serial.println("WARNING: Invalid yaw value during centering");
    showOverlay("CENTER ERROR", 1500);
  }
  
  centeringActive = false;
}

// Quick recenter - now just triggers the non-blocking process
void quickRecenterYaw() {
  startCentering();
}

// Recenter yaw to current position
void recenterYaw() {
  startCentering(); // Use the same simple, non-blocking process
}

// Auto-calibration on first boot
void performAutoCalibration() {
  Serial.println("=== AUTO CALIBRATION ===");
  
  if (screenOn) {
    M5.Display.clear();
    M5.Display.setRotation(0);
    M5.Display.setCursor(0, 10);
    M5.Display.print("AUTO CALIBRATION");
    M5.Display.setCursor(0, 25);
    M5.Display.print("Keep device still");
    M5.Display.setCursor(0, 40);
    M5.Display.print("10 seconds...");
  }
  
  // Use the new IMU calibration system
  startIMUCalibration();
}

// Auto-recenter on boot
void performAutoRecenter() {
  recenterYaw();
}

// Initialize Mahony filter with current device orientation
void initializeMahonyFilter() {
  Serial.println("Initializing Mahony filter with current orientation...");
  
  // Reset quaternion to identity
  initMahonyData();
  
  // Collect IMU data for a few seconds to stabilize the filter
  const int initSamples = 100; // 10 seconds at 10Hz
  
  for (int i = 0; i < initSamples; i++) {
    float gx, gy, gz, ax, ay, az;
    M5.Imu.getGyro(&gx, &gy, &gz);
    M5.Imu.getAccel(&ax, &ay, &az);
    
    // Apply filtering
    filterGyroReadings(gx, gy, gz);
    
    // Update Mahony filter with calibrated data
    MahonyAHRSupdateIMU(
      filteredGx * DEG_TO_RAD,
      filteredGy * DEG_TO_RAD,
      filteredGz * DEG_TO_RAD,
      ax,
      ay,
      az,
      currentIMUFrequency
    );
    
    delay(100); // 100ms = 10Hz
  }
  
  // Get initial yaw reading
  float initialYaw = getYaw();
  Serial.printf("Mahony filter initialized - Initial yaw: %.1f°\n", initialYaw);
}

// Gyro-only calibration (legacy function - now uses new IMU system)
void performGyroCalibration() {
  Serial.println("Gyro calibration - using new IMU calibration system");
  startIMUCalibration();
}

// Full calibration (legacy function - now uses new IMU system)
void performFullCalibration() {
  Serial.println("Full calibration - using new IMU calibration system");
  startIMUCalibration();
}

// —————— NEW IMU CALIBRATION SYSTEM ——————

void startIMUCalibration() {
  Serial.println("=== STARTING IMU CALIBRATION ===");
  
  // Start calibration for accelerometer and gyro
  M5.Imu.setCalibration(IMU_CALIB_STRENGTH, IMU_CALIB_STRENGTH, 0);
  
  imuCalibCountdown = 10;
  isCalibrating = true;
  
  // Show initial calibration message
  showOverlay("CALIBRATING...", 1000);
  
  Serial.println("IMU calibration started - keep device still for 10 seconds");
}

void updateIMUCalibration(uint32_t countdown, bool clear) {
  imuCalibCountdown = countdown;
  
  // Show countdown as overlay
  if (countdown > 0) {
    showOverlay("CALIB " + String(countdown), 1000);
  }

  if (countdown == 0) {
    clear = true;
  }
  
  if (clear) {
    if (countdown > 0) {
      // Start calibration
      M5.Imu.setCalibration(IMU_CALIB_STRENGTH, IMU_CALIB_STRENGTH, 0);
    } else {
      // Stop calibration and save
      M5.Imu.setCalibration(0, 0, 0);
      saveIMUCalibration();
    }
  }
}

void stopIMUCalibration() {
  Serial.println("=== STOPPING IMU CALIBRATION ===");
  
  // Stop all calibration
  M5.Imu.setCalibration(0, 0, 0);
  
  // Save calibration values to NVS
  saveIMUCalibration();
  
  // Recenter yaw after calibration
  recenterYaw();
  
  imuCalibrationActive = false;
  isCalibrating = false;
  
  showOverlay("CALIBRATION OK", 1500);
  
  Serial.println("=== IMU CALIBRATION COMPLETE ===");
}

bool loadIMUCalibration() {
  Serial.println("=== LOADING IMU CALIBRATION FROM NVS ===");
  bool success = M5.Imu.loadOffsetFromNVS();
  
  if (success) {
    Serial.println("✓ IMU calibration loaded successfully");
    
    // Get the loaded offset data for debugging with proper labels
    Serial.println("Loaded bias values:");
    float gyroXBias = 0, gyroYBias = 0, gyroZBias = 0;
    float accelXBias = 0, accelYBias = 0, accelZBias = 0;
    
    for (int i = 0; i < 9; i++) {
      float offset = M5.Imu.getOffsetData(i) * (1.0f / (1 << 19));
      const char* sensorName = (i < 3) ? "Gyro" : (i < 6) ? "Accel" : "Mag";
      const char* axisName = (i % 3 == 0) ? "X" : (i % 3 == 1) ? "Y" : "Z";
      const char* units = (i < 3) ? "°/s" : (i < 6) ? "g" : "µT";
      
      // Store gyro bias values for summary
      if (i == 0) gyroXBias = offset;
      else if (i == 1) gyroYBias = offset;
      else if (i == 2) gyroZBias = offset;
      else if (i == 3) accelXBias = offset;
      else if (i == 4) accelYBias = offset;
      else if (i == 5) accelZBias = offset;
      
      Serial.printf("  %s %s [%d]: %+.6f %s\n", sensorName, axisName, i, offset, units);
    }
    
    // Summary of critical gyro bias values
    float gyroBiasMagnitude = sqrt(gyroXBias*gyroXBias + gyroYBias*gyroYBias + gyroZBias*gyroZBias);
    Serial.printf(">>> GYRO BIAS SUMMARY: X=%+.3f Y=%+.3f Z=%+.3f (Magnitude: %.3f °/s)\n", 
                  gyroXBias, gyroYBias, gyroZBias, gyroBiasMagnitude);
    
    if (gyroBiasMagnitude > 1.0f) {
      Serial.println("⚠️  WARNING: Gyro bias magnitude >1°/s");
    } else if (gyroBiasMagnitude < 0.01f) {
      Serial.println("⚠️  WARNING: Gyro bias very small - calibration may not have worked");
    } else {
      Serial.println("✓ Gyro bias values look reasonable");
    }
    
  } else {
    Serial.println("✗ No IMU calibration found in NVS - will perform auto-calibration");
  }
  
  return success;
}

void saveIMUCalibration() {
  Serial.println("=== SAVING IMU CALIBRATION TO NVS ===");
  
  // Show what we're about to save
  Serial.println("Bias values being saved:");
  for (int i = 0; i < 9; i++) {
    float offset = M5.Imu.getOffsetData(i) * (1.0f / (1 << 19));
    const char* sensorName = (i < 3) ? "Gyro" : (i < 6) ? "Accel" : "Mag";
    const char* axisName = (i % 3 == 0) ? "X" : (i % 3 == 1) ? "Y" : "Z";
    Serial.printf("  %s %s [%d]: %.6f\n", sensorName, axisName, i, offset);
  }
  
  M5.Imu.saveOffsetToNVS();
  Serial.println("=== IMU CALIBRATION SAVED SUCCESSFULLY ===");
}



void setup() {
  Serial.begin(115200);
  Serial.println("=== ZTEERStick STARTUP ===");
  
  // Power management: Set HOLD pin high to maintain power
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  rtc_gpio_hold_en(GPIO_NUM_4);   // keep HOLD=1 while sleeping

  Serial.println("HOLD pin set HIGH to maintain power");
  
  auto cfg = M5.config();
  Serial.println("M5 config created");
  
  M5.begin(cfg);
  Serial.println("M5.begin() completed");
  
  // Initialize LittleFS for image storage
  if (!LittleFS.begin()) {
    Serial.println("LittleFS initialization failed!");
  } else {
    Serial.println("LittleFS initialized successfully");
  }
  
  // Configure power management early
  configurePowerManagement();
  
  // Set initial CPU frequency for BLE
  setCpuFrequencyMhz(BLE_CPU_FREQ);
  Serial.printf("CPU frequency set to %d MHz for BLE\n", BLE_CPU_FREQ);
  
  // Initialize display with 5% brightness
  M5.Display.setRotation(0);
  M5.Display.setTextColor(ORANGE);
  M5.Display.setTextDatum(0);
  M5.Display.setFont(&fonts::Font0);
  M5.Display.setTextSize(2);
  M5.Display.setBrightness(13); // 5% brightness (13/255 ≈ 5%)
  screenOn = true;
  lastButtonTime = millis(); // Screen starts on
  
  // Initialize modern display system
  initializeDisplayBuffer();
  

  showSplashOverlay("ZTEERStick", "logo.jpg", "Let's ride!", 2000);
  
  Serial.println("Display configured at 5% brightness with modern UI");
  
  // Setup LED with PWM for breathing pattern
  pinMode(LED_PIN, OUTPUT);
  ledcSetup(0, 5000, 8);           // PWM channel 0, 5kHz frequency, 8-bit resolution
  ledcAttachPin(LED_PIN, 0);       // Attach LED pin to PWM channel 0
  ledBreathingStartTime = millis();
  
  Serial.println("LED configured for breathing pattern");

  // Initialize button GPIO pins
  pinMode(37, INPUT_PULLUP); // Button A
  pinMode(39, INPUT_PULLUP); // Button B  
  pinMode(35, INPUT_PULLUP); // Button C (Power)
  Serial.println("Button GPIO pins configured (37, 39, 35)");
  
  // Check for factory reset request (hold Button B during startup)
  bool factoryResetRequested = !digitalRead(39);
  if (factoryResetRequested) {
    Serial.println("=== FACTORY RESET REQUESTED ===");
    M5.Display.clear();
    M5.Display.setRotation(0);
    M5.Display.setCursor(10, 20);
    M5.Display.print("FACTORY RESET");
    M5.Display.setCursor(10, 40);
    M5.Display.print("Hold Button B");
    M5.Display.setCursor(10, 60);
    M5.Display.print("for 3 seconds");
    
    // Wait 3 seconds while Button B is held
    bool resetConfirmed = true;
    for (int i = 0; i < 30; i++) {
      if (digitalRead(39)) { // Button B released
        resetConfirmed = false;
        break;
      }
      delay(100);
      
      // Update countdown display
      M5.Display.setCursor(10, 80);
      M5.Display.printf("Countdown: %d", 3 - (i/10));
    }
    
    if (resetConfirmed) {
      Serial.println("Factory reset confirmed - clearing all preferences");
      M5.Display.clear();
      M5.Display.setCursor(10, 20);
      M5.Display.print("CLEARING");
      M5.Display.setCursor(10, 40);
      M5.Display.print("PREFERENCES");
      
      // Clear all stored preferences
      prefs.begin("sterzo", false);
      prefs.clear();
      prefs.end();
      
      M5.Display.setCursor(10, 60);
      M5.Display.print("DONE!");
      M5.Display.setCursor(10, 80);
      M5.Display.print("Restarting...");
      
      delay(2000);
      ESP.restart();
    } else {
      Serial.println("Factory reset cancelled");
      M5.Display.clear();
      M5.Display.setCursor(10, 40);
      M5.Display.print("Reset cancelled");
      delay(1000);
    }
  }

  // Initialize Mahony AHRS
  initMahonyData();
  Serial.println("Mahony AHRS initialized");

  // Initialize gyro drift monitoring
  initGyroDriftMonitoring();

  // Initialize power management variables
  lastMotionTime = millis();
  lastBLEActivityTime = millis();
  lastMotionAccumulatorReset = millis();
  motionAccumulator = 0.0f;
  currentPowerMode = POWER_BLE_WAITING;
  bleStartTime = millis();
  
  // Check if we woke up from deep sleep
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED) {
    Serial.println("=== FRESH BOOT (not from deep sleep) ===");
  } else {
    Serial.printf("=== WOKE FROM DEEP SLEEP (reason: %d) ===\n", wakeup_reason);
    handleWakeupFromSleep();
  }
  
  // Always zero yaw heading on startup/wake-up
  Serial.println("Zeroing yaw heading for fresh start...");
  yawOffset = 0.0f;
  prefs.putFloat("yoff", yawOffset);
  Serial.println("Yaw heading reset to 0°");

  // Initialize preferences
  Serial.println("Loading calibration data...");
  prefs.begin("sterzo", false);
  
  // Check if IMU calibration exists
  if (!calibrationExists()) {
    // First boot - no calibration exists
    Serial.println("No IMU calibration found - performing auto-calibration");
    performAutoCalibration();
  } else {
    // IMU calibration exists - it was loaded by calibrationExists()
    Serial.println("IMU calibration loaded successfully");
    
    // Note: yawOffset already set to 0.0f above - starting fresh
    Serial.printf("Calibration loaded - YawOffset: %.3f° (fresh start) DriftRate: %.2f°/s\n", 
                  yawOffset, YAW_DRIFT_RATE);
    
    // Initialize Mahony filter with current orientation
    initializeMahonyFilter();
  }

  // Initialize BLE system
  Serial.println("Initializing BLE...");
  
  // Start BLE for initial connection attempts
  startBLE();
  
  BLEDevice::init("STERZO");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  pSvc = pServer->createService(STERZO_SERVICE_UUID);
  
  pChar14 = pSvc->createCharacteristic(CHAR14_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pChar30 = pSvc->createCharacteristic(CHAR30_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pChar31 = pSvc->createCharacteristic(CHAR31_UUID, BLECharacteristic::PROPERTY_WRITE);
  pChar32 = pSvc->createCharacteristic(CHAR32_UUID, BLECharacteristic::PROPERTY_INDICATE);
  
  p2902_14 = new BLE2902();          pChar14->addDescriptor(p2902_14);
  p2902_30 = new BLE2902(); p2902_30->setNotifications(true); pChar30->addDescriptor(p2902_30);
  p2902_32 = new BLE2902(); p2902_32->setCallbacks(new char32Desc2902Callbacks()); pChar32->addDescriptor(p2902_32);

  pChar31->setCallbacks(new char31Callbacks());
  pChar32->setCallbacks(new char32Callbacks());

  pSvc->start();
  auto adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(STERZO_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  
  // Configure BLE advertising for low power
  adv->setMinInterval(800);  // Longer intervals for power saving
  adv->setMaxInterval(1600);
  
  BLEDevice::startAdvertising();
  
  Serial.println("BLE advertising started - waiting for connection");

  // IMU wake-on-motion removed - using button-only wake

  // Display startup message
  M5.Display.clear();
  M5.Display.setRotation(0);  
  M5.Display.setCursor(10, 20);
  M5.Display.print("ZTEERStick");
  M5.Display.setCursor(10, 50);
  M5.Display.print("Ready!");
  M5.Display.setCursor(10, 80);
  M5.Display.printf("10Hz IMU");
  M5.Display.setCursor(10, 110);
  M5.Display.printf("5%% Screen");
  
  Serial.println("=== ZTEERStick READY ===");
  
  // Clear the startup message and prepare for the main display
  displaySprite.fillSprite(TFT_BLACK);
  displaySprite.pushSprite(0, 0);

  // Note: Wake-up handling now manages power modes directly
  // No immediate return to sleep needed as wake handler sets appropriate mode
}

void loop() {
  M5.update(); // Update button states

  // Update LED breathing pattern
  updateLEDBreathing();

  // Handle non-blocking centering process
  updateCentering();

  // Handle IMU calibration countdown
  static unsigned long lastCalibUpdate = 0;
  if (isCalibrating) {
    if (millis() - lastCalibUpdate >= 1000) {
      lastCalibUpdate = millis();
      imuCalibCountdown--;
      
      if (imuCalibCountdown > 0) {
        updateIMUCalibration(imuCalibCountdown, false);
      } else {
        stopIMUCalibration();
      }
    }
    // Continue with main loop to ensure display updates during calibration
  }

  // Calculate dynamic sample frequency with proper limiting for 10Hz
  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = micros();
  dt = (currentTime - lastUpdateTime) / 1000000.0f; // Calculate time delta in seconds
  
  // Adaptive frequency limiting based on current IMU frequency
  float minInterval = 1.0f / currentIMUFrequency; // Calculate minimum interval for current frequency
  if (dt < minInterval) {
    // Calculate delay needed to maintain target frequency
    int delayMicros = (int)((minInterval - dt) * 1000000.0f);
    if (delayMicros > 0 && delayMicros < 200000) { // Cap delay at 200ms
      delayMicroseconds(delayMicros);
    }
    M5.update(); // Update buttons even during delay
    return;
  }
  
  lastUpdateTime = currentTime;
  loopfreq = 1.0f / dt; // Calculate loop frequency
  
  // Cap frequency at reasonable limits
  if (loopfreq > currentIMUFrequency * 1.5f) loopfreq = currentIMUFrequency * 1.5f;

  // 1) Read raw IMU, apply bias, fuse
  float gx, gy, gz, ax, ay, az;
  
  // Update IMU data - this also performs calibration if active
  auto imu_update = M5.Imu.update();
  if (imu_update) {
    // Get calibrated IMU data
    auto imu_data = M5.Imu.getImuData();
    gx = imu_data.gyro.x;
    gy = imu_data.gyro.y;
    gz = imu_data.gyro.z;
    ax = imu_data.accel.x;
    ay = imu_data.accel.y;
    az = imu_data.accel.z;
  } else {
    // Fallback to direct reading if update failed
    M5.Imu.getGyro(&gx,&gy,&gz);
    M5.Imu.getAccel(&ax,&ay,&az);
  }

  // Apply low-pass filter to gyro readings
  filterGyroReadings(gx, gy, gz);

  // Update power management with current IMU readings
  updatePowerManagement(gx, gy, gz, ax, ay, az);

  // Debug IMU readings every 5 seconds
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 5000) {
    const char* powerModeStr;
    switch (currentPowerMode) {
      case POWER_BLE_ACTIVE: powerModeStr = "BLE_ACTIVE"; break;
      case POWER_BLE_WAITING: powerModeStr = "BLE_WAITING"; break;
      case POWER_LOW_POWER: powerModeStr = "LOW_POWER"; break;
      case POWER_ULP_SLEEP: powerModeStr = "ULP_SLESLEEP"; break;
      default: powerModeStr = "UNKNOWN"; break;
    }
    
    // Calculate gyro magnitude for drift monitoring
    float gyroMagnitude = sqrt(gx*gx + gy*gy + gz*gz);
    
    // Consolidated status line with all information
    unsigned long currentTime = millis();
    
    // Calculate inactivity timers (same logic as logInactivityStatus)
    int screenOffCountdown = 0;
    int sleepCountdown = 0;
    
    // Screen countdown logic
    if (screenOn) {
      unsigned long timeSinceButton = currentTime - lastButtonTime;
      if (timeSinceButton <= SCREEN_ON_TIMEOUT) {
        screenOffCountdown = (SCREEN_ON_TIMEOUT - timeSinceButton) / 1000;
      } else if (zwiftConnected) {
        screenOffCountdown = 999; // Indefinite when Zwift connected
      }
    }
    
    // Sleep countdown logic based on power mode
    switch (currentPowerMode) {
      case POWER_BLE_WAITING: {
        unsigned long timeSinceBLEStart = currentTime - bleStartTime;
        if (timeSinceBLEStart > BLE_WAIT_TIMEOUT) {
          sleepCountdown = 0; // Should be in low power
        } else {
          sleepCountdown = (BLE_WAIT_TIMEOUT - timeSinceBLEStart) / 1000;
        }
        break;
      }
      case POWER_LOW_POWER: {
        unsigned long timeSinceMotion = currentTime - lastMotionTime;
        unsigned long timeSinceButton = currentTime - lastButtonTime;
        unsigned long timeSinceActivity = min(timeSinceMotion, timeSinceButton);
        if (timeSinceActivity > LOW_POWER_TIMEOUT) {
          sleepCountdown = 0; // Should be sleeping
        } else {
          sleepCountdown = (LOW_POWER_TIMEOUT - timeSinceActivity) / 1000;
        }
        break;
      }
      case POWER_BLE_ACTIVE:
        sleepCountdown = 999; // No sleep when connected
        break;
      default:
        sleepCountdown = 0;
        break;
    }
    
    // Get current yaw for status
    float rawYaw = getYaw();
    float currentRel = rawYaw + yawOffset;
    while (currentRel > 180) currentRel -= 360;
    while (currentRel < -180) currentRel += 360;
    currentRel = constrain(currentRel, -40, 40);
    float currentBin = round(currentRel/1.0f)*1.0f;
    if (abs(currentBin) < 0.1f) currentBin = 0.0f;
    
    Serial.printf("STATUS: Yaw=%.1f° Bin=%.0f° BLE=%s | Screen=%ds Sleep=%ds Motion=%s Zwift=%s | Gyro=%.3f,%.3f,%.3f(%.3f°/s) Accel=%.2f,%.2f,%.2f | Freq=%.1f/%.1fHz %s Screen=%s CPU=%dMHz\n", 
                  currentRel, currentBin, deviceConnected ? "Connected" : "Advertising",
                  screenOffCountdown, sleepCountdown,
                  (currentTime - lastMotionTime < 5000) ? "active" : "idle",
                  zwiftConnected ? "active" : "idle",
                  gx, gy, gz, gyroMagnitude, ax, ay, az, 
                  loopfreq, currentIMUFrequency, powerModeStr,
                  screenOn ? "ON" : "OFF", getCpuFrequencyMhz());
    
    
    lastDebugTime = millis();
  }

  // Gyro drift monitoring - collect samples every second
  if (millis() - lastDriftSampleTime >= 1000) {
    addGyroDriftSample(gx, gy, gz);
    lastDriftSampleTime = millis();
  }

  // Report drift average every 60 seconds
  static unsigned long lastDriftReportTime = 0;
  if (millis() - lastDriftReportTime >= 60000) {
    reportGyroDriftAverage();
    lastDriftReportTime = millis();
  }

  // Update Mahony AHRS with calibrated data (convert gyro to radians)
  // The IMU data is already calibrated by the M5.Imu system
  MahonyAHRSupdateIMU(
    filteredGx * DEG_TO_RAD,
    filteredGy * DEG_TO_RAD,
    filteredGz * DEG_TO_RAD,
    ax,
    ay,
    az,
    currentIMUFrequency  // Use current frequency for consistent filter behavior
  );

  // Get yaw in degrees from Mahony AHRS
  float rawYaw = getYaw();  // Already returns degrees in [-180, 180] range
  
  // Check for valid yaw value
  if (isnan(rawYaw) || isinf(rawYaw)) {
    Serial.println("WARNING: Invalid yaw value detected!");
    rawYaw = 0.0f; // Use 0 as fallback
  }
  
  // Calculate relative yaw: raw + offset
  float rel = rawYaw + yawOffset;
  
  // Normalize relative yaw to [-180, 180] range
  while (rel > 180) rel -= 360;
  while (rel < -180) rel += 360;
  
  // Simple centering force - gradually adjust offset to bring rel back to 0
  static unsigned long lastCenteringTime = millis();
  unsigned long centeringTime = millis();
  float deltaTime = (centeringTime - lastCenteringTime) / 1000.0f;
  
      if (deltaTime >= 1.0f) { // Update every 1 second for simplicity
      // Apply centering force: if rel is positive, decrease offset; if negative, increase offset
      // Use the configured YAW_DRIFT_RATE - now it's the actual rate per second
      float centeringAdjustment = -rel * YAW_DRIFT_RATE * deltaTime;
      
      // Since we're updating every 1 second, the YAW_DRIFT_RATE is already the rate per second
      // No need to scale - just constrain to prevent overcorrection
      float maxAdjustmentPerUpdate = YAW_DRIFT_RATE; // Rate per second = rate per update when deltaTime = 1s
      centeringAdjustment = constrain(centeringAdjustment, -maxAdjustmentPerUpdate, maxAdjustmentPerUpdate);
    
    yawOffset += centeringAdjustment;
    
    // Normalize offset to [-180, 180] range
    while (yawOffset > 180) yawOffset -= 360;
    while (yawOffset < -180) yawOffset += 360;
    
    // Debug centering compensation
    if (DEBUG_MODE && abs(centeringAdjustment) > 0.01f) {
      float actualRatePerSecond = centeringAdjustment / deltaTime;
      Serial.printf("Centering: rel=%.2f° target=%.1f°/s actual=%.3f°/s adj=%.3f° dt=%.1fs\n", 
                    rel, YAW_DRIFT_RATE, actualRatePerSecond, centeringAdjustment, deltaTime);
    }
    
    lastCenteringTime = centeringTime;
  }
  
  // Recalculate relative yaw with updated offset
  rel = rawYaw + yawOffset;
  
  // Normalize relative yaw to [-180, 180] range again
  while (rel > 180) rel -= 360;
  while (rel < -180) rel += 360;
  
  // Clamp to steering range and bin
  rel = constrain(rel, -40, 40);
  float bin = round(rel/1.0f)*1.0f;
  
  // Fix negative zero issue
  if (abs(bin) < 0.1f) {
    bin = 0.0f;
  }

  // Yaw status now included in consolidated status line above

  // notify on true bin‐change
  if (deviceConnected && ind32On && !isnan(lastSentBin) && bin!=lastSentBin) {
    sendSteeringBin(bin);
  }
  lastSentBin = bin;

  // Update modern display with current status
  float batteryVoltage = M5.Power.getBatteryVoltage() / 1000.0f;
  drawModernDisplay(rel, deviceConnected, currentPowerMode, batteryVoltage, (int)loopfreq);
  
  // Log inactivity status periodically
  logInactivityStatus();

  // —————— Button Handling ——————
  static unsigned long buttonAStartTime = 0;
  static unsigned long buttonBStartTime = 0;
  static unsigned long buttonCStartTime = 0;
  static bool buttonAWasPressed = false;
  static bool buttonBWasPressed = false;
  static bool buttonCWasPressed = false;
  
  // Button GPIO pins
  const int BUTTON_A_PIN = 37;
  const int BUTTON_B_PIN = 39;
  const int BUTTON_C_PIN = 35;
  
  // Read button states directly from GPIO
  bool buttonAState = !digitalRead(BUTTON_A_PIN); // Inverted logic
  bool buttonBState = !digitalRead(BUTTON_B_PIN); // Inverted logic  
  bool buttonCState = !digitalRead(BUTTON_C_PIN); // Inverted logic
  
  // Button A (front face) - Short press (<2s): wake screen, Long press (>=2s): recenter
  if (buttonAState && !buttonAWasPressed) {
    buttonAStartTime = millis();
    buttonAWasPressed = true;
    lastButtonTime = millis(); // Update button activity
    Serial.println("Button A pressed - resetting activity timer");
  } else if (!buttonAState && buttonAWasPressed) {
    unsigned long pressDuration = millis() - buttonAStartTime;
    buttonAWasPressed = false;
    
    if (pressDuration < 1000) {
      Serial.println("Button A: Short press - Waking screen");
      if (!screenOn) {
        exitScreenOffMode();
      }
      lastButtonTime = millis(); // Reset screen timer
    } else {
      Serial.println("Button A: Long press - Recenter");
      M5.Speaker.tone(800, 100);
      quickRecenterYaw();
    }
  }
  
  // Button B (top/north face) - Short press (<2s): recenter, Long press (>=2s): full calibration
  if (buttonBState && !buttonBWasPressed) {
    buttonBStartTime = millis();
    buttonBWasPressed = true;
    lastButtonTime = millis(); // Update button activity
    Serial.println("Button B pressed - resetting activity timer");
  } else if (!buttonBState && buttonBWasPressed) {
    unsigned long pressDuration = millis() - buttonBStartTime;
    buttonBWasPressed = false;
    
    if (pressDuration < 2000) {
      Serial.println("Button B: Short press - Recenter");
      M5.Speaker.tone(1000, 100);
      quickRecenterYaw();
    } else {
      Serial.println("Button B: Long press - Full calibration");
      performFullCalibration();
    }
  }
  
  // Button C (Power button) - Power management
  if (buttonCState && !buttonCWasPressed) {
    buttonCStartTime = millis();
    buttonCWasPressed = true;
    lastButtonTime = millis(); // Update button activity
    Serial.println("Button C pressed - resetting activity timer");
  } else if (!buttonCState && buttonCWasPressed) {
    unsigned long pressDuration = millis() - buttonCStartTime;
    buttonCWasPressed = false;
    
    if (pressDuration < 1000) {
      Serial.println("Button C: Short press - Wake screen");
      M5.Speaker.tone(1200, 100);
      
      // Wake screen and reset timer
      if (!screenOn) {
        exitScreenOffMode();
      }
      lastButtonTime = millis(); // Reset screen timer
    } else if (pressDuration >= 1000 && pressDuration < 2000) {
      Serial.println("Button C: Medium press - Screen wake");
      M5.Speaker.tone(1200, 100);
      
      // Wake screen and reset timer
      if (!screenOn) {
        exitScreenOffMode();
      }
      lastButtonTime = millis(); // Reset screen timer
    } else if (pressDuration >= 2000) {
      Serial.println("Button C: Long press - Power off");
      
      if (screenOn) {
        M5.Display.clear();
        M5.Display.setRotation(0);
        M5.Display.setCursor(0, 25);
        M5.Display.print("Powering");
        M5.Display.setCursor(0, 40);
        M5.Display.print("Off...");
        delay(1000);
      }
      
      // Try to power off
      pinMode(4, OUTPUT);
      digitalWrite(4, LOW);
      Serial.println("HOLD pin set LOW for power off");
      delay(2000);
      
      // If we get here, power off failed, so enter deep sleep
      Serial.println("Power off failed, entering deep sleep");
      enterULPSleep();
    }
  }
}



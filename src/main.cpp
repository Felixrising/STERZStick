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
#include "app/types.h"
#include "app/App.h"
#include "app/AppState.h"
#include "boards/BoardConfig.h"
#include "modules/ble/BleService.h"
#include "modules/display/DisplayController.h"
#include "modules/input/ButtonController.h"
#include "modules/power/PowerManager.h"
#include "modules/sensor/ImuService.h"

// —————— VERSION ——————
#define STERZSTICK_VERSION "v1.0-RC3"

#ifdef ESP32
extern HardwareSerial Serial;
#endif

// —————— Function Declarations ——————
void handleButtons();
void powerOffSequence();

// —————— Pin Setup ——————
static const board::BoardConfig& BOARD = board::current();
static ButtonController* gButtonController = nullptr;
static int LED_PIN = BOARD.ledPin;

// —————— MODERN DISPLAY SYSTEM ——————

// —————— MODERN DISPLAY FUNCTIONS ——————

void initializeDisplayBuffer() {
  display::initializeDisplayBuffer();
}

void showOverlay(String text, int duration_ms) {
  display::showOverlay(text, duration_ms);
}

// Start a non-blocking splash overlay
void startSplashOverlay(const char* topText = "", const char* imagePath = "", const char* bottomText = "", int duration_ms = 2000) {
  display::startSplashOverlay(topText, imagePath, bottomText, duration_ms, screenOn);
}

// Draw the splash overlay content (helper function)
void drawSplashContent() {
  display::drawSplashContent();
}

// Update splash overlay state (call from main loop)
void updateSplashOverlay() {
  display::updateSplashOverlay();
}

// Check if splash overlay is currently active
bool isSplashActive() {
  return display::isSplashActive();
}

// Safe display clear - respects active splash
void safeDisplayClear() {
  display::safeDisplayClear();
}

// Safe display operations - check for active splash first
bool canDrawToDisplay() {
  return display::canDrawToDisplay();
}

// Legacy blocking function for compatibility (calls non-blocking version)
void showSplashOverlay(const char* topText = "", const char* imagePath = "", const char* bottomText = "", int duration_ms = 2000) {
  display::showSplashOverlay(topText, imagePath, bottomText, duration_ms, screenOn);
  // Note: No delay() - now non-blocking!
}

void clearOverlay() {
  display::clearOverlay();
}

void drawModernDisplay(float yaw, bool connected, PowerMode powerMode, float battery, int freq) {
  display::drawModernDisplay(yaw, connected, powerMode, battery, freq, screenOn, deviceConnected, zwiftConnected, lastMotionTime);
}

// —————— Calibration Functions ——————

// —————— NEW IMU CALIBRATION SYSTEM ——————

void handleButtons() {
  if (gButtonController) {
    gButtonController->update(screenOn, lastButtonTime);
  }
}

void powerOffSequence() {
  if (screenOn) {
    display::handleEvent(display::DisplayEvent::PowerOff);
    delay(800);
  }
  // Delegate to board-aware power path (PMIC on S3, HOLD on Plus2).
  enterULPSleep();
}

void firmwareSetup() {
  Serial.begin(115200);
  Serial.println("=== STERZStick STARTUP ===");
  Serial.print("Version: ");
  Serial.println(STERZSTICK_VERSION);
  
  // Power management: Set HOLD pin high to maintain power
  pinMode(BOARD.holdPin, OUTPUT);
  digitalWrite(BOARD.holdPin, HIGH);
  rtc_gpio_hold_en((gpio_num_t)BOARD.holdPin);   // keep HOLD=1 while sleeping

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
  
  // Initialize display with wake brightness initially 
  M5.Display.setRotation(0);
  M5.Display.setTextColor(ORANGE);
  M5.Display.setTextDatum(0);
  M5.Display.setFont(&fonts::Font0);
  M5.Display.setTextSize(2);
  M5.Display.setBrightness(display::brightnessWake()); // Higher brightness for startup
  screenOn = true;
  lastButtonTime = millis(); // Screen starts on
  
  // Initialize modern display system
  initializeDisplayBuffer();
  

  startSplashOverlay("STERZStick", "logo.jpg", STERZSTICK_VERSION, 2000);
  
  Serial.println("Display configured at 5% brightness with modern UI");
  
  // Setup LED with PWM for breathing pattern
  pinMode(LED_PIN, OUTPUT);
  ledcSetup(0, 5000, 8);           // PWM channel 0, 5kHz frequency, 8-bit resolution
  ledcAttachPin(LED_PIN, 0);       // Attach LED pin to PWM channel 0
  ledBreathingStartTime = millis();
  
  Serial.println("LED configured for breathing pattern");

  // Initialize button GPIO pins
  pinMode(BOARD.buttonAPin, INPUT_PULLUP); // Button A
  pinMode(BOARD.buttonBPin, INPUT_PULLUP); // Button B
  pinMode(BOARD.buttonCPin, INPUT_PULLUP); // Button C (Power)
  Serial.printf("Button GPIO pins configured (%d, %d, %d)\n", BOARD.buttonAPin, BOARD.buttonBPin, BOARD.buttonCPin);

  gButtonController = new ButtonController(BOARD.buttonAPin, BOARD.buttonBPin, BOARD.buttonCPin);
  gButtonController->setShortPressThreshold(BUTTON_SHORT_PRESS_MS);
  gButtonController->setLongPressThreshold(BUTTON_LONG_PRESS_MS);
  gButtonController->setOnRecenter(quickRecenterYaw);
  gButtonController->setOnFullCalibration(performFullCalibration);
  gButtonController->setOnPowerOff(powerOffSequence);
  gButtonController->setOnWakeScreen(exitScreenOffMode);
  
  // Check for factory reset request (hold Button B during startup)
  bool factoryResetRequested = !digitalRead(BOARD.buttonBPin);
  if (factoryResetRequested) {
    Serial.println("=== FACTORY RESET REQUESTED ===");
    display::handleEvent(display::DisplayEvent::FactoryResetPrompt);
    
    // Wait 3 seconds while Button B is held
    bool resetConfirmed = true;
    for (int i = 0; i < 30; i++) {
      if (digitalRead(BOARD.buttonBPin)) { // Button B released
        resetConfirmed = false;
        break;
      }
      delay(100);
      
      // Update countdown display
      display::handleEvent(display::DisplayEvent::FactoryResetTick, 3 - (i / 10));
    }
    
    if (resetConfirmed) {
      Serial.println("Factory reset confirmed - clearing all preferences");
      display::handleEvent(display::DisplayEvent::FactoryResetClearing);
      
      // Clear all stored preferences
      prefs.begin("sterzo", false);
      prefs.clear();
      prefs.end();
      
      display::handleEvent(display::DisplayEvent::FactoryResetDone);
      
      delay(2000);
      ESP.restart();
    } else {
      Serial.println("Factory reset cancelled");
      display::handleEvent(display::DisplayEvent::FactoryResetCancelled);
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
  if (!saveYawOffsetToPrefs(yawOffset)) {
    Serial.println("WARNING: Failed to persist startup yaw reset");
  }
  Serial.println("Yaw heading reset to 0°");

  // Initialize preferences
  Serial.println("Loading calibration data...");
  
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
    // Fast boot path: skip expensive Mahony warmup and let loop settle naturally.
    Serial.println("Fast boot path: skipping Mahony warmup");
  }

  // Defer BLE startup to loop() so boot path is non-blocking.
  bleStartupDeferredPending = true;
  bleDeferredStartAt = millis() + BLE_DEFER_STARTUP_MS;
  Serial.printf("BLE startup deferred by %lu ms\n", BLE_DEFER_STARTUP_MS);

  // Display startup message (only if splash not active)
  if (canDrawToDisplay()) {
    display::handleEvent(display::DisplayEvent::BootReady);
  }

  
  Serial.println("=== STERZStick READY ===");
  
  // Clear the startup message and prepare for the main display
  display::clearFrameBuffer();
  
  // Switch to normal brightness after startup sequence
  M5.Display.setBrightness(display::brightnessNormal());
  Serial.printf("Display brightness set to normal level (%d)\n", display::brightnessNormal());

  // Note: Wake-up handling now manages power modes directly
  // No immediate return to sleep needed as wake handler sets appropriate mode
}

void firmwareLoop() {
  M5.update(); // Update button states

  // Deferred BLE startup: keep boot responsive and retry if init fails.
  if (bleStartupDeferredPending && millis() >= bleDeferredStartAt) {
    Serial.println("Starting deferred BLE...");
    startBLE();
    if (bleStackInitialized) {
      bleStartupDeferredPending = false;
      bleStartTime = millis();
      Serial.println("Deferred BLE advertising started");
    } else {
      bleDeferredStartAt = millis() + BLE_DEFER_RETRY_MS;
      Serial.printf("Deferred BLE start failed, retrying in %lu ms\n", BLE_DEFER_RETRY_MS);
    }
  }

  // Update LED breathing pattern
  updateLEDBreathing();

  // Update non-blocking splash overlay
  updateSplashOverlay();

  // Handle non-blocking centering process
  updateCentering();

  // Handle button actions every loop pass, including IMU-throttled passes.
  handleButtons();

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

  // Calculate dynamic sample frequency with proper limiting for accurate IMU timing
  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = micros();
  dt = (currentTime - lastUpdateTime) / 1000000.0f; // Calculate time delta in seconds
  
  // Adaptive frequency limiting based on current IMU frequency
  float minInterval = 1.0f / currentIMUFrequency; // Calculate minimum interval for current frequency
  if (dt < minInterval) {
    // Calculate delay needed to maintain target frequency
    int delayMicros = (int)((minInterval - dt) * 1000000.0f);
    
    // Use hybrid approach for power efficiency with timing accuracy:
    // - For delays >5ms: use vTaskDelay for most of the wait (power efficient)
    // - For final <5ms: use delayMicroseconds for precision timing
    if (delayMicros > 5000) {
      int delayMs = (delayMicros - 1000) / 1000; // Leave 1ms for precision adjustment
      if (delayMs > 0 && delayMs < 200) {
        vTaskDelay(pdMS_TO_TICKS(delayMs));
      }
      // Recalculate remaining delay after task delay
      unsigned long afterDelay = micros();
      int remainingMicros = (int)((minInterval - (afterDelay - lastUpdateTime) / 1000000.0f) * 1000000.0f);
      if (remainingMicros > 0 && remainingMicros < 5000) {
        delayMicroseconds(remainingMicros);
      }
    } else if (delayMicros > 0 && delayMicros < 200000) {
      // Short delays: use precise microsecond delay
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

  // Debug IMU readings every 10 seconds (conditionally compiled for power savings)
  #ifdef DEBUG_MODE
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 10000) {
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
  #endif // DEBUG_MODE

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
  // CRITICAL: Use actual measured frequency (loopfreq) for accurate integration, not target frequency
  float actualFreq = loopfreq;
  if (actualFreq < 10.0f || actualFreq > 50.0f) {
    actualFreq = currentIMUFrequency; // Fallback to target if measurement seems wrong
  }
  
  MahonyAHRSupdateIMU(
    filteredGx * DEG_TO_RAD,
    filteredGy * DEG_TO_RAD,
    filteredGz * DEG_TO_RAD,
    ax,
    ay,
    az,
    actualFreq  // Use actual measured frequency for accurate time integration
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
  drawModernDisplay(rel, deviceConnected, currentPowerMode, 0.0f, (int)loopfreq);
  
  // Log inactivity status periodically
  logInactivityStatus();

}

void setup() {
  App::instance().setup();
}

void loop() {
  App::instance().loop();
}



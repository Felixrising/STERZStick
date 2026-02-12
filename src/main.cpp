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
#include "app/AppState.h"
#include "boards/BoardConfig.h"
#include "modules/ble/BleService.h"
#include "modules/display/DisplayController.h"
#include "modules/input/ButtonController.h"
#include "modules/power/PowerManager.h"
#include "modules/sensor/ImuService.h"

// —————— VERSION ——————
#define STERZSTICK_VERSION "v2.0-RC1"

#ifdef ESP32
extern HardwareSerial Serial;
#endif

// —————— Pin Setup ——————
static const board::BoardConfig& BOARD = board::current();
static ButtonController* gButtonController = nullptr;

// —————— Forward Declarations ——————
void handleButtons();
void powerOffSequence();
void firmwareSetup();
void firmwareLoop();

// —————— Button / Power Helpers ——————

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

// —————— SETUP ——————

void firmwareSetup() {
  Serial.begin(115200);
  Serial.println("=== STERZStick STARTUP ===");
  Serial.print("Version: ");
  Serial.println(STERZSTICK_VERSION);

  // Power management: Set HOLD pin high to maintain power (boards that need it).
  if (BOARD.hasHoldPin && BOARD.holdPin >= 0) {
    pinMode(BOARD.holdPin, OUTPUT);
    digitalWrite(BOARD.holdPin, HIGH);
    rtc_gpio_hold_en((gpio_num_t)BOARD.holdPin);
    Serial.println("HOLD pin set HIGH to maintain power");
  } else {
    Serial.println("Board uses PMIC power management (no hold pin)");
  }

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
  M5.Display.setBrightness(display::brightnessWake());
  screenOn = true;
  lastButtonTime = millis();

  // Initialize modern display system
  display::initializeDisplayBuffer();

  display::startSplashOverlay("STERZStick", "logo.jpg", STERZSTICK_VERSION, 2000, screenOn);

  Serial.println("Display configured with modern UI");

  // Setup LED with PWM for breathing pattern (only if board has an LED pin).
  if (BOARD.ledPin >= 0) {
    pinMode(BOARD.ledPin, OUTPUT);
    ledcSetup(0, 5000, 8);
    ledcAttachPin(BOARD.ledPin, 0);
    ledBreathingStartTime = millis();
    Serial.println("LED configured for breathing pattern");
  }

  // Initialize button GPIO pins.
  pinMode(BOARD.buttonAPin, INPUT_PULLUP);
  pinMode(BOARD.buttonBPin, INPUT_PULLUP);
  if (BOARD.buttonCPin >= 0) {
    pinMode(BOARD.buttonCPin, INPUT_PULLUP);
  }
  Serial.printf("Button GPIO pins configured (A=%d, B=%d, C=%d, count=%d)\n",
                BOARD.buttonAPin, BOARD.buttonBPin, BOARD.buttonCPin, BOARD.buttonCount);

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

    bool resetConfirmed = true;
    for (int i = 0; i < 30; i++) {
      if (digitalRead(BOARD.buttonBPin)) {
        resetConfirmed = false;
        break;
      }
      delay(100);
      display::handleEvent(display::DisplayEvent::FactoryResetTick, 3 - (i / 10));
    }

    if (resetConfirmed) {
      Serial.println("Factory reset confirmed - clearing all preferences");
      display::handleEvent(display::DisplayEvent::FactoryResetClearing);

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
  Serial.println("Yaw heading reset to 0");

  // Initialize preferences
  Serial.println("Loading calibration data...");

  if (!calibrationExists()) {
    Serial.println("No IMU calibration found - performing auto-calibration");
    performAutoCalibration();
  } else {
    Serial.println("IMU calibration loaded successfully");
    Serial.printf("Calibration loaded - YawOffset: %.3f (fresh start) DriftRate: %.2f/s\n",
                  yawOffset, YAW_DRIFT_RATE);
    Serial.println("Fast boot path: skipping Mahony warmup");
  }

  // Defer BLE startup to loop() so boot path is non-blocking.
  bleStartupDeferredPending = true;
  bleDeferredStartAt = millis() + BLE_DEFER_STARTUP_MS;
  Serial.printf("BLE startup deferred by %lu ms\n", BLE_DEFER_STARTUP_MS);

  if (display::canDrawToDisplay()) {
    display::handleEvent(display::DisplayEvent::BootReady);
  }

  Serial.println("=== STERZStick READY ===");

  display::clearFrameBuffer();

  // Switch to normal brightness after startup sequence
  M5.Display.setBrightness(display::brightnessNormal());
  Serial.printf("Display brightness set to normal level (%d)\n", display::brightnessNormal());
}

// —————— LOOP ——————

void firmwareLoop() {
  M5.update();

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
  display::updateSplashOverlay();

  // Handle non-blocking centering process
  updateCentering();

  // Handle button actions every loop pass
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
  }

  // Adaptive IMU frequency limiting
  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = micros();
  dt = (currentTime - lastUpdateTime) / 1000000.0f;

  float minInterval = 1.0f / currentIMUFrequency;
  if (dt < minInterval) {
    int delayMicros = (int)((minInterval - dt) * 1000000.0f);

    if (delayMicros > 5000) {
      int delayMs = (delayMicros - 1000) / 1000;
      if (delayMs > 0 && delayMs < 200) {
        vTaskDelay(pdMS_TO_TICKS(delayMs));
      }
      unsigned long afterDelay = micros();
      int remainingMicros = (int)((minInterval - (afterDelay - lastUpdateTime) / 1000000.0f) * 1000000.0f);
      if (remainingMicros > 0 && remainingMicros < 5000) {
        delayMicroseconds(remainingMicros);
      }
    } else if (delayMicros > 0 && delayMicros < 200000) {
      delayMicroseconds(delayMicros);
    }
    M5.update();
    return;
  }

  lastUpdateTime = currentTime;
  loopfreq = 1.0f / dt;
  if (loopfreq > currentIMUFrequency * 1.5f) loopfreq = currentIMUFrequency * 1.5f;

  // Read raw IMU, apply bias, fuse
  float gx, gy, gz, ax, ay, az;
  auto imu_update = M5.Imu.update();
  if (imu_update) {
    auto imu_data = M5.Imu.getImuData();
    gx = imu_data.gyro.x;
    gy = imu_data.gyro.y;
    gz = imu_data.gyro.z;
    ax = imu_data.accel.x;
    ay = imu_data.accel.y;
    az = imu_data.accel.z;
  } else {
    M5.Imu.getGyro(&gx, &gy, &gz);
    M5.Imu.getAccel(&ax, &ay, &az);
  }

  filterGyroReadings(gx, gy, gz);
  updatePowerManagement(gx, gy, gz, ax, ay, az);

  // Debug logging (conditionally compiled)
  #ifdef DEBUG_MODE
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 10000) {
    const char* powerModeStr;
    switch (currentPowerMode) {
      case POWER_BLE_ACTIVE:  powerModeStr = "BLE_ACTIVE";  break;
      case POWER_BLE_WAITING: powerModeStr = "BLE_WAITING"; break;
      case POWER_LOW_POWER:   powerModeStr = "LOW_POWER";   break;
      case POWER_ULP_SLEEP:   powerModeStr = "ULP_SLEEP";   break;
      default:                powerModeStr = "UNKNOWN";      break;
    }

    float gyroMag = sqrt(gx * gx + gy * gy + gz * gz);
    float rawY = getYaw();
    float curRel = rawY + yawOffset;
    while (curRel > 180) curRel -= 360;
    while (curRel < -180) curRel += 360;
    curRel = constrain(curRel, -40, 40);
    float curBin = round(curRel);
    if (abs(curBin) < 0.1f) curBin = 0.0f;

    Serial.printf("STATUS: Yaw=%.1f Bin=%.0f BLE=%s | Gyro=%.3f Freq=%.1f/%.1fHz %s Screen=%s CPU=%dMHz\n",
                  curRel, curBin, deviceConnected ? "Connected" : "Advertising",
                  gyroMag, loopfreq, currentIMUFrequency, powerModeStr,
                  screenOn ? "ON" : "OFF", getCpuFrequencyMhz());
    lastDebugTime = millis();
  }
  #endif

  // Gyro drift monitoring
  if (millis() - lastDriftSampleTime >= 1000) {
    addGyroDriftSample(gx, gy, gz);
    lastDriftSampleTime = millis();
  }

  static unsigned long lastDriftReportTime = 0;
  if (millis() - lastDriftReportTime >= 60000) {
    reportGyroDriftAverage();
    lastDriftReportTime = millis();
  }

  // Update Mahony AHRS with calibrated data
  float actualFreq = loopfreq;
  if (actualFreq < 10.0f || actualFreq > 50.0f) {
    actualFreq = currentIMUFrequency;
  }

  MahonyAHRSupdateIMU(
    filteredGx * DEG_TO_RAD,
    filteredGy * DEG_TO_RAD,
    filteredGz * DEG_TO_RAD,
    ax, ay, az,
    actualFreq
  );

  // Process yaw into steering output (centering + clamp + bin).
  float rawYaw = getYaw();
  float rel, bin;
  processYawToSteering(rawYaw, rel, bin);

  // Notify on true bin change.
  if (deviceConnected && ind32On && !isnan(lastSentBin) && bin != lastSentBin) {
    sendSteeringBin(bin);
  }
  lastSentBin = bin;

  // Update display.
  display::drawModernDisplay(rel, deviceConnected, currentPowerMode, 0.0f, (int)loopfreq,
                             screenOn, deviceConnected, zwiftConnected, lastMotionTime);

  logInactivityStatus();
}

// —————— Arduino Entry Points ——————

void setup() {
  firmwareSetup();
}

void loop() {
  firmwareLoop();
}

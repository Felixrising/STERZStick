#include "modules/power/PowerManager.h"

#include <M5Unified.h>
#include <Preferences.h>
#include <driver/rtc_io.h>
#include <esp_pm.h>
#include <esp_sleep.h>

#include "boards/BoardConfig.h"
#include "modules/ble/BleService.h"
#include "modules/display/DisplayController.h"

extern Preferences prefs;
extern bool screenOn;
extern bool deviceConnected;
extern bool zwiftConnected;
extern bool isCalibrating;
extern bool ulpProgramLoaded;
extern unsigned long lastButtonTime;
extern unsigned long lastMotionTime;
extern unsigned long lastBLEActivityTime;
extern unsigned long bleStartTime;
extern float motionAccumulator;
extern unsigned long lastMotionAccumulatorReset;

extern bool detectMotion(float gx, float gy, float gz, float ax, float ay, float az);

extern enum PowerMode currentPowerMode;

namespace {
constexpr int kBleCpuFreq = 80;
constexpr int kLowPowerCpuFreq = 10;
constexpr unsigned long kScreenOnTimeoutMs = 60000;
constexpr unsigned long kBleWaitTimeoutMs = 300000;
constexpr unsigned long kLowPowerTimeoutMs = 120000;
constexpr unsigned long kNoMotionTimeoutConnectedMs = 300000;

void requestBoardPowerOff() {
  const auto& cfg = board::current();
  if (cfg.hasPmicPowerOff) {
    Serial.println("Requesting PMIC power-off...");
    M5.Power.powerOff();
    delay(300);
    Serial.println("PMIC power-off did not complete, falling back to deep sleep");
    esp_deep_sleep_start();
  } else if (cfg.hasHoldPin) {
    Serial.println("Requesting HOLD-pin power-off...");
    pinMode(cfg.holdPin, OUTPUT);
    digitalWrite(cfg.holdPin, LOW);
    delay(300);
    Serial.println("HOLD power-off did not complete, falling back to deep sleep");
    esp_deep_sleep_start();
  } else {
    Serial.println("No board power-off mechanism, entering deep sleep");
    esp_deep_sleep_start();
  }
}
}  // namespace

void configurePowerManagement() {
  Serial.println("Configuring ESP32 power management...");
#if CONFIG_IDF_TARGET_ESP32S3
  esp_pm_config_esp32s3_t pm_config;
#else
  esp_pm_config_esp32_t pm_config;
#endif
  pm_config.max_freq_mhz = kBleCpuFreq;
  pm_config.min_freq_mhz = kLowPowerCpuFreq;
  pm_config.light_sleep_enable = true;

  esp_err_t ret = esp_pm_configure(&pm_config);
  if (ret == ESP_OK) {
    Serial.println("Power management configured successfully");
  } else {
    Serial.printf("Power management configuration failed: %d\n", ret);
  }

  Serial.println("Power management configured for BLE compatibility");
}

void setupULPProgram() {
  const auto& cfg = board::current();

  // Only configure RTC GPIO wake sources on boards that support them.
  if (!cfg.hasRtcWake) {
    Serial.println("Board has no RTC wake support, skipping ULP/ext wake setup");
    ulpProgramLoaded = true;
    return;
  }

#if !CONFIG_IDF_TARGET_ESP32S3
  Serial.println("Setting up ULP coprocessor program...");

  rtc_gpio_init((gpio_num_t)cfg.buttonAPin);
  rtc_gpio_set_direction((gpio_num_t)cfg.buttonAPin, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en((gpio_num_t)cfg.buttonAPin);
  rtc_gpio_pulldown_dis((gpio_num_t)cfg.buttonAPin);

  rtc_gpio_init((gpio_num_t)cfg.buttonBPin);
  rtc_gpio_set_direction((gpio_num_t)cfg.buttonBPin, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en((gpio_num_t)cfg.buttonBPin);
  rtc_gpio_pulldown_dis((gpio_num_t)cfg.buttonBPin);

  if (cfg.buttonCPin >= 0) {
    rtc_gpio_init((gpio_num_t)cfg.buttonCPin);
    rtc_gpio_set_direction((gpio_num_t)cfg.buttonCPin, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_en((gpio_num_t)cfg.buttonCPin);
    rtc_gpio_pulldown_dis((gpio_num_t)cfg.buttonCPin);
  }

  if (cfg.ext0WakeGpio >= 0) {
    esp_sleep_enable_ext0_wakeup((gpio_num_t)cfg.ext0WakeGpio, 0);
  }
  if (cfg.ext1WakeGpio >= 0) {
    uint64_t ext1_mask = (1ULL << cfg.ext1WakeGpio);
    esp_sleep_enable_ext1_wakeup(ext1_mask, ESP_EXT1_WAKEUP_ALL_LOW);
  }

  Serial.println("ULP program configured with button-only wake sources");
#else
  Serial.println("ESP32-S3: ext0/ext1 wake not used, skipping ULP setup");
#endif

  ulpProgramLoaded = true;
}

void setupIMUWakeup() {
  setupULPProgram();
  Serial.println("IMU wake-on-motion configured via ULP coprocessor");
}

void updateLEDBreathing() {
  // Skip LED breathing entirely if board has no addressable LED pin.
  if (board::current().ledPin < 0) return;

  static bool ledBreathingEnabled = true;
  if (!ledBreathingEnabled) {
    ledcWrite(0, 0);
    return;
  }

  static unsigned long lastLEDUpdate = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastLEDUpdate < 50) return;
  lastLEDUpdate = currentTime;

  float maxBrightness = 12.75f;
  if (deviceConnected) {
    if ((currentTime % 1000) < 100) {
      ledcWrite(0, (int)maxBrightness);
    } else {
      ledcWrite(0, 0);
    }
  } else {
    static unsigned long ledBreathingStartTime = 0;
    if (ledBreathingStartTime == 0) ledBreathingStartTime = currentTime;
    float breathingPeriod = 3000.0f;
    float phase = fmod((currentTime - ledBreathingStartTime), breathingPeriod) / breathingPeriod;
    float brightness = (sin(phase * 2.0f * PI - PI / 2.0f) + 1.0f) * 0.5f;
    ledcWrite(0, (int)(brightness * maxBrightness));
  }
}

void logInactivityStatus() {
  static unsigned long lastLogTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastLogTime < 1000) return;
  lastLogTime = currentTime;
}

void enterBLEWaitingMode() {
  if (currentPowerMode != POWER_BLE_WAITING) {
    Serial.println("Entering BLE waiting mode");
    startBLE();
    setCpuFrequencyMhz(kBleCpuFreq);
    if (screenOn) {
      display::clearFrameBuffer();
      M5.Display.sleep();
      screenOn = false;
    }
    currentPowerMode = POWER_BLE_WAITING;
    bleStartTime = millis();
    Serial.printf("BLE waiting mode - CPU: %d MHz, Screen: OFF\n", kBleCpuFreq);
  }
}

void enterBLEActiveMode() {
  if (currentPowerMode != POWER_BLE_ACTIVE) {
    Serial.println("Entering BLE active mode");
    setCpuFrequencyMhz(kBleCpuFreq);
    currentPowerMode = POWER_BLE_ACTIVE;
    Serial.printf("BLE active mode - CPU: %d MHz\n", kBleCpuFreq);
  }
}

void enterLowPowerMode() {
  if (currentPowerMode != POWER_LOW_POWER) {
    unsigned long currentTime = millis();
    unsigned long timeSinceActivity = min(currentTime - lastMotionTime, currentTime - lastButtonTime);
    if (timeSinceActivity < 60000) {
      Serial.printf("Cannot enter low power - recent activity (%lus ago)\n", timeSinceActivity / 1000);
      return;
    }

    Serial.println("Entering low power mode (no BLE)");
    stopBLE();
    setCpuFrequencyMhz(kLowPowerCpuFreq);
    if (screenOn) {
      display::clearFrameBuffer();
      M5.Display.sleep();
      screenOn = false;
    }
    if (board::current().ledPin >= 0) ledcWrite(0, 2);
    currentPowerMode = POWER_LOW_POWER;
    Serial.printf("Low power mode - CPU: %d MHz, BLE: OFF\n", kLowPowerCpuFreq);
  }
}

void enterScreenOffMode() {
  if (screenOn) {
    Serial.println("Turning off screen");
    display::clearFrameBuffer();
    M5.Display.sleep();
    screenOn = false;
    Serial.println("Screen off");
  }
}

void exitScreenOffMode() {
  if (!screenOn) {
    Serial.println("Turning on screen");
    M5.Display.wakeup();
    M5.Display.setBrightness(display::brightnessNormal());
    screenOn = true;
    Serial.println("Screen on at normal brightness");
  }
}

void enterULPSleep() {
  Serial.println("=== ENTERING POWER OFF MODE ===");
  M5.Display.wakeup();
  M5.Display.setBrightness(display::brightnessWake());
  display::handleEvent(display::DisplayEvent::EnterDeepSleep);
  delay(1200);

  display::clearFrameBuffer();
  M5.Display.sleep();
  if (board::current().ledPin >= 0) ledcWrite(0, 0);
  screenOn = false;
  stopBLE();

  prefs.begin("sterzo", false);
  prefs.putBool("wasAsleep", true);
  prefs.putULong("sleepTime", millis());
  prefs.end();

  currentPowerMode = POWER_ULP_SLEEP;
  Serial.println("Powering off board now...");
  Serial.flush();
  requestBoardPowerOff();
}

void handleWakeupFromSleep() {
  const auto& cfg = board::current();

  // Re-assert hold pin if this board uses one.
  if (cfg.hasHoldPin && cfg.holdPin >= 0) {
    pinMode(cfg.holdPin, OUTPUT);
    digitalWrite(cfg.holdPin, HIGH);
    rtc_gpio_hold_en((gpio_num_t)cfg.holdPin);
  }

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("WAKE-UP: Button A pressed (ext0)");
  } else if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
    Serial.println("WAKE-UP: Button C pressed (ext1)");
  } else {
    Serial.printf("WAKE-UP: Unknown source (%d)\n", wakeup_reason);
  }

  M5.Display.wakeup();
  M5.Display.setBrightness(display::brightnessWake());
  display::safeDisplayClear();
  M5.Display.setRotation(0);
  M5.Display.setBrightness(display::brightnessNormal());
  Serial.printf("Wake complete - brightness set to normal level (%d)\n", display::brightnessNormal());
  enterBLEWaitingMode();
}

void updatePowerManagement(float gx, float gy, float gz, float ax, float ay, float az) {
  unsigned long currentTime = millis();
  bool motionDetected = detectMotion(gx, gy, gz, ax, ay, az);

  if (motionDetected) {
    lastMotionTime = currentTime;
    if (!screenOn && currentTime - lastMotionTime <= 30000) {
      Serial.println("Motion detected - keeping screen on");
    } else {
      Serial.println("Motion detected - resetting activity timer");
    }
  }

  if (deviceConnected) {
    lastBLEActivityTime = currentTime;
    if (currentPowerMode != POWER_BLE_ACTIVE) enterBLEActiveMode();
  }

  bool shouldKeepScreenOn = false;
  if (currentTime - lastButtonTime <= kScreenOnTimeoutMs) shouldKeepScreenOn = true;
  else if (zwiftConnected) shouldKeepScreenOn = true;
  else if (deviceConnected && (currentTime - lastMotionTime <= kNoMotionTimeoutConnectedMs)) shouldKeepScreenOn = true;
  else if (!deviceConnected && (currentTime - lastMotionTime <= 30000)) shouldKeepScreenOn = true;

  if (shouldKeepScreenOn) {
    if (!screenOn) exitScreenOffMode();
  } else {
    if (screenOn) enterScreenOffMode();
  }

  if (!isCalibrating) {
    if (deviceConnected) {
      if (currentPowerMode != POWER_BLE_ACTIVE) enterBLEActiveMode();
    } else {
      unsigned long timeSinceActivity = min(currentTime - lastMotionTime, currentTime - lastButtonTime);
      unsigned long timeSinceBLEStart = currentTime - bleStartTime;
      switch (currentPowerMode) {
        case POWER_BLE_WAITING:
          if (timeSinceBLEStart > kBleWaitTimeoutMs && timeSinceActivity > 60000) {
            Serial.println("BLE timeout with no activity - powering off");
            enterULPSleep();
          } else if (timeSinceActivity < 30000) {
            bleStartTime = currentTime;
          }
          break;
        case POWER_BLE_ACTIVE:
          enterBLEWaitingMode();
          break;
        case POWER_LOW_POWER:
          if (timeSinceActivity > kLowPowerTimeoutMs) enterULPSleep();
          break;
        case POWER_ULP_SLEEP:
          break;
      }
    }
  }
}

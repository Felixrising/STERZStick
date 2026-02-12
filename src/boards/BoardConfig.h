#pragma once

#include <stdint.h>

namespace board {

struct BoardConfig {
  // GPIO pins (-1 = not available on this board)
  int ledPin;
  int holdPin;        // Power-hold GPIO; -1 if PMIC manages power
  int buttonAPin;
  int buttonBPin;
  int buttonCPin;     // -1 if power button is PMIC-managed
  int ext0WakeGpio;   // -1 if not using ESP32 ext0 wake
  int ext1WakeGpio;   // -1 if not using ESP32 ext1 wake

  // Board capabilities
  int  buttonCount;       // Number of user-readable buttons (2 or 3)
  bool hasHoldPin;        // true if power is maintained via a GPIO hold pin
  bool hasPmicPowerOff;   // true if board has PMIC-driven power-off (M5PM1 / AXP)
  bool hasRtcWake;        // true if RTC alarm can wake from deep sleep

  // IMU tuning
  uint8_t imuCalibStrength;  // M5Unified calibration strength (128 typical)
};

const BoardConfig& current();

}  // namespace board

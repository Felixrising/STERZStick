#pragma once

#include <Arduino.h>

enum PowerMode {
  POWER_BLE_ACTIVE,
  POWER_BLE_WAITING,
  POWER_LOW_POWER,
  POWER_ULP_SLEEP
};

struct BatteryInfo {
  int percentage;
  float voltage;
  String status;
  bool isCharging;
  bool isLow;
  bool isCritical;
};

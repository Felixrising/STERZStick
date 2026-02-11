#pragma once

#include <Arduino.h>

class ButtonController {
 public:
  using ActionCallback = void (*)();
  using WakeScreenCallback = void (*)();

  ButtonController(int buttonAPin, int buttonBPin, int buttonCPin);

  void setShortPressThreshold(unsigned long ms) { shortPressMs_ = ms; }
  void setLongPressThreshold(unsigned long ms) { longPressMs_ = ms; }

  void setOnRecenter(ActionCallback cb) { onRecenter_ = cb; }
  void setOnFullCalibration(ActionCallback cb) { onFullCalibration_ = cb; }
  void setOnPowerOff(ActionCallback cb) { onPowerOff_ = cb; }
  void setOnWakeScreen(WakeScreenCallback cb) { onWakeScreen_ = cb; }

  void update(bool& screenOn, unsigned long& lastButtonTime);

 private:
  int buttonAPin_;
  int buttonBPin_;
  int buttonCPin_;
  unsigned long shortPressMs_ = 1000;
  unsigned long longPressMs_ = 2000;

  unsigned long buttonAStartTime_ = 0;
  unsigned long buttonBStartTime_ = 0;
  unsigned long buttonCStartTime_ = 0;
  bool buttonAWasPressed_ = false;
  bool buttonBWasPressed_ = false;
  bool buttonCWasPressed_ = false;

  ActionCallback onRecenter_ = nullptr;
  ActionCallback onFullCalibration_ = nullptr;
  ActionCallback onPowerOff_ = nullptr;
  WakeScreenCallback onWakeScreen_ = nullptr;
};

#include "modules/input/ButtonController.h"

#include <M5Unified.h>

ButtonController::ButtonController(int buttonAPin, int buttonBPin, int buttonCPin)
    : buttonAPin_(buttonAPin), buttonBPin_(buttonBPin), buttonCPin_(buttonCPin) {}

void ButtonController::update(bool& screenOn, unsigned long& lastButtonTime) {
  bool buttonAState = !digitalRead(buttonAPin_);
  bool buttonBState = !digitalRead(buttonBPin_);
  bool buttonCState = !digitalRead(buttonCPin_);

  if (buttonAState && !buttonAWasPressed_) {
    buttonAStartTime_ = millis();
    buttonAWasPressed_ = true;
    lastButtonTime = millis();
    Serial.println("Button A pressed - resetting activity timer");
  } else if (!buttonAState && buttonAWasPressed_) {
    unsigned long pressDuration = millis() - buttonAStartTime_;
    buttonAWasPressed_ = false;

    if (pressDuration < shortPressMs_) {
      Serial.println("Button A: Short press - Waking screen");
      if (!screenOn && onWakeScreen_) {
        onWakeScreen_();
      }
      lastButtonTime = millis();
    } else {
      Serial.println("Button A: Long press - Recenter");
      M5.Speaker.tone(800, 100);
      if (onRecenter_) onRecenter_();
    }
  }

  if (buttonBState && !buttonBWasPressed_) {
    buttonBStartTime_ = millis();
    buttonBWasPressed_ = true;
    lastButtonTime = millis();
    Serial.println("Button B pressed - resetting activity timer");
  } else if (!buttonBState && buttonBWasPressed_) {
    unsigned long pressDuration = millis() - buttonBStartTime_;
    buttonBWasPressed_ = false;

    if (pressDuration < longPressMs_) {
      Serial.println("Button B: Short press - Recenter");
      M5.Speaker.tone(1000, 100);
      if (onRecenter_) onRecenter_();
    } else {
      Serial.println("Button B: Long press - Full calibration");
      if (onFullCalibration_) onFullCalibration_();
    }
  }

  if (buttonCState && !buttonCWasPressed_) {
    buttonCStartTime_ = millis();
    buttonCWasPressed_ = true;
    lastButtonTime = millis();
    Serial.println("Button C pressed - resetting activity timer");
  } else if (!buttonCState && buttonCWasPressed_) {
    unsigned long pressDuration = millis() - buttonCStartTime_;
    buttonCWasPressed_ = false;

    if (pressDuration < shortPressMs_) {
      Serial.println("Button C: Short press - Wake screen");
      M5.Speaker.tone(1200, 100);
      if (!screenOn && onWakeScreen_) {
        onWakeScreen_();
      }
      lastButtonTime = millis();
    } else if (pressDuration < longPressMs_) {
      Serial.println("Button C: Medium press - Screen wake");
      M5.Speaker.tone(1200, 100);
      if (!screenOn && onWakeScreen_) {
        onWakeScreen_();
      }
      lastButtonTime = millis();
    } else {
      Serial.println("Button C: Long press - Power off");
      if (onPowerOff_) onPowerOff_();
    }
  }
}

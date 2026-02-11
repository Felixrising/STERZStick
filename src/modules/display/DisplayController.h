#pragma once

#include <Arduino.h>
#include "app/types.h"

namespace display {

enum class DisplayState {
  Splash,
  Dashboard,
  Message
};

enum class DisplayEvent {
  BootReady,
  BleConnected,
  BleDisconnected,
  CenterCountdown,
  Centering,
  CenterError,
  CalibrationStart,
  CalibrationTick,
  CalibrationOk,
  AutoCalibrationPrompt,
  EnterDeepSleep,
  PowerOff,
  FactoryResetPrompt,
  FactoryResetTick,
  FactoryResetClearing,
  FactoryResetDone,
  FactoryResetCancelled
};

void initializeDisplayBuffer();
void handleEvent(DisplayEvent event, int value = 0);
void showOverlay(String text, int duration_ms);
void startSplashOverlay(const char* topText, const char* imagePath, const char* bottomText, int duration_ms, bool& screenOn);
void drawSplashContent();
void updateSplashOverlay();
bool isSplashActive();
void safeDisplayClear();
bool canDrawToDisplay();
void showSplashOverlay(const char* topText, const char* imagePath, const char* bottomText, int duration_ms, bool& screenOn);
void clearOverlay();
void showMessage(const char* line1, const char* line2 = "", const char* line3 = "", int duration_ms = 0, uint16_t color = 0xFFFF);
void clearMessage();
DisplayState currentState();
void drawModernDisplay(float yaw, bool connected, PowerMode powerMode, float battery, int freq,
                       bool screenOn, bool deviceConnected, bool zwiftConnected, unsigned long lastMotionTime);
void clearFrameBuffer();
int brightnessNormal();
int brightnessWake();

}  // namespace display

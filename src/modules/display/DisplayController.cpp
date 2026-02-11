#include "modules/display/DisplayController.h"

#include <LittleFS.h>
#include <M5Unified.h>

namespace display {

static LGFX_Sprite displaySprite(&M5.Display);
static unsigned long lastDisplayUpdate = 0;
static const unsigned long DISPLAY_UPDATE_INTERVAL = 100;
static float lastDisplayedYaw = 999.0f;
static bool lastDisplayedConnected = false;
static PowerMode lastDisplayedPowerMode = POWER_ULP_SLEEP;
static float lastDisplayedBattery = 0.0f;
static bool lastDisplayedMotionActive = false;
static unsigned long overlayEndTime = 0;
static String overlayText = "";
static float smoothedBatteryVoltage = -1.0f;
static const int BRIGHTNESS_NORMAL = 6;
static const int BRIGHTNESS_WAKE = 25;
static DisplayState state = DisplayState::Dashboard;

struct SplashState {
  bool active = false;
  unsigned long startTime = 0;
  unsigned long duration = 0;
  char topText[32] = "";
  char imagePath[32] = "";
  char bottomText[32] = "";
} splashState;

struct MessageState {
  bool active = false;
  unsigned long endTime = 0;
  uint16_t color = TFT_WHITE;
  char line1[32] = "";
  char line2[32] = "";
  char line3[32] = "";
} messageState;

struct BatteryPoint {
  float voltage;
  float percentage;
};

static const BatteryPoint liionCurve[] = {
    {4.20f, 100.0f}, {4.15f, 95.0f}, {4.11f, 90.0f}, {4.08f, 85.0f}, {4.02f, 80.0f}, {3.98f, 70.0f},
    {3.95f, 60.0f},  {3.91f, 50.0f}, {3.87f, 40.0f}, {3.82f, 30.0f}, {3.79f, 25.0f}, {3.77f, 20.0f},
    {3.74f, 15.0f},  {3.68f, 10.0f}, {3.45f, 5.0f},  {3.27f, 2.0f},  {3.20f, 0.0f}};

static const int curvePoints = sizeof(liionCurve) / sizeof(BatteryPoint);

static int getBatteryPercentage(float voltage) {
  voltage = constrain(voltage, 3.0f, 4.3f);
  if (voltage >= liionCurve[0].voltage) return 100;
  if (voltage <= liionCurve[curvePoints - 1].voltage) return 0;

  for (int i = 0; i < curvePoints - 1; i++) {
    if (voltage >= liionCurve[i + 1].voltage) {
      float v1 = liionCurve[i].voltage;
      float p1 = liionCurve[i].percentage;
      float v2 = liionCurve[i + 1].voltage;
      float p2 = liionCurve[i + 1].percentage;
      float percentage = p1 + ((voltage - v1) * (p2 - p1) / (v2 - v1));
      return constrain((int)round(percentage), 0, 100);
    }
  }
  return 0;
}

static BatteryInfo getBatteryInfo() {
  BatteryInfo info;
  info.voltage = M5.Power.getBatteryVoltage() / 1000.0f;
  info.percentage = getBatteryPercentage(info.voltage);
  info.isCharging = M5.Power.isCharging();
  info.isLow = (info.percentage <= 15);
  info.isCritical = (info.percentage <= 5);
  if (info.isCharging) {
    info.status = "CHARGING";
  } else if (info.isCritical) {
    info.status = "CRITICAL";
  } else if (info.isLow) {
    info.status = "LOW";
  } else {
    info.status = "GOOD";
  }
  return info;
}

static void pushSprite() {
  displaySprite.pushSprite(0, 0);
}

static void drawCenteredLine(const char* text, int y, uint16_t color = TFT_WHITE) {
  if (text == nullptr || text[0] == '\0') return;
  displaySprite.setTextColor(color);
  displaySprite.setTextDatum(MC_DATUM);
  displaySprite.drawString(text, displaySprite.width() / 2, y);
}

struct OverlayTextLayout {
  String line1;
  String line2;
  bool smallFont = false;
  int width = 0;
};

static void setOverlayFont(bool small) {
  if (small) {
    displaySprite.setFont(&fonts::Font0);
  } else {
    displaySprite.setFont(&fonts::Font2);
  }
}

static int measureTextWidth(const String& text, bool small) {
  setOverlayFont(small);
  return displaySprite.textWidth(text);
}

static String truncateToWidth(String text, int maxWidth, bool small) {
  if (measureTextWidth(text, small) <= maxWidth) return text;
  const String ellipsis = "...";
  while (text.length() > 0 && measureTextWidth(text + ellipsis, small) > maxWidth) {
    text.remove(text.length() - 1);
  }
  return text + ellipsis;
}

static OverlayTextLayout computeOverlayTextLayout(const String& text, int maxTextWidth) {
  OverlayTextLayout out;
  out.line1 = text;

  int widthLarge = measureTextWidth(text, false);
  if (widthLarge <= maxTextWidth) {
    out.smallFont = false;
    out.width = widthLarge;
    return out;
  }

  int widthSmall = measureTextWidth(text, true);
  if (widthSmall <= maxTextWidth) {
    out.smallFont = true;
    out.width = widthSmall;
    return out;
  }

  // Two-line wrap attempt for small font.
  int bestMaxWidth = INT32_MAX;
  int bestSplit = -1;
  for (int i = 1; i < (int)text.length() - 1; i++) {
    if (text.charAt(i) != ' ') continue;
    String l1 = text.substring(0, i);
    String l2 = text.substring(i + 1);
    l1.trim();
    l2.trim();
    int w1 = measureTextWidth(l1, true);
    int w2 = measureTextWidth(l2, true);
    if (w1 <= maxTextWidth && w2 <= maxTextWidth) {
      int maxW = max(w1, w2);
      if (maxW < bestMaxWidth) {
        bestMaxWidth = maxW;
        bestSplit = i;
      }
    }
  }

  if (bestSplit >= 0) {
    out.smallFont = true;
    out.line1 = text.substring(0, bestSplit);
    out.line2 = text.substring(bestSplit + 1);
    out.line1.trim();
    out.line2.trim();
    out.width = max(measureTextWidth(out.line1, true), measureTextWidth(out.line2, true));
    return out;
  }

  out.smallFont = true;
  out.line1 = truncateToWidth(text, maxTextWidth, true);
  out.line2 = "";
  out.width = measureTextWidth(out.line1, true);
  return out;
}

static void renderMessageScreen() {
  displaySprite.fillSprite(TFT_BLACK);
  displaySprite.setRotation(0);
  displaySprite.setFont(&fonts::Font2);
  drawCenteredLine(messageState.line1, 24, messageState.color);
  drawCenteredLine(messageState.line2, 44, TFT_WHITE);
  drawCenteredLine(messageState.line3, 64, TFT_WHITE);
  pushSprite();
}

void initializeDisplayBuffer() {
  displaySprite.setColorDepth(8);
  displaySprite.createSprite(M5.Display.width(), M5.Display.height());
  displaySprite.setPaletteColor(1, TFT_WHITE);
  displaySprite.setPaletteColor(2, TFT_GREEN);
  displaySprite.setPaletteColor(3, TFT_RED);
  displaySprite.setPaletteColor(4, 0x39E7);
}

void handleEvent(DisplayEvent event, int value) {
  switch (event) {
    case DisplayEvent::BootReady:
      showMessage("STERZStick", "Ready!", "", 600);
      break;
    case DisplayEvent::BleConnected:
      showOverlay("CONNECTED", 1500);
      break;
    case DisplayEvent::BleDisconnected:
      showOverlay("DISCONNECTED", 1500);
      break;
    case DisplayEvent::CenterCountdown: {
      String msg = "CENTER " + String(value);
      showOverlay(msg, 1100);
      break;
    }
    case DisplayEvent::Centering:
      showOverlay("CENTER", 1000);
      break;
    case DisplayEvent::CenterError:
      showOverlay("CENTER ERROR", 1500);
      break;
    case DisplayEvent::CalibrationStart:
      showOverlay("CAL", 1000);
      break;
    case DisplayEvent::CalibrationTick: {
      String msg = "CALIB " + String(value);
      showOverlay(msg, 1000);
      break;
    }
    case DisplayEvent::CalibrationOk:
      showOverlay("CALIBRATION OK", 1500);
      break;
    case DisplayEvent::AutoCalibrationPrompt:
      showMessage("AUTO CALIBRATION", "Keep device still", "10 seconds...", 1200);
      break;
    case DisplayEvent::EnterDeepSleep:
      showMessage("GOING TO", "DEEP SLEEP", "", 0, ORANGE);
      break;
    case DisplayEvent::PowerOff:
      showMessage("Powering", "Off...", "", 0);
      break;
    case DisplayEvent::FactoryResetPrompt:
      showMessage("FACTORY RESET", "Hold Button B", "for 3 seconds", 0, ORANGE);
      break;
    case DisplayEvent::FactoryResetTick: {
      String countdown = "Countdown: " + String(value);
      showMessage("FACTORY RESET", "Hold Button B", countdown.c_str(), 0, ORANGE);
      break;
    }
    case DisplayEvent::FactoryResetClearing:
      showMessage("CLEARING", "PREFERENCES", "", 0, ORANGE);
      break;
    case DisplayEvent::FactoryResetDone:
      showMessage("DONE!", "Restarting...", "", 0);
      break;
    case DisplayEvent::FactoryResetCancelled:
      showMessage("Reset cancelled", "", "", 1000);
      break;
  }
}

void showOverlay(String text, int duration_ms) {
  unsigned long effectiveDuration = (unsigned long)duration_ms;
  if (isSplashActive()) {
    unsigned long elapsed = millis() - splashState.startTime;
    if (elapsed < splashState.duration) {
      effectiveDuration += (splashState.duration - elapsed);
    }
  }
  overlayText = text;
  overlayEndTime = millis() + effectiveDuration;
}

void startSplashOverlay(const char* topText, const char* imagePath, const char* bottomText, int duration_ms, bool& screenOn) {
  state = DisplayState::Splash;
  splashState.active = true;
  splashState.startTime = millis();
  splashState.duration = duration_ms;
  strncpy(splashState.topText, topText, sizeof(splashState.topText) - 1);
  strncpy(splashState.imagePath, imagePath, sizeof(splashState.imagePath) - 1);
  strncpy(splashState.bottomText, bottomText, sizeof(splashState.bottomText) - 1);

  if (!screenOn) {
    M5.Display.wakeup();
    M5.Display.setBrightness(BRIGHTNESS_NORMAL);
    screenOn = true;
  }
  drawSplashContent();
}

void drawSplashContent() {
  displaySprite.fillSprite(TFT_BLACK);
  displaySprite.setRotation(0);
  displaySprite.setTextColor(WHITE);
  displaySprite.setTextSize(2);
  displaySprite.setTextDatum(MC_DATUM);

  int screenCenterX = displaySprite.width() / 2;
  int screenHeight = displaySprite.height();
  if (strlen(splashState.topText) > 0) {
    displaySprite.setTextDatum(TC_DATUM);
    displaySprite.drawString(splashState.topText, screenCenterX, 10);
  }

  if (strlen(splashState.imagePath) > 0) {
    char fullPath[40];
    snprintf(fullPath, sizeof(fullPath), "/%s", splashState.imagePath);
    if (LittleFS.exists(fullPath)) {
      int imageSize = 135;
      int imageX = screenCenterX - (imageSize / 2);
      int imageY = (screenHeight / 2) - (imageSize / 2);
      displaySprite.drawJpgFile(LittleFS, fullPath, imageX, imageY, 0, 0, 0, 0, 1.0f, 1.061f);
    } else {
      displaySprite.setTextDatum(MC_DATUM);
      displaySprite.setTextSize(1);
      displaySprite.drawString("Image not found:", screenCenterX, screenHeight / 2 - 10);
      displaySprite.drawString(splashState.imagePath, screenCenterX, screenHeight / 2 + 10);
      displaySprite.setTextSize(2);
    }
  }

  if (strlen(splashState.bottomText) > 0) {
    displaySprite.setTextDatum(BC_DATUM);
    displaySprite.drawString(splashState.bottomText, screenCenterX, screenHeight - 10);
  }

  pushSprite();
}

void updateSplashOverlay() {
  if (messageState.active && messageState.endTime > 0 && millis() >= messageState.endTime) {
    clearMessage();
  }

  if (!splashState.active) return;
  if (millis() - splashState.startTime >= splashState.duration) {
    splashState.active = false;
    if (state == DisplayState::Splash) {
      state = DisplayState::Dashboard;
    }
    splashState.topText[0] = '\0';
    splashState.imagePath[0] = '\0';
    splashState.bottomText[0] = '\0';
  }
}

bool isSplashActive() {
  return state == DisplayState::Splash && splashState.active;
}

void safeDisplayClear() {
  if (isSplashActive()) {
    drawSplashContent();
  } else {
    clearFrameBuffer();
  }
}

bool canDrawToDisplay() {
  return state != DisplayState::Splash;
}

void showSplashOverlay(const char* topText, const char* imagePath, const char* bottomText, int duration_ms, bool& screenOn) {
  startSplashOverlay(topText, imagePath, bottomText, duration_ms, screenOn);
}

void clearOverlay() {
  overlayEndTime = 0;
  overlayText = "";
}

void showMessage(const char* line1, const char* line2, const char* line3, int duration_ms, uint16_t color) {
  state = DisplayState::Message;
  messageState.active = true;
  messageState.endTime = duration_ms > 0 ? millis() + (unsigned long)duration_ms : 0;
  messageState.color = color;
  strncpy(messageState.line1, line1 ? line1 : "", sizeof(messageState.line1) - 1);
  strncpy(messageState.line2, line2 ? line2 : "", sizeof(messageState.line2) - 1);
  strncpy(messageState.line3, line3 ? line3 : "", sizeof(messageState.line3) - 1);
  renderMessageScreen();
}

void clearMessage() {
  messageState.active = false;
  messageState.endTime = 0;
  messageState.line1[0] = '\0';
  messageState.line2[0] = '\0';
  messageState.line3[0] = '\0';
  if (state == DisplayState::Message) {
    state = DisplayState::Dashboard;
  }
}

DisplayState currentState() {
  return state;
}

void drawModernDisplay(float yaw, bool connected, PowerMode powerMode, float battery, int freq,
                       bool screenOn, bool deviceConnected, bool zwiftConnected, unsigned long lastMotionTime) {
  (void)battery;
  if (state != DisplayState::Dashboard) return;
  if (!screenOn) return;

  int yawInt = round(yaw);
  bool headingDigitChanged = (abs(yawInt - round(lastDisplayedYaw)) > 0);

  unsigned long displayInterval = DISPLAY_UPDATE_INTERVAL;
  bool hasYawChange = (abs(yaw - lastDisplayedYaw) > 0.5f);
  bool hasStatusChange = (connected != lastDisplayedConnected || powerMode != lastDisplayedPowerMode);
  bool motionActive = (millis() - lastMotionTime < 5000);
  if (zwiftConnected || hasYawChange || hasStatusChange || motionActive) {
    displayInterval = DISPLAY_UPDATE_INTERVAL;
  } else if (deviceConnected) {
    displayInterval = 200;
  } else {
    displayInterval = 1000;
  }
  // Keep status/battery redraws throttled, but update immediately on heading digit change.
  if (!headingDigitChanged && (millis() - lastDisplayUpdate < displayInterval)) return;
  if (isSplashActive()) {
    lastDisplayUpdate = millis();
    return;
  }

  static unsigned long lastBatteryRead = 0;
  static BatteryInfo cachedBatteryInfo;
  if (millis() - lastBatteryRead > 1000) {
    cachedBatteryInfo = getBatteryInfo();
    lastBatteryRead = millis();
  }
  BatteryInfo batteryInfo = cachedBatteryInfo;

  if (smoothedBatteryVoltage < 0) {
    smoothedBatteryVoltage = batteryInfo.voltage;
  } else {
    smoothedBatteryVoltage = smoothedBatteryVoltage * 0.9f + batteryInfo.voltage * 0.1f;
  }
  int batteryPercentage = getBatteryPercentage(smoothedBatteryVoltage);
  bool motionIsActive = (millis() - lastMotionTime <= 30000);

  bool overlayActive = (millis() < overlayEndTime);
  static bool lastOverlayActive = false;
  bool overlayJustExpired = (lastOverlayActive && !overlayActive);

  bool needsUpdate = false;
  if (abs(yawInt - round(lastDisplayedYaw)) > 0 || connected != lastDisplayedConnected || powerMode != lastDisplayedPowerMode ||
      motionIsActive != lastDisplayedMotionActive || abs(batteryPercentage - round(lastDisplayedBattery)) > 0 || overlayActive ||
      overlayJustExpired) {
    needsUpdate = true;
  }
  lastOverlayActive = overlayActive;
  if (!needsUpdate) return;

  const int w = displaySprite.width();
  const int h = displaySprite.height();
  const bool compact = (w <= 140);
  const int headerH = compact ? 14 : 24;
  const int footerH = compact ? 14 : 20;
  const int mainTop = headerH + (compact ? 4 : 8);
  const int mainBottom = h - footerH - (compact ? 8 : 10);
  const int mainCenterY = (mainTop + mainBottom) / 2;

  displaySprite.fillSprite(TFT_BLACK);

  // Header row: use compact content on small displays.
  displaySprite.drawFastHLine(0, headerH, w, TFT_DARKGREY);
  if (compact) displaySprite.setFont(&fonts::Font0);
  else displaySprite.setFont(&fonts::Font2);
  displaySprite.setTextDatum(TL_DATUM);
  displaySprite.setTextColor(connected ? TFT_GREEN : TFT_RED);
  displaySprite.fillCircle(6, 7, 3, connected ? TFT_GREEN : TFT_RED);
  if (compact) {
    displaySprite.drawString(connected ? "ON" : "OFF", 12, 2);
  } else {
    displaySprite.drawString(connected ? "BLE ON" : "BLE OFF", 12, 2);
    displaySprite.setTextDatum(TC_DATUM);
    displaySprite.setTextColor(TFT_LIGHTGREY);
    displaySprite.drawString("STERZ", w / 2, 2);
  }

  displaySprite.setTextDatum(TR_DATUM);
  uint16_t batteryColor = TFT_WHITE;
  if (batteryInfo.isCharging) batteryColor = TFT_CYAN;
  else if (batteryInfo.isCritical) batteryColor = TFT_RED;
  else if (batteryInfo.isLow) batteryColor = TFT_ORANGE;
  displaySprite.setTextColor(batteryColor);
  String battStr = String(batteryPercentage) + "%";
  if (batteryInfo.isCharging) battStr += "+";
  const int batteryBodyW = compact ? 12 : 14;
  const int batteryBodyH = 8;
  const int batteryX = w - (compact ? 16 : 20);
  const int batteryY = 3;
  displaySprite.drawString(battStr, batteryX - 4, compact ? 1 : 2);
  displaySprite.drawRect(batteryX, batteryY, batteryBodyW, batteryBodyH, batteryColor);
  displaySprite.fillRect(batteryX + batteryBodyW, batteryY + 2, 2, 4, batteryColor);

  int batteryLevelWidth = map(batteryPercentage, 0, 100, 0, batteryBodyW - 2);
  uint16_t fillColor;
  if (batteryInfo.isCharging) fillColor = TFT_CYAN;
  else if (batteryPercentage < 5) fillColor = TFT_RED;
  else if (batteryPercentage < 15) fillColor = TFT_ORANGE;
  else fillColor = TFT_GREEN;
  if (batteryLevelWidth > 0) displaySprite.fillRect(batteryX + 1, batteryY + 1, batteryLevelWidth, batteryBodyH - 2, fillColor);
  if (batteryInfo.isCharging && (millis() / 500) % 2 == 0) {
    displaySprite.fillRect(batteryX + 1, batteryY + 1, batteryBodyW - 2, batteryBodyH - 2, TFT_CYAN);
  }

  // Main value zone.
  displaySprite.setTextDatum(TC_DATUM);
  if (compact) displaySprite.setFont(&fonts::Font0);
  else displaySprite.setFont(&fonts::Font2);
  displaySprite.setTextColor(TFT_DARKGREY);
  displaySprite.drawString("STEERING", w / 2, mainTop);
  bool atLimit = (yawInt < -39 || yawInt > 39);
  bool flashOn = (millis() / 500) % 2 == 0;

  // Direction prefix above value: L (left), R (right), blank at zero.
  if (yawInt != 0) {
    const char* dirPrefix = (yawInt < 0) ? "L" : "R";
    if (compact) displaySprite.setFont(&fonts::Font2);
    else displaySprite.setFont(&fonts::Font4);
    displaySprite.setTextColor((atLimit && flashOn) ? TFT_RED : TFT_LIGHTGREY);
    displaySprite.drawString(dirPrefix, w / 2, mainTop + (compact ? 15 : 17));
  }

  displaySprite.setFont(&fonts::Font7);
  displaySprite.setTextDatum(MC_DATUM);
  String yawNumStr = String(abs(yawInt));
  int x_center = w / 2;
  displaySprite.setTextColor((atLimit && flashOn) ? TFT_RED : TFT_WHITE);

  // Keep numeric digits fixed at center.
  displaySprite.drawString(yawNumStr, x_center, mainCenterY);

  // Steering gauge.
  int gaugeWidth = w - 30;
  int gaugeX = (w - gaugeWidth) / 2;
  int gaugeY = mainBottom - 12;
  displaySprite.drawRect(gaugeX, gaugeY, gaugeWidth, 8, TFT_DARKGREY);
  int centerX = gaugeX + gaugeWidth / 2;
  int indicatorPos = map(yawInt, -40, 40, 0, gaugeWidth - 4);
  int indicatorX = gaugeX + 2 + indicatorPos;
  if (yawInt != 0) {
    int fillStart, fillWidth;
    uint16_t directionColor;
    if (yawInt > 0) {
      fillStart = centerX;
      fillWidth = indicatorX - centerX;
      directionColor = TFT_CYAN;
    } else {
      fillStart = indicatorX;
      fillWidth = centerX - indicatorX;
      directionColor = TFT_ORANGE;
    }
    if (fillWidth > 0) displaySprite.fillRect(fillStart, gaugeY + 1, fillWidth, 6, directionColor);
  }
  displaySprite.drawFastVLine(centerX, gaugeY, 8, TFT_WHITE);
  displaySprite.drawFastHLine(centerX - 1, gaugeY - 2, 3, TFT_WHITE);
  if (compact) displaySprite.setFont(&fonts::Font0);
  else displaySprite.setFont(&fonts::Font2);
  displaySprite.setTextDatum(BC_DATUM);
  displaySprite.setTextColor(TFT_DARKGREY);
  displaySprite.drawString("-40", gaugeX, gaugeY - 2);
  displaySprite.drawString("40", gaugeX + gaugeWidth, gaugeY - 2);

  // Footer: compact mode uses status only to avoid overlaps.
  displaySprite.drawFastHLine(0, h - footerH - 2, w, TFT_DARKGREY);
  if (compact) displaySprite.setFont(&fonts::Font0);
  else displaySprite.setFont(&fonts::Font2);
  displaySprite.setTextColor(TFT_WHITE);
  String powerStatus = "";
  if (zwiftConnected) powerStatus = compact ? "CONN" : "CONNECTED";
  else if (motionIsActive) powerStatus = compact ? "ACTIVE" : "ACTIVE";
  else {
    switch (powerMode) {
      case POWER_BLE_ACTIVE: powerStatus = compact ? "CONN" : "CONNECTED"; break;
      case POWER_BLE_WAITING: powerStatus = compact ? "WAIT" : "WAITING"; break;
      case POWER_LOW_POWER: powerStatus = compact ? "STBY" : "STANDBY"; break;
      default: powerStatus = compact ? "STBY" : "STANDBY"; break;
    }
  }
  if (compact) {
    displaySprite.setTextDatum(BC_DATUM);
    displaySprite.drawString(powerStatus, w / 2, h - 2);
  } else {
    displaySprite.setTextDatum(BL_DATUM);
    displaySprite.drawString(powerStatus, 4, h - 3);
    displaySprite.setTextDatum(BR_DATUM);
    displaySprite.setTextColor(TFT_LIGHTGREY);
    displaySprite.drawString(String(freq) + "Hz", w - 3, h - 3);
  }

  if (millis() < overlayEndTime) {
    const int minOverlayW = compact ? 80 : 92;
    const int maxOverlayW = w - 12;
    const int maxTextW = maxOverlayW - 14;
    OverlayTextLayout layout = computeOverlayTextLayout(overlayText, maxTextW);
    int overlayWidth = constrain(layout.width + 14, minOverlayW, maxOverlayW);
    int overlayX = (displaySprite.width() - overlayWidth) / 2;
    int overlayH = layout.line2.length() > 0 ? 40 : (layout.smallFont ? 30 : 44);
    int overlayY = mainCenterY - (overlayH / 2);
    displaySprite.fillRoundRect(overlayX, overlayY, overlayWidth, overlayH, 8, 0x2104);
    displaySprite.drawRoundRect(overlayX, overlayY, overlayWidth, overlayH, 8, TFT_LIGHTGREY);
    displaySprite.setTextDatum(MC_DATUM);
    setOverlayFont(layout.smallFont);
    displaySprite.setTextColor(TFT_WHITE);
    if (layout.line2.length() > 0) {
      displaySprite.drawString(layout.line1, displaySprite.width() / 2, overlayY + 14);
      displaySprite.drawString(layout.line2, displaySprite.width() / 2, overlayY + 27);
    } else {
      displaySprite.drawString(layout.line1, displaySprite.width() / 2, overlayY + (overlayH / 2));
    }
  }

  if (canDrawToDisplay()) pushSprite();

  lastDisplayedYaw = yaw;
  lastDisplayedConnected = connected;
  lastDisplayedPowerMode = powerMode;
  lastDisplayedBattery = batteryPercentage;
  lastDisplayedMotionActive = motionIsActive;
  lastDisplayUpdate = millis();
}

void clearFrameBuffer() {
  displaySprite.fillSprite(TFT_BLACK);
  pushSprite();
  if (state != DisplayState::Splash) {
    state = DisplayState::Dashboard;
  }
  messageState.active = false;
  messageState.endTime = 0;
}

int brightnessNormal() { return BRIGHTNESS_NORMAL; }
int brightnessWake() { return BRIGHTNESS_WAKE; }

}  // namespace display

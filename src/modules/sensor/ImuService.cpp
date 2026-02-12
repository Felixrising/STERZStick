#include "modules/sensor/ImuService.h"

#include <M5Unified.h>
#include <Preferences.h>

#include "boards/BoardConfig.h"
#include "modules/display/DisplayController.h"

extern float twoKp;
extern float twoKi;
extern float q0;
extern float q1;
extern float q2;
extern float q3;
extern float integralFBx;
extern float integralFBy;
extern float integralFBz;
extern char anglesComputed;

extern Preferences prefs;
extern bool screenOn;
extern bool isCalibrating;

extern float yawOffset;
extern float currentIMUFrequency;
extern float motionAccumulator;
extern unsigned long lastMotionAccumulatorReset;

extern float lastAccelMagnitude;
extern float lastGyroMagnitude;
extern float gyroFilterAlpha;
extern float filteredGx;
extern float filteredGy;
extern float filteredGz;

extern float lastMeasuredRawYaw;
extern unsigned long lastYawMeasurementTime;
extern bool hasValidYawMeasurement;

extern float gyroDriftSamples[];
extern int gyroDriftIndex;
extern bool driftArrayFilled;
extern unsigned long lastDriftSampleTime;

extern uint8_t imuCalibCountdown;
extern bool imuCalibrationActive;

extern const float YAW_DRIFT_RATE;
extern const bool DEBUG_MODE;

namespace {
constexpr float kTwoKpDef = (2.0f * 0.5f);
constexpr float kTwoKiDef = (2.0f * 0.0f);
constexpr float kMotionThreshold = 10.0f;
constexpr float kAccelMotionThreshold = 0.2f;
constexpr int kDriftSamplesCount = 60;

bool centeringActive = false;
unsigned long centeringStartTime = 0;
int centeringCountdown = 0;
unsigned long lastCountdownTime = 0;

float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}
}  // namespace

void initMahonyData() {
  twoKp = kTwoKpDef;
  twoKi = kTwoKiDef;
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
  integralFBx = 0.0f;
  integralFBy = 0.0f;
  integralFBz = 0.0f;
  anglesComputed = 0;
}

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float samplefrequency) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    gx += 2.0f * halfex;
    gy += 2.0f * halfey;
    gz += 2.0f * halfez;
  }

  gx *= (0.5f * (1.0f / samplefrequency));
  gy *= (0.5f * (1.0f / samplefrequency));
  gz *= (0.5f * (1.0f / samplefrequency));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  anglesComputed = 0;
}

float getYaw() {
  float yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
  yaw *= 57.29578f;
  if (yaw > 180) yaw -= 360;
  else if (yaw < -180) yaw += 360;
  return -yaw;
}

float getRoll() {
  float roll = atan2f(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
  return roll * 57.29578f;
}

float getPitch() {
  float pitch = asinf(-2.0f * (q1 * q3 - q0 * q2));
  return pitch * 57.29578f;
}

bool detectMotion(float gx, float gy, float gz, float ax, float ay, float az) {
  float gyroMagnitude = sqrt(gx * gx + gy * gy + gz * gz);
  float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);
  float accelDelta = abs(accelMagnitude - lastAccelMagnitude);
  float gyroDelta = abs(gyroMagnitude - lastGyroMagnitude);

  bool gyroMotion = gyroDelta > kMotionThreshold;
  bool accelMotion = accelDelta > kAccelMotionThreshold;

  if (gyroMotion) {
    Serial.printf("MOTION: Gyroscope event. Value: %.2f dps > Threshold: %.2f dps\n", gyroDelta, kMotionThreshold);
  }
  if (accelMotion) {
    Serial.printf("MOTION: Accelerometer event. Value: %.2f g > Threshold: %.2f g\n", accelDelta, kAccelMotionThreshold);
  }

  float alpha = 0.7f;
  lastGyroMagnitude = alpha * lastGyroMagnitude + (1.0f - alpha) * gyroMagnitude;
  lastAccelMagnitude = alpha * lastAccelMagnitude + (1.0f - alpha) * accelMagnitude;

  if (gyroMotion || accelMotion) {
    motionAccumulator += gyroDelta + (accelDelta * 5.0f);
  }

  unsigned long currentTime = millis();
  if (currentTime - lastMotionAccumulatorReset > 30000) {
    motionAccumulator = 0;
    lastMotionAccumulatorReset = currentTime;
  }

  return gyroMotion || accelMotion;
}

bool hasSignificantAccumulatedMotion() { return motionAccumulator > 3.0f; }

void filterGyroReadings(float gx, float gy, float gz) {
  filteredGx = gyroFilterAlpha * gx + (1.0f - gyroFilterAlpha) * filteredGx;
  filteredGy = gyroFilterAlpha * gy + (1.0f - gyroFilterAlpha) * filteredGy;
  filteredGz = gyroFilterAlpha * gz + (1.0f - gyroFilterAlpha) * filteredGz;
}

void initGyroDriftMonitoring() {
  for (int i = 0; i < kDriftSamplesCount; i++) {
    gyroDriftSamples[i] = 0.0f;
  }
  gyroDriftIndex = 0;
  driftArrayFilled = false;
  lastDriftSampleTime = millis();
  Serial.println("Gyro drift monitoring initialized");
}

void addGyroDriftSample(float gx, float gy, float gz) {
  float gyroMagnitude = sqrt(gx * gx + gy * gy + gz * gz);
  gyroDriftSamples[gyroDriftIndex] = gyroMagnitude;
  gyroDriftIndex = (gyroDriftIndex + 1) % kDriftSamplesCount;

  if (gyroDriftIndex == 0 && !driftArrayFilled) {
    driftArrayFilled = true;
    Serial.println("Gyro drift buffer filled - starting 60s trailing average reports");
  }
}

void reportGyroDriftAverage() {
  if (!driftArrayFilled && gyroDriftIndex < 10) {
    return;
  }

  int sampleCount = driftArrayFilled ? kDriftSamplesCount : gyroDriftIndex;
  float sum = 0.0f;
  float minDrift = 999.0f;
  float maxDrift = -999.0f;

  for (int i = 0; i < sampleCount; i++) {
    sum += gyroDriftSamples[i];
    if (gyroDriftSamples[i] < minDrift) minDrift = gyroDriftSamples[i];
    if (gyroDriftSamples[i] > maxDrift) maxDrift = gyroDriftSamples[i];
  }

  float avgDrift = sum / sampleCount;
  float variance = 0.0f;
  for (int i = 0; i < sampleCount; i++) {
    float diff = gyroDriftSamples[i] - avgDrift;
    variance += diff * diff;
  }
  float stdDev = sqrt(variance / sampleCount);

  Serial.println("=== 60-SECOND GYRO DRIFT REPORT ===");
  Serial.printf("Samples: %d | Avg: %.3f°/s | Min: %.3f°/s | Max: %.3f°/s | StdDev: %.3f°/s\n", sampleCount,
                avgDrift, minDrift, maxDrift, stdDev);
  Serial.println("=====================================");
}

bool calibrationExists() { return loadIMUCalibration(); }

void startCentering() {
  Serial.println("Starting centering process...");
  M5.Speaker.tone(800, 50);

  centeringActive = true;
  centeringStartTime = millis();
  centeringCountdown = 3;
  lastCountdownTime = millis();

  display::handleEvent(display::DisplayEvent::CenterCountdown, 3);
  M5.Speaker.tone(600, 100);
}

void updateCentering() {
  if (!centeringActive) return;

  unsigned long currentTime = millis();
  if (centeringCountdown > 0 && currentTime - lastCountdownTime >= 1000) {
    centeringCountdown--;
    lastCountdownTime = currentTime;

    if (centeringCountdown > 0) {
      display::handleEvent(display::DisplayEvent::CenterCountdown, centeringCountdown);
      M5.Speaker.tone(600, 100);
    } else {
      display::handleEvent(display::DisplayEvent::Centering);
      executeCentering();
    }
  }
}

void executeCentering() {
  float currentRawYaw = getYaw();

  if (!isnan(currentRawYaw) && !isinf(currentRawYaw)) {
    yawOffset = -currentRawYaw;
    while (yawOffset > 180) yawOffset -= 360;
    while (yawOffset < -180) yawOffset += 360;

    if (!saveYawOffsetToPrefs(yawOffset)) {
      Serial.println("WARNING: Failed to save yaw offset");
    }
    Serial.printf("Centering complete - Current raw yaw: %.1f°, New offset: %.1f°\n", currentRawYaw, yawOffset);

    lastMeasuredRawYaw = currentRawYaw;
    lastYawMeasurementTime = millis();
    hasValidYawMeasurement = true;
  } else {
    Serial.println("WARNING: Invalid yaw value during centering");
    display::handleEvent(display::DisplayEvent::CenterError);
  }

  centeringActive = false;
}

void quickRecenterYaw() { startCentering(); }

void recenterYaw() { startCentering(); }

void performAutoCalibration() {
  Serial.println("=== AUTO CALIBRATION ===");
  if (screenOn && display::canDrawToDisplay()) {
    display::handleEvent(display::DisplayEvent::AutoCalibrationPrompt);
  }
  startIMUCalibration();
}

void performAutoRecenter() { recenterYaw(); }

void initializeMahonyFilter() {
  Serial.println("Initializing Mahony filter with current orientation...");
  initMahonyData();
  const int initSamples = 100;

  for (int i = 0; i < initSamples; i++) {
    float gx, gy, gz, ax, ay, az;
    M5.Imu.getGyro(&gx, &gy, &gz);
    M5.Imu.getAccel(&ax, &ay, &az);

    filterGyroReadings(gx, gy, gz);
    MahonyAHRSupdateIMU(filteredGx * DEG_TO_RAD, filteredGy * DEG_TO_RAD, filteredGz * DEG_TO_RAD, ax, ay, az,
                        currentIMUFrequency);
    delay(100);
  }

  float initialYaw = getYaw();
  Serial.printf("Mahony filter initialized - Initial yaw: %.1f°\n", initialYaw);
}

void performGyroCalibration() {
  Serial.println("Gyro calibration - using new IMU calibration system");
  startIMUCalibration();
}

void performFullCalibration() {
  Serial.println("Full calibration - using new IMU calibration system");
  startIMUCalibration();
}

void startIMUCalibration() {
  Serial.println("=== STARTING IMU CALIBRATION ===");
  uint8_t calStr = board::current().imuCalibStrength;
  M5.Imu.setCalibration(calStr, calStr, 0);
  imuCalibCountdown = 10;
  isCalibrating = true;
  display::handleEvent(display::DisplayEvent::CalibrationStart);
  Serial.println("IMU calibration started - keep device still for 10 seconds");
}

void updateIMUCalibration(uint32_t countdown, bool clear) {
  imuCalibCountdown = static_cast<uint8_t>(countdown);
  if (countdown > 0) {
    display::handleEvent(display::DisplayEvent::CalibrationTick, (int)countdown);
  }

  if (countdown == 0) {
    clear = true;
  }

  if (clear) {
    if (countdown > 0) {
      uint8_t cs = board::current().imuCalibStrength;
      M5.Imu.setCalibration(cs, cs, 0);
    } else {
      M5.Imu.setCalibration(0, 0, 0);
      saveIMUCalibration();
    }
  }
}

void stopIMUCalibration() {
  Serial.println("=== STOPPING IMU CALIBRATION ===");
  M5.Imu.setCalibration(0, 0, 0);
  saveIMUCalibration();
  recenterYaw();
  imuCalibrationActive = false;
  isCalibrating = false;
  display::handleEvent(display::DisplayEvent::CalibrationOk);
  Serial.println("=== IMU CALIBRATION COMPLETE ===");
}

bool loadIMUCalibration() {
  Serial.println("=== LOADING IMU CALIBRATION FROM NVS ===");
  bool success = M5.Imu.loadOffsetFromNVS();

  if (success) {
    Serial.println("IMU calibration loaded successfully");
    Serial.println("Loaded bias values:");
    float gyroXBias = 0, gyroYBias = 0, gyroZBias = 0;
    float accelXBias = 0, accelYBias = 0, accelZBias = 0;

    for (int i = 0; i < 9; i++) {
      float offset = M5.Imu.getOffsetData(i) * (1.0f / (1 << 19));
      const char* sensorName = (i < 3) ? "Gyro" : (i < 6) ? "Accel" : "Mag";
      const char* axisName = (i % 3 == 0) ? "X" : (i % 3 == 1) ? "Y" : "Z";
      const char* units = (i < 3) ? "dps" : (i < 6) ? "g" : "uT";

      if (i == 0) gyroXBias = offset;
      else if (i == 1) gyroYBias = offset;
      else if (i == 2) gyroZBias = offset;
      else if (i == 3) accelXBias = offset;
      else if (i == 4) accelYBias = offset;
      else if (i == 5) accelZBias = offset;

      Serial.printf("  %s %s [%d]: %+.6f %s\n", sensorName, axisName, i, offset, units);
    }

    float gyroBiasMagnitude = sqrt(gyroXBias * gyroXBias + gyroYBias * gyroYBias + gyroZBias * gyroZBias);
    Serial.printf(">>> GYRO BIAS SUMMARY: X=%+.3f Y=%+.3f Z=%+.3f (Magnitude: %.3f dps)\n", gyroXBias, gyroYBias,
                  gyroZBias, gyroBiasMagnitude);
    (void)accelXBias;
    (void)accelYBias;
    (void)accelZBias;

    if (gyroBiasMagnitude > 1.0f) {
      Serial.println("WARNING: Gyro bias magnitude >1 dps");
    } else if (gyroBiasMagnitude < 0.01f) {
      Serial.println("WARNING: Gyro bias very small - calibration may not have worked");
    } else {
      Serial.println("Gyro bias values look reasonable");
    }
  } else {
    Serial.println("No IMU calibration found in NVS - will perform auto-calibration");
  }

  return success;
}

void saveIMUCalibration() {
  Serial.println("=== SAVING IMU CALIBRATION TO NVS ===");
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

bool saveYawOffsetToPrefs(float offset) {
  if (!prefs.begin("sterzo", false)) {
    Serial.println("ERROR: Failed to open preferences namespace 'sterzo'");
    return false;
  }
  prefs.putFloat("yoff", offset);
  prefs.end();
  return true;
}

void processYawToSteering(float rawYaw, float& outRel, float& outBin) {
  // Validate yaw value.
  if (isnan(rawYaw) || isinf(rawYaw)) {
    Serial.println("WARNING: Invalid yaw value detected!");
    rawYaw = 0.0f;
  }

  // Calculate relative yaw: raw + offset, normalise to [-180, 180].
  float rel = rawYaw + yawOffset;
  while (rel > 180) rel -= 360;
  while (rel < -180) rel += 360;

  // Centering force -- gradually adjust offset to bring rel back to 0.
  static unsigned long lastCenteringTime = millis();
  unsigned long centeringTime = millis();
  float deltaTime = (centeringTime - lastCenteringTime) / 1000.0f;

  if (deltaTime >= 1.0f) {
    float centeringAdjustment = -rel * YAW_DRIFT_RATE * deltaTime;
    float maxAdj = YAW_DRIFT_RATE;
    centeringAdjustment = constrain(centeringAdjustment, -maxAdj, maxAdj);

    yawOffset += centeringAdjustment;
    while (yawOffset > 180) yawOffset -= 360;
    while (yawOffset < -180) yawOffset += 360;

    if (DEBUG_MODE && abs(centeringAdjustment) > 0.01f) {
      float rate = centeringAdjustment / deltaTime;
      Serial.printf("Centering: rel=%.2f adj=%.3f rate=%.3f/s dt=%.1fs\n",
                    rel, centeringAdjustment, rate, deltaTime);
    }
    lastCenteringTime = centeringTime;
  }

  // Recompute with updated offset.
  rel = rawYaw + yawOffset;
  while (rel > 180) rel -= 360;
  while (rel < -180) rel += 360;

  // Clamp to steering range and bin to 1-degree resolution.
  rel = constrain(rel, -40, 40);
  float bin = round(rel / 1.0f) * 1.0f;
  if (abs(bin) < 0.1f) bin = 0.0f;

  outRel = rel;
  outBin = bin;
}

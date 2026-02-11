#pragma once

#include <Arduino.h>
#include <BLE2902.h>
#include <BLEServer.h>
#include <Preferences.h>

#include "app/types.h"

// BLE objects/state shared across modules.
extern BLEServer* pServer;
extern BLEService* pSvc;
extern BLECharacteristic* pChar14;
extern BLECharacteristic* pChar30;
extern BLECharacteristic* pChar31;
extern BLECharacteristic* pChar32;
extern BLE2902* p2902_14;
extern BLE2902* p2902_30;
extern BLE2902* p2902_32;
extern bool bleStackInitialized;
extern bool deviceConnected;
extern bool ind32On;
extern bool challengeOK;
extern bool zwiftConnected;

// Core app state.
extern Preferences prefs;
extern float yawOffset;
extern float lastSentBin;
extern bool isCalibrating;
extern uint8_t imuCalibCountdown;
extern bool imuCalibrationActive;
extern PowerMode currentPowerMode;
extern bool screenOn;
extern bool ulpProgramLoaded;

// Timing and activity state.
extern unsigned long lastButtonTime;
extern unsigned long lastMotionTime;
extern unsigned long lastBLEActivityTime;
extern unsigned long bleStartTime;
extern unsigned long ledBreathingStartTime;
extern unsigned long lastDriftSampleTime;
extern unsigned long lastMotionAccumulatorReset;
extern unsigned long bleDeferredStartAt;
extern bool bleStartupDeferredPending;

// IMU/filter state shared with sensor module.
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
extern float currentIMUFrequency;
extern float dt;
extern float loopfreq;
extern float gyroFilterAlpha;
extern float filteredGx;
extern float filteredGy;
extern float filteredGz;
extern float lastAccelMagnitude;
extern float lastGyroMagnitude;
extern float motionAccumulator;
extern float lastMeasuredRawYaw;
extern unsigned long lastYawMeasurementTime;
extern bool hasValidYawMeasurement;
extern float gyroDriftSamples[];
extern int gyroDriftIndex;
extern bool driftArrayFilled;

// Tunables currently shared in main loop/logging.
extern const bool DEBUG_MODE;
extern const float YAW_DRIFT_RATE;
extern const int DRIFT_SAMPLES_COUNT;
extern const unsigned long SCREEN_ON_TIMEOUT;
extern const unsigned long BLE_WAIT_TIMEOUT;
extern const unsigned long LOW_POWER_TIMEOUT;
extern const unsigned long NO_MOTION_TIMEOUT_CONNECTED;
extern const float NORMAL_IMU_FREQUENCY;
extern const float CALIBRATION_IMU_FREQUENCY;
extern const int BLE_CPU_FREQ;
extern const int LOW_POWER_CPU_FREQ;
extern const int DEEP_SLEEP_CPU_FREQ;
extern const unsigned long BLE_DEFER_STARTUP_MS;
extern const unsigned long BLE_DEFER_RETRY_MS;
extern const unsigned long BUTTON_SHORT_PRESS_MS;
extern const unsigned long BUTTON_LONG_PRESS_MS;

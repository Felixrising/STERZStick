#include "app/AppState.h"

// Mahony filter state.
float twoKp = (2.0f * 0.5f);
float twoKi = (2.0f * 0.0f);
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
char anglesComputed = 0;

// Global services/state.
Preferences prefs;
BLEServer* pServer = nullptr;
BLEService* pSvc = nullptr;
BLECharacteristic* pChar14 = nullptr;
BLECharacteristic* pChar30 = nullptr;
BLECharacteristic* pChar31 = nullptr;
BLECharacteristic* pChar32 = nullptr;
BLE2902* p2902_14 = nullptr;
BLE2902* p2902_30 = nullptr;
BLE2902* p2902_32 = nullptr;
bool bleStackInitialized = false;

bool deviceConnected = false;
bool ind32On = false;
bool challengeOK = false;
bool zwiftConnected = false;
bool screenOn = false;
bool isCalibrating = false;
bool ulpProgramLoaded = false;

// Calibration/steering state.
float yawOffset = 0.0f;
float lastSentBin = NAN;
uint8_t imuCalibCountdown = 0;
bool imuCalibrationActive = false;

// Power/activity state.
PowerMode currentPowerMode = POWER_BLE_WAITING;
unsigned long lastButtonTime = 0;
unsigned long lastMotionTime = 0;
unsigned long lastBLEActivityTime = 0;
unsigned long bleStartTime = 0;
unsigned long ledBreathingStartTime = 0;
unsigned long lastMotionAccumulatorReset = 0;
unsigned long bleDeferredStartAt = 0;
bool bleStartupDeferredPending = false;

// IMU timing/filtering state.
float dt = 0.0f;
float loopfreq = 0.0f;
float currentIMUFrequency = 25.0f;
float gyroFilterAlpha = 0.1f;
float filteredGx = 0.0f, filteredGy = 0.0f, filteredGz = 0.0f;
float lastAccelMagnitude = 1.0f;
float lastGyroMagnitude = 0.0f;
float motionAccumulator = 0.0f;
float lastMeasuredRawYaw = 0.0f;
unsigned long lastYawMeasurementTime = 0;
bool hasValidYawMeasurement = false;

// Drift monitoring.
const int DRIFT_SAMPLES_COUNT = 60;
float gyroDriftSamples[DRIFT_SAMPLES_COUNT];
int gyroDriftIndex = 0;
unsigned long lastDriftSampleTime = 0;
bool driftArrayFilled = false;

// Shared constants.
const bool DEBUG_MODE = false;
const float YAW_DRIFT_RATE = 0.2f;
const unsigned long SCREEN_ON_TIMEOUT = 60000;
const unsigned long BLE_WAIT_TIMEOUT = 300000;
const unsigned long LOW_POWER_TIMEOUT = 120000;
const unsigned long NO_MOTION_TIMEOUT_CONNECTED = 300000;
const float NORMAL_IMU_FREQUENCY = 25.0f;
const float CALIBRATION_IMU_FREQUENCY = 25.0f;
const int BLE_CPU_FREQ = 80;
const int LOW_POWER_CPU_FREQ = 10;
const int DEEP_SLEEP_CPU_FREQ = 80;
const unsigned long BLE_DEFER_STARTUP_MS = 2500;
const unsigned long BLE_DEFER_RETRY_MS = 10000;
const unsigned long BUTTON_SHORT_PRESS_MS = 1000;
const unsigned long BUTTON_LONG_PRESS_MS = 2000;

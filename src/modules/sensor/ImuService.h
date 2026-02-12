#pragma once

#include <Arduino.h>

void initMahonyData();
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float samplefrequency);
float getYaw();
float getRoll();
float getPitch();

bool detectMotion(float gx, float gy, float gz, float ax, float ay, float az);
bool hasSignificantAccumulatedMotion();
void filterGyroReadings(float gx, float gy, float gz);
void initGyroDriftMonitoring();
void addGyroDriftSample(float gx, float gy, float gz);
void reportGyroDriftAverage();

bool calibrationExists();
void startCentering();
void updateCentering();
void executeCentering();
void quickRecenterYaw();
void recenterYaw();
void performAutoCalibration();
void performAutoRecenter();
void initializeMahonyFilter();
void performGyroCalibration();
void performFullCalibration();

void startIMUCalibration();
void updateIMUCalibration(uint32_t countdown, bool clear = false);
void stopIMUCalibration();
bool loadIMUCalibration();
void saveIMUCalibration();
bool saveYawOffsetToPrefs(float offset);

// Applies centering drift compensation and computes the final steering output.
// outRel receives the continuous relative yaw in [-40, 40].
// outBin receives the 1-degree binned steering angle.
void processYawToSteering(float rawYaw, float& outRel, float& outBin);

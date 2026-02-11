#pragma once

void configurePowerManagement();
void setupULPProgram();
void setupIMUWakeup();
void updateLEDBreathing();
void logInactivityStatus();
void enterBLEWaitingMode();
void enterBLEActiveMode();
void enterLowPowerMode();
void enterScreenOffMode();
void exitScreenOffMode();
void enterULPSleep();
void handleWakeupFromSleep();
void updatePowerManagement(float gx, float gy, float gz, float ax, float ay, float az);

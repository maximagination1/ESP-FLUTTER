#pragma once

void initHardwareConfig();
void saveHardwareConfig();

int getLeftServoPin();
int getRightServoPin();
int getLedPin();
int getBatteryPin();
int getBatteryWarningLedPin();
int getStatusLedPin();
int getElrsRxPin();
int getElrsTxPin();

void setHardwarePins(
    int leftServoPin,
    int rightServoPin,
    int ledPin,
    int batteryPin,
    int batteryWarningLedPin,
    int statusLedPin,
    int elrsRxPin,
    int elrsTxPin);

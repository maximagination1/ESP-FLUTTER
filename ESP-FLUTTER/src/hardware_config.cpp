#include "hardware_config.h"

#include "config.h"

#include <Arduino.h>
#include <Preferences.h>

static Preferences prefsHardware;
static const char *HARDWARE_NS = "hardware";
static const char *KEY_LEFT_SERVO_PIN = "lservo";
static const char *KEY_RIGHT_SERVO_PIN = "rservo";
static const char *KEY_LED_PIN = "led";
static const char *KEY_BATTERY_PIN = "batt";
static const char *KEY_BATTERY_WARNING_LED_PIN = "battled";
static const char *KEY_STATUS_LED_PIN = "status";
static const char *KEY_ELRS_RX_PIN = "elrsrx";
static const char *KEY_ELRS_TX_PIN = "elrstx";

static int leftServoPin = LEFT_SERVO_PIN;
static int rightServoPin = RIGHT_SERVO_PIN;
static int ledPin = LED_PIN;
static int batteryPin = BATTERY_PIN;
static int batteryWarningLedPin = BATTERY_WARNING_LED_PIN;
static int statusLedPin = STATUS_LED_PIN;
static int elrsRxPin = ELRS_RX_PIN;
static int elrsTxPin = ELRS_TX_PIN;

static int clampGpio(int pin)
{
    return constrain(pin, 0, 48);
}

void initHardwareConfig()
{
    prefsHardware.begin(HARDWARE_NS, true);
    leftServoPin = prefsHardware.getInt(KEY_LEFT_SERVO_PIN, LEFT_SERVO_PIN);
    rightServoPin = prefsHardware.getInt(KEY_RIGHT_SERVO_PIN, RIGHT_SERVO_PIN);
    ledPin = prefsHardware.getInt(KEY_LED_PIN, LED_PIN);
    batteryPin = prefsHardware.getInt(KEY_BATTERY_PIN, BATTERY_PIN);
    batteryWarningLedPin = prefsHardware.getInt(KEY_BATTERY_WARNING_LED_PIN, BATTERY_WARNING_LED_PIN);
    statusLedPin = prefsHardware.getInt(KEY_STATUS_LED_PIN, STATUS_LED_PIN);
    elrsRxPin = prefsHardware.getInt(KEY_ELRS_RX_PIN, ELRS_RX_PIN);
    elrsTxPin = prefsHardware.getInt(KEY_ELRS_TX_PIN, ELRS_TX_PIN);
    prefsHardware.end();

    leftServoPin = clampGpio(leftServoPin);
    rightServoPin = clampGpio(rightServoPin);
    ledPin = clampGpio(ledPin);
    batteryPin = clampGpio(batteryPin);
    batteryWarningLedPin = clampGpio(batteryWarningLedPin);
    statusLedPin = clampGpio(statusLedPin);
    elrsRxPin = clampGpio(elrsRxPin);
    elrsTxPin = clampGpio(elrsTxPin);
}

void saveHardwareConfig()
{
    prefsHardware.begin(HARDWARE_NS, false);
    prefsHardware.putInt(KEY_LEFT_SERVO_PIN, leftServoPin);
    prefsHardware.putInt(KEY_RIGHT_SERVO_PIN, rightServoPin);
    prefsHardware.putInt(KEY_LED_PIN, ledPin);
    prefsHardware.putInt(KEY_BATTERY_PIN, batteryPin);
    prefsHardware.putInt(KEY_BATTERY_WARNING_LED_PIN, batteryWarningLedPin);
    prefsHardware.putInt(KEY_STATUS_LED_PIN, statusLedPin);
    prefsHardware.putInt(KEY_ELRS_RX_PIN, elrsRxPin);
    prefsHardware.putInt(KEY_ELRS_TX_PIN, elrsTxPin);
    prefsHardware.end();
}

int getLeftServoPin() { return leftServoPin; }
int getRightServoPin() { return rightServoPin; }
int getLedPin() { return ledPin; }
int getBatteryPin() { return batteryPin; }
int getBatteryWarningLedPin() { return batteryWarningLedPin; }
int getStatusLedPin() { return statusLedPin; }
int getElrsRxPin() { return elrsRxPin; }
int getElrsTxPin() { return elrsTxPin; }

void setHardwarePins(
    int newLeftServoPin,
    int newRightServoPin,
    int newLedPin,
    int newBatteryPin,
    int newBatteryWarningLedPin,
    int newStatusLedPin,
    int newElrsRxPin,
    int newElrsTxPin)
{
    leftServoPin = clampGpio(newLeftServoPin);
    rightServoPin = clampGpio(newRightServoPin);
    ledPin = clampGpio(newLedPin);
    batteryPin = clampGpio(newBatteryPin);
    batteryWarningLedPin = clampGpio(newBatteryWarningLedPin);
    statusLedPin = clampGpio(newStatusLedPin);
    elrsRxPin = clampGpio(newElrsRxPin);
    elrsTxPin = clampGpio(newElrsTxPin);
    saveHardwareConfig();
}

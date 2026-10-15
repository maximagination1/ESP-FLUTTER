#include "led_control.h"

#include "config.h"
#include "hardware_config.h"

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

static const uint8_t LED_PWM_CHANNEL = 5;
static const uint32_t LED_PWM_FREQ = 5000;
static const uint8_t LED_PWM_RES_BITS = 8;
static const int LED_PWM_MAX = 255;

static Preferences prefsLed;
static const char *LED_PREF_NS = "led";
static const char *KEY_LED_MODE = "mode";
static const char *KEY_LED_BRIGHTNESS = "bright";
static const char *KEY_LED_SPEED = "speed";

static int ledMode = 0;       // 0=off, 1=solid, 2=fade, 3=strobe
static int brightnessLevel = 10;
static int speedLevel = 5;
static int attachedLedPin = LED_PIN;

static int clampLevel(int value)
{
    if (value < 1) return 1;
    if (value > 10) return 10;
    return value;
}

static int brightnessToDuty()
{
    return map(brightnessLevel, 1, 10, 6, LED_PWM_MAX);
}

static void writeLedDuty(int duty)
{
    duty = constrain(duty, 0, LED_PWM_MAX);
    ledcWrite(LED_PWM_CHANNEL, duty);
}

static void saveLedPrefs()
{
    prefsLed.begin(LED_PREF_NS, false);
    prefsLed.putInt(KEY_LED_MODE, ledMode);
    prefsLed.putInt(KEY_LED_BRIGHTNESS, brightnessLevel);
    prefsLed.putInt(KEY_LED_SPEED, speedLevel);
    prefsLed.end();
}

void initLedControl()
{
    prefsLed.begin(LED_PREF_NS, true);
    ledMode = prefsLed.getInt(KEY_LED_MODE, 0);
    brightnessLevel = prefsLed.getInt(KEY_LED_BRIGHTNESS, brightnessLevel);
    speedLevel = prefsLed.getInt(KEY_LED_SPEED, speedLevel);
    prefsLed.end();

    if (ledMode < 0 || ledMode > 3) ledMode = 0;
    brightnessLevel = clampLevel(brightnessLevel);
    speedLevel = clampLevel(speedLevel);

    ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQ, LED_PWM_RES_BITS);
    attachedLedPin = getLedPin();
    ledcAttachPin(attachedLedPin, LED_PWM_CHANNEL);
    writeLedDuty(0);
}

void reattachLedOutput()
{
    writeLedDuty(0);
    ledcDetachPin(attachedLedPin);
    attachedLedPin = getLedPin();
    ledcAttachPin(attachedLedPin, LED_PWM_CHANNEL);
    updateLedControl();
}

void updateLedControl()
{
    const int maxDuty = brightnessToDuty();

    if (ledMode == 0)
    {
        writeLedDuty(0);
        return;
    }

    if (ledMode == 1)
    {
        writeLedDuty(maxDuty);
        return;
    }

    const unsigned long now = millis();

    if (ledMode == 2)
    {
        const float periodMs = 2600.0f - (speedLevel - 1) * 240.0f;
        const float phase = fmodf((float)now, periodMs) / periodMs;
        const float wave = 0.5f - 0.5f * cosf(phase * TWO_PI);
        const int minDuty = max(1, maxDuty / 20);
        writeLedDuty(minDuty + (int)((maxDuty - minDuty) * wave));
        return;
    }

    if (ledMode == 3)
    {
        const unsigned long periodMs = 1000UL - (unsigned long)((speedLevel - 1) * 100UL);
        const bool on = (now % periodMs) < (periodMs / 2);
        writeLedDuty(on ? maxDuty : 0);
    }
}

void setLedMode(int mode)
{
    if (mode < 0) mode = 0;
    if (mode > 3) mode = 3;
    if (ledMode == mode)
        return;
    ledMode = mode;
    saveLedPrefs();
}

void cycleLedMode()
{
    ledMode = (ledMode + 1) % 4;
    saveLedPrefs();
}

int getLedMode()
{
    return ledMode;
}

void setLedBrightnessLevel(int level)
{
    level = clampLevel(level);
    if (brightnessLevel == level)
        return;
    brightnessLevel = clampLevel(level);
    saveLedPrefs();
}

int getLedBrightnessLevel()
{
    return brightnessLevel;
}

void setLedEffectSpeedLevel(int level)
{
    level = clampLevel(level);
    if (speedLevel == level)
        return;
    speedLevel = clampLevel(level);
    saveLedPrefs();
}

int getLedEffectSpeedLevel()
{
    return speedLevel;
}

const char *getLedModeName()
{
    switch (ledMode)
    {
        case 1: return "Solid";
        case 2: return "Fade";
        case 3: return "Strobe";
        default: return "OFF";
    }
}

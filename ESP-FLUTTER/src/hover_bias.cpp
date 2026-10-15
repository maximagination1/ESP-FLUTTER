#include "hover_bias.h"

#include <Arduino.h>
#include <Preferences.h>

static Preferences prefsHoverBias;
static const char *HOVER_BIAS_NS = "hoverbias";
static const char *KEY_FORWARD_NEUTRAL = "fwd";
static const char *KEY_HOVER_NEUTRAL = "hover";
static const char *KEY_ENABLED = "enabled";

static bool hoverBiasEnabled = false;
static bool filterInitialized = false;
static float currentDevicePitchDeg = 0.0f;
static float filteredBias = 0.0f;
static float forwardNeutralAngle = 95.0f;
static float hoverNeutralAngle = 85.0f;
static const float HOVER_BIAS_SMOOTHING_MS = 35.0f;
static unsigned long lastUpdateMs = 0;

static float clampFloat(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static void saveHoverBiasConfig()
{
    prefsHoverBias.begin(HOVER_BIAS_NS, false);
    prefsHoverBias.putFloat(KEY_FORWARD_NEUTRAL, forwardNeutralAngle);
    prefsHoverBias.putFloat(KEY_HOVER_NEUTRAL, hoverNeutralAngle);
    prefsHoverBias.end();
}

static void saveHoverBiasEnabled()
{
    prefsHoverBias.begin(HOVER_BIAS_NS, false);
    prefsHoverBias.putBool(KEY_ENABLED, hoverBiasEnabled);
    prefsHoverBias.end();
}

void initHoverBias()
{
    prefsHoverBias.begin(HOVER_BIAS_NS, true);
    forwardNeutralAngle = prefsHoverBias.getFloat(KEY_FORWARD_NEUTRAL, forwardNeutralAngle);
    hoverNeutralAngle = prefsHoverBias.getFloat(KEY_HOVER_NEUTRAL, hoverNeutralAngle);
    hoverBiasEnabled = prefsHoverBias.getBool(KEY_ENABLED, hoverBiasEnabled);
    prefsHoverBias.end();

    forwardNeutralAngle = clampFloat(forwardNeutralAngle, 90.0f, 100.0f);
    hoverNeutralAngle = clampFloat(hoverNeutralAngle, 80.0f, 90.0f);
    filterInitialized = false;
    filteredBias = 0.0f;
}

void setHoverBiasEnabled(bool enabled)
{
    if (hoverBiasEnabled == enabled) return;

    hoverBiasEnabled = enabled;
    saveHoverBiasEnabled();
    filterInitialized = false;
    filteredBias = 0.0f;
    lastUpdateMs = millis();
}

bool isHoverBiasEnabled()
{
    return hoverBiasEnabled;
}

void setHoverBiasConfig(float forwardNeutralDeg, float hoverNeutralDeg)
{
    forwardNeutralAngle = clampFloat(forwardNeutralDeg, 90.0f, 100.0f);
    hoverNeutralAngle = clampFloat(hoverNeutralDeg, 80.0f, 90.0f);
    saveHoverBiasConfig();
}

void updateHoverBiasDevicePitch(float devicePitchDeg)
{
    if (!hoverBiasEnabled)
    {
        filterInitialized = false;
        filteredBias = 0.0f;
        lastUpdateMs = millis();
        return;
    }

    currentDevicePitchDeg = clampFloat(devicePitchDeg, 0.0f, 45.0f);
    float targetBias = currentDevicePitchDeg / 45.0f;
    unsigned long now = millis();
    float dtMs = (lastUpdateMs == 0) ? 0.0f : (float)(now - lastUpdateMs);
    lastUpdateMs = now;

    if (!filterInitialized || dtMs <= 0.0f)
    {
        filteredBias = targetBias;
        filterInitialized = true;
        return;
    }

    float alpha = dtMs / (HOVER_BIAS_SMOOTHING_MS + dtMs);
    filteredBias += alpha * (targetBias - filteredBias);
}

float getHoverBias()
{
    return hoverBiasEnabled ? filteredBias : 0.0f;
}

float getHoverBiasNeutralAngle(float fallbackNeutralDeg)
{
    if (!hoverBiasEnabled) return fallbackNeutralDeg;
    float bias = getHoverBias();
    return forwardNeutralAngle + (hoverNeutralAngle - forwardNeutralAngle) * bias;
}

float getHoverBiasDevicePitch()
{
    return currentDevicePitchDeg;
}

float getHoverBiasForwardNeutralAngle()
{
    return forwardNeutralAngle;
}

float getHoverBiasHoverNeutralAngle()
{
    return hoverNeutralAngle;
}

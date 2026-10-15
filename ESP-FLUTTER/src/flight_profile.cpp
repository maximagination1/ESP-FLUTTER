#include "flight_profile.h"
#include "config.h"

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

static Preferences prefsProfile;
static const char *PROFILE_NS = "profiles";
static const char *KEY_MODE = "mode";
static const char *KEY_PRESET_VERSION = "presetVer";
static const int PROFILE_PRESET_VERSION = 9;

static FlightProfileMode currentProfile = FLIGHT_PROFILE_CRUISE;

static FlightProfileSettings profiles[3] = {
    // Cruise: efficient, predictable forward flight.
    {LOWER_FLAP_AMPLITUDE_DEFAULT_DEGREES / MAX_DOWNSTROKE_BELOW_NEUTRAL, UPPER_FLAP_AMPLITUDE_DEFAULT_DEGREES / MAX_UPSTROKE_ABOVE_NEUTRAL, 1.15f, 0.12f, 4.0f, 90.0f, 0},
    // Sport: higher authority and sharper turns for indoor agility.
    {LOWER_FLAP_AMPLITUDE_MAX_DEGREES / MAX_DOWNSTROKE_BELOW_NEUTRAL, UPPER_FLAP_AMPLITUDE_MAX_DEGREES / MAX_UPSTROKE_ABOVE_NEUTRAL, 2.20f, 0.18f, 4.0f, 96.0f, 0},
    // Hover: maximum lift bias with softer steering to avoid rolling out.
    {LOWER_FLAP_AMPLITUDE_DEFAULT_DEGREES / MAX_DOWNSTROKE_BELOW_NEUTRAL, UPPER_FLAP_AMPLITUDE_MAX_DEGREES / MAX_UPSTROKE_ABOVE_NEUTRAL, 0.85f, 0.24f, 4.0f, 84.0f, 0},
};

static const char *profileKey(int index, const char *suffix)
{
    static char key[16];
    snprintf(key, sizeof(key), "p%d_%s", index, suffix);
    return key;
}

static float clampFloat(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static void clampProfile(FlightProfileSettings &settings)
{
    settings.lowerAmplitudeScale = clampFloat(
        settings.lowerAmplitudeScale,
        LOWER_FLAP_AMPLITUDE_MIN_DEGREES / MAX_DOWNSTROKE_BELOW_NEUTRAL,
        LOWER_FLAP_AMPLITUDE_MAX_DEGREES / MAX_DOWNSTROKE_BELOW_NEUTRAL);
    settings.upperAmplitudeScale = clampFloat(
        settings.upperAmplitudeScale,
        UPPER_FLAP_AMPLITUDE_MIN_DEGREES / MAX_UPSTROKE_ABOVE_NEUTRAL,
        UPPER_FLAP_AMPLITUDE_MAX_DEGREES / MAX_UPSTROKE_ABOVE_NEUTRAL);
    settings.steeringGain = clampFloat(settings.steeringGain, 0.0f, 2.50f);
    settings.turnCompensation = clampFloat(settings.turnCompensation, 0.0f, 0.30f);
    settings.frequencyCap = clampFloat(settings.frequencyCap, 2.0f, 6.0f);
    settings.neutralPitch = clampFloat(settings.neutralPitch, 80.0f, 100.0f);
    settings.waveform = settings.waveform == 1 ? 1 : 0;
}

static void saveProfile(int index)
{
    if (index < 0 || index > 2) return;
    prefsProfile.begin(PROFILE_NS, false);
    prefsProfile.putFloat(profileKey(index, "lamp"), profiles[index].lowerAmplitudeScale);
    prefsProfile.putFloat(profileKey(index, "uamp"), profiles[index].upperAmplitudeScale);
    prefsProfile.putFloat(profileKey(index, "sg"), profiles[index].steeringGain);
    prefsProfile.putFloat(profileKey(index, "tc"), profiles[index].turnCompensation);
    prefsProfile.putFloat(profileKey(index, "fc"), profiles[index].frequencyCap);
    prefsProfile.putFloat(profileKey(index, "np"), profiles[index].neutralPitch);
    prefsProfile.putInt(profileKey(index, "wf"), profiles[index].waveform);
    prefsProfile.end();
}

void initFlightProfiles()
{
    prefsProfile.begin(PROFILE_NS, true);
    int storedPresetVersion = prefsProfile.getInt(KEY_PRESET_VERSION, 0);
    int storedMode = prefsProfile.getInt(KEY_MODE, (int)FLIGHT_PROFILE_CRUISE);
    if (storedMode < 0 || storedMode > 2) storedMode = 0;
    currentProfile = (FlightProfileMode)storedMode;

    if (storedPresetVersion > 0)
    {
        for (int i = 0; i < 3; ++i)
        {
            float legacyAmplitudeScale = prefsProfile.getFloat(profileKey(i, "amp"), profiles[i].lowerAmplitudeScale);
            profiles[i].lowerAmplitudeScale = prefsProfile.getFloat(profileKey(i, "lamp"), legacyAmplitudeScale);
            profiles[i].upperAmplitudeScale = prefsProfile.getFloat(profileKey(i, "uamp"), legacyAmplitudeScale);
            profiles[i].steeringGain = prefsProfile.getFloat(profileKey(i, "sg"), profiles[i].steeringGain);
            profiles[i].turnCompensation = prefsProfile.getFloat(profileKey(i, "tc"), profiles[i].turnCompensation);
            profiles[i].frequencyCap = prefsProfile.getFloat(profileKey(i, "fc"), profiles[i].frequencyCap);
            profiles[i].neutralPitch = prefsProfile.getFloat(profileKey(i, "np"), profiles[i].neutralPitch);
            profiles[i].waveform = prefsProfile.getInt(profileKey(i, "wf"), profiles[i].waveform);
            clampProfile(profiles[i]);
        }
    }
    prefsProfile.end();

    if (storedPresetVersion != PROFILE_PRESET_VERSION)
    {
        for (int i = 0; i < 3; ++i)
        {
            clampProfile(profiles[i]);
            saveProfile(i);
        }
        prefsProfile.begin(PROFILE_NS, false);
        prefsProfile.putInt(KEY_PRESET_VERSION, PROFILE_PRESET_VERSION);
        prefsProfile.putInt(KEY_MODE, (int)currentProfile);
        prefsProfile.end();
    }
}

void setFlightProfileMode(FlightProfileMode mode)
{
    if (mode < FLIGHT_PROFILE_CRUISE || mode > FLIGHT_PROFILE_HOVER)
        mode = FLIGHT_PROFILE_CRUISE;
    if (currentProfile == mode)
        return;
    currentProfile = mode;
    prefsProfile.begin(PROFILE_NS, false);
    prefsProfile.putInt(KEY_MODE, (int)currentProfile);
    prefsProfile.end();
}

void setFlightProfileModeFromInt(int mode)
{
    if (mode < 0) mode = 0;
    if (mode > 2) mode = 2;
    setFlightProfileMode((FlightProfileMode)mode);
}

FlightProfileMode getFlightProfileMode()
{
    return currentProfile;
}

const char *getFlightProfileName()
{
    switch (currentProfile)
    {
        case FLIGHT_PROFILE_SPORT: return "Sport";
        case FLIGHT_PROFILE_HOVER: return "Hover";
        default: return "Cruise";
    }
}

float getProfileFrequencyCap()
{
    return profiles[(int)currentProfile].frequencyCap;
}

float getProfileSteeringMultiplier()
{
    switch (currentProfile)
    {
        case FLIGHT_PROFILE_SPORT: return 1.35f;
        case FLIGHT_PROFILE_HOVER: return 0.90f;
        default: return 1.0f;
    }
}

FlightProfileSettings getCurrentFlightProfileSettings()
{
    return profiles[(int)currentProfile];
}

void setCurrentProfileAmplitudeScale(float scale)
{
    setCurrentProfileLowerAmplitudeScale(scale);
    setCurrentProfileUpperAmplitudeScale(scale);
}

void setCurrentProfileLowerAmplitudeScale(float scale)
{
    int index = (int)currentProfile;
    if (fabsf(profiles[index].lowerAmplitudeScale - scale) < 0.001f)
        return;
    profiles[index].lowerAmplitudeScale = scale;
    clampProfile(profiles[index]);
    saveProfile(index);
}

void setCurrentProfileUpperAmplitudeScale(float scale)
{
    int index = (int)currentProfile;
    if (fabsf(profiles[index].upperAmplitudeScale - scale) < 0.001f)
        return;
    profiles[index].upperAmplitudeScale = scale;
    clampProfile(profiles[index]);
    saveProfile(index);
}

void setCurrentProfileSteeringGain(float gain)
{
    int index = (int)currentProfile;
    if (fabsf(profiles[index].steeringGain - gain) < 0.001f)
        return;
    profiles[(int)currentProfile].steeringGain = gain;
    clampProfile(profiles[(int)currentProfile]);
    saveProfile((int)currentProfile);
}

void setCurrentProfileTurnCompensation(float compensation)
{
    int index = (int)currentProfile;
    if (fabsf(profiles[index].turnCompensation - compensation) < 0.001f)
        return;
    profiles[(int)currentProfile].turnCompensation = compensation;
    clampProfile(profiles[(int)currentProfile]);
    saveProfile((int)currentProfile);
}

void setCurrentProfileFrequencyCap(float frequencyCap)
{
    int index = (int)currentProfile;
    frequencyCap = clampFloat(frequencyCap, 2.0f, 6.0f);
    if (fabsf(profiles[index].frequencyCap - frequencyCap) < 0.001f)
        return;
    profiles[index].frequencyCap = frequencyCap;
    clampProfile(profiles[index]);
    saveProfile(index);
}

void setCurrentProfileNeutralPitch(float neutralPitch)
{
    int index = (int)currentProfile;
    if (fabsf(profiles[index].neutralPitch - neutralPitch) < 0.001f)
        return;
    profiles[(int)currentProfile].neutralPitch = neutralPitch;
    clampProfile(profiles[(int)currentProfile]);
    saveProfile((int)currentProfile);
}

void setCurrentProfileWaveform(int waveform)
{
    int index = (int)currentProfile;
    waveform = waveform == 1 ? 1 : 0;
    if (profiles[index].waveform == waveform)
        return;
    profiles[index].waveform = waveform;
    saveProfile((int)currentProfile);
}

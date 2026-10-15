#pragma once

enum FlightProfileMode
{
    FLIGHT_PROFILE_CRUISE = 0,
    FLIGHT_PROFILE_SPORT = 1,
    FLIGHT_PROFILE_HOVER = 2,
};

struct FlightProfileSettings
{
    float lowerAmplitudeScale;
    float upperAmplitudeScale;
    float steeringGain;
    float turnCompensation;
    float frequencyCap;
    float neutralPitch;
    int waveform;
};

void initFlightProfiles();
void setFlightProfileMode(FlightProfileMode mode);
void setFlightProfileModeFromInt(int mode);
FlightProfileMode getFlightProfileMode();
const char *getFlightProfileName();

float getProfileFrequencyCap();
float getProfileSteeringMultiplier();

FlightProfileSettings getCurrentFlightProfileSettings();
void setCurrentProfileAmplitudeScale(float scale);
void setCurrentProfileLowerAmplitudeScale(float scale);
void setCurrentProfileUpperAmplitudeScale(float scale);
void setCurrentProfileSteeringGain(float gain);
void setCurrentProfileTurnCompensation(float compensation);
void setCurrentProfileFrequencyCap(float frequencyCap);
void setCurrentProfileNeutralPitch(float neutralPitch);
void setCurrentProfileWaveform(int waveform);

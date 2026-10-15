#pragma once

void initWingMotion();
void updateWingMotion();

void setFrequency(float freq);
void setAmplitude(float amp);
void setFlapAmplitudeScales(float lowerScale, float upperScale);
float getLowerFlapAmplitudeScale();
float getUpperFlapAmplitudeScale();
void setFrequencyLimit(float freq);
float getFrequencyLimit();
void setFlapWaveform(int waveform);
int getFlapWaveform();
void setServoRefreshRateHz(int hz);
int getServoRefreshRateHz();
void setServoPulseRangeUs(int minUs, int maxUs);
int getServoMinPulseUs();
int getServoMaxPulseUs();
void reattachWingServos();
// Expose for UI handler to set amplitude directly
extern float flapAmplitude;
extern bool amplitudeActive;
void setSteering(float steering);
void setPilotSteering(float steering);
void setIdleLeanSteering(float steering);
void setRollBias(float roll);

// Prevent reapplication of controls for a short time after an explicit release
void lockControlsMs(unsigned long ms);
bool isControlReleaseLocked();

// Immediately stop flapping and center servos (explicit pilot release)
void immediateStopAndCenter();

// Force servos to center and ignore further updates until cleared by new input
void clearForceCenter();

// Query whether a forced center override is active
bool isForceCenterActive();

// Perch mode controls
void startPerch();
void stopPerch();
bool isPerchActive();

// Servo trim (degrees) for mechanical alignment adjustments
void adjustLeftTrim(int delta);
void adjustRightTrim(int delta);
int getLeftTrim();
int getRightTrim();
// Steering trim: fractional degrees [-5..5]
void setSteeringTrim(float v);
float getSteeringTrim();

// Telemetry getters
float getFlapFrequency();
float getFlapAmplitude();
float getPilotSteeringCurrent();
// Servo center/pitch angle (degrees)
void setServoPitchAngle(float deg);
void setServoPitchAngleLive(float deg);
float getServoPitchAngle();
float getCurrentNeutralAngle();
void setSteeringDifferentialGain(float gain);
float getSteeringDifferentialGain();
void setTurnCompensation(float compensation);
float getTurnCompensation();

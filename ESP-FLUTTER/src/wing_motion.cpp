#include "wing_motion.h"
#include "config.h"

#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>
#include "wifi_control.h"
#include "hover_bias.h"
#include "hardware_config.h"
#include <Preferences.h>

#if ENABLE_STEERING_DEBUG
#define DBG(...) Serial.printf(__VA_ARGS__)
#else
#define DBG(...)
#endif


Servo leftServo;
Servo rightServo;

float flapFrequency = 0.0f;
float flapAmplitude = 0.0f;
static float flapFrequencyLimit = DEFAULT_MAX_FLAP_FREQUENCY;
static int flapWaveform = 0; // 0=triangle, 1=sine

// Steering state: targets are set by pilot control code; currents are smoothed toward targets
float targetPilotSteering = 0.0f;
float currentPilotSteering = 0.0f;

unsigned long previousMicros = 0;

// Amplitude resume state: only consider amplitude 'active' after it stays above threshold for a short time
bool amplitudeActive = false;
unsigned long amplitudeResumeStart = 0;
// Startup ramp: limit initial slew for a short window after resume
unsigned long startupRampUntil = 0;
// Lock controls after explicit release to prevent late packet reapplication
unsigned long releaseLockUntil = 0;

float phase = 0.0f;

bool wingsIdle = true;
static bool idleLeanHold = false;
static float idleLeanLeftAngle = 0.0f;
static float idleLeanRightAngle = 0.0f;

// When true, updateWingMotion will force both servos to center and skip smoothing
bool forceCenterActive = false;
// Perch decorative motion active flag
bool perchActive = false;

// Servo trim values (degrees), persisted in Preferences
static int leftTrim = 0;
static int rightTrim = 0;
static Preferences prefsTrim;
static const char *TRIM_NS = "trim";
static const char *KEY_LTRIM = "ltrim";
static const char *KEY_RTRIM = "rtrim";
static const char *KEY_STRIM = "strim";
static float steeringTrim = 0.0f;
static float centerAngle = 90.0f;
static const char *KEY_PITCH = "pitch";
static const char *KEY_STEER_DIFF_GAIN = "sdgain";
static const char *KEY_TURN_COMP = "turncomp";
static const char *KEY_SERVO_HZ = "servoHz";
static const char *KEY_SERVO_MIN_US = "smin";
static const char *KEY_SERVO_MAX_US = "smax";
static float steeringDifferentialGain = STEERING_DIFF_GAIN_DEFAULT;
static float turnLiftBoostFraction = TURN_LIFT_BOOST_MAX_FRACTION;
static float rollBiasDegrees = 0.0f;
static int servoRefreshHz = SERVO_REFRESH_HZ;
static int servoMinUs = SERVO_MIN_US;
static int servoMaxUs = SERVO_MAX_US;
static float lowerFlapAmplitudeScale = LOWER_FLAP_AMPLITUDE_DEFAULT_DEGREES / MAX_DOWNSTROKE_BELOW_NEUTRAL;
static float upperFlapAmplitudeScale = UPPER_FLAP_AMPLITUDE_DEFAULT_DEGREES / MAX_UPSTROKE_ABOVE_NEUTRAL;

static float leftNeutralAngle()
{
    return 180.0f - getCurrentNeutralAngle();
}

static float rightNeutralAngle()
{
    return getCurrentNeutralAngle();
}

static float clampServoAngle(float angle)
{
    return constrain(angle, (float)SERVO_SAFE_MIN_ANGLE, (float)SERVO_SAFE_MAX_ANGLE);
}

static int clampServoRefreshHz(int hz)
{
    if (hz <= 75) return 50;
    if (hz <= 216) return 100;
    return 333;
}

static int clampServoMinUs(int us)
{
    us = constrain(us, SERVO_MIN_US_SLIDER_MIN, SERVO_MIN_US_SLIDER_MAX);
    return ((us + 50) / 100) * 100;
}

static int clampServoMaxUs(int us)
{
    us = constrain(us, SERVO_MAX_US_SLIDER_MIN, SERVO_MAX_US_SLIDER_MAX);
    return ((us + 50) / 100) * 100;
}

static void normalizeServoPulseRange()
{
    servoMinUs = clampServoMinUs(servoMinUs);
    servoMaxUs = clampServoMaxUs(servoMaxUs);
    if (servoMaxUs <= servoMinUs + 200)
        servoMaxUs = clampServoMaxUs(servoMinUs + 200);
}

static void saveServoOutputSettings()
{
    prefsTrim.begin(TRIM_NS, false);
    prefsTrim.putInt(KEY_SERVO_HZ, servoRefreshHz);
    prefsTrim.putInt(KEY_SERVO_MIN_US, servoMinUs);
    prefsTrim.putInt(KEY_SERVO_MAX_US, servoMaxUs);
    prefsTrim.end();
}

static float clampLowerAmplitudeScale(float scale)
{
    return constrain(scale,
        LOWER_FLAP_AMPLITUDE_MIN_DEGREES / MAX_DOWNSTROKE_BELOW_NEUTRAL,
        LOWER_FLAP_AMPLITUDE_MAX_DEGREES / MAX_DOWNSTROKE_BELOW_NEUTRAL);
}

static float clampUpperAmplitudeScale(float scale)
{
    return constrain(scale,
        UPPER_FLAP_AMPLITUDE_MIN_DEGREES / MAX_UPSTROKE_ABOVE_NEUTRAL,
        UPPER_FLAP_AMPLITUDE_MAX_DEGREES / MAX_UPSTROKE_ABOVE_NEUTRAL);
}

static int servoAngleToPulseUs(float angle)
{
    angle = clampServoAngle(angle);
    const float spanUs = (float)(servoMaxUs - servoMinUs);
    return (int)lroundf((float)servoMinUs + ((angle / 180.0f) * spanUs));
}

static void writeLeftServoAngle(float angle)
{
    leftServo.writeMicroseconds(servoAngleToPulseUs(angle));
}

static void writeRightServoAngle(float angle)
{
    rightServo.writeMicroseconds(servoAngleToPulseUs(angle));
}

void setServoPitchAngle(float deg)
{
    if (deg < 80.0f) deg = 80.0f;
    if (deg > 100.0f) deg = 100.0f;
    if (fabsf(centerAngle - deg) < 0.001f)
        return;
    centerAngle = deg;
    prefsTrim.begin(TRIM_NS, false);
    prefsTrim.putFloat(KEY_PITCH, centerAngle);
    prefsTrim.end();
    // The servos are mirrored, so their numeric neutral angles must move in
    // opposite directions for both wings to move together physically.
    float newL = leftNeutralAngle() + leftTrim + rollBiasDegrees;
    float newR = rightNeutralAngle() + rightTrim + rollBiasDegrees;
    newL = constrain(newL, SERVO_SAFE_MIN_ANGLE, SERVO_SAFE_MAX_ANGLE);
    newR = constrain(newR, SERVO_SAFE_MIN_ANGLE, SERVO_SAFE_MAX_ANGLE);
    float prevL = leftServo.read();
    float prevR = rightServo.read();
    if (fabs(newL - prevL) > 0.4f) { writeLeftServoAngle(newL); }
    if (fabs(newR - prevR) > 0.4f) { writeRightServoAngle(newR); }
}

void setServoPitchAngleLive(float deg)
{
    if (deg < 80.0f) deg = 80.0f;
    if (deg > 100.0f) deg = 100.0f;
    centerAngle = deg;
}

float getServoPitchAngle()
{
    return centerAngle;
}

float getCurrentNeutralAngle()
{
    return getHoverBiasNeutralAngle(centerAngle);
}

void setSteeringDifferentialGain(float gain)
{
    if (gain < 0.0f) gain = 0.0f;
    if (gain > STEERING_DIFF_GAIN_MAX) gain = STEERING_DIFF_GAIN_MAX;
    if (fabsf(steeringDifferentialGain - gain) < 0.001f)
        return;
    steeringDifferentialGain = gain;
    prefsTrim.begin(TRIM_NS, false);
    prefsTrim.putFloat(KEY_STEER_DIFF_GAIN, steeringDifferentialGain);
    prefsTrim.end();
}

float getSteeringDifferentialGain()
{
    return steeringDifferentialGain;
}

void setTurnCompensation(float compensation)
{
    if (compensation < 0.0f) compensation = 0.0f;
    if (compensation > TURN_LIFT_BOOST_LIMIT_FRACTION) compensation = TURN_LIFT_BOOST_LIMIT_FRACTION;
    if (fabsf(turnLiftBoostFraction - compensation) < 0.001f)
        return;
    turnLiftBoostFraction = compensation;
    prefsTrim.begin(TRIM_NS, false);
    prefsTrim.putFloat(KEY_TURN_COMP, turnLiftBoostFraction);
    prefsTrim.end();
}

float getTurnCompensation()
{
    return turnLiftBoostFraction;
}

void setRollBias(float roll)
{
    rollBiasDegrees = constrain(roll, -MAX_ROLL_BIAS_DEGREES, MAX_ROLL_BIAS_DEGREES);
}

// Maximum allowed lean steering when wings are idle (degrees)
// Maximum allowed lean steering when wings are idle (degrees)
static const float LEAN_MAX_STEERING = 15.0f;
static const float IDLE_LEAN_GAIN = 3.0f;

// Last servo angles written (used to suppress micro-updates that cause jitter)
static float lastLeftAngle = 0.0f;
static float lastRightAngle = 0.0f;
static const float ACTIVE_FLAP_WRITE_DEADBAND = 0.12f;
static const float STARTUP_NEUTRALIZE_OFFSET_DEGREES = 20.0f;
static const unsigned long STARTUP_NEUTRALIZE_MS = 1000;
static const unsigned long STARTUP_NEUTRALIZE_STEP_MS = 20;

// Forward declaration for perch waveform (defined later)
float perchWave(float phase);
static void smoothStartupNeutralize(float targetLeft, float targetRight);

float butterflyWave(float phase, float topBlend)
{
    float wrapped = fmod(phase, 1.0f);

    // Mostly linear 50/50 stroke. Keep the bottom reversal sharp, but use an
    // adjustable smooth blend around the fully retracted top.
    topBlend = constrain(topBlend, 0.0f, 0.20f);
    if (topBlend > 0.0001f && wrapped < topBlend)
    {
        float t = wrapped / topBlend;
        return -1.0f + (4.0f * topBlend * (-t * t * t + 2.0f * t * t));
    }
    if (topBlend > 0.0001f && wrapped > 1.0f - topBlend)
    {
        float t = (1.0f - wrapped) / topBlend;
        return -1.0f + (4.0f * topBlend * (-t * t * t + 2.0f * t * t));
    }

    if (wrapped < 0.50f)
        return -1.0f + (4.0f * wrapped);

    return 3.0f - (4.0f * wrapped);
}

float directTriangleWave(float phase)
{
    float wrapped = fmod(phase, 1.0f);
    if (wrapped < 0.0f) wrapped += 1.0f;

    if (wrapped < 0.50f)
        return -1.0f + (4.0f * wrapped);

    return 3.0f - (4.0f * wrapped);
}

float sineFlapWave(float phase)
{
    float wrapped = fmod(phase, 1.0f);
    if (wrapped < 0.0f) wrapped += 1.0f;
    return sinf((wrapped * TWO_PI) - (PI * 0.5f));
}

// Trim accessors
void saveTrims()
{
    prefsTrim.begin(TRIM_NS, false);
    prefsTrim.putInt(KEY_LTRIM, leftTrim);
    prefsTrim.putInt(KEY_RTRIM, rightTrim);
    prefsTrim.putFloat(KEY_STRIM, steeringTrim);
    prefsTrim.putFloat(KEY_PITCH, centerAngle);
    prefsTrim.end();
    DBG("Saved trims L=%d R=%d\n", leftTrim, rightTrim);
    // notify any connected UI clients about the new trims (if WS available)
    broadcastTrimUpdate(leftTrim, rightTrim, steeringTrim);
}

void adjustLeftTrim(int delta)
{
    leftTrim += delta;
    if (leftTrim < -30) leftTrim = -30;
    if (leftTrim > 30) leftTrim = 30;
    saveTrims();
    // apply immediate movement so user sees effect
    float current = lastLeftAngle;
    writeLeftServoAngle(current + delta);
}

void adjustRightTrim(int delta)
{
    rightTrim += delta;
    if (rightTrim < -30) rightTrim = -30;
    if (rightTrim > 30) rightTrim = 30;
    saveTrims();
    // apply immediate movement so user sees effect
    float current = lastRightAngle;
    writeRightServoAngle(current + delta);
}

int getLeftTrim() { return leftTrim; }
int getRightTrim() { return rightTrim; }

void setSteeringTrim(float v)
{
    if (v < -5.0f) v = -5.0f;
    if (v > 5.0f) v = 5.0f;
    steeringTrim = v;
    // persist
    prefsTrim.begin(TRIM_NS, false);
    prefsTrim.putFloat(KEY_STRIM, steeringTrim);
    prefsTrim.end();
    // notify UI
    broadcastTrimUpdate(leftTrim, rightTrim, steeringTrim);
}

float getSteeringTrim() { return steeringTrim; }

// ===== Wing Motion Init =====
void initWingMotion()
{
initHoverBias();

    prefsTrim.begin(TRIM_NS, true);
    servoRefreshHz = prefsTrim.getInt(KEY_SERVO_HZ, SERVO_REFRESH_HZ);
    servoMinUs = prefsTrim.getInt(KEY_SERVO_MIN_US, SERVO_MIN_US);
    servoMaxUs = prefsTrim.getInt(KEY_SERVO_MAX_US, SERVO_MAX_US);
    if (prefsTrim.isKey(KEY_LTRIM)) leftTrim = prefsTrim.getInt(KEY_LTRIM, 0);
    if (prefsTrim.isKey(KEY_RTRIM)) rightTrim = prefsTrim.getInt(KEY_RTRIM, 0);
    if (prefsTrim.isKey(KEY_STRIM)) steeringTrim = prefsTrim.getFloat(KEY_STRIM, 0.0f);
    if (prefsTrim.isKey(KEY_PITCH)) centerAngle = prefsTrim.getFloat(KEY_PITCH, 90.0f);
    if (prefsTrim.isKey(KEY_STEER_DIFF_GAIN)) steeringDifferentialGain = prefsTrim.getFloat(KEY_STEER_DIFF_GAIN, STEERING_DIFF_GAIN_DEFAULT);
    if (prefsTrim.isKey(KEY_TURN_COMP)) turnLiftBoostFraction = prefsTrim.getFloat(KEY_TURN_COMP, TURN_LIFT_BOOST_MAX_FRACTION);
    prefsTrim.end();
    if (servoMinUs == 600 && servoMaxUs == 2400)
    {
        servoMinUs = SERVO_MIN_US;
        servoMaxUs = SERVO_MAX_US;
        saveServoOutputSettings();
    }
    servoRefreshHz = clampServoRefreshHz(servoRefreshHz);
    normalizeServoPulseRange();
    if (steeringDifferentialGain < 0.0f) steeringDifferentialGain = 0.0f;
    if (steeringDifferentialGain > STEERING_DIFF_GAIN_MAX) steeringDifferentialGain = STEERING_DIFF_GAIN_MAX;
    if (turnLiftBoostFraction < 0.0f) turnLiftBoostFraction = 0.0f;
    if (turnLiftBoostFraction > TURN_LIFT_BOOST_LIMIT_FRACTION) turnLiftBoostFraction = TURN_LIFT_BOOST_LIMIT_FRACTION;
    if (centerAngle < 80.0f) centerAngle = 80.0f;
    if (centerAngle > 100.0f) centerAngle = 100.0f;

    // Apply trims to initial servo positions before attach, so the first PWM
    // pulse is neutral instead of a brief endpoint command.
    float initLeft = leftNeutralAngle() + leftTrim + rollBiasDegrees;
    float initRight = rightNeutralAngle() + rightTrim + rollBiasDegrees;
    initLeft = constrain(initLeft, SERVO_SAFE_MIN_ANGLE, SERVO_SAFE_MAX_ANGLE);
    initRight = constrain(initRight, SERVO_SAFE_MIN_ANGLE, SERVO_SAFE_MAX_ANGLE);
    smoothStartupNeutralize(initLeft, initRight);

    // Initialize timing reference to avoid a large first delta
    previousMicros = micros();
    DBG("Loaded trims L=%d R=%d -> initLeft=%.1f initRight=%.1f\n", leftTrim, rightTrim, initLeft, initRight);
    DBG("Loaded steeringTrim=%.2f\n", steeringTrim);

}

// ===== Motion Update =====
void updateWingMotion()
{
unsigned long now = micros();

float deltaTime =
    (now - previousMicros) / 1000000.0f;

previousMicros = now;

    // If a forced center is active, short-circuit and keep servos at center
    if (forceCenterActive)
    {
        float tgtL = leftNeutralAngle() + leftTrim + rollBiasDegrees;
        float tgtR = rightNeutralAngle() + rightTrim + rollBiasDegrees;
        writeLeftServoAngle(tgtL);
        writeRightServoAngle(tgtR);
        lastLeftAngle = tgtL;
        lastRightAngle = tgtR;
        // keep other state zeroed to avoid any follow-up smoothing
        flapFrequency = 0.0f;
        flapAmplitude = 0.0f;
        targetPilotSteering = 0.0f;
        currentPilotSteering = 0.0f;
        wingsIdle = true;
        return;
    }

float phaseStep = flapFrequency * deltaTime;
phase += phaseStep;
    bool freqIdle = (flapFrequency <= 0.05f);
    if (freqIdle)
    {
        if (!wingsIdle)
        {
            // Transitioning to idle: stop flapping but do not immediately zero
            // pilot steering targets, so zero-throttle lean remains available.
            phase = 0.0f;

            DBG("WINGS IDLE: transitioning tP=%.2f cP=%.2f\n", targetPilotSteering, currentPilotSteering);

            wingsIdle = true;
        }
    }
    else
    {
        wingsIdle = false;
    }

if (phase >= 1.0f)
{
    phase -= 1.0f;
}

// Generate flapping wave (special-case perch mode)
float wave;
if (isPerchActive())
    wave = perchWave(phase);
else
{
    wave = (flapWaveform == 1) ? sineFlapWave(phase) : directTriangleWave(phase);
}

// Convert the amplitude command to a normalized stroke scale. Physical travel
// is mapped after steering so pitch/trim/steering headroom is respected.
float strokeScale = 1.0f;
float flap = wave * flapAmplitude;
    if (!isPerchActive())
    {
        strokeScale = flapAmplitude / TEST_FLAP_AMPLITUDE_DEGREES;
        strokeScale = constrain(strokeScale, 0.0f, 1.0f);
    }
    // If frequency is effectively zero, force no flap so we only apply steering smoothing
    if (flapFrequency <= 0.05f)
        flap = 0.0f;

#if DIRECT_FLAP_MODE
amplitudeActive = flapAmplitude > AMPLITUDE_IDLE_THRESHOLD;
amplitudeResumeStart = 0;
startupRampUntil = 0;
#else
// Update amplitudeActive state with simple hysteresis debounce
if (flapAmplitude <= AMPLITUDE_IDLE_THRESHOLD)
{
    // amplitude low: mark inactive
    amplitudeActive = false;
    amplitudeResumeStart = 0;
    // cancel any startup ramp pending
    startupRampUntil = 0;
}
else
{
    // amplitude above idle threshold: only begin resume when above the resume threshold
    if (flapAmplitude >= AMPLITUDE_RESUME_THRESHOLD)
    {
        if (!amplitudeActive)
        {
            if (amplitudeResumeStart == 0)
                amplitudeResumeStart = millis();
            else if ((millis() - amplitudeResumeStart) >= AMPLITUDE_RESUME_MS)
            {
                amplitudeActive = true;
                // exiting idle, allow wing updates
                wingsIdle = false;
                DBG("Amplitude sustained: resuming controls (amp=%.2f)\n", flapAmplitude);
                // apply startup ramp to limit immediate steering slew
                startupRampUntil = millis() + STARTUP_RAMP_MS;
                DBG("Startup ramp active until %lu\n", startupRampUntil);
            }
        }
        else
        {
            // refresh resume timer while active
            amplitudeResumeStart = millis();
        }
    }
    else
    {
        // amplitude in the hysteresis gap: reset resume timer
        amplitudeResumeStart = 0;
    }
}
#endif

float leftFlap = flap;
float rightFlap = flap;
float flapCenterOffset = 0.0f;
float directFinalSteering = 0.0f;

// ===== Differential Amplitude Steering =====
// Smooth pilot steering toward its target to avoid jitter and abrupt changes
{
    // If wings are idle or amplitude inactive, apply pilot steering immediately (lean mode)
    if (wingsIdle || !amplitudeActive)
    {
        currentPilotSteering = targetPilotSteering;
        // Compute final steering directly
        float finalSteering = currentPilotSteering;
        finalSteering = constrain(finalSteering, -MAX_FINAL_STEERING, MAX_FINAL_STEERING);

        // Apply a lightly smoothed idle lean around the mirrored neutral angles.
        float leanOffset = constrain(finalSteering, -LEAN_MAX_STEERING, LEAN_MAX_STEERING);
        float newL = leftNeutralAngle() + leftTrim + rollBiasDegrees - leanOffset;
        float newR = rightNeutralAngle() + rightTrim + rollBiasDegrees - leanOffset;
        // Constrain angles to safe servo margins
        newL = constrain(newL, SERVO_SAFE_MIN_ANGLE, SERVO_SAFE_MAX_ANGLE);
        newR = constrain(newR, SERVO_SAFE_MIN_ANGLE, SERVO_SAFE_MAX_ANGLE);
        if (!idleLeanHold)
        {
            idleLeanLeftAngle = lastLeftAngle;
            idleLeanRightAngle = lastRightAngle;
            idleLeanHold = true;
        }
        const float idleLeanTau = 0.045f;
        float idleAlpha = deltaTime / (idleLeanTau + deltaTime);
        idleAlpha = constrain(idleAlpha, 0.15f, 0.75f);
        idleLeanLeftAngle += idleAlpha * (newL - idleLeanLeftAngle);
        idleLeanRightAngle += idleAlpha * (newR - idleLeanRightAngle);
        newL = idleLeanLeftAngle;
        newR = idleLeanRightAngle;
        if (fabs(newL - lastLeftAngle) > 0.4f) { writeLeftServoAngle(newL); lastLeftAngle = newL; }
        if (fabs(newR - lastRightAngle) > 0.4f) { writeRightServoAngle(newR); lastRightAngle = newR; }
        return;
    }
    else
    {
        idleLeanHold = false;
#if DIRECT_FLAP_MODE
        currentPilotSteering = targetPilotSteering;

        if (fabsf(currentPilotSteering) < STEERING_DEADBAND)
            currentPilotSteering = 0.0f;

        directFinalSteering = currentPilotSteering + steeringTrim;
        directFinalSteering = constrain(directFinalSteering, -MAX_FINAL_STEERING, MAX_FINAL_STEERING);
#else
        // Maximum allowed change this frame (degrees) based on configured ramp rate
        float rampScale = 1.0f;
        if (millis() < startupRampUntil)
            rampScale = STARTUP_RAMP_SCALE;

        float maxDelta = STEERING_RAMP_RATE * rampScale * deltaTime;

        // Approach function
        auto approach = [](float current, float target, float maxd)->float
        {
            float d = target - current;
            if (d > maxd)
                return current + maxd;
            if (d < -maxd)
                return current - maxd;
            return target;
        };

        float prevPilot = currentPilotSteering;

        currentPilotSteering = approach(currentPilotSteering, targetPilotSteering, maxDelta);

        // Apply deadband to suppress tiny jitter
        if (fabsf(currentPilotSteering) < STEERING_DEADBAND)
            currentPilotSteering = 0.0f;

        // Compute mixed steering and clamp to final safety limit
        float finalSteering = currentPilotSteering;
        finalSteering = constrain(finalSteering, -MAX_FINAL_STEERING, MAX_FINAL_STEERING);

        // Debug: print when a significant change occurred or when nearing/entering idle
        if (ENABLE_STEERING_DEBUG)
        {
            if (fabsf(currentPilotSteering - prevPilot) > 0.5f)
            {
                DBG("STEER DT=%.3f tP=%.2f cP=%.2f final=%.2f\n", deltaTime, targetPilotSteering, currentPilotSteering, finalSteering);
            }
        }

        // Preserve the existing differential steering geometry as a shift of
        // both numeric servo centers before available flap travel is computed.
        flapCenterOffset = -(finalSteering - steeringTrim);
#endif
    }

    // Perch mode: disable steering mixing and prevent flaps from going below neutral
    if (isPerchActive())
    {
        currentPilotSteering = 0.0f;
        if (leftFlap < 0.0f) leftFlap = 0.0f;
        if (rightFlap < 0.0f) rightFlap = 0.0f;

        // If perching, drive both servos from neutral up to the top safe angle
        float leftCenter = leftNeutralAngle();
        float rightCenter = rightNeutralAngle();
        const float perchRetractionReductionDegrees = 20.0f;
        float leftDeltaUp = fmaxf(0.0f, leftCenter - (float)SERVO_SAFE_MIN_ANGLE - perchRetractionReductionDegrees);
        float rightDeltaUp = fmaxf(0.0f, (float)SERVO_SAFE_MAX_ANGLE - rightCenter - perchRetractionReductionDegrees);
        float leftAngle = leftCenter - (wave * leftDeltaUp);
        // invert right servo direction so both wings move synchronized physically
        float rightAngle = rightCenter + (wave * rightDeltaUp);

        // Constrain angles to safe servo margins to prevent mechanical saturation
        leftAngle = constrain(leftAngle, SERVO_SAFE_MIN_ANGLE, SERVO_SAFE_MAX_ANGLE);
        rightAngle = constrain(rightAngle, SERVO_SAFE_MIN_ANGLE, SERVO_SAFE_MAX_ANGLE);

        {
            float newL = leftAngle + leftTrim;
            float newR = rightAngle + rightTrim;
            if (fabs(newL - lastLeftAngle) > ACTIVE_FLAP_WRITE_DEADBAND) { writeLeftServoAngle(newL); lastLeftAngle = newL; }
            if (fabs(newR - lastRightAngle) > ACTIVE_FLAP_WRITE_DEADBAND) { writeRightServoAngle(newR); lastRightAngle = newR; }
        }

        return;
    }

#if DIRECT_FLAP_MODE
    // Direct mixer: keep the base flap, redistribute a bounded amount of
    // stroke between wings for steering, then constrain only at final output.
    float leftCenter = leftNeutralAngle() + leftTrim + rollBiasDegrees;
    float rightCenter = rightNeutralAngle() + rightTrim + rollBiasDegrees;
    float sideScale = (wave < 0.0f) ? upperFlapAmplitudeScale : lowerFlapAmplitudeScale;
    float baseTravel = ((wave < 0.0f) ? MAX_UPSTROKE_ABOVE_NEUTRAL : MAX_DOWNSTROKE_BELOW_NEUTRAL) * sideScale;
    float steeringNorm = constrain(directFinalSteering / MAX_FINAL_STEERING, -1.0f, 1.0f);
    float turnLiftBoost = 1.0f + (fabsf(steeringNorm) * turnLiftBoostFraction);
    float baseFlapAmplitude = baseTravel * strokeScale * turnLiftBoost;
    float maxWingDelta = baseFlapAmplitude * STEERING_DIFF_MAX_FRACTION * 0.5f;
    float wingDelta = steeringNorm * steeringDifferentialGain * maxWingDelta;
    wingDelta = constrain(wingDelta, -maxWingDelta, maxWingDelta);

    float leftFlapAmplitude = baseFlapAmplitude + wingDelta;
    float rightFlapAmplitude = baseFlapAmplitude - wingDelta;
    leftFlapAmplitude = fmaxf(0.0f, leftFlapAmplitude);
    rightFlapAmplitude = fmaxf(0.0f, rightFlapAmplitude);

    float leftFlapTravel = wave * leftFlapAmplitude;
    float rightFlapTravel = wave * rightFlapAmplitude;
    float leftAngle = leftCenter + leftFlapTravel;
    float rightAngle = rightCenter - rightFlapTravel;

    leftAngle = constrain(leftAngle, SERVO_SAFE_MIN_ANGLE, SERVO_SAFE_MAX_ANGLE);
    rightAngle = constrain(rightAngle, SERVO_SAFE_MIN_ANGLE, SERVO_SAFE_MAX_ANGLE);

    {
        float newL = leftAngle;
        float newR = rightAngle;
        if (fabs(newL - lastLeftAngle) > ACTIVE_FLAP_WRITE_DEADBAND) { writeLeftServoAngle(newL); lastLeftAngle = newL; }
        if (fabs(newR - lastRightAngle) > ACTIVE_FLAP_WRITE_DEADBAND) { writeRightServoAngle(newR); lastRightAngle = newR; }
    }

    return;
#else
    // Include pitch, trims and steering before calculating available travel.
    // This maps each waveform peak exactly to the reachable endpoint instead
    // of commanding beyond it and creating a clipped pause.
    float leftCenter = constrain(
        leftNeutralAngle() + leftTrim + rollBiasDegrees + flapCenterOffset,
        (float)SERVO_SAFE_MIN_ANGLE,
        (float)SERVO_SAFE_MAX_ANGLE);
    float rightCenter = constrain(
        rightNeutralAngle() + rightTrim + rollBiasDegrees + flapCenterOffset,
        (float)SERVO_SAFE_MIN_ANGLE,
        (float)SERVO_SAFE_MAX_ANGLE);

    float availableTravel;
    float configuredTravel;
    if (wave < 0.0f)
    {
        availableTravel = fminf(
            leftCenter - SERVO_SAFE_MIN_ANGLE,
            SERVO_SAFE_MAX_ANGLE - rightCenter);
        configuredTravel = MAX_UPSTROKE_ABOVE_NEUTRAL * upperFlapAmplitudeScale;
    }
    else
    {
        availableTravel = fminf(
            SERVO_SAFE_MAX_ANGLE - leftCenter,
            rightCenter - SERVO_SAFE_MIN_ANGLE);
        configuredTravel = MAX_DOWNSTROKE_BELOW_NEUTRAL * lowerFlapAmplitudeScale;
    }

    float peakTravel = fmaxf(0.0f, fminf(configuredTravel, availableTravel));
    flap = wave * peakTravel * strokeScale;
    float leftAngle = leftCenter + flap;
    float rightAngle = rightCenter - flap;

    // Constrain angles to safe servo margins to prevent mechanical saturation
    leftAngle = constrain(leftAngle, SERVO_SAFE_MIN_ANGLE, SERVO_SAFE_MAX_ANGLE);
    rightAngle = constrain(rightAngle, SERVO_SAFE_MIN_ANGLE, SERVO_SAFE_MAX_ANGLE);

    {
        float newL = leftAngle;
        float newR = rightAngle;
        if (fabs(newL - lastLeftAngle) > ACTIVE_FLAP_WRITE_DEADBAND) { writeLeftServoAngle(newL); lastLeftAngle = newL; }
        if (fabs(newR - lastRightAngle) > ACTIVE_FLAP_WRITE_DEADBAND) { writeRightServoAngle(newR); lastRightAngle = newR; }
    }

    return;
#endif
}

}


// ===== Frequency Setter =====
void setFrequency(float freq)
{
    // Enforce safe frequency limits
    float f = freq;
    if (f < MIN_FLAP_FREQUENCY) f = MIN_FLAP_FREQUENCY;
    if (f > flapFrequencyLimit) f = flapFrequencyLimit;

    // Start each new flap run from neutral, moving into the downstroke.
    // Phase 0.25 is the neutral crossing on the top-to-bottom half-cycle.
    if (flapFrequency <= 0.05f && f > 0.05f)
    {
        phase = 0.75f;
    }

    flapFrequency = f;
}

void setFrequencyLimit(float freq)
{
    flapFrequencyLimit = constrain(freq, MIN_ACTIVE_FLAP_FREQUENCY, ABSOLUTE_MAX_FLAP_FREQUENCY);
    if (flapFrequency > flapFrequencyLimit)
        flapFrequency = flapFrequencyLimit;
}

float getFrequencyLimit()
{
    return flapFrequencyLimit;
}

void setFlapWaveform(int waveform)
{
    flapWaveform = waveform == 1 ? 1 : 0;
}

int getFlapWaveform()
{
    return flapWaveform;
}

void setServoRefreshRateHz(int hz)
{
    servoRefreshHz = clampServoRefreshHz(hz);
    leftServo.setPeriodHertz(servoRefreshHz);
    rightServo.setPeriodHertz(servoRefreshHz);
    saveServoOutputSettings();
}

int getServoRefreshRateHz()
{
    return servoRefreshHz;
}

void setServoPulseRangeUs(int minUs, int maxUs)
{
    servoMinUs = minUs;
    servoMaxUs = maxUs;
    normalizeServoPulseRange();
    saveServoOutputSettings();
}

int getServoMinPulseUs()
{
    return servoMinUs;
}

int getServoMaxPulseUs()
{
    return servoMaxUs;
}

void reattachWingServos()
{
    leftServo.detach();
    rightServo.detach();
    leftServo.setPeriodHertz(servoRefreshHz);
    rightServo.setPeriodHertz(servoRefreshHz);
    leftServo.attach(
        getLeftServoPin(),
        SERVO_ABSOLUTE_MIN_US,
        SERVO_ABSOLUTE_MAX_US);
    rightServo.attach(
        getRightServoPin(),
        SERVO_ABSOLUTE_MIN_US,
        SERVO_ABSOLUTE_MAX_US);
    writeLeftServoAngle(lastLeftAngle);
    writeRightServoAngle(lastRightAngle);
}

static void smoothStartupNeutralize(float targetLeft, float targetRight)
{
    float startLeft = constrain(
        targetLeft - STARTUP_NEUTRALIZE_OFFSET_DEGREES,
        (float)SERVO_SAFE_MIN_ANGLE,
        (float)SERVO_SAFE_MAX_ANGLE);
    float startRight = constrain(
        targetRight + STARTUP_NEUTRALIZE_OFFSET_DEGREES,
        (float)SERVO_SAFE_MIN_ANGLE,
        (float)SERVO_SAFE_MAX_ANGLE);

    lastLeftAngle = startLeft;
    lastRightAngle = startRight;
    reattachWingServos();

    const unsigned long steps = STARTUP_NEUTRALIZE_MS / STARTUP_NEUTRALIZE_STEP_MS;
    for (unsigned long i = 1; i <= steps; ++i)
    {
        float t = (float)i / (float)steps;
        t = t * t * (3.0f - 2.0f * t);
        float left = startLeft + ((targetLeft - startLeft) * t);
        float right = startRight + ((targetRight - startRight) * t);
        writeLeftServoAngle(left);
        writeRightServoAngle(right);
        lastLeftAngle = left;
        lastRightAngle = right;
        delay(STARTUP_NEUTRALIZE_STEP_MS);
    }

    lastLeftAngle = targetLeft;
    lastRightAngle = targetRight;
    writeLeftServoAngle(lastLeftAngle);
    writeRightServoAngle(lastRightAngle);
}

// ===== Amplitude Setter =====
void setAmplitude(float amplitude)
{
    // Light exponential smoothing to prevent brief spikes/zeros toggling idle state
    // Use higher alpha so amplitude decreases faster when pilot releases.
    const float alpha = 0.6f;
    // Clamp input amplitude to configured bounds first
    if (amplitude < MIN_FLAP_AMPLITUDE) amplitude = MIN_FLAP_AMPLITUDE;
    if (amplitude > MAX_FLAP_AMPLITUDE) amplitude = MAX_FLAP_AMPLITUDE;

#if DIRECT_FLAP_MODE
    flapAmplitude = amplitude;
#else
    flapAmplitude = flapAmplitude * (1.0f - alpha) + amplitude * alpha;
#endif

    // Ensure smoothed amplitude stays within limits
    if (flapAmplitude < MIN_FLAP_AMPLITUDE) flapAmplitude = MIN_FLAP_AMPLITUDE;
    if (flapAmplitude > MAX_FLAP_AMPLITUDE) flapAmplitude = MAX_FLAP_AMPLITUDE;

    // If amplitude falls to zero from ordinary control packets, stop flapping
    // and allow the idle lean branch to decide whether to center or lean.
    if (flapAmplitude <= AMPLITUDE_IDLE_THRESHOLD)
    {
        flapAmplitude = 0.0f;
        amplitudeActive = false;
        wingsIdle = true;
        startupRampUntil = 0;
    }
}

void setFlapAmplitudeScales(float lowerScale, float upperScale)
{
    lowerFlapAmplitudeScale = clampLowerAmplitudeScale(lowerScale);
    upperFlapAmplitudeScale = clampUpperAmplitudeScale(upperScale);
}

float getLowerFlapAmplitudeScale()
{
    return lowerFlapAmplitudeScale;
}

float getUpperFlapAmplitudeScale()
{
    return upperFlapAmplitudeScale;
}

// ===== Steering Setter =====
void setPilotSteering(float steering)
{
    // Allow leaning when wings are idle or amplitude inactive: accept pilot steering
    // even when the flap waveform is stopped.
    if (wingsIdle || !amplitudeActive)
    {
        float lean = constrain(steering * IDLE_LEAN_GAIN, -LEAN_MAX_STEERING, LEAN_MAX_STEERING);
        targetPilotSteering = lean;
        DBG("SET pilot target (lean) = %.2f (idle)\n", targetPilotSteering);
        return;
    }

    // Avoid applying stale pilot commands immediately after resume
    if (millis() - getLastControlPacket() > CONTROL_STALE_MS)
    {
        DBG("IGNORED stale pilot input (age=%lu ms)\n", millis() - getLastControlPacket());
        return;
    }

    // Ignore inputs for a short period after explicit release by pilot
    if (millis() < releaseLockUntil)
    {
        DBG("IGNORED pilot input due to release lock (requested=%.2f)\n", steering);
        return;
    }

    // Set target pilot steering; clamp to configured max to preserve servo headroom
    targetPilotSteering = constrain(steering, -MAX_PILOT_STEERING, MAX_PILOT_STEERING);
    DBG("SET pilot target=%.2f (current=%.2f)\n", targetPilotSteering, currentPilotSteering);
}

void setIdleLeanSteering(float steering)
{
    // Keep idle steering fully separate from flapping state. This prevents
    // zero-throttle lean commands from fighting stop/center or amplitude logic.
    forceCenterActive = false;
    flapFrequency = 0.0f;
    flapAmplitude = 0.0f;
    amplitudeActive = false;
    amplitudeResumeStart = 0;
    startupRampUntil = 0;
    phase = 0.0f;
    wingsIdle = true;

    float lean = constrain(steering * IDLE_LEAN_GAIN, -LEAN_MAX_STEERING, LEAN_MAX_STEERING);
    targetPilotSteering = lean;
    currentPilotSteering = lean;
    DBG("SET idle lean target = %.2f\n", targetPilotSteering);
}

// Prevent reapplication of controls for a short time after an explicit release
void lockControlsMs(unsigned long ms)
{
    releaseLockUntil = millis() + ms;
    DBG("Controls locked until %lu\n", releaseLockUntil);
}

bool isControlReleaseLocked()
{
    return millis() < releaseLockUntil;
}

// Immediately stop flapping and center servos for an explicit pilot release
void immediateStopAndCenter()
{
    DBG("Immediate stop and center requested\n");

    // Stop flapping
    flapFrequency = 0.0f;
    phase = 0.0f;

    // Force servos to center immediately
    float tgtL = leftNeutralAngle() + leftTrim + rollBiasDegrees;
    float tgtR = rightNeutralAngle() + rightTrim + rollBiasDegrees;
    idleLeanHold = false;
    idleLeanLeftAngle = tgtL;
    idleLeanRightAngle = tgtR;
    if (fabs(tgtL - lastLeftAngle) > 0.4f) { writeLeftServoAngle(tgtL); lastLeftAngle = tgtL; }
    if (fabs(tgtR - lastRightAngle) > 0.4f) { writeRightServoAngle(tgtR); lastRightAngle = tgtR; }

    // Zero targets and currents so no later smoothing changes them
    targetPilotSteering = 0.0f;
    currentPilotSteering = 0.0f;

    wingsIdle = true;

    // Activate forced center to prevent any other code path from reapplying motions
    forceCenterActive = true;
}

void clearForceCenter()
{
    forceCenterActive = false;
    // allow wing updates
    wingsIdle = false;
    DBG("Force-center cleared by new input\n");
}

bool isForceCenterActive()
{
    return forceCenterActive;
}

// Telemetry getters
float getFlapFrequency()
{
    return flapFrequency;
}

float getFlapAmplitude()
{
    return flapAmplitude;
}

float getPilotSteeringCurrent()
{
    return currentPilotSteering;
}

void startPerch()
{
    perchActive = true;
    // Ensure not in forced-center
    forceCenterActive = false;
    // Immediately set slow decorative frequency and amplitude and mark wings active
    setFrequency(PERCH_FREQUENCY);
    // Force the amplitude immediately (bypass slow smoothing) so decorative motion begins
    flapAmplitude = PERCH_AMPLITUDE;
    // Mark amplitude state active so steering is allowed
    amplitudeActive = true;
    amplitudeResumeStart = 0;
    // clear any startup ramp so full motion is allowed
    startupRampUntil = 0;
    // ensure wings are considered active
    wingsIdle = false;
    phase = 0.0f;
    // prevent pilot inputs from immediately cancelling perch
    lockControlsMs(500);
}

void stopPerch()
{
    perchActive = false;
    // stop decorative motion and return to idle
    setFrequency(0.0f);
    setAmplitude(0.0f);
    // clear locks so pilot can control immediately
    releaseLockUntil = 0;
}

bool isPerchActive()
{
    return perchActive;
}

// Perch mode waveform: outputs values in [0..1] so flaps go from neutral (0)
// up to full excursion (1) and never go below neutral during the down phase.
float perchWave(float phase)
{
    float wrapped = fmod(phase, 1.0f);
    // Sections: rise -> top hold -> fall -> neutral hold
    const float rise = 0.42f;
    const float top  = 0.10f; // brief pause at top
    const float fall = 0.42f;
    const float neutral = 0.06f; // very brief pause at neutral

    if (wrapped < rise)
    {
        float t = wrapped / rise;
        // ease-out cubic for pleasant rise
        return 1.0f - powf(1.0f - t, 3);
    }
    wrapped -= rise;
    if (wrapped < top)
    {
        return 1.0f; // hold at top
    }
    wrapped -= top;
    if (wrapped < fall)
    {
        float t = wrapped / fall;
        // smooth ease-in fall (quadratic)
        return 1.0f - (t * t);
    }
    // neutral hold
    return 0.0f;
}

// Forward declaration for perch waveform used earlier in updateWingMotion
float perchWave(float phase);

#include "wifi_control.h"

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Preferences.h>
#include <esp_now.h>

#include "wing_motion.h"
#include "config.h"
#include <math.h>
#include "flight_mode.h"
#include "flight_profile.h"
#include "hover_bias.h"
#include "led_control.h"
#include "hardware_config.h"
#include "esp_flutter_espnow_packet.h"

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// Telemetry broadcast timing
unsigned long lastTelemetryMillis = 0;
// forward declare websocket event handler
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

const char *ssid = "Butterfly_RC";
const char *password = "12345678";

unsigned long lastControlPacket = 0;
float lastThrottle = 0.0f;
float lastSteer = 0.0f;
bool batteryAdcSaturated = false;
// Amplitude scale multiplier controlled from the web UI.
// Range: 0.6 .. 1.0 (60% .. 100%). Default is 75%.
float amplitudeScale = UPPER_FLAP_AMPLITUDE_DEFAULT_DEGREES / MAX_UPSTROKE_ABOVE_NEUTRAL;
static float lowerAmplitudeScale = LOWER_FLAP_AMPLITUDE_DEFAULT_DEGREES / MAX_DOWNSTROKE_BELOW_NEUTRAL;
static float upperAmplitudeScale = UPPER_FLAP_AMPLITUDE_DEFAULT_DEGREES / MAX_UPSTROKE_ABOVE_NEUTRAL;
static Preferences prefsControl;
static const char *CONTROL_PREF_NS = "control";
static const char *PROTOCOL_PREF_NS = "protocol";
static const char *KEY_PROTOCOL_VALUE = "value";
static const char *KEY_AMP_SCALE = "ampScale";
static const char *KEY_BATT_ENABLED = "battEn";
static const char *KEY_BATT_TOP = "battTop";
static const char *KEY_BATT_BOTTOM = "battBot";
static const char *KEY_BATT_CELLS = "battCells";
static const char *KEY_BATT_CHEM = "battChem";
static const char *KEY_BATT_FS_ENABLED = "battFsEn";
static const char *KEY_BATT_FS_CUTOFF = "battFsV";
static const char *KEY_CONTROL_PROTOCOL = "protocol";
static const char *KEY_LOWER_AMP_SCALE = "lowerAmp";
static const char *KEY_UPPER_AMP_SCALE = "upperAmp";
static const char *KEY_FLIGHT_RECORD_MS = "recordMs";
static const char *KEY_CH_MAP_PREFIX = "chMap";
static unsigned long flightAccumulatedMs = 0;
static unsigned long flightStartMs = 0;
static unsigned long currentFlightSessionMs = 0;
static unsigned long flightRecordMs = 0;
static unsigned long flightRecordSessionBaselineMs = 0;
static bool flightTimerRunning = false;
static uint32_t lastControlSeq = 0;
static uint32_t lastReleaseSeq = 0;

enum ControlProtocol
{
    CONTROL_PROTOCOL_WIFI = 0,
    CONTROL_PROTOCOL_ESPNOW = 1,
    CONTROL_PROTOCOL_ELRS = 2,
};

enum ChannelFunction
{
    CHANNEL_FUNC_UNUSED = 0,
    CHANNEL_FUNC_ROLL = 1,
    CHANNEL_FUNC_PITCH = 2,
    CHANNEL_FUNC_THROTTLE = 3,
    CHANNEL_FUNC_STEERING = 4,
    CHANNEL_FUNC_ARM = 5,
    CHANNEL_FUNC_WAVEFORM = 6,
    CHANNEL_FUNC_FLIGHT_MODE = 7,
    CHANNEL_FUNC_LED_MODE = 8,
};

static ControlProtocol controlProtocol = CONTROL_PROTOCOL_WIFI;
static bool wifiRadioEnabled = true;
static bool elrsWifiShutdownPending = false;
static unsigned long elrsWifiShutdownAtMs = 0;
static bool batteryWarningLatched = false;
static bool batteryLowActive = false;
static bool batteryCriticalActive = false;
static bool batteryMonitorEnabled = true;
static float batteryDividerTopOhms = BATTERY_DIVIDER_TOP_OHMS;
static float batteryDividerBottomOhms = BATTERY_DIVIDER_BOTTOM_OHMS;
static uint8_t batteryCellCount = 2;
static uint8_t batteryChemistry = 0; // 0=LiPo, 1=LiHv
static bool batteryFailsafeEnabled = false;
static float batteryFailsafeCutoffVoltage = 7.00f;
static bool batteryServoFailsafeActive = false;
static uint32_t batteryConfigRevision = 0;
static bool steeringLockEnabled = false;
static int channelMap[8] = {
    CHANNEL_FUNC_ROLL,
    CHANNEL_FUNC_PITCH,
    CHANNEL_FUNC_THROTTLE,
    CHANNEL_FUNC_STEERING,
    CHANNEL_FUNC_ARM,
    CHANNEL_FUNC_WAVEFORM,
    CHANNEL_FUNC_FLIGHT_MODE,
    CHANNEL_FUNC_LED_MODE
};

static volatile bool espNowPacketPending = false;
static EspFlutterNowPacket espNowPendingPacket;
static uint32_t espNowPacketsReceived = 0;
static uint32_t espNowPacketsRejected = 0;
static unsigned long lastEspNowPacketMs = 0;

static const uint8_t RTLOPEZ_RC_DATA = 0x01;
static const uint8_t RTLOPEZ_FC_ALIVE = 0x10;
static const uint8_t RTLOPEZ_FC_DATA = 0x11;
static const uint8_t RTLOPEZ_PAIR_REQ = 0xfe;
static const uint8_t RTLOPEZ_PAIR_RES = 0xff;
static const uint8_t RTLOPEZ_WIFI_CHANNEL_DEFAULT = 7;
static const uint16_t RTLOPEZ_PWM_MIN = 1000;
static const uint16_t RTLOPEZ_PWM_CENTER = 1500;
static const uint16_t RTLOPEZ_PWM_MAX = 2000;
static const unsigned long RTLOPEZ_BEACON_INTERVAL_MS = 50;
static const unsigned long RTLOPEZ_ALIVE_INTERVAL_MS = 1000;
static const unsigned long RTLOPEZ_TELEMETRY_INTERVAL_MS = 500;
static const uint8_t ESPNOW_BROADCAST_PEER[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

struct __attribute__((packed)) RtlopezRcMessage
{
    uint8_t type;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    int8_t ch5;
    int8_t ch6;
    int8_t ch7;
    int8_t ch8;
    uint8_t csum;
};

struct __attribute__((packed)) RtlopezPairRequest
{
    uint8_t type;
    uint8_t channel;
    uint8_t csum;
};

struct __attribute__((packed)) RtlopezAliveMessage
{
    uint8_t type;
    uint8_t csum;
};

struct __attribute__((packed)) RtlopezFcMessage
{
    uint8_t type;
    int16_t ch1;
    int16_t ch2;
    int8_t ch3;
    int8_t ch4;
    uint8_t csum;
};

static volatile bool rtlopezRcPending = false;
static volatile bool rtlopezPairResponsePending = false;
static RtlopezRcMessage rtlopezPendingRc;
static uint8_t rtlopezPendingMac[6] = {0, 0, 0, 0, 0, 0};
static uint8_t rtlopezPeerMac[6] = {0, 0, 0, 0, 0, 0};
static bool rtlopezPaired = false;
static unsigned long rtlopezNextBeaconMs = 0;
static unsigned long rtlopezNextAliveMs = 0;
static unsigned long rtlopezNextTelemetryMs = 0;
static uint32_t rtlopezPacketsReceived = 0;
static uint32_t rtlopezPacketsRejected = 0;
static uint32_t rtlopezPairResponses = 0;
static unsigned long lastRtlopezPacketMs = 0;

static const uint8_t CRSF_SYNC_BYTE = 0xC8;
static const uint8_t CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16;
static const uint8_t CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED = 0x17;
static const uint8_t CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08;
static const uint8_t CRSF_MAX_FRAME_SIZE = 64;
static const uint8_t CRSF_PARSE_BUFFER_SIZE = 160;
static uint8_t crsfFrame[CRSF_MAX_FRAME_SIZE];
static uint8_t crsfIndex = 0;
static uint8_t crsfExpectedLength = 0;
static uint8_t crsfParseBuffer[CRSF_PARSE_BUFFER_SIZE];
static uint8_t crsfParseLength = 0;
static uint16_t crsfChannels[16] = {0};
static bool crsfChannelsReady = false;
static uint32_t crsfPacketsReceived = 0;
static uint32_t crsfPacketsRejected = 0;
static uint32_t crsfPackedRcFrames = 0;
static uint32_t crsfSubsetRcFrames = 0;
static uint32_t crsfValidFrames = 0;
static uint32_t crsfUnknownFrames = 0;
static uint8_t crsfLastRcFrameType = 0;
static uint8_t crsfLastFrameType = 0;
static uint8_t crsfLastFrameLength = 0;
static uint32_t crsfBytesDropped = 0;
static uint32_t crsfRawBytes = 0;
static uint32_t crsfSyncBytes = 0;
static uint32_t crsfLengthFails = 0;
static uint32_t crsfCrcFails = 0;
static unsigned long lastCrsfPacketMs = 0;
static unsigned long crsfNextTelemetryMs = 0;
static float cachedBatteryVoltage = 0.0f;
static float cachedBatteryPercent = 0.0f;
static bool cachedBatteryLow = false;
static bool cachedBatteryCritical = false;
static unsigned long nextBatterySampleMs = 0;
static unsigned long lastCrsfApplyMs = 0;
static unsigned long maxCrsfApplyGapMs = 0;
static unsigned long lastCrsfGapMs = 0;
static unsigned long crsfRateWindowStartMs = 0;
static uint16_t crsfFramesInWindow = 0;
static uint16_t crsfFramesPerSecond = 0;
static uint16_t crsfLastSerialAvailable = 0;
static uint16_t crsfMaxSerialAvailable = 0;
static uint32_t crsfBacklogFlushes = 0;
static const uint16_t CRSF_BACKLOG_FLUSH_THRESHOLD = 1800;
static const uint16_t CRSF_BACKLOG_KEEP_BYTES = 256;
static const uint16_t CRSF_MAX_FLUSH_BYTES_PER_LOOP = 512;
static const uint16_t CRSF_MAX_PARSE_BYTES_PER_LOOP = 512;
static const uint32_t CRSF_PARSE_BUDGET_US = 1200;
static const uint32_t CRSF_FLUSH_BUDGET_US = 500;
static portMUX_TYPE elrsPendingMux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t elrsTaskHandle = nullptr;
static volatile bool elrsPendingChannels = false;
static int elrsPendingPwm[8] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000};
static int lastAppliedThrottlePwm = 1000;
static unsigned long lastThrottleChangeMs = 0;
static int lastRadioPwm[8] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000};
static unsigned long lastRadioChannelChangeMs[8] = {0};
static int radioArmSwitchPos = 0;
static int radioWaveSwitchPos = 0;
static int radioProfileSwitchPos = 0;
static int radioLedSwitchPos = 0;

static bool isCrsfAddressByte(uint8_t value)
{
    return value == CRSF_SYNC_BYTE || value == 0xEA || value == 0xEC || value == 0xEE;
}

static bool isStaleControlSeq(uint32_t seq)
{
    return seq != 0 && (seq <= lastReleaseSeq || seq <= lastControlSeq);
}

static void noteControlSeq(uint32_t seq)
{
    if (seq > lastControlSeq)
        lastControlSeq = seq;
}

static void noteReleaseSeq(uint32_t seq)
{
    if (seq > lastReleaseSeq)
        lastReleaseSeq = seq;
    noteControlSeq(seq);
}

static float lowerAmplitudeDegreesFromScale(float scale);
static float upperAmplitudeDegreesFromScale(float scale);
static float lowerAmplitudeScaleFromDegrees(float degrees);
static float upperAmplitudeScaleFromDegrees(float degrees);

static void updateFlightTimer(float throttle)
{
    const bool flying = throttle > 0.05f;
    const unsigned long now = millis();

    if (flying && !flightTimerRunning)
    {
        flightStartMs = now;
        currentFlightSessionMs = 0;
        flightRecordSessionBaselineMs = 0;
        flightTimerRunning = true;
    }
    else if (!flying && flightTimerRunning)
    {
        unsigned long sessionMs = now - flightStartMs;
        unsigned long recordCandidateMs = sessionMs > flightRecordSessionBaselineMs
            ? sessionMs - flightRecordSessionBaselineMs
            : 0;
        flightAccumulatedMs += sessionMs;
        currentFlightSessionMs = sessionMs;
        flightTimerRunning = false;
        flightRecordSessionBaselineMs = 0;
        if (recordCandidateMs > flightRecordMs)
        {
            flightRecordMs = recordCandidateMs;
            prefsControl.begin(CONTROL_PREF_NS, false);
            prefsControl.putULong(KEY_FLIGHT_RECORD_MS, flightRecordMs);
            prefsControl.end();
        }
    }
}

static unsigned long getFlightTimerMs()
{
    if (!flightTimerRunning)
        return flightAccumulatedMs;
    return flightAccumulatedMs + (millis() - flightStartMs);
}

static unsigned long getCurrentFlightSessionMs()
{
    if (!flightTimerRunning)
        return currentFlightSessionMs;
    return millis() - flightStartMs;
}

static unsigned long getFlightRecordMs()
{
    unsigned long currentSessionMs = flightTimerRunning
        ? getCurrentFlightSessionMs()
        : 0;
    if (currentSessionMs > flightRecordSessionBaselineMs)
        currentSessionMs -= flightRecordSessionBaselineMs;
    else
        currentSessionMs = 0;
    return currentSessionMs > flightRecordMs ? currentSessionMs : flightRecordMs;
}

static void resetFlightRecord()
{
    flightRecordMs = 0;
    flightRecordSessionBaselineMs = flightTimerRunning ? getCurrentFlightSessionMs() : 0;
    prefsControl.begin(CONTROL_PREF_NS, false);
    prefsControl.putULong(KEY_FLIGHT_RECORD_MS, flightRecordMs);
    prefsControl.end();
}

static const char *channelMapKey(uint8_t index)
{
    static char key[12];
    snprintf(key, sizeof(key), "%s%u", KEY_CH_MAP_PREFIX, (unsigned)index);
    return key;
}

static int clampChannelFunction(int value)
{
    if (value < CHANNEL_FUNC_UNUSED || value > CHANNEL_FUNC_LED_MODE)
        return CHANNEL_FUNC_UNUSED;
    return value;
}

static void loadControlPreferences()
{
    prefsControl.begin(CONTROL_PREF_NS, true);
    amplitudeScale = prefsControl.getFloat(KEY_AMP_SCALE, amplitudeScale);
    lowerAmplitudeScale = prefsControl.getFloat(KEY_LOWER_AMP_SCALE, lowerAmplitudeScaleFromDegrees(LOWER_FLAP_AMPLITUDE_DEFAULT_DEGREES));
    upperAmplitudeScale = prefsControl.getFloat(KEY_UPPER_AMP_SCALE, upperAmplitudeScaleFromDegrees(UPPER_FLAP_AMPLITUDE_DEFAULT_DEGREES));
    int storedProtocol = prefsControl.getInt(KEY_CONTROL_PROTOCOL, (int)CONTROL_PROTOCOL_WIFI);
    batteryMonitorEnabled = prefsControl.getBool(KEY_BATT_ENABLED, batteryMonitorEnabled);
    batteryDividerTopOhms = prefsControl.getFloat(KEY_BATT_TOP, batteryDividerTopOhms);
    batteryDividerBottomOhms = prefsControl.getFloat(KEY_BATT_BOTTOM, batteryDividerBottomOhms);
    batteryCellCount = prefsControl.getUChar(KEY_BATT_CELLS, batteryCellCount);
    batteryChemistry = prefsControl.getUChar(KEY_BATT_CHEM, batteryChemistry);
    batteryFailsafeEnabled = prefsControl.getBool(KEY_BATT_FS_ENABLED, batteryFailsafeEnabled);
    batteryFailsafeCutoffVoltage = prefsControl.getFloat(KEY_BATT_FS_CUTOFF, batteryFailsafeCutoffVoltage);
    flightRecordMs = prefsControl.getULong(KEY_FLIGHT_RECORD_MS, flightRecordMs);
    for (uint8_t i = 0; i < 8; ++i)
        channelMap[i] = clampChannelFunction(prefsControl.getInt(channelMapKey(i), channelMap[i]));
    prefsControl.end();
    if (amplitudeScale < 0.6f) amplitudeScale = 0.6f;
    if (amplitudeScale > 1.0f) amplitudeScale = 1.0f;
    lowerAmplitudeScale = lowerAmplitudeScaleFromDegrees(lowerAmplitudeDegreesFromScale(lowerAmplitudeScale));
    upperAmplitudeScale = upperAmplitudeScaleFromDegrees(upperAmplitudeDegreesFromScale(upperAmplitudeScale));
    if (batteryDividerTopOhms >= 1.0f && batteryDividerTopOhms < 1000.0f) batteryDividerTopOhms *= 1000.0f;
    if (batteryDividerBottomOhms >= 1.0f && batteryDividerBottomOhms < 1000.0f) batteryDividerBottomOhms *= 1000.0f;
    if (batteryDividerTopOhms < 1.0f) batteryDividerTopOhms = BATTERY_DIVIDER_TOP_OHMS;
    if (batteryDividerBottomOhms < 1.0f) batteryDividerBottomOhms = BATTERY_DIVIDER_BOTTOM_OHMS;
    if (batteryCellCount < 1) batteryCellCount = 1;
    if (batteryCellCount > 2) batteryCellCount = 2;
    if (batteryChemistry > 1) batteryChemistry = 0;
    if (batteryFailsafeCutoffVoltage < 0.0f) batteryFailsafeCutoffVoltage = 0.0f;
    if (batteryFailsafeCutoffVoltage > 30.0f) batteryFailsafeCutoffVoltage = 30.0f;
    if (storedProtocol == CONTROL_PROTOCOL_ESPNOW)
        controlProtocol = CONTROL_PROTOCOL_ESPNOW;
    else if (storedProtocol == CONTROL_PROTOCOL_ELRS)
        controlProtocol = CONTROL_PROTOCOL_ELRS;
    else
        controlProtocol = CONTROL_PROTOCOL_WIFI;

    Preferences prefsProtocol;
    prefsProtocol.begin(PROTOCOL_PREF_NS, true);
    int dedicatedProtocol = prefsProtocol.getInt(KEY_PROTOCOL_VALUE, (int)controlProtocol);
    prefsProtocol.end();
    if (dedicatedProtocol == CONTROL_PROTOCOL_ESPNOW)
        controlProtocol = CONTROL_PROTOCOL_ESPNOW;
    else if (dedicatedProtocol == CONTROL_PROTOCOL_ELRS)
        controlProtocol = CONTROL_PROTOCOL_ELRS;
    else
        controlProtocol = CONTROL_PROTOCOL_WIFI;
}

static void saveChannelMapping()
{
    prefsControl.begin(CONTROL_PREF_NS, false);
    for (uint8_t i = 0; i < 8; ++i)
        prefsControl.putInt(channelMapKey(i), channelMap[i]);
    prefsControl.end();
}

static void saveAmplitudeScale()
{
    prefsControl.begin(CONTROL_PREF_NS, false);
    prefsControl.putFloat(KEY_AMP_SCALE, amplitudeScale);
    prefsControl.putFloat(KEY_LOWER_AMP_SCALE, lowerAmplitudeScale);
    prefsControl.putFloat(KEY_UPPER_AMP_SCALE, upperAmplitudeScale);
    prefsControl.end();
}

static float lowerAmplitudeDegreesFromScale(float scale)
{
    return constrain(scale * MAX_DOWNSTROKE_BELOW_NEUTRAL,
        LOWER_FLAP_AMPLITUDE_MIN_DEGREES,
        LOWER_FLAP_AMPLITUDE_MAX_DEGREES);
}

static float upperAmplitudeDegreesFromScale(float scale)
{
    return constrain(scale * MAX_UPSTROKE_ABOVE_NEUTRAL,
        UPPER_FLAP_AMPLITUDE_MIN_DEGREES,
        UPPER_FLAP_AMPLITUDE_MAX_DEGREES);
}

static float lowerAmplitudeScaleFromDegrees(float degrees)
{
    degrees = constrain(degrees,
        LOWER_FLAP_AMPLITUDE_MIN_DEGREES,
        LOWER_FLAP_AMPLITUDE_MAX_DEGREES);
    return degrees / MAX_DOWNSTROKE_BELOW_NEUTRAL;
}

static float upperAmplitudeScaleFromDegrees(float degrees)
{
    degrees = constrain(degrees,
        UPPER_FLAP_AMPLITUDE_MIN_DEGREES,
        UPPER_FLAP_AMPLITUDE_MAX_DEGREES);
    return degrees / MAX_UPSTROKE_ABOVE_NEUTRAL;
}

static void saveControlProtocol()
{
    prefsControl.begin(CONTROL_PREF_NS, false);
    prefsControl.putInt(KEY_CONTROL_PROTOCOL, (int)controlProtocol);
    prefsControl.end();

    Preferences prefsProtocol;
    prefsProtocol.begin(PROTOCOL_PREF_NS, false);
    prefsProtocol.putInt(KEY_PROTOCOL_VALUE, (int)controlProtocol);
    prefsProtocol.end();
}

static void applyCurrentFlightProfile()
{
    FlightProfileSettings settings = getCurrentFlightProfileSettings();
    lowerAmplitudeScale = settings.lowerAmplitudeScale;
    upperAmplitudeScale = settings.upperAmplitudeScale;
    amplitudeScale = fmaxf(lowerAmplitudeScale, upperAmplitudeScale);
    setFlapAmplitudeScales(lowerAmplitudeScale, upperAmplitudeScale);
    setSteeringDifferentialGain(settings.steeringGain);
    setTurnCompensation(settings.turnCompensation);
    setServoPitchAngle(settings.neutralPitch);
    setFlapWaveform(settings.waveform);
    setFrequencyLimit(getProfileFrequencyCap());
}

unsigned long getLastControlPacket()
{
    return lastControlPacket;
}

float batteryVoltageToPercent(float voltage)
{
    if (!batteryMonitorEnabled || batteryCellCount == 0)
        return 0.0f;

    float perCell = voltage / (float)batteryCellCount;
    static const float lipoVoltageTable[] = {
        3.30f, 3.60f, 3.70f, 3.75f, 3.80f, 3.85f, 3.90f,
        3.95f, 4.00f, 4.05f, 4.10f, 4.15f, 4.20f
    };
    static const float lihvVoltageTable[] = {
        3.30f, 3.60f, 3.70f, 3.78f, 3.86f, 3.94f, 4.02f,
        4.10f, 4.16f, 4.22f, 4.28f, 4.32f, 4.35f
    };
    static const float percentTable[] = {
        0.00f, 0.10f, 0.20f, 0.30f, 0.40f, 0.50f, 0.60f,
        0.70f, 0.80f, 0.85f, 0.90f, 0.95f, 1.00f
    };
    const float *voltageTable = batteryChemistry == 1 ? lihvVoltageTable : lipoVoltageTable;
    const size_t count = sizeof(lipoVoltageTable) / sizeof(lipoVoltageTable[0]);

    if (perCell <= voltageTable[0]) return 0.0f;
    if (perCell >= voltageTable[count - 1]) return 1.0f;

    for (size_t i = 1; i < count; ++i)
    {
        if (perCell <= voltageTable[i])
        {
            float span = voltageTable[i] - voltageTable[i - 1];
            float position = (perCell - voltageTable[i - 1]) / span;
            return percentTable[i - 1] + position * (percentTable[i] - percentTable[i - 1]);
        }
    }
    return 0.0f;
}

static void updateIndicatorLeds()
{
    const unsigned long now = millis();

    if (!batteryMonitorEnabled || !batteryLowActive)
    {
        digitalWrite(getBatteryWarningLedPin(), LOW);
    }
    else
    {
        const unsigned long period = batteryCriticalActive ? 500UL : 1000UL;
        digitalWrite(getBatteryWarningLedPin(), (now % period) < (period / 2) ? HIGH : LOW);
    }

    if (getFlightMode() == FLIGHT_MODE_ARMED)
    {
        digitalWrite(getStatusLedPin(), HIGH);
        return;
    }

    const uint8_t blinks = controlProtocol == CONTROL_PROTOCOL_ELRS
        ? 3
        : (controlProtocol == CONTROL_PROTOCOL_ESPNOW ? 2 : 1);
    const unsigned long onMs = 120;
    const unsigned long gapMs = 170;
    const unsigned long pauseMs = 900;
    const unsigned long burstMs = blinks * (onMs + gapMs);
    const unsigned long cycleMs = burstMs + pauseMs;
    const unsigned long phase = now % cycleMs;
    bool ledOn = false;
    if (phase < burstMs)
    {
        const unsigned long slot = phase % (onMs + gapMs);
        ledOn = slot < onMs;
    }
    digitalWrite(getStatusLedPin(), ledOn ? HIGH : LOW);
}

static void saveBatteryConfig()
{
    prefsControl.begin(CONTROL_PREF_NS, false);
    prefsControl.putBool(KEY_BATT_ENABLED, batteryMonitorEnabled);
    prefsControl.putFloat(KEY_BATT_TOP, batteryDividerTopOhms);
    prefsControl.putFloat(KEY_BATT_BOTTOM, batteryDividerBottomOhms);
    prefsControl.putUChar(KEY_BATT_CELLS, batteryCellCount);
    prefsControl.putUChar(KEY_BATT_CHEM, batteryChemistry);
    prefsControl.putBool(KEY_BATT_FS_ENABLED, batteryFailsafeEnabled);
    prefsControl.putFloat(KEY_BATT_FS_CUTOFF, batteryFailsafeCutoffVoltage);
    prefsControl.end();
}

static bool updateBatteryWarningState(float voltage)
{
    if (!batteryMonitorEnabled || voltage <= 0.1f)
    {
        batteryWarningLatched = false;
        return false;
    }

    float warningVoltage = batteryCellCount * 3.70f;
    float clearVoltage = warningVoltage + 0.05f;
    if (voltage <= warningVoltage)
        batteryWarningLatched = true;
    else if (voltage >= clearVoltage)
        batteryWarningLatched = false;
    return batteryWarningLatched;
}

static bool isBatteryCritical(float voltage)
{
    return batteryMonitorEnabled && voltage > 0.1f && voltage <= (batteryCellCount * 3.50f);
}

// Calibrated, averaged and filtered battery measurement.
float readBatteryVoltage()
{
    if (!batteryMonitorEnabled)
        return 0.0f;

    static bool initialized = false;
    static float filteredVoltage = 0.0f;
    static uint32_t lastConfigRevision = 0;
    uint32_t millivoltSum = 0;
    uint16_t saturatedSamples = 0;

    if (lastConfigRevision != batteryConfigRevision)
    {
        initialized = false;
        lastConfigRevision = batteryConfigRevision;
    }

    // Discard the first conversion after switching into ADC sampling.
    analogRead(getBatteryPin());
    for (uint16_t i = 0; i < BATTERY_ADC_SAMPLE_COUNT; ++i)
    {
        uint16_t raw = analogRead(getBatteryPin());
        if (raw >= BATTERY_ADC_SATURATION_RAW) ++saturatedSamples;
        millivoltSum += analogReadMilliVolts(getBatteryPin());
        delayMicroseconds(120);
    }

    batteryAdcSaturated = saturatedSamples > (BATTERY_ADC_SAMPLE_COUNT / 4);
    float adcVoltage = (millivoltSum / (float)BATTERY_ADC_SAMPLE_COUNT) / 1000.0f;
    if (adcVoltage < 0.02f)
    {
        initialized = false;
        return 0.0f;
    }

    float divider = BATTERY_VOLTAGE_DIVIDER;
    if (batteryDividerTopOhms > 0.0f && batteryDividerBottomOhms > 0.0f)
        divider = (batteryDividerTopOhms + batteryDividerBottomOhms) / batteryDividerBottomOhms;
    float measuredVoltage = adcVoltage * divider * BATTERY_VOLTAGE_CALIBRATION;
    if (!initialized)
    {
        filteredVoltage = measuredVoltage;
        initialized = true;
    }
    else
    {
        float alpha = measuredVoltage < filteredVoltage
            ? BATTERY_FILTER_FALL_ALPHA
            : BATTERY_FILTER_RISE_ALPHA;
        filteredVoltage += alpha * (measuredVoltage - filteredVoltage);
    }
    return filteredVoltage;
}

static void updateCachedBatteryTelemetry(bool force)
{
    unsigned long now = millis();
    unsigned long interval = controlProtocol == CONTROL_PROTOCOL_ELRS ? 250UL : 150UL;
    if (!force && now < nextBatterySampleMs)
        return;

    cachedBatteryVoltage = readBatteryVoltage();
    cachedBatteryPercent = batteryVoltageToPercent(cachedBatteryVoltage);
    cachedBatteryLow = updateBatteryWarningState(cachedBatteryVoltage);
    cachedBatteryCritical = isBatteryCritical(cachedBatteryVoltage);
    batteryLowActive = cachedBatteryLow;
    batteryCriticalActive = cachedBatteryCritical;
    nextBatterySampleMs = now + interval;
}

static bool isBatteryServoFailsafeActive()
{
    return batteryServoFailsafeActive;
}

static void enforceBatteryServoFailsafe()
{
    bool active = batteryMonitorEnabled &&
        batteryFailsafeEnabled &&
        cachedBatteryVoltage > 0.1f &&
        cachedBatteryVoltage <= batteryFailsafeCutoffVoltage;
    bool wasActive = batteryServoFailsafeActive;
    batteryServoFailsafeActive = active;
    if (!active)
        return;

    stopPerch();
    setFrequency(0.0f);
    setAmplitude(0.0f);
    setRollBias(0.0f);
    if (!wasActive)
    {
        setIdleLeanSteering(0.0f);
        immediateStopAndCenter();
    }
    lastThrottle = 0.0f;
    lastSteer = 0.0f;
    updateFlightTimer(0.0f);
}

static void disableWifiForElrsIsolation()
{
#if ELRS_RF_ISOLATION_WIFI_OFF
    if (!wifiRadioEnabled)
        return;
    webSocket.close();
    server.stop();
    WiFi.softAPdisconnect(true);
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    wifiRadioEnabled = false;
    Serial.println("ELRS RF isolation active: WiFi/AP disabled until reboot");
#endif
}

static void applyRuntimeHardwarePins()
{
    immediateStopAndCenter();
    reattachWingServos();
    reattachLedOutput();
    Serial1.end();
    Serial1.setRxBufferSize(2048);
    Serial1.begin(ELRS_BAUD, SERIAL_8N1, getElrsRxPin(), getElrsTxPin());
    Serial1.setTimeout(0);
    pinMode(getBatteryPin(), INPUT);
    pinMode(getBatteryWarningLedPin(), OUTPUT);
    pinMode(getStatusLedPin(), OUTPUT);
    digitalWrite(getBatteryWarningLedPin(), LOW);
    digitalWrite(getStatusLedPin(), LOW);
    analogReadResolution(12);
    analogSetPinAttenuation(getBatteryPin(), ADC_11db);
    ++batteryConfigRevision;
    updateCachedBatteryTelemetry(true);
}

void handleRoot()
{
        static const char html[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
:root{--bg:#070707;--panel:#0b0b0b;--text:#e6ffe6;--muted:#bfffc2;--accent:#00ff66;--accent-soft:rgba(0,255,102,0.35);--fill:#00ff88;--dark:#00110a}
body.light-theme{--bg:#f4fff7;--panel:#ffffff;--text:#062111;--muted:#145c31;--dark:#ffffff}
body{background:var(--bg);color:var(--text);font-family:Helvetica,Arial,sans-serif;margin:0;padding:8px;height:100vh;overflow:hidden;-webkit-user-select:none;-ms-touch-action:none;box-sizing:border-box}
*{box-sizing:border-box}
.container{max-width:none;width:100%;margin:0 auto;text-align:center;overflow-x:hidden;position:relative}
.title{font-weight:700;margin:6px 68px 8px;text-align:center;font-size:17px;line-height:1.15}
.app-logo{position:absolute;left:0;top:8px;width:64px;height:auto;object-fit:contain;opacity:.98;z-index:5;display:block;pointer-events:none}
.panel{background:var(--panel);padding:10px;border-radius:8px;border:2px solid color-mix(in srgb,var(--accent) 12%,transparent);font-size:13px}
.panel.led-glow-solid{border-color:var(--accent);box-shadow:0 0 var(--panel-led-glow,14px) color-mix(in srgb,var(--accent) var(--panel-led-strength,60%),transparent)}
.panel.led-glow-fade{border-color:var(--accent);animation:panelLedFade var(--panel-led-speed,1.4s) ease-in-out infinite}
.panel.led-glow-strobe{border-color:var(--accent);animation:panelLedStrobe var(--panel-led-speed,0.45s) steps(2,end) infinite}
@keyframes panelLedFade{0%,100%{box-shadow:0 0 2px color-mix(in srgb,var(--accent) 15%,transparent)}50%{box-shadow:0 0 var(--panel-led-glow,16px) color-mix(in srgb,var(--accent) var(--panel-led-strength,65%),transparent)}}
@keyframes panelLedStrobe{0%,49%{box-shadow:0 0 var(--panel-led-glow,16px) color-mix(in srgb,var(--accent) var(--panel-led-strength,70%),transparent)}50%,100%{box-shadow:none}}
    .tele-row{display:flex;flex-direction:column;gap:8px;align-items:center;justify-content:center}
    .bars{display:flex;flex-direction:row;gap:12px;align-items:flex-end}
.bar-wrap{width:41px;height:144px;border:3px solid var(--accent);border-radius:6px;background:#00220a;position:relative;overflow:hidden}
.top-battery-panel{display:none;margin:6px 0 8px;width:100%}
.battery-wrap{--battery-color:#00ff66;--battery-bg:rgba(0,34,10,0.75);border:2px solid var(--battery-color);border-radius:5px;background:var(--battery-bg);height:34px;position:relative;overflow:visible;width:calc(100% - 22px);margin:0 11px}
.battery-wrap:before{content:"";position:absolute;left:-8px;top:3px;width:3px;height:28px;border:0;border-radius:2px;background:var(--battery-color);box-shadow:0 0 6px var(--battery-color)}
.battery-wrap:after{content:"";position:absolute;right:-11px;top:8px;width:8px;height:16px;border:2px solid var(--battery-color);border-left:0;border-radius:0 3px 3px 0;background:var(--battery-bg)}
.battery-wrap .bar-fill{position:absolute;left:2px;right:2px;top:2px;bottom:2px;height:auto!important;display:grid;grid-template-columns:repeat(20,1fr);gap:2px;background:transparent!important}
.battery-seg{border-radius:2px;background:rgba(255,255,255,0.08)}
.battery-seg.active{background:var(--battery-color);box-shadow:0 0 7px var(--battery-color)}
.battery-warning-text{position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);z-index:3;color:#ffffff;font-weight:900;font-size:12px;text-shadow:0 1px 0 #330000,1px 0 0 #330000,0 -1px 0 #330000,-1px 0 0 #330000;display:none;white-space:nowrap;pointer-events:none}
.battery-wrap.critical{animation:batteryPulse 0.85s ease-in-out infinite}
@keyframes batteryPulse{0%,100%{opacity:1}50%{opacity:0.38}}
.bar-fill{position:absolute;left:0;right:0;bottom:0;background:linear-gradient(#00ff88,#007744);height:0%;transition:height 0.06s linear}
.bar-label{font-size:14px;color:var(--muted);margin-bottom:6px}
.bar-percent{font-size:15px;color:#eaffea;margin-top:6px}
.battery-readout{display:flex;justify-content:space-between;font-size:24px;line-height:1;color:var(--muted);margin:5px 2px 0;font-weight:800}
.bar-overlay{color:#f4fff7!important;font-weight:700;font-size:15px}
.butterfly-meter{width:119px;height:108px;position:relative;margin:0 auto;image-rendering:pixelated;--flap-duration:1s;--wing-open-scale:1;--wing-closed-scale:.36;--wing-camber:5deg}
.pixel-butterfly{position:absolute;left:50%;top:50%;width:94px;height:70px;transform:translate(-50%,-50%) scale(1.1);animation:butterflyBob var(--flap-duration) linear infinite}
.pixel-body{position:absolute;left:41px;top:19px;width:12px;height:32px;background:#082412;box-shadow:0 -6px 0 #0d361b,0 7px 0 var(--accent) inset,0 17px 0 #06180c}
.throttle-core{position:absolute;left:33px;top:21px;width:28px;height:28px;border-radius:50%;border:3px solid #062b13;background:radial-gradient(circle,#07140d 0 48%,#0b2f17 50% 100%);z-index:4}
.throttle-core:before{content:"";position:absolute;left:calc(50% - 1.5px);top:calc(50% - 13px);width:3px;height:13px;background:#ff3333;border-radius:2px;transform-origin:50% 100%;transform:rotate(var(--throttle-needle,-130deg))}
.throttle-core:after{content:"";position:absolute;left:50%;top:50%;width:5px;height:5px;border-radius:50%;background:#f4fff7;transform:translate(-50%,-50%)}
.pixel-wing{position:absolute;top:5px;width:43px;height:60px;animation-duration:var(--flap-duration);animation-timing-function:linear;animation-iteration-count:infinite;will-change:transform}
.pixel-wing:before{content:"";position:absolute;left:0;top:0;width:36px;height:31px;background:#06180c;border-radius:7px 13px 7px 3px;box-shadow:0 0 0 3px var(--accent) inset,0 0 8px var(--accent),6px 6px 0 #082412 inset,17px 9px 0 #020904 inset,5px 21px 0 color-mix(in srgb,var(--accent) 36%,#06180c) inset}
.pixel-wing:after{content:"";position:absolute;left:10px;top:31px;width:26px;height:24px;background:#041108;border-radius:4px 4px 11px 7px;box-shadow:0 0 0 3px var(--accent) inset,0 0 7px var(--accent),6px 6px 0 #082412 inset,14px 8px 0 #020904 inset}
.pixel-wing.left{left:-55px;transform-origin:100% 50%;animation-name:leftPixelWing}
.pixel-wing.right{right:1px;transform-origin:0 50%;animation-name:rightPixelWing}
.pixel-wing.left:before{left:auto;right:0;border-radius:7px 13px 7px 3px}
.pixel-wing.left:after{left:auto;right:0;border-radius:4px 4px 11px 7px}
.pixel-wing.right:before{left:7px;border-radius:13px 7px 3px 7px}
.pixel-wing.right:after{left:7px;border-radius:4px 4px 7px 11px}
.butterfly-idle .pixel-butterfly{animation:none;transform:translate(-50%,-50%) scale(1.1)}
.butterfly-idle .pixel-wing{animation:none}
.butterfly-idle .pixel-wing.left{transform:scaleX(var(--wing-open-scale)) scaleY(1.02) skewY(calc(-1 * var(--wing-camber))) rotate(-2deg)}
.butterfly-idle .pixel-wing.right{transform:scaleX(var(--wing-open-scale)) scaleY(1.02) skewY(var(--wing-camber)) rotate(2deg)}
.gauge-center{position:absolute;left:50%;top:61%;transform:translate(-50%,-50%);z-index:5;color:#f4fff7;font-weight:800;font-size:15px;text-shadow:0 2px 0 #001a09,2px 0 0 #001a09,0 -2px 0 #001a09,-2px 0 0 #001a09;pointer-events:none}
.gauge-label{font-size:14px;color:var(--muted);font-weight:700}
@keyframes leftPixelWing{0%,100%{transform:scaleX(var(--wing-closed-scale)) scaleY(1.08)}48%,52%{transform:scaleX(var(--wing-open-scale)) scaleY(1.02) skewY(calc(-1 * var(--wing-camber))) rotate(-2deg)}}
@keyframes rightPixelWing{0%,100%{transform:scaleX(var(--wing-closed-scale)) scaleY(1.08)}48%,52%{transform:scaleX(var(--wing-open-scale)) scaleY(1.02) skewY(var(--wing-camber)) rotate(2deg)}}
@keyframes butterflyBob{0%,100%{transform:translate(-50%,-50.7%) scale(1.1)}50%{transform:translate(-50%,-49.3%) scale(1.1)}}
.telemetry-tables{display:grid;grid-template-columns:1fr 1fr;gap:5px;margin-top:4px;text-align:left}
.tele-table{border-collapse:collapse;width:100%;font-size:12px;color:var(--text);background:color-mix(in srgb,var(--accent) 5%,transparent);border:1px solid color-mix(in srgb,var(--accent) 28%,transparent);border-radius:7px;overflow:hidden}
.tele-table td{padding:3px 5px;border-bottom:1px solid color-mix(in srgb,var(--accent) 16%,transparent)}
.tele-table tr:last-child td{border-bottom:0}
.tele-table td:first-child{color:var(--muted)}
.tele-table td:last-child{text-align:right;color:var(--text);font-weight:700}
.timer-box{border:1px solid var(--accent-soft);border-radius:7px;padding:7px;background:rgba(0,255,102,0.045);font-size:13px;color:var(--muted);text-align:left;min-width:0;width:100%}
.timer-title{text-align:center;color:var(--text);font-weight:800;font-size:16px;margin-bottom:5px}
.timer-table{display:flex;flex-direction:column;gap:2px;width:100%}
.timer-line{display:grid;grid-template-columns:minmax(46px,1fr) minmax(44px,0.9fr) 42px;align-items:center;gap:3px;margin:0;width:100%}
.timer-value{color:var(--text);font-weight:800;font-size:15px}
.timer-reset{font-size:11px;line-height:1.1;padding:4px 5px;border-width:1px;border-radius:5px;text-align:center;min-width:38px;justify-self:end}
.slider-section{width:100%;box-sizing:border-box;margin:6px 0 0;padding:7px;border:1px solid var(--accent-soft);border-radius:8px;display:flex;flex-direction:column;gap:5px;align-items:center}
.prefs-grid{display:grid;grid-template-columns:1fr 1fr;gap:5px;margin:6px 0 0;width:100%}
.prefs-grid .slider-section{width:100%;margin:0}
.prefs-grid .slider-section>div:last-child{width:100%}
.prefs-grid input[type=range]{width:calc(100% - 56px)!important;min-width:0}
.prefs-grid .mode-button{padding:6px 8px}
.slider-section input[type=range]{width:100%!important;max-width:100%;min-width:0;flex:1}
.slider-row{display:flex;align-items:center;gap:8px;width:100%}
.wide-slider-row{display:flex;align-items:center;gap:8px;width:100%}
.wide-slider-row input[type=range]{flex:1;width:100%!important}
.small-row{display:flex;gap:12px;justify-content:space-between;margin-top:10px}
.small-item{flex:1;text-align:center;font-size:13px;color:var(--text)}
        #tele-compact{display:grid;grid-template-columns:minmax(164px,0.92fr) minmax(116px,1.08fr);gap:6px;align-items:start;justify-content:center;padding:3px;overflow:hidden}
        #tele-compact .left{display:flex;flex-direction:column;align-items:flex-start;width:100%;min-width:0}
        #tele-compact .right{display:flex;flex-direction:row;align-items:center;justify-content:center;gap:0;min-width:0}
        .led-tele{display:flex;align-items:center;justify-content:flex-end;gap:6px}
        .led-tele #ledTopModeText{font-size:12px;color:var(--text);white-space:nowrap}
        .led-tele .mode-button{font-size:13px}
        #joystick{width:220px;height:220px;margin:8px auto 4px auto;position:relative;touch-action:none;-webkit-user-select:none;border:3px solid var(--accent-soft);border-radius:16px;overflow:visible;background:color-mix(in srgb,var(--accent) 5%,transparent)}
        #armWatermark{position:absolute;left:0;right:0;top:50%;transform:translateY(-50%);z-index:0;font-size:22px;font-weight:800;letter-spacing:0;text-align:center;opacity:0.5;pointer-events:none}
        #stick{width:80px;height:80px;background:var(--fill);border-radius:50%;position:absolute;left:70px;top:150px;z-index:2}
        #motionSteerNotice{display:none;margin:4px auto 0;color:#ffd86b;font-size:12px;font-weight:700}
        .mode-button{background:transparent;color:var(--accent);border:2px solid var(--accent);padding:8px 12px;border-radius:8px;font-weight:600}
        .mode-button.active{background:var(--accent);color:var(--dark)}
        .mode-button:disabled{opacity:0.6}
        input[type=checkbox],input[type=range]{accent-color:var(--accent)}
        select,input[type=number]{border-color:var(--accent)!important}
.app-footer{font-size:15px;color:var(--muted);opacity:.5;margin:6px 0 0}
</style>
</head>
<body>
<div class="container">
    <div class="title">ESP-FLUTTER Bionic RC Butterfly</div>
    <img class="app-logo" alt="Max Imagination" src="data:image/png;base64,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">
        <div id="topBatteryPanel" class="top-battery-panel">
            <div id="batteryWrap" class="battery-wrap">
                <div id="batteryFill" class="bar-fill"></div>
                <div id="batteryWarningText" class="battery-warning-text"></div>
            </div>
            <div class="battery-readout">
                <span id="batteryPctOverlay">0%</span>
                <span id="batteryVolt">0.00V</span>
            </div>
        </div>
        <div class="panel" id="controllerPanel">
            <div id="tele-compact">
                <div class="left">
                    <div class="timer-box">
                        <div class="timer-title">Timers</div>
                        <div class="timer-table">
                            <div class="timer-line"><span>Current</span><span class="timer-value" id="currentFlightTimer">0:00</span><span></span></div>
                            <div class="timer-line"><span>Total</span><span class="timer-value" id="flightTimer">0:00</span><span></span></div>
                            <div class="timer-line"><span>Record</span><span class="timer-value" id="recordTimer">0:00</span><button id="recordResetButton" class="mode-button timer-reset">RST</button></div>
                            <div class="timer-line"><span>Uptime</span><span class="timer-value" id="uptimeTimer">0:00</span><span></span></div>
                        </div>
                    </div>
                </div>
                <div class="right">
                    <div style="text-align:center;font-size:13px;min-width:112px">
                        <div id="throttleFill" class="butterfly-meter">
                            <div class="pixel-butterfly">
                                <div class="pixel-wing left"></div>
                                <div class="pixel-wing right"></div>
                                <div class="throttle-core"></div>
                                <div class="pixel-body"></div>
                            </div>
                            <div id="throttlePctOverlay" class="gauge-center">0%</div>
                        </div>
                    </div>
                </div>
            </div>
            <div class="telemetry-tables">
                <table class="tele-table">
                    <tr><td>Amplitude</td><td id="tAmp">-</td></tr>
                    <tr><td>Neutral Pitch</td><td id="tNeutral">-</td></tr>
                </table>
                <table class="tele-table">
                    <tr id="phoneTiltRow" style="display:none"><td>Phone Tilt</td><td id="tPhonePitch">-</td></tr>
                    <tr><td>LED</td><td><div class="led-tele"><button id="ledTopButton" class="mode-button" style="padding:5px 10px;min-width:70px">OFF</button><span id="ledTopModeText">Mode - 0</span></div></td></tr>
                </table>
            </div>
        </div>

        <div id="joystick"><div id="armWatermark">DISARMED</div><div id="stick"></div></div>
        <div id="motionSteerNotice">Steering locked: phone tilt controls turning</div>
        <div id="controllerFooter" class="app-footer">&copy; 2026 Max Imagination</div>
</div>

<script>
window.addEventListener('load', function(){
// joystick logic (restored) with periodic heartbeat
const joystick = document.getElementById("joystick");
const stick = document.getElementById("stick");
let centerX = 110; let centerY = 110; let dragging = false;
let currentSteer = 0.0;
let currentThrottle = 0.0;
let motionSteeringEnabledLocal = false;
let latestMotionSteer = 0.0;
let filteredMotionSteer = 0.0;
let motionSteerFilterInit = false;
let motionSteerLastMs = 0;
let joystickSteerFilterInit = false;
let filteredJoystickSteer = 0.0;
let joystickSteerLastMs = 0;
const joystickSteerFilterMs = 18.0;
let controlsReleased = true;
let releaseGuardUntil = 0;
let controlSeq = (Date.now() & 0x7fffffff) || 1;
let controlsArmed = false;
let currentModeName = 'DISARMED';
let controlProtocolLocal = 0;
let joystickSteeringLocked = false;

function wifiControlActive(){
    return controlProtocolLocal === 0;
}

function updateProtocolDisplay(protocol){
    const p = Math.round(protocol || 0);
    controlProtocolLocal = (p === 1 || p === 2) ? p : 0;
    const select = document.getElementById('protocolSelect');
    if (select) select.value = String(controlProtocolLocal);
    if (typeof btnArm !== 'undefined') {
        btnArm.disabled = !wifiControlActive();
        btnArm.title = wifiControlActive() ? '' : 'Arm from CH5 on the ESP-NOW radio';
    }
    joystick.style.opacity = wifiControlActive() ? '1' : '0.52';
    joystick.style.borderColor = wifiControlActive() ? 'var(--accent)' : 'rgba(255,216,107,0.55)';
    if (!wifiControlActive()) {
        dragging = false;
        stick.style.left = '70px';
        stick.style.top = '150px';
        currentSteer = 0.0;
        currentThrottle = 0.0;
        controlsReleased = true;
        if (document.getElementById('motionSteerNotice')) {
            document.getElementById('motionSteerNotice').style.display = 'block';
            document.getElementById('motionSteerNotice').innerText = controlProtocolLocal === 2
                ? 'Joystick locked: ELRS radio controls flight'
                : 'Joystick locked: ESP-NOW radio controls flight';
        }
    } else if (!motionSteeringEnabledLocal && document.getElementById('motionSteerNotice')) {
        document.getElementById('motionSteerNotice').style.display = 'none';
        document.getElementById('motionSteerNotice').innerText = 'Steering locked: phone tilt controls turning';
    }
    if (window._channelMappingBar) {
        window._channelMappingBar.style.display = wifiControlActive() ? 'none' : 'flex';
    }
}

function nextControlSeq(){
    const nowSeq = Date.now() & 0x7fffffff;
    controlSeq = Math.max((controlSeq + 1) >>> 0, nowSeq);
    if (controlSeq === 0) controlSeq = 1;
    return controlSeq;
}

function sendControlPacket(steer, throttle){
    if (!wifiControlActive()) return;
    if (joystickSteeringLocked) steer = 0.0;
    const seq = nextControlSeq();
    if (ws && ws.readyState===1) {
        ws.send(`ctl ${seq} ${steer} ${throttle}`);
    } else {
        fetch(`/control?seq=${seq}&x=${steer}&y=${throttle}`).catch(()=>{});
    }
}

joystick.addEventListener("pointerdown", e => {
    e.preventDefault();
    if (!wifiControlActive()) return;
    if (!controlsArmed) return;
    dragging = true;
    controlsReleased = false;
    releaseGuardUntil = 0;
    joystickSteerFilterInit = false;
    if (joystick.setPointerCapture) joystick.setPointerCapture(e.pointerId);
});
window.addEventListener("pointermove", function(e){
    if(!dragging) return;
    e.preventDefault();
    let rect = joystick.getBoundingClientRect();
    let x = e.clientX - rect.left;
    let y = e.clientY - rect.top;

    let dx = x - centerX;

    // ===== THROTTLE RANGE =====
    let dy = y - 190;

    if(dy < -160) dy = -160;
    if(dy > 0) dy = 0;

    // ===== STEERING LIMIT =====
    let maxSteer = 80;

    if (motionSteeringEnabledLocal || joystickSteeringLocked) {
        dx = 0;
    } else {
        if(dx > maxSteer) dx = maxSteer;
        if(dx < -maxSteer) dx = -maxSteer;
    }

    stick.style.left = (centerX + dx - 40) + "px";
    stick.style.top = (150 + dy) + "px";

    let rawSteer = joystickSteeringLocked ? 0.0 : (motionSteeringEnabledLocal ? latestMotionSteer : (dx / maxSteer));
    let throttle = (-dy) / 160.0;
    let steer = rawSteer;
    if (!motionSteeringEnabledLocal) {
        let filterNow = Date.now();
        if (!joystickSteerFilterInit) {
            filteredJoystickSteer = currentSteer;
            joystickSteerFilterInit = true;
            joystickSteerLastMs = filterNow;
        }
        const filterDt = Math.max(1, filterNow - joystickSteerLastMs);
        joystickSteerLastMs = filterNow;
        const steerAlpha = filterDt / (joystickSteerFilterMs + filterDt);
        filteredJoystickSteer += steerAlpha * (rawSteer - filteredJoystickSteer);
        steer = Math.max(-1, Math.min(1, filteredJoystickSteer));
    }

    // update current values used by heartbeat
    currentSteer = steer;
    currentThrottle = throttle;
    controlsReleased = false;

    // rate-limited immediate send while actively dragging — use HTTP when WS unavailable
    let now = Date.now();
    if(!window.lastSendTime) window.lastSendTime = 0;
    if(now - window.lastSendTime > 20) {
        window.lastSendTime = now;
        sendControlPacket(steer, throttle);
    }
});

// Periodic heartbeat to send current control values even when static
setInterval(function(){
    if (!wifiControlActive()) return;
    if (!controlsArmed) return;
    if (Date.now() < releaseGuardUntil) return;
    if (dragging) return;
    if (controlsReleased && Math.abs(currentSteer) < 0.05 && Math.abs(currentThrottle) < 0.05) return;
    // send at low rate to ensure server keeps state fresh — use HTTP when WS unavailable
    sendControlPacket(currentSteer, currentThrottle);
}, 40);

function resetJoystick(e){
        if (!dragging) return;
        if (e && e.preventDefault) e.preventDefault();
        dragging=false; stick.style.left='70px'; stick.style.top='150px';
        // zero local state immediately
        currentSteer = 0.0; currentThrottle = 0.0;
        filteredJoystickSteer = 0.0;
        joystickSteerFilterInit = false;
        controlsReleased = true;
        releaseGuardUntil = Date.now() + 260;
        window.lastSendTime = Date.now();
        const seq = nextControlSeq();
        if (wifiControlActive()) {
            if (ws && ws.readyState===1) ws.send(`stop ${seq}`);
            fetch(`/stop?seq=${seq}`).catch(()=>{});
        }
    }
window.addEventListener("pointerup", resetJoystick); window.addEventListener("pointercancel", resetJoystick);

    // Telemetry poller ~8Hz
// WebSocket connection for telemetry and control
var ws = null;
let ledUiMode = 0;
let ledUiBrightness = 10;
let ledUiSpeed = 5;
function updateBatteryDisplay(data){
    if (typeof data.battery === 'undefined') return;
    let wrap = document.getElementById('batteryWrap');
    let fill = document.getElementById('batteryFill');
    let pctEl = document.getElementById('batteryPctOverlay');
    let voltEl = document.getElementById('batteryVolt');
    let warningEl = document.getElementById('batteryWarningText');
    if (fill && !fill.dataset.ready) {
        fill.innerHTML = '';
        for (let i = 0; i < 20; i++) {
            const seg = document.createElement('span');
            seg.className = 'battery-seg';
            fill.appendChild(seg);
        }
        fill.dataset.ready = '1';
    }
    const setBatterySegments = (count, color)=>{
        if (!fill) return;
        Array.from(fill.children).forEach((seg, idx)=>{
            seg.classList.toggle('active', idx < count);
            seg.style.background = idx < count ? color : 'rgba(255,255,255,0.08)';
            seg.style.boxShadow = idx < count ? ('0 0 7px ' + color) : 'none';
        });
    };
    const topInput = document.getElementById('batteryTopOhms');
    const bottomInput = document.getElementById('batteryBottomOhms');
    if (typeof data.batteryTopOhms !== 'undefined' && topInput && document.activeElement !== topInput) topInput.value = Math.round(data.batteryTopOhms / 1000.0);
    if (typeof data.batteryBottomOhms !== 'undefined' && bottomInput && document.activeElement !== bottomInput) bottomInput.value = Math.round(data.batteryBottomOhms / 1000.0);
    if (typeof data.batteryCells !== 'undefined' && document.getElementById('batteryCells')) document.getElementById('batteryCells').value = String(data.batteryCells);
    if (typeof data.batteryChemistry !== 'undefined' && document.getElementById('batteryChem')) document.getElementById('batteryChem').value = String(data.batteryChemistry);
    if (typeof data.batteryEnabled !== 'undefined' && document.getElementById('batteryEnable')) document.getElementById('batteryEnable').checked = !!data.batteryEnabled;
    if (typeof data.batteryFailsafeEnabled !== 'undefined' && document.getElementById('batteryFailsafeEnable')) document.getElementById('batteryFailsafeEnable').checked = !!data.batteryFailsafeEnabled;
    if (typeof data.batteryFailsafeCutoff !== 'undefined' && document.getElementById('batteryFailsafeCutoff') && document.activeElement !== document.getElementById('batteryFailsafeCutoff')) document.getElementById('batteryFailsafeCutoff').value = Number(data.batteryFailsafeCutoff || 0).toFixed(2);
    if (data.batteryEnabled === 0){
        if (wrap) {
            wrap.style.setProperty('--battery-color', '#555555');
            wrap.classList.remove('critical');
        }
        setBatterySegments(0, '#555555');
        if (pctEl) pctEl.innerText='OFF';
        if (voltEl) voltEl.innerText='OFF';
        if (warningEl){ warningEl.style.display='none'; warningEl.innerText=''; }
        return;
    }
    if (data.batteryAdcSaturated){
        if (wrap) {
            wrap.style.setProperty('--battery-color', '#ff4444');
            wrap.classList.remove('critical');
        }
        setBatterySegments(20, '#ff4444');
        if (pctEl) pctEl.innerText='ADC';
        if (voltEl) voltEl.innerText='SAT';
        if (warningEl){ warningEl.style.display='block'; warningEl.innerText='ADC Saturated'; }
        return;
    }
    let battV = data.battery || 0.0;
    let battPct = Math.max(0, Math.min(1, data.batteryPct || 0));
    let low = !!data.batteryLow;
    let critical = !!data.batteryCritical;
    let servoFailsafe = !!data.batteryServoFailsafe;
    let outline = (low || servoFailsafe) ? '#ff4444' : '#00ff66';
    if (wrap) {
        wrap.style.setProperty('--battery-color', outline);
        wrap.classList.toggle('critical', critical || servoFailsafe);
    }
    const segments = Math.max(0, Math.min(20, Math.ceil(battPct * 20)));
    setBatterySegments(segments, outline);
    if (warningEl) {
        warningEl.style.display = (low || servoFailsafe) ? 'block' : 'none';
        warningEl.innerText = servoFailsafe ? 'Servo Failsafe' : (critical ? 'Critically Low Voltage' : (low ? 'Low Voltage' : ''));
    }
    if (pctEl) pctEl.innerText=Math.round(battPct*100)+'%';
    if (voltEl) voltEl.innerText=battV.toFixed(2)+'V';
}
function updateThrottleDisplay(data){
    let throttle = Math.max(0, Math.min(1, data.throttle || 0));
    const gauge = document.getElementById('throttleFill');
    const pct = document.getElementById('throttlePctOverlay');
    if (gauge) {
        const freq = Math.max(0, data.frequency || 0);
        const maxDeg = Math.max(1, data.maxFlapDegrees || 90);
        const amplitude = Math.max(0, Math.min(maxDeg, data.amplitude || 0));
        const ampNorm = Math.max(0, Math.min(1, amplitude / maxDeg));
        const duration = freq > 0.05 ? Math.max(0.06, Math.min(0.7, 0.5 / freq)) : 1.2;
        const wingOpenScale = 0.78 + ampNorm * 0.24;
        const wingClosedScale = 0.60 - ampNorm * 0.28;
        const needle = -130 + (throttle * 260);
        gauge.style.setProperty('--flap-duration', duration.toFixed(3) + 's');
        gauge.style.setProperty('--wing-open-scale', wingOpenScale.toFixed(3));
        gauge.style.setProperty('--wing-closed-scale', wingClosedScale.toFixed(3));
        gauge.style.setProperty('--throttle-needle', needle.toFixed(1) + 'deg');
        gauge.classList.toggle('butterfly-idle', throttle <= 0.01 || freq <= 0.05);
    }
    if (pct) pct.innerText = Math.round(throttle * 100) + '%';
}
function updateAmplitudeDisplay(data){
    if (typeof data.maxFlapDegrees !== 'undefined') window._maxFlapDegrees=data.maxFlapDegrees;
    const lowerDegrees = (typeof data.lowerAmplitudeDegrees !== 'undefined') ? data.lowerAmplitudeDegrees : ((data.lowerAmplitudeScale || data.amplitudeScale || 0) * 90);
    const upperDegrees = (typeof data.upperAmplitudeDegrees !== 'undefined') ? data.upperAmplitudeDegrees : ((data.upperAmplitudeScale || data.amplitudeScale || 0) * 90);
    if (window._lowerAmpSlider && typeof lowerDegrees !== 'undefined') {
        const deg = Math.round(lowerDegrees);
        window._lowerAmpSlider.value=deg;
        if (window._lowerAmpVal) window._lowerAmpVal.innerText=deg+'\u00B0';
    }
    if (window._upperAmpSlider && typeof upperDegrees !== 'undefined') {
        const deg = Math.round(upperDegrees);
        window._upperAmpSlider.value=deg;
        if (window._upperAmpVal) window._upperAmpVal.innerText=deg+'\u00B0';
    }
    window._ampInitialized=true;
}
function updateArmDisplay(mode){
    currentModeName = mode || 'UNKNOWN';
    controlsArmed = mode === 'ARMED';
    const watermark = document.getElementById('armWatermark');
    if (watermark) {
        watermark.innerText = controlsArmed ? 'ARMED' : 'DISARMED';
        watermark.style.color = controlsArmed ? '#00ff66' : '#ff4444';
    }
    if (document.getElementById('modeText')) document.getElementById('modeText').innerText = mode || 'UNKNOWN';
    if (typeof btnArm !== 'undefined') {
        btnArm.innerText = controlsArmed ? 'Disarm' : 'Arm';
        btnArm.classList.toggle('active', controlsArmed);
    }
    if (typeof btnTest !== 'undefined') {
        btnTest.classList.toggle('active', mode === 'PERCH');
        btnTest.innerText = mode === 'PERCH' ? 'Perch' : 'Static';
    }
}
function formatDuration(ms){
    let total = Math.max(0, Math.floor((ms||0)/1000));
    let h = Math.floor(total/3600);
    let m = Math.floor((total%3600)/60);
    let s = total%60;
    if (h > 0) return h + ':' + String(m).padStart(2,'0') + ':' + String(s).padStart(2,'0');
    return m + ':' + String(s).padStart(2,'0');
}
function formatDegrees(value){
    return Math.round(Number(value) || 0) + '\u00B0';
}
function updateTimerDisplay(data){
    if (document.getElementById('currentFlightTimer') && typeof data.currentFlightMs !== 'undefined') document.getElementById('currentFlightTimer').innerText = formatDuration(data.currentFlightMs);
    if (document.getElementById('flightTimer') && typeof data.flightMs !== 'undefined') document.getElementById('flightTimer').innerText = formatDuration(data.flightMs);
    if (document.getElementById('recordTimer') && typeof data.recordMs !== 'undefined') document.getElementById('recordTimer').innerText = formatDuration(data.recordMs);
    if (document.getElementById('uptimeTimer') && typeof data.uptimeMs !== 'undefined') document.getElementById('uptimeTimer').innerText = formatDuration(data.uptimeMs);
    if (window._steerGainSlider && typeof data.steeringDiffGain !== 'undefined') {
        let gainPct = Math.round((data.steeringDiffGain || 0) * 100);
        window._steerGainSlider.value = gainPct;
        if (window._steerGainVal) window._steerGainVal.innerText = gainPct + '%';
    }
}
function updateHoverBiasDisplay(data){
    const phoneTiltRow = document.getElementById('phoneTiltRow');
    const tiltEnabled = !!data.hoverBiasEnabled;
    if (phoneTiltRow) phoneTiltRow.style.display = tiltEnabled ? '' : 'none';
    if (document.getElementById('tHoverBias') && typeof data.hoverBias !== 'undefined') document.getElementById('tHoverBias').innerText = Math.round((data.hoverBias||0)*100) + '%';
    if (document.getElementById('tNeutral') && typeof data.currentNeutralAngle !== 'undefined') document.getElementById('tNeutral').innerText = formatDegrees(data.currentNeutralAngle);
    if (tiltEnabled && document.getElementById('tPhonePitch') && typeof data.hoverDevicePitch !== 'undefined') document.getElementById('tPhonePitch').innerText = formatDegrees(data.hoverDevicePitch);
    if (window._hoverBiasEnable && typeof data.hoverBiasEnabled !== 'undefined') {
        window._hoverBiasEnable.checked = !!data.hoverBiasEnabled;
        hoverBiasEnabledLocal = !!data.hoverBiasEnabled;
    }
    if (window._hoverForwardSlider && typeof data.hoverForwardNeutral !== 'undefined') {
        window._hoverForwardSlider.value = Math.round(data.hoverForwardNeutral);
        if (window._hoverForwardVal) window._hoverForwardVal.innerText = Math.round(data.hoverForwardNeutral) + '\u00B0';
    }
    if (window._hoverNeutralSlider && typeof data.hoverNeutral !== 'undefined') {
        window._hoverNeutralSlider.value = Math.round(data.hoverNeutral);
        if (window._hoverNeutralVal) window._hoverNeutralVal.innerText = Math.round(data.hoverNeutral) + '\u00B0';
    }
    if (typeof updateNeutralPitchUiMode === 'function') updateNeutralPitchUiMode();
}
function updateLedDisplay(data){
    const panel = document.getElementById('controllerPanel');
    if (typeof data.ledMode !== 'undefined') {
        const names = ['OFF', 'Solid', 'Fade', 'Strobe'];
        const mode = Math.max(0, Math.min(3, Math.round(data.ledMode || 0)));
        ledUiMode = mode;
        if (window._ledButton) window._ledButton.innerText = names[mode];
        if (window._ledModeText) window._ledModeText.innerText = 'Mode - ' + mode;
    }
    if (window._ledBrightnessSlider && typeof data.ledBrightness !== 'undefined') {
        ledUiBrightness = Math.round(data.ledBrightness);
        window._ledBrightnessSlider.value = ledUiBrightness;
        if (window._ledBrightnessVal) window._ledBrightnessVal.innerText = ledUiBrightness;
    }
    if (window._ledSpeedSlider && typeof data.ledSpeed !== 'undefined') {
        ledUiSpeed = Math.round(data.ledSpeed);
        window._ledSpeedSlider.value = ledUiSpeed;
        if (window._ledSpeedVal) window._ledSpeedVal.innerText = ledUiSpeed;
    }
    if (panel) {
        panel.classList.remove('led-glow-solid','led-glow-fade','led-glow-strobe');
        const strength = Math.max(15, Math.min(80, ledUiBrightness * 8));
        const glow = 5 + ledUiBrightness * 1.8;
        const speedSeconds = Math.max(0.18, 1.65 - ledUiSpeed * 0.13);
        panel.style.setProperty('--panel-led-strength', strength + '%');
        panel.style.setProperty('--panel-led-glow', glow.toFixed(1) + 'px');
        panel.style.setProperty('--panel-led-speed', speedSeconds.toFixed(2) + 's');
        if (ledUiMode === 1) panel.classList.add('led-glow-solid');
        else if (ledUiMode === 2) panel.classList.add('led-glow-fade');
        else if (ledUiMode === 3) panel.classList.add('led-glow-strobe');
    }
}
function updateFlightProfileDisplay(data){
    if (typeof data.controlProtocol !== 'undefined') updateProtocolDisplay(data.controlProtocol);
    if (typeof data.steeringLock !== 'undefined') {
        joystickSteeringLocked = !!data.steeringLock;
        if (window._steeringLockButton) {
            window._steeringLockButton.classList.toggle('active', joystickSteeringLocked);
            window._steeringLockButton.innerText = joystickSteeringLocked ? 'Locked' : 'Unlocked';
        }
        if (joystickSteeringLocked) currentSteer = 0.0;
    }
    if (typeof data.flightProfileName !== 'undefined') {
        if (window._profileButton) window._profileButton.innerText = data.flightProfileName;
        if (window._profileNotice) window._profileNotice.innerText = 'Adjusting: ' + data.flightProfileName;
    }
    if (typeof data.waveform !== 'undefined') {
        const sine = Math.round(data.waveform || 0) === 1;
        if (window._waveformSwitch) window._waveformSwitch.checked = sine;
        if (window._waveformVal) window._waveformVal.innerText = sine ? 'Sine' : 'Triangle';
        if (window._waveButton) window._waveButton.innerText = sine ? 'Sine' : 'Triangle';
    }
    if (typeof data.servoMinUs !== 'undefined') {
        const minUs = Math.round(data.servoMinUs || 800);
        if (window._servoMinSlider) window._servoMinSlider.value = minUs;
        if (window._servoMinVal) window._servoMinVal.innerText = minUs + ' us';
    }
    if (typeof data.servoMaxUs !== 'undefined') {
        const maxUs = Math.round(data.servoMaxUs || 2200);
        if (window._servoMaxSlider) window._servoMaxSlider.value = maxUs;
        if (window._servoMaxVal) window._servoMaxVal.innerText = maxUs + ' us';
    }
    if (typeof data.servoRefreshHz !== 'undefined' && window._servoHzSelect) {
        window._servoHzSelect.value = String(Math.round(data.servoRefreshHz || 50));
    }
    if (typeof data.frequencyCap !== 'undefined') {
        const cap = Math.max(2, Math.min(6, Number(data.frequencyCap || 5)));
        if (window._flapFreqSlider && document.activeElement !== window._flapFreqSlider) window._flapFreqSlider.value = cap.toFixed(1);
        if (window._flapFreqVal) window._flapFreqVal.innerText = cap.toFixed(1) + ' Hz';
    }
    const pinMap = [
        ['leftServoPin', '_leftServoPinInput'],
        ['rightServoPin', '_rightServoPinInput'],
        ['ledPin', '_ledPinInput'],
        ['batteryPin', '_batteryPinInput'],
        ['batteryWarningLedPin', '_batteryWarningLedPinInput'],
        ['statusLedPin', '_statusLedPinInput'],
        ['elrsRxPin', '_elrsRxPinInput'],
        ['elrsTxPin', '_elrsTxPinInput']
    ];
    pinMap.forEach(pair=>{
        const key = pair[0];
        const input = window[pair[1]];
        if (input && typeof data[key] !== 'undefined' && document.activeElement !== input) {
            input.value = Math.round(data[key]);
        }
    });
    if (window._channelMapSelects) {
        for (let i = 0; i < 8; i++) {
            const select = window._channelMapSelects[i];
            const key = 'channelMap' + (i + 1);
            if (select && typeof data[key] !== 'undefined' && document.activeElement !== select) {
                select.value = String(Math.round(data[key]));
            }
        }
    }
}
function updateElrsDisplay(data){
    void data;
}
function initWebSocket(){
    var proto = (location.protocol === 'https:') ? 'wss://' : 'ws://';
    var port = '81';
    ws = new WebSocket(proto + location.hostname + ':' + port + '/');
    ws.onopen = function(){ console.log('WS open'); };
    ws.onclose = function(){ console.log('WS closed'); setTimeout(initWebSocket, 1000); };
    ws.onmessage = function(evt){
        try{ var data = JSON.parse(evt.data); }
        catch(e){ console.log('WS parse err', e); return; }
        updateThrottleDisplay(data);
        updateBatteryDisplay(data);
        document.getElementById('tAmp').innerText = formatDegrees(data.amplitude);
        updateAmplitudeDisplay(data);
        updateTimerDisplay(data);
        updateHoverBiasDisplay(data);
        updateLedDisplay(data);
        updateFlightProfileDisplay(data);
        updateElrsDisplay(data);
        if (document.getElementById('tPilot')) document.getElementById('tPilot').innerText = (data.pilot||0).toFixed(2);
        updateArmDisplay(data.mode || 'UNKNOWN');
        if (document.getElementById('leftTrimVal') && typeof data.leftTrim !== 'undefined') document.getElementById('leftTrimVal').innerText = data.leftTrim;
        if (document.getElementById('rightTrimVal') && typeof data.rightTrim !== 'undefined') document.getElementById('rightTrimVal').innerText = data.rightTrim;
            if (typeof data.steeringTrim !== 'undefined') {
                if (document.getElementById('steerTrimVal')) document.getElementById('steerTrimVal').innerText = (data.steeringTrim||0).toFixed(1);
                if (window.steerTrimSlider) { window.steerTrimSlider.value = Math.round((data.steeringTrim||0)*10); }
            }
            // Ensure the pitch UI is driven only by the authoritative servo center value
            if (typeof data.servoPitch !== 'undefined') {
                if (window._pitchSlider) window._pitchSlider.value = Math.round((data.servoPitch||90));
                if (typeof updateNeutralPitchUiMode === 'function') updateNeutralPitchUiMode();
            }
        if (typeof updateButtonStates === 'function') updateButtonStates(data.mode || 'UNKNOWN');
    };
}
// WebSocket client disabled — prefer HTTP polling/control for reliability
initWebSocket();

// Also fetch a single telemetry snapshot as an HTTP fallback to initialize the UI
fetch('/telemetry').then(r=>r.json()).then(function(data){
    try{
        updateThrottleDisplay(data);
        updateBatteryDisplay(data);
        if (document.getElementById('tAmp')) document.getElementById('tAmp').innerText = formatDegrees(data.amplitude);
        updateAmplitudeDisplay(data);
        updateTimerDisplay(data);
        updateHoverBiasDisplay(data);
        updateLedDisplay(data);
        updateFlightProfileDisplay(data);
        updateElrsDisplay(data);
        if (typeof data.servoPitch !== 'undefined') {
            if (window._pitchSlider) window._pitchSlider.value = Math.round((data.servoPitch||90));
            if (typeof updateNeutralPitchUiMode === 'function') updateNeutralPitchUiMode();
        }
        if (document.getElementById('tPilot')) document.getElementById('tPilot').innerText = (data.pilot||0).toFixed(2);
        updateArmDisplay(data.mode || 'UNKNOWN');
        if (document.getElementById('leftTrimVal') && typeof data.leftTrim !== 'undefined') document.getElementById('leftTrimVal').innerText = data.leftTrim;
        if (document.getElementById('rightTrimVal') && typeof data.rightTrim !== 'undefined') document.getElementById('rightTrimVal').innerText = data.rightTrim;
            if (typeof data.steeringTrim !== 'undefined') {
                if (document.getElementById('steerTrimVal')) document.getElementById('steerTrimVal').innerText = (data.steeringTrim||0).toFixed(1);
                if (window.steerTrimSlider) { window.steerTrimSlider.value = Math.round((data.steeringTrim||0)*10); }
            }
        if (typeof updateButtonStates === 'function') updateButtonStates(data.mode || 'UNKNOWN');
    } catch(e){ console.log('telemetry fetch parse err', e); }
}).catch(()=>{/* ignore fetch errors */});

// Poll telemetry periodically when WebSocket is not connected so UI stays live
let telemetryFetchPending = false;
setInterval(function(){
    if (ws && ws.readyState === 1) return;
    if (telemetryFetchPending) return;
    telemetryFetchPending = true;
    fetch('/telemetry').then(r=>r.json()).then(function(data){
        try{
            // full update for live fields
            updateThrottleDisplay(data);
            updateBatteryDisplay(data);
            if (document.getElementById('tAmp')) document.getElementById('tAmp').innerText = formatDegrees(data.amplitude);
            updateAmplitudeDisplay(data);
            updateTimerDisplay(data);
            updateHoverBiasDisplay(data);
            updateLedDisplay(data);
            updateFlightProfileDisplay(data);
            updateElrsDisplay(data);
            if (typeof data.servoPitch !== 'undefined') {
                if (window._pitchSlider) window._pitchSlider.value = Math.round((data.servoPitch||90));
                if (typeof updateNeutralPitchUiMode === 'function') updateNeutralPitchUiMode();
            }
            if (document.getElementById('tPilot')) document.getElementById('tPilot').innerText = (data.pilot||0).toFixed(2);
            updateArmDisplay(data.mode || 'UNKNOWN');
            if (document.getElementById('leftTrimVal') && typeof data.leftTrim !== 'undefined') document.getElementById('leftTrimVal').innerText = data.leftTrim;
            if (document.getElementById('rightTrimVal') && typeof data.rightTrim !== 'undefined') document.getElementById('rightTrimVal').innerText = data.rightTrim;
        } catch(e){ }
    }).catch(()=>{}).finally(()=>{ telemetryFetchPending = false; });
}, 250);

// Mode selector
function updateButtonStates(mode)
{
    updateArmDisplay(mode);
}

function setMode(v)
{
    // update UI immediately for responsiveness
    updateButtonStates(v === 1 ? 'ARMED' : (v === 2 ? 'PERCH' : 'DISARMED'));
    if (ws && ws.readyState===1) {
        ws.send(`mode ${v}`);
        // telemetry will update via WS
    } else {
        fetch(`/mode?m=${v}`).then(()=>{ fetch('/telemetry').then(r=>r.json()).then(data=>{ updateArmDisplay(data.mode || 'UNKNOWN'); }); });
    }
}

function toggleArm()
{
    if (!wifiControlActive()) return;
    setMode(controlsArmed ? 0 : 1);
}

function togglePerch()
{
    setMode(currentModeName === 'PERCH' ? 0 : 2);
}

// Arm/disarm and perch controls
const modeBar = document.createElement('div');
modeBar.style.display = 'flex'; modeBar.style.gap = '8px'; modeBar.style.marginTop = '18px'; modeBar.style.justifyContent = 'center'; modeBar.style.alignItems='flex-start'; modeBar.style.flexWrap='wrap';
function labeledControl(label, button){
    const wrap = document.createElement('div');
    wrap.style.display='flex'; wrap.style.flexDirection='column'; wrap.style.alignItems='center'; wrap.style.gap='4px';
    const text = document.createElement('div');
    text.style.color='var(--muted)'; text.style.fontSize='12px'; text.style.fontWeight='700';
    text.innerText = label;
    wrap.appendChild(text); wrap.appendChild(button);
    return wrap;
}
const btnArm = document.createElement('button'); btnArm.className='mode-button'; btnArm.innerText = 'Arm'; btnArm.onclick = toggleArm;
const btnProfile = document.createElement('button'); btnProfile.className='mode-button'; btnProfile.innerText='Cruise'; btnProfile.onclick=()=>{
    fetch('/cycleFlightProfile').then(()=>fetch('/telemetry')).then(r=>r.json()).then(data=>{ updateFlightProfileDisplay(data); updateAmplitudeDisplay(data); updateTimerDisplay(data); updateHoverBiasDisplay(data); if (typeof data.servoPitch !== 'undefined' && window._pitchSlider) window._pitchSlider.value=Math.round(data.servoPitch); updateNeutralPitchUiMode(); }).catch(()=>{});
};
const btnWave = document.createElement('button'); btnWave.className='mode-button'; btnWave.innerText='Triangle'; btnWave.onclick=()=>{
    fetch('/cycleWaveform').then(()=>fetch('/telemetry')).then(r=>r.json()).then(data=>{ updateFlightProfileDisplay(data); }).catch(()=>{});
};
const btnTest = document.createElement('button'); btnTest.className='mode-button'; btnTest.innerText = 'Static'; btnTest.onclick = togglePerch;
const btnLed = document.getElementById('ledTopButton'); btnLed.onclick = ()=>{
    fetch('/cycleLed').then(()=>fetch('/telemetry')).then(r=>r.json()).then(updateLedDisplay).catch(()=>{});
};
const recordResetButton = document.getElementById('recordResetButton');
if (recordResetButton) {
    recordResetButton.onclick = ()=>{
        if (!window.confirm('Clear Record Flight Time (RFT)? Select OK for Yes or Cancel for No.')) return;
        fetch('/resetFlightRecord').then(()=>fetch('/telemetry')).then(r=>r.json()).then(updateTimerDisplay).catch(()=>{});
    };
}
const ledModeText = document.getElementById('ledTopModeText');
window._ledButton = btnLed;
window._ledModeText = ledModeText;
window._profileButton = btnProfile;
window._waveButton = btnWave;
const protocolBox = document.createElement('div');
protocolBox.style.display='flex'; protocolBox.style.flexDirection='row'; protocolBox.style.alignItems='flex-end'; protocolBox.style.justifyContent='center'; protocolBox.style.gap='10px'; protocolBox.style.margin='8px auto 2px auto';
protocolBox.style.width='100%';
const protocolColumn = document.createElement('div');
protocolColumn.style.display='flex'; protocolColumn.style.flexDirection='column'; protocolColumn.style.alignItems='center'; protocolColumn.style.gap='4px';
const protocolLabel = document.createElement('div');
protocolLabel.style.color='var(--muted)'; protocolLabel.style.fontSize='12px'; protocolLabel.style.fontWeight='700';
protocolLabel.innerText='Protocol';
const protocolSelect = document.createElement('select');
protocolSelect.id='protocolSelect';
protocolSelect.style.background='var(--panel)'; protocolSelect.style.color='var(--text)'; protocolSelect.style.border='2px solid var(--accent)'; protocolSelect.style.borderRadius='8px'; protocolSelect.style.padding='7px 10px'; protocolSelect.style.fontWeight='700'; protocolSelect.style.minWidth='150px';
protocolSelect.innerHTML='<option value="0">WiFi (App)</option><option value="1">ESP-NOW</option><option value="2">ELRS</option>';
protocolSelect.onchange=()=>{
    const p = Math.max(0, Math.min(2, parseInt(protocolSelect.value || '0', 10)));
    if (p !== 0 && wifiControlActive()) {
        const seq = nextControlSeq();
        if (ws && ws.readyState===1) ws.send(`stop ${seq}`);
        fetch(`/stop?seq=${seq}`).catch(()=>{});
    }
    updateProtocolDisplay(p);
    fetch(`/setControlProtocol?p=${p}`).then(()=>fetch('/telemetry')).then(r=>r.json()).then(data=>{ updateFlightProfileDisplay(data); updateArmDisplay(data.mode || 'UNKNOWN'); }).catch(()=>{});
};
protocolColumn.appendChild(protocolLabel);
protocolColumn.appendChild(protocolSelect);
const steeringLockColumn = document.createElement('div');
steeringLockColumn.style.display='flex'; steeringLockColumn.style.flexDirection='column'; steeringLockColumn.style.alignItems='center'; steeringLockColumn.style.gap='4px';
const steeringLockLabel = document.createElement('div');
steeringLockLabel.style.color='var(--muted)'; steeringLockLabel.style.fontSize='12px'; steeringLockLabel.style.fontWeight='700';
steeringLockLabel.innerText='Joystick Lock';
const steeringLockButton = document.createElement('button');
steeringLockButton.className='mode-button';
steeringLockButton.style.padding='7px 10px';
steeringLockButton.style.minWidth='86px';
steeringLockButton.innerText='Unlocked';
steeringLockButton.onclick=()=>{
    joystickSteeringLocked = !joystickSteeringLocked;
    currentSteer = 0.0;
    filteredJoystickSteer = 0.0;
    latestMotionSteer = 0.0;
    steeringLockButton.classList.toggle('active', joystickSteeringLocked);
    steeringLockButton.innerText = joystickSteeringLocked ? 'Locked' : 'Unlocked';
    fetch(`/setSteeringLock?enabled=${joystickSteeringLocked ? 1 : 0}`).catch(()=>{});
};
steeringLockColumn.appendChild(steeringLockLabel);
steeringLockColumn.appendChild(steeringLockButton);
protocolBox.appendChild(protocolColumn);
protocolBox.appendChild(steeringLockColumn);
window._steeringLockButton = steeringLockButton;
modeBar.appendChild(labeledControl('Arm', btnArm));
modeBar.appendChild(labeledControl('Mode', btnProfile));
modeBar.appendChild(labeledControl('Wave', btnWave));
modeBar.appendChild(labeledControl('Idle Mode', btnTest));
document.querySelector('.container').appendChild(modeBar);

// Tabs: Controller / Preferences
const tabBar = document.createElement('div'); tabBar.style.display='flex'; tabBar.style.gap='8px'; tabBar.style.justifyContent='center'; tabBar.style.marginTop='10px';
const tabController = document.createElement('button'); tabController.className='mode-button'; tabController.innerText='Controller';
const tabPrefs = document.createElement('button'); tabPrefs.className='mode-button'; tabPrefs.innerText='Preferences';
tabController.onclick = ()=>showTab('controller'); tabPrefs.onclick = ()=>showTab('prefs');
tabBar.appendChild(tabController); tabBar.appendChild(tabPrefs);
document.querySelector('.container').insertBefore(tabBar, modeBar);

// Preferences panel (hidden by default)
const prefsDiv = document.createElement('div'); prefsDiv.style.display='none'; prefsDiv.style.marginTop='6px'; prefsDiv.style.textAlign='center'; prefsDiv.style.height='calc(100vh - 72px)'; prefsDiv.style.overflowY='auto'; prefsDiv.style.webkitOverflowScrolling='touch'; prefsDiv.style.overscrollBehavior='contain'; prefsDiv.style.touchAction='pan-y'; prefsDiv.style.padding='0 2px 150px 2px'; prefsDiv.style.boxSizing='border-box';
const profileNotice = document.createElement('div'); profileNotice.style.color='var(--text)'; profileNotice.style.fontSize='12px'; profileNotice.style.fontWeight='700'; profileNotice.style.margin='3px 0'; profileNotice.innerText='Adjusting: Cruise';
prefsDiv.appendChild(profileNotice);
window._profileNotice = profileNotice;

function makeVoltageInput(label, id, value){
    const wrap = document.createElement('div'); wrap.className='slider-section';
    const title = document.createElement('div'); title.innerHTML='<strong>'+label+':</strong>';
    const row = document.createElement('div'); row.style.display='flex'; row.style.alignItems='center'; row.style.justifyContent='center'; row.style.gap='5px';
    const input = document.createElement('input'); input.type='number'; input.id=id; input.value=value; input.min='1'; input.step='1'; input.style.width='72px'; input.style.background='var(--panel)'; input.style.color='var(--text)'; input.style.border='2px solid var(--accent)'; input.style.borderRadius='8px'; input.style.padding='7px';
    const unit = document.createElement('span'); unit.style.color='var(--text)'; unit.style.fontWeight='700'; unit.innerHTML='K&Omega;';
    row.appendChild(input); row.appendChild(unit);
    wrap.appendChild(title); wrap.appendChild(row);
    return {wrap:wrap,input:input};
}
const battEnableWrap = document.createElement('div'); battEnableWrap.className='slider-section';
const battEnableLabel = document.createElement('label'); battEnableLabel.style.display='flex'; battEnableLabel.style.alignItems='center'; battEnableLabel.style.gap='8px'; battEnableLabel.style.color='var(--text)'; battEnableLabel.style.fontWeight='700';
const battEnable = document.createElement('input'); battEnable.type='checkbox'; battEnable.id='batteryEnable'; battEnable.checked=true;
battEnableLabel.appendChild(battEnable); battEnableLabel.appendChild(document.createTextNode('Enable Battery Voltage Measurement'));
battEnableWrap.appendChild(battEnableLabel);
const battTop = makeVoltageInput('Upper Resistor', 'batteryTopOhms', 49);
const battBottom = makeVoltageInput('Lower Resistor', 'batteryBottomOhms', 18);
const battCellsWrap = document.createElement('div'); battCellsWrap.className='slider-section';
const battCellsLabel = document.createElement('div'); battCellsLabel.innerHTML='<strong>Battery Cells:</strong>';
const battCells = document.createElement('select'); battCells.id='batteryCells'; battCells.style.background='var(--panel)'; battCells.style.color='var(--text)'; battCells.style.border='2px solid var(--accent)'; battCells.style.borderRadius='8px'; battCells.style.padding='7px 10px';
for(let i=1;i<=2;i++){ const o=document.createElement('option'); o.value=i; o.innerText=i+'S'; if(i===2)o.selected=true; battCells.appendChild(o); }
battCellsWrap.appendChild(battCellsLabel); battCellsWrap.appendChild(battCells);
const battChemWrap = document.createElement('div'); battChemWrap.className='slider-section';
const battChemLabel = document.createElement('div'); battChemLabel.innerHTML='<strong>Chemistry:</strong>';
const battChem = document.createElement('select'); battChem.id='batteryChem'; battChem.style.background='var(--panel)'; battChem.style.color='var(--text)'; battChem.style.border='2px solid var(--accent)'; battChem.style.borderRadius='8px'; battChem.style.padding='7px 10px';
battChem.innerHTML='<option value="0">LiPo</option><option value="1">LiHV</option>';
battChemWrap.appendChild(battChemLabel); battChemWrap.appendChild(battChem);
const battNote = document.createElement('div'); battNote.style.color='var(--muted)'; battNote.style.fontSize='12px'; battNote.style.margin='8px'; battNote.innerText='Upper resistor is battery positive to ADC. Lower resistor is ADC to ground.';
const battGrid = document.createElement('div'); battGrid.className='prefs-grid';
battGrid.appendChild(battTop.wrap); battGrid.appendChild(battBottom.wrap); battGrid.appendChild(battCellsWrap); battGrid.appendChild(battChemWrap);
const battSectionTitle = document.createElement('div'); battSectionTitle.className='slider-section'; battSectionTitle.style.fontWeight='700'; battSectionTitle.innerText='Voltage Divider';
const battFailsafeWrap = document.createElement('div'); battFailsafeWrap.className='slider-section';
const battFailsafeTitle = document.createElement('div'); battFailsafeTitle.style.fontWeight='700'; battFailsafeTitle.innerText='Battery Servo Failsafe';
const battFailsafeLabel = document.createElement('label'); battFailsafeLabel.style.display='flex'; battFailsafeLabel.style.alignItems='center'; battFailsafeLabel.style.gap='8px'; battFailsafeLabel.style.color='var(--text)'; battFailsafeLabel.style.fontWeight='700';
const battFailsafeEnable = document.createElement('input'); battFailsafeEnable.type='checkbox'; battFailsafeEnable.id='batteryFailsafeEnable';
battFailsafeLabel.appendChild(battFailsafeEnable); battFailsafeLabel.appendChild(document.createTextNode('Enable neutral-hold cutoff'));
const battFailsafeRow = document.createElement('div'); battFailsafeRow.style.display='flex'; battFailsafeRow.style.alignItems='center'; battFailsafeRow.style.justifyContent='center'; battFailsafeRow.style.gap='6px';
const battFailsafeCutoff = document.createElement('input'); battFailsafeCutoff.type='number'; battFailsafeCutoff.id='batteryFailsafeCutoff'; battFailsafeCutoff.value='7.00'; battFailsafeCutoff.min='0'; battFailsafeCutoff.max='30'; battFailsafeCutoff.step='0.01'; battFailsafeCutoff.style.width='88px'; battFailsafeCutoff.style.background='var(--panel)'; battFailsafeCutoff.style.color='var(--text)'; battFailsafeCutoff.style.border='2px solid var(--accent)'; battFailsafeCutoff.style.borderRadius='8px'; battFailsafeCutoff.style.padding='7px';
const battFailsafeUnit = document.createElement('span'); battFailsafeUnit.style.color='var(--text)'; battFailsafeUnit.style.fontWeight='700'; battFailsafeUnit.innerText='V cutoff';
battFailsafeRow.appendChild(battFailsafeCutoff); battFailsafeRow.appendChild(battFailsafeUnit);
const battFailsafeNote = document.createElement('div'); battFailsafeNote.style.color='var(--muted)'; battFailsafeNote.style.fontSize='12px'; battFailsafeNote.innerText='At or below cutoff, wing servos hold neutral and all servo movement commands are ignored.';
battFailsafeWrap.appendChild(battFailsafeTitle); battFailsafeWrap.appendChild(battFailsafeLabel); battFailsafeWrap.appendChild(battFailsafeRow); battFailsafeWrap.appendChild(battFailsafeNote);
function sendBatteryConfig(){
    const en = battEnable.checked ? 1 : 0;
    const fs = battFailsafeEnable.checked ? 1 : 0;
    fetch(`/setBatteryConfig?enabled=${en}&top=${battTop.input.value}&bottom=${battBottom.input.value}&cells=${battCells.value}&chem=${battChem.value}&failsafe=${fs}&cutoff=${battFailsafeCutoff.value}`).catch(()=>{});
}
[battEnable,battTop.input,battBottom.input,battCells,battChem,battFailsafeEnable,battFailsafeCutoff].forEach(el=>el.addEventListener('change', sendBatteryConfig));
// Left servo trim controls
const leftLbl = document.createElement('div'); leftLbl.style.margin='6px'; leftLbl.innerHTML = '<strong>Left Servo Trim</strong><br><span id="leftTrimVal">0</span>&deg;';
const leftMinus = document.createElement('button'); leftMinus.className='mode-button'; leftMinus.innerText='-'; leftMinus.onclick = ()=>{ fetch('/trim?side=L&d=-1').then(()=>fetch('/telemetry').then(r=>r.json()).then(d=>{ if (document.getElementById('leftTrimVal')) document.getElementById('leftTrimVal').innerText = d.leftTrim; })).catch(()=>{}); };
const leftPlus = document.createElement('button'); leftPlus.className='mode-button'; leftPlus.innerText='+'; leftPlus.onclick = ()=>{ fetch('/trim?side=L&d=1').then(()=>fetch('/telemetry').then(r=>r.json()).then(d=>{ if (document.getElementById('leftTrimVal')) document.getElementById('leftTrimVal').innerText = d.leftTrim; })).catch(()=>{}); };
const leftRow = document.createElement('div'); leftRow.style.display='flex'; leftRow.style.gap='8px'; leftRow.style.justifyContent='center'; leftRow.style.width='100%'; leftRow.appendChild(leftMinus); leftRow.appendChild(leftPlus);
leftMinus.style.flex = '1'; leftPlus.style.flex = '1'; leftMinus.style.minWidth='0'; leftPlus.style.minWidth='0';
prefsDiv.appendChild(leftLbl); prefsDiv.appendChild(leftRow);

// Right servo trim controls
const rightLbl = document.createElement('div'); rightLbl.style.margin='6px'; rightLbl.innerHTML = '<strong>Right Servo Trim</strong><br><span id="rightTrimVal">0</span>&deg;';
const rightMinus = document.createElement('button'); rightMinus.className='mode-button'; rightMinus.innerText='-'; rightMinus.onclick = ()=>{ fetch('/trim?side=R&d=-1').then(()=>fetch('/telemetry').then(r=>r.json()).then(d=>{ if (document.getElementById('rightTrimVal')) document.getElementById('rightTrimVal').innerText = d.rightTrim; })).catch(()=>{}); };
const rightPlus = document.createElement('button'); rightPlus.className='mode-button'; rightPlus.innerText='+'; rightPlus.onclick = ()=>{ fetch('/trim?side=R&d=1').then(()=>fetch('/telemetry').then(r=>r.json()).then(d=>{ if (document.getElementById('rightTrimVal')) document.getElementById('rightTrimVal').innerText = d.rightTrim; })).catch(()=>{}); };
const rightRow = document.createElement('div'); rightRow.style.display='flex'; rightRow.style.gap='8px'; rightRow.style.justifyContent='center'; rightRow.style.width='100%'; rightRow.appendChild(rightMinus); rightRow.appendChild(rightPlus);
rightMinus.style.flex = '1'; rightPlus.style.flex = '1'; rightMinus.style.minWidth='0'; rightPlus.style.minWidth='0';
prefsDiv.appendChild(rightLbl); prefsDiv.appendChild(rightRow);

// Steering Trim slider
const steerBar = document.createElement('div'); steerBar.className='slider-section';
const steerLabel = document.createElement('div'); steerLabel.innerHTML = '<strong>Steering Trim:</strong> <span id="steerTrimVal">0.0</span>';
const steerSlider = document.createElement('input'); steerSlider.type='range'; steerSlider.min=-50; steerSlider.max=50; steerSlider.value=0; steerSlider.step=1; steerSlider.style.width='220px'; steerSlider.id='steerTrimSlider';
const steerReset = document.createElement('button'); steerReset.className='mode-button'; steerReset.innerText='Reset';
steerReset.onclick=()=>{ if(window.confirm('Reset Steering Trim to neutral? Select OK for Yes or Cancel for No.')){ steerSlider.value=0; document.getElementById('steerTrimVal').innerText='0.0'; fetch('/setSteerTrim?s=0').catch(()=>{}); } };
const steerRow = document.createElement('div'); steerRow.style.display='flex'; steerRow.style.justifyContent='center'; steerRow.style.alignItems='center'; steerRow.style.gap='8px'; steerRow.appendChild(steerSlider); steerRow.appendChild(steerReset);
steerBar.appendChild(steerLabel); steerBar.appendChild(steerRow); prefsDiv.appendChild(steerBar);

// Steering slider value map: slider -50..50 -> -5.0 .. 5.0
steerSlider.addEventListener('input', ()=>{ let v = (steerSlider.value/10.0); if(document.getElementById('steerTrimVal')) document.getElementById('steerTrimVal').innerText = v.toFixed(1); });
steerSlider.addEventListener('change', ()=>{ let v = (steerSlider.value/10.0); fetch(`/setSteerTrim?s=${v}`).catch(()=>{}); });
window.steerTrimSlider = steerSlider;

// Steering differential gain slider for Direct Flap Mode
const steerGainBar = document.createElement('div'); steerGainBar.className='slider-section';
const steerGainLabel = document.createElement('div'); steerGainLabel.innerHTML = '<strong>Steering Gain:</strong> <span id="steerGainVal">100%</span>';
const steerGainSlider = document.createElement('input'); steerGainSlider.type='range'; steerGainSlider.min=0; steerGainSlider.max=250; steerGainSlider.value=100; steerGainSlider.step=5; steerGainSlider.style.width='220px'; steerGainSlider.id='steerGainSlider';
const steerGainReset = document.createElement('button'); steerGainReset.className='mode-button'; steerGainReset.innerText='Reset';
steerGainReset.onclick=()=>{ if(window.confirm('Reset Steering Gain to default? Select OK for Yes or Cancel for No.')){ steerGainSlider.value=100; document.getElementById('steerGainVal').innerText='100%'; fetch('/setSteerGain?g=100').catch(()=>{}); } };
const steerGainRow = document.createElement('div'); steerGainRow.style.display='flex'; steerGainRow.style.justifyContent='center'; steerGainRow.style.alignItems='center'; steerGainRow.style.gap='8px'; steerGainRow.appendChild(steerGainSlider); steerGainRow.appendChild(steerGainReset);
steerGainBar.appendChild(steerGainLabel); steerGainBar.appendChild(steerGainRow); prefsDiv.appendChild(steerGainBar);

let steerGainDebounce = null;
steerGainSlider.addEventListener('input', ()=>{
    document.getElementById('steerGainVal').innerText = steerGainSlider.value + '%';
    clearTimeout(steerGainDebounce);
    steerGainDebounce = setTimeout(()=>{
        fetch(`/setSteerGain?g=${steerGainSlider.value}`).catch(()=>{});
    }, 80);
});
steerGainSlider.addEventListener('change', ()=>{ fetch(`/setSteerGain?g=${steerGainSlider.value}`).catch(()=>{}); });

const ledBrightnessBar = document.createElement('div'); ledBrightnessBar.className='slider-section';
const ledBrightnessLabel = document.createElement('div'); ledBrightnessLabel.innerHTML = '<strong>LED Brightness:</strong> <span id="ledBrightnessVal">10</span>';
const ledBrightnessSlider = document.createElement('input'); ledBrightnessSlider.type='range'; ledBrightnessSlider.min=1; ledBrightnessSlider.max=10; ledBrightnessSlider.value=10; ledBrightnessSlider.step=1; ledBrightnessSlider.style.width='220px'; ledBrightnessSlider.id='ledBrightnessSlider';
const ledBrightnessReset = document.createElement('button'); ledBrightnessReset.className='mode-button'; ledBrightnessReset.innerText='Reset';
const ledBrightnessRow = document.createElement('div'); ledBrightnessRow.style.display='flex'; ledBrightnessRow.style.alignItems='center'; ledBrightnessRow.style.gap='8px'; ledBrightnessRow.appendChild(ledBrightnessSlider); ledBrightnessRow.appendChild(ledBrightnessReset);
ledBrightnessBar.appendChild(ledBrightnessLabel); ledBrightnessBar.appendChild(ledBrightnessRow); prefsDiv.appendChild(ledBrightnessBar);

const ledSpeedBar = document.createElement('div'); ledSpeedBar.className='slider-section';
const ledSpeedLabel = document.createElement('div'); ledSpeedLabel.innerHTML = '<strong>LED Effect Speed:</strong> <span id="ledSpeedVal">5</span>';
const ledSpeedSlider = document.createElement('input'); ledSpeedSlider.type='range'; ledSpeedSlider.min=1; ledSpeedSlider.max=10; ledSpeedSlider.value=5; ledSpeedSlider.step=1; ledSpeedSlider.style.width='220px'; ledSpeedSlider.id='ledSpeedSlider';
const ledSpeedReset = document.createElement('button'); ledSpeedReset.className='mode-button'; ledSpeedReset.innerText='Reset';
const ledSpeedRow = document.createElement('div'); ledSpeedRow.style.display='flex'; ledSpeedRow.style.alignItems='center'; ledSpeedRow.style.gap='8px'; ledSpeedRow.appendChild(ledSpeedSlider); ledSpeedRow.appendChild(ledSpeedReset);
ledSpeedBar.appendChild(ledSpeedLabel); ledSpeedBar.appendChild(ledSpeedRow); prefsDiv.appendChild(ledSpeedBar);

function sendLedConfig(){
    fetch(`/setLedConfig?brightness=${ledBrightnessSlider.value}&speed=${ledSpeedSlider.value}`).catch(()=>{});
}
ledBrightnessSlider.addEventListener('input', ()=>{ document.getElementById('ledBrightnessVal').innerText = ledBrightnessSlider.value; updateLedDisplay({ledBrightness:ledBrightnessSlider.value,ledSpeed:ledSpeedSlider.value}); sendLedConfig(); });
ledSpeedSlider.addEventListener('input', ()=>{ document.getElementById('ledSpeedVal').innerText = ledSpeedSlider.value; updateLedDisplay({ledBrightness:ledBrightnessSlider.value,ledSpeed:ledSpeedSlider.value}); sendLedConfig(); });
ledBrightnessReset.onclick=()=>{ if(window.confirm('Reset LED Brightness? Select OK for Yes or Cancel for No.')){ ledBrightnessSlider.value=10; document.getElementById('ledBrightnessVal').innerText='10'; updateLedDisplay({ledBrightness:10,ledSpeed:ledSpeedSlider.value}); sendLedConfig(); } };
ledSpeedReset.onclick=()=>{ if(window.confirm('Reset LED Effect Speed? Select OK for Yes or Cancel for No.')){ ledSpeedSlider.value=5; document.getElementById('ledSpeedVal').innerText='5'; updateLedDisplay({ledBrightness:ledBrightnessSlider.value,ledSpeed:5}); sendLedConfig(); } };
window._ledBrightnessSlider = ledBrightnessSlider;
window._ledBrightnessVal = document.getElementById('ledBrightnessVal');
window._ledSpeedSlider = ledSpeedSlider;
window._ledSpeedVal = document.getElementById('ledSpeedVal');

document.querySelector('.container').appendChild(prefsDiv);

// showTab will be defined after amplitude bars are created so DOM refs exist

function makeAmplitudeBar(title, sliderId, valueId, minDeg, maxDeg, defaultDeg, resetText, route){
    const bar = document.createElement('div'); bar.className='slider-section';
    const label = document.createElement('div'); label.style.color='var(--text)'; label.style.textAlign='center'; label.innerHTML='<strong>' + title + '</strong><br><span id="' + valueId + '">' + defaultDeg + '\u00B0</span>';
    const slider = document.createElement('input'); slider.type='range'; slider.min=minDeg; slider.max=maxDeg; slider.value=defaultDeg; slider.step=1; slider.id=sliderId;
    const value = label.querySelector('span');
    const reset = document.createElement('button'); reset.className='mode-button'; reset.innerText='Reset';
    reset.onclick=()=>{ if(window.confirm(resetText + '? Select OK for Yes or Cancel for No.')){ slider.value=defaultDeg; value.innerText=defaultDeg+'\u00B0'; fetch(route + '?deg=' + defaultDeg).catch(()=>{}); } };
    const row = document.createElement('div'); row.className='wide-slider-row'; row.appendChild(slider); row.appendChild(reset);
    bar.appendChild(label); bar.appendChild(row);
    return {bar:bar, slider:slider, value:value, route:route};
}

const lowerAmp = makeAmplitudeBar('Lower Amplitude', 'lowerAmpSlider', 'lowerAmpVal', 50, 90, 75, 'Reset Lower Amplitude to default', '/setLowerAmpScale');
const upperAmp = makeAmplitudeBar('Upper Amplitude', 'upperAmpSlider', 'upperAmpVal', 50, 90, 75, 'Reset Upper Amplitude to default', '/setUpperAmpScale');

const servoMinBar = document.createElement('div'); servoMinBar.className='slider-section';
const servoMinLabel = document.createElement('div'); servoMinLabel.style.color='var(--text)'; servoMinLabel.style.textAlign='center'; servoMinLabel.innerHTML='<strong>Servo Min Pulse:</strong> <span id="servoMinVal">500 us</span>';
const servoMinSlider = document.createElement('input'); servoMinSlider.type='range'; servoMinSlider.min=500; servoMinSlider.max=1200; servoMinSlider.value=500; servoMinSlider.step=100; servoMinSlider.style.width='220px'; servoMinSlider.id='servoMinSlider';
const servoMinReset = document.createElement('button'); servoMinReset.className='mode-button'; servoMinReset.innerText='Reset';
servoMinReset.onclick=()=>{ if(window.confirm('Reset Servo Min Pulse to default? Select OK for Yes or Cancel for No.')){ servoMinSlider.value=500; document.getElementById('servoMinVal').innerText='500 us'; sendServoOutputConfig(); } };
const servoMinRow = document.createElement('div'); servoMinRow.className='slider-row'; servoMinRow.appendChild(servoMinSlider); servoMinRow.appendChild(servoMinReset);
servoMinBar.appendChild(servoMinLabel); servoMinBar.appendChild(servoMinRow);

const servoMaxBar = document.createElement('div'); servoMaxBar.className='slider-section';
const servoMaxLabel = document.createElement('div'); servoMaxLabel.style.color='var(--text)'; servoMaxLabel.style.textAlign='center'; servoMaxLabel.innerHTML='<strong>Servo Max Pulse:</strong> <span id="servoMaxVal">2500 us</span>';
const servoMaxSlider = document.createElement('input'); servoMaxSlider.type='range'; servoMaxSlider.min=1800; servoMaxSlider.max=2500; servoMaxSlider.value=2500; servoMaxSlider.step=100; servoMaxSlider.style.width='220px'; servoMaxSlider.id='servoMaxSlider';
const servoMaxReset = document.createElement('button'); servoMaxReset.className='mode-button'; servoMaxReset.innerText='Reset';
servoMaxReset.onclick=()=>{ if(window.confirm('Reset Servo Max Pulse to default? Select OK for Yes or Cancel for No.')){ servoMaxSlider.value=2500; document.getElementById('servoMaxVal').innerText='2500 us'; sendServoOutputConfig(); } };
const servoMaxRow = document.createElement('div'); servoMaxRow.className='slider-row'; servoMaxRow.appendChild(servoMaxSlider); servoMaxRow.appendChild(servoMaxReset);
servoMaxBar.appendChild(servoMaxLabel); servoMaxBar.appendChild(servoMaxRow);

const servoHzBar = document.createElement('div'); servoHzBar.className='slider-section';
const servoHzLabel = document.createElement('div'); servoHzLabel.style.color='var(--text)'; servoHzLabel.style.textAlign='center'; servoHzLabel.innerHTML='<strong>Servo Frequency:</strong>';
const servoHzSelect = document.createElement('select'); servoHzSelect.id='servoHzSelect'; servoHzSelect.style.background='var(--panel)'; servoHzSelect.style.color='var(--text)'; servoHzSelect.style.border='2px solid var(--accent)'; servoHzSelect.style.borderRadius='8px'; servoHzSelect.style.padding='7px 10px'; servoHzSelect.style.width='100%';
[50,100,333].forEach(hz=>{ const o=document.createElement('option'); o.value=hz; o.innerText=hz + ' Hz'; if(hz===50)o.selected=true; servoHzSelect.appendChild(o); });
servoHzBar.appendChild(servoHzLabel); servoHzBar.appendChild(servoHzSelect);

const flapFreqBar = document.createElement('div'); flapFreqBar.className='slider-section';
const flapFreqLabel = document.createElement('div'); flapFreqLabel.style.color='var(--text)'; flapFreqLabel.style.textAlign='center'; flapFreqLabel.innerHTML='<strong>Flap Frequency:</strong> <span id="flapFreqVal">4.0 Hz</span>';
const flapFreqSlider = document.createElement('input'); flapFreqSlider.type='range'; flapFreqSlider.min=2; flapFreqSlider.max=6; flapFreqSlider.value=4; flapFreqSlider.step=0.1; flapFreqSlider.id='flapFreqSlider';
const flapFreqReset = document.createElement('button'); flapFreqReset.className='mode-button'; flapFreqReset.innerText='Reset';
flapFreqReset.onclick=()=>{ if(window.confirm('Reset Flap Frequency to default? Select OK for Yes or Cancel for No.')){ flapFreqSlider.value=4; document.getElementById('flapFreqVal').innerText='4.0 Hz'; fetch('/setFrequencyCap?hz=4').catch(()=>{}); } };
const flapFreqRow = document.createElement('div'); flapFreqRow.className='wide-slider-row'; flapFreqRow.appendChild(flapFreqSlider); flapFreqRow.appendChild(flapFreqReset);
flapFreqBar.appendChild(flapFreqLabel); flapFreqBar.appendChild(flapFreqRow);

function makePinInput(label, id, value){
    const wrap = document.createElement('div');
    wrap.style.display='grid';
    wrap.style.gridTemplateColumns='1fr 62px';
    wrap.style.alignItems='center';
    wrap.style.gap='6px';
    wrap.style.color='var(--text)';
    wrap.style.fontSize='12px';
    const text = document.createElement('label');
    text.htmlFor = id;
    text.style.textAlign='left';
    text.innerText = label;
    const input = document.createElement('input');
    input.type='number';
    input.id=id;
    input.min='0';
    input.max='48';
    input.step='1';
    input.value=value;
    input.style.width='62px';
    input.style.boxSizing='border-box';
    input.style.background='var(--panel)';
    input.style.color='var(--text)';
    input.style.border='2px solid var(--accent)';
    input.style.borderRadius='8px';
    input.style.padding='6px';
    wrap.appendChild(text);
    wrap.appendChild(input);
    return {wrap:wrap,input:input};
}
const hardwarePinsBar = document.createElement('div'); hardwarePinsBar.className='slider-section';
const hardwarePinsTitle = document.createElement('div'); hardwarePinsTitle.innerHTML='<strong>Hardware Pins</strong>';
const hardwarePinsNote = document.createElement('div'); hardwarePinsNote.style.color='var(--muted)'; hardwarePinsNote.style.fontSize='11px'; hardwarePinsNote.style.lineHeight='1.45'; hardwarePinsNote.style.margin='4px 0 7px'; hardwarePinsNote.innerHTML='Use core GPIO numbers here, not the board silk-screen D labels.<br>Makers should check their exact board pinout before wiring or changing pins.<br>Changes are saved and applied immediately.';
const pinGrid = document.createElement('div'); pinGrid.style.display='grid'; pinGrid.style.gridTemplateColumns='1fr 1fr'; pinGrid.style.gap='7px';
const leftServoPin = makePinInput('Left Servo', 'leftServoPinInput', 2);
const rightServoPin = makePinInput('Right Servo', 'rightServoPinInput', 4);
const ledPin = makePinInput('LED Filament', 'ledPinInput', 3);
const batteryPin = makePinInput('V-divider ADC', 'batteryPinInput', 0);
const batteryWarningLedPin = makePinInput('Battery LED', 'batteryWarningLedPinInput', 8);
const statusLedPin = makePinInput('Status LED', 'statusLedPinInput', 6);
const elrsRxPin = makePinInput('ELRS RX', 'elrsRxPinInput', 9);
const elrsTxPin = makePinInput('ELRS TX', 'elrsTxPinInput', 10);
[leftServoPin,rightServoPin,ledPin,batteryPin,batteryWarningLedPin,statusLedPin,elrsRxPin,elrsTxPin].forEach(item=>pinGrid.appendChild(item.wrap));
hardwarePinsBar.appendChild(hardwarePinsTitle);
hardwarePinsBar.appendChild(hardwarePinsNote);
hardwarePinsBar.appendChild(pinGrid);
window._leftServoPinInput = leftServoPin.input;
window._rightServoPinInput = rightServoPin.input;
window._ledPinInput = ledPin.input;
window._batteryPinInput = batteryPin.input;
window._batteryWarningLedPinInput = batteryWarningLedPin.input;
window._statusLedPinInput = statusLedPin.input;
window._elrsRxPinInput = elrsRxPin.input;
window._elrsTxPinInput = elrsTxPin.input;

const channelMappingBar = document.createElement('div'); channelMappingBar.className='slider-section'; channelMappingBar.style.display='none';
const channelMappingTitle = document.createElement('div'); channelMappingTitle.innerHTML='<strong>Channel Mapping</strong>';
const channelMappingNote = document.createElement('div'); channelMappingNote.style.color='var(--muted)'; channelMappingNote.style.fontSize='11px'; channelMappingNote.style.lineHeight='1.4'; channelMappingNote.innerText='One saved preset is used for both ESP-NOW and ELRS. Defaults match the current radio setup.';
const channelOptions = [
    [0, 'Unused'],
    [1, 'Roll'],
    [2, 'Pitch / Neutral'],
    [3, 'Throttle'],
    [4, 'Steering'],
    [5, 'Arm / Perch'],
    [6, 'Waveform'],
    [7, 'Flight Mode'],
    [8, 'LED Mode']
];
const channelDefaults = [1,2,3,4,5,6,7,8];
const channelGrid = document.createElement('div'); channelGrid.style.display='grid'; channelGrid.style.gridTemplateColumns='1fr 1fr'; channelGrid.style.gap='7px'; channelGrid.style.width='100%';
const channelSelects = [];
for (let i=0; i<8; i++) {
    const wrap = document.createElement('div');
    wrap.style.display='grid';
    wrap.style.gridTemplateColumns='42px 1fr';
    wrap.style.alignItems='center';
    wrap.style.gap='5px';
    const label = document.createElement('label');
    label.htmlFor = 'channelMap' + (i + 1);
    label.style.color='var(--text)';
    label.style.fontWeight='700';
    label.style.fontSize='12px';
    label.innerText = 'CH' + (i + 1);
    const select = document.createElement('select');
    select.id = 'channelMap' + (i + 1);
    select.style.background='var(--panel)';
    select.style.color='var(--text)';
    select.style.border='2px solid var(--accent)';
    select.style.borderRadius='8px';
    select.style.padding='6px 4px';
    select.style.minWidth='0';
    channelOptions.forEach(pair=>{ const o=document.createElement('option'); o.value=pair[0]; o.innerText=pair[1]; if(pair[0]===channelDefaults[i])o.selected=true; select.appendChild(o); });
    channelSelects.push(select);
    wrap.appendChild(label);
    wrap.appendChild(select);
    channelGrid.appendChild(wrap);
}
channelMappingBar.appendChild(channelMappingTitle);
channelMappingBar.appendChild(channelMappingNote);
channelMappingBar.appendChild(channelGrid);
window._channelMappingBar = channelMappingBar;
window._channelMapSelects = channelSelects;

// Neutral pitch controls. Device Tilt swaps the single neutral slider for
// forward/hover endpoint sliders driven by the phone motion sensor.
const pitchBar = document.createElement('div'); pitchBar.className='slider-section';
const pitchLabel = document.createElement('div'); pitchLabel.style.color='var(--text)'; pitchLabel.style.textAlign='center'; pitchLabel.innerHTML='Neutral Pitch Angle: <span id="pitchVal">90\u00B0</span>';
const pitchSlider = document.createElement('input'); pitchSlider.type='range'; pitchSlider.min=80; pitchSlider.max=100; pitchSlider.value=90; pitchSlider.step=1; pitchSlider.id='pitchSlider';
const pitchVal = pitchLabel.querySelector('#pitchVal');
const pitchReset = document.createElement('button'); pitchReset.className='mode-button'; pitchReset.innerText='Reset';
pitchReset.onclick=()=>{ if(window.confirm('Reset Neutral Pitch Angle? Select OK for Yes or Cancel for No.')){ pitchSlider.value=90; hoverForward.slider.value=95; hoverNeutral.slider.value=85; pitchVal.innerText=hoverEnable.checked ? '95\u00B0 - 85\u00B0' : '90\u00B0'; hoverForward.val.innerText='95\u00B0'; hoverNeutral.val.innerText='85\u00B0'; fetch('/setPitch?p=90').catch(()=>{}); fetch('/setHoverBiasConfig?forward=95&hover=85').catch(()=>{}); } };
const pitchRow = document.createElement('div'); pitchRow.className='wide-slider-row'; pitchRow.appendChild(pitchSlider); pitchRow.appendChild(pitchReset);
const hoverTitle = document.createElement('label'); hoverTitle.style.display='flex'; hoverTitle.style.alignItems='center'; hoverTitle.style.gap='8px'; hoverTitle.style.color='var(--text)'; hoverTitle.style.fontWeight='700'; hoverTitle.style.marginTop='2px';
const hoverEnable = document.createElement('input'); hoverEnable.type='checkbox'; hoverEnable.id='hoverBiasEnable';
hoverTitle.appendChild(hoverEnable); hoverTitle.appendChild(document.createTextNode('Device Tilt'));
const hoverSensorStatus = document.createElement('div'); hoverSensorStatus.id='hoverSensorStatus'; hoverSensorStatus.style.color='var(--text)'; hoverSensorStatus.style.fontSize='12px'; hoverSensorStatus.innerText='Sensor: off';

function makeHoverSlider(label, id, min, max, value, unit){
    const wrap = document.createElement('div'); wrap.style.width='100%';
    const text = document.createElement('div'); text.style.color='var(--text)'; text.innerHTML = label + ': <span id="' + id + 'Val">' + value + unit + '</span>';
    const row = document.createElement('div'); row.style.display='flex'; row.style.alignItems='center'; row.style.justifyContent='center'; row.style.gap='8px';
    const slider = document.createElement('input'); slider.type='range'; slider.min=min; slider.max=max; slider.value=value; slider.step=1; slider.style.width='220px'; slider.id=id;
    row.appendChild(slider); wrap.appendChild(text); wrap.appendChild(row);
    return {wrap:wrap, slider:slider, val:text.querySelector('span'), unit:unit};
}

const hoverForward = makeHoverSlider('Forward Flight', 'hoverForwardSlider', 90, 100, 95, '\u00B0');
const hoverNeutral = makeHoverSlider('Hover', 'hoverNeutralSlider', 80, 90, 85, '\u00B0');
pitchBar.appendChild(pitchLabel);
pitchBar.appendChild(pitchRow);
pitchBar.appendChild(hoverForward.wrap);
pitchBar.appendChild(hoverNeutral.wrap);
pitchBar.appendChild(hoverSensorStatus);
pitchBar.appendChild(hoverTitle);

prefsDiv.appendChild(lowerAmp.bar);
prefsDiv.appendChild(upperAmp.bar);
prefsDiv.appendChild(pitchBar);

// Motion steering controls
const motionSteerBar = document.createElement('div'); motionSteerBar.className='slider-section';
const motionSteerTitle = document.createElement('label'); motionSteerTitle.style.display='flex'; motionSteerTitle.style.alignItems='center'; motionSteerTitle.style.gap='8px'; motionSteerTitle.style.color='var(--text)'; motionSteerTitle.style.fontWeight='700';
const motionSteerEnable = document.createElement('input'); motionSteerEnable.type='checkbox'; motionSteerEnable.id='motionSteerEnable';
motionSteerTitle.appendChild(motionSteerEnable); motionSteerTitle.appendChild(document.createTextNode('Motion Steering (Device Tilt)'));
const motionSteerStatus = document.createElement('div'); motionSteerStatus.id='motionSteerStatus'; motionSteerStatus.style.color='var(--text)'; motionSteerStatus.style.fontSize='12px'; motionSteerStatus.innerText='Steering: joystick';
motionSteerBar.appendChild(motionSteerTitle);
motionSteerBar.appendChild(motionSteerStatus);
prefsDiv.appendChild(motionSteerBar);

const gridSteering = document.createElement('div'); gridSteering.className='prefs-grid';
gridSteering.appendChild(steerBar);
gridSteering.appendChild(steerGainBar);
const leftTrimBar = document.createElement('div'); leftTrimBar.className='slider-section';
leftTrimBar.appendChild(leftLbl); leftTrimBar.appendChild(leftRow);
const rightTrimBar = document.createElement('div'); rightTrimBar.className='slider-section';
rightTrimBar.appendChild(rightLbl); rightTrimBar.appendChild(rightRow);
const gridTrims = document.createElement('div'); gridTrims.className='prefs-grid';
gridTrims.appendChild(leftTrimBar);
gridTrims.appendChild(rightTrimBar);
const gridFrequency = document.createElement('div'); gridFrequency.className='prefs-grid';
gridFrequency.appendChild(servoHzBar);
gridFrequency.appendChild(flapFreqBar);
const gridServoPulse = document.createElement('div'); gridServoPulse.className='prefs-grid';
gridServoPulse.appendChild(servoMinBar);
gridServoPulse.appendChild(servoMaxBar);
const gridLed = document.createElement('div'); gridLed.className='prefs-grid';
gridLed.appendChild(ledBrightnessBar);
gridLed.appendChild(ledSpeedBar);

prefsDiv.insertBefore(gridTrims, profileNotice.nextSibling);
prefsDiv.insertBefore(gridSteering, gridTrims.nextSibling);
prefsDiv.insertBefore(upperAmp.bar, pitchBar);
prefsDiv.insertBefore(lowerAmp.bar, pitchBar);
prefsDiv.insertBefore(gridFrequency, pitchBar);
prefsDiv.insertBefore(gridServoPulse, pitchBar);
prefsDiv.insertBefore(hardwarePinsBar, pitchBar);
prefsDiv.insertBefore(channelMappingBar, pitchBar);
prefsDiv.appendChild(battSectionTitle);
prefsDiv.appendChild(battEnableWrap);
prefsDiv.appendChild(battGrid);
prefsDiv.appendChild(battFailsafeWrap);
prefsDiv.appendChild(battNote);
prefsDiv.appendChild(gridLed);
const appearanceBar = document.createElement('div');
appearanceBar.className = 'slider-section';
const appearanceTitle = document.createElement('div');
appearanceTitle.innerHTML = '<strong>Appearance</strong>';
const lightWrap = document.createElement('label');
lightWrap.style.display = 'flex';
lightWrap.style.alignItems = 'center';
lightWrap.style.justifyContent = 'center';
lightWrap.style.gap = '8px';
lightWrap.style.color = 'var(--text)';
const lightToggle = document.createElement('input');
lightToggle.type = 'checkbox';
lightToggle.id = 'lightThemeToggle';
lightWrap.appendChild(lightToggle);
lightWrap.appendChild(document.createTextNode('Light Mode'));
const themeSwatches = document.createElement('div');
themeSwatches.style.display = 'grid';
themeSwatches.style.gridTemplateColumns = 'repeat(5, 1fr)';
themeSwatches.style.gap = '7px';
themeSwatches.style.width = '100%';
const themeColors = [
    ['Green', '#00ff66', '#00ff88'],
    ['Blue', '#40a8ff', '#64d8ff'],
    ['Purple', '#b46cff', '#d39cff'],
    ['Amber', '#ffb238', '#ffd166'],
    ['Red', '#ff4f5e', '#ff8790']
];
function applyUiTheme(){
    const light = lightToggle.checked;
    document.body.classList.toggle('light-theme', light);
    try { localStorage.setItem('espFlutterLightTheme', light ? '1' : '0'); } catch(e) {}
}
function applyAccent(color, fill){
    document.documentElement.style.setProperty('--accent', color);
    document.documentElement.style.setProperty('--fill', fill);
    document.documentElement.style.setProperty('--accent-soft', color + '59');
    try { localStorage.setItem('espFlutterAccent', color); localStorage.setItem('espFlutterFill', fill); } catch(e) {}
}
themeColors.forEach(pair=>{
    const b = document.createElement('button');
    b.className = 'mode-button';
    b.title = pair[0];
    b.style.height = '30px';
    b.style.padding = '0';
    b.style.background = pair[1];
    b.style.borderColor = pair[1];
    b.onclick = ()=>applyAccent(pair[1], pair[2]);
    themeSwatches.appendChild(b);
});
lightToggle.onchange = applyUiTheme;
appearanceBar.appendChild(appearanceTitle);
appearanceBar.appendChild(lightWrap);
appearanceBar.appendChild(themeSwatches);
prefsDiv.appendChild(appearanceBar);
try {
    const savedLight = localStorage.getItem('espFlutterLightTheme') === '1';
    const savedAccent = localStorage.getItem('espFlutterAccent');
    const savedFill = localStorage.getItem('espFlutterFill');
    lightToggle.checked = savedLight;
    applyUiTheme();
    if (savedAccent && savedFill) applyAccent(savedAccent, savedFill);
} catch(e) {}
const prefsSavedNote = document.createElement('div');
prefsSavedNote.style.margin = '12px 0 0';
prefsSavedNote.style.fontSize = '12px';
prefsSavedNote.style.color = 'var(--text)';
prefsSavedNote.style.opacity = '0.5';
prefsSavedNote.innerText = 'All changes are automatically saved';
prefsDiv.appendChild(prefsSavedNote);
const prefsCopyright = document.createElement('div');
prefsCopyright.className = 'app-footer';
prefsCopyright.innerHTML = '&copy; 2026 Max Imagination';
prefsDiv.appendChild(prefsCopyright);
const prefsBottomSpacer = document.createElement('div');
prefsBottomSpacer.style.height = '90px';
prefsBottomSpacer.style.pointerEvents = 'none';
prefsDiv.appendChild(prefsBottomSpacer);

// Position tabs under the title and the battery strip under the tabs.
(function(){
    const container = document.querySelector('.container');
    const titleEl = container.querySelector('.title');
    const topBatteryPanel = document.getElementById('topBatteryPanel');
    container.insertBefore(tabBar, titleEl.nextSibling);
    if (topBatteryPanel) {
        topBatteryPanel.style.display = 'block';
        container.insertBefore(topBatteryPanel, tabBar.nextSibling);
    }

    // Build controller wrapper and move controller elements inside it
    const controllerWrapper = document.createElement('div'); controllerWrapper.id='controllerWrapper'; controllerWrapper.style.marginTop='8px';
    // move existing panel and joystick into wrapper
    const panel = container.querySelector('.panel'); const joy = document.getElementById('joystick'); const notice = document.getElementById('motionSteerNotice'); const controllerFooter = document.getElementById('controllerFooter');
    if (panel) controllerWrapper.appendChild(panel);
    if (typeof protocolBox !== 'undefined') controllerWrapper.appendChild(protocolBox);
    if (notice) controllerWrapper.appendChild(notice);
    if (joy) controllerWrapper.appendChild(joy);
    // Keep flight mode controls on the controller tab.
    if (typeof modeBar !== 'undefined') controllerWrapper.appendChild(modeBar);
    if (controllerFooter) controllerWrapper.appendChild(controllerFooter);

    // insert wrapper after tabs
    container.insertBefore(controllerWrapper, prefsDiv);

    // Define showTab now that ampBar and DOM refs exist
    window.showTab = function(which){
        if (which === 'prefs'){
            if (panel) panel.style.display='none';
            if (joy) joy.style.display='none';
            if (notice) notice.style.display='none';
            if (controllerFooter) controllerFooter.style.display='none';
            if (typeof protocolBox !== 'undefined') protocolBox.style.display='none';
            if (typeof modeBar !== 'undefined' && modeBar) modeBar.style.display='none';
            tabController.classList.remove('active'); tabPrefs.classList.add('active');
            prefsDiv.style.display='block';
        } else {
            if (panel) panel.style.display='block';
            if (joy) joy.style.display='block';
            if (typeof protocolBox !== 'undefined') protocolBox.style.display='flex';
            if (typeof modeBar !== 'undefined' && modeBar) modeBar.style.display='flex';
            if (controllerFooter) controllerFooter.style.display='block';
            updateProtocolDisplay(controlProtocolLocal);
            tabPrefs.classList.remove('active'); tabController.classList.add('active');
            prefsDiv.style.display='none';
        }
    };
    // show controller tab by default
    showTab('controller');
})();

let ampDebounce = null;
function attachAmplitudeHandler(control){
    control.slider.addEventListener('input', ()=>{
        control.value.innerText = Math.round(control.slider.value) + '\u00B0';
        clearTimeout(ampDebounce);
        ampDebounce = setTimeout(()=>{
            fetch(`${control.route}?deg=${control.slider.value}`).catch(()=>{});
        }, 80);
    });
    control.slider.addEventListener('change', ()=>{
        fetch(`${control.route}?deg=${control.slider.value}`).catch(()=>{});
    });
}
attachAmplitudeHandler(lowerAmp);
attachAmplitudeHandler(upperAmp);
function sendServoOutputConfig(){
    fetch(`/setServoOutput?hz=${servoHzSelect.value}&min=${servoMinSlider.value}&max=${servoMaxSlider.value}`).catch(()=>{});
}
let servoOutputDebounce = null;
function updateServoOutputLabels(){
    document.getElementById('servoMinVal').innerText = servoMinSlider.value + ' us';
    document.getElementById('servoMaxVal').innerText = servoMaxSlider.value + ' us';
}
function queueServoOutputConfig(){
    updateServoOutputLabels();
    clearTimeout(servoOutputDebounce);
    servoOutputDebounce = setTimeout(sendServoOutputConfig, 100);
}
servoMinSlider.addEventListener('input', queueServoOutputConfig);
servoMaxSlider.addEventListener('input', queueServoOutputConfig);
servoHzSelect.addEventListener('change', sendServoOutputConfig);
servoMinSlider.addEventListener('change', sendServoOutputConfig);
servoMaxSlider.addEventListener('change', sendServoOutputConfig);
window._servoHzSelect = servoHzSelect;
window._servoMinSlider = servoMinSlider;
window._servoMinVal = document.getElementById('servoMinVal');
window._servoMaxSlider = servoMaxSlider;
window._servoMaxVal = document.getElementById('servoMaxVal');
let flapFreqDebounce = null;
function sendFlapFrequencyCap(){
    fetch(`/setFrequencyCap?hz=${flapFreqSlider.value}`).catch(()=>{});
}
flapFreqSlider.addEventListener('input', ()=>{
    document.getElementById('flapFreqVal').innerText = Number(flapFreqSlider.value).toFixed(1) + ' Hz';
    clearTimeout(flapFreqDebounce);
    flapFreqDebounce = setTimeout(sendFlapFrequencyCap, 80);
});
flapFreqSlider.addEventListener('change', sendFlapFrequencyCap);
window._flapFreqSlider = flapFreqSlider;
window._flapFreqVal = document.getElementById('flapFreqVal');

let hardwarePinDebounce = null;
function sendHardwarePinConfig(){
    fetch(`/setHardwarePins?ls=${leftServoPin.input.value}&rs=${rightServoPin.input.value}&led=${ledPin.input.value}&bat=${batteryPin.input.value}&bwarn=${batteryWarningLedPin.input.value}&stat=${statusLedPin.input.value}&erx=${elrsRxPin.input.value}&etx=${elrsTxPin.input.value}`).catch(()=>{});
}
function queueHardwarePinConfig(){
    clearTimeout(hardwarePinDebounce);
    hardwarePinDebounce = setTimeout(sendHardwarePinConfig, 250);
}
[leftServoPin.input,rightServoPin.input,ledPin.input,batteryPin.input,batteryWarningLedPin.input,statusLedPin.input,elrsRxPin.input,elrsTxPin.input].forEach(input=>{
    input.addEventListener('change', sendHardwarePinConfig);
    input.addEventListener('input', queueHardwarePinConfig);
});
function sendChannelMappingConfig(){
    const params = channelSelects.map((select, idx)=>'ch' + (idx + 1) + '=' + select.value).join('&');
    fetch('/setChannelMapping?' + params).catch(()=>{});
}
channelSelects.forEach(select=>select.addEventListener('change', sendChannelMappingConfig));

pitchSlider.addEventListener('input', updateNeutralPitchUiMode);
pitchSlider.addEventListener('change', ()=>{ let p = pitchSlider.value; fetch(`/setPitch?p=${p}`).catch(()=>{}); });

function updateNeutralPitchUiMode(){
    const tiltEnabled = hoverEnable.checked;
    pitchRow.style.display = tiltEnabled ? 'none' : 'flex';
    hoverForward.wrap.style.display = tiltEnabled ? 'block' : 'none';
    hoverNeutral.wrap.style.display = tiltEnabled ? 'block' : 'none';
    pitchVal.innerText = tiltEnabled
        ? (hoverForward.slider.value + '\u00B0 - ' + hoverNeutral.slider.value + '\u00B0')
        : (pitchSlider.value + '\u00B0');
}
updateNeutralPitchUiMode();

let hoverBiasEnabledLocal = false;
let latestDevicePitch = 0.0;
let hoverTiltSendMs = 0;
let hoverSensorSeenMs = 0;
let hoverMotionSeenMs = 0;
let motionSteerSeenMs = 0;
let hoverFilteredTilt = 0.0;
let hoverFilterInit = false;
let hoverFilterMs = 35.0;
let hoverFilterLastMs = 0;

function setHoverSensorStatus(text){
    const el = document.getElementById('hoverSensorStatus');
    if (el) el.innerText = 'Sensor: ' + text;
}

function setMotionSteerStatus(text){
    const el = document.getElementById('motionSteerStatus');
    if (el) el.innerText = 'Steering: ' + text;
}

function sendHoverBiasConfig(){
    const enabled = hoverEnable.checked ? 1 : 0;
    const fwd = hoverForward.slider.value;
    const hov = hoverNeutral.slider.value;
    fetch(`/setHoverBiasConfig?enabled=${enabled}&forward=${fwd}&hover=${hov}`).catch(()=>{});
}

function sendHoverPitchToEsp32(source, rawPitchDeg){
    if (!hoverBiasEnabledLocal) return;
    if (typeof rawPitchDeg !== 'number' || !isFinite(rawPitchDeg)) return;
    const tiltDeg = Math.max(0, Math.min(45, -rawPitchDeg));
    const now = Date.now();
    if (!hoverFilterInit) {
        hoverFilteredTilt = tiltDeg;
        hoverFilterInit = true;
        hoverFilterLastMs = now;
    } else {
        const dt = Math.max(1, now - hoverFilterLastMs);
        hoverFilterLastMs = now;
        const alpha = dt / (hoverFilterMs + dt);
        hoverFilteredTilt += alpha * (tiltDeg - hoverFilteredTilt);
    }
    latestDevicePitch = hoverFilteredTilt;
    hoverSensorSeenMs = Date.now();
    if (source === 'motion') hoverMotionSeenMs = hoverSensorSeenMs;
    setHoverSensorStatus(source + ' ' + latestDevicePitch.toFixed(0) + '\u00B0');
    if (now - hoverTiltSendMs > 40) {
        hoverTiltSendMs = now;
        fetch(`/hoverBiasTilt?pitch=${latestDevicePitch.toFixed(2)}`).catch(()=>{});
    }
}

function handleDeviceOrientation(e){
    if (Date.now() - hoverMotionSeenMs < 800) return;
    if (typeof e.beta === 'number') sendHoverPitchToEsp32('orientation', e.beta);
}

function handleDeviceMotion(e){
    const g = e.accelerationIncludingGravity;
    if (!g) return;
    if (typeof g.y !== 'number' || typeof g.z !== 'number') return;
    const pitchFromGravity = Math.atan2(-g.y, g.z) * 180.0 / Math.PI;
    sendHoverPitchToEsp32('motion', pitchFromGravity);
    if (motionSteeringEnabledLocal && !joystickSteeringLocked && typeof g.x === 'number') {
        const sideTiltDeg = Math.max(-45, Math.min(45, Math.atan2(g.x, g.z) * 180.0 / Math.PI));
        const targetSteer = Math.max(-1, Math.min(1, -sideTiltDeg / 45.0));
        const now = Date.now();
        if (!motionSteerFilterInit) {
            filteredMotionSteer = targetSteer;
            motionSteerFilterInit = true;
            motionSteerLastMs = now;
        } else {
            const dt = Math.max(1, now - motionSteerLastMs);
            motionSteerLastMs = now;
            const alpha = dt / (30.0 + dt);
            filteredMotionSteer += alpha * (targetSteer - filteredMotionSteer);
        }
        latestMotionSteer = Math.max(-1, Math.min(1, filteredMotionSteer));
        currentSteer = latestMotionSteer;
        if (Math.abs(latestMotionSteer) >= 0.05) controlsReleased = false;
        motionSteerSeenMs = Date.now();
        setMotionSteerStatus('tilt ' + (latestMotionSteer * 45.0).toFixed(0) + '\u00B0');
    }
}

async function requestMotionSensorAccess(){
    if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
        const result = await DeviceOrientationEvent.requestPermission();
        if (result !== 'granted') return false;
    }
    if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
        const result = await DeviceMotionEvent.requestPermission();
        if (result !== 'granted') return false;
    }
    return true;
}

async function enableHoverBiasFromUi(){
    hoverBiasEnabledLocal = hoverEnable.checked;
    updateNeutralPitchUiMode();
    hoverMotionSeenMs = 0;
    hoverSensorSeenMs = 0;
    hoverFilterInit = false;
    hoverFilteredTilt = 0.0;
    hoverFilterLastMs = 0;
    setHoverSensorStatus(hoverBiasEnabledLocal ? 'waiting' : 'off');
    if (hoverBiasEnabledLocal) {
        try {
            if (!(await requestMotionSensorAccess())) {
                hoverEnable.checked = false;
                hoverBiasEnabledLocal = false;
                setHoverSensorStatus('permission denied');
            }
        } catch(e) {
            hoverEnable.checked = false;
            hoverBiasEnabledLocal = false;
            setHoverSensorStatus('permission error');
        }
    }
    sendHoverBiasConfig();
}

async function enableMotionSteeringFromUi(){
    motionSteeringEnabledLocal = motionSteerEnable.checked;
    try { localStorage.setItem('espFlutterMotionSteering', motionSteeringEnabledLocal ? '1' : '0'); } catch(e) {}
    motionSteerSeenMs = 0;
    motionSteerFilterInit = false;
    filteredMotionSteer = 0.0;
    motionSteerLastMs = 0;
    const notice = document.getElementById('motionSteerNotice');
    if (notice) notice.style.display = motionSteeringEnabledLocal ? 'block' : 'none';
    if (motionSteeringEnabledLocal) {
        stick.style.left = '70px';
        currentSteer = latestMotionSteer;
        setMotionSteerStatus('waiting');
        try {
            if (!(await requestMotionSensorAccess())) {
                motionSteerEnable.checked = false;
                motionSteeringEnabledLocal = false;
                currentSteer = 0.0;
                setMotionSteerStatus('permission denied');
            }
        } catch(e) {
            motionSteerEnable.checked = false;
            motionSteeringEnabledLocal = false;
            currentSteer = 0.0;
            setMotionSteerStatus('permission error');
        }
    } else {
        latestMotionSteer = 0.0;
        currentSteer = 0.0;
        setMotionSteerStatus('joystick');
    }
}

hoverEnable.addEventListener('change', enableHoverBiasFromUi);
motionSteerEnable.addEventListener('change', enableMotionSteeringFromUi);
try {
    if (localStorage.getItem('espFlutterMotionSteering') === '1') {
        motionSteerEnable.checked = true;
        enableMotionSteeringFromUi();
    }
} catch(e) {}
window.addEventListener('deviceorientation', handleDeviceOrientation);
window.addEventListener('deviceorientationabsolute', handleDeviceOrientation);
window.addEventListener('devicemotion', handleDeviceMotion);
setInterval(()=>{
    if (motionSteeringEnabledLocal && (motionSteerSeenMs === 0 || Date.now() - motionSteerSeenMs > 1200)) {
        setMotionSteerStatus(window.isSecureContext ? 'waiting for motion' : 'no events over HTTP');
    }
    if (!hoverBiasEnabledLocal) return;
    if (hoverSensorSeenMs === 0 || Date.now() - hoverSensorSeenMs > 1200) {
        setHoverSensorStatus(window.isSecureContext ? 'waiting for motion' : 'no events over HTTP');
    }
}, 500);

for (const item of [hoverForward, hoverNeutral]) {
    item.slider.addEventListener('input', ()=>{
        item.val.innerText = item.slider.value + item.unit;
        updateNeutralPitchUiMode();
    });
    item.slider.addEventListener('change', sendHoverBiasConfig);
}

    // expose elements for telemetry updater
    window._lowerAmpSlider = lowerAmp.slider; window._lowerAmpVal = lowerAmp.value;
    window._upperAmpSlider = upperAmp.slider; window._upperAmpVal = upperAmp.value;
    window._pitchSlider = pitchSlider; window._pitchVal = pitchVal;
    window._steerGainSlider = steerGainSlider; window._steerGainVal = document.getElementById('steerGainVal');
    window._hoverBiasEnable = hoverEnable;
    window._hoverForwardSlider = hoverForward.slider; window._hoverForwardVal = hoverForward.val;
    window._hoverNeutralSlider = hoverNeutral.slider; window._hoverNeutralVal = hoverNeutral.val;
    window._ampInitialized = false;

    if (window.__initTelemetry) {
        try {
            updateAmplitudeDisplay(window.__initTelemetry);
            updateBatteryDisplay(window.__initTelemetry);
            updateTimerDisplay(window.__initTelemetry);
            updateHoverBiasDisplay(window.__initTelemetry);
            updateLedDisplay(window.__initTelemetry);
            updateFlightProfileDisplay(window.__initTelemetry);
            if (document.getElementById('tAmp')) document.getElementById('tAmp').innerText = formatDegrees(window.__initTelemetry.amplitude);
            if (document.getElementById('tPilot')) document.getElementById('tPilot').innerText = (window.__initTelemetry.pilot||0).toFixed(2);
            updateArmDisplay(window.__initTelemetry.mode || 'UNKNOWN');
        } catch(e) {}
    }

    // inject lightweight CSS for centered percentage overlays
    var _s = document.createElement('style');
    _s.innerHTML = '.bar{position:relative}.bar-overlay{position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);pointer-events:none;}';
    document.head.appendChild(_s);

});
</script>
</body>
</html>)rawliteral";

    server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "0");
    server.send_P(200, "text/html; charset=UTF-8", html);
}

static float throttleToFrequency(float throttle)
{
    if (throttle <= CONTROL_THROTTLE_DEADBAND)
        return 0.0f;

    float activeRange = 1.0f - CONTROL_THROTTLE_DEADBAND;
    float normalized = (throttle - CONTROL_THROTTLE_DEADBAND) / activeRange;
    normalized = constrain(normalized, 0.0f, 1.0f);

    float maxFrequency = getProfileFrequencyCap();
    return MIN_ACTIVE_FLAP_FREQUENCY +
        normalized * (maxFrequency - MIN_ACTIVE_FLAP_FREQUENCY);
}

static void applyPilotControlValues(float x, float y, uint32_t seq)
{
    x = constrain(x, -1.0f, 1.0f);
    y = constrain(y, 0.0f, 1.0f);
    if (steeringLockEnabled)
        x = 0.0f;

    if (isStaleControlSeq(seq))
        return;
    noteControlSeq(seq);

    const bool throttleIdle = y <= CONTROL_THROTTLE_DEADBAND;

    if (isBatteryServoFailsafeActive())
    {
        enforceBatteryServoFailsafe();
        if (throttleIdle)
        {
            if (isForceCenterActive())
                clearForceCenter();
            setIdleLeanSteering(-x * IDLE_LEAN_INPUT_AUTHORITY);
            lastThrottle = 0.0f;
            lastSteer = x;
        }
        return;
    }

    if (!throttleIdle && isControlReleaseLocked())
        return;

    if (isForceCenterActive())
        clearForceCenter();

    if (getFlightMode() == FLIGHT_MODE_DISARMED)
    {
        immediateStopAndCenter();
        lastThrottle = 0.0f;
        lastSteer = 0.0f;
        updateFlightTimer(0.0f);
        return;
    }

    if (getFlightMode() == FLIGHT_MODE_PERCH)
        return;

    lastControlPacket = millis();
    lastThrottle = throttleIdle ? 0.0f : y;
    lastSteer = x;
    updateFlightTimer(lastThrottle);

    if (throttleIdle)
    {
        setIdleLeanSteering(-x * IDLE_LEAN_INPUT_AUTHORITY);
        return;
    }

    float scale = powf(y, STEERING_CURVE_EXPONENT);
    float steeringAuthority = STEERING_BASE_AUTHORITY +
        scale * (STEERING_MAX_AUTHORITY - STEERING_BASE_AUTHORITY);
    steeringAuthority *= getProfileSteeringMultiplier();
    steeringAuthority = fminf(steeringAuthority, MAX_FINAL_STEERING);
    setPilotSteering(-x * steeringAuthority);

    setFrequency(throttleToFrequency(y));

    float amplitude;
#if FIXED_AMPLITUDE_TEST_MODE
    amplitude = TEST_FLAP_AMPLITUDE_DEGREES;
#else
    amplitude = MIN_ACTIVE_FLAP_AMPLITUDE +
        y * (MAX_FLAP_AMPLITUDE - MIN_ACTIVE_FLAP_AMPLITUDE);
#endif

    if (amplitude < MIN_FLAP_AMPLITUDE) amplitude = MIN_FLAP_AMPLITUDE;
    if (amplitude > MAX_FLAP_AMPLITUDE) amplitude = MAX_FLAP_AMPLITUDE;

    float finalAmp = amplitude;
    if (finalAmp < MIN_FLAP_AMPLITUDE) finalAmp = MIN_FLAP_AMPLITUDE;
    if (finalAmp > MAX_FLAP_AMPLITUDE) finalAmp = MAX_FLAP_AMPLITUDE;
    setAmplitude(finalAmp);
}

static void setAbsoluteServoTrims(int left, int right)
{
    left = constrain(left, -45, 45);
    right = constrain(right, -45, 45);
    adjustLeftTrim(left - getLeftTrim());
    adjustRightTrim(right - getRightTrim());
}

static void applyEspNowPacket(const EspFlutterNowPacket &packet)
{
    if (packet.flags & ESP_FLUTTER_NOW_FLIGHT_MODE)
    {
        if (isBatteryServoFailsafeActive() && packet.flightMode != FLIGHT_MODE_DISARMED)
        {
            enforceBatteryServoFailsafe();
        }
        else
        {
            setFlightModeFromInt(packet.flightMode);
            if (getFlightMode() == FLIGHT_MODE_PERCH)
                startPerch();
            else
                stopPerch();
        }
    }

    if (packet.flags & ESP_FLUTTER_NOW_FLIGHT_PROFILE)
    {
        setFlightProfileModeFromInt(packet.flightProfile);
        applyCurrentFlightProfile();
    }

    if (packet.flags & ESP_FLUTTER_NOW_WAVEFORM)
    {
        int waveform = packet.waveform == 1 ? 1 : 0;
        setFlapWaveform(waveform);
        setCurrentProfileWaveform(waveform);
    }

    if (packet.flags & ESP_FLUTTER_NOW_AMPLITUDE)
    {
        amplitudeScale = constrain(packet.amplitudeScale, 0.60f, 1.00f);
        lowerAmplitudeScale = lowerAmplitudeScaleFromDegrees(amplitudeScale * MAX_DOWNSTROKE_BELOW_NEUTRAL);
        upperAmplitudeScale = upperAmplitudeScaleFromDegrees(amplitudeScale * MAX_UPSTROKE_ABOVE_NEUTRAL);
        setFlapAmplitudeScales(lowerAmplitudeScale, upperAmplitudeScale);
        setCurrentProfileLowerAmplitudeScale(lowerAmplitudeScale);
        setCurrentProfileUpperAmplitudeScale(upperAmplitudeScale);
        saveAmplitudeScale();
    }

    if (packet.flags & ESP_FLUTTER_NOW_STEERING_TRIM)
        setSteeringTrim(packet.steeringTrim);

    if (packet.flags & ESP_FLUTTER_NOW_STEERING_GAIN)
    {
        setSteeringDifferentialGain(packet.steeringGain);
        setCurrentProfileSteeringGain(getSteeringDifferentialGain());
    }

    if (packet.flags & ESP_FLUTTER_NOW_TURN_COMP)
    {
        setTurnCompensation(packet.turnCompensation);
        setCurrentProfileTurnCompensation(getTurnCompensation());
    }

    if (packet.flags & ESP_FLUTTER_NOW_NEUTRAL_PITCH)
    {
        setServoPitchAngle(packet.neutralPitch);
        setCurrentProfileNeutralPitch(getServoPitchAngle());
    }

    if (packet.flags & ESP_FLUTTER_NOW_HOVER_BIAS)
    {
        setHoverBiasEnabled(packet.hoverBiasEnabled != 0);
        setHoverBiasConfig(packet.hoverForwardNeutral, packet.hoverNeutral);
    }

    if (packet.flags & ESP_FLUTTER_NOW_LED)
    {
        if (packet.ledMode >= 0) setLedMode(packet.ledMode);
        if (packet.ledBrightness >= 1) setLedBrightnessLevel(packet.ledBrightness);
        if (packet.ledSpeed >= 1) setLedEffectSpeedLevel(packet.ledSpeed);
    }

    if (packet.flags & ESP_FLUTTER_NOW_SERVO_TRIMS)
        setAbsoluteServoTrims(packet.leftTrim, packet.rightTrim);

    if ((packet.flags & ESP_FLUTTER_NOW_CONTROL) && controlProtocol == CONTROL_PROTOCOL_ESPNOW)
        applyPilotControlValues(packet.steering, packet.throttle, packet.seq);
}

static uint8_t rtlopezChecksum(const uint8_t *data, size_t len)
{
    uint8_t csum = 0x55;
    for (size_t i = 0; i < len; ++i)
        csum ^= data[i];
    return csum;
}

template<typename T>
static void rtlopezSend(const uint8_t *mac, T &message)
{
    message.csum = rtlopezChecksum(reinterpret_cast<const uint8_t*>(&message), sizeof(T) - 1);
    esp_now_send(mac, reinterpret_cast<const uint8_t*>(&message), sizeof(T));
}

static bool macEquals(const uint8_t *a, const uint8_t *b)
{
    return memcmp(a, b, 6) == 0;
}

static void addEspNowPeer(const uint8_t *mac)
{
    if (esp_now_is_peer_exist(mac))
        return;

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = WIFI_AP_CHANNEL;
    peer.encrypt = false;
#if defined(ESP_IDF_VERSION_MAJOR)
    peer.ifidx = WIFI_IF_AP;
#endif
    esp_now_add_peer(&peer);
}

static uint16_t decodeRtlopezAux(int8_t value)
{
    int decoded = RTLOPEZ_PWM_CENTER + (value * 5);
    return constrain(decoded, (int)RTLOPEZ_PWM_MIN, (int)RTLOPEZ_PWM_MAX);
}

static float pwmToNorm(int value)
{
    value = constrain(value, (int)RTLOPEZ_PWM_MIN, (int)RTLOPEZ_PWM_MAX);
    float norm = (value - (float)RTLOPEZ_PWM_CENTER) / (float)(RTLOPEZ_PWM_MAX - RTLOPEZ_PWM_CENTER);
    if (value < RTLOPEZ_PWM_CENTER)
        norm = (value - (float)RTLOPEZ_PWM_CENTER) / (float)(RTLOPEZ_PWM_CENTER - RTLOPEZ_PWM_MIN);
    return constrain(norm, -1.0f, 1.0f);
}

static float pwmToThrottle(int value)
{
    value = constrain(value, (int)RTLOPEZ_PWM_MIN, (int)RTLOPEZ_PWM_MAX);
    return constrain((value - (float)RTLOPEZ_PWM_MIN) / (float)(RTLOPEZ_PWM_MAX - RTLOPEZ_PWM_MIN), 0.0f, 1.0f);
}

static int decodeThreePositionSwitch(int pwm, int previous)
{
    (void)previous;
    if (pwm <= 1250) return 0;
    if (pwm >= 1750) return 2;
    return 1;
}

static int decodeArmModeSwitch(int pwm, int previous)
{
    if (pwm <= 1250) return 0;              // CH5 low around 1000us: disarmed
    if (pwm >= 1750) return 2;              // CH5 high around 2000us: armed
    if (pwm >= 1300 && pwm <= 1700) return 1; // CH5 middle around 1500us: perch
    return previous;                        // Hold while crossing switch gaps.
}

static bool isThreePositionMiddle(int pwm)
{
    return pwm >= 1300 && pwm <= 1700;
}

static int decodeTwoPositionSwitch(int pwm, int previous)
{
    if (pwm <= 1400) return 0;
    if (pwm >= 1600) return 1;
    return previous;
}

static void sendRtlopezTelemetry()
{
    if (!rtlopezPaired)
        return;

    float voltage = readBatteryVoltage();
    float percent = batteryVoltageToPercent(voltage);
    RtlopezFcMessage telemetry = {
        RTLOPEZ_FC_DATA,
        (int16_t)constrain((int)lroundf(voltage * 1000.0f), 0, 32767),
        (int16_t)constrain((int)lroundf(percent * 100.0f), 0, 100),
        (int8_t)getFlightMode(),
        (int8_t)getFlightProfileMode(),
        0
    };
    rtlopezSend(rtlopezPeerMac, telemetry);
}

static float pwmToPitchNeutral(int value)
{
    value = constrain(value, (int)RTLOPEZ_PWM_MIN, (int)RTLOPEZ_PWM_MAX);
    float t = (value - (float)RTLOPEZ_PWM_MIN) / (float)(RTLOPEZ_PWM_MAX - RTLOPEZ_PWM_MIN);
    return 80.0f + (t * (100.0f - 80.0f));
}

static void applyElrsFlightControlDirect(int ch1, int ch2, int ch3, int ch4);

static int mappedChannelValue(const int incoming[8], ChannelFunction function, int fallback)
{
    for (uint8_t i = 0; i < 8; ++i)
    {
        if (channelMap[i] == (int)function)
            return incoming[i];
    }
    return fallback;
}

static void applyRadioChannels(
    int rawThrottle,
    int ch1,
    int ch2,
    int ch3,
    int ch4,
    int ch5,
    int ch6,
    int ch7,
    int ch8)
{
    const unsigned long now = millis();
    int incoming[8] = {ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8};
    for (uint8_t i = 0; i < 8; ++i)
    {
        if (abs(incoming[i] - lastRadioPwm[i]) >= 2)
            lastRadioChannelChangeMs[i] = now;
        lastRadioPwm[i] = incoming[i];
    }
    lastControlPacket = now;

    const int rollCh = mappedChannelValue(incoming, CHANNEL_FUNC_ROLL, ch1);
    const int pitchCh = mappedChannelValue(incoming, CHANNEL_FUNC_PITCH, ch2);
    const int throttleCh = mappedChannelValue(incoming, CHANNEL_FUNC_THROTTLE, ch3);
    const int steeringCh = mappedChannelValue(incoming, CHANNEL_FUNC_STEERING, ch4);
    const int armCh = mappedChannelValue(incoming, CHANNEL_FUNC_ARM, ch5);
    const int waveformCh = mappedChannelValue(incoming, CHANNEL_FUNC_WAVEFORM, ch6);
    const int profileCh = mappedChannelValue(incoming, CHANNEL_FUNC_FLIGHT_MODE, ch7);
    const int ledCh = mappedChannelValue(incoming, CHANNEL_FUNC_LED_MODE, ch8);
    rawThrottle = throttleCh;

    radioArmSwitchPos = decodeArmModeSwitch(armCh, radioArmSwitchPos);
    const bool elrsCh5IsBinary = controlProtocol == CONTROL_PROTOCOL_ELRS &&
        (armCh <= 1250 || armCh >= 1750);
    const bool elrsPerchFallback = controlProtocol == CONTROL_PROTOCOL_ELRS &&
        elrsCh5IsBinary &&
        armCh <= 1250 &&
        isThreePositionMiddle(waveformCh);
    if (elrsPerchFallback)
        radioArmSwitchPos = 1;

    if (!elrsPerchFallback)
        radioWaveSwitchPos = decodeTwoPositionSwitch(waveformCh, radioWaveSwitchPos);
    radioProfileSwitchPos = decodeThreePositionSwitch(profileCh, radioProfileSwitchPos);
    radioLedSwitchPos = decodeThreePositionSwitch(ledCh, radioLedSwitchPos);

    int requestedLedMode = 1;
    if (radioLedSwitchPos == 0)
        requestedLedMode = 0;
    else if (radioLedSwitchPos == 2)
        requestedLedMode = 2;
    if (requestedLedMode != getLedMode())
        setLedMode(requestedLedMode);

    if (isBatteryServoFailsafeActive())
    {
        enforceBatteryServoFailsafe();
        float throttle = pwmToThrottle(throttleCh);
        if (throttle <= CONTROL_THROTTLE_DEADBAND)
        {
            if (isForceCenterActive())
                clearForceCenter();
            float steering = steeringLockEnabled ? 0.0f : pwmToNorm(steeringCh);
            setIdleLeanSteering(-steering * IDLE_LEAN_INPUT_AUTHORITY);
            lastThrottle = 0.0f;
            lastSteer = steering;
            updateFlightTimer(0.0f);
        }
        return;
    }

    if (radioArmSwitchPos == 1)
    {
        if (getFlightMode() != FLIGHT_MODE_PERCH)
        {
            setFlightMode(FLIGHT_MODE_PERCH);
        }
        if (!isPerchActive())
        {
            startPerch();
        }
        lastThrottle = 0.0f;
        lastSteer = 0.0f;
        updateFlightTimer(0.0f);
    }
    else if (radioArmSwitchPos == 0)
    {
        if (getFlightMode() != FLIGHT_MODE_DISARMED)
        {
            setFlightMode(FLIGHT_MODE_DISARMED);
            stopPerch();
            immediateStopAndCenter();
        }
        setRollBias(0.0f);
        setServoPitchAngleLive(getServoPitchAngle());
        setIdleLeanSteering(0.0f);
    }
    else if (radioArmSwitchPos == 2)
    {
        if (rawThrottle <= (RTLOPEZ_PWM_MIN + 20))
        {
            if (getFlightMode() != FLIGHT_MODE_ARMED)
            {
                setFlightMode(FLIGHT_MODE_ARMED);
                stopPerch();
            }
        }
        else if (getFlightMode() != FLIGHT_MODE_ARMED)
        {
            setFlightMode(FLIGHT_MODE_DISARMED);
            stopPerch();
            immediateStopAndCenter();
        }
    }

    FlightProfileMode requestedProfile = FLIGHT_PROFILE_CRUISE;
    if (radioProfileSwitchPos == 2)
        requestedProfile = FLIGHT_PROFILE_SPORT;
    else if (radioProfileSwitchPos == 1)
        requestedProfile = FLIGHT_PROFILE_HOVER;

    if (requestedProfile != getFlightProfileMode())
    {
        setFlightProfileMode(requestedProfile);
        applyCurrentFlightProfile();
    }

    int requestedWaveform = radioWaveSwitchPos == 1 ? 1 : 0;
    if (requestedWaveform != getFlapWaveform())
    {
        setFlapWaveform(requestedWaveform);
        setCurrentProfileWaveform(requestedWaveform);
    }

    if (getFlightMode() != FLIGHT_MODE_ARMED)
    {
        setRollBias(0.0f);
        if (getFlightMode() == FLIGHT_MODE_DISARMED)
            setServoPitchAngleLive(getServoPitchAngle());
    }
    else
    {
        setRollBias(pwmToNorm(rollCh) * MAX_ROLL_BIAS_DEGREES);
        setServoPitchAngleLive(pwmToPitchNeutral(pitchCh));
    }

    if (getFlightMode() == FLIGHT_MODE_ARMED)
    {
        if (controlProtocol == CONTROL_PROTOCOL_ELRS)
        {
            applyElrsFlightControlDirect(rollCh, pitchCh, throttleCh, steeringCh);
        }
        else
        {
            float steering = steeringLockEnabled ? 0.0f : pwmToNorm(steeringCh);
            float throttle = pwmToThrottle(throttleCh);
            applyPilotControlValues(steering, throttle, 0);
        }
    }
    else
    {
        lastThrottle = 0.0f;
        lastSteer = 0.0f;
        updateFlightTimer(0.0f);
    }
}

static void applyElrsFlightControlDirect(int ch1, int ch2, int ch3, int ch4)
{
    const unsigned long now = millis();
    if (abs(ch3 - lastAppliedThrottlePwm) >= 2)
    {
        lastAppliedThrottlePwm = ch3;
        lastThrottleChangeMs = now;
    }

    float throttle = pwmToThrottle(ch3);
    const bool throttleIdle = throttle <= CONTROL_THROTTLE_DEADBAND;

    lastControlPacket = now;
    lastThrottle = throttleIdle ? 0.0f : throttle;
    lastSteer = steeringLockEnabled ? 0.0f : pwmToNorm(ch4);
    updateFlightTimer(lastThrottle);
    clearForceCenter();

    setRollBias(pwmToNorm(ch1) * MAX_ROLL_BIAS_DEGREES);
    setServoPitchAngleLive(pwmToPitchNeutral(ch2));

    if (throttleIdle)
    {
        setIdleLeanSteering(-lastSteer * IDLE_LEAN_INPUT_AUTHORITY);
        return;
    }

    float scale = powf(throttle, STEERING_CURVE_EXPONENT);
    float steeringAuthority = STEERING_BASE_AUTHORITY +
        scale * (STEERING_MAX_AUTHORITY - STEERING_BASE_AUTHORITY);
    steeringAuthority *= getProfileSteeringMultiplier();
    steeringAuthority = fminf(steeringAuthority, MAX_FINAL_STEERING);
    setPilotSteering(-lastSteer * steeringAuthority);

    setFrequency(throttleToFrequency(throttle));

    float amplitude;
#if FIXED_AMPLITUDE_TEST_MODE
    amplitude = TEST_FLAP_AMPLITUDE_DEGREES;
#else
    amplitude = MIN_ACTIVE_FLAP_AMPLITUDE +
        throttle * (MAX_FLAP_AMPLITUDE - MIN_ACTIVE_FLAP_AMPLITUDE);
#endif
    amplitude = constrain(amplitude, MIN_FLAP_AMPLITUDE, MAX_FLAP_AMPLITUDE);
    setAmplitude(amplitude);
}

static void applyRtlopezRcMessage(const RtlopezRcMessage &message)
{
    if (controlProtocol != CONTROL_PROTOCOL_ESPNOW)
        return;

    applyRadioChannels(
        (int)message.ch3,
        constrain((int)message.ch1, (int)RTLOPEZ_PWM_MIN, (int)RTLOPEZ_PWM_MAX),
        constrain((int)message.ch2, (int)RTLOPEZ_PWM_MIN, (int)RTLOPEZ_PWM_MAX),
        constrain((int)message.ch3, (int)RTLOPEZ_PWM_MIN, (int)RTLOPEZ_PWM_MAX),
        constrain((int)message.ch4, (int)RTLOPEZ_PWM_MIN, (int)RTLOPEZ_PWM_MAX),
        decodeRtlopezAux(message.ch5),
        decodeRtlopezAux(message.ch6),
        decodeRtlopezAux(message.ch7),
        decodeRtlopezAux(message.ch8));
}

static void handleRtlopezPacket(const uint8_t *mac, const uint8_t *data, int len)
{
    if (controlProtocol != CONTROL_PROTOCOL_ESPNOW)
        return;

    if (len < 2 || rtlopezChecksum(data, len - 1) != data[len - 1])
    {
        ++rtlopezPacketsRejected;
        return;
    }

    if (data[0] == RTLOPEZ_PAIR_RES && len == (int)sizeof(RtlopezAliveMessage))
    {
        memcpy(rtlopezPendingMac, mac, 6);
        rtlopezPairResponsePending = true;
        ++rtlopezPairResponses;
        return;
    }

    if (data[0] == RTLOPEZ_RC_DATA && len == (int)sizeof(RtlopezRcMessage))
    {
        if (!rtlopezPaired || !macEquals(mac, rtlopezPeerMac))
        {
            ++rtlopezPacketsRejected;
            return;
        }
        memcpy(&rtlopezPendingRc, data, sizeof(rtlopezPendingRc));
        rtlopezRcPending = true;
        ++rtlopezPacketsReceived;
        lastRtlopezPacketMs = millis();
    }
}

static uint8_t crsfCrc8(const uint8_t *ptr, uint8_t len)
{
    uint8_t crc = 0;
    while (len--)
    {
        crc ^= *ptr++;
        for (uint8_t i = 0; i < 8; ++i)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
}

static int crsfToPwm(uint16_t value)
{
    value = constrain(value, (uint16_t)172, (uint16_t)1811);
    return 1000 + (int)(((uint32_t)(value - 172) * 1000UL) / (1811UL - 172UL));
}

static int crsfRawToPwm(uint16_t value, uint8_t bits)
{
    if (bits == 11)
        return crsfToPwm(value);

    const float scale = (float)(1UL << bits) / 2048.0f;
    uint16_t minRaw = (uint16_t)lroundf(172.0f * scale);
    uint16_t maxRaw = (uint16_t)lroundf(1811.0f * scale);
    value = constrain(value, minRaw, maxRaw);
    return 1000 + (int)(((uint32_t)(value - minRaw) * 1000UL) / (uint32_t)(maxRaw - minRaw));
}

static void unpackCrsfChannels(const uint8_t *payload)
{
    uint32_t bitBuffer = 0;
    uint8_t bitsInBuffer = 0;
    uint8_t byteIndex = 0;
    for (uint8_t ch = 0; ch < 16; ++ch)
    {
        while (bitsInBuffer < 11)
        {
            bitBuffer |= ((uint32_t)payload[byteIndex++]) << bitsInBuffer;
            bitsInBuffer += 8;
        }
        crsfChannels[ch] = bitBuffer & 0x07FF;
        bitBuffer >>= 11;
        bitsInBuffer -= 11;
    }
}

static void unpackCrsfSubsetChannels(const uint8_t *payload, uint8_t payloadLen)
{
    if (payloadLen < 2)
        return;

    uint8_t config = payload[0];
    uint8_t startChannel = config & 0x1F;
    uint8_t resolutionConfig = (config >> 5) & 0x03;
    uint8_t bitsPerChannel = 10 + resolutionConfig;
    if (bitsPerChannel < 10 || bitsPerChannel > 13 || startChannel >= 16)
        return;

    uint32_t bitBuffer = 0;
    uint8_t bitsInBuffer = 0;
    uint8_t byteIndex = 1;
    uint8_t channel = startChannel;
    while (channel < 16 && byteIndex <= payloadLen)
    {
        while (bitsInBuffer < bitsPerChannel && byteIndex < payloadLen)
        {
            bitBuffer |= ((uint32_t)payload[byteIndex++]) << bitsInBuffer;
            bitsInBuffer += 8;
        }
        if (bitsInBuffer < bitsPerChannel)
            break;

        uint16_t raw = bitBuffer & ((1U << bitsPerChannel) - 1U);
        if (bitsPerChannel == 11)
            crsfChannels[channel] = raw;
        else
            crsfChannels[channel] = constrain(
                map(crsfRawToPwm(raw, bitsPerChannel), 1000, 2000, 172, 1811),
                172, 1811);

        bitBuffer >>= bitsPerChannel;
        bitsInBuffer -= bitsPerChannel;
        ++channel;
    }
}

static void noteCrsfRcFrame(uint8_t type)
{
    unsigned long now = millis();
    crsfLastRcFrameType = type;
    lastCrsfPacketMs = now;

    if (lastCrsfApplyMs != 0)
    {
        lastCrsfGapMs = now - lastCrsfApplyMs;
        if (lastCrsfGapMs > maxCrsfApplyGapMs)
            maxCrsfApplyGapMs = lastCrsfGapMs;
    }
    lastCrsfApplyMs = now;

    if (crsfRateWindowStartMs == 0)
        crsfRateWindowStartMs = now;
    ++crsfFramesInWindow;
    unsigned long elapsed = now - crsfRateWindowStartMs;
    if (elapsed >= 500)
    {
        crsfFramesPerSecond = (uint16_t)((crsfFramesInWindow * 1000UL) / elapsed);
        crsfFramesInWindow = 0;
        crsfRateWindowStartMs = now;
    }
}

static void applyCrsfChannels()
{
    if (controlProtocol != CONTROL_PROTOCOL_ELRS)
        return;

    int ch[8];
    for (uint8_t i = 0; i < 8; ++i)
        ch[i] = crsfToPwm(crsfChannels[i]);

    applyRadioChannels(
        ch[2],
        ch[0], ch[1], ch[2], ch[3],
        ch[4], ch[5], ch[6], ch[7]);
}

static void publishCrsfChannelsFromParser()
{
    int ch[8];
    for (uint8_t i = 0; i < 8; ++i)
        ch[i] = crsfToPwm(crsfChannels[i]);

    portENTER_CRITICAL(&elrsPendingMux);
    for (uint8_t i = 0; i < 8; ++i)
        elrsPendingPwm[i] = ch[i];
    elrsPendingChannels = true;
    portEXIT_CRITICAL(&elrsPendingMux);
}

static void applyPendingElrsChannels()
{
    if (controlProtocol != CONTROL_PROTOCOL_ELRS)
        return;

    int ch[8];
    bool pending = false;
    portENTER_CRITICAL(&elrsPendingMux);
    if (elrsPendingChannels)
    {
        for (uint8_t i = 0; i < 8; ++i)
            ch[i] = elrsPendingPwm[i];
        elrsPendingChannels = false;
        pending = true;
    }
    portEXIT_CRITICAL(&elrsPendingMux);

    if (pending)
    {
        applyRadioChannels(
            ch[2],
            ch[0], ch[1], ch[2], ch[3],
            ch[4], ch[5], ch[6], ch[7]);
    }
}

static bool handleCrsfFrame(const uint8_t *frame, uint8_t frameSize, bool applyNow)
{
    if (frameSize < 4)
        return false;
    uint8_t length = frame[1];
    uint8_t type = frame[2];
    uint8_t crc = frame[frameSize - 1];
    if (crsfCrc8(&frame[2], length - 1) != crc)
    {
        ++crsfCrcFails;
        ++crsfPacketsRejected;
        return false;
    }

    ++crsfValidFrames;
    crsfLastFrameType = type;
    crsfLastFrameLength = length;

    if (type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED && length == 24)
    {
        unpackCrsfChannels(&frame[3]);
        ++crsfPacketsReceived;
        ++crsfPackedRcFrames;
        noteCrsfRcFrame(type);
        if (applyNow)
            publishCrsfChannelsFromParser();
        return true;
    }
    else if (type == CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED)
    {
        uint8_t payloadLen = length - 2;
        unpackCrsfSubsetChannels(&frame[3], payloadLen);
        ++crsfPacketsReceived;
        ++crsfSubsetRcFrames;
        noteCrsfRcFrame(type);
        if (applyNow)
            publishCrsfChannelsFromParser();
        return true;
    }
    ++crsfUnknownFrames;
    return false;
}

static void updateElrsReceiver()
{
    int queued = Serial1.available();
    crsfLastSerialAvailable = (uint16_t)constrain(queued, 0, 65535);
    if (crsfLastSerialAvailable > crsfMaxSerialAvailable)
        crsfMaxSerialAvailable = crsfLastSerialAvailable;

    applyPendingElrsChannels();
}

static void processCrsfByte(uint8_t b)
{
    ++crsfRawBytes;

    if (crsfIndex == 0)
    {
        if (!isCrsfAddressByte(b))
            return;
        ++crsfSyncBytes;
        crsfFrame[crsfIndex++] = b;
        return;
    }

    crsfFrame[crsfIndex++] = b;
    if (crsfIndex == 2)
    {
        crsfExpectedLength = b + 2;
        if (crsfExpectedLength < 4 || crsfExpectedLength > CRSF_MAX_FRAME_SIZE)
        {
            ++crsfLengthFails;
            ++crsfPacketsRejected;
            crsfIndex = 0;
            crsfExpectedLength = 0;
        }
        return;
    }

    if (crsfExpectedLength > 0 && crsfIndex >= crsfExpectedLength)
    {
        handleCrsfFrame(crsfFrame, crsfExpectedLength, true);
        crsfIndex = 0;
        crsfExpectedLength = 0;
    }
}

static void elrsReceiverTask(void *)
{
    for (;;)
    {
        uint16_t processed = 0;
        while (Serial1.available() && processed < CRSF_MAX_PARSE_BYTES_PER_LOOP)
        {
            processCrsfByte(Serial1.read());
            ++processed;
        }

        int queued = Serial1.available();
        if (queued > CRSF_BACKLOG_FLUSH_THRESHOLD)
        {
            ++crsfBacklogFlushes;
            uint16_t flushed = 0;
            while (Serial1.available() > CRSF_BACKLOG_KEEP_BYTES &&
                   flushed < CRSF_MAX_FLUSH_BYTES_PER_LOOP)
            {
                Serial1.read();
                ++crsfBytesDropped;
                ++flushed;
            }
            crsfIndex = 0;
            crsfExpectedLength = 0;
            crsfParseLength = 0;
        }

        if (processed == 0)
            vTaskDelay(1);
        else
            taskYIELD();
    }
}

static void flushCrsfInputBounded()
{
    const unsigned long startUs = micros();
    uint16_t flushed = 0;
    while (Serial1.available() &&
           flushed < CRSF_MAX_FLUSH_BYTES_PER_LOOP &&
           (micros() - startUs) < CRSF_FLUSH_BUDGET_US)
    {
        Serial1.read();
        ++crsfBytesDropped;
        ++flushed;
    }
    crsfIndex = 0;
    crsfExpectedLength = 0;
    crsfParseLength = 0;
}

static void sendCrsfBatteryTelemetry(float voltage, float percent)
{
    uint8_t frame[12] = {};
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = 10; // type + 8-byte battery payload + crc
    frame[2] = CRSF_FRAMETYPE_BATTERY_SENSOR;
    uint16_t decivolts = (uint16_t)constrain((int)lroundf(voltage * 10.0f), 0, 65535);
    frame[3] = decivolts >> 8;
    frame[4] = decivolts & 0xFF;
    frame[5] = 0; // current, 0.1A units. Not measured on this craft.
    frame[6] = 0;
    frame[7] = 0; // consumed capacity, mAh. Not measured on this craft.
    frame[8] = 0;
    frame[9] = 0;
    frame[10] = (uint8_t)constrain((int)lroundf(percent * 100.0f), 0, 100);
    frame[11] = crsfCrc8(&frame[2], frame[1] - 1);
    Serial1.write(frame, sizeof(frame));
}

static void onEspNowReceive(const uint8_t *mac, const uint8_t *data, int len)
{
    (void)mac;
    if (len >= 2 && (data[0] == RTLOPEZ_PAIR_RES || data[0] == RTLOPEZ_RC_DATA))
    {
        handleRtlopezPacket(mac, data, len);
        return;
    }

    if (len != (int)sizeof(EspFlutterNowPacket))
    {
        ++espNowPacketsRejected;
        return;
    }

    EspFlutterNowPacket packet;
    memcpy(&packet, data, sizeof(packet));
    if (packet.magic != ESP_FLUTTER_ESPNOW_MAGIC ||
        packet.version != ESP_FLUTTER_ESPNOW_VERSION ||
        packet.size != sizeof(EspFlutterNowPacket))
    {
        ++espNowPacketsRejected;
        return;
    }

    espNowPendingPacket = packet;
    espNowPacketPending = true;
    ++espNowPacketsReceived;
    lastEspNowPacketMs = millis();
}

static void initEspNowReceiver()
{
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW init failed");
        return;
    }
    addEspNowPeer(ESPNOW_BROADCAST_PEER);
    esp_now_register_recv_cb(onEspNowReceive);
    Serial.printf("ESP-NOW receiver ready on WiFi channel %d\n", WiFi.channel());
}

static void updateEspNowReceiver()
{
    unsigned long now = millis();
    if (controlProtocol != CONTROL_PROTOCOL_ESPNOW)
    {
        rtlopezRcPending = false;
        rtlopezPairResponsePending = false;
        if (espNowPacketPending)
        {
            EspFlutterNowPacket packet = espNowPendingPacket;
            espNowPacketPending = false;
            applyEspNowPacket(packet);
        }
        return;
    }

    if (!rtlopezPaired && now >= rtlopezNextBeaconMs)
    {
        RtlopezPairRequest request = {RTLOPEZ_PAIR_REQ, WIFI_AP_CHANNEL, 0};
        rtlopezSend(ESPNOW_BROADCAST_PEER, request);
        rtlopezNextBeaconMs = now + RTLOPEZ_BEACON_INTERVAL_MS;
    }

    if (rtlopezPairResponsePending)
    {
        memcpy(rtlopezPeerMac, rtlopezPendingMac, 6);
        addEspNowPeer(rtlopezPeerMac);
        rtlopezPaired = true;
        rtlopezPairResponsePending = false;
        rtlopezNextAliveMs = 0;
        rtlopezNextTelemetryMs = 0;
        Serial.printf("rtlopez ESP-NOW RC paired: %02X:%02X:%02X:%02X:%02X:%02X\n",
            rtlopezPeerMac[0], rtlopezPeerMac[1], rtlopezPeerMac[2],
            rtlopezPeerMac[3], rtlopezPeerMac[4], rtlopezPeerMac[5]);
    }

    if (rtlopezPaired && now >= rtlopezNextAliveMs)
    {
        RtlopezAliveMessage alive = {RTLOPEZ_FC_ALIVE, 0};
        rtlopezSend(rtlopezPeerMac, alive);
        rtlopezNextAliveMs = now + RTLOPEZ_ALIVE_INTERVAL_MS;
    }

    if (rtlopezPaired && now >= rtlopezNextTelemetryMs)
    {
        sendRtlopezTelemetry();
        rtlopezNextTelemetryMs = now + RTLOPEZ_TELEMETRY_INTERVAL_MS;
    }

    if (rtlopezRcPending)
    {
        RtlopezRcMessage message = rtlopezPendingRc;
        rtlopezRcPending = false;
        applyRtlopezRcMessage(message);
    }

    if (!espNowPacketPending)
        return;

    EspFlutterNowPacket packet = espNowPendingPacket;
    espNowPacketPending = false;
    applyEspNowPacket(packet);
}

void handleControl()
{
    if (controlProtocol != CONTROL_PROTOCOL_WIFI)
    {
        server.send(200, "text/plain", "PROTOCOL_LOCKED");
        return;
    }

    if (server.hasArg("x") && server.hasArg("y"))
    {
        float x =
            server.arg("x").toFloat();

        float y =
            server.arg("y").toFloat();

        x = constrain(x, -1.0f, 1.0f);
        y = constrain(y, 0.0f, 1.0f);
        if (steeringLockEnabled)
            x = 0.0f;
        uint32_t seq = server.hasArg("seq")
            ? (uint32_t)strtoul(server.arg("seq").c_str(), nullptr, 10)
            : 0;
        if (isStaleControlSeq(seq))
        {
            server.send(200, "text/plain", "STALE");
            return;
        }
        noteControlSeq(seq);
        const bool throttleIdle = y <= CONTROL_THROTTLE_DEADBAND;
        const bool idleNeutral = fabsf(x) < 0.05f && throttleIdle;

        if (isBatteryServoFailsafeActive())
        {
            enforceBatteryServoFailsafe();
            if (throttleIdle)
            {
                if (isForceCenterActive())
                    clearForceCenter();
                setIdleLeanSteering(-x * IDLE_LEAN_INPUT_AUTHORITY);
                lastThrottle = 0.0f;
                lastSteer = x;
                updateFlightTimer(0.0f);
                server.send(200, "text/plain", "IDLE_LEAN_CUTOFF");
            }
            else
            {
                server.send(200, "text/plain", "BATTERY_CUTOFF");
            }
            return;
        }

        if (!throttleIdle && isControlReleaseLocked())
        {
            server.send(200, "text/plain", "IGNORED");
            return;
        }

        // If wings are currently forced to center, ignore small follow-up packets (debounce)
        if (isForceCenterActive())
        {
            if (false)
            {
                // still a near-zero packet; ignore to avoid canceling forced-center
                server.send(200, "text/plain", "IGNORED");
                return;
            }
            else
            {
                // real input from pilot — clear forced-center and continue
                clearForceCenter();
            }
        }

        if (getFlightMode() == FLIGHT_MODE_DISARMED)
        {
            immediateStopAndCenter();
            lastThrottle = 0.0f;
            lastSteer = 0.0f;
            updateFlightTimer(0.0f);
            server.send(200, "text/plain", "LOCKED");
            return;
        }

        // If in TEST mode, ignore control packets and keep wings centered
        if (getFlightMode() == FLIGHT_MODE_PERCH)
        {
            // In perch mode, ignore incoming control packets to avoid interrupting the decorative motion
            server.send(200, "text/plain", "PERCH");
            return;
        }

        lastControlPacket = millis();
        lastThrottle = throttleIdle ? 0.0f : y;
        lastSteer = x;
        updateFlightTimer(lastThrottle);

        if (throttleIdle)
        {
            setIdleLeanSteering(-x * IDLE_LEAN_INPUT_AUTHORITY);
            server.send(200, "text/plain", "OK");
            return;
        }

        // ===== PILOT STEERING (throttle-dependent authority) =====
        // y is throttle in [0..1]. Use a configurable curve so steering is gentle at low throttle
        // and progressively stronger at high throttle.
        float t = y;
        float scale = powf(t, STEERING_CURVE_EXPONENT);
        float steeringAuthority = STEERING_BASE_AUTHORITY +
            scale * (STEERING_MAX_AUTHORITY - STEERING_BASE_AUTHORITY);
        steeringAuthority *= getProfileSteeringMultiplier();
        steeringAuthority = fminf(steeringAuthority, MAX_FINAL_STEERING);

        setPilotSteering(-x * steeringAuthority);

        // ===== FLAP FREQUENCY =====
        float freq = throttleToFrequency(y);

        setFrequency(freq);

        float amplitude;

#if FIXED_AMPLITUDE_TEST_MODE
        amplitude = TEST_FLAP_AMPLITUDE_DEGREES;
#else
        amplitude = MIN_ACTIVE_FLAP_AMPLITUDE +
            y * (MAX_FLAP_AMPLITUDE - MIN_ACTIVE_FLAP_AMPLITUDE);
#endif

        // Enforce amplitude safety limits on the computed automatic amplitude.
        if (amplitude < MIN_FLAP_AMPLITUDE) amplitude = MIN_FLAP_AMPLITUDE;
        if (amplitude > MAX_FLAP_AMPLITUDE) amplitude = MAX_FLAP_AMPLITUDE;

        float finalAmp = amplitude;
        if (finalAmp < MIN_FLAP_AMPLITUDE) finalAmp = MIN_FLAP_AMPLITUDE;
        if (finalAmp > MAX_FLAP_AMPLITUDE) finalAmp = MAX_FLAP_AMPLITUDE;

        setAmplitude(finalAmp);

    }

    server.send(200, "text/plain", "OK");
}

void initWiFiControl()
{
    loadControlPreferences();
    initFlightProfiles();
    applyCurrentFlightProfile();

    WiFi.mode(WIFI_AP);
    WiFi.setTxPower(WIFI_POWER_2dBm);
    WiFi.softAP(ssid, password, WIFI_AP_CHANNEL);
    initEspNowReceiver();
    Serial1.setRxBufferSize(2048);
    Serial1.begin(ELRS_BAUD, SERIAL_8N1, getElrsRxPin(), getElrsTxPin());
    Serial1.setTimeout(0);
    if (elrsTaskHandle == nullptr)
    {
        xTaskCreate(elrsReceiverTask, "elrs_rx", 4096, nullptr, 3, &elrsTaskHandle);
    }

    Serial.println("");
    Serial.println("WiFi AP Started");

    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", handleRoot);

    // Initialize persistent flight mode (do not auto-start Perch on boot)
    initFlightMode();
    // Ensure perch is stopped at startup; user must tap Perch to enable it.
    stopPerch();

    // Prepare battery pin for ADC reads
    pinMode(getBatteryPin(), INPUT);
    pinMode(getBatteryWarningLedPin(), OUTPUT);
    pinMode(getStatusLedPin(), OUTPUT);
    digitalWrite(getBatteryWarningLedPin(), LOW);
    digitalWrite(getStatusLedPin(), LOW);
    analogReadResolution(12);
    analogSetPinAttenuation(getBatteryPin(), ADC_11db);
    updateCachedBatteryTelemetry(true);

    server.on("/control", handleControl);

    server.on("/setControlProtocol", []()
    {
        if (!server.hasArg("p")) { server.send(400, "text/plain", "missing protocol"); return; }
        int p = server.arg("p").toInt();
        if (p == CONTROL_PROTOCOL_ESPNOW)
            controlProtocol = CONTROL_PROTOCOL_ESPNOW;
        else if (p == CONTROL_PROTOCOL_ELRS)
            controlProtocol = CONTROL_PROTOCOL_ELRS;
        else
            controlProtocol = CONTROL_PROTOCOL_WIFI;
        saveControlProtocol();
        if (controlProtocol == CONTROL_PROTOCOL_WIFI)
        {
            rtlopezPaired = false;
            rtlopezRcPending = false;
            rtlopezPairResponsePending = false;
        }
        else
        {
            rtlopezNextBeaconMs = 0;
            rtlopezNextAliveMs = 0;
            rtlopezNextTelemetryMs = 0;
            crsfNextTelemetryMs = 0;
        }
        if (controlProtocol == CONTROL_PROTOCOL_ELRS)
        {
            flushCrsfInputBounded();
            crsfIndex = 0;
            crsfExpectedLength = 0;
            crsfParseLength = 0;
            crsfPacketsReceived = 0;
            crsfPacketsRejected = 0;
            crsfPackedRcFrames = 0;
            crsfSubsetRcFrames = 0;
            crsfValidFrames = 0;
            crsfUnknownFrames = 0;
            crsfLastRcFrameType = 0;
            crsfLastFrameType = 0;
            crsfLastFrameLength = 0;
            crsfBytesDropped = 0;
            crsfRawBytes = 0;
            crsfSyncBytes = 0;
            crsfLengthFails = 0;
            crsfCrcFails = 0;
            lastCrsfPacketMs = 0;
            lastCrsfApplyMs = 0;
            maxCrsfApplyGapMs = 0;
            lastCrsfGapMs = 0;
            crsfRateWindowStartMs = 0;
            crsfFramesInWindow = 0;
            crsfFramesPerSecond = 0;
            crsfLastSerialAvailable = 0;
            crsfMaxSerialAvailable = 0;
            crsfBacklogFlushes = 0;
            for (uint8_t i = 0; i < 16; ++i)
                crsfChannels[i] = 992;
            lastThrottleChangeMs = 0;
            lastAppliedThrottlePwm = 1000;
            for (uint8_t i = 0; i < 8; ++i)
                lastRadioChannelChangeMs[i] = 0;
        }
#if ELRS_RF_ISOLATION_WIFI_OFF
        if (controlProtocol == CONTROL_PROTOCOL_ELRS)
        {
            elrsWifiShutdownPending = true;
            elrsWifiShutdownAtMs = millis() + 2500UL;
        }
        else
        {
            elrsWifiShutdownPending = false;
        }
#endif
        immediateStopAndCenter();
        lastThrottle = 0.0f;
        lastSteer = 0.0f;
        updateFlightTimer(0.0f);
        server.send(200, "text/plain", "OK");
    });

    server.on("/setSteeringLock", []()
    {
        if (!server.hasArg("enabled")) { server.send(400, "text/plain", "missing enabled"); return; }
        steeringLockEnabled = server.arg("enabled").toInt() != 0;
        if (steeringLockEnabled)
        {
            lastSteer = 0.0f;
            setPilotSteering(0.0f);
            setIdleLeanSteering(0.0f);
        }
        server.send(200, "text/plain", "OK");
    });

    server.on("/setBatteryConfig", []()
    {
        if (server.hasArg("enabled")) batteryMonitorEnabled = server.arg("enabled").toInt() != 0;
        if (server.hasArg("top"))
        {
            batteryDividerTopOhms = server.arg("top").toFloat();
            if (batteryDividerTopOhms < 1000.0f) batteryDividerTopOhms *= 1000.0f;
        }
        if (server.hasArg("bottom"))
        {
            batteryDividerBottomOhms = server.arg("bottom").toFloat();
            if (batteryDividerBottomOhms < 1000.0f) batteryDividerBottomOhms *= 1000.0f;
        }
        if (server.hasArg("cells")) batteryCellCount = (uint8_t)constrain(server.arg("cells").toInt(), 1, 2);
        if (server.hasArg("chem")) batteryChemistry = (uint8_t)server.arg("chem").toInt();
        if (server.hasArg("failsafe")) batteryFailsafeEnabled = server.arg("failsafe").toInt() != 0;
        if (server.hasArg("cutoff")) batteryFailsafeCutoffVoltage = server.arg("cutoff").toFloat();
        if (batteryDividerTopOhms < 1.0f) batteryDividerTopOhms = BATTERY_DIVIDER_TOP_OHMS;
        if (batteryDividerBottomOhms < 1.0f) batteryDividerBottomOhms = BATTERY_DIVIDER_BOTTOM_OHMS;
        if (batteryCellCount < 1) batteryCellCount = 1;
        if (batteryCellCount > 2) batteryCellCount = 2;
        if (batteryChemistry > 1) batteryChemistry = 0;
        if (batteryFailsafeCutoffVoltage < 0.0f) batteryFailsafeCutoffVoltage = 0.0f;
        if (batteryFailsafeCutoffVoltage > 30.0f) batteryFailsafeCutoffVoltage = 30.0f;
        batteryWarningLatched = false;
        ++batteryConfigRevision;
        saveBatteryConfig();
        updateCachedBatteryTelemetry(true);
        enforceBatteryServoFailsafe();
        server.send(200, "text/plain", "OK");
    });

    // Telemetry endpoint used by web UI (~8Hz)
    server.on("/telemetry", []()
    {
        float telemetryThrottle = lastThrottle;
        float freq = getFlapFrequency();

        // Battery
        updateCachedBatteryTelemetry(false);
        float battV = cachedBatteryVoltage;
        float battPct = cachedBatteryPercent;
        bool battLow = cachedBatteryLow;
        bool battCritical = cachedBatteryCritical;

        // Build telemetry JSON (include trims)
        String json = "{";
        json += "\"throttle\":" + String(telemetryThrottle, 3) + ",";
        json += "\"frequency\":" + String(freq, 3) + ",";
        json += "\"amplitude\":" + String(getFlapAmplitude(), 3) + ",";
        json += "\"amplitudePct\":" + String(getFlapAmplitude() / MAX_FLAP_AMPLITUDE, 3) + ",";
        // Expose current amplitude scale (0.6..1.0).
        json += "\"amplitudeScale\":" + String(amplitudeScale, 3) + ",";
        json += "\"lowerAmplitudeScale\":" + String(lowerAmplitudeScale, 3) + ",";
        json += "\"upperAmplitudeScale\":" + String(upperAmplitudeScale, 3) + ",";
        json += "\"lowerAmplitudeDegrees\":" + String(lowerAmplitudeScale * MAX_DOWNSTROKE_BELOW_NEUTRAL, 1) + ",";
        json += "\"upperAmplitudeDegrees\":" + String(upperAmplitudeScale * MAX_UPSTROKE_ABOVE_NEUTRAL, 1) + ",";
        json += "\"maxFlapDegrees\":" + String(fmaxf(MAX_UPSTROKE_ABOVE_NEUTRAL, MAX_DOWNSTROKE_BELOW_NEUTRAL), 1) + ",";
        json += "\"steeringDiffGain\":" + String(getSteeringDifferentialGain(), 3) + ",";
        json += "\"turnCompensation\":" + String(getTurnCompensation(), 3) + ",";
        json += "\"flightProfile\":" + String((int)getFlightProfileMode()) + ",";
        json += "\"flightProfileName\":\"" + String(getFlightProfileName()) + "\",";
        json += "\"frequencyCap\":" + String(getProfileFrequencyCap(), 3) + ",";
        json += "\"waveform\":" + String(getFlapWaveform()) + ",";
        json += "\"servoRefreshHz\":" + String(getServoRefreshRateHz()) + ",";
        json += "\"servoMinUs\":" + String(getServoMinPulseUs()) + ",";
        json += "\"servoMaxUs\":" + String(getServoMaxPulseUs()) + ",";
        json += "\"leftServoPin\":" + String(getLeftServoPin()) + ",";
        json += "\"rightServoPin\":" + String(getRightServoPin()) + ",";
        json += "\"ledPin\":" + String(getLedPin()) + ",";
        json += "\"batteryPin\":" + String(getBatteryPin()) + ",";
        json += "\"batteryWarningLedPin\":" + String(getBatteryWarningLedPin()) + ",";
        json += "\"statusLedPin\":" + String(getStatusLedPin()) + ",";
        json += "\"elrsRxPin\":" + String(getElrsRxPin()) + ",";
        json += "\"elrsTxPin\":" + String(getElrsTxPin()) + ",";
        json += "\"controlProtocol\":" + String((int)controlProtocol) + ",";
        for (uint8_t i = 0; i < 8; ++i)
            json += "\"channelMap" + String(i + 1) + "\":" + String(channelMap[i]) + ",";
        json += "\"steeringLock\":" + String(steeringLockEnabled ? 1 : 0) + ",";
        json += "\"espNowPackets\":" + String(espNowPacketsReceived) + ",";
        json += "\"espNowRejected\":" + String(espNowPacketsRejected) + ",";
        json += "\"espNowAgeMs\":" + String(lastEspNowPacketMs == 0 ? 0 : (millis() - lastEspNowPacketMs)) + ",";
        json += "\"rtlopezPaired\":" + String(rtlopezPaired ? 1 : 0) + ",";
        json += "\"rtlopezPackets\":" + String(rtlopezPacketsReceived) + ",";
        json += "\"rtlopezRejected\":" + String(rtlopezPacketsRejected) + ",";
        json += "\"rtlopezPairResponses\":" + String(rtlopezPairResponses) + ",";
        json += "\"rtlopezAgeMs\":" + String(lastRtlopezPacketMs == 0 ? 0 : (millis() - lastRtlopezPacketMs)) + ",";
        json += "\"crsfPackets\":" + String(crsfPacketsReceived) + ",";
        json += "\"crsfRejected\":" + String(crsfPacketsRejected) + ",";
        json += "\"crsfRc16\":" + String(crsfPackedRcFrames) + ",";
        json += "\"crsfRc17\":" + String(crsfSubsetRcFrames) + ",";
        json += "\"crsfLastType\":" + String(crsfLastFrameType) + ",";
        json += "\"crsfLastAnyType\":" + String(crsfLastFrameType) + ",";
        json += "\"crsfLastLength\":" + String(crsfLastFrameLength) + ",";
        json += "\"crsfValidFrames\":" + String(crsfValidFrames) + ",";
        json += "\"crsfUnknownFrames\":" + String(crsfUnknownFrames) + ",";
        json += "\"crsfDropped\":" + String(crsfBytesDropped) + ",";
        json += "\"crsfRawBytes\":" + String(crsfRawBytes) + ",";
        json += "\"crsfSyncBytes\":" + String(crsfSyncBytes) + ",";
        json += "\"crsfLengthFails\":" + String(crsfLengthFails) + ",";
        json += "\"crsfCrcFails\":" + String(crsfCrcFails) + ",";
        json += "\"crsfAgeMs\":" + String(lastCrsfPacketMs == 0 ? 0 : (millis() - lastCrsfPacketMs)) + ",";
        json += "\"crsfMaxGapMs\":" + String(maxCrsfApplyGapMs) + ",";
        json += "\"crsfLastGapMs\":" + String(lastCrsfGapMs) + ",";
        json += "\"crsfFps\":" + String(crsfFramesPerSecond) + ",";
        json += "\"crsfAvail\":" + String(crsfLastSerialAvailable) + ",";
        json += "\"crsfMaxAvail\":" + String(crsfMaxSerialAvailable) + ",";
        json += "\"crsfBacklogFlushes\":" + String(crsfBacklogFlushes) + ",";
        json += "\"elrsBaud\":" + String(ELRS_BAUD) + ",";
        json += "\"radioCh1\":" + String(lastRadioPwm[0]) + ",";
        json += "\"radioCh2\":" + String(lastRadioPwm[1]) + ",";
        json += "\"radioCh3\":" + String(lastRadioPwm[2]) + ",";
        json += "\"radioCh4\":" + String(lastRadioPwm[3]) + ",";
        json += "\"radioCh5\":" + String(lastRadioPwm[4]) + ",";
        json += "\"radioCh6\":" + String(lastRadioPwm[5]) + ",";
        json += "\"radioCh7\":" + String(lastRadioPwm[6]) + ",";
        json += "\"radioCh8\":" + String(lastRadioPwm[7]) + ",";
        json += "\"radioArmPos\":" + String(radioArmSwitchPos) + ",";
        json += "\"radioThrottleAgeMs\":" + String(lastThrottleChangeMs == 0 ? 0 : (millis() - lastThrottleChangeMs)) + ",";
        json += "\"radioCh1AgeMs\":" + String(lastRadioChannelChangeMs[0] == 0 ? 0 : (millis() - lastRadioChannelChangeMs[0])) + ",";
        json += "\"radioCh2AgeMs\":" + String(lastRadioChannelChangeMs[1] == 0 ? 0 : (millis() - lastRadioChannelChangeMs[1])) + ",";
        json += "\"radioCh3AgeMs\":" + String(lastRadioChannelChangeMs[2] == 0 ? 0 : (millis() - lastRadioChannelChangeMs[2])) + ",";
        json += "\"radioCh4AgeMs\":" + String(lastRadioChannelChangeMs[3] == 0 ? 0 : (millis() - lastRadioChannelChangeMs[3])) + ",";
        json += "\"radioCh5AgeMs\":" + String(lastRadioChannelChangeMs[4] == 0 ? 0 : (millis() - lastRadioChannelChangeMs[4])) + ",";
        json += "\"radioCh6AgeMs\":" + String(lastRadioChannelChangeMs[5] == 0 ? 0 : (millis() - lastRadioChannelChangeMs[5])) + ",";
        json += "\"radioCh7AgeMs\":" + String(lastRadioChannelChangeMs[6] == 0 ? 0 : (millis() - lastRadioChannelChangeMs[6])) + ",";
        json += "\"radioCh8AgeMs\":" + String(lastRadioChannelChangeMs[7] == 0 ? 0 : (millis() - lastRadioChannelChangeMs[7])) + ",";
        json += "\"currentFlightMs\":" + String(getCurrentFlightSessionMs()) + ",";
        json += "\"flightMs\":" + String(getFlightTimerMs()) + ",";
        json += "\"recordMs\":" + String(getFlightRecordMs()) + ",";
        json += "\"uptimeMs\":" + String(millis()) + ",";
        json += "\"pilot\":" + String(getPilotSteeringCurrent(), 3) + ",";
        json += "\"servoPitch\":" + String(getServoPitchAngle(), 3) + ",";
        json += "\"currentNeutralAngle\":" + String(getCurrentNeutralAngle(), 3) + ",";
        json += "\"hoverBiasEnabled\":" + String(isHoverBiasEnabled() ? 1 : 0) + ",";
        json += "\"hoverBias\":" + String(getHoverBias(), 3) + ",";
        json += "\"hoverDevicePitch\":" + String(getHoverBiasDevicePitch(), 3) + ",";
        json += "\"hoverForwardNeutral\":" + String(getHoverBiasForwardNeutralAngle(), 3) + ",";
        json += "\"hoverNeutral\":" + String(getHoverBiasHoverNeutralAngle(), 3) + ",";
        json += "\"ledMode\":" + String(getLedMode()) + ",";
        json += "\"ledBrightness\":" + String(getLedBrightnessLevel()) + ",";
        json += "\"ledSpeed\":" + String(getLedEffectSpeedLevel()) + ",";
        json += "\"battery\":" + String(battV, 3) + ",";
        json += "\"batteryPct\":" + String(battPct, 3) + ",";
        json += "\"batteryLow\":" + String(battLow ? 1 : 0) + ",";
        json += "\"batteryCritical\":" + String(battCritical ? 1 : 0) + ",";
        json += "\"batteryEnabled\":" + String(batteryMonitorEnabled ? 1 : 0) + ",";
        json += "\"batteryTopOhms\":" + String(batteryDividerTopOhms, 1) + ",";
        json += "\"batteryBottomOhms\":" + String(batteryDividerBottomOhms, 1) + ",";
        json += "\"batteryCells\":" + String(batteryCellCount) + ",";
        json += "\"batteryChemistry\":" + String(batteryChemistry) + ",";
        json += "\"batteryAdcSaturated\":" + String(batteryAdcSaturated ? 1 : 0) + ",";
        json += "\"batteryFailsafeEnabled\":" + String(batteryFailsafeEnabled ? 1 : 0) + ",";
        json += "\"batteryFailsafeCutoff\":" + String(batteryFailsafeCutoffVoltage, 2) + ",";
        json += "\"batteryServoFailsafe\":" + String(batteryServoFailsafeActive ? 1 : 0) + ",";
        json += "\"leftTrim\":" + String(getLeftTrim()) + ",";
        json += "\"rightTrim\":" + String(getRightTrim()) + ",";
        json += "\"steeringTrim\":" + String(getSteeringTrim(), 3);

        // Append mode string
        String modeStr = "UNKNOWN";
        switch (getFlightMode())
        {
            case FLIGHT_MODE_DISARMED: modeStr = "DISARMED"; break;
            case FLIGHT_MODE_ARMED: modeStr = "ARMED"; break;
            case FLIGHT_MODE_PERCH: modeStr = "PERCH"; break;
        }
        json += ",\"mode\":\"" + modeStr + "\"}";

        server.send(200, "application/json", json);
    });

    server.on("/stop", []()
    {
        uint32_t seq = server.hasArg("seq")
            ? (uint32_t)strtoul(server.arg("seq").c_str(), nullptr, 10)
            : 0;
        noteReleaseSeq(seq);
        // Client requested an explicit immediate stop (pointerup).
        // If we're in Perch mode, ignore stop requests to avoid cancelling decorative motion.
        if (getFlightMode() == FLIGHT_MODE_PERCH)
        {
            return;
        }
        lockControlsMs(RELEASE_LOCK_MS);
        immediateStopAndCenter();
        lastThrottle = 0.0f;
        lastSteer = 0.0f;
        updateFlightTimer(0.0f);
        server.send(200, "text/plain", "OK");
    });

    // Trim adjustment endpoint (fallback for HTTP clients)
    // Usage: /trim?side=L&d=1  (side=L or R, d is delta degrees)
    server.on("/trim", []()
    {
        if (!server.hasArg("side") || !server.hasArg("d")) { server.send(400, "text/plain", "missing args"); return; }
        String s = server.arg("side"); int d = server.arg("d").toInt();
        if (s == "L" || s == "l") adjustLeftTrim(d);
        else adjustRightTrim(d);
        server.send(200, "text/plain", "OK");
    });

    // Set amplitude scale (percent or fraction): expects 's' parameter.
    // Accepts either 60..100 (percent) or 0.6..1.0 (fraction). Clamped to [0.6,1.0].
    server.on("/setAmpScale", []()
    {
        if (!server.hasArg("s"))
        {
            server.send(400, "text/plain", "missing scale");
            return;
        }
        float v = server.arg("s").toFloat();
        // If passed as percent (>= 1.0), convert to fraction
        if (v > 1.0f) v = v / 100.0f;
        if (v < 0.6f) v = 0.6f;
        if (v > 1.0f) v = 1.0f;
        amplitudeScale = v;
        lowerAmplitudeScale = lowerAmplitudeScaleFromDegrees(v * MAX_DOWNSTROKE_BELOW_NEUTRAL);
        upperAmplitudeScale = upperAmplitudeScaleFromDegrees(v * MAX_UPSTROKE_ABOVE_NEUTRAL);
        setFlapAmplitudeScales(lowerAmplitudeScale, upperAmplitudeScale);
        setCurrentProfileLowerAmplitudeScale(lowerAmplitudeScale);
        setCurrentProfileUpperAmplitudeScale(upperAmplitudeScale);
        saveAmplitudeScale();
        server.send(200, "text/plain", "OK");
    });

    server.on("/setLowerAmpScale", []()
    {
        if (!server.hasArg("s") && !server.hasArg("deg"))
        {
            server.send(400, "text/plain", "missing scale");
            return;
        }
        float v;
        if (server.hasArg("deg"))
            v = lowerAmplitudeScaleFromDegrees(server.arg("deg").toFloat());
        else
        {
            v = server.arg("s").toFloat();
            if (v > 1.0f) v = v / 100.0f;
            v = lowerAmplitudeScaleFromDegrees(v * MAX_DOWNSTROKE_BELOW_NEUTRAL);
        }
        lowerAmplitudeScale = v;
        amplitudeScale = fmaxf(lowerAmplitudeScale, upperAmplitudeScale);
        setFlapAmplitudeScales(lowerAmplitudeScale, upperAmplitudeScale);
        setCurrentProfileLowerAmplitudeScale(v);
        saveAmplitudeScale();
        server.send(200, "text/plain", "OK");
    });

    server.on("/setUpperAmpScale", []()
    {
        if (!server.hasArg("s") && !server.hasArg("deg"))
        {
            server.send(400, "text/plain", "missing scale");
            return;
        }
        float v;
        if (server.hasArg("deg"))
            v = upperAmplitudeScaleFromDegrees(server.arg("deg").toFloat());
        else
        {
            v = server.arg("s").toFloat();
            if (v > 1.0f) v = v / 100.0f;
            v = upperAmplitudeScaleFromDegrees(v * MAX_UPSTROKE_ABOVE_NEUTRAL);
        }
        upperAmplitudeScale = v;
        amplitudeScale = fmaxf(lowerAmplitudeScale, upperAmplitudeScale);
        setFlapAmplitudeScales(lowerAmplitudeScale, upperAmplitudeScale);
        setCurrentProfileUpperAmplitudeScale(v);
        saveAmplitudeScale();
        server.send(200, "text/plain", "OK");
    });

    // Live amplitude setter used by UI slider (percent in 0..1)
    server.on("/setAmplitude", []()
    {
        if (!server.hasArg("a"))
        {
            server.send(400, "text/plain", "missing amplitude");
            return;
        }
        float p = server.arg("a").toFloat();
        if (p < 0.0f) p = 0.0f;
        if (p > 1.0f) p = 1.0f;
        float amp = p * MAX_FLAP_AMPLITUDE;
#if FIXED_AMPLITUDE_TEST_MODE
        (void)amp;
#else
        // Directly set the internal amplitude to the requested degrees (bypass smoothing)
        // so user adjustments are immediately effective.
        setAmplitude(amp);
#endif
        server.send(200, "text/plain", "OK");
    });

    // Set steering trim (-5..5) via HTTP
    server.on("/setSteerTrim", []()
    {
        if (!server.hasArg("s")) { server.send(400, "text/plain", "missing steer trim"); return; }
        float v = server.arg("s").toFloat();
        if (v < -5.0f) v = -5.0f;
        if (v > 5.0f) v = 5.0f;
        setSteeringTrim(v);
        server.send(200, "text/plain", "OK");
    });

    server.on("/setSteerGain", []()
    {
        if (!server.hasArg("g")) { server.send(400, "text/plain", "missing steer gain"); return; }
        float v = server.arg("g").toFloat();
        if (v > 2.0f) v = v / 100.0f;
        setSteeringDifferentialGain(v);
        setCurrentProfileSteeringGain(getSteeringDifferentialGain());
        server.send(200, "text/plain", "OK");
    });

    server.on("/setTurnComp", []()
    {
        if (!server.hasArg("c")) { server.send(400, "text/plain", "missing turn compensation"); return; }
        float v = server.arg("c").toFloat();
        if (v > 1.0f) v = v / 100.0f;
        setTurnCompensation(v);
        setCurrentProfileTurnCompensation(getTurnCompensation());
        server.send(200, "text/plain", "OK");
    });

    // Set servo pitch center (degrees) via HTTP
    server.on("/setPitch", []()
    {
        if (!server.hasArg("p")) { server.send(400, "text/plain", "missing pitch"); return; }
        float v = server.arg("p").toFloat();
        if (v < 80.0f) v = 80.0f;
        if (v > 100.0f) v = 100.0f;
        setServoPitchAngle(v);
        setCurrentProfileNeutralPitch(getServoPitchAngle());
        server.send(200, "text/plain", "OK");
    });

    server.on("/setWaveform", []()
    {
        if (!server.hasArg("w")) { server.send(400, "text/plain", "missing waveform"); return; }
        int waveform = server.arg("w").toInt() == 1 ? 1 : 0;
        setFlapWaveform(waveform);
        setCurrentProfileWaveform(waveform);
        server.send(200, "text/plain", "OK");
    });

    server.on("/setServoOutput", []()
    {
        if (server.hasArg("hz"))
            setServoRefreshRateHz(server.arg("hz").toInt());
        if (server.hasArg("min") || server.hasArg("max"))
        {
            int minUs = server.hasArg("min") ? server.arg("min").toInt() : getServoMinPulseUs();
            int maxUs = server.hasArg("max") ? server.arg("max").toInt() : getServoMaxPulseUs();
            setServoPulseRangeUs(minUs, maxUs);
        }
        server.send(200, "text/plain", "OK");
    });

    server.on("/setFrequencyCap", []()
    {
        if (!server.hasArg("hz")) { server.send(400, "text/plain", "missing frequency"); return; }
        float cap = server.arg("hz").toFloat();
        setCurrentProfileFrequencyCap(cap);
        setFrequencyLimit(getProfileFrequencyCap());
        server.send(200, "text/plain", "OK");
    });

    server.on("/setHardwarePins", []()
    {
        if (!server.hasArg("ls") || !server.hasArg("rs") || !server.hasArg("led") ||
            !server.hasArg("bat") || !server.hasArg("bwarn") || !server.hasArg("stat") ||
            !server.hasArg("erx") || !server.hasArg("etx"))
        {
            server.send(400, "text/plain", "missing pins");
            return;
        }

        setHardwarePins(
            server.arg("ls").toInt(),
            server.arg("rs").toInt(),
            server.arg("led").toInt(),
            server.arg("bat").toInt(),
            server.arg("bwarn").toInt(),
            server.arg("stat").toInt(),
            server.arg("erx").toInt(),
            server.arg("etx").toInt());
        applyRuntimeHardwarePins();
        server.send(200, "text/plain", "OK");
    });

    server.on("/setChannelMapping", []()
    {
        for (uint8_t i = 0; i < 8; ++i)
        {
            String argName = "ch" + String(i + 1);
            if (server.hasArg(argName))
                channelMap[i] = clampChannelFunction(server.arg(argName).toInt());
        }
        saveChannelMapping();
        server.send(200, "text/plain", "OK");
    });

    server.on("/cycleWaveform", []()
    {
        int waveform = getFlapWaveform() == 1 ? 0 : 1;
        setFlapWaveform(waveform);
        setCurrentProfileWaveform(waveform);
        server.send(200, "text/plain", "OK");
    });

    server.on("/setFlightProfile", []()
    {
        if (!server.hasArg("p")) { server.send(400, "text/plain", "missing profile"); return; }
        setFlightProfileModeFromInt(server.arg("p").toInt());
        applyCurrentFlightProfile();
        server.send(200, "text/plain", "OK");
    });

    server.on("/cycleFlightProfile", []()
    {
        int next = ((int)getFlightProfileMode() + 1) % 3;
        setFlightProfileModeFromInt(next);
        applyCurrentFlightProfile();
        server.send(200, "text/plain", "OK");
    });

    server.on("/cycleLed", []()
    {
        cycleLedMode();
        server.send(200, "text/plain", "OK");
    });

    server.on("/setLedMode", []()
    {
        if (!server.hasArg("m")) { server.send(400, "text/plain", "missing LED mode"); return; }
        setLedMode(server.arg("m").toInt());
        server.send(200, "text/plain", "OK");
    });

    server.on("/setLedConfig", []()
    {
        if (server.hasArg("brightness")) setLedBrightnessLevel(server.arg("brightness").toInt());
        if (server.hasArg("speed")) setLedEffectSpeedLevel(server.arg("speed").toInt());
        server.send(200, "text/plain", "OK");
    });

    server.on("/resetFlightRecord", []()
    {
        resetFlightRecord();
        server.send(200, "text/plain", "OK");
    });

    server.on("/setHoverBiasConfig", []()
    {
        if (server.hasArg("enabled"))
        {
            setHoverBiasEnabled(server.arg("enabled").toInt() != 0);
        }

        float forwardNeutral = getHoverBiasForwardNeutralAngle();
        float hoverNeutral = getHoverBiasHoverNeutralAngle();
        if (server.hasArg("forward")) forwardNeutral = server.arg("forward").toFloat();
        if (server.hasArg("hover")) hoverNeutral = server.arg("hover").toFloat();
        setHoverBiasConfig(forwardNeutral, hoverNeutral);
        server.send(200, "text/plain", "OK");
    });

    server.on("/hoverBiasTilt", []()
    {
        if (!server.hasArg("pitch")) { server.send(400, "text/plain", "missing pitch"); return; }
        updateHoverBiasDevicePitch(server.arg("pitch").toFloat());
        server.send(200, "text/plain", "OK");
    });

    server.on("/mode", []()
    {
        if (!server.hasArg("m"))
        {
            server.send(400, "text/plain", "missing mode");
            return;
        }
        int m = server.arg("m").toInt();
        if (isBatteryServoFailsafeActive() && m != FLIGHT_MODE_DISARMED)
        {
            enforceBatteryServoFailsafe();
            server.send(200, "text/plain", "BATTERY_CUTOFF");
            return;
        }
        setFlightModeFromInt(m);
        if (m == FLIGHT_MODE_PERCH)
        {
            startPerch();
        }
        else
        {
            stopPerch();
            if (m == FLIGHT_MODE_DISARMED)
            {
                immediateStopAndCenter();
                lastThrottle = 0.0f;
                lastSteer = 0.0f;
                updateFlightTimer(0.0f);
            }
        }
        server.send(200, "text/plain", "OK");
    });

    // Start WebSocket server for low-latency control and telemetry
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);

    server.begin();
}

void updateWiFiControl()
{
    updateElrsReceiver();

    if (elrsWifiShutdownPending && millis() >= elrsWifiShutdownAtMs)
    {
        elrsWifiShutdownPending = false;
        disableWifiForElrsIsolation();
    }

    if (wifiRadioEnabled)
    {
        server.handleClient();
        webSocket.loop();
    }

    if (controlProtocol == CONTROL_PROTOCOL_ELRS)
        updateElrsReceiver();

    if (wifiRadioEnabled && controlProtocol != CONTROL_PROTOCOL_ELRS)
        updateEspNowReceiver();

    updateIndicatorLeds();
    updateCachedBatteryTelemetry(false);
    enforceBatteryServoFailsafe();

    if (controlProtocol == CONTROL_PROTOCOL_ELRS && millis() >= crsfNextTelemetryMs)
    {
        updateCachedBatteryTelemetry(false);
        if (batteryMonitorEnabled && cachedBatteryVoltage > 0.1f)
            sendCrsfBatteryTelemetry(cachedBatteryVoltage, cachedBatteryPercent);
        crsfNextTelemetryMs = millis() + ELRS_BATTERY_TELEMETRY_INTERVAL_MS;
    }

    // ===== CONTROL TIMEOUT FAILSAFE =====
    if (millis() - lastControlPacket > 500)
    {
        // If in Perch mode, do not zero frequency/amplitude — decorative motion should continue
        if (getFlightMode() != FLIGHT_MODE_PERCH && getFlightMode() != FLIGHT_MODE_DISARMED)
        {
            if (!isForceCenterActive())
            {
                Serial.println("FAILSAFE ACTIVE");

                // Mark lastThrottle zero so UI reflects stop
                lastThrottle = 0.0f;
                lastSteer = 0.0f;
                updateFlightTimer(0.0f);

                lockControlsMs(RELEASE_LOCK_MS);
                immediateStopAndCenter();
            }
        }
    }

    // Telemetry broadcast via WebSocket (if any clients connected).
    const unsigned long telemetryInterval = 50UL;
    if (webSocket.connectedClients() > 0 && millis() - lastTelemetryMillis > telemetryInterval)
    {
        lastTelemetryMillis = millis();
        // Build telemetry JSON (reuse same as /telemetry)
        float telemetryThrottle = lastThrottle;
        float freq = getFlapFrequency();
        updateCachedBatteryTelemetry(false);
        float battV = cachedBatteryVoltage;
        float battPct = cachedBatteryPercent;
        bool battLow = cachedBatteryLow;
        bool battCritical = cachedBatteryCritical;
        String modeStr = "UNKNOWN";
        switch (getFlightMode()) { case FLIGHT_MODE_DISARMED: modeStr = "DISARMED"; break; case FLIGHT_MODE_ARMED: modeStr = "ARMED"; break; case FLIGHT_MODE_PERCH: modeStr = "PERCH"; break; }
        char json[900];
        snprintf(json, sizeof(json),
            "{\"throttle\":%.3f,\"frequency\":%.3f,\"amplitude\":%.3f,"
            "\"lowerAmplitudeScale\":%.3f,\"upperAmplitudeScale\":%.3f,"
            "\"lowerAmplitudeDegrees\":%.1f,\"upperAmplitudeDegrees\":%.1f,"
            "\"pilot\":%.3f,\"flightProfile\":%d,\"flightProfileName\":\"%s\","
            "\"frequencyCap\":%.3f,\"waveform\":%d,\"controlProtocol\":%d,"
            "\"steeringLock\":%d,\"currentFlightMs\":%lu,\"flightMs\":%lu,"
            "\"recordMs\":%lu,\"uptimeMs\":%lu,\"servoPitch\":%.3f,"
            "\"currentNeutralAngle\":%.3f,\"hoverBiasEnabled\":%d,"
            "\"hoverBias\":%.3f,\"hoverDevicePitch\":%.3f,\"ledMode\":%d,"
            "\"ledBrightness\":%d,\"ledSpeed\":%d,\"battery\":%.3f,"
            "\"batteryPct\":%.3f,\"batteryLow\":%d,\"batteryCritical\":%d,"
            "\"batteryEnabled\":%d,\"batteryFailsafeEnabled\":%d,"
            "\"batteryFailsafeCutoff\":%.2f,\"batteryServoFailsafe\":%d,"
            "\"leftTrim\":%d,\"rightTrim\":%d,\"steeringTrim\":%.3f,"
            "\"mode\":\"%s\"}",
            telemetryThrottle, freq, getFlapAmplitude(),
            lowerAmplitudeScale, upperAmplitudeScale,
            lowerAmplitudeScale * MAX_DOWNSTROKE_BELOW_NEUTRAL,
            upperAmplitudeScale * MAX_UPSTROKE_ABOVE_NEUTRAL,
            getPilotSteeringCurrent(), (int)getFlightProfileMode(), getFlightProfileName(),
            getProfileFrequencyCap(), getFlapWaveform(), (int)controlProtocol,
            steeringLockEnabled ? 1 : 0,
            getCurrentFlightSessionMs(), getFlightTimerMs(), getFlightRecordMs(), millis(),
            getServoPitchAngle(), getCurrentNeutralAngle(),
            isHoverBiasEnabled() ? 1 : 0, getHoverBias(), getHoverBiasDevicePitch(),
            getLedMode(), getLedBrightnessLevel(), getLedEffectSpeedLevel(),
            battV, battPct, battLow ? 1 : 0, battCritical ? 1 : 0,
            batteryMonitorEnabled ? 1 : 0, batteryFailsafeEnabled ? 1 : 0,
            batteryFailsafeCutoffVoltage, batteryServoFailsafeActive ? 1 : 0,
            getLeftTrim(), getRightTrim(), getSteeringTrim(), modeStr.c_str());

        webSocket.broadcastTXT(json);
    }
}

// WebSocket event handler
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length)
{
    if (type == WStype_TEXT)
    {
        // Ensure null-terminated
        char msg[256];
        size_t l = length < 255 ? length : 255;
        memcpy(msg, payload, l);
        msg[l] = '\0';

        // Parse simple space-separated commands
        if (strncmp(msg, "ctl ", 4) == 0)
        {
            if (controlProtocol != CONTROL_PROTOCOL_WIFI) return;
            float x = 0.0f, y = 0.0f;
            uint32_t seq = 0;
            int parsed = sscanf(msg+4, "%lu %f %f", &seq, &x, &y);
            if (parsed != 3)
            {
                seq = 0;
                parsed = sscanf(msg+4, "%f %f", &x, &y);
            }
            if (parsed >= 2)
            {
                if (isStaleControlSeq(seq)) return;
                noteControlSeq(seq);

                x = constrain(x, -1.0f, 1.0f);
                y = constrain(y, 0.0f, 1.0f);
                if (steeringLockEnabled)
                    x = 0.0f;
                const bool throttleIdle = y <= CONTROL_THROTTLE_DEADBAND;
                const bool idleNeutral = fabsf(x) < 0.05f && throttleIdle;

                if (isBatteryServoFailsafeActive())
                {
                    enforceBatteryServoFailsafe();
                    if (throttleIdle)
                    {
                        if (isForceCenterActive())
                            clearForceCenter();
                        setIdleLeanSteering(-x * IDLE_LEAN_INPUT_AUTHORITY);
                        lastThrottle = 0.0f;
                        lastSteer = x;
                        updateFlightTimer(0.0f);
                    }
                    return;
                }

                if (getFlightMode() == FLIGHT_MODE_DISARMED)
                {
                    immediateStopAndCenter();
                    lastThrottle = 0.0f;
                    lastSteer = 0.0f;
                    updateFlightTimer(0.0f);
                    return;
                }

                // If in Perch mode, ignore controls
                if (getFlightMode() == FLIGHT_MODE_PERCH) return;

                if (!throttleIdle && isControlReleaseLocked()) return;

                // Debounce forced center
                if (isForceCenterActive())
                {
                    if (false) return;
                    else clearForceCenter();
                }

                lastControlPacket = millis();
                lastThrottle = throttleIdle ? 0.0f : y;
                lastSteer = x;
                updateFlightTimer(lastThrottle);

                if (throttleIdle)
                {
                    setIdleLeanSteering(-x * IDLE_LEAN_INPUT_AUTHORITY);
                    return;
                }

                float t = y;
                float scale = powf(t, STEERING_CURVE_EXPONENT);
                float steeringAuthority = STEERING_BASE_AUTHORITY + scale * (STEERING_MAX_AUTHORITY - STEERING_BASE_AUTHORITY);
                steeringAuthority *= getProfileSteeringMultiplier();
                steeringAuthority = fminf(steeringAuthority, MAX_FINAL_STEERING);
                setPilotSteering(-x * steeringAuthority);

                float freq = throttleToFrequency(y);
                setFrequency(freq);

                float amplitude;
#if FIXED_AMPLITUDE_TEST_MODE
                amplitude = TEST_FLAP_AMPLITUDE_DEGREES;
#else
                amplitude = MIN_ACTIVE_FLAP_AMPLITUDE +
                    y * (MAX_FLAP_AMPLITUDE - MIN_ACTIVE_FLAP_AMPLITUDE);
#endif
                if (amplitude < MIN_FLAP_AMPLITUDE) amplitude = MIN_FLAP_AMPLITUDE;
                if (amplitude > MAX_FLAP_AMPLITUDE) amplitude = MAX_FLAP_AMPLITUDE;
                float finalAmp = amplitude;
                if (finalAmp < MIN_FLAP_AMPLITUDE) finalAmp = MIN_FLAP_AMPLITUDE;
                if (finalAmp > MAX_FLAP_AMPLITUDE) finalAmp = MAX_FLAP_AMPLITUDE;
                setAmplitude(finalAmp);

            }
        }
        else if (strncmp(msg, "stop", 4) == 0)
        {
            if (controlProtocol != CONTROL_PROTOCOL_WIFI) return;
            uint32_t seq = 0;
            sscanf(msg+4, "%lu", &seq);
            noteReleaseSeq(seq);
            if (getFlightMode() == FLIGHT_MODE_PERCH) return;
            lockControlsMs(RELEASE_LOCK_MS);
            immediateStopAndCenter();
            lastThrottle = 0.0f; lastSteer = 0.0f;
            updateFlightTimer(0.0f);
        }
        else if (strncmp(msg, "scale ", 6) == 0)
        {
            float v = atof(msg+6);
            if (v > 1.0f) v = v / 100.0f;
            if (v < 0.6f) v = 0.6f;
            if (v > 1.0f) v = 1.0f;
            amplitudeScale = v;
            lowerAmplitudeScale = lowerAmplitudeScaleFromDegrees(v * MAX_DOWNSTROKE_BELOW_NEUTRAL);
            upperAmplitudeScale = upperAmplitudeScaleFromDegrees(v * MAX_UPSTROKE_ABOVE_NEUTRAL);
            setFlapAmplitudeScales(lowerAmplitudeScale, upperAmplitudeScale);
            setCurrentProfileLowerAmplitudeScale(lowerAmplitudeScale);
            setCurrentProfileUpperAmplitudeScale(upperAmplitudeScale);
            saveAmplitudeScale();
        }
        else if (strncmp(msg, "steerGain ", 10) == 0)
        {
            float v = atof(msg+10);
            if (v > 2.0f) v = v / 100.0f;
            setSteeringDifferentialGain(v);
        }
        else if (strncmp(msg, "trim ", 5) == 0)
        {
            // format: trim L <delta>  or trim R <delta>
            char side = 'L'; int delta = 0;
            if (sscanf(msg+5, " %c %d", &side, &delta) == 2)
            {
                if (side == 'L' || side == 'l') adjustLeftTrim(delta);
                else adjustRightTrim(delta);
            }
        }
        else if (strncmp(msg, "mode ", 5) == 0)
        {
            int m = atoi(msg+5);
            if (isBatteryServoFailsafeActive() && m != FLIGHT_MODE_DISARMED)
            {
                enforceBatteryServoFailsafe();
                return;
            }
            setFlightModeFromInt(m);
            if (m == FLIGHT_MODE_PERCH)
            {
                startPerch();
            }
            else
            {
                stopPerch();
                if (m == FLIGHT_MODE_DISARMED)
                {
                    immediateStopAndCenter();
                    lastThrottle = 0.0f;
                    lastSteer = 0.0f;
                    updateFlightTimer(0.0f);
                }
            }
        }
    }
}

// Small helper to broadcast trim updates to any connected WS clients
void broadcastTrimUpdate(int left, int right, float steeringTrim)
{
    String json = "{";
    json += "\"leftTrim\":" + String(left) + ",";
    json += "\"rightTrim\":" + String(right) + ",";
    json += "\"steeringTrim\":" + String(steeringTrim, 3);
    json += "}";
    webSocket.broadcastTXT(json);
}

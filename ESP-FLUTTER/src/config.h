#pragma once

#define ESP_FLUTTER_BOARD_SEEED_XIAO_ESP32C3 1
#define ESP_FLUTTER_BOARD_SEEED_XIAO_ESP32S3 2
#define ESP_FLUTTER_BOARD_ESP32_DEVKIT_V1 3
#define ESP_FLUTTER_BOARD_USE_PLATFORMIO 0

// ===== BOARD / PIN SELECTION =====
// Choose exactly one board before flashing:
//   1. Leave one ESP_FLUTTER_SELECTED_BOARD line below uncommented.
//   2. In platformio.ini, set default_envs to the matching environment.
//   3. Wire the hardware to the pins listed under that board's profile below.
//
// Current selected board: Seeed Studio XIAO ESP32S3.
// To switch boards, comment this active line and uncomment the board you want.
#define ESP_FLUTTER_SELECTED_BOARD ESP_FLUTTER_BOARD_SEEED_XIAO_ESP32C3
// #define ESP_FLUTTER_SELECTED_BOARD ESP_FLUTTER_BOARD_SEEED_XIAO_ESP32S3
// #define ESP_FLUTTER_SELECTED_BOARD ESP_FLUTTER_BOARD_ESP32_DEVKIT_V1
// #define ESP_FLUTTER_SELECTED_BOARD ESP_FLUTTER_BOARD_USE_PLATFORMIO
//
// IMPORTANT: Only the Seeed Studio XIAO ESP32C3 wiring has been flight-tested.
// The XIAO ESP32S3 and ESP32 DevKit V1 mappings are experimental starting
// points. Use them at your discretion and verify every GPIO against your exact
// board revision before connecting servos, LEDs, ELRS/CRSF UART, or the
// battery voltage divider.
#ifndef ESP_FLUTTER_SELECTED_BOARD
#define ESP_FLUTTER_SELECTED_BOARD ESP_FLUTTER_BOARD_USE_PLATFORMIO
#endif

#ifndef ESP_FLUTTER_BOARD_PROFILE
#if ESP_FLUTTER_SELECTED_BOARD != ESP_FLUTTER_BOARD_USE_PLATFORMIO
#define ESP_FLUTTER_BOARD_PROFILE ESP_FLUTTER_SELECTED_BOARD
#elif defined(ESP_FLUTTER_PLATFORMIO_BOARD_PROFILE)
#define ESP_FLUTTER_BOARD_PROFILE ESP_FLUTTER_PLATFORMIO_BOARD_PROFILE
#else
#define ESP_FLUTTER_BOARD_PROFILE ESP_FLUTTER_BOARD_SEEED_XIAO_ESP32C3
#endif
#endif

#if ESP_FLUTTER_BOARD_PROFILE == ESP_FLUTTER_BOARD_SEEED_XIAO_ESP32C3
// Seeed Studio XIAO ESP32C3, tested/default wiring:
// Left servo: D0, Right servo: D1, Battery divider ADC: D2,
// LED filament PWM: D3, Status LED: D6, Low-battery LED: D8,
// ELRS/CRSF receiver TX -> D9, receiver RX -> D10.
#define LEFT_SERVO_PIN D0
#define RIGHT_SERVO_PIN D1
#define BATTERY_PIN D2
#define LED_PIN D3
#define BATTERY_WARNING_LED_PIN D8
#define STATUS_LED_PIN D6
#define ELRS_RX_PIN D9
#define ELRS_TX_PIN D10

#elif ESP_FLUTTER_BOARD_PROFILE == ESP_FLUTTER_BOARD_SEEED_XIAO_ESP32S3
// Seeed Studio XIAO ESP32S3, experimental and not flight-tested:
// Left servo: D0, Right servo: D1, Battery divider ADC: D2,
// LED filament PWM: D3, Status LED: D6, Low-battery LED: D8,
// ELRS/CRSF receiver TX -> D9, receiver RX -> D10.
#define LEFT_SERVO_PIN D0
#define RIGHT_SERVO_PIN D1
#define BATTERY_PIN D2
#define LED_PIN D3
#define BATTERY_WARNING_LED_PIN D8
#define STATUS_LED_PIN D6
#define ELRS_RX_PIN D9
#define ELRS_TX_PIN D10

#elif ESP_FLUTTER_BOARD_PROFILE == ESP_FLUTTER_BOARD_ESP32_DEVKIT_V1
// ESP32 DevKit V1 / esp32dev, experimental and not flight-tested:
// Left servo: GPIO25, Right servo: GPIO26, Battery divider ADC: GPIO34,
// LED filament PWM: GPIO27, Status LED: GPIO2, Low-battery LED: GPIO33,
// ELRS/CRSF receiver TX -> GPIO16, receiver RX -> GPIO17.
#define LEFT_SERVO_PIN 25
#define RIGHT_SERVO_PIN 26
#define BATTERY_PIN 34
#define LED_PIN 27
#define BATTERY_WARNING_LED_PIN 33
#define STATUS_LED_PIN 2
#define ELRS_RX_PIN 16
#define ELRS_TX_PIN 17

#else
#error "Unknown ESP_FLUTTER_BOARD_PROFILE selected in config.h"
#endif

// ===== SERVO SETTINGS =====
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define SERVO_ABSOLUTE_MIN_US 500
#define SERVO_ABSOLUTE_MAX_US 2500
#define SERVO_MIN_US_SLIDER_MIN 500
#define SERVO_MIN_US_SLIDER_MAX 1200
#define SERVO_MAX_US_SLIDER_MIN 1800
#define SERVO_MAX_US_SLIDER_MAX 2500
// Servo PWM frame/refresh rate in Hz.
// Allowed values: 50, 100, 200, 250, 300, 333.
// SDS1601 digital servos support 333 Hz with 1520 us neutral and the
// existing 800-2200 us pulse width range. Use 50 Hz for analog/unknown servos.
#ifndef SERVO_REFRESH_HZ
#define SERVO_REFRESH_HZ 50
#endif

#if SERVO_REFRESH_HZ != 50 && SERVO_REFRESH_HZ != 100 && SERVO_REFRESH_HZ != 200 && SERVO_REFRESH_HZ != 250 && SERVO_REFRESH_HZ != 300 && SERVO_REFRESH_HZ != 333
#error "SERVO_REFRESH_HZ must be one of: 50, 100, 200, 250, 300, 333"
#endif

// ===== INITIAL MOTION SETTINGS =====
#define DEFAULT_FREQUENCY 1.5f
#define DEFAULT_AMPLITUDE 90.0f

// ===== UPDATE RATE =====
#define LOOP_DELAY_MS 1

// ===== STEERING AUTHORITY TUNING =====
// Minimum steering authority at zero throttle
#define STEERING_BASE_AUTHORITY 10.0f
// Idle lean input should reach full 15-degree lean only at full stick travel.
// setIdleLeanSteering() applies its own gain internally, so this is lower than
// normal in-flight steering authority.
#define IDLE_LEAN_INPUT_AUTHORITY 5.0f
// Maximum steering authority at full throttle
#define STEERING_MAX_AUTHORITY 60.0f
// Exponent controlling curve shape: >1 compresses low throttle (gentler), <1 makes it more aggressive early
#define STEERING_CURVE_EXPONENT 2.0f

// Maximum pilot steering contribution (degrees or flap units)
#define MAX_PILOT_STEERING 60.0f

// Steering smoothing: maximum change (degrees) per second when responding to pilot inputs
#define STEERING_RAMP_RATE 180.0f

// Small steering values below this threshold are treated as zero to avoid jitter
#define STEERING_DEADBAND 0.05f

// Enable steering debug prints (set to 0 to disable)
#define ENABLE_STEERING_DEBUG 1

// Direct S-DIY-style flap mixer for comparison.
// 1 = direct triangle-wave output with steering added after flap generation.
// 0 = existing ESP-FLUTTER protected/headroom-aware mixer.
#define DIRECT_FLAP_MODE 1

// Direct Flap Mode steering mixer. The hard cap limits the difference between
// left and right flap amplitudes to this fraction of the current flap amplitude.
#define STEERING_DIFF_MAX_FRACTION 0.50f
#define STEERING_DIFF_GAIN_DEFAULT 1.0f
#define STEERING_DIFF_GAIN_MAX 2.5f
// Extra average flap amplitude applied at full steering to compensate for
// lift redirected sideways during coordinated turns.
#define TURN_LIFT_BOOST_MAX_FRACTION 0.15f
#define TURN_LIFT_BOOST_LIMIT_FRACTION 0.30f

// Treat amplitude values below this as idle when wings are effectively stopped
#define AMPLITUDE_IDLE_THRESHOLD 2.0f

// Amplitude resume hysteresis: require amplitude > this threshold for AMPLITUDE_RESUME_MS
#define AMPLITUDE_RESUME_THRESHOLD 5.0f
#define AMPLITUDE_RESUME_MS 300

// When controls resume after idle, limit initial steering slew to STARTUP_RAMP_SCALE
#define STARTUP_RAMP_MS 300
// Fraction of normal ramp rate applied during startup ramp (0.0 - 1.0)
#define STARTUP_RAMP_SCALE 0.25f

// Maximum age (ms) of a control packet before it's considered stale and ignored when resuming
#define CONTROL_STALE_MS 100

// When the pilot explicitly releases the stick (x≈0,y≈0), lock controls for this many ms
#define RELEASE_LOCK_MS 250

// Below this throttle, steering only poses the wings for idle lean and must
// not start the flap waveform.
#define CONTROL_THROTTLE_DEADBAND 0.05f

// ===== SAFETY LIMITS (single documented location) =====
// Maximum allowed magnitude for the final mixed steering applied to flaps
#define MAX_FINAL_STEERING 70.0f
// Maximum RC roll bias. This shifts both servo centers together to lean the
// craft sideways without using the yaw/steering differential.
#define MAX_ROLL_BIAS_DEGREES 12.0f

// Flap amplitude limits in servo degrees from the 90-degree neutral point.
// Keep MIN_FLAP_AMPLITUDE at 0 so failsafe/release can stop and center wings.
#define MIN_ACTIVE_FLAP_AMPLITUDE 40.0f
#define MAX_FLAP_AMPLITUDE 90.0f
#define MIN_FLAP_AMPLITUDE 0.0f

// Test mode: throttle controls frequency only; amplitude stays fixed here.
#define FIXED_AMPLITUDE_TEST_MODE 1
#define TEST_FLAP_AMPLITUDE_DEGREES 90.0f

// Physical peak travel from the 90-degree neutral position.
#define MAX_UPSTROKE_ABOVE_NEUTRAL 90.0f
#define MAX_DOWNSTROKE_BELOW_NEUTRAL 90.0f
#define LOWER_FLAP_AMPLITUDE_MIN_DEGREES 50.0f
#define LOWER_FLAP_AMPLITUDE_MAX_DEGREES 90.0f
#define LOWER_FLAP_AMPLITUDE_DEFAULT_DEGREES 75.0f
#define UPPER_FLAP_AMPLITUDE_MIN_DEGREES 50.0f
#define UPPER_FLAP_AMPLITUDE_MAX_DEGREES 90.0f
#define UPPER_FLAP_AMPLITUDE_DEFAULT_DEGREES 75.0f

// Flap frequency limits (Hz)
#define DEFAULT_MAX_FLAP_FREQUENCY 4.0f
#define ABSOLUTE_MAX_FLAP_FREQUENCY 7.0f
#define MIN_FLAP_FREQUENCY 0.0f
// Once throttle is above CONTROL_THROTTLE_DEADBAND, start flapping here
// instead of crawling up from 0 Hz.
#define MIN_ACTIVE_FLAP_FREQUENCY 1.0f

// Perch (decorative) mode defaults
#define PERCH_FREQUENCY 0.6f
#define PERCH_AMPLITUDE 60.0f

// Servo safe angle margins to avoid mechanical end-stop saturation (degrees)
#define SERVO_SAFE_MIN_ANGLE 0
#define SERVO_SAFE_MAX_ANGLE 180

// WiFi AP / ESP-NOW channel. The transmitter must use this same channel.
#define WIFI_AP_CHANNEL 1

// ELRS / CRSF receiver UART. Wire receiver TX to ELRS_RX_PIN and receiver RX
// to ELRS_TX_PIN for the selected board profile above.
#define ELRS_BAUD 420000
// Battery telemetry sent from the butterfly flight controller to the ELRS
// receiver/transmitter over CRSF. Keep this modest so it never steals time from
// stick packets or servo updates.
#define ELRS_BATTERY_TELEMETRY_INTERVAL_MS 1000UL
// Diagnostic flight mode: after selecting ELRS from the app, turn off the
// ESP32 WiFi/AP radio so it cannot desense or interfere with a nearby 2.4GHz
// ELRS receiver. Reboot the ESP32 to restore the app/AP.
#define ELRS_RF_ISOLATION_WIFI_OFF 0

// Divider: battery -> 49K -> ADC -> 18K -> GND.
#define BATTERY_DIVIDER_TOP_OHMS 49000.0f
#define BATTERY_DIVIDER_BOTTOM_OHMS 18000.0f
#define BATTERY_VOLTAGE_DIVIDER \
    ((BATTERY_DIVIDER_TOP_OHMS + BATTERY_DIVIDER_BOTTOM_OHMS) / BATTERY_DIVIDER_BOTTOM_OHMS)
// With the 49K/18K divider, a fully charged 8.4V pack produces about 2.257V
// at the ADC pin, so the divider math alone should recover pack voltage.
#define BATTERY_VOLTAGE_CALIBRATION 1.0f
#define BATTERY_ADC_SAMPLE_COUNT 8
#define BATTERY_FILTER_RISE_ALPHA 0.12f
#define BATTERY_FILTER_FALL_ALPHA 0.025f
#define BATTERY_ADC_SATURATION_RAW 4075
// 2S LiPo display range. Percentage uses a nonlinear discharge curve.
#define BATTERY_VOLTAGE_MIN 6.6f
#define BATTERY_VOLTAGE_MAX 8.4f
#define BATTERY_WARNING_VOLTAGE 7.4f
#define BATTERY_WARNING_CLEAR_VOLTAGE 7.45f
#define BATTERY_CRITICAL_VOLTAGE 7.0f

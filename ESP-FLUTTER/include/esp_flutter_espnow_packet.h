#pragma once

#include <stdint.h>

#define ESP_FLUTTER_ESPNOW_MAGIC 0x544C4645UL
#define ESP_FLUTTER_ESPNOW_VERSION 1

enum EspFlutterEspNowFlags : uint32_t
{
    ESP_FLUTTER_NOW_CONTROL = 1UL << 0,
    ESP_FLUTTER_NOW_FLIGHT_MODE = 1UL << 1,
    ESP_FLUTTER_NOW_FLIGHT_PROFILE = 1UL << 2,
    ESP_FLUTTER_NOW_WAVEFORM = 1UL << 3,
    ESP_FLUTTER_NOW_AMPLITUDE = 1UL << 4,
    ESP_FLUTTER_NOW_STEERING_TRIM = 1UL << 5,
    ESP_FLUTTER_NOW_STEERING_GAIN = 1UL << 6,
    ESP_FLUTTER_NOW_TURN_COMP = 1UL << 7,
    ESP_FLUTTER_NOW_NEUTRAL_PITCH = 1UL << 8,
    ESP_FLUTTER_NOW_HOVER_BIAS = 1UL << 9,
    ESP_FLUTTER_NOW_LED = 1UL << 10,
    ESP_FLUTTER_NOW_SERVO_TRIMS = 1UL << 11,
};

struct __attribute__((packed)) EspFlutterNowPacket
{
    uint32_t magic;          // ESP_FLUTTER_ESPNOW_MAGIC
    uint8_t version;         // ESP_FLUTTER_ESPNOW_VERSION
    uint8_t packetType;      // reserved, send 0
    uint16_t size;           // sizeof(EspFlutterNowPacket)
    uint32_t seq;            // increasing sequence for control packets
    uint32_t flags;          // OR of EspFlutterEspNowFlags
    float steering;          // -1.0 left, 0.0 center, +1.0 right
    float throttle;          // 0.0 stopped, 1.0 full throttle
    int8_t flightMode;       // 0 disarmed, 1 armed, 2 perch
    int8_t flightProfile;    // 0 cruise, 1 sport, 2 hover
    int8_t waveform;         // 0 triangle, 1 sine
    int8_t ledMode;          // 0 off, 1 solid, 2 fade, 3 strobe
    int8_t ledBrightness;    // 1..10
    int8_t ledSpeed;         // 1..10
    uint8_t hoverBiasEnabled;// 0 disabled, 1 enabled
    uint8_t reserved0;       // send 0
    float amplitudeScale;    // 0.60..1.00
    float steeringTrim;      // -5.0..5.0
    float steeringGain;      // 0.0..2.5
    float turnCompensation;  // 0.0..0.30
    float neutralPitch;      // 80.0..100.0
    float hoverForwardNeutral; // 90.0..100.0
    float hoverNeutral;      // 80.0..90.0
    int16_t leftTrim;        // absolute servo trim in degrees
    int16_t rightTrim;       // absolute servo trim in degrees
};

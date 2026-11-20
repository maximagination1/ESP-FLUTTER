#pragma once

#include <stdint.h>

#define ESP_FLUTTER_UDP_CONTROL_PORT 2390
#define ESP_FLUTTER_UDP_CONTROL_MAGIC 0x4642
#define ESP_FLUTTER_UDP_CONTROL_VERSION 1

// Packed 12-byte little-endian UDP payload. Axis values are in thousandths.
struct __attribute__((packed)) EspFlutterUdpControlPacket
{
    uint16_t magic;
    uint8_t version;
    uint8_t flags;       // Must be zero.
    uint32_t sequence;   // Increment for every datagram; zero is invalid.
    int16_t steering;    // -1000..1000 maps to -1.0..1.0.
    uint16_t throttle;   // 0..1000 maps to 0.0..1.0.
};

static_assert(sizeof(EspFlutterUdpControlPacket) == 12, "UDP control packet layout changed");

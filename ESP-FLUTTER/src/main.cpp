/*
 * ESP-FLUTTER - Bionic RC Butterfly Firmware
 *
 * ESP32 firmware for a servo-driven, radio controlled, flapping-wing bionic butterfly, built
 * with Arduino and PlatformIO. The default board is the Seeed XIAO ESP32-C3.
 *
 * The firmware coordinates wing flapping and steering with adjustable flap
 * frequency, amplitude, waveforms, and servo trim. It provides a WiFi browser
 * controller, ESP-NOW and ELRS radio control options, armed/disarmed and perch
 * modes, and configurable LED effects.
 *
 * Project by Max Imagination.
 * YouTube channel: Max Imagination
 * Visit the channel for the project build and more creative electronics projects.
 */

#include <Arduino.h>

#include "wifi_control.h"
#include "wing_motion.h"
#include "led_control.h"
#include "hardware_config.h"

void setup()
{
Serial.begin(115200);

delay(3000);

Serial.println("ESP-FLUTTER STARTING");

initHardwareConfig();
initWingMotion();
initLedControl();
initWiFiControl();

Serial.println("Butterfly motion engine started.");

}
void loop()
{
updateWingMotion();

updateLedControl();

updateWiFiControl();

delay(1);

}

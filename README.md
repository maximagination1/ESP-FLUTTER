# ESP-FLUTTER Firmware 🦋

Official firmware for **ESP-FLUTTER**, an ESP32-powered bionic RC butterfly.

ESP-FLUTTER uses a XIAO ESP32-C3 and two digital servos to generate controllable flapping-wing
flight and can be controlled wirelessly using a smartphone or RC transmitter.

🎥 **Full Build Tutorial:** [YouTube link]
📁 **Project Files & BOM:** [Project link]

## Features

- **Smartphone web interface** — Control the butterfly and adjust settings through a browser with an on-screen joystick.
- **RC transmitter support** — Supports ESP-NOW transmitters and ExpressLRS receivers using CRSF.
- **Adjustable wing motion** — Configure flapping frequency, amplitude, upper and lower wing travel, and sine or triangle waveforms.
- **Differential-amplitude steering** — Turns by varying the flapping amplitude between the left and right wings, with adjustable sensitivity and turn compensation.
- **Adjustable neutral/pitch angle** — Tune the wings’ neutral position and individual servo and steering trims.
- **Configurable servo parameters** — Adjust pulse limits and refresh rate to suit the servos.
- **Flight profiles** — Customize Cruise, Sport, and Hover profiles with individual motion and steering settings.
- **Arming and perch modes** — Includes armed/disarmed states and a decorative perch mode for gentle wing movement.
- **Experimental phone-tilt control** — An experimental alternative to joystick input.
- **Battery monitoring** — Displays voltage and estimated charge, with configurable 1S/2S LiPo and LiHV settings and low-battery indicators.
- **Optional battery cutoff** — Holds the wings at neutral when voltage reaches a configured cutoff.
- **Live telemetry** — Reports operating status in the web interface and sends battery telemetry through CRSF.
- **LED effects** — Off, solid, fade, and strobe modes with adjustable brightness and effect speed.
- **Saved settings** — Retains configuration across restarts.
- **Configurable hardware pins** — Adapt pin assignments to the hardware.
- **Arduino and PlatformIO project** — Targets the Seeed Studio XIAO ESP32-C3, with experimental profiles for the XIAO ESP32-S3 and ESP32 DevKit V1.

> The current configuration enables fixed-amplitude test mode, where throttle controls flapping frequency. Hover is a tuning profile; it does not provide automatic stabilization or position holding.

## Flashing Firmware

Use the online [ESP Tool](https://espressif.github.io/esptool-js/) to flash the firmware directly from **Google Chrome** through a USB serial connection. No Python installation is required.

These instructions assume your **XIAO ESP32-C3 already has a compatible bootloader installed**.

1. Download and extract the ESP32-C3 flashing package from this repository’s [Releases](https://github.com/maximagination1/ESP-FLUTTER/releases)).
2. Connect the board to your computer using a **USB data cable**.
3. Open [ESP Tool](https://espressif.github.io/esptool-js/) in **Google Chrome**.
4. Click **Connect**, select the board’s serial/COM port, and connect.
5. Add these three files, one at a time, in the same order with their flash addresses:

   | File | Flash Address |
   | --- | --- |
   | `partitions.bin` | `0x8000` |
   | `boot_app0.bin` | `0xe000` |
   | `firmware.bin` | `0x10000` |

6. Leave **Erase All Flash** disabled to preserve the installed bootloader.
7. Click **Program** and wait for flashing to finish.
8. Disconnect from ESP Tool, then press the board’s **RESET** button.

If the board does not connect, hold **BOOT**, press and release **RESET**, then release **BOOT** and try connecting again.

### Open the Controller

1. Connect your phone or computer with WiFi connectivity to **Butterfly_RC**.
2. Enter the WiFi password: **12345678**.
3. Open **http://192.168.4.1** in your browser.
4. Check wing clearance, servo alignment, and control direction before flight.

As Max gets more time, the remaining documentation for ESP-FLUTTER Firmware will be eventually added.
For now, please refer to the video instructions and other files which already contain any info you may need to get set up.

## License & Usage

ESP-FLUTTER is free for personal, hobbyist, and educational use.

The firmware is licensed under the **PolyForm Noncommercial License 1.0.0**.

You are welcome to download, study, modify, and share the firmware for permitted noncommercial purposes.

**Commercial use requires prior written permission from Max Imagination.**

### Disclaimer

ESP-FLUTTER is an experimental, educational DIY project provided **"AS IS"**, without warranties of any kind.

Building and operating this project involves inherent risks associated with electronics, lithium-polymer batteries, moving components, and flight. Users are responsible for safely assembling, testing, and operating their own devices and for complying with applicable laws and regulations.

To the maximum extent permitted by applicable law, the creator assumes no liability for injuries, property damage, equipment failure, or other losses arising from the construction, modification, or use of this project.

Copyright © 2026 Max Imagination.

# BMA580 Motion Wake-Up Demo for ESP-SensairShuttle

Personal Arduino / PlatformIO experiment with a Bosch BMA580 accelerometer and an ESP32-C5 ESP-SensairShuttle board. Displays acceleration and temperature, detects motion interrupts, and wakes the board from light sleep when shaken.

## Personal project

This is an independent personal project, developed in a personal capacity. It is not an official project of my employer or any company, and is not sponsored, endorsed, or maintained by them. The work and opinions presented here are my own. Product and company names identify the hardware and upstream software used; they do not imply affiliation or endorsement.

See [DISCLAIMER.md](DISCLAIMER.md). Project: [mauringo/sensair-bma580-demo](https://github.com/mauringo/sensair-bma580-demo). Reusable library: [mauringo/arduino-bma580](https://github.com/mauringo/arduino-bma580).

## Current behavior

- Shows X/Y/Z acceleration in g, sensor temperature, motion events, interrupt edges, uptime, and wake cause.
- Counts down 20 seconds after the last motion event, then enters light sleep with the display powered off.
- Keeps the BMA580 running so its active-low interrupt can wake the ESP32-C5.
- Uses a 40-second recovery timer if no sensor wake occurs. The display distinguishes `SENSOR GPIO10` from `RECOVERY TIMER`.
- Recreates the LCD driver and performs panel power cycling and full initialization at startup and after wake.

Motion wake was confirmed on the connected board by shaking it. This is a hardware experiment, not a measurement of sleep current or a guarantee of compatibility with other boards.

## Hardware

Target: ESP-SensairShuttle, ESP32-C5 N16R8, BMA580 shuttle sensor, 240 × 284 ST7789 display.

| Signal | ESP32-C5 GPIO |
| --- | --- |
| Sensor SDA | 2 |
| Sensor SCL | 3 |
| BMA580 INT2 / shared CSB | 10 |
| Sensor SDO | 9 |
| LCD power control (LOW = on) | 5 |
| LCD CS / DC | 25 / 26 |
| LCD SCLK / MOSI | 24 / 23 |

The BMA580 interrupt used here is physically shared with CSB and reaches **GPIO10**. The generic shuttle interrupt label at GPIO0 does not carry this BMA580 interrupt on the tested assembly. Sensor interrupt number `2` in the library API means BMA580 INT2, not MCU GPIO2.

The LCD power switch does not cut the sensor supply on the tested mainboard. Keep the sensor powered and verify the wiring before adapting sleep behavior to another board.

## Build and upload

Install PlatformIO, connect the board, then run:

```sh
pio run -e esp32-c5-sensairshuttle
pio run -e esp32-c5-sensairshuttle -t upload
pio device monitor -b 115200
```

`platformio.ini` pins the pioarduino platform release and declares the Adafruit display dependencies. The BMA580 library is vendored in `lib/BMA580` so this project does not depend on a separately published library release. See its [README](lib/BMA580/README.md) and [upstream provenance](lib/BMA580/UPSTREAM.md).

Opening the native USB serial monitor can reset this ESP32-C5 through USB DTR/RTS. USB can also disconnect during light sleep. An upload or monitor reset starts the wake counter again; `BOOT reset:…` identifies startup rather than a sensor wake.

## Configuration and diagnostics

The timing and motion settings are constants near the top of `src/main.cpp`: `SleepAfterMs = 20000`, `RecoveryWakeUs = 40000000`, `MotionThreshold = 10`, and `MotionDuration = 1`. Threshold and duration use Bosch API units; see the upstream definitions before changing them.

The sensor runs at 100 Hz, ±8 g by default. The display temperature is the sensor's internal temperature, not calibrated ambient temperature. Interrupt handling reads and acknowledges the latched sensor status outside the ISR. Serial logs include motion configuration readback, GPIO levels and wake causes. A small persistent wake trace helps diagnose a reset during wake or LCD initialization.

## Host checks

```sh
sh test/bma580/run.sh
```

These checks compile the real Bosch SensorAPI with Arduino/Wire mocks and cover initialization timeouts, data conversion, I2C failures, interrupt mapping/acknowledgment, and countdown rollover. They do not replace hardware validation.

## Repository layout

- `src/`, `include/`: current firmware, panel initialization, countdown helper.
- `lib/BMA580/`: reusable Arduino wrapper plus unmodified Bosch SensorAPI v4.2.0.
- `test/bma580/`: host checks.
- `legacy/`: earlier BMI423 and BMA580 sleep-debug experiments; not built by the current environment.
- `third_party/espressif/`: display initialization attribution and license.

## Licensing and upstream work

The original project code is distributed under [BSD-3-Clause](LICENSE). Bosch SensorAPI files retain their Bosch copyright and [BSD-3-Clause license](lib/BMA580/LICENSE). The display initialization sequence is adapted from Espressif code under Apache-2.0; see [the notice](third_party/espressif/NOTICE.md) and its accompanying license. These projects and their owners are credited for their own work.

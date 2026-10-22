# BMI423 on ESP-SensairShuttle

Arduino/PlatformIO firmware for an ESP-SensairShuttle v1.0 mainboard with a
Bosch BMI423 Shuttle Board 3.0 (BST01953_V1.0) installed in its 7+9-pin socket.
It reports three-axis acceleration, angular velocity, and temperature. The
PlatformIO firmware uses the LCD and USB serial; the Arduino sketch is
sensor-only with USB serial output. No Wi-Fi or account credentials are needed.

## Hardware

Power off before installing or removing the BMI423 daughterboard. Align P1/P2
and pin 1 with the socket markings; do not infer orientation from the table below.
Set the mainboard VDD_SENSOR jumper to **3.3 V**. The sensor supports 1.8 V,
but this firmware uses the ESP32's 3.3 V GPIO levels without level shifting.
Both VDD and VDDIO are supplied through the socket, which also supplies ground.
The shuttle already includes the sensor's decoupling capacitors.

| BMI423 shuttle connector | Signal | SensairShuttle v1.0 connection |
| --- | --- | --- |
| P1.1 | VDD | VDD_SENSOR (3.3 V) |
| P1.2 | VDDIO | VDD_SENSOR (3.3 V) |
| P1.3 | GND | GND |
| P1.6 | INT1 | GPIO28 (unused) |
| P1.7 | INT2 | GPIO0 (unused) |
| P2.1 | CSB | GPIO10, held HIGH by pull-up for I2C |
| P2.2 | SCK/SCL | GPIO3 |
| P2.3 | SDO | GPIO9, driven HIGH to select address 0x69 |
| P2.4 | SDI/SDA | GPIO2 |

Leave the NC and PROM_RW pins unused. The BMI423 is not the BMI270 supplied
with some SensairShuttle kits: firmware expects chip ID **0x00AB**.

INT1 shares GPIO28 with the mainboard BOOT button, but this firmware disables
the sensor's interrupt outputs and does not configure GPIO28 or GPIO0.
Neither interrupt pin is required for polling.

Pin mapping source: [Espressif SensairShuttle v1.0 user guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c5/esp-sensairshuttle/user_guide_v1.0.html#shuttle-board-connector-pins).
Register settings and decoding follow the supplied Bosch BMI423 Quick Start
Guide, revision 1.0 (July 9, 2026), and BMI423 Shuttle Board 3.0 flyer.

## Sensor Configuration

- I2C at 100 kHz, address 0x69.
- Soft reset (0xDEAF at 0x7E), then verify default 24-bit resolution at 0x7B.
- Acceleration: +/-8 g, high-performance mode, 100 Hz ODR, 50 Hz bandwidth
  (0xE028 at 0x26), scaled by 1048576 LSB/g.
- Gyroscope: +/-4000 dps, high-performance mode, 100 Hz ODR, 50 Hz bandwidth
  (0xE058 at 0x27), scaled by 2097.152 LSB/dps.
- Sensor interrupt outputs and data-ready mapping disabled
  (0x0000 at 0x33 and 0x35, verified by readback).
- Burst read 28 bytes at 0x09: discard two dummy bytes, decode six signed
  24-bit axes held in four-byte slots, then signed 16-bit temperature.
- Temperature in Celsius: `25 + raw / 512`.

I2C runs only in the main loop on a 10 ms polling schedule, with no ISR or
interrupt timeout. The LCD labels this mode `POLL`; LCD and serial output
refresh at 5 Hz. This is a latest-sample display, not a lossless logger: display
and serial work can delay polls. Polling is not synchronized to data ready and
does not guarantee a fresh or unique sample on every read. FIFO capture and
motion/VAD features are not enabled. Initialization/read errors replace stale
readings with an error; initialization retries after three seconds.

## Build And Upload

The existing ESP32-C5 N16R8 PlatformIO target and LCD setup are retained.
Only the Adafruit display libraries are needed; Wire is included in Arduino.

```sh
pio run -e esp32-c5-sensairshuttle
pio run -e esp32-c5-sensairshuttle --target upload
pio device monitor -b 115200
```

At rest, one acceleration axis should be close to +/-1 g depending on
orientation, and gyro values should be near zero. An initialization error
prints I2C or register details to serial: check jumper voltage, daughterboard
orientation, chip type, and CS/SDO connections. INT1/INT2 connectivity does not
affect data acquisition in this polling-only firmware.

Before initialization, serial reports SDA/SCL/CS/SDO levels and probes 0x69.
If that address does not ACK, it also probes 0x68. An ACK only at 0x68 suggests
checking SDO/GPIO9; neither address responding suggests checking sensor power,
socket orientation, and mainboard revision. Idle SDA and SCL should be HIGH.
An ACK does not identify the sensor; the chip-ID read still validates BMI423.
Arduino's `endTransmission(false)` only queues the register address, so its
success alone does not establish that the sensor acknowledged a transfer.

## Arduino IDE

Open `SensairBMI423.ino` in this folder. It contains the complete polling-only
sensor firmware and decoder, with USB serial output and no display or SPI
code. No project-specific header or PlatformIO dependency is needed to use
the sketch.

Install **esp32 by Espressif Systems** in Boards Manager (verified with 3.3.8)
and select **ESP32C5 Dev Module**. In Tools, configure:

- USB CDC On Boot: **Enabled**.
- Flash Size: **16MB**.
- PSRAM: **Enabled** (QSPI).
- Serial Monitor: **115200 baud**.

No Library Manager packages are required for the Arduino sketch; Wire is
included in the ESP32 core. Select the USB port and upload, then open Serial
Monitor at **115200 baud**. Keep the sensor voltage jumper at **3.3 V**.

The sketch uses the same sensor configuration as PlatformIO: 100 Hz sensor
ODR, 10 ms timed polling, serial updates at 5 Hz, and no interrupt dependency.
Unlike the PlatformIO version, it does not initialize or use the LCD. Arduino
recursively compiles `src/`, so `src/main.cpp` is guarded by `PLATFORMIO` to
avoid duplicate entry points. When changing sensor configuration or decoding,
keep the sketch and PlatformIO sources in sync.

## Decoder Test

With a host C++ compiler, run:

```sh
g++ -std=c++11 -Wall -Wextra -Werror -I include test/bmi423/test_data.cpp -o bmi423-test
./bmi423-test
```

The test checks Bosch's published sample values, dummy-byte skipping, signed
limits, temperature/scaling, and rejection of incomplete frames. Its
compile-time assertions can also be checked with the installed ESP32 C++
compiler using `-c`; that does not execute the runtime assertions.
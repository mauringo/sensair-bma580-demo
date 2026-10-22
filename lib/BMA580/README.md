# Arduino BMA580 — Bosch SensorAPI Wrapper

An independent Arduino I2C wrapper around the official [Bosch BMA580 SensorAPI](https://github.com/boschsensortec/BMA580_SensorAPI). Supports acceleration, internal temperature, data-ready polling and latched motion interrupts on INT1 or INT2.

## Personal project

This is an independent personal project, developed in a personal capacity. It is not an official project of my employer or any company, and is not sponsored, endorsed, or maintained by them. The work and opinions presented here are my own. Product and company names identify the hardware and upstream software used; they do not imply affiliation or endorsement.

See [DISCLAIMER.md](DISCLAIMER.md). Repository: [mauringo/arduino-bma580](https://github.com/mauringo/arduino-bma580). This wrapper is not an official Bosch Arduino library.

## Install

Copy this directory into your Arduino sketchbook's `libraries` directory, or install a ZIP of this directory using Arduino IDE's **Sketch → Include Library → Add .ZIP Library**. For PlatformIO, put it under your project's `lib/` directory.

The library includes Bosch SensorAPI v4.2.0; no separate Bosch dependency is needed. See [UPSTREAM.md](UPSTREAM.md) for the exact upstream commit.

## Read acceleration

```cpp
#include <Wire.h>
#include <BMA580.h>

BMA580 sensor;

void setup() {
    Serial.begin(115200);
    Wire.begin(); // Select your board's SDA/SCL pins here if needed.
    if (!sensor.begin(Wire)) {
        Serial.println(sensor.lastError());
        while (true) delay(1000);
    }
}

void loop() {
    if (sensor.dataReady()) {
        BMA580::Sample sample;
        if (sensor.read(sample)) {
            Serial.print(sample.x); Serial.print(", ");
            Serial.print(sample.y); Serial.print(", ");
            Serial.println(sample.z);
        }
    }
    delay(10);
}
```

`sample.x`, `y`, and `z` are in g. `rawX`, `rawY`, and `rawZ` contain signed raw samples. `temperatureC` is internal sensor temperature. Defaults: address `0x18`, ±8 g, 100 Hz. Initialize `Wire` and configure a suitable I2C timeout in your application.

The bundled `ReadAcceleration` example includes ESP-SensairShuttle pin configuration; adapt those pins for other boards.

## Motion interrupts

```cpp
// Configure generic motion interrupt on BMA580 INT2:
bool configured = sensor.enableMotionWake(160, 2, 2);

// In loop(), never inside an ISR:
bool motion = false;
if (sensor.readMotionInterrupt(motion) && motion) {
    // Motion detected; status has been acknowledged and rearmed.
}
```

The third argument selects sensor INT1 or INT2, not a microcontroller GPIO. Outputs are latched, active-low and open-drain; use an appropriate pull-up. Threshold/duration are Bosch API units. Status acknowledgment includes a rearm delay, so keep I2C calls outside ISRs. Attach an ISR to the MCU pin wired to the chosen sensor output if needed, and only set a flag there.

MCU sleep and wake configuration belongs to the application. Keep the sensor powered during sleep and use a supported low-level GPIO wake source. On the tested ESP-SensairShuttle assembly BMA580 INT2 shares CSB and reaches GPIO10; this wiring is board-specific.

## API and errors

- `begin(wire, address, range, odr)` initializes the sensor; returns success/failure.
- `dataReady()` checks acceleration availability; false with `lastError() == 0` means no new sample.
- `read(sample)` updates the sample only after all reads succeed.
- `enableMotionWake(threshold=160, duration=2, interruptPin=1)` configures motion detection.
- `readMotionInterrupt(motion)` reads, acknowledges and rearms motion status.
- `lastError()`, `chipId()`, `address()` expose diagnostics.
- `device()` exposes the underlying Bosch device for advanced API calls. Changing range directly can invalidate the wrapper's conversion scale.

Errors include Bosch SensorAPI codes and wrapper codes `-10` (invalid parameters), `-11` (rejected configuration), and `-12` (feature engine initialization timeout).

Transport supports at most 32 bytes per read and 31 bytes per write for Wire buffer compatibility. SPI and streaming FIFO are not implemented. Initialization and feature-engine waits are bounded. Compatibility is declared for Arduino architectures but hardware validation so far covers ESP32-C5; other cores may need adaptation.

## Licensing

The Arduino wrapper is BSD-3-Clause under [LICENSE.wrapper](LICENSE.wrapper). Unmodified Bosch files in `src/bosch/` retain the original Bosch [LICENSE](LICENSE) and notices. See [UPSTREAM.md](UPSTREAM.md).

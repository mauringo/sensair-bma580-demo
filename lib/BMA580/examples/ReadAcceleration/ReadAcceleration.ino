#include <Wire.h>
#include <BMA580.h>

BMA580 sensor;
void setup() {
    Serial.begin(115200);
    delay(2000);
    // ESP-SensairShuttle: keep CS high for I2C, leave SDO unused.
    pinMode(10, INPUT_PULLUP);
    pinMode(9, INPUT);
    pinMode(0, INPUT_PULLUP); // shuttle INT2/CSB: high during startup
#if defined(ARDUINO_ARCH_ESP32)
    Wire.begin(2, 3, 100000);
    Wire.setTimeOut(50);
#else
    Wire.begin(); // board's default SDA/SCL; set a bus timeout if supported
#endif
    if (!sensor.begin(Wire)) {
        Serial.print("BMA580 init error: "); Serial.println(sensor.lastError());
        while (true) delay(1000);
    }
}
void loop() {
    if (sensor.dataReady()) {
        BMA580::Sample sample;
        if (sensor.read(sample)) {
            Serial.print("X:"); Serial.print(sample.x, 4);
            Serial.print(" Y:"); Serial.print(sample.y, 4);
            Serial.print(" Z:"); Serial.print(sample.z, 4);
            Serial.print(" Temp:"); Serial.println(sample.temperatureC, 1);
        }
    }
    if (sensor.lastError() != BMA5_OK) {
        Serial.print("BMA580 read error: "); Serial.println(sensor.lastError());
        delay(1000);
    }
    delay(10);
}

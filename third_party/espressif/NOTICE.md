The LCD command sequence in include/SensairDisplay.h is adapted to Arduino /
Adafruit_ST7789 from Espressif's ESP-SensairShuttle board setup:
https://github.com/espressif/esp-board-manager/blob/b5ce878fb8fbb677ab9d738e2f19d7e8078be1a0/esp_boards/esp_sensairshuttle/setup_device.c

Original copyright: 2025-2026 Espressif Systems (Shanghai) CO LTD.
License: Apache-2.0, reproduced in LICENSE in this directory.
Changes: C++ command table and Adafruit SPI transport; 240x284 drawing area;
original project orientation restored after vendor initialization with 36-row offset.

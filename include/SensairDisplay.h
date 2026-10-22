// SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
// SPDX-License-Identifier: Apache-2.0
// Command sequence adapted from esp-board-manager, esp_sensairshuttle/setup_device.c
// Commit b5ce878fb8fbb677ab9d738e2f19d7e8078be1a0. See third_party/espressif/.
#pragma once
#include <Adafruit_ST7789.h>

class SensairDisplay : public Adafruit_ST7789 {
public:
    using Adafruit_ST7789::Adafruit_ST7789;
    using Adafruit_ST77xx::setColRowStart;
};

inline void initializeSensairPanel(SensairDisplay& tft) {
    // Actual P183B001-V4-CTP panel: 240 x 284, SPI mode 3, 20 MHz.
    tft.init(240, 284, SPI_MODE3);
    tft.setSPISpeed(20000000);
    tft.setColRowStart(0, 0);
    tft.setRotation(0);
    struct Command { uint8_t reg, length, data[14]; uint16_t delayMs; };
    static const Command commands[] = {
        {0x11,0,{},120},
        {0x36,1,{0x00},0},
        {0x3A,1,{0x05},0},
        {0xB2,5,{0x0C,0x0C,0x00,0x33,0x33},0},
        {0xB7,1,{0x05},0},
        {0xBB,1,{0x21},0},
        {0xC0,1,{0x2C},0},
        {0xC2,1,{0x01},0},
        {0xC3,1,{0x15},0},
        {0xC6,1,{0x0F},0},
        {0xD0,1,{0xA7},0},
        {0xD0,2,{0xA4,0xA1},0},
        {0xD6,1,{0xA1},0},
        {0xE0,14,{0xF0,0x05,0x0E,0x08,0x0A,0x17,0x39,0x54,0x4E,0x37,0x12,0x12,0x31,0x37},0},
        {0xE1,14,{0xF0,0x10,0x14,0x0D,0x0B,0x05,0x39,0x44,0x4D,0x38,0x14,0x14,0x2E,0x35},0},
        {0xE4,3,{0x23,0x00,0x00},0},
        {0x21,0,{},0},
        {0x29,0,{},0},
        {0x2C,0,{},0}
    };
    for (const auto& command : commands) {
        tft.sendCommand(command.reg, command.data, command.length);
        if (command.delayMs) delay(command.delayMs);
    }
    // Vendor MADCTL=0 overrides Adafruit rotation. Restore the original
    // portrait orientation after the vendor table; reversed scan needs the
    // unused 36 RAM rows before the 284 visible rows (320 - 284).
    tft.setColRowStart(0, 36);
    tft.setRotation(0);
    tft.setTextWrap(false);
}

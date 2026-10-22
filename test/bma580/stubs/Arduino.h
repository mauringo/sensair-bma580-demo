#pragma once
#include <stdint.h>
#include <stddef.h>
extern uint32_t fakeMs;
inline uint32_t millis() { return fakeMs++; }
inline void delay(uint32_t value) { fakeMs += value; }
inline void delayMicroseconds(uint32_t) {}

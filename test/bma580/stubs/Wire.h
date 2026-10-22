#pragma once
#include <stdint.h>
#include <stddef.h>
#include <vector>
class TwoWire {
public:
    uint8_t regs[256] = {};
    bool nack = false, shortRead = false, initialDummyNack = true;
    uint8_t address = 0, cursor = 0;
    std::vector<uint8_t> tx, rx;
    size_t index = 0;
    void beginTransmission(uint8_t addr) { address = addr; tx.clear(); }
    size_t write(uint8_t byte) { tx.push_back(byte); return 1; }
    size_t write(const uint8_t* bytes, size_t len) {
        tx.insert(tx.end(), bytes, bytes + len); return len;
    }
    uint8_t endTransmission(bool = true) {
        if (nack) return 2;
        cursor = tx[0];
        for (size_t i = 1; i < tx.size(); ++i) {
            uint8_t reg = cursor + i - 1;
            if (reg == 0x11 || reg == 0x12 || reg == 0x13 || reg == 0x14 || reg == 0x15) regs[reg] &= ~tx[i]; // W1C sensor status
            else regs[reg] = tx[i];
        }
        return 0;
    }
    size_t requestFrom(uint8_t, size_t len, bool) {
        rx.clear(); index = 0;
        if (initialDummyNack) { initialDummyNack = false; return 0; }
        if (shortRead) --len;
        for (size_t i = 0; i < len; ++i) rx.push_back(regs[uint8_t(cursor + i)]);
        return len;
    }
    int available() { return rx.size() - index; }
    int read() { return index < rx.size() ? rx[index++] : -1; }
};
extern TwoWire Wire;

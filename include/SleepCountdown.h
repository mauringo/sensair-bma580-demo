#pragma once
#include <stdint.h>

class SleepCountdown {
public:
    explicit SleepCountdown(uint32_t timeoutMs) : timeout_(timeoutMs) {}
    void reset(uint32_t now) { lastActivity_ = now; }
    uint32_t remainingSeconds(uint32_t now) const {
        const uint32_t elapsed = now - lastActivity_;
        return elapsed >= timeout_ ? 0 : (timeout_ - elapsed + 999) / 1000;
    }
    bool expired(uint32_t now) const { return uint32_t(now - lastActivity_) >= timeout_; }
private:
    uint32_t timeout_, lastActivity_ = 0;
};

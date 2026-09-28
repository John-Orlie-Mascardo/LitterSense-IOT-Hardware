#pragma once
#include <stdint.h>

// Calibrated starting values for the fixed 37 cm empty entrance.
constexpr float PRESENCE_MIN_CM = 2.0f;
constexpr float PRESENCE_DETECT_CM = 30.0f;
constexpr float PRESENCE_CLEAR_CM = 34.0f;
constexpr uint32_t PRESENCE_SAMPLE_MS = 100;
constexpr uint32_t PRESENCE_CONFIRM_MS = 5000;

struct UltrasonicPresence {
    bool occupied = false;
    bool confirmed = false;
    bool pending = false;
    uint8_t nearCount = 0;
    uint32_t requestedAt = 0;
    uint32_t sampledAt = 0;

    void request(uint32_t now) {
        requestedAt = now;
        pending = true;
        confirmed = false;
        nearCount = 0; // Require two NEW readings for this RFID request.
    }

    void sample(float cm, uint32_t now) {
        if (uint32_t(now - sampledAt) > 250) nearCount = 0;
        sampledAt = now;
        const bool valid = cm >= PRESENCE_MIN_CM && cm <= 400.0f;
        if (valid && cm <= PRESENCE_DETECT_CM) {
            if (nearCount < 2) ++nearCount;
            if (nearCount == 2) occupied = true;
        } else {
            nearCount = 0; // No echo, invalid range, or outside the detection zone.
            if (valid && cm >= PRESENCE_CLEAR_CM) occupied = false;
        }
        if (pending && uint32_t(now - requestedAt) >= PRESENCE_CONFIRM_MS) pending = false;
        if (pending && nearCount == 2) confirmed = true;
    }

    bool accepted(uint32_t now) const {
        return pending && confirmed && uint32_t(now - requestedAt) < PRESENCE_CONFIRM_MS;
    }
};

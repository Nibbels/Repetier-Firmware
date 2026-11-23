#ifndef PRESSURE_PROTECTION_H
#define PRESSURE_PROTECTION_H

#include <stdint.h>

// The foreground evaluates two pressure windows. The step executor reads the
// single-byte extrusion inhibit published by the faster window.
class PressureProtection {
public:
    struct Limits {
        int16_t minimum;
        int16_t maximum;

        bool exceededBy(int16_t digits) const { return digits < minimum || digits > maximum; }
    };

    Limits pauseLimits() const { return pauseLimits_; }
    void setPauseLimits(Limits limits) { pauseLimits_ = limits; }
    Limits stopLimits() const { return stopLimits_; }
    void setStopLimits(Limits limits) { stopLimits_ = limits; }

    bool pauseEnabled() const { return pauseLimits_.minimum || pauseLimits_.maximum; }
    bool pauseSampleDue(uint32_t now) const { return now - pauseWindow_.sampledAt > EMERGENCY_PAUSE_INTERVAL; }
    bool stopSampleDue(uint32_t now) const { return now - stopWindow_.sampledAt > EMERGENCY_STOP_INTERVAL; }
    bool extrusionBlocked() const { return extrusionBlocked_; }

    bool checkPause(uint32_t now, int16_t digits, bool printingHomed, int16_t& average) {
        pauseWindow_.sampledAt = now;
        if (!printingHomed) {
            pauseWindow_.clear();
            return false;
        }
        return pauseWindow_.add(digits, EMERGENCY_PAUSE_CHECKS, average) && pauseLimits_.exceededBy(average);
    }

    bool checkStop(uint32_t now, int16_t digits) {
        stopWindow_.sampledAt = now;
        int16_t average;
        if (!stopWindow_.add(digits, EMERGENCY_STOP_CHECKS, average))
            return false;
        const bool exceeded = stopLimits_.exceededBy(average);
        extrusionBlocked_ = exceeded;
        return exceeded;
    }

private:
    struct Window {
        uint32_t sampledAt = 0;
        int32_t sum = 0;
        uint8_t count = 0;

        void clear() {
            sum = 0;
            count = 0;
        }

        bool add(int16_t digits, uint8_t samples, int16_t& average) {
            sum += digits;
            if (++count != samples)
                return false;
            average = sum / count;
            clear();
            return true;
        }
    };

    Limits pauseLimits_{EMERGENCY_PAUSE_DIGITS_MIN, EMERGENCY_PAUSE_DIGITS_MAX};
    Limits stopLimits_{EMERGENCY_STOP_DIGITS_MIN, EMERGENCY_STOP_DIGITS_MAX};
    Window pauseWindow_;
    Window stopWindow_;
    volatile bool extrusionBlocked_ = false;
};

#endif

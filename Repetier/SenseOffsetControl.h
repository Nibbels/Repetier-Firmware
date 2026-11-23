#ifndef SENSE_OFFSET_CONTROL_H
#define SENSE_OFFSET_CONTROL_H

#include <stdint.h>

// First-layer pressure learning. ZOffset owns the resulting clearance.
class SenseOffsetControl {
public:
    struct Limits {
        int16_t thresholdDigits;
        int16_t maxMicrometers;
    };

    constexpr explicit SenseOffsetControl(int16_t maxMicrometers)
        : maxMicrometers_(maxMicrometers) {}
    SenseOffsetControl(const SenseOffsetControl&) = delete;
    SenseOffsetControl& operator=(const SenseOffsetControl&) = delete;

    Limits limits() const {
        InterruptProtectedBlock lock;
        return {thresholdDigits_, maxMicrometers_};
    }

    // The caller holds the step interrupt lock while committing the increment.
    bool limitsMatch(const Limits& limits) const {
        return thresholdDigits_ == limits.thresholdDigits && maxMicrometers_ == limits.maxMicrometers;
    }

    void setThresholdDigits(int16_t digits) {
        InterruptProtectedBlock lock;
        thresholdDigits_ = digits;
    }

    void setMaxMicrometers(int16_t micrometers) {
        InterruptProtectedBlock lock;
        maxMicrometers_ = micrometers;
    }

    // M3001/M3909 can load valid saved limits from the step timer's queue.
    void activateStoredLimits(int16_t digits, int16_t micrometers) {
        InterruptProtectedBlock lock;
        if (digits > 0 && digits < EMERGENCY_PAUSE_DIGITS_MAX)
            thresholdDigits_ = digits;
        if (micrometers > 0 && micrometers < 300)
            maxMicrometers_ = micrometers;
    }

    bool disable() {
        InterruptProtectedBlock lock;
        const bool wasEnabled = thresholdDigits_ != 0;
        thresholdDigits_ = 0;
        return wasEnabled;
    }

    bool autoStart() const { return autoStart_; }
    void setAutoStart(bool enabled) { autoStart_ = enabled; }
    bool inLearningRange() const { return inLearningRange_; }

    bool takeSampleSlot(uint32_t now) {
        if (now - lastSampleAt_ <= SENSIBLE_PRESSURE_INTERVAL)
            return false;
        lastSampleAt_ = now;
        return true;
    }

    void resetWindow() {
        sum_ = 0;
        count_ = 0;
        previousAverage_ = 0;
        inLearningRange_ = false;
    }

    void leaveLearningRange() {
        inLearningRange_ = false;
        // Retain the previous trend and the existing signed-window behavior.
        if (sum_ > 0) {
            sum_ = 0;
            count_ = 0;
        }
    }

    uint8_t observe(int16_t digits, int16_t clearanceMicrometers, const Limits& limits) {
        inLearningRange_ = true;
        sum_ += digits;
        if (++count_ < 10)
            return 0;

        const int16_t average = sum_ / count_;
        // Retain half the window. The bounded sum fits exactly in an AVR float,
        // so integer division preserves the former truncation in both signs.
        sum_ /= 2;
        count_ /= 2;

        const int16_t magnitude = average < 0 ? int16_t(-uint16_t(average)) : average;
        const bool falling = magnitude < previousAverage_;
        previousAverage_ = magnitude;
        if (magnitude <= limits.thresholdDigits)
            return 0;
        if (falling)
            return 1;

        const float threshold = limits.thresholdDigits;
        uint8_t increment = 1;
        if (magnitude > threshold * 1.1) {
            float excess = magnitude / threshold - 1;
            if (excess > 2.0f)
                excess = 2.0f;
            increment += int16_t(excess * 12);
        }

        const float fractionUsed = float(clearanceMicrometers) / limits.maxMicrometers;
        if (fractionUsed >= 0.33 && magnitude < threshold * 1.1)
            return 0;
        if (fractionUsed >= 0.25)
            increment -= int16_t(fractionUsed * (increment - 1));
        return increment;
    }

private:
    volatile int16_t thresholdDigits_ = 0;
    volatile int16_t maxMicrometers_;
    bool autoStart_ = false;
    bool inLearningRange_ = false;
    uint32_t lastSampleAt_ = 0;
    int32_t sum_ = 0;
    int16_t previousAverage_ = 0;
    uint8_t count_ = 0;
};

#endif

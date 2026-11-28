#ifndef DIGIT_COMPENSATION_H
#define DIGIT_COMPENSATION_H

#include <stdint.h>

// Foreground DMS filter and Digit Flow regulation. The step timer consumes
// only the published interval scale.
class DigitCompensation {
public:
    struct FlowProfile {
        int16_t minDigits;
        int16_t maxDigits;
        int8_t extrusionPercent;
        int8_t speedPercent;
    };

    constexpr DigitCompensation(int16_t minDigits, int16_t maxDigits)
        : profile_{minDigits, maxDigits, 0, 0} {}
    DigitCompensation(const DigitCompensation&) = delete;
    DigitCompensation& operator=(const DigitCompensation&) = delete;

    FlowProfile flowProfile() const { return profile_; }
    void setFlowProfile(FlowProfile profile) { profile_ = profile; }

    bool digitZEnabled() const { return digitZEnabled_; }
    void setDigitZEnabled(bool enabled) { digitZEnabled_ = enabled; }

    int16_t filteredDigits() const { return filteredDigits_; }
    float extrusionFactor() const { return extrusionFactor_; }
    float feedrateFactor() const { return feedrateFactor_; }
    uint16_t intervalScale() const { return intervalScale_; }

    void resetFlow() {
        neutralizeFactors();
        publishIntervalScale(0);
    }

    // Check the motion phase only after the four-sample filter is primed.
    template <class FlowAllowed>
    void observe(int16_t digits, const volatile char& bedCompensation, FlowAllowed flowAllowed) {
        if (bedCompensation) {
            if (filter(digits))
                updateFlow(flowAllowed());
            else
                neutralizeFactors();
        }

        // Queued M3000/M3001 can change the flag in the step interrupt.
        publishIntervalScale(feedrateFactor_ == 1.0f || !bedCompensation
            ? 0 : uint16_t(1024.0f / feedrateFactor_));
    }

private:
    bool filter(int16_t digits) {
        forceSum_ += digits;
        if (++sampleCount_ < 4) {
            filteredDigits_ = digits;
            return false;
        }

        // Retain three quarters and the division remainder for the next sample.
        filteredDigits_ = forceSum_ / 4;
        forceSum_ -= filteredDigits_;
        --sampleCount_;
        return true;
    }

    void updateFlow(bool allowed) {
        // Preserve the signed 16-bit magnitude used by the AVR thresholds.
        const int16_t magnitude = filteredDigits_ < 0 ? int16_t(-uint16_t(filteredDigits_)) : filteredDigits_;
        extrusionFactor_ = factorAt(magnitude, profile_.extrusionPercent, allowed);
        if (profile_.speedPercent) {
            const float goal = factorAt(magnitude, profile_.speedPercent, allowed);
            feedrateFactor_ += (goal == feedrateFactor_ ? 0.0f : (goal > feedrateFactor_ ? 0.010f : -0.010f));
        } else {
            feedrateFactor_ = 1.0f;
        }
    }

    void neutralizeFactors() {
        extrusionFactor_ = 1.0f;
        feedrateFactor_ = 1.0f;
    }

    float factorAt(int16_t magnitude, int8_t percent, bool allowed) const {
        if (!allowed || !percent || magnitude <= profile_.minDigits)
            return 1.0f;
        if (magnitude >= profile_.maxDigits)
            return 1.0f + 0.01f * percent;
        return 1.0f + 0.01f * percent * (magnitude - profile_.minDigits)
            / (profile_.maxDigits - profile_.minDigits);
    }

    void publishIntervalScale(uint16_t scale) {
        // AVR stores the two bytes separately. Mask interrupts only when the
        // foreground actually changes the value consumed by the step timer.
        if (intervalScale_ != scale) {
            InterruptProtectedBlock lock;
            intervalScale_ = scale;
        }
    }

    FlowProfile profile_;
    int32_t forceSum_ = 0;
    int16_t filteredDigits_ = 0;
    uint8_t sampleCount_ = 0;
    bool digitZEnabled_ = true;
    float extrusionFactor_ = 1.0f;
    float feedrateFactor_ = 1.0f;
    volatile uint16_t intervalScale_ = 0;
};

#endif

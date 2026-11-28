#ifndef Z_OFFSET_H
#define Z_OFFSET_H

#include <stdint.h>

// Foreground state for the saved manual offset and the live SenseOffset.
// The compensation calculation publishes its resulting target to the step timer.
class ZOffset {
public:
    constexpr ZOffset() = default;
    ZOffset(const ZOffset&) = delete;
    ZOffset& operator=(const ZOffset&) = delete;

    int32_t manualMicrometers() const { return manualMicrometers_; }
    int16_t senseMicrometers() const { return senseMicrometers_; }
    int32_t effectiveSteps() const { return effectiveSteps_; }

    int32_t manualSteps(float stepsPerMm) const {
        return toSteps(manualMicrometers_, stepsPerMm);
    }

    void setManualMicrometers(int32_t micrometers, float stepsPerMm) {
        manualMicrometers_ = micrometers;
        refreshSteps(stepsPerMm);
    }

    class SenseIncrement {
        friend class ZOffset;
        SenseIncrement(int16_t micrometers, int32_t steps) : micrometers(micrometers), steps(steps) {}

        int16_t micrometers;
        int32_t steps;
    };

    SenseIncrement prepareSenseIncrement(uint8_t increment, int16_t limit, float stepsPerMm) const {
        const int16_t next = senseMicrometers_ + increment;
        const int16_t bounded = next < limit ? next : limit;
        return SenseIncrement(bounded, toSteps(manualMicrometers_ + bounded, stepsPerMm));
    }

    void applySenseIncrement(const SenseIncrement& increment) {
        senseMicrometers_ = increment.micrometers;
        effectiveSteps_ = increment.steps;
    }

    void clearSenseOffset(float stepsPerMm) {
        senseMicrometers_ = 0;
        refreshSteps(stepsPerMm);
    }

    void adjustWithBedButton(int16_t deltaMicrometers, int32_t manualLimit, float stepsPerMm) {
        int32_t manual = manualMicrometers_ + deltaMicrometers;
        // Snap an edit that crosses zero before transferring any live correction.
        if ((manual > 0 && manual < deltaMicrometers) || (manual < 0 && manual > deltaMicrometers))
            manual = 0;

        if (deltaMicrometers < 0) {
            const int16_t reduction = -deltaMicrometers;
            const int16_t fromSense = senseMicrometers_ < reduction ? senseMicrometers_ : reduction;
            senseMicrometers_ -= fromSense;
            manual += fromSense;
        }

        if (manual < -manualLimit)
            manual = -manualLimit;
        if (manual > manualLimit)
            manual = manualLimit;
        setManualMicrometers(manual, stepsPerMm);
    }

    void refreshSteps(float stepsPerMm) {
        effectiveSteps_ = toSteps(manualMicrometers_ + senseMicrometers_, stepsPerMm);
    }

private:
    static int32_t toSteps(int32_t micrometers, float stepsPerMm) {
        return static_cast<int32_t>(micrometers * stepsPerMm / 1000.0f);
    }

    int32_t manualMicrometers_ = 0;
    int16_t senseMicrometers_ = 0;
    int32_t effectiveSteps_ = 0;
};

#endif // Z_OFFSET_H

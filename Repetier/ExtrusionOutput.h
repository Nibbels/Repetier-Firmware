#ifndef EXTRUSION_OUTPUT_H
#define EXTRUSION_OUTPUT_H

#include <stdint.h>

// Owns the E pulse writer and the Advance timer's signed step balance.
// Change foreground state with step interrupts masked.
class ExtrusionOutput {
public:
    enum class Inhibit : uint8_t { PrintStop = 1, Emergency = 2, MotorsDisabled = 4, Pressure = 8 };

    bool advanceEnabled() const { return advanceEnabled_; }

    // The foreground settles the old motion before changing the E pulse writer.
    void setAdvanceEnabled(bool enabled) {
        if (advanceEnabled_ != enabled) {
            resetDirection();
            advanceEnabled_ = enabled;
        }
    }

    bool blocked() const { return inhibitMask_ != 0; }

    int16_t inhibit(Inhibit reason) {
        inhibitMask_ |= static_cast<uint8_t>(reason);
        const int16_t discarded = pendingSteps_;
        clearSteps();
        return discarded;
    }

    void allow(Inhibit reason) {
        inhibitMask_ &= ~static_cast<uint8_t>(reason);
    }

    void clearSteps() {
        pendingSteps_ = 0;
        leadSteps_ = 0;
        cancelDrain();
    }

    int16_t pendingSteps() const { return pendingSteps_; }
    void addSteps(int16_t steps) { pendingSteps_ += steps; }

    void setLeadSteps(int32_t steps) {
        pendingSteps_ += steps - leadSteps_;
        leadSteps_ = steps;
    }

    // At a handoff the old motion is finished. Release its lead and let the
    // timer emit a final step even if it requires a reversal.
    bool drain() {
        setLeadSteps(0);
        draining_ = pendingSteps_ != 0;
        return !draining_;
    }

    void cancelDrain() { draining_ = false; }

    bool relaxDelayElapsed() const { return !relaxTicks_; }
    void postponeRelaxation() { relaxTicks_ = 70; }

    void onIdleTick() {
        if (relaxTicks_) {
            --relaxTicks_;
        } else if (leadSteps_) {
            pendingSteps_ -= leadSteps_;
            leadSteps_ = 0;
        }
    }

    void setStepInterval(uint8_t ticks) { stepInterval_ = ticks; }
    void resetDirection() { direction_ = 0; }
    uint8_t nextTimerCompare(uint8_t compare);

private:
    volatile int16_t pendingSteps_ = 0;
    volatile int16_t leadSteps_ = 0;
    volatile uint8_t relaxTicks_ = 0;
    uint8_t stepInterval_ = 0;
    int8_t direction_ = 0;
    volatile bool draining_ = false;
    volatile bool advanceEnabled_ = false;
    volatile uint8_t inhibitMask_ = 0;
};

#endif

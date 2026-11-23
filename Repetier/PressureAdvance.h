#ifndef PRESSURE_ADVANCE_H
#define PRESSURE_ADVANCE_H

#include <stdint.h>

// Base E steps and changes in the pressure lead share the extruder timer.
// Writers of multi-byte state run with step interrupts masked.
class PressureAdvance {
public:
    bool active() const { return active_; }

    // The foreground settles the old motion before changing the E pulse writer.
    void setActive(bool active) {
        if (active_ != active) {
            resetDirection();
            active_ = active;
        }
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
    volatile bool active_ = false;
};

#endif

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
            if (enabled)
                armTimer(pendingSteps_);
            else
                stopTimer();
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
        stopTimer();
    }

    int16_t pendingSteps() const { return pendingSteps_; }
    INLINE void addSteps(int16_t steps) {
        const int16_t pending = pendingSteps_ + steps;
        pendingSteps_ = pending;
        armTimer(pending);
    }

    void setLeadSteps(int32_t steps) {
        const int32_t difference = steps - leadSteps_;
        if (difference) {
            leadSteps_ = steps;
            addSteps(static_cast<int16_t>(difference));
        }
    }

    // At a handoff the old motion is finished. Release its lead and let the
    // timer emit a final step even if it requires a reversal.
    bool drain() {
        setLeadSteps(0);
        const int16_t pending = pendingSteps_;
        draining_ = pending != 0;
        if (pending)
            armTimer(pending);
        return pending == 0;
    }

    void cancelDrain() { draining_ = false; }

    bool relaxDelayElapsed() const { return !relaxTicks_; }
    void postponeRelaxation() { relaxTicks_ = 70; }

    void onIdleTick() {
        if (relaxTicks_) {
            --relaxTicks_;
        } else if (leadSteps_) {
            setLeadSteps(0);
        }
    }

    void setStepInterval(uint8_t ticks) { stepInterval_ = ticks; }
    void resetDirection() { direction_ = 0; }
    void serviceTimerCompare();

private:
    INLINE void armTimer(int16_t pending) {
        const uint8_t mask = EXTRUDER_TIMSK;
        if (!(mask & _BV(EXTRUDER_OCIE)))
            startTimer(pending, mask);
    }
    void startTimer(int16_t pending, uint8_t mask);
    static void stopTimer();
    bool canChangeDirection(int16_t pending) const {
        return pending >= 2 || pending <= -2 || draining_;
    }

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

#ifndef XY_MOTION_H
#define XY_MOTION_H

#include <stdint.h>
#include "DirectPosition.h"

// Executed XY positions in steps. A clipped queued move advances nominal
// position but produces no motor pulse. Jogging has its own displacement.
class XYMotion {
public:
    int32_t nominalSteps(uint8_t axis) const { return nominalSteps_[axis]; }
    void setNominalSteps(uint8_t axis, int32_t steps) { nominalSteps_[axis] = steps; }
    void advanceNominal(uint8_t axis, int8_t direction) { nominalSteps_[axis] += direction; }

    int32_t motorSteps(uint8_t axis) const { return motorSteps_[axis]; }
    void setMotorSteps(uint8_t axis, int32_t steps) { motorSteps_[axis] = steps; }
    void recordMotorStep(uint8_t axis, int8_t direction) { motorSteps_[axis] += direction; }

    const DirectPosition& directPosition(uint8_t axis) const { return direct_[axis]; }
    void requestDirect(uint8_t axis, int32_t steps) { direct_[axis].request(steps); }
    void advanceDirect(uint8_t axis, int8_t direction) { direct_[axis].advance(direction); }
    void stopDirect(uint8_t axis) { direct_[axis].stop(); }
    void resetDirect(uint8_t axis) { direct_[axis].reset(); }

    int16_t queuedOvertravel(uint8_t axis) const { return skippedSteps_[axis]; }
    bool hasQueuedOvertravel() const { return queuedOvertravel(0) || queuedOvertravel(1); }
    void clearQueuedOvertravel(uint8_t axis) { skippedSteps_[axis] = 0; }
    void recordSkippedStep(uint8_t axis, int8_t direction) { skippedSteps_[axis] += direction; }

    bool repayQueuedOvertravel(uint8_t axis, int8_t direction) {
        const int16_t blocked = queuedOvertravel(axis);
        if (direction > 0 ? blocked >= 0 : blocked <= 0)
            return false;
        skippedSteps_[axis] = blocked + direction;
        return true;
    }

private:
    volatile int32_t nominalSteps_[2] = {};
    volatile int32_t motorSteps_[2] = {};
    DirectPosition direct_[2];
    volatile int16_t skippedSteps_[2] = {};
};

#endif

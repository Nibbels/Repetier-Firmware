#ifndef MOVE_TARGET_H
#define MOVE_TARGET_H

#include <stdint.h>

// Retained machine coordinates in millimetres and the G-code work offset.
class MoveTarget {
public:
    float machineMM(uint8_t axis) const { return machineMM_[axis]; }
    float remainderMM(uint8_t axis) const { return remainderMM_[axis]; }
    float workOffsetMM(uint8_t axis) const { return workOffsetMM_[axis]; }

    void setMachineMM(uint8_t axis, float mm, float remainder = 0.0f) {
        machineMM_[axis] = mm;
        remainderMM_[axis] = remainder;
    }

    void addMM(uint8_t axis, float distance) {
        if (distance == 0.0f)
            return;

        // TwoSum recovers the rounding error of the addition, including reversals.
        const float previous = machineMM_[axis];
        const float sum = previous + distance;
        const float added = sum - previous;
        const float error = (previous - (sum - added)) + (distance - added);
        const float remainder = remainderMM_[axis] + error;
        const float target = sum + remainder;
        setMachineMM(axis, target, remainder - (target - sum));
    }

    void setFromGCodeMM(uint8_t axis, float mm, bool relative) {
        if (relative)
            addMM(axis, mm);
        else
            setMachineMM(axis, mm - workOffsetMM_[axis]);
    }

    void setWorkOffsetMM(uint8_t axis, float offset) { workOffsetMM_[axis] = offset; }

private:
    volatile float machineMM_[4] = {};
    float remainderMM_[4] = {};
    float workOffsetMM_[3] = {};
};

#endif

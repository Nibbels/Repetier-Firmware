#ifndef XY_OVERTRAVEL_H
#define XY_OVERTRAVEL_H

#include <stdint.h>

// Queued steps that advanced the nominal XY path while a limit suppressed the pulse.
class XYOvertravel {
public:
    int16_t steps(uint8_t axis) const { return steps_[axis]; }
    bool any() const { return steps_[X_AXIS] || steps_[Y_AXIS]; }
    void clear(uint8_t axis) { steps_[axis] = 0; }
    void recordBlockedStep(uint8_t axis, int8_t direction) { steps_[axis] += direction; }

    bool payBack(uint8_t axis, int8_t direction) {
        const int16_t blocked = steps_[axis];
        if (direction > 0 ? blocked >= 0 : blocked <= 0)
            return false;
        steps_[axis] = blocked + direction;
        return true;
    }

private:
    volatile int16_t steps_[2] = {};
};

#endif

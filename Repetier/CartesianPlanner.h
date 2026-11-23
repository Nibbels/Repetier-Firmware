#ifndef CARTESIAN_PLANNER_H
#define CARTESIAN_PLANNER_H

#include <math.h>
#include <stdint.h>

// Last queued XYZ endpoints in steps. XY includes the applied wobble offset.
class CartesianPlanner {
public:
    int32_t queueTarget(uint8_t axis, int32_t targetSteps) {
        const int32_t delta = targetSteps - lastQueuedSteps_[axis];
        lastQueuedSteps_[axis] = targetSteps;
        return delta;
    }

    int32_t lastQueuedSteps(uint8_t axis) const { return lastQueuedSteps_[axis]; }
    void setAxisSteps(uint8_t axis, int32_t steps) { lastQueuedSteps_[axis] = steps; }

    void rescaleAxis(uint8_t axis, float previousMMPerStep, float newStepsPerMM) {
        const float queuedMM = lastQueuedSteps_[axis] * previousMMPerStep;
        lastQueuedSteps_[axis] = lroundf(queuedMM * newStepsPerMM);
    }

private:
    int32_t lastQueuedSteps_[3] = {};
};

#endif

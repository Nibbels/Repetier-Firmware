#ifndef EXTRUSION_PLANNER_H
#define EXTRUSION_PLANNER_H

#include <math.h>
#include <stdint.h>

// Foreground state for logical E and quantization of base extrusion steps.
class ExtrusionPlanner {
public:
    struct LogicalPosition {
        float mm;
        float remainderMM;
    };

    int32_t planSteps(LogicalPosition target, float flow, float stepsPerMm) {
        const float requestedMM = (target.mm - queued_.mm) + (target.remainderMM - queued_.remainderMM);
        const float requestedSteps = requestedMM * flow * stepsPerMm;
        int32_t steps = lroundf(requestedSteps);
        // Keep the fractional motor step in [-0.5, 0.5).
        float remainder = (requestedSteps - steps) + stepRemainder_;
        if (remainder >= 0.5f) {
            ++steps;
            remainder -= 1.0f;
        } else if (remainder < -0.5f) {
            --steps;
            remainder += 1.0f;
        }
        stepRemainder_ = remainder;
        queued_ = target;
        return steps;
    }

    void setLogicalPosition(LogicalPosition position) { queued_ = position; }
    float stepRemainder() const { return stepRemainder_; }
    void restoreStepRemainder(float remainder) { stepRemainder_ = remainder; }

private:
    LogicalPosition queued_{0.0f, 0.0f};
    float stepRemainder_ = 0.0f;
};

#endif

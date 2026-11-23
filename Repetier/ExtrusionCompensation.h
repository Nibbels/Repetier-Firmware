#ifndef EXTRUSION_COMPENSATION_H
#define EXTRUSION_COMPENSATION_H

#include <math.h>

// Geometric extrusion compensation (ECMP) at queued step execution.
// Foreground callers hold the step interrupt lock when writing or saving state.
class ExtrusionCompensation {
public:
    static float prepareRatio(float ratio) {
        // The executor can emit one extra pulse per base step.
        if (!(ratio > 0.0f))
            return 0.0f;
        if (ratio >= 1.0f)
            return 1.0f;

        // Binary32 represents every multiple of 2^-23 in [-1, 2).
        // On that grid, forward and reverse steps cancel exactly.
        constexpr float unitsPerStep = 8388608.0f;
        return lroundf(ratio * unitsPerStep) * (1.0f / unitsPerStep);
    }

    float ratio() const { return ratio_; }
    void publishRatio(float preparedRatio) { ratio_ = preparedRatio; }
    float remainder() const { return remainder_; }
    void restoreRemainder(float remainder) { remainder_ = remainder; }

    void reset() {
        ratio_ = 0.0f;
        remainder_ = 0.0f;
    }

    bool addBaseStep(bool forward) {
        const float ratio = ratio_;
        if (ratio == 0.0f)
            return false;

        if (forward) {
            remainder_ += ratio;
            return remainder_ >= 1.0f;
        }
        remainder_ -= ratio;
        return remainder_ < 0.0f;
    }

    void consumeExtraStep(bool forward) {
        remainder_ += forward ? -1.0f : 1.0f;
    }

private:
    volatile float ratio_ = 0.0f;
    volatile float remainder_ = 0.0f;
};

#endif

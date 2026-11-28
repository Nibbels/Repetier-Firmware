#ifndef EXTRUSION_COMPENSATION_H
#define EXTRUSION_COMPENSATION_H

#include <stdint.h>
#include <math.h>

// Geometric extrusion compensation (ECMP) at queued step execution.
// Foreground callers hold the step interrupt lock when writing or saving state.
// The published ratio and the normalized remainder lie on the exact binary32
// 2^-23 grid. Store the integer units directly: the timer performs no floating
// point work and the priming snapshot needs no conversion under its lock.
class ExtrusionCompensation {
public:
    using Units = int32_t;
    static constexpr Units unitsPerStep = INT32_C(8388608);

    static Units prepareRatio(float ratio) {
        if (!(ratio > 0.0f))
            return 0;
        if (ratio >= 1.0f)
            return unitsPerStep;
        return lroundf(ratio * float(unitsPerStep));
    }

    // Only the foreground LCD reader needs a floating ratio.
    float ratio() const { return float(ratio_) * (1.0f / float(unitsPerStep)); }
    void publishRatio(Units preparedRatio) { ratio_ = preparedRatio; }
    Units remainderUnits() const { return remainder_; }
    void restoreRemainderUnits(Units remainder) { remainder_ = remainder; }

    void reset() {
        ratio_ = 0;
        remainder_ = 0;
    }

    bool addBaseStep(bool forward) {
        const Units ratio = ratio_;
        if (ratio == 0)
            return false;

        // Starting with 0 <= remainder < unitsPerStep and
        // 0 <= ratio <= unitsPerStep, the transient range is
        // [-unitsPerStep, 2*unitsPerStep-1], safely inside int32_t.
        if (forward) {
            remainder_ += ratio;
            return remainder_ >= unitsPerStep;
        }
        remainder_ -= ratio;
        return remainder_ < 0;
    }

    // The executor normalizes after accounting/emitting the extra pulse.
    void consumeExtraStep(bool forward) {
        remainder_ += forward ? -unitsPerStep : unitsPerStep;
    }

private:
    volatile Units ratio_ = 0;
    volatile Units remainder_ = 0;
};

#endif

#ifndef EXTRUSION_COMPENSATION_H
#define EXTRUSION_COMPENSATION_H

// Geometric extrusion compensation (ECMP) at queued step execution.
// Foreground callers hold the step interrupt lock when writing or saving state.
class ExtrusionCompensation {
public:
    float ratio() const { return ratio_; }
    void setRatio(float ratio) { ratio_ = ratio; }
    float remainder() const { return remainder_; }
    void restoreRemainder(float remainder) { remainder_ = remainder; }

    void reset() {
        ratio_ = 0.0f;
        remainder_ = 0.0f;
    }

    bool addBaseStep() {
        if (ratio_ == 0)
            return false;
        remainder_ += ratio_;
        return remainder_ >= 1;
    }

    void consumeWholeSteps() {
        while (remainder_ >= 1)
            remainder_--;
    }

private:
    volatile float ratio_ = 0.0f;
    volatile float remainder_ = 0.0f;
};

#endif

#ifndef RF_RESEARCH_ECMP_Q23_H
#define RF_RESEARCH_ECMP_Q23_H
#include <stdint.h>
#include <math.h>

// Research candidate; no production inclusion. Exact existing 2^-23 grid.
// Foreground converts once before taking the publication lock.
// Normalized remainder lies in [0, one); intermediate in [-one,2*one).
class ExtrusionCompensationQ23 {
public:
    using Units = int32_t;
    static constexpr Units one = INT32_C(8388608);
    static Units prepareRatio(float ratio) {
        if (!(ratio > 0.0f)) return 0;
        if (ratio >= 1.0f) return one;
        return static_cast<Units>(lroundf(ratio * static_cast<float>(one)));
    }
    void publishRatio(Units prepared) { ratio_ = prepared; }
    float ratio() const { return static_cast<float>(ratio_) * (1.0f / one); }
    Units remainderUnits() const { return remainder_; }
    void restoreRemainderUnits(Units saved) { remainder_ = saved; }
    void reset() { ratio_ = 0; remainder_ = 0; }

    bool addBaseStep(bool forward) {
        const Units ratio = ratio_;
        if (!ratio) return false;
        const Units r = remainder_ + (forward ? ratio : -ratio);
        remainder_ = r;
        return forward ? r >= one : r < 0;
    }
    void consumeExtraStep(bool forward) {
        remainder_ += forward ? -one : one;
    }
private:
    volatile Units ratio_ = 0;
    volatile Units remainder_ = 0;
};
static_assert(sizeof(ExtrusionCompensationQ23) == 8, "No extra ECMP SRAM");
#endif

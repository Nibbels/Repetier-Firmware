#ifndef RF_TASK0032_COMPARE_PHASE_CLOCK_H
#define RF_TASK0032_COMPARE_PHASE_CLOCK_H

#include <stdint.h>

// Conservative physical-edge age in Timer1 CTC coordinates, NOT a global
// wall clock. Timer1 owns every operation. Register observations must be
// coherent (AVR's shared 16-bit temporary latch) and IRQs closed.
//
// On vector entry, before changing OCR1A, account one compare period.
// After EVERY OCR1A write, observe the sticky OCF1A flag, without clearing
// it. If set, the next vector may belong to a different TOP: do not credit
// that period. Also skip credit if an edge sample already follows the
// compare whose vector is still pending. Multiple coalesced compares are
// deliberately undercounted, never converted into fictitious time.
//
// Read TCNT1 first, then OCF1A. Reject admission if pending. Sample again
// after the actual physical edge, while still in the edge transaction.
// Counting begins no earlier than the physical edge. No busy waits and
// no dependence on any other timer, IRQ, packing group or motion owner.
class ComparePhaseClock {
public:
    static constexpr int32_t maxAge = 0xffffffL;

    void onCompare(uint16_t previousTop) {
        if (skipCompare_) {
            skipCompare_ = false;
            return;
        }
        const int32_t next = ageAtEpoch_ + int32_t(previousTop) + 1;
        ageAtEpoch_ = next > maxAge ? maxAge : next;
    }

    void afterTopWrite(bool comparePending) {
        if (comparePending)
            skipCompare_ = true;
    }

    void afterPhysicalEdge(uint16_t phase, bool comparePending) {
        ageAtEpoch_ = -int32_t(phase);
        if (comparePending)
            skipCompare_ = true;
    }

    uint32_t elapsedLowerBound(uint16_t phase, bool comparePending) const {
        if (comparePending)
            return 0;
        const int32_t age = ageAtEpoch_ + int32_t(phase);
        if (age <= 0)
            return 0;
        return uint32_t(age > maxAge ? maxAge : age);
    }

    bool admits(uint16_t phase, bool comparePending, uint32_t minimumTicks) const {
        return !comparePending && elapsedLowerBound(phase, false) >= minimumTicks;
    }

private:
    // Initial state has no prior pulse. Saturation is valid only for
    // requested minimum intervals <= maxAge (same 24-bit scheduling domain
    // as Timer1's long-delay path).
    int32_t ageAtEpoch_ = maxAge;
    bool skipCompare_ = false;
};

#endif

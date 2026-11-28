#!/usr/bin/env python3
"""Disposable old-P0 candidate: shared Z compare-phase admission.

The 512-cycle floor comes from the existing executor base-interval floor.
It is NOT the final Z mechanical/ramp/packing policy. CMP still uses its
legacy mass/slot eligibility, then the same physical time admission. All
nominal slots that could emit Z are conservatively deferred as a whole,
even when later opposite-correction cancellation would avoid a pulse.
No production edit, new timer, busy wait or changed heater/IRQ ownership.
"""
import argparse
from pathlib import Path


def replace_once(path, old, new):
    source = path.read_text()
    assert source.count(old) == 1, (path, old)
    path.write_text(source.replace(old, new))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--root", type=Path, required=True)
    a = p.parse_args()
    src = a.root / "Repetier"
    hdr = src / "ComparePhaseClock.h"
    assert not hdr.exists()
    hdr.write_bytes(Path(__file__).with_name("compare-phase-clock.h").read_bytes())

    h = src / "ZMotion.h"
    replace_once(h, '#include "DirectPosition.h"', '''#include "DirectPosition.h"
#ifdef __AVR__
#include "ComparePhaseClock.h"
#endif''')
    replace_once(h, '    static INLINE void finishPulse() { WRITE(Z_STEP_PIN, LOW); }', '''#ifdef __AVR__
    // Only Timer1, with IRQs closed. OCR/TIFR1 have a single owner.
    static INLINE void accountCompare(uint16_t previousTop) {
        physicalClock_.onCompare(previousTop);
    }
    static INLINE void observeTopWrite() {
        physicalClock_.afterTopWrite(TIFR1 & _BV(OCF1A));
    }
    // Zero = admitted. Otherwise return a compare-relative retry time.
    // The sample precedes all logical mutations of a potentially Z slot.
    static INLINE uint32_t physicalRetryAt(uint16_t minimumTicks) {
        const uint16_t phase = TCNT1;
        const bool pending = TIFR1 & _BV(OCF1A);
        if (pending)
            return uint32_t(phase) + minimumTicks;
        const uint32_t elapsed = physicalClock_.elapsedLowerBound(phase, false);
        return elapsed >= minimumTicks ? 0 : uint32_t(phase) + minimumTicks - elapsed;
    }
#endif
    static INLINE void finishPulse() { WRITE(Z_STEP_PIN, LOW); }''')
    replace_once(h, '        motorSteps_ += direction;', '''        motorSteps_ += direction;
#ifdef __AVR__
        const uint16_t phase = TCNT1;
        physicalClock_.afterPhysicalEdge(phase, TIFR1 & _BV(OCF1A));
#endif''')
    replace_once(h, '    volatile int32_t motorSteps_ = 0;', '''#ifdef __AVR__
    static ComparePhaseClock physicalClock_;
#endif
    volatile int32_t motorSteps_ = 0;''')
    replace_once(src / "ZMotion.cpp", '#include "Repetier.h"', '''#include "Repetier.h"

#ifdef __AVR__
ComparePhaseClock ZMotion::physicalClock_;
#endif''')

    step = src / "StepExecutor.cpp"
    replace_once(step, '    const int32_t cmpDiff = Printer::zMotion.stepCorrection();', '''#ifdef __AVR__
    if (ZMotion::physicalRetryAt(minimumStepIntervalTicks))
        return;
#endif
    const int32_t cmpDiff = Printer::zMotion.stepCorrection();''')
    replace_once(step, '    for (; completed < max_loops;) {', '''    for (; completed < max_loops;) {
#ifdef __AVR__
        // Admission before any E/XYZ Bresenham, Advance or ledger change.
        // A later opposite-correction cancellation is deliberately not
        // predicted in this first timing-floor experiment.
        if (move->isZMove() && errors.value[Z_AXIS] < move->delta[Z_AXIS]) {
            const uint32_t retry = ZMotion::physicalRetryAt(minimumStepIntervalTicks);
            if (retry) {
                if (!completed)
                    return retry; // preserve the armed profile and all slot state
                break; // account only the completed prefix, never the rejected slot
            }
        }
#endif''')

    hal = src / "HAL.cpp"
    # setTimer's assembly writes OCR1A and stepperWait. Make the subsequent
    # sticky-flag observation ordered with those MMIO writes.
    replace_once(hal, '        : "r18");                                                                           // Clobber',
                 '        : "r18", "cc", "memory");                                                           // Clobber')
    replace_once(hal, '} // setTimer', '''    ZMotion::observeTopWrite();
} // setTimer''')
    replace_once(hal, '''    // Mechanical block is also a motion fence.''', '''    // Account before every possible OCR1A rewrite. The terminal thermal
    // guard above remains prior to motion accounting.
    ZMotion::accountCompare(OCR1A);
    // Mechanical block is also a motion fence.''')
    replace_once(hal, '''        OCR1A = 3000;
        return;''', '''        OCR1A = 3000;
        ZMotion::observeTopWrite();
        return;''')
    replace_once(hal, '''    if (doExit)
        return;''', '''    if (doExit) {
        ZMotion::observeTopWrite();
        return;
    }''')
    replace_once(hal, '    OCR1A = 61000;', '''    OCR1A = 61000;
    ZMotion::observeTopWrite();''')
    replace_once(hal, '''    OCR1A = 3000;

    sbi(TIMSK1, OCIE1A);''', '''    OCR1A = 3000;
    ZMotion::observeTopWrite();

    sbi(TIMSK1, OCIE1A);''')
    print("prepared shared Z phase clock, physical 512-cycle floor and whole-slot deferral")


if __name__ == "__main__":
    main()

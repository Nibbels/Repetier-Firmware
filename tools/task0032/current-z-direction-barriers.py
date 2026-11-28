#!/usr/bin/env python3
"""Separate correction completion/admission from the last electrical DIR bit.

The old cadence is retained for this independent semantic checkpoint.
A blocked or unserviceable correction does not hold a foreground barrier.
The physical direction is changed only by the Z pulse owner.
"""
import argparse
from pathlib import Path

def rep(p,a,b):
    s=p.read_text()
    assert s.count(a)==1,(str(p),a,s.count(a))
    p.write_text(s.replace(a,b))

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--root",type=Path,required=True)
    a=ap.parse_args()
    fw=a.root/"Repetier"
    h=fw/"ZMotion.h"
    c=fw/"ZMotion.cpp"
    rep(h,"    bool correctionFollowsDirection() const;\n","")
    f=h.read_text()
    begin=f.index("    static INLINE bool directionIsPositive()")
    end=f.index("    static INLINE void finishPulse()",begin)
    physical=f[begin:end]
    f=f[:begin]+f[end:]
    f=f.replace("    enum class Step : uint8_t",physical+"\n    enum class Step : uint8_t",1)
    h.write_text(f)
    f=c.read_text()
    begin=f.index("bool ZMotion::correctionFollowsDirection() const")
    end=f.index("int32_t ZMotion::stepCorrection()",begin)
    f=f[:begin]+"""bool ZMotion::correctionDelaysQueue(bool compensationEnabled) const {
    const int32_t target = correctionTarget_;
    const int32_t pending = target - appliedCorrection_;
    if (!pending)
        return false;

    // Preserve the enabled 0.25 mm queue catch-up policy. For explicit
    // switch-off, unwind any feasible residual regardless of last DIR.
    if (compensationEnabled && (!target || abs(pending) <= queueWaitSteps_))
        return false;
    return !correctionLimited(pending > 0);
}

"""+f[end:]
    c.write_text(f)
    rep(fw/"Printer.cpp","    return !blockAll && zMotion.correctionFollowsDirection();",
        """    return !blockAll && !zMotion.scanInProgress() &&
        (motorWake.readyFor(4) || zMotion.isHomed()) && zMotion.correctionCanMove();""")
    rep(fw/"Printer.cpp","""        {
            InterruptProtectedBlock noInts;
            if (blockAll || !zMotion.correctionCanMove()) {
                break;
            }
        }""","""        if (!needsCMPwait())
            break;""")
    rep(fw/"StepExecutor.cpp","""    if (move.isZMove()) {
        ZMotion::setDirection(move.isZPositiveMove());
    }
""","")
    rep(fw/"StepExecutor.cpp","            ZMotion::setDirection(direct.isZPositiveMove());\n","")
    fixture=a.root/"tests/z-motion.cpp"
    rep(fixture,"    ZMotion::setDirection(true);","    positive = true; // test pin state, not a second production owner")
    rep(fixture,"    ZMotion::setDirection(false);","    positive = false;")
    rep(fixture,"""    reset(500);
    zMotion.requestCorrection(25);
    assert(!StepExecutor::needCmpWait() && Printer::needsCMPwait());""","""    reset(500);
    positive = false; // actual DIR points opposite to the pending correction
    zMotion.requestCorrection(25);
    assert(!StepExecutor::needCmpWait() && Printer::needsCMPwait());
    positive = true;
    assert(!StepExecutor::needCmpWait() && Printer::needsCMPwait());""")
    rep(fixture,"""    Printer::disableCMPnow(true);
    assert(waitedMs > 0 && !Printer::doHeatBedZCompensation);
    assert(motorPosition == 500);""","""    Printer::disableCMPnow();
    assert(StepExecutor::needCmpWait() && Printer::needsCMPwait());
    Printer::disableCMPnow(true);
    assert(waitedMs > 0 && !Printer::doHeatBedZCompensation);
    assert(motorPosition == 500);

    // A residual offset with no reference and no ready motor may not
    // block forever. It remains in the physical ledger until ref reset.
    reset(500);
    zMotion.requestCorrection(10);
    settleCorrection();
    assert(motorPosition == 510);
    zMotion.setHomed(false);
    motorWake.reset();
    Printer::disableCMPnow(true);
    assert(waitedMs == 0 && motorPosition == 510);
    assert(!StepExecutor::needCmpWait() && !Printer::needsCMPwait());""")
    for p in fw.glob("*"):
        if p.suffix in (".h",".cpp",".ino"):
            s=p.read_text()
            assert "correctionFollowsDirection" not in s,p
            assert "ZMotion::setDirection" not in s,p
    print("Z DIR prewrites removed; completion and queue unwind independent of pin")

if __name__=="__main__":
    main()

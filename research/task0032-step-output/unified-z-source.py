#!/usr/bin/env python3
"""Build a unified Timer1 Z-correction counterfactual on the green S1 image.

This intentionally changes correction scheduling and lifecycle semantics.
No production source writes; not a qualified firmware or governor.
"""
import argparse,difflib,importlib.util,pathlib,tempfile
HERE=pathlib.Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("q23_linked",HERE/"q23-linked.py")
q=importlib.util.module_from_spec(spec);spec.loader.exec_module(q)
base_modify=q.modify

def modify(dst,tag):
    assert tag=="unified-z"
    base_modify(dst,"wake-jog-idle")
    def edit(name,old,new):
        p=dst/name;p.write_text(q.replace(p.read_text(),old,new))
    def section(name,start,end,replacement):
        p=dst/name;s=p.read_text();i=s.index(start);j=s.index(end,i)
        p.write_text(s[:i]+replacement+s[j:])
    (dst/"ZCorrectionResearch.h").write_text((HERE/"z-correction-clock.h").read_text())
    edit("StepExecutor.h","    static void stepSlowedZCompensation();\n    static bool needCmpWait();",
        """    // One Timer1 owner, no secondary G-code queue. UINT32_MAX means
    // no correction deadline (nominal/housekeeping still scheduled).
    static void configureCorrection(uint32_t acceleration, uint16_t speed);
    static bool reservesZ();
    static bool correctionRequested();
    static bool correctionConfigured();
    static uint32_t serviceCorrection(uint32_t epoch, bool nominalOwnsZ);""")
    edit("StepExecutor.cpp",'#include "StepExecutor.h"','#include "StepExecutor.h"\n#include "ZCorrectionResearch.h"')
    section("StepExecutor.cpp","void StepExecutor::stepSlowedZCompensation()", "/**\n  Processes the moves from the queue",r'''
namespace {
using namespace ZCorrectionResearch;
// Reuse the already linked RFx000 math, no new reciprocal table.
struct CorrectionMath {
    static uint16_t floorRoot(uint32_t x) {
        if (x>INT32_MAX) return root(x);
        if (!x) return 0;
        const uint16_t r=HAL::integerSqrt(int32_t(x));
        return uint32_t(r)*r>x?r-1:r;
    }
    static uint32_t period(uint16_t speed) { return HAL::CPUDivU2(speed); }
};
struct CorrectionState {
    FastRamp<CorrectionMath> ramp;
    Limits limits{};
    FastPulse armed{};
    int32_t armedError=0;
    uint32_t anchor=0;
    uint32_t due=0;

    void cancel() {
        ramp.cancel();
        armed.sign=0;
    }
};
static_assert(sizeof(CorrectionState)==41, "AVR correction state fixture layout");
CorrectionState zCorrection;

uint32_t correctionClock(uint32_t epoch) {
    // No other Timer1 writer is allowed inside the current compare ISR;
    // protect the AVR temporary high-byte latch regardless.
    InterruptProtectedBlock lock;
    return epoch + TCNT1;
}
}

void StepExecutor::configureCorrection(uint32_t acceleration, uint16_t speed) {
    const auto limits=ZCorrectionResearch::Limits::prepare(acceleration,speed);
    InterruptProtectedBlock lock;
    zCorrection.cancel();
    zCorrection.limits=limits;
}

bool StepExecutor::correctionConfigured() {return zCorrection.limits.maxLevel!=0;}

bool StepExecutor::correctionRequested() {
    return zCorrection.armed.sign || Printer::zMotion.correctionError()!=0;
}

bool StepExecutor::reservesZ() {
    // Direct preparation retains priority; do not move Z during publication.
    if (PrintLine::directMoveActive())
        return PrintLine::directMovePreparing(PrintLine::direct.task) || PrintLine::direct.isZMove();
    if (!Printer::pause.allowsQueueAdvance()) return false;
    auto& queue=PrintLine::queue;
    auto* move=queue.active();
    if (!move && queue.count()) move=&queue.slot(queue.readIndex());
    return move && move->queueTaskId()==QueueTask::None && !move->isWarmUp() && move->isZMove();
}

uint32_t StepExecutor::serviceCorrection(uint32_t epoch, bool nominalOwnsZ) {
    using namespace ZCorrectionResearch;
    auto& c=zCorrection;
    auto& z=Printer::zMotion;
    if (Printer::blockAll || z.scanInProgress() || !z.isHomed() || nominalOwnsZ) {
        c.cancel();
        return UINT32_MAX;
    }
    const int32_t error=z.correctionError();
    if (!error || !z.correctionCanMove() || !c.limits.maxLevel) {
        c.cancel();
        return UINT32_MAX;
    }
    if (!motorWake.readyFor(4) && !ensureMotorsReady(4)) {
        c.cancel();
        return 2000; // retry, never busy-wait for a physical motor
    }

    uint32_t now=correctionClock(epoch);
    if (!c.armed.sign || c.armedError!=error) {
        const auto plan=c.ramp.plan(error,c.limits);
        if (!c.armed.sign || plan.safetyRestart) {
            if (plan.safetyRestart) c.ramp.cancel();
            c.anchor=now;
        }
        uint32_t due=c.anchor+plan.interval;
        // A live update may postpone a previously armed pulse, never
        // accelerate it or trigger a catch-up burst.
        if (c.armed.sign && c.armed.sign==plan.sign && !reached(due,c.due))
            due=c.due;
        c.armed=plan;
        c.armedError=error;
        c.due=due;
    }
    if (c.armed.sign && reached(now,c.due)) {
        // Current target/owner/limit checked above; foreground cannot
        // interleave during Timer1. stepCorrection rechecks physical limit.
        const int32_t moved=z.stepCorrection();
        if (!moved) {
            c.cancel();
            return UINT32_MAX;
        }
        {
            InterruptProtectedBlock pulseLock;
            ZMotion::finishPulse();
        }
        c.ramp.commit(c.armed);
        // Timestamp AFTER the actual pulse, not its old requested deadline.
        // Never repay lateness with an early second pulse.
        c.anchor=correctionClock(epoch);
        c.armedError=z.correctionError();
        c.armed=c.ramp.plan(c.armedError,c.limits);
        c.due=c.anchor+c.armed.interval;
        if (!c.armed.sign) c.ramp.cancel();
    }
    now=correctionClock(epoch);
    if (!c.armed.sign) return UINT32_MAX;
    // Return time relative to the compare epoch, not to ISR exit.
    return reached(now,c.due)?now-epoch:c.due-epoch;
}

''')
    edit("StepExecutor.cpp","""        else if (move->isXOrYMove()) {
            StepExecutor::stepSlowedZCompensation(); // Z is free here. This is no Z-Move
        }
""","")
    edit("ZMotion.h","    bool correctionFollowsDirection() const;\n    bool correctionDelaysQueue(bool compensationEnabled) const;","    int32_t correctionError() const { return pendingCorrection(); }")
    section("ZMotion.cpp","bool ZMotion::correctionFollowsDirection() const", "int32_t ZMotion::stepCorrection()", "")
    # Preserve the old ZMotion field layout for first linked fixture comparison.
    # Remove unused queueWaitSteps_ after the independent new behavior is qualified.
    edit("Printer.h","    static bool needsCMPwait(void);","    static bool correctionInProgress(void);")
    edit("Printer.h","    static void disableCMPnow(bool wait = false);","    static void disableCMPnow(void);")
    edit("Printer.h","Printer::disableCMPnow(true); //true == wait for move while HOMING","Printer::disableCMPnow(); // reference invalidation never moves Z")
    section("Printer.cpp","bool Printer::needsCMPwait(void)", "void Printer::", """bool Printer::correctionInProgress(void) {
    InterruptProtectedBlock lock;
    return !blockAll && zMotion.isHomed() && !zMotion.scanInProgress()
        && StepExecutor::correctionConfigured() && zMotion.correctionCanMove();
}

""")
    p=dst/"Printer.cpp";s=p.read_text()
    i=s.index("void Printer::disableCMPnow(bool wait)");j=s.index("\n}",i)+2
    p.write_text(s[:i]+"""void Printer::disableCMPnow(void) {
    doHeatBedZCompensation = 0;
    InterruptProtectedBlock lock;
    zMotion.requestCorrection(0);
}"""+s[j:])
    for name in ("Commands.cpp","PrintPause.cpp","ScanProbe.cpp"):
        p=dst/name;s=p.read_text();assert "Printer::needsCMPwait()" in s
        p.write_text(s.replace("Printer::needsCMPwait()","Printer::correctionInProgress()"))
    edit("RF.cpp","Printer::disableCMPnow(true);","Printer::disableCMPnow();")
    edit("HeatBedScan.cpp","Printer::disableCMPnow(false);","Printer::disableCMPnow();")
    # Derived values only in foreground, reuse current motion configuration.
    edit("Printer.cpp","""    // 07 11 2017 https://github.com/repetier/Repetier-Firmware/commit/""",
         """    const float zRate=axisMaxFeedrate(Z_AXIS)*stepsPerMm(Z_AXIS);
    const uint16_t correctionRate=zRate>0
        ? uint16_t(RMath::min(zRate,float(F_CPU/StepExecutor::minimumStepIntervalTicks))) : 0;
    const uint32_t printA=maxPrintAccelerationStepsPerSquareSecond_[Z_AXIS];
    const uint32_t travelA=maxTravelAccelerationStepsPerSquareSecond_[Z_AXIS];
    StepExecutor::configureCorrection(printA<travelA?printA:travelA,correctionRate);

    // 07 11 2017 https://github.com/repetier/Repetier-Firmware/commit/""")
    # StepExecutor.h may not be included by Repetier.h on every source file.
    p=dst/"Printer.cpp";s=p.read_text()
    if '#include "StepExecutor.h"' not in s:
        p.write_text(s.replace('#include "Repetier.h"','#include "Repetier.h"\n#include "StepExecutor.h"',1))
    # Actual CTC compare epoch: old OCR1A+1 clocks since prior compare.
    # Existing setTimer retains late guard and atomic register publication.
    # Chunk long nominal waits at 32769 ticks so new targets get observed.
    # No extra hardware timer. This is not a full concurrent clock proof.
    section("HAL.cpp","ISR(TIMER1_COMPA_vect) {","// Heater and secondary fan PWM",r'''
static uint32_t motionEpoch=0;
static uint32_t nominalDeadline=0;

ISR(TIMER1_COMPA_vect) {
    cbi(TIMSK1, OCIE1A);
    const uint16_t elapsedTop=OCR1A;
    OCR1A=61000;
    motionEpoch+=uint32_t(elapsedTop)+1;
    const uint32_t now=motionEpoch;
    using namespace ZCorrectionResearch;
    HAL::allowInterrupts();

    const bool hadZ=StepExecutor::correctionRequested() && StepExecutor::reservesZ();
    if (reached(now,nominalDeadline)) {
        HAL::forbidInterrupts();
        uint32_t delay=3000;
        if (PrintLine::directMoveActive())
            delay=StepExecutor::performDirectMove();
        else if (PrintLine::hasLines() && Printer::pause.allowsQueueAdvance())
            delay=StepExecutor::performQueueMove();
        else
            Printer::extrusionOutput.onIdleTick();
        nominalDeadline=now+delay+1;
    }

    HAL::allowInterrupts();
    uint32_t next=nominalDeadline-now;
    const uint32_t z=StepExecutor::correctionRequested()
        ? StepExecutor::serviceCorrection(now,hadZ || StepExecutor::reservesZ()) : UINT32_MAX;
    // If an extra compare would be immediately followed by nominal motion,
    // share the nominal release instead. This is a conservative initial
    // scheduler spacing hypothesis, not a CPU governor or a new speed cap.
    if (z<next && next-z>=StepExecutor::minimumStepIntervalTicks) next=z;
    if (next>32769) next=32769;
    if (!next) next=1;
    setTimer(next-1);
    sbi(TIMSK1, OCIE1A);
} // ISR(TIMER1_COMPA_vect)

''')
    edit("HAL.cpp",'#include "StepExecutor.h"','#include "StepExecutor.h"\n#include "ZCorrectionResearch.h"')
    edit("HAL.cpp", ''': "=r"(b)
        : "r"(a)
        : "r18", "r19", "r27", "r26");''',
        ''': "=r"(b), "+r"(a)
        :
        : "r18", "r19", "r27", "r26", "cc");''')
    # Fail closed on missed active legacy callers, excluding comments.
    for p in dst.glob("*"):
        if p.suffix not in (".h",".cpp"):continue
        for line in p.read_text().splitlines():
            line=line.split("//")[0]
            assert not any(n in line for n in ("stepSlowedZCompensation(","needCmpWait(","needsCMPwait(","correctionFollowsDirection(","correctionDelaysQueue(","disableCMPnow(true","disableCMPnow(false")),(p,line)
    diff=[]
    for p in sorted(dst.glob("*")):
        if not p.is_file():continue
        old=(q.FW/p.name).read_bytes() if (q.FW/p.name).is_file() else b""
        new=p.read_bytes()
        if old!=new:
            diff.extend(difflib.unified_diff(old.decode().splitlines(True),new.decode().splitlines(True),fromfile="a/Repetier/"+p.name,tofile="b/Repetier/"+p.name))
    return "".join(diff)

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--out",type=pathlib.Path,required=True)
    p.add_argument("--tools",type=pathlib.Path,required=True)
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    q.modify=modify
    with tempfile.TemporaryDirectory(prefix="unified-z-") as tmp:
        q.build(pathlib.Path(tmp)/"Repetier",a.out,"unified-z",a.tools)

if __name__=="__main__":main()

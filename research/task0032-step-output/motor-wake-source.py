"""Temporary-source motor wake counterfactual, never edit production tree.

Timer1 already owns move admission: use it to start a bounded GPIO wake
transaction, return immediately, and retry PREPARED/front on a later compare.
No foreground mailbox, no polling loop. Not qualified for production.
"""
def apply(fw,here,changes,replace):
    def get(p):
        return changes[p] if p in changes else (fw/p).read_text()
    def edit(p,a,b):
        changes[p]=replace(get(p),a,b)
    # Retain old flags as observational fields for linked diagnostics. They
    # are no longer readiness authority; remove after integrated inventory.
    changes["MotorWakeResearch.h"]=(here/"motor-wake.h").read_text().replace(
        "#ifdef __AVR__\nstatic_assert", "extern MotorWake motorWake;\n#ifdef __AVR__\nstatic_assert")
    edit("Printer.h","#define PRINTER_H","#define PRINTER_H\n#include \"MotorWakeResearch.h\"")
    edit("Extruder.h","#define EXTRUDER_H","#define EXTRUDER_H\n#include \"MotorWakeResearch.h\"")
    for axis,bit in zip("XYZ",(1,2,4)):
        p="Printer.h"
        # Stop eligibility before the actual pin transition. Z's existing
        # compensation teardown remains BEFORE this tiny transaction.
        old=f"""#if ({axis}_ENABLE_PIN > -1)
        WRITE({axis}_ENABLE_PIN, !{axis}_ENABLE_ON);
#endif // ({axis}_ENABLE_PIN > -1)

        Printer::enabledStepper[{axis}_AXIS] = 0;"""
        new=f"""        {{
            InterruptProtectedBlock wakeLock;
            motorWake.beforeDisable({bit});
{old}
        }}"""
        edit(p,old,new)
        edit(p,f"void enable{axis}Stepper() {{\n        unmarkAllSteppersDisabled();",
               f"void assert{axis}Stepper() {{")
        edit(p,f"""        if (!Printer::enabledStepper[{axis}_AXIS]) {{
            Printer::enabledStepper[{axis}_AXIS] = 1;
            HAL::delayMilliseconds(stepperEnableDelayMs);
        }}""",f"        Printer::enabledStepper[{axis}_AXIS] = 1;")
    p="Extruder.h"
    edit(p,"static inline void enable() {","static inline void assertEnable() {")
    edit(p,"""        if (!Extruder::current->enabled) {
            Extruder::current->enabled = 1;
            HAL::delayMilliseconds(stepperEnableDelayMs);
        }""","        Extruder::current->enabled = 1;")
    p="Extruder.cpp"
    # Tool switch does not lower SLEEPn; retain physical readiness, inhibit
    # Advance until the newly selected tool is ready. Old outputs drained.
    edit(p,"        current->enabled = 0;\n        current = &extruder[extruderId];",
           """        current = &extruder[extruderId];
        if (!motorWake.readyFor(uint8_t(8U << current->id)))
            Printer::extrusionOutput.inhibit(ExtrusionOutput::Inhibit::MotorsDisabled);""")
    edit(p,"    Extruder::current = &extruder[0];",
           """    Extruder::current = &extruder[0];
    Printer::extrusionOutput.inhibit(ExtrusionOutput::Inhibit::MotorsDisabled);""")
    edit(p,"        const int16_t discarded = Printer::extrusionOutput.inhibit(ExtrusionOutput::Inhibit::MotorsDisabled);",
           """        motorWake.beforeDisable(0x18);
        const int16_t discarded = Printer::extrusionOutput.inhibit(ExtrusionOutput::Inhibit::MotorsDisabled);""")
    p="StepExecutor.h"
    edit(p,"    static long performDirectMove();",
           """    static long performDirectMove();
    // Foreground M99 and Timer1 share the same bounded GPIO transaction.
    static bool ensureMotorsReady(uint8_t requested);""")
    edit(p,"    static void enableSteppers(PrintLine& move);",
           "    static bool enableSteppers(PrintLine& move);")
    # M99 is the only other enable caller in the actual source inventory.
    # Drain before releasing axes; keep foreground services alive during
    # the explicit suspend and subsequent nonblocking wake retries.
    # Admission/reentrant panel interaction still needs integrated P0 replay.
    p="GCodeMotion.cpp"
    changes[p]='#include "StepExecutor.h"\n'+get(p)
    edit(p,"void suspendAxes(GCode* com) {\n    millis_t wait = 10000L;",
           """void suspendAxes(GCode* com) {
    if (!Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Resume))
        return;
    if (Printer::blockAll || Printer::printJob.commandsBlocked())
        return;
    millis_t wait = 10000L;""")
    edit(p,"""    while (wait - HAL::timeInMilliseconds() < 100000L) {
        Commands::checkForPeriodicalActions(Paused); // Check heater and other stuff every n milliseconds
    }
    if (com->hasX())
        Printer::enableXStepper();
    if (com->hasY())
        Printer::enableYStepper();
    if (com->hasZ())
        Printer::enableZStepper();""",
           """    while (wait - HAL::timeInMilliseconds() < 100000L) {
        Commands::checkForPeriodicalActions(Paused);
        if (Printer::blockAll || Printer::printJob.commandsBlocked())
            return;
    }
    const uint8_t requested=(com->hasX()?1:0) | (com->hasY()?2:0) | (com->hasZ()?4:0);
    while (!Printer::blockAll && !Printer::printJob.commandsBlocked()
           && !StepExecutor::ensureMotorsReady(requested)) {
        Commands::checkForPeriodicalActions(Paused);
    }""")
    p="StepExecutor.cpp";s=get(p)
    start=s.index("inline void StepExecutor::enableSteppers(PrintLine& move)")
    end=s.index("inline void StepExecutor::adjustDirections",start)
    changes[p]=s[:start]+"""MotorWake motorWake;

bool StepExecutor::ensureMotorsReady(uint8_t requested) {
    if (Printer::blockAll)
        return false;
    if (motorWake.readyFor(requested))
        return true;

    InterruptProtectedBlock wakeLock;
    if (Printer::blockAll)
        return false;
    const uint8_t newly = motorWake.newChannels(requested);
    if (newly) {
        Printer::unmarkAllSteppersDisabled();
        if (newly & 1) Printer::assertXStepper();
        if (newly & 2) Printer::assertYStepper();
        if (newly & 4) Printer::assertZStepper();
        if (newly & 0x18) Extruder::assertEnable();
        // One deadline for all newly asserted channels, after final GPIO.
        motorWake.afterAsserted(newly, HAL::timeInMilliseconds());
        return false;
    }
    motorWake.service(HAL::timeInMilliseconds());
    return motorWake.readyFor(requested);
}

inline bool StepExecutor::enableSteppers(PrintLine& move) {
    const uint8_t requested = (move.isXMove() ? 1 : 0)
        | (move.isYMove() ? 2 : 0) | (move.isZMove() ? 4 : 0)
        | (move.isEMove() ? uint8_t(8U << Extruder::current->id) : 0);
    if (!ensureMotorsReady(requested))
        return false;
    if (move.isEMove()) {
        InterruptProtectedBlock lock;
        Printer::extrusionOutput.allow(ExtrusionOutput::Inhibit::MotorsDisabled);
    }
    return true;
}

"""+s[end:]
    edit(p,"        enableSteppers(*move); // Set Z direction etc.",
           """        if (!enableSteppers(*move)) {
            queue.releaseFront(); // still mutable, not an executing move
            return 2000;
        }""")
    edit(p,"""        direct.task = (direct.task == DIRECT_PREPARED_STOPPABLE ? DIRECT_RUNNING_STOPPABLE : DIRECT_RUNNING);

        enableSteppers(direct);""",
           """        if (!enableSteppers(direct))
            return 2000; // keep PREPARED; never publish an executing ramp
        direct.task = (direct.task == DIRECT_PREPARED_STOPPABLE ? DIRECT_RUNNING_STOPPABLE : DIRECT_RUNNING);
        // Same Timer1 ownership as queued activation. PWM release in RUNNING
        // may shorten only remaining steps, not the phase/profile being set.
        HAL::allowInterrupts();""")
    # Standalone compensation must not generate a logical step on a sleeping
    # driver, nor start a stale wake on an unhomed bed. A valid homed pending
    # correction can request the Z wake before needCmpWait blocks the queue.
    edit(p,"    const int32_t cmpDiff = Printer::zMotion.stepCorrection();",
           """    if (!motorWake.readyFor(4)) {
        if (!Printer::isAxisHomed(Z_AXIS) || !Printer::zMotion.correctionCanMove()
            || !ensureMotorsReady(4))
            return;
    }
    const int32_t cmpDiff = Printer::zMotion.stepCorrection();""")
    edit(p,"    return Printer::zMotion.correctionDelaysQueue(Printer::doHeatBedZCompensation);",
           """    if (!motorWake.readyFor(4) && !Printer::isAxisHomed(Z_AXIS))
        return false;
    return Printer::zMotion.correctionDelaysQueue(Printer::doHeatBedZCompensation);""")
    # These are foreground pulse writers, not queue activation. An explicit
    # false return must propagate before updating position; scan's existing
    # retreat/error classification must still be checked in final P0 replay.
    edit("ZMotion.cpp","bool ZMotion::ScanMove::step() {\n    const bool positive",
         "bool ZMotion::ScanMove::step() {\n    if (!motorWake.readyFor(4)) return false;\n    const bool positive")
    p="Homing.cpp";s=get(p)
    start=s.index("int8_t Printer::checkHome(int8_t axis)")
    end=s.index("} // checkHome",start)
    a=s[start:end]
    import re
    a,n=re.subn(r"(?m)^(\s*)(Printer::stepAxisStep\(axis(?:,[^;]*)?\);)$",
                r"\1if (blockAll || !motorWake.readyFor(uint8_t(1U << axis))) return -1;\n\1\2",a)
    assert n==4,n
    changes[p]=s[:start]+a+s[end:]
    # A purely physical reset must invalidate the state; boot constructs it
    # empty. Driver-reset sequencing is an independent #107/P0 issue.
    # No changes to the reset or physical settling policy in this candidate.

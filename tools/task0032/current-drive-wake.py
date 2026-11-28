#!/usr/bin/env python3
"""Disposable nonblocking motor wake on the P0 guarded/scan Drive.

Retain the historical conservative settle interval, remove duplicate
enabled flags, and never wake a scan/retreat motor. Normal prepared work
is not marked running until ready. No new motion or timer owner.
"""
from pathlib import Path
import argparse
import hashlib
import json
import re
import subprocess

HERE=Path(__file__).resolve().parent
P0="95415cff552517a40a844cc1935f588baf9a5069"
RESEARCH=HERE.parents[1]/"research/task0032-step-output/motor-wake.h"

def once(s,a,b):
    assert s.count(a)==1,(a,s.count(a))
    return s.replace(a,b)

def region(s,a,b):
    i=s.index(a)
    return s[i:s.index(b,i)]

def candidate(fw):
    files={}
    def get(n): return files.get(n,(fw/n).read_text())
    def edit(n,a,b): files[n]=once(get(n),a,b)
    # P0 has no more M99 or independent foreground STEP engine. Do not
    # silently transplant the historical Homing/ScanMove substitutions.
    calls=re.compile(r"\b(?:enable[XYZ]Stepper|Extruder::enable|enabledStepper)\b")
    found={p.name for p in fw.glob("*") if p.suffix in (".h",".cpp",".ino") and calls.search(p.read_text())}
    assert found=={"Printer.h","Printer.cpp","StepExecutor.cpp"},found
    assert not (fw/"MotorWake.h").exists()
    files["MotorWake.h"]=once(RESEARCH.read_text(),"#ifdef __AVR__\nstatic_assert",
        "extern MotorWake motorWake;\n#ifdef __AVR__\nstatic_assert")
    edit("Printer.h",'#include "HAL.h"','#include "HAL.h"\n#include "MotorWake.h"')
    edit("Extruder.h",'#include "HAL.h"','#include "HAL.h"\n#include "MotorWake.h"')
    edit("Printer.h","    static volatile char enabledStepper[3];\n","")
    edit("Printer.cpp","volatile char Printer::enabledStepper[3] = { 0, 0, 0 };\n","")
    for axis,bit in zip("XYZ",(1,2,4)):
        old=f"""#if ({axis}_ENABLE_PIN > -1)
        WRITE({axis}_ENABLE_PIN, !{axis}_ENABLE_ON);
#endif // ({axis}_ENABLE_PIN > -1)

        Printer::enabledStepper[{axis}_AXIS] = 0;"""
        new=f"""        {{
            InterruptProtectedBlock wakeLock;
            motorWake.beforeDisable({bit});
#if ({axis}_ENABLE_PIN > -1)
            WRITE({axis}_ENABLE_PIN, !{axis}_ENABLE_ON);
#endif // ({axis}_ENABLE_PIN > -1)
        }}"""
        edit("Printer.h",old,new)
        edit("Printer.h",f"void enable{axis}Stepper() {{\n        unmarkAllSteppersDisabled();",
             f"void assert{axis}Stepper() {{")
        edit("Printer.h",f"""
        if (!Printer::enabledStepper[{axis}_AXIS]) {{
            Printer::enabledStepper[{axis}_AXIS] = 1;
            HAL::delayMilliseconds(stepperEnableDelayMs);
        }}""","")
        edit("Printer.h",f"}}  // enable{axis}Stepper",f"}}  // assert{axis}Stepper")
    edit("Printer.h","static bool zStepperEnabled() { return enabledStepper[Z_AXIS] != 0; }",
         "static bool zStepperEnabled() { return motorWake.readyFor(4); }")
    edit("Extruder.h","    char enabled;\n","")
    edit("Extruder.h","static inline void enable() {","static inline void assertEnable() {")
    edit("Extruder.h","""
        if (!Extruder::current->enabled) {
            Extruder::current->enabled = 1;
            HAL::delayMilliseconds(stepperEnableDelayMs);
        }
""","")
    edit("Extruder.h","} // enable","} // assertEnable")
    edit("Extruder.cpp","    Extruder::current = &extruder[0];",
         """    Extruder::current = &extruder[0];
    Printer::extrusionOutput.inhibit(ExtrusionOutput::Inhibit::MotorsDisabled);""")
    # Old tool selection reset a software flag without lowering SLEEPn.
    # Retain readiness of the old physical channel. Do not admit Advance
    # for a new sleeping tool until the next real E move completes wake.
    edit("Extruder.cpp","""        current->enabled = 0;
        current = &extruder[extruderId];""",
         """        current = &extruder[extruderId];
        if (!motorWake.readyFor(uint8_t(8U << current->id)))
            Printer::extrusionOutput.inhibit(ExtrusionOutput::Inhibit::MotorsDisabled);""")
    edit("Extruder.cpp","        const int16_t discarded = Printer::extrusionOutput.inhibit(ExtrusionOutput::Inhibit::MotorsDisabled);",
         """        motorWake.beforeDisable(0x18);
        const int16_t discarded = Printer::extrusionOutput.inhibit(ExtrusionOutput::Inhibit::MotorsDisabled);""")
    edit("Extruder.cpp","            tool.enabled = 0;\n","")
    for tool in (0,1):
        edit("Extruder.cpp",f"      EXT{tool}_EXTRUDER_COOLER_SPEED,\n      0,\n      0",
             f"      EXT{tool}_EXTRUDER_COOLER_SPEED,\n      0")
    # Boot-only reset is the only DRV_RESET1 writer; invalidate eligibility
    # even if a future reset path is added (new call sites need review).
    edit("Drv8711.cpp","void drv8711Init() {\n    WRITE(DRV_RESET1, LOW);",
         "void drv8711Init() {\n    motorWake.reset(); // boot before motion timer setup\n    WRITE(DRV_RESET1, LOW);")
    edit("StepExecutor.h","    static void enableSteppers(PrintLine& move);",
         "    static bool ensureMotorsReady(uint8_t requested);\n    static bool enableSteppers(PrintLine& move);")
    p="StepExecutor.cpp"
    old=region(get(p),"inline void StepExecutor::enableSteppers(PrintLine& move)","inline void StepExecutor::adjustDirections")
    edit(p,old,"""MotorWake motorWake;
static_assert(MotorWake::settleTicks >= stepperEnableDelayMs + 2, "retain physical wake margin");

bool StepExecutor::ensureMotorsReady(uint8_t requested) {
    // All readiness and enable transitions use the same short IRQ
    // transaction. No delay or nested service while the lock is held.
    InterruptProtectedBlock wakeLock;
    if (Printer::blockAll)
        return false;
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if (Printer::printJob.emergencyLocked())
        return false;
#endif
    if (motorWake.readyFor(requested))
        return true;
    const uint8_t newly = motorWake.newChannels(requested);
    if (newly) {
        Printer::unmarkAllSteppersDisabled();
        if (newly & 1) Printer::assertXStepper();
        if (newly & 2) Printer::assertYStepper();
        if (newly & 4) Printer::assertZStepper();
        if (newly & 0x18) Extruder::assertEnable();
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
        if (Printer::blockAll)
            return false;
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        if (Printer::printJob.emergencyLocked())
            return false;
#endif
        Printer::extrusionOutput.allow(ExtrusionOutput::Inhibit::MotorsDisabled);
    }
    return true;
}

""")
    edit(p,"        enableSteppers(*move); // Set Z direction etc.",
         """        if (!enableSteppers(*move)) {
            HAL::forbidInterrupts();
            queue.releaseFront(); // still not executing; lookahead may replan
            return 2000;
        }""")
    # Retain P0's exact guarded clock/profile, scan flags, and no-wake rule.
    # For ordinary jogs keep PREPARED until ready; nested release can refit
    # a not-yet-executing stop tail rather than race a half-started clock.
    old=region(get(p),"        direct.task = (direct.task == DIRECT_PREPARED_STOPPABLE","        directionOwner = DirectionOwner::Direct;")
    assert "FLAG_SCAN_PATH" in old and "enableSteppers(direct);" in old
    edit(p,old,"""        if (direct.flags & FLAG_SCAN_PATH) {
            // Scan/retreat may never wake a motor. A pending ordinary wake
            // is not a settled driver, even though its pin is already high.
            if (!Printer::zStepperEnabled()) {
                finishMove(direct, false);
                return 1000;
            }
            ZMotion::setDirection(direct.isZPositiveMove());
        } else {
            if (!enableSteppers(direct))
                return 2000;
            adjustDirections(direct);
        }
        direct.task = (direct.task == DIRECT_PREPARED_STOPPABLE ? DIRECT_RUNNING_STOPPABLE : DIRECT_RUNNING);
""")
    # Keep the existing cadence; only readiness is changed here. A valid
    # homed correction can request central wake; never wake during scan,
    # terminal fault or invalid travel. Unified-Z timing is separate.
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
    edit("Repetier.vcxproj",'    <ClInclude Include="StepExecutor.h" />',
         '    <ClInclude Include="MotorWake.h" />\n    <ClInclude Include="StepExecutor.h" />')
    edit("Repetier.vcxproj.filters",'    <ClInclude Include="StepExecutor.h">',
         '    <ClInclude Include="MotorWake.h">\n      <Filter>Header Files</Filter>\n    </ClInclude>\n    <ClInclude Include="StepExecutor.h">')
    # No stale physical-readiness mirror, delayed enable or hidden consumer.
    for p in fw.glob("*"):
        if p.suffix in (".cpp",".h",".ino"):
            s=files.get(p.name,p.read_text())
            assert not calls.search(s),p
            assert "delayMilliseconds(stepperEnableDelayMs)" not in s,p
    return files

def git(source,*args):
    return subprocess.check_output(["git","-C",str(source),*args],text=True).strip()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",type=Path,required=True)
    parser.add_argument("--out",type=Path,required=True)
    args=parser.parse_args()
    source=args.source.resolve();out=args.out.resolve()
    assert source!=out and source not in out.parents
    assert git(source,"rev-parse","HEAD")==P0
    assert not git(source,"status","--porcelain")
    assert subprocess.run(["git","-C",str(source),"symbolic-ref","-q","HEAD"],capture_output=True).returncode==1
    out.mkdir(parents=True,exist_ok=True)
    fw=source/"Repetier"
    files=candidate(fw)
    for name,s in files.items():(fw/name).write_text(s)
    subprocess.run(["git","-C",str(source),"add","-N","Repetier/MotorWake.h"],check=True)
    subprocess.run(["git","-C",str(source),"diff","--check"],check=True)
    patch=subprocess.check_output(["git","-C",str(source),"diff","--binary"])
    (out/"wake.diff").write_bytes(patch)
    record={"p0":P0,"changed":sorted(files),"patch_sha256":hashlib.sha256(patch).hexdigest(),
            "source_sha256":{n:hashlib.sha256((fw/n).read_bytes()).hexdigest() for n in files},
            "qualification":"disposable candidate, targeted wake/guard replay and AVR link required"}
    (out/"wake.json").write_text(json.dumps(record,indent=2)+"\n")
    print(json.dumps(record,indent=2))

if __name__=="__main__":
    main()

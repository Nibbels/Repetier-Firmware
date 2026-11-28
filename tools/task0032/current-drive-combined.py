#!/usr/bin/env python3
"""Compose independently qualified core and wake patches on a newer P0 pin.

Reject context drift instead of silently overriding P0 code. Adapt the two
old blocking-wake fixtures to the new contract, retaining the actual-source
pulse/guard and startup assertions. Other canonical replays remain unchanged.
Only disposable detached checkout; never update a production branch.
"""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess

P0 = "e495fd7cb3e3071e984d7c5c69f2d0e94d04ceb1"

def git(source, *args):
    return subprocess.check_output(["git", "-C", str(source), *args], text=True).strip()

def edit(path, old, new):
    s=path.read_text()
    assert s.count(old)==1,(str(path),old,s.count(old))
    path.write_text(s.replace(old,new))

def adapt_homing(source):
    # G28 gained these checks after the independently tested wake pin.
    # Require settled physical readiness, not a second software mirror.
    t=source/"Repetier/Homing.cpp"
    s=t.read_text()
    for axis,mask in (("X",1),("Y",2)):
        old=f"enabledStepper[{axis}_AXIS]"
        assert s.count(old)==2,(axis,s.count(old))
        s=s.replace(old,f"motorWake.readyFor({mask})")
    t.write_text(s)
    for p in (source/"Repetier").glob("*"):
        if p.suffix in (".h",".cpp",".ino"):
            assert "enabledStepper" not in p.read_text(),p

def adapt_tests(source):
    # DRV reset trace: run the real readiness reset, preserve GPIO trace.
    t=source/"tests/drv8711-trace.py"
    edit(t,"static Trace actual;",'''static Trace actual;
#include "MotorWake.h"
MotorWake motorWake;''')
    edit(t,"    drv8711Init();",'''    motorWake.afterAsserted(0x1f,0);
    motorWake.service(27);
    assert(motorWake.readyFor(0x1f));
    drv8711Init();
    assert(motorWake.awakeMask()==0 && motorWake.readyMask()==0);''')
    # Advance handoff test deliberately models activation of the new E
    # channel at the point where it simulates the first real E move.
    t=source/"tests/pause-sense-offset.cpp"
    edit(t,'#include "pause-sense-offset-firmware.inc"','#include "MotorWake.h"\nMotorWake motorWake;\n#include "pause-sense-offset-firmware.inc"')
    edit(t,"    extruder[0].enabled = true;",'''    motorWake.reset();
    motorWake.afterAsserted(8,0);
    motorWake.service(27);''')
    edit(t,"    assert(!extruder[0].enabled && extruder[0].deliveredSteps == 2);",
        "    assert(motorWake.readyFor(8) && !motorWake.readyFor(16) && extruder[0].deliveredSteps == 2);")
    edit(t,"    advance.addSteps(-2);\n    tick();\n    tick();\n    tick();",'''    assert(advance.blocked());
    motorWake.afterAsserted(16,27);
    motorWake.service(54);
    assert(motorWake.readyFor(16));
    advance.allow(ExtrusionOutput::Inhibit::MotorsDisabled);
    advance.addSteps(-2);
    tick();
    tick();
    tick();''')
    # The isolated Z ledger suite intentionally starts with a settled
    # motor. The independent wake fixture covers pending motor startup.
    t=source/"tests/z-motion.cpp"
    edit(t,'#include "ZMotion.h"','#include "MotorWake.h"\nMotorWake motorWake;\n#include "ZMotion.h"')
    edit(t,"bool needsCMPwait();","bool isAxisHomed(uint8_t axis) { assert(axis==Z_AXIS); return zMotion.isHomed(); }\nbool needsCMPwait();")
    edit(t,"    static void stepSlowedZCompensation();","    static bool ensureMotorsReady(uint8_t mask) { assert(motorWake.readyFor(mask)); return true; }\n    static void stepSlowedZCompensation();")
    edit(t,"    zMotion.configureTravel(100.0f, 10.0f);","    motorWake.reset();\n    motorWake.afterAsserted(4,0);\n    motorWake.service(27);\n    zMotion.configureTravel(100.0f, 10.0f);")

    t=source/"tests/task0034-guarded-drive.cpp"
    edit(t,"""void StepExecutor::enableSteppers(PrintLine&) {
    assert(!scanRunning);
    if (Printer::blockAll) return;
}""","""bool StepExecutor::enableSteppers(PrintLine&) {
    // This pulse/ledger fixture supplies an already-settled normal motor.
    // The independent wake fixture covers GPIO and readiness timing.
    assert(!scanRunning);
    return !Printer::blockAll && !Printer::printJob.thermal;
}""")
    t=source/"tests/task0034-direct-enable.py"
    edit(t,'"""Direct-move driver startup must never wait on millis with I=0."""',
        '"""Prepared direct motion yields while waiting for nonblocking driver readiness."""')
    edit(t,"struct PrintLine {","""struct Printer {
    static constexpr bool blockAll=false;
    struct Job {bool commandsBlocked() const {return false;}};
    static Job printJob;
    static bool isAxisHomed(uint8_t){return true;}
    static bool zStepperEnabled(){return false;}
};
Printer::Job Printer::printJob;
struct PrintLine {""")
    edit(t,"""    static void enableSteppers(PrintLine& d) {
        assert(irq && timer1Masked);
        assert(d.task==DIRECT_RUNNING || d.task==DIRECT_RUNNING_STOPPABLE);
        ++enabled;
        // An independent ISR advances the timebase and can observe key release.
        for (int i=0;i<25;++i) { assert(irq); ++ticks; }
        if (d.task==DIRECT_RUNNING_STOPPABLE) d.stepsRemaining=32;
    }""","""    static bool enableSteppers(PrintLine& d) {
        assert(!irq && timer1Masked);
        assert(d.task==DIRECT_PREPARED || d.task==DIRECT_PREPARED_STOPPABLE);
        ++enabled;
        // Time advances between ISR invocations, never in the readiness call.
        return ticks>=27;
    }
    static void finishMove(PrintLine&,bool){assert(false);}""")
    edit(t,"int main() {","""static void settle(int task) {
    for(ticks=0;ticks<27;++ticks) {
        assert(StepExecutor::performDirectMove()==2000);
        assert(!irq && timer1Masked && !directions && !advance && !pulses);
        assert(PrintLine::direct.task==task);
        if(task==DIRECT_PREPARED_STOPPABLE && ticks==5)
            PrintLine::direct.stepsRemaining=32;
    }
}
int main() {""")
    edit(t,"        assert(StepExecutor::performDirectMove()==5000);",
        "        settle(task);\n        assert(StepExecutor::performDirectMove()==5000);")
    edit(t,"enabled==1 && directions==1 && advance==1 && !pulses && ticks==25",
        "enabled==28 && directions==1 && advance==1 && !pulses && ticks==27")
    # Same assertion appears twice.
    s=t.read_text()
    assert s.count("enabled==1 && pulses==1")==3
    t.write_text(s.replace("enabled==1 && pulses==1","enabled==28 && pulses==1"))
    edit(t,"        assert(StepExecutor::performDirectMove()==(16000000UL/42)/2);",
        "        settle(DIRECT_PREPARED);\n        assert(StepExecutor::performDirectMove()==long((16000000UL/42)/2));")
    edit(t,"assert(StepExecutor::stepsPerTimerCall==1 && !pulses && ticks==25);",
        "assert(StepExecutor::stepsPerTimerCall==1 && !pulses && ticks==27);")
    edit(t,"Task0034 direct driver enable: PASS (timebase IRQ admission and stoppable tail)",
        "Task0032 direct driver readiness: PASS (yield, prepared release and guarded single-step startup)")

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source",type=Path,required=True)
    p.add_argument("--core",type=Path,required=True)
    p.add_argument("--wake",type=Path,required=True)
    p.add_argument("--out",type=Path,required=True)
    a=p.parse_args()
    source=a.source.resolve();out=a.out.resolve()
    assert source!=out and source not in out.parents
    assert git(source,"rev-parse","HEAD")==P0
    assert not git(source,"status","--porcelain")
    assert subprocess.run(["git","-C",str(source),"symbolic-ref","-q","HEAD"],capture_output=True).returncode==1
    out.mkdir(parents=True,exist_ok=True)
    patches={}
    for name,p in (("core",a.core.resolve()),("wake",a.wake.resolve())):
        raw=p.read_bytes()
        patches[name]=hashlib.sha256(raw).hexdigest()
        subprocess.run(["git","-C",str(source),"apply","--check",str(p)],check=True)
        subprocess.run(["git","-C",str(source),"apply",str(p)],check=True)
    adapt_homing(source)
    adapt_tests(source)
    subprocess.run(["git","-C",str(source),"add","-N","Repetier/MotorWake.h"],check=True)
    subprocess.run(["git","-C",str(source),"diff","--check"],check=True)
    patch=subprocess.check_output(["git","-C",str(source),"diff","--binary"])
    (out/"combined.diff").write_bytes(patch)
    result={"p0":P0,"patch_inputs":patches,"combined_sha256":hashlib.sha256(patch).hexdigest(),
        "changed":git(source,"diff","--name-only").splitlines(),
        "scope":"disposable composition; full canonical replay/link required"}
    (out/"combined.json").write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps(result,indent=2))

if __name__=="__main__":
    main()

#!/usr/bin/env python3
"""Replay real filament admission after reentrant periodic motion waits."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
src=(root/"Repetier/GCodeThermal.cpp").read_text()
body=src[src.index("static bool prepareFilamentProgram()"):src.index("// Name the admitted frames explicitly")]
harness=r'''
#include <cassert>
#include <cstdint>
#include <initializer_list>
#define PSTR(x) x
#include "PrintJob.h"

static bool scan=false, calibrated=true, settled=true, inhibited=false, paused=false;
struct InterruptProtectedBlock { InterruptProtectedBlock() {} };
namespace Printer {
    PrintJob printJob;
    bool heatingInhibited(){return inhibited;}
    struct { bool isPaused(){return paused;} } pause;
}
static unsigned waits=0;
static void (*duringWait)()=nullptr;
static bool isAnyScanRunning(){return scan;}
namespace Com { void printFLN(const char*){} }
namespace Extruder {
    struct Controller {
        struct { bool pidCalibrationValid(){return calibrated;} } tempControl;
    } extruder;
    Controller* current=&extruder;
}
namespace Commands {
    enum class MotionWait { AllOutputs };
    enum class PauseWait { Cancel };
    bool waitForMotion(MotionWait,PauseWait){
        ++waits;
        if(duringWait) duringWait();
        return settled;
    }
}
''' + body + r'''
static void reset(){
    Printer::printJob=PrintJob{};
    scan=inhibited=paused=false;calibrated=settled=true;waits=0;duringWait=nullptr;
}
int main(){
    using namespace Printer;
    reset();
    assert(prepareFilamentProgram() && waits==1 && printJob.isBusy());
    assert(!prepareFilamentProgram() && waits==1);
    printJob.finishFilamentService();
    assert(printJob.isIdle());
    reset();printJob.admitPrintMotion();
    assert(!prepareFilamentProgram() && waits==0 && printJob.isPrinting());
    reset();assert(printJob.beginFilamentService());
    assert(printJob.beginStop(1));
    assert(!prepareFilamentProgram() && waits==0 && printJob.commandsBlocked());
    reset();scan=true;
    assert(!prepareFilamentProgram() && waits==0 && printJob.isIdle());
    reset();calibrated=false;
    assert(!prepareFilamentProgram() && waits==0);
    reset();settled=false;
    assert(!prepareFilamentProgram() && waits==1 && printJob.isIdle());
    reset();inhibited=true;
    assert(!prepareFilamentProgram() && waits==0 && printJob.isIdle());
    reset();duringWait=[] {inhibited=true;};
    assert(!prepareFilamentProgram() && waits==1 && printJob.isIdle());
    reset();duringWait=[] {calibrated=false;};
    assert(!prepareFilamentProgram() && waits==1 && printJob.isIdle());
    reset();duringWait=[] {paused=true;};
    assert(!prepareFilamentProgram() && waits==1 && printJob.isIdle());
    reset();duringWait=[] {scan=true;};
    assert(!prepareFilamentProgram() && waits==1 && printJob.isIdle());
    reset();duringWait=[] {printJob.admitPrintMotion();};
    assert(!prepareFilamentProgram() && waits==1 && printJob.isPrinting());
    reset();duringWait=[] {assert(printJob.beginFilamentService());};
    assert(!prepareFilamentProgram() && waits==1 && printJob.isBusy());
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    reset();duringWait=[] {assert(printJob.beginThermalEmergency());};
    assert(!prepareFilamentProgram() && waits==1 && printJob.emergencyLocked());
    assert(!prepareFilamentProgram() && waits==1);
#endif
}
'''
with tempfile.TemporaryDirectory() as d:
    cpp=Path(d)/"test.cpp";exe=Path(d)/"test"
    cpp.write_text(harness)
    for flags in ([],["-DRF_THERMAL_EMERGENCY_EXPERIMENT=1"]):
        subprocess.run(["g++","-std=c++11","-Wall","-Wextra","-Werror",
                        *flags,"-I",str(root/"Repetier"),str(cpp),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
print("Task0034 filament ownership: idle, busy, scan, failed wait, reentry and thermal lock PASS")

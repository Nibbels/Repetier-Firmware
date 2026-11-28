#!/usr/bin/env python3
"""Replay real filament admission after reentrant periodic motion waits."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
src=(root/"Repetier/GCodeThermal.cpp").read_text()
body=src[src.index("static bool prepareFilamentProgram()"):src.index("// Name the admitted frames explicitly")]
body+=src[src.index("void loadFilamentWithParameters("):src.index("static __attribute__((noinline)) void unloadFilamentPrepared(")]
body+=src[src.index("void unloadFilamentWithParameters("):src.index("#ifdef X35_ADC_CHANNEL",src.index("void unloadFilamentWithParameters("))]
harness=r'''
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <initializer_list>
#define constrain(x,lo,hi) ((x)<(lo)?(lo):((x)>(hi)?(hi):(x)))
#define EMERGENCY_PAUSE_DIGITS_MAX 11000
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
struct GCode {
    int32_t P=0;
    float F=0;
    bool p=false,f=false;
    bool hasP() const {return p;}
    bool hasF() const {return f;}
};
static unsigned loads=0,unloads=0;
static int16_t observedP=0;
static uint8_t observedF=0;
static void loadFilamentPrepared(int16_t p,uint8_t f){
    assert(Printer::printJob.isBusy() && !Printer::printJob.isPrinting());
    ++loads;observedP=p;observedF=f;
    Printer::printJob.finishFilamentService();
}
static void unloadFilamentPrepared(int16_t p){
    assert(Printer::printJob.isBusy() && !Printer::printJob.isPrinting());
    ++unloads;observedP=p;
    Printer::printJob.finishFilamentService();
}
''' + body + r'''
static void reset(){
    Printer::printJob=PrintJob{};
    scan=inhibited=paused=false;calibrated=settled=true;waits=0;duringWait=nullptr;
    loads=unloads=0;observedP=0;observedF=0;
}
int main(){
    using namespace Printer;
    reset();
    GCode c;
    loadFilament(&c);assert(loads==1 && observedP==2000 && observedF==1 && printJob.isIdle());
    unloadFilament(&c);assert(unloads==1 && observedP==4000 && printJob.isIdle());
    c.p=c.f=true;c.P=3500;c.F=3;
    loadFilament(&c);assert(loads==2 && observedP==3500 && observedF==3);
    loadFilamentWithParameters(3500,3);assert(loads==3 && observedP==3500 && observedF==3);
    c.P=4500;
    unloadFilament(&c);assert(unloads==2 && observedP==4500);
    unloadFilamentWithParameters(4500);assert(unloads==3 && observedP==4500);
    c.P=-30000;c.F=100;
    loadFilament(&c);assert(loads==4 && observedP==20000 && observedF==50);
    unloadFilament(&c);assert(unloads==4 && observedP==EMERGENCY_PAUSE_DIGITS_MAX);
    printJob.admitPrintMotion();
    loadFilamentWithParameters(3500,3);unloadFilamentWithParameters(4500);
    assert(loads==4 && unloads==4 && printJob.isPrinting());
    reset();duringWait=[] {inhibited=true;};
    loadFilamentWithParameters(3500,3);assert(!loads && printJob.isIdle());
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
print("Task0034 filament ownership and shared typed parameters: defaults, clamps, idle, busy, scan, failed wait, reentry and thermal lock PASS")

#!/usr/bin/env python3
"""Replay mechanical stop ordering using actual RF.cpp orchestration.

Mocks hardware/thermal service; separately checks the Timer1 entry fence.
Not a physical retreat or nested IRQ timing proof.
"""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
src=(root/"Repetier/RF.cpp").read_text()
body=src[src.index("void doEmergencyStop("):src.index("} // doEmergencyStop")+1]
hal=(root/"Repetier/HAL.cpp").read_text()
vector=hal[hal.index("ISR(TIMER1_COMPA_vect)"):hal.index("} // ISR(TIMER1_COMPA_vect)")]
assert vector.index("if (Printer::blockAll && !PrintLine::mechanicalRetreatActive())") < vector.index("uint8_t doExit;")
assert vector.index("if(Printer::printJob.emergencyLocked())") < vector.index("if (Printer::blockAll && !PrintLine::mechanicalRetreatActive())")
harness=r'''
#include <cassert>
#include <cstdint>
#include <initializer_list>
#define RF_THERMAL_EMERGENCY_EXPERIMENT 1
#define PSTR(x) x
enum {X_AXIS,Y_AXIS,Z_AXIS,E_AXIS};
enum class StopReason {ZMin,ZBlock};
static bool irq,queued,direct,paused,idle,unlocked,inhibited,thermal,activePrint,injectThermal;
static unsigned targets,retreats,shutdowns,stopCalls;
static char ui_text_emergency_stop[]="stop";
struct InterruptProtectedBlock{
    bool previous=irq;
    InterruptProtectedBlock(){irq=false;}
    ~InterruptProtectedBlock(){irq=previous;}
};
namespace Printer {
bool blockAll;
struct {bool emergencyLocked()const{return thermal;}} printJob;
void lockMovement(){assert(!irq);unlocked=false;}
struct {void cancel(){assert(!irq);paused=false;}} pause;
struct {void cancel(){assert(!irq);idle=false;}} idleStatus;
struct ExtrusionOutput {
    enum class Inhibit {Emergency};
    void inhibit(Inhibit){assert(!irq);inhibited=true;}
} extrusionOutput;
void stopDirectAxis(uint8_t axis){assert(!irq);targets &= ~(1u<<axis);}
float stepsPerMm(uint8_t axis){assert(axis==Z_AXIS);return 100;}
void stopPrint(){assert(blockAll && !queued && !direct);++stopCalls;if(activePrint)activePrint=false;}
void switchEverythingOff(){assert(!thermal && !queued && !direct && !targets);++shutdowns;}
}
using ExtrusionOutput=Printer::ExtrusionOutput;
struct PrintLine {
    static void abortDirectMoveForEmergency(){assert(!irq);direct=false;}
    static void discardQueuedMoves(){assert(!irq);queued=false;}
};
void showError(void*){assert(Printer::blockAll && !queued && !direct && !targets);}
namespace Com {void printF(const char*){}void printFLN(const char*){}}
namespace Calibration {
bool moveZ(int32_t steps,bool emergency){
    assert(emergency && steps==500 && Printer::blockAll && !queued && !direct && !paused && !idle && !unlocked && inhibited && !targets);
    if(thermal)return false;
    ++retreats;
    if(injectThermal)thermal=true;
    return !thermal;
}
}
''' + body + r'''
int main(){
    for(bool enabled:{false,true})
        for(bool printing:{false,true})
            for(bool priorThermal:{false,true})
                for(bool duringThermal:{false,true})
                    for(auto reason:{StopReason::ZMin,StopReason::ZBlock}){
                        irq=enabled; queued=direct=paused=idle=unlocked=true;
                        inhibited=false;targets=15;retreats=shutdowns=stopCalls=0;
                        activePrint=printing;thermal=priorThermal;injectThermal=duringThermal;
                        Printer::blockAll=false;
                        doEmergencyStop(reason);
                        assert(Printer::blockAll && !queued && !direct && !paused && !idle && !unlocked && inhibited && !targets);
                        assert(retreats==unsigned(!priorThermal));
                        assert(shutdowns==unsigned(!priorThermal&&!duringThermal));
                        assert(irq==enabled && stopCalls==1);
                        doEmergencyStop(reason);
                        assert(irq==enabled && stopCalls==1 && retreats==unsigned(!priorThermal));
                    }
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"stop.cpp";exe=Path(d)/"stop"
    p.write_text(harness)
    subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror",str(p),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print("Mechanical emergency: 32 phase/thermal/IRQ cases PASS; Timer1 fence before dispatch")

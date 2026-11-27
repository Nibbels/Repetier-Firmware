#!/usr/bin/env python3
"""Actual in-flight waits/scan aborts must unwind to the emergency scheduler."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
def extract(file,start,end):
    s=(root/"Repetier"/file).read_text()
    a=s.index(start)
    return s[a:s.index(end,a)]

motion=extract("GCodeMotion.cpp","void dwell(","void homeAxes(")
bed=extract("HeatBedScan.cpp","void HeatBedScan::abort()","void HeatBedScan::discard()")
offset=extract("ZOffsetScan.cpp","void ZOffsetScan::abort(bool reloadMatrix) {","} // namespace")
keys=extract("UiActions.cpp","        while (Printer::checkAbortKeys()) {","        bool unmount =")
prelude=r"""
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#define PSTR(x) x
#define UI_TEXT_DO_MHIER_BED_SCAN "ZOS"
using millis_t=uint32_t;
enum {Processing,Paused,X_AXIS,Y_AXIS,Z_AXIS};
static bool locked,retreatOnlyFlag;
static uint32_t now;
static unsigned visits,injectAt,disabled,enabled,discards,finished,reloads,scheduled,continued;
static unsigned moveCalls,homeCalls,queueCalls;
static const char ui_text_heat_bed_scan[]="HBS";
namespace Com {
const char tscanHeatBed[]="HBS";
template<class... Args>void printFLN(Args...){assert(!locked);}
template<class... Args>void printF(Args...){assert(!locked);}
}
namespace HAL {uint32_t timeInMilliseconds(){return now;}}
namespace Commands {
enum class MotionWait{QueueAndZ};
enum class PauseWait{Resume};
bool waitForMotion(MotionWait,PauseWait){return !locked;}
void checkForPeriodicalActions(int=Processing){
    now+=1000;
    if(++visits==injectAt)locked=true;
}
}
namespace Printer {
struct Job{bool emergencyLocked()const{return locked;}}printJob;
struct Idle{void schedule(uint32_t){assert(!locked);++scheduled;}}idleStatus;
bool debugInfo(){return true;}
bool checkAbortKeys(){return visits<3;}
void disableXStepper(){assert(!locked);++disabled;}
void disableYStepper(){assert(!locked);++disabled;}
void disableZStepper(){assert(!locked);++disabled;}
void enableXStepper(){assert(!locked);++enabled;}
void enableYStepper(){assert(!locked);++enabled;}
void enableZStepper(){assert(!locked);++enabled;}
void disableAllSteppersNow(){assert(!locked);++disabled;}
int stepsPerMm(int){return 128;}
float axisHomingFeedrate(int){return 5;}
void homeAxis(bool,bool,bool){
    assert(!locked);++homeCalls;
    Commands::checkForPeriodicalActions();
}
bool queueRelativeMMCoordinates(int,int,int,int,float,bool,bool){
    assert(!locked);++queueCalls;
    Commands::checkForPeriodicalActions();
    return !locked;
}
}
namespace Extruder{
void setHeatedBedTemperature(int){}
void setTemperatureForAllExtruders(int){}
}
struct ScanSession{
    int takeAbort(){return 1;}
    static bool retreatOnly(int){return retreatOnlyFlag;}
};
namespace Calibration{
ScanSession scanSession;
template<class T> void showAbortScanReason(T,int){assert(!locked);}
bool moveZ(int){
    assert(!locked);++moveCalls;
    Commands::checkForPeriodicalActions();
    return !locked;
}
}
namespace BedMatrix{
bool reloadActive(){assert(!locked);++reloads;return true;}
void clearWorking(){assert(!locked);}
}
enum class Phase{Aborting};
struct HeatBedScan{
    Phase phase_;
    void abort();
    void finish(){assert(!locked);++finished;}
};
struct ZOffsetScan{
    Phase phase_;
    void abort(bool);
    void discard(){assert(!locked);++discards;}
};
struct GCode{
    uint32_t P=10000,S=10;
    bool hasP(){return true;}
    bool hasS(){return true;}
    bool hasX(){return true;}
    bool hasY(){return true;}
    bool hasZ(){return true;}
};
"""
main=r"""
static void reset(unsigned injection,bool retreat=false,uint32_t clock=0){
    locked=false;retreatOnlyFlag=retreat;now=clock;
    injectAt=injection;
    visits=disabled=enabled=discards=finished=reloads=scheduled=continued=0;
    moveCalls=homeCalls=queueCalls=0;
}
int main(){
    GCode c;
    HeatBedScan h;
    ZOffsetScan z;
    for(uint32_t clock:{0u,0xfffff000u}){
        reset(0,false,clock);dwell(&c);assert(visits==11);
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        for(unsigned n=1;n<=11;++n){
            reset(n,false,clock);dwell(&c);assert(locked&&visits==n);
        }
#endif
    }
    reset(0);heldButtonWait();assert(visits==3&&continued==1);
    reset(0);h.abort();assert(moveCalls==1&&homeCalls==1&&disabled==1&&finished==1&&reloads==1&&scheduled==1);
    reset(0,true);h.abort();assert(moveCalls==1&&homeCalls==0&&disabled==1&&finished==1);
    reset(0);z.abort(true);assert(queueCalls==1&&homeCalls==1&&disabled==1&&discards==1&&reloads==1&&scheduled==1);
    reset(0,true);z.abort(false);assert(moveCalls==1&&homeCalls==0&&disabled==1&&discards==1);
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    for(unsigned n=1;n<=3;++n){
        reset(n);heldButtonWait();assert(locked&&visits==n&&continued==0);
    }
    for(unsigned n=1;n<=2;++n){
        reset(n);h.abort();assert(locked&&visits==n&&!disabled&&!finished&&!reloads&&!scheduled);
        reset(n);z.abort(true);assert(locked&&visits==n&&!disabled&&!discards&&reloads==1&&!scheduled);
    }
    reset(1,true);h.abort();assert(locked&&!disabled&&!finished&&!scheduled);
    reset(1,true);z.abort(false);assert(locked&&!disabled&&!discards&&!scheduled);
    std::puts("Emergency unwind: 31 injected faults, no later motor enable/disable, script admission or normal scan cleanup");
#endif
    std::puts("Healthy dwell, held button and scan abort paths passed");
}
"""
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"unwind.cpp"
    exe=Path(d)/"unwind"
    p.write_text(prelude+"\n"+motion+"\n"+bed+"\n"+offset+
                 "\nvoid heldButtonWait(){\n"+keys+"\n++continued;\n}\n"+main)
    for flags in ([],["-DRF_THERMAL_EMERGENCY_EXPERIMENT=1"]):
        subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror",
                        *flags,str(p),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True)

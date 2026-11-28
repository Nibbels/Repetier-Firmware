#!/usr/bin/env python3
"""Replay G28's actual homing orchestration and reference publication.
The queue mock models settled switch observations, not the AVR trajectory.
The separate guarded-drive replay exercises physical pulse and limit paths.
"""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
src=(root/"Repetier/Homing.cpp").read_text()
axes=src[src.index("void Printer::homeXAxis()"):src.index("void Printer::homeDigits()")]
g28=src[src.index("__attribute__((noinline)) void Printer::homeAxis("):]
prelude=r"""
#include <cassert>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <initializer_list>
#define PSTR(x) x
#define UI_STATUS_UPD(x) ((void)0)
constexpr int X_AXIS=0,Y_AXIS=1,Z_AXIS=2;
constexpr float ENDSTOP_X_BACK_MOVE=1, ENDSTOP_Y_BACK_MOVE=1, ENDSTOP_Z_BACK_MOVE=1;
constexpr float Z_ENDSTOP_MAX_HYSTERESIS=0.1f;
constexpr float ENDSTOP_X_RETEST_REDUCTION_FACTOR=2, ENDSTOP_Y_RETEST_REDUCTION_FACTOR=2, ENDSTOP_Z_RETEST_REDUCTION_FACTOR=2;
enum Fault { None, NoSeekContact, StuckClosed, NoRetestContact, QueueReject,
             DriverLost, MechanicalStop, JobBlocked, OffsetShort };
static Fault fault;
static int faultAxis, moves[3], refs[3], offsetMoves, digits, reports;
static bool contacts[3], homed[3], masked, panel, drain, zSafe, scan, scanAbort, settings;
static int32_t motor[3], nominal[3];
struct InterruptProtectedBlock {
    bool old=masked;
    InterruptProtectedBlock(){masked=true;}
    void unprotect(){masked=old;}
    ~InterruptProtectedBlock(){unprotect();}
};
struct UIDisplay {
    struct ScopedActionExclusion {
        bool old=panel;
        explicit ScopedActionExclusion(UIDisplay&){panel=true;}
        ~ScopedActionExclusion(){panel=old;}
    };
};
static UIDisplay uid;
namespace HAL { uint32_t timeInMilliseconds(){return 0;} }
namespace Com { void printFLN(const char*){assert(!masked);} }
namespace EEPROM { bool settingsReady(){return settings;} }
namespace RMath { using std::min; using std::max; }
enum class ScanAbort { EndstopProtection };
namespace Calibration {
struct Session {
    bool dispatching(){return scan;}
    void requestAbort(ScanAbort){scanAbort=true;}
} scanSession;
}
struct Extruder { float offsetMM[2]; static Extruder* current; };
Extruder extruder[2]={{{0,0}},{{2,3}}};
Extruder* Extruder::current=&extruder[0];
namespace Printer {
bool blockAll, enabledStepper[3], movementLocked;
struct Job {bool blocked; bool commandsBlocked()const{return blocked;}} printJob;
struct { void recordActivity(uint32_t){assert(!masked);} } inactivity;
struct { void cancel(){} void schedule(uint32_t){} } idleStatus;
int32_t maxSoftEndstopSteps_[3]={10000,10000,10000};
float homingFeedrate_[3]={50,50,5},axisStepsPerMM_[3]={100,100,100},axisMMPerSteps_[3]={0.01f,0.01f,0.01f};
float stepsPerMm(int a){return axisStepsPerMM_[a];}
char anyHomeDir(uint8_t){return -1;}
void resetDirectAxis(int){}
bool isXMinEndstopHit(){return contacts[X_AXIS];}
bool isYMinEndstopHit(){return contacts[Y_AXIS];}
bool isZMinEndstopHit(){return contacts[Z_AXIS];}
bool zStepperEnabled(){return enabledStepper[Z_AXIS];}
bool isAxisHomed(int a){return homed[a];}
bool isZHomeSafe(){return zSafe;}
void setHomed(int x,int y,int z) {
    assert(!masked);
    int val[]={x,y,z};
    for(int a=0;a<3;++a)if(val[a]!=-1)
        homed[a]=val[a]&&!printJob.commandsBlocked();
}
void allowMovement(){movementLocked=false;}
void lockMovement(){movementLocked=true;}
void setAxis(int a,int32_t s){assert(masked);nominal[a]=s;++refs[a];}
void setXAxisSteps(int32_t s){setAxis(X_AXIS,s);}
void setYAxisSteps(int32_t s){setAxis(Y_AXIS,s);}
void setZAxisSteps(int32_t s){setAxis(Z_AXIS,s);}
struct { void setMotorSteps(int a,int32_t s){assert(masked);motor[a]=s;} } xyMotion;
struct {
    int32_t maxSteps(){return 10000;}
    uint16_t minOvertravel(){return 25;}
    void resetDirectPosition(){assert(masked);}
    void clearScanAndCorrection(){assert(masked);}
    void setMotorReference(int32_t s){assert(masked);motor[Z_AXIS]=s;}
    void setHomed(bool value){assert(masked);homed[Z_AXIS]=value;}
} zMotion;
struct { void resetFadeBounds(float){} void resetTracking(){} } layers;
struct { void clearSenseOffset(float){} } zOffset;
struct { void setThresholdDigits(int){} void resetWindow(){} } senseOffsetControl;
struct { void reset(){assert(masked);} } extrusionCompensation;
bool move(int axis,float distance,bool seek) {
    assert(panel&&!masked&&!homed[axis]);
    ++moves[axis];
    const Fault f=axis==faultAxis?fault:None;
    if(f==QueueReject)return false;
    if(seek && moves[axis]==1) {
        assert(distance<0);
        contacts[axis]=f!=NoSeekContact;
        motor[axis]=-123;  // arbitrary uncalibrated ledger, not the command distance
    } else if(distance>0) {
        contacts[axis]=f==StuckClosed;
        motor[axis]+=10;
    } else {
        assert(seek&&!contacts[axis]);
        contacts[axis]=f!=NoRetestContact;
        motor[axis]=-117;
        if(f==DriverLost)enabledStepper[axis]=false;
        if(f==MechanicalStop)blockAll=true;
        if(f==JobBlocked)printJob.blocked=true;
    }
    return true;
}
bool queueRelativeStepsCoordinates(long x,long y,long z,long e,float,bool wait,bool seek) {
    assert(wait&&seek&&!e);
    return x?move(X_AXIS,x,seek):y?move(Y_AXIS,y,seek):move(Z_AXIS,z,seek);
}
bool queueRelativeMMCoordinates(float x,float y,float z,float e,float,bool wait,bool seek=false) {
    assert(wait&&!e);
    return x?move(X_AXIS,x,seek):y?move(Y_AXIS,y,seek):move(Z_AXIS,z,seek);
}
void offsetRelativeStepsCoordinates(long x,long y,long z,long e) {
    assert(!z&&!e&&!masked);
    ++offsetMoves;
    const int axis=x?X_AXIS:Y_AXIS;
    motor[axis]+=x?x:y;
    if(axis==faultAxis&&fault==OffsetShort)--motor[axis];
}
void homeXAxis();void homeYAxis();void homeZAxis();void homeAxis(bool,bool,bool);
void homeDigits(){++digits;}
}
struct PrintLine { static int32_t guardedAxisPosition(uint8_t a){return motor[a];} };
namespace Commands {
enum class MotionWait { AllOutputs };
enum class PauseWait { Cancel };
bool waitForMotion(MotionWait,PauseWait){assert(panel&&!masked);return drain;}
void printCurrentPosition(){assert(!masked);++reports;}
}
"""
main=r"""
void reset(int axis,Fault f) {
    faultAxis=axis;fault=f;offsetMoves=digits=reports=0;
    for(int a=0;a<3;++a){
        moves[a]=refs[a]=0;contacts[a]=false;homed[a]=true;
        Printer::enabledStepper[a]=true;motor[a]=nominal[a]=100;
    }
    masked=panel=scan=scanAbort=false;drain=zSafe=settings=true;
    Printer::printJob.blocked=Printer::blockAll=false;Printer::movementLocked=true;
}
int main(){
    unsigned count=0;
    for(int a=0;a<3;++a) {
        reset(a,None);Printer::homeAxis(a==0,a==1,a==2);
        assert(homed[a]&&refs[a]==1&&nominal[a]==0&&contacts[a]);
        assert(!Printer::movementLocked&&!panel&&!masked&&reports==1);
        assert(moves[a]==(a==Z_AXIS?4:3));
        ++count;
        for(Fault f:{NoSeekContact,StuckClosed,NoRetestContact,QueueReject,DriverLost,MechanicalStop,JobBlocked}) {
            reset(a,f);scan=true;Printer::homeAxis(a==0,a==1,a==2);
            assert(!homed[a]&&refs[a]==0&&Printer::movementLocked&&scanAbort&&!masked&&!panel);
            if(f==NoSeekContact||f==QueueReject)assert(moves[a]==1);
            if(f==StuckClosed)assert(moves[a]==(a==Z_AXIS?11:2));
            ++count;
        }
    }
#if NUM_EXTRUDER > 1
    for(int a:{X_AXIS,Y_AXIS}) {
        reset(a,OffsetShort);Printer::homeAxis(a==0,a==1,false);
        assert(!homed[a]&&refs[a]==1&&offsetMoves==1&&Printer::movementLocked);
        ++count;
    }
#endif
    reset(X_AXIS,NoSeekContact);Printer::homeAxis(true,true,true);
    assert(!homed[X_AXIS]&&!moves[Y_AXIS]&&!moves[Z_AXIS]&&!digits);
    ++count;
    reset(Y_AXIS,NoSeekContact);zSafe=false;Printer::homeAxis(false,false,true);
    assert(!homed[Y_AXIS]&&!moves[Z_AXIS]&&!digits&&Printer::movementLocked);
    ++count;
    reset(X_AXIS,None);drain=false;Printer::homeAxis(true,true,true);
    assert(!moves[0]&&!moves[1]&&!moves[2]&&Printer::movementLocked&&!digits);
    ++count;
    reset(X_AXIS,None);Printer::homeAxis(true,true,true);
    assert(homed[0]&&homed[1]&&homed[2]&&digits==1&&!Printer::movementLocked);
    ++count;
    printf("G28 contact/release/retest, reference and cancellation: %u cases PASS\n",count);
}
"""
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"g28.cpp"
    exe=Path(d)/"g28"
    p.write_text(prelude+"\n"+axes+"\n"+g28+"\n"+main)
    for count in (1,2):
        subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror",
                        f"-DNUM_EXTRUDER={count}",str(p),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True)

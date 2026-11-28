#!/usr/bin/env python3
"""Replay actual new Drive startup and readiness with instrumented GPIO.

The isolated host fixture exercises the exact ensure/enable/queue/direct
functions, not the AVR interrupt scheduler, motor mechanics or step timing.
"""
from pathlib import Path
import argparse
import runpy
import subprocess
import tempfile

gen=runpy.run_path(str(Path(__file__).with_name("current-drive-wake.py")))

PRELUDE=r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#define RF_THERMAL_EMERGENCY_EXPERIMENT 1
#include "MotorWake.h"
constexpr unsigned stepperEnableDelayMs=25;
using speed_t=uint16_t;
enum {X_AXIS,Y_AXIS,Z_AXIS,E_AXIS,FOR_DIRECT=0,FOR_QUEUE=1};
enum {DIRECT_IDLE,DIRECT_PREPARING,DIRECT_PREPARED,DIRECT_PREPARED_STOPPABLE,DIRECT_RUNNING,DIRECT_RUNNING_STOPPABLE};
constexpr uint8_t FLAG_SCAN_PATH=8,FLAG_GUARDED_STOP_MASK=96;
enum class QueueTask {None,Pause,PauseAndPark,EnableZCompensation,DisableZCompensation,EnableSenseOffset};
static bool irq=false,terminal=false,commandsBlocked=false,homed=true;
static uint32_t now;
static unsigned asserts[5],directions,advances,pulses,finishes,allows,releases,retired;
static uint8_t physical;
struct InterruptProtectedBlock {
    bool saved=irq;
    InterruptProtectedBlock(){irq=false;}
    ~InterruptProtectedBlock(){irq=saved;}
};
static void pin(unsigned i){
    assert(!irq && !terminal && i<5);physical|=uint8_t(1u<<i);++asserts[i];
}
namespace HAL {
void allowInterrupts(){irq=true;}
void forbidInterrupts(){irq=false;}
uint32_t timeInMilliseconds(){return now;}
uint32_t CPUDivU2(speed_t speed){assert(speed);return 16000000UL/speed;}
}
struct ExtrusionOutput {
    enum class Inhibit {MotorsDisabled};
    void allow(Inhibit){assert(!irq && !terminal);++allows;}
};
struct PrintPause { enum class Destination {InPlace,Park}; };
struct Printer {
    static bool blockAll;
    struct Job {bool emergencyLocked()const{return terminal;} bool commandsBlocked()const{return ::commandsBlocked||terminal;}};static Job printJob;
    static bool isAxisHomed(uint8_t){return homed;}
    struct Pause {void request(PrintPause::Destination){assert(false);}};static Pause pause;
    static ExtrusionOutput extrusionOutput;
    static void unmarkAllSteppersDisabled(){assert(!irq && !blockAll && !terminal);}
    static void assertXStepper(){pin(0);}
    static void assertYStepper(){pin(1);}
    static void assertZStepper(){pin(2);}
    static bool zStepperEnabled(){return motorWake.readyFor(4);}
    static void enableCMPnow(){assert(false);}
    static void disableCMPnow(){assert(false);}
    static void enableSenseOffsetnow(){assert(false);}
};
bool Printer::blockAll=false;
Printer::Job Printer::printJob;Printer::Pause Printer::pause;ExtrusionOutput Printer::extrusionOutput;
struct Extruder {
    uint8_t id;
    static Extruder* current;
    static void assertEnable(){pin(3+current->id);}
};
Extruder tools[2]={{0},{1}};
Extruder* Extruder::current=&tools[0];
struct ZMotion {static void setDirection(bool){assert(!irq && Printer::zStepperEnabled());++directions;}};
struct Queue;
struct PrintLine {
    static PrintLine direct;
    static struct Queue queue;
    int task=DIRECT_IDLE,primaryAxis=X_AXIS,delta[4]={88,0,0,0},stepsRemaining=120;
    uint8_t flags=0,mask=1;
    struct {speed_t vStart=42;} stepProfile;
    float fullSpeed=60;
    bool isXMove()const{return mask&1;}
    bool isYMove()const{return mask&2;}
    bool isZMove()const{return mask&4;}
    bool isEMove()const{return mask&8;}
    bool isZPositiveMove()const{return true;}
    static bool directMovePreparing(int phase){return phase==DIRECT_PREPARING;}
    QueueTask queueTaskId()const{return QueueTask::None;}
    bool isBlocked()const{return false;}
    bool isWarmUp()const{return false;}
    unsigned getWaitForXLinesFilled()const{return 0;}
    long getWaitTicks()const{return 0;}
    void fixStartAndEndSpeed(){}
};
PrintLine PrintLine::direct;
struct Queue {
    PrintLine item;
    bool occupied=false,activated=false;
    PrintLine* active(){return activated?&item:nullptr;}
    unsigned count()const{return occupied?1:0;}
    PrintLine* activateFront(){assert(occupied&&!activated);activated=true;return &item;}
    void releaseFront(){assert(!irq&&activated);activated=false;++releases;}
    void retireFront(){assert(activated);occupied=activated=false;++retired;}
};
Queue PrintLine::queue;
struct StepExecutor {
    enum class DirectionOwner {None,Direct,Queued};
    static DirectionOwner directionOwner;
    static int32_t directPrimarySteps,queuedPrimarySteps;
    struct Error {void start(int32_t n){assert(n==88);}};
    struct Clock {speed_t peak=42;void start(speed_t n){assert(n==42);}};
    static Error stepErrors[2];
    static Clock stepClocks[2];
    static float nominalFeedrate_;
    static uint32_t stepIntervalTicks;
    static uint8_t stepsPerTimerCall;
    static constexpr uint16_t minimumStepIntervalTicks=512;
    static bool ensureMotorsReady(uint8_t requested);
    static bool enableSteppers(PrintLine& move);
    static void adjustDirections(PrintLine& m){
        assert(!Printer::blockAll && !terminal);
        assert(motorWake.readyFor((m.mask&7)|(m.isEMove()?uint8_t(8U<<Extruder::current->id):0)));
        ++directions;
    }
    static void updateAdvanceSteps(const PrintLine&,speed_t v){assert(v==42);++advances;}
    static void finishMove(PrintLine& m,bool queued){assert(!queued);m.task=DIRECT_IDLE;++finishes;}
    template<int owner> static long performMove(PrintLine* m){
        assert(owner==FOR_QUEUE||owner==FOR_DIRECT);
        assert(!terminal);
        assert(motorWake.readyFor((m->mask&7)|(m->isEMove()?uint8_t(8U<<Extruder::current->id):0)));
        if(Printer::blockAll)assert(owner==FOR_DIRECT&&(m->flags&FLAG_SCAN_PATH));
        ++pulses;return 123;
    }
    static long performDirectMove();
    static long performQueueMove();
};
StepExecutor::DirectionOwner StepExecutor::directionOwner=StepExecutor::DirectionOwner::None;
int32_t StepExecutor::directPrimarySteps=0,StepExecutor::queuedPrimarySteps=0;
StepExecutor::Error StepExecutor::stepErrors[2];
StepExecutor::Clock StepExecutor::stepClocks[2];
float StepExecutor::nominalFeedrate_=0;
uint32_t StepExecutor::stepIntervalTicks=10000;
uint8_t StepExecutor::stepsPerTimerCall=4;
'''

CASES=r'''
static void reset(uint32_t t){
    now=t;irq=false;terminal=Printer::blockAll=commandsBlocked=false;homed=true;physical=0;
    for(auto& n:asserts)n=0;
    directions=advances=pulses=finishes=allows=releases=retired=0;
    motorWake.reset();Extruder::current=&tools[0];
    PrintLine::direct=PrintLine();PrintLine::queue=Queue();
    StepExecutor::directionOwner=StepExecutor::DirectionOwner::None;
    StepExecutor::stepIntervalTicks=10000;StepExecutor::stepsPerTimerCall=4;
}
static void disable(uint8_t mask){
    assert(!irq);motorWake.beforeDisable(mask);physical&=~mask;
}
int main(){
    unsigned count=0;
    for(uint32_t origin:{0u,0xfffffff0u})
    for(int phase:{DIRECT_PREPARED,DIRECT_PREPARED_STOPPABLE})
    for(uint8_t guard:{uint8_t(0),uint8_t(32),uint8_t(64),uint8_t(96)}){
        reset(origin);auto& d=PrintLine::direct;d.task=phase;d.flags=guard;d.mask=7;
        assert(StepExecutor::performDirectMove()==2000);
        assert(d.task==phase && physical==7 && directions==0 && advances==0 && pulses==0);
        assert(asserts[0]==1 && asserts[1]==1 && asserts[2]==1 && !motorWake.readyFor(7));
        for(unsigned n=1;n<MotorWake::settleTicks;++n){
            now=origin+n;
            if(phase==DIRECT_PREPARED_STOPPABLE && n==5)d.stepsRemaining=32; // release while PREPARED
            assert(StepExecutor::performDirectMove()==2000 && d.task==phase);
            assert(!pulses && !directions && asserts[0]==1 && asserts[1]==1 && asserts[2]==1);
        }
        now=origin+MotorWake::settleTicks;
        const long interval=StepExecutor::performDirectMove();
        assert(interval==(guard?long((16000000UL/42)/2):5000L));
        assert(d.task==(phase==DIRECT_PREPARED?DIRECT_RUNNING:DIRECT_RUNNING_STOPPABLE));
        assert(d.stepsRemaining==(phase==DIRECT_PREPARED?120:32));
        assert(!pulses && directions==1 && advances==1 && motorWake.readyFor(7));
        if(guard)assert(StepExecutor::stepsPerTimerCall==1);
        assert(StepExecutor::performDirectMove()==123 && pulses==1);
        ++count;
    }
    // A canceled guarded request must not start a fresh motor, including
    // when an already-asserted driver is still settling.
    for(bool pending:{false,true})for(unsigned cause=0;cause<4;++cause){
        reset(0);auto& d=PrintLine::direct;d.task=DIRECT_PREPARED;d.flags=32;
        if(pending)assert(StepExecutor::performDirectMove()==2000);
        if(cause==0)Printer::blockAll=true;
        if(cause==1)commandsBlocked=true;
        if(cause==2)homed=false;
        if(cause==3)terminal=true;
        const auto old=asserts[0];
        assert(StepExecutor::performDirectMove()==1000 && d.task==DIRECT_IDLE);
        assert(asserts[0]==old && !directions && !pulses && finishes==1);
        assert(StepExecutor::stepsPerTimerCall==1);
        ++count;
    }
    // Queue activation may not freeze an executing slot or consume a path
    // while the readiness deadline is pending. Re-activate after lookahead.
    for(uint32_t origin:{0u,0xfffffff0u}){
        reset(origin);auto& q=PrintLine::queue;q.occupied=true;q.item.mask=9;
        assert(StepExecutor::performQueueMove()==2000 && !q.activated && q.occupied);
        assert(physical==9 && releases==1 && !allows && !directions);
        now=origin+26;
        assert(StepExecutor::performQueueMove()==2000 && !q.activated && releases==2);
        now=origin+27;
        assert(StepExecutor::performQueueMove()==5000 && q.activated && allows==1 && directions==1);
        assert(StepExecutor::performQueueMove()==123 && pulses==1);
        ++count;
    }
    // Scan and mechanical retreat do not assert SLEEPn; a merely asserted
    // but not settled Z channel must be denied. Ready Z can retreat even
    // with blockAll, because the outer P0 dispatcher owns that exception.
    for(bool asserted:{false,true})for(bool blocked:{false,true}){
        reset(0);
        if(asserted)assert(!StepExecutor::ensureMotorsReady(4));
        auto& d=PrintLine::direct;d.task=DIRECT_PREPARED;d.mask=4;d.flags=FLAG_SCAN_PATH|96;
        Printer::blockAll=blocked;
        const auto old=asserts[2];
        assert(StepExecutor::performDirectMove()==1000 && d.task==DIRECT_IDLE);
        assert(asserts[2]==old && !directions && !pulses && finishes==1);
        ++count;
    }
    for(bool blocked:{false,true}){
        reset(0);assert(!StepExecutor::ensureMotorsReady(4));now=27;
        assert(StepExecutor::ensureMotorsReady(4));Printer::blockAll=blocked;
        auto& d=PrintLine::direct;d.task=DIRECT_PREPARED;d.mask=4;d.flags=FLAG_SCAN_PATH|96;
        assert(StepExecutor::performDirectMove()==long((16000000UL/42)/2) && directions==1);
        assert(StepExecutor::performDirectMove()==123 && pulses==1 && asserts[2]==1);
        ++count;
    }
    // Old readiness must not resurrect on disable/re-enable, other
    // channels remain ready, both physical E channels use their own bit.
    reset(0);assert(!StepExecutor::ensureMotorsReady(1));now=27;assert(StepExecutor::ensureMotorsReady(1));
    assert(!StepExecutor::ensureMotorsReady(4));now=50;
    disable(4);assert(!StepExecutor::ensureMotorsReady(4));now=76;
    assert(!StepExecutor::ensureMotorsReady(4)&&motorWake.readyFor(1));
    now=77;assert(StepExecutor::ensureMotorsReady(5)&&asserts[2]==2);
    PrintLine e;e.mask=8;
    assert(!StepExecutor::enableSteppers(e));now+=27;assert(StepExecutor::enableSteppers(e));
    assert(allows==1 && motorWake.readyFor(8));
    Extruder::current=&tools[1];
    assert(!StepExecutor::enableSteppers(e) && allows==1);now+=27;
    assert(StepExecutor::enableSteppers(e)&&allows==2&&motorWake.readyFor(24));
    disable(24);assert(!StepExecutor::enableSteppers(e)&&allows==2);
    terminal=true;now+=27;assert(!StepExecutor::enableSteppers(e)&&allows==2);
    terminal=false;Printer::blockAll=true;assert(!StepExecutor::ensureMotorsReady(2)&&asserts[1]==0);
    ++count;
    std::printf("Actual wake/queue/direct/guarded startup: %u scenarios PASS\n",count);
}
'''

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source",type=Path,required=True)
    p.add_argument("--out",type=Path,required=True)
    a=p.parse_args()
    a.out.mkdir(parents=True,exist_ok=True)
    files=gen["candidate"](a.source/"Repetier")
    s=files["StepExecutor.cpp"]
    def section(start,end):return gen["region"](s,start,end)
    actual=section("MotorWake motorWake;","inline void StepExecutor::adjustDirections")
    actual+=section("long StepExecutor::performQueueMove()","template <int8_t limitDirection>")
    actual+=section("long StepExecutor::performDirectMove()","// Keep the ramp update")
    (a.out/"MotorWake.h").write_text(files["MotorWake.h"])
    (a.out/"wake-replay.cpp").write_text(PRELUDE+actual+CASES)
    # Check that thermal stop has precedence over the mechanical exception
    # in the actual unchanged outer P0 vector, not only in the mock.
    hal=(a.source/"Repetier/HAL.cpp").read_text()
    vector=gen["region"](hal,"ISR(TIMER1_COMPA_vect)","} // ISR(TIMER1_COMPA_vect)")
    assert vector.index("if(Printer::printJob.emergencyLocked())")<vector.index("mechanicalRetreatActive()")<vector.index("performDirectMove()")
    for flags in (["-O2"],["-O1","-fsanitize=undefined","-fno-sanitize-recover=all"]):
        exe=a.out/"wake-replay"
        subprocess.run(["c++","-std=c++11","-Wall","-Wextra","-Werror",*flags,
                        str(a.out/"wake-replay.cpp"),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
    exe.unlink()

if __name__=="__main__":
    main()

#!/usr/bin/env python3
"""Direct-move driver startup must never wait on millis with I=0."""
from pathlib import Path
import subprocess
import tempfile

SRC=Path(__file__).resolve().parents[1]/'Repetier'
stepper=(SRC/'StepExecutor.cpp').read_text()
hal=(SRC/'HAL.cpp').read_text()
start=stepper.index('long StepExecutor::performDirectMove()')
body=stepper[start:stepper.index('} // performDirectMove',start)+1]
vector=hal[hal.index('ISR(TIMER1_COMPA_vect)'):hal.index('} // ISR(TIMER1_COMPA_vect)')]
assert vector.index('cbi(TIMSK1, OCIE1A); // prevent retrigger') < vector.index('StepExecutor::performDirectMove()')
source=r'''
#include <cassert>
#include <initializer_list>
#include <cstdint>
using speed_t = uint16_t;
constexpr uint8_t FLAG_GUARDED_STOP_MASK=96, FLAG_SCAN_PATH=8;
struct ZMotion {static void setDirection(bool){assert(false);}};
constexpr int DIRECT_PREPARING=1,DIRECT_PREPARED=2,DIRECT_PREPARED_STOPPABLE=3,
              DIRECT_RUNNING=4,DIRECT_RUNNING_STOPPABLE=5,FOR_DIRECT=0;
static bool irq=false, timer1Masked=true;
static unsigned enabled=0, directions=0, advance=0, pulses=0, ticks=0;
namespace HAL {
    void allowInterrupts() { assert(timer1Masked); irq=true; }
    void forbidInterrupts() { irq=false; }
    uint32_t CPUDivU2(speed_t speed) { assert(speed); return 16000000UL/speed; }
}
struct PrintLine {
    static PrintLine direct;
    int task=0, primaryAxis=0, delta[1]={88}, stepsRemaining=120;
    uint8_t flags=0;
    bool isZPositiveMove() const { return false; }
    struct { int vStart=42; } stepProfile;
    float fullSpeed=60;
    static bool directMovePreparing(int phase) { return phase==DIRECT_PREPARING; }
};
PrintLine PrintLine::direct;
struct StepExecutor {
    enum class DirectionOwner { None, Direct };
    static DirectionOwner directionOwner;
    static int directPrimarySteps;
    struct Error { void start(int n) { assert(n==88); } };
    struct Clock { int peak=42; void start(int n) { assert(n==42); } };
    static Error stepErrors[1];
    static Clock stepClocks[1];
    static float nominalFeedrate_;
    static uint32_t stepIntervalTicks;
    static uint8_t stepsPerTimerCall;
    static constexpr uint16_t minimumStepIntervalTicks=512;
    static void enableSteppers(PrintLine& d) {
        assert(irq && timer1Masked);
        assert(d.task==DIRECT_RUNNING || d.task==DIRECT_RUNNING_STOPPABLE);
        ++enabled;
        // An independent ISR advances the timebase and can observe key release.
        for (int i=0;i<25;++i) { assert(irq); ++ticks; }
        if (d.task==DIRECT_RUNNING_STOPPABLE) d.stepsRemaining=32;
    }
    static void adjustDirections(PrintLine&) { assert(!irq); ++directions; }
    static void updateAdvanceSteps(PrintLine&,int n) { assert(n==42); ++advance; }
    template<int owner> static long performMove(PrintLine*) { static_assert(owner==FOR_DIRECT,"owner"); ++pulses; return 123; }
    static long performDirectMove();
};
StepExecutor::DirectionOwner StepExecutor::directionOwner=DirectionOwner::None;
int StepExecutor::directPrimarySteps=0;
StepExecutor::Error StepExecutor::stepErrors[1];
StepExecutor::Clock StepExecutor::stepClocks[1];
float StepExecutor::nominalFeedrate_=0;
uint32_t StepExecutor::stepIntervalTicks=10000;
uint8_t StepExecutor::stepsPerTimerCall=4;
''' + body + r'''
int main() {
    for (int task : {DIRECT_PREPARED,DIRECT_PREPARED_STOPPABLE}) {
        irq=false; enabled=directions=advance=pulses=ticks=0;
        PrintLine::direct=PrintLine(); PrintLine::direct.task=task;
        StepExecutor::directionOwner=StepExecutor::DirectionOwner::None;
        assert(StepExecutor::performDirectMove()==5000);
        assert(!irq && timer1Masked && enabled==1 && directions==1 && advance==1 && !pulses && ticks==25);
        assert(StepExecutor::directPrimarySteps==88 && StepExecutor::nominalFeedrate_==60);
        assert(PrintLine::direct.stepsRemaining==(task==DIRECT_PREPARED_STOPPABLE?32:120));
        assert(StepExecutor::performDirectMove()==123 && enabled==1 && pulses==1);
    }
    for (uint8_t flags : {uint8_t(32),uint8_t(64),uint8_t(96)}) {
        irq=false; enabled=directions=advance=pulses=ticks=0;
        PrintLine::direct=PrintLine(); PrintLine::direct.task=DIRECT_PREPARED;
        PrintLine::direct.flags=flags;
        StepExecutor::stepIntervalTicks=10000;
        StepExecutor::stepsPerTimerCall=4;
        assert(StepExecutor::performDirectMove()==(16000000UL/42)/2);
        assert(StepExecutor::stepsPerTimerCall==1 && !pulses && ticks==25);
        assert(StepExecutor::performDirectMove()==123 && enabled==1 && pulses==1);
    }
    PrintLine::direct.task=DIRECT_PREPARING;
    assert(StepExecutor::performDirectMove()==2000 && enabled==1 && pulses==1);
}
'''
with tempfile.TemporaryDirectory() as tmp:
    cpp=Path(tmp)/'test.cpp';exe=Path(tmp)/'test'
    cpp.write_text(source)
    subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror',str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('Task0034 direct driver enable: PASS (timebase IRQ admission and stoppable tail)')

#!/usr/bin/env python3
"""Replay actual M3028 admission and the Z correction clearance predicate."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
motion=(root/'Repetier/GCodeMotion.cpp').read_text()
z=(root/'Repetier/ZMotion.h').read_text()
worker=motion[motion.index('void testHomingRepeatability('):motion.index('void requestPrintPause(')]
predicate=z[z.index('    bool correctionCleared() const {'):z.index('    static INLINE bool minSwitchHit()')]
harness=r'''
#include <cassert>
#include <cstdint>
#include <initializer_list>
#define PSTR(x) x
enum {X_AXIS,Y_AXIS,Z_AXIS};
static bool irq,allowed,idle,paused,queued,direct,blocked,comp;
static unsigned error,axes;
struct InterruptProtectedBlock {
    bool saved=irq;
    InterruptProtectedBlock(){irq=false;}
    ~InterruptProtectedBlock(){irq=saved;}
};
struct ZMotion {
    int32_t correctionTarget_=0,appliedCorrection_=0;
''' + predicate + r'''
};
struct GCode {
    unsigned mask;
    bool hasX(){return mask&1;}
    bool hasY(){return mask&2;}
    bool hasZ(){return mask&4;}
};
bool isHomingAllowed(GCode*){return allowed;}
namespace Com { void printErrorFLN(const char*){++error;} }
namespace PrintLine {
    bool hasLines(){return queued;}
    bool directMoveActive(){return direct;}
}
namespace Printer {
    struct Job {bool isIdle(){return idle;}} printJob;
    struct Pause {bool blocksCalibration(){return paused;}} pause;
    ZMotion zMotion;
    bool blockAll=false,doHeatBedZCompensation=false;
    void checkHome(int axis){axes|=1u<<axis;}
}
''' + worker + r'''
int main(){
    unsigned count=0;
    for(bool enabled:{false,true})
        for(unsigned state=0;state<128;++state)
            for(int32_t target:{-100,0,100})
                for(int32_t applied:{-100,0,100})
                    for(unsigned mask=0;mask<8;++mask){
                        irq=enabled;allowed=state&1;idle=state&2;paused=state&4;
                        queued=state&8;direct=state&16;blocked=state&32;comp=state&64;
                        Printer::blockAll=blocked;Printer::doHeatBedZCompensation=comp;
                        Printer::zMotion.correctionTarget_=target;
                        Printer::zMotion.appliedCorrection_=applied;
                        error=axes=0;
                        GCode c{mask};
                        testHomingRepeatability(&c);
                        bool ready=allowed&&idle&&!paused&&!queued&&!direct&&!blocked&&!comp&&target==0&&applied==0;
                        assert(axes==(ready?mask:0) && error==unsigned(allowed&&!ready));
                        assert(irq==enabled);
                        ++count;
                    }
    assert(count==18432);
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/'admission.cpp'; exe=Path(d)/'admission'
    p.write_text(harness)
    subprocess.run(['c++','-std=c++11','-O2','-Wall','-Wextra','-Werror',str(p),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('Task0034 M3028 admission: 18432 axis/state/IRQ combinations PASS')

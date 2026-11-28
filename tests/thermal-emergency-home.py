#!/usr/bin/env python3
"""Replay actual M3028 orchestration, with the central drive interface mocked."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
src=(root/"Repetier/Homing.cpp").read_text()
start=src.index("int8_t Printer::checkHome(")
end=src.index("} // checkHome",start)+len("} // checkHome")
header=(root/"Repetier/ui.h").read_text()
scope=header[header.index("    class ScopedActionExclusion {"):header.index("    // Remote panel actions")]
motion=(root/"Repetier/motion.h").read_text()
types=motion[motion.index("enum class GuardedStop"):motion.index("#define FOR_DIRECT")]
assert all(t not in src for t in ("stepAxisStep(", "stepHomeCheck(", "startXStep(", "startYStep("))
prelude=r"""
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#define PSTR(x) x
#define XYZ_STEPPER_HIGH_DELAY 100
#define XYZ_STEPPER_LOW_DELAY 100
enum {X_AXIS,Y_AXIS,Z_AXIS};
static int32_t position,initial,scale;
static unsigned calls,failAt,axisExpected;
static bool locked,pressed,homed;
constexpr uint8_t UI_FLAG_SLOW_ACTION_RUNNING=4;
struct UIDisplay {
    volatile uint8_t flags=0x80;
"""+scope+r"""
};
static UIDisplay uid;
namespace Com {
template<class... Args> void printFLN(Args...){assert(!locked);}
}
namespace Printer {
bool blockAll=false;
struct Job {bool commandsBlocked()const{return locked;}} printJob;
bool isAxisHomed(int axis){return homed && axis>=X_AXIS&&axis<=Z_AXIS;}
bool anyEndstop(int){return pressed;}
float stepsPerMm(int){return scale;}
int32_t softwareEndstopSteps(int){return 10000;}
int8_t checkHome(int8_t);
}
"""+types+r"""
struct PrintLine {
    static int32_t guardedAxisPosition(uint8_t axis) {
        assert(axis==axisExpected);
        return position;
    }
    static GuardedMoveResult runGuardedMoveTo(uint8_t axis,int32_t target,uint16_t rate,GuardedStop stop) {
        assert(axis==axisExpected && !locked && (uid.flags & UI_FLAG_SLOW_ACTION_RUNNING));
        const GuardedStop modes[]={GuardedStop::Closed,GuardedStop::Open,GuardedStop::Closed,GuardedStop::Distance};
        const int32_t targets[]={initial-11000,scale,24-scale,initial};
        const uint16_t rates[]={5000,250,250,5000};
        assert(calls<4 && stop==modes[calls] && target==targets[calls] && rate==rates[calls]);
        if(++calls==failAt){
            locked=true;
            return {GuardedMoveStatus::Cancelled,position};
        }
        position=(calls==2?24:calls==4?initial:0);
        return {stop==GuardedStop::Distance?GuardedMoveStatus::PositionReached:GuardedMoveStatus::SwitchReached,position};
    }
};
"""
main=r"""
static void reset(int axis,int steps,unsigned injection,bool panelOwned){
    axisExpected=axis;scale=steps;initial=scale+32;position=initial;
    calls=0;failAt=injection;locked=pressed=false;homed=true;Printer::blockAll=false;
    uid.flags=panelOwned?0x84:0x80;
}
int main(){
    unsigned count=0;
    for(int axis=X_AXIS;axis<=Z_AXIS;++axis)
        for(int steps:{128,1000})
            for(bool panelOwned:{false,true}){
                uint8_t expected=panelOwned?0x84:0x80;
                reset(axis,steps,0,panelOwned);
                assert(Printer::checkHome(axis)==1 && calls==4 && position==initial && uid.flags==expected);
                ++count;
                for(unsigned n=1;n<=4;++n){
                    reset(axis,steps,n,panelOwned);
                    assert(Printer::checkHome(axis)==-1 && calls==n && locked && uid.flags==expected);
                    ++count;
                }
                for(unsigned denied=0;denied<4;++denied){
                    reset(axis,steps,0,panelOwned);
                    if(denied==0)locked=true;
                    if(denied==1)pressed=true;
                    if(denied==2)homed=false;
                    if(denied==3)Printer::blockAll=true;
                    assert(Printer::checkHome(axis)==-1 && calls==0 && uid.flags==expected);
                    ++count;
                }
            }
    assert(count==108);
    std::printf("M3028 central-drive orchestration: %u cases PASS\\n",count);
}
"""

with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"home.cpp"
    exe=Path(d)/"home"
    p.write_text(prelude+"\n"+src[start:end]+"\n"+main)
    for flags in ([],["-DRF_THERMAL_EMERGENCY_EXPERIMENT=1"]):
        subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror",*flags,str(p),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True)

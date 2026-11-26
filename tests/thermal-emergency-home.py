#!/usr/bin/env python3
"""Replay the actual foreground M3028 routine with a fault at every service visit."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
src=(root/"Repetier/Homing.cpp").read_text()
start=src.index("int8_t Printer::checkHome(")
end=src.index("} // checkHome",start)+len("} // checkHome")
prelude=r"""
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
using std::abs;
#define PSTR(x) x
enum {X_AXIS,Y_AXIS,Z_AXIS,Calibrating};
static int position,direction,stepsPerMillimeter,initial;
static unsigned pulses,visits,injectAt,gapUs,maxGapUs;
static bool pressed,locked;
namespace Com {
template<class... Args> void printFLN(Args...){assert(!locked);}
}
namespace Commands {
void checkForPeriodicalActions(int state){
    assert(state==Calibrating);
    maxGapUs=std::max(maxGapUs,gapUs);
    gapUs=0;
    if(++visits==injectAt)locked=true;
}
}
namespace Printer {
struct Job {bool emergencyLocked()const{return locked;}} printJob;
struct Z {long motorSteps()const{return position;}} zMotion;
struct XY {long motorSteps(int)const{return position;}} xyMotion;
bool isAxisHomed(int axis){return axis>=X_AXIS&&axis<=Z_AXIS;}
bool anyEndstop(int){return pressed;}
char anyHomeDir(int){return -1;}
void changeAxisDirection(int,int d){assert(!locked);direction=d;}
int stepsPerMm(int){return stepsPerMillimeter;}
long softwareEndstopSteps(int){return 10000;}
void stepAxisStep(int,uint8_t slower=1){
    assert(!locked); // direct foreground STEP may not bypass terminal state
    ++pulses;
    gapUs+=200u*slower; // configured high+low delays, excludes MCU overhead
    position+=direction;
    if(position<=0)pressed=true;
    if(position>=24)pressed=false;
}
int8_t checkHome(int8_t);
}
"""
main=r"""
static void reset(int scale,unsigned injection){
    stepsPerMillimeter=scale;
    initial=scale+32;
    position=initial;direction=0;
    pulses=visits=gapUs=maxGapUs=0;
    injectAt=injection;
    pressed=locked=false;
}
int main(){
    unsigned cases=0;
    for(int axis=X_AXIS;axis<=Z_AXIS;++axis){
        for(int scale:{128,1000}){
            reset(scale,0);
            assert(Printer::checkHome(axis)==1);
            // Existing M3028 counts the press/release edges separately.
            // Preserve its normal pulse count in both configurations.
            assert(pulses==unsigned(2*initial+2*24-1));
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
            const unsigned normalVisits=visits;
            assert(std::max(maxGapUs,gapUs)<=32000u);
            for(unsigned n=1;n<=normalVisits;++n){
                reset(scale,n);
                assert(Printer::checkHome(axis)==-1);
                assert(locked&&visits==n);
                ++cases;
            }
            reset(scale,0);locked=true;
            assert(Printer::checkHome(axis)==-1&&pulses==0&&visits==0);
            ++cases;
#endif
        }
    }
    std::printf("M3028 foreground replay: six healthy cases, %u terminal injections; nominal delays only\n",cases);
}
"""
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"home.cpp"
    exe=Path(d)/"home"
    p.write_text(prelude+"\n"+src[start:end]+"\n"+main)
    for flags in ([],["-DRF_THERMAL_EMERGENCY_EXPERIMENT=1"]):
        subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror",*flags,str(p),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True)

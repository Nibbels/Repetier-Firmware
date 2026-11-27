#!/usr/bin/env python3
"""Compare the actual optimized direct-stop refit to the historical formula."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
src=(root/"Repetier/motion.cpp").read_text()
body=src[src.index("void PrintLine::limitToStopTail("):src.index("/** \\brief Check if move is new.")]
harness=r'''
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <initializer_list>
namespace RMath { using std::min; }
struct PrintLine {
    int32_t stepsRemaining;
    struct {uint32_t accelSteps,decelSteps;} stepProfile;
    void limitToStopTail(bool);
    void fitRampToRemainingSteps();
};
''' + body + r'''
static unsigned long cases;
static void check(uint32_t a,uint32_t d,int32_t remaining,bool refit){
    PrintLine p{remaining,{a,d}}, expected=p;
    if(remaining>32){
        expected.stepsRemaining=32;
        // Exact old generic behavior, including the 32-bit modular sum.
        if(refit && static_cast<int32_t>(a+d)>32){
            uint32_t reduction=(a+d-32)>>1;
            expected.stepProfile.accelSteps-=std::min(static_cast<int32_t>(a),static_cast<int32_t>(reduction));
            expected.stepProfile.decelSteps-=std::min(static_cast<int32_t>(d),static_cast<int32_t>(reduction));
        }
    }
    p.limitToStopTail(refit);
    assert(p.stepsRemaining==expected.stepsRemaining);
    assert(p.stepProfile.accelSteps==expected.stepProfile.accelSteps);
    assert(p.stepProfile.decelSteps==expected.stepProfile.decelSteps);
    ++cases;
}
int main(){
    for(uint32_t a=0;a<512;++a)
        for(uint32_t d=0;d<512;++d)
            for(int32_t remaining:{0,31,32,33,1024})
                for(bool refit:{false,true})check(a,d,remaining,refit);
    for(uint32_t a=0;a<1000000;++a)check(a,a,1000000,true);
    const uint32_t boundary[]={0,15,16,17,31,32,33,0x3ffffffeUL,0x3fffffffUL,
                               0x40000000UL,0x7fffffffUL,0x80000000UL,0xbfffffffUL,0xc0000000UL,0xffffffffUL};
    for(uint32_t a:boundary)for(uint32_t d:boundary)for(bool refit:{false,true})check(a,d,33,refit);
    assert(cases==3621890);
}
'''
with tempfile.TemporaryDirectory() as d:
    cpp=Path(d)/"test.cpp";exe=Path(d)/"test"
    cpp.write_text(harness)
    subprocess.run(["g++","-std=c++11","-O2","-Wall","-Wextra","-Werror",str(cpp),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print("Task0034 direct stop ramp: 3621890 symmetric/asymmetric and boundary comparisons PASS")

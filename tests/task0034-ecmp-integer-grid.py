#!/usr/bin/env python3
"""Compare the actual ECMP integer accumulator with the former binary32 grid."""
import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

FW = Path(__file__).resolve().parents[1] / "Repetier"
HARNESS = r'''
#include "ExtrusionCompensation.h"
#include <cassert>
#include <cstdio>
#include <limits>
#include <initializer_list>
using E=ExtrusionCompensation;
constexpr int32_t N=E::unitsPerStep;
constexpr float U=1.0f/float(N);
static float oldPrepare(float f) {
    if (!(f>0)) return 0;
    if (f>=1) return 1;
    return lroundf(f*float(N))*U;
}
int main() {
    E e;
    for(float raw:{-1.0f,0.0f,0.5f*U,0.048f,0.25f,0.5f,1.0f,1.5f,
                   std::numeric_limits<float>::quiet_NaN()}) {
        e.publishRatio(E::prepareRatio(raw));
        assert(e.ratio()==oldPrepare(raw));
    }
    unsigned long long cases=0;
    // Every normalized remainder, both directions, for zero, one-unit,
    // a non-binary original ratio, quarter/half, near-full and full ratio.
    for(int32_t r:{0,1,E::prepareRatio(0.048f),N/4,N/2,N-1,N}) {
        e.publishRatio(r);
        const float ratio=float(r)*U;
        for(int32_t initial=0;initial<N;++initial) {
            for(bool forward:{false,true}) {
                e.restoreRemainderUnits(initial);
                float ref=float(initial)*U;
                ref += forward?ratio:-ratio;
                const bool expected=forward?ref>=1.0f:ref<0.0f;
                const bool actual=e.addBaseStep(forward);
                assert(actual==expected);
                if(expected) {
                    ref+=forward?-1.0f:1.0f;
                    e.consumeExtraStep(forward);
                }
                assert(float(e.remainderUnits())*U==ref);
                assert(e.remainderUnits()>=0&&e.remainderUnits()<N);
                // Reverse with the same ratio: exact remainder and pulse parity.
                const bool back=e.addBaseStep(!forward);
                assert(back==actual);
                if(back)e.consumeExtraStep(!forward);
                assert(e.remainderUnits()==initial);
                ++cases;
            }
        }
    }
    std::printf("ECMP exact grid: %llu paired forward/reverse cases\\n",cases);
}
'''
with TemporaryDirectory() as d:
    p=Path(d)
    (p/"replay.cpp").write_text(HARNESS)
    exe=p/"replay"
    subprocess.run([os.environ.get("CXX","c++"),"-std=c++11","-O2","-Wall","-Wextra","-Werror",
                    "-I",str(FW),str(p/"replay.cpp"),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=60)

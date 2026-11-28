#!/usr/bin/env python3
"""Actual diagnostic recorder: phases, timing, unknown windows and rejection."""
from pathlib import Path
import subprocess
import tempfile
fw=Path(__file__).resolve().parents[1]/"Repetier"
source=r'''
#include "ThermalRelayProfile.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>
using namespace ThermalRelayProfile;
static void eq(float a,float b){assert(std::fabs(a-b)<0.0001f);}
// Deliberate 6s heat and 6s cool phases. Temperature at transition is
// supplied before changing duty; the first 2s of off time still heat up.
static void heat(Recorder& r,uint32_t& now,float start,float rate,uint8_t duty){
    for(unsigned i=1;i<=60;++i){now+=100;r.sample(now,start+rate*i*.1f,duty);}
    r.afterDecision(now,start+rate*6,0);
}
static void cool(Recorder& r,uint32_t& now,float base,float rebound,unsigned seconds=6){
    for(unsigned i=1;i<=seconds*10;++i){now+=100;
        const float t=i<=20?base+rebound*i/20.f:base+rebound-(i-20)*.4f;
        r.sample(now,t,0);
    }
    r.completeOff(now);
}
int main(){
    unsigned cases=0;
    for(uint32_t start:{0u,0xfffff000u,0xffffff9cu})for(uint8_t cap:{uint8_t(40),uint8_t(128),uint8_t(255)}){
        uint32_t now=start; Recorder r;r.begin(now,25,cap,cap);
        heat(r,now,25,10,cap);cool(r,now,85,4);assert(!r.completedProfile());
        r.afterDecision(now,73,cap);heat(r,now,73,2,cap);cool(r,now,85,6);
        assert(!r.completedProfile());r.afterDecision(now,75,cap);heat(r,now,75,3,cap);cool(r,now,93,3,7);
        auto p=r.completedProfile();assert(p&&p->completed==3&&p->mask==63);
        eq(p->rates[0],10);eq(p->rates[1],10);eq(p->rates[2],3);eq(p->rates[3],3);
        eq(p->rates[4],4);eq(p->rebound,6);
        assert(p->minOffMs==6000&&p->maxOffMs==7000&&p->peakDelayMs==2000);
        // Continuing the relay after a valid prefix must hide that prefix.
        r.afterDecision(now,76,cap);assert(!r.completedProfile());++cases;
        // Initial above-target cool-down is not a powered cooling episode.
        r.begin(now,200,cap,0);cool(r,now,200,10);assert(!r.completedProfile());
        for(int i=0;i<3;++i){r.afterDecision(now,100,cap);heat(r,now,100,2,cap);cool(r,now,112,1);}
        p=r.completedProfile();assert(p&&p->completed==3);eq(p->rebound,1);++cases;
        // Short phases have no 1s/5s observation: not fabricated zero rates.
        r.begin(now,100,cap,cap);
        for(int i=0;i<3;++i){now+=100;r.sample(now,101,cap);r.afterDecision(now,101,0);
            now+=100;r.sample(now,100,0);r.completeOff(now);
            if(i!=2)r.afterDecision(now,100,cap);}
        p=r.completedProfile();assert(p&&p->mask==0&&p->completed==3);++cases;
        // Invalid samples/duty/gaps invalidate the whole tune's evidence.
        for(unsigned fault=0;fault<6;++fault){
            r.begin(now,100,cap,cap);
            for(int i=0;i<3;++i){
                heat(r,now,100,2,cap);cool(r,now,112,2);
                if(i!=2)r.afterDecision(now,100,cap);
            }
            assert(r.completedProfile());r.afterDecision(now,100,cap);
            if(fault==0)r.sample(now,100,cap);
            if(fault==1){now+=99;r.sample(now,100,cap);}
            if(fault==2){now+=1001;r.sample(now,100,cap);}
            if(fault==3){now+=100;r.sample(now,std::numeric_limits<float>::quiet_NaN(),cap);}
            if(fault==4){now+=100;r.sample(now,100,0);}
            if(fault==5){now+=100;r.sample(now,100,cap);r.afterDecision(now,100,cap-1);}
            for(int i=0;i<3;++i){heat(r,now,100,2,cap);cool(r,now,112,2);if(i!=2)r.afterDecision(now,100,cap);}
            assert(!r.completedProfile());++cases;
        }
    }
    // Jitter: normalize by actual elapsed time, not nominal 100ms.
    Recorder r;uint32_t now=0;r.begin(now,20,255,255);
    for(int k=0;k<3;++k){const uint32_t a=now;
        for(int i=0;i<10;++i){now+=137;r.sample(now,20+(now-a)*.004f,255);}
        r.afterDecision(now,20+(now-a)*.004f,0);
        for(int i=0;i<10;++i){now+=137;r.sample(now,24-i*.1f,0);}
        r.completeOff(now);if(k!=2)r.afterDecision(now,20,255);
    }
    auto p=r.completedProfile();assert(p&&(p->mask&Summary::First1)&&(p->mask&Summary::Later1));
    eq(p->rates[0],4);eq(p->rates[2],4);++cases;
    std::printf("Relay profile: %u cases passed; sizeof Recorder=%zu Summary=%zu (host ABI)\n",cases,sizeof(Recorder),sizeof(Summary));
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/'profile.cpp';exe=Path(d)/'profile';p.write_text(source)
    subprocess.run(['c++','-std=c++11','-O2','-Wall','-Wextra','-Werror','-I',str(fw),str(p),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

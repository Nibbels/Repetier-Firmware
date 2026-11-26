#!/usr/bin/env python3
"""Actual compare-clear template replay with abstract timer semantics.
No assumption that floating a real heater pin is safe; that is a prerequisite.
"""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
code=r"""
#include "CompareClearHotends.h"
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <algorithm>
#include <initializer_list>
struct Crash {};
struct Timer {
    struct Channel {
        uint8_t mode=0,ocr=0;
        bool port=false,ddr=true,latch=false,flag=false;
        unsigned total=0,run=0,maxRun=0;
        bool high() const {return ddr&&(mode?latch:port);}
        void account(unsigned n) {
            if(high()) {total+=n;run+=n;maxRun=std::max(maxRun,run);}
            else run=0;
        }
    } channels[2];
    uint8_t now=0;
    unsigned gap=0,operations=0,crash=0;
    void advance(unsigned ticks) {
        for(auto& ch:channels){
            unsigned delta=uint8_t(ch.ocr-now);
            if(!delta)delta=256;
            if(ticks<delta){ch.account(ticks);continue;}
            ch.account(delta);
            ch.flag=true;
            if(ch.mode==2)ch.latch=false;
            if(ch.mode==3)ch.latch=true;
            ch.account(ticks-delta);
        }
        now=uint8_t(now+ticks);
    }
    void boundary(){
        for(auto& ch:channels)if(!ch.high())ch.run=0;
        if(++operations==crash)throw Crash{};
        advance(gap);
    }
    void portLow(uint8_t c){channels[c].port=false;boundary();}
    void floatOff(uint8_t c){channels[c].ddr=false;boundary();}
    void drive(uint8_t c){channels[c].ddr=true;boundary();}
    void disconnect(uint8_t c){channels[c].mode=0;boundary();}
    void setMode(uint8_t c){channels[c].mode=3;boundary();}
    void clearMode(uint8_t c){channels[c].mode=2;boundary();}
    void force(uint8_t c){auto& ch=channels[c];assert(ch.mode==3);ch.latch=true;boundary();}
    uint8_t counter(){const auto n=now;boundary();return n;}
    void deadline(uint8_t c,uint8_t d){channels[c].ocr=d;boundary();}
    void ack(uint8_t c){channels[c].flag=false;boundary();}
    bool pending(uint8_t c){const bool f=channels[c].flag;boundary();return f;}
};
int main(){
    unsigned cases=0,maxTicks=0,positive=0;
    // Every counter phase and representable legacy nonzero duty, adversarial
    // old compare/latch state, all register/read boundaries of pulse().
    // Timers continue for two entire wraps after the CPU is stopped.
    for(unsigned phase=0;phase<256;++phase)
      for(unsigned duty=2;duty<256;duty+=2)
       for(unsigned gap:{0u,1u,3u})
        for(unsigned crash=0;crash<=15;++crash){
            Timer t;t.now=uint8_t(phase);t.gap=gap;t.crash=crash;
            auto& ch=t.channels[0];
            ch.ocr=uint8_t(phase+gap+1);ch.latch=true;ch.flag=true;
            // A stale OC latch is deliberately allowed when disconnected.
            CompareClearHotends<Timer,2> pwm(t);
            bool committed=false;
            try{committed=pwm.pulse(0,uint8_t(duty));}catch(Crash&){}
            t.advance(512);
            assert(!ch.high()&&ch.maxRun<=duty);
            assert(!t.channels[1].high());
            if(ch.total)++positive;
            if(!crash && !gap)assert(committed&&ch.total==duty);
            maxTicks=std::max(maxTicks,ch.maxRun);++cases;
        }
    // Two independently armed channels must retain their distinct deadlines,
    // no catch-up/extension if the half-frame arrives before the old compare.
    for(unsigned phase=0;phase<256;++phase){
        Timer t;t.now=uint8_t(phase);
        CompareClearHotends<Timer,2> pwm(t);
        assert(pwm.pulse(0,255)&&pwm.pulse(1,128));
        const auto d0=t.channels[0].ocr,d1=t.channels[1].ocr;
        t.advance(64);
        assert(!pwm.pulse(0,255)&&!pwm.pulse(1,255));
        assert(t.channels[0].ocr==d0&&t.channels[1].ocr==d1);
        pwm.revoke(1);
        assert(t.channels[0].high()&&!t.channels[1].high());
        t.advance(190);
        assert(!t.channels[0].high()&&!t.channels[1].high());
        assert(pwm.pulse(0,2)&&pwm.pulse(1,255));
        t.advance(512);
        assert(!t.channels[0].high()&&!t.channels[1].high());
        pwm.revokeAll();
        assert(!t.channels[0].high()&&!t.channels[1].high());
    }
    // Revocation can be interrupted at any operation: either the bounded
    // previous pulse remains or the GPIO LOW has already taken ownership.
    for(unsigned crash=1;crash<=3;++crash){
        Timer t;CompareClearHotends<Timer,1> pwm(t);
        assert(pwm.pulse(0,255));t.advance(10);
        t.operations=0;t.crash=crash;
        try{pwm.revoke(0);}catch(Crash&){}
        t.advance(512);
        assert(!t.channels[0].high()&&t.channels[0].maxRun<=254);
    }
    std::printf("Compare-clear actual template: %u crash-prefix cases, %u produced heat, max %u ticks; two-channel/revoke passed\n",cases,positive,maxTicks);
}
"""
with tempfile.TemporaryDirectory() as temp:
    cpp=Path(temp)/"compare.cpp"
    exe=Path(temp)/"compare"
    cpp.write_text(code)
    subprocess.run(["g++","-std=c++11","-O2","-Wall","-Wextra","-Werror",
                    "-I",str(root/"tools/task0034"),str(cpp),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

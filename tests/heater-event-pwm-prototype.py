#!/usr/bin/env python3
"""Actual Timer5 PWM prototype, with abstract timer register/flag semantics."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
code = r"""
#include "EventHeaterPwm.h"
#include "HeaterLease.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>
struct Clock {
    uint16_t count=0, deadline[3]={};
    bool flags[3]={}, masks[3]={}, pins[3]={}, overflow=false;
    unsigned highs=0, delayAfterWrite=0, delayAfterClear=0;
    uint16_t now() const {return count;}
    bool wrapped() const {return overflow;}
    bool pending(uint8_t c) const {return flags[c];}
    void writeDeadline(uint8_t c,uint16_t x){deadline[c]=x;advance(delayAfterWrite);}
    void clearPending(uint8_t c){assert(!pins[c]&&!masks[c]);flags[c]=false;advance(delayAfterClear);}
    void enable(uint8_t c){masks[c]=true;}
    void disable(uint8_t c){masks[c]=false;}
    void on(uint8_t c){assert(masks[c]&&!flags[c]&&!overflow);pins[c]=true;++highs;}
    void off(uint8_t c){pins[c]=false;}
    void advance(uint32_t n) {
        if(n>=65536u-count) overflow=true;
        for(uint8_t c=0;c<3;++c)
            if(n>uint16_t(deadline[c]-count)) flags[c]=true;
        count=uint16_t(count+n);
    }
};
struct LeaseClock {
    Clock& gpio;
    uint16_t count=0, deadlines[3]={};
    bool flags[3]={}, masks[3]={};
    explicit LeaseClock(Clock& g):gpio(g){}
    uint16_t now() const{return count;}
    uint16_t deadline(uint8_t c) const{return deadlines[c];}
    bool pending(uint8_t c) const{return flags[c];}
    void writeDeadline(uint8_t c,uint16_t x){deadlines[c]=x;}
    void clearPending(uint8_t c){flags[c]=false;}
    void enable(uint8_t c){masks[c]=true;}
    void disable(uint8_t c){masks[c]=false;}
    void off(uint8_t c){gpio.off(c);}
};
template<uint8_t N> struct Lease {
    uint8_t demand[N]={};
    bool fault=false;
    bool frameDuties(uint8_t (&out)[N]){
        for(uint8_t c=0;c<N;++c)out[c]=fault?0:demand[c];
        return !fault;
    }
    bool authorize(uint8_t c,uint8_t d){if(fault)return false;demand[c]=d;return true;}
    void inhibit(){fault=true;for(auto& d:demand)d=0;}
};
template<uint8_t N> void check(){
    using Pwm=EventHeaterPwm<Clock,Lease<N>,N,64>;
    using WiderGap=EventHeaterPwm<Clock,Lease<N>,N,64,256>;
    assert(WiderGap::width(255)==65280 && WiderGap::width(128)==32768);
    unsigned long cases=0;
    for(unsigned duty=0;duty<256;++duty){
        for(uint8_t c=0;c<N;++c){
            for(unsigned now: {0u,1u,63u,64u,511u,512u,32768u,65407u,65408u,65471u,65535u}){
                Clock t;Lease<N> l;Pwm p(t,l);
                l.demand[c]=duty;t.count=now;
                t.flags[c]=true; // stale inactive compare flag
                const unsigned width=(duty&254)==254?65472:(duty&254)*256;
                assert(Pwm::width(duty)==width);
                const bool admitted=now+64<width;
                p.frame();
                assert(t.pins[c]==admitted&&t.masks[c]==admitted);
                assert(t.highs==unsigned(admitted)&&!p.failed());
                if(admitted){
                    assert(t.deadline[c]==width);
                    t.advance(width-now+1);
                    assert(t.flags[c]&&!t.overflow);
                    t.flags[c]=false; // vector entry
                    p.compare(c);
                    assert(!t.pins[c]&&!t.masks[c]&&!p.failed());
                }
                // Next frame with stale compare flag but no live old pulse.
                t.advance(65536u-t.count);t.overflow=false;
                p.frame();
                assert(!p.failed()&&t.pins[c]==(width>64));
                p.revoke(c);
                assert(!t.pins[c]&&!t.masks[c]&&l.demand[c]==0);
                ++cases;
            }
        }
    }
    for(bool compareFirst:{false,true}){
        Clock t;Lease<N> l;Pwm p(t,l);
        for(auto& d:l.demand)d=255;
        p.frame();assert(t.highs==N);
        t.advance(65536);
        if(compareFirst){t.flags[0]=false;p.compare(0);}
        else {t.overflow=false;p.frame();}
        assert(p.failed()&&l.fault);
        for(uint8_t c=0;c<N;++c)assert(!t.pins[c]&&!t.masks[c]);
        t.overflow=false;p.frame();assert(t.highs==N);
    }
    // Timer keeps running while I=0: inject delay in register arming.
    for(unsigned gap : {1u,63u,64u,65u,511u,512u,65536u}){
        for(bool atClear : {false,true}){
            Clock v;Lease<N> g;Pwm e(v,g);
            g.demand[0]=2;
            if(atClear)v.delayAfterClear=gap;else v.delayAfterWrite=gap;
            e.frame();
            assert(v.pins[0]==(gap+64<512));
            assert(e.failed()==(gap>=65536));
        }
    }
    // A revoke arriving after wrap cannot conceal an overdue pulse.
    Clock t;Lease<N> l;Pwm p(t,l);l.demand[0]=255;p.frame();
    t.advance(65536);p.revoke(0);assert(p.failed()&&l.fault&&!t.pins[0]);
    Clock u;Lease<N> m;Pwm q(u,m);m.demand[0]=255;m.fault=true;q.frame();
    assert(q.failed()&&u.highs==0);
    std::printf("channels=%u width/phase cases=%lu\n",N,cases);
}
template<uint8_t N> void combined(){
    using RealLease=HeaterLease<LeaseClock,N,12500,3>;
    using Pwm=EventHeaterPwm<Clock,RealLease,N,64,256>;
    Clock t;LeaseClock lt(t);RealLease l(lt);Pwm p(t,l);
    for(uint8_t c=0;c<N;++c)assert(l.authorize(c,255));
    p.frame();
    assert(t.highs==N);
    for(uint8_t c=0;c<N;++c)p.compare(c);
    lt.count=12501;lt.flags[0]=true;
    t.count=0;t.overflow=false;
    p.frame(); // pending independent expiry blocks subsequent PWM
    assert(l.failed()&&p.failed()&&t.highs==N);
    for(uint8_t c=0;c<N;++c)assert(!t.pins[c]&&!t.masks[c]&&!lt.masks[c]);
    assert(!l.authorize(0,255));

    Clock u;LeaseClock lu(u);RealLease fresh(lu);Pwm q(u,fresh);
    assert(fresh.authorize(0,255));q.frame();assert(u.pins[0]);
    q.inhibit(); // proposed Timer3 expiry vector dispatch
    assert(!u.pins[0]&&!u.masks[0]&&!lu.masks[0]&&fresh.failed()&&q.failed());
}
int main(){check<2>();check<3>();combined<2>();combined<3>();}
"""
with tempfile.TemporaryDirectory() as temp:
    cpp=Path(temp)/"pwm.cpp"
    exe=Path(temp)/"pwm"
    cpp.write_text(code)
    subprocess.run(["g++","-std=c++11","-Wall","-Wextra","-Werror","-O2",
                    "-I",str(root/"tools/task0034"),str(cpp),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

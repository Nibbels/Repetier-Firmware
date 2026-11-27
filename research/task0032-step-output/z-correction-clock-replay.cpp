#include "z-correction-clock.h"
#include <assert.h>
#include <stdio.h>
#include <limits.h>
#include <stdint.h>
#include <algorithm>
#include <cmath>

using namespace ZCorrectionResearch;

static void checkRoot(uint32_t x) {
    const uint32_t r=root(x);
    assert(uint64_t(r)*r<=x);
    assert(uint64_t(r+1)*(r+1)>x);
}
// Exact scalar form of current HAL::CPUDivU2 table contract. The AVR
// fixture below separately compiles the actual linked source.
struct FastMath {
    static uint16_t floorRoot(uint32_t x) {return root(x);}
    static uint32_t period(uint16_t d) {
        if(d<512)return cpuHz/(d<10?10:d);
        const uint32_t w=d<8192?32:4096;
        const uint32_t base=d/w*w;
        return cpuHz/base-((cpuHz/base-cpuHz/(base+w))*(d%w))/w;
    }
};
static uint32_t rng=0x9e3779b9;
static uint32_t random32() {
    rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;
}
int main() {
    uint32_t roots=0;
    for(uint32_t i=0;i<65536;i++) {
        const uint32_t square=i*i;
        checkRoot(square);checkRoot(square+1);roots+=2;
        if(square){checkRoot(square-1);++roots;}
    }
    for(unsigned i=0;i<100000;i++){checkRoot(random32());++roots;}
    checkRoot(UINT32_MAX);++roots;
    for(uint32_t d=10;d<=65535;d++)assert(uint64_t(FastMath::period(uint16_t(d))+1)*d>=cpuHz);
    uint64_t pulses=0;uint32_t profiles=0;uint32_t worstExtra=0;
    const uint32_t as[]={1,6400,128000,256000,UINT32_MAX};
    const uint16_t vs[]={2,384,2000,11520,65535};
    for(auto a:as)for(auto v:vs) {
        const auto limits=Limits::prepare(a,v);
        assert(limits.maxLevel && limits.twiceAcceleration<=uint64_t(a)*2);
        for(int s:{-1,1})for(int d=1;d<=512;d++) {
            Ramp r;FastRamp<FastMath> fast;uint64_t time=0;int remaining=d;
            while(remaining) {
                const auto oldLevel=r.level();
                const auto p=r.plan(s*remaining,limits);
                assert(p.sign==s&&!p.safetyRestart&&p.nextLevel<=uint32_t(remaining-1));
                assert(p.nextLevel<=limits.maxLevel);
                assert(p.nextLevel<=oldLevel+1 && oldLevel<=p.nextLevel+1);
                assert(uint64_t(p.interval)*limits.maxSpeed>=cpuHz);
                if(oldLevel||p.nextLevel) {
                    const double v0=std::sqrt(double(limits.twiceAcceleration)*oldLevel);
                    const double v1=std::sqrt(double(limits.twiceAcceleration)*p.nextLevel);
                    // Reference exact travel time between boundary velocities.
                    assert(double(p.interval)+1e-8>=2.0*cpuHz/(v0+v1));
                }else {
                    assert(remaining==1);
                    assert(double(p.interval)+1e-8>=2.0*cpuHz/std::sqrt(limits.twiceAcceleration/2.0));
                }
                const auto fp=fast.plan(s*remaining,limits);
                assert(fp.sign==p.sign&&fp.nextLevel==p.nextLevel&&fp.safetyRestart==p.safetyRestart);
                assert(fp.interval>=p.interval);
                worstExtra=std::max(worstExtra,fp.interval-p.interval);
                fast.commit(fp);r.commit(p);--remaining;time+=p.interval;++pulses;
            }
            assert(r.level()==0&&time>0);++profiles;
        }
    }
    // Adversarial error magnitude, retarget, config changes and limit stop.
    const auto lim=Limits::prepare(128000,11520);
    assert(!Limits::prepare(0,11520).maxLevel);
    assert(!Limits::prepare(128000,1).maxLevel);
    Ramp r;
    for(int i=0;i<100;i++)r.commit(r.plan(INT32_MAX,lim));
    assert(r.level()==100);
    auto p=r.plan(INT32_MIN,lim);assert(p.sign==-1&&p.safetyRestart&&p.nextLevel==1);
    p=r.plan(1,lim);assert(p.safetyRestart&&p.nextLevel==0);
    r.cancel();assert(!r.level()&&!r.direction());
    assert(!r.plan(0,lim).sign);
    // Merged timeline across wrap. Nominal releases must stay exactly as
    // requested, correction-only events must never advance nominal time.
    uint32_t deadlineCases=0;
    for(unsigned i=0;i<100000;i++) {
        const uint32_t start=random32();
        const uint32_t nominal=100+random32()%1000000;
        const uint32_t corr=1+random32()%1000000;
        Deadlines d{start+nominal,start+corr,true};
        assert(d.nextDelay(start)==std::min(nominal,corr));
        uint32_t n=0,c=0,t=start;
        for(int event=0;event<10;event++) {
            t+=d.nextDelay(t);
            const bool nd=d.nominalDue(t),cd=d.correctionDue(t);
            assert(nd||cd);
            if(nd) {
                ++n;assert(t==start+n*nominal);
                d.afterNominal(t,nominal);
            }
            if(cd) {
                ++c;assert(t==start+c*corr);
                const auto unchanged=d.nominal;
                d.afterCorrection(t,corr);
                assert(unchanged==d.nominal);
            }
        }
        // Explicit delay/late dispatch does not catch up corrections.
        const auto nominalBefore=d.nominal;
        const uint32_t late=t+2000000;
        d.afterCorrection(late,corr);
        assert(!d.correctionDue(late)&&d.nominal==nominalBefore);
        d.afterCorrection(late,0);
        assert(!d.correctionDue(late)&&d.nextDelay(late)==0);
        ++deadlineCases;
    }
    printf("Fast ramp matches level decisions and never shortens reference intervals; max extra ticks=%lu\n",(unsigned long)worstExtra);
    printf("Z correction ramp roots=%lu profiles=%lu pulses=%llu deadline_cases=%lu passed\n",
           (unsigned long)roots,(unsigned long)profiles,(unsigned long long)pulses,(unsigned long)deadlineCases);
}

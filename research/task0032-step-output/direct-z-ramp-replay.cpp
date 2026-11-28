#include "direct-z-ramp.h"
#include <assert.h>
#include <stdio.h>
#include <cmath>
#include <limits.h>

using namespace DirectZResearch;
static uint64_t pulses=0,plateaus=0;
static double worst=0;
static void check(Ramp& direct,ZCorrectionResearch::Ramp& reference,int32_t error,
                  const Limits& lim,const ZCorrectionResearch::Limits& base) {
    const auto p=direct.plan(error,lim);
    const auto r=reference.plan(error,base);
    assert(p.nextLevel==r.nextLevel&&p.sign==r.sign&&p.safetyRestart==r.safetyRestart);
    if(!p.sign) {assert(!p.interval);return;}
    uint32_t l=p.safetyRestart?0:direct.level();
    const double v0=std::sqrt(double(lim.A)*l);
    const double v1=std::sqrt(double(lim.A)*p.nextLevel);
    const double ideal=(l||p.nextLevel)?
        32000000.0/(v0+v1):32000000.0/std::sqrt(lim.A/2.0);
    assert(double(p.interval)+1e-8>=ideal);
    assert(double(p.interval)*base.maxSpeed>=16000000.0);
    const double extra=p.interval/ideal-1;
    if(extra>worst)worst=extra;
    if(l==p.nextLevel&&l==lim.maxLevel) {
        assert(p.interval==lim.plateau);++plateaus;
    }
    direct.commit(p);reference.commit(r);++pulses;
}

int main() {
    const uint32_t accelerations[]={1,6400,128000,256000,UINT32_MAX};
    const uint16_t speeds[]={2,384,2000,11520,65535};
    unsigned profiles=0;
    for(auto a:accelerations)for(auto v:speeds) {
        const auto base=ZCorrectionResearch::Limits::prepare(a,v);
        const auto lim=Limits::prepare(a,v);
        assert(lim.A==base.twiceAcceleration&&lim.maxLevel==base.maxLevel);
        for(int s=-1;s<=1;s+=2)for(int d=1;d<=512;d++) {
            Ramp direct;ZCorrectionResearch::Ramp reference;
            for(int left=d;left;--left)check(direct,reference,s*left,lim,base);
            assert(!direct.level());++profiles;
        }
        // Long plateau and live retarget across zero without overflowing abs.
        Ramp direct;ZCorrectionResearch::Ramp reference;
        for(int i=0;i<2000;i++)check(direct,reference,INT32_MAX,lim,base);
        for(int i=0;i<2000;i++)check(direct,reference,INT32_MIN,lim,base);
        check(direct,reference,1,lim,base);
        direct.cancel();reference.cancel();
        check(direct,reference,0,lim,base);
        // Reconfiguration is a cancel+publication, not a stale cache.
        const auto newBase=ZCorrectionResearch::Limits::prepare(a,384);
        const auto newLim=Limits::prepare(a,384);
        for(int i=0;i<2000;i++)check(direct,reference,INT32_MAX,newLim,newBase);
    }
    Ramp disabled;
    assert(!disabled.plan(10,Limits::prepare(0,11520)).sign);
    assert(!disabled.plan(10,Limits::prepare(128000,1)).sign);
    printf("Direct Z ramp profiles=%u pulses=%llu plateau_hits=%llu max_relative_excess=%.9f passed\n",
           profiles,(unsigned long long)pulses,(unsigned long long)plateaus,worst);
}

// Edge and lifecycle checks for the research ledger, independent of motion.
#include "duration-ledger.h"
#include <cassert>
#include <cstdio>
#include <limits>
using namespace PlannerEstimateResearch;
int main() {
    using L=DurationLedger<24>;
    L l;
    uint64_t remaining=0;
    assert(durationUnits(0)==0);
    assert(durationUnits(.00126f)==13);
    assert(durationUnits(-1)==UINT32_MAX);
    assert(durationUnits(std::numeric_limits<float>::infinity())==UINT32_MAX);
    assert(durationUnits(std::numeric_limits<float>::quiet_NaN())==UINT32_MAX);
    assert(durationUnits(429497.0f)==UINT32_MAX);
    for(unsigned i=0;i<24;++i)l.publish(i,UINT32_MAX-1);
    auto s=l.snapshot(0,true);
    assert(s.full==uint64_t(UINT32_MAX-1)*24 && !s.unavailable);
    assert(L::remaining(s,true,100,remaining));
    assert(remaining==uint64_t(UINT32_MAX-1)*23+100);
    l.replace(1,UINT32_MAX);
    assert(!L::remaining(l.snapshot(0,true),false,0,remaining));
    l.retire(0);l.retire(1);
    assert(l.snapshot(2,true).unavailable==0);
    for(unsigned i=2;i<24;++i)l.retire(i);
    assert(l.snapshot(0,false).full==0);
    for(unsigned n=0;n<100000;++n) {
        uint8_t i=n%24;
        l.publish(i,100+n);
        l.replace(i,200+n);
        l.replace(i,200+n);
        assert(L::remaining(l.snapshot(i,true),true,1,remaining)&&remaining==1);
        l.retire(i);
        assert(l.snapshot(0,false).full==0);
    }
    l.publish(3,UINT32_MAX);l.publish(4,123);
    l.clear();
    assert(l.snapshot(0,false).full==0 && !l.snapshot(0,false).unavailable);
    l.publish(3,42);
    assert(L::remaining(l.snapshot(3,true),false,0,remaining)&&remaining==42);
    l.retire(3);
    std::puts("duration ledger lifecycle: PASS (100000 repeated reuse cycles, wide sum, invalid/clear)");
}

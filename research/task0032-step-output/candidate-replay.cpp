#include "ecmp-q23.h"
#include "motor-wake.h"
#include "ExtrusionCompensation.h" // exact unchanged firmware reference
#include <assert.h>
#include <initializer_list>
#include <limits>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

static constexpr int32_t Q = ExtrusionCompensationQ23::one;
static float asFloat(int32_t v) { return static_cast<float>(v)*(1.0f/Q); }
static uint32_t state = 0x723410f1;
static uint32_t rng() { state ^= state<<13;state ^= state>>17;state ^= state<<5;return state; }
static uint64_t comparisons=0, extras=0, admissions=0;

// Mirror the real StepExecutor admission contract, not its GPIO or IRQ timings.
// Direct and pressure-inhibited / skipped base steps must leave ECMP untouched.
static void step(ExtrusionCompensation& a, ExtrusionCompensationQ23& b,
                 bool forward, bool queued, bool accepted, bool advance) {
    if (!accepted) return;
    ++admissions;
    if (!queued) return;
    const bool x=a.addBaseStep(forward),y=b.addBaseStep(forward);
    assert(x==y);
    assert(a.remainder()==asFloat(b.remainderUnits()));
    if(x){
        // Same extra-pulse decision with either direct output or Advance.
        (void)advance;
        ++extras;
        a.consumeExtraStep(forward);b.consumeExtraStep(forward);
    }
    assert(a.remainder()==asFloat(b.remainderUnits()));
    assert(b.remainderUnits()>=0 && b.remainderUnits()<Q);
    ++comparisons;
}
static void publish(ExtrusionCompensation& a,ExtrusionCompensationQ23& b,float raw) {
    float x=ExtrusionCompensation::prepareRatio(raw);
    auto y=ExtrusionCompensationQ23::prepareRatio(raw);
    assert(x==asFloat(y));
    a.publishRatio(x);b.publishRatio(y);
    assert(a.ratio()==b.ratio());
}
int main() {
    ExtrusionCompensation a;
    ExtrusionCompensationQ23 b;
    // Every existing Q23 ratio code, exact rounding and normalized boundary
    // states. Test both directions without relying on pseudorandom coverage.
    for(int32_t r=0;r<=Q;++r){
        publish(a,b,asFloat(r));
        for(int32_t initial : {int32_t(0), Q-1, Q-r < Q ? Q-r : 0}){
            a.restoreRemainder(asFloat(initial));b.restoreRemainderUnits(initial);
            step(a,b,true,true,true,false);
            step(a,b,false,true,true,true);
            assert(b.remainderUnits()==initial);
        }
    }
    for(float raw: {-1.0f,0.0f,1.0f,2.0f,std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()})
        publish(a,b,raw);
    a.reset();b.reset();
    for(uint32_t i=0;i<1000000;++i){
        uint32_t v=rng();
        if(!(v&15)){
            uint32_t bits=rng(); float raw; memcpy(&raw,&bits,4);
            publish(a,b,raw);
        }
        if((v&255)==15){
            const auto savedA=a.remainder();const auto savedB=b.remainderUnits();
            for(unsigned j=0;j<20;++j) step(a,b,j&1,true,true,j&2);
            // Equivalent M3912 saved-remainder lifecycle; actual source
            // integration replay still required.
            a.restoreRemainder(savedA);b.restoreRemainderUnits(savedB);
        }
        step(a,b,v&16,v&32,v&64,v&128);
        assert(a.remainder()==asFloat(b.remainderUnits()));
    }
    printf("ECMP Q23 comparisons=%llu admissions=%llu extras=%llu\n",
           (unsigned long long)comparisons,(unsigned long long)admissions,(unsigned long long)extras);
    MotorWake w;
    assert(!w.readyFor(1));
    // Wake X/Y/E0 in a single transaction rather than three sequential waits.
    assert(w.newChannels(0x0b)==0x0b);
    w.afterAsserted(0x0b,100);
    for(uint32_t t=100;t<126;++t){w.service(t);assert(!w.readyFor(0x0b));}
    w.afterAsserted(0x0b,120); // redundant enable must not reset deadline
    w.service(126);assert(w.readyFor(0x0b));
    w.afterAsserted(4,130);assert(w.readyFor(0x0b)&&!w.readyFor(4));
    w.afterAsserted(16,140);w.service(156);assert(!w.readyFor(4)&&w.readyFor(0x0b));
    w.service(166);assert(w.readyFor(0x1f));
    w.beforeDisable(0x08);assert(!w.readyFor(8)&&w.readyFor(0x17));
    w.afterAsserted(8,170);w.beforeDisable(0x1f);w.service(1000);assert(w.readyMask()==0);
    // Counter rollover and reset/fault invalidation.
    w.afterAsserted(7,UINT32_MAX-10);
    w.service(14);assert(!w.readyFor(7));
    w.service(15);assert(w.readyFor(7));
    w.reset();w.service(50);assert(!w.readyFor(7));
    for(unsigned m=1;m<=31;++m){
        w.reset();w.afterAsserted(m,0);w.service(25);assert(!w.readyFor(m));
        w.service(26);assert(w.readyFor(m));
        w.beforeDisable(m);assert(!w.readyFor(m));
    }
    puts("MotorWake pure lifecycle tests passed; not hardware or concurrency qualification");
}

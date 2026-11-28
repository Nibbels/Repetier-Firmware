#pragma once
#include "z-correction-clock.h"
#include "inverse-root-generated.h"

namespace DirectZResearch {
using InverseRootResearch::upperPeriod;

// The cap is the only indefinitely repeatable plateau; prepare its period
// once, together with the acceleration-dependent fractional energy term.
// Other equal-level transitions (one step before braking) are not cached.
// A new configuration replaces all derived values together and cancels ramp.
struct Limits {
    uint32_t A,maxLevel,fraction,plateau;
    static Limits prepare(uint32_t acceleration,uint16_t vmax) {
        const auto base=ZCorrectionResearch::Limits::prepare(acceleration,vmax);
        if(!base.maxLevel)return {0,0,0,0};
        const uint32_t a=base.twiceAcceleration;
        return {a,base.maxLevel,(a>>6)*29+((a&63)*29>>6),
                upperPeriod(a*base.maxLevel)};
    }
};
struct Pulse {
    uint32_t nextLevel,interval;
    int8_t sign;
    bool safetyRestart;
};
class Ramp {
    uint32_t level_=0;
    int8_t direction_=0;
public:
    uint32_t level() const {return level_;}
    void cancel(){level_=0;direction_=0;}
    Pulse plan(int32_t error,const Limits& lim) const {
        if(!error||!lim.maxLevel)return {0,0,0,false};
        const uint32_t d=error<0?uint32_t(0)-uint32_t(error):uint32_t(error);
        const int8_t sign=error>0?1:-1;
        const bool restart=level_&&(direction_!=sign||d<level_||level_>lim.maxLevel);
        const uint32_t l=restart?0:level_;
        if(d==1&&!l)return {0,2*upperPeriod(lim.A/2),sign,restart};
        const uint32_t n=d<=l?l-1:(d-l>=2&&l<lim.maxLevel?l+1:l);
        const uint32_t low=l<n?l:n;
        const uint32_t interval=l==n?(l==lim.maxLevel?lim.plateau:upperPeriod(lim.A*l)):
            !low?2*upperPeriod(lim.A):upperPeriod(lim.A*low+lim.fraction);
        return {n,interval,sign,restart};
    }
    void commit(const Pulse& p){level_=p.nextLevel;direction_=p.sign;}
};
#ifdef __AVR__
static_assert(sizeof(Ramp)==5,"direct ramp layout");
static_assert(sizeof(Pulse)==10,"direct pulse layout");
static_assert(sizeof(Limits)==16,"direct limits layout");
#endif
}

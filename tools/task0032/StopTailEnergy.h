#pragma once
#include <stdint.h>

// Reference prepared soft-stop profile in primary-step distance domain.
// Safety/limit aborts remain immediate and never extend the admitted path.
// No storage per queued move. All calculations run with the Timer1 vector
// excluded but global interrupts enabled; publication is the existing byte.
namespace StopTailEnergy {
struct Plan {
    uint32_t energy, floor, drop, steps;
    bool feasible;
};
inline Plan prepare(uint16_t rate, uint16_t exitRate, uint32_t acceleration, uint32_t available) {
    const uint32_t energy=uint32_t(rate)*rate;
    const uint32_t floor=uint32_t(exitRate)*exitRate;
    if(energy<=floor)return {energy,energy,0,0,true};
    if(!acceleration)return {energy,floor,0,available,false};
    const uint32_t drop=acceleration>UINT32_MAX/2?UINT32_MAX:2*acceleration;
    const uint32_t diff=energy-floor;
    const uint32_t count=diff/drop+uint32_t(diff%drop!=0);
    return {energy,floor,drop,count<available?count:available,count<=available};
}
struct Group {
    uint32_t energy, ticks;
    uint16_t root;
    uint8_t steps;
};
// root(x) is a supplied exact floor-root kernel; use a tested integer
// kernel, not float. Carry previous root from the prior prepared group.
// The interval is an upper bound on the ideal constant-acceleration
// group-average duration, not a proof of physical spacing in a pack.
template<class Root>
inline Group next(const Plan& plan, uint16_t previousRoot, uint8_t requested, Root root) {
    const uint8_t count=plan.steps<requested?uint8_t(plan.steps):requested;
    if(!count || !plan.drop || requested>10)return {plan.energy,0,previousRoot,0};
    uint32_t end=plan.energy;
    for(uint8_t n=0;n<count;++n){
        const uint32_t diff=end-plan.floor;
        end=diff<=plan.drop?plan.floor:end-plan.drop;
    }
    const uint16_t nextRoot=root(end);
    const uint32_t denom=uint32_t(previousRoot)+nextRoot;
    if(!denom)return {plan.energy,0,previousRoot,0};
    constexpr uint32_t twiceClock=32000000UL;
    const uint32_t numerator=twiceClock*count;
    const uint32_t ticks=numerator/denom+uint32_t(numerator%denom!=0);
    return {end,ticks,nextRoot,count};
}
inline void commit(Plan& plan,const Group& group) {
    plan.energy=group.energy;
    plan.steps-=group.steps;
}
}

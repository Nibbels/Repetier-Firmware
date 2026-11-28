#pragma once
#include "StopTailEnergy.h"

// Distance-domain reference for ordinary direct moves. Prepared once before
// pulse admission, no per-queue-slot state. The backward stopping envelope
// is enforced during acceleration as well as cruise/deceleration.
namespace PreparedDirectProfile {
struct Limits {
    uint32_t ceiling, exitEnergy, drop, horizon;
    bool valid;
};
inline Limits prepare(uint16_t maximum, uint16_t exitRate, uint32_t acceleration) {
    const uint32_t ceiling=uint32_t(maximum)*maximum;
    const uint32_t floor=uint32_t(exitRate)*exitRate;
    if(!maximum || !acceleration || floor>ceiling)return {ceiling,floor,0,0,false};
    const uint32_t drop=acceleration>UINT32_MAX/2?UINT32_MAX:2*acceleration;
    const uint32_t diff=ceiling-floor;
    return {ceiling,floor,drop,diff/drop+uint32_t(diff%drop!=0),true};
}
inline uint32_t cap(const Limits& lim,uint32_t remaining) {
    return remaining>=lim.horizon?lim.ceiling:lim.exitEnergy+lim.drop*remaining;
}
// Start energy must be within the remaining-distance envelope. A group is
// committed only after all of its admitted logical primary cycles execute.
// This models average group time, not physical pulses clustered within it.
template<class Root>
inline StopTailEnergy::Group next(const Limits& lim,uint32_t energy,uint16_t previousRoot,
                                  uint32_t remaining,uint8_t requested,Root root) {
    const uint8_t count=remaining<requested?uint8_t(remaining):requested;
    if(!lim.valid || !count || requested>10 || energy>cap(lim,remaining))return {energy,0,previousRoot,0};
    const uint32_t ceiling=cap(lim,remaining-count);
    uint32_t end=energy;
    for(uint8_t n=0;n<count && end<ceiling;++n){
        const uint32_t diff=ceiling-end;
        end=diff<=lim.drop?ceiling:end+lim.drop;
    }
    if(end>ceiling)end=ceiling;
    const uint16_t nextRoot=root(end);
    uint32_t denominator=uint32_t(previousRoot)+nextRoot;
    // Rest-to-rest group: symmetric triangle, no discontinuous floor speed.
    if(!denominator){
        uint32_t peak=0;
        for(uint8_t n=0;n<count && peak<lim.ceiling;++n){
            const uint32_t rise=lim.drop/2;
            const uint32_t diff=lim.ceiling-peak;
            peak=diff<=rise?lim.ceiling:peak+rise;
        }
        denominator=root(peak);
        if(!denominator)return {energy,0,previousRoot,0};
    }
    const uint32_t numerator=32000000UL*count;
    return {end,numerator/denominator+uint32_t(numerator%denominator!=0),nextRoot,count};
}
}

#ifndef RF_RESEARCH_Z_CORRECTION_CLOCK_H
#define RF_RESEARCH_Z_CORRECTION_CLOCK_H

#include <stdint.h>

// Candidate component only: no GPIO, no interrupts, no PrintLine queue,
// no float, no heap. One common timer owner calls this after revalidating
// the live target/limits/physical Z ownership. All times are CPU ticks.
namespace ZCorrectionResearch {

constexpr uint32_t cpuHz=16000000UL;

inline uint32_t ceilDiv(uint32_t n,uint32_t d) {
    return n/d + (n%d != 0);
}

// Bounded 16 iterations. AVR cost must be measured, not assumed.
inline uint16_t root(uint32_t x) {
    uint32_t bit=UINT32_C(1)<<30;
    uint32_t r=0;
    while(bit) {
        if(x>=r+bit) {x-=r+bit;r=(r>>1)+bit;}
        else r>>=1;
        bit>>=2;
    }
    return uint16_t(r);
}

struct Limits {
    uint32_t twiceAcceleration;
    uint32_t maxLevel;
    uint16_t maxSpeed;

    // Foreground only, using validated current steps/s and steps/s².
    // A very low vmax needs a reduced acceleration to allow a one-step ramp.
    static Limits prepare(uint32_t acceleration,uint16_t vmax) {
        const uint32_t energy=uint32_t(vmax)*vmax;
        if(!acceleration || vmax<2) return {0,0,0};
        const uint32_t a=acceleration>energy/2 ? energy/2 : acceleration;
        return {2*a,energy/(2*a),vmax};
    }
};

struct Pulse {
    uint32_t nextLevel;
    uint32_t interval;
    int8_t sign;
    bool safetyRestart;
};

class Ramp {
    uint32_t level_=0;
    int8_t direction_=0;
public:
    uint32_t level() const {return level_;}
    int8_t direction() const {return direction_;}
    void cancel() {level_=0;direction_=0;}

    Pulse plan(int32_t error,const Limits& limits) const {
        if(!error || !limits.maxLevel) return {0,0,0,false};
        const uint32_t d=error<0 ? uint32_t(0)-uint32_t(error) : uint32_t(error);
        const int8_t sign=error>0?1:-1;
        // A live target reversal or newly shortened stopping distance may
        // make smooth braking impossible without crossing the new target.
        // Safety choice: disallow stale movement, restart from rest.
        // This is an explicit exception to continuous acceleration, NOT a
        // hardware-qualified hard stop at arbitrary nominal Z speed.
        const bool restart=level_ && (direction_!=sign || d<level_ || level_>limits.maxLevel);
        const uint32_t l=restart?0:level_;
        if(d==1 && l==0)
            return {0,ceilDiv(2*cpuHz,root(limits.twiceAcceleration/2)),sign,restart};
        const uint32_t n=d<=l ? l-1 : (d-l>=2 && l<limits.maxLevel ? l+1 : l);
        const uint32_t den=uint32_t(root(limits.twiceAcceleration*l))+root(limits.twiceAcceleration*n);
        return {n,ceilDiv(2*cpuHz,den),sign,restart};
    }

    // Only after a real legal correction step or explicitly balanced
    // opposing nominal step. Caller must revalidate target, owner and limits.
    void commit(const Pulse& p) {level_=p.nextLevel;direction_=p.sign;}
};

// Wrap-safe on a 32-bit clock, provided all outstanding deadlines and
// comparisons are <2^31 ticks apart. Long idle must be represented by
// periodic housekeeping or an invalid/disarmed deadline.
inline bool reached(uint32_t now,uint32_t deadline) {
    return uint32_t(now-deadline)<(UINT32_C(1)<<31);
}

struct Deadlines {
    uint32_t nominal;
    uint32_t correction;
    bool correctionArmed;

    // Compare using the current epoch; never subtract unsigned absolute
    // timestamps directly or advance the nominal deadline for a Z-only event.
    uint32_t nextDelay(uint32_t now) const {
        const uint32_t n=reached(now,nominal)?0:nominal-now;
        if(!correctionArmed)return n;
        const uint32_t c=reached(now,correction)?0:correction-now;
        return n<c?n:c;
    }
    bool nominalDue(uint32_t now) const {return reached(now,nominal);}
    bool correctionDue(uint32_t now) const {return correctionArmed&&reached(now,correction);}
    void afterNominal(uint32_t actualRelease,uint32_t interval) {
        nominal=actualRelease+interval;
    }
    void afterCorrection(uint32_t actualPulse,uint32_t interval) {
        // Do not catch up missed correction deadlines with burst pulses.
        correction=actualPulse+interval;correctionArmed=interval!=0;
    }
};

#ifdef __AVR__
static_assert(sizeof(Ramp)==5,"AVR correction ramp state");
static_assert(sizeof(Deadlines)==9,"AVR two deadlines");
static_assert(sizeof(Limits)==10,"AVR derived limits");
#endif
} // namespace ZCorrectionResearch
#endif

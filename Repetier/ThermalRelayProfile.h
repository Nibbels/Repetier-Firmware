#pragma once
#include <stdint.h>

// Diagnostic observations only: admitted software PWM is not measured
// electrical power. No thresholds, heater permission or persistence here.
namespace ThermalRelayProfile {
struct Summary {
    enum : uint8_t { First1=1, First5=2, Later1=4, Later5=8, Cool1=16, Cool5=32 };
    float rates[6]{}; // positive rise/fall magnitudes, C/s; see mask
    float rebound=0;
    uint32_t peakDelayMs=0, minOffMs=0, maxOffMs=0;
    uint8_t mask=0, completed=0;
};

// One local recorder during the existing relay experiment. The windows are
// non-overlapping within each phase; they are NOT rolling maxima or bounds
// on unobserved between-sample temperature. Initial cooling is excluded.
class Recorder {
    enum class Phase : uint8_t { Invalid, InitialOff, Heat, Off, Between };
    struct Anchor { uint32_t at; float temp; };
    Summary result_;
    Anchor short_, long_, offStart_;
    uint32_t last_=0, peakAt_=0;
    float peak_=0;
    uint8_t cap_=0;
    Phase phase_=Phase::Invalid;
    static bool plausible(float t) { return t>=-100.f && t<=1000.f; }
    void resetWindows(uint32_t now,float t) { short_={now,t}; long_=short_; }
    void window(Anchor& a,uint32_t now,float t,uint16_t ms,uint8_t index,bool fall) {
        const uint32_t elapsed=now-a.at;
        if(elapsed<ms)return;
        const float delta=fall?a.temp-t:t-a.temp;
        const float rate=delta*1000.f/float(elapsed);
        if(rate>result_.rates[index])result_.rates[index]=rate;
        result_.mask|=uint8_t(1u<<index);
        a={now,t};
    }
public:
    void begin(uint32_t now,float t,uint8_t cap,uint8_t admitted) {
        result_=Summary{};cap_=cap;last_=now;
        phase_=!plausible(t)||cap<40||(admitted!=0&&admitted!=cap)?Phase::Invalid:
            admitted?Phase::Heat:Phase::InitialOff;
        resetWindows(now,t);
    }
    // Call once per validated controller sample, before the next relay
    // decision. Missing/late/duplicated observation time or unexpected
    // admitted duty invalidates the entire diagnostic record.
    void sample(uint32_t now,float t,uint8_t admitted) {
        if(phase_==Phase::Invalid)return;
        const uint32_t elapsed=now-last_;
        if(phase_==Phase::Between||!plausible(t)||elapsed<100||elapsed>1000||
            admitted!=(phase_==Phase::Heat?cap_:0)) {phase_=Phase::Invalid;return;}
        last_=now;
        if(phase_==Phase::Heat) {
            const uint8_t index=result_.completed?2:0;
            window(short_,now,t,1000,index,false);
            window(long_,now,t,5000,index+1,false);
        }else if(phase_==Phase::Off) {
            if(t>peak_){peak_=t;peakAt_=now;}
            window(short_,now,t,1000,4,true);
            window(long_,now,t,5000,5,true);
        }
    }
    // Call at the low relay crossing, even if this is the final successful
    // cycle and no new heating is admitted. Censored off phases never count.
    void completeOff(uint32_t now) {
        if(phase_==Phase::Invalid)return;
        if(now!=last_||(phase_!=Phase::Off&&phase_!=Phase::InitialOff)) {phase_=Phase::Invalid;return;}
        if(phase_==Phase::Off) {
            const uint32_t duration=now-offStart_.at, delay=peakAt_-offStart_.at;
            if(!duration||result_.completed==255){phase_=Phase::Invalid;return;}
            const float rebound=peak_-offStart_.temp;
            if(rebound>result_.rebound)result_.rebound=rebound;
            if(delay>result_.peakDelayMs)result_.peakDelayMs=delay;
            if(!result_.completed||duration<result_.minOffMs)result_.minOffMs=duration;
            if(duration>result_.maxOffMs)result_.maxOffMs=duration;
            ++result_.completed;
        }
        phase_=Phase::Between;
    }
    // Call after the actual relay decision has revoked/admitted the duty.
    void afterDecision(uint32_t now,float t,uint8_t admitted) {
        if(phase_==Phase::Invalid)return;
        if(now!=last_||!plausible(t)){phase_=Phase::Invalid;return;}
        if(phase_==Phase::Heat&&admitted==0) {
            phase_=Phase::Off;offStart_={now,t};peak_=t;peakAt_=now;
            resetWindows(now,t);
        }else if(phase_==Phase::Between&&admitted==cap_) {
            phase_=Phase::Heat;resetWindows(now,t);
        }else if(phase_==Phase::Between||admitted!=(phase_==Phase::Heat?cap_:0)) {
            phase_=Phase::Invalid;
        }
    }
    // The caller must additionally prove that the existing PID tune passed
    // all convergence, gain, fault and cancellation checks. Never publish
    // after an aborted tune, even if some individual cycles completed.
    const Summary* completedProfile() const {
        return phase_==Phase::Between&&result_.completed>=3?&result_:nullptr;
    }
};
}

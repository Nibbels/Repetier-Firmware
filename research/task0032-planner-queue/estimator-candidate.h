// Research-only time estimate; no production headers or state changes.
#pragma once
#include <stdint.h>
#include <string.h>
#include <float.h>
#include <math.h>

namespace PlannerEstimateResearch {
static_assert(sizeof(float)==4 && FLT_RADIX==2 && FLT_MANT_DIG==24,
              "candidate requires IEEE binary32");
inline float minf(float a,float b){return a<b?a:b;}
inline float maxf(float a,float b){return a>b?a:b;}
// For positive normal binary32, exponent-halving gives piecewise linear
// upper approximation to sqrt (maximum relative excess sqrt(9/8)-1).
// +1 ULP covers truncation of the odd low bit. Use only finite nonnegative
// arguments from valid profiles. Slow fallback outside the normal domain.
inline float upperRoot(float x){
    if(x==0.0f)return 0.0f;
    uint32_t bits; memcpy(&bits,&x,4);
    if(bits<0x00800000UL || bits>=0x7f800000UL)return sqrtf(x);
    bits=(bits>>1)+0x1fc00000UL+1;
    float y; memcpy(&y,&bits,4);return y;
}
// Same phase partition as MoveQueue::estimateSeconds; use cheap upper
// speed estimates to obtain a conservative approximation to the analytic
// duration, not a physical execution deadline. Full slots need <=1 root.
template<bool approx>
inline float time(uint32_t total,uint32_t left,uint32_t accel,uint32_t decel,
                  uint32_t acceleration,uint16_t vStart,uint16_t vEnd,uint16_t vMax){
    if(!total || !left || left>total)return 0;
    const uint32_t a=accel<total?accel:total;
    const uint32_t d=decel<total-a?decel:total-a;
    const uint32_t completed=total-left;
    const uint32_t accelerating=completed<a?a-completed:0;
    const uint32_t braking=left<d?left:d;
    const uint32_t cruise=left-accelerating-braking;
    const float s2=float(vStart)*vStart,aa=2.0f*acceleration;
    float peak=minf(float(vMax),31250.0f);
    if(a){const float x=s2+aa*a;peak=minf(peak,approx?upperRoot(x):sqrtf(x));}
    peak=maxf(1.0f,peak);
    float duration=cruise?float(cruise)/peak:0;
    if(accelerating){
        const float x=s2+aa*completed;
        const float from=minf(peak,completed?(approx?upperRoot(x):sqrtf(x)):float(vStart));
        duration+=2.0f*accelerating/(from+peak);
    }
    if(braking){
        const float end=minf(float(vEnd),peak);
        const float x=end*end+(peak*peak-end*end)*(float(braking)/d);
        const float from=braking==d?peak:(approx?upperRoot(x):sqrtf(x));
        duration+=2.0f*braking/(from+end);
    }
    return duration;
}
}

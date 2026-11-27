#pragma once
#include <stdint.h>

namespace ZFeedforwardResearch {
// One active geometric chord. This is a deterministic integer interpolator,
// NOT a physical pulse scheduler. No new PrintLine slots, no live offset
// in the immutable geometric endpoint. All divisions happen in prepare().
// Endpoint range is derived from int16 matrix entries + 13-step zero rule.
class Chord {
    int32_t value_=0,whole_=0;
    uint32_t length_=0,remaining_=0,phase_=0,remainder_=0;
    int8_t sign_=0;
public:
    bool prepare(int32_t first,int32_t last,uint32_t samples) {
        remaining_=0; // invalid preparation cannot continue a stale chord
        if(!samples||samples>INT32_MAX||first<-32768||first>32780||
           last<-32768||last>32780)return false;
        const int32_t d=last-first;
        const uint32_t mag=d<0?uint32_t(-d):uint32_t(d);
        value_=first;length_=remaining_=samples;
        sign_=d<0?-1:1;
        whole_=int32_t(mag/samples)*sign_;
        remainder_=mag%samples;
        // Round ties toward +infinity, independent of travel direction.
        phase_=d<0?(samples-1)/2:samples/2;
        return true;
    }
    int32_t value() const {return value_;}
    uint32_t remaining() const {return remaining_;}
    bool advance() {
        if(!remaining_)return false;
        --remaining_;
        value_+=whole_;
        // samples <= INT32_MAX, phase+remainder < 2*samples: no overflow.
        phase_+=remainder_;
        if(phase_>=length_){phase_-=length_;value_+=sign_;}
        return true;
    }
};
#ifdef __AVR__
static_assert(sizeof(Chord)==25,"AVR active chord size");
#endif
}

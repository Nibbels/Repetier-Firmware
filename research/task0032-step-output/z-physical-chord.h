#pragma once
#include <stdint.h>
namespace ZFeedforwardResearch {
// Interpolate the actual geometric motor coordinate N + C once, rather
// than quantizing N and C independently. Dynamic offset remains separate.
// Bounded length permits cheap AVR16 phase arithmetic. Long chords must
// be split in foreground. Not a pulse generator or admission policy.
class PhysicalChord {
    int32_t value_=0;
    int16_t whole_=0;
    uint16_t length_=0,remaining_=0,phase_=0,remainder_=0;
    int8_t sign_=0;
public:
    bool prepare(int32_t first,int32_t last,uint32_t samples){
        remaining_=0;
        if(!samples||samples>32767U)return false;
        const bool positive=last>=first;
        const uint32_t mag=positive?uint32_t(last)-uint32_t(first):
                                      uint32_t(first)-uint32_t(last);
        const uint32_t whole=mag/samples;
        if(whole>32767U)return false;
        sign_=positive?1:-1;whole_=int16_t(whole)*sign_;
        length_=remaining_=uint16_t(samples);
        remainder_=uint16_t(mag%samples);
        phase_=positive?length_/2:(length_-1)/2;
        value_=first;return true;
    }
    bool advance(){
        if(!remaining_)return false;
        --remaining_;value_+=whole_;
        phase_+=remainder_; // <=65532, no uint16 overflow
        if(phase_>=length_){phase_-=length_;value_+=sign_;}
        return true;
    }
    int32_t value() const{return value_;}
    uint16_t remaining() const{return remaining_;}
};
#ifdef __AVR__
static_assert(sizeof(PhysicalChord)==15,"compact physical chord AVR size");
#endif
}

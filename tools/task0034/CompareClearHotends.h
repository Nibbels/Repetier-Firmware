#pragma once
#include <stdint.h>

// Task0034 research only. A hardware compare CLEAR ends the pulse without
// any interrupt. The AVR OC latch is raised while the physical pin is input;
// only after CLEAR mode and a valid deadline are installed is DDR enabled.
// REQUIRES independently verified OFF bias while DDR=input. Do not use on
// an unknown board, or assume this protects against a failed power switch.
// See Task0034 A5/A19 and the ATmega2560 normal-mode compare/FOC semantics.
// The caller owns IRQ exclusion, fresh heater authorization and frame cadence.
// Timer2 is exclusively reserved for this prototype; both channels share it.
template<class Timer, uint8_t Channels>
class CompareClearHotends {
    static_assert(Channels==1 || Channels==2, "Timer2 has only two OC channels");
    Timer& timer;
    uint8_t active=0;
public:
    explicit CompareClearHotends(Timer& t):timer(t){}
    void revoke(uint8_t c) {
        if(c>=Channels) return;
        // PORT must be LOW before dropping the peripheral override.
        timer.portLow(c);
        timer.disconnect(c);
        timer.drive(c);
        active &= uint8_t(~(1u<<c));
    }
    void revokeAll() {for(uint8_t c=0;c<Channels;++c) revoke(c);}
    // One pulse per 16.384ms half-frame, duration in 64us ticks.
    // 254/256 is intentionally bounded even for a request of 255.
    // A late frame never extends an unfinished pulse and never catches up.
    // False means no new pulse was committed, not necessarily a fault.
    bool pulse(uint8_t c,uint8_t duty) {
        if(c>=Channels) return false;
        const uint8_t width=duty&0xfe;
        if(!width) {revoke(c);return false;}
        const uint8_t bit=uint8_t(1u<<c);
        if((active&bit) && !timer.pending(c)) return false;
        timer.portLow(c);
        timer.floatOff(c);
        // Never expose SET mode to a driven heater output.
        timer.setMode(c);
        timer.force(c);
        timer.clearMode(c);
        const uint8_t now=timer.counter();
        const uint8_t deadline=uint8_t(now+width);
        timer.deadline(c,deadline);
        // Only inactive/physically isolated channel may clear its flag.
        timer.ack(c);
        const uint8_t remaining=uint8_t(deadline-timer.counter());
        if(remaining<=1 || remaining>width || timer.pending(c)) {
            revoke(c);
            return false;
        }
        active |= bit;
        // From here on the timer can only clear, never set, its OC latch.
        // A crash at any earlier instruction leaves the pin OFF by bias.
        timer.drive(c);
        return true;
    }
};

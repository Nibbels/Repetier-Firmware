#pragma once
#include <stdint.h>

// Isolated candidate for the existing runtime LCD re-init command. NOT
// integrated into the shipping or normal-DDRAM candidate. The owner must
// exclude ordinary transfers until completion and mark every logical cell
// dirty after DDRAM clear. No delay calls, no scheduler recursion, no ISR.
//
// The phase/timestamp can later reuse the normal transport's address and
// timestamp while it is suspended. Do not allocate a second framebuffer.
struct LcdResetSequence {
    static constexpr uint8_t inactive=0xff;
    static uint32_t minimumDelay(uint8_t phase) {
        // Preserve the actual old init sequence, including the 320 us
        // pre-byte delay and the 2 ms clear delay. Add four microseconds
        // for the original Arduino micros quantization (nominal 16 MHz).
        switch(phase) {
        case 0: return 500004; // input pins -> output setup
        case 1: return 14;     // setup -> first 8-bit mode nibble
        case 2: return 5004;
        case 3: case 4: return 154;
        case 5: return 474;    // 150 + 320 before first complete byte
        case 7: return 2324;   // clear 2000 + 320
        default: return 324;   // ordinary bytes and final settle
        }
    }

    template<class Sink>
    static void start(uint8_t& phase,uint32_t& completedAt,Sink& sink) {
        sink.inputs();
        completedAt=sink.now();
        phase=0;
    }

    // One bounded operation per visit. Pin direction/initial setup are
    // short groups; all actual LCD nibble and byte transfers are singular.
    // The sink owns exact port masks and the original glyph Flash reads.
    template<class Sink>
    static bool advance(uint8_t& phase,uint32_t& completedAt,Sink& sink) {
        if(phase==inactive) return false;
        const uint32_t elapsed=uint32_t(sink.now()-completedAt);
        if(elapsed<minimumDelay(phase) || elapsed>=0x80000000UL)
            return false;
        switch(phase) {
        case 0: sink.outputs(); break;
        case 1: case 2: case 3: sink.nibble(3); break;
        case 4: sink.nibble(2); break;
        case 5: sink.write(0x28,false); break;
        case 6: sink.write(0x01,false); break;
        case 7: sink.write(0x06,false); break;
        case 8: sink.write(0x0c,false); break;
        case 72: // all seven normal glyph slots complete; allow next owner
            phase=inactive;
            return true;
        default: {
            const uint8_t i=phase-9;
            const uint8_t slot=1+i/9;
            const uint8_t row=i%9;
            if(row==0) sink.write(uint8_t(0x40+(slot<<3)),false);
            else sink.write(sink.glyph(slot,row-1),true);
            break;
        }
        }
        ++phase;
        completedAt=sink.now();
        return true;
    }
};


// Standalone owner used by the exact historical init trace replay.
// The same sequence can also use a suspended transport's cursor and
// deadline by reference without another persistent reset object.
class LcdRuntimeReset {
    uint32_t completedAt_=0;
    uint8_t phase_=LcdResetSequence::inactive;
public:
    bool active() const { return phase_!=LcdResetSequence::inactive; }
    template<class Sink>
    bool begin(Sink& sink) {
        if(active()) return false;
        LcdResetSequence::start(phase_,completedAt_,sink);
        return true;
    }
    template<class Sink>
    bool service(Sink& sink) {
        return LcdResetSequence::advance(phase_,completedAt_,sink);
    }
};

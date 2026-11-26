// Task0034 research prototype, not linked into printer firmware.
// Timer5 normal mode, /8, 65536 ticks/frame. OC pins remain GPIO.
// All calls serialized with I=0. Timer and authorization guard bounds must
// be verified on linked code. No guarantee of physical off with I=0 forever.
#pragma once
#include <stdint.h>

template<class Timer, class Lease, uint8_t Channels, uint16_t Guard, uint16_t FullOffGap = Guard>
class EventHeaterPwm {
    static_assert(Channels > 0 && Channels <= 3, "compare channels");
    static_assert(Guard > 0 && Guard < 512, "prototype arm guard");
    static_assert(FullOffGap >= Guard && FullOffGap <= 1024, "prototype full-duty off gap");
    Timer& timer_;
    Lease& lease_;
    volatile uint8_t active_ = 0;
    volatile bool failed_ = false;

    static uint8_t bit(uint8_t c) { return uint8_t(1u << c); }
    void off(uint8_t c) {
        timer_.off(c);
        active_ &= uint8_t(~bit(c));
        timer_.disable(c);
    }
    void fail() {
        failed_ = true;
        for (uint8_t c = 0; c < Channels; ++c) off(c);
        lease_.inhibit();
    }
public:
    EventHeaterPwm(Timer& timer, Lease& lease) : timer_(timer), lease_(lease) {}
    static uint16_t width(uint8_t demand) {
        const uint8_t quantized = demand & 254;
        // Preserve stock non-full duty; bound nominal full-duty high interval.
        return quantized == 254 ? uint16_t(0u - FullOffGap) : uint16_t(quantized) << 8;
    }
    bool failed() const { return failed_; }

    // Explicit revoke must be sent to both authorization and edge owner.
    // Does not clear a latched fault.
    void revoke(uint8_t c) {
        if (c >= Channels) return;
        // Do not discard evidence of an overdue pulse at revoke.
        if ((active_ & bit(c)) && timer_.wrapped()) { fail(); return; }
        off(c);
        if (!lease_.authorize(c, 0)) fail();
    }

    // Called on overflow vector entry, which has cleared TOV. A still-active
    // pulse at this point crossed a frame boundary without its OFF service.
    // Frame admission also requires live independent Timer3 authorization.
    void frame() {
        if (failed_ || active_ || timer_.wrapped()) { fail(); return; }
        uint8_t duties[Channels];
        if (!lease_.frameDuties(duties)) { fail(); return; }
        for (uint8_t c = 0; c < Channels; ++c) {
            const uint16_t end = width(duties[c]);
            if (timer_.wrapped()) { fail(); return; }
            if (!end) continue;
            const uint16_t now = timer_.now();
            if (now >= end || uint16_t(end - now) <= Guard) continue;
            // Channel is inactive/off; stale OCF from previous frames is
            // harmless. Arm absolute falling edge before GPIO HIGH.
            timer_.writeDeadline(c, end);
            timer_.clearPending(c);
            const uint16_t after = timer_.now();
            if (timer_.wrapped()) { fail(); return; }
            if (timer_.pending(c) || after >= end || uint16_t(end - after) <= Guard)
                continue;
            active_ |= bit(c);
            timer_.enable(c);
            timer_.on(c);
        }
        if (timer_.wrapped()) fail();
    }

    // Compare flag has been cleared by vector entry. If TOV is pending,
    // the OFF was delayed across wrap; latch, don't start another frame.
    void compare(uint8_t c) {
        if (c >= Channels) return;
        if ((active_ & bit(c)) && timer_.wrapped()) { fail(); return; }
        off(c);
    }
    void inhibit() { fail(); }
};

// Task0034 research prototype. Not linked into printer firmware.
// Caller serializes every operation against IRQs and other callers.
// Hardware adapter supplies a free-running 16-bit counter and sticky compare
// flags, and owns the GPIO clear and per-channel compare IRQ masks.
// Snapshot-to-commit must be bounded to Guard ticks on real linked AVR code.
// No physical-off guarantee if CPU is stalled: requires independent OC/reset.
#pragma once
#include <stdint.h>

template<class Timer, uint8_t Channels, uint16_t Duration, uint16_t Guard>
class HeaterLease {
    static_assert(Channels > 0 && Channels <= 3, "one compare per heater");
    static_assert(Duration > Guard && Duration < 32768, "unambiguous interval");
    Timer& timer_;
    uint8_t duty_[Channels] = {};
    bool failed_ = false;

    bool live(uint8_t channel, uint16_t now) const {
        const uint16_t left = uint16_t(timer_.deadline(channel) - now);
        return left > Guard && left <= Duration && !timer_.pending(channel);
    }
    bool checkAll() {
        if (failed_) return false;
        for (uint8_t c = 0; c < Channels; ++c) {
            if (duty_[c] && !live(c, timer_.now())) {
                fail();
                return false;
            }
        }
        return true;
    }
    void stop(uint8_t channel) {
        timer_.off(channel);
        duty_[channel] = 0;
        timer_.disable(channel);
        // No active lease remains; stale flags are cleared only on initial arm.
    }
    void fail() {
        failed_ = true;
        for (uint8_t c = 0; c < Channels; ++c) stop(c);
    }
public:
    explicit HeaterLease(Timer& timer) : timer_(timer) {}
    bool failed() const { return failed_; }

    // Called only after a fresh, completed thermal check, not from PWM.
    bool authorize(uint8_t channel, uint8_t demand) {
        if (channel >= Channels || !checkAll()) return false;
        const uint16_t now = timer_.now();
        const bool active = duty_[channel] != 0;
        if (active && !live(channel, now)) {
            fail();
            return false;
        }
        if (!demand) {
            stop(channel);
            return true;
        }
        if (!active) {
            // No physical HIGH while preparing an inactive channel.
            stop(channel);
            timer_.writeDeadline(channel, uint16_t(now + Duration));
            timer_.clearPending(channel);
        } else {
            // Never clear an active compare flag during renewal.
            timer_.writeDeadline(channel, uint16_t(now + Duration));
        }
        if (!live(channel, timer_.now())) {
            fail();
            return false;
        }
        duty_[channel] = demand;
        timer_.enable(channel);
        return true;
    }

    // Admission at frame start: a pending expiry is a failure even if the
    // compare ISR has lower priority than the PWM handler.
    uint8_t frameDuty(uint8_t channel) {
        if (channel >= Channels || !checkAll() || !duty_[channel]) return 0;
        if (!live(channel, timer_.now())) {
            fail();
            return 0;
        }
        return duty_[channel];
    }

    // Hardware clears compare flag on vector entry. Do not require the flag
    // to still be set; an enabled compare IRQ is itself expiry evidence.
    void expired(uint8_t channel) {
        if (channel < Channels && duty_[channel]) fail();
    }

    void inhibit() { fail(); } // reset-only recovery for this prototype
};

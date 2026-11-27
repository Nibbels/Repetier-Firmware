#ifndef RF_RESEARCH_MOTOR_WAKE_H
#define RF_RESEARCH_MOTOR_WAKE_H
#include <stdint.h>

// Pure lifecycle candidate. No production call site yet. All state changes
// are serialized with the actual SLEEPn GPIO transaction by the caller.
// The foreground owns deadline/service; pulse owners may read the ready byte
// only under a protocol that prevents concurrent disable/revocation.
class MotorWake {
public:
    using Mask = uint8_t;
    static constexpr Mask physical = 0x1f; // X,Y,Z,E0,E1; caller filters fitted.
    // Arduino AVR millis uses 1024us Timer0 overflows with occasional +2ms
    // increments (fractional correction). 26 reported ms can be only
    // 24*1024us if wake falls just before the first overflow. 27 reported
    // ms requires at least 26 overflows, hence >=25*1024us nominal elapsed.
    // Keep historical physical wait, tolerate late service, no spin.
    static constexpr uint32_t settleTicks = 27;
    Mask newChannels(Mask requested) const {
        return requested & physical & ~awake_;
    }
    // Call AFTER all newly selected SLEEPn signals have actually risen.
    // Repeated enable of already awake channels does not reset the deadline.
    void afterAsserted(Mask asserted, uint32_t now) {
        const Mask newly = newChannels(asserted);
        if (!newly) return;
        awake_ |= newly;
        // A shared pending-batch deadline conservatively delays earlier pending
        // channels if another joins. Already ready axes are never delayed.
        deadline_ = now + settleTicks;
    }
    void service(uint32_t now) {
        if (awake_ != ready_ && uint32_t(now - deadline_) < UINT32_C(0x80000000))
            ready_ = awake_;
    }
    // Clear eligibility BEFORE lowering SLEEPn, and invalidate on reset/fault.
    void beforeDisable(Mask channels) {
        ready_ &= ~channels;
        awake_ &= ~channels;
    }
    bool readyFor(Mask requested) const {
        return (requested & ready_) == requested;
    }
    Mask readyMask() const { return ready_; }
    Mask awakeMask() const { return awake_; }
    void reset() { ready_ = awake_ = 0; deadline_ = 0; }
private:
    uint32_t deadline_ = 0;
    uint8_t awake_ = 0;
    volatile uint8_t ready_ = 0;
};
#ifdef __AVR__
static_assert(sizeof(MotorWake)==6,"AVR prototype state");
#endif
#endif

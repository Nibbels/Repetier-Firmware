// Research only. Caller serializes each mutation and snapshot with the
// existing queue publication/retirement transaction. No interrupt control here.
// Slots must be published before replace/retire; reset drops all membership.
#pragma once
#include <stdint.h>
#include <math.h>

namespace PlannerEstimateResearch {
inline uint32_t durationUnits(float seconds) {
    // 0xffffffff is reserved as unavailable. Never saturate silently.
    const float scaled = seconds * 10000.0f;
    if (!(scaled >= 0.0f && scaled < 4294967296.0f))
        return UINT32_MAX;
    // Round to nearest without adding 0.5 to a large binary32 value.
    const uint32_t whole = static_cast<uint32_t>(scaled);
    if (whole == UINT32_MAX) return UINT32_MAX;
    const uint32_t rounded = whole + (scaled - float(whole) >= 0.5f);
    return rounded;
}

template<uint8_t Capacity>
class DurationLedger {
    static_assert(Capacity > 0, "queue must have slots");
    volatile uint32_t slot_[Capacity]{};
    volatile uint64_t sum_ = 0;
    volatile uint8_t unavailable_ = 0;
    void add(uint32_t value) {
        if (value == UINT32_MAX) ++unavailable_;
        else sum_ += value;
    }
    void subtract(uint32_t value) {
        if (value == UINT32_MAX) --unavailable_;
        else sum_ -= value;
    }
public:
    struct Snapshot {
        uint64_t full;
        uint32_t front;
        uint8_t unavailable;
    };
    void publish(uint8_t index, uint32_t value) {
        slot_[index] = value;
        add(value);
    }
    void replace(uint8_t index, uint32_t value) {
        const uint32_t old = slot_[index];
        if (old == value) return;
        subtract(old);
        slot_[index] = value;
        add(value);
    }
    void retire(uint8_t index) {
        subtract(slot_[index]);
        // No need to clear: publish overwrites before membership begins.
    }
    void clear() {
        // O(1); old slot values are not members after reset.
        sum_ = 0;
        unavailable_ = 0;
    }
    Snapshot snapshot(uint8_t front, bool nonempty) const {
        return {sum_, nonempty ? slot_[front] : uint32_t(0), unavailable_};
    }
    // Same units, active tail computed from a coherent front profile/progress
    // snapshot outside the IRQ lock. False means estimate unavailable.
    static bool remaining(Snapshot s, bool active, uint32_t tail, uint64_t& result) {
        if (s.unavailable || (active && (tail == UINT32_MAX || s.front > s.full)))
            return false;
        result = active ? s.full - s.front + tail : s.full;
        return true;
    }
};
}

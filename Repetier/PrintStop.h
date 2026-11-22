#ifndef PRINT_STOP_H
#define PRINT_STOP_H

#include <stdint.h>

class PrintStop {
public:
    bool commandsBlocked() const { return phase_ != Phase::Idle; }

    // Called with interrupts masked before the planner and pause are reset.
    void begin(uint32_t now) {
        phaseSince_ = now;
        phase_ = Phase::Draining;
    }

    void service(uint32_t now);

private:
    enum class Phase : uint8_t { Idle, Draining, WaitingToOutput };

    volatile Phase phase_ = Phase::Idle;
    uint32_t phaseSince_ = 0;
};

#endif // PRINT_STOP_H

#ifndef IDLE_STATUS_H
#define IDLE_STATUS_H

#include <stdint.h>

// Owns the delayed transition from a motion, heat or service message to Ready.
class IdleStatus {
public:
    enum class Transition : uint8_t { Waiting, Deferred, Ready };

    void cancel() { pending_ = false; }
    void schedule(uint32_t startAt);
    Transition poll(uint32_t now, uint32_t idleDelay);

private:
    volatile uint32_t startAt_ = 0;
    volatile bool pending_ = false;
};

#endif

#include "Repetier.h"

void IdleStatus::schedule(uint32_t startAt) {
    InterruptProtectedBlock lock;
    startAt_ = startAt;
    pending_ = true;
}

IdleStatus::Transition IdleStatus::poll(uint32_t now, uint32_t idleDelay) {
    InterruptProtectedBlock lock;
    if (!pending_)
        return Transition::Waiting;

    const int32_t elapsed = static_cast<int32_t>(now - startAt_);
    if (elapsed < 0)
        return Transition::Deferred;
    if (static_cast<uint32_t>(elapsed) <= idleDelay)
        return Transition::Waiting;

    pending_ = false;
    return Transition::Ready;
}

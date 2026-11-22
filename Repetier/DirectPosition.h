#ifndef DIRECT_POSITION_H
#define DIRECT_POSITION_H

#include <stdint.h>

// Direct motion has its own origin. A clipped move leaves the target at the
// displacement actually reached. Multi-byte accesses use the caller's timer lock.
class DirectPosition {
public:
    int32_t steps() const { return steps_; }
    int32_t target() const { return target_; }
    int32_t remaining() const { return target_ - steps_; }

    void request(int32_t distance) { target_ = steps_ + distance; }
    void advance(int8_t direction) { steps_ += direction; }
    void stop() { target_ = steps_; }
    void reset() { target_ = steps_ = 0; }

private:
    volatile int32_t steps_ = 0;
    volatile int32_t target_ = 0;
};

#endif // DIRECT_POSITION_H

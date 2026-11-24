#ifndef Z_SCREW_ADJUSTMENT_H
#define Z_SCREW_ADJUSTMENT_H

#include <stdint.h>

namespace ZScrewAdjustment {

enum class Status : uint8_t { Unmeasured, Ok, BedTooHigh };
struct Advice {
    float millimetres;
    float turns;
    Status status;
};

const Advice& current();
void calculate();

} // namespace ZScrewAdjustment

#endif

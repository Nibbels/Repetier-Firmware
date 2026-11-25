#ifndef MANUAL_MOTION_H
#define MANUAL_MOTION_H

#include <stdint.h>
#include "Constants.h"

#define MOVE_MODE_SINGLE_STEPS 1
#define MOVE_MODE_1_MM 2
#define MOVE_MODE_10_MM 3
#define MOVE_MODE_50_MM 4
#define MOVE_MODE_SINGLE_MOVE 5

// Foreground direct-move admission and feedrate selection, not a queued command.
enum class DirectMoveMode : uint8_t { Configured = 0, Button = 10, ManualPosition = 11 };

class ManualMotion {
public:
    constexpr ManualMotion(uint16_t x, uint16_t y, uint16_t z, uint16_t e)
        : steps_{x, y, z, e} {}

    uint16_t steps(uint8_t axis) const { return steps_[axis]; }
    void setSteps(uint8_t axis, uint16_t steps) { steps_[axis] = steps; }
    void rescaleSteps(uint8_t axis, float ratio) { steps_[axis] *= ratio; }

    char mode(uint8_t axis) const { return modes_[axis]; }
    void setMode(uint8_t axis, char mode) { modes_[axis] = mode; }
    void cycleMode(uint8_t axis) {
        if (++modes_[axis] > MOVE_MODE_SINGLE_MOVE)
            modes_[axis] = MOVE_MODE_SINGLE_STEPS;
    }

    // True selects the G-code coordinate system or feedrate.
    bool coordinateSystem() const { return coordinateSystem_; }
    void setCoordinateSystem(bool value) { coordinateSystem_ = value; }
    bool feedrateSource() const { return feedrateSource_; }
    void setFeedrateSource(bool value) { feedrateSource_ = value; }

private:
    uint16_t steps_[4];
    char modes_[3]{};
    bool coordinateSystem_ = true;
    bool feedrateSource_ = false;
};

#endif

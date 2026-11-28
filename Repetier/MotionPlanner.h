/*
    This file is part of the Repetier-Firmware for RF devices from Conrad Electronic SE.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <stdint.h>

class MoveQueue;
class PrintLine;

class MotionPlanner {
public:
    struct AdvancePeak {
        int steps;
        float extrusionSpeed;
    };

    static AdvancePeak takeAdvancePeak();
    static void prepareMove(PrintLine& move, const float axisDistanceMM[4], float feedrate, const MoveQueue& queue);
    static void prepareStepProfile(PrintLine& move);
    static void updateLookahead(MoveQueue& queue);

private:
    static AdvancePeak advancePeak_;
    static float safeSpeed(PrintLine& move, int8_t drivingAxis);
    static void limitJunction(PrintLine& previous, const PrintLine& next);
    static void planDeceleration(MoveQueue& queue, uint8_t start, uint8_t first);
    static void planAcceleration(MoveQueue& queue, uint8_t first, uint8_t end);
};

#endif

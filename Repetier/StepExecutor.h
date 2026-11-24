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

#ifndef STEP_EXECUTOR_H
#define STEP_EXECUTOR_H

#include <stdint.h>

class PrintLine;

class StepExecutor {
public:
    static constexpr uint16_t minimumStepIntervalTicks = 512;

    static float nominalFeedrate();
    static uint8_t stepsPerInterrupt() { return stepsPerTimerCall; }
    // Foreground configuration; the timer reads the published 16-bit value.
    static uint16_t packingInterval() { return packingInterval_; }
    static void setPackingInterval(uint16_t interval);
    static void stepSlowedZCompensation();
    static bool needCmpWait();
    static long performQueueMove();
    static long performDirectMove();

private:
    struct StepClock {
        uint32_t ticks = 0;
        uint32_t steps = 0;
        uint16_t peak = 0;

        void start(uint16_t speed) {
            peak = speed;
            steps = 0;
            ticks = 0;
        }
    };
    enum class DirectionOwner : int8_t { None, Direct = -1, Queued = 1 };
    static StepClock stepClocks[2];
    static DirectionOwner directionOwner;
    static volatile uint32_t stepIntervalTicks;
    static volatile uint16_t packingInterval_;
    static uint8_t stepsPerTimerCall;
    static volatile int32_t queuedPrimarySteps;
    static int32_t directPrimarySteps;
    static volatile float nominalFeedrate_;

    static void enableSteppers(PrintLine& move);
    static void adjustDirections(PrintLine& move);
    static void updateAdvanceSteps(const PrintLine& move, uint16_t speed);

    static uint16_t nextStepSpeed(PrintLine& move, StepClock& clock, uint8_t pulses, uint32_t interval);
    static long performMove(PrintLine* move, uint8_t forQueue);
    static void finishMove(PrintLine& move, bool queued);
};

#endif

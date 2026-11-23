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

#include "Repetier.h"
#include "MotionPlanner.h"

// ================ Sanity checks ================
#ifndef STEP_PACKING_MIN_INTERVAL
#error Please add new parameter STEP_PACKING_MIN_INTERVAL to your configuration.
#else
#if STEP_PACKING_MIN_INTERVAL < MIN_STEP_PACKING_MIN_INTERVAL || STEP_PACKING_MIN_INTERVAL > MAX_STEP_PACKING_MIN_INTERVAL
#error STEP_PACKING_MIN_INTERVAL should be in range of MIN_STEP_PACKING_MIN_INTERVAL .. MAX_STEP_PACKING_MIN_INTERVAL
#endif // STEP_PACKING_MIN_INTERVAL<MIN_STEP_PACKING_MIN_INTERVAL || STEP_PACKING_MIN_INTERVAL>MAX_STEP_PACKING_MIN_INTERVAL
#endif // STEP_PACKING_MIN_INTERVAL

#ifdef EXTRUDER_SPEED
#error EXTRUDER_SPEED is not used any more. Values are now taken from extruder definition.
#endif // EXTRUDER_SPEED

#ifdef ENDSTOPPULLUPS
#error ENDSTOPPULLUPS is now replaced by individual pullup configuration!
#endif // ENDSTOPPULLUPS

#ifdef EXT0_PID_PGAIN
#error The PID system has changed. Please use the new float number options!
#endif // EXT0_PID_PGAIN

// ####################################################################################
// ##   No configuration below this line - just some errorchecking
// ####################################################################################
#if X_STEP_PIN < 0 || Y_STEP_PIN < 0 || Z_STEP_PIN < 0
#error One of the following pins is not assigned: X_STEP_PIN,Y_STEP_PIN,Z_STEP_PIN
#endif // X_STEP_PIN<0 || Y_STEP_PIN<0 || Z_STEP_PIN<0

#if EXT0_STEP_PIN < 0
#error EXT0_STEP_PIN not set to a pin number.
#endif // EXT0_STEP_PIN<0 && NUM_EXTRUDER>0

#if EXT0_DIR_PIN < 0
#error EXT0_DIR_PIN not set to a pin number.
#endif // EXT0_DIR_PIN<0 && NUM_EXTRUDER>0

#if MOVE_CACHE_SIZE < 5
#error MOVE_CACHE_SIZE must be at least 5
#endif // MOVE_CACHE_SIZE<5

// Inactivity shutdown variables
millis_t previousMillisCmd = 0;
millis_t maxInactiveTime = MAX_INACTIVE_TIME * 1000L;
millis_t stepperInactiveTime = STEPPER_INACTIVE_TIME * 1000L;
long baudrate = BAUDRATE; // Communication speed rate.

uint8_t pwm_pos[NUM_EXTRUDER + 3]; // 0-NUM_EXTRUDER = Heater 0-NUM_EXTRUDER of extruder, NUM_EXTRUDER = Heated bed, NUM_EXTRUDER+1 Board fan, NUM_EXTRUDER+2 = Fan

uint8_t fanSpeed = 0;

MoveQueue PrintLine::queue;
PrintLine PrintLine::direct;

bool PrintLine::hasActiveZMove() {
    InterruptProtectedBlock lock;
    if (direct.task >= DIRECT_PREPARED && direct.stepsRemaining && direct.isZMove())
        return true;
    PrintLine* active = queue.active();
    return active && active->isZMove();
}

void PrintLine::setAxisSteps(uint8_t axis, int32_t steps) {
    if (steps >= 0)
        setPositiveDirectionForAxis(axis);
    else
        steps = -steps;
    delta[axis] = steps;
    if (steps)
        setMoveOfAxis(axis);
}

void PrintLine::setSegmentGeometry(const float axisDistanceMM[]) {
    // The largest step count drives Bresenham, with ties ordered E, Z, X, Y.
    if (delta[Y_AXIS] > delta[X_AXIS] && delta[Y_AXIS] > delta[Z_AXIS] && delta[Y_AXIS] > delta[E_AXIS])
        primaryAxis = Y_AXIS;
    else if (delta[X_AXIS] > delta[Z_AXIS] && delta[X_AXIS] > delta[E_AXIS])
        primaryAxis = X_AXIS;
    else if (delta[Z_AXIS] > delta[E_AXIS])
        primaryAxis = Z_AXIS;
    else
        primaryAxis = E_AXIS;
    stepsRemaining = delta[primaryAxis];

    lookahead.distance = axisDistanceMM[E_AXIS];
    if (isXYZMove()) {
        float distanceSquared = axisDistanceMM[X_AXIS] * axisDistanceMM[X_AXIS]
            + axisDistanceMM[Y_AXIS] * axisDistanceMM[Y_AXIS];
        if (isZMove())
            distanceSquared += axisDistanceMM[Z_AXIS] * axisDistanceMM[Z_AXIS];
        lookahead.distance = RMath::max((float)sqrt(distanceSquared), lookahead.distance);
    }
}

/** \brief Put a move to the current destination coordinates into the movement cache.
  If the cache is full, the method will wait, until a place gets free. During
  wait communication and temperature control is enabled.

  @param abortAtEndstops Stop this move at any endstop during move.
   Otherwise the endstop causes an ignore of moving that axis but coordinates will move
  @param updateWobble Recalculate the XY correction for a G-code XYZ move.
 */
void PrintLine::prepareQueueMove(uint8_t abortAtEndstops, uint8_t pathOptimize, float feedrate, bool updateWobble) {
    Printer::unmarkAllSteppersDisabled(); // ??? hier wird nichts enabled. Nur markiert, auch wenn später oder früher "enablestepper" passiert.
    //evtl. weil dadurch in jedem fall gleich ein stepper aktiviert werden würde -> darum hier schon als aktiv markieren, weil umumgänglich ist.
    // Aber dann müsste man das (timingsicher) auch schon in den Funktionen über prepareDirectMove erledigt haben.

    PrintLine::waitForXFreeLines(1);

    uint8_t newPath = PrintLine::insertWaitMovesIfNeeded(pathOptimize, 0);
    PrintLine* p = &queue.writeSlot();

    p->task = TASK_NO_TASK;
    p->flags = (abortAtEndstops ? FLAG_ABORT_AT_ENDSTOPS : 0);
    p->joinFlags = 0;
    p->dir = 0;
    if (!pathOptimize)
        p->setEndSpeedFixed(true);

    if (updateWobble)
        Printer::updateWobbleFixOffsets();

    float axisDistanceMM[4]; // Axis movement in mm
    // Find direction
    for (uint8_t axis = 0; axis < 4; axis++) {
        int32_t steps;
        if (axis == E_AXIS) {
            const float targetE = Printer::moveTarget.machineMM(E_AXIS);
            const float targetRemainder = Printer::moveTarget.remainderMM(E_AXIS);
            const float flow = Printer::menuExtrusionFactor * Printer::digitCompensation.extrusionFactor();
            steps = Printer::extrusionPlanner.planSteps({targetE, targetRemainder}, flow, Printer::axisStepsPerMM[E_AXIS]);
        } else {
            const int32_t targetSteps = Printer::prepareDestinationSteps(axis);
            steps = Printer::cartesianPlanner.queueTarget(axis, targetSteps);
        }

        const float distanceMM = steps * Printer::axisMMPerSteps[axis];
        if (axis == E_AXIS)
            Printer::filamentPrinted += distanceMM;
        axisDistanceMM[axis] = fabs(distanceMM);
        p->setAxisSteps(axis, steps);
    }

    // need to delete dummy elements, otherwise commands can get locked.
    if (p->isNoMove()) {
        if (newPath)
            discardQueuedMoves();
        // No steps included
        return;
    }

    if (!Printer::startLine.active()) {
        const PrintLayers::PlannedMove move{
            Printer::cartesianPlanner.lastQueuedSteps(Z_AXIS), p->isZMove(), p->isXOrYMove(), p->isEPositiveMove()};
        Printer::layers.recordPlannedMove(move, Printer::axisStepsPerMM[Z_AXIS], g_offsetZCompensationSteps, g_ZCompensationMax);
    }

    p->setSegmentGeometry(axisDistanceMM);

    MotionPlanner::prepareMove(*p, axisDistanceMM, feedrate, queue);

    MotionPlanner::updateLookahead(queue);
    if (pathOptimize)
        Printer::pressureAdvance.postponeRelaxation();

    // Make result permanent
    queue.publish(true);
} // prepareQueueMove

void PrintLine::prepareDirectMove(const int32_t steps[4], bool stoppable, bool feedrateSource) {
    direct.flags = FLAG_ABORT_AT_ENDSTOPS;
    direct.joinFlags = 0;
    direct.fixStartAndEndSpeed();
    direct.dir = 0;

    float axisDistanceMM[4];
    for (uint8_t axis = 0; axis < 4; axis++) {
        direct.setAxisSteps(axis, steps[axis]);
        axisDistanceMM[axis] = fabs(direct.delta[axis] * Printer::axisMMPerSteps[axis]);
    }
    if (direct.isNoMove()) {
        direct.stepsRemaining = 0;
        direct.task = DIRECT_IDLE;
        return;
    }

    direct.setSegmentGeometry(axisDistanceMM);

    float feedrate = (direct.isXOrYMove() ? STANDARD_POSITION_FEEDRATE_XY : direct.isZMove() ? STANDARD_POSITION_FEEDRATE_Z : STANDARD_POSITION_FEEDRATE_E);
    if (feedrateSource == FEEDRATE_GCODE) {
        feedrate = Printer::feedrate;
        // Menu positioning means only one axis moves at once. Hoever this is not valid for continue moves etc. but they should not have gcode feedrates at all.
        if (direct.isZMove() && feedrate > STANDARD_POSITION_FEEDRATE_Z) {
            feedrate = STANDARD_POSITION_FEEDRATE_Z;
        }
    }

    MotionPlanner::prepareMove(direct, axisDistanceMM, feedrate, queue);
    MotionPlanner::prepareStepProfile(direct);

    InterruptProtectedBlock publish;
    if (direct.task == DIRECT_PREPARING_STOPPED)
        direct.limitToStopTail(true);
    direct.task = stoppable ? DIRECT_PREPARED_STOPPABLE : DIRECT_PREPARED;
}

// Call with interrupts masked. During preparation only the phase is shared.
void PrintLine::stopDirectMove() {
    const char phase = direct.task;
    if (directMovePreparing(phase)) {
        direct.task = DIRECT_PREPARING_STOPPED;
        return;
    }
    direct.limitToStopTail(phase == DIRECT_PREPARED_STOPPABLE || phase == DIRECT_PREPARED);
}

void PrintLine::limitToStopTail(bool refitRamp) {
    constexpr int32_t stopTailSteps = 32;
    if (stepsRemaining <= stopTailSteps)
        return;
    stepsRemaining = stopTailSteps;
    if (refitRamp) {
        // A direct move has equal entry and exit speeds. Fitting the earlier
        // symmetric ramp again gives the same result for a shorter move.
        fitRampToRemainingSteps();
    }
}

void PrintLine::fitRampToRemainingSteps() {
    if (static_cast<int32_t>(stepProfile.accelSteps + stepProfile.decelSteps) > stepsRemaining) {
        const uint32_t reduction = (stepProfile.accelSteps + stepProfile.decelSteps - stepsRemaining) >> 1;
        stepProfile.accelSteps -= RMath::min(static_cast<int32_t>(stepProfile.accelSteps), static_cast<int32_t>(reduction));
        stepProfile.decelSteps -= RMath::min(static_cast<int32_t>(stepProfile.decelSteps), static_cast<int32_t>(reduction));
    }
}

/** \brief Check if move is new. If it is insert some dummy moves to allow the path optimizer to work since it does
not act on the first two moves in the queue. The stepper timer will spot these moves and leave some time for
processing.
*/
uint8_t PrintLine::insertWaitMovesIfNeeded(uint8_t pathOptimize, uint8_t waitExtraLines) {
    if (queue.count() == 0
        && Printer::pressureAdvance.relaxDelayElapsed()
        && pathOptimize) // First line after some time - warmup needed
    {
        uint8_t w = 4;
        while (w--) {
            PrintLine* p = &queue.writeSlot();
            p->task = TASK_NO_TASK;
            p->flags = FLAG_WARMUP;
            p->joinFlags = FLAG_JOIN_STEP_PROFILE_READY | FLAG_JOIN_END_FIXED | FLAG_JOIN_START_FIXED;
            p->dir = 0;
            p->setWaitForXLinesFilled(w + waitExtraLines);
            p->setWaitTicks(100000); //repetier changed this from 25k to 100k in https://github.com/repetier/Repetier-Firmware/commit/2385051856278c189d7c1f1fd67acc27825c82ac#diff-11347f18746fb080f5bda21c30428558R1080
            queue.publish(true);
        }
        return 1;
    }
    return 0;
} // insertWaitMovesIfNeeded

void PrintLine::waitForXFreeLines(uint8_t b) {
    while (queue.count() + b > MOVE_CACHE_SIZE) // wait for a free entry in movement cache
    {
        Commands::checkForPeriodicalActions(Processing);
    }
} // waitForXFreeLines

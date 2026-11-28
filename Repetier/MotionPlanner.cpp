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

MotionPlanner::AdvancePeak MotionPlanner::advancePeak_{};

MotionPlanner::AdvancePeak MotionPlanner::takeAdvancePeak() {
    const AdvancePeak peak = advancePeak_;
    advancePeak_ = {};
    return peak;
}

inline float MotionPlanner::safeSpeed(PrintLine& move, int8_t drivingAxis) {
    const auto& lookahead = move.lookahead;
    float xyMin = Printer::xyJerkLimit() * 0.5f;
    float mz = 0;
    float safe(xyMin);
    if (move.isZMove()) {
        mz = Printer::zJerkLimit() * 0.5f;
        if (move.isXOrYMove()) {
            if (fabs(lookahead.speedZ) > mz)
                safe = RMath::min(safe, mz * move.fullSpeed / fabs(lookahead.speedZ));
        } else {
            safe = mz;
        }
    }
    if (move.isEMove()) {
        if (move.isXYZMove())
            safe = RMath::min(safe, 0.5f * Extruder::current->maxEJerk * move.fullSpeed / fabs(lookahead.speedE));
        else
            safe = 0.5f * Extruder::current->maxEJerk; // This is a retraction move
    }

    // A minimum speed keeps the explicit integration stable.
    if (drivingAxis == X_AXIS || drivingAxis == Y_AXIS)
        safe = RMath::max(xyMin, safe);
    else if (drivingAxis == Z_AXIS)
        safe = RMath::max(mz, safe);

    return RMath::min(safe, move.fullSpeed);
} // safeSpeed

static float paceShortMove(float ticks, uint8_t queuedEntries) {
    // This admission policy stretches short segments while the queue refills.
    if (queuedEntries < MOVE_CACHE_LOW && ticks < LOW_TICKS_PER_MOVE)
        ticks += (LOW_TICKS_PER_MOVE - ticks) * 3 / (queuedEntries + 1);
    return ticks;
}

void MotionPlanner::prepareMove(PrintLine& move, const float axisDistanceMM[4], float feedrate, const MoveQueue& queue) {
    const fast8_t drivingAxis = move.primaryAxis;
    auto& lookahead = move.lookahead;
    auto& stepProfile = move.stepProfile;
    if (move.stepsRemaining == 0) { // need at least one step for bresenham
        return;
    }
    float timeForMove = paceShortMove(float(F_CPU) * lookahead.distance / feedrate, queue.count());
    move.timeInTicks = timeForMove;

    // Use the slowest allowed step interval across all moving axes.
    const uint32_t primarySteps = move.stepsRemaining;
    const int32_t requestedInterval = timeForMove / primarySteps;
    const float toTicks = float(F_CPU) / primarySteps;
    int32_t limitInterval = RMath::max(requestedInterval, int32_t(LIMIT_INTERVAL));
    for (uint8_t axis = 0; axis < 4; ++axis) {
        if (move.isMoveOfAxis(axis)) {
            const int32_t axisInterval = axisDistanceMM[axis] * toTicks / Printer::axisMaxFeedrate(axis);
            limitInterval = RMath::max(limitInterval, axisInterval);
        }
    }
    if (limitInterval != requestedInterval)
        timeForMove = float(limitInterval) * primarySteps;

    float inverseTimeS = static_cast<float>(F_CPU) / timeForMove;
    if (move.isXMove()) {
        lookahead.speedX = axisDistanceMM[X_AXIS] * inverseTimeS;
        if (move.isXNegativeMove())
            lookahead.speedX = -lookahead.speedX;
    } else
        lookahead.speedX = 0;
    if (move.isYMove()) {
        lookahead.speedY = axisDistanceMM[Y_AXIS] * inverseTimeS;
        if (move.isYNegativeMove())
            lookahead.speedY = -lookahead.speedY;
    } else
        lookahead.speedY = 0;
    if (move.isZMove()) {
        lookahead.speedZ = axisDistanceMM[Z_AXIS] * inverseTimeS;
        if (move.isZNegativeMove())
            lookahead.speedZ = -lookahead.speedZ;
    } else
        lookahead.speedZ = 0;
    if (move.isEMove()) {
        lookahead.speedE = axisDistanceMM[E_AXIS] * inverseTimeS;
        if (move.isENegativeMove())
            lookahead.speedE = -lookahead.speedE;
    } else
        lookahead.speedE = 0;
    move.fullSpeed = lookahead.distance * inverseTimeS;
    // A secondary axis takes axisSteps / primarySteps of the primary
    // acceleration. Compare exact products and round the allowance down.
    const uint32_t* axisAcceleration = Printer::accelerationSteps(move.isEPositiveMove());
    uint32_t primaryAcceleration = axisAcceleration[drivingAxis];
    for (uint8_t axis = 0; axis < 4; ++axis) {
        const uint32_t axisSteps = move.delta[axis];
        if (!axisSteps || axisAcceleration[axis] >= primaryAcceleration)
            continue;

        const uint64_t allowance = uint64_t(axisAcceleration[axis]) * primarySteps;
        if (uint64_t(primaryAcceleration) * axisSteps > allowance) {
            primaryAcceleration = allowance <= UINT32_MAX
                ? uint32_t(allowance) / axisSteps : allowance / axisSteps;
        }
    }

    // Bresenham phase is now initialized by the executor when the move starts.

    lookahead.invFullSpeed = 1.0 / move.fullSpeed;
    lookahead.accelerationPrim = primaryAcceleration;

    // Scale the acceleration for the timer's fixed-point speed calculation.
    stepProfile.fAcceleration = 262144.0 * (float)lookahead.accelerationPrim / F_CPU;
    lookahead.accelerationDistance2 = 2.0f * primaryAcceleration * RMath::sqr(lookahead.distance) / primarySteps;
    lookahead.startSpeed = lookahead.endSpeed = lookahead.minSpeed = safeSpeed(move, drivingAxis);
    if (lookahead.startSpeed > feedrate) {
        lookahead.startSpeed = lookahead.endSpeed = lookahead.minSpeed = feedrate;
    }
    // Can accelerate to full speed within the line
    if (lookahead.startSpeed * lookahead.startSpeed + lookahead.accelerationDistance2 >= move.fullSpeed * move.fullSpeed)
        move.setNominalMove();

    stepProfile.vMax = F_CPU / limitInterval;

    if (!move.isXYZMove() || !move.isEPositiveMove()) { // No head move or E move only or sucking filament back
        stepProfile.advanceL = 0;
    } else {
        float advlin = fabs(lookahead.speedE) * Extruder::current->advanceL * 0.001 * Printer::stepsPerMm(E_AXIS);
        stepProfile.advanceL = ((65536L * advlin) / stepProfile.vMax);
        if (advlin > advancePeak_.steps) {
            advancePeak_.steps = advlin;
            advancePeak_.extrusionSpeed = fabs(lookahead.speedE);
        }
    }

}

// Convert planned entry and exit speeds to a step profile.
void MotionPlanner::prepareStepProfile(PrintLine& move) {
    auto& stepProfile = move.stepProfile;
    const auto& lookahead = move.lookahead;
    if (move.isStepProfileReady() || move.isWarmUp())
        return;

    float startFactor = lookahead.startSpeed * lookahead.invFullSpeed;
    float endFactor = lookahead.endSpeed * lookahead.invFullSpeed;
    stepProfile.vStart = stepProfile.vMax * startFactor; // starting speed
    stepProfile.vEnd = stepProfile.vMax * endFactor;

    uint32_t vmax2 = HAL::U16SquaredToU32(stepProfile.vMax);

    if (stepProfile.vStart == stepProfile.vMax) {
        stepProfile.accelSteps = 0;
    } else {
        stepProfile.accelSteps = ((vmax2 - HAL::U16SquaredToU32(stepProfile.vStart)) / (lookahead.accelerationPrim << 1)) + 1; // Always add 1 for missing precision
    }
    if (stepProfile.vEnd == stepProfile.vMax) {
        stepProfile.decelSteps = 0;
    } else {
        stepProfile.decelSteps = ((vmax2 - HAL::U16SquaredToU32(stepProfile.vEnd)) / (lookahead.accelerationPrim << 1)) + 1;
    }

    move.fitRampToRemainingSteps();
    move.markStepProfileReady();
}

inline void MotionPlanner::limitJunction(PrintLine& previous, const PrintLine& next) {
    auto& from = previous.lookahead;
    const auto& to = next.lookahead;
    // The XY limit weights the angle between the two moves by the previous move's length.
    float factor = 1.0;
    float lengthFactor = 1.0;
    if (from.distance < MAX_JERK_DISTANCE)
        lengthFactor = static_cast<float>(MAX_JERK_DISTANCE * MAX_JERK_DISTANCE) / (from.distance * from.distance);
    float maxJoinSpeed = RMath::min(next.fullSpeed, previous.fullSpeed);

    const float calculatedJerk = maxJoinSpeed * lengthFactor
        * (1.0 - (to.speedX * from.speedX + to.speedY * from.speedY + to.speedZ * from.speedZ)
            / (next.fullSpeed * previous.fullSpeed));

    if (calculatedJerk > Printer::xyJerkLimit()) {
        factor = Printer::xyJerkLimit() / calculatedJerk;
        if (factor * maxJoinSpeed * 2.0 < Printer::xyJerkLimit())
            factor = Printer::xyJerkLimit() / (2.0 * maxJoinSpeed);
    }

    if ((previous.dir | next.dir) & (16 << Z_AXIS)) {
        const float zJerk = fabs(to.speedZ - from.speedZ);
        if (zJerk > Printer::zJerkLimit())
            factor = RMath::min(factor, Printer::zJerkLimit() / zJerk);
    }

    const float eJerk = fabs(to.speedE - from.speedE);
    if (eJerk > Extruder::current->maxEJerk) {
        factor = RMath::min(factor, Extruder::current->maxEJerk / eJerk);
    }
    from.maxJunctionSpeed = maxJoinSpeed * factor;
}

inline void MotionPlanner::planDeceleration(MoveQueue& queue, uint8_t start, uint8_t first) {
    PrintLine* next = &queue.slot(start);
    float lastJunctionSpeed = next->lookahead.endSpeed;

    while (start != first) {
        MoveQueue::previousIndex(start);
        PrintLine& previous = queue.slot(start);
        previous.block();
        auto& from = previous.lookahead;
        auto& to = next->lookahead;
        lastJunctionSpeed = next->isNominalMove() ? next->fullSpeed
            : sqrt(lastJunctionSpeed * lastJunctionSpeed + to.accelerationDistance2);

        if (lastJunctionSpeed >= from.maxJunctionSpeed) {
            if (from.endSpeed != from.maxJunctionSpeed) {
                previous.invalidateStepProfile();
                from.endSpeed = RMath::max(from.minSpeed, from.maxJunctionSpeed);
            }
            if (to.startSpeed != from.maxJunctionSpeed) {
                to.startSpeed = RMath::max(to.minSpeed, from.maxJunctionSpeed);
                next->invalidateStepProfile();
            }
            lastJunctionSpeed = from.endSpeed;
        } else {
            to.startSpeed = RMath::max(to.minSpeed, lastJunctionSpeed);
            lastJunctionSpeed = from.endSpeed = RMath::max(lastJunctionSpeed, from.minSpeed);
            previous.invalidateStepProfile();
            next->invalidateStepProfile();
        }
        next = &previous;
    }
}

inline void MotionPlanner::planAcceleration(MoveQueue& queue, uint8_t first, uint8_t end) {
    PrintLine* next = &queue.slot(first);
    float entrySpeed = next->lookahead.startSpeed;
    while (first != end) {
        PrintLine& move = *next;
        MoveQueue::nextIndex(first);
        next = &queue.slot(first);
        auto& from = move.lookahead;
        auto& to = next->lookahead;
        float reachableExit = move.isNominalMove() ? move.fullSpeed
            : sqrt(entrySpeed * entrySpeed + from.accelerationDistance2);
        if (reachableExit > from.endSpeed) {
            if (entrySpeed < from.minSpeed) {
                entrySpeed = from.minSpeed;
                from.endSpeed = sqrt(entrySpeed * entrySpeed + from.accelerationDistance2);
            }
            from.startSpeed = entrySpeed;
            to.startSpeed = entrySpeed = RMath::max(RMath::min(from.endSpeed, from.maxJunctionSpeed), to.minSpeed);
            if (from.endSpeed == from.maxJunctionSpeed) {
                move.setEndSpeedFixed(true);
                next->setStartSpeedFixed(true);
            }
            move.invalidateStepProfile();
        } else {
            move.fixStartAndEndSpeed();
            move.invalidateStepProfile();
            if (from.minSpeed > entrySpeed) {
                entrySpeed = from.minSpeed;
                reachableExit = sqrt(entrySpeed * entrySpeed + from.accelerationDistance2);
            }
            from.startSpeed = entrySpeed;
            from.endSpeed = RMath::max(from.minSpeed, reachableExit);
            to.startSpeed = entrySpeed = RMath::max(RMath::min(from.endSpeed, from.maxJunctionSpeed), to.minSpeed);
            next->setStartSpeedFixed(true);
        }
    }
    next->lookahead.startSpeed = RMath::max(next->lookahead.minSpeed, entrySpeed);
}

void MotionPlanner::updateLookahead(MoveQueue& queue) {
    constexpr millis_t planningReserveTicks = MOVE_CACHE_SIZE < 10 ? 4500L * MOVE_CACHE_SIZE : 45000L;
    const uint8_t end = queue.writeIndex();
    PrintLine& newest = queue.writeSlot();

    InterruptProtectedBlock lock;
    // Keep the front and enough following motion available while lookahead runs.
    uint8_t limit = queue.readIndex();
    if (limit != end)
        MoveQueue::nextIndex(limit);
    millis_t reservedTicks = 0;
    while (reservedTicks < planningReserveTicks && limit != end) {
        reservedTicks += queue.slot(limit).timeInTicks;
        MoveQueue::nextIndex(limit);
    }

    uint8_t first = end;
    while (first != limit && !queue.slot(first).isEndSpeedFixed())
        MoveQueue::previousIndex(first);
    if (first != end && queue.slot(first).isEndSpeedFixed())
        MoveQueue::nextIndex(first);

    // Timer1 can continue up to the first move whose profile may be rewritten.
    PrintLine& firstMove = queue.slot(first);
    firstMove.block();
    lock.unprotect();

    if (first == end) {
        newest.setStartSpeedFixed(true);
        prepareStepProfile(newest);
        newest.unblock();
        return;
    }

    uint8_t previousIndex = end;
    MoveQueue::previousIndex(previousIndex);
    PrintLine& previous = queue.slot(previousIndex);

    // E-only moves fix the join. With Advance, an extrusion boundary also
    // keeps the conservative entry and exit so nozzle pressure can change.
    // See Repetier-Firmware issue 837 for the fast travel to print case.
    if (previous.isEOnlyMove() != newest.isEOnlyMove()
        || (Printer::extrusionOutput.advanceEnabled() && previous.isEMove() != newest.isEMove())) {
        previous.setEndSpeedFixed(true);
        newest.setStartSpeedFixed(true);
        prepareStepProfile(newest);
        firstMove.unblock();
        return;
    }

    limitJunction(previous, newest);
    planDeceleration(queue, end, first);
    planAcceleration(queue, first, end);

    // Release each recalculated move as soon as its step profile is ready.
    do {
        PrintLine& move = queue.slot(first);
        prepareStepProfile(move);
        move.unblock();
        MoveQueue::nextIndex(first);
        queue.slot(first).block();
    } while (first != end);

    prepareStepProfile(newest);
    newest.unblock();
}

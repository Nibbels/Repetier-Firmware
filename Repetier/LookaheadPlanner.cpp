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
#include "LookaheadPlanner.h"

inline void LookaheadPlanner::limitJunction(PrintLine& previous, const PrintLine& next) {
    // The XY limit weights the angle between the two moves by the previous move's length.
    float factor = 1.0;
    float lengthFactor = 1.0;
    if (previous.distance < MAX_JERK_DISTANCE)
        lengthFactor = static_cast<float>(MAX_JERK_DISTANCE * MAX_JERK_DISTANCE) / (previous.distance * previous.distance);
    float maxJoinSpeed = RMath::min(next.fullSpeed, previous.fullSpeed);

    const float calculatedJerk = maxJoinSpeed * lengthFactor
        * (1.0 - (next.speedX * previous.speedX + next.speedY * previous.speedY + next.speedZ * previous.speedZ)
            / (next.fullSpeed * previous.fullSpeed));

    if (calculatedJerk > Printer::maxXYJerk) {
        factor = Printer::maxXYJerk / calculatedJerk;
        if (factor * maxJoinSpeed * 2.0 < Printer::maxXYJerk)
            factor = Printer::maxXYJerk / (2.0 * maxJoinSpeed);
    }

    if ((previous.dir | next.dir) & (16 << Z_AXIS)) {
        const float zJerk = fabs(next.speedZ - previous.speedZ);
        if (zJerk > Printer::maxZJerk)
            factor = RMath::min(factor, Printer::maxZJerk / zJerk);
    }

    const float eJerk = fabs(next.speedE - previous.speedE);
    if (eJerk > Extruder::current->maxEJerk) {
        factor = RMath::min(factor, Extruder::current->maxEJerk / eJerk);
    }
    previous.maxJunctionSpeed = maxJoinSpeed * factor;
}

inline void LookaheadPlanner::planDeceleration(MoveQueue& queue, uint8_t start, uint8_t first) {
    PrintLine* next = &queue.slot(start);
    float lastJunctionSpeed = next->endSpeed;

    while (start != first) {
        MoveQueue::previousIndex(start);
        PrintLine& previous = queue.slot(start);
        previous.block();
        lastJunctionSpeed = next->isNominalMove() ? next->fullSpeed
            : sqrt(lastJunctionSpeed * lastJunctionSpeed + next->accelerationDistance2);

        if (lastJunctionSpeed >= previous.maxJunctionSpeed) {
            if (previous.endSpeed != previous.maxJunctionSpeed) {
                previous.invalidateParameter();
                previous.endSpeed = RMath::max(previous.minSpeed, previous.maxJunctionSpeed);
            }
            if (next->startSpeed != previous.maxJunctionSpeed) {
                next->startSpeed = RMath::max(next->minSpeed, previous.maxJunctionSpeed);
                next->invalidateParameter();
            }
            lastJunctionSpeed = previous.endSpeed;
        } else {
            next->startSpeed = RMath::max(next->minSpeed, lastJunctionSpeed);
            lastJunctionSpeed = previous.endSpeed = RMath::max(lastJunctionSpeed, previous.minSpeed);
            previous.invalidateParameter();
            next->invalidateParameter();
        }
        next = &previous;
    }
}

inline void LookaheadPlanner::planAcceleration(MoveQueue& queue, uint8_t first, uint8_t end) {
    PrintLine* next = &queue.slot(first);
    float entrySpeed = next->startSpeed;
    while (first != end) {
        PrintLine& move = *next;
        MoveQueue::nextIndex(first);
        next = &queue.slot(first);
        float reachableExit = move.isNominalMove() ? move.fullSpeed
            : sqrt(entrySpeed * entrySpeed + move.accelerationDistance2);
        if (reachableExit > move.endSpeed) {
            if (entrySpeed < move.minSpeed) {
                entrySpeed = move.minSpeed;
                move.endSpeed = sqrt(entrySpeed * entrySpeed + move.accelerationDistance2);
            }
            move.startSpeed = entrySpeed;
            next->startSpeed = entrySpeed = RMath::max(RMath::min(move.endSpeed, move.maxJunctionSpeed), next->minSpeed);
            if (move.endSpeed == move.maxJunctionSpeed) {
                move.setEndSpeedFixed(true);
                next->setStartSpeedFixed(true);
            }
            move.invalidateParameter();
        } else {
            move.fixStartAndEndSpeed();
            move.invalidateParameter();
            if (move.minSpeed > entrySpeed) {
                entrySpeed = move.minSpeed;
                reachableExit = sqrt(entrySpeed * entrySpeed + move.accelerationDistance2);
            }
            move.startSpeed = entrySpeed;
            move.endSpeed = RMath::max(move.minSpeed, reachableExit);
            next->startSpeed = entrySpeed = RMath::max(RMath::min(move.endSpeed, move.maxJunctionSpeed), next->minSpeed);
            next->setStartSpeedFixed(true);
        }
    }
    next->startSpeed = RMath::max(next->minSpeed, entrySpeed);
}

void LookaheadPlanner::update(MoveQueue& queue) {
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
        newest.updateStepsParameter();
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
        || (Printer::isAdvanceActivated() && previous.isEMove() != newest.isEMove())) {
        previous.setEndSpeedFixed(true);
        newest.setStartSpeedFixed(true);
        newest.updateStepsParameter();
        firstMove.unblock();
        return;
    }

    limitJunction(previous, newest);
    planDeceleration(queue, end, first);
    planAcceleration(queue, first, end);

    // Release each recalculated move as soon as its step profile is ready.
    do {
        PrintLine& move = queue.slot(first);
        move.updateStepsParameter();
        move.unblock();
        MoveQueue::nextIndex(first);
        queue.slot(first).block();
    } while (first != end);

    newest.updateStepsParameter();
    newest.unblock();
}

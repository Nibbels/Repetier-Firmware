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

    if (calculatedJerk > Printer::maxXYJerk) {
        factor = Printer::maxXYJerk / calculatedJerk;
        if (factor * maxJoinSpeed * 2.0 < Printer::maxXYJerk)
            factor = Printer::maxXYJerk / (2.0 * maxJoinSpeed);
    }

    if ((previous.dir | next.dir) & (16 << Z_AXIS)) {
        const float zJerk = fabs(to.speedZ - from.speedZ);
        if (zJerk > Printer::maxZJerk)
            factor = RMath::min(factor, Printer::maxZJerk / zJerk);
    }

    const float eJerk = fabs(to.speedE - from.speedE);
    if (eJerk > Extruder::current->maxEJerk) {
        factor = RMath::min(factor, Extruder::current->maxEJerk / eJerk);
    }
    from.maxJunctionSpeed = maxJoinSpeed * factor;
}

inline void LookaheadPlanner::planDeceleration(MoveQueue& queue, uint8_t start, uint8_t first) {
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

inline void LookaheadPlanner::planAcceleration(MoveQueue& queue, uint8_t first, uint8_t end) {
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
        newest.prepareStepProfile();
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
        newest.prepareStepProfile();
        firstMove.unblock();
        return;
    }

    limitJunction(previous, newest);
    planDeceleration(queue, end, first);
    planAcceleration(queue, first, end);

    // Release each recalculated move as soon as its step profile is ready.
    do {
        PrintLine& move = queue.slot(first);
        move.prepareStepProfile();
        move.unblock();
        MoveQueue::nextIndex(first);
        queue.slot(first).block();
    } while (first != end);

    newest.prepareStepProfile();
    newest.unblock();
}

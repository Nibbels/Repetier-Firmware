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
#include "StepExecutor.h"

static_assert(X_AXIS == 0 && Y_AXIS == 1 && Z_AXIS == 2 && E_AXIS == 3,
              "StepErrors initialization requires the established axis order");
StepExecutor::StepClock StepExecutor::stepClocks[2];
StepExecutor::StepErrors StepExecutor::stepErrors[2];
StepExecutor::DirectionOwner StepExecutor::directionOwner = DirectionOwner::None;
volatile uint32_t StepExecutor::stepIntervalTicks = 10000;
volatile uint16_t StepExecutor::packingInterval_ = STEP_PACKING_MIN_INTERVAL;

void StepExecutor::setPackingInterval(uint16_t interval) {
    InterruptProtectedBlock lock;
    packingInterval_ = interval;
}
uint8_t StepExecutor::stepsPerTimerCall = 1;
volatile int32_t StepExecutor::queuedPrimarySteps;
int32_t StepExecutor::directPrimarySteps;
volatile float StepExecutor::nominalFeedrate_ = 0;

float StepExecutor::nominalFeedrate() {
    InterruptProtectedBlock lock;
    return nominalFeedrate_;
}

static inline bool hasQueuedOvertravel() {
    return Printer::xyMotion.hasQueuedOvertravel() || Printer::zMotion.hasQueuedOvertravel();
}

inline void StepExecutor::enableSteppers(PrintLine& move) {
    if (Printer::blockAll) {
        // do not enable anything in case everything is blocked
        return;
    }

    if (move.isXMove()) {
        Printer::enableXStepper();
    }
    if (move.isYMove()) {
        Printer::enableYStepper();
    }
    if (move.isZMove()) {
        Printer::enableZStepper();
    }
    if (move.isEMove()) {
        Printer::unmarkAllSteppersDisabled(); // Doesnt fit into Extruder::enable() because of forward declare -> TODO
        Extruder::enable();
        InterruptProtectedBlock lock;
        Printer::extrusionOutput.allow(ExtrusionOutput::Inhibit::MotorsDisabled);
    }
}

inline void StepExecutor::adjustDirections(PrintLine& move) {
    if (Printer::blockAll) {
        // do not enable anything in case everything is blocked
        return;
    }

    if (move.isXMove()) {
        Printer::setXDirection(move.isXPositiveMove());
    }
    if (move.isYMove()) {
        Printer::setYDirection(move.isYPositiveMove());
    }
    if (move.isZMove()) {
        ZMotion::setDirection(move.isZPositiveMove());
    }
    if (move.isEMove()) {
        if (!Printer::extrusionOutput.advanceEnabled())
            Extruder::setDirection(move.isEPositiveMove());
    }
}

inline void StepExecutor::updateAdvanceSteps(const PrintLine& move, uint16_t speed) {
    if (!Printer::extrusionOutput.advanceEnabled() || Printer::extrusionOutput.blocked())
        return;

    const int32_t leadSteps = (speed * move.stepProfile.advanceL) >> 16;
    HAL::forbidInterrupts();
    Printer::extrusionOutput.setLeadSteps(leadSteps);
    HAL::allowInterrupts();
}

void StepExecutor::stepSlowedZCompensation() {
    if (Printer::blockAll || Printer::zMotion.scanInProgress()) {
        return;
    }

    // We do not want the zCMP to have a total axis dependand constant speed.
    // Constant jerky speedup and speeddowns sound awfull and we suspect that to cause the z-lift problematic.

    // So we start slowly and get faster with outputting steps by subtracting future wait steps from "mass".
    // When we reach a small amount of leftover todo steps we add something to "mass" to slowdown.
    // Idle zCmp adds to "mass" so we have a slow next start.

    constexpr uint8_t massWeight = 15;
    static uint8_t waitSteps = 0;
    static uint8_t mass = massWeight;
    if (waitSteps) {
        waitSteps--;
        return;
    }

    const int32_t cmpDiff = Printer::zMotion.stepCorrection();
    if (cmpDiff) {
        if (mass < massWeight && abs(cmpDiff) < massWeight)
            mass++;
        else if (mass)
            mass--;

        ZMotion::finishPulse();
    } else if (mass < massWeight) {
        mass++;
    }

    // Do not calculate this work more often then needed.
    waitSteps = mass + stepsPerTimerCall;
}

bool StepExecutor::needCmpWait() {
    if (Printer::blockAll || Printer::zMotion.scanInProgress()) {
        return false;
    }

    return Printer::zMotion.correctionDelaysQueue(Printer::doHeatBedZCompensation);
}

/**
  Processes the moves from the queue and moves the stepper motors one step. If the last step is reached, the next movement from the queue is started.
  The function must be called from a timer loop. It returns the time for the next call. */
long StepExecutor::performQueueMove() {
    auto& queue = PrintLine::queue;
    PrintLine* move = queue.active();
    if (move == nullptr) {
        if (!queue.count()) {
            HAL::forbidInterrupts();
            return 1000;
        }

        move = queue.activateFront();

        switch (move->queueTaskId()) {
        case QueueTask::None: {
            break;
        }

        case QueueTask::Pause: {
            queue.retireFront();
            Printer::pause.request(PrintPause::Destination::InPlace);
            return 1000;
        }

        case QueueTask::PauseAndPark: {
            queue.retireFront();
            Printer::pause.request(PrintPause::Destination::Park);
            return 1000;
        }

        case QueueTask::EnableZCompensation: // M3001
        {
            Printer::enableCMPnow();
            queue.retireFront();
            return 1000;
        }

        case QueueTask::DisableZCompensation: // M3000
        {
            // disable the z compensation
            Printer::disableCMPnow(); // Hier nicht unbedingt warten, das soll im fluss der queue einfach abgeschaltet werden. zu große abstände regelt die needCmpWait
            queue.retireFront();
            return 1000;
        }

        case QueueTask::EnableSenseOffset: // M3909
        {
            Printer::enableSenseOffsetnow();
            queue.retireFront();
            return 1000;
        }

        case QueueTask::UnlockDisplay: // M3117 unlock
        {
            // allow to overwrite the current string again
            uid.unlock();
            queue.retireFront();
            return 1000;
        }

        default: {
            // this is an unknown task - we should never end up here
            queue.retireFront();
            return 1000;
        }
        }

        // Lookahead holds a block until it has refreshed a queued step profile.
        if (move->isBlocked()) {
            queue.releaseFront();
            return 2000;
        }
        HAL::allowInterrupts();

        if (move->isWarmUp()) {
            // This is a warmup move to initalize the path planner correctly. Just waste
            // a bit of time to get the planning up to date.
            if (queue.count() <= move->getWaitForXLinesFilled()) {
                queue.releaseFront();
                return 2000;
            }

            long wait = move->getWaitTicks();
            queue.retireFront();
            return (wait); // waste some time for path optimization to fill up
        }

        enableSteppers(*move); // Set Z direction etc.
        adjustDirections(*move);
        directionOwner = DirectionOwner::Queued;
        move->fixStartAndEndSpeed();

        queuedPrimarySteps = move->delta[move->primaryAxis];
        stepErrors[FOR_QUEUE].start(queuedPrimarySteps);
        stepClocks[FOR_QUEUE].start(move->stepProfile.vStart);
        nominalFeedrate_ = move->fullSpeed;

        HAL::forbidInterrupts();

        updateAdvanceSteps(*move, move->stepProfile.vStart);

        return (stepIntervalTicks >> 1); // Wait 50% to next interrupt.
    }

    if (directionOwner != DirectionOwner::Queued) {
        directionOwner = DirectionOwner::Queued;
        nominalFeedrate_ = move->fullSpeed;
        adjustDirections(*move);
        updateAdvanceSteps(*move, stepClocks[FOR_QUEUE].peak);
    }

    return performMove<FOR_QUEUE>(move);
} // performQueueMove

template <int8_t limitDirection>
static bool canStepAtXYLimit(uint8_t axis, PrintLine* move, uint8_t forQueue) {
    auto& xy = Printer::xyMotion;
    const bool towardLimit = limitDirection > 0 ? move->isPositiveMoveOfAxis(axis) : move->isNegativeMoveOfAxis(axis);
    if (!towardLimit)
        return !forQueue || !xy.repayQueuedOvertravel(axis, -limitDirection);

    if (move->isAbortAtEndstops()) {
        move->setMoveOfAxisFinished(axis);
        if (forQueue)
            xy.clearQueuedOvertravel(axis);
        else
            Printer::stopDirectAxis(axis);
    } else {
        xy.recordSkippedStep(axis, limitDirection);
    }
    return false;
}

static bool canStepXY(uint8_t axis, PrintLine* move, uint8_t forQueue) {
    if (Printer::isMaxEndstopHit(axis))
        return canStepAtXYLimit<1>(axis, move, forQueue);
    if (Printer::isMinEndstopHit(axis))
        return canStepAtXYLimit<-1>(axis, move, forQueue);

    auto& xy = Printer::xyMotion;
    // A returning queue move still accounts for skipped steps when the switch opens.
    if (forQueue && xy.queuedOvertravel(axis)) {
        if (move->isNegativeMoveOfAxis(axis))
            return !xy.repayQueuedOvertravel(axis, -1);
        if (move->isPositiveMoveOfAxis(axis))
            return !xy.repayQueuedOvertravel(axis, 1);
    }
    return true;
}

/**
  Processes the one-and-only direct move and moves the stepper motors one step.
  The function must be called from a timer loop. It returns the time for the next call. */
long StepExecutor::performDirectMove() {
    auto& direct = PrintLine::direct;
    if (PrintLine::directMovePreparing(direct.task)) {
        return 2000;
    }
    if (direct.task == DIRECT_PREPARED_STOPPABLE || direct.task == DIRECT_PREPARED) {
        direct.task = (direct.task == DIRECT_PREPARED_STOPPABLE ? DIRECT_RUNNING_STOPPABLE : DIRECT_RUNNING);

        enableSteppers(direct);
        adjustDirections(direct);
        directionOwner = DirectionOwner::Direct;

        directPrimarySteps = direct.delta[direct.primaryAxis];
        stepErrors[FOR_DIRECT].start(directPrimarySteps);
        stepClocks[FOR_DIRECT].start(direct.stepProfile.vStart);
        nominalFeedrate_ = direct.fullSpeed;

        updateAdvanceSteps(direct, direct.stepProfile.vStart);

        return (stepIntervalTicks >> 1); // Wait 50% to next interrupt.
    }

    if (directionOwner != DirectionOwner::Direct) {
        directionOwner = DirectionOwner::Direct;
        nominalFeedrate_ = direct.fullSpeed;
        adjustDirections(direct);
        updateAdvanceSteps(direct, stepClocks[FOR_DIRECT].peak);
    }

    return performMove<FOR_DIRECT>(&direct);
} // performDirectMove

// Keep the ramp update in the specialized executor, as in the original AVR build.
inline __attribute__((always_inline)) speed_t StepExecutor::nextStepSpeed(PrintLine& move, StepClock& clock, uint8_t pulses, uint32_t interval) {
    const auto& stepProfile = move.stepProfile;
    clock.steps += pulses;
    clock.ticks += interval * pulses;

    if (clock.steps <= stepProfile.accelSteps) {
        const speed_t speed = stepProfile.vStart + HAL::ComputeV(clock.ticks, stepProfile.fAcceleration);
        clock.peak = speed > stepProfile.vMax ? stepProfile.vMax : speed;
        return clock.peak;
    }

    if (move.stepsRemaining > static_cast<int32_t>(stepProfile.decelSteps))
        return stepProfile.accelSteps ? clock.peak : stepProfile.vMax;

    if (!(move.flags & FLAG_DECELERATING)) {
        clock.ticks = 0;
        move.flags |= FLAG_DECELERATING;
    }

    const speed_t slowdown = HAL::ComputeV(clock.ticks, stepProfile.fAcceleration);
    if (slowdown > clock.peak)
        return stepProfile.vEnd;
    const speed_t speed = clock.peak - slowdown;
    return speed < stepProfile.vEnd ? stepProfile.vEnd : speed;
}

// Isolate completion from step scheduling.
inline void StepExecutor::finishMove(PrintLine& move, bool queued) {
    move.dir &= ~240;
    Extruder::current->stepperDirection = 0;

    auto& queue = PrintLine::queue;
    if (queued) {
        queue.retireFront();
    } else {
        Printer::stopDirectAxis(X_AXIS);
        Printer::stopDirectAxis(Y_AXIS);
        Printer::stopDirectAxis(Z_AXIS);
        Printer::stopDirectAxis(E_AXIS);
        move.stepsRemaining = 0;
        move.task = DIRECT_IDLE;
    }

    if (queue.count() == 0 && !isAnyScanRunning()) {
        Printer::idleStatus.schedule(HAL::timeInMilliseconds());
        nominalFeedrate_ = 0;
    }
}

template<uint8_t forQueue>
long StepExecutor::performMove(PrintLine* move) {
    static_assert(forQueue == FOR_QUEUE || forQueue == FOR_DIRECT, "Invalid execution owner");
    auto& errors = stepErrors[forQueue];
    HAL::allowInterrupts();
    fast8_t max_loops = stepsPerTimerCall;
    if (move->stepsRemaining < max_loops)
        max_loops = move->stepsRemaining;
    HAL::forbidInterrupts();

    for (fast8_t loop = 0; loop < max_loops; loop++) {


        if (move->isEMove()) {
            HAL::allowInterrupts();
            bool doESteps = errors.advance(E_AXIS, move->delta[E_AXIS]);
            if (doESteps) {
                // Count step as done, we wont need it later.
                if (forQueue)
                    errors.account(E_AXIS, queuedPrimarySteps);
                else
                    errors.account(E_AXIS, directPrimarySteps);
            }

            // Queue overtravel belongs to the frozen print path. The pressure guard applies to both paths.
            if (Printer::extrusionOutput.blocked() || (forQueue && hasQueuedOvertravel())) {
                doESteps = false;
            }
            HAL::forbidInterrupts();

            if (doESteps) {
                const bool forward = move->isEPositiveMove();
                const int8_t dir = forward ? 1 : -1;
                if (Printer::extrusionOutput.advanceEnabled())
                    Printer::extrusionOutput.addSteps(dir);
                else
                {
                    Extruder::step();
                }
                if (!forQueue)
                    Printer::pause.recordDirectExtrusion(dir);

                if (forQueue && Printer::extrusionCompensation.addBaseStep(forward)) {
                    if (Printer::extrusionOutput.advanceEnabled()) {
                        Printer::extrusionOutput.addSteps(dir);
                        Printer::extrusionCompensation.consumeExtraStep(forward);
                    } else {
                        // Normalize the remainder between the two step pulses.
                        Extruder::unstep();
                        Printer::extrusionCompensation.consumeExtraStep(forward);
                        Extruder::step();
                    }
                }
            }
        }

        if (move->isXMove()) {
            if (errors.advance(X_AXIS, move->delta[X_AXIS])) {
                HAL::allowInterrupts();
                int8_t dir = (Printer::getXDirectionIsPos() ? 1 : -1);
                if (canStepXY(X_AXIS, move, forQueue)) {
                    Printer::startXStep(dir);
                }
                HAL::forbidInterrupts();

                if (forQueue) {
                    errors.account(X_AXIS, queuedPrimarySteps);
                    Printer::xyMotion.advanceNominal(X_AXIS, dir);
                } else if (move->isXMove()) {
                    // Endstop handling clears the axis flag when it rejects a direct step and stops the axis.
                    errors.account(X_AXIS, directPrimarySteps);
                    Printer::xyMotion.advanceDirect(X_AXIS, dir);
                }
            }
        }

        if (move->isYMove()) {
            if (errors.advance(Y_AXIS, move->delta[Y_AXIS])) {
                HAL::allowInterrupts();
                int8_t dir = (Printer::getYDirectionIsPos() ? 1 : -1);
                if (canStepXY(Y_AXIS, move, forQueue)) {
                    Printer::startYStep(dir);
                }
                HAL::forbidInterrupts();

                if (forQueue) {
                    errors.account(Y_AXIS, queuedPrimarySteps);
                    Printer::xyMotion.advanceNominal(Y_AXIS, dir);
                } else if (move->isYMove()) {
                    errors.account(Y_AXIS, directPrimarySteps);
                    Printer::xyMotion.advanceDirect(Y_AXIS, dir);
                }
            }
        }

        if (move->isZMove()) {
            if (errors.advance(Z_AXIS, move->delta[Z_AXIS])) {
                const auto path = forQueue
                    ? (move->isAbortAtEndstops() ? ZMotion::Path::QueuedUntilEndstop : ZMotion::Path::Queued)
                    : (move->task == DIRECT_RUNNING_STOPPABLE ? ZMotion::Path::StoppableDirect : ZMotion::Path::Direct);
                const bool continuing = Printer::zMotion.stepPath(path);
                if (!continuing)
                    move->setMoveOfAxisFinished(Z_AXIS);

                if (forQueue)
                    errors.account(Z_AXIS, queuedPrimarySteps);
                else if (continuing)
                    errors.account(Z_AXIS, directPrimarySteps);
            }
        }
        else if (move->isXOrYMove()) {
            StepExecutor::stepSlowedZCompensation(); // Z is free here. This is no Z-Move
        }

        move->stepsRemaining--;


        if (!Printer::extrusionOutput.advanceEnabled())
            Extruder::unstep();

        Printer::endXYZSteps();
    }                       // for loop
    HAL::allowInterrupts(); // Allow interrupts for other types, timer1 is still disabled

    const speed_t v = nextStepSpeed(*move, stepClocks[forQueue], max_loops, stepIntervalTicks);
    updateAdvanceSteps(*move, v);

    stepIntervalTicks = HAL::CPUDivU2(v);

    // Dieses Limit bedeutet max. 31250 steps/s bei 16mhz.
    if (stepIntervalTicks < minimumStepIntervalTicks)
        stepIntervalTicks = minimumStepIntervalTicks;

    //RETURN manipulated value:
    unsigned long interval = stepIntervalTicks;

    const uint16_t digitFlowScale = Printer::digitCompensation.intervalScale();
    if (digitFlowScale && forQueue && move->primaryAxis <= Y_AXIS) {
        interval *= digitFlowScale; // 1024 == 100%
        interval >>= 10;
    }

    // Now manipulate step-width=interval according to single-, double-, tripple-, ... steppings needs:
    // We know that next time will be a (as example) double or quad step so we let pass twice the time and set the stepsPerTimerCall accordingly.
    stepsPerTimerCall = 1;
    unsigned long one_interval = interval;
    // Keep the existing threshold. The previous unused +64 advance local
    // did not affect this loop; changing that policy is not an ownership refactor.
    const uint16_t packingInterval = packingInterval_;
    while (interval < packingInterval) {
        interval += one_interval;
        stepsPerTimerCall += 1;
    }

    if (move->stepsRemaining <= 0 || move->isNoMove()) {
        finishMove(*move, forQueue);
        interval >>= 1; // The two half-intervals allow time to set up the next move.
    }

    return interval;
} // performMove

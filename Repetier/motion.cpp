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

int maxadv2 = 0;
float maxadvspeed = 0;
uint8_t pwm_pos[NUM_EXTRUDER + 3]; // 0-NUM_EXTRUDER = Heater 0-NUM_EXTRUDER of extruder, NUM_EXTRUDER = Heated bed, NUM_EXTRUDER+1 Board fan, NUM_EXTRUDER+2 = Fan

uint8_t fanSpeed = 0;

MoveQueue PrintLine::queue;
PrintLine PrintLine::direct;
PrintLine::StepClock PrintLine::stepClocks[2];
PrintLine::DirectionOwner PrintLine::directionOwner = DirectionOwner::None;
volatile uint32_t PrintLine::stepIntervalTicks = 10000;
uint8_t PrintLine::stepsPerTimerCall = 1;

bool PrintLine::hasActiveZMove() {
    InterruptProtectedBlock lock;
    if (direct.stepsRemaining && direct.isZMove())
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

    distance = axisDistanceMM[E_AXIS];
    if (isXYZMove()) {
        float distanceSquared = axisDistanceMM[X_AXIS] * axisDistanceMM[X_AXIS]
            + axisDistanceMM[Y_AXIS] * axisDistanceMM[Y_AXIS];
        if (isZMove())
            distanceSquared += axisDistanceMM[Z_AXIS] * axisDistanceMM[Z_AXIS];
        distance = RMath::max((float)sqrt(distanceSquared), distance);
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

    p->calculateMove(axisDistanceMM, p->primaryAxis, feedrate);

    LookaheadPlanner::update(queue);
    if (pathOptimize)
        Printer::pressureAdvance.postponeRelaxation();

    // Make result permanent
    queue.publish(true);
} // prepareQueueMove

void PrintLine::prepareDirectMove(bool stoppable, bool feedrateSource) {
    if (direct.task) {
        // Do not overwrite a running directstep process.
        // If we return here the steps still might be processed as slow direct-stepping.
        // Rework your code if you see this happening.
        return;
    }
    direct.block();
    direct.task = DIRECT_PREPARING;
    direct.flags = FLAG_ABORT_AT_ENDSTOPS;
    direct.joinFlags = 0;
    direct.setEndSpeedFixed(true);
    direct.dir = 0;

    // Find direction
    float axisDistanceMM[4]; // Axis movement in mm
    for (uint8_t axis = 0; axis < 4; axis++) {
        direct.setAxisSteps(axis, Printer::directPosition(axis).remaining());
        axisDistanceMM[axis] = fabs(direct.delta[axis] * Printer::axisMMPerSteps[axis]);
    }
    if (direct.isNoMove()) {
        direct.stepsRemaining = 0;
        direct.unblock();
        direct.task = TASK_NO_TASK;
        return; // No steps included
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

    direct.calculateMove(axisDistanceMM, direct.primaryAxis, feedrate);
    direct.unblock();
    direct.task = (stoppable ? DIRECT_PREPARED_STOPPABLE : DIRECT_PREPARED);
} // prepareDirectMove

void PrintLine::stopDirectMove(void) //Funktion ist bereits zur ausführzeit von InterruptProtectedBlock eingeschlossen!
{
    if (direct.stepsRemaining > 32)
        direct.stepsRemaining = 32; // Leave a short deceleration tail.
} // stopDirectMove

void PrintLine::calculateMove(float axisDistanceMM[], fast8_t drivingAxis, float feedrate) {
    if (stepsRemaining == 0) { // need at least one step for bresenham
        return;
    }
    float timeForMove = (float)(F_CPU)*distance / feedrate; // time is in ticks
    // Small element limiter: This was not present in directmove but is not harmfull.
    if (queue.count() < MOVE_CACHE_LOW && timeForMove < LOW_TICKS_PER_MOVE) // Limit speed to keep cache full.
    {
        timeForMove += ((LOW_TICKS_PER_MOVE - timeForMove)) * 3 / (queue.count() + 1); // Increase time if queue gets empty. Add more time if queue gets smaller.
    }
    timeInTicks = timeForMove;

    // Compute the solwest allowed interval (ticks/step), so maximum feedrate is not violated
    int32_t limitInterval0;
    int32_t limitInterval = limitInterval0 = timeForMove / stepsRemaining; // until not violated by other constraints it is your target speed
    float toTicks = static_cast<float>(F_CPU) / stepsRemaining;

    int32_t axisInterval[4];
    if (isXMove()) {
        axisInterval[X_AXIS] = axisDistanceMM[X_AXIS] * toTicks / (Printer::maxFeedrate[X_AXIS]); // mm*ticks/s/(mm/s*steps) = ticks/step
        limitInterval = RMath::max(axisInterval[X_AXIS], limitInterval);
    } else
        axisInterval[X_AXIS] = 0;

    if (isYMove()) {
        axisInterval[Y_AXIS] = axisDistanceMM[Y_AXIS] * toTicks / Printer::maxFeedrate[Y_AXIS];
        limitInterval = RMath::max(axisInterval[Y_AXIS], limitInterval);
    } else
        axisInterval[Y_AXIS] = 0;

    if (isZMove()) // normally no move in z direction
    {
        axisInterval[Z_AXIS] = axisDistanceMM[Z_AXIS] * toTicks / Printer::maxFeedrate[Z_AXIS]; // must prevent overflow!
        limitInterval = RMath::max(axisInterval[Z_AXIS], limitInterval);
    } else
        axisInterval[Z_AXIS] = 0;

    if (isEMove()) {
        axisInterval[E_AXIS] = axisDistanceMM[E_AXIS] * toTicks / Printer::maxFeedrate[E_AXIS];
        limitInterval = RMath::max(axisInterval[E_AXIS], limitInterval);
    } else
        axisInterval[E_AXIS] = 0;

    ticks_t fullIntervalb = limitInterval = (limitInterval > LIMIT_INTERVAL ? limitInterval : LIMIT_INTERVAL); // This is our target speed
    if (limitInterval != limitInterval0) {
        // new time at full speed = limitInterval*p->stepsRemaining [ticks]
        timeForMove = (float)limitInterval * (float)stepsRemaining; // for large z-distance this overflows with long computation
    }
    float inverseTimeS = static_cast<float>(F_CPU) / timeForMove;
    if (isXMove()) {
        axisInterval[X_AXIS] = static_cast<int32_t>(timeForMove / (axisDistanceMM[X_AXIS] * Printer::axisStepsPerMM[X_AXIS]));
        speedX = axisDistanceMM[X_AXIS] * inverseTimeS;
        if (isXNegativeMove())
            speedX = -speedX;
    } else
        speedX = 0;
    if (isYMove()) {
        axisInterval[Y_AXIS] = static_cast<int32_t>(timeForMove / (axisDistanceMM[Y_AXIS] * Printer::axisStepsPerMM[Y_AXIS]));
        speedY = axisDistanceMM[Y_AXIS] * inverseTimeS;
        if (isYNegativeMove())
            speedY = -speedY;
    } else
        speedY = 0;
    if (isZMove()) {
        axisInterval[Z_AXIS] = static_cast<int32_t>(timeForMove / (axisDistanceMM[Z_AXIS] * Printer::axisStepsPerMM[Z_AXIS]));
        speedZ = axisDistanceMM[Z_AXIS] * inverseTimeS;
        if (isZNegativeMove())
            speedZ = -speedZ;
    } else
        speedZ = 0;
    if (isEMove()) {
        axisInterval[E_AXIS] = static_cast<int32_t>(timeForMove / (axisDistanceMM[E_AXIS] * Printer::axisStepsPerMM[E_AXIS]));
        speedE = axisDistanceMM[E_AXIS] * inverseTimeS;
        if (isENegativeMove())
            speedE = -speedE;
    } else
        speedE = 0;
    fullSpeed = distance * inverseTimeS;
    // long interval = axis_interval[primary_axis]; // time for every step in ticks with full speed
    // If acceleration is enabled, do some Bresenham calculations depending on which axis will lead it.

    // slowest time to accelerate from v0 to limitInterval determines used acceleration
    // t = (v_end-v_start)/a
    float slowestAxisPlateauTimeRepro = 1e15; // repro to reduce division Unit: 1/s
    uint32_t* accel = (isEPositiveMove() ? Printer::maxPrintAccelerationStepsPerSquareSecond : Printer::maxTravelAccelerationStepsPerSquareSecond);

    for (uint8_t axis = 0; axis < 4; axis++) {
        if (isMoveOfAxis(axis)) {
            // v = a * t => t = v/a = F_CPU/(c*a) => 1/t = c*a/F_CPU
            slowestAxisPlateauTimeRepro = RMath::min(slowestAxisPlateauTimeRepro, (float)axisInterval[axis] * (float)accel[axis]); // steps/s^2 * step/tick  Ticks/s^2
        }
    }

    // Initialize every segment, including when reusing a slot from an aborted move.
    error[X_AXIS] = error[Y_AXIS] = error[Z_AXIS] = delta[primaryAxis] >> 1;
    error[E_AXIS] = 0;

    invFullSpeed = 1.0 / fullSpeed;
    accelerationPrim = slowestAxisPlateauTimeRepro / axisInterval[primaryAxis]; // a = v/t = F_CPU/(c*t): Steps/s^2

    // Now we can calculate the new primary axis acceleration, so that the slowest axis max acceleration is not violated
    // Im Interrupt steht quasi die Formel v = a * t / 2^18, darum hier die 262144
    fAcceleration = 262144.0 * (float)accelerationPrim / F_CPU;                                        // will overflow without float!
    accelerationDistance2 = 2.0 * distance * slowestAxisPlateauTimeRepro * fullSpeed / ((float)F_CPU); // mm^2/s^2
    startSpeed = endSpeed = minSpeed = safeSpeed(drivingAxis);
    if (startSpeed > feedrate) {
        startSpeed = endSpeed = minSpeed = feedrate;
    }
    // Can accelerate to full speed within the line
    if (startSpeed * startSpeed + accelerationDistance2 >= fullSpeed * fullSpeed)
        setNominalMove();

    vMax = F_CPU / fullIntervalb; // maximum steps per second, we can reach
    // if(p->vMax>46000)            // gets overflow in N computation
    // p->vMax = 46000;
    // p->plateauN = (p->vMax*p->vMax/p->accelerationPrim)>>1;

    if (!isXYZMove() || !isEPositiveMove()) { // No head move or E move only or sucking filament back
        advanceL = 0;
    } else {
        float advlin = fabs(speedE) * Extruder::current->advanceL * 0.001 * Printer::axisStepsPerMM[E_AXIS];
        advanceL = ((65536L * advlin) / vMax); //advanceLscaled = (65536*vE*k2)/vMax
        if (advlin > maxadv2) {
            maxadv2 = advlin;
            maxadvspeed = fabs(speedE);
        }
    }

} // calculateMove

// Convert planned entry and exit speeds to a step profile.
void PrintLine::updateStepsParameter() {
    if (areParameterUpToDate() || isWarmUp())
        return;

    float startFactor = startSpeed * invFullSpeed;
    float endFactor = endSpeed * invFullSpeed;
    vStart = vMax * startFactor; // starting speed
    vEnd = vMax * endFactor;

    uint32_t vmax2 = HAL::U16SquaredToU32(vMax);

    if (vStart == vMax) {
        accelSteps = 0;
    } else {
        accelSteps = ((vmax2 - HAL::U16SquaredToU32(vStart)) / (accelerationPrim << 1)) + 1; // Always add 1 for missing precision
    }
    if (vEnd == vMax) {
        decelSteps = 0;
    } else {
        decelSteps = ((vmax2 - HAL::U16SquaredToU32(vEnd)) / (accelerationPrim << 1)) + 1;
    }

    if (static_cast<int32_t>(accelSteps + decelSteps) > stepsRemaining) // can't reach limit speed
    {
        uint32_t red = (accelSteps + decelSteps - stepsRemaining) >> 1;
        accelSteps = accelSteps - RMath::min(static_cast<int32_t>(accelSteps), static_cast<int32_t>(red));
        decelSteps = decelSteps - RMath::min(static_cast<int32_t>(decelSteps), static_cast<int32_t>(red));
    }
    setParameterUpToDate();
} // updateStepsParameter

inline float PrintLine::safeSpeed(fast8_t drivingAxis) {
    float xyMin = Printer::maxXYJerk * 0.5f;
    float mz = 0;
    float safe(xyMin);
    if (isZMove()) {
        mz = Printer::maxZJerk * 0.5f;
        if (isXOrYMove()) {
            if (fabs(speedZ) > mz)
                safe = RMath::min(safe, mz * fullSpeed / fabs(speedZ));
        } else {
            safe = mz;
        }
    }
    if (isEMove()) {
        if (isXYZMove())
            safe = RMath::min(safe, 0.5f * Extruder::current->maxEJerk * fullSpeed / fabs(speedE));
        else
            safe = 0.5f * Extruder::current->maxEJerk; // This is a retraction move
    }

    // enforce minimum speed for numerical stability of explicit speed integration
    if (drivingAxis == X_AXIS || drivingAxis == Y_AXIS)
        safe = RMath::max(xyMin, safe);
    else if (drivingAxis == Z_AXIS)
        safe = RMath::max(mz, safe);

    return RMath::min(safe, fullSpeed);
} // safeSpeed

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
            p->joinFlags = FLAG_JOIN_STEPPARAMS_COMPUTED | FLAG_JOIN_END_FIXED | FLAG_JOIN_START_FIXED;
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

void PrintLine::stepSlowedZCompensation() {
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

bool PrintLine::needCmpWait() {
    if (Printer::blockAll || Printer::zMotion.scanInProgress()) {
        return false;
    }

    return Printer::zMotion.correctionDelaysQueue(Printer::doHeatBedZCompensation);
}

/**
  Processes the moves from the queue and moves the stepper motors one step. If the last step is reached, the next movement from the queue is started.
  The function must be called from a timer loop. It returns the time for the next call. */
volatile long queueError;
long PrintLine::performQueueMove() {
    PrintLine* move = queue.active();
    if (move == nullptr) {
        if (!queue.count()) {
            HAL::forbidInterrupts();
            return 1000;
        }

        move = queue.activateFront();

        switch (move->task) {
        case TASK_NO_TASK: {
            break;
        }

        case TASK_PAUSE_PRINT: {
            queue.retireFront();
            Printer::pause.request(PrintPause::Destination::InPlace);
            return 1000;
        }

        case TASK_PAUSE_PRINT_AND_MOVE: {
            queue.retireFront();
            Printer::pause.request(PrintPause::Destination::Park);
            return 1000;
        }

        case TASK_ENABLE_Z_COMPENSATION: // M3001
        {
            Printer::enableCMPnow();
            queue.retireFront();
            return 1000;
        }

        case TASK_DISABLE_Z_COMPENSATION: // M3000
        {
            // disable the z compensation
            Printer::disableCMPnow(); //hier nicht unbedingt warten, das soll im fluss der queue einfach abgeschaltet werden. zu große abstände regelt die needCmpWait
            queue.retireFront();
            return 1000;
        }

        case TASK_ENABLE_SENSE_OFFSET: // M3909
        {
            Printer::enableSenseOffsetnow();
            queue.retireFront();
            return 1000;
        }

        case TASK_UNLOCK_DISPLAY_MSG: // M3117 unlock
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

        if (move->isBlocked()) // This step is in computation - shouldn't happen
        {
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

        move->enableSteppers(); //set Z direction etc.
        move->adjustDirections();
        directionOwner = DirectionOwner::Queued;
        move->fixStartAndEndSpeed();

        queueError = move->delta[move->primaryAxis];
        if (!move->areParameterUpToDate())
            move->updateStepsParameter(); // should never happen, but with bad timings???
        stepClocks[FOR_QUEUE].start(move->vStart);
        Printer::v = move->fullSpeed;

        HAL::forbidInterrupts();

        move->updateAdvanceSteps(move->vStart); //startet advance extruder "etwas" vor der ersten bewegung: gut? ist stepIntervalTicks immer klein genug?

        return (stepIntervalTicks >> 1); //wait 50% to next interrupt.
    }

    if (directionOwner != DirectionOwner::Queued) {
        directionOwner = DirectionOwner::Queued;
        Printer::v = move->fullSpeed;
        move->adjustDirections();
        move->updateAdvanceSteps(stepClocks[FOR_QUEUE].peak);
    }

    return performMove(move, FOR_QUEUE);
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
long directError;
long PrintLine::performDirectMove() {
    if (direct.isBlocked()) // This step is in computation - shouldn't happen
    {
        return 2000;
    }
    if (direct.task == DIRECT_PREPARED_STOPPABLE || direct.task == DIRECT_PREPARED) {
        direct.task = (direct.task == DIRECT_PREPARED_STOPPABLE ? DIRECT_RUNNING_STOPPABLE : DIRECT_RUNNING);

        direct.enableSteppers();
        direct.adjustDirections();
        directionOwner = DirectionOwner::Direct;
        direct.fixStartAndEndSpeed();

        directError = direct.delta[direct.primaryAxis];
        direct.updateStepsParameter(); // should never be needed, but with bad timings???
        stepClocks[FOR_DIRECT].start(direct.vStart);
        Printer::v = direct.fullSpeed;

        direct.updateAdvanceSteps(direct.vStart); //startet advance extruder "etwas" vor der ersten bewegung: gut? ist stepIntervalTicks immer klein genug?

        return (stepIntervalTicks >> 1); //wait 50% to next interrupt.
    }

    if (directionOwner != DirectionOwner::Direct) {
        directionOwner = DirectionOwner::Direct;
        Printer::v = direct.fullSpeed;
        direct.adjustDirections();
        direct.updateAdvanceSteps(stepClocks[FOR_DIRECT].peak);
    }

    return performMove(&direct, FOR_DIRECT);
} // performDirectMove

speed_t PrintLine::nextStepSpeed(StepClock& clock, uint8_t pulses, uint32_t interval) {
    clock.steps += pulses;
    clock.ticks += interval * pulses;

    if (clock.steps <= accelSteps) {
        const speed_t speed = vStart + HAL::ComputeV(clock.ticks, fAcceleration);
        clock.peak = speed > vMax ? vMax : speed;
        return clock.peak;
    }

    if (stepsRemaining > static_cast<int32_t>(decelSteps))
        return accelSteps ? clock.peak : vMax;

    if (!(flags & FLAG_DECELERATING)) {
        clock.ticks = 0;
        flags |= FLAG_DECELERATING;
    }

    const speed_t slowdown = HAL::ComputeV(clock.ticks, fAcceleration);
    if (slowdown > clock.peak)
        return vEnd;
    const speed_t speed = clock.peak - slowdown;
    return speed < vEnd ? vEnd : speed;
}

long PrintLine::performMove(PrintLine* move, uint8_t forQueue) {
    HAL::allowInterrupts();
    fast8_t max_loops = stepsPerTimerCall;
    if (move->stepsRemaining < max_loops)
        max_loops = move->stepsRemaining;
    HAL::forbidInterrupts();

    for (fast8_t loop = 0; loop < max_loops; loop++) {


        if (move->isEMove()) {
            HAL::allowInterrupts();
            bool doESteps = (move->error[E_AXIS] -= move->delta[E_AXIS]) < 0;
            if (doESteps) {
                //count step as done, we wont need it later.
                if (forQueue)
                    move->error[E_AXIS] += queueError;
                else
                    move->error[E_AXIS] += directError;
            }

            // Queue overtravel belongs to the frozen print path. The pressure guard applies to both paths.
            if (Printer::pressureProtection.extrusionBlocked() || (forQueue && PrintLine::hasQueuedOvertravel())) {
                doESteps = false;
            }
            HAL::forbidInterrupts();

            if (doESteps) {
                const bool forward = move->isEPositiveMove();
                const int8_t dir = forward ? 1 : -1;
                if (Printer::isAdvanceActivated())
                    Printer::pressureAdvance.addSteps(dir);
                else
                {
                    Extruder::step();
                }
                if (!forQueue)
                    Printer::pause.recordDirectExtrusion(dir);

                if (forQueue && Printer::extrusionCompensation.addBaseStep(forward)) {
                    if (Printer::isAdvanceActivated()) {
                        Printer::pressureAdvance.addSteps(dir);
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
            if ((move->error[X_AXIS] -= move->delta[X_AXIS]) < 0) {
                HAL::allowInterrupts();
                int8_t dir = (Printer::getXDirectionIsPos() ? 1 : -1);
                if (canStepXY(X_AXIS, move, forQueue)) {
                    Printer::startXStep(dir);
                }
                HAL::forbidInterrupts();

                if (forQueue) {
                    move->error[X_AXIS] += queueError;
                    Printer::xyMotion.advanceNominal(X_AXIS, dir);
                } else if (move->isXMove()) {
                    // Endstop handling clears the axis flag when it rejects a direct step and stops the axis.
                    move->error[X_AXIS] += directError;
                    Printer::xyMotion.advanceDirect(X_AXIS, dir);
                }
            }
        }

        if (move->isYMove()) {
            if ((move->error[Y_AXIS] -= move->delta[Y_AXIS]) < 0) {
                HAL::allowInterrupts();
                int8_t dir = (Printer::getYDirectionIsPos() ? 1 : -1);
                if (canStepXY(Y_AXIS, move, forQueue)) {
                    Printer::startYStep(dir);
                }
                HAL::forbidInterrupts();

                if (forQueue) {
                    move->error[Y_AXIS] += queueError;
                    Printer::xyMotion.advanceNominal(Y_AXIS, dir);
                } else if (move->isYMove()) {
                    move->error[Y_AXIS] += directError;
                    Printer::xyMotion.advanceDirect(Y_AXIS, dir);
                }
            }
        }

        if (move->isZMove()) {
            if ((move->error[Z_AXIS] -= move->delta[Z_AXIS]) < 0) {
                const bool continuing = Printer::zMotion.stepPath(move->zPath(forQueue));
                if (!continuing)
                    move->setZMoveFinished();

                if (forQueue)
                    move->error[Z_AXIS] += queueError;
                else if (continuing)
                    move->error[Z_AXIS] += directError;
            }
        }
        else if (move->isXOrYMove()) {
            PrintLine::stepSlowedZCompensation(); // Z is free here. This is no Z-Move
        }

        move->stepsRemaining--;


        if (!Printer::isAdvanceActivated()) // Use interrupt for movement
            Extruder::unstep();

        Printer::endXYZSteps();
    }                       // for loop
    HAL::allowInterrupts(); // Allow interrupts for other types, timer1 is still disabled

    const speed_t v = move->nextStepSpeed(stepClocks[forQueue], max_loops, stepIntervalTicks);
    move->updateAdvanceSteps(v);

    stepIntervalTicks = HAL::CPUDivU2(v);

    // Dieses Limit bedeutet max. 31250 steps/s bei 16mhz.
    if (stepIntervalTicks < 512)
        stepIntervalTicks = 512;

    //RETURN manipulated value:
    unsigned long interval = stepIntervalTicks;

    const uint16_t digitFlowScale = Printer::digitCompensation.intervalScale();
    if (digitFlowScale && forQueue && move->primaryAxis <= Y_AXIS) {
        interval *= digitFlowScale; // 1024 == 100%
        interval >>= 10;
    }

    //Now manipulate step-width=interval according to single-, double-, tripple-, ... steppings needs:
    //we know that next time will be a (as example) double or quad step so we let pass twice the time and set the stepsPerTimerCall accordingly.
    stepsPerTimerCall = 1;
    unsigned long one_interval = interval;
    uint16_t minInterval = Printer::stepsPackingMinInterval;
    if (Printer::isAdvanceActivated()) {
        minInterval += 64;
    }
    while (interval < Printer::stepsPackingMinInterval) {
        interval += one_interval;
        stepsPerTimerCall += 1;
    }

    if (move->stepsRemaining <= 0 || move->isNoMove()) // line finished
    {
        //Wenn keine Steps mehr da, sollten alle Achsen die benutzt wurden wieder freigegeben werden. Bei Z ist der Sonderfall, dass die Z-Kompensation sich reinschummeln könnte.
        move->setXYZEMoveFinished(); // Wichtig, das die Z-Kompensation wieder weiterarbeiten kann, auch wichtig bei PrintLine::direct!
                                     // Auch wenn kein Z-Move, könnte die Z-Kompensation die Achse Z benutzt haben.

        if (forQueue) {
            queue.retireFront();
        } else {
            //forDirect:
            // Wir dürften das stop nicht brauchen, brauchen es aber:
            //  Abbrechen von Direct-Move geht "sanft" über stepsremaining
            //  Abbrechen von Direct-Move geht hart über setMoveFinished.
            // Bei beiden Abbruchvarianten wird die destination nicht erreicht, also nicht fertig gezählt.
            Printer::stopDirectAxis(X_AXIS);
            Printer::stopDirectAxis(Y_AXIS);
            Printer::stopDirectAxis(Z_AXIS);
            Printer::stopDirectAxis(E_AXIS);
            move->stepsRemaining = 0;
            move->task = DIRECT_IDLE;
        }

        char nIdle = 1;
        if (g_nHeatBedScanStatus || g_nZOSScanStatus)
            nIdle = 0; // we are not idle because the heat bed scan is going on at the moment

        if (queue.count() == 0 && nIdle) {
            g_uStartOfIdle = HAL::timeInMilliseconds(); //end a move
            Printer::v = 0;
        }

        interval >>= 1; // 50% of time to next call to do cur=0, which will fire another interrupt at 50% of the active stepIntervalTicks step time. This works like "move 1 end -> 50% -> prepare move 2 -> 50% -> move 2 begin"
    } // line finished

    return interval;
} // performMove

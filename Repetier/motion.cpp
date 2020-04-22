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

#if EXT0_STEP_PIN < 0 && NUM_EXTRUDER > 0
#error EXT0_STEP_PIN not set to a pin number.
#endif // EXT0_STEP_PIN<0 && NUM_EXTRUDER>0

#if EXT0_DIR_PIN < 0 && NUM_EXTRUDER > 0
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

#if USE_ADVANCE
int maxadv2 = 0;
float maxadvspeed = 0;
volatile int waitRelax = 0; // Delay filament relax at the end of print, could be a simple timeout
#endif                      // USE_ADVANCE

uint8_t pwm_pos[NUM_EXTRUDER + 3]; // 0-NUM_EXTRUDER = Heater 0-NUM_EXTRUDER of extruder, NUM_EXTRUDER = Heated bed, NUM_EXTRUDER+1 Board fan, NUM_EXTRUDER+2 = Fan

uint8_t fanSpeed = 0;

PrintLine PrintLine::lines[MOVE_CACHE_SIZE]; // Cache for print moves.
PrintLine* PrintLine::cur = 0;               // Current printing line
PrintLine PrintLine::direct;                 // direct movement

uint8_t PrintLine::linesWritePos = 0;       // Position where we write the next cached line move.
volatile uint8_t PrintLine::linesCount = 0; // Number of lines cached 0 = nothing to do.
uint8_t PrintLine::linesPos = 0;            // Position for executing line movement.

/** \brief Put a move to the current destination coordinates into the movement cache.
  If the cache is full, the method will wait, until a place gets free. During
  wait communication and temperature control is enabled.

  @param abortAtEndstops Stop this move at any endstop during move.
   Otherwise the endstop causes an ignore of moving that axis but coordinates will move
 */
void PrintLine::prepareQueueMove(uint8_t abortAtEndstops, uint8_t pathOptimize, float feedrate) {
    Printer::unmarkAllSteppersDisabled(); // ??? hier wird nichts enabled. Nur markiert, auch wenn später oder früher "enablestepper" passiert.
    //evtl. weil dadurch in jedem fall gleich ein stepper aktiviert werden würde -> darum hier schon als aktiv markieren, weil umumgänglich ist.
    // Aber dann müsste man das (timingsicher) auch schon in den Funktionen über prepareDirectMove erledigt haben.

    PrintLine::waitForXFreeLines(1);

    uint8_t newPath = PrintLine::insertWaitMovesIfNeeded(pathOptimize, 0);
    PrintLine* p = PrintLine::getNextWriteLine();

    p->task = TASK_NO_TASK;
    p->flags = (abortAtEndstops ? FLAG_ABORT_AT_ENDSTOPS : 0);
    p->joinFlags = 0;
    p->dir = 0;
    if (!pathOptimize)
        p->setEndSpeedFixed(true);

    // Constrain Destinations to values we should be able to reach
    for (uint8_t axis = 0; axis < E_AXIS; axis++) {
        float direct = Printer::getDirectMM(axis);
        float endstop = Printer::maxSoftEndstopSteps[axis] * Printer::axisMMPerSteps[axis];
        if (Printer::destinationMM[axis] + direct >= endstop + 0.1) {
            Printer::destinationMM[axis] = (endstop + 0.1 - direct);
        }
    }

    float axisDistanceMM[4]; // Axis movement in mm
    // Find direction
    bool isNoStepsMove = true;
    for (uint8_t axis = 0; axis < 4; axis++) {
        //error zwischen soll und ist.
        float axisDistanceUnscaledMM = Printer::destinationMM[axis] - Printer::destinationMMLast[axis];

        if (axis == E_AXIS) {
#if FEATURE_DIGIT_FLOW_COMPENSATION
            float axisDistanceFactor = Printer::menuExtrusionFactor * Printer::dynamicExtrusionFactor;
#else
            float axisDistanceFactor = Printer::menuExtrusionFactor;
#endif // FEATURE_DIGIT_FLOW_COMPENSATION

            axisDistanceMM[E_AXIS] = axisDistanceUnscaledMM * axisDistanceFactor;
            Printer::extrudeMultiplyErrorSteps += axisDistanceMM[E_AXIS] * Printer::axisStepsPerMM[E_AXIS];
            p->delta[E_AXIS] = lroundf(Printer::extrudeMultiplyErrorSteps);
            Printer::extrudeMultiplyErrorSteps -= p->delta[E_AXIS];

            Printer::filamentPrinted += p->delta[E_AXIS] * Printer::axisMMPerSteps[E_AXIS];

            Printer::destinationMMLast[E_AXIS] = Printer::destinationMM[E_AXIS];
        } else {
            p->delta[axis] = lroundf(axisDistanceUnscaledMM * Printer::axisStepsPerMM[axis]);
            axisDistanceMM[axis] = float(p->delta[axis]) * Printer::axisMMPerSteps[axis];

            //Update calculated coordinate by move distance.
            Printer::destinationMMLast[axis] += axisDistanceMM[axis];
        }

        if (axisDistanceMM[axis] >= 0)
            p->setPositiveDirectionForAxis(axis);
        if (axisDistanceMM[axis] != 0)
            p->setMoveOfAxis(axis);

        // remove the sign from all driving length
        p->delta[axis] = abs(p->delta[axis]);
        axisDistanceMM[axis] = fabs(axisDistanceMM[axis]);

        // signal that we at least found some steps to queue in move cache
        // The other option is, that we only summed up parts steps for a possible next move which are stored as rounding errors
        if (p->delta[axis])
            isNoStepsMove = false;
    }

    // need to delete dummy elements, otherwise commands can get locked.
    if (isNoStepsMove) {
        if (newPath)
            PrintLine::resetPathPlanner();
        // No steps included
        return;
    }

#if FEATURE_HEAT_BED_Z_COMPENSATION
    // MAIN RULE:
    /* IF there is an extrusion move
          after a Z move
          to different layer height than before
          which is not 0
       THEN
          accept the z height value as new layer */

    // The following two move conditions might not follow each other in one move but in serveral queued moves.
    if (p->isZMove()) {
        int32_t newZ = Printer::getDestinationSteps(Z_AXIS);
        // Z achsen aufstieg/abstieg -> The ternary condition is there to filter out zlifts
        Printer::queuePositionZLayerGuessNew = (newZ == Printer::queuePositionZLayerCurrent) ? 0 : newZ;
    }

    if (p->isEPositiveMove()) {
        // We extrude. Do we have a new z layer height too?
        if (Printer::queuePositionZLayerGuessNew) {
            // It seems we have a new layer height, so shift layers down
            // Set current layer as last layer.
            Printer::queuePositionZLayerLast = Printer::queuePositionZLayerCurrent;
            // Set new layer as current layer
            Printer::queuePositionZLayerCurrent = Printer::queuePositionZLayerGuessNew;

            // If the new layer ist underneath the old layer (near the bed)
            // then this means we can assume the last-layer as 0 after some sort of startline.
            // We need this info to possibly have senseoffset running again for sequential multipart prints
            if (Printer::queuePositionZLayerLast > Printer::queuePositionZLayerCurrent) {
                if (Printer::queuePositionZLayerCurrent < Printer::axisStepsPerMM[Z_AXIS]) { //1mm
                    Printer::queuePositionZLayerLast = 0;
                }
            }

            // Problemfall Startmade: Wenn die Startmade aus etwas weniger als 16 od. MOVE_CACHE_SIZE Teilstücken besteht,
            // was man annehmen kann, dann springt der Pfadplaner über die Startmade einfach drüber und nimmt den ersten Layer als neue Referenz.
            // Das hier ist Preprocessing mit Blick in die Zukunft. Kurz steht in g_minZCompensationSteps z.B. 0.35, anschließend aber z.B. korrekte 0.2mm als g_minZCompensationSteps.
            // würde das nicht klappen, hätten wir SenseOffset in der Startmade was maximal schlecht sein kann, weil evtl. die Digits zu hoch sind.
            //-> Sollte mit dieser Automatik hier nicht vorkommen!
            if (!Printer::queuePositionZLayerLast && Printer::queuePositionZLayerCurrent < Printer::axisStepsPerMM[Z_AXIS]) {
                // < 1mm
                //hiermit hätten wir immer exakt 1 Lage, die der Drucker komplett mit dem Bettprofil abfährt, anschließend ab Layer 2 wird ausgeschlichen +ECMP.
                if (abs(Printer::queuePositionZLayerCurrent - Printer::axisStepsPerMM[Z_AXIS] * AUTOADJUST_STARTMADEN_AUSSCHLUSS) > 5 /* 2um um startmadenhöhe herum nichts tun */) {
                    g_minZCompensationSteps = Printer::queuePositionZLayerCurrent;
                    g_maxZCompensationSteps = g_minZCompensationSteps + (g_offsetZCompensationSteps - g_ZCompensationMax) * 20; //max zulässige kompensation pro lage: 1/20 = 5%
                }
            }
        }
        Printer::queuePositionZLayerGuessNew = 0;
    }
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

    // Define variables that are needed for the Bresenham algorithm. Please note that Z is not currently included in the Bresenham algorithm.
    if (p->delta[Y_AXIS] > p->delta[X_AXIS] && p->delta[Y_AXIS] > p->delta[Z_AXIS] && p->delta[Y_AXIS] > p->delta[E_AXIS])
        p->primaryAxis = Y_AXIS;
    else if (p->delta[X_AXIS] > p->delta[Z_AXIS] && p->delta[X_AXIS] > p->delta[E_AXIS])
        p->primaryAxis = X_AXIS;
    else if (p->delta[Z_AXIS] > p->delta[E_AXIS])
        p->primaryAxis = Z_AXIS;
    else
        p->primaryAxis = E_AXIS;
    p->stepsRemaining = p->delta[p->primaryAxis];

    if (p->isXYZMove()) {
        float xydist2 = axisDistanceMM[X_AXIS] * axisDistanceMM[X_AXIS] + axisDistanceMM[Y_AXIS] * axisDistanceMM[Y_AXIS];
        if (p->isZMove())
            p->distance = RMath::max((float)sqrt(xydist2 + axisDistanceMM[Z_AXIS] * axisDistanceMM[Z_AXIS]), axisDistanceMM[E_AXIS]);
        else
            p->distance = RMath::max((float)sqrt(xydist2), axisDistanceMM[E_AXIS]);
    } else {
        p->distance = axisDistanceMM[E_AXIS];
    }

    p->calculateMove(axisDistanceMM, p->primaryAxis, feedrate);

    updateTrapezoids();
#if USE_ADVANCE
    if (pathOptimize)
        waitRelax = 70;
#endif // USE_ADVANCE

    // Make result permanent
    pushLine();
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
        direct.delta[axis] = Printer::directDestinationSteps[axis] - Printer::directCurrentSteps[axis];
        // no special extrusion handling. this direct drive is only for manual move and button feed. no precision needed.
        if (direct.delta[axis] >= 0)
            direct.setPositiveDirectionForAxis(axis);
        else
            direct.delta[axis] = -direct.delta[axis];
        axisDistanceMM[axis] = fabs(direct.delta[axis] * Printer::axisMMPerSteps[axis]);
        if (direct.delta[axis])
            direct.setMoveOfAxis(axis);
    }
    if (direct.isNoMove()) {
        direct.stepsRemaining = 0;
        direct.unblock();
        direct.task = TASK_NO_TASK;
        return; // No steps included
    }

    // Define variables that are needed for the Bresenham algorithm. Please note that Z is not currently included in the Bresenham algorithm.
    if (direct.delta[Y_AXIS] > direct.delta[X_AXIS] && direct.delta[Y_AXIS] > direct.delta[Z_AXIS] && direct.delta[Y_AXIS] > direct.delta[E_AXIS])
        direct.primaryAxis = Y_AXIS;
    else if (direct.delta[X_AXIS] > direct.delta[Z_AXIS] && direct.delta[X_AXIS] > direct.delta[E_AXIS])
        direct.primaryAxis = X_AXIS;
    else if (direct.delta[Z_AXIS] > direct.delta[E_AXIS])
        direct.primaryAxis = Z_AXIS;
    else
        direct.primaryAxis = E_AXIS;

    if (direct.isXYZMove()) {
        float xydist2 = axisDistanceMM[X_AXIS] * axisDistanceMM[X_AXIS] + axisDistanceMM[Y_AXIS] * axisDistanceMM[Y_AXIS];
        if (direct.isZMove())
            direct.distance = RMath::max((float)sqrt(xydist2 + axisDistanceMM[Z_AXIS] * axisDistanceMM[Z_AXIS]), axisDistanceMM[E_AXIS]);
        else
            direct.distance = RMath::max((float)sqrt(xydist2), axisDistanceMM[E_AXIS]);
    } else
        direct.distance = axisDistanceMM[E_AXIS];

    direct.stepsRemaining = direct.delta[direct.primaryAxis];

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
    if (PrintLine::direct.isXYZMove()) {
        // decelerate and stop
        if (PrintLine::direct.stepsRemaining > 32) //die genaue anzahl der Decelerate Steps sollte hier eigentlich fast egal sein. Besser evtl. die Microsteps der Hauptachse?
        {
            PrintLine::direct.stepsRemaining = 32;
        }
    }
    return;
} // stopDirectMove

void PrintLine::calculateMove(float axisDistanceMM[], fast8_t drivingAxis, float feedrate) {
    if (stepsRemaining == 0) { // need at least one step for bresenham
        return;
    }
    float timeForMove = (float)(F_CPU)*distance / feedrate; // time is in ticks
    // Small element limiter: This was not present in directmove but is not harmfull.
    if (linesCount < MOVE_CACHE_LOW && timeForMove < LOW_TICKS_PER_MOVE) // Limit speed to keep cache full.
    {
        timeForMove += ((LOW_TICKS_PER_MOVE - timeForMove)) * 3 / (linesCount + 1); // Increase time if queue gets empty. Add more time if queue gets smaller.
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

    // Errors for delta move are initialized in timer (except extruder)
    error[X_AXIS] = error[Y_AXIS] = error[Z_AXIS] = delta[primaryAxis] >> 1;

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

#if USE_ADVANCE
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
#endif // USE_ADVANCE

    DEBUG_MEMORY;
} // calculateMove

/** \brief
This is the path planner.
It goes from the last entry and tries to increase the end speed of previous moves in a fashion that the maximum jerk
is never exceeded. If a segment with reached maximum speed is met, the planner stops. Everything left from this
is already optimal from previous updates.
The first 2 entries in the queue are not checked. The first is the one that is already in print and the following will likely become active.

The method is called before linesCount is increased!
*/
void PrintLine::updateTrapezoids() {
    uint8_t first = linesWritePos;
    PrintLine* firstLine;
    PrintLine* act = &lines[linesWritePos];
    InterruptProtectedBlock noInts; //BEGIN_INTERRUPT_PROTECTED;

    // First we find out how far back we could go with optimization.

    uint8_t maxfirst = linesPos; // first non fixed segment

    if (maxfirst != linesWritePos) {
        nextPlannerIndex(maxfirst); // don't touch the line printing
    }

    // Now ignore enough segments to gain enough time for path planning
    millis_t timeleft = 0;

    // Skip as many stored moves as needed to gain enough time for computation
    //millis_t minTime = 4500L * RMath::min(MOVE_CACHE_SIZE,10);

#if MOVE_CACHE_SIZE < 10
#define minTime 4500L * MOVE_CACHE_SIZE
#else
#define minTime 45000L
#endif

    while (timeleft < minTime && maxfirst != linesWritePos) {
        timeleft += lines[maxfirst].timeInTicks;
        nextPlannerIndex(maxfirst);
    }

    // Search last fixed element
    while (first != maxfirst && !lines[first].isEndSpeedFixed())
        previousPlannerIndex(first);

    if (first != linesWritePos && lines[first].isEndSpeedFixed())
        nextPlannerIndex(first);
    // now first points to last segment before the end speed is fixed
    // so start speed is also fixed.

    if (first == linesWritePos) // Nothing to plan
    {
        act->block();       // Prevent stepper interrupt from using this
        noInts.unprotect(); //ESCAPE_INTERRUPT_PROTECTED
        act->setStartSpeedFixed(true);
        act->updateStepsParameter();
        act->unblock();
        return;
    }

    /** \brief now we have at least one additional move for optimization
    that is not a wait move
    First is now the new element or the first element with non fixed end speed.
    anyhow, the start speed of first is fixed  */
    firstLine = &lines[first];
    firstLine->block(); // don't let printer touch this or following segments during update
    noInts.unprotect(); //END_INTERRUPT_PROTECTED;

    uint8_t previousIndex = linesWritePos;
    previousPlannerIndex(previousIndex);
    PrintLine* previous = &lines[previousIndex];

    // filters z-move<->not z-move //Nibbels: Test if this is better with our type of Z-Comp because of bad edges? See https://github.com/repetier/Repetier-Firmware/commit/5fbe3748a0ca55386d5315d5b44c4209bec62fc2#diff-593812a66d7348c87b711b15b1ad5195L696
    /*
    if((previous->primaryAxis == Z_AXIS && act->primaryAxis != Z_AXIS) || (previous->primaryAxis != Z_AXIS && act->primaryAxis == Z_AXIS))
    {
        previous->setEndSpeedFixed(true);
        act->setStartSpeedFixed(true);
        act->updateStepsParameter();
        firstLine->unblock();
        return;
    }
    */

    //Retract:
    if (previous->isEOnlyMove() != act->isEOnlyMove()) {
        previous->setEndSpeedFixed(true);
        act->setStartSpeedFixed(true);
        act->updateStepsParameter();
        firstLine->unblock();
        return;
    }

#if USE_ADVANCE
    /**
     * If we start/stop extrusion we need to do so with lowest possible end speed
     * or advance would leave a drolling extruder and can not adjust fast enough.
     *
     * https://github.com/repetier/Repetier-Firmware/issues/837
     * This exception rule is here for a reason:
     * Case I want to catch is fast travel move and then start with a high extrusion speed.
     * That means if angle is flat you will start with high extrusion speed and need to build up extruder pressure at an instance.
     * So here starting at lower speed makes adding advance steps easy.
     */
    if (Printer::isAdvanceActivated() && previous->isEMove() != act->isEMove()) {
        previous->setEndSpeedFixed(true);
        act->setStartSpeedFixed(true);
        act->updateStepsParameter();
        firstLine->unblock();
        return;
    }
#endif // USE_ADVANCE

    // Set maximum junction speed if we have a real move before
    computeMaxJunctionSpeed(previous, act);
    // Increase speed if possible neglecting current speed
    backwardPlanner(linesWritePos, first);
    // Reduce speed to reachable speeds
    forwardPlanner(first);

    // Update precomputed data
    do {
        lines[first].updateStepsParameter();

        //noInts.protect(); //BEGIN_INTERRUPT_PROTECTED;
        lines[first].unblock(); // start with first block to release next used segment as early as possible
        nextPlannerIndex(first);
        lines[first].block();
        //noInts.unprotect(); //END_INTERRUPT_PROTECTED;

    } while (first != linesWritePos);

    act->updateStepsParameter();
    act->unblock();
} // updateTrapezoids

/* Computes the maximum junction speed of the newly added segment under
optimal conditions. There is no guarantee that the previous move will be able to reach the
speed at all, but if it could exceed it will never exceed this theoretical limit.
if you define ALTERNATIVE_JERK the new jerk computations are used. These
use the cosine of the angle and the maximum speed
Jerk = (1-cos(alpha))*min(v1,v2)
This sets jerk to 0 on zero angle change.
        Old               New
0°:       0               0
30°:     51,8             13.4
45°:     76.53            29.3
90°:    141               100
180°:   200               200
Speed from 100 to 200
        Old               New(min)   New(max)
0°:     100               0          0
30°:    123,9             13.4       26.8
45°:    147.3             29.3       58.6
90°:    223               100        200
180°:   300               200        400
*/

inline void PrintLine::computeMaxJunctionSpeed(PrintLine* previous, PrintLine* current) {
    // if we are here we have two identical move types
    // either pure extrusion -> pure extrusion or
    // move -> move (with or without extrusion)
    // First we compute the normalized jerk for speed 1

    float factor = 1.0;
    float lengthFactor = 1.0;
#if REDUCE_ON_SMALL_SEGMENTS
    if (previous->distance < MAX_JERK_DISTANCE)
        lengthFactor = static_cast<float>(MAX_JERK_DISTANCE * MAX_JERK_DISTANCE) / (previous->distance * previous->distance);
#endif
    float maxJoinSpeed = RMath::min(current->fullSpeed, previous->fullSpeed);

#if ALTERNATIVE_JERK
    float calculatedJerk = maxJoinSpeed * lengthFactor * (1.0 - (current->speedX * previous->speedX + current->speedY * previous->speedY + current->speedZ * previous->speedZ) / (current->fullSpeed * previous->fullSpeed));
#else
    float dx = current->speedX - previous->speedX;
    float dy = current->speedY - previous->speedY;
    float calculatedJerk = sqrt(dx * dx + dy * dy) * lengthFactor;
#endif // ALTERNATIVE_JERK

    if (calculatedJerk > Printer::maxXYJerk) {
        factor = Printer::maxXYJerk / calculatedJerk; // always < 1.0!
        if (factor * maxJoinSpeed * 2.0 < Printer::maxXYJerk)
            factor = Printer::maxXYJerk / (2.0 * maxJoinSpeed);
    }

    if ((previous->dir | current->dir) & 64 /* previous zmove oder current zmove */) {
        float zJerk = fabs(current->speedZ - previous->speedZ);
        if (zJerk > Printer::maxZJerk)
            factor = RMath::min(factor, Printer::maxZJerk / zJerk);
    }

    float eJerk = fabs(current->speedE - previous->speedE);
    if (eJerk > Extruder::current->maxEJerk) {
        factor = RMath::min(factor, Extruder::current->maxEJerk / eJerk);
    }
    previous->maxJunctionSpeed = maxJoinSpeed * factor; // set speed limit
} // computeMaxJunctionSpeed

/** \brief Update parameter used by updateTrapezoids
Computes the acceleration/deceleration steps and advanced parameter associated.
*/
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

/** \brief
Compute the maximum speed from the last entered move.
The backwards planner traverses the moves from last to first looking at deceleration. The RHS of the accelerate/decelerate ramp.

start = last line inserted
last = last element until we check
*/
inline void PrintLine::backwardPlanner(uint8_t start, uint8_t last) {
    PrintLine *act = &lines[start], *previous;
    float lastJunctionSpeed = act->endSpeed; // Start always with safe speed

    while (start != last) {
        previousPlannerIndex(start);
        previous = &lines[start];
        previous->block();
        // Avoid speed calculate once cruising in split delta move
        // Avoid speed calculate if we know we can accelerate within the line
        lastJunctionSpeed = (act->isNominalMove() ? act->fullSpeed : sqrt(lastJunctionSpeed * lastJunctionSpeed + act->accelerationDistance2)); // acceleration is acceleration*distance*2! What can be reached if we try?

        // If that speed is more that the maximum junction speed allowed then ...
        if (lastJunctionSpeed >= previous->maxJunctionSpeed) // Limit is reached
        {
            // If the previous line's end speed has not been updated to maximum speed then do it now
            if (previous->endSpeed != previous->maxJunctionSpeed) {
                previous->invalidateParameter();                                                 // Needs recomputation
                previous->endSpeed = RMath::max(previous->minSpeed, previous->maxJunctionSpeed); // possibly unneeded???
            }

            // If actual line start speed has not been updated to maximum speed then do it now
            if (act->startSpeed != previous->maxJunctionSpeed) {
                act->startSpeed = RMath::max(act->minSpeed, previous->maxJunctionSpeed); // possibly unneeded???
                act->invalidateParameter();
            }
            lastJunctionSpeed = previous->endSpeed;
        } else {
            // Block prev end and act start as calculated speed and recalculate plateau speeds (which could move the speed higher again)
            act->startSpeed = RMath::max(act->minSpeed, lastJunctionSpeed);
            lastJunctionSpeed = previous->endSpeed = RMath::max(lastJunctionSpeed, previous->minSpeed);
            previous->invalidateParameter();
            act->invalidateParameter();
        }
        act = previous;
    } // while loop

} // backwardPlanner

void PrintLine::forwardPlanner(uint8_t first) {
    PrintLine* act;
    PrintLine* next = &lines[first];
    float vmaxRight;
    float leftSpeed = next->startSpeed;
    while (first != linesWritePos) // All except last segment, which has fixed end speed
    {
        act = next;
        nextPlannerIndex(first);
        next = &lines[first];
        // Avoid speed calculate if we know we can accelerate within the line.
        vmaxRight = (act->isNominalMove() ? act->fullSpeed : sqrt(leftSpeed * leftSpeed + act->accelerationDistance2));
        if (vmaxRight > act->endSpeed) // Could be higher next run?
        {
            if (leftSpeed < act->minSpeed) {
                leftSpeed = act->minSpeed;
                act->endSpeed = sqrt(leftSpeed * leftSpeed + act->accelerationDistance2);
            }
            act->startSpeed = leftSpeed;
            next->startSpeed = leftSpeed = RMath::max(RMath::min(act->endSpeed, act->maxJunctionSpeed), next->minSpeed);
            if (act->endSpeed == act->maxJunctionSpeed) // Full speed reached, don't compute again!
            {
                act->setEndSpeedFixed(true);
                next->setStartSpeedFixed(true);
            }
            act->invalidateParameter();
        } else // We can accelerate full speed without reaching limit, which is as fast as possible. Fix it!
        {
            act->fixStartAndEndSpeed();
            act->invalidateParameter();
            if (act->minSpeed > leftSpeed) {
                leftSpeed = act->minSpeed;
                vmaxRight = sqrt(leftSpeed * leftSpeed + act->accelerationDistance2);
            }
            act->startSpeed = leftSpeed;
            act->endSpeed = RMath::max(act->minSpeed, vmaxRight);
            next->startSpeed = leftSpeed = RMath::max(RMath::min(act->endSpeed, act->maxJunctionSpeed), next->minSpeed);
            next->setStartSpeedFixed(true);
        }
    }                                                         // While
    next->startSpeed = RMath::max(next->minSpeed, leftSpeed); // This is the new segment, wgich is updated anyway, no extra flag needed.
} // forwardPlanner

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
    if (linesCount == 0
#if USE_ADVANCE
        && waitRelax == 0
#endif                   // USE_ADVANCE
        && pathOptimize) // First line after some time - warmup needed
    {
        uint8_t w = 4;
        while (w--) {
            PrintLine* p = getNextWriteLine();
            p->flags = FLAG_WARMUP;
            p->joinFlags = FLAG_JOIN_STEPPARAMS_COMPUTED | FLAG_JOIN_END_FIXED | FLAG_JOIN_START_FIXED;
            p->dir = 0;
            p->setWaitForXLinesFilled(w + waitExtraLines);
            p->setWaitTicks(100000); //repetier changed this from 25k to 100k in https://github.com/repetier/Repetier-Firmware/commit/2385051856278c189d7c1f1fd67acc27825c82ac#diff-11347f18746fb080f5bda21c30428558R1080
            pushLine();
        }
        return 1;
    }
    return 0;
} // insertWaitMovesIfNeeded

void PrintLine::waitForXFreeLines(uint8_t b) {
    while (getLinesCount() + b > MOVE_CACHE_SIZE) // wait for a free entry in movement cache
    {
        Commands::checkForPeriodicalActions(Processing);
    }
} // waitForXFreeLines

#if FEATURE_ARC_SUPPORT
// Arc function taken from grbl
// The arc is approximated by generating a huge number of tiny, linear segments. The length of each
// segment is configured in settings.mm_per_arc_segment.
void PrintLine::arc(float* position, float* target, float* offset, float radius, uint8_t isclockwise) {
    //   int acceleration_manager_was_enabled = plan_is_acceleration_manager_enabled();
    //   plan_set_acceleration_manager_enabled(false); // disable acceleration management for the duration of the arc
    float center_axis0 = position[X_AXIS] + offset[X_AXIS];
    float center_axis1 = position[Y_AXIS] + offset[Y_AXIS];
    //float linear_travel = 0; //target[axis_linear] - position[axis_linear];
    float extruder_travel = target[E_AXIS] - position[E_AXIS]; //das kann nicht anders sein, als dass man die extrusion im gcode angibt. man muss vorher verindern, dass in der G3 funktion direkt extrudiert wird.
    //float extruder_travel = (Printer::destinationSteps[E_AXIS] - Printer::currentPositionSteps[E_AXIS]) * Printer::invAxisStepsPerMM[E_AXIS];
    float r_axis0 = -offset[0]; // Radius vector from center to current location
    float r_axis1 = -offset[1];
    float rt_axis0 = target[0] - center_axis0;
    float rt_axis1 = target[1] - center_axis1;
    /*long xtarget = Printer::destinationSteps[X_AXIS];
    long ytarget = Printer::destinationSteps[Y_AXIS];
    long ztarget = Printer::destinationSteps[Z_AXIS];
    long etarget = Printer::destinationSteps[E_AXIS];
    */
    // CCW angle between position and target from circle center. Only one atan2() trig computation required.
    float angular_travel = atan2(r_axis0 * rt_axis1 - r_axis1 * rt_axis0, r_axis0 * rt_axis0 + r_axis1 * rt_axis1);
    if ((!isclockwise && angular_travel <= 0.00001) || (isclockwise && angular_travel < -0.000001)) {
        angular_travel += 2.0f * M_PI;
    }
    if (isclockwise) {
        angular_travel -= 2.0f * M_PI;
    }

    float millimeters_of_travel = fabs(angular_travel) * radius; //hypot(angular_travel*radius, fabs(linear_travel));
    if (millimeters_of_travel < 0.001f) {
        return; // treat as succes because there is nothing to do;
    }
    //uint16_t segments = (radius>=BIG_ARC_RADIUS ? floor(millimeters_of_travel/MM_PER_ARC_SEGMENT_BIG) : floor(millimeters_of_travel/MM_PER_ARC_SEGMENT));
    // Increase segment size if printing faster then computation speed allows
    uint16_t segments = (Printer::feedrate > 60.0f ? floor(millimeters_of_travel / RMath::min(static_cast<float>(MM_PER_ARC_SEGMENT_BIG), Printer::feedrate * 0.01666f * static_cast<float>(MM_PER_ARC_SEGMENT))) : floor(millimeters_of_travel / static_cast<float>(MM_PER_ARC_SEGMENT)));
    if (segments == 0)
        segments = 1;
    /*
    // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
    // by a number of discrete segments. The inverse feed_rate should be correct for the sum of
    // all segments.
    if (invert_feed_rate) { feed_rate *= segments; }
    */
    float theta_per_segment = angular_travel / segments;
    //float linear_per_segment = linear_travel/segments;
    float extruder_per_segment = extruder_travel / segments;

    /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
    and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
    r_T = [cos(phi) -sin(phi);
    sin(phi)  cos(phi] * r ;

    For arc generation, the center of the circle is the axis of rotation and the radius vector is
    defined from the circle center to the initial position. Each line segment is formed by successive
    vector rotations. This requires only two cos() and sin() computations to form the rotation
    matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
    all double numbers are single precision on the Arduino. (True double precision will not have
    round off issues for CNC applications.) Single precision error can accumulate to be greater than
    tool precision in some cases. Therefore, arc path correction is implemented.

    Small angle approximation may be used to reduce computation overhead further. This approximation
    holds for everything, but very small circles and large mm_per_arc_segment values. In other words,
    theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
    to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
    numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
    issue for CNC machines with the single precision Arduino calculations.

    This approximation also allows mc_arc to immediately insert a line segment into the planner
    without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
    a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead.
    This is important when there are successive arc motions.
    */
    // Vector rotation matrix values
    float cos_T = 1 - 0.5 * theta_per_segment * theta_per_segment; // Small angle approximation
    float sin_T = theta_per_segment;

    float arc_target[4];
    float sin_Ti;
    float cos_Ti;
    float r_axisi;
    uint16_t i;
    int8_t count = 0;

    // Initialize the linear axis
    //arc_target[axis_linear] = position[axis_linear];

    // Initialize the extruder axis
    arc_target[E_AXIS] = Printer::destinationMM[E_AXIS];
    //arc_target[E_AXIS] = Printer::currentPositionSteps[E_AXIS] * Printer::invAxisStepsPerMM[E_AXIS];

    for (i = 1; i < segments; i++) {
        // Increment (segments-1)

        if ((count & 3) == 0) {
            //GCode::readFromSerial();
            Commands::checkForPeriodicalActions(Processing);
            //UI_MEDIUM; // do check encoder
        }

        if (count < N_ARC_CORRECTION) { //25 pieces
                                        // Apply vector rotation matrix
            r_axisi = r_axis0 * sin_T + r_axis1 * cos_T;
            r_axis0 = r_axis0 * cos_T - r_axis1 * sin_T;
            r_axis1 = r_axisi;
            count++;
        } else {
            // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
            // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
            cos_Ti = cos(i * theta_per_segment);
            sin_Ti = sin(i * theta_per_segment);
            r_axis0 = -offset[0] * cos_Ti + offset[1] * sin_Ti;
            r_axis1 = -offset[0] * sin_Ti - offset[1] * cos_Ti;
            count = 0;
        }

        // Update arc_target location
        arc_target[X_AXIS] = center_axis0 + r_axis0;
        arc_target[Y_AXIS] = center_axis1 + r_axis1;
        //arc_target[axis_linear] += linear_per_segment;
        arc_target[E_AXIS] += extruder_per_segment;
        Printer::queueFloatCoordinates(arc_target[X_AXIS], arc_target[Y_AXIS], IGNORE_COORDINATE, arc_target[E_AXIS], IGNORE_COORDINATE);
    }
    // Ensure last segment arrives at target location.
    Printer::queueFloatCoordinates(target[X_AXIS], target[Y_AXIS], IGNORE_COORDINATE, target[E_AXIS], IGNORE_COORDINATE);
}
#endif

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
void PrintLine::stepSlowedZCompensation() {
    if (Printer::blockAll) {
        return;
    }

    // We do not want the zCMP to have a total axis dependand constant speed.
    // Constant jerky speedup and speeddowns sound awfull and we suspect that to cause the z-lift problematic.

    // So we start slowly and get faster with outputting steps by subtracting future wait steps from "mass".
    // When we reach a small amount of leftover todo steps we add something to "mass" to slowdown.
    // Idle zCmp adds to "mass" so we have a slow next start.

#define massWeight 15
    static uint8_t waitSteps = 0;
    static uint8_t mass = massWeight;
    if (waitSteps) {
        waitSteps--;
        return;
    }

    int32_t cmpDiff = Printer::compensatedPositionTargetStepsZ - Printer::compensatedPositionCurrentStepsZ;

    if (cmpDiff > 0) {
        // here we shall move the z-axis only in case performQueueMove() is not moving into the other direction at the moment
        bool posZ = Printer::getZDirectionIsPos();
        if (!posZ) {
            posZ = true;
            // set the direction only in case it is not set already
            Printer::setZDirection(posZ);
        }
        // we must move the heat bed do the bottom
        if (posZ) {
            Printer::startZStep();
#if STEPPER_HIGH_DELAY > 0
            HAL::delayMicroseconds(STEPPER_HIGH_DELAY);
#endif // STEPPER_HIGH_DELAY>0
            Printer::compensatedPositionCurrentStepsZ++;

            // adjust a sort of acceleration
            if (mass < massWeight && abs(cmpDiff) < massWeight)
                mass++;
            else if (mass)
                mass--;

            Printer::endZStep();
            waitSteps = mass + Printer::stepsPerTimerCall;

            return;
        }
    }

    if (cmpDiff < 0) {
        // here we shall move the z-axis only in case performQueueMove() is not moving into the other direction at the moment
        bool posZ = Printer::getZDirectionIsPos();
        if (posZ) {
            posZ = false;
            // set the direction only in case it is not set already
            Printer::setZDirection(posZ);
        }
        // we must move the heat bed to the top
        if (!posZ) {
            Printer::startZStep();
#if STEPPER_HIGH_DELAY > 0
            HAL::delayMicroseconds(STEPPER_HIGH_DELAY);
#endif // STEPPER_HIGH_DELAY>0

            // adjust a sort of acceleration
            Printer::compensatedPositionCurrentStepsZ--;
            if (mass < massWeight && abs(cmpDiff) < massWeight)
                mass++;
            else if (mass)
                mass--;

            Printer::endZStep();
            waitSteps = mass + Printer::stepsPerTimerCall;

            return;
        }
    }

    // Idlen baut Timeout auf, also Trägheit
    // Steppen baut das ab.
    if (!cmpDiff && mass < massWeight) {
        mass++;
    }

    // Do not calculate this work more often then needed.
    waitSteps = mass + Printer::stepsPerTimerCall;
}

long PrintLine::needCmpWait() {
    // Pause a bit, if z-compensation is way out of line: this is usefull when starting prints using very deep bed-leveling and custom z-endstop switches which can override a lot.
    uint16_t ZcmpNachlauf = abs(Printer::compensatedPositionCurrentStepsZ - Printer::compensatedPositionTargetStepsZ);

    //wenn die zkompensation wegen z.B. Startposition über 0.25mm hinterherhängt: pause und warten.
    if (ZcmpNachlauf > (uint16_t(Printer::axisStepsPerMM[Z_AXIS]) >> 2) && Printer::compensatedPositionTargetStepsZ) {
        return true;
    }

    if (ZcmpNachlauf && !Printer::doHeatBedZCompensation) {
        if (!Printer::checkCMPblocked()) { //wenn blocked, kann die nächste bewegung das aufheben. Hier gehts nur um wenige steps in Z, da ist das egal, wenn mal nicht kurz pausiert wird.
            return true;
        }
    }

    return false; //ignore this.
}
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

/**
  Processes the moves from the queue and moves the stepper motors one step. If the last step is reached, the next movement from the queue is started.
  The function must be called from a timer loop. It returns the time for the next call. */
volatile long queueError;
long PrintLine::performQueueMove() {
    if (cur == NULL) {
        if (!linesCount) {
            HAL::forbidInterrupts();
            return 1000;
        }

        setCurrentLine();

        switch (cur->task) {
        case TASK_NO_TASK: {
            break;
        }

        case TASK_PAUSE_PRINT: {
            removeCurrentLineForbidInterrupt();
            // the printing shall be paused without moving of the printer/miller head
            if (!g_pauseMode) {
                g_uPauseTime = HAL::timeInMilliseconds();
                g_pauseMode = PAUSE_MODE_PAUSED; // Blocks Queue
                g_pauseStatus = PAUSE_STATUS_GOTO_PAUSE;
                g_pauseBeepDone = 0;
            }
            return 1000;
        }

        case TASK_PAUSE_PRINT_AND_MOVE: {
            removeCurrentLineForbidInterrupt();
            // the printing shall be paused with moving of the printer/miller head to the pause position
            if (!g_pauseMode) {
                g_uPauseTime = HAL::timeInMilliseconds();
                g_pauseMode = PAUSE_MODE_PAUSED_AND_MOVED; // Blocks Queue
                g_pauseStatus = PAUSE_STATUS_GOTO_PAUSE_AND_MOVE;
                g_pauseBeepDone = 0;
            }
            return 1000;
        }

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
        case TASK_ENABLE_Z_COMPENSATION: // M3001 M3141
        {
            Printer::enableCMPnow();
            removeCurrentLineForbidInterrupt();
            return 1000;
        }

        case TASK_DISABLE_Z_COMPENSATION: // M3000 M3140
        {
            // disable the z compensation
            Printer::disableCMPnow(); //hier nicht unbedingt warten, das soll im fluss der queue einfach abgeschaltet werden. zu große abstände regelt die needCmpWait
            removeCurrentLineForbidInterrupt();
            return 1000;
        }

#if FEATURE_SENSIBLE_PRESSURE
        case TASK_ENABLE_SENSE_OFFSET: // M3909
        {
            Printer::enableSenseOffsetnow();
            removeCurrentLineForbidInterrupt();
            return 1000;
        }
#endif // FEATURE_SENSIBLE_PRESSURE
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

        case TASK_UNLOCK_DISPLAY_MSG: // M3117 unlock
        {
            // allow to overwrite the current string again
            uid.unlock();
            removeCurrentLineForbidInterrupt();
            return 1000;
        }

        default: {
            // this is an unknown task - we should never end up here
            removeCurrentLineForbidInterrupt();
            return 1000;
        }
        }

        if (cur->isBlocked()) // This step is in computation - shouldn't happen
        {
            cur = NULL;
            return 2000;
        }
        HAL::allowInterrupts();

        if (cur->isWarmUp()) {
            // This is a warmup move to initalize the path planner correctly. Just waste
            // a bit of time to get the planning up to date.
            if (linesCount <= cur->getWaitForXLinesFilled()) {
                cur = NULL;
                return 2000;
            }

            long wait = cur->getWaitTicks();
            removeCurrentLineForbidInterrupt();
            return (wait); // waste some time for path optimization to fill up
        }

        cur->enableSteppers(); //set Z direction etc.
        cur->adjustDirections();
        Printer::lastDirectionSovereignty = DIR_QUEUE;
        cur->fixStartAndEndSpeed();

        queueError = cur->delta[cur->primaryAxis];
        if (!cur->areParameterUpToDate())
            cur->updateStepsParameter(); // should never happen, but with bad timings???
        Printer::vMaxReached[FOR_QUEUE] = cur->vStart;
        Printer::stepNumber[FOR_QUEUE] = 0;
        Printer::timer[FOR_QUEUE] = 0;
        Printer::v = cur->fullSpeed;

        HAL::forbidInterrupts();

#if USE_ADVANCE
        cur->updateAdvanceSteps(cur->vStart); //startet advance extruder "etwas" vor der ersten bewegung: gut? ist Printer::interval immer klein genug?
#endif                                        // USE_ADVANCE

        return (Printer::interval >> 1); //wait 50% to next interrupt.
    }

    if (Printer::lastDirectionSovereignty != DIR_QUEUE) {
        Printer::lastDirectionSovereignty = DIR_QUEUE;
        Printer::v = cur->fullSpeed;
        cur->adjustDirections();
#if USE_ADVANCE
        cur->updateAdvanceSteps(Printer::vMaxReached[DIR_QUEUE]);
#endif // USE_ADVANCE
    }

    return performMove(cur, FOR_QUEUE);
} // performQueueMove

/**
 * Check if our queued move is within or without specified print volume
 *
 * The coordinate system might sometimes move out of the print volume box but the printer is not allowed to move then.
 */
bool inBauraum(uint8_t axisXY, PrintLine* move, uint8_t forQueue) {
    // entry assumes that isXMove is true

    if (Printer::isMaxEndstopHit(axisXY)) {
        if (move->isPositiveMoveOfAxis(axisXY)) {
            // all directMoves have abort set, some queueMoves too (Homing and optional scans etc.)
            if (move->isAbortAtEndstops()) {
                move->setMoveOfAxisFinished(axisXY);
                if (!forQueue)
                    Printer::stopDirectAxis(axisXY);
                Printer::outOfPrintVolume[axisXY] = 0;

                return false;
            }
            // If we do not abort the Z-move outside of boundarys we will count the oversteps and block the physical step
            Printer::outOfPrintVolume[axisXY]++;

            return false;
        }

        if (Printer::outOfPrintVolume[axisXY] > 0) {
            // If we are in the endstop and have oversteps while moving z to minus we pay back ignored plusZ by ignoring minusZ until overMaxEndstopZ is even.
            Printer::outOfPrintVolume[axisXY]--;

            return false;
        }

        return true;
    }

    if (Printer::isMinEndstopHit(axisXY)) {
        if (move->isNegativeMoveOfAxis(axisXY)) {
            // all directMoves have abort set, some queueMoves too (Homing and optional scans etc.)
            if (move->isAbortAtEndstops()) {
                move->setMoveOfAxisFinished(axisXY);
                if (!forQueue)
                    Printer::stopDirectAxis(axisXY);
                Printer::outOfPrintVolume[axisXY] = 0;

                return false;
            }
            // If we do not abort the Z-move outside of boundarys we will count the oversteps and block the physical step
            Printer::outOfPrintVolume[axisXY]--;

            return false;
        }

        if (Printer::outOfPrintVolume[axisXY] < 0) {
            // If we are in the endstop and have oversteps while moving z to minus we pay back ignored plusZ by ignoring minusZ until overMaxEndstopZ is even.
            Printer::outOfPrintVolume[axisXY]++;

            return false;
        }

        return true;
    }

    /* Angeblich kein Knopf gedrückt, aber overSteps übrig. -> overSteps zurückbauen und output ignorieren, der Schalter könnte wackeln. */
    if (Printer::outOfPrintVolume[axisXY] != 0) {
        if (Printer::outOfPrintVolume[axisXY] > 0 && move->isNegativeMoveOfAxis(axisXY)) {
            Printer::outOfPrintVolume[axisXY]--;

            return false;
        } else if (Printer::outOfPrintVolume[axisXY] < 0 && move->isPositiveMoveOfAxis(axisXY)) {
            Printer::outOfPrintVolume[axisXY]++;

            return false;
        }
    }

    return true;
}

/**
* This special Endstop treatment is only needed because if we have preplanned moves they will not care about relative move-back-restrictions otherwise.
* we could just handle this like in x and y by comparing the coordinates towards real step count, but within Z we have all sorts of z-manipulation.
* so I introduce one more static long to count steps we ignored while hitting an endstop. Those steps first have to by "payed back" to unlock moving z away from this endstop again.
*
* Even if z compensation would still adjust somehow those remembered steps have to be undone before the next relative queuemove can drive z again.
* But on that height z-compensation is equal for all x-y-positions
*/
bool inBauraumZ(PrintLine* move, uint8_t forQueue) {
    if (Printer::isZMaxEndstopHit()) {
        if (move->isZPositiveMove()) {
            if (move->isAbortAtEndstops()) { // all directMoves have abort set, some queueMoves too (Homing and optional scans etc.)
                move->setZMoveFinished();
                if (!forQueue)
                    Printer::stopDirectAxis(Z_AXIS);
                Printer::outOfPrintVolumeZ = 0;

                return false;
            }
            // If we do not abort the Z-move outside of boundarys we will count the oversteps and block the physical step
            Printer::outOfPrintVolumeZ++;

            return false;
        }

        if (Printer::outOfPrintVolumeZ != 0) {
            // If we are in the endstop and have oversteps while moving z to minus we pay back ignored plusZ by ignoring minusZ until overMaxEndstopZ is even.
            Printer::outOfPrintVolumeZ--;

            return false;
        }

        return true;
    }

    if (Printer::isZMinEndstopHit()) {
        /**
        * Printer Z-Min is a very special endstop. It is not being handled by coordinate virtualizing, because:
        *
        * There is no chance that we will end up being limited by queue-Moves endstop crossing here
        * because near z-min we have a warped z-coordinate system
        * The Gcode coordinate queue is always beleaving it drives at constant levels (when driving lower than z-min-endstop as example).
        * The Gcode coordinate queue cannot set gcode coordinates lower than Z=0 even if the system drives lower than z-min-endstop.
        */

        // unhomed stop
        // directDrive stop
        // if not then we are homed
        // we allow to overdrive Z-min a little bit so that also G-Codes are able to move to a smaller z-position even when Z-min has fired already
        if (move->isZNegativeMove()) {
            if (
                !Printer::isAxisHomed(Z_AXIS)
                || (!forQueue && move->task == DIRECT_RUNNING_STOPPABLE)
                || (Printer::currentZSteps <= -1 * long(Printer::maxZOverrideSteps))) {
                move->setZMoveFinished();
                Printer::outOfPrintVolumeZ = 0;

                return false;
            }
        }

        return true;
    }

    /* Angeblich kein Knopf gedrückt, aber overSteps übrig. -> overSteps zurückbauen und output ignorieren, der Schalter könnte wackeln. */
    else if (Printer::outOfPrintVolumeZ > 0) {
        // If we are in the endstop and have oversteps while moving z to minus we pay back ignored plusZ by ignoring minusZ until overMaxEndstopZ is even.
        Printer::outOfPrintVolumeZ--;

        return false;
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
        Printer::lastDirectionSovereignty = DIR_DIRECT;
        direct.fixStartAndEndSpeed();

        directError = direct.delta[direct.primaryAxis];
        direct.updateStepsParameter(); // should never be needed, but with bad timings???
        Printer::vMaxReached[FOR_DIRECT] = direct.vStart;
        Printer::stepNumber[FOR_DIRECT] = 0;
        Printer::timer[FOR_DIRECT] = 0;
        Printer::v = direct.fullSpeed;

#if USE_ADVANCE
        direct.updateAdvanceSteps(direct.vStart); //startet advance extruder "etwas" vor der ersten bewegung: gut? ist Printer::interval immer klein genug?
#endif                                            // USE_ADVANCE

        return (Printer::interval >> 1); //wait 50% to next interrupt.
    }

    if (Printer::lastDirectionSovereignty != DIR_DIRECT) {
        Printer::lastDirectionSovereignty = DIR_DIRECT;
        Printer::v = direct.fullSpeed;
        direct.adjustDirections();
#if USE_ADVANCE
        direct.updateAdvanceSteps(Printer::vMaxReached[FOR_DIRECT]);
#endif // USE_ADVANCE
    }

    return performMove(&direct, FOR_DIRECT);
} // performDirectMove

long PrintLine::performMove(PrintLine* move, uint8_t forQueue) {
    HAL::allowInterrupts();
    fast8_t max_loops = Printer::stepsPerTimerCall;
    if (move->stepsRemaining < max_loops)
        max_loops = move->stepsRemaining;
    HAL::forbidInterrupts();

    for (fast8_t loop = 0; loop < max_loops; loop++) {

#if STEPPER_HIGH_DELAY > 0
        if (loop)
            HAL::delayMicroseconds(STEPPER_HIGH_DELAY);
#endif // STEPPER_HIGH_DELAY > 0

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

            // Active pressure is to high to extrude
            if (g_nEmergencyESkip || !(Printer::outOfPrintVolume[X_AXIS] == 0 && Printer::outOfPrintVolume[Y_AXIS] == 0 && Printer::outOfPrintVolumeZ == 0)) {
                doESteps = false;
            }
            HAL::forbidInterrupts();

            if (doESteps) {
                int8_t dir = (move->isEPositiveMove() ? 1 : -1);
#if USE_ADVANCE
                if (Printer::isAdvanceActivated())
                    Printer::extruderStepsNeeded += dir;
                else
#endif // USE_ADVANCE
                {
                    Extruder::step();
                }
                if (!forQueue)
                    g_nContinueSteps[E_AXIS] -= dir;

#if FEATURE_HEAT_BED_Z_COMPENSATION
                // This Block adds % parts of steps to extrusion because of higher layer heights caused by ZCMP
                // We calculated compensatedPositionOverPercE as a result of streched layer heights. Here we add material to the layer to compensate stretching.
                if (Printer::compensatedPositionOverPercE != 0) {
                    Printer::compensatedPositionCollectTinyE += Printer::compensatedPositionOverPercE;
                    if (Printer::compensatedPositionCollectTinyE >= 1) {
#if USE_ADVANCE
                        if (Printer::isAdvanceActivated()) {
                            Printer::extruderStepsNeeded += dir;
                            if (!forQueue)
                                g_nContinueSteps[E_AXIS] -= dir;
                            while (Printer::compensatedPositionCollectTinyE >= 1) {
                                Printer::compensatedPositionCollectTinyE--; //notfalls bei überkompensation - sollte nicht vorkommen.
                            }
                        } else
#endif // USE_ADVANCE
                        {
                            Extruder::unstep();
                            // this is a steps spacer, not a double code from some lines above.
                            if (!forQueue)
                                g_nContinueSteps[E_AXIS] -= dir;
                            while (Printer::compensatedPositionCollectTinyE >= 1) {
                                Printer::compensatedPositionCollectTinyE--; //notfalls bei überkompensation - sollte nicht vorkommen.
                            }
                            Extruder::step();
                        }
                    }
                }
#endif // FEATURE_HEAT_BED_Z_COMPENSATION
            }
        }

        if (move->isXMove()) {
            if ((move->error[X_AXIS] -= move->delta[X_AXIS]) < 0) {
                HAL::allowInterrupts();
                int8_t dir = (Printer::getXDirectionIsPos() ? 1 : -1);
                if (inBauraum(X_AXIS, move, forQueue)) {
                    Printer::startXStep(dir);
                }
                HAL::forbidInterrupts();

                if (forQueue) {
                    move->error[X_AXIS] += queueError;
                    Printer::currentSteps[X_AXIS] += dir;
                } else {
                    move->error[X_AXIS] += directError;
                    g_nContinueSteps[X_AXIS] -= dir;
                    Printer::directCurrentSteps[X_AXIS] += dir;
                }
            }
        }

        if (move->isYMove()) {
            if ((move->error[Y_AXIS] -= move->delta[Y_AXIS]) < 0) {
                HAL::allowInterrupts();
                int8_t dir = (Printer::getYDirectionIsPos() ? 1 : -1);
                if (inBauraum(Y_AXIS, move, forQueue)) {
                    Printer::startYStep(dir);
                }
                HAL::forbidInterrupts();

                if (forQueue) {
                    move->error[Y_AXIS] += queueError;
                    Printer::currentSteps[Y_AXIS] += dir;
                } else {
                    move->error[Y_AXIS] += directError;
                    g_nContinueSteps[Y_AXIS] -= dir;
                    Printer::directCurrentSteps[Y_AXIS] += dir;
                }
            }
        }

        if (move->isZMove()) {
            if ((move->error[Z_AXIS] -= move->delta[Z_AXIS]) < 0) {
                int8_t dir = (Printer::getZDirectionIsPos() ? 1 : -1);
                //070118better zCMP strategy: if zCMP is against Queue/Direct, dont do some step and count it done @Queue/Direct and count it done @zCMP.

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
                bool cmpup = (Printer::compensatedPositionCurrentStepsZ < Printer::compensatedPositionTargetStepsZ);
                bool cmpdown = (Printer::compensatedPositionCurrentStepsZ > Printer::compensatedPositionTargetStepsZ);
                if (dir < 0 && cmpup) {
                    Printer::compensatedPositionCurrentStepsZ++;
                } else if (dir > 0 && cmpdown) {
                    Printer::compensatedPositionCurrentStepsZ--;
                } else {
                    HAL::allowInterrupts();
                    //no conflicting zCMP: Do the Z-Step.
                    if (inBauraumZ(move, forQueue)) {
                        Printer::startZStep(dir);
                    }
                    HAL::forbidInterrupts();
                }
#else
                Printer::startZStep();
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

                if (forQueue) {
                    move->error[Z_AXIS] += queueError;
                    Printer::currentSteps[Z_AXIS] += dir;
                } else {
                    move->error[Z_AXIS] += directError;
                    g_nContinueSteps[Z_AXIS] -= dir;
                    Printer::directCurrentSteps[Z_AXIS] += dir;
                }
                // Note: There is no need to check whether we are past the z-min endstop here because G-Codes typically are not able to go past the z-min endstop anyways.
            }
        }
#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
        else if (move->isXOrYMove()) {
            PrintLine::stepSlowedZCompensation(); // Z is free here. This is no Z-Move
        }
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

        move->stepsRemaining--;

#if STEPPER_HIGH_DELAY > 0
        HAL::delayMicroseconds(STEPPER_HIGH_DELAY);
#endif // #if STEPPER_HIGH_DELAY>0

#if USE_ADVANCE
        if (!Printer::isAdvanceActivated()) // Use interrupt for movement
#endif                                      // USE_ADVANCE
            Extruder::unstep();

        Printer::endXYZSteps();
    }                       // for loop
    HAL::allowInterrupts(); // Allow interrupts for other types, timer1 is still disabled

    /***
    Printer::interval -> printers interval
    interval          -> actual sent interval that might be manipulated
    v                 -> not manipulated active speed
    ***/
    //v [Steps/s] -> interval in [ [OP/s] / [Steps/s] ] = OPs/Steps = Time inbetween two steps.
    speed_t v;

    Printer::stepNumber[forQueue] += max_loops;
    Printer::timer[forQueue] += (Printer::interval * max_loops);

    //If acceleration is enabled on this move and we are in the acceleration segment, calculate the current interval
    if (move->moveAccelerating(forQueue)) // we are accelerating
    {
        v = HAL::ComputeV(Printer::timer[forQueue], move->fAcceleration);
        v += move->vStart;
        if (v > move->vMax)
            v = move->vMax;
        Printer::vMaxReached[forQueue] = v;
    } else if (move->moveDecelerating(forQueue)) // time to slow down
    {
        // Printer::timer got reset the first time reaching here.
        //dieses v ist hier erst gegenbeschleunigend und wird dann gleich abgezogen -> das ist die korrektur.
        unsigned int v_inv = HAL::ComputeV(Printer::timer[forQueue], move->fAcceleration); //hier negativgeschleunigung positiv ausgerechnet.
        if (v_inv > Printer::vMaxReached[forQueue])                                        // if deceleration goes too far it can become too large -> schneller vorab-limiter, eigentlich Printer::vMaxReached - move-vEnd, aber wie programmiert scheints egal und schneller zu sein.
            v = move->vEnd;
        else {
            v = Printer::vMaxReached[forQueue] - v_inv; //flip positive calculation to decelerated speed.
            if (v < move->vEnd)
                v = move->vEnd; // extra steps at the end of desceleration due to rounding errors
        }
    } else // full speed reached
    {
        // If we had acceleration, we need to use the latest vMaxReached and interval
        // If we started full speed, we need to use move->fullInterval and vMax
        v = (!move->accelSteps ? move->vMax : Printer::vMaxReached[forQueue]);
    }
#if USE_ADVANCE
    move->updateAdvanceSteps(v);
#endif // USE_ADVANCE

    Printer::interval = HAL::CPUDivU2(v);

    // Dieses Limit bedeutet max. 31250 steps/s bei 16mhz.
    if (Printer::interval < 512)
        Printer::interval = 512;

    //RETURN manipulated value:
    unsigned long interval = Printer::interval;

#if FEATURE_DIGIT_FLOW_COMPENSATION
    if (Printer::interval_mod) {
        //if we have to add some time because we need live adjusting then add it. But we should not calculate with * and / here so we have to prepare some delay. The preparation might be inprecise. So check it.
        if (forQueue && move->primaryAxis <= Y_AXIS) {
            interval *= Printer::interval_mod; //1024 == 100%
            interval >>= 10;                   // geteilt durch 1024 = 2^10
        }
    }
#endif // FEATURE_DIGIT_FLOW_COMPENSATION

    //Now manipulate step-width=interval according to single-, double-, tripple-, ... steppings needs:
    //we know that next time will be a (as example) double or quad step so we let pass twice the time and set the stepsPerTimerCall accordingly.
    Printer::stepsPerTimerCall = 1;
    unsigned long one_interval = interval;
    uint16_t minInterval = Printer::stepsPackingMinInterval;
    if (Printer::isAdvanceActivated()) {
        minInterval += 64;
    }
    while (interval < Printer::stepsPackingMinInterval) {
        interval += one_interval;
        Printer::stepsPerTimerCall += 1;
    }

    if (move->stepsRemaining <= 0 || move->isNoMove()) // line finished
    {
        //Wenn keine Steps mehr da, sollten alle Achsen die benutzt wurden wieder freigegeben werden. Bei Z ist der Sonderfall, dass die Z-Kompensation sich reinschummeln könnte.
        move->setXYZEMoveFinished(); // Wichtig, das die Z-Kompensation wieder weiterarbeiten kann, auch wichtig bei PrintLine::direct!
                                     // Auch wenn kein Z-Move, könnte die Z-Kompensation die Achse Z benutzt haben.

        if (forQueue) {
            removeCurrentLineForbidInterrupt();
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
#if FEATURE_MILLING_MODE
        if (Printer::operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE
#if FEATURE_HEAT_BED_Z_COMPENSATION
            if (g_nHeatBedScanStatus || g_nZOSScanStatus)
                nIdle = 0; // we are not idle because the heat bed scan is going on at the moment
#endif                     // FEATURE_HEAT_BED_Z_COMPENSATION
#if FEATURE_MILLING_MODE
        } else {
#if FEATURE_WORK_PART_Z_COMPENSATION
            if (g_nWorkPartScanStatus)
                nIdle = 0; // we are not idle because the work part scan is going on at the moment
#endif                     // FEATURE_WORK_PART_Z_COMPENSATION
        }
#endif // FEATURE_MILLING_MODE

        if (linesCount == 0 && nIdle) {
            g_uStartOfIdle = HAL::timeInMilliseconds(); //end a move
            Printer::v = 0;
        }

        interval >>= 1; // 50% of time to next call to do cur=0, which will fire another interrupt at 50% of the active Printer::interval step time. This works like "move 1 end -> 50% -> prepare move 2 -> 50% -> move 2 begin"
        DEBUG_MEMORY;
    } // line finished

    return interval;
} // performMove

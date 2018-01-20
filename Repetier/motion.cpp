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
#ifndef STEP_DOUBLER_FREQUENCY
#error Please add new parameter STEP_DOUBLER_FREQUENCY to your configuration.
#else
#if STEP_DOUBLER_FREQUENCY<5000 || STEP_DOUBLER_FREQUENCY>12000
#error STEP_DOUBLER_FREQUENCY should be in range 5000-12000.
#endif // STEP_DOUBLER_FREQUENCY<5000 || STEP_DOUBLER_FREQUENCY>12000
#endif // STEP_DOUBLER_FREQUENCY

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
#if X_STEP_PIN<0 || Y_STEP_PIN<0 || Z_STEP_PIN<0
#error One of the following pins is not assigned: X_STEP_PIN,Y_STEP_PIN,Z_STEP_PIN
#endif // X_STEP_PIN<0 || Y_STEP_PIN<0 || Z_STEP_PIN<0

#if EXT0_STEP_PIN<0 && NUM_EXTRUDER>0
#error EXT0_STEP_PIN not set to a pin number.
#endif // EXT0_STEP_PIN<0 && NUM_EXTRUDER>0

#if EXT0_DIR_PIN<0 && NUM_EXTRUDER>0
#error EXT0_DIR_PIN not set to a pin number.
#endif // EXT0_DIR_PIN<0 && NUM_EXTRUDER>0

#if MOVE_CACHE_SIZE<5
#error MOVE_CACHE_SIZE must be at least 5
#endif // MOVE_CACHE_SIZE<5

// Inactivity shutdown variables
millis_t            previousMillisCmd   = 0;
millis_t            maxInactiveTime     = MAX_INACTIVE_TIME*1000L;
millis_t            stepperInactiveTime = STEPPER_INACTIVE_TIME*1000L;
long                baudrate            = BAUDRATE;                     // Communication speed rate.

#if USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
int                 maxadv              = 0;
#endif // ENABLE_QUADRATIC_ADVANCE
int                 maxadv2             = 0;
float               maxadvspeed         = 0;
#endif // USE_ADVANCE

uint8_t             pwm_pos[NUM_EXTRUDER+3];                // 0-NUM_EXTRUDER = Heater 0-NUM_EXTRUDER of extruder, NUM_EXTRUDER = Heated bed, NUM_EXTRUDER+1 Board fan, NUM_EXTRUDER+2 = Fan
volatile int        waitRelax           = 0;                // Delay filament relax at the end of print, could be a simple timeout

PrintLine PrintLine::lines[MOVE_CACHE_SIZE];            // Cache for print moves.
PrintLine *PrintLine::cur = 0;                          // Current printing line

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
PrintLine           PrintLine::direct;                          // direct movement
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

uint8_t             PrintLine::linesWritePos    = 0;    // Position where we write the next cached line move.
volatile uint8_t    PrintLine::linesCount       = 0;    // Number of lines cached 0 = nothing to do.
uint8_t             PrintLine::linesPos         = 0;    // Position for executing line movement.

/** \brief Move printer the given number of steps. Puts the move into the queue. Used by e.g. homing commands. */
void PrintLine::moveRelativeDistanceInSteps(long x,long y,long z,long e,float feedrate,bool waitEnd,bool checkEndstop)
{
    Printer::queuePositionTargetSteps[X_AXIS] = Printer::queuePositionLastSteps[X_AXIS] + x;
    Printer::queuePositionTargetSteps[Y_AXIS] = Printer::queuePositionLastSteps[Y_AXIS] + y;
    Printer::queuePositionTargetSteps[Z_AXIS] = Printer::queuePositionLastSteps[Z_AXIS] + z;
    Printer::queuePositionTargetSteps[E_AXIS] = Printer::queuePositionLastSteps[E_AXIS] + e;

    prepareQueueMove(checkEndstop, false, feedrate);

    Printer::updateCurrentPosition();
    if(waitEnd)
        Commands::waitUntilEndOfAllMoves();

    previousMillisCmd = HAL::timeInMilliseconds();
} // moveRelativeDistanceInSteps


void PrintLine::moveRelativeDistanceInStepsReal(long x,long y,long z,long e,float feedrate,bool waitEnd)
{
    float   newPosition[3];


    newPosition[X_AXIS] = Printer::queuePositionCommandMM[X_AXIS] + x * Printer::invAxisStepsPerMM[X_AXIS];
    newPosition[Y_AXIS] = Printer::queuePositionCommandMM[Y_AXIS] + y * Printer::invAxisStepsPerMM[Y_AXIS];
    newPosition[Z_AXIS] = Printer::queuePositionCommandMM[Z_AXIS] + z * Printer::invAxisStepsPerMM[Z_AXIS];
    if(!Printer::isPositionAllowed( newPosition[X_AXIS], newPosition[Y_AXIS], newPosition[Z_AXIS]))
    {
        return; // ignore this move
    }

    Printer::queuePositionCommandMM[X_AXIS] = newPosition[X_AXIS];
    Printer::queuePositionCommandMM[Y_AXIS] = newPosition[Y_AXIS];
    Printer::queuePositionCommandMM[Z_AXIS] = newPosition[Z_AXIS];

    Printer::moveToReal(Printer::queuePositionCommandMM[X_AXIS],Printer::queuePositionCommandMM[Y_AXIS],Printer::queuePositionCommandMM[Z_AXIS],
                        (Printer::queuePositionLastSteps[E_AXIS] + e) * Printer::invAxisStepsPerMM[E_AXIS],feedrate);
    Printer::updateCurrentPosition();
    if(waitEnd)
        Commands::waitUntilEndOfAllMoves();

    previousMillisCmd = HAL::timeInMilliseconds();

} // moveRelativeDistanceInStepsReal


/** \brief Put a move to the current destination coordinates into the movement cache.
  If the cache is full, the method will wait, until a place gets free. During
  wait communication and temperature control is enabled.
  @param check_endstops Read endstop during move. */
void PrintLine::prepareQueueMove(uint8_t check_endstops,uint8_t pathOptimize, float feedrate)
{
    Printer::unmarkAllSteppersDisabled(); // ??? hier wird nichts enabled. Nur markiert, auch wenn später oder früher "enablestepper" passiert.
    //evtl. weil dadurch in jedem fall gleich ein stepper aktiviert werden würde -> darum hier schon als aktiv markieren, weil umumgänglich ist. Aber dann müsste man das (timingsicher) auch schon in den Funktionen über prepareDirectMove erledigt haben.

    PrintLine::waitForXFreeLines(1);

    uint8_t newPath = PrintLine::insertWaitMovesIfNeeded(pathOptimize, 0);
    PrintLine *p = PrintLine::getNextWriteLine();

    p->task = 0;

    float axisDistanceMM[4]; // Axis movement in mm
    p->flags = (check_endstops ? FLAG_CHECK_ENDSTOPS : 0);
    p->joinFlags = 0;
    if(!pathOptimize) p->setEndSpeedFixed(true);
    p->dir = 0;

    Printer::constrainQueueDestinationCoords(); //not in newest repetier!

    // Find direction
    for(uint8_t axis=0; axis < 4; axis++)
    {
        p->delta[axis] = Printer::queuePositionTargetSteps[axis] - Printer::queuePositionLastSteps[axis];
        
#if FEATURE_HEAT_BED_Z_COMPENSATION
        //find highest Z layer with extrusion beneath g_maxZCompensationSteps for calculating the bordercrossing infill additions
        if(axis == Z_AXIS){
            if(p->delta[axis] != 0){ //z achsen aufstieg/abstieg -> wir müssen durch erkennung von extrusion mögliche zlifts filtern!
                InterruptProtectedBlock noInts;
                Printer::queuePositionZLayerCurrent_cand = Printer::queuePositionTargetSteps[Z_AXIS] + Extruder::current->zOffset;
    #if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
                Printer::queuePositionZLayerCurrent_cand += Printer::directPositionCurrentSteps[Z_AXIS];
    #endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
                noInts.unprotect();
                if(Printer::queuePositionZLayerCurrent_cand == Printer::queuePositionZLayerCurrent) Printer::queuePositionZLayerCurrent_cand = 0; //nach zlift nicht neu bestimmen.
            }
        }
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

        if(axis == E_AXIS)
        {
            Printer::extrudeMultiplyError += (static_cast<float>(p->delta[E_AXIS]) * Printer::extrusionFactor
 #if FEATURE_DIGIT_FLOW_COMPENSATION
                            * g_nDigitFlowCompensation_flowmulti
 #endif // FEATURE_DIGIT_FLOW_COMPENSATION
               );
            p->delta[E_AXIS] = static_cast<int32_t>(Printer::extrudeMultiplyError);
            Printer::extrudeMultiplyError -= p->delta[E_AXIS];
            Printer::filamentPrinted += p->delta[E_AXIS] * Printer::invAxisStepsPerMM[axis];
#if FEATURE_HEAT_BED_Z_COMPENSATION
            //prüfe ob in der letzten angefahrenen Z layerhöhe extrudiert wird: dann kein zlift!
            if(Printer::queuePositionZLayerCurrent_cand){
                if(p->delta[axis] > 0){
                    InterruptProtectedBlock noInts;
                    long nCurrentPositionStepsZ = Printer::queuePositionTargetSteps[Z_AXIS] + Extruder::current->zOffset;
#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
                    nCurrentPositionStepsZ += Printer::directPositionCurrentSteps[Z_AXIS];
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
                    noInts.unprotect();
                    if(Printer::queuePositionZLayerCurrent_cand == nCurrentPositionStepsZ){
                        Printer::queuePositionZLayerLast = Printer::queuePositionZLayerCurrent;
                        Printer::queuePositionZLayerCurrent = Printer::queuePositionZLayerCurrent_cand;
                        if(Printer::queuePositionZLayerLast > Printer::queuePositionZLayerCurrent){
                            if(Printer::queuePositionZLayerCurrent < Printer::axisStepsPerMM[Z_AXIS]){ //1mm
                                Printer::queuePositionZLayerLast = 0;
                            }
                        }
#if AUTOADJUST_MIN_MAX_ZCOMP
                        //Problemfall Startmade: Wenn die Startmade aus etwas weniger als 16 od. MOVE_CACHE_SIZE Teilstücken besteht, was man annehmen kann, dann springt der Pfadplaner über die Startmade einfach drüber und nimmt den ersten Layer als neue Referenz. Das hier ist Preprocessing mit Blick in die Zukunft. Kurz steht in g_minZCompensationSteps z.B. 0.35, anschließend aber z.B. korrekte 0.2mm als g_minZCompensationSteps. 
                        //würde das nicht klappen, hätten wir SenseOffset in der Startmade was maximal schlecht sein kann, weil evtl. die Digits zu hoch sind. -> Sollte mit dieser Automatik hier nicht vorkommen!
                        if(g_auto_minmaxZCompensationSteps && !Printer::queuePositionZLayerLast && Printer::queuePositionZLayerCurrent < Printer::axisStepsPerMM[Z_AXIS]){ //1mm
                            //hiermit hätten wir immer exakt 1 Lage, die der Drucker komplett mit dem Bettprofil abfährt, anschließend ab Layer 2 wird ausgeschlichen +ECMP.
                            if(abs(Printer::queuePositionZLayerCurrent - Printer::axisStepsPerMM[Z_AXIS]*AUTOADJUST_STARTMADEN_AUSSCHLUSS) > 5 /* 2um um startmadenhöhe herum nichts tun */){
                                g_minZCompensationSteps = Printer::queuePositionZLayerCurrent;
                                g_maxZCompensationSteps = g_minZCompensationSteps + (g_offsetZCompensationSteps - g_ZCompensationMax) * 20; //max zulässige kompensation pro lage: 1/20 = 5%
                            }
                        }
#endif //AUTOADJUST_MIN_MAX_ZCOMP
                    }
                    Printer::queuePositionZLayerCurrent_cand = 0;
                }
            }
#endif // FEATURE_HEAT_BED_Z_COMPENSATION
        }
        if(p->delta[axis] >= 0)
            p->setPositiveDirectionForAxis(axis);
        else
            p->delta[axis] = -p->delta[axis];
        axisDistanceMM[axis] = p->delta[axis] * Printer::invAxisStepsPerMM[axis];
        if(p->delta[axis]) p->setMoveOfAxis(axis);
        Printer::queuePositionLastSteps[axis] = Printer::queuePositionTargetSteps[axis];
    }
    if(p->isNoMove())
    {
        if(newPath)     // need to delete dummy elements, otherwise commands can get locked.
            PrintLine::resetPathPlanner();
        return;         // No steps included
    }
    float xydist2;

#if ENABLE_BACKLASH_COMPENSATION
    if((p->isXYZMove()) && ((p->dir & 7)^(Printer::backlashDir & 7)) & (Printer::backlashDir >> 3))   // We need to compensate backlash, add a move
    {
        waitForXFreeLines(2);
        uint8_t wpos2 = linesWritePos+1;
        if(wpos2>=MOVE_CACHE_SIZE) wpos2 = 0;
            PrintLine *p2 = &lines[wpos2];
        memcpy(p2,p,sizeof(PrintLine));     // Move current data to p2
        uint8_t changed = (p->dir & 7)^(Printer::backlashDir & 7);
        float back_diff[4];                 // Axis movement in mm
        back_diff[E_AXIS] = 0;
        back_diff[X_AXIS] = (changed & 1 ? (p->isXPositiveMove() ? Printer::backlash[X_AXIS] : -Printer::backlash[X_AXIS]) : 0);
        back_diff[Y_AXIS] = (changed & 2 ? (p->isYPositiveMove() ? Printer::backlash[Y_AXIS] : -Printer::backlash[Y_AXIS]) : 0);
        back_diff[Z_AXIS] = (changed & 4 ? (p->isZPositiveMove() ? Printer::backlash[Z_AXIS] : -Printer::backlash[Z_AXIS]) : 0);
        p->dir &=7;                         // x,y and z are already correct
        for(uint8_t i=0; i < 4; i++)
        {
            float f = back_diff[i]*Printer::axisStepsPerMM[i];
            p->delta[i] = abs((long)f);
            if(p->delta[i]) p->dir |= 16<<i;
        }
        // Define variables that are needed for the Bresenham algorithm. Please note that  Z is not currently included in the Bresenham algorithm.
        if(p->delta[Y_AXIS] > p->delta[X_AXIS] && p->delta[Y_AXIS] > p->delta[Z_AXIS]) 
            p->primaryAxis = Y_AXIS;
        else if (p->delta[X_AXIS] > p->delta[Z_AXIS] ) 
            p->primaryAxis = X_AXIS;
        else 
            p->primaryAxis = Z_AXIS;
        p->stepsRemaining = p->delta[p->primaryAxis];
        // Feedrate calc based on XYZ travel distance
        xydist2 = back_diff[X_AXIS] * back_diff[X_AXIS] + back_diff[Y_AXIS] * back_diff[Y_AXIS];
        if(p->isZMove())
            p->distance = sqrt(xydist2 + back_diff[Z_AXIS] * back_diff[Z_AXIS]);
        else
            p->distance = sqrt(xydist2);
        Printer::backlashDir = (Printer::backlashDir & 56) | (p2->dir & 7);
        p->calculateQueueMove(back_diff, pathOptimize, p->primaryAxis);
        p = p2;                             // use saved instance for the real move
    }
#endif // ENABLE_BACKLASH_COMPENSATION

    // Define variables that are needed for the Bresenham algorithm. Please note that Z is not currently included in the Bresenham algorithm.
    if(p->delta[Y_AXIS] > p->delta[X_AXIS] && p->delta[Y_AXIS] > p->delta[Z_AXIS] && p->delta[Y_AXIS] > p->delta[E_AXIS]) 
        p->primaryAxis = Y_AXIS;
    else if (p->delta[X_AXIS] > p->delta[Z_AXIS] && p->delta[X_AXIS] > p->delta[E_AXIS]) 
        p->primaryAxis = X_AXIS;
    else if (p->delta[Z_AXIS] > p->delta[E_AXIS]) 
        p->primaryAxis = Z_AXIS;
    else 
        p->primaryAxis = E_AXIS;
    p->stepsRemaining = p->delta[p->primaryAxis];
    if(p->isXYZMove())
    {
        xydist2 = axisDistanceMM[X_AXIS] * axisDistanceMM[X_AXIS] + axisDistanceMM[Y_AXIS] * axisDistanceMM[Y_AXIS];
        if(p->isZMove())
            p->distance = RMath::max((float)sqrt(xydist2 + axisDistanceMM[Z_AXIS] * axisDistanceMM[Z_AXIS]),fabs(axisDistanceMM[E_AXIS]));
        else
            p->distance = RMath::max((float)sqrt(xydist2),fabs(axisDistanceMM[E_AXIS]));
    }
    else
        p->distance = fabs(axisDistanceMM[E_AXIS]);
    p->calculateQueueMove(axisDistanceMM, pathOptimize, p->primaryAxis, feedrate);

} // prepareQueueMove


#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
void PrintLine::prepareDirectMove(void)
{
    Printer::unmarkAllSteppersDisabled(); // ??? hier wird nichts enabled. Nur markiert, auch wenn später oder früher "enablestepper" passiert.
    //evtl. weil dadurch in jedem fall gleich ein stepper aktiviert werden würde -> darum hier schon als aktiv markieren, weil umumgänglich ist. Aber dann müsste man das (timingsicher) auch schon in den Funktionen über prepareDirectMove erledigt haben.

    PrintLine *p = &PrintLine::direct;

    p->task = 0;

    float axisDistanceMM[4]; // Axis movement in mm
    p->flags = FLAG_CHECK_ENDSTOPS;
    p->joinFlags = 0;
    p->setEndSpeedFixed(true);
    p->dir = 0;

    Printer::constrainDirectDestinationCoords();

    // Find direction
    for(uint8_t axis=0; axis < 4; axis++)
    {
        p->delta[axis] = Printer::directPositionTargetSteps[axis] - Printer::directPositionCurrentSteps[axis];
        if(axis == E_AXIS)
        {
            Printer::extrudeMultiplyError += (static_cast<float>(p->delta[E_AXIS]) * Printer::extrusionFactor
 #if FEATURE_DIGIT_FLOW_COMPENSATION
                            * g_nDigitFlowCompensation_flowmulti
 #endif // FEATURE_DIGIT_FLOW_COMPENSATION
               );
            p->delta[E_AXIS] = static_cast<int32_t>(Printer::extrudeMultiplyError);
            Printer::extrudeMultiplyError -= p->delta[E_AXIS];
            Printer::filamentPrinted += p->delta[E_AXIS] * Printer::invAxisStepsPerMM[axis];
        }
        if(p->delta[axis] >= 0)
            p->setPositiveDirectionForAxis(axis);
        else
            p->delta[axis] = -p->delta[axis];
        axisDistanceMM[axis] = p->delta[axis] * Printer::invAxisStepsPerMM[axis];
        if(p->delta[axis]) p->setMoveOfAxis(axis);
        Printer::directPositionLastSteps[axis] = Printer::directPositionTargetSteps[axis];
    }
    if(p->isNoMove())
    {
        p->stepsRemaining = 0;
        return;         // No steps included
    }
    float xydist2;

    // Define variables that are needed for the Bresenham algorithm. Please note that Z is not currently included in the Bresenham algorithm.
    if(p->delta[Y_AXIS] > p->delta[X_AXIS] && p->delta[Y_AXIS] > p->delta[Z_AXIS] && p->delta[Y_AXIS] > p->delta[E_AXIS]) 
        p->primaryAxis = Y_AXIS;
    else if (p->delta[X_AXIS] > p->delta[Z_AXIS] && p->delta[X_AXIS] > p->delta[E_AXIS]) 
        p->primaryAxis = X_AXIS;
    else if (p->delta[Z_AXIS] > p->delta[E_AXIS]) 
        p->primaryAxis = Z_AXIS;
    else 
        p->primaryAxis = E_AXIS;
    p->stepsRemaining = p->delta[p->primaryAxis];
    if(p->isXYZMove())
    {
        xydist2 = axisDistanceMM[X_AXIS] * axisDistanceMM[X_AXIS] + axisDistanceMM[Y_AXIS] * axisDistanceMM[Y_AXIS];
        if(p->isZMove())
            p->distance = RMath::max((float)sqrt(xydist2 + axisDistanceMM[Z_AXIS] * axisDistanceMM[Z_AXIS]),fabs(axisDistanceMM[E_AXIS]));
        else
            p->distance = RMath::max((float)sqrt(xydist2),fabs(axisDistanceMM[E_AXIS]));
    }
    else
        p->distance = fabs(axisDistanceMM[E_AXIS]);
    p->calculateDirectMove(axisDistanceMM, false, p->primaryAxis);

} // prepareDirectMove


void PrintLine::stopDirectMove( void ) //Funktion ist bereits zur ausführzeit von InterruptProtectedBlock eingeschlossen!
{
    if( PrintLine::direct.isXYZMove() )
    {
        // decelerate and stop
        if( PrintLine::direct.stepsRemaining > RF_MICRO_STEPS )
        {
            PrintLine::direct.stepsRemaining = RF_MICRO_STEPS;
        }
    }
    return;
} // stopDirectMove
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING


void PrintLine::calculateQueueMove(float axisDistanceMM[],uint8_t pathOptimize, fast8_t drivingAxis, float feedrate)
{
    long    axisInterval[4];
    float timeForMove = (float)(F_CPU) * distance / feedrate
#if FEATURE_DIGIT_FLOW_COMPENSATION
                                                                      / g_nDigitFlowCompensation_feedmulti
#endif // FEATURE_DIGIT_FLOW_COMPENSATION
    ; // time is in ticks
    
    if(linesCount < MOVE_CACHE_LOW && timeForMove < LOW_TICKS_PER_MOVE)   // Limit speed to keep cache full.
    {
        //OUT_P_I("L:",lines_count);
        timeForMove += (3 * (LOW_TICKS_PER_MOVE - timeForMove)) / (linesCount + 1); // Increase time if queue gets empty. Add more time if queue gets smaller.
        //OUT_P_F_LN("Slow ",time_for_move);
    }
    timeInTicks = timeForMove;
    UI_MEDIUM;                      // do check encoder
    // Compute the solwest allowed interval (ticks/step), so maximum feedrate is not violated
    int32_t limitInterval0;
    int32_t limitInterval = limitInterval0 = timeForMove/stepsRemaining; // until not violated by other constraints it is your target speed
    float toTicks = static_cast<float>(F_CPU) / stepsRemaining;
    if(isXMove())
    {
        axisInterval[X_AXIS] = axisDistanceMM[X_AXIS] * toTicks / (Printer::maxFeedrate[X_AXIS]); // mm*ticks/s/(mm/s*steps) = ticks/step
        limitInterval = RMath::max(axisInterval[X_AXIS], limitInterval);
    } else axisInterval[X_AXIS] = 0;

    if(isYMove())
    {
        axisInterval[Y_AXIS] = axisDistanceMM[Y_AXIS] * toTicks / Printer::maxFeedrate[Y_AXIS];
        limitInterval = RMath::max(axisInterval[Y_AXIS], limitInterval);
    } else axisInterval[Y_AXIS] = 0;


    if(isZMove())                   // normally no move in z direction
    {
        axisInterval[Z_AXIS] = axisDistanceMM[Z_AXIS] * toTicks / Printer::maxFeedrate[Z_AXIS]; // must prevent overflow!
        limitInterval = RMath::max(axisInterval[Z_AXIS], limitInterval);
    } else axisInterval[Z_AXIS] = 0;

    if(isEMove())
    {
        axisInterval[E_AXIS] = axisDistanceMM[E_AXIS] * toTicks / Printer::maxFeedrate[E_AXIS];
        limitInterval = RMath::max(axisInterval[E_AXIS], limitInterval);
    } else axisInterval[E_AXIS] = 0;

    fullInterval = limitInterval = (limitInterval > LIMIT_INTERVAL ? limitInterval : LIMIT_INTERVAL);   // This is our target speed
    if(limitInterval != limitInterval0) {
        // new time at full speed = limitInterval*p->stepsRemaining [ticks]
        timeForMove = (float)limitInterval * (float)stepsRemaining;                                     // for large z-distance this overflows with long computation
    }
    float inverseTimeS = static_cast<float>(F_CPU) / timeForMove;
    if(isXMove())
    {
        axisInterval[X_AXIS] = timeForMove / delta[X_AXIS];
        speedX = axisDistanceMM[X_AXIS] * inverseTimeS;
        if(isXNegativeMove()) speedX = -speedX;
    } else speedX = 0;
    if(isYMove())
    {
        axisInterval[Y_AXIS] = timeForMove / delta[Y_AXIS];
        speedY = axisDistanceMM[Y_AXIS] * inverseTimeS;
        if(isYNegativeMove()) speedY = -speedY;
    } else speedY = 0;
    if(isZMove())
    {
        axisInterval[Z_AXIS] = timeForMove / delta[Z_AXIS];
        speedZ = axisDistanceMM[Z_AXIS] * inverseTimeS;
        if(isZNegativeMove()) speedZ = -speedZ;
    } else speedZ = 0;
    if(isEMove())
    {
        axisInterval[E_AXIS] = timeForMove / delta[E_AXIS];
        speedE = axisDistanceMM[E_AXIS] * inverseTimeS;
        if(isENegativeMove()) speedE = -speedE;
    }
    fullSpeed = distance * inverseTimeS;
    // long interval = axis_interval[primary_axis]; // time for every step in ticks with full speed
    // If acceleration is enabled, do some Bresenham calculations depending on which axis will lead it.

#ifdef RAMP_ACCELERATION
    // slowest time to accelerate from v0 to limitInterval determines used acceleration
    // t = (v_end-v_start)/a
    float slowestAxisPlateauTimeRepro = 1e15; // repro to reduce division Unit: 1/s
    uint32_t* accel = (isEPositiveMove() ?  Printer::maxPrintAccelerationStepsPerSquareSecond : Printer::maxTravelAccelerationStepsPerSquareSecond);
    
    for(uint8_t i=0; i < 4 ; i++)
    {
        if(isMoveOfAxis(i))
        {
            // v = a * t => t = v/a = F_CPU/(c*a) => 1/t = c*a/F_CPU
            slowestAxisPlateauTimeRepro = RMath::min(slowestAxisPlateauTimeRepro, (float)axisInterval[i] * (float)accel[i]);     // steps/s^2 * step/tick  Ticks/s^2
        }
    }

    // Errors for delta move are initialized in timer (except extruder)
    error[X_AXIS] = error[Y_AXIS] = error[Z_AXIS] = delta[primaryAxis] >> 1;

    invFullSpeed = 1.0/fullSpeed;
    accelerationPrim = slowestAxisPlateauTimeRepro / axisInterval[primaryAxis];                 // a = v/t = F_CPU/(c*t): Steps/s^2

    // Now we can calculate the new primary axis acceleration, so that the slowest axis max acceleration is not violated
    fAcceleration = 262144.0*(float)accelerationPrim / F_CPU; // will overflow without float!
    accelerationDistance2 = 2.0 * distance * slowestAxisPlateauTimeRepro * fullSpeed / ((float)F_CPU);  // mm^2/s^2
    startSpeed = endSpeed = minSpeed = safeSpeed(drivingAxis);
    if(startSpeed > feedrate
#if FEATURE_DIGIT_FLOW_COMPENSATION
                                      * g_nDigitFlowCompensation_feedmulti
#endif // FEATURE_DIGIT_FLOW_COMPENSATION
    ){
        startSpeed = endSpeed = minSpeed = feedrate
#if FEATURE_DIGIT_FLOW_COMPENSATION
                                      * g_nDigitFlowCompensation_feedmulti
#endif // FEATURE_DIGIT_FLOW_COMPENSATION
        ;
    }
    // Can accelerate to full speed within the line
    if (startSpeed * startSpeed + accelerationDistance2 >= fullSpeed * fullSpeed)
        setNominalMove();

    vMax = F_CPU / fullInterval;    // maximum steps per second, we can reach
    // if(p->vMax>46000)            // gets overflow in N computation
    // p->vMax = 46000;
    // p->plateauN = (p->vMax*p->vMax/p->accelerationPrim)>>1;

#if USE_ADVANCE
    if(!isXYZMove() || !isEPositiveMove()) {   // No head move or E move only or sucking filament back
#ifdef ENABLE_QUADRATIC_ADVANCE
        advanceRate = 0;
        advanceFull = 0;
#endif // ENABLE_QUADRATIC_ADVANCE
        advanceL = 0;
    }
    else
    {
        float advlin = fabs(speedE) * Extruder::current->advanceL * 0.001 * Printer::axisStepsPerMM[E_AXIS];
        advanceL = (uint16_t)((65536L * advlin) / vMax); //advanceLscaled = (65536*vE*k2)/vMax
#if ENABLE_QUADRATIC_ADVANCE
        advanceFull = 65536 * Extruder::current->advanceK * speedE * speedE; // Steps*65536 at full speed
        long steps = (HAL::U16SquaredToU32(vMax)) / (accelerationPrim << 1); // v^2/(2*a) = steps needed to accelerate from 0-vMax
        advanceRate = advanceFull / steps;
        if((advanceFull >> 16) > maxadv) {
            maxadv = (advanceFull >> 16);
            maxadvspeed = fabs(speedE);
        }
#endif // ENABLE_QUADRATIC_ADVANCE
        if(advlin > maxadv2) {
            maxadv2 = advlin;
            maxadvspeed = fabs(speedE);
        }
    }
#endif // USE_ADVANCE

    UI_MEDIUM;          // do check encoder
    updateTrapezoids();
    // how much steps on primary axis do we need to reach target feedrate
    // p->plateauSteps = (long) (((float)p->acceleration *0.5f / slowestAxisPlateauTimeRepro + p->vMin) *1.01f/slowestAxisPlateauTimeRepro);
#else
#if USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    advanceRate = 0;    // No advance for constant speeds
    advanceFull = 0;
#endif // ENABLE_QUADRATIC_ADVANCE
#endif // USE_ADVANCE
#endif // RAMP_ACCELERATION

    // Make result permanent
    if (pathOptimize) waitRelax = 70;
    pushLine();

    DEBUG_MEMORY;

} // calculateQueueMove


#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
void PrintLine::calculateDirectMove(float axisDistanceMM[],uint8_t pathOptimize, fast8_t drivingAxis)
{
    long    axisInterval[4];
    float   timeForMove = (float)(F_CPU)*distance / (isXOrYMove() ? DIRECT_FEEDRATE_XY : isZMove() ? DIRECT_FEEDRATE_Z : DIRECT_FEEDRATE_E);    // time is in ticks

    timeInTicks = timeForMove;
    UI_MEDIUM;                      // do check encoder

    // Compute the solwest allowed interval (ticks/step), so maximum feedrate is not violated
    long limitInterval = timeForMove/stepsRemaining; // until not violated by other constraints it is your target speed

    if(isXMove())
    {
        axisInterval[X_AXIS] = fabs(axisDistanceMM[X_AXIS]) * F_CPU / (DIRECT_FEEDRATE_XY * stepsRemaining); // mm*ticks/s/(mm/s*steps) = ticks/step
        limitInterval = RMath::max(axisInterval[X_AXIS],limitInterval);
    }
    else
    {
        axisInterval[X_AXIS] = 0;
    }

    if(isYMove())
    {
        axisInterval[Y_AXIS] = fabs(axisDistanceMM[Y_AXIS])*F_CPU / (DIRECT_FEEDRATE_XY * stepsRemaining);
        limitInterval = RMath::max(axisInterval[Y_AXIS],limitInterval);
    }
    else
    {
        axisInterval[Y_AXIS] = 0;
    }

    if(isZMove())                   // normally no move in z direction
    {
        axisInterval[Z_AXIS] = fabs((float)axisDistanceMM[Z_AXIS])*(float)F_CPU / (float)(DIRECT_FEEDRATE_Z*stepsRemaining); // must prevent overflow!
        limitInterval = RMath::max(axisInterval[Z_AXIS],limitInterval);
    }
    else
    {
        axisInterval[Z_AXIS] = 0;
    }

    if(isEMove())
    {
        axisInterval[E_AXIS] = fabs(axisDistanceMM[E_AXIS])*F_CPU / (DIRECT_FEEDRATE_E * stepsRemaining);
        limitInterval = RMath::max(axisInterval[E_AXIS],limitInterval);
    }
    else
    {
        axisInterval[E_AXIS] = 0;
    }

    fullInterval = limitInterval = (limitInterval > LIMIT_INTERVAL ? limitInterval : LIMIT_INTERVAL);   // This is our target speed

    // new time at full speed = limitInterval*p->stepsRemaining [ticks]
    timeForMove = (float)limitInterval * (float)stepsRemaining;                                     // for large z-distance this overflows with long computation
    float inv_time_s = (float)F_CPU / timeForMove;

    if(isXMove())
    {
        axisInterval[X_AXIS] = timeForMove / delta[X_AXIS];
        speedX = axisDistanceMM[X_AXIS] * inv_time_s;
        if(isXNegativeMove()) speedX = -speedX;
    }
    else
    {
        speedX = 0;
    }

    if(isYMove())
    {
        axisInterval[Y_AXIS] = timeForMove/delta[Y_AXIS];
        speedY = axisDistanceMM[Y_AXIS] * inv_time_s;
        if(isYNegativeMove()) speedY = -speedY;
    }
    else
    {
        speedY = 0;
    }

    if(isZMove())
    {
        axisInterval[Z_AXIS] = timeForMove/delta[Z_AXIS];
        speedZ = axisDistanceMM[Z_AXIS] * inv_time_s;
        if(isZNegativeMove()) speedZ = -speedZ;
    }
    else
    {
        speedZ = 0;
    }

    if(isEMove())
    {
        axisInterval[E_AXIS] = timeForMove/delta[E_AXIS];
        speedE = axisDistanceMM[E_AXIS] * inv_time_s;
        if(isENegativeMove()) speedE = -speedE;
    }
    fullSpeed = distance * inv_time_s;
    // long interval = axis_interval[primary_axis]; // time for every step in ticks with full speed
    // If acceleration is enabled, do some Bresenham calculations depending on which axis will lead it.

#ifdef RAMP_ACCELERATION
    // slowest time to accelerate from v0 to limitInterval determines used acceleration
    // t = (v_end-v_start)/a
    float           slowestAxisPlateauTimeRepro = 1e15; // repro to reduce division Unit: 1/s
    unsigned long*  accel                           = (isEPositiveMove() ?  Printer::maxPrintAccelerationStepsPerSquareSecond : Printer::maxTravelAccelerationStepsPerSquareSecond);

    for(uint8_t i=0; i < 4 ; i++)
    {
        if(isMoveOfAxis(i))
        {
            // v = a * t => t = v/a = F_CPU/(c*a) => 1/t = c*a/F_CPU
            slowestAxisPlateauTimeRepro = RMath::min(slowestAxisPlateauTimeRepro,(float)axisInterval[i] * (float)accel[i]);     // steps/s^2 * step/tick  Ticks/s^2
        }
    }

    // Errors for delta move are initialized in timer (except extruder)
    error[X_AXIS] = error[Y_AXIS] = error[Z_AXIS] = delta[primaryAxis] >> 1;

    invFullSpeed = 1.0/fullSpeed;
    accelerationPrim = slowestAxisPlateauTimeRepro / axisInterval[primaryAxis];                 // a = v/t = F_CPU/(c*t): Steps/s^2

    // Now we can calculate the new primary axis acceleration, so that the slowest axis max acceleration is not violated
    fAcceleration = 262144.0*(float)accelerationPrim/F_CPU;                                         // will overflow without float!
    accelerationDistance2 = 2.0*distance*slowestAxisPlateauTimeRepro*fullSpeed/((float)F_CPU);  // mm^2/s^2
    startSpeed = endSpeed = minSpeed = safeSpeed(drivingAxis);
    if(startSpeed > (isXOrYMove() ? DIRECT_FEEDRATE_XY : isZMove() ? DIRECT_FEEDRATE_Z : DIRECT_FEEDRATE_E)
#if FEATURE_DIGIT_FLOW_COMPENSATION
                                      * g_nDigitFlowCompensation_feedmulti
#endif // FEATURE_DIGIT_FLOW_COMPENSATION
    ){
        startSpeed = endSpeed = minSpeed = (isXOrYMove() ? DIRECT_FEEDRATE_XY : isZMove() ? DIRECT_FEEDRATE_Z : DIRECT_FEEDRATE_E)
#if FEATURE_DIGIT_FLOW_COMPENSATION
                                      * g_nDigitFlowCompensation_feedmulti
#endif // FEATURE_DIGIT_FLOW_COMPENSATION
        ;
    }
    // Can accelerate to full speed within the line
    if (startSpeed * startSpeed + accelerationDistance2 >= fullSpeed * fullSpeed)
        setNominalMove();

    vMax = F_CPU / fullInterval;    // maximum steps per second, we can reach
    // if(p->vMax>46000)            // gets overflow in N computation
    // p->vMax = 46000;
    // p->plateauN = (p->vMax*p->vMax/p->accelerationPrim)>>1;

#if USE_ADVANCE
    if( !isXYZMove() || !isEPositiveMove() )
    {
#ifdef ENABLE_QUADRATIC_ADVANCE
        advanceRate = 0;            // No head move or E move only or sucking filament back
        advanceFull = 0;
#endif // ENABLE_QUADRATIC_ADVANCE
        advanceL = 0;
    }
    else
    {
        float advlin = fabs(speedE) * Extruder::current->advanceL * 0.001 * Printer::axisStepsPerMM[E_AXIS];
        advanceL = (uint16_t)((65536L * advlin) / vMax); //advanceLscaled = (65536*vE*k2)/vMax
#if ENABLE_QUADRATIC_ADVANCE
        advanceFull = 65536 * Extruder::current->advanceK * speedE * speedE; // Steps*65536 at full speed
        long steps = (HAL::U16SquaredToU32(vMax)) / (accelerationPrim << 1); // v^2/(2*a) = steps needed to accelerate from 0-vMax
        advanceRate = advanceFull / steps;
        if((advanceFull >> 16) > maxadv) {
            maxadv = (advanceFull >> 16);
            maxadvspeed = fabs(speedE);
        }
#endif // ENABLE_QUADRATIC_ADVANCE
        if(advlin > maxadv2) {
            maxadv2 = advlin;
            maxadvspeed = fabs(speedE);
        }
    }
#endif // USE_ADVANCE

    UI_MEDIUM;          // do check encoder
    updateTrapezoids();
    // how much steps on primary axis do we need to reach target feedrate
    // p->plateauSteps = (long) (((float)p->acceleration *0.5f / slowestAxisPlateauTimeRepro + p->vMin) *1.01f/slowestAxisPlateauTimeRepro);
#else
#if USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    advanceRate = 0;    // No advance for constant speeds
    advanceFull = 0;
#endif // ENABLE_QUADRATIC_ADVANCE
#endif // USE_ADVANCE
#endif // RAMP_ACCELERATION

    // Make result permanent
    if (pathOptimize) waitRelax = 70;
    DEBUG_MEMORY;

    started = 0;
    task    = TASK_NO_TASK;

} // calculateDirectMove
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING


/** \brief
This is the path planner.
It goes from the last entry and tries to increase the end speed of previous moves in a fashion that the maximum jerk
is never exceeded. If a segment with reached maximum speed is met, the planner stops. Everything left from this
is already optimal from previous updates.
The first 2 entries in the queue are not checked. The first is the one that is already in print and the following will likely become active.

The method is called before linesCount is increased!
*/
void PrintLine::updateTrapezoids()
{
    uint8_t     first = linesWritePos;
    PrintLine*  firstLine;
    PrintLine*  act = &lines[linesWritePos];
    InterruptProtectedBlock noInts; //BEGIN_INTERRUPT_PROTECTED;
    
    // First we find out how far back we could go with optimization.

    uint8_t maxfirst = linesPos;            // first non fixed segment

    if(maxfirst != linesWritePos)
    {
        nextPlannerIndex(maxfirst);         // don't touch the line printing
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
    
    while(timeleft < minTime && maxfirst != linesWritePos)
    {
        timeleft += lines[maxfirst].timeInTicks;
        nextPlannerIndex(maxfirst);
    }

    // Search last fixed element
    while(first != maxfirst && !lines[first].isEndSpeedFixed())
        previousPlannerIndex(first);

    if(first != linesWritePos && lines[first].isEndSpeedFixed())
        nextPlannerIndex(first);
    // now first points to last segment before the end speed is fixed
    // so start speed is also fixed.

    if(first == linesWritePos)              // Nothing to plan
    {
        act->block(); // Prevent stepper interrupt from using this
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
    firstLine->block();                     // don't let printer touch this or following segments during update
    noInts.unprotect(); //END_INTERRUPT_PROTECTED;

    uint8_t previousIndex = linesWritePos;
    previousPlannerIndex(previousIndex);
    PrintLine *previous = &lines[previousIndex];

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
    if( previous->isEOnlyMove() != act->isEOnlyMove() )
    {
        previous->maxJunctionSpeed = RMath::min(previous->endSpeed,act->startSpeed);  //https://github.com/repetier/Repetier-Firmware/issues/716#issuecomment-347000465
        /*
        if(act->isEOnlyMove())      previous->maxJunctionSpeed = previous->safeSpeed(previous->primaryAxis); //previous->endSpeed; //previous->endSpeed;
        if(previous->isEOnlyMove()) previous->maxJunctionSpeed = act->safeSpeed(act->primaryAxis); //act->startSpeed; //previous->endSpeed;
        */
        previous->setEndSpeedFixed(true);
        act->setStartSpeedFixed(true);
        act->updateStepsParameter();
        firstLine->unblock();
        return;
    }else{
        computeMaxJunctionSpeed(previous, act);  // Set maximum junction speed if we have a real move before
    }
    // Increase speed if possible neglecting current speed
    backwardPlanner(linesWritePos,first);
    
    // Reduce speed to reachable speeds
    forwardPlanner(first);

    // Update precomputed data
    do
    {
        lines[first].updateStepsParameter();

        //noInts.protect(); //BEGIN_INTERRUPT_PROTECTED;
        lines[first].unblock();             // start with first block to release next used segment as early as possible
        nextPlannerIndex(first);
        lines[first].block();
        //noInts.unprotect(); //END_INTERRUPT_PROTECTED;

    }while(first!=linesWritePos);

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

inline void PrintLine::computeMaxJunctionSpeed(PrintLine *previous,PrintLine *current)
{
#if USE_ADVANCE
    if(Printer::isAdvanceActivated())
    {
        // if we start/stop extrusion we need to do so with lowest possible end speed
        // or advance would leave a drolling extruder and can not adjust fast enough.
        
        //https://github.com/repetier/Repetier-Firmware/issues/716#issuecomment-347000465 @1)
        if(previous->isEMove() != current->isEMove() && previous->isEMove()) //&& (previous->isXOrYMove() || current->isXOrYMove())
        {
            previous->setEndSpeedFixed(true);
            current->setStartSpeedFixed(true);
            previous->endSpeed = current->startSpeed = previous->maxJunctionSpeed = RMath::min(previous->endSpeed,current->startSpeed);
            previous->invalidateParameter();
            current->invalidateParameter();
            return;
        }
    }
#endif // USE_ADVANCE

    // if we are here we have to identical move types
    // either pure extrusion -> pure extrusion or
    // move -> move (with or without extrusion)
    // First we compute the normalized jerk for speed 1

    float factor = 1.0;
    float lengthFactor = 1.0;
#if REDUCE_ON_SMALL_SEGMENTS
    if(previous->distance < MAX_JERK_DISTANCE)
        lengthFactor = static_cast<float>(MAX_JERK_DISTANCE * MAX_JERK_DISTANCE) / (previous->distance * previous->distance);
#endif
    float maxJoinSpeed = RMath::min(current->fullSpeed, previous->fullSpeed);

#if ALTERNATIVE_JERK
    float jerk = maxJoinSpeed * lengthFactor * (1.0 - (current->speedX * previous->speedX + current->speedY * previous->speedY + current->speedZ * previous->speedZ) / (current->fullSpeed * previous->fullSpeed));
#else
    float dx = current->speedX - previous->speedX;
    float dy = current->speedY - previous->speedY;
    float jerk = sqrt(dx * dx + dy * dy) * lengthFactor;
#endif // ALTERNATIVE_JERK

    if(jerk > Printer::maxJerk) {
        factor = Printer::maxJerk / jerk; // always < 1.0!
        if(factor * maxJoinSpeed * 2.0 < Printer::maxJerk)
            factor = Printer::maxJerk / (2.0 * maxJoinSpeed);
    }

    if((previous->dir | current->dir) & 64) {
        float dz = fabs(current->speedZ - previous->speedZ);
        if(dz > Printer::maxZJerk)
            factor = RMath::min(factor, Printer::maxZJerk / dz);
    }

    float eJerk = fabs(current->speedE - previous->speedE);
    if(eJerk > Extruder::current->maxStartFeedrate)
        factor = RMath::min(factor, Extruder::current->maxStartFeedrate / eJerk);
    previous->maxJunctionSpeed = maxJoinSpeed * factor; // set speed limit

} // computeMaxJunctionSpeed





/** \brief Update parameter used by updateTrapezoids
Computes the acceleration/deceleration steps and advanced parameter associated.
*/
void PrintLine::updateStepsParameter()
{
    if(areParameterUpToDate() || isWarmUp()) return;

    float startFactor = startSpeed * invFullSpeed;
    float endFactor   = endSpeed   * invFullSpeed;
    vStart = vMax * startFactor;    // starting speed
    vEnd   = vMax * endFactor;

    uint32_t vmax2 = HAL::U16SquaredToU32(vMax);
    accelSteps = ((vmax2 - HAL::U16SquaredToU32(vStart)) / (accelerationPrim << 1)) + 1; // Always add 1 for missing precision
    decelSteps = ((vmax2 - HAL::U16SquaredToU32(vEnd))  /(accelerationPrim << 1)) + 1;

#if USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    advanceStart = (float)advanceFull * startFactor * startFactor;
    advanceEnd   = (float)advanceFull * endFactor   * endFactor;
#endif // ENABLE_QUADRATIC_ADVANCE
#endif // USE_ADVANCE

    if(static_cast<int32_t>(accelSteps + decelSteps) >= stepsRemaining)     // can't reach limit speed
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
inline void PrintLine::backwardPlanner(uint8_t start,uint8_t last)
{
    PrintLine *act = &lines[start], *previous;
    float lastJunctionSpeed = act->endSpeed;    // Start always with safe speed

    while(start != last)
    {
        previousPlannerIndex(start);
        previous = &lines[start];
        previous->block();
        // Avoid speed calculate once cruising in split delta move
        // Avoid speed calculate if we know we can accelerate within the line
        lastJunctionSpeed = (act->isNominalMove() ? act->fullSpeed : sqrt(lastJunctionSpeed * lastJunctionSpeed + act->accelerationDistance2));     // acceleration is acceleration*distance*2! What can be reached if we try?

        // If that speed is more that the maximum junction speed allowed then ...
        if(lastJunctionSpeed >= previous->maxJunctionSpeed)     // Limit is reached
        {
            // If the previous line's end speed has not been updated to maximum speed then do it now
            if(previous->endSpeed != previous->maxJunctionSpeed)
            {
                previous->invalidateParameter();                // Needs recomputation
                previous->endSpeed = RMath::max(previous->minSpeed, previous->maxJunctionSpeed);     // possibly unneeded???
            }

            // If actual line start speed has not been updated to maximum speed then do it now
            if(act->startSpeed != previous->maxJunctionSpeed)
            {
                act->startSpeed = RMath::max(act->minSpeed, previous->maxJunctionSpeed);             // possibly unneeded???
                act->invalidateParameter();
            }
            lastJunctionSpeed = previous->endSpeed;
        }
        else
        {
            // Block prev end and act start as calculated speed and recalculate plateau speeds (which could move the speed higher again)
            act->startSpeed = RMath::max(act->minSpeed, lastJunctionSpeed);
            lastJunctionSpeed = previous->endSpeed = RMath::max(lastJunctionSpeed, previous->minSpeed);
            previous->invalidateParameter();
            act->invalidateParameter();
        }
        act = previous;
    } // while loop

} // backwardPlanner


void PrintLine::forwardPlanner(uint8_t first)
{
    PrintLine *act;
    PrintLine *next = &lines[first];
    float vmaxRight;
    float leftSpeed = next->startSpeed;
    while(first != linesWritePos)           // All except last segment, which has fixed end speed
    {
        act = next;
        nextPlannerIndex(first);
        next = &lines[first];
        // Avoid speed calculate if we know we can accelerate within the line.
        vmaxRight = (act->isNominalMove() ? act->fullSpeed : sqrt(leftSpeed * leftSpeed + act->accelerationDistance2));
        if(vmaxRight > act->endSpeed)       // Could be higher next run?
        {
            if(leftSpeed < act->minSpeed)
            {
                leftSpeed = act->minSpeed;
                act->endSpeed = sqrt(leftSpeed * leftSpeed + act->accelerationDistance2);
            }
            act->startSpeed = leftSpeed;
            next->startSpeed = leftSpeed = RMath::max(RMath::min(act->endSpeed,act->maxJunctionSpeed),next->minSpeed);
            if(act->endSpeed == act->maxJunctionSpeed)          // Full speed reached, don't compute again!
            {
                act->setEndSpeedFixed(true);
                next->setStartSpeedFixed(true);
            }
            act->invalidateParameter();
        }
        else     // We can accelerate full speed without reaching limit, which is as fast as possible. Fix it!
        {
            act->fixStartAndEndSpeed();
            act->invalidateParameter();
            if(act->minSpeed > leftSpeed)
            {
                leftSpeed = act->minSpeed;
                vmaxRight = sqrt(leftSpeed * leftSpeed + act->accelerationDistance2);
            }
            act->startSpeed = leftSpeed;
            act->endSpeed = RMath::max(act->minSpeed,vmaxRight);
            next->startSpeed = leftSpeed = RMath::max(RMath::min(act->endSpeed,act->maxJunctionSpeed),next->minSpeed);
            next->setStartSpeedFixed(true);
        }
    } // While
    next->startSpeed = RMath::max(next->minSpeed,leftSpeed);    // This is the new segment, wgich is updated anyway, no extra flag needed.
} // forwardPlanner


inline float PrintLine::safeSpeed(fast8_t drivingAxis)
{
    float xyMin = Printer::maxJerk * 0.5f;
    float mz = 0;
    float safe(xyMin);
    if(isZMove()) {
        mz = Printer::maxZJerk * 0.5f;
        if(isXOrYMove()) {
            if(fabs(speedZ) > mz)
                safe = RMath::min(safe, mz * fullSpeed / fabs(speedZ));
        } else {
            safe = mz;
        }
    }
    if(isEMove()) {
        if(isXYZMove())
            safe = RMath::min(safe, 0.5f * Extruder::current->maxStartFeedrate * fullSpeed / fabs(speedE));
        else
            safe = 0.5f * Extruder::current->maxStartFeedrate; // This is a retraction move
    }
    
    // enforce minimum speed for numerical stability of explicit speed integration
    if(drivingAxis == X_AXIS || drivingAxis == Y_AXIS) safe = RMath::max(xyMin, safe);
    else if(drivingAxis == Z_AXIS)                     safe = RMath::max(mz, safe);
 
    return RMath::min(safe, fullSpeed);
} // safeSpeed


/** \brief Check if move is new. If it is insert some dummy moves to allow the path optimizer to work since it does
not act on the first two moves in the queue. The stepper timer will spot these moves and leave some time for
processing.
*/
uint8_t PrintLine::insertWaitMovesIfNeeded(uint8_t pathOptimize, uint8_t waitExtraLines)
{
    if(linesCount == 0 && waitRelax == 0 && pathOptimize)           // First line after some time - warmup needed
    {
        uint8_t w = 4;
        while(w--)
        {
            PrintLine *p = getNextWriteLine();
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

void PrintLine::waitForXFreeLines(uint8_t b)
{
    while(linesCount + b > MOVE_CACHE_SIZE)     // wait for a free entry in movement cache
    {
        //GCode::readFromSerial();
        Commands::checkForPeriodicalActions();
    }

} // waitForXFreeLines


bool PrintLine::checkForXFreeLines(uint8_t freeLines)
{
    if(linesCount+freeLines>MOVE_CACHE_SIZE)
    {
        // we have less than freeLines free lines within our cache
        return false;
    }

    // we have more than freeLines free lines within our cache
    return true;

} // checkForXFreeLines


#if FEATURE_ARC_SUPPORT
// Arc function taken from grbl
// The arc is approximated by generating a huge number of tiny, linear segments. The length of each
// segment is configured in settings.mm_per_arc_segment.
void PrintLine::arc(float *position, float *target, float *offset, float radius, uint8_t isclockwise)
{
    // int acceleration_manager_was_enabled = plan_is_acceleration_manager_enabled();
    // plan_set_acceleration_manager_enabled(false); // disable acceleration management for the duration of the arc
    float center_axis0 = position[0] + offset[0];
    float center_axis1 = position[1] + offset[1];
    //float linear_travel = 0;              // target[axis_linear] - position[axis_linear];
    float extruder_travel = (Printer::queuePositionTargetSteps[E_AXIS]-Printer::queuePositionLastSteps[E_AXIS])*Printer::invAxisStepsPerMM[E_AXIS];
    float r_axis0 = -offset[0];             // Radius vector from center to current location
    float r_axis1 = -offset[1];
    float rt_axis0 = target[0] - center_axis0;
    float rt_axis1 = target[1] - center_axis1;
    //long xtarget = Printer::queuePositionTargetSteps[X_AXIS];
    //long ytarget = Printer::queuePositionTargetSteps[Y_AXIS];
    //long ztarget = Printer::queuePositionTargetSteps[Z_AXIS];
    //long etarget = Printer::queuePositionTargetSteps[E_AXIS];

    // CCW angle between position and target from circle center. Only one atan2() trig computation required.
    float angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
    if (angular_travel < 0)
    {
        angular_travel += 2*M_PI;
    }
    if (isclockwise)
    {
        angular_travel -= 2*M_PI;
    }

    float millimeters_of_travel = fabs(angular_travel)*radius;  // hypot(angular_travel*radius, fabs(linear_travel));
    if (millimeters_of_travel < 0.001)
    {
        return;
    }

    //  uint16_t segments = (radius>=BIG_ARC_RADIUS ? floor(millimeters_of_travel/MM_PER_ARC_SEGMENT_BIG) : floor(millimeters_of_travel/MM_PER_ARC_SEGMENT));
    // Increase segment size if printing faster then computation speed allows
    uint16_t segments = (Printer::feedrate>60 ? floor(millimeters_of_travel/RMath::min(MM_PER_ARC_SEGMENT_BIG,Printer::feedrate*0.01666*MM_PER_ARC_SEGMENT)) : floor(millimeters_of_travel/MM_PER_ARC_SEGMENT));
    if(segments == 0) segments = 1;

    /*
      // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
      // by a number of discrete segments. The inverse feed_rate should be correct for the sum of
      // all segments.
      if (invert_feed_rate) { feed_rate *= segments; }
    */
    float theta_per_segment = angular_travel/segments;
    //float linear_per_segment = linear_travel/segments;
    float extruder_per_segment = extruder_travel/segments;

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
    float cos_T = 1-0.5*theta_per_segment*theta_per_segment; // Small angle approximation
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
    arc_target[3] = Printer::queuePositionLastSteps[E_AXIS]*Printer::invAxisStepsPerMM[E_AXIS];

    for (i = 1; i<segments; i++)
    {
        if((count & 4) == 0)
        {
            //GCode::readFromSerial();
            Commands::checkForPeriodicalActions();
            UI_MEDIUM; // do check encoder
        }

        if (count < N_ARC_CORRECTION)  //25 pieces
        {
            // Apply vector rotation matrix
            r_axisi = r_axis0*sin_T + r_axis1*cos_T;
            r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
            r_axis1 = r_axisi;
            count++;
        }
        else
        {
            // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
            // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
            cos_Ti  = cos(i*theta_per_segment);
            sin_Ti  = sin(i*theta_per_segment);
            r_axis0 = -offset[0]*cos_Ti + offset[1]*sin_Ti;
            r_axis1 = -offset[0]*sin_Ti - offset[1]*cos_Ti;
            count = 0;
        }

        // Update arc_target location
        arc_target[0] = center_axis0 + r_axis0;
        arc_target[1] = center_axis1 + r_axis1;
        //arc_target[axis_linear] += linear_per_segment;
        arc_target[3] += extruder_per_segment;
        
        Printer::moveToReal(arc_target[0],arc_target[1],IGNORE_COORDINATE,arc_target[3],IGNORE_COORDINATE);
    }
    // Ensure last segment arrives at target location.
    
    Printer::moveToReal(target[0],target[1],IGNORE_COORDINATE,target[3],IGNORE_COORDINATE);

} // arc
#endif // FEATURE_ARC_SUPPORT


long PrintLine::performPauseCheck(){
    if(cur == NULL)
    {
#if FEATURE_PAUSE_PRINTING
        if( g_pauseStatus > PAUSE_STATUS_PAUSED && g_pauseStatus <= PAUSE_STATUS_HEATING ) //siehe gate in Printer::allowQueueMove
        {
            // pause the print now
            switch(g_pauseStatus){
                case PAUSE_STATUS_GOTO_PAUSE1: //pause 1 ohne move, aber mit Retract
                {
#if FEATURE_MILLING_MODE
                    if( Printer::operatingMode == OPERATING_MODE_PRINT )
                    { // we do not process the extruder in case we are not in operating mode "print"
#endif // FEATURE_MILLING_MODE
                       if( g_nPauseSteps[E_AXIS] )
                       {
                           Printer::directPositionTargetSteps[E_AXIS] -= g_nPauseSteps[E_AXIS];
                           g_nContinueSteps[E_AXIS] =                    g_nPauseSteps[E_AXIS];
                           PrintLine::prepareDirectMove();
                       }
#if FEATURE_MILLING_MODE
                    }
#endif // FEATURE_MILLING_MODE
                    g_pauseStatus = PAUSE_STATUS_PAUSED;
                    break;
                }
                case PAUSE_STATUS_GOTO_PAUSE2: //pause 2 mit move
                case PAUSE_STATUS_GOTO_PAUSE3: //pause 3 mit zmove milling unterscheidung in zdeterminepauseposition
                {
                    determinePausePosition(); //für milling wird Z ausgelassen und nachgeholt.
                    PrintLine::prepareDirectMove(); //X, Y, Z for Printing, X, Y for Milling -> dann Z for Milling
                    g_pauseStatus = PAUSE_STATUS_PAUSED;
                    break;
                }
                case PAUSE_STATUS_PREPARE_CONTINUE1: //extrude at continue from normal pause
                {
                    if( g_nContinueSteps[E_AXIS] )
                    {
                        Printer::directPositionTargetSteps[E_AXIS] += g_nContinueSteps[E_AXIS];
                        PrintLine::prepareDirectMove();
                    }
                    g_pauseStatus = PAUSE_STATUS_PAUSED;
                    break;
                }
                case PAUSE_STATUS_PREPARE_CONTINUE2_1: //Move back
                {
#if FEATURE_MILLING_MODE
                    bool modeprint = ( Printer::operatingMode == OPERATING_MODE_PRINT );
#else
                    bool modeprint = true;
#endif // FEATURE_MILLING_MODE
                    if( modeprint )
                    {
                        Printer::directPositionTargetSteps[X_AXIS] += g_nContinueSteps[X_AXIS];
                        Printer::directPositionTargetSteps[Y_AXIS] += g_nContinueSteps[Y_AXIS];
                        Printer::directPositionTargetSteps[Z_AXIS] += g_nContinueSteps[Z_AXIS];
                        Printer::directPositionTargetSteps[E_AXIS] += g_nContinueSteps[E_AXIS];
                    }
                    else
                    {
                        // in operating mode mill, we have 2 continue positions because we have to move into x/y direction before we shall enter the work part
                        Printer::directPositionTargetSteps[X_AXIS] += g_nContinueSteps[X_AXIS];
                        Printer::directPositionTargetSteps[Y_AXIS] += g_nContinueSteps[Y_AXIS];
                    }
                    PrintLine::prepareDirectMove();
                    g_pauseStatus = PAUSE_STATUS_PAUSED;
                    break;
                }
                case PAUSE_STATUS_PREPARE_CONTINUE2_2: //Additional move to old position for milling
                {
                    if( g_nContinueSteps[Z_AXIS] )
                    {
                        Printer::directPositionTargetSteps[Z_AXIS] += g_nContinueSteps[Z_AXIS];
                        PrintLine::prepareDirectMove();
                    }
                    g_pauseStatus = PAUSE_STATUS_PAUSED;
                    break;
                }
            }
            HAL::forbidInterrupts();
            return true;
        }
#endif //FEATURE_PAUSE_PRINTING

        // Pause a bit, if z-compensation is way out of line: this is usefull when starting prints using very deep bed-leveling and custom z-endstop switches which can override a lot.
        short ZcmpNachlauf = abs( Printer::compensatedPositionCurrentStepsZ - Printer::compensatedPositionTargetStepsZ );

        if( ZcmpNachlauf > ZAXIS_STEPS_PER_MM * 0.25f && Printer::compensatedPositionTargetStepsZ ){
            HAL::forbidInterrupts();
            return true;
        }

        if( ZcmpNachlauf && !Printer::doHeatBedZCompensation ){
            HAL::forbidInterrupts();
            return true;
        }
    }
    return false; //ignore this.
}

/**
  Processes the moves from the queue and moves the stepper motors one step. If the last step is reached, the next movement from the queue is started.
  The function must be called from a timer loop. It returns the time for the next call. */
volatile long queueError;
long PrintLine::performQueueMove()
{
    if(cur == NULL)
    {
        if( !linesCount )
        {
            HAL::forbidInterrupts();
            return 1000;
        }

        setCurrentLine();

        if( cur->task )
        {
            switch( cur->task )
            {
#if FEATURE_PAUSE_PRINTING
                case TASK_PAUSE_PRINT:
                {
                    if( g_pauseMode >= PAUSE_MODE_PAUSED )
                    {
                        // this operation can not be performed in case we are paused already
                        removeCurrentLineForbidInterrupt();
                        return 1000;
                    }

                    // the printing shall be paused without moving of the printer/miller head
                    if( linesCount )
                    {
                        g_pauseMode     = PAUSE_MODE_PAUSED;
                        g_pauseStatus   = PAUSE_STATUS_TASKGOTO_PAUSE_1;
                        g_nContinueSteps[X_AXIS] = 0;
                        g_nContinueSteps[Y_AXIS] = 0;
                        g_nContinueSteps[Z_AXIS] = 0;
                        g_nContinueSteps[E_AXIS] = 0;
#if FEATURE_MILLING_MODE
                        if( Printer::operatingMode == OPERATING_MODE_PRINT )
                        {
                            // we do not process the extruder in case we are not in operating mode "print"
#endif // FEATURE_MILLING_MODE
                            if( g_nPauseSteps[E_AXIS] )
                            {
                                Printer::directPositionTargetSteps[E_AXIS] -= g_nPauseSteps[E_AXIS];
                                g_nContinueSteps[E_AXIS]                   =  g_nPauseSteps[E_AXIS];
                                prepareDirectMove();
                            }
#if FEATURE_MILLING_MODE
                        }
#endif // FEATURE_MILLING_MODE
                        g_uPauseTime    = HAL::timeInMilliseconds();
                        g_pauseBeepDone = 0;
                    }
                    removeCurrentLineForbidInterrupt();
                    return 1000;
                }
                
                case TASK_PAUSE_PRINT_AND_MOVE:
                {
                    if( g_pauseMode >= PAUSE_MODE_PAUSED_AND_MOVED )
                    {
                        // this operation can not be performed in case we are in pause position 2 already
                        removeCurrentLineForbidInterrupt();
                        return 1000;
                    }

                    // the printing shall be paused and the printer head shall be moved away
                    if( linesCount )
                    {
                        g_pauseMode     = PAUSE_MODE_PAUSED_AND_MOVED;
                        g_pauseStatus   = PAUSE_STATUS_TASKGOTO_PAUSE_2;
                        g_nContinueSteps[E_AXIS] = 0;
                        if( g_pauseMode == PAUSE_MODE_NONE ) //retract nur wenn nicht bereits in pause 1 passiert.
                        {
#if FEATURE_MILLING_MODE
                            if( Printer::operatingMode == OPERATING_MODE_PRINT )
                            {
                                // we do not process the extruder in case we are not in operating mode "print"
#endif // FEATURE_MILLING_MODE
                                // move the extruder only in case we are not in pause position 1 already
                                if( g_nPauseSteps[E_AXIS] )
                                {
                                    Printer::directPositionTargetSteps[E_AXIS] -= g_nPauseSteps[E_AXIS];
                                    g_nContinueSteps[E_AXIS]                   =  g_nPauseSteps[E_AXIS];
                                }
#if FEATURE_MILLING_MODE
                            }
#endif // FEATURE_MILLING_MODE
                        }
                        determinePausePosition();
                        prepareDirectMove();
                        g_uPauseTime    = HAL::timeInMilliseconds();
                        g_pauseBeepDone = 0;
                    }
                    removeCurrentLineForbidInterrupt();
                    return 1000;
                }
#endif // FEATURE_PAUSE_PRINTING

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
                case TASK_ENABLE_Z_COMPENSATION:
                {
                    Printer::enableCMPnow( );
                    removeCurrentLineForbidInterrupt();
                    return 1000;
                }

                case TASK_DISABLE_Z_COMPENSATION:
                {
                    // disable the z compensation
                    Printer::disableCMPnow(); //hier nicht unbedingt warten, das soll im fluss der queue einfach abgeschaltet werden. zu große abstände regelt die performPauseCheck 
                    removeCurrentLineForbidInterrupt();
                    return 1000;
                }
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

                default:
                {
                    // this is an unknown task - we should never end up here
                    removeCurrentLineForbidInterrupt();
                    return 1000;
                }
            }
        }

        if(cur->isBlocked())   // This step is in computation - shouldn't happen
        {
            cur = NULL;
            return 2000;
        }
        HAL::allowInterrupts();

#ifdef INCLUDE_DEBUG_NO_MOVE
        if(Printer::debugNoMoves())   // simulate a move, but do nothing in reality
        {
            removeCurrentLineForbidInterrupt();
            return 1000;
        }
#endif // INCLUDE_DEBUG_NO_MOVE

        if(cur->isWarmUp())
        {
            // This is a warmup move to initalize the path planner correctly. Just waste
            // a bit of time to get the planning up to date.
            if(linesCount<=cur->getWaitForXLinesFilled())
            {
                cur = NULL;
                return 2000;
            }

            long wait = cur->getWaitTicks();
            removeCurrentLineForbidInterrupt();
            return(wait); // waste some time for path optimization to fill up
        }

        cur->enableSteppers(); //set Z direction etc.
        cur->fixStartAndEndSpeed();

        HAL::allowInterrupts();
        queueError = cur->delta[cur->primaryAxis];
        if(!cur->areParameterUpToDate())  // should never happen, but with bad timings???
        {
            cur->updateStepsParameter();
        }
        Printer::vMaxReached = cur->vStart;
        Printer::stepNumber=0;
        Printer::timer = 0;
        HAL::forbidInterrupts();

#if USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
        Printer::advanceExecuted = cur->advanceStart;
#endif // ENABLE_QUADRATIC_ADVANCE

        cur->updateAdvanceSteps(cur->vStart,0,false);
#endif // USE_ADVANCE

        return Printer::interval;
    }

    return performMove(cur, true);

} // performQueueMove


#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
/**
  Processes the one-and-only direct move and moves the stepper motors one step.
  The function must be called from a timer loop. It returns the time for the next call. */
long directError;
long PrintLine::performDirectMove()
{
    if(direct.started == 0)
    {
        if(direct.isBlocked())   // This step is in computation - shouldn't happen
        {
            return 2000;
        }
        direct.enableSteppers(); //set Z direction etc.
        direct.fixStartAndEndSpeed();

        HAL::allowInterrupts(); //Nibbels todo: prüfen ob das unsinnig ist. unterfunktionen checken. vgl oben 3 zeilen
        directError = direct.delta[direct.primaryAxis];
        if(!direct.areParameterUpToDate())  // should never happen, but with bad timings???
        {
            direct.updateStepsParameter();
        }
        Printer::vMaxReached = direct.vStart;
        Printer::stepNumber=0;
        Printer::timer = 0;
        HAL::forbidInterrupts();

#if USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
        Printer::advanceExecuted = direct.advanceStart;
#endif // ENABLE_QUADRATIC_ADVANCE

        direct.updateAdvanceSteps(direct.vStart,0,false);
#endif // USE_ADVANCE

        return Printer::interval; // Wait an other 50% from last step to make the 100% full
    }

    return performMove(&direct, false);

} // performDirectMove


void PrintLine::performDirectSteps( void )
{
    static unsigned long g_uLastDirectStepTime = 0;
    if( (HAL::timeInMilliseconds() - g_uLastDirectStepTime) >= MANUAL_MOVE_INTERVAL )
    {
        bool bDone = false;
        g_uLastDirectStepTime = HAL::timeInMilliseconds();
        
        if( Printer::directPositionCurrentSteps[E_AXIS] > Printer::directPositionTargetSteps[E_AXIS] )
        {
            bDone = true;
#if USE_ADVANCE
            if( Printer::isAdvanceActivated() ) // Use interrupt for movement
            {
                Printer::extruderStepsNeeded--;
                Printer::directPositionCurrentSteps[E_AXIS] --;
            }
            else
#endif // USE_ADVANCE
            {
              if( Extruder::current->stepperDirection <= 0 )
              {
                // this function shall move the extruder only in case performQueueMove() does not move into the other direction at the moment

                // we must retract filament
                if( Extruder::current->stepperDirection == 0 )
                {
                    // set the direction only in case it is not set already
                    Extruder::setDirection( false );
                    HAL::delayMicroseconds( 1 );
                }
                Extruder::step();
                Printer::insertStepperHighDelay();
                Extruder::unstep();
                Printer::directPositionCurrentSteps[E_AXIS] --;

                if( Printer::directPositionCurrentSteps[E_AXIS] == Printer::directPositionTargetSteps[E_AXIS] )
                {
                    // we have reached the target position
                    if( !isQueueEMove() ) Extruder::current->stepperDirection = 0;
                }
              }
            }
        }
        
        if( !bDone )
        {
            char    nDirectionX = 0;
            char    nDirectionY = 0;
            char    nDirectionZ = 0;

            char    zFirst      = 0;
            char    zLast       = 0;


#if FEATURE_MILLING_MODE
            if( Printer::operatingMode == OPERATING_MODE_MILL )
            {
                // in operating mode "mill", we must move only the z-axis while we leave the work part and when we enter the work part
                if( Printer::directPositionCurrentSteps[Z_AXIS] < Printer::directPositionTargetSteps[Z_AXIS] )
                {
                    // we shall move downwards - do this before any x/y movement
                    zFirst = 1;
                }
                else
                {
                    if( Printer::directPositionCurrentSteps[X_AXIS] != Printer::directPositionTargetSteps[X_AXIS] ||
                        Printer::directPositionCurrentSteps[Y_AXIS] != Printer::directPositionTargetSteps[Y_AXIS] )
                    {
                        // we shall move in x and/or y direction - do not move the heat bed upwards during this time
                        zLast = 1;
                    }
                }
            }
#endif // FEATURE_MILLING_MODE

            if( !zFirst )
            {
                if( Printer::directPositionCurrentSteps[X_AXIS] < Printer::directPositionTargetSteps[X_AXIS] )
                {
                    // move the extruder to the right
                    if( !Printer::stepperDirection[X_AXIS] ) Printer::setXDirection(true);
                    
                    if( Printer::getXDirectionIsPos() ){
                        nDirectionX = 1;
                        bDone       = true;
                    }
                }
                else if( Printer::directPositionCurrentSteps[X_AXIS] > Printer::directPositionTargetSteps[X_AXIS] )
                {
                    // move the extruder to the left
                    if( !Printer::stepperDirection[X_AXIS] ) Printer::setXDirection(false);
                    
                    if( !Printer::getXDirectionIsPos() ){
                        nDirectionX = -1;
                        bDone       = true;
                    }
                }else{
                    if( !isQueueXMove() ) Printer::stepperDirection[X_AXIS] = 0; //setXMoveFinished() für einzel-directsteps
                }

                if( Printer::directPositionCurrentSteps[Y_AXIS] < Printer::directPositionTargetSteps[Y_AXIS] )
                {
                    // move the heat bed to the front
                    if( !Printer::stepperDirection[Y_AXIS] ) Printer::setYDirection(true);
                    
                    if( Printer::getYDirectionIsPos() )
                    {
                        nDirectionY = 1;
                        bDone       = true;
                    }
                }
                else if( Printer::directPositionCurrentSteps[Y_AXIS] > Printer::directPositionTargetSteps[Y_AXIS] )
                {
                    // move the heat bed to the back
                    if( !Printer::stepperDirection[Y_AXIS] ) Printer::setYDirection(false);
                    
                    if( !Printer::getYDirectionIsPos() ){
                        nDirectionY = -1;
                        bDone       = true;
                    }
                }else{
                    if( !isQueueYMove() ) Printer::stepperDirection[Y_AXIS] = 0; //setYMoveFinished() für einzel-directsteps
                }
            }
    
            if( !Printer::blockAll && !zLast )
            {
                // we have to move into z-direction
                if( Printer::directPositionCurrentSteps[Z_AXIS] < Printer::directPositionTargetSteps[Z_AXIS] )
                {
                    // move the heat bed to the bottom
                    if( !Printer::stepperDirection[Z_AXIS] ) Printer::setZDirection(true);
                    
                    if( Printer::getZDirectionIsPos() )
                    {
                        nDirectionZ = 1;
                        bDone       = true;
                    }
                }
                else if( Printer::directPositionCurrentSteps[Z_AXIS] > Printer::directPositionTargetSteps[Z_AXIS] )
                {
                    // move the heat bed to the top
                    if( !Printer::stepperDirection[Z_AXIS] ) Printer::setZDirection(false);
                    
                    if( !Printer::getZDirectionIsPos() )
                    {
                        nDirectionZ = -1;
                        bDone       = true;
                    }
                }else{
                    if( !isDirectOrQueueOrCompZMove() ) Printer::stepperDirection[Z_AXIS] = 0; //setZMoveFinished() für einzel-directsteps
                }
            }

            int8_t nIterations = 4;
            while( nDirectionX || nDirectionY || nDirectionZ )
            {
                HAL::delayMicroseconds( EXTENDED_BUTTONS_STEPPER_DELAY );

                if( nDirectionX ) Printer::startXStep();
                if( nDirectionY ) Printer::startYStep();
                if( nDirectionZ ) Printer::startZStep();

                HAL::delayMicroseconds( EXTENDED_BUTTONS_STEPPER_DELAY );

                if( nDirectionX )
                {
                    Printer::endXStep();
                    Printer::directPositionCurrentSteps[X_AXIS] += nDirectionX;
                    if( Printer::directPositionCurrentSteps[X_AXIS] == Printer::directPositionTargetSteps[X_AXIS] )
                    {
                        // we have finished to move in x-direction
                        nDirectionX = 0;
                        if( !isQueueXMove() ) Printer::stepperDirection[X_AXIS] = 0; //setXMoveFinished() für einzel-directsteps
                    }
                }
                if( nDirectionY )
                {
                    Printer::endYStep();
                    Printer::directPositionCurrentSteps[Y_AXIS] += nDirectionY;
                    if( Printer::directPositionCurrentSteps[Y_AXIS] == Printer::directPositionTargetSteps[Y_AXIS] )
                    {
                        // we have finished to move in y-direction
                        nDirectionY = 0;
                        if( !isQueueYMove() ) Printer::stepperDirection[Y_AXIS] = 0; //setYMoveFinished() für einzel-directsteps
                    }
                }
                if( nDirectionZ )
                {
                    Printer::endZStep();
                    Printer::directPositionCurrentSteps[Z_AXIS] += nDirectionZ;
                    if( Printer::directPositionCurrentSteps[Z_AXIS] == Printer::directPositionTargetSteps[Z_AXIS] )
                    {
                        // we have finished to move in z-direction
                        nDirectionZ = 0;
                        if( !isDirectOrQueueOrCompZMove() ) Printer::stepperDirection[Z_AXIS] = 0; //setZMoveFinished() für einzel-directsteps
                    }
                }

                nIterations --;
                if( !nIterations )
                {
                    // we shall not loop here too long - when we exit here, we will come back to here later and continue the remaining steps
                    break;
                }
            }
        }
        
        if( !bDone )
        {
            if( Printer::directPositionCurrentSteps[E_AXIS] < Printer::directPositionTargetSteps[E_AXIS] )
            {
#if USE_ADVANCE
                if( Printer::isAdvanceActivated() ) // Use interrupt for movement
                {
                    Printer::extruderStepsNeeded++;
                    Printer::directPositionCurrentSteps[E_AXIS] ++;
                }
                else
#endif // USE_ADVANCE
                {
                    if( Extruder::current->stepperDirection >= 0 )
                    {
                        // this function shall move the extruder only in case performQueueMove() does not move into the other direction at the moment
                        // we must output filament
                        if( Extruder::current->stepperDirection == 0 )
                        {
                            // set the direction only in case it is not set already
                            Extruder::setDirection( true );
                            HAL::delayMicroseconds( 1 );
                        }
                        Extruder::step();
                        Printer::insertStepperHighDelay();
                        Extruder::unstep();
                        Printer::directPositionCurrentSteps[E_AXIS] ++;

                        if( Printer::directPositionCurrentSteps[E_AXIS] == Printer::directPositionTargetSteps[E_AXIS] )
                        {
                            // we have reached the target position
                            if( !isQueueEMove() ) Extruder::current->stepperDirection = 0;
                        }
                    }
                }
            }
        }
    }

} // performDirectSteps
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING


long PrintLine::performMove(PrintLine* move, char forQueue)
{
    static bool compensatedPositionPushE = false;
    //static bool compensatedPositionPushZ = false;
    
    HAL::allowInterrupts();
    move->checkEndstops(forQueue);
    fast8_t max_loops = Printer::stepsPerTimerCall;
    if(move->stepsRemaining < max_loops) max_loops = move->stepsRemaining;
    HAL::forbidInterrupts();

    for(fast8_t loop=0; loop < max_loops; loop++) {

#if STEPPER_HIGH_DELAY+DOUBLE_STEP_DELAY > 0
        if(loop) HAL::delayMicroseconds(STEPPER_HIGH_DELAY+DOUBLE_STEP_DELAY);
#endif // STEPPER_HIGH_DELAY+DOUBLE_STEP_DELAY > 0

        if(move->isEMove())
        {
            if((move->error[E_AXIS] -= move->delta[E_AXIS]) < 0
#if FEATURE_HEAT_BED_Z_COMPENSATION
            || compensatedPositionPushE
#endif // FEATURE_HEAT_BED_Z_COMPENSATION
            ) //we want to push some E step out if it is planned OR needed because of ZCMP-E-Compensation
            {
#if USE_ADVANCE
                if(Printer::isAdvanceActivated()) // Use interrupt for movement
                {
                    if(move->isEPositiveMove())
                        Printer::extruderStepsNeeded++;
                    else
                        Printer::extruderStepsNeeded--;
                }
                else
#endif // USE_ADVANCE
                    Extruder::step();

#if FEATURE_HEAT_BED_Z_COMPENSATION
                if(!compensatedPositionPushE){
                    //add % parts of steps to extrusion because of higher layer heights caused by ZCMP
                    if(Printer::compensatedPositionOverPercE != 0.0f){
                        //if we have primaryAxis as E_AXIS we would not be able to smuggle in steps because they are in one row - and it wouldnt make sense anyways, because those moves are retracts typically.
                        if(move->primaryAxis != E_AXIS) Printer::compensatedPositionCollectTinyE += Printer::compensatedPositionOverPercE;
                        if(Printer::compensatedPositionCollectTinyE >= 1.0f){
                            compensatedPositionPushE = true;
                            while (Printer::compensatedPositionCollectTinyE >= 2.0f) Printer::compensatedPositionCollectTinyE--; //notfalls bei überkompensation - sollte nicht vorkommen.
                        }
                    /*
                    }else{
                        Printer::compensatedPositionCollectTinyE = 0.0f; //clear rest if out of compensation?? --> really necessary or is some part of one step acceptable because we are never gonna come back to CMP if we moved out??
                    */
                    }
#endif // FEATURE_HEAT_BED_Z_COMPENSATION
                    //only count step if it is not caused by ZCMP-E-Compensation
                    if( forQueue )  move->error[E_AXIS] += queueError;
                    else            move->error[E_AXIS] += directError;
#if FEATURE_HEAT_BED_Z_COMPENSATION
                }else{
                    //never count or update queue/direct steps if we are smuggling a step amongst others because of ZCMP-E-Compensation
                    compensatedPositionPushE = false;
                    Printer::compensatedPositionCollectTinyE--;
                }
#endif // FEATURE_HEAT_BED_Z_COMPENSATION
            }
        }
        if(move->isXMove())
        {
            if((move->error[X_AXIS] -= move->delta[X_AXIS]) < 0)
            {
                Printer::startXStep();

                if( forQueue )
                {
                    move->error[X_AXIS] += queueError;
                    Printer::queuePositionCurrentSteps[X_AXIS] += (Printer::getXDirectionIsPos() ? 1 : -1);
                }
                else
                {
                    move->error[X_AXIS] += directError;
                    Printer::directPositionCurrentSteps[X_AXIS] += (Printer::getXDirectionIsPos() ? 1 : -1);
                }
            }
        }
        if(move->isYMove())
        {
            if((move->error[Y_AXIS] -= move->delta[Y_AXIS]) < 0)
            {
                Printer::startYStep();

                if( forQueue )
                {
                    move->error[Y_AXIS] += queueError;
                    Printer::queuePositionCurrentSteps[Y_AXIS] += (Printer::getYDirectionIsPos() ? 1 : -1);
                }
                else
                {
                    move->error[Y_AXIS] += directError;
                    Printer::directPositionCurrentSteps[Y_AXIS] += (Printer::getYDirectionIsPos() ? 1 : -1);
                }
            }
        }
        if(move->isZMove())
        {
            if( Printer::blockAll 
            || ( Printer::stepperDirection[Z_AXIS] < 0 && move->task == TASK_MOVE_FROM_BUTTON && Printer::isZMinEndstopHit() ) 
            || ( Printer::stepperDirection[Z_AXIS] > 0 && move->task == TASK_MOVE_FROM_BUTTON && Printer::isZMaxEndstopHit() ) )
            {
                // as soon as the z-axis is blocked, we must finish all z-axis moves
                // Nibbels: Sobald während des Directmoves der Schalter erreicht wird, abbrechen. Anschließend kommen nur noch Single-Steps!!
                move->stepsRemaining = 0;
                move->setZMoveFinished();
            }
            else
            {
                if((move->error[Z_AXIS] -= move->delta[Z_AXIS]) < 0) {
                    //070118better zCMP strategy: if zCMP is against Queue/Direct, dont do some step and count it done @Queue/Direct and count it done @zCMP.
                    char dir = (Printer::getZDirectionIsPos() ? 1 : -1);
                    bool cmpup = (Printer::compensatedPositionCurrentStepsZ < Printer::compensatedPositionTargetStepsZ);
                    bool cmpdown = (Printer::compensatedPositionCurrentStepsZ > Printer::compensatedPositionTargetStepsZ);
                    if(dir < 0 && cmpup){
                        Printer::compensatedPositionCurrentStepsZ++;
                    }else if(dir > 0 && cmpdown){
                        Printer::compensatedPositionCurrentStepsZ--;
                    }else{
                        //no conflicting zCMP: Do the Z-Step.
                        Printer::startZStep();
                    }

                    //070118 einfügen von zusätzlichen Z-steps macht evtl. zu wenig sinn, weil Z normalerweise die Hauptachse ist. würde nur bei einer art ständigem vasenmodus was bringen?? Ich kann nicht mehr als 100% Z-Steps einmogeln. Bei E war das kein Problem, weils normalerweise nicht die Hauptachse ist.
                    /*
                    if((move->error[Z_AXIS] -= move->delta[Z_AXIS]) < 0 || compensatedPositionPushZ) {
                    
                    if (!compensatedPositionPushZ && (dir > 0 && cmpup || dir < 0 && cmpdown)){
                        compensatedPositionPushZ = true;
                    }
                    und if compensatedPositionPushZ dann löschen und hier reinspringen und das hier drunter nicht zählen, weil es nicht zu queue oder direct gehört und auch nicht in die koordinatenachse ..
                    if(!compensatedPositionPushZ){
                        forQueue...
                    }
                    */

                    if( forQueue )
                    {
                        move->error[Z_AXIS] += queueError;
                        Printer::queuePositionCurrentSteps[Z_AXIS] += dir;
                    }
                    else
                    {
                        move->error[Z_AXIS] += directError;
                        Printer::directPositionCurrentSteps[Z_AXIS] += dir;
                    }
                    // Note: There is no need to check whether we are past the z-min endstop here because G-Codes typically are not able to go past the z-min endstop anyways.
                }
            }
        }
        Printer::insertStepperHighDelay();

#if USE_ADVANCE
        if(!Printer::isAdvanceActivated()) // Use interrupt for movement
#endif // USE_ADVANCE
            Extruder::unstep();

        move->stepsRemaining--;
        Printer::endXYZSteps();
    } // for loop
    HAL::allowInterrupts(); // Allow interrupts for other types, timer1 is still disabled

#if RAMP_ACCELERATION
    //If acceleration is enabled on this move and we are in the acceleration segment, calculate the current interval
    if (move->moveAccelerating())   // we are accelerating
    {
        Printer::vMaxReached = HAL::ComputeV(Printer::timer,move->fAcceleration)+move->vStart;
        if(Printer::vMaxReached > move->vMax) Printer::vMaxReached = move->vMax;
        unsigned int v = Printer::updateStepsPerTimerCall(Printer::vMaxReached);
        Printer::interval = HAL::CPUDivU2(v);
        Printer::timer+=Printer::interval;
#if USE_ADVANCE
        move->updateAdvanceSteps(Printer::vMaxReached, max_loops, true);
#endif // USE_ADVANCE
        Printer::stepNumber += max_loops; // is only used by moveAccelerating --> Nibbels TODO: Check if this is maybe right here: source repetier development
    }
    else if (move->moveDecelerating())     // time to slow down
    {
        unsigned int v = HAL::ComputeV(Printer::timer,move->fAcceleration);
        if (v > Printer::vMaxReached)   // if deceleration goes too far it can become too large
            v = move->vEnd;
        else
        {
            v = Printer::vMaxReached - v;
            if (v < move->vEnd) v = move->vEnd; // extra steps at the end of desceleration due to rounding errors
        }
#if USE_ADVANCE
        move->updateAdvanceSteps(v, max_loops, false); // needs original v
#endif // USE_ADVANCE
        v = Printer::updateStepsPerTimerCall(v);
        Printer::interval = HAL::CPUDivU2(v);
        Printer::timer += Printer::interval;
    }
    else // full speed reached
    {
        // If we had acceleration, we need to use the latest vMaxReached and interval
        // If we started full speed, we need to use move->fullInterval and vMax
#if USE_ADVANCE
        move->updateAdvanceSteps((!move->accelSteps ? move->vMax : Printer::vMaxReached), 0, true);
#endif // USE_ADVANCE
        if(!move->accelSteps) {
            if(move->vMax > Printer::stepsDoublerFrequency) {
#if ALLOW_QUADSTEPPING
                if(move->vMax > Printer::stepsDoublerFrequency * 2) {
                    Printer::stepsPerTimerCall = 4;
                    Printer::interval = move->fullInterval << 2;
                } else {
                    Printer::stepsPerTimerCall = 2;
                    Printer::interval = move->fullInterval << 1;
                }
#else // ALLOW_QUADSTEPPING
                Printer::stepsPerTimerCall = 2;
                Printer::interval = move->fullInterval << 1;
#endif // ALLOW_QUADSTEPPING
            } else {
                Printer::stepsPerTimerCall = 1;
                Printer::interval = move->fullInterval;
            }
        }
    }
#else
    Printer::stepsPerTimerCall = 1;
    Printer::interval = move->fullInterval; // without RAMPS always use full speed
#endif // RAMP_ACCELERATION
    long interval = Printer::interval;
    if( move->stepsRemaining <= 0 || move->isNoMove() )  // line finished
    {
        if( move->stepsRemaining <= 0 ) //Wenn keine Steps mehr da, sollten alle Achsen die benutzt wurden wieder freigegeben werden. Bei Z ist der Sonderfall, dass die Z-Kompensation sich reinschummeln könnte.
        {
            move->setXMoveFinished();
            move->setYMoveFinished();
            move->setZMoveFinished(); //Wichtig, das die Z-Kompensation wieder weiterarbeiten kann, auch wichtig bei PrintLine::direct! 
            //Auch wenn kein Z-Move, könnte die Z-Kompensation die Achse Z benutzt haben.
            //gesetztes endZCompensationStep bei Z finish wäre egal, beendet wird auch ohne Richtung.
            move->setEMoveFinished();
        }

        if( forQueue )
        {
            removeCurrentLineForbidInterrupt();
        }
        else
        {
            move->stepsRemaining = 0;
            move->task           = TASK_NO_TASK;

            Printer::directPositionTargetSteps[X_AXIS] = Printer::directPositionLastSteps[X_AXIS] = Printer::directPositionCurrentSteps[X_AXIS];
            Printer::directPositionTargetSteps[Y_AXIS] = Printer::directPositionLastSteps[Y_AXIS] = Printer::directPositionCurrentSteps[Y_AXIS];
            Printer::directPositionTargetSteps[Z_AXIS] = Printer::directPositionLastSteps[Z_AXIS] = Printer::directPositionCurrentSteps[Z_AXIS];
            Printer::directPositionTargetSteps[E_AXIS] = Printer::directPositionLastSteps[E_AXIS] = Printer::directPositionCurrentSteps[E_AXIS];

            Commands::printCurrentPosition();
        }

        char    nIdle = 1;

#if FEATURE_MILLING_MODE
        if( Printer::operatingMode == OPERATING_MODE_PRINT )
        {
#if FEATURE_HEAT_BED_Z_COMPENSATION
            if( g_nHeatBedScanStatus || g_ZOSScanStatus )
            {
                // we are not idle because the heat bed scan is going on at the moment
                nIdle = 0;
            }
#endif // FEATURE_HEAT_BED_Z_COMPENSATION
        }
        else
        {
#if FEATURE_WORK_PART_Z_COMPENSATION
            if( g_nWorkPartScanStatus )
            {
                // we are not idle because the work part scan is going on at the moment
                nIdle = 0;
            }
#endif // FEATURE_WORK_PART_Z_COMPENSATION
        }
#else
#if FEATURE_HEAT_BED_Z_COMPENSATION
        if( g_nHeatBedScanStatus || g_ZOSScanStatus )
        {
            // we are not idle because the heat bed scan is going on at the moment
            nIdle = 0;
        }
#endif // FEATURE_HEAT_BED_Z_COMPENSATION
#endif // FEATURE_MILLING_MODE

        if(linesCount == 0 && nIdle)
        {
            g_uStartOfIdle = HAL::timeInMilliseconds();
        }

        interval = Printer::interval = interval >> 1; // 50% of time to next call to do cur=0
        DEBUG_MEMORY;
    } // line finished

    return interval;
} // performMove

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


#ifndef MOTION_H
#define MOTION_H

/** Marks the first step of a new move */
#define FLAG_WARMUP 1
#define FLAG_NOMINAL 2
#define FLAG_DECELERATING 4
#define FLAG_ACCELERATION_ENABLED 8
#define FLAG_CHECK_ENDSTOPS 16
#define FLAG_SKIP_ACCELERATING 32
#define FLAG_SKIP_DEACCELERATING 64
#define FLAG_BLOCKED 128

#define FLAG_JOIN_STEPPARAMS_COMPUTED   1   // Are the step parameter computed
#define FLAG_JOIN_END_FIXED             2   // The right speed is fixed. Don't check this block or any block to the left.
#define FLAG_JOIN_START_FIXED           4   // The left speed is fixed. Don't check left block.
#define FLAG_JOIN_START_RETRACT         8   // Start filament retraction at move start
#define FLAG_JOIN_END_RETRACT           16  // Wait for filament pushback, before ending move
#define FLAG_JOIN_NO_RETRACT            32  // Disable retract for this line
#define FLAG_JOIN_WAIT_EXTRUDER_UP      64  // Wait for the extruder to finish it's up movement
#define FLAG_JOIN_WAIT_EXTRUDER_DOWN    128 // Wait for the extruder to finish it's down movement

class UIDisplay;
class PrintLine
{
    friend class UIDisplay;

public:
    static uint8_t      linesPos;       // Position for executing line movement
    static PrintLine    lines[];
    static uint8_t      linesWritePos;  // Position where we write the next cached line move
    flag8_t             joinFlags;
    volatile flag8_t    flags;
    volatile uint8_t    started;

private:
    flag8_t             primaryAxis;
    int32_t             timeInTicks;
    flag8_t             dir;                        ///< Direction of movement. 1 = X+, 2 = Y+, 4= Z+, values can be combined.
    int32_t             delta[4];                   ///< Steps we want to move.
    int32_t             error[4];                   ///< Error calculation for Bresenham algorithm
    float               speedX;                     ///< Speed in x direction at vMax in mm/s
    float               speedY;                     ///< Speed in y direction at vMax in mm/s
    float               speedZ;                     ///< Speed in z direction at vMax in mm/s
    float               speedE;                     ///< Speed in E direction at vMax in mm/s
    float               fullSpeed;                  ///< Desired speed mm/s
    float               invFullSpeed;               ///< 1.0/fullSpeed for faster computation
    float               accelerationDistance2;      ///< Real 2.0*distance*acceleration mm²/s²
    float               maxJunctionSpeed;           ///< Max. junction speed between this and next segment
    float               startSpeed;                 ///< Starting speed in mm/s
    float               endSpeed;                   ///< Exit speed in mm/s
    float               minSpeed;
    float               distance;
    //ticks_t             fullInterval;               ///< interval at full speed in ticks/step.
    uint32_t            accelSteps;                 ///< How much steps does it take, to reach the plateau.
    uint32_t            decelSteps;                 ///< How much steps does it take, to reach the end speed.
    uint32_t            accelerationPrim;           ///< Acceleration along primary axis
    uint32_t            fAcceleration;              ///< accelerationPrim*262144/F_CPU
    speed_t             vMax;                       ///< Maximum reached speed in steps/s.
    speed_t             vStart;                     ///< Starting speed in steps/s.
    speed_t             vEnd;                       ///< End speed in steps/s

#if USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    int32_t             advanceRate;               ///< Advance steps at full speed
    int32_t             advanceFull;               ///< Maximum advance at fullInterval [steps*65536]
    int32_t             advanceStart;
    int32_t             advanceEnd;
#endif // ENABLE_QUADRATIC_ADVANCE

    uint16_t            advanceL;                   ///< Recomputated L value
#endif // USE_ADVANCE

public:
    int32_t             stepsRemaining;            ///< Remaining steps, until move is finished
    static PrintLine*   cur;
    char                task;

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
    static PrintLine    direct;
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
    
    static volatile uint8_t linesCount; // Number of lines cached 0 = nothing to do

    inline bool areParameterUpToDate()
    {
        return joinFlags & FLAG_JOIN_STEPPARAMS_COMPUTED;
    } // areParameterUpToDate

    inline void invalidateParameter()
    {
        joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED;
    } // invalidateParameter

    inline void setParameterUpToDate()
    {
        joinFlags |= FLAG_JOIN_STEPPARAMS_COMPUTED;
    } // setParameterUpToDate

    inline bool isStartSpeedFixed()
    {
        return joinFlags & FLAG_JOIN_START_FIXED;
    } // isStartSpeedFixed

    inline void setStartSpeedFixed(bool newState)
    {
        joinFlags = (newState ? joinFlags | FLAG_JOIN_START_FIXED : joinFlags & ~FLAG_JOIN_START_FIXED);
    } // setStartSpeedFixed

    inline void fixStartAndEndSpeed()
    {
        joinFlags |= FLAG_JOIN_END_FIXED | FLAG_JOIN_START_FIXED;
    } // fixStartAndEndSpeed

    inline bool isEndSpeedFixed()
    {
        return joinFlags & FLAG_JOIN_END_FIXED;
    } // isEndSpeedFixed

    inline void setEndSpeedFixed(bool newState)
    {
        joinFlags = (newState ? joinFlags | FLAG_JOIN_END_FIXED : joinFlags & ~FLAG_JOIN_END_FIXED);
    } // setEndSpeedFixed

    inline bool isWarmUp()
    {
        return flags & FLAG_WARMUP;
    } // isWarmUp

    inline uint8_t getWaitForXLinesFilled()
    {
        return primaryAxis;
    } // getWaitForXLinesFilled

    inline void setWaitForXLinesFilled(uint8_t b)
    {
        primaryAxis = b;
    } // setWaitForXLinesFilled

    inline bool isExtruderForwardMove()
    {
        return (dir & 136)==136; //isEPositiveMove()
    } // isExtruderForwardMove

    inline void block()
    {
        flags |= FLAG_BLOCKED;
    } // block

    inline void unblock()
    {
        flags &= ~FLAG_BLOCKED;
    } // unblock

    inline bool isBlocked()
    {
        return flags & FLAG_BLOCKED;
    } // isBlocked

    inline bool isCheckEndstops()
    {
        return flags & FLAG_CHECK_ENDSTOPS;
    } // isCheckEndstops

    inline bool isNominalMove()
    {
        return flags & FLAG_NOMINAL;
    } // isNominalMove

    inline void setNominalMove()
    {
        flags |= FLAG_NOMINAL;
    } // setNominalMove

    inline void checkEndstops(char forQueue)
    {
        if(isCheckEndstops())
        {
            //Min-Axis:
            if(isXNegativeMove() && Printer::isXMinEndstopHit())
                setXMoveFinished();
            if(isYNegativeMove() && Printer::isYMinEndstopHit())
                setYMoveFinished();
            //Max-Axis:
            if(isXPositiveMove() && Printer::isXMaxEndstopHit()){
                setXMoveFinished();
                if(forQueue){
                  Printer::queuePositionLastSteps[X_AXIS] = Printer::queuePositionTargetSteps[X_AXIS] = Printer::queuePositionCurrentSteps[X_AXIS];
                }else{
                  Printer::directPositionLastSteps[X_AXIS] = Printer::directPositionTargetSteps[X_AXIS] = Printer::directPositionCurrentSteps[X_AXIS]; //Wenn man G28 und G1 Z200 macht, er vorher gestoppt wird und man zurückfährt, landet er im Minus. Weil der Drucker denkt, er wäre von 200 gestartet.
                }
                Printer::updateCurrentPosition(true);
            }
            if(isYPositiveMove() && Printer::isYMaxEndstopHit()){
                setYMoveFinished();
                if(forQueue){
                  Printer::queuePositionLastSteps[Y_AXIS] = Printer::queuePositionTargetSteps[Y_AXIS] = Printer::queuePositionCurrentSteps[Y_AXIS];
                }else{
                  Printer::directPositionLastSteps[Y_AXIS] = Printer::directPositionTargetSteps[Y_AXIS] = Printer::directPositionCurrentSteps[Y_AXIS]; //Wenn man G28 und G1 Z200 macht, er vorher gestoppt wird und man zurückfährt, landet er im Minus. Weil der Drucker denkt, er wäre von 200 gestartet.
                }
                Printer::updateCurrentPosition(true);
            }
            if(isZPositiveMove() && (Printer::isZMaxEndstopHit() || Printer::currentZSteps > (Printer::maxSteps[Z_AXIS] /*- Printer::minSteps[Z_AXIS]*/) + abs(Z_OVERRIDE_MAX * 2) ))
            {
                setZMoveFinished();
                if(forQueue){
                  Printer::queuePositionLastSteps[Z_AXIS] = Printer::queuePositionTargetSteps[Z_AXIS] = Printer::queuePositionCurrentSteps[Z_AXIS]; //Wenn man G28 und G1 Z200 macht, er vorher gestoppt wird und man zurückfährt, landet er im Minus. Weil der Drucker denkt, er wäre von 200 gestartet.
                }else{
                  Printer::directPositionLastSteps[Z_AXIS] = Printer::directPositionTargetSteps[Z_AXIS] = Printer::directPositionCurrentSteps[Z_AXIS]; //Wenn man G28 und G1 Z200 macht, er vorher gestoppt wird und man zurückfährt, landet er im Minus. Weil der Drucker denkt, er wäre von 200 gestartet.
                }
                Printer::updateCurrentPosition(true);
            }
        }

        // Test Z-Axis every step, otherwise it could easyly ruin your printer!
        if(isZNegativeMove() && Printer::isZMinEndstopHit())
        {
            if( Printer::isAxisHomed(Z_AXIS) && PrintLine::direct.task != TASK_MOVE_FROM_BUTTON)
            {
                if( Printer::currentZSteps <= -Z_OVERRIDE_MAX )
                {
                    // --> setZMoveFinished(); //-> some lines down!
                }else{
                    // we allow to overdrive Z-min a little bit so that also G-Codes are able to move to a smaller z-position even when Z-min has fired already
                    return;
                }
            }
            setZMoveFinished();
        }
    } // checkEndstops

    inline void setXMoveFinished()
    {
        dir&=~16;
        Printer::stepperDirection[X_AXIS] = 0;
    } // setXMoveFinished

    inline void setYMoveFinished()
    {
        dir&=~32;
        Printer::stepperDirection[Y_AXIS] = 0;
    } // setYMoveFinished

    inline void setZMoveFinished()
    {
        dir&=~64;
        Printer::stepperDirection[Z_AXIS] = 0;
    } // setZMoveFinished

    inline void setEMoveFinished()
    {
        dir&=~128;
        Extruder::current->stepperDirection = 0;
    } // setEMoveFinished

    inline void setXYMoveFinished()
    {
        dir&=~48;
        Printer::stepperDirection[Y_AXIS] = 0;
        Printer::stepperDirection[X_AXIS] = 0;
    } // setXYMoveFinished

    inline bool isXPositiveMove()
    {
        return (dir & 17)==17;
    } // isXPositiveMove

    inline bool isXNegativeMove()
    {
        return (dir & 17)==16;
    } // isXNegativeMove

    inline bool isYPositiveMove()
    {
        return (dir & 34)==34;
    } // isYPositiveMove

    inline bool isYNegativeMove()
    {
        return (dir & 34)==32;
    } // isYNegativeMove

    inline bool isZPositiveMove()
    {
        return (dir & 68)==68;
    } // isZPositiveMove

    inline bool isZNegativeMove()
    {
        return (dir & 68)==64;
    } // isZNegativeMove

    inline bool isEPositiveMove()
    {
        return (dir & 136)==136;
    } // isEPositiveMove

    inline bool isENegativeMove()
    {
        return (dir & 136)==128;
    } // isENegativeMove

    inline bool isXMove()
    {
        return (dir & 16);
    } // isXMove

    inline bool isYMove()
    {
        return (dir & 32);
    } // isYMove

    inline bool isXOrYMove()
    {
        return dir & 48;
    } // isXOrYMove

    inline bool isZMove()
    {
        return (dir & 64);
    } // isZMove

    inline bool isEMove()
    {
        return (dir & 128);
    } // isEMove

    inline bool isEOnlyMove()
    {
        return (dir & 240)==128;
    } // isEOnlyMove

    inline bool isNoMove()
    {
        return (dir & 240)==0;
    } // isNoMove

    inline bool isXYZMove()
    {
        return dir & 112;
    } // isXYZMove

    inline bool isMoveOfAxis(uint8_t axis)
    {
        return (dir & (16<<axis));
    } // isMoveOfAxis

    inline void setMoveOfAxis(uint8_t axis)
    {
        dir |= 16<<axis;
    } // setMoveOfAxis

    inline void setPositiveDirectionForAxis(uint8_t axis)
    {
        dir |= 1<<axis;
    } // setPositiveDirectionForAxis

    inline static void resetPathPlanner()
    {
        linesCount    = 0;
        linesPos = linesWritePos;
    } // resetPathPlanner

#if USE_ADVANCE
    inline void updateAdvanceSteps(speed_t v,uint8_t max_loops,bool accelerate)
    {
        if(!Printer::isAdvanceActivated()) return;
#ifdef ENABLE_QUADRATIC_ADVANCE
        long advanceTarget = Printer::advanceExecuted;
        if(accelerate) {
            for(uint8_t loop = 0; loop < max_loops; loop++) advanceTarget += advanceRate;
            if(advanceTarget > advanceFull)
                advanceTarget = advanceFull;
        } else {
            for(uint8_t loop = 0; loop < max_loops; loop++) advanceTarget -= advanceRate;
            if(advanceTarget < advanceEnd)
                advanceTarget = advanceEnd;
        }
        long h = HAL::mulu16xu16to32(v, advanceL);
        int tred = ((advanceTarget + h) >> 16);
        HAL::forbidInterrupts();
        Printer::extruderStepsNeeded += tred - Printer::advanceStepsSet;
        if(tred > 0 && Printer::advanceStepsSet <= 0)
            Printer::extruderStepsNeeded += Extruder::current->advanceBacklash;
        else if(tred < 0 && Printer::advanceStepsSet >= 0)
            Printer::extruderStepsNeeded -= Extruder::current->advanceBacklash;
        Printer::advanceStepsSet = tred;
        HAL::allowInterrupts();
        Printer::advanceExecuted = advanceTarget;
#else
        int tred = HAL::mulu6xu16shift16(v, advanceL);
        HAL::forbidInterrupts();
        Printer::extruderStepsNeeded += tred - Printer::advanceStepsSet;
        if(tred > 0 && Printer::advanceStepsSet <= 0)
            Printer::extruderStepsNeeded += (Extruder::current->advanceBacklash << 1);
        else if(tred < 0 && Printer::advanceStepsSet >= 0)
            Printer::extruderStepsNeeded -= (Extruder::current->advanceBacklash << 1);
        Printer::advanceStepsSet = tred;
        HAL::allowInterrupts();
        (void)max_loops;
        (void)accelerate;
#endif // ENABLE_QUADRATIC_ADVANCE
    } // updateAdvanceSteps
#endif // USE_ADVANCE

    INLINE bool moveDecelerating()
    {
        if(stepsRemaining <= static_cast<int32_t>(decelSteps))
        {
            if (!(flags & FLAG_DECELERATING)) //reset "timer" only once.
            {
                Printer::timer = 0;
                flags |= FLAG_DECELERATING;
            }
            return true;
        }
        else return false;
    } // moveDecelerating

    INLINE bool moveAccelerating()
    {
        return Printer::stepNumber <= accelSteps;
    } // moveAccelerating

    void updateStepsParameter();
    inline float safeSpeed(fast8_t drivingAxis);
    void calculateQueueMove(float axis_diff[],uint8_t pathOptimize, fast8_t drivingAxis, float feedrate
    );

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
    void calculateDirectMove(float axis_diff[],uint8_t pathOptimize, fast8_t drivingAxis);
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

    INLINE long getWaitTicks()
    {
        return timeInTicks;
    } // getWaitTicks

    INLINE void setWaitTicks(long wait)
    {
        timeInTicks = wait;
    } // setWaitTicks

    static INLINE bool hasLines()
    {
        return linesCount;
    } // hasLines

    static INLINE void setCurrentLine()
    {
        cur = &lines[linesPos];
    } // setCurrentLine

    static INLINE void removeCurrentLineForbidInterrupt()
    {
        nextPlannerIndex(linesPos);
        cur->task = TASK_NO_TASK;
        cur = NULL;
        HAL::forbidInterrupts();
        --linesCount;
    } // removeCurrentLineForbidInterrupt

    static INLINE void pushLine()
    {
        nextPlannerIndex(linesWritePos);
        InterruptProtectedBlock noInts;
        linesCount++;
        g_uStartOfIdle = 0; //tell the printer work is being done.
    } // pushLine

    static PrintLine *getNextWriteLine()
    {
        return &lines[linesWritePos];
    } // getNextWriteLine

    static inline void computeMaxJunctionSpeed(PrintLine *previous,PrintLine *current);
    static long performPauseCheck();
    static long performQueueMove();

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
    static long performDirectMove( void );
    static void performDirectSteps( void );
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

    static long performMove(PrintLine* move, char forQueue);
    static void waitForXFreeLines(uint8_t b=1);
    static bool checkForXFreeLines(uint8_t freeLines=1);
    static inline void forwardPlanner(uint8_t p);
    static inline void backwardPlanner(uint8_t p,uint8_t last);
    static void updateTrapezoids();
    static uint8_t insertWaitMovesIfNeeded(uint8_t pathOptimize, uint8_t waitExtraLines);
    static void prepareQueueMove(uint8_t check_endstops, uint8_t pathOptimize, float feedrate);

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
    static void prepareDirectMove( void );
    static void stopDirectMove( void );
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

    static void moveRelativeDistanceInSteps(long x,long y,long z,long e,float feedrate,bool waitEnd,bool check_endstop);
    static void moveRelativeDistanceInStepsReal(long x,long y,long z,long e,float feedrate,bool waitEnd);

#if FEATURE_ARC_SUPPORT
    static void arc(float *position, float *target, float *offset, float radius, uint8_t isclockwise);
#endif // FEATURE_ARC_SUPPORT

    static INLINE void previousPlannerIndex(uint8_t &p)
    {
        p = (p ? p - 1 : MOVE_CACHE_SIZE - 1);
    } // previousPlannerIndex

    static INLINE void nextPlannerIndex(uint8_t& p)
    {
        p = (p >= MOVE_CACHE_SIZE - 1 ? 0 : p + 1);
    } // nextPlannerIndex

    static inline void queueTask( char task )
    {
        PrintLine*  p;


        p = getNextWriteLine();
        p->task = task;
  
        nextPlannerIndex( linesWritePos );
        InterruptProtectedBlock noInts; //BEGIN_INTERRUPT_PROTECTED
        linesCount++;
        //END_INTERRUPT_PROTECTED
        return;
    } // queueTask

    inline void enableSteppers( void )
    {
        if( Printer::blockAll )
        {
            // do not enable anything in case everything is blocked
            return;
        }

        // Only enable axis that are moving. If the axis doesn't need to move then it can stay disabled depending on configuration.
        if(isXMove())
        {
            Printer::enableXStepper();
            Printer::setXDirection(isXPositiveMove());
        }
        if(isYMove())
        {
            Printer::enableYStepper();
            Printer::setYDirection(isYPositiveMove());
        }
        if(isZMove())
        {
            Printer::enableZStepper();
            Printer::setZDirection(isZPositiveMove());
        }
        if(isEMove())
        {
            Printer::unmarkAllSteppersDisabled(); //doesnt fit into Extruder::enable() because of forward declare -> TODO
            Extruder::enable();
#if USE_ADVANCE
            if(!Printer::isAdvanceActivated()) // Set direction if no advance/OPS enabled
#endif
                  Extruder::setDirection(isEPositiveMove());
        }
        started = 1;
    } // enableSteppers

};

inline bool isQueueXMove(){
    if( PrintLine::cur ) if( PrintLine::cur->isXMove() ) return true;
    return false;
} //isQueueXMove
inline bool isQueueYMove(){
    if( PrintLine::cur ) if( PrintLine::cur->isYMove() ) return true;
    return false;
} //isQueueYMove
inline bool isQueueEMove(){
    if( PrintLine::cur ) if( PrintLine::cur->isEMove() ) return true;
    return false;
} //isQueueEMove

inline bool isDirectOrQueueOrCompZMove(){
    if( PrintLine::cur ) if( PrintLine::cur->isZMove() ) return true;
    if( Printer::directPositionCurrentSteps[Z_AXIS] != Printer::directPositionTargetSteps[Z_AXIS] ) return true; 
    // davor version conrad 1.39, auskommentiert version nibbels: if( PrintLine::direct.isZMove() ) return true;
    if( Printer::endZCompensationStep ) return true;
    return false;
} //isDirectOrQueueOrCompZMove

#endif // MOTION_H

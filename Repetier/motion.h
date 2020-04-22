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
#define FLAG_ABORT_AT_ENDSTOPS 16
#define FLAG_SKIP_ACCELERATING 32
#define FLAG_SKIP_DEACCELERATING 64
#define FLAG_BLOCKED 128

#define FLAG_JOIN_STEPPARAMS_COMPUTED 1  // Are the step parameter computed
#define FLAG_JOIN_END_FIXED 2            // The right speed is fixed. Don't check this block or any block to the left.
#define FLAG_JOIN_START_FIXED 4          // The left speed is fixed. Don't check left block.
#define FLAG_JOIN_START_RETRACT 8        // Start filament retraction at move start
#define FLAG_JOIN_END_RETRACT 16         // Wait for filament pushback, before ending move
#define FLAG_JOIN_NO_RETRACT 32          // Disable retract for this line
#define FLAG_JOIN_WAIT_EXTRUDER_UP 64    // Wait for the extruder to finish it's up movement
#define FLAG_JOIN_WAIT_EXTRUDER_DOWN 128 // Wait for the extruder to finish it's down movement

#define DIRECT_IDLE 0
#define DIRECT_PREPARING 1
#define DIRECT_PREPARED 2
#define DIRECT_RUNNING 3

#define DIRECT_PREPARED_STOPPABLE 12
#define DIRECT_RUNNING_STOPPABLE 13

#define FOR_DIRECT 0
#define FOR_QUEUE 1

class UIDisplay;
class PrintLine {
    friend class UIDisplay;

public:
    static uint8_t linesPos; // Position for executing line movement
    static PrintLine lines[];
    static uint8_t linesWritePos; // Position where we write the next cached line move
    flag8_t joinFlags;
    volatile flag8_t flags;

private:
    flag8_t primaryAxis;
    int32_t timeInTicks;
    flag8_t dir;                 ///< Direction of movement. 1 = X+, 2 = Y+, 4= Z+, values can be combined.
    int32_t delta[4];            ///< Steps we want to move.
    int32_t error[4];            ///< Error calculation for Bresenham algorithm
    float speedX;                ///< Speed in x direction at vMax in mm/s
    float speedY;                ///< Speed in y direction at vMax in mm/s
    float speedZ;                ///< Speed in z direction at vMax in mm/s
    float speedE;                ///< Speed in E direction at vMax in mm/s
    float fullSpeed;             ///< Desired speed mm/s
    float invFullSpeed;          ///< 1.0/fullSpeed for faster computation
    float accelerationDistance2; ///< Real 2.0*distance*acceleration mm²/s²
    float maxJunctionSpeed;      ///< Max. junction speed between this and next segment
    float startSpeed;            ///< Starting speed in mm/s
    float endSpeed;              ///< Exit speed in mm/s
    float minSpeed;
    float distance;
    uint32_t accelSteps;       ///< How much steps does it take, to reach the plateau.
    uint32_t decelSteps;       ///< How much steps does it take, to reach the end speed.
    uint32_t accelerationPrim; ///< Acceleration along primary axis
    uint32_t fAcceleration;    ///< accelerationPrim*262144/F_CPU
    speed_t vMax;              ///< Maximum reached speed in steps/s.
    speed_t vStart;            ///< Starting speed in steps/s.
    speed_t vEnd;              ///< End speed in steps/s

#if USE_ADVANCE
    uint32_t advanceL; ///< Recomputated L value
#endif                 // USE_ADVANCE

public:
    int32_t stepsRemaining; ///< Remaining steps, until move is finished
    static PrintLine* cur;
    char task;
    static PrintLine direct;

    static volatile uint8_t linesCount; // Number of lines cached 0 = nothing to do

    inline bool areParameterUpToDate() {
        return joinFlags & FLAG_JOIN_STEPPARAMS_COMPUTED;
    } // areParameterUpToDate

    inline void invalidateParameter() {
        joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED;
    } // invalidateParameter

    inline void setParameterUpToDate() {
        joinFlags |= FLAG_JOIN_STEPPARAMS_COMPUTED;
    } // setParameterUpToDate

    inline bool isStartSpeedFixed() {
        return joinFlags & FLAG_JOIN_START_FIXED;
    } // isStartSpeedFixed

    inline void setStartSpeedFixed(bool newState) {
        joinFlags = (newState ? joinFlags | FLAG_JOIN_START_FIXED : joinFlags & ~FLAG_JOIN_START_FIXED);
    } // setStartSpeedFixed

    inline void fixStartAndEndSpeed() {
        joinFlags |= FLAG_JOIN_END_FIXED | FLAG_JOIN_START_FIXED;
    } // fixStartAndEndSpeed

    inline bool isEndSpeedFixed() {
        return joinFlags & FLAG_JOIN_END_FIXED;
    } // isEndSpeedFixed

    inline void setEndSpeedFixed(bool newState) {
        joinFlags = (newState ? joinFlags | FLAG_JOIN_END_FIXED : joinFlags & ~FLAG_JOIN_END_FIXED);
    } // setEndSpeedFixed

    inline bool isWarmUp() {
        return flags & FLAG_WARMUP;
    } // isWarmUp

    inline uint8_t getWaitForXLinesFilled() {
        // This function seems to produce a pausing if X_AXIS is the primary axis
        // or if y is the primary axis and only one line in queue
        // or if z axis is the primary and only two lines in queue
        // or if e axis is the primary and only three lines in queue
        //
        // ?????? :D But still original Repetier
        return primaryAxis;
    } // getWaitForXLinesFilled

    inline void setWaitForXLinesFilled(uint8_t b) {
        primaryAxis = b;
    } // setWaitForXLinesFilled

    inline bool isExtruderForwardMove() {
        return (dir & 136) == 136; //isEPositiveMove()
    }                              // isExtruderForwardMove

    inline void block() {
        flags |= FLAG_BLOCKED;
    } // block

    inline void unblock() {
        flags &= ~FLAG_BLOCKED;
    } // unblock

    inline bool isBlocked() {
        return flags & FLAG_BLOCKED;
    } // isBlocked

    // If we reach endstops or softendstops we have two options:
    // 1) abort the move and place coordinate to "is"
    // 2) stop the physical move but let "the virtual" coordinates flow. Hereby we can revert the coordinates by moving the other way.
    inline bool isAbortAtEndstops() {
        return flags & FLAG_ABORT_AT_ENDSTOPS;
    } // isAbortAtEndstops

    inline bool isNominalMove() {
        return flags & FLAG_NOMINAL;
    } // isNominalMove

    inline void setNominalMove() {
        flags |= FLAG_NOMINAL;
    } // setNominalMove

    inline void setXMoveFinished() {
        dir &= ~16;
    } // setXMoveFinished

    inline void setYMoveFinished() {
        dir &= ~32;
    } // setYMoveFinished

    inline void setZMoveFinished() {
        dir &= ~64;
    } // setZMoveFinished

    inline void setEMoveFinished() {
        dir &= ~128;
        Extruder::current->stepperDirection = 0;
    } // setEMoveFinished

    inline void setXYMoveFinished() {
        dir &= ~48;
    } // setXYMoveFinished

    inline void setXYZEMoveFinished() {
        dir &= ~240;
        Extruder::current->stepperDirection = 0;
    } // setXYMoveFinished

    inline bool isXPositiveMove() {
        return (dir & 17) == 17;
    } // isXPositiveMove

    inline bool isXNegativeMove() {
        return (dir & 17) == 16;
    } // isXNegativeMove

    inline bool isYPositiveMove() {
        return (dir & 34) == 34;
    } // isYPositiveMove

    inline bool isYNegativeMove() {
        return (dir & 34) == 32;
    } // isYNegativeMove

    inline bool isZPositiveMove() {
        return (dir & 68) == 68;
    } // isZPositiveMove

    inline bool isZNegativeMove() {
        return (dir & 68) == 64;
    } // isZNegativeMove

    inline bool isEPositiveMove() {
        return (dir & 136) == 136;
    } // isEPositiveMove

    inline bool isENegativeMove() {
        return (dir & 136) == 128;
    } // isENegativeMove

    inline bool isXMove() {
        return (dir & 16);
    } // isXMove

    inline bool isYMove() {
        return (dir & 32);
    } // isYMove

    inline bool isXOrYMove() {
        return dir & 48;
    } // isXOrYMove

    inline bool isZMove() {
        return (dir & 64);
    } // isZMove

    inline bool isEMove() {
        return (dir & 128);
    } // isEMove

    inline bool isEOnlyMove() {
        return (dir & 240) == 128;
    } // isEOnlyMove

    inline bool isNoMove() {
        return (dir & 240) == 0;
    } // isNoMove

    inline bool isXYZMove() {
        return dir & 112;
    } // isXYZMove

    inline bool isMoveOfAxis(uint8_t axis) {
        return (dir & (16 << axis));
    } // isMoveOfAxis

    inline void setMoveOfAxisFinished(uint8_t axis) {
        dir &= ~(16 << axis);
    } // setMoveOfAxisFinished

    inline bool isPositiveMoveOfAxis(uint8_t axis) {
        uint8_t mask = (17 << axis);

        return (dir & mask) == mask;
    } // isPositiveMoveOfAxis

    inline bool isNegativeMoveOfAxis(uint8_t axis) {
        return (dir & (17 << axis)) == (16 << axis);
    } // isPositiveMoveOfAxis

    inline void setMoveOfAxis(uint8_t axis) {
        dir |= 16 << axis;
    } // setMoveOfAxis

    inline void setPositiveDirectionForAxis(uint8_t axis) {
        dir |= 1 << axis;
    } // setPositiveDirectionForAxis

    inline static void resetPathPlanner() {
        linesCount = 0;
        linesPos = linesWritePos;
    } // resetPathPlanner

#if USE_ADVANCE
    inline void updateAdvanceSteps(speed_t v) {
        if (!Printer::isAdvanceActivated())
            return;

        int32_t tred = (v * advanceL) >> 16;
        HAL::forbidInterrupts();
        Printer::extruderStepsNeeded += tred - Printer::advanceStepsSet;
        Printer::advanceStepsSet = tred;
        HAL::allowInterrupts();
    }  // updateAdvanceSteps
#endif // USE_ADVANCE

    INLINE bool moveDecelerating(uint8_t forQueue) {
        if (stepsRemaining <= static_cast<int32_t>(decelSteps)) {
            if (!(flags & FLAG_DECELERATING)) //reset "timer" only once.
            {
                Printer::timer[forQueue] = 0;
                flags |= FLAG_DECELERATING;
            }
            return true;
        }

        return false;
    } // moveDecelerating

    INLINE bool moveAccelerating(uint8_t forQueue) {
        return Printer::stepNumber[forQueue] <= accelSteps;
    } // moveAccelerating

    void updateStepsParameter();
    inline float safeSpeed(fast8_t drivingAxis);
    void calculateMove(float axis_diff[], fast8_t drivingAxis, float feedrate);

    INLINE long getWaitTicks() {
        return timeInTicks;
    } // getWaitTicks

    INLINE void setWaitTicks(long wait) {
        timeInTicks = wait;
    } // setWaitTicks

    static INLINE bool hasLines() {
        return linesCount;
    } // hasLines

    static uint8_t getLinesCount() {
        InterruptProtectedBlock noInts;
        return linesCount;
    }

    static INLINE void setCurrentLine() {
        cur = &lines[linesPos];
    } // setCurrentLine

    static INLINE void removeCurrentLineForbidInterrupt() {
        HAL::forbidInterrupts();
        nextPlannerIndex(linesPos);
        cur->task = TASK_NO_TASK;
        cur = NULL;
        --linesCount;
    } // removeCurrentLineForbidInterrupt

    static INLINE void pushLine() {
        nextPlannerIndex(linesWritePos);
        InterruptProtectedBlock noInts;
        linesCount++;
        g_uStartOfIdle = 0; //tell the printer work is being done.
    }                       // pushLine

    static PrintLine* getNextWriteLine() {
        return &lines[linesWritePos];
    } // getNextWriteLine

    static inline void computeMaxJunctionSpeed(PrintLine* previous, PrintLine* current);

    static long needCmpWait();
    static void stepSlowedZCompensation();
    static long performQueueMove();
    static long performDirectMove();

    static long performMove(PrintLine* move, uint8_t forQueue);
    static void waitForXFreeLines(uint8_t b = 1);
    static inline void forwardPlanner(uint8_t p);
    static inline void backwardPlanner(uint8_t p, uint8_t last);
    static void updateTrapezoids();
    static uint8_t insertWaitMovesIfNeeded(uint8_t pathOptimize, uint8_t waitExtraLines);
    static void prepareQueueMove(uint8_t abortAtEndstops, uint8_t pathOptimize, float feedrate);
    static void prepareDirectMove(bool stoppable, bool feedrateSource);
    static void stopDirectMove(void);
#if FEATURE_ARC_SUPPORT
    static void arc(float* position, float* target, float* offset, float radius, uint8_t isclockwise);
#endif // FEATURE_ARC_SUPPORT

    static INLINE void previousPlannerIndex(uint8_t& p) {
        p = (p ? p - 1 : MOVE_CACHE_SIZE - 1);
    } // previousPlannerIndex

    static INLINE void nextPlannerIndex(uint8_t& p) {
        p = (p >= MOVE_CACHE_SIZE - 1 ? 0 : p + 1);
    } // nextPlannerIndex

    static inline void queueTask(char task) {
        PrintLine* p;

        p = getNextWriteLine();
        p->task = task;

        nextPlannerIndex(linesWritePos);
        InterruptProtectedBlock noInts; //BEGIN_INTERRUPT_PROTECTED
        linesCount++;
        //END_INTERRUPT_PROTECTED
        return;
    } // queueTask

    inline void enableSteppers(void) {
        if (Printer::blockAll) {
            // do not enable anything in case everything is blocked
            return;
        }

        // Only enable axis that are moving. If the axis doesn't need to move then it can stay disabled depending on configuration.
        if (isXMove()) {
            Printer::enableXStepper();
        }
        if (isYMove()) {
            Printer::enableYStepper();
        }
        if (isZMove()) {
            Printer::enableZStepper();
        }
        if (isEMove()) {
            Printer::unmarkAllSteppersDisabled(); //doesnt fit into Extruder::enable() because of forward declare -> TODO
            Extruder::enable();
        }
    } // enableSteppers

    inline void adjustDirections(void) {
        if (Printer::blockAll) {
            // do not enable anything in case everything is blocked
            return;
        }

        // Only enable axis that are moving. If the axis doesn't need to move then it can stay disabled depending on configuration.
        if (isXMove()) {
            Printer::setXDirection(isXPositiveMove());
        }
        if (isYMove()) {
            Printer::setYDirection(isYPositiveMove());
        }
        if (isZMove()) {
            Printer::setZDirection(isZPositiveMove());
        }
        if (isEMove()) {
#if USE_ADVANCE
            if (!Printer::isAdvanceActivated()) // Set direction if no advance/OPS enabled
#endif
                Extruder::setDirection(isEPositiveMove());
        }
    } // adjustDirections
};

#endif // MOTION_H

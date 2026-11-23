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
#define FLAG_ABORT_AT_ENDSTOPS 16
#define FLAG_BLOCKED 128

#define FLAG_JOIN_STEP_PROFILE_READY 1  // The foreground has prepared the step ramp.
#define FLAG_JOIN_END_FIXED 2            // The right speed is fixed. Don't check this block or any block to the left.
#define FLAG_JOIN_START_FIXED 4          // The left speed is fixed. Don't check left block.

enum DirectMovePhase : char {
    DIRECT_IDLE,
    DIRECT_PREPARING,
    DIRECT_PREPARING_STOPPABLE,
    DIRECT_PREPARING_STOPPED,
    DIRECT_PREPARED,
    DIRECT_PREPARED_STOPPABLE,
    DIRECT_RUNNING,
    DIRECT_RUNNING_STOPPABLE,
};

#define FOR_DIRECT 0
#define FOR_QUEUE 1

class MoveQueue;
class PrintLine {
    friend class MoveQueue;
    friend class MotionPlanner;
    friend class StepExecutor;

private:
    flag8_t joinFlags;
    volatile flag8_t flags;

    static MoveQueue queue;
    static PrintLine direct;
    inline bool isStepProfileReady() const {
        return joinFlags & FLAG_JOIN_STEP_PROFILE_READY;
    } // isStepProfileReady

    inline void invalidateStepProfile() {
        joinFlags &= ~FLAG_JOIN_STEP_PROFILE_READY;
    } // invalidateStepProfile

    inline void markStepProfileReady() {
        joinFlags |= FLAG_JOIN_STEP_PROFILE_READY;
    } // markStepProfileReady

    void fitRampToRemainingSteps();
    void limitToStopTail(bool refitRamp);

    static bool directMovePreparing(char phase) {
        return phase >= DIRECT_PREPARING && phase <= DIRECT_PREPARING_STOPPED;
    }

    flag8_t primaryAxis;
    int32_t timeInTicks;
    flag8_t dir;                 ///< Direction of movement. 1 = X+, 2 = Y+, 4= Z+, values can be combined.
    int32_t delta[4];            ///< Steps we want to move.
    int32_t error[4];            ///< Error calculation for Bresenham algorithm
    float fullSpeed; ///< Segment feed in mm/s, also published to the status display.

    // Lookahead revises these values while the slot is owned by the planner.
    struct LookaheadData {
        float speedX;
        float speedY;
        float speedZ;
        float speedE;
        float invFullSpeed;
        float accelerationDistance2;
        float maxJunctionSpeed;
        float startSpeed;
        float endSpeed;
        float minSpeed;
        float distance;
        uint32_t accelerationPrim;
    };

    // Timer1 consumes this profile in motor steps and timer ticks.
    struct StepProfile {
        uint32_t accelSteps;
        uint32_t decelSteps;
        uint32_t fAcceleration; ///< Scaled acceleration for HAL::ComputeV.
        speed_t vMax;
        speed_t vStart;
        speed_t vEnd;
        uint32_t advanceL;
    };

    LookaheadData lookahead;
    StepProfile stepProfile;

    void setAxisSteps(uint8_t axis, int32_t steps);
    void setSegmentGeometry(const float axisDistanceMM[]);

    int32_t stepsRemaining; ///< Remaining steps, until move is finished
    volatile char task;

public:
    static bool directMoveActive() { return direct.task != DIRECT_IDLE; }
    static bool hasStoppableDirectMove() {
        const char task = direct.task;
        return task == DIRECT_PREPARING_STOPPABLE || task == DIRECT_PREPARED_STOPPABLE || task == DIRECT_RUNNING_STOPPABLE;
    }

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



    INLINE long getWaitTicks() {
        return timeInTicks;
    } // getWaitTicks

    INLINE void setWaitTicks(long wait) {
        timeInTicks = wait;
    } // setWaitTicks

    static uint8_t queuedMoveCount();
    static INLINE bool hasLines() { return queuedMoveCount() != 0; }
    static bool hasActiveQueuedMove();
    static bool hasActiveZMove();
    static void discardQueuedMoves();


    static void waitForXFreeLines(uint8_t b = 1);
    static uint8_t insertWaitMovesIfNeeded(uint8_t pathOptimize, uint8_t waitExtraLines);
    static void prepareQueueMove(uint8_t abortAtEndstops, uint8_t pathOptimize, float feedrate, bool updateWobble = false);
    // Reserve the slot while the caller publishes the offset targets under its timer lock.
    static void reserveDirectMove(bool stoppable) {
        direct.task = stoppable ? DIRECT_PREPARING_STOPPABLE : DIRECT_PREPARING;
    }
    static void prepareDirectMove(const int32_t steps[4], bool stoppable, bool feedrateSource);
    static void stopDirectMove();

    static void queueTask(char task);


};

// The foreground fills and publishes slots. The step timer activates and retires them.
class MoveQueue {
public:
    uint8_t count() const { return count_; }
    uint8_t readIndex() const { return readIndex_; }
    uint8_t writeIndex() const { return writeIndex_; }
    PrintLine* active() const { return active_; }
    PrintLine& slot(uint8_t index) { return slots_[index]; }
    PrintLine& writeSlot() { return slot(writeIndex_); }

    void publish(bool startsMotion) {
        nextIndex(writeIndex_);
        InterruptProtectedBlock lock;
        ++count_;
        if (startsMotion)
            Printer::idleStatus.cancel();
    }

    PrintLine* activateFront() {
        active_ = &slot(readIndex_);
        return active_;
    }

    void releaseFront() { active_ = nullptr; }

    void retireFront() {
        HAL::forbidInterrupts();
        nextIndex(readIndex_);
        active_->task = TASK_NO_TASK;
        active_ = nullptr;
        --count_;
    }

    void clear() {
        InterruptProtectedBlock lock;
        count_ = 0;
        readIndex_ = writeIndex_;
        active_ = nullptr;
    }

    static void nextIndex(uint8_t& index) {
        index = index >= MOVE_CACHE_SIZE - 1 ? 0 : index + 1;
    }

    static void previousIndex(uint8_t& index) {
        index = index ? index - 1 : MOVE_CACHE_SIZE - 1;
    }

private:
    PrintLine slots_[MOVE_CACHE_SIZE];
    PrintLine* active_ = nullptr;
    uint8_t readIndex_ = 0;
    uint8_t writeIndex_ = 0;
    volatile uint8_t count_ = 0;
};

inline uint8_t PrintLine::queuedMoveCount() { return queue.count(); }
inline bool PrintLine::hasActiveQueuedMove() { return queue.active() != nullptr; }
inline void PrintLine::discardQueuedMoves() { queue.clear(); }

inline void PrintLine::queueTask(char task) {
    queue.writeSlot().task = task;
    queue.publish(false);
}

#endif // MOTION_H

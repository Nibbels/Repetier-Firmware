#ifndef MOVE_QUEUE_H
#define MOVE_QUEUE_H

#include "Printer.h"

#include "motion.h"

// The foreground fills and publishes slots. The step timer activates and retires them.
class MoveQueue {
public:
    struct RemainingMotion {
        float xyzMm;
        float filamentMm;
        float seconds;
    };

    // Foreground only. The result includes the unfinished part of the front move.
    RemainingMotion remainingMotion() const;

    uint8_t count() const { return count_; }
    uint8_t readIndex() const { return readIndex_; }
    uint8_t writeIndex() const { return writeIndex_; }
    PrintLine* active() { return active_; }
    PrintLine& slot(uint8_t index) { return slots_[index]; }
    PrintLine& writeSlot() { return slot(writeIndex_); }

    void publish(bool startsMotion) {
        nextIndex(writeIndex_);
        InterruptProtectedBlock lock;
        ++count_;
        if (startsMotion)
            Printer::idleStatus.cancel();
    }

    void publishTask(QueueTask task) {
        PrintLine& move = writeSlot();
        move.flags |= FLAG_QUEUE_TASK;
        move.setQueueTaskId(task);
        publish(false);
    }

    PrintLine* activateFront() {
        active_ = &slot(readIndex_);
        return active_;
    }

    void releaseFront() { active_ = nullptr; }

    void retireFront() {
        HAL::forbidInterrupts();
        nextIndex(readIndex_);
        active_->setQueueTaskId(QueueTask::None);
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
    static float estimateSeconds(const PrintLine& move, uint32_t totalSteps, uint32_t remainingSteps);

    PrintLine slots_[MOVE_CACHE_SIZE];
    PrintLine* active_ = nullptr;
    uint8_t readIndex_ = 0;
    uint8_t writeIndex_ = 0;
    volatile uint8_t count_ = 0;
};

inline const MoveQueue& PrintLine::queuedMoves() { return queue; }
inline uint8_t PrintLine::queuedMoveCount() { return queue.count(); }
inline bool PrintLine::hasActiveQueuedMove() { return queue.active() != nullptr; }
inline void PrintLine::discardQueuedMoves() { queue.clear(); }

inline void PrintLine::queueTask(QueueTask task) { queue.publishTask(task); }

#endif

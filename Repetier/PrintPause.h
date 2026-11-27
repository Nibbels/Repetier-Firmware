#ifndef PRINT_PAUSE_H
#define PRINT_PAUSE_H

#include <stdint.h>

class PrintPause {
public:
    enum class Destination : uint8_t { InPlace, Park };

    // The step timer reads one byte to decide whether the print queue may run.
    bool allowsQueueAdvance() const { return phase_ == Phase::Running; }
    bool isPaused() const { return !allowsQueueAdvance(); }
    bool isSettled() const {
        const Phase phase = phase_;
        return phase == Phase::Paused || phase == Phase::Standby;
    }
    bool isWarmPause() const { return phase_ == Phase::Paused; }
    bool isReheating() const { return phase_ == Phase::Reheating; }

    void request(Destination destination);
    void queueRequest(Destination destination);
    void pressPause();
    bool resume();
    void cancel();
    void serviceMotion();
    void serviceStandby(uint32_t now);

    bool allowsManualMove() const;
    int32_t limitZJog(int32_t steps) const;
    void extrudeManually(int32_t steps);

    // Track accepted direct E steps, then remove any canceled before output.
    void recordDirectExtrusion(int8_t direction) {
        if (tracksDirectExtrusion())
            extruderReturnSteps_ -= direction;
    }

    void discardPendingExtrusion(int16_t steps) {
        if (steps && tracksDirectExtrusion())
            extruderReturnSteps_ += steps;
    }

    void setParkSteps(uint8_t axis, int32_t steps) { parkSteps_[axis] = steps; }
    void scaleParkSteps(uint8_t axis, float ratio) { parkSteps_[axis] *= ratio; }
    void restoreDefaultParking();

    struct SenseOffsetSample {
        bool wasPaused;
        bool resetWindow;
    };
    SenseOffsetSample beginSenseOffsetSample();
    bool allowsSenseOffsetLearning() const {
        return allowsQueueAdvance() && !pauseSinceSenseSample_;
    }

private:
    enum class Phase : uint8_t {
        Running,
        RetractRequested,
        RetractAndParkRequested,
        SettlingExtrusion,
        ParkRequested,
        Parking,
        Paused,
        Standby,
        Reheating,
        ReturnRequested,
        Retracting,
        Returning,
        ManualExtrusion
    };

    bool tracksDirectExtrusion() const {
        const Phase phase = phase_;
        return phase == Phase::Retracting || phase == Phase::Returning || phase == Phase::ManualExtrusion;
    }

    void dispatchPendingMotion();
    void waitUntilSettled();
    bool acceptsPauseRequest() const;
    void retract(Destination destination);
    void park();
    void returnToPrint();
    bool moveBackToPrintPosition();
    bool restoreExtruderTemperatures();
    void settle(uint32_t now, uint32_t extraWarmTime = 0);
    bool finishExtrusion(Phase phase);
    bool moveWhile(Phase phase, int32_t x, int32_t y, int32_t z, int32_t e);
    int32_t stepsToReturn(uint8_t axis) const;

    volatile Phase phase_ = Phase::Running;
    volatile bool pauseSinceSenseSample_ = false;
    bool parked_ = false;
    bool servicingMotion_ = false; // foreground owner; cancel does not release
    uint32_t standbyAt_ = 0;
    int32_t parkSteps_[4] = {};
    int32_t returnPosition_[3] = {};
    volatile int32_t extruderReturnSteps_ = 0;
};

#endif // PRINT_PAUSE_H

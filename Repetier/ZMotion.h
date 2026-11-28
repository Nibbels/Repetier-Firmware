#ifndef Z_MOTION_H
#define Z_MOTION_H

#include "DirectPosition.h"

// Execution and travel limits for the Z motor. The planner and bed/DMS
// calculations supply movement and correction requests in integer motor steps.
class ZMotion {
public:
    enum class Path : uint8_t { Queued, QueuedUntilEndstop, Direct, StoppableDirect };

    constexpr explicit ZMotion(float stepsPerMm)
        : minOvertravel_(uint16_t(stepsPerMm * Z_ENDSTOP_DRIVE_OVER)),
          queueWaitSteps_(uint16_t(stepsPerMm) >> 2) {}
    ZMotion(const ZMotion&) = delete;
    ZMotion& operator=(const ZMotion&) = delete;

    int32_t motorSteps() const {
        InterruptProtectedBlock lock;
        return motorSteps_;
    }
    int32_t nominalSteps() const {
        InterruptProtectedBlock lock;
        return nominalSteps_;
    }
    int32_t motorOffsetSteps() const {
        InterruptProtectedBlock lock;
        const int32_t motor = motorSteps_;
        const int32_t nominal = nominalSteps_;
        lock.unprotect();
        return motor - nominal;
    }
    int32_t scanSteps() const { return scanSteps_; }
    INLINE bool hasQueuedOvertravel() const { return queuedOvertravel_ != 0; }
    bool isHomed() const { return homed_; }
    int32_t maxSteps() const { return maxSteps_; }
    uint16_t minOvertravel() const { return minOvertravel_; }
    const DirectPosition& directPosition() const { return direct_; }

    // Reference changes are paired with planner state under the caller's lock.
    void setNominalSteps(int32_t steps) { nominalSteps_ = steps; }
    void setMotorReference(int32_t steps) { motorSteps_ = steps; }
    void rescaleMotorReference(float ratio) { motorSteps_ *= ratio; }
    void configureTravel(float stepsPerMm, float axisLengthMm);
    void setHomed(bool homed) {
        InterruptProtectedBlock lock;
        homed_ = homed;
        if (!homed)
            queuedOvertravel_ = 0;
    }

    void requestDirectMove(int32_t steps) { direct_.request(steps); }
    void stopDirectMove() { direct_.stop(); }
    void resetDirectPosition() { direct_.reset(); }
    void clearScanAndCorrection() {
        scanSteps_ = 0;
        appliedCorrection_ = correctionTarget_ = 0;
    }

    // The caller publishes this under the same lock as the extrusion correction.
    void requestCorrection(int32_t steps) { correctionTarget_ = steps; }
    int32_t correctionTarget() const { return correctionTarget_; }
    // Direct foreground diagnostics require no deferred correction, even
    // one currently clipped by a travel limit (it could become movable).
    bool correctionCleared() const {
        InterruptProtectedBlock lock;
        return correctionTarget_ == 0 && appliedCorrection_ == 0;
    }

    static INLINE bool minSwitchHit() { return READ(Z_MIN_PIN) == LOW; }
    bool maxEndstopHit() const {
#if MOTHERBOARD == DEVICE_TYPE_RF1000
        InterruptProtectedBlock lock;
#endif
        return atUpperEndstop();
    }
    bool manualStepExceedsMinimum(int32_t steps) const {
        return steps < 0 && minSwitchHit() && motorSteps() + steps < -int32_t(minOvertravel_);
    }

    static INLINE bool directionIsPositive() { return (READ(Z_DIR_PIN) != 0) ^ INVERT_Z_DIR; }
    static INLINE void setDirection(bool positive) {
        WRITE(Z_DIR_PIN, positive != INVERT_Z_DIR);
    }
    static INLINE void finishPulse() { WRITE(Z_STEP_PIN, LOW); }

    // Timer1 owns path steps. A false result terminates this axis of the segment.
    bool stepPath(Path path);
    int32_t stepCorrection();
    bool correctionCanMove() const;
    bool correctionFollowsDirection() const;
    bool correctionDelaysQueue(bool compensationEnabled) const;

    bool scanInProgress() const { return scanInProgress_; }

    class ScanMove {
    public:
        ScanMove(ZMotion& motion, int8_t direction) : motion_(motion), direction_(direction) {
            motion_.scanInProgress_ = true;
        }
        ~ScanMove() { motion_.scanInProgress_ = false; }
        ScanMove(const ScanMove&) = delete;
        ScanMove& operator=(const ScanMove&) = delete;

        bool step();

    private:
        ZMotion& motion_;
        const int8_t direction_;
    };

    void stepHomeCheck(uint8_t delayMultiplier);

private:
    enum class Step : uint8_t { Pulse, Suppressed, Stop };

    INLINE bool atUpperEndstop() const {
#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
        return READ(Z_MAX_PIN) == LOW;
#else
        return atSoftwareMaximum();
#endif
    }
    bool atSoftwareMaximum() const { return motorSteps_ >= maxSteps_; }
    bool atMinOvertravel() const { return motorSteps_ <= -int32_t(minOvertravel_); }
    bool correctionLimited(bool positive) const;
    bool balanceOpposingCorrection(int8_t direction);
    Step checkQueuedStep(int8_t direction, bool abortAtEndstop);
    Step checkDirectStep(int8_t direction, bool stopAtMinSwitch);
    Step recoverQueuedOvertravel();

    int32_t pendingCorrection() const { return correctionTarget_ - appliedCorrection_; }
    INLINE void startPulse(int8_t direction) {
        WRITE(Z_STEP_PIN, HIGH);
        motorSteps_ += direction;
    }
    int8_t timedPulse(uint8_t delayMultiplier);

    volatile int32_t motorSteps_ = 0;
    volatile int32_t nominalSteps_ = 0;
    volatile int32_t queuedOvertravel_ = 0;
    volatile int32_t correctionTarget_ = 0;
    volatile int32_t appliedCorrection_ = 0;
    DirectPosition direct_;
    int32_t scanSteps_ = 0;
    volatile int32_t maxSteps_ = 0;
    volatile uint16_t minOvertravel_;
    volatile uint16_t queueWaitSteps_;
    volatile bool scanInProgress_ = false;
    volatile bool homed_ = false;
};

#endif // Z_MOTION_H

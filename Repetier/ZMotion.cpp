#include "Repetier.h"

void ZMotion::configureTravel(float stepsPerMm, float axisLengthMm) {
    const int32_t maximum = int32_t(stepsPerMm * axisLengthMm);
    const uint16_t overtravel = uint16_t(stepsPerMm * Z_ENDSTOP_DRIVE_OVER);
    const uint16_t waitSteps = uint16_t(stepsPerMm) >> 2;

    InterruptProtectedBlock lock;
    maxSteps_ = maximum;
    minOvertravel_ = overtravel;
    queueWaitSteps_ = waitSteps;
}

bool ZMotion::correctionLimited(bool positive) const {
    if (!positive)
        return isHomed() ? atMinOvertravel() : minSwitchHit();

    if (atSoftwareMaximum())
        return true;
#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    return atUpperEndstop();
#else
    return false;
#endif
}

bool ZMotion::correctionCanMove() const {
    const int32_t pending = pendingCorrection();
    return pending && !correctionLimited(pending > 0);
}

bool ZMotion::correctionFollowsDirection() const {
    const int32_t pending = pendingCorrection();
    if (!pending || (pending > 0) != directionIsPositive())
        return false;
    return !correctionLimited(pending > 0);
}

bool ZMotion::correctionDelaysQueue(bool compensationEnabled) const {
    const int32_t target = correctionTarget_;
    const int32_t pending = target - appliedCorrection_;
    if (!pending)
        return false;

    // Catch up when more than 0.25 mm behind. During switch-off, wait for the
    // remaining correction when the motor already points in that direction.
    if (!target || abs(pending) <= queueWaitSteps_) {
        if (compensationEnabled || (pending > 0) != directionIsPositive())
            return false;
    }
    return !correctionLimited(pending > 0);
}

int32_t ZMotion::stepCorrection() {
    const int32_t pending = pendingCorrection();
    if (!pending || correctionLimited(pending > 0))
        return 0;

    const bool positive = pending > 0;
    if (directionIsPositive() != positive)
        setDirection(positive);
    const int8_t direction = positive ? 1 : -1;
    startPulse(direction);
    appliedCorrection_ += direction;
    return pending;
}

bool ZMotion::balanceOpposingCorrection(int8_t direction) {
    const int32_t pending = pendingCorrection();
    if (!pending || (pending > 0) == (direction > 0))
        return false;
    // Keep the retreat step available when the pending correction is blocked.
    if (correctionLimited(pending > 0))
        return false;

    appliedCorrection_ -= direction;
    return true;
}

ZMotion::Step ZMotion::recoverQueuedOvertravel() {
    if (!queuedOvertravel_)
        return Step::Pulse;

    --queuedOvertravel_;
    return Step::Suppressed;
}

ZMotion::Step ZMotion::checkQueuedStep(int8_t direction, bool abortAtEndstop) {
    if (atUpperEndstop()) {
        if (direction < 0) {
            if (minSwitchHit()) {
                queuedOvertravel_ = 0;
                return Step::Stop;
            }
            return recoverQueuedOvertravel();
        }
        if (abortAtEndstop) {
            queuedOvertravel_ = 0;
            return Step::Stop;
        }
        ++queuedOvertravel_;
        return Step::Suppressed;
    }

    if (!minSwitchHit()) {
        // Also consume previously suppressed steps if the upper contact reopens.
        return recoverQueuedOvertravel();
    }

    if (direction < 0 && (!isHomed() || atMinOvertravel())) {
        queuedOvertravel_ = 0;
        return Step::Stop;
    }
    return Step::Pulse;
}

ZMotion::Step ZMotion::checkDirectStep(int8_t direction, bool stopAtMinSwitch) {
    if (atUpperEndstop()) {
        if (direction < 0) {
            if (minSwitchHit())
                return Step::Stop;
            return Step::Pulse;
        }
        direct_.stop();
        return Step::Stop;
    }
    if (direction > 0 || !minSwitchHit())
        return Step::Pulse;
    if (stopAtMinSwitch || !isHomed() || atMinOvertravel())
        return Step::Stop;
    return Step::Pulse;
}

// With AVR LTO, keep the hot Z path in the specialized step executor.
#if defined(__AVR__)
__attribute__((always_inline))
#endif
bool ZMotion::stepPath(Path path) {
    const int8_t direction = directionIsPositive() ? 1 : -1;
    if (path == Path::Scan) {
        // Scan travel uses the same physical pulse/position owner, but
        // must never balance compensation or consume direct/nominal debt.
        if (direction > 0 ? atUpperEndstop() : (!isHomed() || atMinOvertravel()))
            return false;
        HAL::allowInterrupts();
        startPulse(direction);
        HAL::forbidInterrupts();
        scanSteps_ += direction;
        return true;
    }
    const bool queued = path == Path::Queued || path == Path::QueuedUntilEndstop;
    Step result = Step::Suppressed;
    if (!balanceOpposingCorrection(direction)) {
        // Timer1 is disabled for this ISR. Other timers may run during the
        // endstop check and pulse start, as they do for X and Y.
        HAL::allowInterrupts();
        result = queued ? checkQueuedStep(direction, path == Path::QueuedUntilEndstop)
                        : checkDirectStep(direction, path == Path::StoppableDirect);
        if (result == Step::Pulse)
            startPulse(direction);
        HAL::forbidInterrupts();
    }

    if (queued) {
        // A clipped queue step still advances the nominal path.
        nominalSteps_ += direction;
    } else if (result != Step::Stop) {
        direct_.advance(direction);
    }
    return result != Step::Stop;
}


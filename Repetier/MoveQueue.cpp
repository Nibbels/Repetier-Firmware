#include "Repetier.h"
#include "StepExecutor.h"

MoveQueue PrintLine::queue;

MoveQueue::RemainingMotion MoveQueue::remainingMotion() const {
    uint8_t index, entries;
    int32_t frontSteps;
    {
        InterruptProtectedBlock lock;
        index = readIndex_;
        entries = count_;
        frontSteps = active_ ? active_->stepsRemaining : -1;
    }

    RemainingMotion result{};
    // Timer1 may retire these slots after the snapshot. Only the foreground
    // can reuse them. Geometry and ramp parameters are stable during this call.
    for (uint8_t offset = 0; offset < entries; ++offset, nextIndex(index)) {
        const PrintLine& move = slots_[index];
        if (move.flags & (FLAG_WARMUP | FLAG_QUEUE_TASK))
            continue;
        const int32_t total = move.delta[move.primaryAxis];
        const int32_t left = offset == 0 && frontSteps >= 0 ? frontSteps : total;
        if (left <= 0 || total <= 0)
            continue;

        const auto& lookahead = move.lookahead;
        const float fraction = float(left) / total;
        const float scale = lookahead.distance * lookahead.invFullSpeed * fraction;
        result.xyzMm += sqrtf(RMath::sqr(lookahead.speedX) + RMath::sqr(lookahead.speedY)
                              + RMath::sqr(lookahead.speedZ)) * scale;
        result.filamentMm += fabsf(lookahead.speedE) * scale;
        result.seconds += estimateSeconds(move, total, left);
    }
    return result;
}

float MoveQueue::estimateSeconds(const PrintLine& move, uint32_t total, uint32_t left) {
    const auto& ramp = move.stepProfile;
    const uint32_t acceleration = ramp.accelSteps < total ? ramp.accelSteps : total;
    const uint32_t unaccelerated = total - acceleration;
    const uint32_t deceleration = ramp.decelSteps < unaccelerated ? ramp.decelSteps : unaccelerated;

    const uint32_t completed = total - left;
    const uint32_t accelerating = completed < acceleration ? acceleration - completed : 0;
    const uint32_t braking = left < deceleration ? left : deceleration;
    const uint32_t cruising = left - accelerating - braking;

    const float twiceAcceleration = 2.0f * move.lookahead.accelerationPrim;
    const float startSquared = RMath::sqr(float(ramp.vStart));
    float peak = RMath::min(float(ramp.vMax), float(F_CPU / StepExecutor::minimumStepIntervalTicks));
    if (acceleration)
        peak = RMath::min(peak, sqrtf(startSquared + twiceAcceleration * acceleration));
    peak = RMath::max(1.0f, peak);

    // Each ramp uses its average step rate under constant acceleration.
    float seconds = cruising / peak;
    if (accelerating) {
        const float from = RMath::min(peak, sqrtf(startSquared + twiceAcceleration * completed));
        seconds += 2.0f * accelerating / (from + peak);
    }
    if (braking) {
        const float end = RMath::min(float(ramp.vEnd), peak);
        const float endSquared = RMath::sqr(end);
        const float from = braking == deceleration ? peak :
            sqrtf(endSquared + (RMath::sqr(peak) - endSquared) * (float(braking) / deceleration));
        seconds += 2.0f * braking / (from + end);
    }
    return seconds;
}

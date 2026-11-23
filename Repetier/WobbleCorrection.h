#ifndef WOBBLE_CORRECTION_H
#define WOBBLE_CORRECTION_H

#include <math.h>
#include <stdint.h>

class WobbleCorrection {
public:
    struct Profile {
        int8_t phase; // +100 = +pi, -100 = -pi.
        int16_t xMicrometers;
        int16_t yAtXMinMicrometers;
        int16_t yAtXMaxMicrometers;
    };

    const Profile& profile() const { return profile_; }
    void setProfile(Profile profile) { profile_ = profile; }
    void setPhase(int8_t phase) { profile_.phase = phase; }
    void setXAmplitude(int16_t micrometers) { profile_.xMicrometers = micrometers; }
    void setYMinAmplitude(int16_t micrometers) { profile_.yAtXMinMicrometers = micrometers; }
    void setYMaxAmplitude(int16_t micrometers) { profile_.yAtXMaxMicrometers = micrometers; }
    bool enabled() const { return profile_.xMicrometers | profile_.yAtXMinMicrometers | profile_.yAtXMaxMicrometers; }
    float offsetMM(uint8_t axis) const { return offsetMM_[axis]; }

    void clearAxis(uint8_t axis) {
        offsetMM_[axis] = 0.0f;
        activeAxes_ &= ~(1U << axis);
    }

    void clear() {
        if (activeAxes_) {
            clearAxis(X_AXIS);
            clearAxis(Y_AXIS);
        }
    }

    void update(int32_t motorXSteps, float motorZMM, int32_t maxXSteps) {
        // The spindle advances 5 mm per rotation from the machine Z reference.
        const float phase = 6.2832f * (motorZMM / 5.0f) - profile_.phase * 0.031428f;
        const bool correctX = profile_.xMicrometers != 0;
        const bool correctY = profile_.yAtXMinMicrometers || profile_.yAtXMaxMicrometers;
        offsetMM_[X_AXIS] = correctX ? profile_.xMicrometers * cos(phase) * 0.001f : 0.0f;
        offsetMM_[Y_AXIS] = 0.0f;
        if (correctY) {
            const float xFraction = float(motorXSteps) / maxXSteps;
            const float amplitude = (1 - xFraction) * profile_.yAtXMinMicrometers + xFraction * profile_.yAtXMaxMicrometers;
            offsetMM_[Y_AXIS] = amplitude * sin(phase) * 0.001f;
        }
        activeAxes_ = (correctX << X_AXIS) | (correctY << Y_AXIS);
    }

    float applyToTarget(uint8_t axis, float nominalMM, float directMM, double limitMM) {
        if (!activeAxes_)
            return nominalMM;

        float target = nominalMM + offsetMM_[axis];
        if (target + directMM >= limitMM) {
            target = limitMM - directMM;
            // Reuse the correction actually applied at the limit on E-only moves.
            offsetMM_[axis] = target - nominalMM;
        }
        return target;
    }

private:
    Profile profile_{};
    float offsetMM_[2] = {};
    uint8_t activeAxes_ = 0;
};

#endif

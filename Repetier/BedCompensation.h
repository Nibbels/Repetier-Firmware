#ifndef BED_COMPENSATION_H
#define BED_COMPENSATION_H

#include <stdint.h>

namespace BedCompensation {
struct Geometry {
    int32_t nominalZSteps;
    int32_t fadeStartSteps;
    int32_t fadeEndSteps;
    int32_t previousLayerSteps;
    int32_t currentLayerSteps;
    int32_t matrixTopSteps;
};

struct Result {
    int32_t zCorrectionSteps;
    float extraExtrusionPerStep;
};

inline float extrusionRatio(int32_t riseSteps, int32_t spanSteps) {
    return riseSteps && spanSteps ? float(riseSteps) / float(spanSteps) : 0.0f;
}

// The matrix reader supplies the local bed height only when the active region needs it.
template <typename MatrixReader>
Result calculate(const Geometry& geometry, MatrixReader readMatrixDepth) {
    constexpr int32_t zeroZClearanceSteps = 13;
    const int32_t z = geometry.nominalZSteps;
    if (z < 0)
        return {geometry.matrixTopSteps + zeroZClearanceSteps, 0.0f};

    if (z >= geometry.fadeEndSteps) {
        float extraExtrusion = 0.0f;
        if (geometry.previousLayerSteps < geometry.fadeEndSteps && geometry.fadeEndSteps <= geometry.currentLayerSteps) {
            const int32_t rise = geometry.matrixTopSteps - readMatrixDepth();
            const int32_t overlap = geometry.fadeEndSteps - geometry.previousLayerSteps;
            const int32_t layerSpan = geometry.currentLayerSteps - geometry.previousLayerSteps;
            const int32_t fadeSpan = geometry.fadeEndSteps - geometry.fadeStartSteps;
            extraExtrusion = extrusionRatio(rise * overlap, fadeSpan * layerSpan);
        }
        return {geometry.matrixTopSteps, extraExtrusion};
    }

    const int32_t bedDepth = readMatrixDepth();
    if (z <= geometry.fadeStartSteps)
        return {bedDepth + (z == 0 ? zeroZClearanceSteps : 0), 0.0f};

    const int32_t fadeSpan = geometry.fadeEndSteps - geometry.fadeStartSteps;
    const int32_t localDelta = bedDepth - geometry.matrixTopSteps;
    const int32_t remainingFade = geometry.fadeEndSteps - z;
    const int32_t correction = geometry.matrixTopSteps + (localDelta * remainingFade) / fadeSpan;

    // At the first crossing only the part of the layer inside the fade stretches.
    int32_t rise = geometry.matrixTopSteps - bedDepth;
    int32_t span = fadeSpan;
    if (geometry.previousLayerSteps < geometry.fadeStartSteps && geometry.fadeStartSteps < geometry.currentLayerSteps) {
        rise *= geometry.currentLayerSteps - geometry.fadeStartSteps;
        span *= geometry.currentLayerSteps - geometry.previousLayerSteps;
    }
    return {correction, extrusionRatio(rise, span)};
}

inline int32_t digitZSteps(float digits, float stepsPerMm) {
    // Use the magnitude for either sensor polarity and the calibrated 2^17 divisor.
    const int32_t scaledForce = int32_t(digits * stepsPerMm);
    const uint32_t magnitude = scaledForce < 0 ? 0U - uint32_t(scaledForce) : uint32_t(scaledForce);
    const int32_t steps = magnitude >> 17;
    constexpr int32_t maxDigitZSteps = 600;
    return steps > maxDigitZSteps ? maxDigitZSteps : steps;
}
} // namespace BedCompensation

#endif // BED_COMPENSATION_H

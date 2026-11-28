#ifndef PRINT_LAYERS_H
#define PRINT_LAYERS_H

#include <stdint.h>

// Layer detection and fade boundaries in nominal planned Z steps.
// The foreground compares these boundaries with the executing nominal Z.
class PrintLayers {
public:
    struct PlannedMove {
        int32_t zSteps;
        bool movesZ;
        bool movesXY;
        bool positiveExtrusion;
    };

    constexpr explicit PrintLayers(float stepsPerMm)
        : fadeStartSteps_(defaultFadeStart(stepsPerMm)),
          fadeEndSteps_(defaultFadeEnd(stepsPerMm)) {}
    PrintLayers(const PrintLayers&) = delete;
    PrintLayers& operator=(const PrintLayers&) = delete;

    int32_t currentLayerSteps() const { return currentLayerSteps_; }
    int32_t previousLayerSteps() const { return previousLayerSteps_; }
    int32_t fadeStartSteps() const { return fadeStartSteps_; }
    int32_t fadeEndSteps() const { return fadeEndSteps_; }

    void recordPlannedMove(const PlannedMove& move, float stepsPerMm, int32_t matrixMaxSteps, int16_t matrixMinSteps) {
        // Returning to the current layer cancels a pending Z hop.
        if (move.movesZ)
            pendingLayerSteps_ = move.zSteps == currentLayerSteps_ ? 0 : move.zSteps;
        if (!move.positiveExtrusion)
            return;

        int32_t layerSteps = pendingLayerSteps_;
        // The first XY extrusion can share the height of the prime.
        if (!layerSteps && !currentLayerSteps_ && move.movesXY && move.zSteps > 0)
            layerSteps = move.zSteps;
        pendingLayerSteps_ = 0;
        if (!layerSteps)
            return;

        previousLayerSteps_ = currentLayerSteps_;
        currentLayerSteps_ = layerSteps;
        if (layerSteps < stepsPerMm) {
            // A descent within the first millimeter can begin a new object.
            if (previousLayerSteps_ > layerSteps)
                previousLayerSteps_ = 0;
            if (!previousLayerSteps_) {
                fadeStartSteps_ = layerSteps;
                fadeEndSteps_ = layerSteps + (matrixMaxSteps - matrixMinSteps) * 20;
            }
        }
    }

    void resetTracking() {
        pendingLayerSteps_ = 0;
        clearConfirmedLayers();
    }

    void clearConfirmedLayers() {
        currentLayerSteps_ = 0;
        previousLayerSteps_ = 0;
    }

    void resetFadeBounds(float stepsPerMm) {
        fadeStartSteps_ = defaultFadeStart(stepsPerMm);
        fadeEndSteps_ = defaultFadeEnd(stepsPerMm);
    }

    void rescaleFadeBounds(float ratio) {
        fadeStartSteps_ *= ratio;
        fadeEndSteps_ *= ratio;
    }

private:
    static constexpr int32_t defaultFadeStart(float stepsPerMm) { return int32_t(0.33f * stepsPerMm); }
    static constexpr int32_t defaultFadeEnd(float stepsPerMm) { return int32_t(10.0f * stepsPerMm); }

    int32_t pendingLayerSteps_ = 0;
    int32_t currentLayerSteps_ = 0;
    int32_t previousLayerSteps_ = 0;
    int32_t fadeStartSteps_;
    int32_t fadeEndSteps_;
};

#endif // PRINT_LAYERS_H

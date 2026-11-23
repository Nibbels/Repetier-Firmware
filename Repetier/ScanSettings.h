#ifndef SCAN_SETTINGS_H
#define SCAN_SETTINGS_H

struct ScanSettings {
    struct Axis {
        long startSteps = 0;
        long strideMm = 0;
        long strideSteps = 0;
        long endMarginSteps = 0;
        long maxPositionSteps = 0;
    };

    constexpr explicit ScanSettings(float initialLiftMm) : startLiftMm(initialLiftMm) {}

    Axis x;
    Axis y;
    float startLiftMm;
    short fastApproachSteps = 0;
    short slowApproachSteps = 0;
    short fastRetreatSteps = 0;
    short slowRetreatSteps = 0;
    short maxTravelSteps = 0;
    unsigned short fastStepDelayMs = 0;
    unsigned short slowStepDelayMs = 0;
    unsigned short idleDelayMs = 0;
    unsigned short contactPressureDelta = 0;
    unsigned short retryPressureDelta = 0;
    unsigned short idlePressureDelta = 0;
    short idlePressureMin = 0;
    short idlePressureMax = 0;
    char pressureReadCount = 0;
    unsigned short pressureReadDelayMs = 0;
    short pressureTolerance = 0;
};

#endif

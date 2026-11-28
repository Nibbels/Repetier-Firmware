#ifndef SCAN_PROBE_H
#define SCAN_PROBE_H

#include "CalibrationScan.h"
#include "PressureMeasurement.h"
#include "ScanSettings.h"

namespace Calibration {

using PressureResult = PressureMeasurement::Result;
constexpr int heatBedScanRetries = 5;

const ScanSettings& probeSettings();
extern ScanSession scanSession;

bool moveZ(int steps, bool emergencyRetreat = false);
void showAbortScanReason(const void* scanName, ScanAbort reason);

bool zOffsetScanRunning();
void tickHeatBedScan();
void tickZOffsetScan();
void discardHeatBedScan();
void discardZOffsetScan();

class ScanPressure {
public:
    PressureResult average(short& digits, uint16_t settleMs = 0);

    PressureResult afterStep(short& digits, uint16_t settleMs);

    PressureResult idle(short& digits);

    bool service(millis_t now);

    void reset();

private:
    PressureResult measure(short& digits, uint16_t settleMs, PressureMeasurement::Settings settings);

    PressureMeasurement measurement_;
    uint8_t idleComparisons_ = 0;
    short previousIdle_ = 0;
};

extern ScanPressure scanPressure;

enum class ProbeProgress : uint8_t { Pending, Done, Failed };

class ScanProbe {
public:
    // The same measurement is used for a mesh cell, hot-nozzle calibration and Z-offset correction.
    ProbeProgress measurePoint();
    ProbeProgress retreat();
    long measuredZ() const { return measuredZ_; }

    void setPressureLimits(short baseline, uint16_t contactDelta, uint16_t retryDelta) {
        contactMin_ = baseline - contactDelta;
        contactMax_ = baseline + contactDelta;
        releaseMin_ = baseline - retryDelta;
        releaseMax_ = baseline + retryDelta;
    }

    void forgetPreviousPoint() { lastProbeZ_ = 0; }

    void reset() {
        scanPressure.reset();
        releaseStarted_ = false;
        pointPhase_ = PointPhase::ApproachCoarse;
        retreatPhase_ = RetreatPhase::Idle;
    }

private:
    enum class PointPhase : uint8_t { ApproachCoarse, ReleaseCoarse, ApproachFine, ReleaseFine };
    enum class RetreatPhase : uint8_t { Idle, BeforeMove, AfterMove };

    ProbeProgress approach(bool fine);
    ProbeProgress release(uint8_t divisor);
    ProbeProgress advancePoint(ProbeProgress progress, PointPhase next);
    ProbeProgress travelLimit(ScanAbort reason) const {
        scanSession.requestRetryOrAbort(reason);
        return ProbeProgress::Failed;
    }

    long lastProbeZ_ = 0;
    long releaseStartZ_ = 0;
    long measuredZ_ = 0;
    short contactMin_ = 0;
    short contactMax_ = 0;
    short releaseMin_ = 0;
    short releaseMax_ = 0;
    PointPhase pointPhase_ = PointPhase::ApproachCoarse;
    millis_t retreatStartedAt_ = 0;
    bool releaseStarted_ = false;
    RetreatPhase retreatPhase_ = RetreatPhase::Idle;
};

extern ScanProbe scanProbe;

} // namespace Calibration

#endif

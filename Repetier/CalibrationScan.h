#ifndef CALIBRATION_SCAN_H
#define CALIBRATION_SCAN_H

#include <stdint.h>

enum class HeatBedScanMode : uint8_t { Standard = 0, Pla = 1, Abs = 2 };

struct ZOffsetScanPoint {
    uint8_t x;
    uint8_t y;
};

enum class ScanAbort : uint8_t {
    None = 0,
    Canceled = 1,
    StartPressure = 3,
    IdlePressure,
    AveragePressure,
    AboveStart,
    StepDeltaOutOfRange,
    MatrixDimensions,
    UnevenBed,
    MinEndstop,
    MaxEndstop,
    MaxCompensation,
    EndstopProtection,
    MissingMatrix,
    MatrixOverflow,
    MatrixHeight,
    DmsUnavailable,
    DmsOverload,
};

class ScanSession {
public:
    void reset() {
        abort_ = ScanAbort::None;
        armRetries(0);
    }

    static bool retreatOnly(ScanAbort reason) {
        return reason == ScanAbort::DmsUnavailable || reason == ScanAbort::DmsOverload;
    }

    void requestAbort(ScanAbort reason) {
        if (!aborting() || (retreatOnly(reason) && !retreatOnly(abort_)))
            abort_ = reason;
        armRetries(0);
    }

    bool aborting() const { return abort_ != ScanAbort::None; }

    ScanAbort takeAbort() {
        const auto reason = abort_;
        abort_ = ScanAbort::None;
        return reason;
    }

    void armRetries(uint8_t count) {
        retriesRemaining_ = count;
        retryRequested_ = false;
    }

    void requestRetryOrAbort(ScanAbort reason) {
        if (aborting())
            return;
        if (retriesRemaining_)
            retryRequested_ = true;
        else
            requestAbort(reason);
    }

    bool takeRetryRequest() {
        const bool requested = retryRequested_;
        retryRequested_ = false;
        return requested;
    }

    bool consumeRetry() {
        retryRequested_ = false;
        if (aborting() || !retriesRemaining_)
            return false;
        --retriesRemaining_;
        return true;
    }

    uint8_t retriesRemaining() const { return retriesRemaining_; }

    bool dispatching() const { return dispatching_; }
    void beginDispatch() { dispatching_ = true; }

    bool deferDiscard() {
        if (!dispatching_)
            return false;
        discardRequested_ = true;
        return true;
    }

    bool finishDispatch() {
        dispatching_ = false;
        const bool discard = discardRequested_;
        discardRequested_ = false;
        return discard;
    }

private:
    ScanAbort abort_ = ScanAbort::None;
    uint8_t retriesRemaining_ = 0;
    bool retryRequested_ = false;
    bool dispatching_ = false;
    bool discardRequested_ = false;
};

#endif

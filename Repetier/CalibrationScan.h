#ifndef CALIBRATION_SCAN_H
#define CALIBRATION_SCAN_H

#include <stdint.h>

enum class HeatBedScanMode : uint8_t { Standard = 0, Pla = 1, Abs = 2 };

struct ZOffsetScanPoint {
    uint8_t x;
    uint8_t y;
};

// One catalogue for scan reason, stable event code, and diagnostic text.
// Numeric ScanAbort values retain the established state-machine ABI;
// event codes are independent of them (1..11 are reserved for thermal).
// The two text fragments fit the stock 16-column panel and are also used
// by the history decoder. Do not infer motion/recovery policy from a code.
#define RF_SCAN_FAULTS(X) \
    X(StartPressure,        3, 12, "Bad start force",    "") \
    X(IdlePressure,         4, 13, "Bad idle force",     "") \
    X(AveragePressure,      5, 14, "Bad force signal",   "") \
    X(AboveStart,           6, 15, "Bug SOS",            "") \
    X(StepDeltaOutOfRange,   7, 16, "Leftovers, snot",    "incrustation") \
    X(MatrixDimensions,     8, 17, "Bug MDO",            "") \
    X(UnevenBed,            9, 18, "Uneven bed, dirt",   "hole, bad ground") \
    X(MinEndstop,          10, 19, "Min Endstop",        "") \
    X(MaxEndstop,          11, 20, "Max Endstop",        "") \
    X(MaxCompensation,    12, 21, "Reached limit",      "max compensation") \
    X(EndstopProtection,   13, 22, "Reached endstop",    "protection limit") \
    X(MissingMatrix,       14, 23, "Missing or",         "Invalid Matrix") \
    X(MatrixOverflow,      15, 24, "Correction would",   "overflow matrix") \
    X(MatrixHeight,        16, 25, "Badly wrong",        "Z-Screw/Z-Start") \
    X(DmsUnavailable,      17, 26, "DMS read failed",    "") \
    X(DmsOverload,         18, 27, "DMS overload",       "") \
    X(HeaterTargetRejected,19, 28, "Bed target",         "not admitted") \
    X(HeaterTargetChanged, 20, 29, "Bed target",         "changed")

enum class ScanAbort : uint8_t {
    None = 0,
    Canceled = 1,
#define RF_SCAN_ENUM(name, value, code, first, second) name = value,
    RF_SCAN_FAULTS(RF_SCAN_ENUM)
#undef RF_SCAN_ENUM
};

constexpr uint8_t scanFaultCode(ScanAbort reason) {
    // Use an explicit mapping, not an ordinal cast to the persisted ABI.
    return
#define RF_SCAN_CODE(name, value, code, first, second) reason == ScanAbort::name ? code :
    RF_SCAN_FAULTS(RF_SCAN_CODE)
#undef RF_SCAN_CODE
    0;
}

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
    // Foreground publishes a single-byte stop condition for Timer1.
    volatile ScanAbort abort_ = ScanAbort::None;
    uint8_t retriesRemaining_ = 0;
    bool retryRequested_ = false;
    bool dispatching_ = false;
    bool discardRequested_ = false;
};

#endif

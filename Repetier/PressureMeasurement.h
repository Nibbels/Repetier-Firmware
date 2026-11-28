#ifndef PRESSURE_MEASUREMENT_H
#define PRESSURE_MEASUREMENT_H

#include <stdint.h>

class PressureMeasurement {
public:
    enum class Result : uint8_t { Idle, Pending, Stable, Unstable, ReadError };

    struct Settings {
        uint8_t readings;
        uint16_t spacingMs;
        uint16_t tolerance;
    };

    void begin(Settings settings, uint32_t now, uint16_t settleMs = 0) {
        settings_ = settings;
        attempts_ = 0;
        startBatch(now, settleMs);
        result_ = settings_.readings ? Result::Pending : Result::ReadError;
    }

    bool acceptsSample(uint32_t startedAtMs) const {
        return result_ == Result::Pending && static_cast<int32_t>(startedAtMs - nextAt_) >= 0;
    }

    bool record(uint32_t startedAtMs, uint32_t readAtMs, int16_t digits) {
        if (!acceptsSample(startedAtMs))
            return false;

        sum_ += digits;
        if (digits < minimum_)
            minimum_ = digits;
        if (digits > maximum_)
            maximum_ = digits;
        if (++count_ < settings_.readings) {
            nextAt_ = startedAtMs + minimumSpacing();
            return true;
        }

        if (static_cast<int32_t>(maximum_) - minimum_ < settings_.tolerance) {
            result_ = Result::Stable;
        } else if (++attempts_ >= 5) {
            result_ = Result::Unstable;
        } else {
            startBatch(readAtMs, 100);
        }
        return true;
    }

    void readFailed() { result_ = Result::ReadError; }
    void reset() { result_ = Result::Idle; }

    Result result() const { return result_; }
    int16_t minimum() const { return minimum_; }
    int16_t maximum() const { return maximum_; }
    int16_t average() const { return sum_ / count_; }

private:
    uint16_t minimumSpacing() const { return settings_.spacingMs ? settings_.spacingMs : 1; }

    void startBatch(uint32_t now, uint16_t settleMs) {
        sum_ = 0;
        count_ = 0;
        minimum_ = INT16_MAX;
        maximum_ = INT16_MIN;
        // Require the start of a different conversion after settling.
        nextAt_ = now + settleMs + minimumSpacing();
    }

    Settings settings_{};
    uint32_t nextAt_ = 0;
    int32_t sum_ = 0;
    int16_t minimum_ = 0;
    int16_t maximum_ = 0;
    uint8_t count_ = 0;
    uint8_t attempts_ = 0;
    Result result_ = Result::Idle;
};

#endif

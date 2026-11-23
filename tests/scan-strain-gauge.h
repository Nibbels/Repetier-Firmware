#ifndef TEST_SCAN_STRAIN_GAUGE_H
#define TEST_SCAN_STRAIN_GAUGE_H

#include "StrainGauge.h"

class ScanStrainGauge {
public:
    void service(uint32_t now, int16_t digits) {
        if (started_ && now - startedAt_ < 125)
            return;
        if (started_)
            published_ = {startedAt_, now, static_cast<uint16_t>(published_.sequence + 1), input_};
        startedAt_ = now;
        input_ = digits;
        started_ = true;
    }

    StrainGauge::Availability latest(uint32_t, StrainGauge::Sample& sample) const {
        if (disconnected)
            return StrainGauge::Availability::Fault;
        if (!published_.sequence)
            return StrainGauge::Availability::Waiting;
        sample = published_;
        return StrainGauge::Availability::Ready;
    }

    int16_t correctedDigits(const StrainGauge::Sample& sample) const { return sample.rawDigits; }
    uint32_t latestStart() const { return published_.startedAtMs; }
    unsigned conversionsStarted() const { return started_ ? published_.sequence + 1 : 0; }

    bool disconnected = false;

private:
    StrainGauge::Sample published_{};
    uint32_t startedAt_ = 0;
    int16_t input_ = 0;
    bool started_ = false;
};

#endif

#ifndef STRAIN_GAUGE_H
#define STRAIN_GAUGE_H

#include <stdint.h>
#include "TwiTransfer.h"

class StrainGauge {
public:
    struct Sample {
        uint32_t startedAtMs;
        uint32_t readAtMs;
        uint16_t sequence;
        int16_t rawDigits;
    };

    enum class Availability : uint8_t { Waiting, Ready, Fault };
    static constexpr uint8_t historyCapacity = 8;

    // Foreground only. The requested rate sets an upper limit on new starts.
    void begin(uint32_t now);
    void service(uint32_t now);
    void setSampleRateHz(uint8_t hz);

    // Copy a published snapshot. Age zero selects the newest sample.
    bool history(uint8_t age, Sample& sample) const;
    Availability latest(uint32_t now, Sample& sample) const;
    int16_t correctedDigits(const Sample& sample) const;
    bool readDigits(uint32_t now, int16_t& digits) const;
    int16_t lastKnownDigits() const;

    bool zeroingEnabled() const { return zeroingEnabled_; }
    void setZeroingEnabled(bool enabled) { zeroingEnabled_ = enabled; }
    int16_t zeroOffset() const { return zeroOffset_; }
    int16_t effectiveZeroOffset() const { return zeroingEnabled_ ? zeroOffset_ : 0; }

    void clearZeroOffset();
    bool setZeroFromAverage(int16_t digits);

private:
    enum class Phase : uint8_t { Off, Configure, WaitForIdle, Trigger, CheckConversion, ReadResult };

    void startTransfer(uint32_t now);
    void completeTransfer(uint32_t now);
    void checkStatus(uint32_t now, Phase next);
    void retry(uint32_t now);
    void publish(uint32_t now);

    TwiTransfer transfer_;
    // The foreground writes into a spare slot. Publication changes only the
    // read indices and availability under interrupt protection.
    static constexpr uint8_t historySlots = historyCapacity + 1;
    Sample history_[historySlots]{};
    uint32_t nextAt_ = 0;
    uint32_t startedAt_ = 0;
    uint32_t busyUntil_ = 0;
    uint16_t sequence_ = 0;
    volatile uint16_t sampleIntervalMs_ = 125;
    volatile int16_t zeroOffset_ = 0;
    volatile uint8_t historyNext_ = 0;
    volatile uint8_t historySize_ = 0;
    volatile Availability availability_ = Availability::Fault;
    volatile bool zeroingEnabled_ = true;
    Phase phase_ = Phase::Off;
    bool awaitingTransfer_ = false;
    bool triggered_ = false;
};

#endif

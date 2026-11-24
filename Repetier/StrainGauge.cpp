#include "HAL.h"
#include "StrainGauge.h"

namespace {
constexpr uint8_t singleShot16BitGain1 = 0x1c;
constexpr uint8_t startOrBusy = 0x80;
constexpr uint8_t startConversion = singleShot16BitGain1 | startOrBusy;
constexpr uint16_t firstStatusCheckMs = 120;
constexpr uint16_t statusPollMs = 5;
constexpr uint16_t conversionTimeoutMs = 200;
constexpr uint16_t retryDelayMs = 100;
}

void StrainGauge::begin(uint32_t now) {
    if (phase_ == Phase::Off) {
        availability_ = Availability::Waiting;
        phase_ = Phase::Configure;
        nextAt_ = now;
    }
}

void StrainGauge::setSampleRateHz(uint8_t hz) {
    hz = constrain(hz, 1, 8);
    const uint16_t interval = (1000U + hz - 1) / hz;
    InterruptProtectedBlock lock;
    sampleIntervalMs_ = interval;
}

void StrainGauge::service(uint32_t now) {
    if (awaitingTransfer_) {
        transfer_.service(now);
        if (transfer_.busy())
            return;
        awaitingTransfer_ = false;
        if (transfer_.succeeded())
            completeTransfer(now);
        else
            retry(now);
    }

    if (phase_ != Phase::Off && static_cast<int32_t>(now - nextAt_) >= 0)
        startTransfer(now);
}

void StrainGauge::startTransfer(uint32_t now) {
    switch (phase_) {
    case Phase::Configure:
        awaitingTransfer_ = transfer_.write(I2C_ADDRESS_STRAIN_GAUGE, &singleShot16BitGain1, 1, now);
        break;
    case Phase::Trigger:
        if (triggered_ && now - startedAt_ < sampleIntervalMs_)
            break;
        awaitingTransfer_ = transfer_.write(I2C_ADDRESS_STRAIN_GAUGE, &startConversion, 1, now);
        if (awaitingTransfer_) {
            // The ADC cannot start before this timestamp. A probe can use it
            // to exclude conversions that might include a preceding move.
            startedAt_ = now;
            triggered_ = true;
        }
        break;
    case Phase::WaitForIdle:
    case Phase::CheckConversion:
    case Phase::ReadResult:
        awaitingTransfer_ = transfer_.read(I2C_ADDRESS_STRAIN_GAUGE, 3, now);
        break;
    case Phase::Off:
        break;
    }
}

void StrainGauge::completeTransfer(uint32_t now) {
    switch (phase_) {
    case Phase::Configure:
        // Switching from continuous to single-shot can leave a conversion in flight.
        phase_ = Phase::WaitForIdle;
        busyUntil_ = now + conversionTimeoutMs;
        nextAt_ = now;
        break;
    case Phase::WaitForIdle:
        checkStatus(now, Phase::Trigger);
        break;
    case Phase::Trigger:
        phase_ = Phase::CheckConversion;
        busyUntil_ = now + conversionTimeoutMs;
        nextAt_ = now + firstStatusCheckMs;
        break;
    case Phase::CheckConversion:
        // Fetch the value in a separate frame after observing idle. A
        // conversion can finish between the data bytes and ST/BSY.
        checkStatus(now, Phase::ReadResult);
        break;
    case Phase::ReadResult:
        if (transfer_.byteAt(2) != singleShot16BitGain1 || static_cast<int32_t>(now - busyUntil_) >= 0) {
            retry(now);
        } else {
            publish(now);
            phase_ = Phase::Trigger;
            nextAt_ = now;
        }
        break;
    case Phase::Off:
        break;
    }
}

void StrainGauge::checkStatus(uint32_t now, Phase next) {
    const uint8_t configuration = transfer_.byteAt(2);
    if (static_cast<int32_t>(now - busyUntil_) >= 0) {
        retry(now);
    } else if (configuration == singleShot16BitGain1) {
        phase_ = next;
        nextAt_ = now;
    } else if (configuration == startConversion) {
        nextAt_ = now + statusPollMs;
    } else {
        retry(now);
    }
}

void StrainGauge::retry(uint32_t now) {
    availability_ = Availability::Fault;
    phase_ = Phase::Configure;
    nextAt_ = now + retryDelayMs;
}

void StrainGauge::publish(uint32_t now) {
    const uint16_t raw = (uint16_t(transfer_.byteAt(0)) << 8) | transfer_.byteAt(1);
    uint8_t next = historyNext_;
    history_[next] = {startedAt_, now, ++sequence_, static_cast<int16_t>(raw)};
    if (++next == historySlots)
        next = 0;
    const uint8_t size = historySize_;
    InterruptProtectedBlock lock;
    historyNext_ = next;
    if (size < historyCapacity)
        historySize_ = size + 1;
    availability_ = Availability::Ready;
}

bool StrainGauge::history(uint8_t age, Sample& sample) const {
    if (age >= historySize_)
        return false;
    const int8_t index = historyNext_ - 1 - age;
    sample = history_[index < 0 ? index + historySlots : index];
    return true;
}

StrainGauge::Availability StrainGauge::latest(uint32_t now, Sample& sample) const {
    const Availability status = availability_;
    if (status != Availability::Ready)
        return status;
    if (!history(0, sample))
        return Availability::Fault;
    const uint16_t maxAgeMs = sampleIntervalMs_ + conversionTimeoutMs;
    return now - sample.startedAtMs <= maxAgeMs
        ? Availability::Ready : Availability::Fault;
}

int16_t StrainGauge::correctedDigits(const Sample& sample) const {
    int16_t digits = sample.rawDigits;
    // Leave values near the ADC rails in raw digits.
    if (-27768 < digits && digits < 27767)
        digits -= effectiveZeroOffset();
    return digits;
}

bool StrainGauge::readDigits(uint32_t now, int16_t& digits) const {
    Sample sample;
    if (latest(now, sample) != Availability::Ready)
        return false;
    digits = correctedDigits(sample);
    return true;
}

int16_t StrainGauge::lastKnownDigits() const {
    Sample sample;
    return history(0, sample) ? correctedDigits(sample) : 0;
}

void StrainGauge::clearZeroOffset() {
    InterruptProtectedBlock lock;
    zeroOffset_ = 0;
}

bool StrainGauge::setZeroFromAverage(int16_t digits) {
    if (digits <= -5000 || digits >= 5000)
        return false;
    InterruptProtectedBlock lock;
    zeroOffset_ = digits;
    return true;
}

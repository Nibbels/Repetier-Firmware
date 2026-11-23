#include <cassert>
#include <cstdint>
#include <vector>
#include "StrainGauge.h"

#define I2C_ADDRESS_STRAIN_GAUGE 0x49
#define constrain(value, low, high) ((value) < (low) ? (low) : (value) > (high) ? (high) : (value))
struct InterruptProtectedBlock { ~InterruptProtectedBlock() {} };

namespace {
struct {
    bool disconnected = false;
    bool continuous = true;
    bool stuckBusy = false;
    bool checkedReady = false;
    uint32_t readyAt = 0;
    uint16_t conversionMs = 125;
    int16_t result = -32000;
    int16_t nextResult = -32000;
    unsigned requests = 0;
    unsigned configurations = 0;
    std::vector<uint32_t> triggers;

    bool busy(uint32_t now) {
        if (stuckBusy || static_cast<int32_t>(now - readyAt) < 0)
            return true;
        result = nextResult;
        return false;
    }
} adc;
}

TwiTransfer* TwiTransfer::active_ = nullptr;

bool TwiTransfer::write(uint8_t address, const uint8_t* bytes, uint8_t length, uint32_t now) {
    assert(address == I2C_ADDRESS_STRAIN_GAUGE && !active_ && length == 1);
    active_ = this;
    ++adc.requests;
    state_ = adc.disconnected ? State::Error : State::Done;
    if (adc.disconnected)
        return true;
    if (bytes[0] == 0x1c) {
        adc.continuous = false;
        ++adc.configurations;
    } else {
        assert(bytes[0] == 0x9c && !adc.continuous && !adc.busy(now));
        adc.readyAt = now + adc.conversionMs;
        adc.checkedReady = false;
        adc.triggers.push_back(now);
        adc.nextResult = -2000 + static_cast<int16_t>(adc.triggers.size());
    }
    return true;
}

bool TwiTransfer::read(uint8_t address, uint8_t length, uint32_t now) {
    assert(address == I2C_ADDRESS_STRAIN_GAUGE && !active_ && length == 3);
    active_ = this;
    ++adc.requests;
    state_ = adc.disconnected ? State::Error : State::Done;
    const bool ready = !adc.busy(now);
    // Simulate ST switching to ready after the two output bytes were read.
    const uint16_t value = ready && adc.checkedReady ? adc.result : -32768;
    bytes_[0] = value >> 8;
    bytes_[1] = value & 0xff;
    bytes_[2] = adc.continuous ? 0x8c : ready ? 0x1c : 0x9c;
    adc.checkedReady = ready;
    return true;
}

void TwiTransfer::service(uint32_t) {
    if (busy())
        active_ = nullptr;
}

#include "strain-gauge-firmware.inc"

int main() {
    using Availability = StrainGauge::Availability;
    StrainGauge gauge;
    StrainGauge::Sample sample{};
    uint32_t now = UINT32_MAX - 42;
    adc.readyAt = now + 39;
    gauge.begin(now);
    assert(gauge.latest(now, sample) == Availability::Waiting && !gauge.history(0, sample));

    auto run = [&](unsigned duration) {
        for (unsigned t = 0; t < duration; ++t)
            gauge.service(now++);
    };
    run(38);
    assert(adc.triggers.empty());
    run(170);
    assert(adc.configurations == 1 && gauge.latest(now, sample) == Availability::Ready);
    assert(sample.sequence == 1 && sample.rawDigits == -1999 && sample.startedAtMs == adc.triggers.front());
    assert(sample.readAtMs - sample.startedAtMs >= 125);
    assert(!gauge.history(1, sample));
    assert(gauge.setZeroFromAverage(-1900) && gauge.lastKnownDigits() == -99);
    const unsigned readsBefore = adc.requests;
    int16_t corrected = 0;
    for (int i = 0; i < 12; ++i)
        assert(gauge.readDigits(now, corrected) && corrected == -99);
    assert(adc.requests == readsBefore);

    run(1410);
    assert(gauge.history(0, sample) && sample.sequence >= 12);
    const uint16_t newest = sample.sequence;
    for (uint8_t age = 0; age < StrainGauge::historyCapacity; ++age) {
        assert(gauge.history(age, sample) && sample.sequence == newest - age);
        assert(sample.rawDigits == -2000 + sample.sequence && sample.readAtMs - sample.startedAtMs >= 125);
    }
    assert(!gauge.history(8, sample));
    for (unsigned i = 1; i < adc.triggers.size(); ++i)
        assert(adc.triggers[i] - adc.triggers[i - 1] >= 125);

    gauge.setSampleRateHz(3);
    adc.conversionMs = 153;
    const unsigned before = adc.triggers.size();
    run(1300);
    assert(adc.triggers.size() >= before + 3);
    for (unsigned i = before; i < adc.triggers.size(); ++i)
        assert(adc.triggers[i] - adc.triggers[i - 1] >= 334);
    assert(gauge.latest(now, sample) == Availability::Ready);
    assert(gauge.latest(now + 600, sample) == Availability::Fault);

    adc.disconnected = true;
    run(600);
    assert(gauge.latest(now, sample) == Availability::Fault && gauge.history(7, sample));
    adc.disconnected = false;
    adc.stuckBusy = true;
    run(1000);
    assert(gauge.latest(now, sample) == Availability::Fault);
    adc.stuckBusy = false;
    run(600);
    assert(gauge.latest(now, sample) == Availability::Ready && sample.sequence > newest);

    const uint16_t beforeReset = sample.sequence;
    const unsigned beforeTrigger = adc.triggers.size();
    for (int ms = 0; ms < 335 && adc.triggers.size() == beforeTrigger; ++ms)
        run(1);
    assert(adc.triggers.size() > beforeTrigger);
    adc.continuous = true;
    run(600);
    assert(gauge.latest(now, sample) == Availability::Ready && sample.sequence > beforeReset);
    assert(adc.configurations >= 2);
}

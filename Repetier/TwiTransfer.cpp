#include "TwiTransfer.h"
#include <Arduino.h>
#include <util/twi.h>

namespace {
constexpr uint8_t twiProceed = _BV(TWINT) | _BV(TWEN);
constexpr uint32_t twiClockHz = 100000;
constexpr uint32_t twiTimeoutMs = 3;
}

TwiTransfer* TwiTransfer::active_ = nullptr;

void TwiTransfer::beginBus() {
    pinMode(SDA, INPUT_PULLUP);
    pinMode(SCL, INPUT_PULLUP);
    TWSR = 0;
    TWBR = (F_CPU / twiClockHz - 16) / 2;
    TWCR = _BV(TWEN);
}

bool TwiTransfer::start(uint8_t addressByte, uint8_t length, uint32_t now) {
    if (active_ || !length || length > sizeof(bytes_))
        return false;
    active_ = this;
    addressByte_ = addressByte;
    length_ = length;
    position_ = 0;
    lastEventAt_ = now;
    state_ = State::Moving;
    TWCR = twiProceed | _BV(TWSTA);
    return true;
}

bool TwiTransfer::write(uint8_t address, const uint8_t* bytes, uint8_t length, uint32_t now) {
    if (!start(address << 1, length, now))
        return false;
    for (uint8_t i = 0; i < length; ++i)
        bytes_[i] = bytes[i];
    return true;
}

bool TwiTransfer::read(uint8_t address, uint8_t length, uint32_t now) {
    return start((address << 1) | 1, length, now);
}

void TwiTransfer::stop(bool success) {
    state_ = success ? State::Stop : State::StopAfterError;
    TWCR = twiProceed | _BV(TWSTO);
}

void TwiTransfer::finish(bool success) {
    state_ = success ? State::Done : State::Error;
    active_ = nullptr;
}

void TwiTransfer::receiveByte() {
    // NACK the last byte to terminate a master read.
    TWCR = position_ + 1 < length_ ? twiProceed | _BV(TWEA) : twiProceed;
}

void TwiTransfer::service(uint32_t now) {
    if (!busy())
        return;

    const uint8_t control = TWCR;
    if (state_ != State::Moving) {
        if (!(control & _BV(TWSTO))) {
            finish(state_ == State::Stop);
            return;
        }
    } else if (control & _BV(TWINT)) {
        lastEventAt_ = now;
        switch (TW_STATUS) {
        case TW_START:
        case TW_REP_START:
            TWDR = addressByte_;
            TWCR = twiProceed;
            break;
        case TW_MT_SLA_ACK:
        case TW_MT_DATA_ACK:
            if (position_ < length_) {
                TWDR = bytes_[position_++];
                TWCR = twiProceed;
            } else {
                stop(true);
            }
            break;
        case TW_MR_SLA_ACK:
            receiveByte();
            break;
        case TW_MR_DATA_ACK:
            bytes_[position_++] = TWDR;
            receiveByte();
            break;
        case TW_MR_DATA_NACK:
            bytes_[position_++] = TWDR;
            stop(position_ == length_);
            break;
        case TW_MT_ARB_LOST:
            TWCR = twiProceed;
            finish(false);
            break;
        default:
            stop(false);
            break;
        }
        return;
    }

    // Progress already signalled by the hardware takes precedence over time
    // spent elsewhere in the foreground. Reset only when the bus is stuck.
    if (now - lastEventAt_ >= twiTimeoutMs) {
        TWCR = 0;
        TWCR = _BV(TWEN);
        finish(false);
    }
}

bool TwiTransfer::wait() {
    while (busy())
        service(millis());
    return succeeded();
}

void TwiTransfer::finishActive() {
    if (active_)
        active_->wait();
}

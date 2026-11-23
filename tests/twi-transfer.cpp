#include <cassert>
#include <cstdint>
#include "TwiTransfer.h"

constexpr uint8_t TWINT = 7, TWEA = 6, TWSTA = 5, TWSTO = 4, TWEN = 2, TWIE = 0;
constexpr uint8_t TW_START = 0x08, TW_REP_START = 0x10, TW_MT_SLA_ACK = 0x18, TW_MT_SLA_NACK = 0x20;
constexpr uint8_t TW_MT_DATA_ACK = 0x28, TW_MT_ARB_LOST = 0x38, TW_MR_SLA_ACK = 0x40;
constexpr uint8_t TW_MR_DATA_ACK = 0x50, TW_MR_DATA_NACK = 0x58;
#define _BV(bit) (1 << (bit))
#define TW_STATUS (TWSR & 0xf8)
constexpr uint32_t F_CPU = 16000000;
constexpr uint8_t SDA = 20, SCL = 21, INPUT_PULLUP = 2;

uint8_t TWSR = 0xff, TWBR = 0, TWDR = 0;
struct ControlRegister {
    uint8_t value = 0;
    operator uint8_t() const { return value; }
    void operator=(uint8_t bits) {
        assert(!(bits & _BV(TWIE)));
        value = bits & ~_BV(TWINT);
    }
} TWCR;

void pinMode(uint8_t pin, uint8_t mode) {
    assert((pin == SDA || pin == SCL) && mode == INPUT_PULLUP);
}
uint32_t millis() { return 0; }

#include "twi-transfer-firmware.inc"

void event(TwiTransfer& transfer, uint8_t status, uint8_t data = 0, uint32_t now = 0) {
    TWCR.value = _BV(TWEN) | _BV(TWINT);
    TWDR = data;
    TWSR = status;
    transfer.service(now);
}

void stopComplete(TwiTransfer& transfer) {
    assert(transfer.busy() && (TWCR & _BV(TWSTO)));
    TWCR.value &= ~_BV(TWSTO);
    transfer.service(0);
    assert(!transfer.busy());
}

int main() {
    TwiTransfer::beginBus();
    assert(TWSR == 0 && TWBR == 72 && TWCR == _BV(TWEN));
    TwiTransfer adc, eeprom;
    assert(adc.read(0x49, 3, 0) && adc.busy());
    assert(!eeprom.read(0x50, 1, 0));
    adc.service(2);
    assert(adc.busy() && (TWCR & _BV(TWSTA)));
    event(adc, TW_START, 0, 5); // The hardware finished while the foreground was elsewhere.
    assert(TWDR == 0x93);
    event(adc, TW_MR_SLA_ACK);
    assert(TWCR & _BV(TWEA));
    event(adc, TW_MR_DATA_ACK, 0xfe);
    assert(TWCR & _BV(TWEA));
    event(adc, TW_MR_DATA_ACK, 0xdc);
    assert(!(TWCR & _BV(TWEA)));
    event(adc, TW_MR_DATA_NACK, 0x1c);
    assert(!eeprom.read(0x50, 1, 0));
    stopComplete(adc);
    assert(adc.succeeded() && adc.byteAt(0) == 0xfe && adc.byteAt(1) == 0xdc && adc.byteAt(2) == 0x1c);

    const uint8_t bytes[] = {0x12, 0x34, 0x56};
    assert(eeprom.write(0x50, bytes, sizeof(bytes), 0));
    event(eeprom, TW_START);
    assert(TWDR == 0xa0);
    event(eeprom, TW_MT_SLA_ACK);
    assert(TWDR == 0x12);
    event(eeprom, TW_MT_DATA_ACK);
    assert(TWDR == 0x34);
    event(eeprom, TW_MT_DATA_ACK);
    assert(TWDR == 0x56);
    event(eeprom, TW_MT_DATA_ACK);
    stopComplete(eeprom);
    assert(eeprom.succeeded() && adc.succeeded());

    assert(eeprom.read(0x50, 1, 0));
    event(eeprom, TW_START);
    event(eeprom, TW_MR_SLA_ACK);
    assert(!(TWCR & _BV(TWEA)));
    event(eeprom, TW_MR_DATA_NACK, 0x42);
    stopComplete(eeprom);
    assert(eeprom.succeeded() && eeprom.byteAt(0) == 0x42);

    assert(eeprom.write(0x50, bytes, sizeof(bytes), 0));
    event(eeprom, TW_START);
    event(eeprom, TW_MT_SLA_NACK);
    stopComplete(eeprom);
    assert(!eeprom.succeeded());

    assert(adc.read(0x49, 3, UINT32_MAX - 1));
    adc.service(0);
    assert(adc.busy());
    adc.service(1);
    assert(!adc.busy() && !adc.succeeded() && TWCR == _BV(TWEN));
    assert(adc.read(0x49, 3, 2));
    event(adc, TW_MT_ARB_LOST);
    assert(!adc.busy() && !adc.succeeded() && !(TWCR & _BV(TWSTO)));
}

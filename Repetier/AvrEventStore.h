#pragma once
#include "EventLog.h"
#ifdef __AVR__
#include <avr/io.h>
#include <avr/interrupt.h>

namespace EventLog {
// Task-private candidate for ATmega2560. No absolute production allocation.
// Owner must serialize ALL EEPROM accesses (including existing avr-libc
// access, settings transactions and their periodic-service yields).
// ready() is checked in the same non-reentrant foreground service as read;
// no other EEPROM user may run from an ISR or between ready/read/startWrite.
// Settings must NOT enter a blocking EEPROM operation while a log byte is
// busy; defer until ready or service a bounded asynchronous settings writer.
template<uint16_t Base,uint8_t Slots,class Owner> class AvrStore {
    static_assert(Base>=0x0C00,"event region must not overlap settings/legacy");
    static_assert(uint32_t(Base)+Ring<Slots>::Bytes<=uint32_t(E2END)+1,
                  "event region outside physical EEPROM");
    const Owner& owner_;
public:
    explicit AvrStore(const Owner& owner):owner_(owner){}
    bool ready() const {
        return owner_.logMayAccess() && !(EECR&_BV(EEPE)) &&
               !(SPMCSR&_BV(SPMEN));
    }
    uint8_t read(uint16_t offset) const {
        // Caller guarantees ready and exclusive foreground access.
        EEAR=Base+offset;
        EECR|=_BV(EERE);
        return EEDR;
    }
    bool startWrite(uint16_t offset,uint8_t value) const {
        if(offset>=Ring<Slots>::Bytes||!ready())return false;
        // Set address/data and erase+write mode outside the IRQ mask;
        // no EEPROM-ready IRQ is enabled by this driver.
        EEAR=Base+offset;
        EEDR=value;
        EECR&=uint8_t(~(_BV(EEPM0)|_BV(EEPM1)));
        const uint8_t saved=SREG;
        cli();
        // ATmega2560 EECR is SBI-addressable; EEMPE->EEPE adjacent,
        // two 2-cycle instructions. Never let compiler split the pair.
        asm volatile("sbi %0, %1\n\tsbi %0, %2"
                     :: "I" (_SFR_IO_ADDR(EECR)), "I" (EEMPE), "I" (EEPE)
                     : "memory");
        SREG=saved;
        return true;
    }
};
}
#endif

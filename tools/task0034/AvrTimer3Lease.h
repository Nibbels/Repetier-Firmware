// Research-only Timer3 adapter. Do not link into firmware until ownership,
// linked critical section duration, initialization and failure paths qualify.
#pragma once
#include <avr/io.h>
#include <stdint.h>

template<void (*Off)(uint8_t)> class AvrTimer3Lease {
    static uint8_t mask(uint8_t c) { return uint8_t(_BV(OCIE3A) << c); }
public:
    // Exclusive Timer3 ownership. Called with IRQs masked, all heaters OFF.
    // Normal mode, disconnected OC pins, /256 => 16 us at 16 MHz.
    static void begin() {
        static_assert(F_CPU == 16000000UL, "prototype assumes 16 MHz");
        static_assert(OCIE3A == OCF3A && OCIE3B == OCF3B && OCIE3C == OCF3C
                      && OCIE3B == OCIE3A+1 && OCIE3C == OCIE3A+2, "mask mapping");
        TIMSK3 = 0;
        TCCR3B = 0;
        TCCR3A = 0;
        TCCR3C = 0;
        TCNT3 = 0;
        OCR3A = OCR3B = OCR3C = 0;
        TIFR3 = _BV(OCF3A) | _BV(OCF3B) | _BV(OCF3C) | _BV(TOV3);
        TCCR3B = _BV(CS32);
    }
    uint16_t now() const { return TCNT3; }
    uint16_t deadline(uint8_t c) const {
        switch(c) { case 0: return OCR3A; case 1: return OCR3B; default: return OCR3C; }
    }
    bool pending(uint8_t c) const { return TIFR3 & mask(c); }
    void writeDeadline(uint8_t c, uint16_t x) {
        switch(c) { case 0: OCR3A=x; break; case 1: OCR3B=x; break; default: OCR3C=x; break; }
    }
    // W1C direct assignment; never read-modify-write a TIFR register.
    void clearPending(uint8_t c) { TIFR3=mask(c); }
    void enable(uint8_t c) { TIMSK3 |= mask(c); }
    void disable(uint8_t c) { TIMSK3 &= uint8_t(~mask(c)); }
    void off(uint8_t c) { Off(c); }
};

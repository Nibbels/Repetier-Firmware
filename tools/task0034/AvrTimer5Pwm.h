// Research only. Exclusive Timer5 use; disconnect OC5A/B/C so PL3 Z STEP
// remains GPIO. Call with IRQs masked and heater pins already safely LOW.
#pragma once
#include <avr/io.h>
#include <stdint.h>
template<void (*On)(uint8_t), void (*Off)(uint8_t)>
class AvrTimer5Pwm {
    static uint8_t mask(uint8_t c) { return uint8_t(_BV(OCIE5A) << c); }
public:
    static void begin() {
        static_assert(F_CPU == 16000000UL, "prototype assumes 16 MHz");
        static_assert(OCIE5A == OCF5A && OCIE5B == OCF5B && OCIE5C == OCF5C
                   && OCIE5B == OCIE5A+1 && OCIE5C == OCIE5A+2, "mask mapping");
        TIMSK5=0;
        TCCR5B=0;
        TCCR5A=0;
        TCCR5C=0;
        TCNT5=0;
        OCR5A=OCR5B=OCR5C=0;
        TIFR5=_BV(OCF5A)|_BV(OCF5B)|_BV(OCF5C)|_BV(TOV5);
        TIMSK5=_BV(TOIE5);
        TCCR5B=_BV(CS51);
    }
    uint16_t now() const { return TCNT5; }
    bool wrapped() const { return TIFR5 & _BV(TOV5); }
    bool pending(uint8_t c) const { return TIFR5 & mask(c); }
    void writeDeadline(uint8_t c,uint16_t x) {
        switch(c) { case 0: OCR5A=x; break; case 1: OCR5B=x; break; default: OCR5C=x; break; }
    }
    void clearPending(uint8_t c) { TIFR5=mask(c); }
    void enable(uint8_t c) { TIMSK5 |= mask(c); }
    void disable(uint8_t c) { TIMSK5 &= uint8_t(~mask(c)); }
    void on(uint8_t c) { On(c); }
    void off(uint8_t c) { Off(c); }
};

#pragma once
#include <avr/io.h>
#include <stdint.h>

// Task0034 non-production Timer2 adapter for the two routed hotend OC pins.
// PB4=OC2A, PH6=OC2B on ATmega2560. Never silently use on a different pin map.
// No interrupts, no automatic rising edge in the running/armed CLEAR mode.
// External OFF bias while DDR is input is an essential precondition.
// This adapter does not control PH5/PE2 bed and does not allocate Timer4/5.
#ifndef __AVR_ATmega2560__
#error Compare-clear adapter is specific to ATmega2560
#endif
class AvrTimer2CompareClear {
    static constexpr uint8_t clockBits=_BV(CS22)|_BV(CS21)|_BV(CS20);
    static constexpr uint8_t mask(uint8_t c) {return c?(_BV(COM2B1)|_BV(COM2B0)):(_BV(COM2A1)|_BV(COM2A0));}
    static constexpr uint8_t flag(uint8_t c) {return c?_BV(OCF2B):_BV(OCF2A);}
    static void mode(uint8_t c,uint8_t value) {
        TCCR2A=(TCCR2A&uint8_t(~mask(c)))|(c?(value<<COM2B0):(value<<COM2A0));
    }
public:
    // Called before heater admission, with all interrupts masked.
    // Fail without touching a timer already in asynchronous mode.
    static bool begin() {
        if(ASSR & (_BV(AS2)|_BV(EXCLK)|_BV(TCN2UB)|_BV(OCR2AUB)|_BV(OCR2BUB)|_BV(TCR2AUB)|_BV(TCR2BUB))) return false;
        TIMSK2=0;
        TCCR2B=0;
        TCCR2A=0;
        TCNT2=0;
        OCR2A=OCR2B=0;
        TIFR2=_BV(OCF2A)|_BV(OCF2B)|_BV(TOV2);
        TCCR2B=clockBits;
        return true;
    }
    static void portLow(uint8_t c) {
        if(c) PORTH&=uint8_t(~_BV(PH6));else PORTB&=uint8_t(~_BV(PB4));
    }
    static void floatOff(uint8_t c) {
        if(c) DDRH&=uint8_t(~_BV(PH6));else DDRB&=uint8_t(~_BV(PB4));
    }
    static void drive(uint8_t c) {
        if(c) DDRH|=_BV(PH6);else DDRB|=_BV(PB4);
    }
    static void disconnect(uint8_t c) {mode(c,0);}
    static void setMode(uint8_t c) {mode(c,3);}
    static void clearMode(uint8_t c) {mode(c,2);}
    static void force(uint8_t c) {
        // FOC is a strobe, not a PWM edge generator or OCF acknowledgement.
        TCCR2B=clockBits|(c?_BV(FOC2B):_BV(FOC2A));
    }
    static uint8_t counter(){return TCNT2;}
    static void deadline(uint8_t c,uint8_t value){if(c)OCR2B=value;else OCR2A=value;}
    static void ack(uint8_t c){TIFR2=flag(c);}
    static bool pending(uint8_t c){return TIFR2&flag(c);}
};

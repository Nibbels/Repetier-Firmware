/*
    This file is part of the Repetier-Firmware for RF devices from Conrad Electronic SE.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef HAL_H
#define HAL_H

/**
  ATmega2560 hardware access, timers, ADC publication and PWM requests.
  BoardConfiguration.h selects RF model and pins; UsbSerial and TwiTransfer
  own the respective peripheral transports. Other MCUs are not supported.
*/

#include "BoardConfiguration.h"
#include <avr/pgmspace.h>
#include <avr/io.h>
#include "UsbSerial.h"
#define prog_char char

#define INLINE __attribute__((always_inline))

#define PACK

// Give an enabled stepper driver time to settle before the first step.
constexpr unsigned int stepperEnableDelayMs = 25;

#define FSTRINGVALUE(var, value) const char var[] PROGMEM = value;
#define FSTRINGVAR(var) static const char var[] PROGMEM;
#define FSTRINGPARAM(var) PGM_P var

#include <avr/eeprom.h>
#include <avr/wdt.h>
/** \brief Prescale factor, timer0 runs at.

All known arduino boards use 64. This value is needed for the extruder timing. */
#define TIMER0_PRESCALE 64

#define ANALOG_PRESCALER _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2)

#undef HardwareSerial_h
#define HardwareSerial_h // Don't use standard serial console

#include <inttypes.h>

#include "Arduino.h"
#include "fastio.h"

#define _STEP_WITH_COUNT(IO, pending, remaining) \
    HAL::stepWithCount(_SFR_IO_ADDR(DIO ## IO ## _WPORT), DIO ## IO ## _PIN, pending, remaining)
#define STEP_WITH_COUNT(IO, pending, remaining) _STEP_WITH_COUNT(IO, pending, remaining)

#ifndef WATCHDOG_PIN
#error Watchdog pin not defined. We really need some Watchdog pin with a working watchdogtimer!
#endif

//7 means down till 2.1hz is possible:
#define PART_FAN_MODE_MAX 7

#define PART_FAN_MODE_PWM 0
#define PART_FAN_MODE_PDM 1

// Indices follow the temperature ADC channel order in HAL.cpp.
enum TemperatureAdcIndex : uint8_t {
    Hotend0AdcIndex,
#if NUM_EXTRUDER > 1
    Hotend1AdcIndex,
#endif
    BedAdcIndex,
#ifdef X35_ADC_CHANNEL
    ChamberAdcIndex,
#endif
    TemperatureAdcCount
};

class InterruptProtectedBlock {
private:
    uint8_t sreg;

public:
    inline void protect() {
        cli();
    }

    inline void unprotect() {
        SREG = sreg;
    }

    inline InterruptProtectedBlock(bool later = false) {
        sreg = SREG;
        if (!later)
            cli();
    }

    inline ~InterruptProtectedBlock() {
        SREG = sreg;
    }
};

#define EEPROM_OFFSET 0
#define SECONDS_TO_TICKS(s) (unsigned long)(s * (float)F_CPU)
#define ANALOG_REDUCE_BITS 0
#define ANALOG_REDUCE_FACTOR 1

#define MAX_RAM 8192

#define bit_clear(x, y) x &= ~(1 << y)
#define bit_set(x, y) x |= (1 << y)

#define LIMIT_INTERVAL ((F_CPU / 40000) + 1)

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

typedef uint16_t speed_t;
typedef uint32_t ticks_t;
typedef uint32_t millis_t;
typedef uint8_t flag8_t;
typedef int8_t fast8_t;
typedef uint8_t ufast8_t;

#define FAST_INTEGER_SQRT

// Implement serial communication for one stream only!
/*
  HardwareSerial.h - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 28 September 2010 by Mark Sproul

  Modified to use only 1 queue with fixed length by Repetier
*/

#define OUT_P_I(p, i) Com::printF(PSTR(p), (int)(i))
#define OUT_P_I_LN(p, i) Com::printFLN(PSTR(p), (int)(i))
#define OUT_P_L(p, i) Com::printF(PSTR(p), (long)(i))
#define OUT_P_L_LN(p, i) Com::printFLN(PSTR(p), (long)(i))
#define OUT_P_F(p, i) Com::printF(PSTR(p), (float)(i))
#define OUT_P_F_LN(p, i) Com::printFLN(PSTR(p), (float)(i))
#define OUT_P_FX(p, i, x) Com::printF(PSTR(p), (float)(i), x)
#define OUT_P_FX_LN(p, i, x) Com::printFLN(PSTR(p), (float)(i), x)
#define OUT_P(p) Com::printF(PSTR(p))
#define OUT_P_LN(p) Com::printFLN(PSTR(p))
#define OUT_ERROR_P(p) Com::printErrorF(PSTR(p))
#define OUT_ERROR_P_LN(p) \
    { \
        Com::printErrorF(PSTR(p)); \
        Com::println(); \
    }
#define OUT(v) Com::print(v)
#define OUT_LN Com::println()

class HAL {
public:
    HAL();
    virtual ~HAL();

    // return val'val
    static uint16_t integerSqrt(int32_t a);

    /** \brief Optimized division
    Normally the C compiler will compute a long/long division, which takes ~670 Ticks.
    This version is optimized for a 16 bit dividend and recognises the special cases
    of a 24 bit and 16 bit dividend, which offen, but not always occur in updating the
    interval.
    */
    static inline int32_t Div4U2U(uint32_t a, uint16_t b) {
        // r14/r15 remainder
        // r16 counter
        __asm volatile(
            "clr r14 \n\t"
            "sub r15,r15 \n\t"
            "tst %D0 \n\t"
            "brne do32%= \n\t"
            "tst %C0 \n\t"
            "breq donot24%= \n\t"
            "rjmp do24%= \n\t"
            "donot24%=:"
            "ldi r16,17 \n\t" // 16 Bit divide
            "d16u_1%=:"
            "rol %A0 \n\t"
            "rol %B0 \n\t"
            "dec r16 \n\t"
            "brne   d16u_2%= \n\t"
            "rjmp end%= \n\t"
            "d16u_2%=:"
            "rol r14 \n\t"
            "rol r15 \n\t"
            "sub r14,%A2 \n\t"
            "sbc r15,%B2 \n\t"
            "brcc   d16u_3%= \n\t"
            "add r14,%A2 \n\t"
            "adc r15,%B2 \n\t"
            "clc \n\t"
            "rjmp d16u_1%= \n\t"
            "d16u_3%=:"
            "sec \n\t"
            "rjmp d16u_1%= \n\t"
            "do32%=:" // divide full 32 bit
            "rjmp do32B%= \n\t"
            "do24%=:" // divide 24 bit

            "ldi r16,25 \n\t" // 24 Bit divide
            "d24u_1%=:"
            "rol %A0 \n\t"
            "rol %B0 \n\t"
            "rol %C0 \n\t"
            "dec r16 \n\t"
            "brne   d24u_2%= \n\t"
            "rjmp end%= \n\t"
            "d24u_2%=:"
            "rol r14 \n\t"
            "rol r15 \n\t"
            "sub r14,%A2 \n\t"
            "sbc r15,%B2 \n\t"
            "brcc   d24u_3%= \n\t"
            "add r14,%A2 \n\t"
            "adc r15,%B2 \n\t"
            "clc \n\t"
            "rjmp d24u_1%= \n\t"
            "d24u_3%=:"
            "sec \n\t"
            "rjmp d24u_1%= \n\t"

            "do32B%=:" // divide full 32 bit

            "ldi r16,33 \n\t" // 32 Bit divide
            "d32u_1%=:"
            "rol %A0 \n\t"
            "rol %B0 \n\t"
            "rol %C0 \n\t"
            "rol %D0 \n\t"
            "dec r16 \n\t"
            "brne   d32u_2%= \n\t"
            "rjmp end%= \n\t"
            "d32u_2%=:"
            "rol r14 \n\t"
            "rol r15 \n\t"
            "sub r14,%A2 \n\t"
            "sbc r15,%B2 \n\t"
            "brcc   d32u_3%= \n\t"
            "add r14,%A2 \n\t"
            "adc r15,%B2 \n\t"
            "clc \n\t"
            "rjmp d32u_1%= \n\t"
            "d32u_3%=:"
            "sec \n\t"
            "rjmp d32u_1%= \n\t"

            "end%=:" // end
            : "=&r"(a)
            : "0"(a), "r"(b)
            : "r14", "r15", "r16");
        return a;

    } // Div4U2U

    static inline unsigned long U16SquaredToU32(unsigned int val) {
        long res;

        __asm volatile( // 15 Ticks
            "mul %A1,%A1 \n\t"
            "movw %A0,r0 \n\t"
            "mul %B1,%B1 \n\t"
            "movw %C0,r0 \n\t"
            "mul %A1,%B1 \n\t"
            "clr %A1 \n\t"
            "add %B0,r0 \n\t"
            "adc %C0,r1 \n\t"
            "adc %D0,%A1 \n\t"
            "add %B0,r0 \n\t"
            "adc %C0,r1 \n\t"
            "adc %D0,%A1 \n\t"
            "clr r1 \n\t"
            : "=&r"(res), "=r"(val)
            : "1"(val));
        return res;

    } // U16SquaredToU32

    static inline speed_t ComputeV(long timer, long accel) {
        unsigned int res;

        // 38 Ticks
        __asm volatile( // 0 = res, 1 = timer, 2 = accel %D2=0 ,%A1 are unused is free
            // Result LSB first: %A0, %B0, %A1
            "mul %B1,%A2 \n\t"
            "mov %A0,r1 \n\t"
            "mul %B1,%C2 \n\t"
            "mov %B0,r0 \n\t"
            "mov %A1,r1 \n\t"
            "mul %B1,%B2 \n\t"
            "add %A0,r0 \n\t"
            "adc %B0,r1 \n\t"
            "adc %A1,%D2 \n\t"
            "mul %C1,%A2 \n\t"
            "add %A0,r0 \n\t"
            "adc %B0,r1 \n\t"
            "adc %A1,%D2 \n\t"
            "mul %C1,%B2 \n\t"
            "add %B0,r0 \n\t"
            "adc %A1,r1 \n\t"
            "mul %D1,%A2 \n\t"
            "add %B0,r0 \n\t"
            "adc %A1,r1 \n\t"
            "mul %C1,%C2 \n\t"
            "add %A1,r0 \n\t"
            "mul %D1,%B2 \n\t"
            "add %A1,r0 \n\t"
            "lsr %A1 \n\t"
            "ror %B0 \n\t"
            "ror %A0 \n\t"
            "lsr %A1 \n\t"
            "ror %B0 \n\t"
            "ror %A0 \n\t"
            "clr r1 \n\t"
            : "=&r"(res), "=r"(timer), "=r"(accel)
            : "1"(timer), "2"(accel)
            :);
        // unsigned int v = ((timer>>8)*cur->accel)>>10;
        return res;

    } // ComputeV

    static inline void digitalWrite(uint8_t pin, uint8_t value) {
        ::digitalWrite(pin, value);

    } // digitalWrite

    static inline uint8_t digitalRead(uint8_t pin) {
        return ::digitalRead(pin);

    } // digitalRead

    static inline void pinMode(uint8_t pin, uint8_t mode) {
        ::pinMode(pin, mode);

    } // pinMode

    static int32_t CPUDivU2(unsigned int divisor);

    static INLINE void stepWithCount(uint8_t port, uint8_t bit, volatile int16_t& pending, int16_t remaining) {
        // Book the step while STEP is high. The stores take four cycles and CBI takes two.
        // DRV8711 requires STEP high >= 1.9 us. Round UP to whole CPU cycles
        // (31 at 16 MHz = 1.9375 us). Shorter pulses are hardware experiments only.
        constexpr unsigned long highCycles = (19UL * F_CPU + 9999999UL) / 10000000UL;
        constexpr unsigned long waitCycles = highCycles - 6;
        uint8_t counter;
        asm volatile(
            "sbi %[port], %[bit]\n\t"
            "std %a[pending]+1, %B[remaining]\n\t"
            "st %a[pending], %A[remaining]\n\t"
            "ldi %[counter], %[loops]\n"
            "1: dec %[counter]\n\t"
            "brne 1b\n\t"
            ".rept %[tail]\n\t"
            "nop\n\t"
            ".endr\n\t"
            "cbi %[port], %[bit]"
            : [counter] "=&d"(counter)
            : [port] "I"(port), [bit] "I"(bit), [pending] "b"(&pending),
              [remaining] "r"(remaining), [loops] "M"(waitCycles / 3), [tail] "n"(waitCycles % 3)
            : "cc", "memory");
    }

    static inline void delayMicroseconds(unsigned int delayUs) // See https://www.arduino.cc/en/Reference/AttachInterrupt for interrupt-warning.
    {
        ::delayMicroseconds(delayUs);
    } // delayMicroseconds

    static inline void delayMilliseconds(unsigned int delayMs) {
        ::delay(delayMs);
    } // delayMilliseconds

    static inline void eprSetByte(unsigned int pos, uint8_t value) {
        uint8_t oldval = eprGetByte(pos);
        if (oldval != value)
            eeprom_write_byte((uint8_t*)(EEPROM_OFFSET + pos), value);

    } // eprSetByte

    static inline void eprSetInt16(unsigned int pos, int16_t value) {
        int16_t oldval = eprGetInt16(pos);
        if (oldval != value)
            eeprom_write_word((unsigned int*)(EEPROM_OFFSET + pos), value);

    } // eprSetInt16

    static inline void eprSetInt32(unsigned int pos, int32_t value) {
        int32_t oldval = eprGetInt32(pos);
        if (oldval != value)
            eeprom_write_dword((uint32_t*)(EEPROM_OFFSET + pos), value);

    } // eprSetInt32

    static inline void eprSetFloat(unsigned int pos, float value) {
        float oldval = eprGetFloat(pos);
        if (oldval != value)
            eeprom_write_block(&value, (void*)(EEPROM_OFFSET + pos), 4);

    } // eprSetFloat

    static inline uint8_t eprGetByte(unsigned int pos) {
        return eeprom_read_byte((uint8_t*)(EEPROM_OFFSET + pos));

    } // eprGetByte

    static inline int16_t eprGetInt16(unsigned int pos) {
        return eeprom_read_word((uint16_t*)(EEPROM_OFFSET + pos));

    } // eprGetInt16

    static inline int32_t eprGetInt32(unsigned int pos) {
        return eeprom_read_dword((uint32_t*)(EEPROM_OFFSET + pos));

    } // eprGetInt32

    static inline float eprGetFloat(unsigned int pos) {
        float v;
        eeprom_read_block(&v, (void*)(EEPROM_OFFSET + pos), 4); // newer gcc have eeprom_read_block but not arduino 22
        return v;

    } // eprGetFloat

    // Faster version of InterruptProtectedBlock.
    // For safety it may only be called from within an
    // interrupt handler.
    static inline void allowInterrupts() {
        sei();
    } // allowInterrupts

    static inline void forbidInterrupts() {
        cli();
    } // forbidInterrupts

    static inline unsigned long timeInMilliseconds() {
        return millis();
    } // timeInMilliseconds

    static inline unsigned long timeInMicroseconds() {
        return micros();
    } // timeInMicroseconds

    static inline char readFlashByte(PGM_P ptr) {
        return pgm_read_byte(ptr);

    } // readFlashByte

    static inline void serialSetBaudrate(long baud) {
        UsbSerial::begin(baud);

    } // serialSetBaudrate

    static inline bool serialByteAvailable() {
        return UsbSerial::available();

    } // serialByteAvailable

    static inline uint8_t serialReadByte() {
        return UsbSerial::read();

    } // serialReadByte

    static inline void serialWriteByte(char b) {
        UsbSerial::write(b);

    } // serialWriteByte

    static inline void serialFlush() {
        UsbSerial::flush();

    } // serialFlush

    static void setupTimer();

    static void resetPwmOutputs();
    static uint8_t outputDuty(uint8_t channel);
#ifdef RF_HEATER_LEASE_EXPERIMENT
    // A foreground reservation for one exact ADC publication. Conversion
    // runs outside the IRQ lock; grant checks the reservation under lock.
    class HeaterSample {
        uint8_t index_ = 255;
    public:
        HeaterSample() = default;
        ~HeaterSample();
        HeaterSample(const HeaterSample&) = delete;
        HeaterSample& operator=(const HeaterSample&) = delete;
        bool read(uint8_t index, uint16_t& raw);
        void release();
    private:
        friend class HAL;
        bool consume(uint8_t channel);
    };
    static void setHeaterDuty(uint8_t channel, uint8_t duty, HeaterSample* sample = nullptr);
    static bool heaterLeaseFault();
#else
    static void setHeaterDuty(uint8_t channel, uint8_t duty);
#endif
    static void setBoardFan(bool enabled);
    static uint8_t partFanSpeed();
    static void setPartFanSpeed(uint8_t speed);
    static uint8_t partFanDivisor();
    static void setPartFanDivisor(uint8_t divisor);
    static uint8_t partFanMode();
    static void setPartFanMode(uint8_t mode);
    static uint8_t partFanMinDuty();
    static uint8_t partFanMaxDuty();
    static void setPartFanDutyRange(uint8_t minimum, uint8_t maximum);
    static void showStartReason();
    static int getFreeRam();
    static void resetHardware();

    // Nibbels: Use watchdog to trigger external watchdog with constant time function according to command loop.
    static void WDT_Init(void);

    // Watchdog support
    static void startWatchdog();
    static void stopWatchdog();
    static void tellWatchdogOk();
    static bool watchdogEnabled();

    static bool takeScanTick();
    static bool takeCompensationTick();
    static bool takeFeatureTick();
    static bool takeTemperatureTick();
    static bool temperatureTickPending();

    inline static void testWatchdog() {
        while (1) {
        }
    } // testWatchdog

    inline static float maxExtruderTimerFrequency() {
        return (float)F_CPU / TIMER0_PRESCALE;

    } // maxExtruderTimerFrequency

    static void analogStart();
    static uint16_t temperatureAdcSample(uint8_t index); // Raw diagnostic snapshot.
    // Non-consuming snapshot; false before first publication or after expiry.
    static bool temperatureAdcSample(uint8_t index, uint16_t& sample);
};

#define EXTRUDER_TIMER_VECTOR TIMER0_COMPA_vect
#define EXTRUDER_OCR OCR0A
#define EXTRUDER_TCCR TCCR0A
#define EXTRUDER_TIMSK TIMSK0
#define EXTRUDER_OCIE OCIE0A
#define PWM_TIMER_VECTOR TIMER0_COMPB_vect
#define PWM_OCR OCR0B
#define PWM_TCCR TCCR0A
#define PWM_TIMSK TIMSK0
#define PWM_OCIE OCIE0B

#endif // HAL_H

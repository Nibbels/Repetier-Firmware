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

#include "Repetier.h"
#include "StepExecutor.h"

namespace {
const uint8_t temperatureAdcChannels[TemperatureAdcCount] PROGMEM = {
    EXT0_TEMPSENSOR_PIN,
#if NUM_EXTRUDER > 1
    EXT1_TEMPSENSOR_PIN,
#endif
    HEATED_BED_SENSOR_PIN,
#ifdef X35_ADC_CHANNEL
    TEMP_3_PIN,
#endif
};
// Only the currently selected channel accumulates; a complete batch resets both.
volatile uint8_t temperatureAdcCount = 0;
volatile uint16_t temperatureAdcSum = 0;
volatile uint8_t temperatureAdcPosition = 0;
volatile uint16_t temperatureAdcValues[TemperatureAdcCount] = { 0 };
// A full scan takes at most 32.768 ms at the nominal 256 us compare cadence.
// Permit a 100 ms publication age; this is a software limit, not a physical bound.
constexpr uint32_t temperatureAdcMaxAgeMs = 100;
volatile uint32_t temperatureAdcPublishedAt[TemperatureAdcCount] = { 0 };
volatile bool temperatureAdcReady[TemperatureAdcCount] = { false };

inline __attribute__((always_inline)) void serviceTemperatureAdc() {
    // ADSC alone can also be clear when the ADC has been disabled.
    // ADIF proves a conversion completed since the last acknowledged result.
    const uint8_t control = ADCSRA;
    if ((control & (_BV(ADEN) | _BV(ADSC) | _BV(ADIF))) != (_BV(ADEN) | _BV(ADIF)))
        return;

    const uint16_t sum = temperatureAdcSum + ADCW;
    const uint8_t count = temperatureAdcCount + 1;
    if (count >= 32) {
        const uint8_t index = temperatureAdcPosition;
        // Scale 32 ten-bit samples to the sensor tables' 12-bit range.
        temperatureAdcValues[index] = sum >> 3;
        temperatureAdcPublishedAt[index] = HAL::timeInMilliseconds();
        temperatureAdcReady[index] = true;
        temperatureAdcSum = 0;
        temperatureAdcCount = 0;

        const uint8_t next = index < TemperatureAdcCount - 1 ? index + 1 : 0;
        temperatureAdcPosition = next;
        const uint8_t channel = pgm_read_byte(&temperatureAdcChannels[next]);
        if (channel & 8)
            ADCSRB |= _BV(MUX5);
        else
            ADCSRB &= ~_BV(MUX5);
        ADMUX = (ADMUX & ~(0x1F)) | (channel & 7);
    } else {
        temperatureAdcSum = sum;
        temperatureAdcCount = count;
    }
    // ADIF is write-one-to-clear; acknowledge the old result and start the next.
    ADCSRA |= _BV(ADIF) | _BV(ADSC);
}

volatile unsigned long watchdogHeartbeat = 0;
volatile bool watchdogActive = false;

// Separate bytes allow each ISR to post an event without a shared bit update.
volatile bool scanTick = false;
volatile bool compensationTick = false;
volatile bool featureTick = false;
volatile bool temperatureTick = false;

bool takeTick(volatile bool& pending) {
    if (!pending)
        return false;
    pending = false;
    return true;
}

inline void pingWatchdog() {
    if (watchdogActive && HAL::timeInMilliseconds() - watchdogHeartbeat < WATCHDOG_MAIN_LOOP_TIMEOUT)
        WRITE(WATCHDOG_PIN, READ(WATCHDOG_PIN) ? 0 : 1);
}
} // namespace

bool HAL::takeScanTick() { return takeTick(scanTick); }
bool HAL::takeCompensationTick() { return takeTick(compensationTick); }
bool HAL::takeFeatureTick() { return takeTick(featureTick); }
bool HAL::takeTemperatureTick() { return takeTick(temperatureTick); }
bool HAL::temperatureTickPending() { return temperatureTick; }
bool HAL::watchdogEnabled() { return watchdogActive; }

HAL::HAL() {
    //ctor
} // HAL

HAL::~HAL() {
    //dtor
} // ~HAL

uint16_t HAL::integerSqrt(int32_t a) {
    // http://www.mikrocontroller.net/articles/AVR_Arithmetik#32_Bit_.2F_32_Bit
    //-----------------------------------------------------------
    // Fast and short 32 bits AVR sqrt routine, avr-gcc ABI compliant
    // R25:R24 = SQRT (R25:R24:R23:R22) rounded to the
    // nearest integer (0.5 rounds up)
    // Destroys R18-R19,R22-R23,R26-R27
    // Cycles incl call & ret = 265-310
    // Stack incl call = 2-3
    //-----------------------------------------------------------

    uint16_t b;

    __asm volatile(
        "ldi   R19, 0xc0 \n\t"
        "clr   R18 \n\t" // rotation mask in R19:R18
        "ldi   R27, 0x40 \n\t"
        "sub   R26, R26 \n\t" // developing sqrt in R27:R26, C=0
        "1:  brcs  2f \n\t"   // C --> Bit is always 1
        "cp    %C1, R26 \n\t"
        "cpc   %D1, R27 \n\t" // Does test value fit?
        "brcs  3f \n\t"       // C --> nope, bit is 0
        "2:  sub   %C1, R26 \n\t"
        "sbc   %D1, R27 \n\t" // Adjust argument for next bit
        "or    R26, R18 \n\t"
        "or    R27, R19 \n\t" // Set bit to 1
        "3:  lsr   R19 \n\t"
        "ror   R18 \n\t" // Shift right mask, C --> end loop
        "eor   R27, R19 \n\t"
        "eor   R26, R18 \n\t" // Shift right only test bit in result
        "rol   %A1 \n\t"      // Bit 0 only set if end of loop
        "rol   %B1 \n\t"
        "rol   %C1 \n\t"
        "rol   %D1 \n\t"    // Shift left remaining argument (C used at 1:)
        "sbrs  %A1, 0 \n\t" // Skip if 15 bits developed
        "rjmp  1b \n\t"     // Develop 15 bits of the sqrt
        "brcs  4f \n\t"     // C--> Last bits always 1
        "cp    R26, %C1 \n\t"
        "cpc   R27, %D1 \n\t"     // Test for last bit 1
        "brcc  5f \n\t"           // NC --> bit is 0
        "4:  sbc   %B1, R19 \n\t" // Subtract C (any value from 1 to 0x7f will do)
        "sbc   %C1, R26 \n\t"
        "sbc   %D1, R27 \n\t" // Update argument for test
        "inc   R26 \n\t"      // Last bit is 1
        "5:  lsl   %B1 \n\t"  // Only bit 7 matters
        "rol   %C1 \n\t"
        "rol   %D1 \n\t" // Remainder * 2 + C
        "brcs  6f \n\t"  // C --> Always round up
        "cp    R26, %C1 \n\t"
        "cpc   R27, %D1 \n\t" // C decides rounding
        "6:  adc   R26, R19 \n\t"
        "adc   R27, R19 \n\t" // Round up if C (R19=0)
        "mov   %B0, R27 \n\t" // return in R25:R24 for avr-gcc ABI compliance
        "mov   %A0, R26 \n\t"
        : "=r"(b)
        : "r"(a)
        : "r18", "r19", "r27", "r26");
    return b;
} // integerSqrt

const uint16_t fast_div_lut[17] PROGMEM = { 0, F_CPU / 4096, F_CPU / 8192, F_CPU / 12288, F_CPU / 16384, F_CPU / 20480, F_CPU / 24576, F_CPU / 28672, F_CPU / 32768, F_CPU / 36864, F_CPU / 40960, F_CPU / 45056, F_CPU / 49152, F_CPU / 53248, F_CPU / 57344, F_CPU / 61440, F_CPU / 65536 };

const uint16_t slow_div_lut[257] PROGMEM = { 0, 0, 0, 0, 0, 0, 0, 0, F_CPU / 256, F_CPU / 288, F_CPU / 320, F_CPU / 352, F_CPU / 384, F_CPU / 416, F_CPU / 448, F_CPU / 480, F_CPU / 512, F_CPU / 544, F_CPU / 576, F_CPU / 608, F_CPU / 640, F_CPU / 672, F_CPU / 704, F_CPU / 736, F_CPU / 768, F_CPU / 800, F_CPU / 832, F_CPU / 864, F_CPU / 896, F_CPU / 928, F_CPU / 960, F_CPU / 992, F_CPU / 1024, F_CPU / 1056, F_CPU / 1088, F_CPU / 1120, F_CPU / 1152, F_CPU / 1184, F_CPU / 1216, F_CPU / 1248, F_CPU / 1280, F_CPU / 1312, F_CPU / 1344, F_CPU / 1376, F_CPU / 1408, F_CPU / 1440, F_CPU / 1472, F_CPU / 1504, F_CPU / 1536, F_CPU / 1568, F_CPU / 1600, F_CPU / 1632, F_CPU / 1664, F_CPU / 1696, F_CPU / 1728, F_CPU / 1760, F_CPU / 1792, F_CPU / 1824, F_CPU / 1856, F_CPU / 1888, F_CPU / 1920, F_CPU / 1952, F_CPU / 1984, F_CPU / 2016, F_CPU / 2048, F_CPU / 2080, F_CPU / 2112, F_CPU / 2144, F_CPU / 2176, F_CPU / 2208, F_CPU / 2240, F_CPU / 2272, F_CPU / 2304, F_CPU / 2336, F_CPU / 2368, F_CPU / 2400, F_CPU / 2432, F_CPU / 2464, F_CPU / 2496, F_CPU / 2528, F_CPU / 2560, F_CPU / 2592, F_CPU / 2624, F_CPU / 2656, F_CPU / 2688, F_CPU / 2720, F_CPU / 2752, F_CPU / 2784, F_CPU / 2816, F_CPU / 2848, F_CPU / 2880, F_CPU / 2912, F_CPU / 2944, F_CPU / 2976, F_CPU / 3008, F_CPU / 3040, F_CPU / 3072, F_CPU / 3104, F_CPU / 3136, F_CPU / 3168, F_CPU / 3200, F_CPU / 3232, F_CPU / 3264, F_CPU / 3296, F_CPU / 3328, F_CPU / 3360, F_CPU / 3392, F_CPU / 3424, F_CPU / 3456, F_CPU / 3488, F_CPU / 3520, F_CPU / 3552, F_CPU / 3584, F_CPU / 3616, F_CPU / 3648, F_CPU / 3680, F_CPU / 3712, F_CPU / 3744, F_CPU / 3776, F_CPU / 3808, F_CPU / 3840, F_CPU / 3872, F_CPU / 3904, F_CPU / 3936, F_CPU / 3968, F_CPU / 4000, F_CPU / 4032, F_CPU / 4064, F_CPU / 4096, F_CPU / 4128, F_CPU / 4160, F_CPU / 4192, F_CPU / 4224, F_CPU / 4256, F_CPU / 4288, F_CPU / 4320, F_CPU / 4352, F_CPU / 4384, F_CPU / 4416, F_CPU / 4448, F_CPU / 4480, F_CPU / 4512, F_CPU / 4544, F_CPU / 4576, F_CPU / 4608, F_CPU / 4640, F_CPU / 4672, F_CPU / 4704, F_CPU / 4736, F_CPU / 4768, F_CPU / 4800, F_CPU / 4832, F_CPU / 4864, F_CPU / 4896, F_CPU / 4928, F_CPU / 4960, F_CPU / 4992, F_CPU / 5024, F_CPU / 5056, F_CPU / 5088, F_CPU / 5120, F_CPU / 5152, F_CPU / 5184, F_CPU / 5216, F_CPU / 5248, F_CPU / 5280, F_CPU / 5312, F_CPU / 5344, F_CPU / 5376, F_CPU / 5408, F_CPU / 5440, F_CPU / 5472, F_CPU / 5504, F_CPU / 5536, F_CPU / 5568, F_CPU / 5600, F_CPU / 5632, F_CPU / 5664, F_CPU / 5696, F_CPU / 5728, F_CPU / 5760, F_CPU / 5792, F_CPU / 5824, F_CPU / 5856, F_CPU / 5888, F_CPU / 5920, F_CPU / 5952, F_CPU / 5984, F_CPU / 6016, F_CPU / 6048, F_CPU / 6080, F_CPU / 6112, F_CPU / 6144, F_CPU / 6176, F_CPU / 6208, F_CPU / 6240, F_CPU / 6272, F_CPU / 6304, F_CPU / 6336, F_CPU / 6368, F_CPU / 6400, F_CPU / 6432, F_CPU / 6464, F_CPU / 6496, F_CPU / 6528, F_CPU / 6560, F_CPU / 6592, F_CPU / 6624, F_CPU / 6656, F_CPU / 6688, F_CPU / 6720, F_CPU / 6752, F_CPU / 6784, F_CPU / 6816, F_CPU / 6848, F_CPU / 6880, F_CPU / 6912, F_CPU / 6944, F_CPU / 6976, F_CPU / 7008, F_CPU / 7040, F_CPU / 7072, F_CPU / 7104, F_CPU / 7136, F_CPU / 7168, F_CPU / 7200, F_CPU / 7232, F_CPU / 7264, F_CPU / 7296, F_CPU / 7328, F_CPU / 7360, F_CPU / 7392, F_CPU / 7424, F_CPU / 7456, F_CPU / 7488, F_CPU / 7520, F_CPU / 7552, F_CPU / 7584, F_CPU / 7616, F_CPU / 7648, F_CPU / 7680, F_CPU / 7712, F_CPU / 7744, F_CPU / 7776, F_CPU / 7808, F_CPU / 7840, F_CPU / 7872, F_CPU / 7904, F_CPU / 7936, F_CPU / 7968, F_CPU / 8000, F_CPU / 8032, F_CPU / 8064, F_CPU / 8096, F_CPU / 8128, F_CPU / 8160, F_CPU / 8192 };

/** \brief approximates division of F_CPU/divisor
In the stepper interrupt a division is needed, which is a slow operation.
The result is used for timer calculation where small errors are ok. This
function uses lookup tables to find a fast approximation of the result.
*/
int32_t HAL::CPUDivU2(unsigned int divisor) {
    int32_t res;
    unsigned short table;

    if (divisor < 8192) {
        if (divisor < 512) {
            if (divisor < 10)
                divisor = 10;
            return Div4U2U(F_CPU, divisor); // These entries have overflows in lookuptable!
        }
        table = (unsigned short)&slow_div_lut[0];
        __asm volatile( // needs 64 ticks neu 49 Ticks
            "mov r18,%A1 \n\t"
            "andi r18,31 \n\t" // divisor & 31 in r18
            "lsr %B1 \n\t"     // divisor >> 4
            "ror %A1 \n\t"
            "lsr %B1 \n\t"
            "ror %A1 \n\t"
            "lsr %B1 \n\t"
            "ror %A1 \n\t"
            "lsr %B1 \n\t"
            "ror %A1 \n\t"
            "andi %A1,254 \n\t"
            "add %A2,%A1 \n\t" // table+divisor>>3
            "adc %B2,%B1 \n\t"
            "lpm %A0,Z+ \n\t"  // y0 in res
            "lpm %B0,Z+ \n\t"  // %C0,%D0 are 0
            "movw r4,%A0 \n\t" // y0 nach gain (r4-r5)
            "lpm r0,Z+ \n\t"   // gain = gain-y1
            "sub r4,r0 \n\t"
            "lpm r0,Z+ \n\t"
            "sbc r5,r0 \n\t"
            "mul r18,r4 \n\t"  // gain*(divisor & 31)
            "movw %A1,r0 \n\t" // divisor not needed any more, use for byte 0,1 of result
            "mul r18,r5 \n\t"
            "add %B1,r0 \n\t"
            "mov %A2,r1 \n\t"
            "lsl %A1 \n\t"
            "rol %B1 \n\t"
            "rol %A2 \n\t"
            "lsl %A1 \n\t"
            "rol %B1 \n\t"
            "rol %A2 \n\t"
            "lsl %A1 \n\t"
            "rol %B1 \n\t"
            "rol %A2 \n\t"
            "sub %A0,%B1 \n\t"
            "sbc %B0,%A2 \n\t"
            "clr %C0 \n\t"
            "clr %D0 \n\t"
            "clr r1 \n\t"
            : "=&r"(res), "=&d"(divisor), "=&z"(table)
            : "1"(divisor), "2"(table)
            : "r18", "r4", "r5");
        return res;
        /*unsigned short adr0 = (unsigned short)&slow_div_lut+(divisor>>4)&1022;
        long y0=    pgm_read_dword_near(adr0);
        long gain = y0-pgm_read_dword_near(adr0+2);
        return y0-((gain*(divisor & 31))>>5);*/
    } else {
        table = (unsigned short)&fast_div_lut[0];
        __asm volatile( // needs 49 ticks
            "movw r18,%A1 \n\t"
            "andi r19,15 \n\t" // divisor & 4095 in r18,r19
            "lsr %B1 \n\t"     // divisor >> 3, then %B1 is 2*(divisor >> 12)
            "lsr %B1 \n\t"
            "lsr %B1 \n\t"
            "andi %B1,254 \n\t"
            "add %A2,%B1 \n\t" // table+divisor>>11
            "adc %B2,r1 \n\t"  //
            "lpm %A0,Z+ \n\t"  // y0 in res
            "lpm %B0,Z+ \n\t"
            "movw r4,%A0 \n\t" // y0 to gain (r4-r5)
            "lpm r0,Z+ \n\t"   // gain = gain-y1
            "sub r4,r0 \n\t"
            "lpm r0,Z+ \n\t"
            "sbc r5,r0 \n\t"   // finished - result has max. 16 bit
            "mul r18,r4 \n\t"  // gain*(divisor & 4095)
            "movw %A1,r0 \n\t" // divisor not needed any more, use for byte 0,1 of result
            "mul r19,r5 \n\t"
            "mov %A2,r0 \n\t" // %A2 = byte 3 of result
            "mul r18,r5 \n\t"
            "add %B1,r0 \n\t"
            "adc %A2,r1 \n\t"
            "mul r19,r4 \n\t"
            "add %B1,r0 \n\t"
            "adc %A2,r1 \n\t"
            "andi %B1,240 \n\t" // >> 12
            "swap %B1 \n\t"
            "swap %A2 \r\n"
            "mov %A1,%A2 \r\n"
            "andi %A1,240 \r\n"
            "or %B1,%A1 \r\n"
            "andi %A2,15 \r\n"
            "sub %A0,%B1 \n\t"
            "sbc %B0,%A2 \n\t"
            "clr %C0 \n\t"
            "clr %D0 \n\t"
            "clr r1 \n\t"
            : "=&r"(res), "=&d"(divisor), "=&z"(table)
            : "1"(divisor), "2"(table)
            : "r18", "r19", "r4", "r5");
        return res;
        /*
        // The asm mimics the following code
        unsigned short adr0 = (unsigned short)&fast_div_lut+(divisor>>11)&254;
        unsigned short y0=  pgm_read_word_near(adr0);
        unsigned short gain = y0-pgm_read_word_near(adr0+2);
        return y0-(((long)gain*(divisor & 4095))>>12);*/
    }

} // CPUDivU2

void HAL::setupTimer() {
    EXTRUDER_TCCR = 0;                       // Normal mode on the shared timer.
    EXTRUDER_TIMSK &= ~_BV(EXTRUDER_OCIE);   // Arm Compare A when Advance has work.

    PWM_TCCR = 0; // Setup PWM interrupt
    PWM_OCR = 64;
    PWM_TIMSK |= (1 << PWM_OCIE);

    TCCR1A = 0; // Stepper timer 1 interrupt to no prescale CTC mode
    TCCR1C = 0;
    TIMSK1 = 0;
    TCCR1B = (_BV(WGM12) | _BV(CS10)); // no prescaler == 0.0625 usec tick | 001 = clk/1
    OCR1A = 65500;                     // start off with a slow frequency.
    TIMSK1 |= (1 << OCIE1A);           // Enable interrupt

#ifdef RGB_LIGHT_R_PIN
    // Configure Timer 4 for RGB-PWM
    TCCR4A = 0; // clear Register
    TCCR4B = 0;

    ICR4 = 20480; // set frequency to 100Hz = 10ms

    TCCR4A |= (1 << WGM41);                              // Fast PWM
    TCCR4B |= (1 << WGM42) | (1 << WGM43) | (1 << CS41); // Prescaler = 8

    SET_OUTPUT(RGB_LIGHT_R_PIN); // PH3 - Pin 15 ( OC4A )
    SET_OUTPUT(RGB_LIGHT_G_PIN); // PH4 - Pin 16 ( OC4B )
    SET_OUTPUT(RGB_LIGHT_B_PIN); // PH5 - Pin 17 ( OC4C )

    TCCR4A |= (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1); // enable fast PWM

    OCR4A = 0; // default R = 0 at startup
    OCR4B = 0; // default G = 0 at startup
    OCR4C = 0; // default B = 0 at startup
#endif // RGB_LIGHT_R_PIN
} // setupTimer

void HAL::showStartReason() {
    // Check startup - does nothing if bootloader sets MCUSR to 0
    uint8_t mcu = MCUSR;

    if (MCUSR != 0) //Printer::debugInfo()
    {
        if (mcu & 1)
            Com::printInfoFLN(Com::tPowerUp);
        if (mcu & 2)
            Com::printInfoFLN(Com::tExternalReset);
        if (mcu & 4)
            Com::printInfoFLN(Com::tBrownOut);
        if (mcu & 8)
            Com::printInfoFLN(Com::tWatchdog);
        if (mcu & 32)
            Com::printInfoFLN(Com::tSoftwareReset);
    } else {
        Com::printInfoFLN(Com::tUnknownReset);
    }
    MCUSR = 0;
} // showStartReason

int HAL::getFreeRam() {
    int freeram = 0;
    InterruptProtectedBlock noInts; //BEGIN_INTERRUPT_PROTECTED
    uint8_t *heapptr, *stackptr;
    heapptr = (uint8_t*)malloc(4); // get heap pointer
    free(heapptr);                 // free up the memory again (sets heapptr to 0)
    stackptr = (uint8_t*)(SP);     // save value of stack pointer
    freeram = (int)stackptr - (int)heapptr;
    //END_INTERRUPT_PROTECTED
    return freeram;

} // getFreeRam

void (*resetFunc)(void) = 0; // declare reset function @ address 0

void HAL::resetHardware() {
    resetFunc();
} // resetHardware

uint16_t HAL::temperatureAdcSample(uint8_t index) {
    // The PWM interrupt can publish between the two byte loads on AVR.
    InterruptProtectedBlock lock;
    return temperatureAdcValues[index];
}

bool HAL::temperatureAdcSample(uint8_t index, uint16_t& sample) {
    if (index >= TemperatureAdcCount) {
        sample = 0;
        return false;
    }
    uint32_t publishedAt;
    {
        // Value, readiness and timestamp describe the same complete batch.
        InterruptProtectedBlock lock;
        sample = temperatureAdcValues[index];
        if (!temperatureAdcReady[index])
            return false;
        publishedAt = temperatureAdcPublishedAt[index];
    }
    // Read the clock after the snapshot so a just-published timestamp is not
    // in the future. Unsigned subtraction works across ordinary millis wrap.
    if (static_cast<uint32_t>(timeInMilliseconds() - publishedAt) <= temperatureAdcMaxAgeMs)
        return true;
    {
        // Expired data must not become valid again on a later clock wrap.
        // Do not invalidate a newer batch published since the snapshot.
        InterruptProtectedBlock lock;
        if (temperatureAdcPublishedAt[index] == publishedAt)
            temperatureAdcReady[index] = false;
    }
    return false;
}

void HAL::analogStart() {
    // Called during boot before enabling Timer0 COMPB.
    ADMUX = _BV(REFS0); // Use AVCC as the temperature ADC reference.
    temperatureAdcPosition = 0;
    temperatureAdcCount = 0;
    temperatureAdcSum = 0;
    for (uint8_t i = 0; i < TemperatureAdcCount; i++) {
        temperatureAdcValues[i] = 0;
        temperatureAdcPublishedAt[i] = 0;
        temperatureAdcReady[i] = false;
    }
    ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADIF) | ANALOG_PRESCALER;
    while (ADCSRA & _BV(ADSC)) { } // wait for conversion
    /* ADCW must be read once, otherwise the next result is wrong. */
    uint16_t dummyADCResult;
    dummyADCResult = ADCW;
    (void)dummyADCResult;
    // The Timer0 COMPB handler services the conversion loop.
    uint8_t channel = pgm_read_byte(&temperatureAdcChannels[temperatureAdcPosition]);

    if (channel & 8) // Reading channel 0-7 or 8-15?
        ADCSRB |= _BV(MUX5);
    else
        ADCSRB &= ~_BV(MUX5);

    ADMUX = (ADMUX & ~(0x1F)) | (channel & 7);
    ADCSRA |= _BV(ADIF) | _BV(ADSC); // Discard dummy result and start the first channel.

} // analogStart

//initialize watchdog
void HAL::WDT_Init(void) {
    // Just to be safe since we can not clear WDE if WDRF is set
    MCUSR &= ~(1 << WDRF);
    //disable interrupts
    cli();
    //reset watchdog
    wdt_reset();
    //set up WDT interrupt
    WDTCSR = (1 << WDCE) | (1 << WDE);
    // timeout in XXX millisecond, disable reset mode. Must be done in one operation --> S66  http://www.atmel.com/Images/Atmel-2549-8-bit-AVR-Microcontroller-ATmega640-1280-1281-2560-2561_datasheet.pdf
    //WDTCSR = 1<<WDP0; //32ms
    WDTCSR = 0; //16ms
    //Enable global interrupts
    sei();

    // enable watchdog interrupt only mode
    WDTCSR |= _BV(WDIE);
}

//Watchdog timeout ISR
ISR(WDT_vect) {
    pingWatchdog();
    WDTCSR |= (1 << WDIE); //Nibbels: nächstes mal kein Reset durch internen Watchdog, sondern wieder dieser interrupt.
    compensationTick = true; //Tell commandloop that 16ms have passed
}

// Watchdog support
void HAL::startWatchdog() {
    Com::printFLN(Com::tStartWatchdog);
    // external watchdog
    SET_OUTPUT(WATCHDOG_PIN);
    tellWatchdogOk();
    watchdogActive = true;
    pingWatchdog();      //Nibbels: Hier macht der mehr sinn!
    HAL::WDT_Init();     //Nibbels: use watchdogtimer to test var and trigger
} // startWatchdog

void HAL::stopWatchdog() {
    watchdogActive = false;
                              // external watchdog
    WRITE(WATCHDOG_PIN, LOW); //Nibbels: In case you stop the watchdog it has to be set floating. thatwhy disable internal pullup as this will make the state like OUTPUT-high. This is not really needed as this function does not do anything when starting the arduino. rightnow at start it sets a pin to input which is already input.
    SET_INPUT(WATCHDOG_PIN);
} // stopWatchdog

void HAL::tellWatchdogOk() {
    const auto now = HAL::timeInMilliseconds();
    InterruptProtectedBlock lock;
    watchdogHeartbeat = now;
}

// ================== Interrupt handling ======================
/** \brief Sets the timer 1 compare value to delay ticks.
This function sets the OCR1A compare counter to get the next interrupt
at delay ticks measured from the last interrupt. delay must be << 2^24 */
inline void setTimer(uint32_t delay) {
    __asm volatile(
        "cli \n\t"
        "tst %C[delay] \n\t" //if(delay<65536) {
        "brne else%= \n\t"   // Still > 65535
        "cpi %B[delay],255 \n\t"
        "breq else%= \n\t"        // delay <65280
        "sts stepperWait,r1 \n\t" // stepperWait = 0;
        "sts stepperWait+1,r1 \n\t"
        "sts stepperWait+2,r1 \n\t"
        "lds %C[delay],%[time] \n\t" // Read TCNT1
        "lds %D[delay],%[time]+1 \n\t"
        "ldi r18,100 \n\t" // Add 100 to TCNT1
        "add %C[delay],r18 \n\t"
        "adc %D[delay],r1 \n\t"
        "cp %A[delay],%C[delay] \n\t" // delay<TCNT1+1
        "cpc %B[delay],%D[delay] \n\t"
        "brcc exact%= \n\t"
        "sts %[ocr]+1,%D[delay] \n\t" //  OCR1A = TCNT1+100;
        "sts %[ocr],%C[delay] \n\t"
        "rjmp end%= \n\t"
        "exact%=: sts %[ocr]+1,%B[delay] \n\t" //  OCR1A = delay;
        "sts %[ocr],%A[delay] \n\t"
        "rjmp end%= \n\t"
        "else%=: subi   %B[delay], 0x80 \n\t" //} else { stepperWait = delay-32768;
        "sbci   %C[delay], 0x00 \n\t"
        "sts stepperWait,%A[delay] \n\t"
        "sts stepperWait+1,%B[delay] \n\t"
        "sts stepperWait+2,%C[delay] \n\t"
        "ldi    %D[delay], 0x80 \n\t" //OCR1A = 32768;
        "sts    %[ocr]+1, %D[delay] \n\t"
        "sts    %[ocr], r1 \n\t"
        "end%=: \n\t"
        : [ delay ] "=&d"(delay)                                                            // Output
        : "0"(delay), [ ocr ] "i"(_SFR_MEM_ADDR(OCR1A)), [ time ] "i"(_SFR_MEM_ADDR(TCNT1)) // Input
        : "r18");                                                                           // Clobber

    /* // Assembler above replaced this code
      if(delay<65280) {
        stepperWait = 0;
        unsigned int count = TCNT1+100;
        if(delay<count)
          OCR1A = count;
        else
          OCR1A = delay;
      } else {
        stepperWait = delay-32768;
        OCR1A = 32768;
      }*/

} // setTimer

volatile long __attribute__((used)) stepperWait = 0;

/** \brief Timer interrupt routine to drive the stepper motors.
*/
ISR(TIMER1_COMPA_vect) {
    uint8_t doExit;
    __asm volatile(
        "ldi %[ex],0 \n\t"
        "lds r23,stepperWait+2 \n\t"
        "tst r23 \n\t"     //if(stepperWait<65536) {
        "brne else%= \n\t" // Still > 65535
        "lds r23,stepperWait+1 \n\t"
        "tst r23 \n\t"
        "brne last%= \n\t" // Still not 0, go ahead
        "lds r22,stepperWait \n\t"
        "breq end%= \n\t" // stepperWait is 0, do your work
        "last%=: \n\t"
        "sts %[ocr]+1,r23 \n\t" //  OCR1A = stepper wait;
        "sts %[ocr],r22 \n\t"
        "sts stepperWait,r1 \n\t"
        "sts stepperWait+1,r1 \n\t"
        "rjmp end1%= \n\t"
        "else%=: lds r22,stepperWait+1 \n\t" //} else { stepperWait = stepperWait-32768;
        "subi   r22, 0x80 \n\t"
        "sbci   r23, 0x00 \n\t"
        "sts stepperWait+1,r22 \n\t" // ocr1a stays 32768
        "sts stepperWait+2,r23 \n\t"
        "end1%=: ldi %[ex],1 \n\t"
        "end%=: \n\t"
        : [ ex ] "=&d"(doExit)
        : [ ocr ] "i"(_SFR_MEM_ADDR(OCR1A))
        : "r22", "r23");
    if (doExit)
        return;

    cbi(TIMSK1, OCIE1A); // prevent retrigger timer by disabling timer interrupt. Should be faster than guarding with insideTimer1.

    OCR1A = 61000;

    if (!PrintLine::hasActiveQueuedMove() && !PrintLine::directMoveActive()) {
        StepExecutor::stepSlowedZCompensation(); // Z is free here.
        // Wait if the z-CMP is under heavy workload
        if (StepExecutor::needCmpWait()) {
            HAL::forbidInterrupts();
            setTimer(3000);
            sbi(TIMSK1, OCIE1A);

            return;
        }
    }

    if (PrintLine::directMoveActive()) {
        setTimer(StepExecutor::performDirectMove());
        sbi(TIMSK1, OCIE1A);

        return;
    }

    if (PrintLine::hasLines() && Printer::pause.allowsQueueAdvance()) {
        setTimer(StepExecutor::performQueueMove());
        sbi(TIMSK1, OCIE1A);

        return;
    }

    Printer::extrusionOutput.onIdleTick();

    stepperWait = 0; // Important because of optimization in asm at begin
    OCR1A = 3000;

    sbi(TIMSK1, OCIE1A);
} // ISR(TIMER1_COMPA_vect)

// Heater and secondary fan PWM use the stock duty cycle resolution.
constexpr uint8_t heaterPwmStep = 2;
constexpr uint8_t heaterPwmMask = 0xFE;
constexpr uint8_t coolerPwmStep = 1;
constexpr uint8_t coolerPwmMask = 0xFF;
constexpr uint8_t partFanPwmStep = 1;
constexpr uint8_t partFanPwmMask = 0xFF;
static_assert(PART_FAN_DEFAULT_PWM_SPEED_DIVISOR > 0, "Part fan divisor must be positive");

/**
* PART FAN CONFIGURATION
*/
namespace {
// Hotends, bed, board fan and part fan, in the existing PWM channel order.
volatile uint8_t requestedDuty[NUM_EXTRUDER + 3];
constexpr uint8_t boardFanChannel = NUM_EXTRUDER + 1;
constexpr uint8_t partFanChannel = NUM_EXTRUDER + 2;
uint8_t requestedFanSpeed = 0;
volatile uint8_t fanKickstart = 0;
volatile uint8_t fanMode = PART_FAN_MODE_PWM;
volatile uint8_t fanDivisor = PART_FAN_DEFAULT_PWM_SPEED_DIVISOR;
uint8_t fanMaxDuty = PART_FAN_PWM_MAX;
uint8_t fanMinDuty = PART_FAN_PWM_MIN;
} // namespace

void HAL::resetPwmOutputs() {
    for (auto& duty : requestedDuty)
        duty = 0;
}

uint8_t HAL::outputDuty(uint8_t channel) { return requestedDuty[channel]; }
void HAL::setHeaterDuty(uint8_t channel, uint8_t duty) { requestedDuty[channel] = duty; }
void HAL::setBoardFan(bool enabled) { requestedDuty[boardFanChannel] = enabled ? 255 : 0; }
uint8_t HAL::partFanSpeed() { return requestedFanSpeed; }
uint8_t HAL::partFanDivisor() { return fanDivisor; }
uint8_t HAL::partFanMode() { return fanMode; }
uint8_t HAL::partFanMinDuty() { return fanMinDuty; }
uint8_t HAL::partFanMaxDuty() { return fanMaxDuty; }

void HAL::setPartFanDutyRange(uint8_t minimum, uint8_t maximum) {
    fanMinDuty = minimum;
    fanMaxDuty = maximum;
}

void HAL::setPartFanDivisor(uint8_t divisor) {
    InterruptProtectedBlock lock;
    if (!divisor)
        divisor = 1;
    fanDivisor = divisor <= PART_FAN_MODE_MAX ? divisor : PART_FAN_DEFAULT_PWM_SPEED_DIVISOR;
}

void HAL::setPartFanMode(uint8_t mode) {
    fanMode = mode ? PART_FAN_MODE_PDM : PART_FAN_MODE_PWM;
}

void HAL::setPartFanSpeed(uint8_t speed) {
    requestedFanSpeed = speed;
    uint8_t duty = speed;
    if (speed > 0 && speed < 255) {
        const long scaled = long(speed) * (fanMaxDuty - fanMinDuty) / 255 + fanMinDuty;
        duty = constrain(scaled, 0, 255);
    }

    InterruptProtectedBlock lock;
    if (!fanKickstart && duty > requestedDuty[partFanChannel] && speed < PART_FAN_KICKSTART_THRESHOLD)
        fanKickstart = PART_FAN_KICKSTART_TIME_BOOST / 10;
    requestedDuty[partFanChannel] = duty;
}


#define pulseDensityModulate(pin, density, error, invert) \
    { \
        uint8_t carry; \
        carry = error + (invert ? 255 - density : density); \
        WRITE(pin, (carry < error)); \
        error = carry; \
    }
/**
This timer is called 3906 times per second. It is used to update pwm values for heater and some other frequent jobs.
*/
ISR(PWM_TIMER_VECTOR) {
    static uint8_t pwm_count_heater = 0;
    static uint8_t pwm_count_cooler = 0;
    static uint8_t pwm_count_part_fan = 0;

    static uint8_t pwm_pos_set[NUM_EXTRUDER + 3];
#if ((defined(EXT0_HEATER_PIN) && EXT0_HEATER_PIN > -1 && EXT0_EXTRUDER_COOLER_PIN > -1) || (NUM_EXTRUDER > 1 && EXT1_EXTRUDER_COOLER_PIN > -1 && EXT1_EXTRUDER_COOLER_PIN != EXT0_EXTRUDER_COOLER_PIN))
    static uint8_t pwm_cooler_pos_set[NUM_EXTRUDER];
#endif
    PWM_OCR += 64;

    if (pwm_count_heater == 0) {
#if EXT0_HEATER_PIN > -1
        if ((pwm_pos_set[0] = (requestedDuty[0] & heaterPwmMask)) > 0)
            WRITE(EXT0_HEATER_PIN, HIGH);
#endif // EXT0_HEATER_PIN>-1

#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN > -1 && NUM_EXTRUDER > 1
        if ((pwm_pos_set[1] = (requestedDuty[1] & heaterPwmMask)) > 0)
            WRITE(EXT1_HEATER_PIN, HIGH);
#endif // defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1 && NUM_EXTRUDER>1

        if ((pwm_pos_set[NUM_EXTRUDER] = (requestedDuty[NUM_EXTRUDER] & heaterPwmMask)) > 0)
            WRITE(HEATED_BED_HEATER_PIN, HIGH);
    }

    if (pwm_count_cooler == 0) {
#if EXT0_HEATER_PIN > -1 && EXT0_EXTRUDER_COOLER_PIN > -1
        if ((pwm_cooler_pos_set[0] = extruder[0].coolerPWM) > 0)
            WRITE(EXT0_EXTRUDER_COOLER_PIN, 1);
#endif // EXT0_HEATER_PIN > -1 && EXT0_EXTRUDER_COOLER_PIN > -1

#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN > -1 && NUM_EXTRUDER > 1
#if EXT1_EXTRUDER_COOLER_PIN > -1 && EXT1_EXTRUDER_COOLER_PIN != EXT0_EXTRUDER_COOLER_PIN
        if ((pwm_cooler_pos_set[1] = extruder[1].coolerPWM) > 0)
            WRITE(EXT1_EXTRUDER_COOLER_PIN, 1);
#endif // EXT1_EXTRUDER_COOLER_PIN > -1 && EXT1_EXTRUDER_COOLER_PIN != EXT0_EXTRUDER_COOLER_PIN
#endif // defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN > -1 && NUM_EXTRUDER > 1

#if FAN_BOARD_PIN > -1
        if ((pwm_pos_set[NUM_EXTRUDER + 1] = (requestedDuty[NUM_EXTRUDER + 1] & coolerPwmMask)) > 0)
            WRITE(FAN_BOARD_PIN, 1);
#endif // FAN_BOARD_PIN>-1
    }

#if EXT0_HEATER_PIN > -1
    if (pwm_pos_set[0] == pwm_count_heater && pwm_pos_set[0] != heaterPwmMask)
        WRITE(EXT0_HEATER_PIN, LOW);
#if EXT0_EXTRUDER_COOLER_PIN > -1
    if (pwm_cooler_pos_set[0] == pwm_count_cooler && pwm_cooler_pos_set[0] != coolerPwmMask)
        WRITE(EXT0_EXTRUDER_COOLER_PIN, 0);
#endif // #if EXT0_EXTRUDER_COOLER_PIN > -1
#endif // #if EXT0_HEATER_PIN>-1

#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN > -1 && NUM_EXTRUDER > 1
    if (pwm_pos_set[1] == pwm_count_heater && pwm_pos_set[1] != heaterPwmMask)
        WRITE(EXT1_HEATER_PIN, LOW);
#if EXT1_EXTRUDER_COOLER_PIN > -1 && EXT1_EXTRUDER_COOLER_PIN != EXT0_EXTRUDER_COOLER_PIN
    if (pwm_cooler_pos_set[1] == pwm_count_cooler && pwm_cooler_pos_set[1] != coolerPwmMask)
        WRITE(EXT1_EXTRUDER_COOLER_PIN, 0);
#endif // EXT1_EXTRUDER_COOLER_PIN>-1 && EXT1_EXTRUDER_COOLER_PIN!=EXT0_EXTRUDER_COOLER_PIN
#endif // defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1 && NUM_EXTRUDER>1

    if (pwm_pos_set[NUM_EXTRUDER] == pwm_count_heater && pwm_pos_set[NUM_EXTRUDER] != heaterPwmMask)
        WRITE(HEATED_BED_HEATER_PIN, LOW);

#if FAN_BOARD_PIN > -1
    if (pwm_pos_set[NUM_EXTRUDER + 1] == pwm_count_cooler && pwm_pos_set[NUM_EXTRUDER + 1] != coolerPwmMask)
        WRITE(FAN_BOARD_PIN, 0);
#endif // #if FAN_BOARD_PIN>-1

    // run the heater PWM speed like we were used to:
    pwm_count_heater += heaterPwmStep;
    // run alle the secondary cooler PWM speed like we were used to:
    pwm_count_cooler += coolerPwmStep;

    /**
    PART FAN SPEED CONTROL
    */
    if (fanMode == PART_FAN_MODE_PDM) {
        pulseDensityModulate(FAN_PIN, (fanKickstart ? 255 : requestedDuty[NUM_EXTRUDER + 2]), pwm_pos_set[NUM_EXTRUDER + 2], false);
    } else {
        // divide part fan 15.3Hz PWM by factor:
        static int counterSpeed15_3Div = 0;
        if (++counterSpeed15_3Div >= fanDivisor) {
            counterSpeed15_3Div = 0;

            if (pwm_count_part_fan == 0) {
                //count = 0 -> AN, wenn Powerlevel über Minimum
                //je nach PWM-Mask zählen nur obere bits!
                //je nach PWM-Mask (z.b. 1111 11xx b) gibts spezielle PWM-Steps (z.b. 4 = 100 b) welche den counter nur innerhalb PWM-Mask hochzählen.
                pwm_pos_set[NUM_EXTRUDER + 2] = (requestedDuty[NUM_EXTRUDER + 2] & partFanPwmMask);
                if (pwm_pos_set[NUM_EXTRUDER + 2] > 0)
                    WRITE(FAN_PIN, 1); //AN
            }
            //count = powerlevel, aber nicht powerlevel == max. -> AUS
            uint8_t powerlevel = fanKickstart ? 255 : pwm_pos_set[NUM_EXTRUDER + 2];
            if (powerlevel == pwm_count_part_fan && powerlevel != partFanPwmMask)
                WRITE(FAN_PIN, 0); //AUS

            pwm_count_part_fan += partFanPwmStep;
        }
    }
    /**
    END PART FAN SPEED CONTROL
    */

    static int counter100Periodical = 0; // Approximate a 100ms timer :: blocks pingwatchdog s commandloop if not working
    counter100Periodical++;
    if (counter100Periodical == 196) //halbe 100ms Zeit -> Ping 50ms
    {
        featureTick = true;
    } else if (counter100Periodical >= 391) //(int)(F_CPU/40960))
    {
        counter100Periodical = 0;
        temperatureTick = true;
        featureTick = true; //volle 100ms Zeit -> Ping 50ms
    }

#ifdef RGB_LIGHT_R_PIN
    bool rgb_10ms_change_should_be_now = false;
#endif // RGB_LIGHT_R_PIN

    static char counter10Periodical = 0; // Approximate a 10ms timer :: blocks pingwatchdog s commandloop if not working
    if (++counter10Periodical >= 39) {
        counter10Periodical = 0;
        scanTick = true;
#ifdef RGB_LIGHT_R_PIN
        rgb_10ms_change_should_be_now = true;
#endif // RGB_LIGHT_R_PIN

        //hier her versetzt, um 10ms einstell-auflösung zu erhalten. Darum /10 statt bisher /100 beim Init in setFanSpeed().
        //fan anfangs auf high zum anlaufen, aber das muss beendet werden:
        if (fanKickstart)
            fanKickstart--;
    }

    serviceTemperatureAdc();

    if (pwm_count_heater & 4)
        uid.fastAction();

#ifdef RGB_LIGHT_R_PIN
    if (rgb_10ms_change_should_be_now)
        RgbLighting::advanceFade();
#endif // RGB_LIGHT_R_PIN
} // ISR(PWM_TIMER_VECTOR)

namespace {
constexpr uint8_t advanceCompareGuard = 2;
constexpr uint8_t advanceDirectionDelay = 40;

uint8_t advanceStepDelay(uint8_t interval) {
    return interval < advanceCompareGuard ? advanceCompareGuard : interval;
}

void scheduleAdvanceCompare(uint8_t delay, uint8_t latestElapsed) {
    const uint8_t due = EXTRUDER_OCR;
    const uint8_t now = TCNT0;
    const uint8_t elapsed = now - due;
    // Keep the cadence when the compare is still reachable. A late handler
    // starts a new interval so the counter does not have to wrap to reach it.
    EXTRUDER_OCR = (elapsed <= latestElapsed ? due : now) + delay;
    // The old compare can match again during a handler delayed by a wrap.
    TIFR0 = _BV(OCF0A);
}
} // namespace

void ExtrusionOutput::startTimer(int16_t pending, uint8_t mask) {
    if (!advanceEnabled() || !pending || blocked())
        return;

    const int8_t direction = pending > 0 ? 1 : -1;
    if (direction_ != direction && !canChangeDirection(pending))
        return;

    EXTRUDER_OCR = TCNT0 + advanceStepDelay(stepInterval_);
    TIFR0 = _BV(OCF0A); // A masked match can leave the flag set.
    EXTRUDER_TIMSK = mask | _BV(EXTRUDER_OCIE);
}

void ExtrusionOutput::stopTimer() {
    EXTRUDER_TIMSK &= ~_BV(EXTRUDER_OCIE);
}

void ExtrusionOutput::serviceTimerCompare() {
    const int16_t pending = pendingSteps();
    if (pending) {
        const int8_t direction = pending > 0 ? 1 : -1;
        if (direction_ == direction) {
            const int16_t remaining = pending - direction;
            Extruder::timedStep(pendingSteps_, remaining);
            if (remaining) {
                const uint8_t delay = advanceStepDelay(stepInterval_);
                scheduleAdvanceCompare(delay, delay - advanceCompareGuard);
                return;
            }
        } else if (canChangeDirection(pending)) {
            Extruder::setDirection(direction > 0);
            direction_ = direction;
            scheduleAdvanceCompare(stepInterval_ + advanceDirectionDelay, stepInterval_);
            return;
        }
    }
    stopTimer();
}

// Inline the direction path so the interrupt only saves the registers it uses.
ISR(EXTRUDER_TIMER_VECTOR, __attribute__((flatten))) {
    Printer::extrusionOutput.serviceTimerCompare();
}

#if !defined CASE_LIGHT_PIN || CASE_LIGHT_PIN < 0
#error The case light pin must be defined in case the case light feature shall be used.
#endif //!defined CASE_LIGHT_PIN || CASE_LIGHT_PIN < 0

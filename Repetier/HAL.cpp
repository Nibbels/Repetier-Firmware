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

#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
#ifndef RF_HEATER_LEASE_EXPERIMENT
#error Reset watchdog experiment requires the heater lease experiment.
#endif
// Non-release candidate. AVR CRT .init2 has initialized SP and r1.
// Capture reset cause before disabling a watchdog inherited from reset.
// The actual installed bootloader runs before this code and is unqualified.
#ifndef __AVR_ATmega2560__
#error Early watchdog reset sequence is specific to ATmega2560.
#endif
static_assert(WDRF==3 && WDCE==4 && WDE==3, "watchdog bit mapping");
extern "C" {
volatile uint8_t heaterWatchdogResetCause __attribute__((section(".noinit"), used, externally_visible));
// Best-effort retained evidence: a bootloader may erase MCUSR or use SRAM.
// A valid cookie from the recovery entry is written AFTER the bootloader.
// The live-session cookie must never be treated as the only safety barrier.
volatile uint32_t heaterResetCookie __attribute__((section(".noinit"), used, externally_visible));
constexpr uint32_t heaterLiveCookie=0xd39a6c51UL;
constexpr uint32_t heaterRecoveryCookie=0xd39a6c52UL;

// DO NOT remove or move this apparently unused function. Some Mega2560
// STK500v2 binaries (Arduino 1.0.5 through AVR core 1.8.6, blob 68d2e18b)
// set EIND=1 in CRT startup and use EICALL with Z=0 on a watchdog reset.
// The resulting WORD PC 0x10000 is FLASH BYTE ADDRESS 0x20000. Without a
// reserved entry the bootloader can execute arbitrary application code
// after disabling the watchdog. A direct JMP ignores EIND.
// The generated linker script must KEEP this section exactly at 0x20000,
// and the ELF/HEX verification must confirm every byte. No bootloader write.
// See coordination/tasks/Task0034.md A15/A16 and ArduinoCore-avr issue #183.
// This cannot rescue a bootloader stuck before entering application flash.
void heaterWatchdogRecoveryEntry() __attribute__((naked, used, externally_visible, section(".rf_watchdog_recovery")));
void heaterWatchdogRecoveryEntry() {
    asm volatile(
        "cli\n\t"
        // Mark this particular entry even if the bootloader cleared MCUSR.
        // Do not use stack, C++ objects, or initialize unknown board pins.
        "ldi r24,0x52\n\t"
        "sts heaterResetCookie,r24\n\t"
        "ldi r24,0x6c\n\t"
        "sts heaterResetCookie+1,r24\n\t"
        "ldi r24,0x9a\n\t"
        "sts heaterResetCookie+2,r24\n\t"
        "ldi r24,0xd3\n\t"
        "sts heaterResetCookie+3,r24\n\t"
        "jmp 0\n\t");
    // No RET: CRT at byte zero resets SP and EIND and initializes RAM.
}
void heaterWatchdogEarlyInit() __attribute__((naked, used, externally_visible, section(".init3")));
void heaterWatchdogEarlyInit() {
    // Basic assembly only in a naked function. ATmega2560 MCUSR I/O=0x34,
    // WDTCSR data address=0x60; r1 is zero after CRT .init2.
    asm volatile(
        "cli\n\t"
        "in r24,0x34\n\t"
        "sts heaterWatchdogResetCause,r24\n\t"
        "andi r24,0xf7\n\t"
        "out 0x34,r24\n\t"
        "wdr\n\t"
        "ldi r24,0x18\n\t"
        "sts 0x60,r24\n\t"
        "sts 0x60,r1\n\t"
        // Mandatory link guard: a naked startup symbol is always retained.
        // The special linker script defines this symbol only when the
        // recovery entry is asserted at the exact flash byte address.
        "ldi r24,lo8(__rf_watchdog_recovery_link_guard)\n\t");
    // Fall through to .init4, never RET.
}
}
#endif

namespace {
#ifdef RF_HEATER_LEASE_EXPERIMENT
void beginHeaterLease();
#endif
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
#ifdef RF_HEATER_LEASE_EXPERIMENT
// Timer3 is independent of Arduino millis and owns TOV3 exclusively here.
enum : uint8_t {
    SampleValid=1, SampleFresh=2, SampleBusy=4, SampleReserved=8, SampleMatch=16,
    SamplePrevious=32, ReservationPrevious=64
};
bool heaterClockContinuity();
volatile uint8_t heaterSampleState[TemperatureAdcCount] = {};
volatile uint16_t heaterSampleStamp[TemperatureAdcCount] = {};
volatile uint16_t heaterBatchStart=0;
volatile bool heaterBatchValid=false;
volatile bool heaterBatchPrevious=false;
volatile bool heaterLeaseReady=false;
constexpr uint16_t heaterSampleMaxAge=6250; // experimental 100ms from batch start
// A live active compare proves less than one wrap since its last grant,
// which followed a synchronized sample consume. Never clear an active
// compare flag at renewal. This requires a bounded consume-to-arm path.
void synchronizeHeaterSamples() {
    if (!(TIFR3 & _BV(TOV3))) return;
    const bool continuous=heaterClockContinuity();
    for(uint8_t c=0;c<TemperatureAdcCount;++c) {
        uint8_t s=heaterSampleState[c];
        if(!continuous) {
            s &= SampleBusy;
        } else {
            if(s&SamplePrevious)
                s &= uint8_t(~(SampleValid|SampleFresh|SampleMatch|SamplePrevious));
            else if(s&SampleValid)
                s |= SamplePrevious;
            if(s&ReservationPrevious)
                s &= uint8_t(~(SampleReserved|SampleMatch|ReservationPrevious));
            else if(s&SampleReserved)
                s |= ReservationPrevious;
        }
        heaterSampleState[c]=s;
    }
    if(!continuous || heaterBatchPrevious) heaterBatchValid=false;
    heaterBatchPrevious=heaterBatchValid;
    TIFR3=_BV(TOV3); // never clear compare expiry flags here
}
bool heaterClockSnapshot(uint16_t& now) {
    // Counter can cross BOTTOM during the register read. Retry once only.
    for(uint8_t attempt=0;attempt<2;++attempt) {
        synchronizeHeaterSamples();
        now=TCNT3;
        if(!(TIFR3&_BV(TOV3))) return true;
    }
    synchronizeHeaterSamples();
    return false;
}
bool heaterAgeLive(uint16_t now,uint16_t stamp,bool previous) {
    return (previous ? now<stamp : now>=stamp)
        && uint16_t(now-stamp)<=heaterSampleMaxAge;
}
bool heaterSampleLive(uint8_t c) {
    uint16_t now;
    if(!heaterClockSnapshot(now) || !(heaterSampleState[c]&SampleValid)) return false;
    if(!heaterAgeLive(now,heaterSampleStamp[c],heaterSampleState[c]&SamplePrevious)) {
        heaterSampleState[c] &= SampleBusy|SampleReserved|ReservationPrevious;
        return false;
    }
    return true;
}
void publishHeaterSample(uint8_t index, uint16_t raw) {
    uint16_t now;
    const bool clockValid=heaterClockSnapshot(now);
    // Preserve the exact old reservation, not its relation to the latest
    // publication. An observed ADC rail invalidates it immediately.
    heaterSampleState[index] &= SampleBusy|SampleReserved|ReservationPrevious;
    if(raw==0 || raw>=4092) heaterSampleState[index] &= SampleBusy;
    if(clockValid && heaterLeaseReady && heaterBatchValid &&
       heaterAgeLive(now,heaterBatchStart,heaterBatchPrevious)) {
        heaterSampleStamp[index]=heaterBatchStart;
        heaterSampleState[index] |= SampleValid|SampleFresh|
            (heaterBatchPrevious?SamplePrevious:0);
    }
    // Use the coherent snapshot, before the next conversion begins.
    // Any later wrap is processed before the next use of these records.
    heaterBatchStart=now;
    heaterBatchValid=clockValid && heaterLeaseReady;
    heaterBatchPrevious=false;
}
#endif

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
#ifdef RF_HEATER_LEASE_EXPERIMENT
        publishHeaterSample(index, sum >> 3);
#endif
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

#ifndef RF_HEATER_RESET_WDT_EXPERIMENT
volatile unsigned long watchdogHeartbeat = 0;
#endif
volatile bool watchdogActive = false;

#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
// Distinguish pre-start boot from a runtime stop. Once armed, the external
// supervisor must stay driven until a real reset, even on repeated stops.
// Likewise never restart an expired progress clock via startWatchdog().
bool watchdogStarted=false;

// Timer5 is borrowed by this experiment as an interrupt-free progress clock.
// It runs in normal mode with every OC pin disconnected (OC5A is a STEP pin
// on RF1000!). No Timer5 IRQ and no hardware waveform output are enabled.
// Compare A is a sticky expiry witness, NOT an ISR. A foreground thermal
// pass may extend a still-live permission, never clear an expired one.
// This avoids millis/coalesced-overflow ABA and allows the external reset
// supervisor to be fed only while a real thermal pass is recent.
// Timer5 event-driven heater PWM work must coordinate this allocation.
constexpr uint16_t watchdogProgressDuration=12500; // 200ms at /256, 16MHz
constexpr uint16_t watchdogProgressGuard=4;
static_assert(F_CPU==16000000UL,"watchdog progress clock assumes 16MHz");
bool watchdogProgressLive() {
    const uint16_t left=uint16_t(OCR5A-TCNT5);
    return left>watchdogProgressGuard && left<=watchdogProgressDuration
        && !(TIFR5&_BV(OCF5A));
}
void beginWatchdogProgress() {
    TIMSK5=0;
    TCCR5B=0;
    TCCR5A=0;
    TCCR5C=0;
    TCNT5=0;
    OCR5A=watchdogProgressDuration;
    // Only initial arming after boot may acknowledge this sticky flag.
    TIFR5=_BV(OCF5A);
    TCCR5B=_BV(CS52);
}
bool renewWatchdogProgress() {
    if(!watchdogProgressLive()) return false;
    const uint16_t now=TCNT5;
    // Recheck the previous deadline using the same coherent clock before
    // replacing OCR5A. Never clear a pending old compare on renewal.
    const uint16_t left=uint16_t(OCR5A-now);
    if(left<=watchdogProgressGuard || left>watchdogProgressDuration
        || (TIFR5&_BV(OCF5A))) return false;
    OCR5A=uint16_t(now+watchdogProgressDuration);
    return watchdogProgressLive();
}
#endif

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
#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
    // TPS3820 needs FALLING edges (nominal 200ms timeout, 112..300ms
    // in current TI timing table). Alternating 16ms pulses give a falling
    // edge every ~32ms while thermal progress is authorized. When the
    // permission expires leave WDI driven and static, never float it.
    // External reset is intended to precede the 1s nominal internal WDT;
    // actual oscillator, board routing and reset/power bias are unqualified.
    if (watchdogActive && watchdogProgressLive())
#else
    if (watchdogActive && HAL::timeInMilliseconds() - watchdogHeartbeat < WATCHDOG_MAIN_LOOP_TIMEOUT)
#endif
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

// Task0025 period tables: preserve the legacy interpolation exactly.
namespace {
static_assert(F_CPU == 16000000UL, "Period table encoding requires the RFx000 16 MHz clock");

// The legacy interval used an exact division below 512 and linear interpolation
// between integer quotients above it. All calculations here are compile-time.
constexpr uint32_t interpolatedPeriod(uint32_t d, uint32_t width) {
    return F_CPU / (d / width * width) -
           ((F_CPU / (d / width * width) - F_CPU / (d / width * width + width))
            * (d % width)) / width;
}
constexpr uint32_t legacyPeriod(uint32_t d) {
    return d < 512 ? F_CPU / (d < 10 ? 10 : d) :
           interpolatedPeriod(d, d < 8192 ? 32 : 4096);
}

// On the three upper ranges the period decreases by at most one tick over
// a block of 4, 8 or 16 divisors respectively. Pack the final (lower) period
// in 12 bits and the number of leading entries one tick higher in 4 bits.
// These are compile-time calculations, not an ISR search or interpolation.
constexpr uint16_t leadingHigher(uint32_t start, uint32_t last) {
    return legacyPeriod(start) == last ? 0 : 1 + leadingHigher(start + 1, last);
}
constexpr uint16_t packedPeriod(uint32_t start, uint32_t width) {
    return static_cast<uint16_t>(legacyPeriod(start + width - 1) |
           (leadingHigher(start, legacyPeriod(start + width - 1)) << 12));
}
constexpr uint32_t lowPeriod(uint32_t d) { return legacyPeriod(d); }
constexpr uint16_t midPeriod(uint32_t d) { return static_cast<uint16_t>(legacyPeriod(d)); }
constexpr uint16_t high4Period(uint32_t d) { return packedPeriod(d, 4); }
constexpr uint16_t high8Period(uint32_t d) { return packedPeriod(d, 8); }
constexpr uint16_t high16Period(uint32_t d) { return packedPeriod(d, 16); }

// Expand regular compile-time table entries without a generated source dump.
#define RF_PERIOD_1(F, B, S) F(B)
#define RF_PERIOD_2(F, B, S) RF_PERIOD_1(F, B, S), RF_PERIOD_1(F, (B) + 1UL * (S), S)
#define RF_PERIOD_4(F, B, S) RF_PERIOD_2(F, B, S), RF_PERIOD_2(F, (B) + 2UL * (S), S)
#define RF_PERIOD_8(F, B, S) RF_PERIOD_4(F, B, S), RF_PERIOD_4(F, (B) + 4UL * (S), S)
#define RF_PERIOD_16(F, B, S) RF_PERIOD_8(F, B, S), RF_PERIOD_8(F, (B) + 8UL * (S), S)
#define RF_PERIOD_32(F, B, S) RF_PERIOD_16(F, B, S), RF_PERIOD_16(F, (B) + 16UL * (S), S)
#define RF_PERIOD_64(F, B, S) RF_PERIOD_32(F, B, S), RF_PERIOD_32(F, (B) + 32UL * (S), S)
#define RF_PERIOD_128(F, B, S) RF_PERIOD_64(F, B, S), RF_PERIOD_64(F, (B) + 64UL * (S), S)
#define RF_PERIOD_256(F, B, S) RF_PERIOD_128(F, B, S), RF_PERIOD_128(F, (B) + 128UL * (S), S)
#define RF_PERIOD_512(F, B, S) RF_PERIOD_256(F, B, S), RF_PERIOD_256(F, (B) + 256UL * (S), S)
static const uint32_t lowPeriods[512] PROGMEM = {
    RF_PERIOD_512(lowPeriod, 0UL, 1UL),
};

static const uint16_t midPeriods[7680] PROGMEM = {
    RF_PERIOD_512(midPeriod, 512UL, 1UL),
    RF_PERIOD_512(midPeriod, 1024UL, 1UL),
    RF_PERIOD_512(midPeriod, 1536UL, 1UL),
    RF_PERIOD_512(midPeriod, 2048UL, 1UL),
    RF_PERIOD_512(midPeriod, 2560UL, 1UL),
    RF_PERIOD_512(midPeriod, 3072UL, 1UL),
    RF_PERIOD_512(midPeriod, 3584UL, 1UL),
    RF_PERIOD_512(midPeriod, 4096UL, 1UL),
    RF_PERIOD_512(midPeriod, 4608UL, 1UL),
    RF_PERIOD_512(midPeriod, 5120UL, 1UL),
    RF_PERIOD_512(midPeriod, 5632UL, 1UL),
    RF_PERIOD_512(midPeriod, 6144UL, 1UL),
    RF_PERIOD_512(midPeriod, 6656UL, 1UL),
    RF_PERIOD_512(midPeriod, 7168UL, 1UL),
    RF_PERIOD_512(midPeriod, 7680UL, 1UL),
};

static const uint16_t high4Periods[1024] PROGMEM = {
    RF_PERIOD_512(high4Period, 8192UL, 4UL),
    RF_PERIOD_512(high4Period, 10240UL, 4UL),
};

static const uint16_t high8Periods[512] PROGMEM = {
    RF_PERIOD_512(high8Period, 12288UL, 8UL),
};

static const uint16_t high16Periods[3072] PROGMEM = {
    RF_PERIOD_512(high16Period, 16384UL, 16UL),
    RF_PERIOD_512(high16Period, 24576UL, 16UL),
    RF_PERIOD_512(high16Period, 32768UL, 16UL),
    RF_PERIOD_512(high16Period, 40960UL, 16UL),
    RF_PERIOD_512(high16Period, 49152UL, 16UL),
    RF_PERIOD_512(high16Period, 57344UL, 16UL),
};

#undef RF_PERIOD_512
#undef RF_PERIOD_256
#undef RF_PERIOD_128
#undef RF_PERIOD_64
#undef RF_PERIOD_32
#undef RF_PERIOD_16
#undef RF_PERIOD_8
#undef RF_PERIOD_4
#undef RF_PERIOD_2
#undef RF_PERIOD_1

// AVR-GCC -Os expands the 16-bit shift by four into a loop. Exchange
// nibbles explicitly; seven instructions, independent of the divisor.
inline uint16_t periodIndex16(uint16_t divisor) {
#if defined(__AVR__)
    uint8_t upperLow;
    __asm__ volatile(
        "swap %A[value] \n\t"
        "swap %B[value] \n\t"
        "andi %A[value],0x0f \n\t"
        "mov %[tmp],%B[value] \n\t"
        "andi %[tmp],0xf0 \n\t"
        "or %A[value],%[tmp] \n\t"
        "andi %B[value],0x0f"
        : [value] "+d"(divisor), [tmp] "=&d"(upperLow)
        :
        : "cc");
    return divisor;
#else
    return divisor >> 4;
#endif
}

inline uint16_t unpackPeriod(uint16_t packed, uint8_t offset) {
    const uint8_t threshold = packed >> 12;
    uint16_t period = packed & 0x0fffU;
#if defined(__AVR__)
    // Add the comparison carry directly: fixed cost, no conditional branch.
    __asm__ volatile(
        "cp %[offset],%[threshold] \n\t"
        "adc %A[period],__zero_reg__ \n\t"
        "adc %B[period],__zero_reg__"
        : [period] "+&r"(period)
        : [offset] "r"(offset), [threshold] "r"(threshold)
        : "cc");
#else
    period += offset < threshold;
#endif
    return period;
}
} // namespace

// Convert step frequency into exactly the legacy timer interval. The tables
// live in program memory; no SRAM, variable-length division or interpolation.
int32_t HAL::CPUDivU2(unsigned int divisor) {
    if (divisor >= 16384)
        return unpackPeriod(pgm_read_word(&high16Periods[periodIndex16(divisor) - 1024]), divisor & 15);
    if (divisor >= 12288)
        return unpackPeriod(pgm_read_word(&high8Periods[(divisor >> 3) - 1536]), divisor & 7);
    if (divisor >= 8192)
        return unpackPeriod(pgm_read_word(&high4Periods[(divisor >> 2) - 2048]), divisor & 3);
    if (divisor >= 512)
        return pgm_read_word(&midPeriods[divisor - 512]);
    return pgm_read_dword(&lowPeriods[divisor]);
} // CPUDivU2

void HAL::setupTimer() {
#ifdef RF_HEATER_LEASE_EXPERIMENT
    beginHeaterLease();
#endif
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

} // setupTimer

void HAL::showStartReason() {
    // Check startup - does nothing if bootloader sets MCUSR to 0
#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
    uint8_t mcu = heaterWatchdogResetCause;
    if (mcu != 0)
#else
    uint8_t mcu = MCUSR;

    if (MCUSR != 0)
#endif //Printer::debugInfo()
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

// AVR linker symbol: first byte beyond static storage. The weak allocator
// frontier is optional, so diagnostics alone do not pull malloc into the image.
extern "C" {
extern char __heap_start;
extern char* __brkval __attribute__((weak));
}

int HAL::getFreeRam() {
    InterruptProtectedBlock lock;
    uintptr_t heapEnd = reinterpret_cast<uintptr_t>(&__heap_start);
    if (&__brkval && __brkval)
        heapEnd = reinterpret_cast<uintptr_t>(__brkval);
    // Contiguous stack-to-heap gap, not total free blocks or an ISR watermark.
    return static_cast<int>(SP) - static_cast<int>(heapEnd);
} // getFreeRam

void (*resetFunc)(void) = 0; // declare reset function @ address 0

void HAL::resetHardware() {
    // This legacy jump to address zero is not a peripheral hardware reset.
    // Remove heater power before restarting, even if the PWM ISR was pending.
    forbidInterrupts();
    for (uint8_t channel = 0; channel <= NUM_EXTRUDER; ++channel)
        setHeaterDuty(channel, 0);
    PWM_TIMSK &= ~_BV(PWM_OCIE);
#ifdef RF_HEATER_LEASE_EXPERIMENT
    TIMSK3 = 0;
    TCCR3B = 0;
#endif
#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
    // Prefer an actual peripheral reset after emergency shutdown. Do not
    // jump into CRT with live timers; no WDI pulses or WDR here. External
    // supervisor should win, internal WDT is independent backup. During
    // pre-watchdog boot retain the legacy safe-output fallback.
    if(watchdogStarted) for(;;) {}
#endif
    resetFunc();
    // A restart must not return to an interrupted print.
    for (;;) {}
} // resetHardware

uint16_t HAL::temperatureAdcSample(uint8_t index) {
    // The PWM interrupt can publish between the two byte loads on AVR.
    InterruptProtectedBlock lock;
    return temperatureAdcValues[index];
}

bool HAL::temperatureAdcSample(uint8_t index, uint16_t& sample
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
                              , uint32_t* publication
#endif
) {
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
    if (static_cast<uint32_t>(timeInMilliseconds() - publishedAt) <= temperatureAdcMaxAgeMs) {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        if(publication)*publication=publishedAt;
#endif
        return true;
    }
    {
        // Expired data must not become valid again on a later clock wrap.
        // Do not invalidate a newer batch published since the snapshot.
        InterruptProtectedBlock lock;
        if (temperatureAdcPublishedAt[index] == publishedAt)
            temperatureAdcReady[index] = false;
    }
    return false;
}

#ifdef RF_HEATER_LEASE_EXPERIMENT
HAL::HeaterSample::~HeaterSample() { release(); }
void HAL::HeaterSample::release() {
    InterruptProtectedBlock lock;
    if(index_!=255) {
        heaterSampleState[index_] &= SampleValid|SampleFresh|SamplePrevious;
        index_=255;
    }
}
bool HAL::HeaterSample::read(uint8_t index,uint16_t& raw
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
                            , uint32_t* publication
#endif
) {
    InterruptProtectedBlock lock;
    raw=0;
    if(index_!=255 || index>=TemperatureAdcCount || !heaterLeaseReady) return false;
    // Preserve diagnostic validity independently from grant eligibility.
    // In particular a conservative wrap invalidation must not impersonate
    // a broken sensor during a healthy autotune. No valid ticket, no renewal.
    if(!HAL::temperatureAdcSample(index,raw
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
                                  ,publication
#endif
    )) return false;
    if(raw>0 && raw<4092 && heaterSampleLive(index) &&
       (heaterSampleState[index] & (SampleFresh|SampleBusy))==SampleFresh) {
        heaterSampleState[index] |= SampleBusy|SampleReserved|SampleMatch;
        if(heaterSampleState[index]&SamplePrevious) heaterSampleState[index] |= ReservationPrevious;
        stamp_=heaterSampleStamp[index];
        index_=index;
    }
    return true;
}
bool HAL::HeaterSample::consume(uint8_t channel) {
    // Caller holds the IRQ lock through grant publication.
    if(index_!=channel || index_==255) return false;
    uint16_t now;
    if(!heaterClockSnapshot(now) || !(heaterSampleState[index_]&SampleReserved)) return false;
    const bool young=heaterAgeLive(now,stamp_,heaterSampleState[index_]&ReservationPrevious);
    // Consume only the reserved batch. A newer batch retains its eligibility.
    if(heaterSampleState[index_]&SampleMatch)
        heaterSampleState[index_] &= uint8_t(~SampleFresh);
    heaterSampleState[index_] &= uint8_t(~(SampleReserved|SampleMatch|ReservationPrevious));
    return young;
}
#endif

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
#ifdef RF_HEATER_LEASE_EXPERIMENT
        heaterSampleState[i]=0;
#endif
    }
#ifdef RF_HEATER_LEASE_EXPERIMENT
    heaterBatchValid=false;
    heaterBatchPrevious=false;
#endif
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
#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
    // avr-libc emits the timed WDTCSR unlock sequence. Reset-only, not WDIE.
    // 1s nominal independent fallback, deliberately slower than the external
    // TPS3820 path (progress permission ~200ms + supervisor ~200ms nominal).
    // This is a research setting, not a guaranteed ordering or hard bound.
    wdt_enable(WDTO_1S);
#else
    // Just to be safe since we can not clear WDE if WDRF is set
    MCUSR &= ~(1 << WDRF);
    // Disable interrupts
    cli();
    // Reset watchdog
    wdt_reset();
    // Set up WDT interrupt
    WDTCSR = (1 << WDCE) | (1 << WDE);
    // timeout in XXX millisecond, disable reset mode. Must be done in one operation --> S66  http://www.atmel.com/Images/Atmel-2549-8-bit-AVR-Microcontroller-ATmega640-1280-1281-2560-2561_datasheet.pdf
    //WDTCSR = 1<<WDP0; //32ms
    WDTCSR = 0; //16ms
    // Enable global interrupts
    sei();

    // enable watchdog interrupt only mode
    WDTCSR |= _BV(WDIE);
#endif
}

#ifndef RF_HEATER_RESET_WDT_EXPERIMENT
// Watchdog timeout ISR
ISR(WDT_vect) {
    pingWatchdog();
    WDTCSR |= (1 << WDIE); // Nibbels: nächstes mal kein Reset durch internen Watchdog, sondern wieder dieser interrupt.
    compensationTick = true; // Tell commandloop that 16ms have passed
}
#endif

// Watchdog support
void HAL::startWatchdog() {
#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
    if(watchdogStarted) return;
#endif
    Com::printFLN(Com::tStartWatchdog);
    // external watchdog
#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
    // A known recovery entry or interrupted heating session is not normal
    // continuation. At this point board identification and safe GPIO setup
    // have completed; no command has been admitted yet. MCUSR alone is
    // insufficient: the known Arduino bootloaders erase it.
    const uint32_t cookie=heaterResetCookie;
    const bool resetDuringHeat=cookie==heaterLiveCookie
        && !(heaterWatchdogResetCause&_BV(PORF));
    if(cookie==heaterRecoveryCookie || resetDuringHeat
        || (heaterWatchdogResetCause&_BV(WDRF))) {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        // The live cookie alone cannot distinguish a watchdog from another
        // reset. Only WDRF or the known watchdog recovery entry is evidence.
        ThermalEmergency::latchSystemFault(
            cookie==heaterRecoveryCookie || (heaterWatchdogResetCause&_BV(WDRF))
            ? ThermalEmergency::Cause::WatchdogReset
            : ThermalEmergency::Cause::InterruptedHeating);
        Printer::stopPrint();
#else
        Printer::latchOutputInhibit(true);
#endif
        Printer::markTemperatureReadingsAvailable();
#ifndef RF_THERMAL_EMERGENCY_EXPERIMENT
        Com::printErrorFLN(PSTR("Watchdog/recovery: heating inhibited until restart"));
#endif
    }
    // One-shot retained marker. An unrecognized power cycle may latch once
    // if SRAM survives and MCUSR is lost; the next restart can clear it.
    // No stale target or duty is restored on any boot.
    heaterResetCookie=0;
    InterruptProtectedBlock lock;
    WRITE(WATCHDOG_PIN,LOW);
    SET_OUTPUT(WATCHDOG_PIN);
    beginWatchdogProgress();
    watchdogStarted=true;
    watchdogActive=true;
    pingWatchdog();
    HAL::WDT_Init();
#else
    SET_OUTPUT(WATCHDOG_PIN);
    tellWatchdogOk();
    watchdogActive = true;
    pingWatchdog();      // Nibbels: Hier macht der mehr sinn!
    HAL::WDT_Init();     // Nibbels: use watchdogtimer to test var and trigger
#endif
} // startWatchdog

void HAL::stopWatchdog() {
#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
    // Only pre-start boot may float WDI. A runtime stop must not disable
    // the external supervisor or feed the internal one.
    if(watchdogStarted) {
        InterruptProtectedBlock lock;
        watchdogActive=false;
        for(uint8_t c=0;c<=NUM_EXTRUDER;++c) setHeaterDuty(c,0);
        return;
    }
#endif
    watchdogActive = false;
                              // external watchdog
    WRITE(WATCHDOG_PIN, LOW); // Nibbels: In case you stop the watchdog it has to be set floating. thatwhy disable internal pullup as this will make the state like OUTPUT-high. This is not really needed as this function does not do anything when starting the arduino. rightnow at start it sets a pin to input which is already input.
    SET_INPUT(WATCHDOG_PIN);
} // stopWatchdog

void HAL::tellWatchdogOk() {
#ifndef RF_HEATER_RESET_WDT_EXPERIMENT
    const auto now = HAL::timeInMilliseconds();
    InterruptProtectedBlock lock;
    watchdogHeartbeat = now;
#endif
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
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    // A terminal thermal stop is not a normal queue drain. No new STEP
    // output or queued control task, even if an old caller republishes work.
    if(Printer::printJob.emergencyLocked()) {
        cbi(TIMSK1,OCIE1A);
        return;
    }
#endif
    uint8_t doExit;
    __asm volatile(
        "lds r23,stepperWait+2 \n\t"
        "tst r23 \n\t"
        "brne else%= \n\t" // At least 65536 ticks remain.
        "lds r23,stepperWait+1 \n\t"
        "lds r22,stepperWait \n\t" // Always load BOTH bytes of the final interval.
        "mov %[ex],r22 \n\t"
        "or %[ex],r23 \n\t"
        "breq end%= \n\t" // Both bytes zero: execute the move.
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
        : "r22", "r23", "cc", "memory");
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
#ifdef RF_HEATER_LEASE_EXPERIMENT
// Non-release candidate: /256, 16us/tick, nominal 200ms lease.
// Deadline and guard require full-system timing qualification.
constexpr uint16_t heaterLeaseDuration = 12500;
constexpr uint16_t heaterLeaseGuard = 4;
volatile bool heaterLeaseFailed = false;
#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
volatile uint8_t heaterGrantProgress = 0;
#endif
static_assert(F_CPU == 16000000UL && NUM_EXTRUDER <= 2, "lease clock assumptions");
static_assert(OCIE3A == OCF3A && OCIE3B == OCF3B && OCIE3C == OCF3C
              && OCIE3B == OCIE3A+1 && OCIE3C == OCIE3A+2, "compare bit mapping");
inline uint8_t leaseMask(uint8_t channel) { return uint8_t(_BV(OCIE3A) << channel); }
inline uint16_t leaseDeadline(uint8_t channel) {
    switch (channel) { case 0: return OCR3A; case 1: return OCR3B; default: return OCR3C; }
}
inline void writeLeaseDeadline(uint8_t channel, uint16_t deadline) {
    switch (channel) { case 0: OCR3A=deadline; break; case 1: OCR3B=deadline; break; default: OCR3C=deadline; break; }
}
void beginHeaterLease() {
    InterruptProtectedBlock lock;
    TIMSK3=0;
    TCCR3B=0;
    TCCR3A=0;
    TCCR3C=0;
    TCNT3=0;
    OCR3A=OCR3B=OCR3C=0;
    TIFR3=_BV(OCF3A)|_BV(OCF3B)|_BV(OCF3C)|_BV(TOV3);
    TCCR3B=_BV(CS32);
    heaterLeaseReady=true;
}
// IRQs masked by caller. No serial, no float, no motion calls.
void failHeaterLease() {
    heaterLeaseFailed=true;
#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
    heaterGrantProgress=0;
#endif
#if EXT0_HEATER_PIN > -1
    WRITE(EXT0_HEATER_PIN, LOW);
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN > -1 && NUM_EXTRUDER > 1
    WRITE(EXT1_HEATER_PIN, LOW);
#endif
    WRITE(HEATED_BED_HEATER_PIN, LOW);
    for(uint8_t c=0;c<=NUM_EXTRUDER;++c) requestedDuty[c]=0;
    TIMSK3=0;
}
inline __attribute__((always_inline)) bool leaseWindowLive(uint16_t deadline,uint8_t mask,uint16_t now) {
    const uint16_t left=uint16_t(deadline-now);
    return left>heaterLeaseGuard && left<=heaterLeaseDuration && !(TIFR3 & mask);
}
bool leaseLive(uint8_t c,uint16_t now) {
    return leaseWindowLive(leaseDeadline(c),leaseMask(c),now);
}
// Fixed channels in the frame and clock-continuity checks avoid the variable
// register selection and bit shifts in the generic grant/renewal path.
template<uint8_t c>
inline __attribute__((always_inline)) bool fixedLeaseLive(uint16_t now) {
    static_assert(c<=NUM_EXTRUDER,"heater channel");
    return leaseWindowLive(leaseDeadline(c),leaseMask(c),now);
}
template<uint8_t c>
inline __attribute__((always_inline)) bool continuityChannel(uint16_t now) {
    return requestedDuty[c] && (TIMSK3&leaseMask(c)) && fixedLeaseLive<c>(now);
}
template<uint8_t c>
inline __attribute__((always_inline)) bool validActiveChannel(uint16_t now) {
    return !requestedDuty[c] || fixedLeaseLive<c>(now);
}
bool heaterClockContinuity() {
    if(!heaterLeaseReady || heaterLeaseFailed) return false;
    const uint16_t now=TCNT3;
    return continuityChannel<0>(now) || continuityChannel<1>(now)
#if NUM_EXTRUDER > 1
        || continuityChannel<2>(now)
#endif
        ;
}
bool checkHeaterLeases() {
    if (!heaterLeaseReady || heaterLeaseFailed) return false;
    const uint16_t now=TCNT3;
    if(validActiveChannel<0>(now) && validActiveChannel<1>(now)
#if NUM_EXTRUDER > 1
        && validActiveChannel<2>(now)
#endif
        ) return true;
    failHeaterLease();
    return false;
}
#endif
} // namespace

#ifdef RF_HEATER_LEASE_EXPERIMENT
bool HAL::heaterLeaseFault() { return heaterLeaseFailed; }
#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
// The regular thermal pass and autotune may complete in either order.
// Bit 7 is consumed once with the per-channel grant bits. Autotune cannot
// keep the watchdog alive on its own if the regular service is stalled.
constexpr uint8_t thermalPassComplete=0x80;
static void tryThermalWatchdogFeed(bool heatDemand,bool regularPass) {
    InterruptProtectedBlock lock;
    if(!watchdogActive || !heaterLeaseReady) return;
    checkHeaterLeases(); // expired demands are removed before considering feed
    if(regularPass) heaterGrantProgress |= thermalPassComplete;
    if(!(heaterGrantProgress&thermalPassComplete)) return;
    bool anyPower=false;
    for(uint8_t c=0;c<=NUM_EXTRUDER;++c) {
        if(!requestedDuty[c]) continue;
        anyPower=true;
        if(!(heaterGrantProgress & _BV(c))) return;
    }
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(Printer::isAnyTempsensorDefect()) {
        // Retain diagnostics only while the real completed thermal pass
        // demonstrates all software heater outputs off. Read GPIO, not
        // current: a failed-closed MOSFET/triac is outside this proof.
        if(!regularPass || anyPower || TIMSK3 || READ(EXT0_HEATER_PIN)
#if NUM_EXTRUDER > 1
            || READ(EXT1_HEATER_PIN)
#endif
            || READ(HEATED_BED_HEATER_PIN))return;
    }
#endif
    // Do not resurrect a missed hardware progress deadline, even if a
    // delayed foreground pass now looks healthy. Both watchdogs must expire.
    if(!renewWatchdogProgress()) return;
    // Feed only after a complete foreground thermal pass and fresh grants
    // for all currently powered channels since the previous feed.
    // A latched all-off fault may retain diagnostics if progress is timely.
    heaterGrantProgress=0;
    if(heatDemand || anyPower) heaterResetCookie=heaterLiveCookie;
    else if(!heaterLeaseFailed && !Printer::heatingInhibited()) heaterResetCookie=0;
    wdt_reset();
}
void HAL::completeThermalWatchdogPass(bool heatDemand) {
    tryThermalWatchdogFeed(heatDemand,true);
}
void HAL::completeAutotuneWatchdogPass() {
    tryThermalWatchdogFeed(true,false);
}
#endif
ISR(TIMER3_COMPA_vect) { failHeaterLease(); }
ISR(TIMER3_COMPB_vect, ISR_ALIASOF(TIMER3_COMPA_vect));
#if NUM_EXTRUDER > 1
ISR(TIMER3_COMPC_vect, ISR_ALIASOF(TIMER3_COMPA_vect));
#endif
#endif

void HAL::resetPwmOutputs() {
    for (auto& duty : requestedDuty)
        duty = 0;
}

uint8_t HAL::outputDuty(uint8_t channel) { return requestedDuty[channel]; }
void HAL::setHeaterDuty(uint8_t channel, uint8_t duty
#ifdef RF_HEATER_LEASE_EXPERIMENT
    , HeaterSample* sample
#endif
) {
    if (channel > NUM_EXTRUDER)
        return;
#ifdef RF_HEATER_LEASE_EXPERIMENT
    InterruptProtectedBlock lock;
    const bool allowed=checkHeaterLeases();
    if (duty) {
#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
        // No positive grant after the progress clock has expired, even if
        // an old heater deadline still happens to be live.
        if(!watchdogActive || !watchdogProgressLive()) { failHeaterLease(); return; }
#endif
        if (!allowed || Printer::heatingInhibited()) return;
        // No new grant from a diagnostic read, expired or consumed reservation.
        // Existing power can only survive until its previous deadline.
        if(!sample || !sample->consume(channel)) return;
        const bool active=requestedDuty[channel]!=0;
        const uint16_t now=TCNT3;
        if(active && !leaseLive(channel,now)) { failHeaterLease(); return; }
        writeLeaseDeadline(channel,uint16_t(now+heaterLeaseDuration));
        // W1C only while previously inactive.
        if(!active) TIFR3=leaseMask(channel);
        if(!leaseLive(channel,TCNT3)) { failHeaterLease(); return; }
#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
        // Retain interruption evidence before publishing any positive duty.
        // This is diagnostic redundancy, not the sole reset protection.
        heaterResetCookie=heaterLiveCookie;
        heaterGrantProgress |= _BV(channel);
#endif
        requestedDuty[channel]=duty;
        TIMSK3 |= leaseMask(channel);
        return;
    }
    // A pending decision made before this revoke cannot rearm the heater.
    if(heaterSampleState[channel]&SampleMatch)
        heaterSampleState[channel] &= uint8_t(~SampleFresh);
    heaterSampleState[channel] &= uint8_t(~(SampleReserved|SampleMatch|ReservationPrevious));
    TIMSK3 &= uint8_t(~leaseMask(channel));
#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
    heaterGrantProgress &= uint8_t(~_BV(channel));
#endif
#else
    if (duty) {
        requestedDuty[channel] = duty;
        return;
    }
    InterruptProtectedBlock lock;
#endif

    // Revoke the requested duty and the active output together. The PWM
    // handler can only raise a heater at frame start, from requestedDuty.
    requestedDuty[channel] = 0;
    switch (channel) {
#if EXT0_HEATER_PIN > -1
        case 0: WRITE(EXT0_HEATER_PIN, LOW); break;
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN > -1 && NUM_EXTRUDER > 1
        case 1: WRITE(EXT1_HEATER_PIN, LOW); break;
#endif
        case NUM_EXTRUDER: WRITE(HEATED_BED_HEATER_PIN, LOW); break;
    }
}
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

#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
    // Move the legacy nominal 16ms compensation/WDI event away from WDT.
    // Alternate 62/63 serviced compares: 125 * 256us = 32ms.
    // Coalescence/overload can delay this event; WDT reset is independent.
    static uint8_t compensationCountdown=62;
    static bool longCompensationPeriod=false;
    if(--compensationCountdown==0) {
        longCompensationPeriod=!longCompensationPeriod;
        compensationCountdown=longCompensationPeriod?63:62;
        compensationTick=true;
        pingWatchdog();
    }
#endif

    if (pwm_count_heater == 0) {
#ifdef RF_HEATER_LEASE_EXPERIMENT
#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
        // PWM timer starts before startWatchdog() during setup. There is
        // no heater demand then; do not latch a boot-time false fault.
        if(watchdogActive && !watchdogProgressLive()) failHeaterLease();
#endif
        checkHeaterLeases(); // pending expiry wins before any rising edge
#endif
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
                // count = 0 -> AN, wenn Powerlevel über Minimum
                // Je nach PWM-Mask zählen nur obere bits!
                // Je nach PWM-Mask (z.b. 1111 11xx b) gibts spezielle PWM-Steps (z.b. 4 = 100 b) welche den counter nur innerhalb PWM-Mask hochzählen.
                pwm_pos_set[NUM_EXTRUDER + 2] = (requestedDuty[NUM_EXTRUDER + 2] & partFanPwmMask);
                if (pwm_pos_set[NUM_EXTRUDER + 2] > 0)
                    WRITE(FAN_PIN, 1); //AN
            }
            // count = powerlevel, aber nicht powerlevel == max. -> AUS
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
    if (counter100Periodical == 196) // Halbe 100ms Zeit -> Ping 50ms
    {
        featureTick = true;
    } else if (counter100Periodical >= 391) //(int)(F_CPU/40960))
    {
        counter100Periodical = 0;
        temperatureTick = true;
        featureTick = true; // Volle 100ms Zeit -> Ping 50ms
    }


    static char counter10Periodical = 0; // Approximate a 10ms timer :: blocks pingwatchdog s commandloop if not working
    if (++counter10Periodical >= 39) {
        counter10Periodical = 0;
        scanTick = true;

        // Hier her versetzt, um 10ms einstell-auflösung zu erhalten. Darum /10 statt bisher /100 beim Init in setFanSpeed().
        // Fan anfangs auf high zum anlaufen, aber das muss beendet werden:
        if (fanKickstart)
            fanKickstart--;
    }

    serviceTemperatureAdc();

    if (pwm_count_heater & 4)
        uid.fastAction();

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

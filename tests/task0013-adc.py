#!/usr/bin/env python3
"""Task0013: actual HAL ADC acquisition and freshness, with modeled registers.

ADIF is write-one-to-clear. This is a host logic replay, not an electrical,
interrupt-latency or AVR cycle simulator.
"""
import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

FW = Path(__file__).resolve().parents[1] / "Repetier"


def section(source, start, end):
    a = source.index(start)
    return source[a:source.index(end, a)]


HARNESS = r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#define PROGMEM
#define _BV(n) (1U << (n))
#define ADEN 7
#define ADSC 6
#define ADIF 4
#define REFS0 6
#define MUX5 3
#define ANALOG_PRESCALER 7
#define EXT0_TEMPSENSOR_PIN 0
#define EXT1_TEMPSENSOR_PIN 9
#define HEATED_BED_SENSOR_PIN 2
#if CHAMBER
#define X35_ADC_CHANNEL 12
#define TEMP_3_PIN X35_ADC_CHANNEL
#endif
constexpr uint8_t TemperatureAdcCount = NUM_EXTRUDER + 1 + CHAMBER;
static uint16_t ADCW;
static uint8_t ADMUX, muxB;
#define ADCSRB muxB
static uint32_t now;
static bool lockActive;
struct InterruptProtectedBlock {
    InterruptProtectedBlock() { assert(!lockActive); lockActive = true; }
    ~InterruptProtectedBlock() { lockActive = false; }
};
static uint8_t pgm_read_byte(const uint8_t* p) { return *p; }
struct AdcControl {
    uint8_t value = 0;
    bool completeDummy = false;
    operator uint8_t() {
        if (completeDummy && (value & _BV(ADSC))) {
            completeDummy = false;
            complete(999);
        }
        return value;
    }
    AdcControl& operator=(uint8_t v) {
        const uint8_t oldFlag = value & _BV(ADIF);
        value = v & ~_BV(ADIF);
        if (!(v & _BV(ADIF))) value |= oldFlag;
        return *this;
    }
    AdcControl& operator|=(uint8_t v) { return *this = value | v; }
    void complete(uint16_t sample) {
        assert((value & (_BV(ADEN) | _BV(ADSC))) == (_BV(ADEN) | _BV(ADSC)));
        ADCW = sample;
        value = (value & ~_BV(ADSC)) | _BV(ADIF);
    }
} ADCSRA;
struct HAL {
    static uint32_t timeInMilliseconds() { return now; }
    static uint16_t temperatureAdcSample(uint8_t);
    static bool temperatureAdcSample(uint8_t, uint16_t&);
    static void analogStart();
};
#include "adc.inc"

static uint16_t read(uint8_t index, bool expected) {
    uint16_t value = 12345;
    assert(HAL::temperatureAdcSample(index, value) == expected);
    assert(!lockActive);
    return value;
}

static void start() {
    ADCSRA.completeDummy = true;
    HAL::analogStart();
    assert(temperatureAdcPosition == 0 && temperatureAdcCount == 0 && temperatureAdcSum == 0);
    assert((ADCSRA.value & (_BV(ADEN) | _BV(ADSC) | _BV(ADIF))) == (_BV(ADEN) | _BV(ADSC)));
    for (uint8_t i=0; i<TemperatureAdcCount; ++i) assert(read(i, false) == 0);
    assert(read(TemperatureAdcCount, false) == 0);
}

static void batch(uint8_t index, uint16_t sample) {
    assert(temperatureAdcPosition == index);
    const auto channel = temperatureAdcChannels[index];
    assert((ADMUX & 0x1f) == (channel & 7));
    assert(bool(ADCSRB & _BV(MUX5)) == bool(channel & 8));
    for (unsigned i=0; i<32; ++i) {
        // A still-running conversion must not contribute the prior ADCW.
        serviceTemperatureAdc();
        assert(temperatureAdcCount == i);
        ADCSRA.complete(sample);
        serviceTemperatureAdc();
        assert((ADCSRA.value & (_BV(ADSC) | _BV(ADIF))) == _BV(ADSC));
        if (i<31) assert(temperatureAdcCount == i+1);
    }
    assert(temperatureAdcCount == 0 && temperatureAdcSum == 0);
    assert(temperatureAdcPosition == (index+1) % TemperatureAdcCount);
    assert(read(index, true) == sample*4);
    assert(HAL::temperatureAdcSample(index) == sample*4);
}

int main() {
    start();
    // ADSC clear without completion, or ADEN disabled, is not a new sample.
    ADCSRA.value = _BV(ADEN);
    for (unsigned i=0; i<64; ++i) serviceTemperatureAdc();
    assert(temperatureAdcCount == 0 && read(0, false) == 0);
    ADCSRA.value = _BV(ADIF);
    for (unsigned i=0; i<64; ++i) serviceTemperatureAdc();
    assert(temperatureAdcCount == 0 && read(0, false) == 0);
    start();
    for (uint8_t i=0; i<TemperatureAdcCount; ++i) {
        now = 8*(i+1);
        batch(i, uint16_t(100+i));
        if (i+1 < TemperatureAdcCount) assert(read(i+1, false) == 0);
    }
    now = 108;
    assert(read(0, true) == 400); // inclusive age boundary
    assert(read(0, true) == 400); // reading does not consume a batch
    ++now;
    assert(read(0, false) == 400);
    assert(read(1, true) == 404); // expiry is per channel
    now = 8; // observed expired sample must not revalidate on later wrap
    assert(read(0, false) == 400);
    now = 200;
    batch(0, 100); // same numerical value, new completed measurement
    assert(read(0, true) == 400);
    now = UINT32_MAX - 10;
    start();
    batch(0, 1023);
    now = 89; // modular age 100
    assert(read(0, true) == 4092);
    ++now;
    assert(read(0, false) == 4092);
    now = 0;
    start();
    batch(0, 0);
    assert(read(0, true) == 0); // both timestamp zero and raw zero can be real
    now = 10;
    for (unsigned i=0; i<31; ++i) {
        ADCSRA.complete(200);
        serviceTemperatureAdc();
    }
    assert(read(1, false) == 0);
    now = 101;
    assert(read(0, false) == 0);
    ADCSRA.complete(200);
    serviceTemperatureAdc();
    assert(read(1, true) == 800);
    start();
    std::printf("PASS ADC %u channels: startup, event flags, batches, mux, expiry and wrap\n", TemperatureAdcCount);
}
'''


def main():
    source = (FW / "HAL.cpp").read_text()
    adc = ("namespace {\n"
           + section(source, "const uint8_t temperatureAdcChannels", "#ifndef RF_HEATER_RESET_WDT_EXPERIMENT\nvolatile unsigned long watchdogHeartbeat")
           + "}\n"
           + section(source, "uint16_t HAL::temperatureAdcSample(", "//initialize watchdog"))
    with TemporaryDirectory(prefix="rfx-hal-adc-") as tmp:
        work = Path(tmp)
        (work / "adc.inc").write_text(adc)
        (work / "replay.cpp").write_text(HARNESS)
        for extruders, chamber in ((1, 0), (2, 1), (1, 1)):
            exe = work / "replay"
            subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-O2",
                            f"-DNUM_EXTRUDER={extruders}", f"-DCHAMBER={chamber}",
                            str(work / "replay.cpp"), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True, timeout=10)


if __name__ == "__main__":
    main()

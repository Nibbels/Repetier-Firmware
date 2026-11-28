#!/usr/bin/env python3
"""Replay the actual conversion code, with a host ADC/flash adapter.

Frozen transcript hashes from target 1b0dcee6c12fd6264caf29f75673960130483be2.
Every uint8 sensor ID and every 12-bit sample is tested for both board
calibrations, using both the diagnostic and standard ticket read paths.
This tests conversion and the range predicate, not lease grant or shutdown.
"""
import argparse
import hashlib
import os
from pathlib import Path
import re
import subprocess
from tempfile import TemporaryDirectory

GOLDEN = {
    "RF1000": "2fe65dbd5e680f0235a484cad5c511c2d17ccdbf830ff2092477ef4cf4a187ed",
    "RF2000": "793c4a4d06bac3ca88142ada1ff373c342f7fdf95e9bcece272d77b96df8ac6e",
}

HARNESS = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "ThermalEmergency.h"
#define PROGMEM
#define TEMP_INT_TO_FLOAT(t) ((float)(t) / (float)(1 << CELSIUS_EXTRA_BITS))
// AVR reads 16-bit words from flash, and the flash table pointers are 16-bit.
// The host retains native pointer width but preserves signed 16-bit table data.
static_assert(sizeof(short) == 2 && sizeof(float) == 4, "host widths");
static uint8_t pgm_read_byte(const uint8_t* p) { return *p; }
static int16_t pgm_read_word(const int16_t* p) { return *p; }
static const int16_t* pgm_read_word(const int16_t* const* p) { return *p; }
static unsigned reads, ticketReads;
static uint16_t sample;
static bool fresh = true;
struct HAL {
    static bool temperatureAdcSample(uint8_t index, uint16_t& value, uint32_t* publishedAt=nullptr) {
        assert(index == 2); ++reads; value = sample;
        if(fresh && publishedAt)*publishedAt=123;
        return fresh;
    }
    struct HeaterSample {
        bool read(uint8_t index, uint16_t& value, uint32_t* publishedAt) {
            ++ticketReads;
            return HAL::temperatureAdcSample(index, value, publishedAt);
        }
    };
};
struct TemperatureController {
    uint8_t sensorType, sensorPin;
    int16_t currentTemperature;
    float currentTemperatureC;
    bool updateCurrentTemperature(HAL::HeaterSample* = nullptr, ThermalEmergency::SensorObservation* = nullptr);
};
#include "conversion.inc"
static void emit(uint32_t v, unsigned bytes) {
    for (unsigned i = 0; i < bytes; ++i) std::putchar((v >> (8*i)) & 255);
}
int main() {
    for (unsigned type = 0; type < 256; ++type) {
        const bool ntc = (type >= 1 && type <= 4) || (type >= 8 && type <= 14);
        const bool analog = ntc || type == 53 || type == 60 || type == 100;
        for (unsigned raw = 0; raw < 4096; ++raw) {
            sample = raw; reads = ticketReads = 0;
            TemperatureController c{uint8_t(type), 2, -123, -999.0f};
            ThermalEmergency::SensorObservation observation;
            const bool supported = c.updateCurrentTemperature(nullptr, &observation);
            assert(observation.rawKnown==analog);
            if(analog)assert(observation.publishedAt==123 && observation.raw==raw);
            assert(supported == ((analog && raw > 0 && raw < 4092) || type == 0));
            assert(reads == unsigned(analog) && ticketReads == 0);
            TemperatureController measured{uint8_t(type), 2, -123, -999.0f};
            HAL::HeaterSample ticket;
            ThermalEmergency::SensorObservation measuredObservation;
            assert(measured.updateCurrentTemperature(&ticket, &measuredObservation) == supported);
            assert(reads == 2 * unsigned(analog) && ticketReads == unsigned(analog));
            assert(measured.currentTemperature == c.currentTemperature);
            assert(measured.currentTemperatureC == c.currentTemperatureC);
            assert(measuredObservation.rawKnown == observation.rawKnown);
            assert(measuredObservation.raw == observation.raw);
            assert(measuredObservation.publishedAt == observation.publishedAt);
            assert(measuredObservation.invalidCause == observation.invalidCause);
            assert(c.currentTemperature == (analog ? int(raw) : type == 0 ? 25 : 4095));
            assert(std::isfinite(c.currentTemperatureC));
            if (!analog && type != 0) assert(c.currentTemperatureC == MAX_DEFECT_TEMPERATURE + 1.0f);
            if (type == 0) assert(c.currentTemperatureC == 25.0f);
            const bool outsideRange = c.currentTemperatureC < MIN_DEFECT_TEMPERATURE ||
                                      c.currentTemperatureC > MAX_DEFECT_TEMPERATURE;
            uint32_t bits; std::memcpy(&bits, &c.currentTemperatureC, 4);
            emit(uint16_t(c.currentTemperature), 2); emit(bits, 4);
            // Retain the old supported-type bit in the conversion-only golden transcript.
            // Rail authorization is separately asserted above.
            emit(analog || type == 0, 1); emit(outsideRange, 1);
        }
    }
    // Stale ADC samples may be converted for diagnostics but never authorize heat.
    fresh = false;
    sample = 102;
    for (unsigned route=0; route<2; ++route) {
        TemperatureController c{60, 2, -123, -999.0f};
        HAL::HeaterSample ticket;
        ThermalEmergency::SensorObservation observation;
        reads=ticketReads=0;
        assert(!c.updateCurrentTemperature(route ? &ticket : nullptr, &observation));
        assert(reads==1 && ticketReads==route);
        assert(!observation.rawKnown && observation.invalidCause==ThermalEmergency::Cause::StaleSample);
    }
}
'''


def macro(text, name):
    return re.search(r"^#define " + name + r"\s+([^\n]+)", text, re.M)[1].strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware", type=Path, default=Path(__file__).resolve().parents[1] / "Repetier")
    parser.add_argument("--capture", action="store_true", help="print transcript hashes without checking (baseline capture only)")
    args = parser.parse_args()
    fw = args.firmware
    source = (fw / "TemperatureSensors.cpp").read_text()
    source = source[source.index('#include "Repetier.h"') + len('#include "Repetier.h"'):]
    with TemporaryDirectory(prefix="rfx-temp-sensors-") as tmp:
        work = Path(tmp)
        (work / "conversion.inc").write_text(source)
        (work / "replay.cpp").write_text(HARNESS)
        for board, expected in GOLDEN.items():
            profile = (fw / (board + ".h")).read_text()
            defines = {name: macro(profile, name) for name in (
                "SENSOR_TEMPERATURE_OFFSETS_C", "MIN_DEFECT_TEMPERATURE", "MAX_DEFECT_TEMPERATURE")}
            defines["ANALOG_REDUCE_BITS"] = macro((fw / "HAL.h").read_text(), "ANALOG_REDUCE_BITS")
            defines["CELSIUS_EXTRA_BITS"] = macro((fw / "Extruder.h").read_text(), "CELSIUS_EXTRA_BITS")
            assert defines["ANALOG_REDUCE_BITS"] == "0" and defines["CELSIUS_EXTRA_BITS"] == "3"
            exe = work / "replay"
            subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-ffp-contract=off",
                            "-Wall", "-Wextra", "-Werror", "-DRF_HEATER_LEASE_EXPERIMENT=1",
                            *[f"-D{k}={v}" for k,v in defines.items()],
                            "-I", str(fw), str(work / "replay.cpp"), "-o", str(exe)], check=True)
            output = subprocess.check_output([str(exe)], timeout=20)
            assert len(output) == 256 * 4096 * 8
            digest = hashlib.sha256(output).hexdigest()
            print(f"{board}: 1048576 conversions per read path plus stale checks, SHA256 {digest}", flush=True)
            if not args.capture:
                assert digest == expected, (board, expected, digest)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Compile the actual ScanSettings implementation with host-only printer/G-code stubs.

Factory travel/retreat fixtures are the four supported build configurations.
The common margin/lift definitions and all setter bodies come from firmware.
"""
import os
from pathlib import Path
import re
import subprocess
from tempfile import TemporaryDirectory

ROOT = Path(__file__).resolve().parents[1]
FW = ROOT / "Repetier"

STUB = r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include "ScanSettings.h"
enum { X_AXIS, Y_AXIS, Z_AXIS };
struct Printer {
    static float stepsPerMm(uint8_t axis) {
        return axis == Z_AXIS ? 1280.0f : 4.761875f * 32;
    }
    static float axisLengthMm(uint8_t axis) { return axis == X_AXIS ? X_LENGTH : Y_LENGTH; }
    static bool debugInfo() { return false; }
};
#define PSTR(x) x
struct Com {
    template<class... T> static void printF(T...) {}
    template<class... T> static void printFLN(T...) {}
};
struct GCode {
    int32_t S;
    int M = 0;
    bool present = true;
    explicit GCode(int32_t value) : S(value) {}
    bool hasS() const { return present; }
};
static int invalid = 0;
void showInvalidSyntax(int) { ++invalid; }
namespace Calibration { const ScanSettings& probeSettings(); }
'''

CHECKS = r'''
using namespace GCodeCommands::Calibration;
static void set(void (*fn)(GCode*), int32_t value) {
    GCode command(value);
    fn(&command);
}
int main() {
    const ScanSettings& s = ::Calibration::probeSettings();
    assert(scanStartLiftMm() == 1.0f);
    restoreDefaultScanParameters();
    const float xy = Printer::stepsPerMm(X_AXIS);
    const float z = Printer::stepsPerMm(Z_AXIS);
    assert(s.x.startSteps == long(xy * (NUM_EXTRUDER == 2 ? 0 : 15)));
    assert(s.x.endMarginSteps == long(xy * (NUM_EXTRUDER == 2 ? 0 : 5)));
    assert(s.x.maxPositionSteps == long(xy * (X_LENGTH - (NUM_EXTRUDER == 2 ? 0 : 5))));
    assert(s.y.startSteps == long(xy * 30));
    assert(s.y.endMarginSteps == long(xy * 5));
    assert(s.y.maxPositionSteps == long(xy * (Y_LENGTH - 5)));
    assert(s.x.strideMm == 20 && s.y.strideMm == 20);
    assert(s.x.strideSteps == long(xy * 20) && s.y.strideSteps == long(xy * 20));
    assert(s.fastApproachSteps == -25 && s.slowApproachSteps == -5);
    assert(s.fastRetreatSteps == short(z * HEAT_BED_SCAN_DOWN_FAST_MM));
    assert(s.slowRetreatSteps == 5 && s.maxTravelSteps == short(z * Z_ENDSTOP_DRIVE_OVER));
    assert(s.fastStepDelayMs == 1 && s.slowStepDelayMs == 100 && s.idleDelayMs == 250);
    assert(s.contactPressureDelta == 10 && s.retryPressureDelta == 5 && s.idlePressureDelta == 0);
    assert(s.idlePressureMin == -7500 && s.idlePressureMax == 7500);
    assert(s.pressureReadCount == 3 && s.pressureReadDelayMs == 15 && s.pressureTolerance == 15);

    for (int32_t requested : {-1000, 10, 37, 100, 1000}) {
        const long accepted = requested < 10 ? 10 : requested > 100 ? 100 : requested;
        const long oldY = scanStrideMm(Y_AXIS);
        set(setScanXStep, requested);
        assert(scanStrideMm(X_AXIS) == accepted && s.x.strideSteps == long(xy * accepted));
        assert(scanStrideMm(Y_AXIS) == oldY);
        set(setScanYStep, requested);
        assert(scanStrideMm(Y_AXIS) == accepted && s.y.strideSteps == long(xy * accepted));
    }
    GCode missing(20);
    missing.present = false;
    setScanXStep(&missing);
    setScanYStep(&missing);
    assert(invalid == 2 && scanStrideMm(X_AXIS) == 100 && scanStrideMm(Y_AXIS) == 100);

    set(setScanXStart, -100); set(setScanYStart, 1000);
    assert(s.x.startSteps == long(xy * 5) && s.y.startSteps == long(xy * (Y_LENGTH - 5)));
    set(setScanXStart, 1000); set(setScanYStart, -100);
    assert(s.x.startSteps == long(xy * (X_LENGTH - 5)) && s.y.startSteps == long(xy * 5));
    set(setScanXEnd, -100); set(setScanYEnd, 1000);
    assert(s.x.endMarginSteps == long(xy * 5) && s.y.endMarginSteps == long(xy * (Y_LENGTH - 5)));
    assert(s.x.maxPositionSteps == long(xy * X_LENGTH - s.x.endMarginSteps));
    assert(s.y.maxPositionSteps == long(xy * Y_LENGTH - s.y.endMarginSteps));
    set(setScanXEnd, 1000); set(setScanYEnd, -100);
    assert(s.x.endMarginSteps == long(xy * (X_LENGTH - 5)) && s.y.endMarginSteps == long(xy * 5));

    set(setFastApproachStep, -100); set(setSlowApproachStep, 1000);
    assert(s.fastApproachSteps == -1 && s.slowApproachSteps == -50);
    set(setFastApproachStep, 1000); set(setSlowApproachStep, -100);
    assert(s.fastApproachSteps == -50 && s.slowApproachSteps == -1);
    set(setFastRetreatStep, -100); set(setSlowRetreatStep, 1000);
    assert(s.fastRetreatSteps == 64 && s.slowRetreatSteps == 50);
    set(setFastRetreatStep, 100000); set(setSlowRetreatStep, -100);
    assert(s.fastRetreatSteps == 6400 && s.slowRetreatSteps == 1);
    set(setFastScanDelay, -100); set(setSlowScanDelay, 10000); set(setScanIdleDelay, 100000);
    assert(s.fastStepDelayMs == 1 && s.slowStepDelayMs == 1000 && s.idleDelayMs == 10000);
    set(setFastScanDelay, 10000); set(setSlowScanDelay, -100); set(setScanIdleDelay, -100);
    assert(s.fastStepDelayMs == 1000 && s.slowStepDelayMs == 1 && s.idleDelayMs == 1);
    set(setContactPressureDelta, -100); set(setRetryPressureDelta, 10000); set(setIdlePressureTolerance, 10000);
    assert(s.contactPressureDelta == 1 && s.retryPressureDelta == 1000 && s.idlePressureDelta == 1000);
    set(setContactPressureDelta, 10000); set(setRetryPressureDelta, -100); set(setIdlePressureTolerance, -100);
    assert(s.contactPressureDelta == 1000 && s.retryPressureDelta == 1 && s.idlePressureDelta == 1);
    set(setPressureReadCount, 1000); set(setPressureReadDelay, -100); set(setPressureReadTolerance, 10000);
    assert(s.pressureReadCount == 100 && s.pressureReadDelayMs == 1 && s.pressureTolerance == 1000);
    set(setPressureReadCount, -100); set(setPressureReadDelay, 10000); set(setPressureReadTolerance, -100);
    assert(s.pressureReadCount == 1 && s.pressureReadDelayMs == 1000 && s.pressureTolerance == 1);

    for (float lift : {0.3f, 6.0f}) {
        setScanStartLiftMm(lift);
        restoreDefaultScanParameters();
        assert(scanStartLiftMm() == lift && s.startLiftMm == lift);
        assert(scanStrideMm(X_AXIS) == 20 && scanStrideMm(Y_AXIS) == 20);
        assert(s.fastApproachSteps == -25 && s.contactPressureDelta == 10);
    }
    std::puts("ScanSettings defaults, clamps, LCD stride and lift retention: PASS");
}
'''


def main():
    firmware = (FW / "ScanSettings.cpp").read_text(encoding="utf-8")
    firmware = re.sub(r'^#include "[^"]+"\s*$', "", firmware, flags=re.M)
    rf = (FW / "RF.h").read_text(encoding="utf-8")
    constants = "\n".join(re.findall(
        r"^constexpr (?:int|float) (?:heatBedScan(?:XStart|XEnd|YStart|YEnd|MinStep)Mm|defaultHeatBedScanLiftMm) = [^;]+;",
        rf, flags=re.M))
    assert constants.count(";") == 6
    variants = (
        ("RF1000", 1, 245, 245, "0.8f", "0.2f"),
        ("RF2000", 2, 180, 245, "1.3f", "0.1f"),
        ("RF2000v2", 2, 170, 290, "3.0f", "0.2f"),
        ("RF2000-X44", 1, 230, 245, "1.3f", "0.1f"),
    )
    with TemporaryDirectory(prefix="rfx-scan-settings-") as temp:
        build = Path(temp)
        for name, tools, x, y, travel, retreat in variants:
            source = build / "scan-settings.cpp"
            source.write_text("#include <initializer_list>\n"
                              + f"#define NUM_EXTRUDER {tools}\n#define X_LENGTH {x}\n#define Y_LENGTH {y}\n"
                              + f"#define Z_ENDSTOP_DRIVE_OVER {travel}\n#define HEAT_BED_SCAN_DOWN_FAST_MM {retreat}\n"
                              + STUB + constants + "\n" + firmware + CHECKS, encoding="utf-8")
            exe = build / "scan-settings"
            print(name, flush=True)
            subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                            "-I", str(FW), str(source), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True, timeout=10)


if __name__ == "__main__":
    main()

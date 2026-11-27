#!/usr/bin/env python3
"""Replay the production RGB translation unit with AVR-width clock and Timer4 stubs."""
from pathlib import Path
import os
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
src = root / "Repetier"
# Use the actual profile, tolerance and EEPROM definitions; avoid compiling unrelated firmware.
defs = []
for name in ("RF2000.h", "Configuration.h", "Eeprom.h"):
    for line in (src / name).read_text(encoding="utf-8", errors="replace").splitlines():
        if re.match(r"#define\s+(?:RGB_(?:LIGHT_|HEATING_|PRINTING_|COOLING_|IDLE_|MANUAL_)\w+|EPR_RF_RGB_\w+|COOLDOWN_THRESHOLD|MAX_ROOM_TEMPERATURE)\s", line):
            defs.append(line)
stub = r'''
#pragma once
#include <stdint.h>
#include <cmath>
#include <cassert>
using std::fabs;
using PGM_P = const char*;
#define PSTR(s) s
enum { RGB_MODE_OFF, RGB_MODE_WHITE, RGB_MODE_AUTOMATIC, RGB_MODE_MANUAL };
enum { RGB_STATUS_NOT_AUTOMATIC, RGB_STATUS_AUTOMATIC, RGB_STATUS_HEATING,
       RGB_STATUS_PRINTING, RGB_STATUS_COOLING, RGB_STATUS_IDLE, RGB_STATUS_COLOR_CHANGE };
#ifdef ENABLE_RGB
#define RGB_LIGHT_R_PIN 6
#define RGB_LIGHT_G_PIN 7
#define RGB_LIGHT_B_PIN 8
#endif
enum { COM4A1=7, COM4B1=5, COM4C1=3 };
static uint8_t TCCR4A = 2; // WGM41 must survive RGB writes
static uint16_t OCR4A, OCR4B, OCR4C;
static uint32_t clockMs;
static unsigned writes, checksums, address, value;
struct InterruptProtectedBlock { InterruptProtectedBlock() {} ~InterruptProtectedBlock() {} };
struct HAL {
    static uint32_t timeInMilliseconds() { return clockMs; }
    static uint8_t eprGetByte(unsigned a) { return uint8_t(a==address?value:0xff); }
    static void eprSetByte(unsigned a, unsigned v) { ++writes; address=a; value=v; }
};
struct EEPROM {
    static void updateChecksum() { ++checksums; }
    static void storeRgbMode(uint8_t v);
    static void storeRgbColor(uint8_t profile,uint8_t channel,uint8_t v);
};
struct Com {
    static void printF(PGM_P) {}
    static void print(char) {}
    static void printFLN(PGM_P) {}
    static void printFLN(PGM_P, unsigned) {}
};
struct GCode {
    int P, S; bool p=true, s=true;
    GCode(int channel, int level): P(channel), S(level) {}
    bool hasP() const { return p; }
    bool hasS() const { return s; }
};
struct Temperature { float targetTemperatureC=0, currentTemperatureC=20; };
struct Extruder { Temperature tempControl; };
static Extruder extruder[NUM_EXTRUDER];
static Temperature heatedBedController;
'''
test = r'''
#include "RgbLighting.cpp"
#ifdef ENABLE_RGB
// Exercise the real semantic-to-storage binding, not a second mapping.
#include "rgb-storage.inc"
using namespace RgbLighting;
static void expect(int r, int g, int b) {
    assert((TCCR4A & 2) != 0);
    assert(((TCCR4A >> COM4A1) & 1) == (r != 0));
    assert(((TCCR4A >> COM4B1) & 1) == (g != 0));
    assert(((TCCR4A >> COM4C1) & 1) == (b != 0));
    if (r) assert(OCR4A == r*80);
    if (g) assert(OCR4B == g*80);
    if (b) assert(OCR4C == b*80);
}
static void settle(int r, int g, int b) {
    for (int i=0; i<255; ++i) advanceFade();
    expect(r,g,b);
    advanceFade(); expect(r,g,b);
}
static void cool() {
    for (auto& e: extruder) e.tempControl = Temperature();
    heatedBedController = Temperature();
}
static void idleAt(uint32_t start) {
    cool();
    selectMode(RGB_MODE_AUTOMATIC);
    clockMs=start;
    updateStatus();
    settle(RGB_IDLE_R,RGB_IDLE_G,RGB_IDLE_B);
    clockMs=start+uint32_t(RGB_LIGHT_COLOR_CHANGE_DELAY*1000+999);
    updateStatus();
    settle(RGB_IDLE_R,RGB_IDLE_G,RGB_IDLE_B);
    clockMs=start+uint32_t((RGB_LIGHT_COLOR_CHANGE_DELAY+1)*1000);
    updateStatus(); settle(255,0,0);
    updateStatus(); settle(255,255,0);
    updateStatus(); settle(0,255,0);
    updateStatus(); settle(0,255,255);
    updateStatus(); settle(0,0,255);
    updateStatus(); settle(255,0,255);
    updateStatus(); settle(255,0,0);
}
int main() {
    reset(); initialize(); expect(0,0,0);
    assert(mode()==RGB_LIGHT_DEFAULT_MODE);
    advanceFade(); expect(1,1,1);
    settle(255,255,255);
    selectMode(RGB_MODE_OFF); assert(address==EPR_RF_RGB_LIGHT_MODE && value==RGB_MODE_OFF);
    advanceFade(); expect(254,254,254); settle(0,0,0);
    requestWhiteToggle(); updateStatus(); assert(forcedWhite()); settle(255,255,255);
    requestWhiteToggle(); updateStatus(); assert(!forcedWhite()); settle(0,0,0);
    selectMode(RGB_MODE_MANUAL); settle(RGB_MANUAL_R,RGB_MANUAL_G,RGB_MANUAL_B);
    GCode edit(1, 42);
    requestWhiteToggle(); updateStatus(); settle(255,255,255);
    unsigned count=writes;
    setManualLight(&edit); assert(writes==count+1 && address==EPR_RF_RGB_MANUAL_R && value==42);
    assert(checksums==writes);
    updateStatus(); settle(255,255,255);
    requestWhiteToggle(); updateStatus(); settle(42,RGB_MANUAL_G,RGB_MANUAL_B);
    GCode invalid(1,256); count=writes; setManualLight(&invalid); assert(writes==count);
    invalid.S=-1; setManualLight(&invalid); assert(writes==count);
    invalid.P=4; invalid.S=1; setManualLight(&invalid); assert(writes==count);
    invalid.P=1; invalid.p=false; setManualLight(&invalid); assert(writes==count);
    invalid.p=true; invalid.s=false; setManualLight(&invalid); assert(writes==count);
    idleAt(100); idleAt(0xfffffff0u); idleAt(0);
    // Temperature-driven status interrupts the color cycle.
    extruder[0].tempControl.targetTemperatureC=200;
    extruder[0].tempControl.currentTemperatureC=100;
    updateStatus(); settle(RGB_HEATING_R,RGB_HEATING_G,RGB_HEATING_B);
    requestWhiteToggle(); updateStatus(); settle(255,255,255);
    GCode heating(2,37); setHeatingLight(&heating);
    assert(address==EPR_RF_RGB_HEATING_G && value==37);
    updateStatus(); settle(255,255,255);
    requestWhiteToggle(); updateStatus(); settle(RGB_HEATING_R,37,RGB_HEATING_B);
    extruder[0].tempControl.currentTemperatureC=200;
    updateStatus(); settle(RGB_PRINTING_R,RGB_PRINTING_G,RGB_PRINTING_B);
    heatedBedController.targetTemperatureC=60;
    heatedBedController.currentTemperatureC=30;
    updateStatus(); settle(RGB_HEATING_R,37,RGB_HEATING_B);
    heatedBedController.currentTemperatureC=60;
    updateStatus(); settle(RGB_PRINTING_R,RGB_PRINTING_G,RGB_PRINTING_B);
    if (NUM_EXTRUDER>1) {
        extruder[1].tempControl.targetTemperatureC=200;
        extruder[1].tempControl.currentTemperatureC=100;
        updateStatus(); settle(RGB_HEATING_R,37,RGB_HEATING_B);
        extruder[1].tempControl.currentTemperatureC=200;
        extruder[0].tempControl.currentTemperatureC=100;
        // Inherited ordering: last active hotend wins. Do not silently change policy.
        updateStatus(); settle(RGB_PRINTING_R,RGB_PRINTING_G,RGB_PRINTING_B);
    }
    for (auto& e: extruder) e.tempControl.targetTemperatureC=0;
    heatedBedController.targetTemperatureC=0;
    updateStatus(); settle(RGB_COOLING_R,RGB_COOLING_G,RGB_COOLING_B);
    cool(); updateStatus(); settle(RGB_IDLE_R,RGB_IDLE_G,RGB_IDLE_B);
    GCode modeCommand(RGB_MODE_OFF,0); modeCommand.s=false;
    setRgbLightMode(&modeCommand); settle(0,0,0);
    modeCommand.P=4; count=writes; setRgbLightMode(&modeCommand);
    assert(mode()==RGB_MODE_OFF && writes==count);
    restoreMode(RGB_MODE_MANUAL); initialize(); settle(42,RGB_MANUAL_G,RGB_MANUAL_B);
    selectMode(RGB_MODE_WHITE); settle(255,255,255);
    requestWhiteToggle(); updateStatus(); requestWhiteToggle(); updateStatus();
    settle(255,255,255); assert(!forcedWhite());
    assert(checksums==writes);
}
#else
int main() { assert(TCCR4A==2 && OCR4A==0 && OCR4B==0 && OCR4C==0); }
#endif
'''
with tempfile.TemporaryDirectory(prefix="rgb-replay-") as tmp:
    path = Path(tmp)
    (path / "Repetier.h").write_text(stub + "\n" + "\n".join(defs) + "\n")
    (path / "RgbLighting.cpp").write_bytes((src / "RgbLighting.cpp").read_bytes())
    (path / "main.cpp").write_text(test)
    owner=(src / "Eeprom.cpp").read_text(encoding="utf-8")
    byte=owner[owner.index("namespace {\nvoid saveLegacyByte"):owner.index("void EEPROM::storeCaseLight")]
    rgb=owner[owner.index("void EEPROM::storeRgbMode("):owner.index("void EEPROM::storeFanDivisor(")]
    (path / "rgb-storage.inc").write_text(byte+rgb,encoding="utf-8")
    for variant, count, enabled in (("RF1000", 1, False), ("RF2000", 2, True),
                                    ("RF2000v2", 2, True), ("RF2000-X44", 1, True)):
        args = [os.environ.get("CXX", "g++"), "-std=c++11", "-O2",
                "-DNUM_EXTRUDER=" + str(count)]
        if enabled:
            args += ["-DENABLE_RGB"]
        subprocess.run(args + [str(path / "main.cpp"), "-o", str(path / "replay")], check=True)
        subprocess.run([str(path / "replay")], check=True)
        print(variant + ": RGB replay passed")

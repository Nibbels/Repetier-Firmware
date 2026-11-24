#!/usr/bin/env python3
"""Replay actual M205 writers and M206 dispatch against frozen EEPROM fixtures.

Host HAL models little-endian AVR cells, not electrical write latency.
Full module loading, boot migration and physical power-loss tests are separate.
"""
import json
import os
from pathlib import Path
import re
import struct
import subprocess
from tempfile import TemporaryDirectory

ROOT = Path(__file__).resolve().parent.parent
FW = ROOT / "Repetier"
TEST = ROOT / "tests"


def rows(path):
    return [line.split("\t") for line in path.read_text().splitlines()
            if line and not line.startswith("#")]


def section(source, start, end):
    return source[source.index(start):source.index(end, source.index(start))]


def method(source, name):
    match = re.search(r"^[^\n]*EEPROM::" + name + r"\([^\n]*\) \{", source, re.M)
    if not match:
        raise AssertionError(name)
    start = match.start()
    depth = 1
    i = match.end()
    while depth:
        depth += (source[i] == "{") - (source[i] == "}")
        i += 1
    return source[start:i]


def fixture(variant):
    image = bytearray([0xA5] * 2048)
    used = set()
    lines = []
    for model, address, kind, digits, label in rows(TEST / "eeprom-report-golden.tsv"):
        if model != variant:
            continue
        address, kind, digits = int(address), int(kind), int(digits)
        value = [128 + address % 128, -address, -1000000 - address,
                 -(address % 20 + 0.25)][kind]
        raw = struct.pack(["<B", "<h", "<i", "<f"][kind], value)
        span = set(range(address, address + len(raw)))
        assert not used & span and max(span) < len(image), address
        used |= span
        image[address:address + len(raw)] = raw
        rendered = f"{value:.{digits}f}" if kind == 3 else str(value)
        prefix = f"Extr.{(address - 200) // 100 + 1} " if 200 <= address < 800 else ""
        lines.append(f"EPR:{kind} {address} {rendered} {prefix}{label}\r\n")
    assert lines
    return ("static const uint8_t initial[] = {" + ",".join(map(str, image)) + "};\n"
            + "static const char wanted[] = " + json.dumps("".join(lines)) + ";\n"
            + f"static const unsigned reportLines = {len(lines)};\n")


def main():
    source = (FW / "Eeprom.cpp").read_text()
    header = (FW / "Eeprom.h").read_text()
    actual_layout = dict(re.findall(
        r"^#define (EPR_\w+|EEPROM_EXTRUDER_\w+)\s+(\d+)\b", header, re.M))
    assert actual_layout == dict(rows(TEST / "eeprom-layout-golden.tsv"))
    assert re.search(r"^#define EEPROM_PROTOCOL_VERSION 8\b", header, re.M)
    assert re.search(r"^constexpr uint8_t eepromMagic = 44;", source, re.M)
    communication = (FW / "Communication.cpp").read_text()
    com_header = (FW / "Communication.h").read_text().replace("int value)", "int16_t value)")
    constants = "\n".join(re.findall(
        r'^FSTRINGVALUE\(Com::t\w+, "(?:[^"\\]|\\.)*"\)', communication, re.M))
    # The 16-bit host mapping follows tests/communication-format.py. On
    # single-tool builds the header omits E1; omit its definition as well.
    com_methods = communication[communication.index("void Com::cap("):].replace(
        "int value)", "int16_t value)")
    methods = "\n".join(method(source, name) for name in (
        "update", "computeChecksum", "updateChecksum", "clearEEPROM",
        "getExtruderOffset", "storeManualZOffset"))
    # All report writers (and any private helpers) are in the tail.
    methods += "\n" + section(source, "void EEPROM::writeSettings()", "uint8_t EEPROM::computeChecksum()")
    methods += "\n" + source[source.index("void EEPROM::writeExtruderPrefix("):]
    harness = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
using std::isnan;
using std::isinf;
using uint = unsigned int;
using millis_t = uint32_t;
using PGM_P = const char*;
#define PROGMEM
#define FSTRINGPARAM(name) const char* name
#define FSTRINGVAR(name) static const char name[];
#define FSTRINGVALUE(name, value) const char name[] = value;
#define PSTR(value) value
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3
#define DEVICE_TYPE_RF1000 1
#define DEVICE_TYPE_RF2000 2
#define DEVICE_TYPE_RF2000v2 3
#define EEPROM_OFFSET 0
#define PERIODICAL_ACTIONS_CALL_INTERVAL 100
#define Processing 0
static std::string output;
static uint8_t bytes[2048];
static std::vector<unsigned> delays;
static unsigned loads, periodic;
static millis_t clockMs;
struct Commands { static void checkForPeriodicalActions(int) { ++periodic; } };
struct Printer {
    struct Offset { int32_t manualMicrometers() const { return -12345; } };
    static Offset zOffset;
};
Printer::Offset Printer::zOffset;
struct HAL {
    static char readFlashByte(const char* p) { return *p; }
    static void serialWriteByte(char c) { output += c; }
    static millis_t timeInMilliseconds() { return clockMs++; }
    static void delayMilliseconds(unsigned ms) { delays.push_back(ms); }
    static uint32_t get(unsigned pos, unsigned count) {
        assert(pos + count <= sizeof bytes);
        uint32_t value = 0;
        for (unsigned i = 0; i < count; ++i) value |= uint32_t(bytes[pos+i]) << (8*i);
        return value;
    }
    static void put(unsigned pos, unsigned count, uint32_t value) {
        assert(pos + count <= sizeof bytes);
        for (unsigned i = 0; i < count; ++i) bytes[pos+i] = uint8_t(value >> (8*i));
    }
    static uint8_t eprGetByte(unsigned p) { return get(p, 1); }
    static int16_t eprGetInt16(unsigned p) { return int16_t(get(p, 2)); }
    static int32_t eprGetInt32(unsigned p) { return int32_t(get(p, 4)); }
    static float eprGetFloat(unsigned p) {
        uint32_t raw = get(p, 4); float f; std::memcpy(&f, &raw, 4); return f;
    }
    static void eprSetByte(unsigned p, uint8_t v) { put(p, 1, v); }
    static void eprSetInt16(unsigned p, int16_t v) { put(p, 2, uint16_t(v)); }
    static void eprSetInt32(unsigned p, int32_t v) { put(p, 4, uint32_t(v)); }
    static void eprSetFloat(unsigned p, float f) {
        uint32_t raw; std::memcpy(&raw, &f, 4); put(p, 4, raw);
    }
};
struct GCode {
    uint8_t T; uint16_t P; int32_t S; float X;
    bool t = true, p = true, s = true, x = true;
    bool hasT() const { return t; } bool hasP() const { return p; }
    bool hasS() const { return s; } bool hasX() const { return x; }
};
#include "communication-host.h"
#include "communication-methods.inc"
#include "Eeprom.h"
#include "eeprom-methods.inc"
#include "fixture.inc"
void EEPROM::readDataFromEEPROM() { ++loads; }
static uint8_t independentChecksum(const uint8_t* image) {
    unsigned sum = 0;
    for (unsigned p = 0; p < sizeof bytes; ++p) if (p != 104) sum += image[p];
    return uint8_t(sum);
}
static void expectWrite(GCode& code, const std::vector<uint8_t>& payload) {
    uint8_t expected[sizeof bytes];
    std::memcpy(expected, bytes, sizeof bytes);
    for (unsigned i = 0; i < payload.size(); ++i) expected[code.P+i] = payload[i];
    expected[104] = independentChecksum(expected);
    unsigned before = loads;
    EEPROM::update(&code);
    assert(loads == before + 1);
    assert(std::memcmp(bytes, expected, sizeof bytes) == 0);
}
int main() {
    static_assert(sizeof(float) == 4, "AVR float fixture width");
    std::memcpy(bytes, initial, sizeof bytes);
    EEPROM::writeSettings();
    if (output != wanted) {
        std::cerr << "M205 expected:\n" << wanted << "\nactual:\n" << output;
        return 1;
    }
    assert(delays.size() == reportLines);
    assert(std::all_of(delays.begin(), delays.end(), [](unsigned ms) { return ms == 4; }));
    assert(std::memcmp(bytes, initial, sizeof bytes) == 0);
    EEPROM::updateChecksum();
    assert(bytes[104] == independentChecksum(bytes) && periodic > 0);
    GCode code; code.P = 1800; code.T = 0; code.S = 255;
    expectWrite(code, {0xff});
    code.T = 1; code.S = -32768; expectWrite(code, {0, 0x80});
    code.T = 2; code.S = INT32_MIN; expectWrite(code, {0, 0, 0, 0x80});
    code.T = 3; code.X = -1.25f; expectWrite(code, {0, 0, 0xa0, 0xbf});
    code.x = false; expectWrite(code, {});
    code.T = 0; code.s = false; expectWrite(code, {});
    code.s = true; code.t = false; expectWrite(code, {});
    code.t = true; code.p = false; expectWrite(code, {});
    code.p = true; code.T = 9; expectWrite(code, {});
    uint8_t expected[sizeof bytes]; std::memcpy(expected, bytes, sizeof bytes);
    expected[1038] = 0xc7; expected[1039] = 0xcf;
    expected[1040] = expected[1041] = 0xff;
    expected[104] = independentChecksum(expected);
    EEPROM::storeManualZOffset();
    assert(std::memcmp(bytes, expected, sizeof bytes) == 0);
    EEPROM::storeManualZOffset();
    assert(std::memcmp(bytes, expected, sizeof bytes) == 0);
    assert(EEPROM::getExtruderOffset(0) == 200 && EEPROM::getExtruderOffset(1) == 300);
    periodic = 0; EEPROM::clearEEPROM();
    assert(std::all_of(bytes, bytes + sizeof bytes, [](uint8_t b) { return b == 0; }));
    assert(periodic > 0);
    std::cout << "EEPROM golden report, raw M206 writes and checksum passed\n";
}
'''
    with TemporaryDirectory(prefix="rfx-eeprom-") as tmp:
        build = Path(tmp)
        (build / "communication-host.h").write_text(com_header)
        (build / "eeprom-methods.inc").write_text(methods)
        (build / "replay.cpp").write_text(harness)
        for variant, n, board in (
            ("RF1000", 1, 1), ("RF2000", 2, 2),
            ("RF2000", 2, 3), ("RF2000-X44", 1, 2),
        ):
            values = constants if n > 1 else re.sub(
                r"^FSTRINGVALUE\(Com::tEPRPrinter_STEPPER_E1,.*\)\n?", "", constants, flags=re.M)
            (build / "communication-methods.inc").write_text(values + "\n" + com_methods)
            (build / "fixture.inc").write_text(fixture(variant))
            executable = build / "replay"
            flags = [f"-DNUM_EXTRUDER={n}", f"-DMOTHERBOARD={board}"]
            if board != 1:
                flags.append("-DRGB_LIGHT_R_PIN=1")
            subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-O2",
                            "-Wall", "-Wextra", "-Werror", *flags, "-I", str(FW),
                            str(build / "replay.cpp"), "-o", str(executable)], check=True)
            subprocess.run([str(executable)], check=True, timeout=10)
            print(variant, "board", board, "passed", flush=True)


if __name__ == "__main__":
    main()

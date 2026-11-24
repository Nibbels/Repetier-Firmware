#!/usr/bin/env python3
"""Golden output replay of Com numeric formatting, with a byte-capture HAL.

The host replay maps AVR int parameters to int16_t to preserve overload
resolution. It captures output bytes using the host's floating-point ABI.
"""
import argparse
import os
from pathlib import Path
import re
import subprocess
from tempfile import TemporaryDirectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ubsan', action='store_true', help='detect signed overflow in the formatter')
    args = parser.parse_args()
    firmware = Path(__file__).resolve().parent.parent / 'Repetier'
    source = (firmware / 'Communication.cpp').read_text(encoding='utf-8')
    header = (firmware / 'Communication.h').read_text(encoding='utf-8')
    # Keep the AVR int overloads distinct on a host where int32_t is int.
    header = header.replace('#include "HAL.h"\n', '').replace('int value)', 'int16_t value)')
    methods = source[source.index('void Com::cap('):].replace('int value)', 'int16_t value)')
    constants = '\n'.join(re.findall(r'^FSTRINGVALUE\(Com::t\w+, "(?:[^"\\]|\\.)*"\)', source, re.M))
    harness = r'''
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
using std::isnan;
using std::isinf;
static std::string output;
struct HAL {
    static char readFlashByte(const char* p) { return *p; }
    static void serialWriteByte(char c) { output += c; }
};
#define PROGMEM
#define FSTRINGPARAM(name) const char* name
#define FSTRINGVAR(name) static const char name[];
#define FSTRINGVALUE(name, value) const char name[] = value;
#define PSTR(value) value
#define NUM_EXTRUDER 2
#include "communication-host.h"
#include "communication-methods.inc"
static void expect(const char* label, const std::string& wanted) {
    if (output != wanted) {
        std::cerr << label << ": expected [" << wanted << "] got [" << output << "]\n";
        std::exit(1);
    }
    output.clear();
}
int main() {
    Com::print(int32_t(0)); expect("zero", "0");
    Com::print(int32_t(-17)); expect("negative", "-17");
    Com::print(std::numeric_limits<int32_t>::max()); expect("signed max", "2147483647");
    Com::print(std::numeric_limits<int32_t>::min()); expect("signed min", "-2147483648");
    Com::print(std::numeric_limits<uint32_t>::max()); expect("unsigned max", "4294967295");
    Com::printNumber(1000000000u); expect("powers of ten", "1000000000");
    Com::printFloat(1.999f, 2); expect("rounding carry", "2.00");
    Com::printFloat(-1.25f, 2); expect("negative float", "-1.25");
    // Preserve observed legacy host rounding; AVR float evaluation must be checked separately.
    Com::printFloat(0.125f, 2, true); expect("legacy halfway rounding", "0,12");
    Com::printFloat(1.5f, 0, true); expect("zero decimals", "2");
    Com::printFloat(-0.0f, 2); expect("negative zero", "0.00");
    Com::printFloat(std::numeric_limits<float>::quiet_NaN(), 2); expect("NaN", "NAN");
    Com::printFloat(std::numeric_limits<float>::infinity(), 2); expect("Inf", "INF");
    Com::printFloat(-std::numeric_limits<float>::infinity(), 2); expect("negative Inf", "INF");
    Com::printF("N:", int32_t(-12)); expect("signed prefix", "N:-12");
    Com::printF("N:", std::numeric_limits<int16_t>::min()); expect("16-bit signed prefix", "N:-32768");
    Com::printFLN("N:", std::numeric_limits<int16_t>::max()); expect("16-bit signed line", "N:32767\r\n");
    Com::printFLN("N:", std::numeric_limits<int32_t>::min()); expect("32-bit signed line", "N:-2147483648\r\n");
    Com::printFLN("U:", std::numeric_limits<uint32_t>::max()); expect("unsigned line", "U:4294967295\r\n");
    Com::printFLN("F:", 1.25f, 2, true); expect("float line", "F:1,25\r\n");
    Com::printFLN("S:", "abc"); expect("text line", "S:abc\r\n");
    Com::printErrorFLN("bad"); expect("error", "Error:bad\r\n");
    Com::printWarningFLN("bad"); expect("warning", "Warning:bad\r\n");
    Com::printInfoFLN("ok"); expect("info", "Info:ok\r\n");
    Com::cap("X:1"); expect("cap", "Cap:X:1\r\n");
    Com::config("V:", int16_t(-17)); expect("config 16-bit signed", "Config:V:-17\r\n");
    Com::config("V:", int32_t(-17)); expect("config signed", "Config:V:-17\r\n");
    Com::config("V:", uint32_t(17)); expect("config unsigned", "Config:V:17\r\n");
    Com::config("V:", 1.25f, 2); expect("config float", "Config:V:1.25\r\n");
    Com::config("V:", "abc"); expect("config text", "Config:V:abc\r\n");
    int32_t integers[] = {-1, 0, 2};
    Com::printArrayFLN("I:", integers, 3); expect("integer array", "I: -1 0 2\r\n");
    float floats[] = {-1.25f, 0.0f, 2.5f};
    Com::printArrayFLN("F:", floats, 3, 2); expect("float array", "F: -1.25 0.00 2.50\r\n");
    Com::printSharpLine(); expect("sharp line", std::string(70, '#') + "\r\n");
    std::cout << "communication golden output passed\n";
}
'''
    with TemporaryDirectory(prefix='rfx-communication-') as directory:
        build = Path(directory)
        (build / 'communication-host.h').write_text(header, encoding='utf-8')
        (build / 'communication-methods.inc').write_text(constants + '\n' + methods, encoding='utf-8')
        (build / 'replay.cpp').write_text(harness, encoding='utf-8')
        executable = build / 'replay'
        flags = ['-fsanitize=undefined', '-fno-sanitize-recover=undefined'] if args.ubsan else []
        subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror', *flags, str(build / 'replay.cpp'), '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True, timeout=10)


if __name__ == '__main__':
    main()

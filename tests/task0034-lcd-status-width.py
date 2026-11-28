#!/usr/bin/env python3
"""Actual-source UI status storage replay; two physical widths, four profiles."""
import os
from pathlib import Path
import re
import subprocess
from tempfile import TemporaryDirectory

fw = Path(__file__).resolve().parents[1] / "Repetier"
h = (fw / "ui.h").read_text()
fmt = (fw / "UiFormatting.cpp").read_text()

def section(s, first, last):
    start = s.index(first)
    return s[start:s.index(last, start + len(first))]

geometry = section(h, "#define UI_ROWS 4", "//mtype usw.")
max_cols = re.search(r"^#define MAX_COLS \d+", h, re.M)[0]
storage = re.search(r"^\s*char statusMsg\[[^\n;]+\];", h, re.M)[0]
parser = section(fmt, "void UIDisplay::parse(", "// Format a complete placeholder")
percent = section(fmt, "        case '%': {", "        case 'V': {")
status = section(fmt, "        case 'o': {", "            if (c2 == 'f')")
setters = section(fmt, "void UIDisplay::setStatusP(", "} // setStatus\n") + "}\n"

harness = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#define DEVICE_TYPE_RF1000 1
#define DEVICE_TYPE_RF2000 2
#define DEVICE_TYPE_RF2000v2 3
using PGM_P = const char*;
uint8_t pgm_read_byte(const char* p) { return uint8_t(*p); }
namespace Printer {
bool error = false;
bool isUIErrorMessage() { return error; }
void setUIErrorMessage(bool v) { error = v; }
}
#include "geometry.inc"
static_assert(UI_ROWS == 4 && UI_COLS == EXPECTED_COLS, "physical geometry");
struct UIDisplay {
    uint8_t left = 0xa5;
#include "storage.inc"
    uint8_t right = 0x5a;
    bool statusIsTemplate_ = false;
    uint8_t col = 0;
    char printCols[MAX_COLS+1] = {};
    void parse(char*, bool, bool = true);
    void renderField(char, char, bool, bool, int&, float&);
    void setStatusP(PGM_P, bool = false);
    void setStatus(char*, bool = false);
    void checkGuards() const {
        assert(left == 0xa5 && right == 0x5a);
    }
    std::string rendered() {
        char field[] = "%os";
        col = 0;
        parse(field, false);
        checkGuards();
        return printCols;
    }
};
static_assert(sizeof(UIDisplay::statusMsg) == UI_COLS+1, "status capacity");
#include "firmware.inc"

// Only the percent escape and status expansion are exercised here.
// Other dynamic fields are covered by the existing firmware replays.
std::string templateExpected(const std::string& clipped) {
    std::string out;
    for (size_t i = 0; i < clipped.size();) {
        char ch = clipped[i++];
        if (ch != '%') { out += ch; continue; }
        if (i == clipped.size()) { out += '%'; break; }
        char first = clipped[i++];
        if (i == clipped.size()) { out += '%'; out += first; break; }
        char second = clipped[i++];
        if (first == '%' && second == '%') out += '%';
        else assert(first == 'o' && second == 's');
    }
    return out;
}

int main() {
    UIDisplay ui;
    unsigned cases = 0;
    auto check = [&](std::string input) {
        const auto clipped = input.substr(0, UI_COLS);
        const auto expected = templateExpected(clipped);
        for (bool flash : {false,true}) {
            Printer::error = false;
            std::memset(ui.statusMsg, 0x7e, sizeof(ui.statusMsg));
            if (flash) ui.setStatusP(input.c_str());
            else ui.setStatus(&input[0]);
            assert(ui.statusIsTemplate_ == flash);
            assert(std::string(ui.statusMsg) == clipped);
            assert(ui.rendered() == (flash ? expected : clipped));
            // An error freezes ordinary updates, regardless of source type.
            ui.setStatusP("FAULT", true);
            assert(Printer::error);
            ui.setStatusP(input.c_str());
            ui.setStatus(&input[0]);
            assert(ui.rendered() == "FAULT");
            // Another explicit error may replace it.
            ui.setStatus(&input[0], true);
            assert(!ui.statusIsTemplate_);
            assert(ui.rendered() == clipped);
            ui.checkGuards();
            ++cases;
        }
    };
    for (unsigned n = 0; n <= 64; ++n) check(std::string(n, 'X'));
    for (unsigned prefix = 0; prefix <= 32; ++prefix) {
        for (unsigned suffix = 0; suffix <= 32; ++suffix) {
            for (const char* token : {"%%%","%os"}) {
                check(std::string(prefix,'X') + token + std::string(suffix,'Y'));
            }
        }
        check(std::string(prefix,'X') + "%");
        check(std::string(prefix,'X') + "%o");
    }
    check("%%%foo%osbar%%%baz");
    std::cout << "UI status: " << UI_COLS << " cols, " << cases
              << " storage/render/error cases PASS\n";
}
'''

render = """void UIDisplay::renderField(char c1, char c2, bool ram, bool allowMessageExpansion, int& ivalue, float& fvalue) {
    (void)allowMessageExpansion; (void)ivalue; (void)fvalue;
    switch(c1) {
""" + percent + status + """
            break;
        }
        default: assert(false && "unexpected fixture field");
    }
}
"""

with TemporaryDirectory(prefix="rfx-ui-status-") as tmp:
    p = Path(tmp)
    (p / "geometry.inc").write_text(geometry + "\n" + max_cols + "\n")
    (p / "storage.inc").write_text(storage + "\n")
    (p / "firmware.inc").write_text(parser + render + setters)
    (p / "test.cpp").write_text(harness)
    for board, width, extruders in ((1,16,1),(2,20,2),(3,20,2),(2,20,1)):
        exe = p / "test"
        subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-O1",
                        "-Wall", "-Wextra", "-Werror", "-fno-pie", "-no-pie",
                        "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
                        f"-DMOTHERBOARD={board}", f"-DEXPECTED_COLS={width}",
                        f"-DNUM_EXTRUDER={extruders}", str(p / "test.cpp"),
                        "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True, timeout=10,
                       env={**os.environ, "ASAN_OPTIONS": "detect_leaks=0"})

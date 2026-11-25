"""Exhaustive check of the actual flash period table source against the legacy interval."""
import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

root = Path(__file__).resolve().parents[1]
source = (root / "Repetier" / "HAL.cpp").read_text()
start = source.index("// Task0025 period tables:")
end = source.index("} // CPUDivU2", start) + len("} // CPUDivU2")
harness = r"""
#include <cstdint>
#include <cstdio>
#include <cassert>
#define F_CPU 16000000UL
#define PROGMEM
#define pgm_read_word(p) (*(p))
#define pgm_read_dword(p) (*(p))
struct HAL { static int32_t CPUDivU2(unsigned int divisor); };
""" + source[start:end] + r"""
static uint32_t reference(uint32_t d) {
    if (d < 512)
        return 16000000UL / (d < 10 ? 10 : d);
    const uint32_t width = d < 8192 ? 32 : 4096;
    const uint32_t base = d - d % width;
    const uint32_t a = 16000000UL / base;
    const uint32_t b = 16000000UL / (base + width);
    return a - (a - b) * (d - base) / width;
}
int main() {
    uint32_t prev = UINT32_MAX;
    for (uint32_t d = 0; d <= 65535; ++d) {
        const uint32_t got = HAL::CPUDivU2(d);
        const uint32_t expected = reference(d);
        if (got != expected || got > prev) {
            std::fprintf(stderr, "divisor=%lu got=%lu expected=%lu previous=%lu\n",
                         (unsigned long)d, (unsigned long)got,
                         (unsigned long)expected, (unsigned long)prev);
            return 1;
        }
        prev = got;
    }
    static_assert(sizeof(lowPeriods) == 2048, "Unexpected low table size");
    static_assert(sizeof(midPeriods) == 15360, "Unexpected mid table size");
    static_assert(sizeof(high4Periods)+sizeof(high8Periods)+sizeof(high16Periods)==9216,
                  "Unexpected packed table size");
    std::puts("All 65536 legacy period values and monotonicity verified");
}
"""
with TemporaryDirectory() as tmp:
    src = Path(tmp) / "period.cpp"
    exe = Path(tmp) / "period"
    src.write_text(harness)
    subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-O2",
                    "-Wall", "-Wextra", "-Werror", str(src), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)

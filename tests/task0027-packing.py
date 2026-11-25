"""Check the actual packing block, using AVR-width integer types.

The UI/EEPROM restrict the packing threshold to 3300..5000. G-code/UI
Digit Flow speed percent is -90..99 with a 0.01 regulator step; at the
fast end the interval cannot fall below roughly 255 ticks. Use a wider
tested positive domain (40..5001) to cover every signed fast8_t pack count.
"""
import os
from pathlib import Path
import re
import subprocess
from tempfile import TemporaryDirectory

root = Path(__file__).resolve().parents[1]
source = (root / "Repetier" / "StepExecutor.cpp").read_text()
start = source.index("    // Packing keeps the legacy threshold")
end = source.index("    if (move->stepsRemaining <= 0", start)
block = source[start:end]
for board in ("RF1000.h", "RF2000.h"):
    config = (root / "Repetier" / board).read_text()
    lo = int(re.search(r"#define MIN_STEP_PACKING_MIN_INTERVAL\s+(\d+)", config)[1])
    hi = int(re.search(r"#define MAX_STEP_PACKING_MIN_INTERVAL\s+(\d+)", config)[1])
    assert (lo, hi) == (3300, 5000)
harness = r"""
#include <cstdint>
#include <cstdio>
#include <cassert>
#define MAX_STEP_PACKING_MIN_INTERVAL 5000

static uint32_t pack(uint32_t interval, uint16_t packingInterval_, uint8_t& stepsPerTimerCall) {
""" + block + r"""
    return interval;
}
int main() {
    unsigned long count = 0;
    for (uint16_t threshold = 3300; threshold <= 5000; ++threshold) {
        for (uint32_t single = 40; single <= 5001; ++single) {
            uint8_t pulses = 0;
            uint32_t result = pack(single, threshold, pulses);
            uint32_t n = single >= threshold ? 1 : (threshold + single - 1) / single;
            assert(n <= 127 && pulses == n && result == single * n);
            ++count;
        }
        for (uint32_t single : {65535u, 65536u, 1600000u, 0x3fffffu, UINT32_MAX}) {
            uint8_t pulses = 0;
            uint32_t result = pack(single, threshold, pulses);
            assert(pulses == 1 && result == single);
            ++count;
        }
    }
    std::printf("Packing: %lu exact group/interval comparisons passed\n", count);
}
"""
harness = "#include <initializer_list>\n" + harness
with TemporaryDirectory() as tmp:
    src = Path(tmp) / "packing.cpp"
    exe = Path(tmp) / "packing"
    src.write_text(harness)
    subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-O2",
                    "-Wall", "-Wextra", "-Werror", str(src), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)

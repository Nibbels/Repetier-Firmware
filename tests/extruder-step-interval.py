#!/usr/bin/env python3
"""Replay the actual foreground extrusion timer interval conversion."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "Repetier" / "Extruder.cpp").read_text(encoding="utf-8")
helper = source[source.index("static uint8_t boundedExtruderStepInterval("):
                source.index("void Extruder::applyCurrentSettings()")]
body = r"""
#include <cstdint>
#include <cassert>
#include <cmath>
#include <limits>
#include <cstdio>
""" + helper + r"""
int main() {
    // All previously representable, nonzero ratios preserve the result
    // of truncation followed by the historical 15-tick cap.
    for (unsigned i=1024; i<256*1024; ++i) {
        const float ratio=i/1024.0f;
        const uint8_t old=uint8_t(std::floor(ratio));
        assert(boundedExtruderStepInterval(ratio)==(old>15?15:old));
    }
    for (unsigned i=0; i<1024; ++i)
        assert(boundedExtruderStepInterval(i/1024.0f)==1);
    for (unsigned i=256; i<65536; ++i)
        assert(boundedExtruderStepInterval(float(i))==15);
    assert(boundedExtruderStepInterval(1e30f)==15);
    assert(boundedExtruderStepInterval(std::numeric_limits<float>::infinity())==15);
    assert(boundedExtruderStepInterval(std::numeric_limits<float>::quiet_NaN())==15);
    assert(boundedExtruderStepInterval(-1.0f)==15);
    puts("Extruder interval: 327424 ratio cases plus nonfinite/invalid values; PASS");
}
"""
with tempfile.TemporaryDirectory(prefix="rfx-extruder-interval-") as tmp:
    path = Path(tmp)
    (path / "replay.cpp").write_text(body, encoding="utf-8")
    subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-O2",
                    "-Wall", "-Wextra", "-Werror", str(path / "replay.cpp"),
                    "-o", str(path / "replay")], check=True)
    subprocess.run([str(path / "replay")], check=True)

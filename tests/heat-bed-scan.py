"""Replay the bed scan against a small simulated mesh and a test clock."""

import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory


def main():
    tests = Path(__file__).resolve().parent
    source = (tests.parent / "Repetier/RF.cpp").read_text(encoding="utf-8")
    start = source.index("namespace {\n\nclass HeatBedScan")
    end = source.index("\nbool heatBedScanRunning(", start)
    with TemporaryDirectory(prefix="rfx-bed-scan-") as temp:
        build = Path(temp)
        (build / "heat-bed-scan-firmware.inc").write_text(source[start:end], encoding="utf-8")
        executable = build / "scan"
        subprocess.run([
            os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-I", str(build), str(tests / "heat-bed-scan.cpp"), "-o", str(executable),
        ], check=True)
        for scenario in ("standard", "pla", "abs", "retry", "cancel"):
            subprocess.run([str(executable), scenario], check=True, timeout=10)


if __name__ == "__main__":
    main()

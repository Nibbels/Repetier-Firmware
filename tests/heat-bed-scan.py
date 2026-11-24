"""Replay the bed scan against a small simulated mesh and a test clock."""

import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory


def main():
    tests = Path(__file__).resolve().parent
    firmware = tests.parent / "Repetier"
    source = (firmware / "HeatBedScan.cpp").read_text(encoding="utf-8")
    probe = (firmware / "ScanProbe.cpp").read_text(encoding="utf-8")
    matrix = (firmware / "BedMatrix.cpp").read_text(encoding="utf-8")
    start = source.rindex("namespace {", 0, source.index("class HeatBedScan"))
    end = source.index("\nbool heatBedScanRunning(", start)
    with TemporaryDirectory(prefix="rfx-bed-scan-") as temp:
        build = Path(temp)
        (build / "heat-bed-scan-firmware.inc").write_text(source[start:end], encoding="utf-8")
        (build / "scan-pressure-firmware.inc").write_text(
            '#include "ScanProbe.h"\n'
            + probe[probe.index("namespace Calibration {"):probe.index("void showAbortScanReason(")]
            + "} // namespace Calibration\n",
            encoding="utf-8",
        )
        matrix_sections = (
            ("void clearWorking()", "uint8_t storedMaxIndex("),
            ("char prepareWorking()", "ZOffsetResult applyZOffset("),
        )
        (build / "bed-matrix-scan-firmware.inc").write_text(
            '#include "BedMatrix.h"\n'
            + "namespace BedMatrix {\n"
            + "\n".join(matrix[matrix.index(first):matrix.index(last, matrix.index(first))]
                        for first, last in matrix_sections)
            + "\n} // namespace BedMatrix\n",
            encoding="utf-8",
        )
        executable = build / "scan"
        subprocess.run([
            os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-I", str(build), "-I", str(firmware), str(tests / "heat-bed-scan.cpp"), "-o", str(executable),
        ], check=True)
        for scenario in (
            "standard", "pla", "abs", "retry", "cancel", "disconnected", "overload",
            "overload-and-cancel", "stop-mid-grid", "invalid-grid", "save-failed",
        ):
            subprocess.run([str(executable), scenario], check=True, timeout=10)


if __name__ == "__main__":
    main()

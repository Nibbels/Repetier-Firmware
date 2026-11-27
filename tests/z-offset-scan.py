"""Run the firmware's Z-offset scan against deterministic probe and EEPROM inputs."""

import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory


def main():
    tests = Path(__file__).resolve().parent
    firmware = tests.parent / "Repetier"
    source = (firmware / "ZOffsetScan.cpp").read_text(encoding="utf-8")
    probe = (firmware / "ScanProbe.cpp").read_text(encoding="utf-8")
    matrix = (firmware / "BedMatrix.cpp").read_text(encoding="utf-8")
    start = source.rindex("namespace {", 0, source.index("class ZOffsetScan"))
    end = source.index("\nvoid startZOScan(", start)
    with TemporaryDirectory(prefix="rfx-z-offset-scan-") as temp:
        build = Path(temp)
        (build / "z-offset-scan-firmware.inc").write_text(source[start:end], encoding="utf-8")
        (build / "scan-pressure-firmware.inc").write_text(
            '#include "ScanProbe.h"\n'
            + probe[probe.index("namespace Calibration {"):probe.index("void showAbortScanReason(")]
            + "} // namespace Calibration\n",
            encoding="utf-8",
        )
        matrix_sections = (
            ("void clearWorking()", "bool invalidateAllStored("),
            ("bool reloadActive()", "void beginScanGrid()"),
            ("uint8_t maxIndex(", "char prepareWorking()"),
            ("void report(char format)", "void applyThermalOffset("),
            ("ZOffsetResult applyZOffset(", "} // namespace BedMatrix"),
        )
        (build / "bed-matrix-scan-firmware.inc").write_text(
            '#include "BedMatrix.h"\n'
            + matrix[matrix.index("bool bedMatrixAvailable()"):matrix.index("\n", matrix.index("bool bedMatrixAvailable()"))]
            + "\n"
            + "namespace BedMatrix {\n"
            + "\n".join(matrix[matrix.index(first):matrix.index(last, matrix.index(first))]
                        for first, last in matrix_sections)
            + "\n} // namespace BedMatrix\n",
            encoding="utf-8",
        )
        runtime = (firmware / "RF.cpp").read_text(encoding="utf-8")
        (build / "scan-dispatch-firmware.inc").write_text(
            runtime[runtime.index("bool isAnyScanRunning("):runtime.index("void handleStartStandby(")],
            encoding="utf-8",
        )
        executable = build / "scan"
        subprocess.run([
            os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-I", str(build), "-I", str(firmware), str(tests / "z-offset-scan.cpp"), "-o", str(executable),
        ], check=True)
        for scenario in (
            "normal", "retry", "missing-matrix", "level", "tiny-matrix", "cancel-during-home",
            "stop-during-home", "disconnected", "overload", "busy-admission",
        ):
            subprocess.run([str(executable), scenario], check=True, timeout=10)


if __name__ == "__main__":
    main()

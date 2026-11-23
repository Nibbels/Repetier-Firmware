"""Replay the bed scan against a small simulated mesh and a test clock."""

import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory


def main():
    tests = Path(__file__).resolve().parent
    firmware = tests.parent / "Repetier"
    source = (firmware / "RF.cpp").read_text(encoding="utf-8")
    start = source.index("namespace {\n\nclass HeatBedScan")
    end = source.index("\nbool heatBedScanRunning(", start)
    with TemporaryDirectory(prefix="rfx-bed-scan-") as temp:
        build = Path(temp)
        (build / "heat-bed-scan-firmware.inc").write_text(source[start:end], encoding="utf-8")
        pressure_start = source.index("static PressureMeasurement::Settings pressureMeasurementSettings();")
        pressure_end = source.index("static bool zOffsetScanRunning();", pressure_start)
        service_start = source.index("static PressureMeasurement::Settings pressureMeasurementSettings() {")
        service_end = source.index("short readAveragePressure(", service_start)
        (build / "scan-pressure-firmware.inc").write_text(
            source[pressure_start:pressure_end] + source[service_start:service_end], encoding="utf-8",
        )
        executable = build / "scan"
        subprocess.run([
            os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-I", str(build), "-I", str(firmware), str(tests / "heat-bed-scan.cpp"), "-o", str(executable),
        ], check=True)
        for scenario in ("standard", "pla", "abs", "retry", "cancel", "disconnected", "overload", "invalid-grid"):
            subprocess.run([str(executable), scenario], check=True, timeout=10)


if __name__ == "__main__":
    main()

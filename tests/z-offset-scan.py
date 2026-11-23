"""Run the firmware's Z-offset scan against deterministic probe and EEPROM inputs."""

import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory


def main():
    tests = Path(__file__).resolve().parent
    firmware = tests.parent / "Repetier"
    source = (firmware / "RF.cpp").read_text(encoding="utf-8")
    start = source.index("namespace {\n\nclass ZOffsetScan")
    end = source.index("\nvoid startZOScan(", start)
    with TemporaryDirectory(prefix="rfx-z-offset-scan-") as temp:
        build = Path(temp)
        (build / "z-offset-scan-firmware.inc").write_text(source[start:end], encoding="utf-8")
        pressure_start = source.index("static PressureMeasurement::Settings pressureMeasurementSettings();")
        pressure_end = source.index("static bool zOffsetScanRunning();", pressure_start)
        service_start = source.index("static PressureMeasurement::Settings pressureMeasurementSettings() {")
        service_end = source.index("short readAveragePressure(", service_start)
        (build / "scan-pressure-firmware.inc").write_text(
            source[pressure_start:pressure_end] + source[service_start:service_end], encoding="utf-8",
        )
        discard_start = source.index("void discardScans()")
        discard_end = source.index("\nfloat g_ZSchraubenSollDrehungenWarm_U", discard_start)
        dispatch_start = source.index("void handleScanWorkTasks(")
        dispatch_end = source.index("\nvoid handleStartStandby(", dispatch_start)
        (build / "scan-dispatch-firmware.inc").write_text(
            source[discard_start:discard_end] + source[dispatch_start:dispatch_end], encoding="utf-8",
        )
        executable = build / "scan"
        subprocess.run([
            os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-I", str(build), "-I", str(firmware), str(tests / "z-offset-scan.cpp"), "-o", str(executable),
        ], check=True)
        for scenario in (
            "normal", "retry", "missing-matrix", "level", "tiny-matrix", "cancel-during-home",
            "stop-during-home", "disconnected", "overload",
        ):
            subprocess.run([str(executable), scenario], check=True, timeout=10)


if __name__ == "__main__":
    main()

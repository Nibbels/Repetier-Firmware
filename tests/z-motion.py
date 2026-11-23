"""Replay Z moves and correction waits with the firmware's pulse accounting."""

import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory


def section(source, start, end):
    first = source.index(start)
    return source[first:source.index(end, first + len(start))]


def main():
    tests = Path(__file__).resolve().parent
    firmware = tests.parent / "Repetier"
    z_motion = (firmware / "ZMotion.cpp").read_text(encoding="utf-8")
    executor = (firmware / "StepExecutor.cpp").read_text(encoding="utf-8")
    printer = (firmware / "Printer.cpp").read_text(encoding="utf-8")
    rf = (firmware / "RF.cpp").read_text(encoding="utf-8")
    fragments = [
        z_motion.removeprefix('#include "Repetier.h"\n'),
        section(executor, "void StepExecutor::stepSlowedZCompensation()", "\n/**\n  Processes the moves"),
        section(printer, "bool Printer::needsCMPwait(", "\nint8_t Printer::anyHomeDir("),
        section(rf, "static bool moveZ(int nSteps, bool emergencyRetreat) {", "\nvoid restoreDefaultScanParameters("),
    ]
    with TemporaryDirectory(prefix="rfx-z-motion-") as directory:
        build = Path(directory)
        (build / "z-motion-firmware.inc").write_text("\n".join(fragments), encoding="utf-8")
        for model in ("RF1000", "RF2000", "RF2000v2"):
            executable = build / model
            subprocess.run([
                os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                "-I", str(firmware), "-I", str(build), "-DMOTHERBOARD=DEVICE_TYPE_" + model,
                str(tests / "z-motion.cpp"), "-o", str(executable),
            ], check=True)
            subprocess.run([str(executable)], check=True)
            print(model + " Z motion replay passed", flush=True)


if __name__ == "__main__":
    main()

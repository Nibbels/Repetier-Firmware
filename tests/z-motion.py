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
    probe = (firmware / "ScanProbe.cpp").read_text(encoding="utf-8")
    printer_header = (firmware / "Printer.h").read_text(encoding="utf-8")
    no_wait = section(printer_header, "static inline void disableCMPnow() {", "\n    static void disableCMPnow(bool wait);")
    no_wait = no_wait.replace("static inline", "inline", 1)
    fragments = [
        z_motion.removeprefix('#include "Repetier.h"\n'),
        section(executor, "void StepExecutor::stepSlowedZCompensation()", "\n/**\n  Processes the moves"),
        "namespace Printer {\n" + no_wait + "\n}",
        section(printer, "bool Printer::needsCMPwait(", "\nvoid Printer::stopPrint("),
        section(probe, "bool moveZ(int nSteps, bool emergencyRetreat) {", "\n} // namespace Calibration"),
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

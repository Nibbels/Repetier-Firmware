"""Build and run the pause replay with the current firmware routines."""

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
    rf = (firmware / "RF.cpp").read_text(encoding="utf-8")
    rf_header = (firmware / "RF.h").read_text(encoding="utf-8")
    ui = (firmware / "ui.cpp").read_text(encoding="utf-8")
    motion = (firmware / "motion.h").read_text(encoding="utf-8")
    fragments = [
        section(rf_header, "#define UI_ACTION_RF_HEAT_BED_UP", "#define UI_ACTION_RF_EXTRUDER_RETRACT"),
        section(motion, "#define DIRECT_PREPARED_STOPPABLE", "\n")
        + "\n" + section(motion, "#define DIRECT_RUNNING_STOPPABLE", "\n"),
        section(rf, "volatile long g_nPauseSteps[4]", "\n#ifdef RGB_LIGHT_R_PIN"),
        section(rf_header, "inline bool isPauseSettled()", "\nextern long g_nEmergencyPauseDigitsMin"),
        section(rf, "static short readStrainGauge()", "\nvoid adjustPressureLimits("),
        section(rf, "long getZMatrixDepth_CurrentXY(", "\nvoid showAbortScanReason("),
        section(rf, "void handleStrainGaugeFeatures(", "\nvoid handlePauseTime("),
        section(rf, "void requestPrintPause(bool moveAway) {", "\nvoid loopFeatures("),
        section(rf, "static void waitForPauseTransition()", "\nvoid setExtruderCurrent("),
        section(rf, "void moveZAction(int8_t direction) {", "} // moveZAction") + "}",
        "void extrudeFromPanel() { switch (UI_ACTION_RF_EXTRUDER_OUTPUT) {\n"
        + section(ui, "case UI_ACTION_RF_EXTRUDER_OUTPUT:", "case UI_ACTION_RF_EXTRUDER_RETRACT:")
        + "} }",
    ]
    with TemporaryDirectory(prefix="rfx-pause-replay-") as directory:
        build = Path(directory)
        (build / "pause-sense-offset-firmware.inc").write_text("\n".join(fragments), encoding="utf-8")
        executable = build / "pause-replay"
        subprocess.run([
            os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-I", str(firmware), "-I", str(build), str(tests / "pause-sense-offset.cpp"),
            "-o", str(executable),
        ], check=True)
        for scenario in ("park", "short-pause", "interrupt"):
            subprocess.run([str(executable), scenario], check=True)


if __name__ == "__main__":
    main()

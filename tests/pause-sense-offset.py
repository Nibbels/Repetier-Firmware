"""Replay print lifecycle transitions with the current firmware routines."""

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
    ui_header = (firmware / "ui.h").read_text(encoding="utf-8")
    motion = (firmware / "motion.h").read_text(encoding="utf-8")
    move_planner = (firmware / "motion.cpp").read_text(encoding="utf-8")
    printer = (firmware / "Printer.cpp").read_text(encoding="utf-8")
    pause = (firmware / "PrintPause.cpp").read_text(encoding="utf-8")
    stop = (firmware / "PrintStop.cpp").read_text(encoding="utf-8")
    fragments = [
        section(rf_header, "#define UI_ACTION_RF_HEAT_BED_UP", "#define UI_ACTION_RF_PAUSE"),
        section(ui_header, "#define UI_FLAG_FAST_KEY_ACTION", "#define UI_FLAG_KEY_TEST_RUNNING")
        + section(ui_header, "#define UI_FLAG_KEY_TEST_RUNNING", "\n"),
        section(motion, "#define DIRECT_IDLE", "#define FOR_DIRECT"),
        section(motion, "    static bool hasStoppableDirectMove()", "\n\n")
        .replace("    static bool hasStoppableDirectMove()", "bool PrintLine::hasStoppableDirectMove()", 1),
        section(rf, "long g_nEmergencyPauseDigitsMin", "\n#ifdef RGB_LIGHT_R_PIN"),
        section(rf, "static short readStrainGauge()", "\nvoid adjustPressureLimits("),
        section(rf, "long getZMatrixDepth_CurrentXY(", "\nvoid showAbortScanReason("),
        section(rf, "namespace {\nstruct SenseOffsetSamples", "\nvoid handleScanWorkTasks("),
        pause.removeprefix('#include "Repetier.h"\n'),
        stop.removeprefix('#include "Repetier.h"\n'),
        section(printer, "void Printer::stopPrint()", "\n#define START_EXTRUDER_CONFIG"),
        section(printer, "void Printer::homeZAxis()", "\nvoid Printer::homeDigits("),
        section(rf, "bool isAnyScanRunning()", "\nvoid moveXAction("),
        section(rf, "void moveZAction(int8_t direction) {", "} // moveZAction") + "}",
        section(ui, "void UIDisplay::pressPlay()", "\nvoid UIDisplay::mainSwitchCase("),
        section(ui, "void UIDisplay::fastAction()", "} // fastAction") + "}",
        "void extrudeFromPanel() { const int action = UI_ACTION_RF_EXTRUDER_OUTPUT; switch (action) {\n"
        + section(ui, "case UI_ACTION_RF_EXTRUDER_OUTPUT:", "case UI_ACTION_RF_PAUSE:")
        + "} }",
        "void trackPlannedLayer(PrintLine* p) {\n"
        + section(move_planner, "    if (!g_startLineActive)", "\n    // Define variables that are needed")
        + "}",
        "void runStartLine(GCode* pCommand) { switch (3912) {\n"
        + section(rf, "        case 3912:", "\n        case 3913:")
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
        for scenario in (
            "park", "short-pause", "interrupt", "panel", "return", "jog", "cancel", "queue", "home", "layer-fade",
            "layer-scale", "compensation-bounds",
            "digit-flow", "prime-complete", "prime-skip", "prime-abort",
        ):
            subprocess.run([str(executable), scenario], check=True, timeout=10)


if __name__ == "__main__":
    main()

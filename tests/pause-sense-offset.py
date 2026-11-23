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
    printer_header = (firmware / "Printer.h").read_text(encoding="utf-8")
    eeprom = (firmware / "Eeprom.h").read_text(encoding="utf-8")
    pause = (firmware / "PrintPause.cpp").read_text(encoding="utf-8")
    stop = (firmware / "PrintStop.cpp").read_text(encoding="utf-8")
    dms = (firmware / "StrainGauge.cpp").read_text(encoding="utf-8")
    start_line = (firmware / "StartLine.cpp").read_text(encoding="utf-8")
    hal = (firmware / "HAL.cpp").read_text(encoding="utf-8")
    fragments = [
        section(eeprom, "#define EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX", "#define EPR_RF_HEATED_BED_SENSOR_TYPE"),
        section(rf_header, "#define UI_ACTION_RF_HEAT_BED_UP", "#define UI_ACTION_RF_PAUSE"),
        section(ui_header, "#define UI_FLAG_FAST_KEY_ACTION", "#define UI_FLAG_KEY_TEST_RUNNING")
        + section(ui_header, "#define UI_FLAG_KEY_TEST_RUNNING", "\n"),
        section(motion, "#define DIRECT_IDLE", "#define FOR_DIRECT"),
        section(motion, "    static bool hasStoppableDirectMove()", "\n\n")
        .replace("    static bool hasStoppableDirectMove()", "bool PrintLine::hasStoppableDirectMove()", 1),
        dms.removeprefix('#include "Repetier.h"\n#include <Wire.h>\n'),
        section(rf, "static short sampleStrainGauge()", "\nvoid adjustPressureLimits("),
        section(rf, "short readAveragePressure(", "} // readAveragePressure") + "}",
        section(rf, "long getZMatrixDepth_CurrentXY(", "\nvoid showAbortScanReason("),
        section(rf, "static void serviceSenseOffset(", "\nvoid handleScanWorkTasks("),
        pause.removeprefix('#include "Repetier.h"\n'),
        stop.removeprefix('#include "Repetier.h"\n'),
        "namespace Printer {\n"
        + section(printer_header, "    static INLINE void setEPositionMM(", "\n    static INLINE void stopDirectAxis(")
        .replace("static INLINE void ", "void ")
        .replace("float remainder = 0.0f", "float remainder")
        + "\n}",
        section(printer, "inline bool isExtrusionAllowed(", "\n/**\n* Set the printers feedrate"),
        section(printer, "void Printer::updateWobbleFixOffsets(", "\ninline bool isExtrusionAllowed("),
        section(printer, "void Printer::setFeedrate(", "\n/** Move to machine coordinates"),
        section(printer, "void Printer::stopPrint()", "\n#define START_EXTRUDER_CONFIG"),
        section(printer, "void Printer::homeZAxis()", "\nvoid Printer::homeAxis("),
        section(printer, "void Printer::enableSenseOffsetnow(", "\nvoid Printer::enableCMPnow("),
        section(rf, "bool isAnyScanRunning()", "\nvoid moveXAction("),
        section(rf, "void moveZAction(int8_t direction) {", "} // moveZAction") + "}",
        section(ui, "void UIDisplay::pressPlay()", "\nvoid UIDisplay::mainSwitchCase("),
        section(ui, "void UIDisplay::fastAction()", "} // fastAction") + "}",
        section(ui, "namespace {\nconstexpr double modMenuSenseLimit", "\nvoid UIDisplay::zOffsetPlusAction()"),
        "void extrudeFromPanel() { const int action = UI_ACTION_RF_EXTRUDER_OUTPUT; switch (action) {\n"
        + section(ui, "case UI_ACTION_RF_EXTRUDER_OUTPUT:", "case UI_ACTION_RF_PAUSE:")
        + "} }",
        "void trackPlannedLayer(PrintLine* p) {\n"
        + section(move_planner, "    if (!Printer::startLine.active())", "\n    p->setSegmentGeometry(")
        + "}",
        "int32_t planExtrusionMove() {\nint32_t steps;\n"
        + section(move_planner, "            const float targetE =", "\n        } else {")
        + "\nreturn steps;\n}",
        "int32_t planCartesianAxis(uint8_t axis) {\nint32_t steps;\n"
        + section(move_planner, "            const int32_t targetSteps = Printer::prepareDestinationSteps(axis);", "\n        }")
        + "\nreturn steps;\n}",
        section(move_planner, "void PrintLine::setAxisSteps(", "\n/** \\brief Put a move"),
        section(motion, "#define FLAG_WARMUP", "\n#define DIRECT_IDLE"),
        section(motion, "class MoveQueue {", "\ninline uint8_t PrintLine::queuedMoveCount"),
        "MoveQueue PrintLine::queue;",
        section(move_planner, "bool PrintLine::hasActiveZMove()", "\nvoid PrintLine::setAxisSteps("),
        section(move_planner, "speed_t PrintLine::nextStepSpeed(", "\nlong PrintLine::performMove("),
        section(move_planner, "uint8_t PrintLine::insertWaitMovesIfNeeded(", "\nvoid PrintLine::waitForXFreeLines("),
        section(hal, "uint8_t PressureAdvance::nextTimerCompare(", "\nISR(EXTRUDER_TIMER_VECTOR)"),
        "void rescalePlannedAxis(uint8_t axis) {\nusing namespace Printer;\n"
        + section(printer, "        const float mmPerStep = 1.0f / axisStepsPerMM[axis];", "\n\n        /** Acceleration")
        + "\n}",
        "void executeExtrusionStep(PrintLine* move, bool forQueue) {\n"
        + section(move_planner, "        if (move->isEMove()) {", "\n        if (move->isXMove()) {")
        + "\n}",
        section(move_planner, "template <int8_t limitDirection>", "\n/**\n  Processes the one-and-only direct move"),
        start_line.removeprefix('#include "Repetier.h"\n'),
        "void configureSenseOffset(GCode* pCommand) { switch (3909) {\n"
        + section(rf, "        case 3909:", "\n        case 3911:")
        + "} }",
        "void configurePressure(int command, GCode* pCommand) { switch (command) {\n"
        + section(rf, "        case 3075:", "\n        case 3079:")
        + "} }",
    ]
    with TemporaryDirectory(prefix="rfx-pause-replay-") as directory:
        build = Path(directory)
        (build / "pause-sense-offset-firmware.inc").write_text("\n".join(fragments), encoding="utf-8")
        (build / "step-clock-firmware.inc").write_text(
            section(motion, "    struct StepClock {", "    enum class DirectionOwner"), encoding="utf-8",
        )
        executable = build / "pause-replay"
        subprocess.run([
            os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-I", str(firmware), "-I", str(build), str(tests / "pause-sense-offset.cpp"),
            "-o", str(executable),
        ], check=True)
        for scenario in (
            "park", "short-pause", "interrupt", "panel", "return", "jog", "cancel", "queue", "home", "layer-fade",
            "layer-scale", "compensation-bounds",
            "digit-flow", "prime-complete", "prime-skip", "prime-abort", "prime-parameters",
            "sense-window", "sense-settings", "sense-limit-interrupt",
            "pressure-pause", "pressure-stop", "digit-zero",
            "extrusion-planning",
            "gcode-target", "wobble-target", "cartesian-planning", "segment-geometry", "move-queue", "ecmp-stepping",
            "advance-timer", "step-clock", "xy-limits",
        ):
            subprocess.run([str(executable), scenario], check=True, timeout=10)


if __name__ == "__main__":
    main()

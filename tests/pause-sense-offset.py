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
    probe = (firmware / "ScanProbe.cpp").read_text(encoding="utf-8")
    pressure = (firmware / "PressureFeatures.cpp").read_text(encoding="utf-8")
    matrix = (firmware / "BedMatrix.cpp").read_text(encoding="utf-8")
    panel_motion = (firmware / "PanelMotion.cpp").read_text(encoding="utf-8")
    rf_header = (firmware / "RF.h").read_text(encoding="utf-8")
    ui = (firmware / "ui.cpp").read_text(encoding="utf-8")
    ui_actions = (firmware / "UiActions.cpp").read_text(encoding="utf-8")
    ui_header = (firmware / "ui.h").read_text(encoding="utf-8")
    motion = (firmware / "motion.h").read_text(encoding="utf-8")
    move_queue = (firmware / "MoveQueue.h").read_text(encoding="utf-8")
    move_reserve = (firmware / "MoveQueue.cpp").read_text(encoding="utf-8")
    move_planner = (firmware / "motion.cpp").read_text(encoding="utf-8")
    planner = (firmware / "MotionPlanner.cpp").read_text(encoding="utf-8")
    executor = (firmware / "StepExecutor.cpp").read_text(encoding="utf-8")
    executor_header = (firmware / "StepExecutor.h").read_text(encoding="utf-8")
    printer = (firmware / "Printer.cpp").read_text(encoding="utf-8")
    homing = (firmware / "Homing.cpp").read_text(encoding="utf-8")
    printer_header = (firmware / "Printer.h").read_text(encoding="utf-8")
    eeprom = (firmware / "Eeprom.h").read_text(encoding="utf-8")
    eeprom_source = (firmware / "Eeprom.cpp").read_text(encoding="utf-8")
    pause = (firmware / "PrintPause.cpp").read_text(encoding="utf-8")
    idle = (firmware / "IdleStatus.cpp").read_text(encoding="utf-8")
    stop = (firmware / "PrintJob.cpp").read_text(encoding="utf-8")
    dms = (firmware / "StrainGauge.cpp").read_text(encoding="utf-8")
    start_line = (firmware / "StartLine.cpp").read_text(encoding="utf-8")
    hal = (firmware / "HAL.cpp").read_text(encoding="utf-8")
    commands = (firmware / "Commands.cpp").read_text(encoding="utf-8")
    motion_commands = (firmware / "GCodeMotion.cpp").read_text(encoding="utf-8")
    thermal_commands = (firmware / "GCodeThermal.cpp").read_text(encoding="utf-8")
    extruder = (firmware / "Extruder.cpp").read_text(encoding="utf-8")
    fragments = [
        section(eeprom, "#define EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX", "#define EPR_RF_HEATED_BED_SENSOR_TYPE"),
        section(rf_header, "#define UI_ACTION_RF_HEAT_BED_UP", "#define UI_ACTION_RF_PAUSE"),
        section(ui_header, "#define UI_FLAG_KEY_TEST_RUNNING", "\n"),
        section(motion, "    static bool hasStoppableDirectMove()", "\n\n")
        .replace("    static bool hasStoppableDirectMove()", "bool PrintLine::hasStoppableDirectMove()", 1),
        dms[dms.index("int16_t StrainGauge::correctedDigits("):],
        section(pressure, "void updateDigitCompensation(", "static void serviceSenseOffset("),
        'const ScanSettings& probeSettings() { return scanSettings; }',
        section(probe, "constexpr int16_t scanOverloadDigits =", "\n"),
        section(probe, "static PressureMeasurement::Settings pressureMeasurementSettings() {", "PressureResult ScanPressure::average"),
        probe[probe.index("short readAveragePressure("):].replace("    using namespace Calibration;\n", ""),
        section(matrix, "long getZMatrixDepth_CurrentXY(", "static void initCompensationMatrix(void) {"),
        pressure[pressure.index("static void serviceSenseOffset("):],
        section(rf, "void handleStartStandby(", "\nvoid outputObject("),
        section(rf, "void outputObject(bool showerrors)", "} // outputObject") + "}",
        pause.removeprefix('#include "Repetier.h"\n'),
        idle[idle.index("void IdleStatus::schedule("):],
        stop.removeprefix('#include "Repetier.h"\n#include "GCodeInput.h"\n'),
        "namespace Printer {\n"
        + section(printer_header, "    static INLINE void setEPositionMM(", "\n    static INLINE void stopDirectAxis(")
        .replace("static INLINE void ", "void ")
        .replace("float remainder = 0.0f", "float remainder")
        + "\n}",
        section(printer, "inline bool isExtrusionAllowed(", "\n/**\n* Set the printers feedrate"),
        section(printer, "void Printer::updateWobbleFixOffsets(", "\ninline bool isExtrusionAllowed("),
        section(printer, "void Printer::setFeedrate(", "\n/** Move to machine coordinates"),
        section(printer, "void Printer::stopPrint()", "\n#define START_EXTRUDER_CONFIG"),
        section(homing, "void Printer::homeZAxis()", "\nvoid Printer::homeAxis("),
        section(printer, "void Printer::enableSenseOffsetnow(", "\nvoid Printer::enableCMPnow("),
        section(eeprom_source, "void EEPROM::publishSenseOffsetSettings()", "\nvoid EEPROM::storeDriveLimit("),
        section(rf, "bool isAnyScanRunning()", "\nvoid discardScans(").replace("Calibration::", ""),
        section(panel_motion, "void moveZAction(int8_t direction) {", "} // moveZAction") + "}",
        section(ui_actions, "void UIDisplay::pressPlay()", "\nvoid UIDisplay::mainSwitchCase("),
        section(ui, "void UIDisplay::fastAction()", "} // fastAction") + "}",
        section(ui_actions, "namespace {\nconstexpr double modMenuSenseLimit", "\nvoid UIDisplay::zOffsetPlusAction()"),
        "void extrudeFromPanel(int action = UI_ACTION_RF_EXTRUDER_OUTPUT) { switch (action) {\n"
        + section(ui_actions, "case UI_ACTION_RF_EXTRUDER_OUTPUT:", "case UI_ACTION_RF_PAUSE:")
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
        section(move_queue, "class MoveQueue {", "\ninline const MoveQueue& PrintLine::queuedMoves"),
        "MoveQueue PrintLine::queue;",
        move_reserve[move_reserve.index("MoveQueue::RemainingMotion"):],
        section(move_planner, "PrintLine::ActiveMove PrintLine::activeMove()", "\nvoid PrintLine::setAxisSteps("),
        section(executor, "speed_t StepExecutor::nextStepSpeed(", "\n// Isolate completion"),
        move_planner[move_planner.index("void PrintLine::prepareDirectMove("):move_planner.index("/** \\brief Check if move is new.")]
        .replace("void PrintLine::stopDirectMove() {", "void PrintLine::applyDirectStop() {", 1),
        planner.split('#include "MotionPlanner.h"\n', 1)[1],
        section(move_planner, "uint8_t PrintLine::insertWaitMovesIfNeeded(", "\nbool PrintLine::waitForXFreeLines("),
        section(hal, "namespace {\nconstexpr uint8_t advanceCompareGuard", "\n#if !defined CASE_LIGHT_PIN"),
        section(executor, "inline void StepExecutor::updateAdvanceSteps(", "\nvoid StepExecutor::stepSlowedZCompensation("),
        section(commands, "bool Commands::waitForMotion(", "\nvoid Commands::printCurrentPosition()"),
        section(extruder, "bool Extruder::selectExtruderById(", "\nstatic uint8_t boundedExtruderStepInterval("),
        section(printer, "void Printer::applyAdvanceSettings(", "\nvoid Printer::updateWobbleFixOffsets("),
        section(motion_commands, "void configureAdvance(", "void finishMotion("),
        # Synthetic test input only; no public G-code can inject raw E steps.
        "void injectAdvanceTestSteps(GCode* com) {"
        " if (com->hasS() && Printer::extrusionOutput.advanceEnabled() && !Printer::pause.isPaused()"
        " && !Printer::extrusionOutput.blocked()) { InterruptProtectedBlock lock; Printer::extrusionOutput.addSteps(com->S); } }",
        "void runAdvanceCommand(int command, GCode* com) {"
        " if (command == 233) configureAdvance(com); }",
        "void rescalePlannedAxis(uint8_t axis) {\nusing namespace Printer;\n"
        + section(printer, "        const float mmPerStep = 1.0f / axisStepsPerMM_[axis];", "\n\n        /** Acceleration")
        + "\n}",
        "void executeExtrusionStep(PrintLine* move, bool forQueue) {\n"
        + "    auto& errors = StepExecutor::stepErrors[forQueue];\n"
        + section(executor, "        if (move->isEMove()) {", "\n        if (move->isXMove()) {")
        + "\n}",
        section(executor, "template <int8_t limitDirection>", "\n/**\n  Processes the one-and-only direct move"),
        start_line.removeprefix('#include "Repetier.h"\n'),
        section(motion_commands, "void setSenseOffsetParameters(", "\nvoid setDigitFlowParameters("),
        "void configureSenseOffset(GCode* command) { setSenseOffsetParameters(command); }",
        section(motion_commands, "void setEmergencyPauseDigits(", "\nvoid setSenseOffsetParameters("),
        "void configurePressure(int code, GCode* command) {"
        " assert(code == 3075); setEmergencyPauseDigits(command); }",
        section(thermal_commands, "static bool prepareFilamentProgram(", "\n#ifdef X35_ADC_CHANNEL"),
        "void runFilamentCommand(int code, GCode* command) {"
        " if (code == 3913) loadFilament(command); else unloadFilament(command); }",
    ]
    with TemporaryDirectory(prefix="rfx-pause-replay-") as directory:
        build = Path(directory)
        (build / "pause-sense-offset-firmware.inc").write_text("\n".join(fragments), encoding="utf-8")
        (build / "step-clock-firmware.inc").write_text(
            section(executor_header, "    static constexpr uint16_t minimumStepIntervalTicks", "\n\n")
            + "\n" + section(executor_header, "    struct StepClock {", "    enum class DirectionOwner"), encoding="utf-8",
        )
        (build / "step-profile-firmware.inc").write_text(
            section(motion, "    struct LookaheadData {", "    LookaheadData lookahead;"), encoding="utf-8",
        )
        (build / "lookahead-flags-firmware.inc").write_text(
            section(motion, "#define FLAG_WARMUP", "\nenum DirectMovePhase")
            + section(motion, "    inline bool isStepProfileReady()", "    void fitRampToRemainingSteps();")
            + section(motion, "    inline void setStartSpeedFixed(", "    inline uint8_t getWaitForXLinesFilled()")
            + section(motion, "    inline void block()", "\n    // If we reach endstops")
            + section(motion, "    inline bool isNominalMove()", "    inline bool isXPositiveMove()"), encoding="utf-8",
        )
        (build / "direct-phase-firmware.inc").write_text(
            section(motion, "enum DirectMovePhase", "#define FOR_DIRECT"), encoding="utf-8",
        )
        (build / "stop-reason-firmware.inc").write_text(
            section(rf_header, "enum class StopReason", "void doEmergencyStop("), encoding="utf-8",
        )
        (build / "direct-control-firmware.inc").write_text(
            section(motion, "    QueueTask queueTaskId() const", "\n\npublic:") + "\n"
            + section(motion, "    static bool directMovePreparing(", "\n\n")
            + section(motion, "    static void reserveDirectMove(", "    static void stopDirectMove()"),
            encoding="utf-8",
        )
        (build / "active-move-firmware.inc").write_text(
            section(motion, "    struct ActiveMove {", "    static void discardQueuedMoves();"),
            encoding="utf-8",
        )
        executable = build / "pause-replay"
        compile_replay = [
            os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-I", str(firmware), "-I", str(build), str(tests / "pause-sense-offset.cpp"),
        ]
        subprocess.run([*compile_replay, "-o", str(executable)], check=True)
        for scenario in (
            "park", "pause-manual-e", "pause-manual-e-advance", "pause-filament-program", "short-pause", "interrupt", "panel", "return",
            "jog", "cancel", "queue", "home", "layer-fade",
            "layer-scale", "compensation-bounds",
            "digit-flow", "prime-complete", "prime-skip", "prime-abort", "prime-parameters",
            "sense-window", "sense-settings", "sense-limit-interrupt",
            "pressure-pause", "pressure-stop", "pressure-frozen-z", "digit-zero", "pressure-measurement",
            "extrusion-planning",
            "gcode-target", "output-safety-lock", "wobble-target", "cartesian-planning", "segment-geometry", "move-queue", "move-reserve",
            "idle-status", "print-job", "ecmp-stepping",
            "advance-timer", "advance-mode", "advance-cancel", "motion-wait", "pressure-advance-pause", "advance-output", "advance-profile",
            "step-clock", "lookahead", "acceleration-limits", "short-move-pacing", "direct-ramp", "xy-limits",
        ):
            subprocess.run([str(executable), scenario], check=True, timeout=10)
        subprocess.run([*compile_replay, "-DNUM_EXTRUDER=2", "-o", str(executable)], check=True)
        for scenario in ("advance-tool", "pressure-stop", "pressure-frozen-z"):
            subprocess.run([str(executable), scenario], check=True, timeout=10)


if __name__ == "__main__":
    main()

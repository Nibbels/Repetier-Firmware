#include "Repetier.h"

StartLine Printer::startLine;

namespace {
struct Settings {
    DigitCompensation::FlowProfile flow;
    uint8_t extrusion;
    uint8_t speed;
    uint8_t lines;
    float y;
};

Settings readSettings(GCode& command) {
    Settings settings{{1500, 4500, -75, -90}, 20, 15, 2, 23.0f + Extruder::current->id};
    if (command.hasS() && command.hasP()) {
        const int16_t s = abs(int16_t(command.S));
        const int16_t p = abs(int16_t(command.P));
        const int16_t low = s < p ? s : p;
        const int16_t high = s > p ? s : p;
        settings.flow.minDigits = low < 500 ? 500 : low;
        settings.flow.maxDigits = high < settings.flow.minDigits ? settings.flow.minDigits : high;
    }
    if (command.hasE())
        settings.extrusion = constrain(uint8_t(command.E), 10, 30);
    if (command.hasF())
        settings.speed = constrain(uint8_t(command.F), 5, 50);
    if (command.hasI())
        settings.lines = constrain(uint8_t(command.I), 1, 5);
    if (command.hasY()) {
        const float maxY = Printer::xyMaxSteps(Y_AXIS) * Printer::mmPerStep(Y_AXIS) * 0.5f;
        settings.y = command.Y;
        if (settings.y > maxY)
            settings.y = maxY;
        if (settings.y < 0.0f)
            settings.y = 0.0f;
    }
    return settings;
}

void queueLines(const Settings& settings) {
    if (Printer::printJob.commandsBlocked())
        return;

    constexpr float xMargin = 15.0f;
    constexpr float startZ = 0.35f;
    constexpr float diagonalY = NUM_EXTRUDER > 1 ? 5.0f : 0.0f;
    const float toolOffsetX = NUM_EXTRUDER > 1 ? extruder[1].offsetMM[X_AXIS] : 0.0f;
    float x = xMargin;
    float y = settings.y;
    float e = 0.0f;
    Printer::queueFloatCoordinates(x, y, startZ, IGNORE_COORDINATE,
                                   RMath::min(Printer::axisHomingFeedrate(X_AXIS), Printer::axisHomingFeedrate(Y_AXIS)));

    for (uint8_t line = 0; line < settings.lines && !Printer::printJob.commandsBlocked(); ++line) {
        const float startX = x, startY = y, startE = e;
        const float length = Printer::axisLengthMm(X_AXIS) - 2 * xMargin - toolOffsetX;
        if (line & 1) {
            x -= length;
            y -= diagonalY;
        } else {
            x += length;
            y += diagonalY;
        }
        e += settings.extrusion * length / 200;

        for (float progress = 0.0025f; progress <= 1.0f; progress += 0.0025f) {
            Commands::checkForPeriodicalActions(Processing);
            if (Printer::printJob.commandsBlocked() || Printer::checkPlayKey())
                return;

            // Short segments let the DMS brake respond to nozzle load.
            Printer::queueFloatCoordinates(startX + (x - startX) * progress,
                                           startY + (y - startY) * progress,
                                           IGNORE_COORDINATE,
                                           startE + (e - startE) * progress, settings.speed);
        }
        if (Printer::printJob.commandsBlocked())
            return;

        y += 1.5f;
        y += float(NUM_EXTRUDER - 1);
        Printer::queueFloatCoordinates(IGNORE_COORDINATE, y, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::axisHomingFeedrate(Y_AXIS));
    }
}

struct ExtrusionState {
    float position;
    float positionRemainder;
    float stepRemainder;
    float ecmpRemainder;
};
}

void StartLine::run(GCode& command) {
    if (!Extruder::allHeatersCalibrated()) {
        Com::printFLN(PSTR("Error: PID tune required before printing"));
        return;
    }
    if (!Commands::waitForMotion(Commands::MotionWait::QueueAndZ, Commands::PauseWait::Resume))
        return;
    if (!Printer::areAxisHomed() || !Printer::doHeatBedZCompensation) {
        Com::printFLN(PSTR("M3912 error missing homing or zCMP"));
        return;
    }

    Printer::printJob.admitPrintMotion();
    Com::printFLN(PSTR("Auto-Startmade"));
    const auto savedFlow = Printer::digitCompensation.flowProfile();
    const bool savedRelativeE = Printer::gcodeModes.relativeEOverride();
    ExtrusionState savedE;
    {
        InterruptProtectedBlock lock;
        savedE = {Printer::moveTarget.machineMM(E_AXIS), Printer::moveTarget.remainderMM(E_AXIS),
                  Printer::extrusionPlanner.stepRemainder(), Printer::extrusionCompensation.remainder()};
        Printer::layers.resetTracking();
        active_ = true;
    }

    const auto settings = readSettings(command);
    Printer::digitCompensation.setFlowProfile(settings.flow);
    Printer::gcodeModes.setRelativeEOverride(false);
    Printer::setEPositionMM(0);
    queueLines(settings);

    const bool settled = Commands::waitForMotion(Commands::MotionWait::QueueAndZ, Commands::PauseWait::Resume);
    Printer::digitCompensation.setFlowProfile(savedFlow);
    Printer::digitCompensation.resetFlow();
    {
        InterruptProtectedBlock lock;
        if (settled) {
            Printer::setEPositionMM(savedE.position, savedE.positionRemainder);
            Printer::extrusionPlanner.restoreStepRemainder(savedE.stepRemainder);
            Printer::extrusionCompensation.restoreRemainder(savedE.ecmpRemainder);
        }
        Printer::layers.resetTracking();
        active_ = false;
    }
    Printer::gcodeModes.setRelativeEOverride(savedRelativeE);
}

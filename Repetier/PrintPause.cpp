#include "Repetier.h"

namespace {
constexpr millis_t standbyDelayMs = EXTRUDER_CURRENT_PAUSE_DELAY < 30000 ? 30000 : EXTRUDER_CURRENT_PAUSE_DELAY;
constexpr millis_t reheatGraceMs = 240000;
constexpr float standbyDropC = 100.0f;
}

PrintPause Printer::pause;

bool PrintPause::acceptsPauseRequest() const {
    return allowsQueueAdvance() && !Printer::printJob.commandsBlocked() && !Printer::blockAll && !isAnyScanRunning();
}

void PrintPause::request(Destination destination) {
    InterruptProtectedBlock noInts;
    if (!acceptsPauseRequest())
        return;

    pauseSinceSenseSample_ = true;
    phase_ = destination == Destination::Park ? Phase::RetractAndParkRequested : Phase::RetractRequested;
}

void PrintPause::queueRequest(Destination destination) {
    while (PrintLine::queuedMoveCount() >= MOVE_CACHE_SIZE) {
        if (!acceptsPauseRequest())
            return;
        Commands::checkForPeriodicalActions(Processing);
    }

    {
        InterruptProtectedBlock noInts;
        if (!acceptsPauseRequest())
            return;
        PrintLine::queueTask(destination == Destination::Park ? QueueTask::PauseAndPark : QueueTask::Pause);
    }

    // Another pause can take effect while the request waits for its place in the queue.
    while (acceptsPauseRequest() && (PrintLine::hasLines() || Printer::needsCMPwait()))
        Commands::checkForPeriodicalActions(Processing);
}

void PrintPause::pressPause() {
    // Calibration owns its own stop/abort state; a print pause cannot freeze it.
    if (isAnyScanRunning())
        return;
    if (allowsQueueAdvance()) {
        if (!PrintLine::hasLines()) {
            Com::printFLN(PSTR("pausePrint(): nothing is printed"));
            showError((void*)ui_text_pause, (void*)ui_text_operation_denied);
            return;
        }
        if (!Printer::areAxisHomed()) {
            Com::printFLN(PSTR("pausePrint(): home position is unknown"));
            showError((void*)ui_text_pause, (void*)ui_text_home_unknown);
            return;
        }

        Com::printFLN(PSTR("pausing..."));
        request(Destination::InPlace);
        waitUntilSettled();
        return;
    }

    if (!isSettled() || Printer::blockAll)
        return;

    if (!parked_) {
        phase_ = Phase::ParkRequested;
        Com::printFLN(PSTR("moving to pause position..."));
        waitUntilSettled();
        return;
    }

    if (isWarmPause()) {
        const millis_t now = HAL::timeInMilliseconds();
        Printer::idleStatus.schedule(now + 30000);
        standbyAt_ = now;
        UI_STATUS_UPD(UI_TEXT_COOLING_DOWN);
    }
}

bool PrintPause::resume() {
    {
        InterruptProtectedBlock noInts;
        if (!isSettled() || Printer::blockAll)
            return false;

        if (PrintLine::hasStoppableDirectMove())
            PrintLine::stopDirectMove();
        else if (PrintLine::directMoveActive())
            return false;

        phase_ = Phase::Reheating;
    }

    Printer::idleStatus.cancel();
    UI_STATUS_UPD(UI_TEXT_CONTINUING);
    uid.exitmenu();

    const bool heated = restoreExtruderTemperatures();
    if (!isReheating())
        return true;

    if (heated) {
        // A first Play restores the temperature. A second Play returns to print.
        const millis_t now = HAL::timeInMilliseconds();
        UI_STATUS_UPD(UI_TEXT_CONTINUING_READY);
        Printer::idleStatus.schedule(now + 30000);
        settle(now, reheatGraceMs);
    } else {
        phase_ = Phase::ReturnRequested;
        waitUntilSettled();
    }
    return true;
}

void PrintPause::cancel() {
    InterruptProtectedBlock noInts;
    // A pause that ended between samples still invalidates the old DMS window.
    parked_ = false;
    extruderReturnSteps_ = 0;
    phase_ = Phase::Running;
    Printer::setMenuMode(MENU_MODE_PAUSED, false);
}

void PrintPause::serviceMotion() {
    // A canceled phase can return to Running while a transition still
    // owns a foreground wait frame. Do not start a second transition
    // until the first has unwound; periodic thermal service still runs.
    if (servicingMotion_ || !isPaused() || isAnyScanRunning() ||
        PrintLine::directMoveActive() || Printer::blockAll)
        return;
    servicingMotion_ = true;
    dispatchPendingMotion();
    servicingMotion_ = false;
}

__attribute__((noinline)) void PrintPause::dispatchPendingMotion() {
    switch (phase_) {
    case Phase::RetractRequested:
        retract(Destination::InPlace);
        break;
    case Phase::RetractAndParkRequested:
        retract(Destination::Park);
        break;
    case Phase::ParkRequested:
        park();
        break;
    case Phase::ReturnRequested:
        returnToPrint();
        break;
    default:
        break;
    }
}

void PrintPause::serviceStandby(uint32_t now) {
    if (!isWarmPause() || static_cast<int32_t>(now - standbyAt_) <= 0)
        return;

    Com::printFLN(PSTR("Pause: Saving Energy"));
    for (uint8_t i = 0; i < NUM_EXTRUDER; ++i) {
        setExtruderCurrent(i, EXTRUDER_CURRENT_PAUSED);
        auto& heater = extruder[i].tempControl;
        if (!heater.paused) {
            const float target = heater.targetTemperatureC;
            const uint8_t drop = static_cast<uint8_t>(constrain(target, 0.0f, standbyDropC));
            Extruder::setTemperatureForExtruder(target - drop, i);
            heater.paused = drop;
        }
    }
    phase_ = Phase::Standby;
}

bool PrintPause::allowsManualMove() const {
    if (allowsQueueAdvance())
        return true;
    return isSettled() && !PrintLine::directMoveActive();
}

int32_t PrintPause::limitZJog(int32_t steps) const {
    if (steps >= 0 || allowsQueueAdvance())
        return steps;

    const int32_t remaining = stepsToReturn(Z_AXIS);
    if (remaining >= 0)
        return 0;
    return steps < remaining ? remaining : steps;
}

void PrintPause::extrudeManually(int32_t steps) {
    if (!steps || !allowsManualMove())
        return;

    if (!isSettled()) {
        Printer::offsetRelativeStepsCoordinates(0, 0, 0, steps);
        return;
    }

    const Phase settledPhase = phase_;
    const int32_t returnBeforeJog = extruderReturnSteps_;
    phase_ = Phase::ManualExtrusion;
    if (!moveWhile(Phase::ManualExtrusion, 0, 0, 0, steps))
        return;

    // The signed change is the motion delivered by the motor. Honor the new
    // filament position after a jog and keep the retract if all steps were blocked.
    if (extruderReturnSteps_ != returnBeforeJog)
        extruderReturnSteps_ = 0;
    phase_ = settledPhase;
}

void PrintPause::restoreDefaultParking() {
    // The selected extruder supplies the E retract.
    setParkSteps(X_AXIS, 0);
    setParkSteps(Y_AXIS, long(Printer::stepsPerMm(Y_AXIS) * 200));
    setParkSteps(Z_AXIS, long(Printer::stepsPerMm(Z_AXIS) * DEFAULT_PAUSE_MM_Z_PRINT));
}

PrintPause::SenseOffsetSample PrintPause::beginSenseOffsetSample() {
    InterruptProtectedBlock noInts;
    const SenseOffsetSample pauseSample = {isPaused(), pauseSinceSenseSample_};
    pauseSinceSenseSample_ = false;
    return pauseSample;
}

void PrintPause::waitUntilSettled() {
    while (isPaused() && !isSettled() && !Printer::blockAll)
        Commands::checkForPeriodicalActions(Paused);
}

void PrintPause::retract(Destination destination) {
    // Publish the active phase before calling anything that can service the loop.
    phase_ = Phase::SettlingExtrusion;
    parked_ = false;
    for (uint8_t axis = X_AXIS; axis <= Z_AXIS; ++axis)
        returnPosition_[axis] = Printer::directPosition(axis).steps();
    extruderReturnSteps_ = 0;
    Printer::idleStatus.cancel();
    Printer::setMenuMode(MENU_MODE_PAUSED, true);
    UI_STATUS_UPD(UI_TEXT_PAUSING);
    uid.exitmenu();
    Com::printFLN(PSTR("RequestPause:"));

    // The queued move can be frozen with E steps still due. Complete those
    // before the pause retract starts keeping its own return distance.
    if (!finishExtrusion(Phase::SettlingExtrusion))
        return;
    phase_ = Phase::Retracting;
    if (!moveWhile(Phase::Retracting, 0, 0, 0, -abs(parkSteps_[E_AXIS])))
        return;

    if (destination == Destination::Park) {
        park();
    } else {
        UI_STATUS_UPD(UI_TEXT_PAUSED);
        settle(HAL::timeInMilliseconds());
    }
}

void PrintPause::park() {
    const bool fromHold = phase_ == Phase::ParkRequested;
    phase_ = Phase::Parking;
    if (fromHold) {
        UI_STATUS_UPD(UI_TEXT_PAUSING);
        uid.exitmenu();
    }
    if (!moveWhile(Phase::Parking, 0, 0, abs(parkSteps_[Z_AXIS]), 0))
        return;
    if (!moveWhile(Phase::Parking, parkSteps_[X_AXIS], parkSteps_[Y_AXIS], 0, 0))
        return;

    parked_ = true;
    UI_STATUS_UPD(UI_TEXT_PAUSED);
    settle(HAL::timeInMilliseconds());
}

void PrintPause::returnToPrint() {
    // A manual extrusion jog may have ended while its last E steps are due.
    phase_ = Phase::SettlingExtrusion;
    if (!finishExtrusion(Phase::SettlingExtrusion))
        return;
    phase_ = Phase::Returning;
    if (moveBackToPrintPosition()) {
        Com::printFLN(PSTR("RequestContinue:"));
        cancel();
        if (!PrintLine::hasLines())
            Printer::idleStatus.schedule(HAL::timeInMilliseconds());
        UI_STATUS_UPD(UI_TEXT_PRINT_POS);
        return;
    }

    if (phase_ != Phase::Returning || Printer::blockAll)
        return;

    Com::printFLN(PSTR("Pause return incomplete"));
    showWarning(ui_text_pause, ui_text_operation_denied);
    settle(HAL::timeInMilliseconds());
}

bool PrintPause::moveBackToPrintPosition() {
    // Check the displacement actually reached before returning the next axis.
    if (!moveWhile(Phase::Returning, stepsToReturn(X_AXIS), stepsToReturn(Y_AXIS), 0, 0)
        || stepsToReturn(X_AXIS) || stepsToReturn(Y_AXIS))
        return false;
    if (!moveWhile(Phase::Returning, 0, 0, stepsToReturn(Z_AXIS), 0) || stepsToReturn(Z_AXIS))
        return false;
    return moveWhile(Phase::Returning, 0, 0, 0, extruderReturnSteps_) && !extruderReturnSteps_;
}

bool PrintPause::restoreExtruderTemperatures() {
    bool waitForHeat = false;
    for (uint8_t i = 0; i < NUM_EXTRUDER; ++i) {
        setExtruderCurrent(i, Drv8711::configuredCurrent(E_AXIS + i));
        auto& heater = extruder[i].tempControl;
        if (heater.paused) {
            Extruder::setTemperatureForExtruder(heater.targetTemperatureC + heater.paused, i);
            waitForHeat = true;
        }
    }
    if (waitForHeat) {
        UI_STATUS_UPD(UI_TEXT_HEATING_UP);
        for (uint8_t i = 0; i < NUM_EXTRUDER && isReheating(); ++i)
            extruder[i].tempControl.waitForTargetTemperature(ADD_CONTINUE_AFTER_PAUSE_TEMP_TOLERANCE, true);
    }
    return waitForHeat;
}

void PrintPause::settle(uint32_t now, uint32_t extraWarmTime) {
    standbyAt_ = now + standbyDelayMs + extraWarmTime;
    phase_ = Phase::Paused;
}

bool PrintPause::finishExtrusion(Phase phase) {
    auto& output = Printer::extrusionOutput;
    while (phase_ == phase && !Printer::blockAll) {
        {
            InterruptProtectedBlock lock;
            if (!PrintLine::directMoveActive() && output.drain())
                return true;
        }
        Commands::checkForPeriodicalActions(Paused);
    }
    output.cancelDrain();
    return false;
}

bool PrintPause::moveWhile(Phase phase, int32_t x, int32_t y, int32_t z, int32_t e) {
    Printer::offsetRelativeStepsCoordinates(x, y, z, e);
    return phase_ == phase && !Printer::blockAll && (!e || finishExtrusion(phase));
}

int32_t PrintPause::stepsToReturn(uint8_t axis) const {
    InterruptProtectedBlock noInts;
    return returnPosition_[axis] - Printer::directPosition(axis).steps();
}

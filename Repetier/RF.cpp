/*
    This file is part of the Repetier-Firmware for RF devices from Conrad Electronic SE.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Repetier.h"
#include "BedMatrix.h"
#include "CaseFan.h"
#include "ExternalEeprom.h"
#include "ScanProbe.h"
#include "GCodeInput.h"

namespace AccessoryOutputs {
namespace {
char states[] = {
    CASE_LIGHTS_DEFAULT_ON,
#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    0, FET1_DEFAULT_ON, FET2_DEFAULT_ON, FET3_DEFAULT_ON
#endif
};
}

char state(Output output) {
    return states[static_cast<uint8_t>(output)];
}

void stage(Output output, char value) {
    states[static_cast<uint8_t>(output)] = value;
}

void apply(Output output) {
    switch (output) {
#if !HOTEND_COOLER_USES_PIN(CASE_LIGHT_PIN)
    case Output::CaseLight: WRITE(CASE_LIGHT_PIN, state(output)); break;
#endif
#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    case Output::Mains: WRITE(OUTPUT_230V_PIN, state(output)); break;
#if FET1 > -1 && !HOTEND_COOLER_USES_PIN(FET1)
    case Output::Fet1: WRITE(FET1, state(output)); break;
#endif
#if FET2 > -1 && !HOTEND_COOLER_USES_PIN(FET2)
    case Output::Fet2: WRITE(FET2, state(output)); break;
#endif
#if FET3 > -1
    // Legacy M3300 P3 is a raw override even when CaseFan uses X45.
    case Output::Fet3: WRITE(FET3, state(output)); break;
#endif
#endif
    default: break;
    }
}

void set(Output output, char value) {
    stage(output, value);
    apply(output);
}

void toggle(Output output) {
    set(output, !state(output));
}

void initializeBoardOutputs() {
#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    stage(Output::Mains, 0);
    SET_OUTPUT(OUTPUT_230V_PIN);
    apply(Output::Mains);
#if FET1 > -1 && !HOTEND_COOLER_USES_PIN(FET1)
    SET_OUTPUT(FET1);
    apply(Output::Fet1);
#endif
#if FET2 > -1 && !HOTEND_COOLER_USES_PIN(FET2)
    SET_OUTPUT(FET2);
    apply(Output::Fet2);
#endif
#if FET3 > -1 && CASE_FAN_PIN < 0
    SET_OUTPUT(FET3);
    apply(Output::Fet3);
#endif
#endif
}

void initializeCaseLight() {
#if !HOTEND_COOLER_USES_PIN(CASE_LIGHT_PIN)
    SET_OUTPUT(CASE_LIGHT_PIN);
    apply(Output::CaseLight);
#endif
}
} // namespace AccessoryOutputs

static void setupForPrinting() {
    Printer::setSomeTempsensorDefect(false);

    // read the settings from the EEPROM
    Printer::setAxisHomingFeedrate(X_AXIS, HAL::eprGetFloat(EPR_X_HOMING_FEEDRATE_PRINT));
    Printer::setAxisHomingFeedrate(Y_AXIS, HAL::eprGetFloat(EPR_Y_HOMING_FEEDRATE_PRINT));
    Printer::setAxisHomingFeedrate(Z_AXIS, HAL::eprGetFloat(EPR_Z_HOMING_FEEDRATE_PRINT));

    Printer::setAxisLengthMm(X_AXIS, HAL::eprGetFloat(EPR_X_LENGTH));
    if (Printer::axisLengthMm(X_AXIS) <= 0 || Printer::axisLengthMm(X_AXIS) > 245.0f) {
        Printer::setAxisLengthMm(X_AXIS, X_MAX_LENGTH_PRINT);
        HAL::eprSetFloat(EPR_X_LENGTH, Printer::axisLengthMm(X_AXIS));
        EEPROM::updateChecksum();
    }

    BedMatrix::restoreAtStartup();

    Printer::pause.restoreDefaultParking();

    Printer::updateDerivedParameter();

    Printer::idleStatus.schedule(HAL::timeInMilliseconds());
} // setupForPrinting

void initRF(void) {
    Printer::strainGauge.begin(HAL::timeInMilliseconds());

    Printer::disableCMPnow(true);
    setupForPrinting();
} // initRF

bool isAnyScanRunning() {
    return heatBedScanRunning() || Calibration::zOffsetScanRunning();
}

void discardScans() {
    // A homing or move wait can service the print stop while a scan phase is on
    // the stack. Complete the discard when that phase returns.
    if (Calibration::scanSession.deferDiscard())
        return;
    Calibration::discardHeatBedScan();
    Calibration::discardZOffsetScan();
}

void handleScanWorkTasks(bool tickDue) {
    if (Calibration::scanSession.dispatching() || !isAnyScanRunning())
        return;
    if (Printer::blockAll || Printer::printJob.commandsBlocked()) {
        discardScans();
        return;
    }
    if (Printer::pause.isPaused())
        return;

    Calibration::scanSession.beginDispatch();
    const bool measurementReady = !Calibration::scanSession.aborting() && Calibration::scanPressure.service(HAL::timeInMilliseconds());
    if (tickDue || measurementReady) {
        if (heatBedScanRunning())
            Calibration::tickHeatBedScan();
        else
            Calibration::tickZOffsetScan();
    }
    if (Calibration::scanSession.finishDispatch() || Printer::blockAll || Printer::printJob.commandsBlocked())
        discardScans();
}

void handleStartStandby(millis_t uTime) {
#if (MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2) && CASE_FAN_PIN > -1
    CaseFan::service();
#endif

    if (!Printer::pause.isPaused() && !Printer::startLine.active()) {
        const auto idleTransition = Printer::idleStatus.poll(uTime, MINIMAL_IDLE_TIME);
        if (idleTransition == IdleStatus::Transition::Deferred)
            return;
        if (idleTransition == IdleStatus::Transition::Ready && !PrintLine::hasLines() && !PrintLine::directMoveActive()) {
            Printer::printJob.finishPrintMotion();
            showIdle();
        }
    }

    /**
     * Check if the Printer should turn off Steppers or Everything
     */
    // A preceding service may have recorded activity after the initial clock sample.
    const millis_t inactivityTime = HAL::timeInMilliseconds();
    if (PrintLine::hasLines() || Printer::printJob.isBusy() || Printer::pause.isPaused()) {
        Printer::inactivity.recordActivity(inactivityTime);
    } else {
        if (Printer::inactivity.shutdownTimeoutExpired(inactivityTime)) {
            Printer::switchEverythingOff(); //kill not only steppers
        } else {
            Printer::setAllSwitchedOff(false); // reset if not time to kill: prevents repeated kills
        }

        if (Printer::inactivity.stepperTimeoutExpired(inactivityTime) && !Printer::areAllSteppersDisabled()) {
            Printer::disableAllSteppersNow();
            Commands::setFanSpeed((uint8_t)0);
        }
    }
}

void outputObject(bool showerrors) {
    if (PrintLine::hasLines() || !Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Cancel)) {
        if (showerrors)
            showError((void*)ui_text_output_object, (void*)ui_text_operation_denied);
        return;
    }
    if (!Printer::areAxisHomed()) {
        if (showerrors)
            showError((void*)ui_text_output_object, (void*)ui_text_home_unknown);
        return;
    }

    Printer::idleStatus.cancel();
    UI_STATUS_UPD(UI_TEXT_OUTPUTTING_OBJECT);
    Com::printFLN(PSTR("outputObject"));
    Commands::printCurrentPosition();

    // disable the fan
    Commands::setFanSpeed((uint8_t)0);

    {
        GCodeInput::executeFlashScript(Com::tOutputObjectPrint);
    }

    if (!Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Cancel))
        return;

    // disable all steppers
    Printer::disableAllSteppersNow();

    Printer::idleStatus.schedule(HAL::timeInMilliseconds());
} // outputObject

void loopFeatures() //wird so aufgerufen, dass es ein ~100ms Takt sein sollte.
{
    static char nEntered = 0;
    if (!nEntered) {
        nEntered++;

        millis_t uTime = HAL::timeInMilliseconds();
        Printer::printJob.service(uTime);

        handleStartStandby(uTime);
        handleStrainGaugeFeatures(uTime);

        Printer::pause.serviceStandby(uTime);

#ifdef RGB_LIGHT_R_PIN
        RgbLighting::updateStatus();
#endif // RGB_LIGHT_R_PIN

        nEntered--;
    }

    // Direct-move waits can re-enter this loop. Only pending requests start a transition.
    Printer::pause.serviceMotion();
} // loopFeatures

void queueTask(char task) {
    if (PrintLine::waitForXFreeLines(1))
        PrintLine::queueTask(task);
} // queueTask

void cleanupEPositions(void) {
    InterruptProtectedBlock noInts;

    Printer::setEPositionMM(0);
    Printer::resetDirectAxis(E_AXIS);

    noInts.unprotect();
} // cleanupEPositions

unsigned char isMovingAllowed(const char* pszCommand, char outputLog) {
    if (!Printer::movementUnlocked()) {
        // A host may keep sending motion commands after a controller reset.
        Com::printF(pszCommand);
        Com::printFLN(PSTR(": movement locked until homing or panel input"));
        return 0;
    }
    if (Printer::blockAll) {
        // do not allow to move in case the movements have been blocked
        return 0;
    }

    if (isAnyScanRunning()) {
        // do not allow manual movements while the heat bed scan is in progress
        if (Printer::debugErrors() && outputLog) {
            Com::printF(pszCommand);
            Com::printFLN(PSTR(": command can not be used while scan is in progress"));
        }
        return 0;
    }

    // we allow the manual movements at the moment
    return 1;

} // isMovingAllowed

unsigned char isHomingAllowed(GCode* com, char outputLog) {

    if (isAnyScanRunning()) {
        // do not allow homing while the heat bed scan is in progress
        if (Printer::debugErrors() && outputLog) {
            Com::printFLN(PSTR("G28: homing can not be performed while the heat bed scan is in progress"));
        }
        showError((void*)ui_text_home, (void*)ui_text_operation_denied);

        return 0;
    }

    if (!com) {
        // there is nothing more which we can check
        return 1;
    }

    // we allow the homing at the moment
    return 1;
} // isHomingAllowed

void determineHardwareType(void) {
    unsigned short uTemp;

    Printer::wrongType = 0;

    // check the stored header format
    uTemp = ExternalEeprom::readWord(I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_HEADER_FORMAT);

    if (uTemp != TYPE_EEPROM_FORMAT) {
        // we could not read the header format or the header format is wrong
        ExternalEeprom::writeWord(I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_HEADER_FORMAT, TYPE_EEPROM_FORMAT);
        ExternalEeprom::writeWord(I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_BOARD_TYPE, MOTHERBOARD);

        uTemp = ExternalEeprom::readWord(I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_HEADER_FORMAT);

        if (uTemp != TYPE_EEPROM_FORMAT) {
            if (MOTHERBOARD == DEVICE_TYPE_RF1000) {
                // the RF1000 does not provide the type EEPROM, thus we should run at an RF1000 board when we end up here
                return;
            }

            // we end up here in case this firmware is for the RF2000, but the current board does not seem to be an RF2000 board
            Printer::wrongType = 1;
            return;
        }

        if (MOTHERBOARD == DEVICE_TYPE_RF1000) {
            // we end up here in case this firmware is for the RF1000, but the current board seems to be an RF2000 board
            Printer::wrongType = 1;
            return;
        }

        // when we end up here this firmware is for the RF2000 and the current board seems to be an RF2000 board
        return;
    }

    if (MOTHERBOARD == DEVICE_TYPE_RF1000) {
        // we end up here in case this firmware is for the RF1000, but the current board seems to be an RF2000 board
        Printer::wrongType = 1;
        return;
    }

    // when we end up here this firmware is for the RF2000 and the current board seems to be an RF2000 board
    if (ExternalEeprom::readWord(I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_BOARD_TYPE) != MOTHERBOARD) {
        ExternalEeprom::writeWord(I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_BOARD_TYPE, MOTHERBOARD);
    }

    return;

} // determineHardwareType

void doEmergencyStop(char reason) {
    {
        InterruptProtectedBlock lock;
        Printer::blockAll = 1;
        Printer::extrusionOutput.inhibit(ExtrusionOutput::Inhibit::Emergency);
    }

    showError((void*)ui_text_emergency_stop);

    Com::printF(PSTR("doEmergencyStop(): block all"));
    if (reason == STOP_BECAUSE_OF_Z_MIN) {
        Com::printFLN(PSTR(" (Z-Min)"));
    } else if (reason == STOP_BECAUSE_OF_Z_BLOCK) {
        Com::printFLN(PSTR(" (Z-Block)"));
    }

    Printer::stopPrint(); //and tell usb senders to stop

    Calibration::moveZ(int(Printer::stepsPerMm(Z_AXIS) * 5), true);

    // we are not going to perform any further operations until the restart of the firmware
    Printer::switchEverythingOff();
} // doEmergencyStop

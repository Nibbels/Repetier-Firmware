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

// RF board integration and accessory outputs; scan, pressure, matrix and lighting services live in their own modules.

#include "Repetier.h"
#include "EepromZoneAvr.h"
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
    case Output::CaseLight: Lighting::set(state(output) ? Lighting::White : Lighting::Black); break;
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
    Lighting::initialize();
    apply(Output::CaseLight);
}
} // namespace AccessoryOutputs

static void setupForPrinting() {
    uid.beginBootEye();
    BedMatrix::restoreAtStartup([]() { uid.serviceBootEye(); });
    uid.finishBootEye();
#ifdef RF_EEPROM_ZONE_EXPERIMENT
    EEPROM::reportRecovery();
#endif

    Printer::pause.restoreDefaultParking();

    Printer::updateDerivedParameter();

    Printer::idleStatus.schedule(HAL::timeInMilliseconds());
} // setupForPrinting

void initRF(void) {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    // Recovery was recognized by startWatchdog() at the end of setup.
    // Do not run the normal matrix/boot-animation/idle initialization
    // after that terminal state. Diagnostic-only scheduler owns progress.
    if(Printer::printJob.emergencyLocked())return;
#endif
    Printer::strainGauge.begin(HAL::timeInMilliseconds());
    EepromZones::AvrDevice::enableRuntimeServices();

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

static __attribute__((noinline)) void dispatchScanWork(bool tickDue) {
    const bool measurementReady = !Calibration::scanSession.aborting() && Calibration::scanPressure.service(HAL::timeInMilliseconds());
    if (tickDue || measurementReady) {
        if (heatBedScanRunning())
            Calibration::tickHeatBedScan();
        else
            Calibration::tickZOffsetScan();
    }
}

// Keep the guarded scan dispatch frame out of the reentrant periodic pump.
__attribute__((noinline)) void handleScanWorkTasks(bool tickDue) {
    if (Calibration::scanSession.dispatching() || !isAnyScanRunning())
        return;
    if (Printer::blockAll || Printer::printJob.commandsBlocked()) {
        discardScans();
        return;
    }
    if (!Printer::printJob.isIdle() || Printer::pause.blocksCalibration() || GCodeInput::flashScriptActive())
        return;

    Calibration::scanSession.beginDispatch();
    dispatchScanWork(tickDue);
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
    if (PrintLine::hasLines() || PrintLine::directMoveActive() || Printer::printJob.isBusy() || Printer::pause.isPaused()) {
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
    const auto admitted = [] {
        return !GCodeInput::flashScriptActive() && !isAnyScanRunning() &&
               (Printer::printJob.isIdle() || Printer::printJob.isPrinting());
    };
    // A wait can pump the panel. Never enter an output script after a
    // calibration or filament operation has acquired the machine.
    if (!admitted() || PrintLine::hasLines() ||
        !Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Cancel) ||
        !admitted()) {
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

    // A canceled/refused output program must not reach motor shutdown.
    if (!GCodeInput::executeOutputProgram())
        return;

    if (!Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Cancel))
        return;

    // disable all steppers
    Printer::disableAllSteppersNow();

    Printer::idleStatus.schedule(HAL::timeInMilliseconds());
} // outputObject

static __attribute__((noinline)) void servicePrintFeatures() {
    millis_t uTime = HAL::timeInMilliseconds();
    Printer::printJob.service(uTime);

    handleStartStandby(uTime);
    handleStrainGaugeFeatures(uTime);

    Printer::pause.serviceStandby(uTime);
}

// The periodic pump can re-enter through feature/motion waits. Do not
// reserve the feature dispatch frame in every invocation of that pump.
__attribute__((noinline)) void loopFeatures() // Wird so aufgerufen, dass es ein ~100ms Takt sein sollte.
{
    static char nEntered = 0;
    if (!nEntered) {
        nEntered++;

        servicePrintFeatures();
        nEntered--;
    }

    // Direct-move waits can re-enter this loop. Only pending requests start a transition.
    Printer::pause.serviceMotion();
} // loopFeatures

void queueTask(QueueTask task) {
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
        if (outputLog) {
            Com::printF(pszCommand);
            Com::printFLN(PSTR(": command can not be used while scan is in progress"));
        }
        return 0;
    }

    // we allow the manual movements at the moment
    return 1;

} // isMovingAllowed

unsigned char isHomingAllowed(GCode* com, char outputLog) {
#ifdef RF_EEPROM_ZONE_EXPERIMENT
    if(!EEPROM::settingsReady())return 0;
#endif

    if (isAnyScanRunning()) {
        // do not allow homing while the heat bed scan is in progress
        if (outputLog) {
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

bool hardwareTypeMatches() {
    unsigned short uTemp;

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
                return true;
            }

            // we end up here in case this firmware is for the RF2000, but the current board does not seem to be an RF2000 board
            return false;
        }

        if (MOTHERBOARD == DEVICE_TYPE_RF1000) {
            // we end up here in case this firmware is for the RF1000, but the current board seems to be an RF2000 board
            return false;
        }

        // when we end up here this firmware is for the RF2000 and the current board seems to be an RF2000 board
        return true;
    }

    if (MOTHERBOARD == DEVICE_TYPE_RF1000) {
        // we end up here in case this firmware is for the RF1000, but the current board seems to be an RF2000 board
        return false;
    }

    // when we end up here this firmware is for the RF2000 and the current board seems to be an RF2000 board
    if (ExternalEeprom::readWord(I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_BOARD_TYPE) != MOTHERBOARD) {
        ExternalEeprom::writeWord(I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_BOARD_TYPE, MOTHERBOARD);
    }

    return true;

} // hardwareTypeMatches

void doEmergencyStop(StopReason reason) {
    {
        InterruptProtectedBlock lock;
        if (Printer::blockAll)
            return;
        Printer::blockAll = 1;
        Printer::lockMovement();
        Printer::extrusionOutput.inhibit(ExtrusionOutput::Inhibit::Emergency);
        Printer::pause.cancel();
        Printer::idleStatus.cancel();
        PrintLine::abortDirectMoveForEmergency();
        PrintLine::discardQueuedMoves();
        for (uint8_t axis = X_AXIS; axis <= E_AXIS; ++axis)
            Printer::stopDirectAxis(axis);
    }

    showError((void*)ui_text_emergency_stop);

    Com::printF(PSTR("doEmergencyStop(): block all"));
    if (reason == StopReason::ZMin) {
        Com::printFLN(PSTR(" (Z-Min)"));
    } else if (reason == StopReason::ZBlock) {
        Com::printFLN(PSTR(" (Z-Block)"));
    }

    Printer::stopPrint(); // And tell usb senders to stop

    Calibration::moveZ(int(Printer::stepsPerMm(Z_AXIS) * 5), true);
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    // Thermal emergency takes precedence even during the retreat. Its
    // holding-torque policy must not be overridden by this older frame.
    if (Printer::printJob.emergencyLocked())
        return;
#endif

    // we are not going to perform any further operations until the restart of the firmware
    Printer::switchEverythingOff();
} // doEmergencyStop

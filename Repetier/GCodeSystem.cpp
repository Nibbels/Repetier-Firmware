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

// System, diagnostics and configuration G-code handlers, including legacy simulation refusal.

#include "Repetier.h"
#include "GCodeCommands.h"

namespace GCodeCommands {

void showInvalidSyntax(unsigned int currentMCode) {
    if (Printer::debugErrors()) {
        Com::printF(PSTR("M"), (int)currentMCode);
        Com::printFLN(PSTR(": invalid syntax"));
    }
}


// Process the retired bit at receive admission, even while a stop drains input.
// Never let a subsequent debug mask re-arm machine outputs.
bool rejectLegacySimulation(GCode* com) {
    if (!com->hasM() || com->M != 111 || !com->hasS() || !(static_cast<uint8_t>(com->S) & 8))
        return false;
    Printer::setDebugMask(com->S);
    if (!Printer::legacySimulationRejected()) {
        Printer::latchOutputInhibit(false);
        Printer::stopPrint();
    }
    Com::printErrorFLN(PSTR("M111 bit 8 unsupported; job rejected. Stop host and restart printer."));
    return true;
}

void setDebugLevel(GCode* com) {
    if (com->hasS())
        Printer::setDebugMask(com->S);
}

void reportPosition() {
    Commands::printCurrentPosition();
}

void reportFirmware() {
    if (Printer::debugInfo()) {
        Com::printFLN(Com::tFirmware);
    }
    //		Com::cap(PSTR("EEPROM:1"));
    //#else
    //		Com::cap(PSTR("EEPROM:0"));
    //#endif

    //		Com::cap(PSTR("EMERGENCY_PARSER:0"));
    Commands::reportPrinterUsage();
}

void setPanelMessage(GCode* com) {
    if (com->hasString()) {
        UI_STATUS_UPD_RAM(com->text);
    }
}

void reportEndstops() {
    if (!Printer::debugInfo()) {
        return;
    }

    Commands::waitForMotion(Commands::MotionWait::QueueAndZ, Commands::PauseWait::Resume);

    Com::printF(Com::tXMinColon);
    Com::printF(Printer::isXMinEndstopHit() ? Com::tHSpace : Com::tLSpace);

    Com::printF(Com::tSoftDash);
    Com::printF(Com::tXMaxColon);
    Com::printF(Printer::isXMaxEndstopHit() ? Com::tHSpace : Com::tLSpace);

    Com::printF(Com::tYMinColon);
    Com::printF(Printer::isYMinEndstopHit() ? Com::tHSpace : Com::tLSpace);

    Com::printF(Com::tSoftDash);
    Com::printF(Com::tYMaxColon);
    Com::printF(Printer::isYMaxEndstopHit() ? Com::tHSpace : Com::tLSpace);

    Com::printF(Com::tZMinColon);
    Com::printF(Printer::isZMinEndstopHit() ? Com::tHSpace : Com::tLSpace);

#if MOTHERBOARD == DEVICE_TYPE_RF1000
    Com::printF(Com::tSoftDash);
#endif
    Com::printF(Com::tZMaxColon);
    Com::printF(Printer::isZMaxEndstopHit() ? Com::tHSpace : Com::tLSpace);

    Com::println();
}

void reportEeprom() {
    EEPROM::writeSettings();
}

void setEepromValue(GCode* com) {
#ifdef RF_EEPROM_ZONE_EXPERIMENT
    // The checked field-specific owner decides admission, including live
    // DMIN/DMAX. Do not drain a print before validating the request.
    EEPROM::update(com);
#else
    if (Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Resume))
        EEPROM::update(com);
#endif
}

void reportConfiguration() {
    Printer::showConfiguration();
}

void saveConfiguration() {
    EEPROM::storeDataIntoEEPROM(false);
#ifdef RF_EEPROM_ZONE_EXPERIMENT
    if(!EEPROM::lastOperationSucceeded())return;
#endif
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(Printer::printJob.emergencyLocked())return;
#endif
    Com::printInfoF(Com::tConfigStoredEEPROM);
}

void loadConfiguration() {
    if (!Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Resume))
        return;
    EEPROM::readDataFromEEPROM();
#ifdef RF_EEPROM_ZONE_EXPERIMENT
    if(!EEPROM::lastOperationSucceeded())return;
#endif
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(Printer::printJob.emergencyLocked())return;
#endif

    if (Printer::debugInfo()) {
        Com::printInfoF(Com::tConfigLoadedEEPROM);
    }
}

void restoreConfigurationDefaults() {
    if (!Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Resume))
        return;
    EEPROM::restoreEEPROMSettingsFromConfiguration();
#ifdef RF_EEPROM_ZONE_EXPERIMENT
    if(!EEPROM::lastOperationSucceeded())return;
#else
    EEPROM::storeDataIntoEEPROM(false);
#endif
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(Printer::printJob.emergencyLocked())return;
#endif

    UI_STATUS(UI_TEXT_RESTORE_DEFAULTS);
    showInformation(PSTR(UI_TEXT_CONFIGURATION), PSTR(UI_TEXT_RESTORE_DEFAULTS), PSTR(UI_TEXT_OK));
}

void requestHeatBedScan(GCode* pCommand) {
    if (pCommand->hasS() && (pCommand->S == int(HeatBedScanMode::Pla) || pCommand->S == int(HeatBedScanMode::Abs))) {
        startHeatBedScan(static_cast<HeatBedScanMode>(pCommand->S));
        return;
    }
    startHeatBedScan(HeatBedScanMode::Standard);
}

void accessPanel(GCode* pCommand) {
    if (pCommand->hasP()) {
        switch (pCommand->P) {
        case UI_ACTION_OK:       // 1001
        case UI_ACTION_NEXT:     // 1
        case UI_ACTION_PREVIOUS: // 2
        case UI_ACTION_BACK:     // 1000
        case UI_ACTION_RIGHT:    // 1129

        case UI_ACTION_RF_HEAT_BED_UP:      // 514
        case UI_ACTION_RF_HEAT_BED_DOWN:    // 515
        case UI_ACTION_RF_EXTRUDER_RETRACT: // 517
        case UI_ACTION_RF_EXTRUDER_OUTPUT:  // 516
        case UI_ACTION_RF_CONTINUE:         // 1519
        case UI_ACTION_RF_PAUSE:            // 1518
        {
            Com::printFLN(PSTR("RequestMenu:Press:"), pCommand->P);
            uid.executeAction(pCommand->P);
        } break;
        }
    } else {
        Com::printF(PSTR("RequestMenu:"), UI_ROWS);

        Com::printF(PSTR(","), MAX_COLS + 1);
        Com::printF(PSTR(":"));
        if (pCommand->hasL()) {
            // Print line by line for human readability
            Com::println();
            Com::println();
        }
        // we print out UI_ROWS x MAX_COLS+1 field.
        for (uint8_t row = 0; row < UI_ROWS; row++) {
            const char* line = uid.cachedRow(row);
            bool nulled = false;
            for (uint8_t col = 0; col < MAX_COLS + 1; col++) {
                const char character = line[col];
                if (character == 0 || nulled) {
                    nulled = true;
                    Com::print(' '); //null as space
                    continue;
                }

                if (isprint(character)) {
                    Com::print(character);
                    continue;
                }

                switch (character) {
                case 2: {
                    Com::print('\'');
                    break;
                }
                case 5: {
                    Com::print('!');
                    break;
                }
                case (char)CHAR_SELECTOR: {
                    Com::print('>');
                    break;
                }
                case (char)CHAR_SELECTED: {
                    Com::print('*');
                    break;
                }
                default:
                    Com::print('_'); //others?
                }
            }
            if (pCommand->hasL()) {
                // Print line by line for human readability
                Com::println();
            }
        }
        Com::println();
    }
}

void runRfDiagnostic(GCode* pCommand) {
    if (pCommand->hasP()) {
        switch (pCommand->P) {
        case 1: {
            Commands::checkFreeMemory();
            Commands::writeLowestFreeRAM();
            Com::printFLN(PSTR("lowest free RAM: "), Commands::lowestRAMValue);

            Com::printFLN(PSTR("z-compensation matrix x: "), COMPENSATION_MATRIX_MAX_X);
            Com::printFLN(PSTR("z-compensation matrix y: "), COMPENSATION_MATRIX_MAX_Y);
            break;
        }
        case 2: {
            Com::printFLN(PSTR("Homing feedrates:"));
            Com::printF(PSTR("x = "), Printer::axisHomingFeedrate(X_AXIS));
            Com::printF(PSTR(", y = "), Printer::axisHomingFeedrate(Y_AXIS));
            Com::printFLN(PSTR(", z = "), Printer::axisHomingFeedrate(Z_AXIS));
            break;
        }
        case 4: {
            // simulate blocking of all axes
            Com::printFLN(PSTR("M3200: block all"));

            doEmergencyStop(StopReason::ZBlock);
            break;
        }
        case 5: {
            // simulate a temp sensor error
            Com::printFLN(PSTR("M3200: simulating defect sensor"));
            //ext 0 soll defekt aussehen
            TemperatureController* act0 = tempController[0];
            act0->setSensorDefect(true);
            //ext 1/bett soll decoupled aussehen
            TemperatureController* act1 = tempController[1];
            act1->setSensorDecoupled(true);
            // Generelle defekt-einstufung
            Printer::latchOutputInhibit(true);
            Printer::stopPrint();
            // Fehlermeldung ausgeben
            reportTempsensorAndHeaterErrors();
            break;
        }


        case 9: {
            Com::printFLN(PSTR("debug level: "), Printer::debugMask());
            break;
        }

        case 13: {
            Com::printFLN(PSTR("LCD re-initialization"));
            uid.initializeLCD();
            break;
        }
        case 14: {
            const auto reserve = PrintLine::queuedMoves().remainingMotion();
            Com::printF(PSTR("Motion buffer XYZ mm: "), reserve.xyzMm, 3);
            Com::printF(PSTR(" E mm: "), reserve.filamentMm, 3);
            Com::printFLN(PSTR(" s: "), reserve.seconds, 4);
            break;
        }
        }
    }
}

void setPanelStatusOverride(GCode* pCommand) {
    if (pCommand->hasString()) {
        if (!uid.isLocked()) {
            // ensure that the current text won't be overwritten
            uid.lock();
            if (Printer::debugInfo()) {
                Com::printFLN(PSTR("M3117: lock"));
            }
        }
        uid.setStatus(pCommand->text, false, true);
        uid.refreshPage();
    } else {
        if (uid.isLocked()) {
            // allow to overwrite the current string again
            // Put the unlock into queue to avoid racetime conditions with prior commands
            queueTask(QueueTask::UnlockDisplay);
            Com::printFLN(PSTR("M3117: queued unlock"));
        }
    }
}

} // namespace GCodeCommands

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
#include "GCodeCommands.h"

namespace GCodeCommands {

void showInvalidSyntax(unsigned int currentMCode) {
    if (Printer::debugErrors()) {
        Com::printF(PSTR("M"), (int)currentMCode);
        Com::printFLN(PSTR(": invalid syntax"));
    }
}


void setDebugLevel(GCode* com) {
    if (com->hasS()) {
        Printer::debugLevel = com->S;
    }
    if (Printer::debugDryrun()) // simulate movements without printing
    {
        Extruder::setTemperatureForAllExtruders(0);
    }
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
    if (Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Resume))
        EEPROM::update(com);
}

void reportConfiguration() {
    Printer::showConfiguration();
}

void saveConfiguration() {
    EEPROM::storeDataIntoEEPROM(false);
    Com::printInfoF(Com::tConfigStoredEEPROM);
}

void loadConfiguration() {
    if (!Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Resume))
        return;
    EEPROM::readDataFromEEPROM();

    if (Printer::debugInfo()) {
        Com::printInfoF(Com::tConfigLoadedEEPROM);
    }
}

void restoreConfigurationDefaults() {
    if (!Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Resume))
        return;
    EEPROM::restoreEEPROMSettingsFromConfiguration();
    EEPROM::storeDataIntoEEPROM(false);

    UI_STATUS(UI_TEXT_RESTORE_DEFAULTS);
    showInformation(PSTR(UI_TEXT_CONFIGURATION), PSTR(UI_TEXT_RESTORE_DEFAULTS), PSTR(UI_TEXT_OK));
}

void configureRfDebug(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 0)
            nTemp = 0;

        g_debugLevel = (char)nTemp;
        if (Printer::debugInfo()) {
            Com::printFLN(PSTR("M3005: new debug level: "), g_debugLevel);
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
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
            //print line by line for human readability
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
                //print line by line for human readability
                Com::println();
            }
        }
        Com::println();
    }
}

void reportDeviceVersion() {
    Com::printFLN(PSTR("Device Type: "), UI_PRINTER_NAME);
    Com::printFLN(PSTR("Firmware Version: "), UI_VERSION_STRING);
}

void testWatchdog(GCode* pCommand) {
    if (pCommand->hasS()) {
        if (pCommand->S == 666) {
            if (HAL::watchdogEnabled()) {
                HAL::stopWatchdog();
                Com::printFLN(PSTR("M3090: WARNING the watchdog is disabled now"));
            } else {
                HAL::startWatchdog();
                Com::printFLN(PSTR("M3090: the watchdog has been activated"));
            }
        }
    } else if (pCommand->hasP()) {
        HAL::delayMilliseconds(10000);
    } else if (pCommand->hasT()) {
        for (int iii = 0; iii < 1000; iii++)
            HAL::delayMicroseconds(10000);
    } else {
        if (Printer::debugInfo()) {
            Com::printFLN(PSTR("M3090: the watchdog is going to reset the firmware"));
        }
        HAL::delayMilliseconds(100);
        HAL::testWatchdog();
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

            doEmergencyStop(STOP_BECAUSE_OF_Z_BLOCK);
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
            //generelle defekt-einstufung
            Printer::setSomeTempsensorDefect(true);
            //fehlermeldung ausgeben
            reportTempsensorAndHeaterErrors();
            break;
        }


        case 9: {
            Com::printFLN(PSTR("debug level: "), Printer::debugLevel);
            break;
        }

        case 13: {
            Com::printFLN(PSTR("LCD re-initialization"));
            uid.initializeLCD();
        }
        }
    }
}

void setPanelStatusOverride(GCode* pCommand) {
    if (pCommand->hasString()) {
        if (!uid.locked) {
            // ensure that the current text won't be overwritten
            uid.lock();
            if (Printer::debugInfo()) {
                Com::printFLN(PSTR("M3117: lock"));
            }
        }
        uid.setStatus(pCommand->text, false, true);
        uid.refreshPage();
    } else {
        if (uid.locked) {
            // allow to overwrite the current string again
            // Put the unlock into queue to avoid racetime conditions with prior commands
            queueTask(TASK_UNLOCK_DISPLAY_MSG);
            Com::printFLN(PSTR("M3117: queued unlock"));
        }
    }
}

} // namespace GCodeCommands

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
    {
        Com::printF(PSTR("M"), (int)currentMCode);
        Com::printFLN(PSTR(": invalid syntax"));
    }
}


// Reject retired input modes at receive admission, even while a stop drains input.
// Retired modes remain rejected; no debug state or command is supported.
bool rejectLegacySimulation(GCode* com) {
    const bool inchMode = com->hasG() && com->G == 20;
    const bool simulation = com->hasM() && com->M == 111 && com->hasS() && (static_cast<uint8_t>(com->S) & 8);
    if (!inchMode && !simulation)
        return false;
    if (!Printer::legacySimulationRejected()) {
        Printer::latchOutputInhibit(false);
        Printer::stopPrint();
    }
    if (inchMode)
        Com::printErrorFLN(PSTR("G20 unsupported; millimeters only. Stop host and restart printer."));
    else
        Com::printErrorFLN(PSTR("M111 bit 8 unsupported; job rejected. Stop host and restart printer."));
    return true;
}

void reportPosition() {
    Commands::printCurrentPosition();
}

void reportFirmware() {
    Com::printFLN(Com::tFirmware);
    //		Com::cap(PSTR("EEPROM:1"));
    //#else
    //		Com::cap(PSTR("EEPROM:0"));
    //#endif

    //		Com::cap(PSTR("EMERGENCY_PARSER:0"));
}

void setPanelMessage(GCode* com) {
    if (com->hasString()) {
        UI_STATUS_UPD_RAM(com->text);
    }
}

void reportEndstops() {
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
    // The checked field-specific owner decides admission, including live
    // DMIN/DMAX. Do not drain a print before validating the request.
    EEPROM::update(com);

}

void reportConfiguration() {
    Printer::showConfiguration();
}

void restoreConfigurationDefaults() {
    if (!Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Resume))
        return;
    EEPROM::restoreEEPROMSettingsFromConfiguration();
    if(!EEPROM::lastOperationSucceeded() || EEPROM::recoveryPending())return;
    if(Printer::printJob.emergencyLocked())return;

    UI_STATUS(UI_TEXT_RESTORE_DEFAULTS);
    showInformation(PSTR(UI_TEXT_CONFIGURATION), PSTR(UI_TEXT_RESTORE_DEFAULTS), PSTR(UI_TEXT_OK));
}

} // namespace GCodeCommands

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

// G/M/T dispatch and rejected-job admission; domain handlers are in GCodeMotion, GCodeThermal and GCodeSystem.

#include "Repetier.h"
#include "GCodeCommands.h"
#include "DiagnosticHistory.h"

namespace GCodeCommands {
namespace {

void dispatchMotion(GCode* com) {
    switch (com->G) {
    case 0:
    case 1: queueLinearMove(com); break;
    case 4: dwell(com); break;
    case 21: break; // Millimeters are the only supported unit.
    case 28: homeAxes(com); break;
    case 90: selectAbsolutePositioning(com); break;
    case 91: selectRelativePositioning(com); break;
    case 92: setLogicalPosition(com); break;
    }
    Printer::inactivity.recordActivity(HAL::timeInMilliseconds());
}

void dispatchMachine(GCode* com) {
    switch (com->M) {
    // Motion, extrusion, tools and inactivity.
    case 82: return selectAbsoluteExtrusion();
    case 83: return selectRelativeExtrusion();
    case 84: return configureStepperShutdown(com);
    case 201: return setPrintAcceleration(com);
    case 202: return setTravelAcceleration(com);
    case 220: return setFeedrateMultiplier(com);
    case 221: return setFlowMultiplier(com);
    case 233: return configureAdvance(com);
    case 400: return finishMotion();
    case 3006: return setManualZOffset(com);
    case 3070: return requestPrintPause(com);
    case 3079: return outputObject();
    case 3105: return setPauseParkingOffsets(com);
    case 3912:
        Printer::startLine.run(*com);
        return;
    case 3913: return loadFilament(com);
    case 3914: return unloadFilament(com);

    // Heaters, fans and board outputs.
    case 104: return setHotendTemperature(com);
    case 105: return reportTemperature(com);
    case 106: return startPartFan(com);
    case 107: return stopPartFan();
    case 109: return waitForHotend(com);
    case 116: return waitForTemperatures();
    case 140: return setBedTemperature(com);
    case 190: return waitForBed(com);
    case 355: return setCaseLight(com);
#if (MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2) && CASE_FAN_PIN > -1
    case 3120: return turnOnCaseFan();
    case 3121: return scheduleCaseFanOff(com);
#endif
#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    case 3300: return setAccessoryFet(com);
    case 3301: return setMainsOutput(com);
#endif
#ifdef X35_ADC_CHANNEL
    case 3941: return readOptionalTemperature();
#endif

    // Bed compensation, mesh and calibration.
    case 3000: return Calibration::disableBedCompensation();
    case 3001: return Calibration::enableBedCompensation();
    case 3009: return Calibration::selectBedMatrix(com);
    case 3013: return Calibration::reportBedMatrix(com);
    case 3900:
    case 3901: return Calibration::configureZOffsetScan(com);
    case 3902: return Calibration::adjustBedMatrix(com);

    // Strain gauge and pressure limits.
    case 3075: return setEmergencyPauseDigits(com);
    case 3909: return setSenseOffsetParameters(com);
    case 3911: return setDigitFlowParameters(com);

    // Configuration, panel and diagnostics.
    case 114: return reportPosition();
    case 115: return reportFirmware();
    case PanelMessage: return setPanelMessage(com);
    case 119: return reportEndstops();
    case 205:
        return reportEeprom();
    case 206: return setEepromValue(com);
    case 360: return reportConfiguration();
    case DiagnosticHistory::ReadCommand: return DiagnosticHistory::request();
    case 502: return restoreConfigurationDefaults();
    case 3028: return testHomingRepeatability(com);
    }
}

} // namespace
} // namespace GCodeCommands

// Keep host dispatch out of the permanently active Arduino main frame.
// Typed internal programs no longer reenter this dispatch path.
__attribute__((noinline)) void Commands::executeGCode(GCode* com) {
    if(Printer::printJob.emergencyLocked())return;
    if (GCodeCommands::rejectLegacySimulation(com))
        return;
    // An unsupported input mode is not a motion mode. Discard all work, including
    // already buffered commands, except status.
    if (Printer::legacySimulationRejected()
        && (com->hasG() || !com->hasM()
            || (com->M != 105 && com->M != 114 && com->M != 115))) {
        Com::printErrorFLN(PSTR("Job rejected; stop host and restart printer."));
        return;
    }
    if(!EEPROM::settingsReady()) {
        // Retain diagnostics and explicit settings recovery; no ordinary
        // motion, heating, scan or script entry while settings are untrusted.
        if(com->hasG() || !com->hasM() ||
           (com->M!=105 && com->M!=112 && com->M!=114 &&
            com->M!=115 && com->M!=205 && com->M!=206 &&
            com->M!=502
            && com->M!=DiagnosticHistory::ReadCommand
            )) {
            Com::printErrorFLN(PSTR("EEPROM settings not acknowledged"));
            return;
        }
    }
    if (com->hasG()) {
        GCodeCommands::dispatchMotion(com);
    } else if (com->hasM()) // Process M Code
    {
        GCodeCommands::dispatchMachine(com);
    } else if (com->hasT()) // Process T code
    {
        Extruder::selectExtruderById(com->T, true);
    } else if (com->hasF()) // Process F code http://linuxcnc.org/docs/html/gcode/other-code.html#sec:set-feed-rate
    {
        Printer::setFeedrate(com->F);
    } else {
        {
            Com::printF(Com::tUnknownCommand);
            com->printCommand();
        }
    }
} // executeGCode

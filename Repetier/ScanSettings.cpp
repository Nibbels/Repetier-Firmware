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
#include "ScanProbe.h"
#include "GCodeCommands.h"

constexpr int heatBedScanStepMm = 20;
constexpr int heatBedScanContactPressureDelta = 10;
constexpr int heatBedScanRetryPressureDelta = 5;
constexpr int heatBedScanIdlePressureDelta = 0;
constexpr int heatBedScanIdlePressureMin = -7500;
constexpr int heatBedScanIdlePressureMax = 7500;
constexpr float heatBedScanUpFastMm = -0.02f;
constexpr float heatBedScanUpSlowMm = -0.004f;
constexpr float heatBedScanDownSlowMm = 0.004f;
constexpr int heatBedScanFastStepDelayMs = 1;
constexpr int heatBedScanSlowStepDelayMs = 100;
constexpr int heatBedScanIdleDelayMs = 250;
constexpr int heatBedScanPressureReads = 3;
constexpr int heatBedScanPressureTolerance = 15;
constexpr int heatBedScanPressureReadDelayMs = 15;

static ScanSettings scanSettings(defaultHeatBedScanLiftMm);

float scanStartLiftMm() { return scanSettings.startLiftMm; }
void setScanStartLiftMm(float liftMm) { scanSettings.startLiftMm = liftMm; }
long scanStrideMm(uint8_t axis) { return axis == X_AXIS ? scanSettings.x.strideMm : scanSettings.y.strideMm; }

const ScanSettings& Calibration::probeSettings() { return scanSettings; }

void restoreDefaultScanParameters(void) {
    // we must restore the default scan parameters for the heat bed scan
    scanSettings.x.startSteps = long(Printer::stepsPerMm(X_AXIS) * heatBedScanXStartMm);
    scanSettings.x.strideMm = heatBedScanStepMm;
    scanSettings.x.strideSteps = long(Printer::stepsPerMm(X_AXIS) * heatBedScanStepMm);
    scanSettings.x.endMarginSteps = long(Printer::stepsPerMm(X_AXIS) * heatBedScanXEndMm);
    scanSettings.x.maxPositionSteps = long((Printer::axisLengthMm(X_AXIS) - heatBedScanXEndMm) * Printer::stepsPerMm(X_AXIS));

    scanSettings.y.startSteps = long(Printer::stepsPerMm(Y_AXIS) * heatBedScanYStartMm);
    scanSettings.y.strideMm = heatBedScanStepMm;
    scanSettings.y.strideSteps = long(Printer::stepsPerMm(Y_AXIS) * heatBedScanStepMm);
    scanSettings.y.endMarginSteps = long(Printer::stepsPerMm(Y_AXIS) * heatBedScanYEndMm);
    scanSettings.y.maxPositionSteps = long((Printer::axisLengthMm(Y_AXIS) - heatBedScanYEndMm) * Printer::stepsPerMm(Y_AXIS));

    scanSettings.fastApproachSteps = short(Printer::stepsPerMm(Z_AXIS) * heatBedScanUpFastMm);
    scanSettings.slowApproachSteps = short(Printer::stepsPerMm(Z_AXIS) * heatBedScanUpSlowMm);
    scanSettings.fastRetreatSteps = short(Printer::stepsPerMm(Z_AXIS) * HEAT_BED_SCAN_DOWN_FAST_MM);
    scanSettings.slowRetreatSteps = short(Printer::stepsPerMm(Z_AXIS) * heatBedScanDownSlowMm);

    /* Maximum number of steps to scan after the Z-min switch has been reached. If the probe cannot find the surface
       within this travel, it retries before aborting.
       Note that the head bed scan matrix consists of 16 bit signed values, thus more then 32767 steps will lead to an overflow! */
    scanSettings.maxTravelSteps = short(Z_ENDSTOP_DRIVE_OVER * Printer::stepsPerMm(Z_AXIS));

    scanSettings.fastStepDelayMs = heatBedScanFastStepDelayMs;
    scanSettings.slowStepDelayMs = heatBedScanSlowStepDelayMs;
    scanSettings.idleDelayMs = heatBedScanIdleDelayMs;

    scanSettings.contactPressureDelta = heatBedScanContactPressureDelta;
    scanSettings.retryPressureDelta = heatBedScanRetryPressureDelta;
    scanSettings.idlePressureDelta = heatBedScanIdlePressureDelta;
    scanSettings.idlePressureMin = heatBedScanIdlePressureMin;
    scanSettings.idlePressureMax = heatBedScanIdlePressureMax;

    scanSettings.pressureReadCount = heatBedScanPressureReads;
    scanSettings.pressureTolerance = heatBedScanPressureTolerance;
    scanSettings.pressureReadDelayMs = heatBedScanPressureReadDelayMs;
} // restoreDefaultScanParameters

void outputScanParameters(void) {
    if (Printer::debugInfo()) {
        Com::printFLN(PSTR("outputScanParameters(): current scan parameters:"));

        Com::printF(PSTR(""), Printer::stepsPerMm(X_AXIS));
        Com::printFLN(PSTR(";[steps];axisStepsPerMM[X_AXIS]"));
        Com::printF(PSTR(""), Printer::stepsPerMm(Y_AXIS));
        Com::printFLN(PSTR(";[steps];axisStepsPerMM[Y_AXIS]"));
        Com::printF(PSTR(""), Printer::stepsPerMm(Z_AXIS));
        Com::printFLN(PSTR(";[steps];axisStepsPerMM[Z_AXIS]"));

        Com::printF(PSTR(""), scanSettings.x.startSteps);
        Com::printFLN(PSTR(";[steps];g_nScanXStartSteps"));
        Com::printF(PSTR(""), scanSettings.x.strideSteps);
        Com::printFLN(PSTR(";[steps];g_nScanXStepSizeSteps"));
        Com::printF(PSTR(""), scanSettings.x.endMarginSteps);
        Com::printFLN(PSTR(";[steps];g_nScanXEndSteps"));
        Com::printF(PSTR(""), scanSettings.x.maxPositionSteps);
        Com::printFLN(PSTR(";[steps];g_nScanXMaxPositionSteps"));

        Com::printF(PSTR(""), scanSettings.y.startSteps);
        Com::printFLN(PSTR(";[steps];g_nScanYStartSteps"));
        Com::printF(PSTR(""), scanSettings.y.strideSteps);
        Com::printFLN(PSTR(";[steps];g_nScanYStepSizeSteps"));
        Com::printF(PSTR(""), scanSettings.y.endMarginSteps);
        Com::printFLN(PSTR(";[steps];g_nScanYEndSteps"));
        Com::printF(PSTR(""), scanSettings.y.maxPositionSteps);
        Com::printFLN(PSTR(";[steps];g_nScanYMaxPositionSteps"));

        Com::printF(PSTR(""), (int)scanSettings.fastApproachSteps);
        Com::printFLN(PSTR(";[steps];g_nScanHeatBedUpFastSteps"));
        Com::printF(PSTR(""), (int)scanSettings.slowApproachSteps);
        Com::printFLN(PSTR(";[steps];g_nScanHeatBedUpSlowSteps"));
        Com::printF(PSTR(""), (int)scanSettings.fastRetreatSteps);
        Com::printFLN(PSTR(";[steps];g_nScanHeatBedDownFastSteps"));
        Com::printF(PSTR(""), (int)scanSettings.slowRetreatSteps);
        Com::printFLN(PSTR(";[steps];g_nScanHeatBedDownSlowSteps"));
        Com::printF(PSTR(""), (int)scanSettings.fastStepDelayMs);
        Com::printFLN(PSTR(";[ms];g_nScanFastStepDelay"));
        Com::printF(PSTR(""), (int)scanSettings.slowStepDelayMs);
        Com::printFLN(PSTR(";[ms];g_nScanSlowStepDelay"));
        Com::printF(PSTR(""), (int)scanSettings.idleDelayMs);
        Com::printFLN(PSTR(";[ms];g_nScanIdleDelay"));

        Com::printF(PSTR(""), (int)scanSettings.contactPressureDelta);
        Com::printFLN(PSTR(";[digits];g_nScanContactPressureDelta"));
        Com::printF(PSTR(""), (int)scanSettings.retryPressureDelta);
        Com::printFLN(PSTR(";[digits];g_nScanRetryPressureDelta"));
        Com::printF(PSTR(""), (int)scanSettings.idlePressureDelta);
        Com::printFLN(PSTR(";[digits];g_nScanIdlePressureDelta"));

        Com::printF(PSTR(""), (int)scanSettings.pressureReadCount);
        Com::printFLN(PSTR(";[-];g_nScanPressureReads"));
        Com::printF(PSTR(""), (int)scanSettings.pressureTolerance);
        Com::printFLN(PSTR(";[digits];g_nScanPressureTolerance"));
        Com::printF(PSTR(""), (int)scanSettings.pressureReadDelayMs);
        Com::printFLN(PSTR(";[ms];g_nScanPressureReadDelay"));
    }
} // outputScanParameters

namespace GCodeCommands {
namespace Calibration {

void setScanXStart(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 5)
            nTemp = 5;
        if (nTemp > (Printer::axisLengthMm(X_AXIS) - 5))
            nTemp = Printer::axisLengthMm(X_AXIS) - 5;

        scanSettings.x.startSteps = (long)((float)nTemp * Printer::stepsPerMm(X_AXIS));
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3020: new x start position: "), nTemp);
            Com::printF(PSTR(" [mm], "), (int)scanSettings.x.startSteps);
            Com::printFLN(PSTR(" [steps]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setScanYStart(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 5)
            nTemp = 5;
        if (nTemp > (Printer::axisLengthMm(Y_AXIS) - 5))
            nTemp = Printer::axisLengthMm(Y_AXIS) - 5;

        scanSettings.y.startSteps = (long)((float)nTemp * Printer::stepsPerMm(Y_AXIS));
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3021: new y start position: "), nTemp);
            Com::printF(PSTR(" [mm], "), (int)scanSettings.y.startSteps);
            Com::printFLN(PSTR(" [steps]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setScanXStep(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < heatBedScanMinStepMm)
            nTemp = heatBedScanMinStepMm;
        if (nTemp > 100)
            nTemp = 100;

        scanSettings.x.strideMm = nTemp;
        scanSettings.x.strideSteps = (long)((float)nTemp * Printer::stepsPerMm(X_AXIS));
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3022: new x step size: "), (int)nTemp);
            Com::printF(PSTR(" [mm], "), (int)scanSettings.x.strideSteps);
            Com::printFLN(PSTR(" [steps]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setScanYStep(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < heatBedScanMinStepMm)
            nTemp = heatBedScanMinStepMm;
        if (nTemp > 100)
            nTemp = 100;

        scanSettings.y.strideMm = nTemp;
        scanSettings.y.strideSteps = (long)((float)nTemp * Printer::stepsPerMm(Y_AXIS));
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3023: new y step size: "), (int)nTemp);
            Com::printF(PSTR(" [mm], "), (int)scanSettings.y.strideSteps);
            Com::printFLN(PSTR(" [steps]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setScanXEnd(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 5)
            nTemp = 5;
        if (nTemp > (Printer::axisLengthMm(X_AXIS) - 5))
            nTemp = Printer::axisLengthMm(X_AXIS) - 5;

        scanSettings.x.endMarginSteps = (long)((float)nTemp * Printer::stepsPerMm(X_AXIS));
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3024: new x end position: "), nTemp);
            Com::printF(PSTR(" [mm], "), (int)scanSettings.x.endMarginSteps);
            Com::printFLN(PSTR(" [steps]"));
        }

        scanSettings.x.maxPositionSteps = long(Printer::axisLengthMm(X_AXIS) * Printer::stepsPerMm(X_AXIS) - scanSettings.x.endMarginSteps);
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3024: new x max position: "), (int)scanSettings.x.maxPositionSteps);
            Com::printFLN(PSTR(" [steps]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setScanYEnd(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 5)
            nTemp = 5;
        if (nTemp > (Printer::axisLengthMm(Y_AXIS) - 5))
            nTemp = Printer::axisLengthMm(Y_AXIS) - 5;

        scanSettings.y.endMarginSteps = (long)((float)nTemp * Printer::stepsPerMm(Y_AXIS));
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3025: new y end position: "), nTemp);
            Com::printF(PSTR(" [mm], "), (int)scanSettings.y.endMarginSteps);
            Com::printFLN(PSTR(" [steps]"));
        }

        scanSettings.y.maxPositionSteps = long(Printer::axisLengthMm(Y_AXIS) * Printer::stepsPerMm(Y_AXIS) - scanSettings.y.endMarginSteps);
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3025: new y max position: "), (int)scanSettings.y.maxPositionSteps);
            Com::printFLN(PSTR(" [steps]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setFastApproachStep(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp > 50)
            nTemp = 50;
        if (nTemp < 1)
            nTemp = 1;

        scanSettings.fastApproachSteps = -nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3030: new fast step size for moving of the bed up: "), (int)scanSettings.fastApproachSteps);
            Com::printFLN(PSTR(" [steps]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setSlowApproachStep(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp > 50)
            nTemp = 50;
        if (nTemp < 1)
            nTemp = 1;

        scanSettings.slowApproachSteps = -nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3031: new slow step size for moving of the bed up: "), (int)scanSettings.slowApproachSteps);
            Com::printFLN(PSTR(" [steps]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setFastRetreatStep(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < Printer::stepsPerMm(Z_AXIS) / 20)
            nTemp = Printer::stepsPerMm(Z_AXIS) / 20;
        if (nTemp > Printer::stepsPerMm(Z_AXIS) * 5)
            nTemp = Printer::stepsPerMm(Z_AXIS) * 5;

        scanSettings.fastRetreatSteps = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3032: new fast step size for moving of the bed down: "), (int)scanSettings.fastRetreatSteps);
            Com::printFLN(PSTR(" [steps]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setSlowRetreatStep(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 1)
            nTemp = 1;
        if (nTemp > 50)
            nTemp = 50;

        scanSettings.slowRetreatSteps = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3033: new slow step size for moving of the bed down: "), (int)scanSettings.slowRetreatSteps);
            Com::printFLN(PSTR(" [steps]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setFastScanDelay(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 1)
            nTemp = 1;
        if (nTemp > 1000)
            nTemp = 1000;

        scanSettings.fastStepDelayMs = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3040: new delay between two fast movements: "), (int)scanSettings.fastStepDelayMs);
            Com::printFLN(PSTR(" [ms]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setSlowScanDelay(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 1)
            nTemp = 1;
        if (nTemp > 1000)
            nTemp = 1000;

        scanSettings.slowStepDelayMs = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3041: new delay between two slow movements: "), (int)scanSettings.slowStepDelayMs);
            Com::printFLN(PSTR(" [ms]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setScanIdleDelay(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 1)
            nTemp = 1;
        if (nTemp > 10000)
            nTemp = 10000;

        scanSettings.idleDelayMs = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3042: new idle delay: "), (int)scanSettings.idleDelayMs);
            Com::printFLN(PSTR(" [ms]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setContactPressureDelta(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 1)
            nTemp = 1;
        if (nTemp > 1000)
            nTemp = 1000;

        scanSettings.contactPressureDelta = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3050: new contact pressure delta: "), (int)scanSettings.contactPressureDelta);
            Com::printFLN(PSTR(" [digits]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setRetryPressureDelta(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 1)
            nTemp = 1;
        if (nTemp > 1000)
            nTemp = 1000;

        scanSettings.retryPressureDelta = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3051: new retry pressure delta: "), (int)scanSettings.retryPressureDelta);
            Com::printFLN(PSTR(" [digits]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setIdlePressureTolerance(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 1)
            nTemp = 1;
        if (nTemp > 1000)
            nTemp = 1000;

        scanSettings.idlePressureDelta = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3052: new idle pressure delta: "), (int)scanSettings.idlePressureDelta);
            Com::printFLN(PSTR(" [digits]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setPressureReadCount(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 1)
            nTemp = 1;
        if (nTemp > 100)
            nTemp = 100;

        scanSettings.pressureReadCount = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3053: new pressure reads per measurement: "), (int)scanSettings.pressureReadCount);
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setPressureReadDelay(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 1)
            nTemp = 1;
        if (nTemp > 1000)
            nTemp = 1000;

        scanSettings.pressureReadDelayMs = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3054: new delay between two pressure reads: "), (int)scanSettings.pressureReadDelayMs);
            Com::printFLN(PSTR(" [ms]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setPressureReadTolerance(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 1)
            nTemp = 1;
        if (nTemp > 1000)
            nTemp = 1000;

        scanSettings.pressureTolerance = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3055: new scan pressure tolerance: "), (int)scanSettings.pressureTolerance);
            Com::printFLN(PSTR(" [digits]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

} // namespace Calibration
} // namespace GCodeCommands

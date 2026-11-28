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

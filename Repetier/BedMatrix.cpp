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
#include "BedCompensation.h"
#include "GCodeInput.h"
#include "GCodeCommands.h"
#include "ScanProbe.h"
#include "ExternalEeprom.h"

static long g_offsetZCompensationSteps = 0;
static short g_ZCompensationMax = 0;
static char g_nActiveHeatBed = 1;

// g_ZMatrixChangedInRam soll 1 werden, wenn ZOS, Offsetänderung der Matrix etc. Sonst wäre Sichern der Matrix unnötig.
static volatile unsigned char g_ZMatrixChangedInRam = 0;

static short g_ZCompensationMatrix[COMPENSATION_MATRIX_MAX_X][COMPENSATION_MATRIX_MAX_Y];
static unsigned char g_uZMatrixMax[2] = { 0, 0 };

uint8_t activeBedMatrixSlot() { return g_nActiveHeatBed; }
bool bedMatrixAvailable() { return g_ZCompensationMatrix[0][0] == EEPROM_FORMAT; }
bool bedMatrixHasUnsavedChanges() { return g_ZMatrixChangedInRam; }
long bedMatrixHighestZSteps() { return g_offsetZCompensationSteps; }
short bedMatrixLowestZSteps() { return g_ZCompensationMax; }
void invalidateBedMatrixForReload() { g_ZCompensationMatrix[0][0] = EEPROM_FORMAT - 1; }

static void initCompensationMatrix();
static void outputCompensationMatrix(char format = 0);
static char prepareCompensationMatrix();
static void determineCompensationOffsetZ();
static void adjustCompensationMatrix(short steps);
static char loadCompensationMatrix(unsigned int address, void (*progress)() = nullptr);
static bool saveWorkingMatrix(unsigned int address);

#include "BedMatrixInterpolation.h"

long getZMatrixDepth_CurrentXY(void) {
    // We need the current queue steps here. No Direct, No Offsets.
    // We need to insert where the nozzle is, relative to 0 homing coordinates which are always min endstop because we dropped axis offsets.
    // Because our RF2000 Matrix is 180mm width. That is the center of the bed. left and right are spaces. -> Center buildarea starts at 0 gcode coordinates and thats what we want.
    return getZMatrixDepth(
        Printer::xyMotion.nominalSteps(X_AXIS),
        Printer::xyMotion.nominalSteps(Y_AXIS));
}

void recalculateZCompensation(void) {
    BedCompensation::Result result{0, 0.0f};
    if (Printer::doHeatBedZCompensation) {
        const auto& layers = Printer::layers;
        const BedCompensation::Geometry geometry{
            Printer::zMotion.nominalSteps(),
            layers.fadeStartSteps(),
            layers.fadeEndSteps(),
            layers.previousLayerSteps(),
            layers.currentLayerSteps(),
            g_offsetZCompensationSteps,
        };
        result = BedCompensation::calculate(geometry, [] { return getZMatrixDepth_CurrentXY(); });
        result.zCorrectionSteps += Printer::zOffset.effectiveSteps();
        if (Printer::digitCompensation.digitZEnabled())
            result.zCorrectionSteps += BedCompensation::digitZSteps(Printer::digitCompensation.filteredDigits(), Printer::stepsPerMm(Z_AXIS));
    }

    const float ratio = ExtrusionCompensation::prepareRatio(result.extraExtrusionPerStep);
    // Finish quantization before disabling step interrupts.
    asm volatile("" : : "g"(ratio));
    InterruptProtectedBlock lock;
    Printer::zMotion.requestCorrection(result.zCorrectionSteps);
    Printer::extrusionCompensation.publishRatio(ratio);
}

static void initCompensationMatrix(void) {
    // clear all fields of the compensation matrix
    memset(g_ZCompensationMatrix, 0, sizeof(g_ZCompensationMatrix));
    g_uZMatrixMax[X_AXIS] = g_uZMatrixMax[Y_AXIS] = 0;
    g_offsetZCompensationSteps = 0;
    g_ZCompensationMax = 0;
    g_ZMatrixChangedInRam = 0;
} // initCompensationMatrix

#include "BedMatrixGrid.h"

#include "BedMatrixStorage.h"

#include "BedMatrixDiagnostics.h"

namespace BedMatrix {

void restoreAtStartup(void (*progress)()) {
    loadActiveHeatBedFromExtEEPROM();
    if (!bedMatrixAvailable())
        prepareZCompensationWithProgress(progress);
}

void clearWorking() { initCompensationMatrix(); }
bool invalidateAllStored(void (*progress)()) {
    Printer::disableCMPnow(true);
    initCompensationMatrix();
    return invalidateStoredMatrixMarkers(progress);
}
bool reloadActive() { return loadCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed)) == 0; }
void beginScanGrid() { g_ZCompensationMatrix[0][0] = EEPROM_FORMAT; }
void recordScanColumn(uint8_t x, short positionMm) { g_ZCompensationMatrix[x][0] = positionMm; }

void recordScanPoint(uint8_t x, uint8_t y, short positionMm, short zSteps) {
    g_ZCompensationMatrix[x][y] = zSteps;
    g_ZCompensationMatrix[0][y] = positionMm;
    if (x > g_uZMatrixMax[X_AXIS])
        g_uZMatrixMax[X_AXIS] = x;
    if (y > g_uZMatrixMax[Y_AXIS])
        g_uZMatrixMax[Y_AXIS] = y;
}

uint8_t maxIndex(uint8_t axis) { return g_uZMatrixMax[axis]; }

uint8_t storedMaxIndex(uint8_t axis) {
    const unsigned int dimension = axis == X_AXIS ? EEPROM_OFFSET_DIMENSION_X : EEPROM_OFFSET_DIMENSION_Y;
    const unsigned int address = EEPROM_SECTOR_SIZE * g_nActiveHeatBed + dimension;
    return (uint8_t)ExternalEeprom::readWord(I2C_ADDRESS_EXTERNAL_EEPROM, address);
}

short entry(uint8_t x, uint8_t y) { return g_ZCompensationMatrix[x][y]; }
char prepareWorking() { return prepareCompensationMatrix(); }
void updateExtents() { determineCompensationOffsetZ(); }
void report(char format) { outputCompensationMatrix(format); }
void applyThermalOffset(short zSteps) { adjustCompensationMatrix(zSteps); }

bool saveScannedGrid() {
    g_ZMatrixChangedInRam = 1;
    return saveWorkingMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed));
}

ZOffsetResult applyZOffset(long nZ, ZOffsetScanPoint point, float gradient, long maximumHeight) {
    ZOffsetResult result{};
    // Nibbels: weight change because of distance. lerne bettwinkelausgleich.
    const int xIntervals = RMath::max(1, g_uZMatrixMax[X_AXIS] - 2);
    const int yIntervals = RMath::max(1, g_uZMatrixMax[Y_AXIS] - 2);
    const float x_bed_len_quadrat = xIntervals * xIntervals;
    const float y_bed_len_quadrat = yIntervals * yIntervals;
    float x_dist = 0;
    float y_dist = 0;
    float xy_weight = 0;
    long weighted_nZ = 0;
    long newValue = 0;

    for (short x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) {
        for (short y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
            x_dist = (point.x - x) * (point.x - x) / x_bed_len_quadrat; // Normierter indexabstand
            y_dist = (point.y - y) * (point.y - y) / y_bed_len_quadrat; // Normierter indexabstand
            x_dist = sqrt(x_dist);                                                                    //Linearisieren. für Auto-Matrix-Leveling
            y_dist = sqrt(y_dist);                                                                    //Linearisieren. für Auto-Matrix-Leveling
            // Das ist nur ein kreisabstand, wenn die messpunkte quadratisch angeordnet sind, ist aber nicht so?
            // evtl. todo: achse faktor skalieren, sodass kreis x/y=(10/13)
            xy_weight = 1 - sqrt(x_dist * x_dist + y_dist * y_dist); // Linear gewichtet
            if (xy_weight < 0.0)
                xy_weight = 0;
            if (xy_weight > 1.0)
                xy_weight = 1.0; // Kann aber nicht wirklich vorkommen.
            weighted_nZ = (long)(gradient * xy_weight * (float)nZ + (1.0 - gradient) * (float)nZ);
            newValue = (long)g_ZCompensationMatrix[x][y] + weighted_nZ;
            if (newValue > 32767 || newValue < -32768)
                result.overflow = true;
            if (newValue > 0)
                result.aboveZero = true; // Darf nicht positiv werden. //darf nicht über limit sein.
            if (newValue > maximumHeight)
                result.tooHigh = true; // Darf nicht positiv werden. //darf nicht über limit sein.
            g_ZCompensationMatrix[x][y] = (short)newValue;
        }
    }
    return result;
}

void finishZOffset() {
    determineCompensationOffsetZ();
    g_ZMatrixChangedInRam = 1;
}

} // namespace BedMatrix

#include "BedMatrixCommands.h"

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
static char loadCompensationMatrix(unsigned int address);
static bool saveWorkingMatrix(unsigned int address);

namespace {
struct MatrixSpan {
    uint8_t low, high;
    long distance, width;

    long depth(long lowDepth, long highDepth) const {
        return width ? lowDepth + (highDepth - lowDepth) * distance / width : lowDepth;
    }
};

MatrixSpan matrixSpan(uint8_t axis, long position) {
    const float stepsPerMm = Printer::stepsPerMm(axis);
    const auto coordinate = [axis, stepsPerMm](uint8_t index) {
        const short mm = axis == X_AXIS ? g_ZCompensationMatrix[index][0] : g_ZCompensationMatrix[0][index];
        return long(mm * stepsPerMm);
    };

    uint8_t low = 1;
    long start = coordinate(low);
    if (position > start) {
        for (uint8_t high = 2; high <= g_uZMatrixMax[axis]; ++high) {
            const long end = coordinate(high);
            if (position <= end)
                return {low, high, position - start, end - start};
            low = high;
            start = end;
        }
    }
    return {low, low, 0, 0};
}
} // namespace

long getZMatrixDepth(long x, long y) {
    const auto column = matrixSpan(X_AXIS, x);
    const auto row = matrixSpan(Y_AXIS, y);
    const long front = column.depth(g_ZCompensationMatrix[column.low][row.low], g_ZCompensationMatrix[column.high][row.low]);
    const long back = column.depth(g_ZCompensationMatrix[column.low][row.high], g_ZCompensationMatrix[column.high][row.high]);
    return row.depth(front, back);
}

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

static void outputCompensationMatrix(char format) {
    short x;
    short y;

    Com::printFLN(PSTR("front left ... front right"));
    Com::printFLN(PSTR("...        ...         ..."));
    Com::printFLN(PSTR("back left  ...  back right"));

    for (y = 0; y <= g_uZMatrixMax[Y_AXIS]; y++) {
        for (x = 0; x <= g_uZMatrixMax[X_AXIS]; x++) {
            if (x == 0 || y == 0) {
                Com::printF(Com::tSemiColon, g_ZCompensationMatrix[x][y]);
            } else {
                if (format) {
                    // output in [mm]
                    Com::printF(Com::tSemiColon, g_ZCompensationMatrix[x][y] / Printer::stepsPerMm(Z_AXIS));
                } else {
                    // output in [steps]
                    Com::printF(Com::tSemiColon, g_ZCompensationMatrix[x][y]);
                }
            }
        }
        Com::printFLN(PSTR(" "));
    }

    Com::printF(PSTR("offset = "), g_offsetZCompensationSteps);
    Com::printF(PSTR(" [steps] (= "), (float)g_offsetZCompensationSteps * Printer::mmPerStep(Z_AXIS));
    Com::printFLN(PSTR(" [mm])"));

    Com::printF(PSTR("warpage = "), g_ZCompensationMax - g_offsetZCompensationSteps);
    Com::printF(PSTR(" [steps] (= "), float(g_ZCompensationMax - g_offsetZCompensationSteps) * Printer::mmPerStep(Z_AXIS));
    Com::printFLN(PSTR(" [mm])"));

    Com::printFLN(PSTR("g_uZMatrixMax[X_AXIS] = "), g_uZMatrixMax[X_AXIS]);
    Com::printFLN(PSTR("g_uZMatrixMax[Y_AXIS] = "), g_uZMatrixMax[Y_AXIS]);

    {
        Com::printFLN(PSTR("g_nActiveHeatBed = "), g_nActiveHeatBed);
    }

} // outputCompensationMatrix

static char prepareCompensationMatrix(void) {
    short x;
    short y;

    // perform some safety checks first
    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("zCMP: invalid version: "), g_ZCompensationMatrix[0][0]);
            Com::printF(PSTR(" (expected: "), EEPROM_FORMAT);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }

    if (g_uZMatrixMax[X_AXIS] >= COMPENSATION_MATRIX_MAX_X || g_uZMatrixMax[X_AXIS] < 2) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("zCMP: invalid x dimension: "), g_uZMatrixMax[X_AXIS]);
            Com::printF(PSTR(" (max expected: "), COMPENSATION_MATRIX_MAX_X - 1);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }

    if (g_uZMatrixMax[Y_AXIS] >= COMPENSATION_MATRIX_MAX_Y || g_uZMatrixMax[Y_AXIS] < 2) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("zCMP: invalid y dimension: "), g_uZMatrixMax[Y_AXIS]);
            Com::printF(PSTR(" (max expected: "), COMPENSATION_MATRIX_MAX_Y - 1);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }

    const bool padLeft = g_ZCompensationMatrix[2][0] > 0;
    const bool padFront = g_ZCompensationMatrix[0][2] > 0;
    const bool padRight = g_ZCompensationMatrix[g_uZMatrixMax[X_AXIS]][0] < short(Printer::axisLengthMm(X_AXIS));
    const bool padBack = g_ZCompensationMatrix[0][g_uZMatrixMax[Y_AXIS]] < short(Printer::axisLengthMm(Y_AXIS));
    const int lastX = g_uZMatrixMax[X_AXIS] - !padLeft + padRight;
    const int lastY = g_uZMatrixMax[Y_AXIS] - !padFront + padBack;
    if (lastX < 2 || lastX >= COMPENSATION_MATRIX_MAX_X || lastY < 2 || lastY >= COMPENSATION_MATRIX_MAX_Y) {
        Com::printFLN(PSTR("zCMP: prepared matrix dimensions out of range"));
        return -1;
    }

    if (padLeft) {
        // we have to fill x[1] with the values of x[2]
        /*      if( Printer::debugInfo() )
        {
            Com::printFLN( PSTR( "prepareCompensationMatrix(): x[2] > 0" ) );
        }
*/
        g_ZCompensationMatrix[1][0] = 0;
        for (y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
            g_ZCompensationMatrix[1][y] = g_ZCompensationMatrix[2][y];
        }
    } else {
        // we have to shift all x columns one index
        for (x = 1; x < g_uZMatrixMax[X_AXIS]; x++) {
            for (y = 0; y <= g_uZMatrixMax[Y_AXIS]; y++) {
                g_ZCompensationMatrix[x][y] = g_ZCompensationMatrix[x + 1][y];
            }
        }

        // we have one x column less now
        g_uZMatrixMax[X_AXIS]--;
    }

    if (padRight) {
        // we have to fill x[g_uZMatrixMax[X_AXIS]] with the values of x[g_uZMatrixMax[X_AXIS]-1]
        g_ZCompensationMatrix[g_uZMatrixMax[X_AXIS] + 1][0] = short(Printer::axisLengthMm(X_AXIS));
        for (y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
            g_ZCompensationMatrix[g_uZMatrixMax[X_AXIS] + 1][y] = g_ZCompensationMatrix[g_uZMatrixMax[X_AXIS]][y];
        }

        // we have one x column more now
        g_uZMatrixMax[X_AXIS]++;
    } else {
        // there is nothing else to do here
    }

    if (padFront) {
        // we have to fill y[1] with the values of y[2]
        g_ZCompensationMatrix[0][1] = 0;
        for (x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) {
            g_ZCompensationMatrix[x][1] = g_ZCompensationMatrix[x][2];
        }
    } else {
        // we have to shift all y columns one index
        for (x = 0; x <= g_uZMatrixMax[X_AXIS]; x++) {
            for (y = 1; y < g_uZMatrixMax[Y_AXIS]; y++) {
                g_ZCompensationMatrix[x][y] = g_ZCompensationMatrix[x][y + 1];
            }
        }

        // we have one y column less now
        g_uZMatrixMax[Y_AXIS]--;
    }

    if (padBack) {
        // we have to fill y[g_uZMatrixMax[Y_AXIS]] with the values of y[g_uZMatrixMax[Y_AXIS]-1]
        g_ZCompensationMatrix[0][g_uZMatrixMax[Y_AXIS] + 1] = short(Printer::axisLengthMm(Y_AXIS));
        for (x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) {
            g_ZCompensationMatrix[x][g_uZMatrixMax[Y_AXIS] + 1] = g_ZCompensationMatrix[x][g_uZMatrixMax[Y_AXIS]];
        }

        // we have one y column more now
        g_uZMatrixMax[Y_AXIS]++;
    }

    // determine the minimal distance between extruder and heat bed
    determineCompensationOffsetZ();

    return 0;
} // prepareCompensationMatrix

static void determineCompensationOffsetZ(void) {
    short x;
    short y;
    short uMax = -32768;
    short uMin = 32767;

    for (x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) {
        for (y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
            if (g_ZCompensationMatrix[x][y] > uMax) {
                uMax = g_ZCompensationMatrix[x][y];
            }
            if (g_ZCompensationMatrix[x][y] < uMin) {
                uMin = g_ZCompensationMatrix[x][y];
            }
        }
    }
    g_ZCompensationMax = uMin;
    g_offsetZCompensationSteps = uMax;
} // determineCompensationOffsetZ

static void adjustCompensationMatrix(short nZ) {
    short x;
    short y;
    short nOffset = getZMatrixDepth_CurrentXY();
    short deltaZ = nZ - nOffset;

    for (x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) {
        for (y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
            g_ZCompensationMatrix[x][y] += deltaZ;
        }
    }

    // determine the minimal distance between extruder and heat bed
    determineCompensationOffsetZ();
} // adjustCompensationMatrix

static bool writeMatrixSector(unsigned int address, bool clear) {
    const uint8_t eeprom = I2C_ADDRESS_EXTERNAL_EEPROM;
    const unsigned int formatAddress = address + EEPROM_OFFSET_SECTOR_FORMAT;
    const unsigned short format = clear ? 0 : EEPROM_FORMAT;
    const uint8_t lastX = clear ? COMPENSATION_MATRIX_MAX_X - 1 : g_uZMatrixMax[X_AXIS];
    const uint8_t lastY = clear ? COMPENSATION_MATRIX_MAX_Y - 1 : g_uZMatrixMax[Y_AXIS];
    const unsigned short microsteps = clear ? 0 : drv8711ModeValue_2_MicroSteps(Printer::motorMicroStepsModeValue[Z_AXIS]);

    // Invalidate the sector before replacing any payload. A reset mid-save
    // must leave this sector unavailable to the loader.
    unsigned short storedFormat;
    if (!ExternalEeprom::writeWord(eeprom, formatAddress, 0)
        || !ExternalEeprom::readWord(eeprom, formatAddress, storedFormat) || storedFormat != 0
        || !ExternalEeprom::writeWord(eeprom, EEPROM_OFFSET_HEADER_FORMAT, EEPROM_FORMAT)
        || !ExternalEeprom::writeWord(eeprom, address + EEPROM_OFFSET_DIMENSION_X, clear ? 0 : lastX)
        || !ExternalEeprom::writeWord(eeprom, address + EEPROM_OFFSET_DIMENSION_Y, clear ? 0 : lastY)
        || !ExternalEeprom::writeWord(eeprom, address + EEPROM_OFFSET_MICRO_STEPS, microsteps)) {
        return false;
    }

    unsigned int offset = address + EEPROM_OFFSET_MATRIX_START;
    for (uint8_t x = 0; x <= lastX; ++x) {
        for (uint8_t y = 0; y <= lastY; ++y) {
            if (!ExternalEeprom::writeWord(eeprom, offset, clear ? 0 : g_ZCompensationMatrix[x][y]))
                return false;
            offset += 2;
        }
        GCodeInput::keepAlive(Processing);
    }

    // A read after the marker write also waits for the last EEPROM write cycle.
    return (clear || ExternalEeprom::writeWord(eeprom, formatAddress, format))
        && ExternalEeprom::readWord(eeprom, formatAddress, storedFormat) && storedFormat == format;
}

static bool saveWorkingMatrix(unsigned int address) {
    if (!address || g_ZCompensationMatrix[0][0] != EEPROM_FORMAT
        || g_uZMatrixMax[X_AXIS] < 2 || g_uZMatrixMax[X_AXIS] >= COMPENSATION_MATRIX_MAX_X
        || g_uZMatrixMax[Y_AXIS] < 2 || g_uZMatrixMax[Y_AXIS] >= COMPENSATION_MATRIX_MAX_Y) {
        Com::printFLN(PSTR("saveMatrix(): invalid matrix"));
        return false;
    }
    if (!writeMatrixSector(address, false)) {
        Com::printFLN(PSTR("saveMatrix(): EEPROM write failed"));
        return false;
    }

    determineCompensationOffsetZ();
    if (address == static_cast<unsigned int>(EEPROM_SECTOR_SIZE * g_nActiveHeatBed))
        g_ZMatrixChangedInRam = 0;
    return true;
}

static bool saveCompensationMatrix(unsigned int address) {
    if (isAnyScanRunning()) {
        Com::printFLN(PSTR("saveMatrix(): scan in progress"));
        return false;
    }
    return saveWorkingMatrix(address);
}

bool saveActiveBedMatrix() {
    return saveCompensationMatrix(static_cast<unsigned int>(EEPROM_SECTOR_SIZE * g_nActiveHeatBed));
}

static bool saveActiveHeatBedToExtEEPROM(char newActiveHeatBed) {
    if (newActiveHeatBed < 1 || newActiveHeatBed > EEPROM_MAX_HEAT_BED_SECTORS) {
        return false;
    }

    const uint8_t device = I2C_ADDRESS_EXTERNAL_EEPROM;
    const unsigned int address = EEPROM_OFFSET_ACTIVE_HEAT_BED_Z_MATRIX;
    unsigned short stored;
    if (ExternalEeprom::readWord(device, address, stored) && stored == static_cast<unsigned short>(newActiveHeatBed))
        return true;
    if (ExternalEeprom::writeWord(device, address, newActiveHeatBed)
        && ExternalEeprom::readWord(device, address, stored) && stored == static_cast<unsigned short>(newActiveHeatBed))
        return true;
    Com::printFLN(PSTR("selectMatrix(): EEPROM write failed"));
    return false;
}

static bool loadActiveHeatBedFromExtEEPROM() {
    const unsigned short slot = ExternalEeprom::readWord(I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_OFFSET_ACTIVE_HEAT_BED_Z_MATRIX);

    if (slot < 1 || slot > EEPROM_MAX_HEAT_BED_SECTORS) {
        Com::printFLN(PSTR("Invalid active heat bed within ext. EEPROM detected: "), uint32_t(slot));
        if (g_nActiveHeatBed < 1 || g_nActiveHeatBed > EEPROM_MAX_HEAT_BED_SECTORS) {
            g_nActiveHeatBed = 1;
        }
        saveActiveHeatBedToExtEEPROM(g_nActiveHeatBed);

        return false;
    }

    g_nActiveHeatBed = slot;
    return true;
}

static char loadCompensationMatrix(unsigned int uAddress) {
    if (uAddress == 0) {
        Com::printFLN(PSTR("loadMatrix(): valid uAddress - aborted!"));

        return -1;
    }

    unsigned short uTemp;
    unsigned short uDimensionX;
    unsigned short uDimensionY;
    unsigned short uMicroSteps;
    unsigned int uOffset;
    short x;
    short y;
    float fMicroStepCorrection;

    Printer::disableCMPnow(true); // vorher, nicht nachher ausschalten, sonst arbeitet unter Umständen die alte matrix.
    // The payload is read into the working buffer. Publish the format marker
    // and dimensions only after the whole sector has been accepted.
    g_ZCompensationMatrix[0][0] = 0;

    // check the stored header format
    if (!ExternalEeprom::readWord(I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_OFFSET_HEADER_FORMAT, uTemp))
        return -1;
    if (uTemp != EEPROM_FORMAT) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("loadMatrix(): invalid header format: "), (int)uTemp);
            Com::printF(PSTR(" (expected: "), EEPROM_FORMAT);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }


    // check the stored sector format
    if (!ExternalEeprom::readWord(I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_SECTOR_FORMAT, uTemp))
        return -1;
    if (uTemp != EEPROM_FORMAT) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("loadMatrix(): invalid sector format: "), (int)uTemp);
            Com::printF(PSTR(" (expected: "), EEPROM_FORMAT);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }

    // check the stored x dimension
    if (!ExternalEeprom::readWord(I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_DIMENSION_X, uDimensionX))
        return -1;
    if (uDimensionX < 2 || uDimensionX >= COMPENSATION_MATRIX_MAX_X) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("loadMatrix(): invalid x dimension: "), (int)uDimensionX);
            Com::printF(PSTR(" (max expected: "), COMPENSATION_MATRIX_MAX_X - 1);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }

    // check the stored y dimension
    if (!ExternalEeprom::readWord(I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_DIMENSION_Y, uDimensionY))
        return -1;
    if (uDimensionY < 2 || uDimensionY >= COMPENSATION_MATRIX_MAX_Y) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("loadMatrix(): invalid y dimension: "), (int)uDimensionY);
            Com::printF(PSTR(" (max expected: "), COMPENSATION_MATRIX_MAX_Y - 1);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }

    // check the stored microsteps
    if (!ExternalEeprom::readWord(I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_MICRO_STEPS, uMicroSteps))
        return -1;
    if (!uMicroSteps || uMicroSteps > 256 || (uMicroSteps & (uMicroSteps - 1))) {
        Com::printFLN(PSTR("loadMatrix(): invalid microstep count: "), uint32_t(uMicroSteps));
        return -1;
    }

    if (uMicroSteps ==
        drv8711ModeValue_2_MicroSteps(Printer::motorMicroStepsModeValue[Z_AXIS])
    ) {
        // the current z-compensation matrix has been determined with the current micro step setting, there is nothing to recalculate
        fMicroStepCorrection = 1.0;
    } else {
        // the current z-compensation matrix has been determined with a higher than the current micro step setting, we must divide all z-correction values
        fMicroStepCorrection = (float)
                                   drv8711ModeValue_2_MicroSteps(Printer::motorMicroStepsModeValue[Z_AXIS])
            / (float)uMicroSteps;

        Com::printF(PSTR("loadMatrix(): micro step correction = "), fMicroStepCorrection);
        Com::printF(PSTR(" (stored="), (int)uMicroSteps);
        Com::printF(PSTR(", current="), (int)
                                            drv8711ModeValue_2_MicroSteps(Printer::motorMicroStepsModeValue[Z_AXIS])
        );
        Com::printFLN(PSTR(")"));
    }

    // read out the actual compensation values
    uOffset = uAddress + EEPROM_OFFSET_MATRIX_START;
    for (x = 0; x <= uDimensionX; x++) {
        for (y = 0; y <= uDimensionY; y++) {
            if (!ExternalEeprom::readWord(I2C_ADDRESS_EXTERNAL_EEPROM, uOffset, uTemp)) {
                Com::printFLN(PSTR("loadMatrix(): EEPROM read failed"));
                return -1;
            }
            const int16_t value = static_cast<int16_t>(uTemp);
            if (x == 0 && y == 0) {
                if (value != EEPROM_FORMAT) {
                    Com::printFLN(PSTR("loadMatrix(): invalid matrix format: "), value);
                    return -1;
                }
            } else if (x == 0 || y == 0) {
                // we must not modify our header row/column
                g_ZCompensationMatrix[x][y] = value;
            } else {
                // we may have to update all z-compensation values
                long transformedMatrixElement = long((float)value * fMicroStepCorrection);
                if (transformedMatrixElement > 32767 || transformedMatrixElement < -32768) {
                    Com::printFLN(PSTR("MicroStep transf.: overflow"));

                    return -1;
                }
                g_ZCompensationMatrix[x][y] = short(transformedMatrixElement);
            }
            uOffset += 2;
        }
        GCodeInput::keepAlive(Processing);
    }

    g_uZMatrixMax[X_AXIS] = uDimensionX;
    g_uZMatrixMax[Y_AXIS] = uDimensionY;
    determineCompensationOffsetZ();
    g_ZCompensationMatrix[0][0] = EEPROM_FORMAT;
    g_ZMatrixChangedInRam = 0; //Nibbels: Marker, dass die Matrix gespeichert werden kann oder eben nicht, weils unverändert keinen Sinn macht.

    return 0;
} // loadCompensationMatrix

static bool clearCompensationMatrix(unsigned int address) {
    // The caller has selected this sector. Invalidate the RAM copy even if the
    // sector could only be partially cleared.
    initCompensationMatrix();
    if (!writeMatrixSector(address, true)) {
        Com::printFLN(PSTR("clearMatrix(): EEPROM write failed"));
        return false;
    }
    if (Printer::debugInfo())
        Com::printFLN(PSTR("clearMatrix(): matrix cleared"));
    return true;
}

static void clearExternalEEPROM(void) {
    unsigned short i;
    unsigned short uMax = 32768;
    unsigned short uTemp;
    unsigned short uLast = 0;

    if (Printer::debugInfo()) {
        Com::printFLN(PSTR("clearExtEEPROM(): erasing chip memory ..."));
    }

    // the external EEPROM is able to store 262.144 kBit (= 32.768 kByte)
    for (i = 0; i < uMax; i++) {
        if (!ExternalEeprom::writeByte(I2C_ADDRESS_EXTERNAL_EEPROM, i, 0)) {
            Com::printFLN(PSTR("clearExtEEPROM(): EEPROM write failed"));
            return;
        }
        Commands::checkForPeriodicalActions(Processing);

        if (Printer::debugInfo()) {
            uTemp = i / 100;
            if (uTemp != uLast) {
                Com::printF(PSTR("clearExtEEPROM(): "), (int)i);
                Com::printFLN(PSTR(" / "), (long)uMax);
                uLast = uTemp;
            }
        }
    }

    if (Printer::debugInfo()) {
        Com::printFLN(PSTR("clearExtEEPROM(): erasing complete"));
    }
} // clearExternalEEPROM

bool switchActiveHeatBed(char newActiveHeatBed) {
    if (isAnyScanRunning()) {
        Com::printFLN(PSTR("selectMatrix(): scan in progress"));
        return false;
    }
    if (!saveActiveHeatBedToExtEEPROM(newActiveHeatBed)) {
        // do not allow not-supported z-compensation matrix
        return false;
    }

    g_nActiveHeatBed = newActiveHeatBed;

    if (loadCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed))) {
        // there is no valid z-compensation matrix available
        initCompensationMatrix();
    }
    return true;
} // switchActiveHeatBed

void prepareZCompensation(void) {
    if (COMPENSATION_MATRIX_SIZE > EEPROM_SECTOR_SIZE) {
        if (Printer::debugErrors()) {
            Com::printFLN(PSTR("prepareZCompensation(): the size of the compensation matrix is too big"));
        }

        // TODO: show a message at the display in this case
        return;
    }

    // restore the default scan parameters
    restoreDefaultScanParameters();

    // restore the last known compensation matrix
    // this operation must be performed after restoring of the default scan parameters because the values from the EEPROM can overwrite some scan parameters
    if (loadCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed))) {
        // there is no valid compensation matrix available
        initCompensationMatrix();
        Com::printFLN(PSTR("prepareZCompensation(): the compensation matrix is not available"));
    }

} // prepareZCompensation

float g_ZSchraubenSollDrehungenWarm_U = 0;
float g_ZSchraubenSollKorrekturWarm_mm = 0;
char g_ZSchraubeOk = 0;

void calculateZScrewCorrection(void) {
    Com::printFLN(PSTR(" "));
    Com::printFLN(PSTR("Z-Schrauben-Helper: "));

    /*IDEAL ist es, wenn mand den Drucker "kalt" und frisch aufgeheizt einstellt.*/

    //Gerade eben muss ein Z-Scan die Matrix korrigiert haben.
    //Der Extruder darf sich seither nicht verändert (T0 -> T1) und nicht abgekühlt haben
    //Dann wird pro °C des aktiven Extruders ~0.001mm Längung angenommen.
    //Dann wird pro °C des Heizbettes ~0.0015mm Längung angenommen.

    //Wegen der unbekannten Nachlängung werden 0.15mm Puffer auf Z=0 angestrebt.
    //Einfach so: wird ein Puffer von 0.05 angestrebt. Die ideale Matrix-Verschiebung wird auf 260°C Hotend / 120°C Bett, was ein Extremwert darstellen soll auf -0.05mm angepeilt.

    //Config
    const float maxExtruderTemperature = Extruder::current->tempControl.heaterCutoffC;
    float maxBedTemperature = 120.0f; //120°C ist ok... mit 180 zu rechnen wäre übertrieben.
    float BedThermalExplansionInMikrons = 1.5f;
    float ExtruderThermalExpansionInMikrons = 0.95f;
    float maxNachdehnungInMikrons = 150.0f;
    float RestAbstandInMikrons = 50.0f;

    //Ist-Abstand.
    float MatrixMaximumInMikrons = (float)g_offsetZCompensationSteps * Printer::mmPerStep(Z_AXIS) * 1000.0f;

    float ExtruderTemperature = Extruder::current->tempControl.currentTemperatureC;
    if (ExtruderTemperature < 20.0f)
        ExtruderTemperature = 20.0f; //Wenn zu kalt oder undefiniert dann Standardbedingungen annehmen.

    float BedTemperature = Extruder::getHeatedBedTemperature();

    //Umrechnung des aktuellen Zustandes auf die heißesten Werte:
    float MinDistanceInMikronsKalt = MatrixMaximumInMikrons
        + (maxExtruderTemperature - ExtruderTemperature) * ExtruderThermalExpansionInMikrons
        + (maxBedTemperature - BedTemperature) * BedThermalExplansionInMikrons;
    float MinDistanceInMikronsWarm = MinDistanceInMikronsKalt + maxNachdehnungInMikrons;

    Com::printFLN(PSTR("- Alle Werte in Mikrometern / Einheit [um] -"));
    Com::printFLN(PSTR("Matrix-Minimum: "), MatrixMaximumInMikrons);
    Com::printFLN(PSTR("Weitere Extruderausdehnung maximal: "), (maxExtruderTemperature - ExtruderTemperature) * ExtruderThermalExpansionInMikrons);
    Com::printFLN(PSTR("Weitere Heizbettausdehnung maximal: "), (maxBedTemperature - BedTemperature) * BedThermalExplansionInMikrons);
    Com::printFLN(PSTR("Maximalwert-zMatrix bei Maximaltemperaturen (kalter Drucker): "), MinDistanceInMikronsKalt);
    Com::printFLN(PSTR("Maximalwert-zMatrix bei Maximaltemperaturen (durchgewaermter Drucker): "), MinDistanceInMikronsWarm);

    //z.B. -200 <-- um diesen Wert dürfte man korrigieren, wenn der Drucker zum Messzeitpunkt voll durchgewärmt wäre. Weiß er aber nicht!
    //Ein vorgewärmter Drucker justiert das Heizbett eher auf -0.2, ein kalter Drucker justiert es eher auf -0.05 bei Spitzentemperaturen. Beides ist ok.
    //float SollkorrekturKalt = (MinDistanceInMikronsKalt + RestAbstandInMikrons);
    //z.B.  -50 <-- diesen Wert darf man in jedem Fall korrigieren.
    float SollkorrekturWarm = (MinDistanceInMikronsWarm + RestAbstandInMikrons);

    /*
    Wenn ich den Test mit einem bereits warmen Drucker mache, plane ich unnötig eine Sicherheit ein, die ich nur einrechne, weil der Drucker aktuell kalt sein könnte.
    Also warmer Drucker: Bett weiter hoch justieren, also Schraube weiter rein, also Drehsinn Minus.
    */
    // |+0-|..Puffer..|.......Nachdehnung........|Kalt-Soll-Einstellung|
    // |+0-|..Puffer..|Warm-Soll-Einstellung|

    /* TIPP: -> Schraube bis maxNachdehnungInMikrons ~ 150um weiter runter(=Bett weiter hoch =Drehsinn Minus) empfehlen, wenn der Drucker aktuell "mehr druchgewärmt" wäre. */

    Com::println();
    Com::printSharpLine();
    Com::printF(PSTR("Sollkorrektur: "), SollkorrekturWarm, 0);
    Com::printF(PSTR(" [um] = "), SollkorrekturWarm * 0.001f, 3);
    Com::printFLN(PSTR(" [mm]"));

    //meldung:
    g_ZSchraubenSollKorrekturWarm_mm = SollkorrekturWarm * 0.001f;

    float ZSchraubenDrehungenWarm = SollkorrekturWarm * 0.002f; //[Sollkorrektur in mm] geteilt durch [Gewinde: 0.5 mm/Umdrehung] -> Sollkorrektur / 1000 / 0.5
    Com::printF(PSTR("Sollumdrehungen: "), ZSchraubenDrehungenWarm, 1);
    Com::printF(PSTR(" [U] = "), ZSchraubenDrehungenWarm * 360, 0);
    Com::printF(PSTR(" [Grad]"));

    //meldung:
    g_ZSchraubenSollDrehungenWarm_U = ZSchraubenDrehungenWarm;

    if (ZSchraubenDrehungenWarm > 0)
        Com::printFLN(PSTR(" (+ heisst rausdrehen/linksrum/gegen die Uhr)")); //Bett wird nach unten justiert
    else
        Com::printFLN(PSTR(" (- heisst reindrehen/rechtsrum/im Uhrzeigersinn)")); //Bett wird nach oben justiert

    Com::printFLN(PSTR("Je kaelter der Gesamtdrucker aktuell ist (nach langer Pause frisch angeschaltet), desto besser der Korrekturwert."));
    g_ZSchraubeOk = -1;                                                             //neg -> Matrix negativ -> ok. ausser, wenn:
#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2        //TODO: Prüfen ob das beim RF2000v2 stimmen wird.
    if (-0.5f <= ZSchraubenDrehungenWarm && SollkorrekturWarm < 40.0f /* [um] */) { // < 0.25mm = 0.5Umdrehungen ist mit dem RF2000 nicht machbar.
        Com::printFLN(PSTR(" (Die Z-Schraube ist ok!)"));                           //das ist die Änderung in M3-Regelgewinde-Z-Schrauben-Umdrehungen
        //meldung:
        g_ZSchraubeOk = -1;                                                                        //neg
    } else if (SollkorrekturWarm >= 40.0f /* [um] */) {                                            //dann bin ich rechnerisch um Z = 0 (50um mit 10um toleranz, die ich fordere.)
                                                                                                   //eine korrektur von mehr als +40um heißt, ich bin gerade vermutlich im Z>0
        Com::printFLN(PSTR(" (Die Z-Schraube weiter raus! Das Bett scheint zu hoch zu liegen.)")); //das ist die Änderung in M3-Regelgewinde-Z-Schrauben-Umdrehungen
        //meldung:
        g_ZSchraubeOk = 1; //pos
    }
    Com::printFLN(PSTR(" (RF2000: Minimal eine halbe Schraubendrehung (dZ=0.25mm-Schritte) einstellbar.)")); //das ist die Änderung in M3-Regelgewinde-Z-Schrauben-Umdrehungen
#else                                                                                                        //if MOTHERBOARD == DEVICE_TYPE_RF1000
    if (SollkorrekturWarm >= 40.0f /* [um] */) { //dann bin ich rechnerisch um Z = 0 (50um mit 10um toleranz, die ich fordere.)
        //eine korrektur von mehr als +40um heißt, ich bin gerade vermutlich im Z>0
        Com::printFLN(PSTR(" (Die Z-Schraube weiter raus! Das Bett scheint zu hoch zu liegen.)")); //das ist die Änderung in M3-Regelgewinde-Z-Schrauben-Umdrehungen
        //meldung:
        g_ZSchraubeOk = 1; //pos
    }
#endif
    Com::printSharpLine();
} // calculateZScrewCorrection

/**************************************************************************************************************************************/

/**************************************************************************************************************************************/

void fixKeramikLochInMatrix(void) {
    //Com::printFLN( PSTR( "fixKeramikLochInMatrix(): 1 Init" ) );

    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
        // we load the z compensation matrix before its first usage because this can take some time
        prepareZCompensation();
    }
    if (g_ZCompensationMatrix[0][0] == EEPROM_FORMAT) {
        // search for deepest hole in bed-z-matrix and fix it according to surrounding values

        long peak_hole = 0;
        long peak_x = 0;
        long peak_y = 0;

        long deepness = 0;
        long heights = 0;
        char div = 0;

        for (short x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) { //in der matrix ist alles von 1 an (?) -> also vermutlich anzahl = indexmax.
            for (short y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {

                heights = 0;
                div = 0;

                //circle around matrixposition
                for (short xx = x - 1; xx <= x + 1; xx++) {
                    for (short yy = y - 1; yy <= y + 1; yy++) {                                                     //iterate all points
                        if (xx != x || yy != y) {                                                                   //nicht den punkt in der mitte
                            if (xx <= g_uZMatrixMax[X_AXIS] && xx >= 1 && yy <= g_uZMatrixMax[Y_AXIS] && yy >= 1) { //nicht punkte ausserhalb der matrix
                                heights += (long)g_ZCompensationMatrix[xx][yy];
                                div += 1;
                            }
                        }
                    }
                }
                deepness = (long)((float)heights / div) - g_ZCompensationMatrix[x][y]; //nur täler, negative werte.
                if (deepness > peak_hole && div > 3) {                                 //nicht an ecken, sonst immer das tiefste loch suchen.
                    peak_hole = deepness;
                    peak_x = x;
                    peak_y = y;
                }
            }
        }

        //Com::printFLN( PSTR( "fixKeramikLochInMatrix(): 3 Extremwert" ) );
        //Com::printF( PSTR( "peak_x = " ), peak_x );
        //Com::printF( PSTR( "; peak_y = " ), peak_y );
        //Com::printF( PSTR( "; peak_hole = " ), peak_hole );
        //Com::printFLN( PSTR( "g_ZCompensationMatrix[peak_x,peak_y] =" ), g_ZCompensationMatrix[peak_x][peak_y] );

        if (peak_hole > 100 && peak_x > 0 && peak_y > 0) {
            //loch groß genug
            g_ZCompensationMatrix[peak_x][peak_y] += peak_hole;
            Com::printF(PSTR("Fixed: [peak_x,peak_y]="), g_ZCompensationMatrix[peak_x][peak_y]);
            g_ZMatrixChangedInRam = 1;
        } else {
            Com::printF(PSTR("Fix not needed dh<100 dh="), peak_hole);
        }

    } else {
        Com::printFLN(Com::tError);
    }

    determineCompensationOffsetZ(); //zur sicherheit und zukünftiger kompatibilität (Popel rausrechnen, würde den minimalen Z-Abstand verändern.)

    //Com::printFLN( PSTR( "fixKeramikLochInMatrix(): finished" ) );
    return;
} // fixKeramikLochInMatrix

/**************************************************************************************************************************************/

/**************************************************************************************************************************************/

void setMatrixNull(void) {
    //Damit kann die Matrix eingeebnet werden. Anschließend mit M3902 Zx.x verschoben, dann mit dem M3900/M3901 mit Distance-Schalter ein Auto-Bed-Leveling auf die Alte Art gemacht werden.
    //Man hätte dann eine total ebene Matrix. Die Z-Kompensation ist dann faktisch aus, kann aber weiterlaufen.

    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
        // we load the z compensation matrix before its first usage because this can take some time
        prepareZCompensation();
    }
    if (g_ZCompensationMatrix[0][0] == EEPROM_FORMAT) {
        for (short x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) { //in der matrix ist alles von 1 an (?) -> also vermutlich anzahl = indexmax.
            for (short y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
                g_ZCompensationMatrix[x][y] = 0;
            }
        }
        g_ZMatrixChangedInRam = 1; //man kan die matrix mit diesem marker nun sichern.
        Com::printFLN(PSTR("Matrix set to 0!"));
    } else {
        Com::printFLN(Com::tError);
    }
    determineCompensationOffsetZ(); //zur sicherheit und zukünftiger kompatibilität (Popel rausrechnen, würde den minimalen Z-Abstand verändern.)
    return;
} // setMatrixNull

/**************************************************************************************************************************************/

/**************************************************************************************************************************************/
/**************************************************************************************************************************************/

/**************************************************************************************************************************************/

namespace BedMatrix {

void restoreAtStartup() {
    loadActiveHeatBedFromExtEEPROM();
    if (!bedMatrixAvailable())
        prepareZCompensation();
}

void clearWorking() { initCompensationMatrix(); }
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
    //Nibbels: weight change because of distance. lerne bettwinkelausgleich.
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
            x_dist = (point.x - x) * (point.x - x) / x_bed_len_quadrat; //normierter indexabstand
            y_dist = (point.y - y) * (point.y - y) / y_bed_len_quadrat; //normierter indexabstand
            x_dist = sqrt(x_dist);                                                                    //Linearisieren. für Auto-Matrix-Leveling
            y_dist = sqrt(y_dist);                                                                    //Linearisieren. für Auto-Matrix-Leveling
            //das ist nur ein kreisabstand, wenn die messpunkte quadratisch angeordnet sind, ist aber nicht so?
            // evtl. todo: achse faktor skalieren, sodass kreis x/y=(10/13)
            xy_weight = 1 - sqrt(x_dist * x_dist + y_dist * y_dist); //linear gewichtet
            if (xy_weight < 0.0)
                xy_weight = 0;
            if (xy_weight > 1.0)
                xy_weight = 1.0; //kann aber nicht wirklich vorkommen.
            weighted_nZ = (long)(gradient * xy_weight * (float)nZ + (1.0 - gradient) * (float)nZ);
            newValue = (long)g_ZCompensationMatrix[x][y] + weighted_nZ;
            if (newValue > 32767 || newValue < -32768)
                result.overflow = true;
            if (newValue > 0)
                result.aboveZero = true; //darf nicht positiv werden. //darf nicht über limit sein.
            if (newValue > maximumHeight)
                result.tooHigh = true; //darf nicht positiv werden. //darf nicht über limit sein.
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

namespace GCodeCommands {
namespace Calibration {

void disableBedCompensation() {
    if (Printer::debugInfo()) {
        Com::printFLN(PSTR("M3000: disabling z compensation"));
    }
    queueTask(TASK_DISABLE_Z_COMPENSATION);
}

void enableBedCompensation() {
    if (isAnyScanRunning()) {
        Com::printFLN(PSTR("M3001: z compensation is unavailable during a scan"));
        return;
    }
    if (Printer::doHeatBedZCompensation) {
        Com::printFLN(PSTR("M3001: z compensation is enabled already"));
        return;
    }

    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
        // we try to load the z compensation matrix before its first usage because this can take some time
        prepareZCompensation();
    }

    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
        Com::printF(PSTR("M3001: z compensation can not be enabled. Heat bed compensation matrix not valid ( "), g_ZCompensationMatrix[0][0]);
        Com::printF(PSTR(" / "), EEPROM_FORMAT);
        Com::printFLN(PSTR(" )"));
        return;
    }

    if (!Printer::areAxisHomed()) {
        Com::printFLN(PSTR("M3001: z compensation can not be enabled. Home position is unknown"));
        showError((void*)ui_text_z_compensation, (void*)ui_text_home_unknown);
        return;
    }

    queueTask(TASK_ENABLE_Z_COMPENSATION);

    //wenn der nutzer im menü "autostart" gewählt hat, dann senseoffset sofort mit M3001 mitstarten.
    //Die Einstellwerte kommen dann aus dem EEPROM
    if (Printer::senseOffsetControl.autoStart())
        queueTask(TASK_ENABLE_SENSE_OFFSET);
}

void selectBedMatrix(GCode* pCommand) {
    long nTemp;
    if (Printer::doHeatBedZCompensation) {
        // do not allow to change the current heat bed z-compensation matrix while the z-compensation is active
        if (Printer::debugErrors()) {
            Com::printFLN(PSTR("M3009: heat bed z matrix can not be changed while z-compensation is active"));
        }

        showError((void*)ui_text_z_compensation, (void*)ui_text_operation_denied);
        return;
    }

    if (pCommand->hasS()) {
        nTemp = pCommand->S;

        if (nTemp < 1 || nTemp > EEPROM_MAX_HEAT_BED_SECTORS) {
            if (Printer::debugErrors()) {
                Com::printF(PSTR("M3009: invalid heat bed z matrix ("), nTemp);
                Com::printFLN(PSTR(")"));
            }
            return;
        }

        if (g_nActiveHeatBed != nTemp) {
            // we have to switch to another heat bed z-compensation matrix
            switchActiveHeatBed((char)nTemp);
        }
    }

    if (Printer::debugInfo()) {
        Com::printFLN(PSTR("M3009: currently active heat bed z matrix: "), g_nActiveHeatBed);
    }
}

void clearBedMatrix(GCode* pCommand) {
    long nTemp;
    if (isAnyScanRunning()) {
        Com::printFLN(PSTR("M3011: scan in progress"));
        return;
    }
    if (Printer::doHeatBedZCompensation) {
        if (Printer::debugErrors()) {
            Com::printFLN(PSTR("M3011: the heat bed z matrix can not be cleared while the z-compensation is active"));
        }

        showError((void*)ui_text_z_compensation, (void*)ui_text_operation_denied);
        return;
    }
    if (pCommand->hasS()) {
        nTemp = pCommand->S;
    } else {
        // we clear the current z-compensation matrix in case no other z-compensation matrix is specified
        nTemp = g_nActiveHeatBed;
    }

    if (nTemp < 1 || nTemp > EEPROM_MAX_HEAT_BED_SECTORS) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("M3011: invalid heat bed z matrix ("), nTemp);
            Com::printFLN(PSTR(")"));
        }
        return;
    }

    if (saveActiveHeatBedToExtEEPROM((char)nTemp)) {
        // switch to the specified z-compensation matrix
        g_nActiveHeatBed = (char)nTemp;
        if (clearCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed)))
            Com::printFLN(PSTR("M3011: cleared heat bed z matrix: "), nTemp);
    }
}

void reportBedMatrix(GCode* pCommand) {
    long nTemp;
    char format;
    char nOldHeatBed = 127;

    if (pCommand->hasS()) {
        nTemp = pCommand->S;
    } else {
        // we output the current z-compensation matrix in case no other z-compensation matrix is specified
        nTemp = g_nActiveHeatBed;
    }

    if (pCommand->hasP()) {
        format = pCommand->P ? 1 : 0;
    } else {
        format = 0;
    }

    if (nTemp < 1 || nTemp > EEPROM_MAX_HEAT_BED_SECTORS) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("M3013: invalid heat bed z matrix ("), nTemp);
            Com::printFLN(PSTR(")"));
        }
        return;
    }

    if (g_nActiveHeatBed != nTemp) {
        // remember current z-compensation matrix
        nOldHeatBed = g_nActiveHeatBed;
        // we have to switch to another z-compensation matrix
        if (!switchActiveHeatBed((char)nTemp))
            return;
    }

    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
        // we load the z-compensation matrix before its first usage because this can take some time
        prepareZCompensation();
    }

    if (g_ZCompensationMatrix[0][0] == EEPROM_FORMAT) {
        if (Printer::debugInfo()) {
            Com::printFLN(PSTR("M3013: current heat bed z-compensation matrix: "));
        }
        outputCompensationMatrix(format);
    } else {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("M3013: the heat bed z-compensation matrix is not valid ( "), g_ZCompensationMatrix[0][0]);
            Com::printF(PSTR(" / "), EEPROM_FORMAT);
            Com::printFLN(PSTR(" )"));
        }

        showError((void*)ui_text_z_compensation, (void*)ui_text_invalid_matrix);
    }

    if (nOldHeatBed != 127) {
        // restore z-compensation matrix, in case we switched them
        switchActiveHeatBed(nOldHeatBed);
    }
}

void eraseMatrixEeprom() {
    if (isAnyScanRunning()) {
        Com::printFLN(PSTR("clearExtEEPROM(): scan in progress"));
        return;
    }
    clearExternalEEPROM();
}

void adjustBedMatrix(GCode* pCommand) {
    if (isAnyScanRunning()) {
        Com::printFLN(PSTR("M3902: scan in progress"));
        return;
    }
    //NMM Funktion 1 - R=Repair
    if (pCommand->hasR()) {
        //search for a hole within the heat beds z-Matrix
        fixKeramikLochInMatrix();
    }

    if (pCommand->hasE()) {
        //completly wipe the matrix-data to zero->flatten the matrix to nothing.
        setMatrixNull();
    }
    //NMM Funktion 2 - Z=Offset manuell nachstellen
    if (pCommand->hasZ()) {
        if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
            // we load the z compensation matrix before its first usage because this can take some time
            Com::printFLN(PSTR("M3901: INFO Die Z-Matrix wurde aus dem EEPROM gelesen."));
            prepareZCompensation();
        }
        if (g_ZCompensationMatrix[0][0] == EEPROM_FORMAT) {
            //search for a hole within the heat beds z-Matrix
            float hochrunter = (float)pCommand->Z;
            if (hochrunter > 0.2f)
                hochrunter = 0.2f;
            if (hochrunter < -0.2f)
                hochrunter = -0.2f;

            Com::printSharpLine();

            if (hochrunter < 0.0f) {
                Com::printFLN(PSTR("M3902: Duese-Bett-Abstand wird kleiner gemacht. "), hochrunter);
            } else if (hochrunter > 0.0f) {
                Com::printFLN(PSTR("M3902: Duese-Bett-Abstand wird groesser gemacht. "), hochrunter);
            } else {
                Com::printFLN(PSTR("M3902: Duese-Bett-Abstand bleibt gleich. Das Offset wird in die Matrix verrechnet und genullt."), hochrunter);
            }
            Com::printFLN(PSTR("M3902: -Z heisst Bett hoch/weniger Abstand, +Z heisst Bett runter/mehr Abstand. "));
            Com::printFLN(PSTR("M3902: Bei Z=0 ändert sich nichts, doch es wird das aktuelle Offset in die Matrix verrechnet. "));

            Com::printSharpLine();

            // determine the minimal distance between extruder and heat bed
            determineCompensationOffsetZ(); //-> schreibt kleinsten abstand in g_offsetZCompensationSteps, sollte schon drin sein, aber man weiß nie.
            Com::printFLN(PSTR("M3902: Alt::Min. Bett-Hotend Abstand [Steps] = "), -1 * (int)g_offsetZCompensationSteps);

            long hochrunterSteps = long(hochrunter * Printer::stepsPerMm(Z_AXIS)); //axissteps ist auch float

            Com::printFLN(PSTR("M3902: Veraenderung Z [mm] = "), hochrunter);
            Com::printFLN(PSTR("M3902: Veraenderung Z [Steps] = "), hochrunterSteps);

            if (hochrunter == 0.00f) {
                //wenn Z=0.0 soll alles so bleibenk aber das Offset ins Matrix-Offset reingerechnet und genullt werden.
                //das Offset muss negativ eingehen, denn wenn die Matrix "weniger tief" ist, bleibt die Düse weiter weg vom Bett.
                hochrunterSteps = Printer::zOffset.manualSteps(Printer::stepsPerMm(Z_AXIS));
                Com::printFLN(PSTR("M3902: Veraenderung = zOffset --> zMatrix; Offset = 0;"));
            }

            //das ist ein negativer wert! Je mehr Abstand, desto negativer. Positiv verboten.
            if (g_offsetZCompensationSteps + hochrunterSteps > (long(Printer::stepsPerMm(Z_AXIS) * ::Calibration::probeSettings().startLiftMm) - -1 * ::Calibration::probeSettings().fastApproachSteps)) {
                Com::printF(PSTR("M3902: Fehler::Z-Matrix wuerde positiv werden. Das waere eine Kollision um "), (int)(g_offsetZCompensationSteps + hochrunterSteps));
                Com::printFLN(PSTR(" [Steps] bzw. "), Printer::mmPerStep(Z_AXIS) * (float)(g_offsetZCompensationSteps + hochrunterSteps), 2);
                Com::printFLN(PSTR(" [mm]"));
            } else {
                Com::printFLN(PSTR("M3902: Neu::Min. Bett-Extruder Restabstand = "), -1 * (int)(g_offsetZCompensationSteps));

                short x;
                short y;
                short deltaZ = (short)hochrunterSteps;
                bool overflow = false;
                bool overnull = false;
                bool overH = false;

                for (x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) {
                    for (y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
                        if ((long)g_ZCompensationMatrix[x][y] + (long)deltaZ >= 32767) {
                            overflow = true;
                        } else {
                            g_ZCompensationMatrix[x][y] += deltaZ;
                            if (g_ZCompensationMatrix[x][y] > 0)
                                overnull = true; //overflow oder kollision, kann einfach nicht sein und abfrage ist kurz.
                            if (g_ZCompensationMatrix[x][y] > (long(Printer::stepsPerMm(Z_AXIS) * ::Calibration::probeSettings().startLiftMm) - -1 * ::Calibration::probeSettings().fastApproachSteps))
                                overH = true;
                        }
                    }
                }
                if (overnull) {
                    Com::printFLN(PSTR("M3901: WARNING::positive Matrix::Please fix Z-Screw"));
                }
                if (overH) {
                    Com::printFLN(PSTR("M3901: ERROR::sehr positive Matrix::ReLoading zMatrix from EEPROM to RAM"));
                    if (loadCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed))) {
                        // Error: there is no valid compensation matrix available
                        initCompensationMatrix();
                    }
                } else if (overflow) {
                    Com::printFLN(PSTR("M3901: ERROR::Overflow in Matrix::ReLoading zMatrix from EEPROM to RAM"));
                    if (loadCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed))) {
                        // Error: there is no valid compensation matrix available
                        initCompensationMatrix();
                    }
                } else {
                    if (hochrunter == 0.00f) {
                        Printer::zOffset.setManualMicrometers(0, Printer::stepsPerMm(Z_AXIS));
                        EEPROM::storeManualZOffset();
                    }
                    g_ZMatrixChangedInRam = 1;
                }
            }

            // determine the minimal distance between extruder and heat bed
            determineCompensationOffsetZ();
            Com::printFLN(PSTR("M3902: Neu::Min. Bett-Extruder Restabstand [Steps] = "), -1 * (int)g_offsetZCompensationSteps);
            Com::printFLN(PSTR("M3902: Neu::Min. Bett-Extruder Restabstand [mm] = "), -1 * Printer::mmPerStep(Z_AXIS) * (float)g_offsetZCompensationSteps, 2);
        } else {
            Com::printFLN(PSTR("M3902: ERROR::Matrix Initialisation Error!"));
            Com::printFLN(PSTR("M3902: INFO Die Z-Matrix konnte nicht aus dem EEPROM gelesen werden."));
            Com::printFLN(PSTR("M3902: INFO Vermutlich nie einen Heat-Bed-Scan gemacht."));
        }
    }

    //NMM Funktion 3 - S=Save,Sichern der Matrix an spezielle EEPROM-Position
    if (pCommand->hasS()) {
        if (pCommand->S >= 1 && pCommand->S <= EEPROM_MAX_HEAT_BED_SECTORS) {
            // save the determined values to the EEPROM @ savepoint "pCommand->S" Standard: 1..9
            unsigned int savepoint = (unsigned int)pCommand->S;
            if (saveCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * savepoint)))
                Com::printFLN(PSTR("M3902: Save the Matrix::OK"));
            else
                Com::printFLN(PSTR("M3902: Save the Matrix::ERROR"));
        } else {
            Com::printFLN(PSTR("M3902: Save the Matrix::ERROR::invalid savepoint, invalid active heatbedmatrix"));
        }
    }
}

} // namespace Calibration
} // namespace GCodeCommands

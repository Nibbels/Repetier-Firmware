// Private implementation fragment; included only by BedMatrix.cpp.
#ifndef BED_MATRIX_STORAGE_H
#define BED_MATRIX_STORAGE_H

static bool writeMatrixSector(unsigned int address, bool clear) {
    const uint8_t eeprom = I2C_ADDRESS_EXTERNAL_EEPROM;
    const unsigned int formatAddress = address + EEPROM_OFFSET_SECTOR_FORMAT;
    const unsigned short format = clear ? 0 : EEPROM_FORMAT;
    const uint8_t lastX = clear ? COMPENSATION_MATRIX_MAX_X - 1 : g_uZMatrixMax[X_AXIS];
    const uint8_t lastY = clear ? COMPENSATION_MATRIX_MAX_Y - 1 : g_uZMatrixMax[Y_AXIS];
    const unsigned short microsteps = clear ? 0 : drv8711ModeValue_2_MicroSteps(Drv8711::microstepMode(Z_AXIS));

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
        drv8711ModeValue_2_MicroSteps(Drv8711::microstepMode(Z_AXIS))
    ) {
        // the current z-compensation matrix has been determined with the current micro step setting, there is nothing to recalculate
        fMicroStepCorrection = 1.0;
    } else {
        // the current z-compensation matrix has been determined with a higher than the current micro step setting, we must divide all z-correction values
        fMicroStepCorrection = (float)
                                   drv8711ModeValue_2_MicroSteps(Drv8711::microstepMode(Z_AXIS))
            / (float)uMicroSteps;

        Com::printF(PSTR("loadMatrix(): micro step correction = "), fMicroStepCorrection);
        Com::printF(PSTR(" (stored="), (int)uMicroSteps);
        Com::printF(PSTR(", current="), (int)
                                            drv8711ModeValue_2_MicroSteps(Drv8711::microstepMode(Z_AXIS))
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


/**************************************************************************************************************************************/

/**************************************************************************************************************************************/

#endif // BED_MATRIX_STORAGE_H

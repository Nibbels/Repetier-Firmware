// Private implementation fragment; included only by BedMatrix.cpp.
#ifndef BED_MATRIX_COMMANDS_H
#define BED_MATRIX_COMMANDS_H

namespace GCodeCommands {
namespace Calibration {

void disableBedCompensation() {
    queueTask(QueueTask::DisableZCompensation);
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

    queueTask(QueueTask::EnableZCompensation);

    // Wenn der nutzer im menü "autostart" gewählt hat, dann senseoffset sofort mit M3001 mitstarten.
    // Die Einstellwerte kommen dann aus dem EEPROM
    if (Printer::senseOffsetControl.autoStart())
        queueTask(QueueTask::EnableSenseOffset);
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

    Com::printFLN(PSTR("M3009: currently active heat bed z matrix: "), g_nActiveHeatBed);
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
        Com::printFLN(PSTR("M3013: current heat bed z-compensation matrix: "));
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

void adjustBedMatrix(GCode* pCommand) {
    if (isAnyScanRunning()) {
        Com::printFLN(PSTR("M3902: scan in progress"));
        return;
    }
    // NMM Funktion 1 - R=Repair
    if (pCommand->hasR()) {
        // Search for a hole within the heat beds z-Matrix
        fixKeramikLochInMatrix();
    }

    if (pCommand->hasE()) {
        // Completly wipe the matrix-data to zero->flatten the matrix to nothing.
        setMatrixNull();
    }
    // NMM Funktion 2 - Z=Offset manuell nachstellen
    if (pCommand->hasZ()) {
        if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
            // we load the z compensation matrix before its first usage because this can take some time
            Com::printFLN(PSTR("M3901: INFO Die Z-Matrix wurde aus dem EEPROM gelesen."));
            prepareZCompensation();
        }
        if (g_ZCompensationMatrix[0][0] == EEPROM_FORMAT) {
            // Search for a hole within the heat beds z-Matrix
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
            determineCompensationOffsetZ(); // -> schreibt kleinsten abstand in g_offsetZCompensationSteps, sollte schon drin sein, aber man weiß nie.
            Com::printFLN(PSTR("M3902: Alt::Min. Bett-Hotend Abstand [Steps] = "), -1 * (int)g_offsetZCompensationSteps);

            long hochrunterSteps = long(hochrunter * Printer::stepsPerMm(Z_AXIS)); // axissteps ist auch float

            Com::printFLN(PSTR("M3902: Veraenderung Z [mm] = "), hochrunter);
            Com::printFLN(PSTR("M3902: Veraenderung Z [Steps] = "), hochrunterSteps);

            if (hochrunter == 0.00f) {
                // Wenn Z=0.0 soll alles so bleibenk aber das Offset ins Matrix-Offset reingerechnet und genullt werden.
                // Das Offset muss negativ eingehen, denn wenn die Matrix "weniger tief" ist, bleibt die Düse weiter weg vom Bett.
                hochrunterSteps = Printer::zOffset.manualSteps(Printer::stepsPerMm(Z_AXIS));
                Com::printFLN(PSTR("M3902: Veraenderung = zOffset --> zMatrix; Offset = 0;"));
            }

            // Das ist ein negativer wert! Je mehr Abstand, desto negativer. Positiv verboten.
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
                                overnull = true; // Overflow oder kollision, kann einfach nicht sein und abfrage ist kurz.
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

    // NMM Funktion 3 - S=Save,Sichern der Matrix an spezielle EEPROM-Position
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

#endif // BED_MATRIX_COMMANDS_H

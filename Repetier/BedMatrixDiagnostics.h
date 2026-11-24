// Private implementation fragment; included only by BedMatrix.cpp.
#ifndef BED_MATRIX_DIAGNOSTICS_H
#define BED_MATRIX_DIAGNOSTICS_H

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

#endif // BED_MATRIX_DIAGNOSTICS_H

// Private implementation fragment; included only by BedMatrix.cpp.
#ifndef BED_MATRIX_GRID_H
#define BED_MATRIX_GRID_H

static char prepareCompensationMatrix(void) {
    short x;
    short y;

    // perform some safety checks first
    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
        {
            Com::printF(PSTR("zCMP: invalid version: "), g_ZCompensationMatrix[0][0]);
            Com::printF(PSTR(" (expected: "), EEPROM_FORMAT);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }

    if (g_uZMatrixMax[X_AXIS] >= COMPENSATION_MATRIX_MAX_X || g_uZMatrixMax[X_AXIS] < 2) {
        {
            Com::printF(PSTR("zCMP: invalid x dimension: "), g_uZMatrixMax[X_AXIS]);
            Com::printF(PSTR(" (max expected: "), COMPENSATION_MATRIX_MAX_X - 1);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }

    if (g_uZMatrixMax[Y_AXIS] >= COMPENSATION_MATRIX_MAX_Y || g_uZMatrixMax[Y_AXIS] < 2) {
        {
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

#endif // BED_MATRIX_GRID_H

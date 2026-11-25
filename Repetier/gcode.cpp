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

// Decoded G-code command utilities and reporting; serial framing and parsing belong to GCodeInput.cpp.

#include "Repetier.h"

/** \brief Print command on serial console */
void GCode::printCommand() {
    if (hasN()) {
        Com::print('N');
        Com::print(int32_t(N)); //65536 wäre max, nicht 32768 -> sonst sind N-xxx in der Log
        Com::print(' ');
    }
    if (hasM()) {
        Com::print('M');
        Com::print(int32_t(M));
        Com::print(' ');
    }
    if (hasG()) {
        Com::print('G');
        Com::print(int32_t(G));
        Com::print(' ');
    }
    if (hasT()) {
        Com::print('T');
        Com::print((int)T);
        Com::print(' ');
    }
    if (hasX()) {
        Com::printF(Com::tX, X, 3);
    }
    if (hasY()) {
        Com::printF(Com::tY, Y, 3);
    }
    if (hasZ()) {
        Com::printF(Com::tZ, Z, 3);
    }
    if (hasE()) {
        Com::printF(Com::tE, E, 4);
    }
    if (hasF()) {
        Com::printF(Com::tF, F, 3);
    }
    if (hasS()) {
        Com::printF(Com::tS, S);
    }
    if (hasP()) {
        Com::printF(Com::tP, P);
    }
    if (hasI()) {
        Com::printF(Com::tI, I, 3);
    }
    if (hasJ()) {
        Com::printF(Com::tJ, J, 3);
    }
    if (hasR()) {
        Com::printF(Com::tR, R, 3);
    }
    if (hasD()) {
        Com::printF(Com::tD, D, 3);
    }
    if (hasC()) {
        Com::printF(Com::tC, C, 3);
    }
    if (hasH()) {
        Com::printF(Com::tH, H, 3);
    }
    if (hasA()) {
        Com::printF(Com::tA, A, 3);
    }
    if (hasB()) {
        Com::printF(Com::tB, B, 3);
    }
    if (hasK()) {
        Com::printF(Com::tK, K, 3);
    }
    if (hasL()) {
        Com::printF(Com::tL, L, 3);
    }
    if (hasO()) {
        Com::printF(Com::tO, O, 3);
    }
    if (hasString()) {
        Com::print(text);
    }
    Com::println();

} // printCommand

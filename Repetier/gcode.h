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

// Decoded command data and firmware busy-state values; see GCodeInput for receiver ownership.

#ifndef GCODE_H
#define GCODE_H

#include <stdint.h>

enum FirmwareState { NotBusy = 0,
                     Processing,
                     Paused,
                     WaitHeater,
                     Calibrating };

class GCode {
    friend class GCodeInput;

    uint16_t params;
    uint16_t params2;

public:
    uint16_t N; ///< Line number reduced to 16 bit
    uint16_t M; ///< G-code M value if set
    uint16_t G; ///< G-code G value if set
    float X;    ///< G-code X value if set
    float Y;    ///< G-code Y value if set
    float Z;    ///< G-code Z value if set
    float E;    ///< G-code E value if set
    float F;    ///< G-code F value if set
    int32_t S;  ///< G-code S value if set
    int32_t P;  ///< G-code P value if set
    float I;    ///< G-code I value if set
    float J;    ///< G-code J value if set
    float R;    ///< G-code R value if set
    // Protocol version 3:
    float D; ///< G-code D value if set
    float C; ///< G-code C value if set
    float H; ///< G-code H value if set
    float A; ///< G-code A value if set
    float B; ///< G-code B value if set
    float K; ///< G-code K value if set
    float L; ///< G-code L value if set
    float O; ///< G-code O value if set

    char* text; ///< Text message of g-code if present.
    // Moved the byte to the end and aligned ints on short boundary
    // Old habit from PC, which require alignments for data types such as int and long to be on 2 or 4 byte boundary
    // Otherwise, the compiler adds padding, wasted space.
    uint8_t T; // This may not matter on any of these controllers, but it can't hurt

    // True if origin did not come from serial console. That way we can send status messages to
    // a host only if he would normally not know about the mode switch.
    bool internalCommand;

    inline bool hasM() {
        return ((params & 2) != 0);
    } // hasM

    inline bool hasN() {
        return ((params & 1) != 0);
    } // hasN

    inline bool hasG() {
        return ((params & 4) != 0);
    } // hasG

    inline void setX(char set) {
        if (set)
            params |= 8;
        else
            params &= ~8;
    } // setX

    inline bool hasX() {
        return ((params & 8) != 0);
    } // hasX

    inline void setY(char set) {
        if (set)
            params |= 16;
        else
            params &= ~16;
    } // setY

    inline bool hasY() {
        return ((params & 16) != 0);
    } // hasY

    inline void setZ(char set) {
        if (set)
            params |= 32;
        else
            params &= ~32;
    } // setZ

    inline bool hasZ() {
        return ((params & 32) != 0);
    } // hasZ

    inline bool hasNoXYZ() {
        return ((params & 56) == 0);
    } // hasNoXYZ

    inline bool hasE() {
        return ((params & 64) != 0);
    } // hasE

    //(params & 128) param1 Bit 7 :  always set to distinguish binary from ASCII line.

    inline bool hasF() {
        return ((params & 256) != 0);
    } // hasF

    inline bool hasT() {
        return ((params & 512) != 0);
    } // hasT

    inline bool hasS() {
        return ((params & 1024) != 0);
    } // hasS

    inline bool hasP() {
        return ((params & 2048) != 0);
    } // hasP

    inline bool isV2() {
        return ((params & 4096) != 0);
    } // isV2

    inline bool hasString() {
        return ((params & 32768) != 0);
    } // hasString

    inline bool hasI() {
        return ((params2 & 1) != 0);
    } // hasI

    inline bool hasJ() {
        return ((params2 & 2) != 0);
    } // hasJ

    inline bool hasR() {
        return ((params2 & 4) != 0);
    } // hasR

    // New identifiers
    inline bool hasD() {
        return ((params2 & 8) != 0);
    }
    inline bool hasC() {
        return ((params2 & 16) != 0);
    }
    inline bool hasH() {
        return ((params2 & 32) != 0);
    }
    inline bool hasA() {
        return ((params2 & 64) != 0);
    }
    inline bool hasB() {
        return ((params2 & 128) != 0);
    }
    inline bool hasK() {
        return ((params2 & 256) != 0);
    }
    inline bool hasL() {
        return ((params2 & 512) != 0);
    }
    inline bool hasO() {
        return ((params2 & 1024) != 0);
    }

    inline long getS(long def) {
        return (hasS() ? S : def);
    } // getS
    inline long getP(long def) {
        return (hasP() ? P : def);
    } // getP
    inline void setFormatError() {
        params2 |= 32768;
    }
    inline bool hasFormatError() {
        return ((params2 & 32768) != 0);
    }
    void printCommand();
};

#endif // GCODE_H

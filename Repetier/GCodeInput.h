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

#ifndef GCODE_INPUT_H
#define GCODE_INPUT_H

#include "gcode.h"

class GCodeInput {
public:
    static constexpr uint32_t keepAliveIntervalMs = 2000;

    static void readFromSerial();
    static GCode* peekCurrentCommand();
    static void popCurrentCommand();
    static void keepAlive(FirmwareState state);
    static void executeFlashScript(const char* cmd);

private:
    static bool parseAscii(GCode& command, char* line, bool fromSerial);
    static bool parseBinary(GCode& command, uint8_t* buffer, int8_t length);
    static uint8_t computeBinarySize(char* frame);
    static void requestResend();
    static void checkAndPushCommand(GCode& command);
    static void reportCorruptCommand(GCode& command);
};

#endif

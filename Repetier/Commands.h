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


#ifndef COMMANDS_H
#define COMMANDS_H

class Commands
{
public:
    static void commandLoop();
    static void checkForPeriodicalActions( enum FirmwareState state = NotBusy );
    static void executeGCode(GCode *com);
    static void waitUntilEndOfAllMoves();
    static void waitUntilEndOfZOS();
    static void printCurrentPosition();
    static void printTemperatures(bool showRaw = false);
#if FAN_PIN>-1 && FEATURE_FAN_CONTROL
    static void setFanSpeed(uint8_t speed, bool recalc = false); /// Set fan speed 0..255
    static void adjustFanFrequency(uint8_t speed_mode);
    static void adjustFanMode(uint8_t output_mode = 0);
#endif // FAN_PIN>-1 && FEATURE_FAN_CONTROL
    static void changeFeedrateMultiply(int factorInPercent);
    static void changeFlowrateMultiply(float factorInPercent);
    static void reportPrinterUsage();
    static void emergencyStop();
    static void checkFreeMemory();
    static void writeLowestFreeRAM();
    static int lowestRAMValue;
    static int lowestRAMValueSend;

}; // Commands


#endif // COMMANDS_H

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

#ifndef REPETIER_H
#define REPETIER_H

#include "BoardConfiguration.h"
#include "HAL.h"
#include "gcode.h"
#include "RF.h"
#include "ui.h"
#include "Communication.h"

#undef min
#undef max

class RMath {
public:
    static inline float min(float a, float b) {
        if (a < b)
            return a;
        return b;
    } // min

    static inline float max(float a, float b) {
        if (a < b)
            return b;
        return a;
    } // max

    static inline long min(long a, long b) {
        if (a < b)
            return a;
        return b;
    } // min

    static inline long max(long a, long b) {
        if (a < b)
            return b;
        return a;
    } // max

    static inline int min(int a, int b) {
        if (a < b)
            return a;
        return b;
    } // min

    static inline int max(int a, int b) {
        if (a < b)
            return b;
        return a;
    } // max

    static inline unsigned int min(unsigned int a, unsigned int b) {
        if (a < b)
            return a;
        return b;
    } // min

    static inline unsigned int max(unsigned int a, unsigned int b) {
        if (a < b)
            return b;
        return a;
    } // max

    static inline long sqr(long a) {
        return a * a;
    } // sqr

    static inline float sqr(float a) {
        return a * a;
    } // sqr
};

// Indices follow the temperature ADC channel order in HAL.cpp.
enum TemperatureAdcIndex : uint8_t {
    Hotend0AdcIndex,
#if NUM_EXTRUDER > 1
    Hotend1AdcIndex,
#endif
    BedAdcIndex,
#ifdef X35_ADC_CHANNEL
    ChamberAdcIndex,
#endif
    TemperatureAdcCount
};


#include "Extruder.h"


extern void drv8711Init();

#include "Printer.h"
#include "motion.h"
#include "MoveQueue.h"




#include "Commands.h"
#include "Eeprom.h"

#endif // REPETIER_H

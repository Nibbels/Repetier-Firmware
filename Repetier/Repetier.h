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

#include "Constants.h"

//https://gcc.gnu.org/onlinedocs/cpp/Stringizing.html#Stringizing for using number-#defines in string-#defines
#define xstr(a) str(a)
#define str(a) #a

#include "Configuration.h"

#if MOTHERBOARD == DEVICE_TYPE_RF1000
#include "RF1000.h"
#elif MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
#include "RF2000.h"
#else
#error Unsupported RFx000 model
#endif

#ifndef EXT0_HOTEND_PROFILE
#define EXT0_HOTEND_PROFILE DEFAULT_HOTEND_PROFILE
#endif
#ifndef EXT1_HOTEND_PROFILE
#define EXT1_HOTEND_PROFILE DEFAULT_HOTEND_PROFILE
#endif
#ifndef EXT0_TEMPSENSOR_TYPE
#define EXT0_TEMPSENSOR_TYPE DEFAULT_HOTEND_SENSOR_TYPE
#endif
#ifndef EXT1_TEMPSENSOR_TYPE
#define EXT1_TEMPSENSOR_TYPE DEFAULT_HOTEND_SENSOR_TYPE
#endif

#include "pins.h"
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

// Indices follow the order of osAnalogInputChannels.
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

extern const uint8 osAnalogInputChannels[TemperatureAdcCount] PROGMEM;
extern volatile uint8 osAnalogInputCounter[TemperatureAdcCount];
extern volatile uint osAnalogInputBuildup[TemperatureAdcCount];
extern volatile uint8 osAnalogInputPos; // Current sampling position
extern volatile uint osAnalogInputValues[TemperatureAdcCount];
extern uint8_t pwm_pos[NUM_EXTRUDER + 3]; // 0-NUM_EXTRUDER = Heater 0-NUM_EXTRUDER of extruder, NUM_EXTRUDER = Heated bed, NUM_EXTRUDER+1 Board fan, NUM_EXTRUDER+2 = Fan
extern uint8_t fanSpeed;                  //remember user input fan speed at a 0..255 scale.

#include "Extruder.h"


extern void drv8711Init();

#include "Printer.h"
#include "motion.h"

extern long baudrate;

extern volatile uint8_t execute100msPeriodical;
extern volatile uint8_t execute50msPeriodical;
extern volatile uint8_t execute16msPeriodical;
extern volatile uint8_t execute10msPeriodical;

extern uint8_t fanKickstart;

#include "Commands.h"
#include "Eeprom.h"

#endif // REPETIER_H

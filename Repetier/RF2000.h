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

#ifndef RF2000_H
#define RF2000_H

// RF2000 and RF2000v2 share the configuration below. Select their factory
// differences here before deriving motion, heater and scan settings.
#if MOTHERBOARD == DEVICE_TYPE_RF2000v2
// Identity and Z endstop
#define UI_PRINTER_NAME "RF2000v2"
#define Z_ENDSTOP_DRIVE_OVER 3.0f
#define Z_ENDSTOP_MAX_HYSTERESIS 0.2f

// Travel limits in mm
#define X_MAX_LENGTH_DUAL 170
#define Y_MAX_LENGTH 290
#define Z_MAX_LENGTH 185

// Extruder and hotend
#define DEFAULT_HOTEND_PROFILE HotendProfiles::v3
#define EXT0_STEPS_PER_MM (8.0 * RF_MICRO_STEPS_E)
#define EXT1_STEPS_PER_MM (8.00 * RF_MICRO_STEPS_E)
#define DEFAULT_HOTEND_SENSOR_TYPE 13
#define EXT1_X_OFFSET_MM 33.6f
#define EXT1_Y_OFFSET_MM 0.0f

// Bed heater PID and scan
#define HEATED_BED_PID_PGAIN 34.0
#define HEATED_BED_PID_IGAIN 0.015
#define HEATED_BED_PID_DGAIN 110.0
#define HEAT_BED_SCAN_DOWN_FAST_MM 0.2f

// Part fan frequency divisor and PWM range (1–254)
#define PART_FAN_DEFAULT_PWM_SPEED_DIVISOR 5
#define PART_FAN_PWM_MIN 90
#define PART_FAN_PWM_MAX 193
#else
// Identity and Z endstop
#define UI_PRINTER_NAME "RF2000"
#define Z_ENDSTOP_DRIVE_OVER 1.3f
#define Z_ENDSTOP_MAX_HYSTERESIS 0.1f

// Travel limits in mm
#define X_MAX_LENGTH_DUAL 180
#define Y_MAX_LENGTH 245
#define Z_MAX_LENGTH 200

// Extruder and hotend
#define DEFAULT_HOTEND_PROFILE HotendProfiles::v2
#define EXT0_STEPS_PER_MM (8.75 * RF_MICRO_STEPS_E)
#define EXT1_STEPS_PER_MM (8.75 * RF_MICRO_STEPS_E)
#define DEFAULT_HOTEND_SENSOR_TYPE 3
#define EXT1_X_OFFSET_MM 33.9f
#define EXT1_Y_OFFSET_MM 0.1f

// Bed heater PID and scan
#define HEATED_BED_PID_PGAIN 53.74
#define HEATED_BED_PID_IGAIN 7.48
#define HEATED_BED_PID_DGAIN 96.52
#define HEAT_BED_SCAN_DOWN_FAST_MM 0.1f

// Part fan frequency divisor and PWM range (1–254)
#define PART_FAN_DEFAULT_PWM_SPEED_DIVISOR 3
#define PART_FAN_PWM_MIN 1
#define PART_FAN_PWM_MAX 160
#endif

/** \brief Number of extruders */
#ifndef NUM_EXTRUDER
#define NUM_EXTRUDER 2 // 1 = Single, 2 = Dual
#endif

// ##########################################################################################
// ##   main hardware configuration
// ##########################################################################################

#if NUM_EXTRUDER < 1 || NUM_EXTRUDER > 2
#error RFx000 firmware supports one or two extruders.
#endif

// ##########################################################################################
// ##   Calibration
// ##########################################################################################

/** \brief maximum positions in mm - only fixed numbers!
For delta robot Z_MAX_LENGTH is the maximum travel of the towers and should be set to the distance between the hotend
and the platform when the printer is at its home position.
If EEPROM is enabled these values will be overidden with the values in the EEPROM */
#if NUM_EXTRUDER == 2
#define X_MAX_LENGTH_PRINT X_MAX_LENGTH_DUAL // [mm]
#else
#define X_MAX_LENGTH_PRINT 230 // [mm]
#endif                         // NUM_EXTRUDER == 2

/** \brief Drive settings for printers with cartesian drive systems */
/** \brief Number of steps for a 1mm move in x direction.
For xy gantry use 2*belt moved!
Overridden if EEPROM activated. */
#define XAXIS_STEPS_PER_MM float(4.761875 * (float)RF_MICRO_STEPS_XY)

/** \brief Number of steps for a 1mm move in y direction.
For xy gantry use 2*belt moved!
Overridden if EEPROM activated.*/
#define YAXIS_STEPS_PER_MM float(4.761875 * (float)RF_MICRO_STEPS_XY)

/** \brief Number of steps for a 1mm move in z direction  Overridden if EEPROM activated.*/
#define ZAXIS_STEPS_PER_MM float(80 * (float)RF_MICRO_STEPS_Z)

// ##########################################################################################
// ##   Configuration of the 1. extruder
// ##########################################################################################

#define EXT0_X_OFFSET_MM 0.0f
#define EXT0_Y_OFFSET_MM 0.0f

/** \brief Analog input pin for measuring hotend temperature */
#define EXT0_TEMPSENSOR_PIN TEMP_0_PIN

/** \brief Which pin enables the heater */
#define EXT0_HEATER_PIN HEATER_0_PIN
#define EXT0_STEP_PIN E0_STEP_PIN
#define EXT0_DIR_PIN E0_DIR_PIN

/** \brief set to 0/1 for normal/inverse direction */
#define EXT0_INVERSE false
#define EXT0_ENABLE_PIN E0_ENABLE_PIN

/** \brief For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1 */
#define EXT0_ENABLE_ON true

/** \brief The following speed settings are for skeinforge 40+ where e is the
length of filament pulled inside the heater. For repsnap or older
skeinforge use higher values.
Overridden if EEPROM activated. */
#define EXT0_MAX_FEEDRATE 25

/** \brief Feedrate from halted extruder in mm/s
Overridden if EEPROM activated. */
#define EXT0_MAX_START_FEEDRATE 12

/** \brief Acceleration in mm/s^2
Overridden if EEPROM activated. */
#define EXT0_MAX_ACCELERATION 6000

/** \brief Faktor for the advance algorithm. 0 disables the algorithm.  Overridden if EEPROM activated.
L is the linear factor and seems to be working better then the quadratic dependency. */
#define EXT0_ADVANCE_L 0.0f

/** \brief PWM speed for the cooler fan. 0=off 255=full speed */
#define EXT0_EXTRUDER_COOLER_SPEED 255

#if NUM_EXTRUDER == 2

// ##########################################################################################
// ##   Configuration of the 2. extruder
// ##########################################################################################

/** \brief Analog input pin for measuring hotend temperature */
#define EXT1_TEMPSENSOR_PIN TEMP_1_PIN

/** \brief Which pin enables the heater */
#define EXT1_HEATER_PIN HEATER_1_PIN
#define EXT1_STEP_PIN E1_STEP_PIN
#define EXT1_DIR_PIN E1_DIR_PIN

/** \brief set to 0/1 for normal/inverse direction */
#define EXT1_INVERSE true
#define EXT1_ENABLE_PIN E1_ENABLE_PIN

/** \brief For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1 */
#define EXT1_ENABLE_ON true

/** \brief The following speed settings are for skeinforge 40+ where e is the
length of filament pulled inside the heater. For repsnap or older
skeinforge use heigher values.
Overridden if EEPROM activated. */
#define EXT1_MAX_FEEDRATE 25

/** \brief Feedrate from halted extruder in mm/s
Overridden if EEPROM activated. */
#define EXT1_MAX_START_FEEDRATE 12

/** \brief Acceleration in mm/s^2
Overridden if EEPROM activated. */
#define EXT1_MAX_ACCELERATION 6000

/** \brief Faktor for the advance algorithm. 0 disables the algorithm.  Overridden if EEPROM activated.
L is the linear factor and seems to be working better then the quadratic dependency. */
#define EXT1_ADVANCE_L 0.0f

/** \brief Motor steps to remove backlash for advance alorithm. These are the steps
needed to move the motor cog in reverse direction until it hits the driving
cog. Direct drive extruder need 0. */
/** \brief PWM speed for the cooler fan. 0=off 255=full speed */
#define EXT1_EXTRUDER_COOLER_SPEED 255
#endif // NUM_EXTRUDER == 2

// ##########################################################################################
// ##   Configuration of the heated bed
// ##########################################################################################

/** \brief Maximal temperature which can be set for the heating bed */
#define HEATED_BED_MAX_TEMP 180

/** \brief Select type of your heated bed. Uses the same sensor types as EXT0_TEMPSENSOR_TYPE */
#define HEATED_BED_SENSOR_TYPE 3

/** \brief Analog pin of analog sensor to read temperature of heated bed.  */
#define HEATED_BED_SENSOR_PIN TEMP_2_PIN

/** \brief Pin to enable heater for bed. */
#define HEATED_BED_HEATER_PIN HEATER_2_PIN

/** \brief The maximum value, I-gain can contribute to the output.
The precise values may differ for different nozzle/resistor combination.
 Overridden if EEPROM activated. */
#define HEATED_BED_PID_INTEGRAL_DRIVE_MAX 80

/** \brief lower value for integral part
The I state should converge to the exact heater output needed for the target temperature.
To prevent a long deviation from the target zone, this value limits the lower value.
A good start is 30 lower then the optimal value. You need to leave room for cooling.
 Overridden if EEPROM activated. */
#define HEATED_BED_PID_INTEGRAL_DRIVE_MIN 5

/** \brief maximum time the heater can be switched on. Max = 255.  Overridden if EEPROM activated.*/
#define HEATED_BED_PID_MAX 255

/** \brief Extreme values to detect defect thermistors. */
#define MIN_DEFECT_TEMPERATURE -10
#define MAX_DEFECT_TEMPERATURE 300

// Temperature offsets at sensor readings of 60, 80, 100, 120, 140 and 160 °C.
#define SENSOR_TEMPERATURE_OFFSETS_C { -2, -4, -6, -8, -10, -10 }

// ##########################################################################################
// ##   Configuration of the 4. temperature sensor
// ##########################################################################################

// Sensor type at X35 (NTC 3950 100k by default).
#define RESERVE_ANALOG_SENSOR_TYPE 14

// ##########################################################################################
// ##   Configuration of the endstops
// ##########################################################################################

/*
Known Limits for Z_ENDSTOP_DRIVE_OVER to avoid when extending value:
Z-EndStop button crash RF1000: ~ >0.8f
Z-EndStop optical button crash RF2000: ~ >1.3f
Crash with Einhausung/Plexiglas backside RFx000: ~ >5.0 .. 6.0f
Crash with backside metal RFx000: ~ >10.0..12.0f
Overflow in Z-Matrix: >12.7f
*/

/** \brief If during homing the endstop is reached, how many mm should the printer move back for the second try */
#define ENDSTOP_X_BACK_MOVE 5.0f
#define ENDSTOP_Y_BACK_MOVE 5.0f
#define ENDSTOP_Z_BACK_MOVE float(Z_ENDSTOP_MAX_HYSTERESIS + Z_ENDSTOP_DRIVE_OVER) //0.1mm sind theoretische maximale hysterese beim Schalter loslassen. Original RF2000: <0.01

/** \brief For higher precision you can reduce the speed for the second test on the endstop
during homing operation. The homing speed is divided by the value. 1 = same speed, 2 = half speed */
#define ENDSTOP_X_RETEST_REDUCTION_FACTOR 20
#define ENDSTOP_Y_RETEST_REDUCTION_FACTOR 20
#define ENDSTOP_Z_RETEST_REDUCTION_FACTOR 20

// ##########################################################################################
// ##   Motor Current & Stepper configurations
// ##########################################################################################

/** \brief Motor Current MAX setting */
#define MOTOR_CURRENT_MAX \
    { 145, 140, 120, 105, 105 } // Values 0-255 (126 = ~2A), order: driver 1 (x), driver 2 (y), driver 3 (z), driver 4 (extruder 1), driver 5 (extruder 2)

/** \brief Motor Current settings at start: Tweak with menu for better silence <-> stability
// The RF2000 has one more stepper and the same 8A-24V power supply as RF1000. We think that this is the reason for lower stepper currents set by conrad renkforce - but dont know.
// I increased MAX a little bit over stock settings, but decreased normal settings to fairly low power and noise. Take that into account when tuning in your steppers according your needs.
*/
#define MOTOR_CURRENT_NORMAL \
    { 110, 110, 105, 90, 90 }
#define MOTOR_CURRENT_MIN EXTRUDER_CURRENT_PAUSED

// ##########################################################################################
// ##   miscellaneous configurations
// ##########################################################################################

/** \brief For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1 */
#define X_ENABLE_ON 1
#define Y_ENABLE_ON 1
#define Z_ENABLE_ON 1

/** \brief Inverting axis direction */
#define INVERT_X_DIR true
#define INVERT_Y_DIR false
#define INVERT_Z_DIR false

#define XYZ_STEPPER_HIGH_DELAY 100 // [us] speed for moveZ
#define XYZ_STEPPER_LOW_DELAY 100  // [us] speed for moveZ

// ##########################################################################################
// ##   Script configuration
// ##########################################################################################

/** \brief Automatic filament change, unmounting of the filament - ensure that G1 does not attempt to extrude more than EXTRUDE_MAXLENGTH */
#define UNMOUNT_FILAMENT_SCRIPT_SOFT "M3914 P4500"
#define UNMOUNT_FILAMENT_SCRIPT_HARD "M109 S" xstr(UI_SET_PRESET_EXTRUDER_TEMP_PLA) "\nG21\nG92 E0\nG1 E-90 F500\nG92 E0\nM104 S0"

/** \brief Automatic filament mount, mounting of the filament with heating. Stop @ XXXX digits */
#define MOUNT_FILAMENT_SCRIPT_SOFT "M3913 P3500 F3"
#define MOUNT_FILAMENT_SCRIPT_HARD "M109 S" xstr(UI_SET_PRESET_EXTRUDER_TEMP_ABS) "\nG21\nG92 E0\nG1 E90 F100\nG92 E0\nM104 S0"

// ##########################################################################################
// ##   Part Fan configuration
// ##########################################################################################

/** Some fans won't start for low values, but would run if started with higher power at the beginning.
This defines the full power duration before returning to set value. Time is in milliseconds
Only values which are a factor of 10ms or 0==OFF will work precisely */
#define PART_FAN_KICKSTART_THRESHOLD 128   // 0 ... 255 equals 0 ... 100 % (unscaled!)
#define PART_FAN_KICKSTART_TIME_OFF_ON 400 // [ms]
#define PART_FAN_KICKSTART_TIME_BOOST 100  // [ms]

/** \brief use PDM instead of PWM for part fan */
// --> set EEPROM value or change mode in printers menu

// ##########################################################################################
// ##   configuration of the speed vs. cpu usage
// ##########################################################################################

/**
\brief The firmware can only handle a limited interrupt frequency cleanly.
If you need higher speeds
a faster solution is needed, and this is to double/tripple/quadruple the steps in one interrupt call.
This is like reducing your 1/16th microstepping to 1/8 or 1/4. It is much cheaper then 1 or 3
additional stepper interrupts with all it's overhead. As a result you can gain higher Output-Step-Frequencies.

STEP_PACKING_MIN_INTERVAL can be changed in Menu-> Configuration->Stepper->DblFq:
*/
#define STEP_PACKING_MIN_INTERVAL 3300 // (F_CPU / safe Steprate)

#define MIN_STEP_PACKING_MIN_INTERVAL 3300
#define MAX_STEP_PACKING_MIN_INTERVAL 5000

/** \brief Low filled cache size.
If the cache contains less then MOVE_CACHE_LOW segments, the time per segment is limited to LOW_TICKS_PER_MOVE clock cycles.
If a move would be shorter, the feedrate will be reduced. This should prevent buffer underflows. Set this to 0 if you
don't care about empty buffers during print. */
#define MOVE_CACHE_LOW 10

/** \brief Cycles per move, if move cache is low.
This value must be high enough, that the buffer has time to fill up. The problem only occurs at the beginning of a print or
if you are printing many very short segments at high speed.*/
#define LOW_TICKS_PER_MOVE 250000

// For configuration of speed vs. cpu RF_MICRO_STEPS_ @ CONFIGURATION.h as well!

#define DEFAULT_PAUSE_MM_Z_PRINT 30 // [mm] Lift during print pause

// ##########################################################################################
// ##   configuration of the FET outputs
// ##########################################################################################

/** \brief Set the 24V FET output defaults  */
#define FET1_DEFAULT_ON 0 // 1 = on, 0 = off
#define FET2_DEFAULT_ON 0 // 1 = on, 0 = off
// The case fan uses FET3 on X45. Its startup state is set by the fan configuration.
#define FET3_DEFAULT_ON 0 // 1 = on, 0 = off

// ##########################################################################################
// ##   configuration of the RGB light effects
// ##########################################################################################

/** \brief Specfies the default RGB light mode */
#define RGB_LIGHT_DEFAULT_MODE RGB_MODE_WHITE

/** \brief Specifies the time interval after which the RGB light status switches from idle to color change */
#define RGB_LIGHT_COLOR_CHANGE_DELAY 30 // [s]

/** \brief Specfies the temperature tolerance for the switching of the RGB light */
#define RGB_LIGHT_TEMP_TOLERANCE 5.0 // [°C]

/** \brief Specifies the color for heating up */
#define RGB_HEATING_R 255
#define RGB_HEATING_G 0
#define RGB_HEATING_B 0

/** \brief Specifies the color for printing */
#define RGB_PRINTING_R 255
#define RGB_PRINTING_G 255
#define RGB_PRINTING_B 255

/** \brief Specifies the color for cool down */
#define RGB_COOLING_R 0
#define RGB_COOLING_G 0
#define RGB_COOLING_B 255

/** \brief Specifies the color for idle */
#define RGB_IDLE_R 255
#define RGB_IDLE_G 255
#define RGB_IDLE_B 255

/** \brief Specifies the color for Manual input */
#define RGB_MANUAL_R 128
#define RGB_MANUAL_G 128
#define RGB_MANUAL_B 255

#endif // RF2000_H

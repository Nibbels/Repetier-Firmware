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


#ifndef CONFIGURATION_H
#define CONFIGURATION_H


// ##########################################################################################
// ##   IMPORTANT
// ##########################################################################################

/* Some words on units:
 * Speed           is in   mm/s
 * Acceleration    in in   mm/s^2
 * Temperature     is in   degrees celsius
 * 
 * For easy configuration, the default settings enable parameter storage in EEPROM.
 * This means, after the first upload many variables can only be changed using the special
 * M commands as described in the documentation. Changing these values in the configuration.h
 * file has no effect. Parameters overriden by EEPROM settings are calibartion values, extruder
 * values except thermistor tables and some other parameter likely to change during usage
 * like advance steps or ops mode.
 * To override EEPROM settings with config settings, set EEPROM_MODE 0
 */

// ##########################################################################################
// ##   main hardware configuration
// ##########################################################################################

/** \brief Define the type of your device */
//#define MOTHERBOARD                         DEVICE_TYPE_RF1000
//#define MOTHERBOARD                         DEVICE_TYPE_RF2000
//#define MOTHERBOARD                         DEVICE_TYPE_RF2000v2

#ifndef MOTHERBOARD
    #error Device type (RF1000 / RF2000 / RF2000v2) is not defined. Edit Configuration.h or pass the corresponding option to the compiler.
#endif
#if MOTHERBOARD == DEVICE_TYPE_RF2000v2
    #error This Mod-Firmware has not been massively tested on a RF2000v2 yet. But the firmware has been created with care. Please remove this message if you acknowledged this.
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000v2

/**
 * \brief EEPROM storage mode
 * Set the EEPROM_MODE to 0 if you always want to use the settings in this configuration file. If not,
 * set it to a value not stored in the first EEPROM-byte used. If you later want to overwrite your current
 * EEPROM settings with configuration defaults, just select an other value. On the first call to epr_init()
 * it will detect a mismatch of the first byte and copy default values into EEPROM. If the first byte
 * matches, the stored values are used to overwrite the settings.
 * 
 * IMPORTANT: With mode <>0 some changes in Configuration.h are not set any more, as they are
 *            taken from the EEPROM.
 */
#define EEPROM_MODE                         44

// ##########################################################################################
// ##    supported features
// ##########################################################################################

/** \brief Auto-Retract within hardcoded scripts: Pause / Output_Object / ... Vom Hotend abhängig! V2: 10mm, E3D: 1-2mm (?) */
#define SCRIPT_RETRACT_MM                   1                                                    //[mm] Firmwares E-Retract */

// Default SenseOffset limit in µm and sampling interval in ms.
#define SENSIBLE_PRESSURE_MAX_OFFSET        180
#define SENSIBLE_PRESSURE_INTERVAL          50

/** \brief Enables debug outputs from the heat bed scan */
#define DEBUG_HEAT_BED_SCAN                 0                                                   // 0 = off, 1 = on, 2 = on with more debug outputs

/**
 * XYZ_POSITION_BUTTON_DIRECTION = -1 : This fits to you if you want more intuitivity when choosing the Up-Down-Buttons within "Menu:Position->X-/Y-/Z-Position".
 * XYZ_POSITION_BUTTON_DIRECTION = 1 : This fits more if you want to stick to standard coordinates direction.
 */
#define XYZ_POSITION_BUTTON_DIRECTION       1                                                   // 1 = on, 0 = off

/** \brief If a checksum is sent, all future comamnds must also contain a checksum. Increases reliability especially for binary protocol. */
#define FEATURE_CHECKSUM_FORCED             0                                                   // 1 = on, 0 = off

// ##########################################################################################
// ##   Common extruder configuration
// ##########################################################################################

/** \brief for each extruder, fan will stay on until extruder temperature is below this value */
#define EXTRUDER_FAN_COOL_TEMP              50

/** \brief Maximal temperature which can be set for the extruder */
#define EXTRUDER_MAX_TEMP                   275

// ##########################################################################################
// ##   Hotend V1
// ##########################################################################################

/** \brief The maximum value, I-gain can contribute to the output. */
#define HT2_PID_INTEGRAL_DRIVE_MAX          130
/** \brief lower value for integral part. */
#define HT2_PID_INTEGRAL_DRIVE_MIN          30
/** \brief P-gain. */
#define HT2_PID_P                           37.52
/** \brief I-gain. */
#define HT2_PID_I                           10
/** \brief Dgain. */
#define HT2_PID_D                           35.18

// ##########################################################################################
// ##   Hotend V2
// ##########################################################################################

/** \brief The maximum value, I-gain can contribute to the output. */
#define HT3_PID_INTEGRAL_DRIVE_MAX          120
/** \brief lower value for integral part. */
#define HT3_PID_INTEGRAL_DRIVE_MIN          30
/** \brief P-gain. */
#define HT3_PID_P                           12.5
/** \brief I-gain. */
#define HT3_PID_I                           3.2
/** \brief Dgain. */
#define HT3_PID_D                           18

// ##########################################################################################
// ##	Hotend V3
// ##########################################################################################

/** \brief The maximum value, I-gain can contribute to the output. */
#define HT4_PID_INTEGRAL_DRIVE_MAX          120
/** \brief lower value for integral part. */
#define HT4_PID_INTEGRAL_DRIVE_MIN          30
/** \brief P-gain. */
#define HT4_PID_P                           17.11
/** \brief I-gain. */
#define HT4_PID_I                           2.05
/** \brief Dgain. */
#define HT4_PID_D                           35.75

// ##########################################################################################
// ##   common configuration
// ##########################################################################################

/**
 * \brief Define the to-be-used micro steps.
 * Note that high MICRO_STEPS limit your speed because of the limit in 8bit-CPU calculation power.
 * See "configuration of the speed vs. cpu usage" within RF1000.h / RF2000.h
 * 
 * Your way to high microsteps is dual-stepping, quadstepping, octostepping. On high speeds that packs steps together to save CPU. Read rf1000.de for the downside. Menu->Stepper->DblFQ: is the setting for the shifting speed.
 * High microstepping make your speed low. Dont use 256, avoid 128.
 * 256 Microsteps sound very bad and is slower 30mm/s in X and Y. Avoid it! 128 is not much better. 64 might be ok for XY.
 * Better dont rise Z because it is already slow with 2560steps/mm @32 Microsteps.
 * Change your EEPROMs Steps/mm accordingly if you change RF_MICRO_STEPS_ in this configuration. The Stepper menu adjusts steps/mm when you change the microstep setting there.
 */

#define RF_MICRO_STEPS_Z                    16                                                   // standard/best 32 or 16
#define RF_MICRO_STEPS_XY                   32                                                   // standard/best 32 or 64
#define RF_MICRO_STEPS_E                    32                                                   // standard/best 32 or 64 (or 128?? --> untested!)

// ##########################################################################################
// ##   Feedrate and Movement settings
// ##########################################################################################

/** \brief After x seconds of inactivity, the stepper motors are disabled.
Set to 0 to leave them enabled.
This helps cooling the Stepper motors between two print jobs.
Overridden if EEPROM activated. */
#define STEPPER_INACTIVE_TIME               600

/** \brief After x seconds of inactivity, the system will go down as far it can.
It will at least disable all stepper motors and heaters. If the board has
a power pin, it will be disabled, too.
Set value to 0 for disabled.
Overridden if EEPROM activated. */
#define MAX_INACTIVE_TIME                   0L

/** \brief Maximum feedrate, the system allows. Higher feedrates are reduced to these values.
The axis order in all axis related arrays is X, Y, Z
Overridden if EEPROM activated. */
#define MAX_FEEDRATE_X                    150
#define MAX_FEEDRATE_Y                    150
#define MAX_FEEDRATE_Z                      9

/** \brief Home position speed in mm/s. Overridden if EEPROM activated. These values can be overridden by EEPROM but are considered as maximum allowed values */
#define HOMING_FEEDRATE_X_PRINT            80
#define HOMING_FEEDRATE_Y_PRINT            80
#define HOMING_FEEDRATE_Z_PRINT             9


/** \brief Speed for direct movements in mm/s. Overridden if EEPROM activated. */
#define STANDARD_POSITION_FEEDRATE_XY     100
#define STANDARD_POSITION_FEEDRATE_Z        9
#define STANDARD_POSITION_FEEDRATE_E       25

// ##########################################################################################
// ##   Acceleration settings
// ##########################################################################################

// RF2000: Tests haben gezeigt, dass x-y-acceleration unter 2000 oder unter 1500 das Teil ziemlich gut aussieht.

/** \brief X, Y, Z max acceleration in mm/s^2 for printing moves or retracts. Make sure your printer can go that high!
Overridden if EEPROM activated. */
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X                  1500
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y                  1500
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z                  100

/** \brief X, Y, Z max acceleration in mm/s^2 for travel moves.  Overridden if EEPROM activated. */
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X           1500
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y           1500
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z           100

/** \brief X, Y, Z acceleration limits */
#define ACCELERATION_MAX_XY										6000
#define ACCELERATION_MIN_XY										100
#define ACCELERATION_MAX_Z                                      100
#define ACCELERATION_MIN_Z                                      5

/** \brief X, Y, Z acceleration menu change step size */
#define ACCELERATION_MENU_CHANGE_XY								100
#define ACCELERATION_MENU_CHANGE_Z                              5

#if ACCELERATION_MAX_XY < MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X || MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X < ACCELERATION_MIN_XY
#error Please respect the given acceleration limits or change them according your needs within Configuration.h
#endif // NUM_EXTRUDER > 2 || NUM_EXTRUDER < 0
#if ACCELERATION_MAX_XY < MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y || MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y < ACCELERATION_MIN_XY
#error Please respect the given acceleration limits or change them according your needs within Configuration.h
#endif // NUM_EXTRUDER > 2 || NUM_EXTRUDER < 0
#if ACCELERATION_MAX_Z < MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z || MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z < ACCELERATION_MAX_Z
#error Please respect the given acceleration limits or change them according your needs within Configuration.h
#endif // NUM_EXTRUDER > 2 || NUM_EXTRUDER < 0
#if ACCELERATION_MAX_XY < MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X || MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X < ACCELERATION_MIN_XY
#error Please respect the given x travel acceleration limits or change them according your needs within Configuration.h
#endif // NUM_EXTRUDER > 2 || NUM_EXTRUDER < 0
#if ACCELERATION_MAX_XY < MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y || MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y < ACCELERATION_MIN_XY
#error Please respect the given y travel acceleration limits or change them according your needs within Configuration.h
#endif // NUM_EXTRUDER > 2 || NUM_EXTRUDER < 0
#if ACCELERATION_MAX_Z < MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z || MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z < ACCELERATION_MAX_Z
#error Please respect the given z travel acceleration limits or change them according your needs within Configuration.h
#endif // NUM_EXTRUDER > 2 || NUM_EXTRUDER < 0

// E-ACCELERATION is extruder configuration and has no strict limits rightnow.

// ##########################################################################################
// ##   Jerk settings
// ##########################################################################################

/** \brief Maximum allowable jerk.
Caution: This is no real jerk in a physical meaning.
The jerk determines your start speed and the maximum speed at the join of two segments.
Its unit is mm/s. If the printer is standing still, the start speed is jerk/2. At the
join of two segments, the speed difference is limited to the jerk value.

Examples:
For all examples jerk is assumed as 40.

Segment 1: vx = 50, vy = 0
Segment 2: vx = 0, vy = 50
v_diff = sqrt((50-0)^2+(0-50)^2) = 70.71
v_diff > jerk => vx_1 = vy_2 = jerk/v_diff*vx_1 = 40/70.71*50 = 28.3 mm/s at the join

Segment 1: vx = 50, vy = 0
Segment 2: vx = 35.36, vy = 35.36
v_diff = sqrt((50-35.36)^2+(0-35.36)^2) = 38.27 < jerk
Corner can be printed with full speed of 50 mm/s

Overridden if EEPROM activated. */
#define MAX_JERK                            13.0                 //std: 20, aber RFx000 sieht zwischen ca. 7 und 18 am besten aus: Renkforce sagt 10
#define MAX_ZJERK                           0.3                  //std: 0.3

//that will slowdown if you have sever direction changes in a short distance which is nearly the same as adding several jerks in a short sequence.
#define REDUCE_ON_SMALL_SEGMENTS            1
#define MAX_JERK_DISTANCE                   0.6
//for a more logical jerk computation.
#define ALTERNATIVE_JERK                    1

// ##########################################################################################
// ##   debugging
// ##########################################################################################

/**
 * \brief Writes the free RAM to output, if it is less then at the last test. Should always return
 * values >500 for safety, since it doesn't catch every function call. Nice to tweak cache
 * usage or for seraching for memory induced errors. Switch it off for production, it costs execution time.
 */
#define DEBUG_FREE_MEMORY                   0                                                   // 1 = on, 0 = off

#if DEBUG_FREE_MEMORY
 #define DEBUG_MEMORY                        Commands::checkFreeMemory();
#else
 #define DEBUG_MEMORY
#endif // DEBUG_FREE_MEMORY

// ##########################################################################################
// ##   configuration of the extended buttons
// ##########################################################################################

#define EXTENDED_BUTTONS_COUNTER_NORMAL     4                                                   // 39 ~ run 100 times per second, 4 ~ run 1000 times per second
#define EXTENDED_BUTTONS_COUNTER_FAST       4                                                   // 39 ~ run 100 times per second, 4 ~ run 1000 times per second
#define EXTENDED_BUTTONS_STEPPER_DELAY      1                                                   // [us]


// ##########################################################################################
// ##   configuration of the output object functionality
// ##########################################################################################

/** \brief The following script allows to configure the exact behavior of the automatic object output */
#define OUTPUT_OBJECT_SCRIPT_PRINT          "G21\nG92 E0\nG1 E-" xstr(SCRIPT_RETRACT_MM) "\nG4 P100\nG92 E0\nG90\nG1 Z200 F540\nG1 Y245 F4800"

// ##########################################################################################
// ##   configuration of the pause functionality
// ##########################################################################################

/** \brief Specifies the time interval after the pausing of the print at which the extruder current is reduced */
#define EXTRUDER_CURRENT_PAUSE_DELAY        60000                                               // [ms] Minimum 30000 -> 30s.

/** \brief Specifies the extruder current which shall be use after pausing of the print and before continuing of the print */
#define EXTRUDER_CURRENT_PAUSED             32                                                  // ~0.5A

// ##########################################################################################
// ##   configuration of the emergency pause functionality
// ##########################################################################################

/**
 * \brief Specifies the pressure at which the emergency pause shall be performed, in [digits]
 * @ ca. +- 15000 the sensors tend to start bending
 * With RF1.37r2.Mod the Emergency-Pause-Features limits can be changed in EEPROM and Printers Menu. Here are the absolute maximum limits:
 */
#define EMERGENCY_PAUSE_DIGITS_MIN          -11000                                 // [digits]
#define EMERGENCY_PAUSE_DIGITS_MAX           11000                                 // [digits]

/** \brief Specifies the interval at which the pressure check shall be performed, in [ms] */
#define EMERGENCY_PAUSE_INTERVAL            100

/** \brief Specifies the number of pressure values which shall be averaged. The emergency pause can be detected each EMERGENCY_PAUSE_INTERVAL * EMERGENCY_PAUSE_CHECKS [ms] */
#define EMERGENCY_PAUSE_CHECKS              10

// ##########################################################################################
// ##   configuration of the emergency stop functionality
// ##########################################################################################

/**
 * \brief 
 * Specifies the pressure at which 
 * - the z emergency stop shall be performed
 * - extrusion is forbidden
 * 
 * Do not set them to Zero.
 */
#define EMERGENCY_STOP_DIGITS_MIN           -12500                                 // [digits]
#define EMERGENCY_STOP_DIGITS_MAX            12500                                 // [digits]

/** \brief Specifies the interval at which the pressure check shall be performed, in [ms] */
#define EMERGENCY_STOP_INTERVAL             10

/** \brief Specifies the number of pressure values which shall be averaged. The emergency stop can be detected each EMERGENCY_STOP_INTERVAL * EMERGENCY_STOP_CHECKS [ms] */
#define EMERGENCY_STOP_CHECKS               3

// ##########################################################################################
// ##   configuration of user-defined thermistor tables
// ##########################################################################################
/**
 * There are many different thermistors, which can be combined with different resistors. This result
 * in unpredictable number of tables. As a resolution, the user can define one table here, that can
 * be used as type 5 for thermister type in extruder/heated bed definition. Make sure, the number of entries
 * matches the value in NUM_TEMPS_USERTHERMISTOR0. If you span definition over multiple lines, make sure to end
 * each line, except the last, with a backslash. The table format is {{adc1,temp1},{adc2,temp2}...} with
 * increasing adc values. For more informations, read
 * http://hydraraptor.blogspot.com/2007/10/measuring-temperature-easy-way.html
 * 
 * If you have a sprinter temperature table, you have to multiply the first value with 4 and the second with 8.
 * This firmware works with increased precision, so the value reads go from 0 to 4095 and the temperature is
 * temperature*8.
 * 
 * If you have a PTC thermistor instead of a NTC thermistor, keep the adc values increasing and use themistor types 50-52 instead of 5-7!
 */

/** \brief Number of entries in the user thermistor table 0. Set to 0 to disable it. */
#define NUM_TEMPS_USERTHERMISTOR0           28
#define USER_THERMISTORTABLE0  {\
  {1*4,864*8},{21*4,280*8},{25*4,270*8},{29*4,260*8},{33*4,250*8},{39*4,240*8},{46*4,230*8},{54*4,220*8},{64*4,210*8},{75*4,200*8},\
  {90*4,190*8},{107*4,180*8},{128*4,170*8},{154*4,160*8},{184*4,150*8},{221*4,140*8},{265*4,130*8},{316*4,120*8},{375*4,110*8},\
  {441*4,105*8},{513*4,100*8},{588*4,100*8},{734*4,80*8},{856*4,60*8},{938*4,40*8},{986*4,20*8},{1008*4,0*8},{1018*4,-20*8} }

/** \brief Number of entries in the user thermistor table 1. Set to 0 to disable it. */
#define NUM_TEMPS_USERTHERMISTOR1           0
#define USER_THERMISTORTABLE1               {}

/** \brief Number of entries in the user thermistor table 2. Set to 0 to disable it. */
#define NUM_TEMPS_USERTHERMISTOR2           0
#define USER_THERMISTORTABLE2               {}

/**
 * \brief If defined, creates a thermistor table at startup.
 * 
 * If you don't feel like computing the table on your own, you can use this generic method. It is
 * a simple approximation which may be not as accurate as a good table computed from the reference
 * values in the datasheet. You can increase precision if you use a temperature/resistance for
 * R0/T0, which is near your operating temperature. This will reduce precision for lower temperatures,
 * which are not realy important. The resistors must fit the following schematic:
 * @code
 * VREF ---- R2 ---+--- Termistor ---+-- GND
 *                 |                 |
 *                 +------ R1 -------+
 *                 |                 |
 *                 +---- Capacitor --+
 *                 |
 *                 V measured
 * @endcode
 * 
 * If you don't have R1, set it to 0.
 * The capacitor is for reducing noise from long thermistor cable. If you don't have one, it's OK.
 * 
 * If you need the generic table, uncomment the following define.
 */
//#define USE_GENERIC_THERMISTORTABLE_1

/* Some examples for different thermistors:

EPCOS B57560G104+ : R0 = 100000  T0 = 25  Beta = 4036
EPCOS 100K Thermistor (B57560G1104F) :  R0 = 100000  T0 = 25  Beta = 4092
ATC Semitec 104GT-2 : R0 = 100000  T0 = 25  Beta = 4267
Honeywell 100K Thermistor (135-104LAG-J01)  : R0 = 100000  T0 = 25  Beta = 3974

*/

/** \brief Reference Temperature */
#define GENERIC_THERM1_T0                   25

/** \brief Resistance at reference temperature */
#define GENERIC_THERM1_R0                   200000

/**
 * \brief Beta value of thermistor
 * You can use the beta from the datasheet or compute it yourself.
 * See http://reprap.org/wiki/MeasuringThermistorBeta for more details.
 */
#define GENERIC_THERM1_BETA                 8304

/** \brief Start temperature for generated thermistor table */
#define GENERIC_THERM1_MIN_TEMP             -20

/** \brief End Temperature for generated thermistor table */
#define GENERIC_THERM1_MAX_TEMP             300
#define GENERIC_THERM1_R1                   0
#define GENERIC_THERM1_R2                   4700

// The same for table 2 and 3 if needed

/** \brief USE_GENERIC_THERMISTORTABLE_2 */
#define GENERIC_THERM2_T0                   170
#define GENERIC_THERM2_R0                   1042.7f
#define GENERIC_THERM2_BETA                 4036
#define GENERIC_THERM2_MIN_TEMP             -20
#define GENERIC_THERM2_MAX_TEMP             300
#define GENERIC_THERM2_R1                   0
#define GENERIC_THERM2_R2                   4700

/** \brief USE_GENERIC_THERMISTORTABLE_3 */
#define GENERIC_THERM3_T0                   170
#define GENERIC_THERM3_R0                   1042.7f
#define GENERIC_THERM3_BETA                 4036
#define GENERIC_THERM3_MIN_TEMP             -20
#define GENERIC_THERM3_MAX_TEMP             300
#define GENERIC_THERM3_R1                   0
#define GENERIC_THERM3_R2                   4700

/** \brief Supply voltage to ADC, can be changed by setting ANALOG_REF below to different value. */
#define GENERIC_THERM_VREF                  5

/** 
 * \brief Number of entries in generated table. One entry takes 4 bytes. Higher number of entries increase computation time too.
 * Value is used for all generic tables created. 
 */
#define GENERIC_THERM_NUM_ENTRIES           33

/** \brief If enabled, writes the created generic table to serial port at startup. */
//#define PRINT_GENERIC_TEMP_TABLE

// ##########################################################################################
// ##   sensor heater decoupling
// ##########################################################################################

/** 
 * \brief The firmware checks if the heater and sensor got decoupled, which is dangerous. Since it will never reach target
 * temperature, the heater will stay on for every which can burn your printer or house.
 * As an additional barrier to your smoke detectors (I hope you have one above your printer) we now
 * do some more checks to detect if something got wrong. 
 */

// If the temp. is on hold target, it may not sway more then this degrees celsius, or we mark sensor as defect.
#define DECOUPLING_TEST_MAX_HOLD_VARIANCE         20                   //°K (+-)

// Minimum temp. rise we expect after the set duration of full heating is over.
// Always keep a good safety margin to get no false positives. If your period is e.g. 10 seconds
// because at startup you already need 7 seconds until heater starts to rise temp. for sensor
// then you have 3 seconds of increased heating to reach 1°C.
#define DECOUPLING_TEST_MIN_TEMP_RISE              1                   //°C

// Time in ms between a heater action and test of success. Must be more then time between turning heater on and first temp. rise! 
// 0 will disable decoupling test 
#define EXT0_DECOUPLE_TEST_PERIOD                  12000
#define EXT1_DECOUPLE_TEST_PERIOD                  12000
#define BED_DECOUPLE_TEST_PERIOD                   12000

// ##########################################################################################
// ##   configure the SD Card
// ##########################################################################################

/** \brief  Select whether the SD card is supported. */
#define SDSUPPORT                           1                                                   // 1 = supported, 0 = not supported

/** \brief  Change to true if you get a inserted message on removal. */
#define SDCARDDETECTINVERTED                false

/** \brief Show extended directory including file length. Don't use this with Pronterface! */
#define SD_EXTENDED_DIR                     true

/** \brief Total size of the buffer used to store the long filenames */
// If you need more maximum file name length than 26 chars 
// - For SD-Printing using a Gcode / using Octoprints sd file listing
// - Or for seeing the full filenames within the SD-Card file menu
// then increase MAX_VFAT_ENTRIES to 3. But that has to be payed with ~65bytes of Ram.
// (Printing a file using the SD card menu will start the print using the file number, so you dont need to see the full file length.)
#define MAX_VFAT_ENTRIES                    (2)
#define LONG_FILENAME_LENGTH                (13*MAX_VFAT_ENTRIES+1)
#define SD_MAX_FOLDER_DEPTH                 2


// ##########################################################################################
// ##   configuration of the manual steps
// ##########################################################################################

/** \brief Configuration of the manual steps */
// (Jeder dieser Werte*steps/mm der Achse muss in unsigned short passen)
#define DEFAULT_MANUAL_MM_X                    0.1f                            // [mm]
#define DEFAULT_MANUAL_MM_Y                    0.1f                            // [mm]
#define DEFAULT_MANUAL_MM_Z                    0.01f                           // [mm] -> Wert im Menü->Position->Z-Steps: xxx um!
#define DEFAULT_MANUAL_MM_E                    0.25f                           // [mm]

//Das hier drunter sind einigermaßen sinnvolle Stepsizes, wenn man Microsteps = 32 eingestellt hat, bei 16 verdoppeln sich die Zahlenwerte in mm.
//Siehe: https://github.com/RF1000community/Repetier-Firmware/issues/4
//Dieser statische Ansatz wird evtl. mal umgebaut. Man könnte auch eine Funktion schreiben, die sinnvolle Einstellwerte automatisch anhand Microsteps und Mikrometertabelle sucht.
#define NUM_ACCEPTABLE_STEP_SIZE_TABLE    7
// (Jeder dieser Werte muss in unsigned short passen)
#define ACCEPTABLE_STEP_SIZE_TABLE { 5,13,26,51,64,128,256 }

// ###############################################################################
// ##   Values for menu settings
// ###############################################################################

/** 
 * \brief Select the language to use.
 * 0 = English
 * 1 = German 
 */
#define UI_LANGUAGE                         0

/** \brief How many ms should a single page be shown, until it is switched to the next one.*/
#define UI_PAGES_DURATION                   4000

/** 
 * \brief Uncomment if you don't want automatic page switching. You can still switch the
 * info pages with next/previous button/click-encoder
 */
#define UI_DISABLE_AUTO_PAGESWITCH          true

/** \brief Time to return to info menu if x millisconds no key was pressed. Set to 0 to disable it. */
#define UI_PRINT_AUTORETURN_TO_MENU_AFTER   120000

/**
 * \brief Normally cou want a next/previous actions with every click of your encoder.
 * Unfotunately, the encoder have a different count of phase changes between clicks.
 * Select an encoder speed from 0 = fastest to 2 = slowest that results in one menu move per click.
 */
#define UI_ENCODER_SPEED                    1

/** \brief bounce time of keys in milliseconds */
#define UI_KEY_BOUNCETIME                   10

/** \brief First time in ms until repeat of action. */
#define UI_KEY_FIRST_REPEAT                 500

/** \brief Reduction of repeat time until next execution. */
#define UI_KEY_REDUCE_REPEAT                50

/** \brief Lowest repeat time. */
#define UI_KEY_MIN_REPEAT                   50

/** \brief Values used for preheat */
#define UI_SET_PRESET_HEATED_BED_TEMP_PLA   60
#define UI_SET_PRESET_EXTRUDER_TEMP_PLA     180
#define UI_SET_PRESET_HEATED_BED_TEMP_ABS   100
#define UI_SET_PRESET_EXTRUDER_TEMP_ABS     210

/** \brief Values used for unload(unmount)/load(mount) filament */
#define UI_SET_EXTRUDER_MIN_TEMP_UNMOUNT     90                                                 //hat anfangs hohe toleranz.
#define UI_SET_EXTRUDER_MAX_TEMP_UNMOUNT    240
#define UI_SET_EXTRUDER_TEMP_MOUNT          210

/** \brief Extreme values */
#define UI_SET_MIN_HEATED_BED_TEMP          50
#define UI_SET_MAX_HEATED_BED_TEMP          160
#define UI_SET_MIN_EXTRUDER_TEMP            70
#define UI_SET_MAX_EXTRUDER_TEMP            270
#define UI_SET_EXTRUDER_FEEDRATE            1.5f                                                 // [mm/sec]
#define UI_SET_EXTRUDER_RETRACT_DISTANCE    3                                                   // [mm]
#define COOLDOWN_THRESHOLD                  40                                                  // [°C]

#define SPEED_MIN_MILLIS                    300
#define SPEED_MAX_MILLIS                    50
#define SPEED_MAGNIFICATION                 100.0f

/**
 * \brief Specifies if you want to see the TipDown Support within Z-Configuration Menu. 
 * This is only usefull if you have a dual hotend with possible servo or spring z-offset-shift.
 */
#define UI_SHOW_TIPDOWN_IN_ZCONFIGURATION   0                                                   // 1 = show, 0 = hide


// ##########################################################################################
// ##   general external EEPROM configuration
// ##########################################################################################

#define EEPROM_DELAY                        10                                                  // [ms]


// ##########################################################################################
// ##   external EEPROM which is used for the z-compensation (32.768 bytes)
// ##########################################################################################
/*
Each sector is 1536 bytes.

Sector 0 holds the EEPROM format at byte 0 and the active heat-bed sector at byte 4.
Bytes 2 and 3 and sectors starting at sector 10 are reserved for legacy data.

Sectors 1 through 9 hold heat-bed matrices. Each starts with the format at byte 0,
the X and Y dimensions at bytes 2 and 4, and the microstep setting at byte 6.
Bytes 8 through 27 are reserved. The matrix starts at byte 28.
*/

#define EEPROM_OFFSET_SECTOR_FORMAT                 0
#define EEPROM_OFFSET_DIMENSION_X                   2
#define EEPROM_OFFSET_DIMENSION_Y                   4
#define EEPROM_OFFSET_MICRO_STEPS                   6
#define EEPROM_OFFSET_MATRIX_START                  28

#define EEPROM_SECTOR_SIZE                          1536                                        // [bytes]
#define EEPROM_MAX_HEAT_BED_SECTORS                 9

#define EEPROM_OFFSET_HEADER_FORMAT                 0                                           // [bytes]
#define EEPROM_OFFSET_ACTIVE_HEAT_BED_Z_MATRIX      4                                           // [bytes]

#define EEPROM_FORMAT                               7


// ##########################################################################################
// ##   external EEPROM which is used for the type information (32.768 bytes)
// ##########################################################################################
/*
we use blocks of 2 kByte size for the structure of our EEPROM

00000 ... 02047 bytes [2 kBytes] = general information
  00000 [2 bytes] EEPROM format
  00002 [2 bytes] board type
*/

#define TYPE_EEPROM_OFFSET_HEADER_FORMAT    0                                                   // [bytes]
#define TYPE_EEPROM_OFFSET_BOARD_TYPE       2                                                   // [bytes]

#define TYPE_EEPROM_FORMAT                  1


// ##########################################################################################
// ##   miscellaneous configurations
// ##########################################################################################

/** \brief Defines the Z-Offset stepsize interval for the Menu-Action in [um] */
#define Z_OFFSET_MENU_STEPS                 25
/** \brief Allows to change the amount of Z-Offset which is changed by a push of the Z-Up or Z-Down button ONLY within the Mod Menu Page 2 */
#define Z_OFFSET_BUTTON_STEPS               5

/** \brief Defines the default z scale */
#define DEFAULT_Z_SCALE_MODE                Z_VALUE_MODE_LAYER

/** \brief Minimal temperature which can be reached by cooling */
#define MAX_ROOM_TEMPERATURE                40                                                  // [°C]

/** \brief Defines the I2C address for the strain gauge */
#define I2C_ADDRESS_STRAIN_GAUGE            0x49

/** \brief Defines which strain gauge is used for the heat bed scan */
#define ACTIVE_STRAIN_GAUGE                 0x49

/** \brief Defines the I2C address for the external EEPROM which stores the z-compensation matrix */
#define I2C_ADDRESS_EXTERNAL_EEPROM         0x50

/** \brief Defines the I2C address for the external EEPROM which stores type information */
#define I2C_ADDRESS_TYPE_EEPROM             0x51

/**
 * \brief Allows to use this firmware together with the non-Repetier PC applications
 * Without this special handling, the firmware may complain about checksum errors from non-Repetier PC applications (e.g. Cura, ...) and
 * non-Repetier PC applications may fall over the debug outputs of the firmware.
 */
#define ALLOW_EXTENDED_COMMUNICATION        2                                                   // 0 = do not allow, 1 = allow "Wait", 2 = allow "Wait" and debug outputs

/** 
 * \brief Configuration of the external watchdog
 * The TPS3820 of the RF1000/RF2000 resets about 112/200/310 (min/typical/max) ms after the last time when it was triggered
 * http://pdf1.alldatasheet.com/datasheet-pdf/view/29215/TI/TPS3820-50DBVT.html
 * t_d in datasheet is delay time: how long reset is triggered after timeout: 15...25...37ms for TPS3820.
 */
#define WATCHDOG_MAIN_LOOP_TIMEOUT          20000UL                                             // [ms] -> uhrzeit intern scheint nicht immer zu stimmen!

/** \brief Longer-lasting operations shall call our periodical actions at least each defined time interval */
#define PERIODICAL_ACTIONS_CALL_INTERVAL    10                                                  // [ms]

/** \brief The display shows that the device is idle after no new commands were processed for longer than the minimal idle time */
#define MINIMAL_IDLE_TIME                   500                                                 // [ms]

/** \brief If enabled you can select the distance your filament gets retracted during a M140 command, after a given temperature is reached. */
#define RETRACT_DURING_HEATUP               true

/**
 * \brief PID control only works target temperature +/- PID_CONTROL_RANGE.
 * If you get much overshoot at the first temperature set, because the heater is going full power too long, you
 * need to increase this value. For one 6.8 Ohm heater 10 is ok. With two 6.8 Ohm heater use 15.
 */
#define PID_CONTROL_RANGE                   30
/** \brief If you change those you might have to do fresh autotunePIDs on your heaters. */
#define PID_CONTROL_DRIVE_MAX_LIMIT_FACTOR  10.0f //this was 10
#define PID_CONTROL_DRIVE_MIN_LIMIT_FACTOR  -1.0f //this was 10 but -1.0 works well with drive max 100 and drive min 5. If this number is negative you get a real PID control, no PD+posI-control anymore.

/**
 * \brief Prevent extrusions longer then x mm for one command. This is especially important if you abort a print. Then the
 * extrusion position might be at any value like 23344. If you then have an G1 E-2 it will roll back 23 meter!
 */
#define EXTRUDE_MAXLENGTH                   100.0f

/**
 * \brief Set PID scaling
 * PID values assume a usable range from 0-255. This can be further limited to EXT0_PID_MAX by two methods.
 * Set the value to 0: Normal computation, just clip output to EXT0_PID_MAX if computed value is too high.
 * Set value to 1: Scale PID by EXT0_PID_MAX/256 and then clip to EXT0_PID_MAX.
 * If your EXT0_PID_MAX is low, you should prefer the second method.
 */
#define SCALE_PID_TO_MAX                    0

/**
 * \brief Temperature range for target temperature to hold in M109 command. 5 means +/-5 degC
 * Uncomment define to force the temperature into the range for given watchperiod.
 */
#define TEMP_TOLERANCE                      2.0f                                               // [°C]

/**
 * \brief Additional special temperature tolerance range when unpausing print.
 * Faster start is better here, because reaching pause position might take a while - for a bit less oozing
 */
#define ADD_CONTINUE_AFTER_PAUSE_TEMP_TOLERANCE         2                                      // [°C]

/** \brief Bits of the ADC converter */
#define ANALOG_INPUT_BITS                   10

/** \brief Build median from 2^ANALOG_INPUT_SAMPLE samples */
#define ANALOG_INPUT_SAMPLE                 5
#define ANALOG_REF_AREF                     0
#define ANALOG_REF_AVCC                     _BV(REFS0)
#define ANALOG_REF_INT_1_1                  _BV(REFS1)
#define ANALOG_REF_INT_2_56                 _BV(REFS0) | _BV(REFS1)
#define ANALOG_REF                          ANALOG_REF_AVCC

/** \brief Communication speed. Overridden if EEPROM activated. */
#define BAUDRATE                            115200

/**
 * \brief Cache size for incoming commands.
 * There should be no reason to increase this cache. Commands are nearly immediately sent to
 * execution.
 */
#define GCODE_BUFFER_SIZE                   2

/**
 * \brief Appends the linenumber after every ok send, to acknowledge the received command.
 * Uncomment for plain ok ACK if your host has problems with this
 */
#define ACK_WITH_LINENUMBER                 1

/**
 * \brief Communication errors can swollow part of the ok, which tells the host software to send
 * the next command. Not receiving it will cause your printer to stop. Sending this string every
 * second, if our queue is empty should prevent this. Comment it, if you don't wan't this feature.
 */
#define WAITING_IDENTIFIER                  "wait"

/**
 * \brief Sets time for echo debug
 * You can set M111 1 which enables ECHO of commands sent. This define specifies the position,
 * when it will be executed. In the original FiveD software, echo is done after receiving the
 * command. With checksum you know, how it looks from the sending string. With this define
 * uncommented, you will see the last command executed. To be more specific: It is written after
 * execution. This helps tracking errors, because there may be 8 or more commands in the queue
 * and it is elsewise difficult to know, what your reprap is currently doing.
 */
#define ECHO_ON_EXECUTE                     1

/**
 * \brief If the firmware is busy, it will send a busy signal to host signaling that
 * everything is fine and it only takes a bit longer to finish. That way the
 * host can keep timeout short so in case of communication errors the resulting
 * blobs are much smaller. Set to 0 to disable it.
 */
#define KEEP_ALIVE_INTERVAL                 2000                                                // [ms]

/** \brief  Turn the case light on/off per default */
#define CASE_LIGHTS_DEFAULT_ON              0

/** \brief Define the on temperature and the off delay for the fan */
#define CASE_FAN_ON_TEMPERATURE             50                                                  // [°C]
#define CASE_FAN_OFF_DELAY                  60000                                               // [ms]

/** \brief Specify whether the case fan all be always on */
#define CASE_FAN_ALWAYS_ON                  0                                                   // 1 = always on, 0 = automatic switching and switching via G-Code

/** \brief Defines the default behavior of the Position X/Y/Z menus */
#define DEFAULT_MOVE_MODE_X                 MOVE_MODE_SINGLE_STEPS
#define DEFAULT_MOVE_MODE_Y                 MOVE_MODE_SINGLE_STEPS
#define DEFAULT_MOVE_MODE_Z                 MOVE_MODE_SINGLE_STEPS

/** \brief Nibbels/PeterKA Testfeature: It can check if you lost steps and test your buttons hysteresis and give a good hint if your printer lost steps by hardware fault */
#define FEATURE_CHECK_HOME                  1

/** 
 * \brief Z-Wobble elimination tool
 * Two offsets [dx, dy] = f(z) in x and y direction can simulate hardware z-wobble.
 * With much care you can adjust this software z-wobble as an opposite to the hardware introduced z-wobble.
 * The underlying principle is the same principle used for noise suppression.
 *
 * This Feature activates a new configuration menu wherin you can adjust parameters to produce the sinodial offset.
 * The simulated wobbles frequency (on z-axis) is always 5mm which fits the spindles thread of 5mm.
 * Within the menu you can configure the amplitude for X, Y-left and Y-right and the shift (phase) within Z.
 *
 * Read here for more information: http://www.rf1000.de/viewtopic.php?f=7&t=2281
 */
#define FEATURE_Kurt67_WOBBLE_FIX           0

#endif // CONFIGURATION_H

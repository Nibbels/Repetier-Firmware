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
Speed           is in   mm/s
Acceleration    in in   mm/s^2
Temperature     is in   degrees celsius

For easy configuration, the default settings enable parameter storage in EEPROM.
This means, after the first upload many variables can only be changed using the special
M commands as described in the documentation. Changing these values in the configuration.h
file has no effect. Parameters overriden by EEPROM settings are calibartion values, extruder
values except thermistor tables and some other parameter likely to change during usage
like advance steps or ops mode.
To override EEPROM settings with config settings, set EEPROM_MODE 0 */


// ##########################################################################################
// ##   main hardware configuration
// ##########################################################################################

/** \brief Define the type of your device */
#define MOTHERBOARD                         DEVICE_TYPE_RF1000   //Digibike Change
//#define MOTHERBOARD                         DEVICE_TYPE_RF2000
//#define MOTHERBOARD                         DEVICE_TYPE_RF2000v2

#ifndef MOTHERBOARD
    #error Device type (RF1000 / RF2000 / RF2000v2) is not defined. Edit Configuration.h or pass the corresponding option to the compiler.
#endif
#if MOTHERBOARD == DEVICE_TYPE_RF2000_V2
    #error This Firmware does not support RF2000v2 yet. Renkforce did not release any validated RF2000V2.h yet.
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000_V2



/** \brief EEPROM storage mode
Set the EEPROM_MODE to 0 if you always want to use the settings in this configuration file. If not,
set it to a value not stored in the first EEPROM-byte used. If you later want to overwrite your current
EEPROM settings with configuration defaults, just select an other value. On the first call to epr_init()
it will detect a mismatch of the first byte and copy default values into EEPROM. If the first byte
matches, the stored values are used to overwrite the settings.

IMPORTANT: With mode <>0 some changes in Configuration.h are not set any more, as they are
           taken from the EEPROM. */
#define EEPROM_MODE                         121


// ##########################################################################################
// ##    supported features
// ##########################################################################################

/** \brief Allows to use 6 additional hardware buttons */
#define FEATURE_EXTENDED_BUTTONS            1                                                   // 1 = on, 0 = off

/** \brief Allows to pause the processing of G-Codes */
#define FEATURE_PAUSE_PRINTING              1                                                   // 1 = on, 0 = off
/** \brief Enables/diables the emergency pause in case of too high pressure ... the emergency pause can be turned on only in case the general pause functionality is available */
#if FEATURE_PAUSE_PRINTING
  #define FEATURE_EMERGENCY_PAUSE           1                                                   // 1 = on, 0 = off
#endif // FEATURE_PAUSE_PRINTING

/** \brief Specifies if you want to see the pressure digits within the repetier-server/repetier-host temperature message */
#define FEATURE_PRINT_PRESSURE              1                                                   // 1 = on, 0 = off

/** \brief Specifies if you want to adjust your average pressure to zero digits after homing. This pushes the weight-scale to zero by adding the idle pressure as an offset. */
#define FEATURE_ZERO_DIGITS                 1

/** \brief Auto-Retract within hardcoded scripts: Pause / Output_Object / ... Vom Hotend abhängig! V2: 10mm, E3D: 1-2mm (?) */
#define SCRIPT_RETRACT_MM                   1                                                   //[mm] Firmwares E-Retract */

/** \brief Enables automatic compensation in z direction for the operationg mode "print" */
#define FEATURE_HEAT_BED_Z_COMPENSATION     1                                                   // 1 = on, 0 = off
/** \brief Enables the precise heat bed scan */
#if FEATURE_HEAT_BED_Z_COMPENSATION
  #define FEATURE_PRECISE_HEAT_BED_SCAN       1                                                 // 1 = on, 0 = off
  #define FEATURE_DIGIT_Z_COMPENSATION           1                                               // 1 = on, 0 = off
  #define FEATURE_DIGIT_FLOW_COMPENSATION        1                                               // 1 = on, 0 = off
  #define FEATURE_SENSIBLE_PRESSURE              1                                               // 1 = on, 0 = off
  #if FEATURE_SENSIBLE_PRESSURE
    // mittels SENSIBLE_PRESSURE soll im grunde ausschließlich die wärmeausdehnung in einem perfekt kalibrierten system (HBS,mhier) kompensiert werden:
    // Max lift in [um]; Standard: 180um=0,18mm, darf nie 0 sein!! größer 0.2 macht normalerweise keinen Sinn.
    // Läuft dieser Wert ins Limit ist die Düse nicht in Ordnung, die Digit-Begrenzung zu niedrig oder das Z-Offset falsch justiert.
    #define SENSIBLE_PRESSURE_MAX_OFFSET                180
    #define SENSIBLE_PRESSURE_INTERVAL                  100                                      //weniger macht keinen sinn. ob diese einschränkung sinn macht, aber sie bleibt vorerst mal da!
  #endif // FEATURE_SENSIBLE_PRESSURE
  /** \brief Enables debug outputs from the heat bed scan */
  #define DEBUG_HEAT_BED_SCAN                 0                                                   // 0 = off, 1 = on, 2 = on with more debug outputs
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

/** \brief The Firmwares disalowes movement before you at least: pressed a printers button, set a temperature, homed once 
If you did not do this, a previous watchdog reset is assumed and fail-drive against some border without homing is blocked thatway. 
This is a fix for repetier-server not knowing that the printer reset and still sending commands
*/
#define FEATURE_UNLOCK_MOVEMENT             1
/**
XYZ_POSITION_BUTTON_DIRECTION = -1 : This fits to you if you want more intuitivity when choosing the Up-Down-Buttons within "Menu:Position->X-/Y-/Z-Position".
XYZ_POSITION_BUTTON_DIRECTION = 1 : This fits more if you want to stick to standard coordinates direction.
*/
#define XYZ_POSITION_BUTTON_DIRECTION       1

/** \brief Enables/disables the emergency stop in case of too high pressure */
#define FEATURE_EMERGENCY_STOP_ALL          1                                                   // 1 = on, 0 = off

/** \brief Enables/disables the set to x/y origin feature */
#define FEATURE_SET_TO_XY_ORIGIN            1                                                   // 1 = on, 0 = off

/** \brief Enables/disables the park feature */
#define FEATURE_PARK                        0                                                   // 1 = on, 0 = off

/** \brief Specifies whether the x, y and z-positions can be changed manually (e.g. via the "Position X/Y/Z" menus or via the hardware buttons) in case the according axis is unknown.
The position of an axis is unknown until the axis has been homed. The position of an axis becomes unknown in case its stepper is disabled.
Enabling of the following feature can be dangerous because it allows to manually drive the printer above its max x/y/z position. */
#define FEATURE_ALLOW_UNKNOWN_POSITIONS     1                                                   // 1 = allow, 0 = do not allow

/** \brief Ditto printing allows 2 extruders to do the same action. This effectively allows
to print an object two times at the speed of one. Works only with dual extruder setup. */
#define FEATURE_DITTO_PRINTING              0                                                   // 1 = on, 0 = off

/** \brief You can store the current position with M401 and go back to it with M402. This works only if feature is set to true. */
#define FEATURE_MEMORY_POSITION             1                                                   // 1 = on, 0 = off

/** \brief If a checksum is sent, all future comamnds must also contain a checksum. Increases reliability especially for binary protocol. */
#define FEATURE_CHECKSUM_FORCED             0                                                   // 1 = on, 0 = off

/** \brief Enables/disables the support for the fan control */
#define FEATURE_FAN_CONTROL                 1                                                   // 1 = on, 0 = off

/** \brief Enables/disables the support for G2/G3 arc commands */
#define FEATURE_ARC_SUPPORT                 1                                                   // 1 = on, 0 = off

/** \brief Enables/disables the beeper */
#define FEATURE_BEEPER                      1                                                   // 1 = on, 0 = off

/** \brief A watchdog resets the printer, if a signal is not send within predifined time limits. That way we can be sure that the board
is always running and is not hung up for some unknown reason. */
#define FEATURE_WATCHDOG                    1                                                   // 1 = on, 0 = off

/** \brief Defines whether a change within the menu shall be stored to the EEPROM automatically or not. */
#define FEATURE_AUTOMATIC_EEPROM_UPDATE     1                                                   // 1 = the EEPROM is updated automatically after each change via the menu, 0 = the EEPROM must be updated manually via the "Store to EEPROM" menu item

/** \brief Allows to use the service interval */
#define FEATURE_SERVICE_INTERVAL            0                                                   // 1 = on, 0 = off

/** \brief Allows to use the case light @ X19 */
#define FEATURE_CASE_LIGHT                  1                                                   // 1 = on, 0 = off

/** \brief Allows to control up to 3 servos
Servos are controlled by a pulse width normally between 500 and 2500 with 1500ms in center position. 0 turns servo off.
WARNING: Servos can draw a considerable amount of current. Make sure your system can handle this or you may risk your hardware! */
#define FEATURE_SERVO                       1                                                   // 1 = on, 0 = off

// ##########################################################################################
// ##   common configuration
// ##########################################################################################

/** \brief Define the to-be-used micro steps.
Note that high MICRO_STEPS limit your speed because of the limit in 8bit-CPU calculation power.
See "configuration of the speed vs. cpu usage" within RF1000.h / RF2000.h

Your way to high microsteps is dual-stepping, quadstepping, octostepping. On high speeds that packs steps together to save CPU. Read rf1000.de for the downside. Menu->Stepper->DblFQ: is the setting for the shifting speed.
High microstepping make your speed low. Dont use 256, avoid 128.
256 Microsteps sound very bad and is slower 30mm/s in X and Y. Avoid it! 128 is not much better. 64 might be ok for XY.
Better dont rise Z because it is already slow with 2560steps/mm @32 Microsteps.
Change your EEPROMs Steps/mm accordingly if you change RF_MICRO_STEPS_ in this configuration. Steps/mm are autoadjusted if you use FEATURE_ADJUSTABLE_MICROSTEPSs Menu!
 */
 
#define RF_MICRO_STEPS_Z                    32                                                   // standard/best 32 or 16
#define RF_MICRO_STEPS_XY                   32                                                   // standard/best 32 or 64
#define RF_MICRO_STEPS_E                    32                                                   // standard/best 32 or 64 (or 128?? --> untested!)

/** \brief Enables/disables that you can switch the micro step setting within Menu->Configuration->Stepper and it is saved in hidden EEPROM */
#define FEATURE_ADJUSTABLE_MICROSTEPS       1

// ##########################################################################################
// ##   debugging
// ##########################################################################################

/** \brief Writes the free RAM to output, if it is less then at the last test. Should always return
values >500 for safety, since it doesn't catch every function call. Nice to tweak cache
usage or for seraching for memory induced errors. Switch it off for production, it costs execution time. */
#define DEBUG_FREE_MEMORY                             1                                         // 1 = on, 0 = off

#if DEBUG_FREE_MEMORY
 #define DEBUG_MEMORY                        Commands::checkFreeMemory();
#else
 #define DEBUG_MEMORY
#endif // DEBUG_FREE_MEMORY

// ##########################################################################################
// ##   configuration of the extended buttons
// ##########################################################################################

#if FEATURE_EXTENDED_BUTTONS

#define EXTENDED_BUTTONS_COUNTER_NORMAL     4                                                   // 39 ~ run 100 times per second, 4 ~ run 1000 times per second
#define EXTENDED_BUTTONS_COUNTER_FAST       4                                                   // 39 ~ run 100 times per second, 4 ~ run 1000 times per second
#define EXTENDED_BUTTONS_STEPPER_DELAY      1                                                   // [us]

#endif // FEATURE_EXTENDED_BUTTONS


// ##########################################################################################
// ##   configuration of the output object functionality
// ##########################################################################################

/** \brief The following script allows to configure the exact behavior of the automatic object output */
#define OUTPUT_OBJECT_SCRIPT_PRINT          "G21\nG92 E0\nG1 E-" xstr(SCRIPT_RETRACT_MM) "\nG92 E0\nG90\nG1 Z200 F720\nG1 Y245 F4800"
#define OUTPUT_OBJECT_SCRIPT_MILL           "G28 Z0\nG21\nG91\nG1 Y245 F4800"

// ##########################################################################################
// ##   configuration of the pause functionality
// ##########################################################################################

#if FEATURE_PAUSE_PRINTING 

#if !FEATURE_HEAT_BED_Z_COMPENSATION && !FEATURE_WORK_PART_Z_COMPENSATION
    #error FEATURE_PAUSE_PRINTING can not be used without FEATURE_HEAT_BED_Z_COMPENSATION or FEATURE_WORK_PART_Z_COMPENSATION
#endif // !FEATURE_HEAT_BED_Z_COMPENSATION && !FEATURE_WORK_PART_Z_COMPENSATION

/** \brief Specifies the time interval after the pausing of the print at which the extruder current is reduced */
#define EXTRUDER_CURRENT_PAUSE_DELAY        60000                                                // [ms] or 0, in order to disable the lowering of the extruder current
#endif // FEATURE_PAUSE_PRINTING

/** \brief Specifies the extruder current which shall be use after pausing of the print and before continuing of the print */
#define EXTRUDER_CURRENT_PAUSED             32                                                  // ~0.5A

// ##########################################################################################
// ##   configuration of the park functionality
// ##########################################################################################

#if FEATURE_PARK

/** \brief Specifies the park position, in [mm] */
#define PARK_POSITION_X                     0                                                  // [mm]
#define PARK_POSITION_Y                     120                                                // [mm]
#define PARK_POSITION_Z                     175                                                // [mm]

#endif // FEATURE_PARK


// ##########################################################################################
// ##   configuration of the emergency pause functionality
// ##########################################################################################

#if FEATURE_EMERGENCY_PAUSE

/** \brief Specifies the pressure at which the emergency pause shall be performed, in [digits] 
@ ca. +- 15000 the sensors tend to start bending
With RF1.37r2.Mod the Emergency-Pause-Features limits can be changed in EEPROM and Printers Menu. Here are the absolute maximum limits:
*/
#define EMERGENCY_PAUSE_DIGITS_MIN          -15000
#define EMERGENCY_PAUSE_DIGITS_MAX          15000

/** \brief Specifies the interval at which the pressure check shall be performed, in [ms] */
#define EMERGENCY_PAUSE_INTERVAL            100

/** \brief Specifies the number of pressure values which shall be averaged. The emergency pause can be detected each EMERGENCY_PAUSE_INTERVAL * EMERGENCY_PAUSE_CHECKS [ms] */
#define EMERGENCY_PAUSE_CHECKS              10

#endif // FEATURE_EMERGENCY_PAUSE


// ##########################################################################################
// ##   configuration of the emergency stop functionality
// ##########################################################################################

#if FEATURE_EMERGENCY_STOP_ALL

/** \brief Specifies the pressure at which the emergency stop shall be performed, in [digits] 
With RF1.37r6.Mod the Emergency-ZStop-Features limits can be changed in EEPROM and Printers Menu. Here are the absolute maximum limits:
Do not set them to Zero.
*/
#define EMERGENCY_STOP_DIGITS_MIN           -10000
#define EMERGENCY_STOP_DIGITS_MAX           10000

/** \brief Specifies the interval at which the pressure check shall be performed, in [ms] */
#define EMERGENCY_STOP_INTERVAL             10

/** \brief Specifies the number of pressure values which shall be averaged. The emergency stop can be detected each EMERGENCY_STOP_INTERVAL * EMERGENCY_STOP_CHECKS [ms] */
#define EMERGENCY_STOP_CHECKS               3

#endif // FEATURE_EMERGENCY_STOP_ALL


// ##########################################################################################
// ##   configuration of the service intervall
// ##########################################################################################

#if FEATURE_SERVICE_INTERVAL
/** \brief Wie setzte ich den Interval wieder zurück, ohne die Firmware neu aufzuspielen? 
Um diese Meldung zurück zu setzen muss man den RFx000 ausschalten, die Knöpfe "links", "rauf" und "runter" drücken (und alle drei gedrückt halten), den RFx000 einschalten und die Knöpfe ca. 5-10 Sekunden danach loslassen.
Damit werden die Service-Zähler wieder auf 0 zurück gestellt. */

/** \brief Specifies the max printed hours [h] */
#define HOURS_PRINTED_UNTIL_SERVICE         100

/** \brief Specifies the max milling hours [h] */
#define HOURS_MILLED_UNTIL_SERVICE          100

/** \brief Specifies the max printed filament [m] */
#define FILAMENT_PRINTED_UNTIL_SERVICE      50000

#endif // FEATURE_SERVICE_INTERVAL

// ##########################################################################################
// ##   configuration of user-defined thermistor tables
// ##########################################################################################
/*
There are many different thermistors, which can be combined with different resistors. This result
in unpredictable number of tables. As a resolution, the user can define one table here, that can
be used as type 5 for thermister type in extruder/heated bed definition. Make sure, the number of entries
matches the value in NUM_TEMPS_USERTHERMISTOR0. If you span definition over multiple lines, make sure to end
each line, except the last, with a backslash. The table format is {{adc1,temp1},{adc2,temp2}...} with
increasing adc values. For more informations, read
http://hydraraptor.blogspot.com/2007/10/measuring-temperature-easy-way.html

If you have a sprinter temperature table, you have to multiply the first value with 4 and the second with 8.
This firmware works with increased precision, so the value reads go from 0 to 4095 and the temperature is
temperature*8.

If you have a PTC thermistor instead of a NTC thermistor, keep the adc values increasing and use themistor types 50-52 instead of 5-7!
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

/** \brief If defined, creates a thermistor table at startup.

If you don't feel like computing the table on your own, you can use this generic method. It is
a simple approximation which may be not as accurate as a good table computed from the reference
values in the datasheet. You can increase precision if you use a temperature/resistance for
R0/T0, which is near your operating temperature. This will reduce precision for lower temperatures,
which are not realy important. The resistors must fit the following schematic:
@code
VREF ---- R2 ---+--- Termistor ---+-- GND
                |                 |
                +------ R1 -------+
                |                 |
                +---- Capacitor --+
                |
                V measured
@endcode

If you don't have R1, set it to 0.
The capacitor is for reducing noise from long thermistor cable. If you don't have one, it's OK.

If you need the generic table, uncomment the following define.
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

/** \brief Beta value of thermistor

You can use the beta from the datasheet or compute it yourself.
See http://reprap.org/wiki/MeasuringThermistorBeta for more details.
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

/** \brief Number of entries in generated table. One entry takes 4 bytes. Higher number of entries increase computation time too.
Value is used for all generic tables created. */
#define GENERIC_THERM_NUM_ENTRIES           33

/** \brief If enabled, writes the created generic table to serial port at startup. */
//#define PRINT_GENERIC_TEMP_TABLE


// ##########################################################################################
// ##   duplicate motor drivers
// ##########################################################################################

/** \brief If you have an unused extruder stepper free, you could use it to drive the second z motor
instead of driving both with a single stepper. The same works for the other axis if needed. */

#define FEATURE_TWO_XSTEPPER                false
#define X2_STEP_PIN                         E1_STEP_PIN
#define X2_DIR_PIN                          E1_DIR_PIN
#define X2_ENABLE_PIN                       E1_ENABLE_PIN

#define FEATURE_TWO_YSTEPPER                false
#define Y2_STEP_PIN                         E1_STEP_PIN
#define Y2_DIR_PIN                          E1_DIR_PIN
#define Y2_ENABLE_PIN                       E1_ENABLE_PIN

#define FEATURE_TWO_ZSTEPPER                false
#define Z2_STEP_PIN                         E1_STEP_PIN
#define Z2_DIR_PIN                          E1_DIR_PIN
#define Z2_ENABLE_PIN                       E1_ENABLE_PIN


// ##########################################################################################
// ##   configure the SD Card
// ##########################################################################################

/** \brief  Select whether the SD card is supported. */
#define SDSUPPORT                           1                                                   // 1 = supported, 0 = not supported

/** \brief  Change to true if you get a inserted message on removal. */
#define SDCARDDETECTINVERTED                false

/** \brief Show extended directory including file length. Don't use this with Pronterface! */
#define SD_EXTENDED_DIR                     true

#define MAX_VFAT_ENTRIES                    (2)

/** \brief Total size of the buffer used to store the long filenames */
#define LONG_FILENAME_LENGTH                (13*MAX_VFAT_ENTRIES+1)
#define SD_MAX_FOLDER_DEPTH                 2


// ##########################################################################################
// ##   configuration of the manual steps
// ##########################################################################################

#if FEATURE_EXTENDED_BUTTONS

/** \brief Configuration of the manual steps */
#define DEFAULT_MANUAL_MM_X                    0.1f                            // [mm]
#define DEFAULT_MANUAL_MM_Y                    0.1f                            // [mm]
#define DEFAULT_MANUAL_MM_Z                    0.01f                           // [mm] -> Wert im Menü->Position->Z-Steps: xxx um!
#define DEFAULT_MANUAL_MM_E                    0.25f                           // [mm]

//Das hier drunter sind einigermaßen sinnvolle Stepsizes, wenn man Microsteps = 32 eingestellt hat, bei 16 verdoppeln sich die Zahlenwerte in mm.
//Siehe: https://github.com/RF1000community/Repetier-Firmware/issues/4
//Dieser statische Ansatz wird evtl. mal umgebaut. Man könnte auch eine Funktion schreiben, die sinnvolle Einstellwerte automatisch anhand Microsteps und Mikrometertabelle sucht.
#define NUM_ACCEPTABLE_STEP_SIZE_TABLE    7
#define ACCEPTABLE_STEP_SIZE_TABLE { 5,13,26,51,64,128,256 }
#endif // FEATURE_EXTENDED_BUTTONS


// ###############################################################################
// ##   Values for menu settings
// ###############################################################################

/** \brief 
Select the language to use.
0 = English
1 = German */
#define UI_LANGUAGE                         1   //Digibike Change

/** \brief How many ms should a single page be shown, until it is switched to the next one.*/
#define UI_PAGES_DURATION                   4000

/** \brief Uncomment if you don't want automatic page switching. You can still switch the
info pages with next/previous button/click-encoder */
#define UI_DISABLE_AUTO_PAGESWITCH          true

/** \brief Time to return to info menu if x millisconds no key was pressed. Set to 0 to disable it. */
#define UI_PRINT_AUTORETURN_TO_MENU_AFTER   120000
#define UI_MILL_AUTORETURN_TO_MENU_AFTER    0

/** \brief Normally cou want a next/previous actions with every click of your encoder.
Unfotunately, the encoder have a different count of phase changes between clicks.
Select an encoder speed from 0 = fastest to 2 = slowest that results in one menu move per click. */
#define UI_ENCODER_SPEED                    1

/** \brief There are 2 ways to change positions. You can move by increments of 1/0.1 mm resulting in more menu entries
and requiring many turns on your encode. The alternative is to enable speed dependent positioning. It will change
the move distance depending on the speed you turn the encoder. That way you can move very fast and very slow in the
same setting. */
#define UI_SPEEDDEPENDENT_POSITIONING       true

/** \brief bounce time of keys in milliseconds */
#define UI_KEY_BOUNCETIME                   10

/** \brief First time in ms until repeat of action. */
#define UI_KEY_FIRST_REPEAT                 500

/** \brief Reduction of repeat time until next execution. */
#define UI_KEY_REDUCE_REPEAT                50

/** \brief Lowest repeat time. */
#define UI_KEY_MIN_REPEAT                   50

/** \brief Default beeper mode. */
#define BEEPER_MODE                         1                                                   // 1 = on, 0 = off

/** \brief Beeper sound definitions for short beeps during key actions
and longer beeps for important actions.
Parameter is delay in microseconds and the second is the number of repetitions.
Values must be in range 1..255 */
#define BEEPER_SHORT_SEQUENCE                   2,2
#define BEEPER_LONG_SEQUENCE                    8,8
#define BEEPER_START_PRINTING_SEQUENCE          100,2
#define BEEPER_STOP_PRINTING_SEQUENCE           100,3
#define BEEPER_PAUSE_SEQUENCE                   50,3
#define BEEPER_CONTINUE_SEQUENCE                50,2
#define BEEPER_START_HEAT_BED_SCAN_SEQUENCE     100,2
#define BEEPER_ABORT_HEAT_BED_SCAN_SEQUENCE     250,5
#define BEEPER_STOP_HEAT_BED_SCAN_SEQUENCE      100,3
#define BEEPER_START_WORK_PART_SCAN_SEQUENCE    100,2
#define BEEPER_ABORT_WORK_PART_SCAN_SEQUENCE    250,5
#define BEEPER_STOP_WORK_PART_SCAN_SEQUENCE     100,3
//FEATURE_ALIGN_EXTRUDERS
 #define BEEPER_START_ALIGN_EXTRUDERS_SEQUENCE  100,2
 #define BEEPER_ABORT_ALIGN_EXTRUDERS_SEQUENCE  250,5
 #define BEEPER_STOP_ALIGN_EXTRUDERS_SEQUENCE   100,3
#define BEEPER_ABORT_SET_POSITION_SEQUENCE      250,5
#define BEEPER_ACCEPT_SET_POSITION_SEQUENCE     100,2
#define BEEPER_SERVICE_INTERVALL_SEQUNCE        100,3
#define BEEPER_ALIGN_EXTRUDERS_SEQUNCE          50,5
#define BEEPER_WRONG_FIRMWARE_SEQUNCE           255,16

/** \brief Values used for preheat */
#define UI_SET_PRESET_HEATED_BED_TEMP_PLA   60
#define UI_SET_PRESET_EXTRUDER_TEMP_PLA     180
#define UI_SET_PRESET_HEATED_BED_TEMP_ABS   100
#define UI_SET_PRESET_EXTRUDER_TEMP_ABS     210

/** \brief Values used for unload(unmount)/load(mount) filament */
#define UI_SET_EXTRUDER_MIN_TEMP_UNMOUNT     90                                                 //hat anfangs hohe toleranz.
#define UI_SET_EXTRUDER_MAX_TEMP_UNMOUNT    180
#define UI_SET_EXTRUDER_TEMP_MOUNT          200

/** \brief Extreme values */
#define UI_SET_MIN_HEATED_BED_TEMP          55
#define UI_SET_MAX_HEATED_BED_TEMP          160
#define UI_SET_MIN_EXTRUDER_TEMP            70
#define UI_SET_MAX_EXTRUDER_TEMP            270
#define UI_SET_EXTRUDER_FEEDRATE            1.5f                                                 // [mm/sec]
#define UI_SET_EXTRUDER_RETRACT_DISTANCE    3                                                   // [mm]
#define COOLDOWN_THRESHOLD                  40                                                  // [°C]

#define SHOW_DEBUGGING_MENU                 0                                                   // 1 = show, 0 = hide

#define SPEED_MIN_MILLIS                    300
#define SPEED_MAX_MILLIS                    50
#define SPEED_MAGNIFICATION                 100.0f

/** \brief Specifies if you want to see the TipDown Support within Z-Configuration Menu. This is only usefull if you have a dual hotend with possible servo or spring z-offset-shift.*/
#define UI_SHOW_TIPDOWN_IN_ZCONFIGURATION   1                                                   // 1 = show, 0 = hide //Digibike change


// ##########################################################################################
// ##   general external EEPROM configuration
// ##########################################################################################

#define EEPROM_DELAY                        10                                                  // [ms]


// ##########################################################################################
// ##   external EEPROM which is used for the z-compensation (32.768 bytes)
// ##########################################################################################
/*
we use blocks of 2 kByte size for the structure of our EEPROM

00000 ... 01535 bytes [EEPROM_SECTOR_SIZE Bytes] = general information
  00000 [2 bytes] EEPROM format
  00002 [2 bytes] active work part z-compensation matrix (1 ... EEPROM_MAX_WORK_PART_SECTORS)
  00004 [2 bytes] active heat bed z-compensation matrix (1 ... EEPROM_MAX_HEAT_BED_SECTORS)

01536 ... 03071 bytes [EEPROM_SECTOR_SIZE Bytes] = heat bed z-compensation matrix 1
  01536 [2 bytes] x-dimension of the heat bed z-compensation matrix 1
  01538 [2 bytes] y-dimension of the heat bed z-compensation matrix 1
  01540 [2 bytes] used micro steps
  01542 [12 bytes] reserved
  01554 [x bytes] heat bed z-compensation matrix 1

03072 ... 04597 bytes [EEPROM_SECTOR_SIZE Bytes] = heat bed z-compensation matrix 2
  ...

15360 ... 16895 bytes [EEPROM_SECTOR_SIZE Bytes]  = work part compensation matrix 1
  15360 [2 bytes] x-dimension of the work part compensation matrix 1
  15362 [2 bytes] y-dimension of the work part compensation matrix 1
  15364 [2 bytes] used micro steps
  15366 [2 bytes] x start position of the work part scan [mm]
  15368 [2 bytes] y start position of the work part scan [mm]
  15370 [2 bytes] x step size of the work part scan [mm]
  15372 [2 bytes] y step size of the work part scan [mm]
  15374 [2 bytes] x end position of the work part scan [mm]
  15376 [2 bytes] y end position of the work part scan [mm]
  15378 [x bytes] work part z-compensation matrix 1

16895 ... 18430 bytes [2 kBytes]  = work part compensation matrix 2
  ...
*/

#define EEPROM_OFFSET_SECTOR_FORMAT                 0
#define EEPROM_OFFSET_DIMENSION_X                   2
#define EEPROM_OFFSET_DIMENSION_Y                   4
#define EEPROM_OFFSET_MICRO_STEPS                   6
#define EEPROM_OFFSET_X_START_MM                    16
#define EEPROM_OFFSET_Y_START_MM                    18
#define EEPROM_OFFSET_X_STEP_MM                     20
#define EEPROM_OFFSET_Y_STEP_MM                     22
#define EEPROM_OFFSET_X_END_MM                      24
#define EEPROM_OFFSET_Y_END_MM                      26
#define EEPROM_OFFSET_MATRIX_START                 28

#define EEPROM_SECTOR_SIZE                          1536                                        // [bytes]
#define EEPROM_MAX_WORK_PART_SECTORS                9
#define EEPROM_MAX_HEAT_BED_SECTORS                 9

#define EEPROM_OFFSET_HEADER_FORMAT                 0                                           // [bytes]
#define EEPROM_OFFSET_ACTIVE_WORK_PART_Z_MATRIX     2                                           // [bytes]
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
#define DEFAULT_Z_SCALE_MODE                Z_VALUE_MODE_Z_MIN

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

/** \brief Allows to use this firmware together with the non-Repetier PC applications
Without this special handling, the firmware may complain about checksum errors from non-Repetier PC applications (e.g. Cura, ...) and
non-Repetier PC applications may fall over the debug outputs of the firmware. */
#define ALLOW_EXTENDED_COMMUNICATION        2                                                   // 0 = do not allow, 1 = allow "Wait", 2 = allow "Wait" and debug outputs

/** \brief Configuration of the external watchdog
The TPS3820 of the RF1000/RF2000 resets about 112/200/310 (min/typical/max) ms after the last time when it was triggered
http://pdf1.alldatasheet.com/datasheet-pdf/view/29215/TI/TPS3820-50DBVT.html 
t_d in datasheet is delay time: how long reset is triggered after timeout: 15...25...37ms for TPS3820.
*/
#define WATCHDOG_MAIN_LOOP_TIMEOUT          20000UL                                             // [ms] -> uhrzeit intern scheint nicht immer zu stimmen!

/** \brief Longer-lasting operations shall call our periodical actions at least each defined time interval */
#define PERIODICAL_ACTIONS_CALL_INTERVAL    10                                                  // [ms]

/** \brief The display shows that the device is idle after no new commands were processed for longer than the minimal idle time */
#define MINIMAL_IDLE_TIME                   500                                                 // [ms]
    
/** \brief If enabled you can select the distance your filament gets retracted during a M140 command, after a given temperature is reached. */
#define RETRACT_DURING_HEATUP               true

/** \brief PID control only works target temperature +/- PID_CONTROL_RANGE.
If you get much overshoot at the first temperature set, because the heater is going full power too long, you
need to increase this value. For one 6.8 Ohm heater 10 is ok. With two 6.8 Ohm heater use 15. */
#define PID_CONTROL_RANGE                   30
/** If you change those you might have to do fresh autotunePIDs on your heaters. */
#define PID_CONTROL_DRIVE_MAX_LIMIT_FACTOR  10.0f //this was 10
#define PID_CONTROL_DRIVE_MIN_LIMIT_FACTOR  -1.0f //this was 10 but -1.0 works well with drive max 100 and drive min 5. If this number is negative you get a real PID control, no PD+posI-control anymore.

/** \brief Prevent extrusions longer then x mm for one command. This is especially important if you abort a print. Then the
extrusion position might be at any value like 23344. If you then have an G1 E-2 it will roll back 23 meter! */
#define EXTRUDE_MAXLENGTH                   100

/** \brief Set PID scaling
PID values assume a usable range from 0-255. This can be further limited to EXT0_PID_MAX by two methods.
Set the value to 0: Normal computation, just clip output to EXT0_PID_MAX if computed value is too high.
Set value to 1: Scale PID by EXT0_PID_MAX/256 and then clip to EXT0_PID_MAX.
If your EXT0_PID_MAX is low, you should prefer the second method. */
#define SCALE_PID_TO_MAX                    0

/** \brief Temperature range for target temperature to hold in M109 command. 5 means +/-5 degC
Uncomment define to force the temperature into the range for given watchperiod. */
#define TEMP_TOLERANCE                      2.0f                                               // [°C]

/** \brief Additional special temperature tolerance range when unpausing print. Faster start is better here, because reaching pause position might take a while - for a bit less oozing */
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

/** \brief Step to split a circle in small Lines */
#define MM_PER_ARC_SEGMENT                  1
#define MM_PER_ARC_SEGMENT_BIG              3

/** \brief After this count of steps a new SIN / COS caluclation is startet to correct the circle interpolation */
#define N_ARC_CORRECTION                    25

/** \brief Communication speed.
Overridden if EEPROM activated. */
#define BAUDRATE                            115200

/** \brief Some boards like Gen7 have a power on pin, to enable the atx power supply. If this is defined,
the power will be turned on without the need to call M80 if initially started. */
#define ENABLE_POWER_ON_STARTUP

/** \brief If you use an ATX power supply you need the power pin to work non inverting. For some special
boards you might need to make it inverting. */
#define POWER_INVERTING                     false

/** \brief Cache size for incoming commands.
There should be no reason to increase this cache. Commands are nearly immediately sent to
execution. */
#define GCODE_BUFFER_SIZE                   2

/** \brief Appends the linenumber after every ok send, to acknowledge the received command. Uncomment for plain ok ACK if your host has problems with this */
#define ACK_WITH_LINENUMBER                 1

/** \brief Communication errors can swollow part of the ok, which tells the host software to send
the next command. Not receiving it will cause your printer to stop. Sending this string every
second, if our queue is empty should prevent this. Comment it, if you don't wan't this feature. */
#define WAITING_IDENTIFIER                  "wait"

/** \brief Sets time for echo debug
You can set M111 1 which enables ECHO of commands sent. This define specifies the position,
when it will be executed. In the original FiveD software, echo is done after receiving the
command. With checksum you know, how it looks from the sending string. With this define
uncommented, you will see the last command executed. To be more specific: It is written after
execution. This helps tracking errors, because there may be 8 or more commands in the queue
and it is elsewise difficult to know, what your reprap is currently doing. */
#define ECHO_ON_EXECUTE                     1

/** \brief If the firmware is busy, it will send a busy signal to host signaling that
 everything is fine and it only takes a bit longer to finish. That way the 
 host can keep timeout short so in case of communication errors the resulting
 blobs are much smaller. Set to 0 to disable it. */
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

/** \brief For Nibbels external interrupt 3 plus an extra pin is used for reading digital calipers. You will have to solder some logic-converter from 1.8v to 5v see http://www.instructables.com/id/Reading-Digital-Callipers-with-an-Arduino-USB/ */
#define FEATURE_READ_CALIPER               0                                                   // 0 = OFF, 1 = ON
#define FEATURE_READ_CALIPER_INT_PIN       RESERVE_DIGITAL_PIN_PD3                             // RF2000/RF1000: RESERVE_DIGITAL_PIN_PD3 is INT3 for having clocks falling edges collected
#define FEATURE_READ_CALIPER_DATA_PIN      RESERVE_DIGITAL_PIN_PE4                             // RF2000: RESERVE_DIGITAL_PIN_PE4 is some reserve pin for reading off data while clocks falling edge.
#if FEATURE_READ_CALIPER && MOTHERBOARD == DEVICE_TYPE_RF1000 && FEATURE_READ_CALIPER_DATA_PIN == RESERVE_DIGITAL_PIN_PE4
 #error You cannot use RESERVE_DIGITAL_PIN_PE4 on an RF1000, please connect and choose another one.
#endif

/** \brief For Nibbels external interrupt 3 button at RF1000 X25.8 und RF2000 X34.2 "EXT_IRQ"/INT3 
You can activate this to 1 and connect some Button. If you connect ground to pull the pullup down you will let the firmware jump into interrupt routine */
#define FEATURE_USER_INT3                   0                                                   // 0 = OFF, 1 = ON

#if FEATURE_READ_CALIPER && FEATURE_USER_INT3
 #error You cannot use FEATURE_READ_CALIPER and FEATURE_USER_INT3 at the same time with stock programming. Please change pins/etc. and remove this errorcheck
#endif

/** beta!!! Nibbels/PeterKA only!!! -> Testfeature: It can check if you lost steps and test your buttons hysteresis */
#define FEATURE_CHECK_HOME                  0

/** \brief This adds some GCode M3029 to simulate Key-Press by GCode and to read whats inside the printers Display rightnow. */
#define FEATURE_SEE_DISPLAY                 1

/** \brief This feature allows you to extrude into thin air to messure the filaments viscosity value using dms sensors */
#define FEATURE_VISCOSITY_TEST              0

/** \brief This is some testing function for reading the stepper drivers status bits while operation */
#define FEATURE_READ_STEPPER_STATUS         0

/** \brief Automatic Startline */
#define FEATURE_STARTLINE                   1

#endif // CONFIGURATION_H

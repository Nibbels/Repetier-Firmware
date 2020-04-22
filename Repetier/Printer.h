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

#include "Commands.h"

#ifndef PRINTER_H
#define PRINTER_H

// ##########################################################################################
// ##   status flags
// ##########################################################################################

#define PRINTER_FLAG0_STEPPER_DISABLED 1
#define PRINTER_FLAG0_SEPERATE_EXTRUDER_INT 2
#define PRINTER_FLAG0_TEMPSENSOR_DEFECT 4
#define PRINTER_FLAG0_FORCE_CHECKSUM 8
#define PRINTER_FLAG0_MANUAL_MOVE_MODE 16
#define PRINTER_FLAG0_LARGE_MACHINE 128

#define PRINTER_FLAG1_AUTOMOUNT 2
#define PRINTER_FLAG1_ANIMATION 4
#define PRINTER_FLAG1_ALLSWITCHEDOFF 8
#define PRINTER_FLAG1_UI_ERROR_MESSAGE 16
#define PRINTER_FLAG1_Z_ORIGIN_SET 64

#define PRINTER_FLAG2_RESET_FILAMENT_USAGE 4
#define PRINTER_FLAG2_GOT_TEMPS 32

#define PRINTER_FLAG3_X_HOMED 1  // flag3 alike original repetier
#define PRINTER_FLAG3_Y_HOMED 2  // flag3 alike original repetier
#define PRINTER_FLAG3_Z_HOMED 4  // flag3 alike original repetier
#define PRINTER_FLAG3_PRINTING 8 // flag3 alike original repetier

#define KOSYS_GCODE true
#define KOSYS_DIRECTOFFSET false

#define FEEDRATE_GCODE true
#define FEEDRATE_DIRECTCONFIG false

#define DIR_QUEUE 1
#define DIR_DIRECT -1

class Printer {
public:
#if USE_ADVANCE
    static volatile int extruderStepsNeeded; // This many extruder steps are still needed, <0 = reverse steps needed.
    static uint8_t maxExtruderSpeed;         // Timer delay for end extruder speed
    static volatile int advanceStepsSet;
#endif // USE_ADVANCE

    static uint8_t menuMode;
    static float axisStepsPerMM[];
    static float axisMMPerSteps[];
    static float maxFeedrate[];
    static float homingFeedrate[];
    static float maxAccelerationMMPerSquareSecond[];
    static float maxTravelAccelerationMMPerSquareSecond[];
#if FEATURE_MILLING_MODE
    static short max_milling_all_axis_acceleration;
#endif // FEATURE_MILLING_MODE
    static uint32_t maxPrintAccelerationStepsPerSquareSecond[];
    static uint32_t maxTravelAccelerationStepsPerSquareSecond[];
    static uint8_t relativeCoordinateMode;         // Determines absolute (false) or relative Coordinates (true).
    static uint8_t relativeExtruderCoordinateMode; // Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.
    static uint8_t unitIsInches;
    static uint8_t debugLevel;
    static uint8_t flag0;
    static uint8_t flag1;
    static uint8_t flag2;
    static uint8_t flag3;
    static uint8_t stepsPerTimerCall;
    static uint16_t stepsPackingMinInterval;
    static volatile unsigned long interval; // Last step duration in ticks.
    static volatile float v;                // Last planned printer speed.
    static unsigned long timer[2];          // used for acceleration/deceleration timing
    static unsigned long stepNumber[2];     // Step number in current move.
#if FEATURE_DIGIT_FLOW_COMPENSATION
    static unsigned short interval_mod; // additional step duration in ticks to slow the printer down live
#endif                                  // FEATURE_DIGIT_FLOW_COMPENSATION
    static int8_t lastDirectionSovereignty;
    static float originOffsetMM[3];
    static volatile float destinationMM[4]; // Target in mm from origin.
    static float destinationMMLast[4];      // Position in mm from origin.

    static long maxSoftEndstopSteps[3];     // For software endstops, limit of move in positive direction. (=Homing-Offset + Achsenlänge)
    static float axisLengthMM[3];           // Länge des überfahrbaren Bereichs im positiven Homing. (=Schienen-Fahrweg - Homing-Offset - 2x ExtruderOffset)
    static float feedrate;                  // Last requested feedrate.
    static int feedrateMultiply;            // Multiplier for feedrate in percent (factor 1 = 100)
    static float dynamicFeedrateFactor;     // Feedrate multiplier factor for digit compensation (1.0 = 100%)
    static float menuExtrusionFactor;       // Flow multiplier factor (1.0 = 100%)
    static float dynamicExtrusionFactor;    // Flow multiplier factor for digit compensation (1.0 = 100%)
    static float extrudeMultiplyErrorSteps; // collects the extrusion error.
    static float maxXYJerk;                 // Maximum allowed jerk in mm/s
    static float maxZJerk;                  // Maximum allowed jerk in z direction in mm/s
    static speed_t vMaxReached[2];          // Maximumu reached speed
    static unsigned long msecondsPrinting;  // Milliseconds of printing time (means time with heated extruder)
    static unsigned long msecondsMilling;   // Milliseconds of milling time
    static float filamentPrinted;           // mm of filament printed since counting started
    static long ZOffset;                    // Z Offset in um
    static char ZMode;                      // Z Scale
    static char moveMode[3];                // move mode which is applied within the Position X/Y/Z menus
    static bool moveKosys;                  // true = GCode, false = DirectMove / OffsetMove
    static bool movePositionFeedrateChoice; // select the feedrate for menu positioning: feedrate from last gcode or standard speed

    // This is some buffer but only for a limited amount of overdrive.
    static volatile int16_t outOfPrintVolume[2];
    static volatile int32_t outOfPrintVolumeZ;

#if FEATURE_MEMORY_POSITION
    static float memoryX;
    static float memoryY;
    static float memoryZ;
    static float memoryE;
    static float memoryF;
#endif // FEATURE_MEMORY_POSITION

#if FEATURE_HEAT_BED_Z_COMPENSATION
    static volatile char doHeatBedZCompensation;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_WORK_PART_Z_COMPENSATION
    static volatile char doWorkPartZCompensation;
    static volatile long staticCompensationZ; // this is the z-delta which can occur in case the x/y position of the z-origin from the work part scan is different to the x/y position of the z-origin from the moment of the start of the milling
#endif                                        // FEATURE_WORK_PART_Z_COMPENSATION

    static volatile long currentSteps[3];
    static volatile char blockAll;

    static volatile long currentXSteps;
    static volatile long currentYSteps;
    static volatile long currentZSteps;
    static uint16_t maxZOverrideSteps;

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
    static volatile long compensatedPositionTargetStepsZ;
    static volatile long compensatedPositionCurrentStepsZ;
    static volatile float compensatedPositionOverPercE;
    static volatile float compensatedPositionCollectTinyE;

    static long queuePositionZLayerGuessNew;
    static volatile long queuePositionZLayerCurrent;
    static volatile long queuePositionZLayerLast;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

    static volatile long directDestinationSteps[4];
    static volatile long directCurrentSteps[4];

#if FEATURE_MILLING_MODE
    static char operatingMode;
    static float drillFeedrate;
    static float drillZDepth;
#endif // FEATURE_MILLING_MODE

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
    static char ZEndstopType;
    static char ZEndstopUnknown;
    static char lastZDirection;
    static char endstopZMinHit;
    static char endstopZMaxHit;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

#if FEATURE_CONFIGURABLE_MILLER_TYPE
    static char MillerType;
#endif // FEATURE_CONFIGURABLE_MILLER_TYPE

#if STEPPER_ON_DELAY
    static char enabledStepper[3];
#endif // STEPPER_ON_DELAY

#if FEATURE_BEEPER
    static char enableBeeper;
#endif // FEATURE_BEEPER

#if FEATURE_CASE_LIGHT
    static char enableCaseLight;
#endif // FEATURE_CASE_LIGHT

#if FEATURE_RGB_LIGHT_EFFECTS
    static char RGBLightMode;
    static char RGBLightStatus;
    static unsigned long RGBLightIdleStart;
    static char RGBButtonBackPressed;
    static char RGBLightModeForceWhite;
#endif // FEATURE_RGB_LIGHT_EFFECTS

#if FEATURE_230V_OUTPUT
    static char enable230VOutput;
#endif // FEATURE_230V_OUTPUT

#if FEATURE_24V_FET_OUTPUTS
    static char enableFET1;
    static char enableFET2;
    static char enableFET3;
#endif // FEATURE_24V_FET_OUTPUTS

#if FEATURE_CASE_FAN
    static bool ignoreFanOn;
    static millis_t prepareFanOff;
    static unsigned long fanOffDelay;
#endif // FEATURE_CASE_FAN

#if FEATURE_TYPE_EEPROM
    static unsigned char wrongType;
#endif // FEATURE_TYPE_EEPROM

#if FEATURE_UNLOCK_MOVEMENT
    static unsigned char g_unlock_movement;
#endif //FEATURE_UNLOCK_MOVEMENT

#if FEATURE_SENSIBLE_PRESSURE
    static bool g_senseoffset_autostart;
#endif //FEATURE_SENSIBLE_PRESSURE

    static uint8_t motorCurrent[DRV8711_NUM_CHANNELS];

#if FEATURE_ZERO_DIGITS
    static bool g_pressure_offset_active;
    static short g_pressure_offset;
#endif // FEATURE_ZERO_DIGITS

#if FEATURE_ADJUSTABLE_MICROSTEPS
    static uint8_t motorMicroStepsModeValue[DRV8711_NUM_CHANNELS]; //1=2MS, 2=4MS, 3=8MS, 4=16MS, 5=32MS, 6=64MS, 7=128MS, 8=256MS
#endif                                                             // FEATURE_ADJUSTABLE_MICROSTEPS

#if FEATURE_Kurt67_WOBBLE_FIX
    static int8_t wobblePhaseXY;
    //static int8_t           wobblePhaseZ;
    static int16_t wobbleAmplitudes[3 /*4*/];  //X, Y(X_0), Y(X_max), /*Z*/
    static float lastWobbleFixOffset[2 /*3*/]; //< last calculated target wobbleFixOffsets for display output.
#endif                                         // FEATURE_Kurt67_WOBBLE_FIX

    static INLINE void setMenuMode(uint8_t mode, bool on) {
        if (on)
            menuMode |= mode;
        else
            menuMode &= ~mode;
    } // setMenuMode

    static INLINE bool isMenuMode(uint8_t mode) {
        return (menuMode & mode) == mode;
    } // isMenuMode

    static INLINE bool debugEcho() {
        return ((debugLevel & 1) != 0);
    } // debugEcho

    static INLINE bool debugInfo() {
        return ((debugLevel & 2) != 0);
    } // debugInfo

    static INLINE bool debugErrors() {
        return ((debugLevel & 4) != 0);
    } // debugErrors

    static INLINE bool debugDryrun() {
        return ((debugLevel & 8) != 0);
    } // debugDryrun

    /** \brief Disable stepper motor for x direction. */
    static INLINE void disableXStepper() {
#if (X_ENABLE_PIN > -1)
        WRITE(X_ENABLE_PIN, !X_ENABLE_ON);
#endif // (X_ENABLE_PIN > -1)

#if FEATURE_TWO_XSTEPPER && (X2_ENABLE_PIN > -1)
        WRITE(X2_ENABLE_PIN, !X_ENABLE_ON);
#endif // FEATURE_TWO_XSTEPPER && (X2_ENABLE_PIN > -1)

#if STEPPER_ON_DELAY
        Printer::enabledStepper[X_AXIS] = 0;
#endif // STEPPER_ON_DELAY

        // when the stepper is disabled we loose our home position because somebody else can move our mechanical parts
        setHomed(false, -1, -1);
        InterruptProtectedBlock noInts; //HAL::forbidInterrupts();

        Printer::setXAxisSteps(0);
        Printer::currentXSteps = 0;
        Printer::resetDirectAxis(X_AXIS);

        killPausePrint();

        noInts.unprotect(); //HAL::allowInterrupts();

        Commands::printCurrentPosition();
    } // disableXStepper

    /** \brief Disable stepper motor for y direction. */
    static INLINE void disableYStepper() {
#if (Y_ENABLE_PIN > -1)
        WRITE(Y_ENABLE_PIN, !Y_ENABLE_ON);
#endif // (Y_ENABLE_PIN > -1)

#if FEATURE_TWO_YSTEPPER && (Y2_ENABLE_PIN > -1)
        WRITE(Y2_ENABLE_PIN, !Y_ENABLE_ON);
#endif // FEATURE_TWO_YSTEPPER && (Y2_ENABLE_PIN > -1)

#if STEPPER_ON_DELAY
        Printer::enabledStepper[Y_AXIS] = 0;
#endif // STEPPER_ON_DELAY

        // when the stepper is disabled we loose our home position because somebody else can move our mechanical parts
        setHomed(-1, false, -1);

        InterruptProtectedBlock noInts; //HAL::forbidInterrupts();

        Printer::setYAxisSteps(0);
        Printer::currentYSteps = 0;
        Printer::resetDirectAxis(Y_AXIS);

        killPausePrint();

        noInts.unprotect(); //HAL::allowInterrupts();

        Commands::printCurrentPosition();
    } // disableYStepper

    /** \brief Disable stepper motor for z direction. */
    static INLINE void disableZStepper() {
        InterruptProtectedBlock noInts;
#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
        Printer::compensatedPositionTargetStepsZ = Printer::compensatedPositionCurrentStepsZ = g_nZScanZPosition = 0;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
        noInts.unprotect();

        // when the stepper is disabled we loose our home position because somebody else can move our mechanical parts
        setHomed(-1, -1, false); // disable CMP mit wait ist bei unhome Z mit drin. //Printer::disableCMPnow(true); //fahre vom heizbett auf 0 bevor stepper aus.

#if (Z_ENABLE_PIN > -1)
        WRITE(Z_ENABLE_PIN, !Z_ENABLE_ON);
#endif // (Z_ENABLE_PIN > -1)

#if FEATURE_TWO_ZSTEPPER && (Z2_ENABLE_PIN > -1)
        WRITE(Z2_ENABLE_PIN, !Z_ENABLE_ON);
#endif // FEATURE_TWO_ZSTEPPER && (Z2_ENABLE_PIN > -1)

#if STEPPER_ON_DELAY
        Printer::enabledStepper[Z_AXIS] = 0;
#endif // STEPPER_ON_DELAY

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
        Printer::lastZDirection = 0;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

        noInts.protect();
        Printer::setZAxisSteps(0);
        Printer::currentZSteps = 0;
        Printer::resetDirectAxis(Z_AXIS);

#if FEATURE_HEAT_BED_Z_COMPENSATION
        Printer::queuePositionZLayerLast = 0;
        Printer::queuePositionZLayerCurrent = 0;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_FIND_Z_ORIGIN
        g_nZOriginPosition[X_AXIS] = 0;
        g_nZOriginPosition[Y_AXIS] = 0;
        g_nZOriginPosition[Z_AXIS] = 0;
        Printer::setZOriginSet(false); //flag wegen statusnachricht
#endif                                 // FEATURE_FIND_Z_ORIGIN

        killPausePrint();

        noInts.unprotect();

        Commands::printCurrentPosition();
    } // disableZStepper

    /** \brief Enable stepper motor for x direction. */
    static INLINE void enableXStepper() {
        unmarkAllSteppersDisabled();
#if (X_ENABLE_PIN > -1)
        WRITE(X_ENABLE_PIN, X_ENABLE_ON);
#endif // (X_ENABLE_PIN > -1)

#if FEATURE_TWO_XSTEPPER && (X2_ENABLE_PIN > -1)
        WRITE(X2_ENABLE_PIN, X_ENABLE_ON);
#endif // FEATURE_TWO_XSTEPPER && (X2_ENABLE_PIN > -1)

#if STEPPER_ON_DELAY
        if (!Printer::enabledStepper[X_AXIS]) {
            Printer::enabledStepper[X_AXIS] = 1;
            HAL::delayMilliseconds(STEPPER_ON_DELAY);
        }
#endif // STEPPER_ON_DELAY
    }  // enableXStepper

    /** \brief Enable stepper motor for y direction. */
    static INLINE void enableYStepper() {
        unmarkAllSteppersDisabled();
#if (Y_ENABLE_PIN > -1)
        WRITE(Y_ENABLE_PIN, Y_ENABLE_ON);
#endif // (Y_ENABLE_PIN > -1)

#if FEATURE_TWO_YSTEPPER && (Y2_ENABLE_PIN > -1)
        WRITE(Y2_ENABLE_PIN, Y_ENABLE_ON);
#endif // FEATURE_TWO_YSTEPPER && (Y2_ENABLE_PIN > -1)

#if STEPPER_ON_DELAY
        if (!Printer::enabledStepper[Y_AXIS]) {
            Printer::enabledStepper[Y_AXIS] = 1;
            HAL::delayMilliseconds(STEPPER_ON_DELAY);
        }
#endif // STEPPER_ON_DELAY
    }  // enableYStepper

    /** \brief Enable stepper motor for z direction. */
    static INLINE void enableZStepper() {
        unmarkAllSteppersDisabled();
#if (Z_ENABLE_PIN > -1)
        WRITE(Z_ENABLE_PIN, Z_ENABLE_ON);
#endif // (Z_ENABLE_PIN > -1)

#if FEATURE_TWO_ZSTEPPER && (Z2_ENABLE_PIN > -1)
        WRITE(Z2_ENABLE_PIN, Z_ENABLE_ON);
#endif // FEATURE_TWO_ZSTEPPER && (Z2_ENABLE_PIN > -1)

#if STEPPER_ON_DELAY
        if (!Printer::enabledStepper[Z_AXIS]) {
            Printer::enabledStepper[Z_AXIS] = 1;
            HAL::delayMilliseconds(STEPPER_ON_DELAY);
        }
#endif // STEPPER_ON_DELAY
    }  // enableZStepper

    static INLINE void setXDirection(bool positive) {
        if (positive) {
            // extruder moves to the right
            WRITE(X_DIR_PIN, !INVERT_X_DIR);
#if FEATURE_TWO_XSTEPPER
            WRITE(X2_DIR_PIN, !INVERT_X_DIR);
#endif // FEATURE_TWO_XSTEPPER
        } else {
            // extruder moves to the left
            WRITE(X_DIR_PIN, INVERT_X_DIR);
#if FEATURE_TWO_XSTEPPER
            WRITE(X2_DIR_PIN, INVERT_X_DIR);
#endif // FEATURE_TWO_XSTEPPER
        }
    } // setXDirection

    static INLINE void setYDirection(bool positive) {
        if (positive) {
            // heat bed moves to the front
            WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
#if FEATURE_TWO_YSTEPPER
            WRITE(Y2_DIR_PIN, !INVERT_Y_DIR);
#endif // FEATURE_TWO_YSTEPPER
        } else {
            // heat bed moves to the back
            WRITE(Y_DIR_PIN, INVERT_Y_DIR);
#if FEATURE_TWO_YSTEPPER
            WRITE(Y2_DIR_PIN, INVERT_Y_DIR);
#endif // FEATURE_TWO_YSTEPPER
        }
    } // setYDirection

    static INLINE void setZDirection(bool positive) {
        if (positive) {
            // heat bed moves to the bottom
            WRITE(Z_DIR_PIN, !INVERT_Z_DIR);
#if FEATURE_TWO_ZSTEPPER
            WRITE(Z2_DIR_PIN, !INVERT_Z_DIR);
#endif // FEATURE_TWO_YSTEPPER
#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
            lastZDirection = 1;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS
        } else {
            // heat bed moves to the top
            WRITE(Z_DIR_PIN, INVERT_Z_DIR);
#if FEATURE_TWO_ZSTEPPER
            WRITE(Z2_DIR_PIN, INVERT_Z_DIR);
#endif // FEATURE_TWO_YSTEPPER
#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
            lastZDirection = -1;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS
        }
    } // setZDirection

    static INLINE void startXStep(int8_t dir = (Printer::getXDirectionIsPos() ? 1 : -1)) {
        WRITE(X_STEP_PIN, HIGH);
#if FEATURE_TWO_XSTEPPER
        WRITE(X2_STEP_PIN, HIGH);
#endif // FEATURE_TWO_XSTEPPER
        Printer::currentXSteps += dir;
    } // startXStep

    static INLINE void startYStep(int8_t dir = (Printer::getYDirectionIsPos() ? 1 : -1)) {
        WRITE(Y_STEP_PIN, HIGH);
#if FEATURE_TWO_YSTEPPER
        WRITE(Y2_STEP_PIN, HIGH);
#endif // FEATURE_TWO_YSTEPPER
        Printer::currentYSteps += dir;
    } // startYStep

    static INLINE void startZStep(int8_t dir = (Printer::getZDirectionIsPos() ? 1 : -1)) {
        WRITE(Z_STEP_PIN, HIGH);
#if FEATURE_TWO_ZSTEPPER
        WRITE(Z2_STEP_PIN, HIGH);
#endif // FEATURE_TWO_ZSTEPPER
        Printer::currentZSteps += dir;
    } // startZStep

    static INLINE void endZStep(void) {
        WRITE(Z_STEP_PIN, LOW);
#if FEATURE_TWO_ZSTEPPER
        WRITE(Z2_STEP_PIN, LOW);
#endif // FEATURE_TWO_ZSTEPPER
    }  // endZStep

    static INLINE void endYStep(void) {
        WRITE(Y_STEP_PIN, LOW);
    } // endZStep

    static INLINE void endXStep(void) {
        WRITE(X_STEP_PIN, LOW);
    } // endZStep

    static INLINE void endXYZSteps() {
        endXStep();
        endYStep();
        endZStep();
    } // endXYZSteps

    static INLINE bool getZDirectionIsPos() {
        return ((READ(Z_DIR_PIN) != 0) ^ INVERT_Z_DIR);
    } // getZDirectionIsPos

    static INLINE bool getYDirectionIsPos() {
        return ((READ(Y_DIR_PIN) != 0) ^ INVERT_Y_DIR);
    } // getYDirection

    static INLINE bool getXDirectionIsPos() {
        return ((READ(X_DIR_PIN) != 0) ^ INVERT_X_DIR);
    } // getXDirection

    static INLINE uint8_t isLargeMachine() {
        return flag0 & PRINTER_FLAG0_LARGE_MACHINE;
    } // isLargeMachine

    static INLINE void setLargeMachine(uint8_t b) {
        flag0 = (b ? flag0 | PRINTER_FLAG0_LARGE_MACHINE : flag0 & ~PRINTER_FLAG0_LARGE_MACHINE);
    } // setLargeMachine

    static INLINE uint8_t isAdvanceActivated() {
        return flag0 & PRINTER_FLAG0_SEPERATE_EXTRUDER_INT;
    } // isAdvanceActivated

    static INLINE void setAdvanceActivated(uint8_t b) {
        flag0 = (b ? flag0 | PRINTER_FLAG0_SEPERATE_EXTRUDER_INT : flag0 & ~PRINTER_FLAG0_SEPERATE_EXTRUDER_INT);
    } // setAdvanceActivated

    static INLINE uint8_t isAxisHomed(int8_t b) //X_AXIS 0, Y_AXIS 1, Z_AXIS 2
    {
        switch (b) {
        case X_AXIS: {
            return (flag3 & PRINTER_FLAG3_X_HOMED);
        }
        case Y_AXIS: {
            return (flag3 & PRINTER_FLAG3_Y_HOMED);
        }
        case Z_AXIS: {
            return (flag3 & PRINTER_FLAG3_Z_HOMED);
        }
        }
        return 0;
    } // isAxisHomed

    static inline uint8_t isZHomeSafe() //experimentelle funktion, die nicht viel abdeckt, das ist ein test. ... TODO: merge with function isHomingAllowed
    {
        bool problematisch = false;
        if (Extruder::current->offsetMM[Z_AXIS] != 0)
            problematisch = true; //wenn rechtes gefedertes Hotend tiefer, dann evtl. kollision
        if (g_offsetZCompensationSteps > 0)
            problematisch = true; //wenn matrix positiv, dann evtl. problem
        if (isAxisHomed(Y_AXIS) && Printer::currentYSteps <= 5 * YAXIS_STEPS_PER_MM)
            problematisch = false; //vorherige Probleme egal, wenn bett nach hinten gefahren
#if FEATURE_ALIGN_EXTRUDERS
        if (g_nAlignExtrudersStatus)
            problematisch = false; //das homing passiert in Z einzeln, liegt aber neben dem Bett.
#endif                             // FEATURE_ALIGN_EXTRUDERS
#if FEATURE_HEAT_BED_Z_COMPENSATION
        if (g_nHeatBedScanStatus)
            problematisch = false; //das homing passiert in Z einzeln, liegt aber neben dem Bett.
#endif                             //FEATURE_HEAT_BED_Z_COMPENSATION
        if (problematisch)
            return 0; //während Z-Scan gibts einen homeZ, der ist aber nicht relevant, den case gibts nicht!
        else
            return 1;
    } // isZHomeSafe

    static int8_t anyHomeDir(uint8_t axis);
#if FEATURE_CHECK_HOME
    static bool anyEndstop(uint8_t axis);
    static void changeAxisDirection(uint8_t axis, int8_t direction);
    static void startAxisStep(uint8_t axis);
    static void endAxisStep(uint8_t axis);
    static void stepAxisStep(uint8_t axis, uint8_t slower = 1);
    static int8_t checkHome(int8_t axis);
#endif //FEATURE_CHECK_HOME

    static INLINE bool areAxisHomed() //X_AXIS && Y_AXIS && Z_AXIS
    {
        return (bool)(Printer::isAxisHomed(Z_AXIS) && Printer::isAxisHomed(Y_AXIS) && Printer::isAxisHomed(X_AXIS));
    } // areAxisHomed

    static inline void setHomed(int8_t x = -1, int8_t y = -1, int8_t z = -1) {
        if (x != -1) {
            x = (g_uBlockCommands ? false : x); // block commands while homing does break moves so no safe sethome is possible
            flag3 = (x ? flag3 | PRINTER_FLAG3_X_HOMED : flag3 & ~PRINTER_FLAG3_X_HOMED);
        }
        if (y != -1) {
            y = (g_uBlockCommands ? false : y); // block commands while homing does break moves so no safe sethome is possible
            flag3 = (y ? flag3 | PRINTER_FLAG3_Y_HOMED : flag3 & ~PRINTER_FLAG3_Y_HOMED);
        }
        if (z != -1) {
            z = (g_uBlockCommands ? false : z); // block commands while homing does break moves so no safe sethome is possible
            flag3 = (z ? flag3 | PRINTER_FLAG3_Z_HOMED : flag3 & ~PRINTER_FLAG3_Z_HOMED);
        }
        if (x == false)
            Printer::outOfPrintVolume[X_AXIS] = 0;
        if (y == false)
            Printer::outOfPrintVolume[Y_AXIS] = 0;
        if (z == false)
            Printer::outOfPrintVolumeZ = 0;
#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
        if (!isAxisHomed(Z_AXIS)) {
            Printer::disableCMPnow(true); //true == wait for move while HOMING
        }
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
    }  // setHomed

    static INLINE uint8_t isZOriginSet() {
        return flag1 & PRINTER_FLAG1_Z_ORIGIN_SET;
    } // isZOriginSet

    static INLINE void setZOriginSet(uint8_t b) {
        flag1 = (b ? flag1 | PRINTER_FLAG1_Z_ORIGIN_SET : flag1 & ~PRINTER_FLAG1_Z_ORIGIN_SET);
    } // setZOriginSet

    static INLINE uint8_t isAllSwitchedOff() {
        return flag1 & PRINTER_FLAG1_ALLSWITCHEDOFF;
    } // isAllSwitchedOff

    static INLINE void setAllSwitchedOff(uint8_t b) {
        flag1 = (b ? flag1 | PRINTER_FLAG1_ALLSWITCHEDOFF : flag1 & ~PRINTER_FLAG1_ALLSWITCHEDOFF);
    } // setAllSwitchedOff

    static INLINE uint8_t isAutomount() {
        return flag1 & PRINTER_FLAG1_AUTOMOUNT;
    } // isAutomount

    static INLINE void setAutomount(uint8_t b) {
        flag1 = (b ? flag1 | PRINTER_FLAG1_AUTOMOUNT : flag1 & ~PRINTER_FLAG1_AUTOMOUNT);
    } // setAutomount

    static INLINE uint8_t isAnimation() {
        return flag1 & PRINTER_FLAG1_ANIMATION;
    } // isAnimation

    static INLINE void setAnimation(uint8_t b) {
        flag1 = (b ? flag1 | PRINTER_FLAG1_ANIMATION : flag1 & ~PRINTER_FLAG1_ANIMATION);
    } // setAnimation

    static INLINE uint8_t isUIErrorMessage() {
        return flag1 & PRINTER_FLAG1_UI_ERROR_MESSAGE;
    } // isUIErrorMessage

    static INLINE void setUIErrorMessage(uint8_t b) {
        flag1 = (b ? flag1 | PRINTER_FLAG1_UI_ERROR_MESSAGE : flag1 & ~PRINTER_FLAG1_UI_ERROR_MESSAGE);
    } // setUIErrorMessage

    static INLINE uint8_t isPrinting() {
        return flag3 & PRINTER_FLAG3_PRINTING;
    }

    static INLINE void setPrinting(bool b) {
        flag3 = (b ? flag3 | PRINTER_FLAG3_PRINTING : flag3 & ~PRINTER_FLAG3_PRINTING);
        if (!b) {
            Printer::setMenuMode(MENU_MODE_SD_PRINTING, b);
        }
        Printer::setMenuMode(MENU_MODE_PRINTING, b);
    }

    static INLINE void toggleAnimation() {
        setAnimation(!isAnimation());
    } // toggleAnimation

    static INLINE float convertToMM(float x) {
        return (unitIsInches ? x * 25.4 : x);
    } // convertToMM

    static INLINE bool isXMinEndstopHit() {
#if X_MIN_PIN > -1 && MIN_HARDWARE_ENDSTOP_X
        return READ(X_MIN_PIN) != ENDSTOP_X_MIN_INVERTING;
#else
        return false;
#endif // X_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_X
    }  // isXMinEndstopHit

    static INLINE bool isYMinEndstopHit() {
#if Y_MIN_PIN > -1 && MIN_HARDWARE_ENDSTOP_Y
        return READ(Y_MIN_PIN) != ENDSTOP_Y_MIN_INVERTING;
#else
        return false;
#endif // Y_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_Y
    }  // isYMinEndstopHit

    static INLINE bool isZMinEndstopHit() {
#if Z_MIN_PIN > -1 && MIN_HARDWARE_ENDSTOP_Z

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS

        if (ZEndstopType == ENDSTOP_TYPE_SINGLE) {
#if FEATURE_MILLING_MODE
            if (operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE

                // in case there is only one z-endstop and we are in operating mode "print", the z-min endstop must be connected
                return READ(Z_MIN_PIN) != ENDSTOP_Z_MIN_INVERTING;
#if FEATURE_MILLING_MODE
            } else {
                // in case there is only one z-endstop and we are in operating mode "mill", the z-min endstop is not connected and can not be detected
                return false;
            }
#endif // FEATURE_MILLING_MODE
        }

        // we end up here in case the z-min and z-max endstops are connected in a circuit
        if (READ(Z_MIN_PIN) != ENDSTOP_Z_MIN_INVERTING) {
            // either the min or the max endstop is hit
            if (ZEndstopUnknown) {
                // this is save as long as we do not allow any movement except the z-homing while ZEndstopUnknown is != 0
                return false;
            }

            if (endstopZMaxHit == ENDSTOP_IS_HIT) {
                // when the z-max endstop is hit already we know that the z-min endstop is not hit
                return false;
            }

            if (endstopZMinHit == ENDSTOP_IS_HIT) {
                // we remember that the z-min endstop is hit at the moment
                return true;
            }

            if (lastZDirection > 0) {
                // z-min was not hit and we are moving downwards, so z-min can not become hit right now
                return false;
            }

            // the last z-direction is unknown or the heat bed has been moved upwards, thus we have to assume that the z-min endstop is hit
            endstopZMinHit = ENDSTOP_IS_HIT;
            endstopZMaxHit = ENDSTOP_NOT_HIT;
            return true;
        }

        // no z endstop is hit
        if (endstopZMinHit == ENDSTOP_IS_HIT) {
            endstopZMinHit = ENDSTOP_WAS_HIT;
        }
        if (endstopZMaxHit == ENDSTOP_IS_HIT) {
            endstopZMaxHit = ENDSTOP_WAS_HIT;
        }
        ZEndstopUnknown = 0;
        return false;

#else // FEATURE_CONFIGURABLE_Z_ENDSTOPS

        return READ(Z_MIN_PIN) != ENDSTOP_Z_MIN_INVERTING;

#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

#else  //Z_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_Z
        return false;
#endif // Z_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_Z
    }  // isZMinEndstopHit

    static inline bool isXMaxSoftEndstopHit() {
        return Printer::currentXSteps > Printer::maxSoftEndstopSteps[X_AXIS];
    }

    static inline bool isYMaxSoftEndstopHit() {
        return Printer::currentYSteps > Printer::maxSoftEndstopSteps[Y_AXIS];
    }

    static inline bool isZMaxSoftEndstopHit() {
        return Printer::currentZSteps > Printer::maxSoftEndstopSteps[Z_AXIS];
    }

    static INLINE bool isXMaxEndstopHit() {
#if X_MAX_PIN > -1 && MAX_HARDWARE_ENDSTOP_X
        return READ(X_MAX_PIN) != ENDSTOP_X_MAX_INVERTING;
#else
        return isXMaxSoftEndstopHit();
#endif // X_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_X
    }  // isXMaxEndstopHit

    static INLINE bool isYMaxEndstopHit() {
#if Y_MAX_PIN > -1 && MAX_HARDWARE_ENDSTOP_Y
        return READ(Y_MAX_PIN) != ENDSTOP_Y_MAX_INVERTING;
#else
        return isYMaxSoftEndstopHit();
#endif // Y_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Y
    }  // isYMaxEndstopHit

    static inline bool isZMaxEndstopHit() {
#if Z_MAX_PIN > -1 && MAX_HARDWARE_ENDSTOP_Z

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS

        if (ZEndstopType == ENDSTOP_TYPE_SINGLE) {
#if FEATURE_MILLING_MODE
            if (operatingMode == OPERATING_MODE_MILL) {
                // in case there is only one z-endstop and we are in operating mode "mill", the z-max endstop must be connected
                return READ(Z_MAX_PIN) != ENDSTOP_Z_MAX_INVERTING;
            }
#endif // FEATURE_MILLING_MODE

            // in case there is only one z-endstop and we are in operating mode "print", the z-max endstop is not connected and can not be detected
            return isZMaxSoftEndstopHit();
        }

        // we end up here in case the z-min and z-max endstops are connected in a circuit
        if (READ(Z_MAX_PIN) != ENDSTOP_Z_MAX_INVERTING) {
            // either the min or the max endstop is hit
            if (ZEndstopUnknown) {
                // this is save as long as we do not allow any movement except the z-homing while ZEndstopUnknown is != 0
                return false;
            }

            if (endstopZMinHit == ENDSTOP_IS_HIT) {
                // when the z-min endstop is hit already we know that the z-max endstop is not hit
                return false;
            }

            if (endstopZMaxHit == ENDSTOP_IS_HIT) {
                // we remember that the z-max endstop is hit at the moment
                return true;
            }

            if (lastZDirection < 0) {
                // z-max was not hit and we are moving upwards, so z-max can not become hit right now
                return false;
            }

            if (Printer::isAxisHomed(Z_AXIS)) {
                if (currentZSteps < long(Printer::axisStepsPerMM[Z_AXIS]) * 5) {
                    // we are close to z-min, so z-max can not become hit right now -> Nibbels: was wenn der drucker unten aufwacht? dann ist erst currentZSteps 0 und .. ??? TODO
                    return false;
                }
            }

            // the last z-direction is unknown or the heat bed has been moved downwards, thus we have to assume that the z-max endstop is hit
            endstopZMinHit = ENDSTOP_NOT_HIT;
            endstopZMaxHit = ENDSTOP_IS_HIT;
            return true;
        }

        // no z endstop is hit
        if (endstopZMinHit == ENDSTOP_IS_HIT) {
            endstopZMinHit = ENDSTOP_WAS_HIT;
        }
        if (endstopZMaxHit == ENDSTOP_IS_HIT) {
            endstopZMaxHit = ENDSTOP_WAS_HIT;
        }
        ZEndstopUnknown = 0;
        return false;

#else
        return READ(Z_MAX_PIN) != ENDSTOP_Z_MAX_INVERTING;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

#else
        return isZMaxSoftEndstopHit();
#endif // Z_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Z
    }  // isZMaxEndstopHit

    static INLINE bool isMinEndstopHit(uint8_t axis) {
        if (axis == X_AXIS)
            return isXMinEndstopHit();
        if (axis == Y_AXIS)
            return isYMinEndstopHit();
        if (axis == Z_AXIS)
            return isZMinEndstopHit();
        return false;
    }
    static INLINE bool isMaxEndstopHit(uint8_t axis) {
        if (axis == X_AXIS)
            return isXMaxEndstopHit();
        if (axis == Y_AXIS)
            return isYMaxEndstopHit();
        if (axis == Z_AXIS)
            return isZMaxEndstopHit();
        return false;
    }

    static INLINE bool areAllSteppersDisabled() {
        return flag0 & PRINTER_FLAG0_STEPPER_DISABLED;
    } // areAllSteppersDisabled

    static INLINE void markAllSteppersDisabled() {
        flag0 |= PRINTER_FLAG0_STEPPER_DISABLED;

#if FAN_BOARD_PIN > -1
        pwm_pos[NUM_EXTRUDER + 1] = 0;
#endif // FAN_BOARD_PIN
    }  // markAllSteppersDisabled

    static INLINE void unmarkAllSteppersDisabled() {
        flag0 &= ~PRINTER_FLAG0_STEPPER_DISABLED;

#if FAN_BOARD_PIN > -1
        pwm_pos[NUM_EXTRUDER + 1] = 255;
#endif // FAN_BOARD_PIN
    }  // unmarkAllSteppersDisabled

    static void disableAllSteppersNow() {
        if (!areAllSteppersDisabled()) {
            UI_STATUS_UPD(UI_TEXT_STEPPER_DISABLED);
        }
#if FEATURE_UNLOCK_MOVEMENT
        Printer::g_unlock_movement = 0; //again lock movement until homing or keypress or another print happens. --> toooooo much? Ich aktiviers: http://www.rf1000.de/viewtopic.php?f=70&t=2282
#endif                                  //FEATURE_UNLOCK_MOVEMENT
        disableXStepper();
        disableYStepper();
        disableZStepper();
        markAllSteppersDisabled();
        Extruder::disableAllExtruders();
    } // disableAllSteppersNow

    static INLINE void setSomeTempsensorDefect(bool defect) {
        Printer::flag0 = (defect ? flag0 | PRINTER_FLAG0_TEMPSENSOR_DEFECT : flag0 & ~PRINTER_FLAG0_TEMPSENSOR_DEFECT);
    } // setSomeTempsensorDefect

    static INLINE bool isAnyTempsensorDefect() {
#if FEATURE_MILLING_MODE
        if (Printer::operatingMode != OPERATING_MODE_PRINT) {
            // we do not support temperature sensors in case we are not in operating mode print
            return 0;
        }
#endif // FEATURE_MILLING_MODE

        return (flag0 & PRINTER_FLAG0_TEMPSENSOR_DEFECT);
    } // isAnyTempsensorDefect

    static INLINE bool isManualMoveMode() {
        return (flag0 & PRINTER_FLAG0_MANUAL_MOVE_MODE);
    } // isManualMoveMode

    static INLINE void setManualMoveMode(bool on) {
        flag0 = (on ? flag0 | PRINTER_FLAG0_MANUAL_MOVE_MODE : flag0 & ~PRINTER_FLAG0_MANUAL_MOVE_MODE);
    } // setManualMoveMode

    static inline float currentZPositionMM() {
        if (Printer::ZMode == Z_VALUE_MODE_Z_MIN) {
            // show the Enstop distance
            return Printer::currentZSteps * Printer::axisMMPerSteps[Z_AXIS];
        }

        //		//TODO: Das stimmt bestimmt nicht :D -> Nur fürs Milling und sollte sich von oben Z_VALUE_MODE_Z_MIN unterscheiden.
        //		if (Printer::ZMode == Z_VALUE_MODE_Z_ORIGIN)
        //		{
        //			// return all values in [mm]
        //			long fvalue = Printer::currentZSteps;
        //#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
        //			// add the current z-compensation
        //			fvalue += Printer::compensatedPositionCurrentStepsZ; //da drin: zoffset + senseoffset + digitcompensation
        //			fvalue += g_nZScanZPosition;
        //#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
        //
        //#if FEATURE_FIND_Z_ORIGIN
        //			fvalue += g_nZOriginPosition[Z_AXIS];
        //#endif // FEATURE_FIND_Z_ORIGIN
        //
        //			return float(fvalue * Printer::axisMMPerSteps[Z_AXIS]);
        //		}

        // Z_VALUE_MODE_LAYER:
        // show the G-Code Commanded Z
        return Printer::currentSteps[Z_AXIS] * Printer::axisMMPerSteps[Z_AXIS];
    } // currentZPositionMM

#if NUM_EXTRUDER > 1
    static INLINE float getMaxExtruderOffsetMM(uint8_t axis) {
        float off = 0.0f;
        // Reposition extruder that way, that all extruders can be selected at home pos.
        for (uint8_t i = 0; i < NUM_EXTRUDER; i++)
            off = RMath::max(off, fabs(extruder[i].offsetMM[axis]));

        return off;
    }
#endif

    static void updateDerivedParameter();
    static void switchEverythingOff();
    static void updateAdvanceActivated();
    static bool isZMoveActive();

    static INLINE void setXAxisSteps(int32_t x) {
        InterruptProtectedBlock noInts;
        //G92 Xx:
        Printer::currentSteps[X_AXIS] = x;
        Printer::destinationMMLast[X_AXIS] = Printer::destinationMM[X_AXIS] = x * axisMMPerSteps[X_AXIS];
        noInts.unprotect();
    }
    static INLINE void setYAxisSteps(int32_t y) {
        InterruptProtectedBlock noInts;
        //G92 Yx:
        Printer::currentSteps[Y_AXIS] = y;
        Printer::destinationMMLast[Y_AXIS] = Printer::destinationMM[Y_AXIS] = y * axisMMPerSteps[Y_AXIS];
        noInts.unprotect();
    }
    static INLINE void setZAxisSteps(int32_t z) {
        InterruptProtectedBlock noInts;
        //G92 Zx:
        Printer::currentSteps[Z_AXIS] = z;
        Printer::destinationMMLast[Z_AXIS] = Printer::destinationMM[Z_AXIS] = z * axisMMPerSteps[Z_AXIS];
        noInts.unprotect();
    }
    static INLINE void setEAxisSteps(int32_t e) {
        InterruptProtectedBlock noInts;
        //G92 Ex:
        // Vorsicht, currentSteps[E_AXIS] wäre Overflow.
        Printer::destinationMMLast[E_AXIS] = Printer::destinationMM[E_AXIS] = e * axisMMPerSteps[E_AXIS];
        noInts.unprotect();
    }

    static INLINE void stopDirectAxis(uint8_t axis) {
        Printer::directDestinationSteps[axis] = Printer::directCurrentSteps[axis];
    }
    static INLINE void resetDirectAxis(uint8_t axis) {
        Printer::directDestinationSteps[axis] = Printer::directCurrentSteps[axis] = 0;
    }

    static int32_t getPlannedDirectAxisSteps(uint8_t axis) {
        InterruptProtectedBlock noInts;
        int32_t plannedAxisSteps = Printer::currentSteps[axis];
        plannedAxisSteps += Printer::directDestinationSteps[axis];
        noInts.unprotect();

        return plannedAxisSteps;
    }

    static void setup();
    static void setFeedrate(float feedrate);
    static bool queueGCodeCoordinates(GCode* com, bool noDriving = false);
    static void queueFloatCoordinates(float x, float y, float z, float e, float feedrate);
    static void queueRelativeStepsCoordinates(long x, long y, long z, long e, float feedrate, bool waitEnd, bool abortAtEndstops = false);
    static void queueRelativeMMCoordinates(float x, float y, float z, float e, float feedrate, bool waitEnd, bool abortAtEndstops = false);
    static void offsetRelativeStepsCoordinates(int32_t dx, int32_t dy, int32_t dz, int32_t de, uint8_t configuration = 0);
    static void homeDigits();
    static void homeAxis(bool xaxis, bool yaxis, bool zaxis); /// Home axis
    static void setOrigin(float xOff, float yOff, float zOff);
    static void addKurtWobbleFixOffset();

    static INLINE long getDestinationSteps(uint8_t axis) {
        return lroundf(Printer::destinationMM[axis] * Printer::axisStepsPerMM[axis]);
    } // getDestinationSteps

    static INLINE float getDirectMM(uint8_t axis) {
        return (float)Printer::directDestinationSteps[axis] * Printer::axisMMPerSteps[axis];
    } // getDirectMM

    static INLINE uint8_t getFanSpeed(bool percent = false) {
        if (!percent)
            return fanSpeed; //int
        if (!fanSpeed)
            return 0; //0%
        if (fanSpeed <= 3)
            return 1;                                       //1%
        return (uint8_t)(((uint16_t)fanSpeed * 100) / 255); //%
    }                                                       // getFanSpeed

#if FEATURE_MEMORY_POSITION
    static void MemoryPosition();
    static void GoToMemoryPosition(bool x, bool y, bool z, bool e, float feed);
#endif // FEATURE_MEMORY_POSITION

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
#if FEATURE_SENSIBLE_PRESSURE
    static void enableSenseOffsetnow(void);
#endif // FEATURE_SENSIBLE_PRESSURE
    static bool needsCMPwait(void);
    static bool checkCMPblocked(void);
    static void enableCMPnow(void);
    static void disableCMPnow(bool wait = false);
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

    static void stopPrint();
    static bool checkAbortKeys(void);
    static bool checkPlayKey(void);
    static void showConfiguration();

private:
    static void homeXAxis();
    static void homeYAxis();
    static void homeZAxis();
};

#endif // PRINTER_H

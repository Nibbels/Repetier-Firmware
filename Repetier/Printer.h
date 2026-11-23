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

#include "ZOffset.h"
#include "SenseOffsetControl.h"
#include "ZMotion.h"
#include "PrintLayers.h"
#include "PrintPause.h"
#include "PrintStop.h"
#include "DigitCompensation.h"
#include "PressureProtection.h"
#include "StrainGauge.h"
#include "StartLine.h"
#include "ExtrusionPlanner.h"
#include "ExtrusionCompensation.h"
#include "PressureAdvance.h"
#include "MoveTarget.h"
#include "WobbleCorrection.h"
#include "CartesianPlanner.h"
#include "XYMotion.h"

// ##########################################################################################
// ##   status flags
// ##########################################################################################

#define PRINTER_FLAG0_STEPPER_DISABLED 1
#define PRINTER_FLAG0_SEPERATE_EXTRUDER_INT 2
#define PRINTER_FLAG0_TEMPSENSOR_DEFECT 4
#define PRINTER_FLAG0_MANUAL_MOVE_MODE 16
#define PRINTER_FLAG0_LARGE_MACHINE 128

#define PRINTER_FLAG1_ANIMATION 4
#define PRINTER_FLAG1_ALLSWITCHEDOFF 8
#define PRINTER_FLAG1_UI_ERROR_MESSAGE 16

#define PRINTER_FLAG2_RESET_FILAMENT_USAGE 4
#define PRINTER_FLAG2_GOT_TEMPS 32

#define PRINTER_FLAG3_X_HOMED 1  // flag3 alike original repetier
#define PRINTER_FLAG3_Y_HOMED 2  // flag3 alike original repetier
#define PRINTER_FLAG3_PRINTING 8 // flag3 alike original repetier

#define KOSYS_GCODE true
#define KOSYS_DIRECTOFFSET false

#define FEEDRATE_GCODE true
#define FEEDRATE_DIRECTCONFIG false

class Printer {
public:
    static PressureAdvance pressureAdvance;

    static uint8_t menuMode;
    static float axisStepsPerMM[];
    static float axisMMPerSteps[];
    static float maxFeedrate[];
    static float homingFeedrate[];
    static float maxAccelerationMMPerSquareSecond[];
    static float maxTravelAccelerationMMPerSquareSecond[];
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
    static uint16_t stepsPackingMinInterval;
    static volatile float v;                // Last planned printer speed.

    static long maxSoftEndstopSteps[2];     // X/Y software endstops. Z limits belong to zMotion.
    static float axisLengthMM[3];           // Länge des überfahrbaren Bereichs im positiven Homing. (=Schienen-Fahrweg - Homing-Offset - 2x ExtruderOffset)
    static float feedrate;                  // Last requested feedrate.
    static int feedrateMultiply;            // Multiplier for feedrate in percent (factor 1 = 100)
    static float menuExtrusionFactor;       // Flow multiplier factor (1.0 = 100%)
    static float maxXYJerk;                 // Maximum allowed jerk in mm/s
    static float maxZJerk;                  // Maximum allowed jerk in z direction in mm/s
    static unsigned long msecondsPrinting;  // Milliseconds of printing time (means time with heated extruder)
    static float filamentPrinted;           // mm of filament printed since counting started
    static ZOffset zOffset;
    static SenseOffsetControl senseOffsetControl;
    static ZMotion zMotion;
    static PrintLayers layers;
    static DigitCompensation digitCompensation;
    static PressureProtection pressureProtection;
    static StrainGauge strainGauge;
    static StartLine startLine;
    static ExtrusionPlanner extrusionPlanner;
    static ExtrusionCompensation extrusionCompensation;
    static MoveTarget moveTarget;
    static WobbleCorrection wobbleCorrection;
    static CartesianPlanner cartesianPlanner;
    static XYMotion xyMotion;
    static PrintPause pause;
    static PrintStop printStop;
    static char ZMode;                      // Z Scale
    static char moveMode[3];                // move mode which is applied within the Position X/Y/Z menus
    static bool moveKosys;                  // true = GCode, false = DirectMove / OffsetMove
    static bool movePositionFeedrateChoice; // select the feedrate for menu positioning: feedrate from last gcode or standard speed

    static volatile char doHeatBedZCompensation;

    static volatile char blockAll;

    static char enabledStepper[3];

    static char enableCaseLight;

#ifdef RGB_LIGHT_R_PIN
    static char RGBLightMode;
    static char RGBLightStatus;
    static unsigned long RGBLightIdleStart;
    static char RGBButtonBackPressed;
    static char RGBLightModeForceWhite;
#endif // RGB_LIGHT_R_PIN

#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    static char enable230VOutput;

    static char enableFET1;
    static char enableFET2;
    static char enableFET3;

    static bool ignoreFanOn;
    static millis_t prepareFanOff;
    static unsigned long fanOffDelay;
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2

    static unsigned char wrongType;

    static unsigned char g_unlock_movement;


    static uint8_t motorCurrent[DRV8711_NUM_CHANNELS];


    static uint8_t motorMicroStepsModeValue[DRV8711_NUM_CHANNELS]; //1=2MS, 2=4MS, 3=8MS, 4=16MS, 5=32MS, 6=64MS, 7=128MS, 8=256MS

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

        Printer::enabledStepper[X_AXIS] = 0;

        // when the stepper is disabled we loose our home position because somebody else can move our mechanical parts
        setHomed(false, -1, -1);
        InterruptProtectedBlock noInts; //HAL::forbidInterrupts();

        Printer::setXAxisSteps(0);
        Printer::xyMotion.setMotorSteps(X_AXIS, 0);
        Printer::resetDirectAxis(X_AXIS);

        pause.cancel();

        noInts.unprotect(); //HAL::allowInterrupts();

        Commands::printCurrentPosition();
    } // disableXStepper

    /** \brief Disable stepper motor for y direction. */
    static INLINE void disableYStepper() {
#if (Y_ENABLE_PIN > -1)
        WRITE(Y_ENABLE_PIN, !Y_ENABLE_ON);
#endif // (Y_ENABLE_PIN > -1)

        Printer::enabledStepper[Y_AXIS] = 0;

        // when the stepper is disabled we loose our home position because somebody else can move our mechanical parts
        setHomed(-1, false, -1);

        InterruptProtectedBlock noInts; //HAL::forbidInterrupts();

        Printer::setYAxisSteps(0);
        Printer::xyMotion.setMotorSteps(Y_AXIS, 0);
        Printer::resetDirectAxis(Y_AXIS);

        pause.cancel();

        noInts.unprotect(); //HAL::allowInterrupts();

        Commands::printCurrentPosition();
    } // disableYStepper

    /** \brief Disable stepper motor for z direction. */
    static INLINE void disableZStepper() {
        InterruptProtectedBlock noInts;
        zMotion.clearScanAndCorrection();
        noInts.unprotect();

        // when the stepper is disabled we loose our home position because somebody else can move our mechanical parts
        setHomed(-1, -1, false); // disable CMP mit wait ist bei unhome Z mit drin. //Printer::disableCMPnow(true); //fahre vom heizbett auf 0 bevor stepper aus.

#if (Z_ENABLE_PIN > -1)
        WRITE(Z_ENABLE_PIN, !Z_ENABLE_ON);
#endif // (Z_ENABLE_PIN > -1)

        Printer::enabledStepper[Z_AXIS] = 0;

        noInts.protect();
        Printer::setZAxisSteps(0);
        zMotion.setMotorReference(0);
        zMotion.resetDirectPosition();

        layers.clearConfirmedLayers();

        pause.cancel();

        noInts.unprotect();

        Commands::printCurrentPosition();
    } // disableZStepper

    /** \brief Enable stepper motor for x direction. */
    static INLINE void enableXStepper() {
        unmarkAllSteppersDisabled();
#if (X_ENABLE_PIN > -1)
        WRITE(X_ENABLE_PIN, X_ENABLE_ON);
#endif // (X_ENABLE_PIN > -1)

        if (!Printer::enabledStepper[X_AXIS]) {
            Printer::enabledStepper[X_AXIS] = 1;
            HAL::delayMilliseconds(stepperEnableDelayMs);
        }
    }  // enableXStepper

    /** \brief Enable stepper motor for y direction. */
    static INLINE void enableYStepper() {
        unmarkAllSteppersDisabled();
#if (Y_ENABLE_PIN > -1)
        WRITE(Y_ENABLE_PIN, Y_ENABLE_ON);
#endif // (Y_ENABLE_PIN > -1)

        if (!Printer::enabledStepper[Y_AXIS]) {
            Printer::enabledStepper[Y_AXIS] = 1;
            HAL::delayMilliseconds(stepperEnableDelayMs);
        }
    }  // enableYStepper

    /** \brief Enable stepper motor for z direction. */
    static INLINE void enableZStepper() {
        unmarkAllSteppersDisabled();
#if (Z_ENABLE_PIN > -1)
        WRITE(Z_ENABLE_PIN, Z_ENABLE_ON);
#endif // (Z_ENABLE_PIN > -1)

        if (!Printer::enabledStepper[Z_AXIS]) {
            Printer::enabledStepper[Z_AXIS] = 1;
            HAL::delayMilliseconds(stepperEnableDelayMs);
        }
    }  // enableZStepper

    static INLINE void setXDirection(bool positive) {
        if (positive) {
            // extruder moves to the right
            WRITE(X_DIR_PIN, !INVERT_X_DIR);
        } else {
            // extruder moves to the left
            WRITE(X_DIR_PIN, INVERT_X_DIR);
        }
    } // setXDirection

    static INLINE void setYDirection(bool positive) {
        if (positive) {
            // heat bed moves to the front
            WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
        } else {
            // heat bed moves to the back
            WRITE(Y_DIR_PIN, INVERT_Y_DIR);
        }
    } // setYDirection

    static INLINE void startXStep(int8_t dir = (Printer::getXDirectionIsPos() ? 1 : -1)) {
        WRITE(X_STEP_PIN, HIGH);
        Printer::xyMotion.recordMotorStep(X_AXIS, dir);
    } // startXStep

    static INLINE void startYStep(int8_t dir = (Printer::getYDirectionIsPos() ? 1 : -1)) {
        WRITE(Y_STEP_PIN, HIGH);
        Printer::xyMotion.recordMotorStep(Y_AXIS, dir);
    } // startYStep

    static INLINE void endYStep(void) {
        WRITE(Y_STEP_PIN, LOW);
    } // endZStep

    static INLINE void endXStep(void) {
        WRITE(X_STEP_PIN, LOW);
    } // endZStep

    static INLINE void endXYZSteps() {
        endXStep();
        endYStep();
        ZMotion::finishPulse();
    } // endXYZSteps

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
            return zMotion.isHomed();
        }
        }
        return 0;
    } // isAxisHomed

    static inline uint8_t isZHomeSafe() //experimentelle funktion, die nicht viel abdeckt, das ist ein test. ... TODO: merge with function isHomingAllowed
    {
        bool problematisch = false;
        if (g_offsetZCompensationSteps > 0)
            problematisch = true; //wenn matrix positiv, dann evtl. problem
        if (isAxisHomed(Y_AXIS) && Printer::xyMotion.motorSteps(Y_AXIS) <= 5 * YAXIS_STEPS_PER_MM)
            problematisch = false; //vorherige Probleme egal, wenn bett nach hinten gefahren
        if (g_nHeatBedScanStatus)
            problematisch = false; //das homing passiert in Z einzeln, liegt aber neben dem Bett.
        if (problematisch)
            return 0; //während Z-Scan gibts einen homeZ, der ist aber nicht relevant, den case gibts nicht!
        else
            return 1;
    } // isZHomeSafe

    static int8_t anyHomeDir(uint8_t axis);
    static bool anyEndstop(uint8_t axis);
    static void changeAxisDirection(uint8_t axis, int8_t direction);
    static void stepAxisStep(uint8_t axis, uint8_t slower = 1);
    static int8_t checkHome(int8_t axis); // M3028 homing repeatability test.

    static INLINE bool areAxisHomed() //X_AXIS && Y_AXIS && Z_AXIS
    {
        return (bool)(Printer::isAxisHomed(Z_AXIS) && Printer::isAxisHomed(Y_AXIS) && Printer::isAxisHomed(X_AXIS));
    } // areAxisHomed

    static inline void setHomed(int8_t x = -1, int8_t y = -1, int8_t z = -1) {
        if (x != -1) {
            x = printStop.commandsBlocked() ? false : x; // A stop can interrupt homing.
            flag3 = (x ? flag3 | PRINTER_FLAG3_X_HOMED : flag3 & ~PRINTER_FLAG3_X_HOMED);
        }
        if (y != -1) {
            y = printStop.commandsBlocked() ? false : y;
            flag3 = (y ? flag3 | PRINTER_FLAG3_Y_HOMED : flag3 & ~PRINTER_FLAG3_Y_HOMED);
        }
        if (z != -1)
            zMotion.setHomed(!printStop.commandsBlocked() && z);
        if (x == false)
            xyMotion.clearQueuedOvertravel(X_AXIS);
        if (y == false)
            xyMotion.clearQueuedOvertravel(Y_AXIS);
        if (!isAxisHomed(Z_AXIS)) {
            Printer::disableCMPnow(true); //true == wait for move while HOMING
        }
    }  // setHomed

    static INLINE uint8_t isAllSwitchedOff() {
        return flag1 & PRINTER_FLAG1_ALLSWITCHEDOFF;
    } // isAllSwitchedOff

    static INLINE void setAllSwitchedOff(uint8_t b) {
        flag1 = (b ? flag1 | PRINTER_FLAG1_ALLSWITCHEDOFF : flag1 & ~PRINTER_FLAG1_ALLSWITCHEDOFF);
    } // setAllSwitchedOff

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
        Printer::setMenuMode(MENU_MODE_PRINTING, b);
    }

    static INLINE void toggleAnimation() {
        setAnimation(!isAnimation());
    } // toggleAnimation

    static INLINE float convertToMM(float x) {
        return (unitIsInches ? x * 25.4 : x);
    } // convertToMM

    static INLINE bool isXMinEndstopHit() {
        return READ(X_MIN_PIN) == LOW;
    }  // isXMinEndstopHit

    static INLINE bool isYMinEndstopHit() {
        return READ(Y_MIN_PIN) == LOW;
    }  // isYMinEndstopHit

    static INLINE bool isZMinEndstopHit() {
        return ZMotion::minSwitchHit();
    }  // isZMinEndstopHit

    static INLINE bool isXMaxEndstopHit() {
        return Printer::xyMotion.motorSteps(X_AXIS) >= Printer::maxSoftEndstopSteps[X_AXIS];
    }  // isXMaxEndstopHit

    static INLINE bool isYMaxEndstopHit() {
        return Printer::xyMotion.motorSteps(Y_AXIS) >= Printer::maxSoftEndstopSteps[Y_AXIS];
    }  // isYMaxEndstopHit

    static inline bool isZMaxEndstopHit() {
        return zMotion.maxEndstopHit();
    }  // isZMaxEndstopHit

    static inline int32_t softwareEndstopSteps(uint8_t axis) {
        return axis == Z_AXIS ? zMotion.maxSteps() : maxSoftEndstopSteps[axis];
    }

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
        Printer::g_unlock_movement = 0; //again lock movement until homing or keypress or another print happens. --> toooooo much? Ich aktiviers: http://www.rf1000.de/viewtopic.php?f=70&t=2282
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
            return zMotion.motorSteps() * Printer::axisMMPerSteps[Z_AXIS];
        }

        // Z_VALUE_MODE_LAYER:
        // show the G-Code Commanded Z
        return zMotion.nominalSteps() * Printer::axisMMPerSteps[Z_AXIS];
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

    static INLINE void setXAxisSteps(int32_t x) {
        InterruptProtectedBlock noInts;
        cartesianPlanner.setAxisSteps(X_AXIS, x);
        xyMotion.setNominalSteps(X_AXIS, x);
        moveTarget.setMachineMM(X_AXIS, x * axisMMPerSteps[X_AXIS]);
        wobbleCorrection.clearAxis(X_AXIS);
        noInts.unprotect();
    }
    static INLINE void setYAxisSteps(int32_t y) {
        InterruptProtectedBlock noInts;
        cartesianPlanner.setAxisSteps(Y_AXIS, y);
        xyMotion.setNominalSteps(Y_AXIS, y);
        moveTarget.setMachineMM(Y_AXIS, y * axisMMPerSteps[Y_AXIS]);
        wobbleCorrection.clearAxis(Y_AXIS);
        noInts.unprotect();
    }
    static INLINE void setZAxisSteps(int32_t z) {
        InterruptProtectedBlock noInts;
        zMotion.setNominalSteps(z);
        cartesianPlanner.setAxisSteps(Z_AXIS, z);
        moveTarget.setMachineMM(Z_AXIS, z * axisMMPerSteps[Z_AXIS]);
        noInts.unprotect();
    }
    static INLINE void setEPositionMM(float e, float remainder = 0.0f) {
        moveTarget.setMachineMM(E_AXIS, e, remainder);
        extrusionPlanner.setLogicalPosition({e, remainder});
    }

    static INLINE void stopDirectAxis(uint8_t axis) {
        if (axis == Z_AXIS)
            zMotion.stopDirectMove();
        else if (axis == E_AXIS)
            extruderDirect.stop();
        else
            xyMotion.stopDirect(axis);
    }
    static INLINE void resetDirectAxis(uint8_t axis) {
        if (axis == Z_AXIS)
            zMotion.resetDirectPosition();
        else if (axis == E_AXIS)
            extruderDirect.reset();
        else
            xyMotion.resetDirect(axis);
    }
    static INLINE const DirectPosition& directPosition(uint8_t axis) {
        if (axis == Z_AXIS)
            return zMotion.directPosition();
        return axis == E_AXIS ? extruderDirect : xyMotion.directPosition(axis);
    }
    static INLINE int32_t directSteps(uint8_t axis) {
        InterruptProtectedBlock lock;
        return directPosition(axis).steps();
    }

    static int32_t getPlannedDirectAxisSteps(uint8_t axis) {
        InterruptProtectedBlock noInts;
        int32_t plannedAxisSteps = axis == Z_AXIS ? zMotion.nominalSteps() : xyMotion.nominalSteps(axis);
        plannedAxisSteps += directPosition(axis).target();
        noInts.unprotect();

        return plannedAxisSteps;
    }

    static void setup();
    static void setFeedrate(float feedrate);
    static bool queueGCodeCoordinates(GCode* com);
    static void queueFloatCoordinates(float x, float y, float z, float e, float feedrate);
    static void queueRelativeStepsCoordinates(long x, long y, long z, long e, float feedrate, bool waitEnd, bool abortAtEndstops = false);
    static void queueRelativeMMCoordinates(float x, float y, float z, float e, float feedrate, bool waitEnd, bool abortAtEndstops = false);
    static void offsetRelativeStepsCoordinates(int32_t dx, int32_t dy, int32_t dz, int32_t de, uint8_t configuration = 0);
    static void homeDigits();
    static void homeAxis(bool xaxis, bool yaxis, bool zaxis); /// Home axis
    static void setOrigin(float xOff, float yOff, float zOff);
    static void updateWobbleFixOffsets();
    static int32_t prepareDestinationSteps(uint8_t axis);

    static INLINE float getDirectMM(uint8_t axis) {
        InterruptProtectedBlock lock;
        const int32_t target = directPosition(axis).target();
        lock.unprotect();
        return target * Printer::axisMMPerSteps[axis];
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

    static void enableSenseOffsetnow(void);
    static bool needsCMPwait(void);
    static void enableCMPnow(void);
    static void disableCMPnow(bool wait = false);

    static void stopPrint();
    static bool checkAbortKeys(void);
    static bool checkPlayKey(void);
    static void showConfiguration();

private:
    static DirectPosition extruderDirect;

    static void homeXAxis();
    static void homeYAxis();
    static void homeZAxis();
};

#endif // PRINTER_H

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

#include "ManualMotion.h"
#include "GCodeModes.h"

#include "ZOffset.h"
#include "SenseOffsetControl.h"
#include "ZMotion.h"
#include "PrintLayers.h"
#include "PrintPause.h"
#include "PrintJob.h"
#include "MachineInactivity.h"
#include "IdleStatus.h"
#include "DigitCompensation.h"
#include "PressureProtection.h"
#include "PreheatProfiles.h"
#include "PrintUsage.h"
#include "StrainGauge.h"
#include "StartLine.h"
#include "ExtrusionPlanner.h"
#include "ExtrusionCompensation.h"
#include "ExtrusionOutput.h"
#include "MoveTarget.h"
#include "WobbleCorrection.h"
#include "CartesianPlanner.h"
#include "XYMotion.h"

// ##########################################################################################
// ##   status flags
// ##########################################################################################

#define PRINTER_FLAG0_STEPPER_DISABLED 1
#define PRINTER_FLAG0_TEMPSENSOR_DEFECT 4
#define PRINTER_FLAG0_MANUAL_MOVE_MODE 16
#define PRINTER_FLAG0_LARGE_MACHINE 128

#define PRINTER_FLAG1_ANIMATION 4
#define PRINTER_FLAG1_ALLSWITCHEDOFF 8
#define PRINTER_FLAG1_UI_ERROR_MESSAGE 16

#define PRINTER_FLAG2_GOT_TEMPS 32

#define PRINTER_FLAG3_X_HOMED 1  // flag3 alike original repetier
#define PRINTER_FLAG3_Y_HOMED 2  // flag3 alike original repetier

#define KOSYS_GCODE true
#define KOSYS_DIRECTOFFSET false

#define FEEDRATE_GCODE true
#define FEEDRATE_DIRECTCONFIG false

class Printer {
    static uint8_t menuMode_;
    static float axisStepsPerMM_[4];
    static float axisMMPerSteps_[4];
    static float maxFeedrate_[4];
    static float homingFeedrate_[3];
    static float axisLengthMM_[3];
    static float maxAccelerationMMPerSquareSecond_[4];
    static float maxTravelAccelerationMMPerSquareSecond_[4];
    static long maxSoftEndstopSteps_[2];
    static uint32_t maxPrintAccelerationStepsPerSquareSecond_[4];
    static uint32_t maxTravelAccelerationStepsPerSquareSecond_[4];
    static float maxXYJerk_;
    static float maxZJerk_;

public:
    static ExtrusionOutput extrusionOutput;
    static PreheatProfiles preheatProfiles;

    // Set raw calibration and publish the derived limits with updateDerivedParameter().
    static float stepsPerMm(uint8_t axis) { return axisStepsPerMM_[axis]; }
    static void setStepsPerMm(uint8_t axis, float value) { axisStepsPerMM_[axis] = value; }
    static float mmPerStep(uint8_t axis) { return axisMMPerSteps_[axis]; }
    static float axisMaxFeedrate(uint8_t axis) { return maxFeedrate_[axis]; }
    static void setAxisMaxFeedrate(uint8_t axis, float value) { maxFeedrate_[axis] = value; }
    static float axisHomingFeedrate(uint8_t axis) { return homingFeedrate_[axis]; }
    static void setAxisHomingFeedrate(uint8_t axis, float value) { homingFeedrate_[axis] = value; }
    static float axisLengthMm(uint8_t axis) { return axisLengthMM_[axis]; }
    static void setAxisLengthMm(uint8_t axis, float value) { axisLengthMM_[axis] = value; }
    static float printAcceleration(uint8_t axis) { return maxAccelerationMMPerSquareSecond_[axis]; }
    static void setPrintAcceleration(uint8_t axis, float value) { maxAccelerationMMPerSquareSecond_[axis] = value; }
    static float travelAcceleration(uint8_t axis) { return maxTravelAccelerationMMPerSquareSecond_[axis]; }
    static void setTravelAcceleration(uint8_t axis, float value) { maxTravelAccelerationMMPerSquareSecond_[axis] = value; }
    static long xyMaxSteps(uint8_t axis) { return maxSoftEndstopSteps_[axis]; }
    static float xyJerkLimit() { return maxXYJerk_; }
    static void setXYJerkLimit(float value) { maxXYJerk_ = value; }
    static float zJerkLimit() { return maxZJerk_; }
    static void setZJerkLimit(float value) { maxZJerk_ = value; }
    static const uint32_t* accelerationSteps(bool printing) {
        return printing ? maxPrintAccelerationStepsPerSquareSecond_ : maxTravelAccelerationStepsPerSquareSecond_;
    }
    static void applyToolMotionSettings(float stepsPerMm, float feedrate, float acceleration) {
        axisStepsPerMM_[E_AXIS] = stepsPerMm;
        axisMMPerSteps_[E_AXIS] = 1.0f / stepsPerMm;
        maxFeedrate_[E_AXIS] = feedrate;
        maxTravelAccelerationMMPerSquareSecond_[E_AXIS] = acceleration;
        maxAccelerationMMPerSquareSecond_[E_AXIS] = acceleration;
        const auto accelerationSteps = uint32_t(acceleration * stepsPerMm);
        maxTravelAccelerationStepsPerSquareSecond_[E_AXIS] = accelerationSteps;
        maxPrintAccelerationStepsPerSquareSecond_[E_AXIS] = accelerationSteps;
    }

    static GCodeModes gcodeModes;
    static uint8_t debugLevel;
    static uint8_t flag0;
    static uint8_t flag1;
    static uint8_t flag2;
    static uint8_t flag3;
    static uint16_t stepsPackingMinInterval;

    static float feedrate;                  // Last requested feedrate.
    static int feedrateMultiply;            // Multiplier for feedrate in percent (factor 1 = 100)
    static float menuExtrusionFactor;       // Flow multiplier factor (1.0 = 100%)
    static PrintUsage printUsage;
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
    static PrintJob printJob;
    static MachineInactivity inactivity;
    static IdleStatus idleStatus;
    static char ZMode;                      // Z Scale
    static ManualMotion manualMotion;

    static volatile char doHeatBedZCompensation;

    static volatile char blockAll;

    static char enabledStepper[3];

    static char enableCaseLight;


#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    static char enable230VOutput;

    static char enableFET1;
    static char enableFET2;
    static char enableFET3;

#endif // MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2

    static unsigned char wrongType;

    static bool movementUnlocked() { return movementUnlocked_; }
    static void allowMovement() { movementUnlocked_ = true; }
    static void lockMovement() { movementUnlocked_ = false; }


    static uint8_t motorCurrent[DRV8711_NUM_CHANNELS];


    static uint8_t motorMicroStepsModeValue[DRV8711_NUM_CHANNELS]; //1=2MS, 2=4MS, 3=8MS, 4=16MS, 5=32MS, 6=64MS, 7=128MS, 8=256MS

    static INLINE void setMenuMode(uint8_t mode, bool on) {
        if (on)
            menuMode_ |= mode;
        else
            menuMode_ &= ~mode;
    } // setMenuMode

    static INLINE uint8_t activeMenuModes() {
        return menuMode_ | (printJob.isBusy() ? MENU_MODE_PRINTING : 0);
    }

    static INLINE bool isMenuMode(uint8_t mode) {
        return (activeMenuModes() & mode) == mode;
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
        if (bedMatrixHighestZSteps() > 0)
            problematisch = true; //wenn matrix positiv, dann evtl. problem
        if (isAxisHomed(Y_AXIS) && Printer::xyMotion.motorSteps(Y_AXIS) <= 5 * YAXIS_STEPS_PER_MM)
            problematisch = false; //vorherige Probleme egal, wenn bett nach hinten gefahren
        if (heatBedScanRunning())
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
            x = printJob.commandsBlocked() ? false : x; // A stop can interrupt homing.
            flag3 = (x ? flag3 | PRINTER_FLAG3_X_HOMED : flag3 & ~PRINTER_FLAG3_X_HOMED);
        }
        if (y != -1) {
            y = printJob.commandsBlocked() ? false : y;
            flag3 = (y ? flag3 | PRINTER_FLAG3_Y_HOMED : flag3 & ~PRINTER_FLAG3_Y_HOMED);
        }
        if (z != -1)
            zMotion.setHomed(!printJob.commandsBlocked() && z);
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

    static INLINE void toggleAnimation() {
        setAnimation(!isAnimation());
    } // toggleAnimation

    static INLINE float convertToMM(float x) {
        return (gcodeModes.inches() ? x * 25.4 : x);
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
        return Printer::xyMotion.motorSteps(X_AXIS) >= Printer::xyMaxSteps(X_AXIS);
    }  // isXMaxEndstopHit

    static INLINE bool isYMaxEndstopHit() {
        return Printer::xyMotion.motorSteps(Y_AXIS) >= Printer::xyMaxSteps(Y_AXIS);
    }  // isYMaxEndstopHit

    static inline bool isZMaxEndstopHit() {
        return zMotion.maxEndstopHit();
    }  // isZMaxEndstopHit

    static inline int32_t softwareEndstopSteps(uint8_t axis) {
        return axis == Z_AXIS ? zMotion.maxSteps() : maxSoftEndstopSteps_[axis];
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
        HAL::setBoardFan(false);
#endif // FAN_BOARD_PIN
    }  // markAllSteppersDisabled

    static INLINE void unmarkAllSteppersDisabled() {
        flag0 &= ~PRINTER_FLAG0_STEPPER_DISABLED;

#if FAN_BOARD_PIN > -1
        HAL::setBoardFan(true);
#endif // FAN_BOARD_PIN
    }  // unmarkAllSteppersDisabled

    static void disableAllSteppersNow() {
        if (!areAllSteppersDisabled()) {
            UI_STATUS_UPD(UI_TEXT_STEPPER_DISABLED);
        }
        lockMovement(); // Homing or a panel key authorizes the next move.
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
            return zMotion.motorSteps() * Printer::mmPerStep(Z_AXIS);
        }

        // Z_VALUE_MODE_LAYER:
        // show the G-Code Commanded Z
        return zMotion.nominalSteps() * Printer::mmPerStep(Z_AXIS);
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
    static void applyAdvanceSettings();

    static INLINE void setXAxisSteps(int32_t x) {
        InterruptProtectedBlock noInts;
        cartesianPlanner.setAxisSteps(X_AXIS, x);
        xyMotion.setNominalSteps(X_AXIS, x);
        moveTarget.setMachineMM(X_AXIS, x * axisMMPerSteps_[X_AXIS]);
        wobbleCorrection.clearAxis(X_AXIS);
        noInts.unprotect();
    }
    static INLINE void setYAxisSteps(int32_t y) {
        InterruptProtectedBlock noInts;
        cartesianPlanner.setAxisSteps(Y_AXIS, y);
        xyMotion.setNominalSteps(Y_AXIS, y);
        moveTarget.setMachineMM(Y_AXIS, y * axisMMPerSteps_[Y_AXIS]);
        wobbleCorrection.clearAxis(Y_AXIS);
        noInts.unprotect();
    }
    static INLINE void setZAxisSteps(int32_t z) {
        InterruptProtectedBlock noInts;
        zMotion.setNominalSteps(z);
        cartesianPlanner.setAxisSteps(Z_AXIS, z);
        moveTarget.setMachineMM(Z_AXIS, z * axisMMPerSteps_[Z_AXIS]);
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
    static bool queueRelativeStepsCoordinates(long x, long y, long z, long e, float feedrate, bool waitEnd, bool abortAtEndstops = false);
    static bool queueRelativeMMCoordinates(float x, float y, float z, float e, float feedrate, bool waitEnd, bool abortAtEndstops = false);
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
        return target * Printer::mmPerStep(axis);
    } // getDirectMM

    static INLINE uint8_t getFanSpeed(bool percent = false) {
        if (!percent)
            return HAL::partFanSpeed(); //int
        if (!HAL::partFanSpeed())
            return 0; //0%
        if (HAL::partFanSpeed() <= 3)
            return 1;                                       //1%
        return (uint8_t)(((uint16_t)HAL::partFanSpeed() * 100) / 255); //%
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
    static bool movementUnlocked_;
    static DirectPosition extruderDirect;

    static void homeXAxis();
    static void homeYAxis();
    static void homeZAxis();
};

#endif // PRINTER_H

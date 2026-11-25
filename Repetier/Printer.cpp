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

// Machine setup, axis calibration and foreground motion admission; see MotionPlanner and StepExecutor for planning and execution.

#include "Repetier.h"
#include "GCodeInput.h"
#include "TwiTransfer.h"
#include "CaseFan.h"

ExtrusionOutput Printer::extrusionOutput;
PreheatProfiles Printer::preheatProfiles;
GCodeModes Printer::gcodeModes;
ManualMotion Printer::manualMotion(
    lroundf(XAXIS_STEPS_PER_MM * DEFAULT_MANUAL_MM_X),
    lroundf(YAXIS_STEPS_PER_MM * DEFAULT_MANUAL_MM_Y),
    lroundf(ZAXIS_STEPS_PER_MM * DEFAULT_MANUAL_MM_Z),
    lroundf(EXT0_STEPS_PER_MM * DEFAULT_MANUAL_MM_E));


//Stepper Movement Variables
float Printer::axisStepsPerMM_[4] = { XAXIS_STEPS_PER_MM, YAXIS_STEPS_PER_MM, ZAXIS_STEPS_PER_MM, 1 };                      ///< Number of steps per mm needed.
float Printer::axisMMPerSteps_[4] = { 1.0f / XAXIS_STEPS_PER_MM, 1.0f / YAXIS_STEPS_PER_MM, 1.0f / ZAXIS_STEPS_PER_MM, 1 }; ///< Inverse of axisStepsPerMM for faster conversion
float Printer::maxFeedrate_[4] = { MAX_FEEDRATE_X, MAX_FEEDRATE_Y, MAX_FEEDRATE_Z, STANDARD_POSITION_FEEDRATE_E };          ///< Maximum allowed feedrate. //STANDARD_POSITION_FEEDRATE_E added by nibbels, wird aber überschrieben.
float Printer::homingFeedrate_[3] = { HOMING_FEEDRATE_X_PRINT, HOMING_FEEDRATE_Y_PRINT, HOMING_FEEDRATE_Z_PRINT };          ///< dass zumindest etwas sinnvolles drin steht, wird überschrieben.

float Printer::maxAccelerationMMPerSquareSecond_[4] = { MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z };                            ///< X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts
float Printer::maxTravelAccelerationMMPerSquareSecond_[4] = { MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X, MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y, MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z }; ///< X, Y, Z max acceleration in mm/s^2 for travel moves

/** Acceleration in steps/s^2 in printing mode.*/
uint32_t Printer::maxPrintAccelerationStepsPerSquareSecond_[4];
/** Acceleration in steps/s^2 in movement mode.*/
uint32_t Printer::maxTravelAccelerationStepsPerSquareSecond_[4];


uint8_t Printer::flag0 = 0;
uint8_t Printer::flag1 = 0;
uint8_t Printer::flag2 = 0;
uint8_t Printer::flag3 = 0;

uint8_t Printer::debugLevel = 6; ///< Debug output: 1 echo, 2 info, 4 error; bit 8 retired.

uint8_t Printer::menuMode_ = 0;


long Printer::maxSoftEndstopSteps_[2] = { 0 };
float Printer::axisLengthMM_[3] = { 0 };       // Länge des überfahrbaren Bereichs im positiven Homing. (=Schienen-Fahrweg - Homing-Offset - 2x ExtruderOffset)
float Printer::feedrate = 10;                 // Last requested feedrate.
int Printer::feedrateMultiply = 100;          // Multiplier for feedrate in percent (factor 100 = 100%)
float Printer::menuExtrusionFactor = 1.0;     // Flow multiplier factor (1.0 = 100%)
float Printer::maxXYJerk_;                                         // Maximum allowed jerk in mm/s
float Printer::maxZJerk_;                                          // Maximum allowed jerk in z direction in mm/s
PrintUsage Printer::printUsage;
ZOffset Printer::zOffset;
SenseOffsetControl Printer::senseOffsetControl{SENSIBLE_PRESSURE_MAX_OFFSET};
ZMotion Printer::zMotion{ZAXIS_STEPS_PER_MM};
MachineInactivity Printer::inactivity(STEPPER_INACTIVE_TIME * 1000UL, MAX_INACTIVE_TIME * 1000UL);
IdleStatus Printer::idleStatus;
PrintLayers Printer::layers{ZAXIS_STEPS_PER_MM};
DigitCompensation Printer::digitCompensation{
    short(abs(EMERGENCY_PAUSE_DIGITS_MAX) * 0.7), short(abs(EMERGENCY_PAUSE_DIGITS_MAX))
};
PressureProtection Printer::pressureProtection;
StrainGauge Printer::strainGauge;
ExtrusionPlanner Printer::extrusionPlanner;
ExtrusionCompensation Printer::extrusionCompensation;
MoveTarget Printer::moveTarget;
WobbleCorrection Printer::wobbleCorrection;
CartesianPlanner Printer::cartesianPlanner;
XYMotion Printer::xyMotion;
char Printer::ZMode = DEFAULT_Z_SCALE_MODE;                       // Z-min distance or nominal layer height

volatile char Printer::doHeatBedZCompensation = false;

volatile char Printer::blockAll = 0;

DirectPosition Printer::extruderDirect;

char Printer::enabledStepper[3] = { 0, 0, 0 };



//When the printer resets, we should not do movement, because it would not be homed. At least some interaction with buttons or temperature-commands are needed to allow movement.
bool Printer::movementUnlocked_ = false;



void Printer::updateDerivedParameter() {
    // das extruder offset dürfen wir zur achslänge addieren, weil in dem Axis-Length von z.b. 180 mm schon angenommen wird, dass der extruder mit beiden nozzles jeden punkt erreicht.
    // Offset von links ist gesetzt, aber überfahren rechts muss aber noch möglich werden.
    // Darum 1x das Offset mehr zulassen und die Achsenlänge immer als reine Druckbreite definieren.
    maxSoftEndstopSteps_[X_AXIS] = (long)(axisStepsPerMM_[X_AXIS] * (axisLengthMM_[X_AXIS]
#if NUM_EXTRUDER > 1
                                                                   + getMaxExtruderOffsetMM(X_AXIS)
#endif
                                                                       ));
    maxSoftEndstopSteps_[Y_AXIS] = (long)(axisStepsPerMM_[Y_AXIS] * (axisLengthMM_[Y_AXIS]
#if NUM_EXTRUDER > 1
                                                                   + getMaxExtruderOffsetMM(Y_AXIS)
#endif
                                                                       ));
    zMotion.configureTravel(axisStepsPerMM_[Z_AXIS], axisLengthMM_[Z_AXIS]);

    for (uint8_t axis = 0; axis < 4; axis++) {
        const float mmPerStep = 1.0f / axisStepsPerMM_[axis];
        if (axis < E_AXIS && mmPerStep != axisMMPerSteps_[axis])
            cartesianPlanner.rescaleAxis(axis, axisMMPerSteps_[axis], axisStepsPerMM_[axis]);
        axisMMPerSteps_[axis] = mmPerStep;

        /** Acceleration in steps/s^2 in printing mode.*/
        maxPrintAccelerationStepsPerSquareSecond_[axis] = uint32_t(maxAccelerationMMPerSquareSecond_[axis] * axisStepsPerMM_[axis]);
        /** Acceleration in steps/s^2 in movement mode.*/
        maxTravelAccelerationStepsPerSquareSecond_[axis] = uint32_t(maxTravelAccelerationMMPerSquareSecond_[axis] * axisStepsPerMM_[axis]);
    }

    // 07 11 2017 https://github.com/repetier/Repetier-Firmware/commit/e76875ec2d04bd0dbfdd9a157270ee03f4731d5f#diff-dbe11559ff43e09563388a4968911e40L1973
    // For numeric stability we need to start accelerations at a minimum speed and hence ensure that the
    // jerk is at least 2 * minimum speed.

    // For xy moves the minimum speed is multiplied with 1.41 to enforce the condition also for diagonals since the
    // driving axis is the problematic speed.

    float accel;
    accel = RMath::max(maxAccelerationMMPerSquareSecond_[X_AXIS], maxTravelAccelerationMMPerSquareSecond_[X_AXIS]);

    float minimumSpeed = 1.41 * accel * sqrt(2.0f / (axisStepsPerMM_[X_AXIS] * accel));
    accel = RMath::max(maxAccelerationMMPerSquareSecond_[Y_AXIS], maxTravelAccelerationMMPerSquareSecond_[Y_AXIS]);
    float minimumSpeed2 = 1.41 * accel * sqrt(2.0f / (axisStepsPerMM_[Y_AXIS] * accel));
    if (minimumSpeed2 > minimumSpeed) {
        minimumSpeed = minimumSpeed2;
    }

    if (maxXYJerk_ < 2 * minimumSpeed) { // Enforce minimum start speed if target is faster and jerk too low
        maxXYJerk_ = 2 * minimumSpeed;
        Com::printFLN(PSTR("XY jerk was too low, setting to "), maxXYJerk_);
    }

    accel = RMath::max(maxAccelerationMMPerSquareSecond_[Z_AXIS], maxTravelAccelerationMMPerSquareSecond_[Z_AXIS]);
    float minimumZSpeed = 0.5 * accel * sqrt(2.0f / (axisStepsPerMM_[Z_AXIS] * accel));
    if (maxZJerk_ < 2 * minimumZSpeed) {
        maxZJerk_ = 2 * minimumZSpeed;
        Com::printFLN(PSTR("Z jerk was too low, setting to "), maxZJerk_);
    }
} // updateDerivedParameter

/** \brief Stop heater and stepper motors. Disable power,if possible. */
void Printer::switchEverythingOff() {
    if (Printer::isAllSwitchedOff())
        return;

    // disable the fan
    Commands::setFanSpeed((uint8_t)0);

    Printer::disableAllSteppersNow();

    Extruder::setTemperatureForAllExtruders(0);
    Extruder::setHeatedBedTemperature(0);
    UI_STATUS_UPD(UI_TEXT_SWITCHED_OFF);

    Printer::setAllSwitchedOff(true);
} // switchEverythingOff

void Printer::applyAdvanceSettings() {
    bool active = false;
    for (uint8_t i = 0; i < NUM_EXTRUDER; ++i) {
        if (extruder[i].advanceL != 0) {
            active = true;
            break;
        }
    }
    InterruptProtectedBlock lock;
    extrusionOutput.setAdvanceEnabled(active);
}

void Printer::updateWobbleFixOffsets() {
    // The factory settings bypass all floating-point work.
    if (!wobbleCorrection.enabled()) {
        wobbleCorrection.clear();
        return;
    }

    InterruptProtectedBlock noInts;
    const long motorX = xyMotion.motorSteps(X_AXIS);
    const long motorZ = zMotion.motorSteps();
    noInts.unprotect();

    wobbleCorrection.update(motorX, motorZ * axisMMPerSteps_[Z_AXIS], maxSoftEndstopSteps_[X_AXIS]);
}

int32_t Printer::prepareDestinationSteps(uint8_t axis) {
    const float direct = getDirectMM(axis);
    const float endstop = softwareEndstopSteps(axis) * axisMMPerSteps_[axis];
    const double limit = endstop + 0.1;
    float targetMM = moveTarget.machineMM(axis);
    if (targetMM + direct >= limit) {
        targetMM = limit - direct;
        moveTarget.setMachineMM(axis, targetMM);
    }

    if (axis < Z_AXIS)
        targetMM = wobbleCorrection.applyToTarget(axis, targetMM, direct, limit);

    return lroundf(targetMM * axisStepsPerMM_[axis]);
}

inline bool isExtrusionAllowed(float e) {
    // calculate total amount of extrusion respecting coordinate mode
    float de = e;
    if (!Printer::gcodeModes.relativeE()) {
        de = (e - Printer::moveTarget.machineMM(E_AXIS)) - Printer::moveTarget.remainderMM(E_AXIS);
    }

    if (fabs(de) * Printer::flowMultiplier() * Printer::digitCompensation.extrusionFactor() > EXTRUDE_MAXLENGTH) {
        return false;
    }

    if (Printer::heatingInhibited()) {
        return false;
    }

    return true;
}

/**
* Set the printers feedrate according to active unit settings
*/
void Printer::setFeedrate(float feedrate) {
    if (feedrate <= 0)
        return;

    if (Printer::gcodeModes.inches())
        Printer::feedrate = feedrate * (float)Printer::feedrateMultiply * 0.0042333f; // Factor is 25.4/60/100 = convertToMM(0.00016666666f)
    else
        Printer::feedrate = feedrate * (float)Printer::feedrateMultiply * 0.00016666666f;
}

// Convert G-code positions to millimetres in the planner's coordinate frame.
bool Printer::queueGCodeCoordinates(GCode* com) {
    const bool isXYZMove = !com->hasNoXYZ();
    if (com->hasX())
        moveTarget.setFromGCodeMM(X_AXIS, convertToMM(com->X), gcodeModes.relativeXYZ());
    if (com->hasY())
        moveTarget.setFromGCodeMM(Y_AXIS, convertToMM(com->Y), gcodeModes.relativeXYZ());
    if (com->hasZ())
        moveTarget.setFromGCodeMM(Z_AXIS, convertToMM(com->Z), gcodeModes.relativeXYZ());

    float e = convertToMM(com->E);
    bool isEMove = com->hasE() && isExtrusionAllowed(e);
    if (isEMove) {
        if (gcodeModes.relativeE())
            moveTarget.addMM(E_AXIS, e);
        else
            moveTarget.setMachineMM(E_AXIS, e);
    }

    if (com->hasF()) {
        Printer::setFeedrate(com->F);
    }

    if (!isXYZMove && !isEMove)
        return false;
    const bool published = PrintLine::prepareQueueMove(false, true, Printer::feedrate, isXYZMove);
    if (published && !com->internalCommand)
        printJob.admitPrintMotion();
    return published;
} // queueGCodeCoordinates

/** Move to machine coordinates, retaining the correction on omitted axes. */
void Printer::queueFloatCoordinates(float x, float y, float z, float e, float feedrate) {
    if (x != IGNORE_COORDINATE) {
        moveTarget.setMachineMM(X_AXIS, x);
        wobbleCorrection.clearAxis(X_AXIS);
    }
    if (y != IGNORE_COORDINATE) {
        moveTarget.setMachineMM(Y_AXIS, y);
        wobbleCorrection.clearAxis(Y_AXIS);
    }
    if (z != IGNORE_COORDINATE)
        moveTarget.setMachineMM(Z_AXIS, z);
    if (e != IGNORE_COORDINATE)
        moveTarget.setMachineMM(E_AXIS, e);
    if (feedrate == IGNORE_COORDINATE)
        feedrate = Printer::feedrate;

    PrintLine::prepareQueueMove(false, true, feedrate);

    Printer::inactivity.recordActivity(HAL::timeInMilliseconds());
} // queueFloatCoordinates

/** \brief Move printer the given number of steps. Puts the move into the queue. Used by e.g. homing commands. */
bool Printer::queueRelativeStepsCoordinates(long dx, long dy, long dz, long de, float feedrate, bool waitEnd, bool abortAtEndstops) {
    return queueRelativeMMCoordinates(dx * axisMMPerSteps_[X_AXIS], dy * axisMMPerSteps_[Y_AXIS],
                                      dz * axisMMPerSteps_[Z_AXIS], de * axisMMPerSteps_[E_AXIS],
                                      feedrate, waitEnd, abortAtEndstops);
} // queueRelativeStepsCoordinates

/** \brief Move printer the given number of mm. Puts the move into the queue. */
bool Printer::queueRelativeMMCoordinates(float dx, float dy, float dz, float de, float feedrate, bool waitEnd, bool abortAtEndstops) {
    if (Printer::blockAll || Printer::printJob.commandsBlocked())
        return false;

    moveTarget.addMM(X_AXIS, dx);
    moveTarget.addMM(Y_AXIS, dy);
    moveTarget.addMM(Z_AXIS, dz);
    moveTarget.addMM(E_AXIS, de);

    PrintLine::prepareQueueMove(abortAtEndstops, !waitEnd, feedrate);

    if (waitEnd && !Commands::waitForMotion(Commands::MotionWait::QueueAndZ, Commands::PauseWait::Resume))
        return false;

    Printer::inactivity.recordActivity(HAL::timeInMilliseconds());
    return true;
} // queueRelativeMMCoordinates

/**
 * This function transfers the axis without affecting the axis values.
 * This is expecially usefull to transfer the extruder 0 to 1 and backwards.
 */
void Printer::offsetRelativeStepsCoordinates(int32_t dx, int32_t dy, int32_t dz, int32_t de, DirectMoveMode configuration) {
    if (dx == 0 && dy == 0 && dz == 0 && de == 0)
        return;
    if (configuration != DirectMoveMode::Configured && !pause.allowsManualMove())
        return;

    while (PrintLine::directMoveActive()) {
        Commands::checkForPeriodicalActions(Processing);
    }

    const bool stoppable = configuration == DirectMoveMode::Button;
    const bool feedrateSource = (stoppable || configuration == DirectMoveMode::ManualPosition)
        ? manualMotion.feedrateSource() : FEEDRATE_DIRECTCONFIG;
    const int32_t steps[4] = {dx, dy, dz, de};
    {
        InterruptProtectedBlock lock;
        xyMotion.requestDirect(X_AXIS, dx);
        xyMotion.requestDirect(Y_AXIS, dy);
        zMotion.requestDirectMove(dz);
        extruderDirect.request(de);
        if (stoppable)
            uid.rememberJogButton();
        PrintLine::reserveDirectMove(stoppable);
    }

    PrintLine::prepareDirectMove(steps, stoppable, feedrateSource);
    if (stoppable)
        return;

    while (PrintLine::directMoveActive()) {
        Commands::checkForPeriodicalActions(Processing);
    }
}

void Printer::setOrigin(float xOff, float yOff, float zOff) {
    Com::printF(PSTR("setOrigin():"));
    bool someFailed = false;

    if (isAxisHomed(X_AXIS) || xOff == 0.0f) {
        moveTarget.setWorkOffsetMM(X_AXIS, xOff);
        Com::printF(PSTR("x="), xOff);
    } else {
        Com::printF(PSTR("x-fail"));
        someFailed = true;
    }
    if (isAxisHomed(Y_AXIS) || yOff == 0.0f) {
        moveTarget.setWorkOffsetMM(Y_AXIS, yOff);
        Com::printF(PSTR(" y="), yOff);
    } else {
        Com::printF(PSTR(" y-fail"));
        someFailed = true;
    }
    if (isAxisHomed(Z_AXIS) || zOff == 0.0f) {
        moveTarget.setWorkOffsetMM(Z_AXIS, zOff);
        Com::printF(PSTR(" z="), zOff);
    } else {
        Com::printF(PSTR(" z-fail"));
        someFailed = true;
    }
    Com::println();

    if (someFailed) {
        // we can not set the origin when we do not know the home position
        Com::printFLN(PSTR("WARNING: Home positions were unknown/Value not 0 -> ignored!"));
        showError((void*)ui_text_set_origin, (void*)ui_text_home_unknown);
    }
} // setOrigin

void Printer::setup() {
    HAL::stopWatchdog();

    HAL::resetPwmOutputs();

    HAL::allowInterrupts();

    TwiTransfer::beginBus();

    if (!hardwareTypeMatches()) {
        EEPROM::initBaudrate();
        HAL::serialSetBaudrate(EEPROM::configuredBaudrate());
        Com::printErrorFLN(PSTR(UI_TEXT_WRONG_FIRMWARE));

        // Halt before initializing hardware outputs for a different board.
        while (1) {
        }
    }

    //Initialize Step Pins
    SET_OUTPUT(X_STEP_PIN);
    SET_OUTPUT(Y_STEP_PIN);
    SET_OUTPUT(Z_STEP_PIN);

    //Initialize Dir Pins
#if X_DIR_PIN > -1
    SET_OUTPUT(X_DIR_PIN);
#endif // X_DIR_PIN>-1

#if Y_DIR_PIN > -1
    SET_OUTPUT(Y_DIR_PIN);
#endif // Y_DIR_PIN>-1

#if Z_DIR_PIN > -1
    SET_OUTPUT(Z_DIR_PIN);
#endif // Z_DIR_PIN>-1

    //Steppers default to disabled.
#if X_ENABLE_PIN > -1
    SET_OUTPUT(X_ENABLE_PIN);
    if (!X_ENABLE_ON)
        WRITE(X_ENABLE_PIN, HIGH);
    disableXStepper();
#endif // X_ENABLE_PIN > -1

#if Y_ENABLE_PIN > -1
    SET_OUTPUT(Y_ENABLE_PIN);
    if (!Y_ENABLE_ON)
        WRITE(Y_ENABLE_PIN, HIGH);
    disableYStepper();
#endif // Y_ENABLE_PIN > -1

#if Z_ENABLE_PIN > -1
    SET_OUTPUT(Z_ENABLE_PIN);
    if (!Z_ENABLE_ON)
        WRITE(Z_ENABLE_PIN, HIGH);
    disableZStepper();
#endif // Z_ENABLE_PIN > -1

    // Endstop inputs and pull-ups.
#if X_MIN_PIN > -1
    SET_INPUT(X_MIN_PIN);

    PULLUP(X_MIN_PIN, HIGH);
#else
#error You have defined hardware x min endstop without pin assignment. Set pin number for X_MIN_PIN
#endif // X_MIN_PIN>-1

#if Y_MIN_PIN > -1
    SET_INPUT(Y_MIN_PIN);

    PULLUP(Y_MIN_PIN, HIGH);
#else
#error You have defined hardware y min endstop without pin assignment. Set pin number for Y_MIN_PIN
#endif // Y_MIN_PIN>-1

#if Z_MIN_PIN > -1
    SET_INPUT(Z_MIN_PIN);

    PULLUP(Z_MIN_PIN, HIGH);
#else
#error You have defined hardware z min endstop without pin assignment. Set pin number for Z_MIN_PIN
#endif // Z_MIN_PIN>-1

#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
#if Z_MAX_PIN > -1
    SET_INPUT(Z_MAX_PIN);

    PULLUP(Z_MAX_PIN, HIGH);
#else
#error You have defined hardware z max endstop without pin assignment. Set pin number for Z_MAX_PIN
#endif // Z_MAX_PIN>-1
#endif // RF2000 / RF2000v2

    SET_OUTPUT(FAN_PIN);
    WRITE(FAN_PIN, LOW);

#if FAN_BOARD_PIN > -1
    SET_OUTPUT(FAN_BOARD_PIN);
    WRITE(FAN_BOARD_PIN, LOW);
#endif // FAN_BOARD_PIN>-1

#if EXT0_HEATER_PIN > -1
    SET_OUTPUT(EXT0_HEATER_PIN);
    WRITE(EXT0_HEATER_PIN, LOW);
#endif // EXT0_HEATER_PIN>-1

#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN > -1 && NUM_EXTRUDER > 1
    SET_OUTPUT(EXT1_HEATER_PIN);
    WRITE(EXT1_HEATER_PIN, LOW);
#endif // defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1 && NUM_EXTRUDER>1

#if EXT0_EXTRUDER_COOLER_PIN > -1
    SET_OUTPUT(EXT0_EXTRUDER_COOLER_PIN);
    WRITE(EXT0_EXTRUDER_COOLER_PIN, LOW);
#endif // EXT0_EXTRUDER_COOLER_PIN >- 1

#if defined(EXT1_EXTRUDER_COOLER_PIN) && EXT1_EXTRUDER_COOLER_PIN > -1 && NUM_EXTRUDER > 1
    SET_OUTPUT(EXT1_EXTRUDER_COOLER_PIN);
    WRITE(EXT1_EXTRUDER_COOLER_PIN, LOW);
#endif // defined(EXT1_EXTRUDER_COOLER_PIN) && EXT1_EXTRUDER_COOLER_PIN >- 1 && NUM_EXTRUDER > 1

    extrusionOutput.clearSteps();

    maxXYJerk_ = MAX_JERK;
    maxZJerk_ = MAX_ZJERK;

    flag0 = PRINTER_FLAG0_STEPPER_DISABLED;

    Printer::manualMotion.setMode(X_AXIS, DEFAULT_MOVE_MODE_X);
    Printer::manualMotion.setMode(Y_AXIS, DEFAULT_MOVE_MODE_Y);
    Printer::manualMotion.setMode(Z_AXIS, DEFAULT_MOVE_MODE_Z);

    doHeatBedZCompensation = 0; //init

    blockAll = 0;


    Printer::setAxisHomingFeedrate(X_AXIS, HOMING_FEEDRATE_X_PRINT);
    Printer::setAxisHomingFeedrate(Y_AXIS, HOMING_FEEDRATE_Y_PRINT);
    Printer::setAxisHomingFeedrate(Z_AXIS, HOMING_FEEDRATE_Z_PRINT);
    Printer::setAxisLengthMm(X_AXIS, X_MAX_LENGTH_PRINT);

    Printer::setAxisLengthMm(Y_AXIS, Y_MAX_LENGTH);
    Printer::setAxisLengthMm(Z_AXIS, Z_MAX_LENGTH);

#ifdef RGB_LIGHT_R_PIN
    RgbLighting::reset();
#endif // RGB_LIGHT_R_PIN

    uid.initialize();

    EEPROM::initBaudrate();
    HAL::serialSetBaudrate(EEPROM::configuredBaudrate());

    // sending of this information tells the Repetier-Host that the firmware has restarted - never delete or change this to-be-sent information
    Com::println();
    Com::printFLN(Com::tStart); //http://forum.repetier.com/discussion/comment/16949/#Comment_16949

    HAL::showStartReason();
    Extruder::initExtruder();

    SET_OUTPUT(HEATED_BED_HEATER_PIN);
    WRITE(HEATED_BED_HEATER_PIN, LOW);
    Extruder::initHeatedBed();

    HAL::analogStart();

    // configure all DRV8711
    drv8711Init();

    // Read settings from eeprom if wanted [readDataFromEEPROM or destroy corrupted eeprom]
    EEPROM::init();

    AccessoryOutputs::initializeBoardOutputs();
#if (MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2) && CASE_FAN_PIN > -1
    CaseFan::initialize();
#endif
    AccessoryOutputs::initializeCaseLight();

    updateDerivedParameter();
    Commands::checkFreeMemory();
    Commands::writeLowestFreeRAM();
    HAL::setupTimer();

    Extruder::applyCurrentSettings();

#ifdef RGB_LIGHT_R_PIN
    RgbLighting::initialize();
#endif // RGB_LIGHT_R_PIN

    lockMovement();

    HAL::startWatchdog();
} // setup()

void Printer::enableSenseOffsetnow(void) {
    const int16_t storedDigits = HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS);
    const int16_t storedLimit = HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX);
    senseOffsetControl.activateStoredLimits(storedDigits, storedLimit);
}

void Printer::enableCMPnow(void) {
    if (doHeatBedZCompensation || !areAxisHomed() || !bedMatrixAvailable() || isAnyScanRunning()) {
        return;
    }
    doHeatBedZCompensation = 1;
}

bool Printer::needsCMPwait(void) {
    InterruptProtectedBlock noInts;
    return !blockAll && zMotion.correctionFollowsDirection();
}

void Printer::disableCMPnow(bool wait) {
    doHeatBedZCompensation = 0;
    {
        InterruptProtectedBlock noInts;
        zMotion.requestCorrection(0);
    }

    while (wait) {
        {
            InterruptProtectedBlock noInts;
            if (blockAll || !zMotion.correctionCanMove()) {
                break;
            }
        }
        HAL::delayMilliseconds(1);
        //checkforperiodical .. possible loop inside scans and homing.
    }
}

void Printer::stopPrint() {
    InterruptProtectedBlock noInts;
    if (!printJob.beginStop(HAL::timeInMilliseconds()))
        return;
    Printer::idleStatus.cancel();
    extrusionOutput.inhibit(ExtrusionOutput::Inhibit::PrintStop);
    if (pause.isPaused() && PrintLine::directMoveActive())
        PrintLine::stopDirectMove();
    pause.cancel();

    //erase the coordinates and kill the current taskplaner:
    PrintLine::discardQueuedMoves();

    setXAxisSteps(xyMotion.nominalSteps(X_AXIS));
    setYAxisSteps(xyMotion.nominalSteps(Y_AXIS));
    setZAxisSteps(zMotion.nominalSteps());
    Printer::setEPositionMM(0); // G92 E0

    noInts.unprotect();

    Com::printFLN(PSTR("RequestStop:"));     // Repetier-Server
    Com::printFLN(PSTR("// action:cancel")); // OctoPrint
    Com::printFLN(PSTR("USB print stopped."));

    Commands::printCurrentPosition();
    UI_STATUS_UPD(UI_TEXT_STOP_PRINT);
} // stopPrint

#define START_EXTRUDER_CONFIG(i) \
    Com::printF(Com::tConfig); \
    Com::printF(Com::tExtrDot, i + 1); \
    Com::print(':');
void Printer::showConfiguration() {
    Com::config(PSTR("Baudrate:"), EEPROM::configuredBaudrate());
    Com::config(PSTR("InputBuffer:"), UsbSerial::receiveCapacity);
    Com::config(PSTR("NumExtruder:"), NUM_EXTRUDER);
    Com::config(PSTR("MixingExtruder:"), 0);
    Com::config(PSTR("HeatedBed:"), 1);
    Com::config(PSTR("Fan:"), true);
    Com::config(PSTR("Fan2:0"));
    Com::config(PSTR("LCD:"), 0);
    Com::config(PSTR("SoftwarePowerSwitch:"), 0);
    Com::config(PSTR("XHomeDir:"), -1);
    Com::config(PSTR("YHomeDir:"), -1);
    Com::config(PSTR("ZHomeDir:"), -1);

    Com::config(PSTR("XHomePos:"), 0.0f, 2);
    Com::config(PSTR("YHomePos:"), 0.0f, 2);
    Com::config(PSTR("ZHomePos:"), 0.0f, 2);

    Com::config(PSTR("SupportG10G11:"), 0);
    Com::config(PSTR("SupportLocalFilamentchange:"), 0);
    Com::config(PSTR("CaseLights:"), 0);
    Com::config(PSTR("EEPROM:"), 1);
    Com::config(PSTR("PrintlineCache:"), MOVE_CACHE_SIZE);
    Com::config(PSTR("JerkXY:"), Printer::xyJerkLimit());
    Com::config(PSTR("JerkZ:"), Printer::zJerkLimit());
    Com::config(PSTR("KeepAliveInterval:"), GCodeInput::keepAliveIntervalMs);

    Com::config(PSTR("XMin:"), 0);
    Com::config(PSTR("YMin:"), 0);
    Com::config(PSTR("ZMin:"), 0);
    Com::config(PSTR("XMax:"), Printer::axisLengthMm(X_AXIS));
    Com::config(PSTR("YMax:"), Printer::axisLengthMm(Y_AXIS));
    Com::config(PSTR("ZMax:"), Printer::axisLengthMm(Z_AXIS));
    Com::config(PSTR("XSize:"), Printer::axisLengthMm(X_AXIS));
    Com::config(PSTR("YSize:"), Printer::axisLengthMm(Y_AXIS));
    Com::config(PSTR("ZSize:"), Printer::axisLengthMm(Z_AXIS));
    Com::config(PSTR("XPrintAccel:"), Printer::printAcceleration(X_AXIS));
    Com::config(PSTR("YPrintAccel:"), Printer::printAcceleration(Y_AXIS));
    Com::config(PSTR("ZPrintAccel:"), Printer::printAcceleration(Z_AXIS));
    Com::config(PSTR("XTravelAccel:"), Printer::travelAcceleration(X_AXIS));
    Com::config(PSTR("YTravelAccel:"), Printer::travelAcceleration(Y_AXIS));
    Com::config(PSTR("ZTravelAccel:"), Printer::travelAcceleration(Z_AXIS));
    Com::config(PSTR("PrinterType:Cartesian"));

    Com::config(PSTR("MaxBedTemp:"), HEATED_BED_MAX_TEMP);
    for (fast8_t i = 0; i < NUM_EXTRUDER; i++) {
        START_EXTRUDER_CONFIG(i)
        Com::printFLN(PSTR("Jerk:"), extruder[i].maxEJerk);
        START_EXTRUDER_CONFIG(i)
        Com::printFLN(PSTR("MaxSpeed:"), extruder[i].maxFeedrate);
        START_EXTRUDER_CONFIG(i)
        Com::printFLN(PSTR("Acceleration:"), extruder[i].maxAcceleration);
        //START_EXTRUDER_CONFIG(i)
        //	Com::printFLN(PSTR("Diameter:"), extruder[i].diameter);
        START_EXTRUDER_CONFIG(i)
        Com::printFLN(PSTR("MaxTemp:"), static_cast<uint32_t>(extruder[i].tempControl.heaterCutoffC));
    }
}

bool Printer::checkAbortKeys(void) {
    int16_t activeKeys = 0;
    uid.ui_check_keys(activeKeys);
    if (activeKeys == UI_ACTION_OK || activeKeys == UI_ACTION_BACK) {
        return true;
    }
    return false;
}

bool Printer::checkPlayKey(void) {
    if (pause.allowsQueueAdvance()) {
        int16_t activeKeys = 0;
        uid.ui_check_keys(activeKeys);
        if (activeKeys == UI_ACTION_RF_CONTINUE) {
            return true;
        }
    }
    return false;
}

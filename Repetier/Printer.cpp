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

#include "Repetier.h"
#include <Wire.h>

#if USE_ADVANCE
uint8_t Printer::maxExtruderSpeed;         // Timer delay for end extruder speed
volatile int Printer::extruderStepsNeeded; // This many extruder steps are still needed, <0 = reverse steps needed.
volatile int Printer::advanceStepsSet;
#endif // USE_ADVANCE

uint8_t Printer::unitIsInches = 0; // 0 = Units are mm, 1 = units are inches.

//Stepper Movement Variables
float Printer::axisStepsPerMM[4] = { XAXIS_STEPS_PER_MM, YAXIS_STEPS_PER_MM, ZAXIS_STEPS_PER_MM, 1 };                      ///< Number of steps per mm needed.
float Printer::axisMMPerSteps[4] = { 1.0f / XAXIS_STEPS_PER_MM, 1.0f / YAXIS_STEPS_PER_MM, 1.0f / ZAXIS_STEPS_PER_MM, 1 }; ///< Inverse of axisStepsPerMM for faster conversion
float Printer::maxFeedrate[4] = { MAX_FEEDRATE_X, MAX_FEEDRATE_Y, MAX_FEEDRATE_Z, STANDARD_POSITION_FEEDRATE_E };          ///< Maximum allowed feedrate. //STANDARD_POSITION_FEEDRATE_E added by nibbels, wird aber überschrieben.
float Printer::homingFeedrate[3] = { HOMING_FEEDRATE_X_PRINT, HOMING_FEEDRATE_Y_PRINT, HOMING_FEEDRATE_Z_PRINT };          ///< dass zumindest etwas sinnvolles drin steht, wird überschrieben.

float Printer::maxAccelerationMMPerSquareSecond[4] = { MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z };                            ///< X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts
float Printer::maxTravelAccelerationMMPerSquareSecond[4] = { MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X, MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y, MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z }; ///< X, Y, Z max acceleration in mm/s^2 for travel moves
#if FEATURE_MILLING_MODE
short Printer::max_milling_all_axis_acceleration = MILLER_ACCELERATION; //miller min speed is limited to too high speed because of acceleration-formula. We use this value in milling mode and make it adjustable within small numbers like 5 to 100 or something like that.
#endif                                                                  // FEATURE_MILLING_MODE

/** Acceleration in steps/s^2 in printing mode.*/
uint32_t Printer::maxPrintAccelerationStepsPerSquareSecond[4];
/** Acceleration in steps/s^2 in movement mode.*/
uint32_t Printer::maxTravelAccelerationStepsPerSquareSecond[4];

uint8_t Printer::relativeCoordinateMode = false;         // Determines absolute (false) or relative Coordinates (true).
uint8_t Printer::relativeExtruderCoordinateMode = false; // Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.

volatile float Printer::destinationMM[4] = { 0, 0, 0, 0 }; // planned precise mm.
float Printer::destinationMMLast[4] = { 0, 0, 0, 0 };      // rounded-queued mm.
float Printer::originOffsetMM[3] = { 0, 0, 0 };
uint8_t Printer::flag0 = 0;
uint8_t Printer::flag1 = 0;
uint8_t Printer::flag2 = 0;
uint8_t Printer::flag3 = 0;

#if ALLOW_EXTENDED_COMMUNICATION < 2
uint8_t Printer::debugLevel = 0; ///< Bitfield defining debug output. 1 = echo, 2 = info, 4 = error, 8 = dry run., 16 = Only communication, 32 = No moves
#else
uint8_t Printer::debugLevel = 6; ///< Bitfield defining debug output. 1 = echo, 2 = info, 4 = error, 8 = dry run., 16 = Only communication, 32 = No moves
#endif // ALLOW_EXTENDED_COMMUNICATION < 2

uint8_t Printer::stepsPerTimerCall = 1;
uint16_t Printer::stepsPackingMinInterval = STEP_PACKING_MIN_INTERVAL;
uint8_t Printer::menuMode = 0;

volatile unsigned long Printer::interval = 10000; // Last step duration in ticks.
volatile float Printer::v = 0;                    // Last planned printer speed.
unsigned long Printer::timer[2] = { 0 };          // Used for acceleration/deceleration timing
unsigned long Printer::stepNumber[2] = { 0 };     // Step number in current move.
#if FEATURE_DIGIT_FLOW_COMPENSATION
unsigned short Printer::interval_mod = 0; // additional step duration in ticks to slow the printer down live
#endif                                    // FEATURE_DIGIT_FLOW_COMPENSATION
int8_t Printer::lastDirectionSovereignty = 0;

long Printer::maxSoftEndstopSteps[3] = { 0 }; // For software endstops, limit of move in positive direction. (=Homing-Offset + Achsenlänge)
float Printer::axisLengthMM[3] = { 0 };       // Länge des überfahrbaren Bereichs im positiven Homing. (=Schienen-Fahrweg - Homing-Offset - 2x ExtruderOffset)
float Printer::feedrate = 10;                 // Last requested feedrate.
int Printer::feedrateMultiply = 100;          // Multiplier for feedrate in percent (factor 100 = 100%)
float Printer::dynamicFeedrateFactor = 1.0;   // Feedrate multiplier factor for digit compensation (1.0 = 100%)
float Printer::menuExtrusionFactor = 1.0;     // Flow multiplier factor (1.0 = 100%)
float Printer::dynamicExtrusionFactor = 1.0;  // Flow multiplier factor for digit compensation (1.0 = 100%)
float Printer::extrudeMultiplyErrorSteps = 0;
float Printer::maxXYJerk;                                         // Maximum allowed jerk in mm/s
float Printer::maxZJerk;                                          // Maximum allowed jerk in z direction in mm/s
unsigned int Printer::vMaxReached[2];                             // Maximum reached speed [FOR_DIRECT, FOR_QUEUE]
unsigned long Printer::msecondsPrinting = 0;                      // Milliseconds of printing time (means time with heated extruder)
unsigned long Printer::msecondsMilling = 0;                       // Milliseconds of milling time
float Printer::filamentPrinted = 0.0f;                            // mm of filament printed since counting started
long Printer::ZOffset = 0;                                        // Z Offset in um
char Printer::ZMode = DEFAULT_Z_SCALE_MODE;                       // Z Scale  1 = show the z-distance to z-min (print) or to the z-origin (mill), 2 = show the z-distance to the surface of the heat bed (print) or work part (mill)
char Printer::moveMode[3];                                        // move mode which is applied within the Position X/Y/Z menus
bool Printer::moveKosys = KOSYS_GCODE;                            // true = GCode, false = DirectMove / OffsetMove
bool Printer::movePositionFeedrateChoice = FEEDRATE_DIRECTCONFIG; // select the feedrate for menu positioning: feedrate from last gcode or standard speed

// This is some buffer but only for a limited amount of overdrive.
volatile int16_t Printer::outOfPrintVolume[2] = { 0 };
volatile int32_t Printer::outOfPrintVolumeZ = 0;

#if FEATURE_MEMORY_POSITION
float Printer::memoryX;
float Printer::memoryY;
float Printer::memoryZ;
float Printer::memoryE;
float Printer::memoryF;
#endif // FEATURE_MEMORY_POSITION

#if FEATURE_HEAT_BED_Z_COMPENSATION
volatile char Printer::doHeatBedZCompensation = false;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_WORK_PART_Z_COMPENSATION
volatile char Printer::doWorkPartZCompensation = false;
volatile long Printer::staticCompensationZ = 0;
#endif // FEATURE_WORK_PART_Z_COMPENSATION

volatile long Printer::currentSteps[3] = { 0, 0, 0 };
volatile char Printer::blockAll = 0;

volatile long Printer::currentXSteps = 0;                                                  //das ist der X-Zähler der GCodes zum Zählen des tiefsten Schalterdruckpunkts /Schaltercrash.
volatile long Printer::currentYSteps = 0;                                                  //das ist der Y-Zähler der GCodes zum Zählen des tiefsten Schalterdruckpunkts /Schaltercrash.
volatile long Printer::currentZSteps = 0;                                                  //das ist der Z-Zähler der GCodes zum Zählen des tiefsten Schalterdruckpunkts /Schaltercrash.
uint16_t Printer::maxZOverrideSteps = uint16_t(ZAXIS_STEPS_PER_MM * Z_ENDSTOP_DRIVE_OVER); //das ist der Z-Zähler der GCodes zum Zählen des tiefsten Schalterdruckpunkts /Schaltercrash.

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
volatile long Printer::compensatedPositionTargetStepsZ = 0;
volatile long Printer::compensatedPositionCurrentStepsZ = 0;

volatile float Printer::compensatedPositionOverPercE = 0.0f;
volatile float Printer::compensatedPositionCollectTinyE = 0.0f;

long Printer::queuePositionZLayerGuessNew = 0;
volatile long Printer::queuePositionZLayerCurrent = 0;
volatile long Printer::queuePositionZLayerLast = 0;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

volatile long Printer::directDestinationSteps[4] = { 0, 0, 0, 0 };
volatile long Printer::directCurrentSteps[4] = { 0, 0, 0, 0 };

#if FEATURE_MILLING_MODE
char Printer::operatingMode;
float Printer::drillFeedrate;
float Printer::drillZDepth;
#endif // FEATURE_MILLING_MODE

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
char Printer::ZEndstopType;
char Printer::ZEndstopUnknown;
char Printer::lastZDirection;
char Printer::endstopZMinHit;
char Printer::endstopZMaxHit;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

#if FEATURE_CONFIGURABLE_MILLER_TYPE
char Printer::MillerType;
#endif // FEATURE_CONFIGURABLE_MILLER_TYPE

#if STEPPER_ON_DELAY
char Printer::enabledStepper[3] = { 0, 0, 0 };
#endif // STEPPER_ON_DELAY

#if FEATURE_BEEPER
char Printer::enableBeeper;
#endif // FEATURE_BEEPER

#if FEATURE_CASE_LIGHT
char Printer::enableCaseLight = CASE_LIGHTS_DEFAULT_ON;
#endif // FEATURE_CASE_LIGHT

#if FEATURE_RGB_LIGHT_EFFECTS
char Printer::RGBLightMode;
char Printer::RGBLightStatus;
unsigned long Printer::RGBLightIdleStart;
char Printer::RGBButtonBackPressed;
char Printer::RGBLightModeForceWhite;
#endif // FEATURE_RGB_LIGHT_EFFECTS

#if FEATURE_230V_OUTPUT
char Printer::enable230VOutput;
#endif // FEATURE_230V_OUTPUT

#if FEATURE_24V_FET_OUTPUTS
char Printer::enableFET1 = FET1_DEFAULT_ON;
char Printer::enableFET2 = FET2_DEFAULT_ON;
char Printer::enableFET3 = FET3_DEFAULT_ON;
#endif // FEATURE_24V_FET_OUTPUTS

#if FEATURE_CASE_FAN
bool Printer::ignoreFanOn = false;
millis_t Printer::prepareFanOff = 0;
unsigned long Printer::fanOffDelay = 0;
#endif // FEATURE_CASE_FAN

#if FEATURE_TYPE_EEPROM
unsigned char Printer::wrongType;
#endif // FEATURE_TYPE_EEPROM

#if FEATURE_UNLOCK_MOVEMENT
//When the printer resets, we should not do movement, because it would not be homed. At least some interaction with buttons or temperature-commands are needed to allow movement.
unsigned char Printer::g_unlock_movement = 0;
#endif //FEATURE_UNLOCK_MOVEMENT

#if FEATURE_SENSIBLE_PRESSURE
bool Printer::g_senseoffset_autostart = false;
#endif //FEATURE_SENSIBLE_PRESSURE

uint8_t Printer::motorCurrent[DRV8711_NUM_CHANNELS] = { 0 };

#if FEATURE_ZERO_DIGITS
bool Printer::g_pressure_offset_active = true;
short Printer::g_pressure_offset = 0;
#endif // FEATURE_ZERO_DIGITS

#if FEATURE_ADJUSTABLE_MICROSTEPS
//RF_MICRO_STEPS_ have values 0=FULL 1=2MS, 2=4MS, 3=8MS, 4=16MS, 5=32MS, 6=64MS, 7=128MS, 8=256MS
uint8_t Printer::motorMicroStepsModeValue[DRV8711_NUM_CHANNELS] = { 0 }; //init later because of recalculation of value
#endif                                                                   // FEATURE_ADJUSTABLE_MICROSTEPS

#if FEATURE_Kurt67_WOBBLE_FIX
int8_t Printer::wobblePhaseXY = 0;             //+100 = +PI | -100 = -PI
int16_t Printer::wobbleAmplitudes[3] = { 0 };  //X, Y(X_0), Y(X_max), /*Z*/
float Printer::lastWobbleFixOffset[2] = { 0 }; ///< last calculated target wobbleFixOffsets for relative coordinates and display output.
#endif                                         // FEATURE_Kurt67_WOBBLE_FIX

void Printer::updateDerivedParameter() {
    // das extruder offset dürfen wir zur achslänge addieren, weil in dem Axis-Length von z.b. 180 mm schon angenommen wird, dass der extruder mit beiden nozzles jeden punkt erreicht.
    // Offset von links ist gesetzt, aber überfahren rechts muss aber noch möglich werden.
    // Darum 1x das Offset mehr zulassen und die Achsenlänge immer als reine Druckbreite definieren.
    maxSoftEndstopSteps[X_AXIS] = (long)(axisStepsPerMM[X_AXIS] * (axisLengthMM[X_AXIS]
#if NUM_EXTRUDER > 1
                                                                   + getMaxExtruderOffsetMM(X_AXIS)
#endif
                                                                       ));
    maxSoftEndstopSteps[Y_AXIS] = (long)(axisStepsPerMM[Y_AXIS] * (axisLengthMM[Y_AXIS]
#if NUM_EXTRUDER > 1
                                                                   + getMaxExtruderOffsetMM(Y_AXIS)
#endif
                                                                       ));
    // Sonderbehandlung für Z.
    maxSoftEndstopSteps[Z_AXIS] = (long)(axisStepsPerMM[Z_AXIS] * axisLengthMM[Z_AXIS]);

    for (uint8_t axis = 0; axis < 4; axis++) {
        axisMMPerSteps[axis] = 1.0f / axisStepsPerMM[axis];

#if FEATURE_MILLING_MODE
        if (Printer::operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE
            /** Acceleration in steps/s^2 in printing mode.*/
            maxPrintAccelerationStepsPerSquareSecond[axis] = uint32_t(maxAccelerationMMPerSquareSecond[axis] * axisStepsPerMM[axis]);
            /** Acceleration in steps/s^2 in movement mode.*/
            maxTravelAccelerationStepsPerSquareSecond[axis] = uint32_t(maxTravelAccelerationMMPerSquareSecond[axis] * axisStepsPerMM[axis]);
#if FEATURE_MILLING_MODE
        } else {
            /** Acceleration in steps/s^2 in milling mode.*/
            maxPrintAccelerationStepsPerSquareSecond[axis] = uint32_t(MILLER_ACCELERATION * axisStepsPerMM[axis]);
            /** Acceleration in steps/s^2 in milling-movement mode.*/
            maxTravelAccelerationStepsPerSquareSecond[axis] = uint32_t(MILLER_ACCELERATION * axisStepsPerMM[axis]);
        }
#endif // FEATURE_MILLING_MODE
    }

    // 07 11 2017 https://github.com/repetier/Repetier-Firmware/commit/e76875ec2d04bd0dbfdd9a157270ee03f4731d5f#diff-dbe11559ff43e09563388a4968911e40L1973
    // For numeric stability we need to start accelerations at a minimum speed and hence ensure that the
    // jerk is at least 2 * minimum speed.

    // For xy moves the minimum speed is multiplied with 1.41 to enforce the condition also for diagonals since the
    // driving axis is the problematic speed.

    float accel;
#if FEATURE_MILLING_MODE
    if (Printer::operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE
        accel = RMath::max(maxAccelerationMMPerSquareSecond[X_AXIS], maxTravelAccelerationMMPerSquareSecond[X_AXIS]);
#if FEATURE_MILLING_MODE
    } else {
        accel = MILLER_ACCELERATION;
    }
#endif // FEATURE_MILLING_MODE

    float minimumSpeed = 1.41 * accel * sqrt(2.0f / (axisStepsPerMM[X_AXIS] * accel));
#if FEATURE_MILLING_MODE
    if (Printer::operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE
        accel = RMath::max(maxAccelerationMMPerSquareSecond[Y_AXIS], maxTravelAccelerationMMPerSquareSecond[Y_AXIS]);
#if FEATURE_MILLING_MODE
    } else {
        accel = MILLER_ACCELERATION;
    }
#endif // FEATURE_MILLING_MODE
    float minimumSpeed2 = 1.41 * accel * sqrt(2.0f / (axisStepsPerMM[Y_AXIS] * accel));
    if (minimumSpeed2 > minimumSpeed) {
        minimumSpeed = minimumSpeed2;
    }

    if (maxXYJerk < 2 * minimumSpeed) { // Enforce minimum start speed if target is faster and jerk too low
        maxXYJerk = 2 * minimumSpeed;
        Com::printFLN(PSTR("XY jerk was too low, setting to "), maxXYJerk);
    }

#if FEATURE_MILLING_MODE
    if (Printer::operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE
        accel = RMath::max(maxAccelerationMMPerSquareSecond[Z_AXIS], maxTravelAccelerationMMPerSquareSecond[Z_AXIS]);
#if FEATURE_MILLING_MODE
    }
#endif // FEATURE_MILLING_MODE
    float minimumZSpeed = 0.5 * accel * sqrt(2.0f / (axisStepsPerMM[Z_AXIS] * accel));
    if (maxZJerk < 2 * minimumZSpeed) {
        maxZJerk = 2 * minimumZSpeed;
        Com::printFLN(PSTR("Z jerk was too low, setting to "), maxZJerk);
    }

    Printer::updateAdvanceActivated();
} // updateDerivedParameter

/** \brief Stop heater and stepper motors. Disable power,if possible. */
void Printer::switchEverythingOff() {
    if (Printer::isAllSwitchedOff())
        return;

#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
    // disable the fan
    Commands::setFanSpeed((uint8_t)0);
#endif // FAN_PIN>-1 && FEATURE_FAN_CONTROL

    Printer::disableAllSteppersNow();

    Extruder::setTemperatureForAllExtruders(0, false);
    Extruder::setHeatedBedTemperature(0);
    UI_STATUS_UPD(UI_TEXT_SWITCHED_OFF);

    Printer::setAllSwitchedOff(true);
} // switchEverythingOff

void Printer::updateAdvanceActivated() {
    Printer::setAdvanceActivated(false);
#if USE_ADVANCE
    for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
        if (extruder[i].advanceL != 0) {
            Printer::setAdvanceActivated(true);
        }
    }
#endif // USE_ADVANCE
} // updateAdvanceActivated

#if FEATURE_Kurt67_WOBBLE_FIX
void Printer::addKurtWobbleFixOffset() {
    /*
    Zielkoordinate in Z ist: "z"
    Wir fügen beim Umrechnen in Steps vorher noch ein Offset in XYZ ein.
    offsetZ = amplitude des hubs                     * drehposition spindel
    offsetY = amplitude in Richtung Y linke Spindel  * drehposition spindel
    offsetY = amplitude in Richtung Y rechte Spindel * drehposition spindel
    offsetX = amplitude in Richtung X                * drehposition spindel

    "sinOffset = amplitude * sin( 2*Pi*(Z/5mm + zStartOffset) );"
    */

    //vorausgerechnete konstanten.
    float zweiPi = 6.2832f;          //2*Pi
    float hundertstelPi = 0.031428f; //Pi/100
    float spindelSteigung = 5.0f;    //[mm]
    float zSchalterZ = Printer::currentZSteps * Printer::axisMMPerSteps[Z_AXIS];

    //wobble durch Bauchtanz der Druckplatte
    //wobbleX oder wobbleY(x0) oder wobbleX(x245)
    if (Printer::wobbleAmplitudes[0]) {
        float anglePositionWobble = cos(zweiPi * (zSchalterZ / spindelSteigung) - (float)Printer::wobblePhaseXY * hundertstelPi);
        //für das wobble in Richtung X-Achse gilt immer dieselbe Amplitude, weil im extremfall beide gegeneinander arbeiten und sich aufheben könnten, oder die Spindeln arbeiten zusammen.
        float wOffsetX = Printer::wobbleAmplitudes[0] * anglePositionWobble * 0.001; //offset in [mm]
        if (Printer::relativeCoordinateMode) {
            //fraglich wie lange das genau bleibt. Ist aber auch so beim relativen Drucken ein Problem.
            //addiere nur die Änderung des Offsets.
            Printer::destinationMM[X_AXIS] += (wOffsetX - Printer::lastWobbleFixOffset[X_AXIS]);
        } else {
            Printer::destinationMM[X_AXIS] += wOffsetX;
        }
        Printer::lastWobbleFixOffset[X_AXIS] = wOffsetX;
    }
    if (Printer::wobbleAmplitudes[1] || Printer::wobbleAmplitudes[2]) {
        float anglePositionWobble = sin(zweiPi * (zSchalterZ / spindelSteigung) - (float)Printer::wobblePhaseXY * hundertstelPi);
        //gilt eher die Y-Achsen-Richtung-Amplitude links (x=0) oder rechts (x=achsenlänge)? (abhängig von der ziel-x-position wird anteilig verrechnet.)
        float xPosPercent = float(Printer::currentXSteps) / float(Printer::maxSoftEndstopSteps[X_AXIS]);
        float wOffsetY = ((1 - xPosPercent) * Printer::wobbleAmplitudes[1] + (xPosPercent)*Printer::wobbleAmplitudes[2]) * anglePositionWobble * 0.001; //offset in [mm]
        if (Printer::relativeCoordinateMode) {
            //fraglich wie lange das genau bleibt. Ist aber auch so beim relativen Drucken ein Problem.
            //addiere nur die Änderung des Offsets.
            Printer::destinationMM[Y_AXIS] += (wOffsetY - Printer::lastWobbleFixOffset[Y_AXIS]);
        } else {
            Printer::destinationMM[Y_AXIS] += wOffsetY;
        }
        Printer::lastWobbleFixOffset[Y_AXIS] = wOffsetY;
    }
} // addKurtWobbleFixOffset
#endif // FEATURE_Kurt67_WOBBLE_FIX

inline bool isExtrusionAllowed(float e) {
    // calculate total amount of extrusion respecting coordinate mode
    float de = e;
    if (!Printer::relativeCoordinateMode && !Printer::relativeExtruderCoordinateMode) {
        de = e - Printer::destinationMM[E_AXIS];
    }

#if FEATURE_DIGIT_FLOW_COMPENSATION
    if (fabs(de) * Printer::menuExtrusionFactor * Printer::dynamicExtrusionFactor > EXTRUDE_MAXLENGTH) {
        return false;
    }
#else
    if (fabs(de) * Printer::menuExtrusionFactor > EXTRUDE_MAXLENGTH) {
        return false;
    }
#endif // FEATURE_DIGIT_FLOW_COMPENSATION

    if (Printer::debugDryrun()) {
        return false;
    }

    return true;
}

bool Printer::isZMoveActive() {
    if (PrintLine::direct.stepsRemaining && PrintLine::direct.isZMove()) {
        return true;
    }
    if (PrintLine::cur == NULL) {
        return false;
    }
    if (PrintLine::cur->isZMove()) {
        return true;
    }

    return false;
}

/**
* Set the printers feedrate according to active unit settings
*/
void Printer::setFeedrate(float feedrate) {
    if (feedrate <= 0)
        return;

    if (Printer::unitIsInches)
        Printer::feedrate = feedrate * (float)Printer::feedrateMultiply * 0.0042333f; // Factor is 25.4/60/100 = convertToMM(0.00016666666f)
    else
        Printer::feedrate = feedrate * (float)Printer::feedrateMultiply * 0.00016666666f;
}

/**
\brief Sets the destination coordinates to values stored in com.

For the computation of the destination, the following facts are considered:
- Are units inches or mm.
- Relative or absolute positioning with special case only extruder relative.
- Offset in x and y direction for multiple extruder support.

- originOffsetMM is only affecting gcode coordinates??
*/
bool Printer::queueGCodeCoordinates(GCode* com, bool noDriving) {
    InterruptProtectedBlock noInts;
    bool isXYZMove = !com->hasNoXYZ();
    if (relativeCoordinateMode) {
        if (com->hasX())
            destinationMM[X_AXIS] += convertToMM(com->X);
        if (com->hasY())
            destinationMM[Y_AXIS] += convertToMM(com->Y);
        if (com->hasZ())
            destinationMM[Z_AXIS] += convertToMM(com->Z);
    } else //absolute Coordinate Mode
    {
        if (com->hasX())
            destinationMM[X_AXIS] = convertToMM(com->X) - Printer::originOffsetMM[X_AXIS];
        if (com->hasY())
            destinationMM[Y_AXIS] = convertToMM(com->Y) - Printer::originOffsetMM[Y_AXIS];
        if (com->hasZ())
            destinationMM[Z_AXIS] = convertToMM(com->Z) - Printer::originOffsetMM[Z_AXIS];
    }
    noInts.unprotect();

    float e = convertToMM(com->E);
    bool isEMove = com->hasE() && isExtrusionAllowed(e);
    if (isEMove) {
        noInts.protect();
        if (relativeCoordinateMode || relativeExtruderCoordinateMode) {
            destinationMM[E_AXIS] += e;
        } else //absolute Coordinate Mode
        {
            destinationMM[E_AXIS] = e;
        }
        noInts.unprotect();
    }

    if (com->hasF()) {
        Printer::setFeedrate(com->F);
    }

#if FEATURE_Kurt67_WOBBLE_FIX
    if (isXYZMove) {
        addKurtWobbleFixOffset();
    }
#endif // FEATURE_Kurt67_WOBBLE_FIX

    if (noDriving) {
        return isXYZMove || isEMove;
    }

    if (isXYZMove || isEMove) {
        PrintLine::prepareQueueMove(false, true, Printer::feedrate);

        return true;
    }

    // report back moves which do not turn steppers
    return false;
} // queueGCodeCoordinates

/**
 * Move to Cartesian coordinates
 *
 * Does ignore origin offset? TODO
 * Does ignore wobble fix
 */
void Printer::queueFloatCoordinates(float x, float y, float z, float e, float feedrate) {
    InterruptProtectedBlock noInts;
    if (x == IGNORE_COORDINATE)
        x = destinationMM[X_AXIS];
    if (y == IGNORE_COORDINATE)
        y = destinationMM[Y_AXIS];
    if (z == IGNORE_COORDINATE)
        z = destinationMM[Z_AXIS];
    if (e == IGNORE_COORDINATE)
        e = destinationMM[E_AXIS];
    if (feedrate == IGNORE_COORDINATE)
        feedrate = Printer::feedrate;

    destinationMM[X_AXIS] = x;
    destinationMM[Y_AXIS] = y;
    destinationMM[Z_AXIS] = z;
    destinationMM[E_AXIS] = e;
    noInts.unprotect();

    PrintLine::prepareQueueMove(false, true, feedrate);

    previousMillisCmd = HAL::timeInMilliseconds(); //prevent inactive shutdown of steppers/temps
} // queueFloatCoordinates

/** \brief Move printer the given number of steps. Puts the move into the queue. Used by e.g. homing commands. */
void Printer::queueRelativeStepsCoordinates(long dx, long dy, long dz, long de, float feedrate, bool waitEnd, bool abortAtEndstops) {
    InterruptProtectedBlock noInts;
    destinationMM[X_AXIS] += dx * axisMMPerSteps[X_AXIS];
    destinationMM[Y_AXIS] += dy * axisMMPerSteps[Y_AXIS];
    destinationMM[Z_AXIS] += dz * axisMMPerSteps[Z_AXIS];
    destinationMM[E_AXIS] += de * axisMMPerSteps[E_AXIS];
    noInts.unprotect();

    PrintLine::prepareQueueMove(abortAtEndstops, !waitEnd, feedrate);

    if (waitEnd)
        Commands::waitUntilEndOfAllMoves();

    previousMillisCmd = HAL::timeInMilliseconds(); //prevent inactive shutdown of steppers/temps
} // queueRelativeStepsCoordinates

/** \brief Move printer the given number of mm. Puts the move into the queue. */
void Printer::queueRelativeMMCoordinates(float dx, float dy, float dz, float de, float feedrate, bool waitEnd, bool abortAtEndstops) {
    InterruptProtectedBlock noInts;
    destinationMM[X_AXIS] += dx;
    destinationMM[Y_AXIS] += dy;
    destinationMM[Z_AXIS] += dz;
    destinationMM[E_AXIS] += de;
    noInts.unprotect();

    PrintLine::prepareQueueMove(abortAtEndstops, !waitEnd, feedrate);

    if (waitEnd)
        Commands::waitUntilEndOfAllMoves();

    previousMillisCmd = HAL::timeInMilliseconds(); //prevent inactive shutdown of steppers/temps
} // queueRelativeMMCoordinates

/**
 * This function transfers the axis without affecting the axis values.
 * This is expecially usefull to transfer the extruder 0 to 1 and backwards.
 */
#define NORMAL_DIRECT false
#define STOPPABLE_DIRECT true
void Printer::offsetRelativeStepsCoordinates(int32_t dx, int32_t dy, int32_t dz, int32_t de, uint8_t configuration) {
#if FEATURE_MILLING_MODE
    // we do not process the extruder in case we are not in operating mode "print"
    if (Printer::operatingMode != OPERATING_MODE_PRINT) {
        de = 0;
    }
#endif // FEATURE_MILLING_MODE

    if (dx != 0 || dy != 0 || dz != 0 || de != 0) {
        while (PrintLine::direct.task) {
            // If another offset move is going on, we have to wait
            Commands::checkForPeriodicalActions(Processing);
        }

        InterruptProtectedBlock noInts;
        //braucht man das Stop noch?
        Printer::stopDirectAxis(X_AXIS);
        Printer::stopDirectAxis(Y_AXIS);
        Printer::stopDirectAxis(Z_AXIS);
        Printer::stopDirectAxis(E_AXIS);

        Printer::directDestinationSteps[X_AXIS] += dx;
        Printer::directDestinationSteps[Y_AXIS] += dy;
        Printer::directDestinationSteps[Z_AXIS] += dz;
        Printer::directDestinationSteps[E_AXIS] += de;

        bool feedrateSource = FEEDRATE_DIRECTCONFIG;
        if (configuration == TASK_MOVE_FROM_BUTTON || configuration == TASK_MOVE_POSITION_MANUAL) {
            // In position menu or when moving using z-buttons we always choose the feedrate type configured in menu->position->Pos-Feedrate {direct or gcode}
            // single Z feedrate is always limited to direct config as well as max-feedrate
            feedrateSource = Printer::movePositionFeedrateChoice;
        }

        // If we start this move by pressing a button we dont want to wait for it to finish and instead have it listening to the buttons release
        if (configuration == TASK_MOVE_FROM_BUTTON) {
            PrintLine::prepareDirectMove(STOPPABLE_DIRECT, feedrateSource);
            //noInts handled by destructor

            return;
        }

        PrintLine::prepareDirectMove(NORMAL_DIRECT, feedrateSource);

        //release Interrupts for processing
        noInts.unprotect();

        while (PrintLine::direct.task) {
            // Wait until this offset move is finished. Otherwise could be interrupted by queue-moves and end up directStepping
            Commands::checkForPeriodicalActions(Processing);
        }
    }
}

void Printer::setOrigin(float xOff, float yOff, float zOff) {
    Com::printF(PSTR("setOrigin():"));
    bool someFailed = false;

    if (isAxisHomed(X_AXIS) || xOff == 0.0f) {
        originOffsetMM[X_AXIS] = xOff;
        Com::printF(PSTR("x="), originOffsetMM[X_AXIS]);
    } else {
        Com::printF(PSTR("x-fail"));
        someFailed = true;
    }
    if (isAxisHomed(Y_AXIS) || yOff == 0.0f) {
        originOffsetMM[Y_AXIS] = yOff;
        Com::printF(PSTR(" y="), originOffsetMM[Y_AXIS]);
    } else {
        Com::printF(PSTR(" y-fail"));
        someFailed = true;
    }
    if (isAxisHomed(Z_AXIS) || zOff == 0.0f) {
        originOffsetMM[Z_AXIS] = zOff;
        Com::printF(PSTR(" z="), originOffsetMM[Z_AXIS]);
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

    for (uint8_t i = 0; i < NUM_EXTRUDER + 3; i++)
        pwm_pos[i] = 0;

#if FEATURE_USER_INT3
    SET_INPUT(RESERVE_DIGITAL_PIN_PD3);
    PULLUP(RESERVE_DIGITAL_PIN_PD3, HIGH);
#endif //FEATURE_USER_INT3

#if FEATURE_READ_CALIPER
    // read for using pins : https://www.arduino.cc/en/Tutorial/DigitalPins
    //where the clock comes in and triggers an interrupt which reads data then:
    SET_INPUT(FEATURE_READ_CALIPER_INT_PIN);     //input as default already this is here for explaination more than really having an input.
    PULLUP(FEATURE_READ_CALIPER_INT_PIN, HIGH);  //do I need this pullup??
                                                 //where data is to read when Int triggers because of clock from caliper:
    SET_INPUT(FEATURE_READ_CALIPER_DATA_PIN);    //input as default already this is here for explaination more than really having an input.
    PULLUP(FEATURE_READ_CALIPER_DATA_PIN, HIGH); //do I need this pullup??
#endif                                           //FEATURE_READ_CALIPER

    HAL::allowInterrupts();

#if FEATURE_USER_INT3
    attachInterrupt(digitalPinToInterrupt(RESERVE_DIGITAL_PIN_PD3), USER_INTERRUPT3_HOOK, FALLING);
#endif //FEATURE_USER_INT3

#if FEATURE_READ_CALIPER
    attachInterrupt(digitalPinToInterrupt(FEATURE_READ_CALIPER_INT_PIN), FEATURE_READ_CALIPER_HOOK, FALLING);
#endif //FEATURE_READ_CALIPER

#if FEATURE_BEEPER
    enableBeeper = BEEPER_MODE;
#endif // FEATURE_BEEPER

    Wire.begin();

#if FEATURE_TYPE_EEPROM
    determineHardwareType();

    if (wrongType) {
        // this firmware is not for this hardware
        while (1) {
            // this firmware shall not try to do anything at this hardware
        }
    }
#endif // FEATURE_TYPE_EEPROM

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

#if FEATURE_TWO_XSTEPPER
    SET_OUTPUT(X2_STEP_PIN);
    SET_OUTPUT(X2_DIR_PIN);

#if X2_ENABLE_PIN > -1
    SET_OUTPUT(X2_ENABLE_PIN);
    if (!X_ENABLE_ON)
        WRITE(X2_ENABLE_PIN, HIGH);
#endif // X2_ENABLE_PIN > -1
#endif // FEATURE_TWO_XSTEPPER

#if FEATURE_TWO_YSTEPPER
    SET_OUTPUT(Y2_STEP_PIN);
    SET_OUTPUT(Y2_DIR_PIN);

#if Y2_ENABLE_PIN > -1
    SET_OUTPUT(Y2_ENABLE_PIN);
    if (!Y_ENABLE_ON)
        WRITE(Y2_ENABLE_PIN, HIGH);
#endif // Y2_ENABLE_PIN > -1
#endif // FEATURE_TWO_YSTEPPER

#if FEATURE_TWO_ZSTEPPER
    SET_OUTPUT(Z2_STEP_PIN);
    SET_OUTPUT(Z2_DIR_PIN);

#if X2_ENABLE_PIN > -1
    SET_OUTPUT(Z2_ENABLE_PIN);
    if (!Z_ENABLE_ON)
        WRITE(Z2_ENABLE_PIN, HIGH);
#endif // X2_ENABLE_PIN > -1
#endif // FEATURE_TWO_ZSTEPPER

        //endstop pullups
#if MIN_HARDWARE_ENDSTOP_X
#if X_MIN_PIN > -1
    SET_INPUT(X_MIN_PIN);

#if ENDSTOP_PULLUP_X_MIN
    PULLUP(X_MIN_PIN, HIGH);
#endif // ENDSTOP_PULLUP_X_MIN
#else
#error You have defined hardware x min endstop without pin assignment. Set pin number for X_MIN_PIN
#endif // X_MIN_PIN>-1
#endif // MIN_HARDWARE_ENDSTOP_X

#if MIN_HARDWARE_ENDSTOP_Y
#if Y_MIN_PIN > -1
    SET_INPUT(Y_MIN_PIN);

#if ENDSTOP_PULLUP_Y_MIN
    PULLUP(Y_MIN_PIN, HIGH);
#endif // ENDSTOP_PULLUP_Y_MIN
#else
#error You have defined hardware y min endstop without pin assignment. Set pin number for Y_MIN_PIN
#endif // Y_MIN_PIN>-1
#endif // MIN_HARDWARE_ENDSTOP_Y

#if MIN_HARDWARE_ENDSTOP_Z
#if Z_MIN_PIN > -1
    SET_INPUT(Z_MIN_PIN);

#if ENDSTOP_PULLUP_Z_MIN
    PULLUP(Z_MIN_PIN, HIGH);
#endif // ENDSTOP_PULLUP_Z_MIN
#else
#error You have defined hardware z min endstop without pin assignment. Set pin number for Z_MIN_PIN
#endif // Z_MIN_PIN>-1
#endif // MIN_HARDWARE_ENDSTOP_Z

#if MAX_HARDWARE_ENDSTOP_X
#if X_MAX_PIN > -1
    SET_INPUT(X_MAX_PIN);

#if ENDSTOP_PULLUP_X_MAX
    PULLUP(X_MAX_PIN, HIGH);
#endif // ENDSTOP_PULLUP_X_MAX
#else
#error You have defined hardware x max endstop without pin assignment. Set pin number for X_MAX_PIN
#endif // X_MAX_PIN>-1
#endif // MAX_HARDWARE_ENDSTOP_X

#if MAX_HARDWARE_ENDSTOP_Y
#if Y_MAX_PIN > -1
    SET_INPUT(Y_MAX_PIN);

#if ENDSTOP_PULLUP_Y_MAX
    PULLUP(Y_MAX_PIN, HIGH);
#endif // ENDSTOP_PULLUP_Y_MAX
#else
#error You have defined hardware y max endstop without pin assignment. Set pin number for Y_MAX_PIN
#endif // Y_MAX_PIN>-1
#endif // MAX_HARDWARE_ENDSTOP_Y

#if MAX_HARDWARE_ENDSTOP_Z
#if Z_MAX_PIN > -1
    SET_INPUT(Z_MAX_PIN);

#if ENDSTOP_PULLUP_Z_MAX
    PULLUP(Z_MAX_PIN, HIGH);
#endif // ENDSTOP_PULLUP_Z_MAX
#else
#error You have defined hardware z max endstop without pin assignment. Set pin number for Z_MAX_PIN
#endif // Z_MAX_PIN>-1
#endif // MAX_HARDWARE_ENDSTOP_Z

#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
    SET_OUTPUT(FAN_PIN);
    WRITE(FAN_PIN, LOW);
#endif // FAN_PIN>-1 && FEATURE_FAN_CONTROL

#if FAN_BOARD_PIN > -1
    SET_OUTPUT(FAN_BOARD_PIN);
    WRITE(FAN_BOARD_PIN, LOW);
#endif // FAN_BOARD_PIN>-1

#if EXT0_HEATER_PIN > -1
    SET_OUTPUT(EXT0_HEATER_PIN);
    WRITE(EXT0_HEATER_PIN, HEATER_PINS_INVERTED);
#endif // EXT0_HEATER_PIN>-1

#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN > -1 && NUM_EXTRUDER > 1
    SET_OUTPUT(EXT1_HEATER_PIN);
    WRITE(EXT1_HEATER_PIN, HEATER_PINS_INVERTED);
#endif // defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1 && NUM_EXTRUDER>1

#if EXT0_EXTRUDER_COOLER_PIN > -1
    SET_OUTPUT(EXT0_EXTRUDER_COOLER_PIN);
    WRITE(EXT0_EXTRUDER_COOLER_PIN, LOW);
#endif // EXT0_EXTRUDER_COOLER_PIN >- 1

#if defined(EXT1_EXTRUDER_COOLER_PIN) && EXT1_EXTRUDER_COOLER_PIN > -1 && NUM_EXTRUDER > 1
    SET_OUTPUT(EXT1_EXTRUDER_COOLER_PIN);
    WRITE(EXT1_EXTRUDER_COOLER_PIN, LOW);
#endif // defined(EXT1_EXTRUDER_COOLER_PIN) && EXT1_EXTRUDER_COOLER_PIN >- 1 && NUM_EXTRUDER > 1

#if USE_ADVANCE
    advanceStepsSet = 0;
#endif // USE_ADVANCE

    maxXYJerk = MAX_JERK;
    maxZJerk = MAX_ZJERK;

    flag0 = PRINTER_FLAG0_STEPPER_DISABLED;

    moveMode[X_AXIS] = DEFAULT_MOVE_MODE_X;
    moveMode[Y_AXIS] = DEFAULT_MOVE_MODE_Y;
    moveMode[Z_AXIS] = DEFAULT_MOVE_MODE_Z;

#if FEATURE_HEAT_BED_Z_COMPENSATION
    doHeatBedZCompensation = 0; //init
#endif                          // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_WORK_PART_Z_COMPENSATION
    doWorkPartZCompensation = 0; //init
    staticCompensationZ = 0;     //init
#endif                           // FEATURE_WORK_PART_Z_COMPENSATION

    blockAll = 0;

#if FEATURE_MILLING_MODE
    operatingMode = DEFAULT_OPERATING_MODE;
    drillFeedrate = 0.0;
    drillZDepth = 0.0;
#endif // FEATURE_MILLING_MODE

#if FEATURE_MILLING_MODE
    if (Printer::operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE
        Printer::homingFeedrate[X_AXIS] = HOMING_FEEDRATE_X_PRINT;
        Printer::homingFeedrate[Y_AXIS] = HOMING_FEEDRATE_Y_PRINT;
        Printer::homingFeedrate[Z_AXIS] = HOMING_FEEDRATE_Z_PRINT;
        Printer::axisLengthMM[X_AXIS] = X_MAX_LENGTH_PRINT;
#if FEATURE_MILLING_MODE
    } else {
        Printer::homingFeedrate[X_AXIS] = HOMING_FEEDRATE_X_MILL;
        Printer::homingFeedrate[Y_AXIS] = HOMING_FEEDRATE_Y_MILL;
        Printer::homingFeedrate[Z_AXIS] = HOMING_FEEDRATE_Z_MILL;
        Printer::axisLengthMM[X_AXIS] = X_MAX_LENGTH_MILL;
    }
#endif // FEATURE_MILLING_MODE

    Printer::axisLengthMM[Y_AXIS] = Y_MAX_LENGTH;
    Printer::axisLengthMM[Z_AXIS] = Z_MAX_LENGTH;

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
    ZEndstopType = DEFAULT_Z_ENDSTOP_TYPE;
    ZEndstopUnknown = 0;
    lastZDirection = 0;
    endstopZMinHit = ENDSTOP_NOT_HIT;
    endstopZMaxHit = ENDSTOP_NOT_HIT;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

#if FEATURE_CASE_FAN
    fanOffDelay = CASE_FAN_OFF_DELAY;
#endif // FEATURE_CASE_FAN

#if FEATURE_RGB_LIGHT_EFFECTS
    RGBLightMode = RGB_LIGHT_DEFAULT_MODE;
    if (RGBLightMode == RGB_MODE_AUTOMATIC) {
        RGBLightStatus = RGB_STATUS_AUTOMATIC;
    } else {
        RGBLightStatus = RGB_STATUS_NOT_AUTOMATIC;
    }
    RGBLightIdleStart = 0;
    RGBButtonBackPressed = 0;
    RGBLightModeForceWhite = 0;
#endif // FEATURE_RGB_LIGHT_EFFECTS

#if USE_ADVANCE
    extruderStepsNeeded = 0;
#endif // USE_ADVANCE

    UI_INITIALIZE;

    EEPROM::initBaudrate();
    HAL::serialSetBaudrate(baudrate);

    // sending of this information tells the Repetier-Host that the firmware has restarted - never delete or change this to-be-sent information
    Com::println();
    Com::printFLN(Com::tStart); //http://forum.repetier.com/discussion/comment/16949/#Comment_16949

    HAL::showStartReason();
    Extruder::initExtruder();

#if HEATED_BED_HEATER_PIN > -1
    SET_OUTPUT(HEATED_BED_HEATER_PIN);
    WRITE(HEATED_BED_HEATER_PIN, HEATER_PINS_INVERTED);
    Extruder::initHeatedBed();
#endif // HEATED_BED_HEATER_PIN>-1

    HAL::analogStart();

    // configure all DRV8711
    drv8711Init();

    // Read settings from eeprom if wanted [readDataFromEEPROM or destroy corrupted eeprom]
    EEPROM::init();

#if FEATURE_230V_OUTPUT
    enable230VOutput = OUTPUT_230V_DEFAULT_ON;
    SET_OUTPUT(OUTPUT_230V_PIN);
    WRITE(OUTPUT_230V_PIN, enable230VOutput);
#endif // FEATURE_230V_OUTPUT

#if FEATURE_24V_FET_OUTPUTS
#if FET1 > -1
    SET_OUTPUT(FET1);
    WRITE(FET1, enableFET1);
#endif // FET1
#if FET2 > -1
    SET_OUTPUT(FET2);
    WRITE(FET2, enableFET2);
#endif // FET2
#if FET3 > -1 && !(FEATURE_CASE_FAN && CASE_FAN_PIN > -1)
    SET_OUTPUT(FET3);
    WRITE(FET3, enableFET3);
#endif // FET3
#endif // FEATURE_24V_FET_OUTPUTS

#if FEATURE_CASE_FAN && CASE_FAN_PIN > -1
    SET_OUTPUT(CASE_FAN_PIN);

#if CASE_FAN_ALWAYS_ON
    WRITE(CASE_FAN_PIN, 1);
#else
    WRITE(CASE_FAN_PIN, 0);
#endif // CASE_FAN_ALWAYS_ON
#endif // FEATURE_CASE_FAN && CASE_FAN_PIN > -1

#if FEATURE_CASE_LIGHT
    SET_OUTPUT(CASE_LIGHT_PIN);
    WRITE(CASE_LIGHT_PIN, enableCaseLight);
#endif // FEATURE_CASE_LIGHT

    updateDerivedParameter();
    Commands::checkFreeMemory();
    Commands::writeLowestFreeRAM();
    HAL::setupTimer();

    Extruder::selectExtruderById(0);

#if SDSUPPORT
    sd.mount(true /* Silent mount because otherwise RF1000 prints errors if no sdcard is present at boottime*/);
#endif // SDSUPPORT

#if FEATURE_RGB_LIGHT_EFFECTS
    setRGBLEDs(0, 0, 0);

    switch (RGBLightMode) {
    case RGB_MODE_OFF: {
        setRGBTargetColors(0, 0, 0);
        break;
    }
    case RGB_MODE_WHITE: {
        setRGBTargetColors(255, 255, 255);
        break;
    }
    case RGB_MODE_AUTOMATIC: {
        setRGBTargetColors(g_uRGBIdleR, g_uRGBIdleG, g_uRGBIdleB);
        break;
    }
    case RGB_MODE_MANUAL: {
        setRGBTargetColors(g_uRGBManualR, g_uRGBManualG, g_uRGBManualB);
        break;
    }
    }
#endif // FEATURE_RGB_LIGHT_EFFECTS

#if FEATURE_UNLOCK_MOVEMENT
    g_unlock_movement = 0;
#endif //FEATURE_UNLOCK_MOVEMENT

    HAL::startWatchdog();
} // setup()

#if FEATURE_MEMORY_POSITION
void Printer::MemoryPosition() {
    Commands::waitUntilEndOfAllMoves();

    Printer::memoryX = Printer::destinationMM[X_AXIS];
    Printer::memoryY = Printer::destinationMM[Y_AXIS];
    Printer::memoryZ = Printer::destinationMM[Z_AXIS];
    Printer::memoryE = Printer::destinationMM[E_AXIS];
    Printer::memoryF = Printer::feedrate;
} // MemoryPosition

void Printer::GoToMemoryPosition(bool x, bool y, bool z, bool e, float feed) {
    bool all = !(x || y || z);
    queueFloatCoordinates((all || x ? Printer::memoryX : IGNORE_COORDINATE), (all || y ? Printer::memoryY : IGNORE_COORDINATE), (all || z ? Printer::memoryZ : IGNORE_COORDINATE), (e ? Printer::memoryE : IGNORE_COORDINATE),
                          feed);
    Printer::feedrate = Printer::memoryF;
} // GoToMemoryPosition
#endif // FEATURE_MEMORY_POSITION

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

#if FEATURE_SENSIBLE_PRESSURE
void Printer::enableSenseOffsetnow(void) {
    short oldval = HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS);
    if (oldval > 0 && oldval < EMERGENCY_PAUSE_DIGITS_MAX) {
        g_nSensiblePressureDigits = oldval;
    }
    oldval = HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX);
    if (oldval > 0 && oldval < 300) {
        g_nSensiblePressureOffsetMax = oldval;
    }
}
#endif // FEATURE_SENSIBLE_PRESSURE

void Printer::enableCMPnow(void) {
#if FEATURE_MILLING_MODE
    if (Printer::operatingMode == OPERATING_MODE_MILL) {
#if FEATURE_WORK_PART_Z_COMPENSATION
        if (Printer::doWorkPartZCompensation)
            return; // false;
#endif              // FEATURE_WORK_PART_Z_COMPENSATION
    } else
#endif // FEATURE_MILLING_MODE
    {
#if FEATURE_HEAT_BED_Z_COMPENSATION
        if (Printer::doHeatBedZCompensation)
            return; // false;
#endif              // FEATURE_HEAT_BED_Z_COMPENSATION
    }

    if (!Printer::areAxisHomed()) {
        return; // false;
    }

    // enable the z compensation
    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT)
        return; // false;

        // enable the z compensation only in case we have valid compensation values
#if FEATURE_MILLING_MODE
    if (Printer::operatingMode == OPERATING_MODE_MILL) {
#if FEATURE_WORK_PART_Z_COMPENSATION
        Printer::doWorkPartZCompensation = 1;
#if FEATURE_FIND_Z_ORIGIN
        if (g_nZOriginPosition[X_AXIS] || g_nZOriginPosition[Y_AXIS]) {
            Printer::staticCompensationZ = getZMatrixDepth(g_nZOriginPosition[X_AXIS], g_nZOriginPosition[Y_AXIS]); //determineStaticCompensationZ();
        } else {
            // we know nothing about a static z-delta in case we do not know the x and y positions at which the z-origin has been determined
            Printer::staticCompensationZ = 0;
        }
#else
        // we know nothing about a static z-delta when we do not have the automatic search of the z-origin available
        Printer::staticCompensationZ = 0;
#endif // FEATURE_FIND_Z_ORIGIN
#endif // FEATURE_WORK_PART_Z_COMPENSATION
    } else
#endif // FEATURE_MILLING_MODE
    {
#if FEATURE_HEAT_BED_Z_COMPENSATION
        Printer::doHeatBedZCompensation = 1;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION
    }
}

bool Printer::needsCMPwait(void) {
    if (abs(Printer::compensatedPositionCurrentStepsZ - Printer::compensatedPositionTargetStepsZ)) {
        if (!Printer::checkCMPblocked())
            return true;
    }
    return false;
}

bool Printer::checkCMPblocked(void) {
    //die zcmp läuft dann, wenn die achse nicht gegenläufig steht und reserviert ist:
    //true = zcmp ist blockiert -> ist mal möglich aber wenn nichts läuft ein fehler, weil die achse nicht zeitnah von einer anderen funktion freigegeben wird.
    //false = zcmp darf sich aktuell korrigieren, wenn sonst keine prioritären Einschränkungen aktiv sind. -> auf kompensationsende warten müsste erfolgreich verlaufen
    return ((Printer::compensatedPositionCurrentStepsZ < Printer::compensatedPositionTargetStepsZ) && !Printer::getZDirectionIsPos())
        || ((Printer::compensatedPositionCurrentStepsZ > Printer::compensatedPositionTargetStepsZ) && Printer::getZDirectionIsPos());
}

void Printer::disableCMPnow(bool wait) {
#if FEATURE_MILLING_MODE
    if (Printer::operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE
#if FEATURE_HEAT_BED_Z_COMPENSATION
        Printer::doHeatBedZCompensation = 0;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION
#if FEATURE_MILLING_MODE
#if FEATURE_WORK_PART_Z_COMPENSATION
    } else {
        Printer::doWorkPartZCompensation = 0;
        Printer::staticCompensationZ = 0;
    }
#endif                                            // FEATURE_WORK_PART_Z_COMPENSATION
#endif                                            // FEATURE_MILLING_MODE
    Printer::compensatedPositionTargetStepsZ = 0; //tell CMP to move to 0. TODO: Care for positive matrix beds.

    while (wait && compensatedPositionCurrentStepsZ - compensatedPositionTargetStepsZ) //warte auf queue befehlsende
    {
        HAL::delayMilliseconds(1);
        //checkforperiodical .. possible loop inside scans and homing.
    }
}
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

int8_t Printer::anyHomeDir(uint8_t axis) {
    int8_t nHomeDir = 0;
    switch (axis) {
    case X_AXIS: {
        if ((MIN_HARDWARE_ENDSTOP_X && X_MIN_PIN > -1 && X_HOME_DIR == -1) || (MAX_HARDWARE_ENDSTOP_X && X_MAX_PIN > -1 && X_HOME_DIR == 1)) {
            nHomeDir = X_HOME_DIR; //-1 -> RFx000.h
        }
        break;
    }
    case Y_AXIS: {
        if ((MIN_HARDWARE_ENDSTOP_Y && Y_MIN_PIN > -1 && Y_HOME_DIR == -1) || (MAX_HARDWARE_ENDSTOP_Y && Y_MAX_PIN > -1 && Y_HOME_DIR == 1)) {
            nHomeDir = Y_HOME_DIR; //-1 -> RFx000.h
        }
        break;
    }
    case Z_AXIS: {
#if FEATURE_MILLING_MODE
        if (Printer::operatingMode == OPERATING_MODE_PRINT && (MIN_HARDWARE_ENDSTOP_Z && Z_MIN_PIN > -1)) {
            // in operating mode "print" we use the z min endstop
            nHomeDir = Z_HOME_DIR;
        } else if (MAX_HARDWARE_ENDSTOP_Z && Z_MAX_PIN > -1) {
            // in operating mode "mill" we use the z max endstop
            nHomeDir = -1 * Z_HOME_DIR;
        }
#else
        if ((MIN_HARDWARE_ENDSTOP_Z && Z_MIN_PIN > -1 && Z_HOME_DIR == -1) || (MAX_HARDWARE_ENDSTOP_Z && Z_MAX_PIN > -1 && Z_HOME_DIR == 1)) {
            nHomeDir = Z_HOME_DIR; //-1 -> RFx000.h
        }
        break;
#endif // FEATURE_MILLING_MODE
    }
    }
    return nHomeDir;
}

#if FEATURE_CHECK_HOME
bool Printer::anyEndstop(uint8_t axis) {
    int endstop = 0;
    for (uint8_t i = 1; i <= 3; i++) {
        switch (axis) {
        case X_AXIS: {
            endstop += (Printer::isXMinEndstopHit() || Printer::isXMaxEndstopHit());
            break;
        }
        case Y_AXIS: {
            endstop += (Printer::isYMinEndstopHit() || Printer::isYMaxEndstopHit());
            break;
        }
        case Z_AXIS: {
            endstop += (Printer::isZMinEndstopHit() || Printer::isZMaxEndstopHit());
            break;
        }
        }
    }
    return (bool)(endstop > 1); //endstop muss sauber gedrückt sein.
}

void Printer::changeAxisDirection(uint8_t axis, int8_t direction) {
    switch (axis) {
    case X_AXIS: {
        Printer::setXDirection((direction > 0 ? true : false));
        break;
    }
    case Y_AXIS: {
        Printer::setYDirection((direction > 0 ? true : false));
        break;
    }
    case Z_AXIS: {
        Printer::setZDirection((direction > 0 ? true : false));
        break;
    }
    }
}

void Printer::startAxisStep(uint8_t axis) {
    switch (axis) {
    case X_AXIS: {
        Printer::startXStep();
        break;
    }
    case Y_AXIS: {
        Printer::startYStep();
        break;
    }
    case Z_AXIS: {
        Printer::startZStep();
        break;
    }
    }
}
void Printer::endAxisStep(uint8_t axis) {
    switch (axis) {
    case X_AXIS: {
        Printer::endXStep();
        break;
    }
    case Y_AXIS: {
        Printer::endYStep();
        break;
    }
    case Z_AXIS: {
        Printer::endZStep();
        break;
    }
    }
}

void Printer::stepAxisStep(uint8_t axis, uint8_t slower) {
    HAL::delayMicroseconds(XYZ_STEPPER_HIGH_DELAY * slower);
    Printer::startAxisStep(axis);
    HAL::delayMicroseconds(XYZ_STEPPER_LOW_DELAY * slower);
    Printer::endAxisStep(axis);
}

int8_t Printer::checkHome(int8_t axis) //X_AXIS 0, Y_AXIS 1, Z_AXIS 2
{
    //do not allow Z jet because we have to check compensation behaviour
    Com::printFLN(PSTR("checkHome: start axis"), axis);
    //if(axis == Z_AXIS) return -1; //noch verboten, weil solange das läuft der emergency-Stop nicht funktioniert. funktion wird evtl. wie HBS oder ZOS geteilt.

    if (axis > Z_AXIS)
        return -1;

    //do not allow checkHome without Home
    if (!Printer::isAxisHomed(axis))
        return -1;
    Com::printFLN(PSTR("ishomed!"));

    //do not allow checkHome when some endstop is already reached
    if (Printer::anyEndstop(axis))
        return -1; //knopf gedrückt = kein sinnvoller rückfahrweg bzw. nicht vertrauenswürdige coordinaten.

    //get the right homeDirection for moving -> 0 is none.
    char nHomeDir = Printer::anyHomeDir(axis);
    Com::printFLN(PSTR("nHomeDir="), nHomeDir);
    if (!nHomeDir)
        return -1;

    //set stepper direction as homing direction
    Printer::changeAxisDirection(axis, nHomeDir);

    //previous motion state:
    long oldCurrentSteps;
    if (axis == Z_AXIS)
        oldCurrentSteps = Printer::currentZSteps; //zählt hoch und runter. alternativ achsen-kosys wie bei x und y, aber wegen zcompensation problematisch, weil verzerrt (??-> noch nicht fertig durchdacht).
    else if (axis == Y_AXIS)
        oldCurrentSteps = Printer::currentYSteps;
    else
        oldCurrentSteps = Printer::currentXSteps;
    //write previous motion state to log
    Com::printFLN(PSTR("Current position steps="), oldCurrentSteps);

    //drive axis towards endstop:
    long returnSteps = 0;
    bool finish = false;
    int mmLoops = Printer::axisStepsPerMM[axis]; //fahre in mm-Blöcken.

    while (1) {
        Commands::checkForPeriodicalActions(Calibrating);

        for (int i = 0; i < mmLoops; i++) {
            Printer::stepAxisStep(axis);
            if (Printer::anyEndstop(axis)) {
                finish = true;
            } else {
                returnSteps += nHomeDir;
            }
            //verbot unendlich weiterzufahren, wenn endstop kaputt
            if (abs(returnSteps) > abs(Printer::maxSoftEndstopSteps[axis]) * 110 / 100)
                finish = true;

            if (finish)
                break;
        }
        if (finish)
            break;
    }

    //steps to log
    Com::printFLN(PSTR("Return steps="), returnSteps);

    Com::printFLN(PSTR("Recheck endstop:"));
    //check ob endstop da (oder defekt oder nicht erreicht: return)
    if (!Printer::anyEndstop(axis))
        return -1;

    //change stepper direction to drive against homing direction
    Printer::changeAxisDirection(axis, -1 * nHomeDir);
    //Ein paar steps zurück, bis schalter wieder losgelassen wurde:
    finish = false; // Neue Abbruchbedingung
    for (int i = 0; i < mmLoops; i++) {
        Printer::stepAxisStep(axis, 20 /*20x langsamer*/);
        //end if button is not pressed anymore.
        if (!Printer::anyEndstop(axis)) {
            finish = true;
        } else {
            returnSteps += -1 * nHomeDir;
        }
        if (finish)
            break;
    }

    //check ob endstop nach 1mm immernoch gedrückt:
    if (Printer::anyEndstop(axis))
        return -1;

    //merke schalter-aus-punkt für hysteresemessung:
    long hysterese = returnSteps;

    //Neu und langsamer in den Schalter fahren:
    //change stepper direction to drive against homing direction
    Printer::changeAxisDirection(axis, nHomeDir);
    finish = false; // Neue Abbruchbedingung
    for (int i = 0; i < mmLoops; i++) {
        Printer::stepAxisStep(axis, 20 /*20x langsamer*/);
        //end if button is not pressed anymore.
        if (Printer::anyEndstop(axis)) {
            finish = true;
        } else {
            returnSteps += nHomeDir;
        }
        if (finish)
            break;
    }
    //check ob endstop nach 1mm immernoch nicht gedrückt:
    if (!Printer::anyEndstop(axis))
        return -1;

    hysterese -= returnSteps;
    Com::printFLN(PSTR("aus-ein-hysterese="), hysterese);

    //check ob steps verloren gehen:
    long delta;
    if (axis == Z_AXIS)
        delta = Printer::currentZSteps; //zählt hoch und runter. alternativ achsen-kosys wie bei x und y, aber wegen zcompensation problematisch, weil verzerrt (??-> noch nicht fertig durchdacht).
    else if (axis == Y_AXIS)
        delta = Printer::currentYSteps;
    else
        delta = Printer::currentXSteps;

    Com::printFLN(PSTR("Steps delta="), delta);

    //fahre zurück an Startpunkt:
    Printer::changeAxisDirection(axis, -1 * nHomeDir);

    returnSteps = abs(returnSteps);
    while (returnSteps > 0) {
        Commands::checkForPeriodicalActions(Calibrating);

        if (returnSteps >= mmLoops) {
            returnSteps -= mmLoops;
        } else {
            mmLoops = returnSteps;
            returnSteps = 0;
        }

        for (int i = 0; i < mmLoops; i++) {
            Printer::stepAxisStep(axis);
        }
    }

    return 1;
} // checkHome
#endif // FEATURE_CHECK_HOME

void Printer::homeXAxis() {
    char nHomeDir = Printer::anyHomeDir(X_AXIS);

    if (nHomeDir) {
        UI_STATUS_UPD(UI_TEXT_HOME_X);
        Printer::resetDirectAxis(X_AXIS);

        int32_t offX = 0;
#if NUM_EXTRUDER > 1
        // Reposition extruder that way, that all extruders can be selected at home pos.
        for (uint8_t i = 0; i < NUM_EXTRUDER; i++)
            offX = (nHomeDir == -1)
                ? RMath::max(offX, int32_t(extruder[i].offsetMM[X_AXIS] * Printer::axisStepsPerMM[X_AXIS]))
                : RMath::min(offX, int32_t(extruder[i].offsetMM[X_AXIS] * Printer::axisStepsPerMM[X_AXIS]));
#endif // NUM_EXTRUDER>1

#if FEATURE_MILLING_MODE
        if (Printer::operatingMode == OPERATING_MODE_MILL)
            offX = 0; // in operating mode mill, there is no extruder offset
#endif                // FEATURE_MILLING_MODE

        Printer::queueRelativeStepsCoordinates(long(2 * abs(maxSoftEndstopSteps[X_AXIS] + 2 * offX) * nHomeDir), 0, 0, 0, homingFeedrate[X_AXIS], false, true); //stop at endstop
        Printer::queueRelativeMMCoordinates(-1 * ENDSTOP_X_BACK_MOVE * nHomeDir, 0, 0, 0, homingFeedrate[X_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, false);
        Printer::queueRelativeMMCoordinates(2 * ENDSTOP_X_BACK_MOVE * nHomeDir, 0, 0, 0, homingFeedrate[X_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, true); //stop at endstop, wait

        // currentXSteps ist die Schalter-X-Koordinate, die die X-Steps per Dir-Pin abzählt.																																			//currentYSteps ist die Schalter-Y-Koordinate, die die Y-Steps per Dir-Pin abzählt.
        Printer::currentXSteps = (nHomeDir == -1) ? 0 : maxSoftEndstopSteps[X_AXIS];

#if NUM_EXTRUDER > 1
        if (offX) {
            //Wir halten z.B. von Links soviel Abstand wie der größte Extruder-Offset. Wir fahren aber nur so weit, dass das aktuelle Hotend dort steht.
            Printer::offsetRelativeStepsCoordinates(-1 * nHomeDir * (offX * axisMMPerSteps[X_AXIS] - Extruder::current->offsetMM[X_AXIS]) * Printer::axisStepsPerMM[X_AXIS], 0, 0, 0);
        }
#endif // NUM_EXTRUDER>1

        // Set axis value here.
        Printer::setXAxisSteps((nHomeDir == -1) ? 0 : maxSoftEndstopSteps[X_AXIS]);

        // show that we are active
        previousMillisCmd = HAL::timeInMilliseconds(); //prevent inactive shutdown of steppers/temps
        setHomed(true, -1, -1);
        UI_STATUS_UPD("");
    }
} // homeXAxis

void Printer::homeYAxis() {
    char nHomeDir = Printer::anyHomeDir(Y_AXIS);

    if (nHomeDir) {
        UI_STATUS_UPD(UI_TEXT_HOME_Y);
        Printer::resetDirectAxis(Y_AXIS);

        int32_t offY = 0;
#if NUM_EXTRUDER > 1
        // Reposition extruder that way, that all extruders can be selected at home pos.
        for (uint8_t i = 0; i < NUM_EXTRUDER; i++)
            offY = (nHomeDir == -1)
                ? RMath::max(offY, int32_t(extruder[i].offsetMM[Y_AXIS] * Printer::axisStepsPerMM[Y_AXIS]))
                : RMath::min(offY, int32_t(extruder[i].offsetMM[Y_AXIS] * Printer::axisStepsPerMM[Y_AXIS]));
#endif // NUM_EXTRUDER>1

#if FEATURE_MILLING_MODE
        if (Printer::operatingMode == OPERATING_MODE_MILL)
            offY = 0; // in operating mode mill, there is no extruder offset
#endif                // FEATURE_MILLING_MODE

        Printer::queueRelativeStepsCoordinates(0, long(2 * abs(maxSoftEndstopSteps[Y_AXIS] + 2 * offY) * nHomeDir), 0, 0, homingFeedrate[Y_AXIS], false, true); //stop at endstop
        Printer::queueRelativeMMCoordinates(0, -1 * ENDSTOP_Y_BACK_MOVE * nHomeDir, 0, 0, homingFeedrate[Y_AXIS] / ENDSTOP_Y_RETEST_REDUCTION_FACTOR, false);
        Printer::queueRelativeMMCoordinates(0, 2 * ENDSTOP_Y_BACK_MOVE * nHomeDir, 0, 0, homingFeedrate[Y_AXIS] / ENDSTOP_Y_RETEST_REDUCTION_FACTOR, true, true); //stop at endstop, wait

        //currentYSteps ist die Schalter-Y-Koordinate, die die Y-Steps per Dir-Pin abzählt.
        Printer::currentYSteps = (nHomeDir == -1) ? 0 : maxSoftEndstopSteps[Y_AXIS];
#if NUM_EXTRUDER > 1
        if (offY) {
            //Wir halten z.B. von Links soviel Abstand wie der größte Extruder-Offset. Wir fahren aber nur so weit, dass das aktuelle Hotend dort steht.
            Printer::offsetRelativeStepsCoordinates(0, -1 * nHomeDir * (offY * axisMMPerSteps[Y_AXIS] - Extruder::current->offsetMM[Y_AXIS]) * Printer::axisStepsPerMM[Y_AXIS], 0, 0);
        }
#endif // NUM_EXTRUDER>1

        // Set axis value here.
        Printer::setYAxisSteps((nHomeDir == -1) ? 0 : maxSoftEndstopSteps[Y_AXIS]);

        // show that we are active
        previousMillisCmd = HAL::timeInMilliseconds(); //prevent inactive shutdown of steppers/temps
        setHomed(-1, true, -1);
        UI_STATUS_UPD("");
    }
} // homeYAxis

void Printer::homeZAxis() {
    char nHomeDir = Printer::anyHomeDir(Z_AXIS);

    if (nHomeDir) {
        UI_STATUS_UPD(UI_TEXT_HOME_Z);

        // if we have circuit-type Z endstops and we don't know at which endstop we currently are, first move down a bit
#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
        if (Printer::ZEndstopUnknown) {
            //RF1000 und Min-oder-Max gedrückt - nicht klar welcher. Man fährt immer nach unten! Der Schalter hält das aus.
            Printer::queueRelativeMMCoordinates(0, 0, ENDSTOP_Z_BACK_MOVE, 0, homingFeedrate[Z_AXIS] / ENDSTOP_Z_RETEST_REDUCTION_FACTOR, true); //drucker muss immer nach
        }
#endif

        //homing ausschalten und zCMP (...) auch.
        setHomed(-1, -1, false);

        InterruptProtectedBlock noInts;
#if FEATURE_FIND_Z_ORIGIN
        //das ist die Z-Origin-Höhe und ihre XY-Scan-Stelle.
        g_nZOriginPosition[X_AXIS] = 0;
        g_nZOriginPosition[Y_AXIS] = 0;
        g_nZOriginPosition[Z_AXIS] = 0;
        Printer::setZOriginSet(false); //removes flag wegen statusnachricht
#endif                                 // FEATURE_FIND_Z_ORIGIN

        Printer::resetDirectAxis(Z_AXIS);

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
        //das ist diese Scan-Positions-Z-Zusatzachse für MoveZ-Bewegungen.
        g_nZScanZPosition = 0;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
        noInts.unprotect();

        //1. Schnelles Fahren bis zum Schalterkontakt:
        //Ist der Schalter gedrückt, wird sofort geskipped.
        Printer::queueRelativeStepsCoordinates(0, 0, (maxSoftEndstopSteps[Z_AXIS] + 2 * int32_t(maxZOverrideSteps)) * nHomeDir, 0, homingFeedrate[Z_AXIS], true, true);

        //2. in jedem Fall Freifahren vom Schalterkontakt:
        //ENDSTOP_Z_BACK_MOVE größer als 32768 ist eigentlich nicht möglich, nicht sinnvoll und würde, da das überfahren bei 32microsteps von der z-matrix >-12,7mm abhängig ist verboten sein.
        //darum ist uint16_t in jedem fall ohne overflow.
        for (uint16_t step = 0; step < uint16_t(axisStepsPerMM[Z_AXIS] * ENDSTOP_Z_BACK_MOVE); step += uint16_t(0.1f * axisStepsPerMM[Z_AXIS])) {
            //faktor *2 und *5 : doppelt/5x so schnell beim Zurücksetzen als nachher beim langsamst hinfahren. Sonst dauert das ewig.
            if (Printer::isZMinEndstopHit()) {
                //schalter noch gedrückt, wir müssen weiter aus dem schalter rausfahren, aber keinesfalls mehr als ENDSTOP_Z_BACK_MOVE
                Printer::queueRelativeMMCoordinates(0, 0, 0.1f * (-1 * nHomeDir), 0, float(homingFeedrate[Z_AXIS] / ENDSTOP_Z_RETEST_REDUCTION_FACTOR * 5.0f), true);
            } else { //wir sind aus dem schalterbereich raus, müssten also nicht weiter zurücksetzen:
                     //1) egal ob der schalter zu anfang überfahren war oder nicht: etwas zurücksetzen, nachdem der schalter angefahren wurde.
                     //2) hier wird in jedem Fall etwas weiter weggefahren, sodass man wieder neu auf Z anfahren kann.
                Printer::queueRelativeMMCoordinates(0, 0, Z_ENDSTOP_MAX_HYSTERESIS * (-1 * nHomeDir), 0, float(homingFeedrate[Z_AXIS] / ENDSTOP_Z_RETEST_REDUCTION_FACTOR * 2.0f), true);
                break;
            }
        }

        //3. langsames Fahren bis zum Schalterkontakt:
        Printer::queueRelativeMMCoordinates(0, 0, (0.1f + ENDSTOP_Z_BACK_MOVE) * nHomeDir, 0, float(homingFeedrate[Z_AXIS] / ENDSTOP_Z_RETEST_REDUCTION_FACTOR), true, true);

#if FEATURE_MILLING_MODE
        //4. Wenn Millingmode dann nochmal freifahren und erst anschließend Koordinate nullen.
        if (Printer::operatingMode == OPERATING_MODE_MILL) {
            // when the milling mode is active and we are in operating mode "mill", we use the z max endstop and we free the z-max endstop after it has been hit
            Printer::queueRelativeMMCoordinates(0, 0, (LEAVE_Z_MAX_ENDSTOP_AFTER_HOME + Z_ENDSTOP_MAX_HYSTERESIS) * (-1 * nHomeDir), 0, float(homingFeedrate[Z_AXIS]), true);
        }
#endif // FEATURE_MILLING_MODE

        //currentZSteps ist die Schalter-Z-Koordinate, die die Z-Steps per Dir-Pin abzählt.
        // Wenn wir im Milingmode sind, dann homen wir und setzen den NUllpunkt direkt in die Mitte des theoretischen Fahrwegs.
        // Die absoluten Achsengrenzen werden gesetzt, vor der zweite Extruder sein offset bekommt.
        Printer::currentZSteps = (nHomeDir == -1) ? 0 : maxSoftEndstopSteps[Z_AXIS];

#if NUM_EXTRUDER == 2
#if FEATURE_MILLING_MODE
        if (Printer::operatingMode == OPERATING_MODE_PRINT)
#endif // FEATURE_MILLING_MODE
        {
            // Standardverhalten: Kein Offset für Z vorsehen, es gibt kein Homing-Offset.
            if (Extruder::current->id == extruder[1].id && Extruder::current->offsetMM[Z_AXIS] != 0.0f) {
                // Wenn beim Homing von Extruder Rechts ein Offset aktiv ist, dann hat der ein negatives Offset.
                // Wir verkleinern bei Z nicht künstlich unseren Bauraum um den Extruder-Abstand. Darum einfach per Offset hinfahren.
                Printer::offsetRelativeStepsCoordinates(0, 0, -1 * nHomeDir * (0 - Extruder::current->offsetMM[Z_AXIS]) * Printer::axisStepsPerMM[Z_AXIS], 0);
            }
        }
#endif // NUM_EXTRUDER>1

        //5. Setzen der aktuellen End-Position auf die Koordinate, zu der das Homing gehört.
        // Dann steht der Fräser auf Höhe Z-Length-Halbe. Danch wrid sowieso mit findZOrigin oder origin gearbeitet.
        Printer::setZAxisSteps((nHomeDir == -1) ? 0 : maxSoftEndstopSteps[Z_AXIS]);

        //6. Korrektur der Flags
        setHomed(-1, -1, true);
        // show that we are active
        previousMillisCmd = HAL::timeInMilliseconds(); //prevent inactive shutdown of steppers/temps
#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
        ZEndstopUnknown = false;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS
        UI_STATUS_UPD("");
    }
} // homeZAxis

void Printer::homeDigits() {
#if FEATURE_ZERO_DIGITS
    short nTempPressure = 0;
    if (Printer::g_pressure_offset_active) { //only adjust pressure if you do a full homing.
        Printer::g_pressure_offset = 0;      //prevent to messure already adjusted offset -> without = 0 this would only iterate some bad values.
        if (!readAveragePressure(&nTempPressure)) {
            if (-5000 < nTempPressure && nTempPressure < 5000) {
                Com::printFLN(PSTR("DigitOffset = "), nTempPressure);
                Printer::g_pressure_offset = nTempPressure;
            } else {
                //those high values shouldnt happen! fix your machine... DONT ZEROSCALE DIGITS
                Com::printFLN(PSTR("DigitOffset failed "), nTempPressure);
            }
        } else {
            Com::printFLN(PSTR("DigitOffset failed reading "));
            g_abortZScan = 0;
        }
    }
#endif // FEATURE_ZERO_DIGITS
}

void Printer::homeAxis(bool xaxis, bool yaxis, bool zaxis) // home non-delta printer
{
    g_uStartOfIdle = 0; //start of homing xyz

    //Bei beliebiger user interaktion oder Homing soll G1 etc. erlaubt werden. Dann ist der Drucker nicht abgestürzt, sondern bedient worden.
#if FEATURE_UNLOCK_MOVEMENT
    g_unlock_movement = 1;
#endif //FEATURE_UNLOCK_MOVEMENT

    char homingOrder;
#if FEATURE_MILLING_MODE
    if (operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE
        homingOrder = HOMING_ORDER_PRINT;
#if FEATURE_MILLING_MODE
    } else {
        homingOrder = HOMING_ORDER_MILL;
    }
#endif // FEATURE_MILLING_MODE

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
    //ich weiß nicht ob das überhaupt ein gutes verhalten ist. Es ist nicht gut, wenn die z-schraube zu weit reingeschraubt ist und daher z-homing über dem bett verboten sein sollte.
    //Printer::ZEndstopUnknown == 1 gibts nur, wenn der RF1000 mit Circuitschaltung mit einem der z-endstops gedrückt aufwacht, oder man in dieser position auf den endstop umstellt.
    //Ein Z-Homing mit EndstopUnknown == 1 fährt immer erst nach unten, also ist unsere Druckerdüse safe, wenn wir erst Z homen bevor X und Y gehomed werden darf.
    if (Printer::ZEndstopUnknown && homingOrder < 5 /*not z first -> do z first*/) {
        if (homingOrder == HOME_ORDER_XYZ
            || homingOrder == HOME_ORDER_XZY) {
            homingOrder = HOME_ORDER_ZXY;
        } else if (homingOrder == HOME_ORDER_YXZ
                   || homingOrder == HOME_ORDER_YZX) {
            homingOrder = HOME_ORDER_ZYX;
        }
    }
#endif //FEATURE_CONFIGURABLE_Z_ENDSTOPS

#if FEATURE_MILLING_MODE
    if (operatingMode == OPERATING_MODE_PRINT) { //wollte nicht milling mode, weil ich die mechanik da nicht kenne, dieses if ist unter umständen nutzlos.
#endif                                           // FEATURE_MILLING_MODE
        if ((!yaxis && zaxis) || (/* z vor y */ homingOrder == HOME_ORDER_XZY || homingOrder == HOME_ORDER_ZXY || homingOrder == HOME_ORDER_ZYX)) {
            // do not allow homing Z-Only within menu, when the Extruder is configured < 0 and over bed.
            if (!Printer::isZHomeSafe()) {
                homeYAxis(); //bevor die düse gegen das bett knallen könnte, weil positive z-matix oder tipdown-extruder sollte erst y genullt werden: kann das im printermode schädlich sein?
                //wenn Z genullt wird, sollte auch Y genullt werden dürfen.
            }
        }
#if FEATURE_MILLING_MODE
    }
#endif // FEATURE_MILLING_MODE

    switch (homingOrder) {
    case HOME_ORDER_XYZ: {
        if (xaxis)
            homeXAxis();
        if (yaxis)
            homeYAxis();
        if (zaxis)
            homeZAxis();
        break;
    }
    case HOME_ORDER_XZY: {
        if (xaxis)
            homeXAxis();
        if (zaxis)
            homeZAxis();
        if (yaxis)
            homeYAxis();
        break;
    }
    case HOME_ORDER_YXZ: {
        if (yaxis)
            homeYAxis();
        if (xaxis)
            homeXAxis();
        if (zaxis)
            homeZAxis();
        break;
    }
    case HOME_ORDER_YZX: {
        if (yaxis)
            homeYAxis();
        if (zaxis)
            homeZAxis();
        if (xaxis)
            homeXAxis();
        break;
    }
    case HOME_ORDER_ZXY: {
        if (zaxis)
            homeZAxis();
        if (xaxis)
            homeXAxis();
        if (yaxis)
            homeYAxis();
        break;
    }
    case HOME_ORDER_ZYX: {
        if (zaxis)
            homeZAxis();
        if (yaxis)
            homeYAxis();
        if (xaxis)
            homeXAxis();
        break;
    }
    }

    if (xaxis && yaxis && zaxis)
        Printer::homeDigits();

    g_uStartOfIdle = HAL::timeInMilliseconds(); //homing xyz just ended
    Commands::printCurrentPosition();
} // homeAxis

void Printer::stopPrint() //function for aborting USB and SD-Prints
{
    if (!Printer::isPrinting())
        return;
    g_uStartOfIdle = 0; //jetzt nicht mehr in showidle() gehen, das erledigt später g_uStopTime;

#if SDSUPPORT
    if (sd.sdmode) //prüfung auf !sdmode sollte hier eigenlicht nicht mehr nötig sein, aber ..
    {
        //block sdcard from reading more.
        Com::printFLN(PSTR("SD print stopped."));
        sd.sdmode = 0;
    } else
#endif //SDSUPPORT
    {
        //block some of the usb sources from sending more data
        Com::printFLN(PSTR("RequestStop:"));     //tell repetierserver to stop.
        Com::printFLN(PSTR("// action:cancel")); //tell octoprint to cancel print. > 1.3.7  https://github.com/foosel/OctoPrint/issues/2367#issuecomment-357554341
        Com::printFLN(PSTR("USB print stopped."));
    }

    InterruptProtectedBlock noInts;
    g_uBlockCommands = 1; //keine gcodes mehr ausführen bis beenden beendet.
    //now mark the print to get cleaned up after some time:
    g_uStopTime = HAL::timeInMilliseconds(); //starts output object in combination with g_uBlockCommands

    if (g_pauseStatus != PAUSE_STATUS_NONE) {
        // the printing is paused at the moment
        g_uPauseTime = 0;
        g_pauseStatus = PAUSE_STATUS_NONE;
        g_pauseMode = PAUSE_MODE_NONE;
    }
    Printer::setMenuMode(MENU_MODE_PAUSED, false); //egal ob nicht gesetzt.

    //erase the coordinates and kill the current taskplaner:
    PrintLine::resetPathPlanner();
    PrintLine::cur = NULL;

    Printer::setXAxisSteps(Printer::currentSteps[X_AXIS]);
    Printer::setYAxisSteps(Printer::currentSteps[Y_AXIS]);
    Printer::setZAxisSteps(Printer::currentSteps[Z_AXIS]);
    Printer::setEAxisSteps(0); //G92 E0:

    noInts.unprotect();

    Commands::printCurrentPosition();
    UI_STATUS_UPD(UI_TEXT_STOP_PRINT);
} // stopPrint

#define START_EXTRUDER_CONFIG(i) \
    Com::printF(Com::tConfig); \
    Com::printF(Com::tExtrDot, i + 1); \
    Com::print(':');
void Printer::showConfiguration() {
    Com::config(PSTR("Baudrate:"), baudrate);
#ifndef EXTERNALSERIAL
    Com::config(PSTR("InputBuffer:"), SERIAL_BUFFER_SIZE - 1);
#endif
    Com::config(PSTR("NumExtruder:"), NUM_EXTRUDER);
    Com::config(PSTR("MixingExtruder:"), 0);
    Com::config(PSTR("HeatedBed:"), HAVE_HEATED_BED);
    Com::config(PSTR("SDCard:"), SDSUPPORT);
    Com::config(PSTR("Fan:"), FAN_PIN > -1 && FEATURE_FAN_CONTROL);
    Com::config(PSTR("Fan2:0"));
    Com::config(PSTR("LCD:"), 0);
    Com::config(PSTR("SoftwarePowerSwitch:"), 0);
    Com::config(PSTR("XHomeDir:"), X_HOME_DIR);
    Com::config(PSTR("YHomeDir:"), Y_HOME_DIR);
    Com::config(PSTR("ZHomeDir:"), Z_HOME_DIR);

    Com::config(PSTR("XHomePos:"), (X_HOME_DIR > 0 ? Printer::axisLengthMM[X_AXIS] : 0), 2);
    Com::config(PSTR("YHomePos:"), (Y_HOME_DIR > 0 ? Printer::axisLengthMM[Y_AXIS] : 0), 2);
    Com::config(PSTR("ZHomePos:"), (Z_HOME_DIR > 0 ? Printer::axisLengthMM[Z_AXIS] : 0), 2);

    Com::config(PSTR("SupportG10G11:"), 0);
    Com::config(PSTR("SupportLocalFilamentchange:"), 0);
    Com::config(PSTR("CaseLights:"), 0);
    Com::config(PSTR("EEPROM:"), EEPROM_MODE != 0);
    Com::config(PSTR("PrintlineCache:"), MOVE_CACHE_SIZE);
    Com::config(PSTR("JerkXY:"), Printer::maxXYJerk);
    Com::config(PSTR("JerkZ:"), Printer::maxZJerk);
    Com::config(PSTR("KeepAliveInterval:"), KEEP_ALIVE_INTERVAL);

    Com::config(PSTR("XMin:"), 0);
    Com::config(PSTR("YMin:"), 0);
    Com::config(PSTR("ZMin:"), 0);
    Com::config(PSTR("XMax:"), Printer::axisLengthMM[X_AXIS]);
    Com::config(PSTR("YMax:"), Printer::axisLengthMM[Y_AXIS]);
    Com::config(PSTR("ZMax:"), Printer::axisLengthMM[Z_AXIS]);
    Com::config(PSTR("XSize:"), Printer::axisLengthMM[X_AXIS]);
    Com::config(PSTR("YSize:"), Printer::axisLengthMM[Y_AXIS]);
    Com::config(PSTR("ZSize:"), Printer::axisLengthMM[Z_AXIS]);
    Com::config(PSTR("XPrintAccel:"), Printer::maxAccelerationMMPerSquareSecond[X_AXIS]);
    Com::config(PSTR("YPrintAccel:"), Printer::maxAccelerationMMPerSquareSecond[Y_AXIS]);
    Com::config(PSTR("ZPrintAccel:"), Printer::maxAccelerationMMPerSquareSecond[Z_AXIS]);
    Com::config(PSTR("XTravelAccel:"), Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS]);
    Com::config(PSTR("YTravelAccel:"), Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS]);
    Com::config(PSTR("ZTravelAccel:"), Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS]);
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
        Com::printFLN(PSTR("MaxTemp:"), EXTRUDER_MAX_TEMP);
    }
}

extern void ui_check_keys(int& action);

bool Printer::checkAbortKeys(void) {
    int16_t activeKeys = 0;
    uid.ui_check_keys(activeKeys);
    if (activeKeys == UI_ACTION_OK || activeKeys == UI_ACTION_BACK) {
        return true;
    }
    return false;
}

bool Printer::checkPlayKey(void) {
    if (g_pauseMode == PAUSE_MODE_NONE) {
        int16_t activeKeys = 0;
        uid.ui_check_keys(activeKeys);
        if (activeKeys == UI_ACTION_RF_CONTINUE) {
            return true;
        }
    }
    return false;
}

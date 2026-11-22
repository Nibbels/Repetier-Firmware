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

#include <Arduino.h> // capital A so it is error prone on case-sensitive filesystems
#include "Repetier.h"
#include <Wire.h>

FSTRINGVALUE(ui_text_error, UI_TEXT_ERROR)
FSTRINGVALUE(ui_text_warning, UI_TEXT_WARNING)
FSTRINGVALUE(ui_text_information, UI_TEXT_INFORMATION)
FSTRINGVALUE(ui_text_heat_bed_scan, UI_TEXT_HEAT_BED_SCAN)
FSTRINGVALUE(ui_text_output_object, UI_TEXT_OUTPUT_OBJECT)
FSTRINGVALUE(ui_text_set_origin, UI_TEXT_SET_ORIGIN)
FSTRINGVALUE(ui_text_pause, UI_TEXT_PAUSE)
FSTRINGVALUE(ui_text_home, UI_TEXT_HOME)
FSTRINGVALUE(ui_text_z_compensation, UI_TEXT_Z_COMPENSATION)
FSTRINGVALUE(ui_text_x_axis, UI_TEXT_X_AXIS)
FSTRINGVALUE(ui_text_y_axis, UI_TEXT_Y_AXIS)
FSTRINGVALUE(ui_text_z_axis, UI_TEXT_Z_AXIS)
#if FEATURE_ALIGN_EXTRUDERS
FSTRINGVALUE(ui_text_align_extruders, UI_TEXT_ALIGN_EXTRUDERS);
#endif // FEATURE_ALIGN_EXTRUDERS
FSTRINGVALUE(ui_text_extruder, UI_TEXT_EXTRUDER)
FSTRINGVALUE(ui_text_autodetect_pid, UI_TEXT_AUTODETECT_PID)
FSTRINGVALUE(ui_text_temperature_manager, UI_TEXT_TEMPERATURE_MANAGER)
FSTRINGVALUE(ui_text_home_unknown, UI_TEXT_HOME_UNKNOWN)
FSTRINGVALUE(ui_text_saving_needless, UI_TEXT_SAVING_NEEDLESS)
FSTRINGVALUE(ui_text_operation_denied, UI_TEXT_OPERATION_DENIED)
FSTRINGVALUE(ui_text_emergency_pause, UI_TEXT_EMERGENCY_PAUSE)
FSTRINGVALUE(ui_text_emergency_stop, UI_TEXT_EMERGENCY_STOP)
FSTRINGVALUE(ui_text_invalid_matrix, UI_TEXT_INVALID_MATRIX)
FSTRINGVALUE(ui_text_min_reached, UI_TEXT_MIN_REACHED)
FSTRINGVALUE(ui_text_min_reached_unhomed, UI_TEXT_MIN_REACHED_UNHOMED)
FSTRINGVALUE(ui_text_max_reached, UI_TEXT_MAX_REACHED)
FSTRINGVALUE(ui_text_temperature_wrong, UI_TEXT_TEMPERATURE_WRONG)
FSTRINGVALUE(ui_text_timeout, UI_TEXT_TIMEOUT)
FSTRINGVALUE(ui_text_sensor_error, UI_TEXT_SENSOR_ERROR)
FSTRINGVALUE(ui_text_heater_error, UI_TEXT_HEATER_ERROR)
FSTRINGVALUE(ui_text_heat_bed_zoffset_search_status, UI_TEXT_HEAT_BED_ZOFFSET_SEARCH_STATUS)
FSTRINGVALUE(ui_text_heat_bed_zoffset_fix_z1, UI_TEXT_HEAT_BED_ZOFFSET_FIX_Z1)
FSTRINGVALUE(ui_text_question, UI_TEXT_UNKNOWN)
FSTRINGVALUE(ui_text_heat_bed_zoffset_fix_z2, UI_TEXT_HEAT_BED_ZOFFSET_FIX_Z2)
FSTRINGVALUE(ui_text_statusmsg, UI_TEXT_STATUSMSG)

FSTRINGVALUE(ui_text_saving_success, UI_TEXT_SAVING_SUCCESS)

millis_t g_uStartOfIdle = 0;

long g_offsetZCompensationSteps = 0;
short g_ZCompensationMax = 0;
long g_minZCompensationSteps = long(HEAT_BED_Z_COMPENSATION_MIN_MM * ZAXIS_STEPS_PER_MM); //pre init
long g_maxZCompensationSteps = long(HEAT_BED_Z_COMPENSATION_MAX_MM * ZAXIS_STEPS_PER_MM); //pre init
long g_diffZCompensationSteps = g_maxZCompensationSteps - g_minZCompensationSteps;
volatile unsigned char g_nHeatBedScanStatus = 0;
char g_nActiveHeatBed = 1;

//ZOS
//Nibbels: Das ist wie die g_nHeatBedScanStatus, die Schwestervariable, für den ZOS-Scan -> Vorsicht, wenn man sowas einführt müssen die überall vermerkt werden, weil sonst z.B. der G-Code weiter vorgeführt wird.
// g_ZMatrixChangedInRam soll 1 werden, wenn ZOS, Offsetänderung der Matrix etc. Sonst wäre Sichern der Matrix unnötig.
volatile unsigned char g_ZMatrixChangedInRam = 0;
volatile unsigned char g_nZOSScanStatus = 0;
//Nibbels:
unsigned char g_ZOSTestPoint[2] = { SEARCH_HEAT_BED_OFFSET_SCAN_POSITION_INDEX_X, SEARCH_HEAT_BED_OFFSET_SCAN_POSITION_INDEX_Y };
float g_ZOSlearningRate = 1.0f;
float g_ZOSlearningGradient = 0.0f;
long g_min_nZScanZPosition = 0;
unsigned char g_ZOS_Auto_Matrix_Leveling_State = 0; //if 1 do multiple scans to correct matrix. while correcting this is rising for switch cases see state 50+

float g_scanStartZLiftMM = HEAT_BED_SCAN_Z_START_MM;

unsigned long g_lastScanTime = 0;
unsigned long g_scanStartTime = 0;
char g_scanRetries = 0;
char g_retryZScan = 0;
unsigned char g_retryStatus = 0;

char g_nHeatBedScanMode = 0;

char g_abortZScan = 0;
short g_ZCompensationMatrix[COMPENSATION_MATRIX_MAX_X][COMPENSATION_MATRIX_MAX_Y];
unsigned char g_uZMatrixMax[2] = { 0, 0 };
long g_nZScanZPosition = 0;
long g_nLastZScanZPosition = 0;

short g_nMaxPressureContact;
short g_nMaxPressureRetry;
short g_nMaxPressureIdle;
short g_nMinPressureContact;
short g_nMinPressureRetry;
short g_nMinPressureIdle;
short g_nFirstIdlePressure;
short g_nCurrentIdlePressure;

// configurable scan parameters - the proper default values are set by restoreDefaultScanParameters()
long g_nScanXStartSteps = 0;
long g_nScanXStepSizeMM = 0;
long g_nScanXStepSizeSteps = 0;
long g_nScanXEndSteps = 0;
long g_nScanXMaxPositionSteps = 0;
long g_nScanYStartSteps = 0;
long g_nScanYStepSizeMM = 0;
long g_nScanYStepSizeSteps = 0;
long g_nScanYEndSteps = 0;
long g_nScanYMaxPositionSteps = 0;
short g_nScanHeatBedUpFastSteps = 0;
short g_nScanHeatBedUpSlowSteps = 0;
short g_nScanHeatBedDownFastSteps = 0;
short g_nScanHeatBedDownSlowSteps = 0;
short g_nScanZMaxCompensationSteps = 0;
unsigned short g_nScanFastStepDelay = 0;
unsigned short g_nScanSlowStepDelay = 0;
unsigned short g_nScanIdleDelay = 0;
unsigned short g_nScanContactPressureDelta = 0;
unsigned short g_nScanRetryPressureDelta = 0;
unsigned short g_nScanIdlePressureDelta = 0;
short g_nScanIdlePressureMin = 0;
short g_nScanIdlePressureMax = 0;
char g_nScanPressureReads = 0;
unsigned short g_nScanPressureReadDelay = 0;
short g_nScanPressureTolerance = 0;

long g_staticZSteps = 0;
// Enabled only while firmware-generated M3912 moves are planned/executed.
volatile uint8_t g_startLineActive = 0;
char g_debugLevel = 0;
char g_debugLog = 0;
millis_t g_uStopTime = 0;
volatile millis_t g_uBlockCommands = 0;

// other configurable parameters
unsigned short g_nManualSteps[4] = {
    lroundf(XAXIS_STEPS_PER_MM * DEFAULT_MANUAL_MM_X),
    lroundf(YAXIS_STEPS_PER_MM* DEFAULT_MANUAL_MM_Y),
    lroundf(ZAXIS_STEPS_PER_MM* DEFAULT_MANUAL_MM_Z),
    lroundf(EXT0_STEPS_PER_MM* DEFAULT_MANUAL_MM_E)
}; //pre init

volatile long g_nPauseSteps[4] = { long(XAXIS_STEPS_PER_MM * DEFAULT_PAUSE_MM_X_PRINT),
                                   long(XAXIS_STEPS_PER_MM* DEFAULT_PAUSE_MM_Y_PRINT),
                                   long(ZAXIS_STEPS_PER_MM* DEFAULT_PAUSE_MM_Z_PRINT),
                                   long(EXT0_STEPS_PER_MM* DEFAULT_PAUSE_MM_E) }; //pre init
static long pauseDirectPositionSteps[3];
volatile long g_nContinueExtruderSteps = 0;
volatile char g_pauseStatus = PAUSE_STATUS_NONE;
volatile char g_pauseMode = PAUSE_MODE_NONE;
volatile millis_t g_uPauseTime = 0;

long g_nEmergencyPauseDigitsMin = EMERGENCY_PAUSE_DIGITS_MIN; //short reicht eigentlich
long g_nEmergencyPauseDigitsMax = EMERGENCY_PAUSE_DIGITS_MAX; //short reicht eigentlich

short g_nEmergencyStopZAndEMin = EMERGENCY_STOP_DIGITS_MIN;
short g_nEmergencyStopZAndEMax = EMERGENCY_STOP_DIGITS_MAX;
bool g_nEmergencyESkip = false;

// SenseOffset was proposed by Wessix and implemented by Nibbels.
// It adds clearance when DMS force is high on the first layer.
short g_nSensiblePressureDigits = 0; //init auf zahl bringt stand 1.37r2 nur was wenn zcompensation an. darum lesen aus eeprom oder setzen auf wert nur durch Mcode
short g_nSensiblePressureOffsetMax = SENSIBLE_PRESSURE_MAX_OFFSET;
short g_nSensiblePressureOffset = 0;
char g_nSensiblePressure1stMarke = 0; //sagt, ob regelung aktiv oder inaktiv, wegen Z-Limits

short g_nLastDigits = 0;
float g_nDigitZCompensationDigits = 0.0f;
bool g_nDigitZCompensationDigits_active = true;
int8_t g_nDigitFlowCompensation_intense = 0;                                        // +- % Standard 0 heißt flowmulti wird zu 1.0f
int8_t g_nDigitFlowCompensation_speed_intense = 0;                                  // +- % Standard 0 heißt feedmulti wird zu 1.0f
short g_nDigitFlowCompensation_Fmin = short(abs(EMERGENCY_PAUSE_DIGITS_MAX) * 0.7); //mögliche Standardwerte
short g_nDigitFlowCompensation_Fmax = short(abs(EMERGENCY_PAUSE_DIGITS_MAX));       //mögliche Standardwerte -> z.b. gut wenn das die pause-digits sind.

#if FEATURE_ALIGN_EXTRUDERS
volatile unsigned char g_nAlignExtrudersStatus = 0;
#endif // FEATURE_ALIGN_EXTRUDERS

#ifdef RGB_LIGHT_R_PIN
unsigned char g_uRGBHeatingR = RGB_HEATING_R;
unsigned char g_uRGBHeatingG = RGB_HEATING_G;
unsigned char g_uRGBHeatingB = RGB_HEATING_B;
unsigned char g_uRGBPrintingR = RGB_PRINTING_R;
unsigned char g_uRGBPrintingG = RGB_PRINTING_G;
unsigned char g_uRGBPrintingB = RGB_PRINTING_B;
unsigned char g_uRGBCoolingR = RGB_COOLING_R;
unsigned char g_uRGBCoolingG = RGB_COOLING_G;
unsigned char g_uRGBCoolingB = RGB_COOLING_B;
unsigned char g_uRGBIdleR = RGB_IDLE_R;
unsigned char g_uRGBIdleG = RGB_IDLE_G;
unsigned char g_uRGBIdleB = RGB_IDLE_B;
unsigned char g_uRGBManualR = RGB_MANUAL_R;
unsigned char g_uRGBManualG = RGB_MANUAL_G;
unsigned char g_uRGBManualB = RGB_MANUAL_B;
volatile unsigned char g_uRGBCurrentR = 0;
volatile unsigned char g_uRGBCurrentG = 0;
volatile unsigned char g_uRGBCurrentB = 0;
unsigned char g_uRGBTargetR = 0;
unsigned char g_uRGBTargetG = 0;
unsigned char g_uRGBTargetB = 0;
#endif // RGB_LIGHT_R_PIN

static void setupForPrinting();

void initRF(void) {
    // initialize the strain gauge
    initStrainGauge();

    Printer::disableCMPnow(true);
    setupForPrinting();
} // initRF

void initStrainGauge(void) {
    // configure DMS #1 (0x8C = single mode, 16 bits, gain = *1)
    Wire.beginTransmission(I2C_ADDRESS_STRAIN_GAUGE);
    Wire.write(0x8C);
    Wire.endTransmission();
    return;

} // initStrainGauge

short readStrainGauge(unsigned char uAddress) //readStrainGauge dauert etwas unter einer Millisekunde!
{
    unsigned char Register;
    short Result;

    Wire.beginTransmission(uAddress);
    Wire.requestFrom((uint8_t)uAddress, (uint8_t)3);

    Result = Wire.read();
    Result = Result << 8;
    Result += Wire.read();

    Register = Wire.read();
    (void)Register; //Nibbels: Tut so als würde die variable benutzt werden. Macht aber nix.
    Wire.endTransmission();

    if (Printer::g_pressure_offset_active && -27768 < Result && Result < 27767) {
        Result -= Printer::g_pressure_offset; //no overflow possible: pressure_offset ist 5000 max.
    }

    /* brief: This is for correcting sinking hotends at high digit values because of DMS-Sensor by Nibbels  */
    static long nSensibleCompensationSum = 0;
    static char nSensibleCompensationChecks = 0;
    if (Printer::doHeatBedZCompensation) {
        //wenn ein retract stattfindet und das nicht echtzeit-schnell funktioniert, könnte es leichte probleme geben, aber prinzipiell wäre dann der Einfluss nicht so schädlich, wie ständig die digitabsenkung zu ignorieren.
        //das folgende sammelt immer 4 messwerte, schreibt den mittelwert raus und minimiert sammelwert und zähler auf 75%, dann den vierten wert neu in den topf..
        nSensibleCompensationSum += Result;
        nSensibleCompensationChecks += 1;
        if (nSensibleCompensationChecks == 4) {
            InterruptProtectedBlock noInts;
            g_nDigitZCompensationDigits = (float)(nSensibleCompensationSum / 4); //*0.25, nachkommsstellen sind egal.
            noInts.unprotect();
            nSensibleCompensationSum = (long)g_nDigitZCompensationDigits * 3; //(nSensibleCompensationSum >> 1) + (nSensibleCompensationSum >> 2);--> sign-extension?? //nSensibleCompensationSum*0.75
            nSensibleCompensationChecks -= 1;                                 //*=0.75 bei 4 ist 3
            // M3912 feeds high DMS force into a progressive speed/E brake,
            // irrespective of the previous print's first-layer boundary.
            bool digitFlowAllowed = Printer::currentSteps[Z_AXIS] > g_minZCompensationSteps;
            digitFlowAllowed = digitFlowAllowed || g_startLineActive;
            if (g_nDigitFlowCompensation_intense != 0) {
                short active_summed_digits = abs(static_cast<short>(g_nDigitZCompensationDigits));
                /*
                unter unterem digits limit: flow = 1.000
                zwischen beiden limits    : flow = 1.000 + anteil an maximaler auslenkung
                über oberem   digits limit: flow = 1.000 + maximale auslenkung plus oder minus
                */
                if (active_summed_digits <= g_nDigitFlowCompensation_Fmin) {
                    Printer::dynamicExtrusionFactor = 1.0f;
                } else if (active_summed_digits >= g_nDigitFlowCompensation_Fmax) {
                    if (digitFlowAllowed)
                        Printer::dynamicExtrusionFactor = 1.0f + 0.01f * g_nDigitFlowCompensation_intense;
                    else
                        Printer::dynamicExtrusionFactor = 1.0f;
                } else {
                    if (digitFlowAllowed)
                        Printer::dynamicExtrusionFactor = 1.0f + 0.01f * g_nDigitFlowCompensation_intense * (active_summed_digits - g_nDigitFlowCompensation_Fmin) / (g_nDigitFlowCompensation_Fmax - g_nDigitFlowCompensation_Fmin);
                    else
                        Printer::dynamicExtrusionFactor = 1.0f;
                }
            } else {
                Printer::dynamicExtrusionFactor = 1.0f;
            }
            if (g_nDigitFlowCompensation_speed_intense != 0) {
                short active_summed_digits = abs(static_cast<short>(g_nDigitZCompensationDigits));
                /*
                unter unterem digits limit: feed = 1.000
                zwischen beiden limits    : feed = 1.000 + anteil an maximaler auslenkung
                über oberem   digits limit: feed = 1.000 + maximale auslenkung plus oder minus
                */
                float goal = 1.0f;
                if (active_summed_digits <= g_nDigitFlowCompensation_Fmin) {
                    goal = 1.0f;
                } else if (active_summed_digits >= g_nDigitFlowCompensation_Fmax) {
                    if (digitFlowAllowed)
                        goal = 1.0f + 0.01f * g_nDigitFlowCompensation_speed_intense;
                } else {
                    if (digitFlowAllowed)
                        goal = 1.0f + 0.01f * g_nDigitFlowCompensation_speed_intense * (active_summed_digits - g_nDigitFlowCompensation_Fmin) / (g_nDigitFlowCompensation_Fmax - g_nDigitFlowCompensation_Fmin);
                }
                Printer::dynamicFeedrateFactor += (goal == Printer::dynamicFeedrateFactor ? 0.0f : (goal > Printer::dynamicFeedrateFactor ? 0.010f : -0.010f));
            } else {
                Printer::dynamicFeedrateFactor = 1.0f;
            }

        } else {
            Printer::dynamicExtrusionFactor = 1.0f;
            Printer::dynamicFeedrateFactor = 1.0f;
            InterruptProtectedBlock noInts;
            g_nDigitZCompensationDigits = (float)Result; //startwert / failwert
            noInts.unprotect();
        }
    }

    if (Printer::dynamicFeedrateFactor == 1.0f || !Printer::doHeatBedZCompensation) {
        Printer::interval_mod = 0; //0 = off / 1024 = off but 0 saves calc in interrupt.
    } else {
        float speed_feed_modifier = 1024.0f / Printer::dynamicFeedrateFactor; //1024 = 100% -> 234% speed, heißt interval kürzer. 56% speed heißt interval länger.
        Printer::interval_mod = (unsigned short)speed_feed_modifier;
    }
    g_nLastDigits = Result;
    return Result;
} // readStrainGauge

void adjustPressureLimits(short IdlePressure) {
    g_nMinPressureContact = IdlePressure - g_nScanContactPressureDelta;
    g_nMaxPressureContact = IdlePressure + g_nScanContactPressureDelta;
    g_nMinPressureRetry = IdlePressure - g_nScanRetryPressureDelta;
    g_nMaxPressureRetry = IdlePressure + g_nScanRetryPressureDelta;
    g_nMinPressureIdle = IdlePressure - g_nScanIdlePressureDelta;
    g_nMaxPressureIdle = IdlePressure + g_nScanIdlePressureDelta;
}

void startHeatBedScan(void) {
#if FEATURE_ALIGN_EXTRUDERS
    if (g_nAlignExtrudersStatus)
        return;
#endif //FEATURE_ALIGN_EXTRUDERS
    //if( g_nHeatBedScanStatus ) return;
    if (g_nZOSScanStatus)
        return;
    if (g_nHeatBedScanStatus) {
        // abort the heat bed scan
        if (Printer::debugInfo()) {
            Com::printFLN(PSTR("HBS: cancelled"));
        }
        g_abortZScan = SCAN_ABORT_REASON_CANCELED;
    } else {
        if (Printer::isPrinting()) {
            // there is some printing in progress at the moment - do not start the heat bed scan in this case
            if (Printer::debugErrors()) {
                Com::printFLN(Com::tPrintingIsInProcessError);
            }

            showError((void*)ui_text_heat_bed_scan, (void*)ui_text_operation_denied);
        } else {
            // start the heat bed scan
            g_nHeatBedScanStatus = 1;
            g_abortZScan = 0; //dont kill job on start
            g_retryZScan = 0;
        }
    }
    return;
} // startHeatBedScan

void scanHeatBed(void) {
#if FEATURE_ALIGN_EXTRUDERS
    if (g_nAlignExtrudersStatus)
        return;
#endif //FEATURE_ALIGN_EXTRUDERS
    //if( g_nHeatBedScanStatus ) return;
    if (g_nZOSScanStatus)
        return;

    static unsigned char nIndexX;
    static unsigned char nIndexY;
    static char nIndexYDirection;
    static long nX;
    static long nY;
    static long nYDirection;
#if DEBUG_HEAT_BED_SCAN
    static short nContactPressure;
#endif // DEBUG_HEAT_BED_SCAN
    //unsigned char         nLastHeatBedScanStatus = g_nHeatBedScanStatus;
    short nTempPressure;
    long nTempPosition;

    // directions:
    // +x = to the right
    // -x = to the left
    // +y = heat bed moves to the front
    // -y = heat bed moves to the back
    // +z = heat bed moves down
    // -z = heat bed moves up

    if (g_abortZScan) {
        showAbortScanReason((void*)ui_text_heat_bed_scan, g_abortZScan);
        // the scan has been aborted
        g_abortZScan = 0;

        // avoid to crash the extruder against the heat bed during the following homing
        moveZ(int(Printer::axisStepsPerMM[Z_AXIS] * 5));

        // start at the home position
        Printer::homeAxis(true, true, true);

        // turn off the engines
        Printer::disableAllSteppersNow();

        // disable all heaters
        Extruder::setHeatedBedTemperature(0);
        Extruder::setTemperatureForAllExtruders(0);

        if (Printer::debugInfo()) {
            Com::printF(Com::tscanHeatBed);
            Com::printFLN(PSTR("aborted"));
        }

        g_uStartOfIdle = HAL::timeInMilliseconds() + 30000; //abort scanHeatBed

        // restore the compensation values from the EEPROM
        if (loadCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed))) {
            // there is no valid compensation matrix available
            initCompensationMatrix();
        }

        g_nHeatBedScanStatus = 0;
        g_nLastZScanZPosition = 0;
        g_retryZScan = 0;
        g_retryStatus = 0;

        return;
    }

    if (g_nHeatBedScanStatus) {
        // show that we are active
        previousMillisCmd = HAL::timeInMilliseconds(); //prevent inactive shutdown of steppers/temps
        GCode::keepAlive(Calibrating);

        if (g_nHeatBedScanStatus != 15 && g_nHeatBedScanStatus != 20 && g_nHeatBedScanStatus != 22 && g_nHeatBedScanStatus != 123 && g_nHeatBedScanStatus != 125 && g_nHeatBedScanStatus != 136 && g_nHeatBedScanStatus != 137) {
            // there are a few cases where we do not want to change the current status text
            UI_STATUS(UI_TEXT_HEAT_BED_SCAN);
        }

        if (g_retryZScan) {
            // we have to retry to scan the current position
            g_nHeatBedScanStatus = g_retryStatus;
            g_retryZScan = 0;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("retry -> "), g_retryStatus);
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
        }

        switch (g_nHeatBedScanStatus) {
        case 1: {
            g_uStartOfIdle = 0; // scanHeatBed
            g_scanStartTime = HAL::timeInMilliseconds();
            g_abortZScan = 0;
#if DEBUG_HEAT_BED_SCAN
            nContactPressure = 0;
#endif // DEBUG_HEAT_BED_SCAN
            g_retryStatus = 0;
            g_nLastZScanZPosition = 0; //sodass dass bei mehreren scans nicht die letzte position als abstands limit feststeht.

            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("scan started"));
            }

            // clear all fields of the heat bed compensation matrix
            initCompensationMatrix();

            g_uZMatrixMax[X_AXIS] = g_uZMatrixMax[Y_AXIS] = 0;

            // output the currently used scan parameters
            outputScanParameters();

            g_nHeatBedScanStatus = 10;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("1->10"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 10: {
            // the scan is performed with the left extruder
            Extruder::selectExtruderById(0);

            if (g_nHeatBedScanMode == HEAT_BED_SCAN_MODE_PLA) {
                Extruder::setHeatedBedTemperature(PRECISE_HEAT_BED_SCAN_BED_TEMP_PLA);
                Extruder::setTemperatureForAllExtruders(PRECISE_HEAT_BED_SCAN_EXTRUDER_TEMP_SCAN);
            } else if (g_nHeatBedScanMode == HEAT_BED_SCAN_MODE_ABS) {
                Extruder::setHeatedBedTemperature(PRECISE_HEAT_BED_SCAN_BED_TEMP_ABS);
                Extruder::setTemperatureForAllExtruders(PRECISE_HEAT_BED_SCAN_EXTRUDER_TEMP_SCAN);
            }

            g_nHeatBedScanStatus = 15;
            g_lastScanTime = HAL::timeInMilliseconds();

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printFLN(PSTR("scanHeatBed(): 10->15"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 15: {
            if ((HAL::timeInMilliseconds() - g_lastScanTime) < HEAT_BED_SCAN_DELAY) {
                // do not check too often
                break;
            }

            if (testHeatBedTemperature()) {
                // we did not reach the proper temperature
                g_lastScanTime = HAL::timeInMilliseconds();
                break;
            }
            g_nHeatBedScanStatus = 20;
            g_lastScanTime = HAL::timeInMilliseconds();

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("15->20"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 20: {
            if ((HAL::timeInMilliseconds() - g_lastScanTime) < HEAT_BED_SCAN_DELAY) {
                // do not check too often
                break;
            }

            if (testExtruderTemperature()) {
                // we did not reach the proper temperature
                g_lastScanTime = HAL::timeInMilliseconds();
                break;
            }
            g_nHeatBedScanStatus = 22;
            g_lastScanTime = HAL::timeInMilliseconds();

            if (g_nHeatBedScanMode) {
                if (Printer::debugInfo()) {
                    Com::printF(Com::tscanHeatBed);
                    Com::printFLN(PSTR("warmup delay [s] = "), PRECISE_HEAT_BED_SCAN_WARMUP_DELAY);
                }
            }

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("20->22"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 22: {
            if (g_nHeatBedScanMode) {
                // wait some time so that the desired target temperature is reached in all parts of our components
                unsigned long uRemainingSeconds;

                uRemainingSeconds = (HAL::timeInMilliseconds() - g_lastScanTime) / 1000;
                if (uRemainingSeconds < PRECISE_HEAT_BED_SCAN_WARMUP_DELAY) {
                    char szStatus[32];
                    strcpy(szStatus, UI_TEXT_HEATING);
                    addLong(szStatus, PRECISE_HEAT_BED_SCAN_WARMUP_DELAY - uRemainingSeconds, 3);
                    strcat(szStatus, "[s]");
                    UI_STATUS_UPD_RAM(szStatus);
                    break;
                }
            }

            // start at the home position
            Printer::homeAxis(true, true, true);
            Commands::waitUntilEndOfAllMoves(); //scanHeatBed

            // move a bit away from the heat bed in order to achieve better measurements in case of hardware configurations where the extruder is very close to the heat bed after the z-homing
            moveZ(long(Printer::axisStepsPerMM[Z_AXIS] * g_scanStartZLiftMM));

            g_nHeatBedScanStatus = 25;
            g_lastScanTime = HAL::timeInMilliseconds();

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("22->25"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 25: {
            // move to the first position
            Printer::queueRelativeStepsCoordinates(g_nScanXStartSteps, 0, 0, 0, Printer::homingFeedrate[X_AXIS], false, true);
            Printer::queueRelativeStepsCoordinates(0, g_nScanYStartSteps, 0, 0, Printer::homingFeedrate[Y_AXIS], true, true);

            g_nHeatBedScanStatus = 30;
            g_lastScanTime = HAL::timeInMilliseconds();

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("25->30"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 30: {
            if ((HAL::timeInMilliseconds() - g_lastScanTime) < HEAT_BED_SCAN_DELAY) {
                // do not check too early
                break;
            }

            if (readIdlePressure(&g_nFirstIdlePressure)) {
                g_abortZScan = SCAN_ABORT_REASON_AVERAGE_PRESSURE;
                // we were unable to determine the idle pressure
                break;
            }

            g_nHeatBedScanStatus = 35;
            g_lastScanTime = HAL::timeInMilliseconds();

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("30->35"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 35: {
            nX = g_nScanXStartSteps;
            nY = g_nScanYStartSteps;
            nYDirection = g_nScanYStepSizeSteps; // we start to move the heat bed from the back to the front
            nIndexYDirection = 1;
            nIndexX = 2;
            nIndexY = 2;

            adjustPressureLimits(g_nFirstIdlePressure);

            // store also the version of this heat bed compensation matrix
            g_ZCompensationMatrix[0][0] = EEPROM_FORMAT;

            g_nHeatBedScanStatus = 40;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("35->40"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 39: {
            nTempPosition = nX + g_nScanXStepSizeSteps;
            if (nTempPosition > g_nScanXMaxPositionSteps) {
                // we end up here when the scan is complete
                g_nHeatBedScanStatus = 60;

#if DEBUG_HEAT_BED_SCAN == 2
                if (Printer::debugInfo()) {
                    Com::printF(Com::tscanHeatBed);
                    Com::printFLN(PSTR("39->60"));
                }
#endif // DEBUG_HEAT_BED_SCAN == 2
                break;
            }

            // move to the next x-position
            Printer::queueRelativeStepsCoordinates(g_nScanXStepSizeSteps, 0, 0, 0, Printer::homingFeedrate[X_AXIS], true, true);
            nX += g_nScanXStepSizeSteps;
            nIndexX++;

            if (nIndexX > COMPENSATION_MATRIX_MAX_X) {
#if DEBUG_HEAT_BED_SCAN == 2
                if (Printer::debugErrors()) {
                    Com::printF(Com::tscanHeatBed);
                    Com::printFLN(PSTR("x-dimension of the z matrix became too big: "), nIndexX);
                }
#endif // DEBUG_HEAT_BED_SCAN == 2
                g_abortZScan = SCAN_ABORT_REASON_MATRIX_DIMENSION;
                break;
            }

            if (nYDirection > 0) {
                // we were moving from the front to the back during this column, so we have to move from the back to the front during the next column
                nYDirection = -g_nScanYStepSizeSteps; // we start to move the heat bed from the back to the front
                nIndexYDirection = -1;
            } else {
                // we were moving from the back to the front during this column, so we have to move from the front to the back during the next column
                nYDirection = g_nScanYStepSizeSteps; // we start to move the heat bed from the back to the front
                nIndexYDirection = 1;
            }

            g_nHeatBedScanStatus = 40;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("39->40"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 40: {
            // safety checks
            if (nX <= g_nScanXMaxPositionSteps) {
                // remember also the exact x-position of this row/column
                g_ZCompensationMatrix[nIndexX][0] = (short)((float)nX / Printer::axisStepsPerMM[X_AXIS] + 0.5); // convert to mm

                g_nHeatBedScanStatus = 49;
                g_lastScanTime = HAL::timeInMilliseconds();

#if DEBUG_HEAT_BED_SCAN == 2
                if (Printer::debugInfo()) {
                    Com::printF(Com::tscanHeatBed);
                    Com::printFLN(PSTR("40->49"));
                }
#endif // DEBUG_HEAT_BED_SCAN == 2
                break;
            }

            // we end up here when the scan is complete
            g_nHeatBedScanStatus = 60;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("40->60"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        //############################################################### ERROR HANDLING Case 45 105 139 + 132
        case 45: {
            moveZ(long(Printer::axisStepsPerMM[Z_AXIS] * g_scanStartZLiftMM)); //spacing for bed
            Printer::homeAxis(false, true, false);                             //Home Y

            g_scanRetries--;
            g_nHeatBedScanStatus = 46;
            g_lastScanTime = HAL::timeInMilliseconds();

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo())
                Com::printFLN(PSTR("45"));
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 46: {
            // home the z-axis in order to find the starting point again
            Printer::homeAxis(false, false, true);                             //Neben Bett: Home Z
            moveZ(long(Printer::axisStepsPerMM[Z_AXIS] * g_scanStartZLiftMM)); //spacing for bed
            g_nLastZScanZPosition = 0;                                         //Reset last scan position to avoid abort due to this limitter

            g_nHeatBedScanStatus = 47;
            g_lastScanTime = HAL::timeInMilliseconds();

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo())
                Com::printFLN(PSTR("46"));
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 47: {
            Printer::queueRelativeStepsCoordinates(0, nY, 0, 0, Printer::homingFeedrate[Y_AXIS], true, true);

            g_nHeatBedScanStatus = 48;
            g_lastScanTime = HAL::timeInMilliseconds();

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo())
                Com::printFLN(PSTR("47"));
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 48: // Remeasure inconsistent pressure values to have new clean fence levels
        {
            if ((HAL::timeInMilliseconds() - g_lastScanTime) < HEAT_BED_SCAN_DELAY) {
                // do not check too early
                break;
            }

            if (readIdlePressure(&g_nFirstIdlePressure)) {
                g_abortZScan = SCAN_ABORT_REASON_AVERAGE_PRESSURE;
                // we were unable to determine the idle pressure
                break;
            } else {
                adjustPressureLimits(g_nFirstIdlePressure);
            }
            g_nHeatBedScanStatus = 50;
            g_lastScanTime = HAL::timeInMilliseconds();

            break;
        }
        //############################################################### /ERROR HANDLING
        case 49: {
            g_scanRetries = HEAT_BED_SCAN_RETRIES;
            g_retryStatus = 45;
            g_nHeatBedScanStatus = 50;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("49->50"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 50: {
            if ((HAL::timeInMilliseconds() - g_lastScanTime) < g_nScanIdleDelay) {
                // do not check too early
                break;
            }

            // scan this point
            if (testIdlePressure()) {
                // the current idle pressure is not plausible
                if (g_scanRetries) {
                    g_retryZScan = 1;
                } else {
                    g_abortZScan = SCAN_ABORT_REASON_IDLE_PRESSURE;
                }
                break;
            }

            // we should consider that the idle presse can change slightly
            adjustPressureLimits(g_nCurrentIdlePressure);

            g_nHeatBedScanStatus = 51;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("50->51"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 51: {
            // move fast to the surface
            moveZMinusUpFast();
            g_nHeatBedScanStatus = 52;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("51->52"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 52: {
            // move a little bit away from the surface
            moveZPlusDownSlow();

            g_nHeatBedScanStatus = 53;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("52->53"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 53: {
            // move slowly to the surface
            moveZMinusUpSlow(&nTempPressure);
            moveZPlusDownSlow(8); //and slowslowly back near idle pressure
#if DEBUG_HEAT_BED_SCAN
            nContactPressure = nTempPressure;
#endif // DEBUG_HEAT_BED_SCAN

            g_nHeatBedScanStatus = 54;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("53->54"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 54: {
            // remember the z-position and the exact y-position of this row/column
            g_ZCompensationMatrix[nIndexX][nIndexY] = (short)g_nZScanZPosition;
            g_ZCompensationMatrix[0][nIndexY] = (short)((float)nY / Printer::axisStepsPerMM[Y_AXIS] + 0.5); // convert to mm

            if (nIndexX > g_uZMatrixMax[X_AXIS]) {
                g_uZMatrixMax[X_AXIS] = nIndexX;
            }

            if (nIndexY > g_uZMatrixMax[Y_AXIS]) {
                g_uZMatrixMax[Y_AXIS] = nIndexY;
            }

            g_nHeatBedScanStatus = 55;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("54->55"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 55: {
            // move away from the surface
            if (moveZPlusDownFast()) {
                g_abortZScan = SCAN_ABORT_REASON_AVERAGE_PRESSURE;
            }

            if (nYDirection > 0) {
                nTempPosition = nY + nYDirection;

                if (nTempPosition > g_nScanYMaxPositionSteps) {
                    // we have reached the end of this column
                    g_nHeatBedScanStatus = 39;

#if DEBUG_HEAT_BED_SCAN == 2
                    if (Printer::debugInfo()) {
                        Com::printF(Com::tscanHeatBed);
                        Com::printFLN(PSTR("55->39"));
                    }
#endif // DEBUG_HEAT_BED_SCAN == 2
                    break;
                }
            } else {
                nTempPosition = nY + nYDirection;

                if (nTempPosition < g_nScanYStartSteps) {
                    // we have reached the end of this column
                    g_nHeatBedScanStatus = 39;

#if DEBUG_HEAT_BED_SCAN == 2
                    if (Printer::debugInfo()) {
                        Com::printF(Com::tscanHeatBed);
                        Com::printFLN(PSTR("55->39"));
                    }
#endif // DEBUG_HEAT_BED_SCAN == 2
                    break;
                }
            }

            // move to the next y-position
            Printer::queueRelativeStepsCoordinates(0, nYDirection, 0, 0, Printer::homingFeedrate[Y_AXIS], true, true);
            nY += nYDirection;
            nIndexY += nIndexYDirection;

            if (nIndexY > COMPENSATION_MATRIX_MAX_Y) {
#if DEBUG_HEAT_BED_SCAN == 2
                if (Printer::debugErrors()) {
                    Com::printF(Com::tscanHeatBed);
                    Com::printFLN(PSTR("y-dimension of the z matrix became too big: "), nIndexY);
                }
#endif // DEBUG_HEAT_BED_SCAN == 2
                g_abortZScan = SCAN_ABORT_REASON_MATRIX_DIMENSION;
                break;
            }

            g_nHeatBedScanStatus = 49;
            g_lastScanTime = HAL::timeInMilliseconds();

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("55->49"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 60: {
            // avoid to crash the extruder against the heat bed during the following homing
            moveZ(int(Printer::axisStepsPerMM[Z_AXIS] * 5));

            // move back to the home position
            Printer::homeAxis(true, true, true);

            if (!g_nHeatBedScanMode) {
                // disable all heaters
                Extruder::setHeatedBedTemperature(0);
                Extruder::setTemperatureForAllExtruders(0);
            }

            g_nHeatBedScanStatus = 65;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("60->65"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 65: {
            if (Printer::debugInfo()) {
                // output the determined compensation
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("raw heat bed z matrix: "));
                determineCompensationOffsetZ();
                outputCompensationMatrix();
            }

            g_nHeatBedScanStatus = 70;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("65->70"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 70: {
            g_nHeatBedScanStatus = 75;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("70->75"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 75: {
            if (Printer::debugInfo()) {
                // output the pure scan time
                Com::printF(Com::tscanHeatBed);
                Com::printF(PSTR("total scan time: "), long((HAL::timeInMilliseconds() - g_scanStartTime) / 1000));
                Com::printFLN(PSTR(" [s]"));
            }

            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("g_uZMatrixMax[Y_AXIS].1 = "), (int)g_uZMatrixMax[Y_AXIS]);
            }

            // prepare the heat bed compensation matrix for fast usage during the actual printing
            prepareCompensationMatrix();

            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("g_uZMatrixMax[Y_AXIS].2 = "), (int)g_uZMatrixMax[Y_AXIS]);

                // output the converted heat bed compensation matrix
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("converted heat bed z matrix: "));
                outputCompensationMatrix();
            }

            g_nHeatBedScanStatus = 80;
            g_lastScanTime = HAL::timeInMilliseconds();

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("75->80"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 80: {
            if ((HAL::timeInMilliseconds() - g_lastScanTime) < HEAT_BED_SCAN_DELAY) {
                // do not check too early
                break;
            }

            // compare the idle pressure at the beginning and at the end
            if (readAveragePressure(&nTempPressure)) {
                g_abortZScan = SCAN_ABORT_REASON_AVERAGE_PRESSURE;
            }

            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("idle pressure at start: "), g_nFirstIdlePressure);
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("idle pressure at stop: "), nTempPressure);
            }

#if NUM_EXTRUDER == 2
            // we have to align the two extruders
            g_nHeatBedScanStatus = 100;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("80->100"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
#else  // NUM_EXTRUDER == 2
            if (g_nHeatBedScanMode) {
                // we have to determine the z-offset which is caused by different extruder temperatures
                g_nHeatBedScanStatus = 130;

#if DEBUG_HEAT_BED_SCAN == 2
                if (Printer::debugInfo()) {
                    Com::printF(Com::tscanHeatBed);
                    Com::printFLN(PSTR("80->130"));
                }
#endif // DEBUG_HEAT_BED_SCAN == 2
            } else {
                // we are done
                g_nHeatBedScanStatus = 150;

#if DEBUG_HEAT_BED_SCAN == 2
                if (Printer::debugInfo()) {
                    Com::printF(Com::tscanHeatBed);
                    Com::printFLN(PSTR("80->150"));
                }
#endif // DEBUG_HEAT_BED_SCAN == 2
            }
#endif // NUM_EXTRUDER == 2
            break;
        }
        //################################ JUMP 100 / 130 / 150 >>
        case 100: {
            // we are homed at the moment - move to the position where both extruders shall be aligned to the same z position
            Printer::queueRelativeMMCoordinates(HEAT_BED_SCAN_X_CALIBRATION_POINT_MM, HEAT_BED_SCAN_Y_CALIBRATION_POINT_MM, 0, 0, RMath::min(Printer::homingFeedrate[X_AXIS], Printer::homingFeedrate[Y_AXIS]), true, true);

            g_lastScanTime = HAL::timeInMilliseconds();
            g_scanRetries = HEAT_BED_SCAN_RETRIES;
            g_retryStatus = 105;
            g_nHeatBedScanStatus = 110;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("100->110"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        //############################################################### ERROR HANDLING Case 45 105 139 + 132
        case 105: {
            moveZ(long(Printer::axisStepsPerMM[Z_AXIS] * g_scanStartZLiftMM)); //spacing for bed
            Printer::homeAxis(false, true, false);                             //Home Y

            g_scanRetries--;
            g_nHeatBedScanStatus = 106;
            g_lastScanTime = HAL::timeInMilliseconds();

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("105"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 106: {
            // home the z-axis in order to find the starting point again
            Printer::homeAxis(false, false, true); //Neben Bett: Home Z
            // ensure that there is no z endstop hit before we perform the z-axis homing
            moveZ(long(Printer::axisStepsPerMM[Z_AXIS] * g_scanStartZLiftMM)); //spacing for bed - wird von moveUpFast() später korrigiert.
            g_nLastZScanZPosition = 0;                                         //Reset last scan position to avoid abort due to this limitter

            g_nHeatBedScanStatus = 107;
            g_lastScanTime = HAL::timeInMilliseconds();

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo())
                Com::printFLN(PSTR("106"));
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 107: {
            Printer::queueRelativeMMCoordinates(0, HEAT_BED_SCAN_Y_CALIBRATION_POINT_MM, 0, 0, Printer::homingFeedrate[Y_AXIS], true, true);
            g_nHeatBedScanStatus = 108;
            g_lastScanTime = HAL::timeInMilliseconds();

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo())
                Com::printFLN(PSTR("107"));
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 108: // Remeasure inconsistent pressure values to have new clean fence levels
        {
            if ((HAL::timeInMilliseconds() - g_lastScanTime) < HEAT_BED_SCAN_DELAY) {
                // do not check too early
                break;
            }

            if (readIdlePressure(&g_nFirstIdlePressure)) {
                g_abortZScan = SCAN_ABORT_REASON_AVERAGE_PRESSURE;
                // we were unable to determine the idle pressure
                break;
            } else {
                adjustPressureLimits(g_nFirstIdlePressure);
            }
            g_nHeatBedScanStatus = 110;
            g_lastScanTime = HAL::timeInMilliseconds();

            break;
        }
        //############################################################### /ERROR HANDLING
        case 110: {
            if ((HAL::timeInMilliseconds() - g_lastScanTime) < g_nScanIdleDelay) {
                // do not check too early
                break;
            }

            // scan this point
            if (testIdlePressure()) {
                // the current idle pressure is not plausible
                if (g_scanRetries) {
                    g_retryZScan = 1;
                } else {
                    g_abortZScan = SCAN_ABORT_REASON_IDLE_PRESSURE;
                }
                break;
            }

            // we should consider that the idle presse can change slightly
            adjustPressureLimits(g_nCurrentIdlePressure);

            g_nHeatBedScanStatus = 120;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("110->120"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 120: {
            // move to the surface
            moveZMinusUpFast();

            g_nHeatBedScanStatus = 121;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("120->121"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 121: {
            // ensure that we do not remember any previous z-position at this moment
            g_nLastZScanZPosition = 0;

            // move a little bit away from the surface
            moveZPlusDownSlow();

            g_nHeatBedScanStatus = 122;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("121->122"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 122: {
            // move slowly to the surface
            moveZMinusUpSlow(&nTempPressure);
            moveZPlusDownSlow(8); //and slowslowly back near idle pressure

            g_nHeatBedScanStatus = 123;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("122->123"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 123: {
            // the left extruder is at the surface now - show that the user must move also the right extruder to the surface in order to get them to the same z-height
            UI_STATUS_UPD(UI_TEXT_ALIGN_EXTRUDERS);

            g_lastScanTime = HAL::timeInMilliseconds();

            g_nContinueButtonPressed = 0;
            g_nHeatBedScanStatus = 125;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("123->125"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 125: {
            // wait until the continue button has been pressed
            if (!g_nContinueButtonPressed) {
                unsigned long uRemainingSeconds;
                uRemainingSeconds = (HAL::timeInMilliseconds() - g_lastScanTime) / 1000;
                if (uRemainingSeconds > HEAT_BED_SCAN_ALIGN_EXTRUDERS_ABORT_DELAY + 4) {
                    g_nHeatBedScanStatus = 149;
                    break;
                }
                break;
            }

            if (g_nHeatBedScanMode) {
                // we have to determine the z-offset which is caused by different extruder temperatures
                g_nHeatBedScanStatus = 132;

#if DEBUG_HEAT_BED_SCAN == 2
                if (Printer::debugInfo()) {
                    Com::printF(Com::tscanHeatBed);
                    Com::printFLN(PSTR("125->132"));
                }
#endif // DEBUG_HEAT_BED_SCAN == 2
            } else {
                // we are done
                g_nHeatBedScanStatus = 149;

#if DEBUG_HEAT_BED_SCAN == 2
                if (Printer::debugInfo()) {
                    Com::printF(Com::tscanHeatBed);
                    Com::printFLN(PSTR("125->149"));
                }
#endif // DEBUG_HEAT_BED_SCAN == 2
            }
            break;
        }
        //################################ JUMP 132 / 149 >>
        case 130: {
            // we are homed at the moment - move to the position where we shall determine the length offset which is caused by the heated up extruder
            Printer::queueRelativeMMCoordinates(HEAT_BED_SCAN_X_CALIBRATION_POINT_MM, HEAT_BED_SCAN_Y_CALIBRATION_POINT_MM, 0, 0, RMath::min(Printer::homingFeedrate[X_AXIS], Printer::homingFeedrate[Y_AXIS]), true, true);

            g_lastScanTime = HAL::timeInMilliseconds();
            g_nHeatBedScanStatus = 133;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("130->133"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 132: // we have to determine the z-offset which is caused by different extruder temperatures
        {
            // determine the z-home position
            //if (Printer::currentZSteps < 0){
            //Printer::homeAxis( false, false, true); //Nibbels: wäre schädlich für zu hohe druckbetten.
            // -> Warum homen, wir fahren nun zum Heizen sowieso 10mm weg. Vorher waren wir direkt am Bett. Unter oder Über Zschalter ist egal.
            // -> Wills nicht rausmachen, könnte ich aber. Schlädlich wenn Bett zuuu tief wegen watchdog - glaube nicht?
            // -> Wenn Extruder unter 0, dann homen, sonst nicht.
            // -> 08.02.2018: wir haben ein homing von case 60, scanposition ist konsistent, warum homen ...
            //}
            g_lastScanTime = HAL::timeInMilliseconds();
            g_nHeatBedScanStatus = 133;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("132->133"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 133: {
            // move the heat bed 5mm down
            moveZ(int(Printer::axisStepsPerMM[Z_AXIS] * 5));
            g_lastScanTime = HAL::timeInMilliseconds();
            g_nHeatBedScanStatus = 134;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("133->134"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 134: {
            // move the heat bed 5mm down
            moveZ(int(Printer::axisStepsPerMM[Z_AXIS] * 5));
            g_lastScanTime = HAL::timeInMilliseconds();
            g_nHeatBedScanStatus = 135;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("134->135"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 135: {
            g_lastScanTime = HAL::timeInMilliseconds();
            // at this point we are homed and we are above the x/y position at which we shall perform the measurement of the z-offset with the hot extruder(s)
            if (g_nHeatBedScanMode == HEAT_BED_SCAN_MODE_PLA) {
                Extruder::setTemperatureForAllExtruders(PRECISE_HEAT_BED_SCAN_EXTRUDER_TEMP_PLA);
            } else if (g_nHeatBedScanMode == HEAT_BED_SCAN_MODE_ABS) {
                Extruder::setTemperatureForAllExtruders(PRECISE_HEAT_BED_SCAN_EXTRUDER_TEMP_ABS);
            }

            g_nHeatBedScanStatus = 136;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("135->136"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 136: {
            if ((HAL::timeInMilliseconds() - g_lastScanTime) < HEAT_BED_SCAN_DELAY) {
                // do not check too often
                break;
            }

            if (testExtruderTemperature()) {
                // we did not reach the proper temperature
                g_lastScanTime = HAL::timeInMilliseconds();
                break;
            }

            g_nHeatBedScanStatus = 137;
            g_lastScanTime = HAL::timeInMilliseconds();

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("136->137"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2

            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("calibration delay [s] =  "), (uint32_t)PRECISE_HEAT_BED_SCAN_CALIBRATION_DELAY);
            }
            break;
        }
        case 137: {
            if (g_nHeatBedScanMode) {
                // wait some time so that the desired target temperature is reached in all parts of our components
                unsigned long uRemainingSeconds;

                uRemainingSeconds = (HAL::timeInMilliseconds() - g_lastScanTime) / 1000;
                if (uRemainingSeconds < PRECISE_HEAT_BED_SCAN_CALIBRATION_DELAY) {
                    char szStatus[32];
                    strcpy(szStatus, UI_TEXT_HEATING);
                    addLong(szStatus, PRECISE_HEAT_BED_SCAN_CALIBRATION_DELAY - uRemainingSeconds, 3);
                    strcat(szStatus, "[s]");
                    UI_STATUS_UPD_RAM(szStatus);
                    break;
                }
            }

            g_scanRetries = HEAT_BED_SCAN_RETRIES;
            g_retryStatus = 139;
            g_nHeatBedScanStatus = 144;
            g_lastScanTime = HAL::timeInMilliseconds();

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("137->144"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        //############################################################### ERROR HANDLING Case 45 105 139 + 132
        case 139: {
            moveZ(long(Printer::axisStepsPerMM[Z_AXIS] * g_scanStartZLiftMM)); //spacing for bed
            Printer::homeAxis(false, true, false);                             //Home Y

            g_scanRetries--;
            g_nHeatBedScanStatus = 140;
            g_lastScanTime = HAL::timeInMilliseconds();

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo())
                Com::printFLN(PSTR("139"));
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }

        case 140: {
            // home the z-axis in order to find the starting point again
            Printer::homeAxis(false, false, true);                             //Neben Bett: Home Z
            moveZ(long(Printer::axisStepsPerMM[Z_AXIS] * g_scanStartZLiftMM)); //spacing for bed
            g_nLastZScanZPosition = 0;                                         //Reset last scan position to avoid abort due to this limitter

            g_nHeatBedScanStatus = 141;
            g_lastScanTime = HAL::timeInMilliseconds();

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo())
                Com::printFLN(PSTR("140"));
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 141: {
            Printer::queueRelativeMMCoordinates(0, HEAT_BED_SCAN_Y_CALIBRATION_POINT_MM, 0, 0, Printer::homingFeedrate[Y_AXIS], true, true);

            g_nHeatBedScanStatus = 142;
            g_lastScanTime = HAL::timeInMilliseconds();

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo())
                Com::printFLN(PSTR("141"));
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 142: // Remeasure inconsistent pressure values to have new clean fence levels
        {
            if ((HAL::timeInMilliseconds() - g_lastScanTime) < HEAT_BED_SCAN_DELAY) {
                // do not check too early
                break;
            }

            if (readIdlePressure(&g_nFirstIdlePressure)) {
                g_abortZScan = SCAN_ABORT_REASON_AVERAGE_PRESSURE;
                // we were unable to determine the idle pressure
                break;
            } else {
                adjustPressureLimits(g_nFirstIdlePressure);
            }
            g_nHeatBedScanStatus = 144;
            g_lastScanTime = HAL::timeInMilliseconds();

            break;
        }

        //############################################################### /ERROR HANDLING
        case 144: {
            if ((HAL::timeInMilliseconds() - g_lastScanTime) < g_nScanIdleDelay) {
                // do not check too early
                break;
            }

            // scan this point
            if (testIdlePressure()) {
                // the current idle pressure is not plausible
                if (g_scanRetries) {
                    g_retryZScan = 1;
                } else {
                    g_abortZScan = SCAN_ABORT_REASON_IDLE_PRESSURE;
                }
                break;
            }

            // we should consider that the idle presse can change slightly
            adjustPressureLimits(g_nCurrentIdlePressure);

            g_nHeatBedScanStatus = 145;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("144->145"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 145: {
            // move to the surface
            moveZMinusUpFast();

            g_nHeatBedScanStatus = 146;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("145->146"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 146: {
            // ensure that we do not remember any previous z-position at this moment
            g_nLastZScanZPosition = 0;

            // move a little bit away from the surface
            moveZPlusDownSlow();

            g_nHeatBedScanStatus = 147;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("146->147"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 147: {
            // move slowly to the surface
            moveZMinusUpSlow(&nTempPressure);
            moveZPlusDownSlow(8); //and slowslowly back near idle pressure

            g_nHeatBedScanStatus = 148;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("147->148"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 148: {
            // adjust the current z-position to the compensation matrix in order to consider the different length of the extruder at higher temperatures
            adjustCompensationMatrix((short)g_nZScanZPosition);

            if (Printer::debugInfo()) {
                // output the converted heat bed compensation matrix
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("adjusted heat bed z matrix: "));
                outputCompensationMatrix();
            }

            g_nHeatBedScanStatus = 149;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("148->149"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 149: {
            // avoid to crash the extruder against the heat bed during the following homing
            moveZ(int(Printer::axisStepsPerMM[Z_AXIS] * 5));

            // move back to the home position
            Printer::homeAxis(true, true, true);
            g_nHeatBedScanStatus = 150;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("149->150"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 150: {
            // turn off the engines
            Printer::disableAllSteppersNow();

            // disable all heaters
            Extruder::setHeatedBedTemperature(0);
            Extruder::setTemperatureForAllExtruders(0);

            g_nHeatBedScanStatus = 160;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("150->160"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 160: {
            // save the determined values to the EEPROM
            saveCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed));
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("the heat bed z matrix has been saved"));
            }

            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("the scan has been completed"));
            }
            //UI_STATUS_UPD( UI_TEXT_HEAT_BED_SCAN_DONE );
            g_uStartOfIdle = HAL::timeInMilliseconds(); //end scanHeatBed

            showInformation(PSTR(UI_TEXT_HEAT_BED_SCAN_DONE), (void*)ui_text_saving_success, PSTR(UI_TEXT_OK)); //tell user the scan was a success

            g_nHeatBedScanStatus = 0;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo()) {
                Com::printF(Com::tscanHeatBed);
                Com::printFLN(PSTR("160->0"));
            }
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        }
    }
} // scanHeatBed

#if FEATURE_ALIGN_EXTRUDERS
void startAlignExtruders(void) {
    if (g_nAlignExtrudersStatus) {
        // abort the alignment of the extruders
        g_abortZScan = SCAN_ABORT_REASON_CANCELED;
        return;
    } else {
        if (Printer::isPrinting()) {
            // there is some printing in progress at the moment - do not start to align the extruders in this case
            if (Printer::debugErrors()) {
                Com::printFLN(Com::tPrintingIsInProcessError);
            }
            showError((void*)ui_text_align_extruders, (void*)ui_text_operation_denied);
            return;
        }
        if (abs(extruder[0].tempControl.currentTemperatureC - extruder[1].tempControl.currentTemperatureC) > 10
            /* Diese Delta-Prüfung ist irrelevant, wenn ein Sensor defekt markiert ist. Ich brauch das ab und an beim Testen als Ausnahme: */
            && !extruder[0].tempControl.isSensorDefect()
            && !extruder[1].tempControl.isSensorDefect()) {
            if (Printer::debugErrors()) {
                Com::printFLN(PSTR("startAlignExtruders(): error temperature difference too big"));
            }
            showError((void*)ui_text_align_extruders, (void*)ui_text_temperature_wrong);
            return;
        }

        if (Printer::doHeatBedZCompensation
            || !Printer::areAxisHomed()
            || Printer::currentXSteps * Printer::axisMMPerSteps[X_AXIS] < HEAT_BED_SCAN_X_START_MM
            || Printer::currentYSteps * Printer::axisMMPerSteps[Y_AXIS] < HEAT_BED_SCAN_Y_START_MM
            || Printer::currentZSteps) {
            Printer::homeAxis(true, true, true);
            Printer::queueRelativeMMCoordinates(HEAT_BED_SCAN_X_CALIBRATION_POINT_MM, HEAT_BED_SCAN_Y_CALIBRATION_POINT_MM, 0, 0, RMath::min(Printer::homingFeedrate[X_AXIS], Printer::homingFeedrate[Y_AXIS]), true, true);
        }

        // we are ready to align the extruders at the current x and y position with the current temperature
        // the user can choose the x and y position as well as the to-be-used temperatures of the extruders
        g_nAlignExtrudersStatus = 100;
        g_abortZScan = 0; //dont kill job on start
    }
} // startAlignExtruders

void alignExtruders(void) {
#if FEATURE_ALIGN_EXTRUDERS
    //if( g_nAlignExtrudersStatus ) return;
#endif //FEATURE_ALIGN_EXTRUDERS
    if (g_nHeatBedScanStatus)
        return;
    if (g_nZOSScanStatus)
        return;

    if (g_abortZScan) {
        showAbortScanReason((void*)ui_text_align_extruders, g_abortZScan);
        // the alignment has been aborted
        g_abortZScan = 0;

        // avoid to crash the extruder against the heat bed during a following move
        moveZ(int(Printer::axisStepsPerMM[Z_AXIS] * 5));

        Com::printFLN(PSTR("alignExtruders(): aborted"));

        g_nAlignExtrudersStatus = 0;
        g_uStartOfIdle = HAL::timeInMilliseconds() + 30000; //alignExtruders aborted
        return;
    }

    if (g_nAlignExtrudersStatus) {
        // show that we are active
        previousMillisCmd = HAL::timeInMilliseconds(); //prevent inactive shutdown of steppers/temps
        GCode::keepAlive(Calibrating);

        if (g_nAlignExtrudersStatus != 123 && g_nAlignExtrudersStatus != 125) {
            // there are a few cases where we do not want to change the current status text
            UI_STATUS(UI_TEXT_ALIGN_EXTRUDERS);
        }

        switch (g_nAlignExtrudersStatus) {
        case 100: {
            // when we are here we assume that all preconditions for the alignment of the extruders are fulfilled already
            g_lastScanTime = HAL::timeInMilliseconds();
            g_scanRetries = HEAT_BED_SCAN_RETRIES;
            g_retryStatus = 105;
            g_nLastZScanZPosition = 0; //sodass dass bei mehreren scans nicht die letzte position als abstands limit feststeht.
            g_nAlignExtrudersStatus = 110;
            g_uStartOfIdle = 0; //align extruders
#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo())
                Com::printFLN(PSTR("alignExtruders(): 100 -> 110"));
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 110: {
            if ((HAL::timeInMilliseconds() - g_lastScanTime) < g_nScanIdleDelay) {
                // do not check too early
                break;
            }

            // scan this point
            if (testIdlePressure()) {
                g_abortZScan = SCAN_ABORT_REASON_IDLE_PRESSURE;
                // the current idle pressure is not plausible
                // g_retryZScan = 1; //not really needed, already set.
                break;
            }

            // we should consider that the idle presse can change slightly
            adjustPressureLimits(g_nCurrentIdlePressure);

            g_nAlignExtrudersStatus = 120;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo())
                Com::printFLN(PSTR("110 -> 120"));
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 120: {
            // move to the surface
            moveZMinusUpFast();

            g_nAlignExtrudersStatus = 121;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo())
                Com::printFLN(PSTR("120 -> 121"));
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 121: {
            // ensure that we do not remember any previous z-position at this moment
            g_nLastZScanZPosition = 0;

            // move a little bit away from the surface
            moveZPlusDownSlow();

            g_nAlignExtrudersStatus = 122;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo())
                Com::printFLN(PSTR("121 -> 122"));
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 122: {
            // move slowly to the surface
            short nTempPressure = 0;
            moveZMinusUpSlow(&nTempPressure);
            moveZPlusDownSlow(8); // entspannen

            g_nAlignExtrudersStatus = 123;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo())
                Com::printFLN(PSTR("122 -> 123"));
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 123: {
            // the left extruder is at the surface now - show that the user must move also the right extruder to the surface in order to get them to the same z-height
            UI_STATUS_UPD(UI_TEXT_ALIGN_EXTRUDERS);

            g_lastScanTime = HAL::timeInMilliseconds();

            g_nContinueButtonPressed = 0;
            g_nAlignExtrudersStatus = 125;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo())
                Com::printFLN(PSTR("123 -> 125"));
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 125: {
            // wait until the continue button has been pressed
            if (!g_nContinueButtonPressed) {
                unsigned long uRemainingSeconds;
                uRemainingSeconds = (HAL::timeInMilliseconds() - g_lastScanTime) / 1000;
                if (uRemainingSeconds > HEAT_BED_SCAN_ALIGN_EXTRUDERS_ABORT_DELAY + 4) {
                    g_abortZScan = SCAN_ABORT_REASON_PRINTER_LONELY;
                    Extruder::setTemperatureForAllExtruders(0);
                    Extruder::setHeatedBedTemperature(0);
                    break;
                }
                break;
            }

            // we are done
            g_nAlignExtrudersStatus = 145;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo())
                Com::printFLN(PSTR("125 -> 145"));
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 145: {
            // avoid to crash the extruder against the heat bed during the following moves
            moveZ(int(Printer::axisStepsPerMM[Z_AXIS] * 5));

            Printer::homeAxis(true, true, true);

            g_nAlignExtrudersStatus = 160;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo())
                Com::printFLN(PSTR("145 -> 160"));
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        case 160: {
            if (Printer::debugInfo()) {
                Com::printFLN(PSTR("alignExtruders(): the alignment has been completed"));
            }

            showInformation(PSTR(UI_TEXT_ALIGN_EXTRUDERS_DONE));
            g_uStartOfIdle = HAL::timeInMilliseconds() + 30000; //alignExtruders ended

            g_nAlignExtrudersStatus = 0;

#if DEBUG_HEAT_BED_SCAN == 2
            if (Printer::debugInfo())
                Com::printFLN(PSTR("160 -> 0"));
#endif // DEBUG_HEAT_BED_SCAN == 2
            break;
        }
        }
    }
    return;
} // alignExtruders
#endif // FEATURE_ALIGN_EXTRUDERS

/**************************************************************************************************************************************/
void startZOScan(bool automatrixleveling) {
#if FEATURE_ALIGN_EXTRUDERS
    if (g_nAlignExtrudersStatus)
        return;
#endif //FEATURE_ALIGN_EXTRUDERS
    if (g_nHeatBedScanStatus)
        return;
        //if( g_nZOSScanStatus ) return;

    if (g_nZOSScanStatus) {
        // abort the heat bed scan
        Com::printFLN(PSTR("ZOS cancelled"));
        g_abortZScan = SCAN_ABORT_REASON_CANCELED;
        abortSearchHeatBedZOffset(false);
    } else {
        Com::printFLN(PSTR("ZOS started"));
        g_nZOSScanStatus = 1;
        // start the heat bed scan
        g_abortZScan = 0; //dont kill job on start
        if (automatrixleveling)
            g_ZOS_Auto_Matrix_Leveling_State = 1; //aktiviert besonderer modus, bei dem der ZOffsetScan mehrfach in schleife scant und ein schiefes Bett geraderückt, aber die Welligkeit des ursprünglichen HBS behält.
    }
} // startZOScan

void searchZOScan(void) {
#if FEATURE_ALIGN_EXTRUDERS
    if (g_nAlignExtrudersStatus)
        return;
#endif //FEATURE_ALIGN_EXTRUDERS
    if (g_nHeatBedScanStatus)
        return;
        //if( g_nZOSScanStatus ) return;

    if (!g_nZOSScanStatus)
        return;

    if (g_nHeatBedScanStatus
    ) {
        g_nZOSScanStatus = 0;
        g_ZOS_Auto_Matrix_Leveling_State = 0;
        return;
    }

    if (g_nZOSScanStatus) {
        previousMillisCmd = HAL::timeInMilliseconds(); //prevent inactive shutdown of steppers/temps
        GCode::keepAlive(Calibrating);

        switch (g_nZOSScanStatus) {
        case 1: {
            g_retryZScan = 0;
            g_uStartOfIdle = 0; //zeige nicht gleich wieder Printer Ready an.
            Com::printFLN(PSTR("ZOS init"));
            // when the heat bed Z offset is searched, the z-compensation must be disabled
            g_nZOSScanStatus = 2;
            g_min_nZScanZPosition = long(Printer::axisStepsPerMM[Z_AXIS] * g_scanStartZLiftMM); //nur nutzen wenn kleiner.
            g_scanRetries = 0;                                                                  // never retry   TODO allow retries?
            g_abortZScan = 0;                                                                   // will be set in case of error inside moveZMinusUpFast/Slow
            g_nLastZScanZPosition = 0;                                                          //sodass dass bei mehreren scans nicht die letzte position als abstands limit feststeht.
            break;
        }
        case 2: {
#if DEBUG_HEAT_BED_SCAN == 2
            Com::printFLN(PSTR("Home"));
#endif // DEBUG_HEAT_BED_SCAN == 2

            //bissel übertrieben, sollte aber jede eventualität abfangen: Wir brauchen die maximale Matrix-Dimension auch schon hier (ganz grob) und wollen nicht so lange warten.
            unsigned short uDimensionX, uDimensionY;
            if (g_ZCompensationMatrix[0][0] == EEPROM_FORMAT) {
                uDimensionX = g_uZMatrixMax[X_AXIS] - 1;
                uDimensionY = g_uZMatrixMax[Y_AXIS] - 1;
            } else {
                uDimensionX = (unsigned char)readWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_SECTOR_SIZE * g_nActiveHeatBed + EEPROM_OFFSET_DIMENSION_X) - 1;
                uDimensionY = (unsigned char)readWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_SECTOR_SIZE * g_nActiveHeatBed + EEPROM_OFFSET_DIMENSION_Y) - 1;
            }
            if (uDimensionX > COMPENSATION_MATRIX_MAX_X - 1)
                uDimensionX = (unsigned char)(COMPENSATION_MATRIX_MAX_X - 1);
            if (uDimensionY > COMPENSATION_MATRIX_MAX_Y - 1)
                uDimensionY = (unsigned char)(COMPENSATION_MATRIX_MAX_Y - 1);

            //HERE THE FUNCTION MIGHT JUMP IN TO REDO SCANS FOR AUTO_MATRIX_LEVELING
            switch (g_ZOS_Auto_Matrix_Leveling_State) {
            case 1: {
                //Normal start, but overwrite first scan coordinate:
                g_ZOSTestPoint[X_AXIS] = (unsigned char)(uDimensionX / 2);
                g_ZOSTestPoint[Y_AXIS] = (unsigned char)(uDimensionY / 2);
                g_ZOSlearningRate = 1.0f;
                g_ZOSlearningGradient = 0.0f;
                g_min_nZScanZPosition = long(Printer::axisStepsPerMM[Z_AXIS] * g_scanStartZLiftMM); //nur nutzen wenn kleiner.
                Com::printF(Com::tAutoMatrixLeveling, g_ZOSTestPoint[X_AXIS]);
                Com::printFLN(PSTR(" Y:"), g_ZOSTestPoint[Y_AXIS]);
                break;
            }
            case 2: {
                g_ZOSTestPoint[X_AXIS] = 1; //will be constrained
                g_ZOSTestPoint[Y_AXIS] = 1; //will be constrained
                g_ZOSlearningRate = 0.8f;
                g_ZOSlearningGradient = 1.0f;
                g_min_nZScanZPosition = long(Printer::axisStepsPerMM[Z_AXIS] * g_scanStartZLiftMM); //nur nutzen wenn kleiner.
                Com::printF(Com::tAutoMatrixLeveling, g_ZOSTestPoint[X_AXIS]);
                Com::printFLN(PSTR(" Y:"), g_ZOSTestPoint[Y_AXIS]);
                break;
            }
            case 3: {
                g_ZOSTestPoint[X_AXIS] = uDimensionX; //will be constrained
                g_ZOSTestPoint[Y_AXIS] = uDimensionY; //will be constrained
                g_ZOSlearningRate = 0.8f;
                g_ZOSlearningGradient = 1.0f;
                g_min_nZScanZPosition = long(Printer::axisStepsPerMM[Z_AXIS] * g_scanStartZLiftMM); //nur nutzen wenn kleiner.
                Com::printF(Com::tAutoMatrixLeveling, g_ZOSTestPoint[X_AXIS]);
                Com::printFLN(PSTR(" Y:"), g_ZOSTestPoint[Y_AXIS]);
                break;
            }
            case 4: {
                g_ZOSTestPoint[X_AXIS] = (unsigned char)(uDimensionX / 2);
                g_ZOSTestPoint[Y_AXIS] = (unsigned char)(uDimensionY / 2 - 1);
                g_ZOSlearningRate = 0.8f;
                g_ZOSlearningGradient = 1.0f;
                g_min_nZScanZPosition = long(Printer::axisStepsPerMM[Z_AXIS] * g_scanStartZLiftMM); //nur nutzen wenn kleiner.
                Com::printF(Com::tAutoMatrixLeveling, g_ZOSTestPoint[X_AXIS]);
                Com::printFLN(PSTR(" Y:"), g_ZOSTestPoint[Y_AXIS]);
                break;
            }
            case 5: {
                g_ZOSTestPoint[X_AXIS] = 1;           //will be constrained
                g_ZOSTestPoint[Y_AXIS] = uDimensionY; //will be constrained
                g_ZOSlearningRate = 0.8f;
                g_ZOSlearningGradient = 1.0f;
                g_min_nZScanZPosition = long(Printer::axisStepsPerMM[Z_AXIS] * g_scanStartZLiftMM); //nur nutzen wenn kleiner.
                Com::printF(Com::tAutoMatrixLeveling, g_ZOSTestPoint[X_AXIS]);
                Com::printFLN(PSTR(" Y:"), g_ZOSTestPoint[Y_AXIS]);
                break;
            }
            case 6: {
                g_ZOSTestPoint[X_AXIS] = uDimensionX; //will be constrained
                g_ZOSTestPoint[Y_AXIS] = 1;           //will be constrained
                g_ZOSlearningRate = 0.8f;
                g_ZOSlearningGradient = 1.0f;
                g_min_nZScanZPosition = long(Printer::axisStepsPerMM[Z_AXIS] * g_scanStartZLiftMM); //nur nutzen wenn kleiner.
                Com::printF(Com::tAutoMatrixLeveling, g_ZOSTestPoint[X_AXIS]);
                Com::printFLN(PSTR(" Y:"), g_ZOSTestPoint[Y_AXIS]);
                break;
            }
            case 7: {
                g_ZOSTestPoint[X_AXIS] = (unsigned char)(uDimensionX / 2);
                g_ZOSTestPoint[Y_AXIS] = (unsigned char)(uDimensionY / 2 + 1);
                g_ZOSlearningRate = 0.8f;
                g_ZOSlearningGradient = 1.0f;
                g_min_nZScanZPosition = long(Printer::axisStepsPerMM[Z_AXIS] * g_scanStartZLiftMM); //nur nutzen wenn kleiner.
                Com::printF(Com::tAutoMatrixLeveling, g_ZOSTestPoint[X_AXIS]);
                Com::printFLN(PSTR(" Y:"), g_ZOSTestPoint[Y_AXIS]);
                g_ZOS_Auto_Matrix_Leveling_State = 0; //Auto-Matrix-Leveling ends after last scan.
                break;
            }
            default: {

            } // No AUTO_MATRIX_LEVELING
            //Normal start: End after this scan.
            }

            // start at the home position
            Printer::homeAxis(true, true, true); //home z resets ZCMP
            g_nZOSScanStatus = 3;
            break;
        }
        case 3: {
#if DEBUG_HEAT_BED_SCAN == 2
            Com::printF(PSTR("Spacing Z"), g_min_nZScanZPosition);
            Com::printFLN(PSTR(" [Steps]"));
#endif // DEBUG_HEAT_BED_SCAN == 2

            moveZ(g_min_nZScanZPosition); //Wenn man hier mit queueRelativeStepsCoordinates über die queue Z verfährt, zeigt das display 10mm statt 5mm an. Weil currentZPositionSteps addiert und man die hier braucht. Also während dem Scan immer nur die moveZ verwenden.

            // move a bit away from the heat bed in order to achieve better measurements in case of hardware configurations where the extruder is very close to the heat bed after the z-homing
            UI_STATUS_UPD(UI_TEXT_ZCALIB);
            g_nZOSScanStatus = 4;
            break;
        }
        case 4: {
#if DEBUG_HEAT_BED_SCAN == 2
            Com::printFLN(PSTR("Load Matrix"));
#endif // DEBUG_HEAT_BED_SCAN == 2

            // load the unaltered compensation matrix from the EEPROM
            if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT || g_ZOSlearningRate == 1.0) {
                Com::printFLN(PSTR("Loading zMatrix from EEPROM"));
                if (loadCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed))) {
                    // Error: there is no valid compensation matrix available
                    initCompensationMatrix();
                }
            } else {
                Com::printFLN(PSTR("Reusing existing zMatrix"));
            }

            // safety check on the current matrix
            if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
                Com::printFLN(PSTR("ERROR::prev. matrix invalid!"));
#if DEBUG_HEAT_BED_SCAN == 2
                Com::printFLN(PSTR("Neuen HBS machen!"));
#endif // DEBUG_HEAT_BED_SCAN == 2
                g_abortZScan = SCAN_ABORT_REASON_MISSING_MATRIX;
                abortSearchHeatBedZOffset(false);
                break;
            }
            g_nZOSScanStatus = 5;
            break;
        }
        case 5: {
            //Constrain X-Y for Scanposition to valid locations at noninterpolated matrix-scanpoints.
            if (g_ZOSTestPoint[X_AXIS] < 1 + ((HEAT_BED_SCAN_X_START_MM == 0) ? 0 : 1))
                g_ZOSTestPoint[X_AXIS] = 1 + ((HEAT_BED_SCAN_X_START_MM == 0) ? 0 : 1);
            if (g_ZOSTestPoint[X_AXIS] > g_uZMatrixMax[X_AXIS] - 1)
                g_ZOSTestPoint[X_AXIS] = g_uZMatrixMax[X_AXIS] - 1; //2..n-1
            if (g_ZOSTestPoint[Y_AXIS] < 1 + ((HEAT_BED_SCAN_Y_START_MM == 0) ? 0 : 1))
                g_ZOSTestPoint[Y_AXIS] = 1 + ((HEAT_BED_SCAN_Y_START_MM == 0) ? 0 : 1);
            if (g_ZOSTestPoint[Y_AXIS] > g_uZMatrixMax[Y_AXIS] - 1)
                g_ZOSTestPoint[Y_AXIS] = g_uZMatrixMax[Y_AXIS] - 1; //2..n-1
            // move to the first scan position of the heat bed scan matrix
            float xScanPosition = g_ZCompensationMatrix[g_ZOSTestPoint[X_AXIS]][0]; // + g_nScanXStartSteps; <-- NEIN! Man muss nur die jeweils erste und letzte Matrix-Zeile meiden, ausser HEAT_BED_SCAN_X_START_MM ist 0 oder HEAT_BED_SCAN_Y_START_MM ist 0
            float yScanPosition = g_ZCompensationMatrix[0][g_ZOSTestPoint[Y_AXIS]]; // + g_nScanYStartSteps; <-- NEIN!

#if SEARCH_HEAT_BED_OFFSET_SCAN_POSITION_RAND_MM > 0
            xScanPosition += random(-1 * SEARCH_HEAT_BED_OFFSET_SCAN_POSITION_RAND_MM, SEARCH_HEAT_BED_OFFSET_SCAN_POSITION_RAND_MM);
            yScanPosition += random(-1 * SEARCH_HEAT_BED_OFFSET_SCAN_POSITION_RAND_MM, SEARCH_HEAT_BED_OFFSET_SCAN_POSITION_RAND_MM);
#endif //SEARCH_HEAT_BED_OFFSET_SCAN_POSITION_RAND_MM

#if DEBUG_HEAT_BED_SCAN == 2
            Com::printF(PSTR("Scan Position X+Y"));
            Com::printF(PSTR("= ("), xScanPosition);
            Com::printF(PSTR(", "), yScanPosition);
            Com::printFLN(PSTR(") [(x,y) Steps]"));
#endif // DEBUG_HEAT_BED_SCAN == 2
            Printer::queueRelativeMMCoordinates(xScanPosition, yScanPosition, 0, 0, RMath::min(Printer::homingFeedrate[X_AXIS], Printer::homingFeedrate[Y_AXIS]), true, true);
            g_nZOSScanStatus = 6;
            break;
        }
        case 6: {
            g_scanRetries = 20; //für 9, 10, 20
            g_nZOSScanStatus = 9;
            break;
        }

        case 7: //ab hier bei Fehler:
        {
            HAL::delayMilliseconds(HEAT_BED_SCAN_DELAY);
            if (moveZPlusDownFast()) {
				g_abortZScan = SCAN_ABORT_REASON_AVERAGE_PRESSURE;
			}
            g_nZOSScanStatus = 9;
            break;
        }
        case 9: {
#if DEBUG_HEAT_BED_SCAN == 2
            Com::printFLN(PSTR("Idle Pressure"));
#endif // DEBUG_HEAT_BED_SCAN == 2
            if (readIdlePressure(&g_nCurrentIdlePressure)) {
                //Problem mit Digits, die wackeln:
                if (g_scanRetries-- > 0) {
                    g_nZOSScanStatus = 7;
                    g_abortZScan = 0;
                    break;
                } else {
                    Com::printFLN(PSTR("ERROR::the idle pressure could not be determined"));
                    g_abortZScan = SCAN_ABORT_REASON_IDLE_PRESSURE;
                    abortSearchHeatBedZOffset(false);
                }
                break;
            }

            g_nMinPressureContact = g_nCurrentIdlePressure - SEARCH_HEAT_BED_OFFSET_CONTACT_PRESSURE_DELTA;
            g_nMaxPressureContact = g_nCurrentIdlePressure + SEARCH_HEAT_BED_OFFSET_CONTACT_PRESSURE_DELTA;
            g_nMinPressureRetry = g_nCurrentIdlePressure - SEARCH_HEAT_BED_OFFSET_RETRY_PRESSURE_DELTA;
            g_nMaxPressureRetry = g_nCurrentIdlePressure + SEARCH_HEAT_BED_OFFSET_RETRY_PRESSURE_DELTA;
            g_nMinPressureIdle = g_nCurrentIdlePressure - SEARCH_HEAT_BED_OFFSET_IDLE_PRESSURE_DELTA;
            g_nMaxPressureIdle = g_nCurrentIdlePressure + SEARCH_HEAT_BED_OFFSET_IDLE_PRESSURE_DELTA;

#if DEBUG_HEAT_BED_SCAN == 2
            Com::printFLN(PSTR("Idle Values:"));
            Com::printFLN(PSTR(" g_nMinPressureContact = "), g_nMinPressureContact);
            Com::printFLN(PSTR(" g_nMaxPressureContact = "), g_nMaxPressureContact);
            Com::printFLN(PSTR(" g_nMinPressureRetry = "), g_nMinPressureRetry);
            Com::printFLN(PSTR(" g_nMaxPressureRetry = "), g_nMaxPressureRetry);
            Com::printFLN(PSTR(" g_nMinPressureIdle = "), g_nMinPressureIdle);
            Com::printFLN(PSTR(" g_nMaxPressureIdle = "), g_nMaxPressureIdle);
#endif // DEBUG_HEAT_BED_SCAN == 2
            g_nZOSScanStatus = 10;
            break;
        }
        case 10: {
            Com::printFLN(PSTR("Approaching HeatBed"));

            // move to the surface
            long oldZsteps = g_nZScanZPosition;

            moveZMinusUpFast(); // ------
            HAL::delayMilliseconds(g_nScanSlowStepDelay);

            //Wenn Filament langsam nachgibt, wandert evtl. die Kraft langsam. Hier prüfen, ob idle digits gültig.
            short nTempPressure;
            if (readAveragePressure(&nTempPressure)) {
                if (g_scanRetries <= 0) {
                    g_abortZScan = SCAN_ABORT_REASON_AVERAGE_PRESSURE;
                }
                g_retryZScan = 1;
            }

            long didZsteps = abs(g_nZScanZPosition - oldZsteps);
            Com::printFLN(PSTR("FastUp:"), didZsteps);

            // move 2 intervals (but not more that you went down!) back away from the surface
            long revertZsteps = (didZsteps > 2 * abs(g_nScanHeatBedUpFastSteps) ? 2 * abs(g_nScanHeatBedUpFastSteps) : didZsteps);
            Com::printFLN(PSTR("RevertDown:"), revertZsteps);
            moveZ(revertZsteps); // ++

            HAL::delayMilliseconds(g_nScanSlowStepDelay);
            //rescan force and look if you reverted the contact pressure to the old state:
            short nTempPressureUp;
            if (readAveragePressure(&nTempPressureUp)) {
                if (g_scanRetries <= 0) {
                    g_abortZScan = SCAN_ABORT_REASON_AVERAGE_PRESSURE;
                }
                g_retryZScan = 1;
            }
            //check if the old pressure state is reached again, retry if not. If not you measured some melted plastic or your DMS is driving away.
            if (abs(nTempPressureUp - nTempPressure) < SEARCH_HEAT_BED_OFFSET_CONTACT_PRESSURE_DELTA) {
                if (g_scanRetries > 0) g_abortZScan = 0;
                g_retryZScan = 1;
            }
            Com::printF(PSTR("RevertPdelta:"), abs(nTempPressureUp - nTempPressure));
            Com::printFLN(PSTR("/"), SEARCH_HEAT_BED_OFFSET_CONTACT_PRESSURE_DELTA);

            if (g_scanRetries > 0 && g_retryZScan) {
                g_retryZScan = 0;
                g_scanRetries--;
                Com::printFLN(PSTR("Bettsuchproblem 10 -> 7 :"), g_scanRetries);
                g_nZOSScanStatus = 7;
                break;
            }

            // check for error
            if (g_abortZScan) {
                // will be set in case of error inside moveZMinusUpFast/Slow -> != 0 AFTER RETURN would temper with normal HBS-Scan function @ABORT
                Com::printFLN(PSTR("ERROR::Cannot find surface"));
                abortSearchHeatBedZOffset(false);
                break;
            }

            g_nZOSScanStatus = 20;
            break;
        }
        case 20: {
            Com::printFLN(PSTR("Testing Surface "));
            bool prebreak = false;
            // we have roughly found the surface, now we perform the precise slow scan SEARCH_HEAT_BED_OFFSET_SCAN_ITERATIONS times
            for (int i = 1; i <= SEARCH_HEAT_BED_OFFSET_SCAN_ITERATIONS; i++) {
                Com::printFLN(PSTR("10."), i);
                long Z = g_nZScanZPosition;

                // move from moveZMinusUpFast() down again -> für neuen anlauf
                moveZ(abs(g_nScanHeatBedUpSlowSteps)); // +++..
                Com::printFLN(PSTR("DownFine:"), (g_nZScanZPosition - Z));
                Z = g_nZScanZPosition;

                // move slowly to the surface
                short nTempPressure;
                moveZMinusUpSlow(&nTempPressure, 2); // -
                Com::printFLN(PSTR("UpFine:"), (g_nZScanZPosition - Z));
                Z = g_nZScanZPosition;

                // kraft zurückfahren
                g_nLastZScanZPosition = 0; //dont remember any old z-positions -> no check if deltaZ got too high.
                moveZPlusDownSlow(8);      // +
                Com::printFLN(PSTR("DownFine:"), (g_nZScanZPosition - Z));
                // messen
                Z = g_nZScanZPosition;

                // check for error / errorhandling
                if (g_scanRetries > 0 && g_retryZScan) {
                    g_retryZScan = 0;
                    g_scanRetries--;
                    Com::printFLN(PSTR("Suchproblem 20 -> 7:"), g_scanRetries);
                    g_nZOSScanStatus = 7;
                    prebreak = true;
                    break;
                }
                if (g_abortZScan) {
                    // will be set in case of error inside moveZMinusUpFast/Slow -> != 0 AFTER RETURN would temper with normal HBS-Scan function @ABORT
                    Com::printFLN(PSTR("ERROR::cannot find surface in slow scan"));
                    abortSearchHeatBedZOffset(false);
                    prebreak = true;
                    break;
                }

                // keep the minimum as the final result
                if (g_nZScanZPosition < g_min_nZScanZPosition)
                    g_min_nZScanZPosition = g_nZScanZPosition;

                // show height
                Com::printFLN(PSTR("Z = "), g_nZScanZPosition * Printer::axisMMPerSteps[Z_AXIS], 4);
            }
            if (prebreak)
                break;
            g_nZOSScanStatus = 50;
            break;
        }
        case 50: {
            // compute number of steps we need to shift the entire matrix by
            long matrixwert = getZMatrixDepth_CurrentXY();
            long nZ = g_min_nZScanZPosition - matrixwert; //g_ZCompensationMatrix[g_ZOSTestPoint[X_AXIS]][g_ZOSTestPoint[Y_AXIS]]; --> NEU: Mit Interpolation, also können wir überall scannen.

            Com::printFLN(PSTR("Matrix-Wert Z = "), matrixwert);
            Com::printF(PSTR("Minimum Z = "), g_min_nZScanZPosition);
            Com::printFLN(PSTR(" dZ = "), nZ);

            // update the matrix: shift by nZ and check for integer overflow
            bool overflow = false;
            bool overH = false;
            bool overnull = false;

            //Nibbels: scaling nZ according to learning Rate for additional corrective scans
            nZ = (long)((float)nZ * g_ZOSlearningRate);

            //Nibbels: weight change because of distance. lerne bettwinkelausgleich.
            float x_bed_len_quadrat = (float)((g_uZMatrixMax[X_AXIS] - 2) * (g_uZMatrixMax[X_AXIS] - 2)); //index zwischenabstamd x_n - x_0
            float y_bed_len_quadrat = (float)((g_uZMatrixMax[Y_AXIS] - 2) * (g_uZMatrixMax[Y_AXIS] - 2));
            float x_dist = 0;
            float y_dist = 0;
            float xy_weight = 0;
            long weighted_nZ = 0;
            long newValue = 0;

            for (short x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) {
                for (short y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
                    x_dist = (g_ZOSTestPoint[X_AXIS] - x) * (g_ZOSTestPoint[X_AXIS] - x) / x_bed_len_quadrat; //normierter indexabstand
                    y_dist = (g_ZOSTestPoint[Y_AXIS] - y) * (g_ZOSTestPoint[Y_AXIS] - y) / y_bed_len_quadrat; //normierter indexabstand
                    x_dist = sqrt(x_dist);                                                                    //Linearisieren. für Auto-Matrix-Leveling
                    y_dist = sqrt(y_dist);                                                                    //Linearisieren. für Auto-Matrix-Leveling
                    //das ist nur ein kreisabstand, wenn die messpunkte quadratisch angeordnet sind, ist aber nicht so?
                    // evtl. todo: achse faktor skalieren, sodass kreis x/y=(10/13)
                    xy_weight = 1 - sqrt(x_dist * x_dist + y_dist * y_dist); //linear gewichtet
                    if (xy_weight < 0.0)
                        xy_weight = 0;
                    if (xy_weight > 1.0)
                        xy_weight = 1.0; //kann aber nicht wirklich vorkommen.
                    weighted_nZ = (long)(g_ZOSlearningGradient * xy_weight * (float)nZ + (1.0 - g_ZOSlearningGradient) * (float)nZ);
                    newValue = (long)g_ZCompensationMatrix[x][y] + weighted_nZ;
                    if (newValue > 32767 || newValue < -32768)
                        overflow = true;
                    if (newValue > 0)
                        overnull = true; //darf nicht positiv werden. //darf nicht über limit sein.
                    if (newValue > (long(Printer::axisStepsPerMM[Z_AXIS] * g_scanStartZLiftMM) + g_nScanHeatBedUpFastSteps))
                        overH = true; //darf nicht positiv werden. //darf nicht über limit sein.
                    g_ZCompensationMatrix[x][y] = (short)newValue;
                }
            }
            // fail if overflow occurred
            if (overflow) {
                // load the unaltered compensation matrix from the EEPROM since the current in-memory matrix is invalid
                Com::printFLN(PSTR("Matrix Overflow!"));
                g_abortZScan = SCAN_ABORT_REASON_OVERFLOWING_MATRIX;
                abortSearchHeatBedZOffset(true);
                break;
            }
            // fail if z>(Starthöhe - ein bisschen) occurred
            if (overH) {
                // load the unaltered compensation matrix from the EEPROM since the current in-memory matrix is bigger than z=zero
                Com::printFLN(PSTR("ERROR::Z-Matrix höher Start-Z!"));
                g_abortZScan = SCAN_ABORT_REASON_BAD_HEIGHT_MATRIX;
                abortSearchHeatBedZOffset(true);
                break;
            }
            // fail if z>0 occurred
            if (overnull) {
                // load the unaltered compensation matrix from the EEPROM since the current in-memory matrix is bigger than z=zero
                Com::printFLN(PSTR("FIX::Clean Hotend-Nozzle"));
                Com::printFLN(PSTR("FIX::Fix Z-Schraube"));
            }
            // determine the minimal distance between extruder and heat bed
            determineCompensationOffsetZ();
            g_ZMatrixChangedInRam = 1; //man kan die matrix mit diesem marker nun sichern.

            g_nZOSScanStatus = 51;
            break;
        }
        case 51: {
            //HERE THE ZOS MIGHT REDO SCANS FOR AUTO_MATRIX_LEVELING
            switch (g_ZOS_Auto_Matrix_Leveling_State) {
            case 0: {
                g_nZOSScanStatus = 99; // No AUTO_MATRIX_LEVELING
                break;
            }
            default: {
                g_nZOSScanStatus = 2; // Goto next AUTO_MATRIX_LEVELING Setting
                g_ZOS_Auto_Matrix_Leveling_State++;
                moveZ(Printer::axisStepsPerMM[Z_AXIS]);
                Printer::homeAxis(true, true, false);
                moveZ(-g_nZScanZPosition); // g_nZScanZPosition counts z-steps. we need to move the heatbed down to be at z=0 again
                outputCompensationMatrix(1);
            }
            }
            break;
        }
        case 99: {
            moveZ(Printer::axisStepsPerMM[Z_AXIS]);
            Printer::homeAxis(false, true, false);
            moveZ(-g_nZScanZPosition); // g_nZScanZPosition counts z-steps. we need to move the heatbed down to be at z=0 again
            g_nZOSScanStatus = 100;    // No AUTO_MATRIX_LEVELING
            break;
        }
        case 100: {
            g_nZOSScanStatus = 0;
            calculateZScrewCorrection();

            showMyPage((void*)ui_text_heat_bed_zoffset_search_status, (void*)ui_text_heat_bed_zoffset_fix_z1, (void*)ui_text_heat_bed_zoffset_fix_z2, (void*)ui_text_statusmsg);
            g_nAutoReturnMessage = true;
            g_nAutoReturnTime = HAL::timeInMilliseconds() + 30000;
            g_uStartOfIdle = HAL::timeInMilliseconds() + 30000; //go to printer ready/ignore status   //end searchZOScan

            //g_nZScanZPosition is 0 now.
            Com::printFLN(PSTR("ZOS finished"));
            break;
        }
        }
    }
} // searchZOScan

void abortSearchHeatBedZOffset(bool reloadMatrix) {
    showAbortScanReason(PSTR(UI_TEXT_DO_MHIER_BED_SCAN), g_abortZScan);

    if (reloadMatrix) {
        if (loadCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed))) {
            // Error: there is no valid compensation matrix available
            initCompensationMatrix();
        }
    }
    g_abortZScan = 0;
    g_nZOSScanStatus = 0;
    g_retryZScan = 0;
    g_nLastZScanZPosition = 0;

    // the search has been aborted
    Com::printFLN(PSTR("ZOS aborted"));

    // move the heatbed 5mm down to avoid collisions, then home all axes
    Printer::queueRelativeMMCoordinates(0, 0, 5, 0, Printer::homingFeedrate[Z_AXIS], true, true);
    Printer::homeAxis(true, true, true);

    // turn off all steppers and extruders
    Printer::disableAllSteppersNow();

    g_ZOS_Auto_Matrix_Leveling_State = 0;
    g_uStartOfIdle = HAL::timeInMilliseconds() + 30000; //abort searchHeatBedZOffset
} /* searchHeatBedZOffset */

float g_ZSchraubenSollDrehungenWarm_U = 0;
float g_ZSchraubenSollKorrekturWarm_mm = 0;
char g_ZSchraubeOk = 0;

void calculateZScrewCorrection(void) {
    Com::printFLN(PSTR(" "));
    Com::printFLN(PSTR("Z-Schrauben-Helper: "));

    /*IDEAL ist es, wenn mand den Drucker "kalt" und frisch aufgeheizt einstellt.*/

    //Gerade eben muss ein Z-Scan die Matrix korrigiert haben.
    //Der Extruder darf sich seither nicht verändert (T0 -> T1) und nicht abgekühlt haben
    //Dann wird pro °C des aktiven Extruders ~0.001mm Längung angenommen.
    //Dann wird pro °C des Heizbettes ~0.0015mm Längung angenommen.

    //Wegen der unbekannten Nachlängung werden 0.15mm Puffer auf Z=0 angestrebt.
    //Einfach so: wird ein Puffer von 0.05 angestrebt. Die ideale Matrix-Verschiebung wird auf 260°C Hotend / 120°C Bett, was ein Extremwert darstellen soll auf -0.05mm angepeilt.

    //Config
    float maxExtruderTemperature = (float)EXTRUDER_MAX_TEMP;
    float maxBedTemperature = 120.0f; //120°C ist ok... mit 180 zu rechnen wäre übertrieben.
    float BedThermalExplansionInMikrons = 1.5f;
    float ExtruderThermalExpansionInMikrons = 0.95f;
    float maxNachdehnungInMikrons = 150.0f;
    float RestAbstandInMikrons = 50.0f;

    //Ist-Abstand.
    float MatrixMaximumInMikrons = (float)g_offsetZCompensationSteps * Printer::axisMMPerSteps[Z_AXIS] * 1000.0f;

    float ExtruderTemperature = Extruder::current->tempControl.currentTemperatureC;
    if (ExtruderTemperature < 20.0f)
        ExtruderTemperature = 20.0f; //Wenn zu kalt oder undefiniert dann Standardbedingungen annehmen.

    float BedTemperature = Extruder::getHeatedBedTemperature();
    if (BedTemperature == -1)
        maxBedTemperature = BedTemperature = 20.0f; //Wenn kein Heated-Bed dann Standardbedingungen annehmen.

    //Umrechnung des aktuellen Zustandes auf die heißesten Werte:
    float MinDistanceInMikronsKalt = MatrixMaximumInMikrons
        + (maxExtruderTemperature - ExtruderTemperature) * ExtruderThermalExpansionInMikrons
        + (maxBedTemperature - BedTemperature) * BedThermalExplansionInMikrons;
    float MinDistanceInMikronsWarm = MinDistanceInMikronsKalt + maxNachdehnungInMikrons;

    Com::printFLN(PSTR("- Alle Werte in Mikrometern / Einheit [um] -"));
    Com::printFLN(PSTR("Matrix-Minimum: "), MatrixMaximumInMikrons);
    Com::printFLN(PSTR("Weitere Extruderausdehnung maximal: "), (maxExtruderTemperature - ExtruderTemperature) * ExtruderThermalExpansionInMikrons);
    Com::printFLN(PSTR("Weitere Heizbettausdehnung maximal: "), (maxBedTemperature - BedTemperature) * BedThermalExplansionInMikrons);
    Com::printFLN(PSTR("Maximalwert-zMatrix bei Maximaltemperaturen (kalter Drucker): "), MinDistanceInMikronsKalt);
    Com::printFLN(PSTR("Maximalwert-zMatrix bei Maximaltemperaturen (durchgewaermter Drucker): "), MinDistanceInMikronsWarm);

    //z.B. -200 <-- um diesen Wert dürfte man korrigieren, wenn der Drucker zum Messzeitpunkt voll durchgewärmt wäre. Weiß er aber nicht!
    //Ein vorgewärmter Drucker justiert das Heizbett eher auf -0.2, ein kalter Drucker justiert es eher auf -0.05 bei Spitzentemperaturen. Beides ist ok.
    //float SollkorrekturKalt = (MinDistanceInMikronsKalt + RestAbstandInMikrons);
    //z.B.  -50 <-- diesen Wert darf man in jedem Fall korrigieren.
    float SollkorrekturWarm = (MinDistanceInMikronsWarm + RestAbstandInMikrons);

    /*
    Wenn ich den Test mit einem bereits warmen Drucker mache, plane ich unnötig eine Sicherheit ein, die ich nur einrechne, weil der Drucker aktuell kalt sein könnte.
    Also warmer Drucker: Bett weiter hoch justieren, also Schraube weiter rein, also Drehsinn Minus.
    */
    // |+0-|..Puffer..|.......Nachdehnung........|Kalt-Soll-Einstellung|
    // |+0-|..Puffer..|Warm-Soll-Einstellung|

    /* TIPP: -> Schraube bis maxNachdehnungInMikrons ~ 150um weiter runter(=Bett weiter hoch =Drehsinn Minus) empfehlen, wenn der Drucker aktuell "mehr druchgewärmt" wäre. */

    Com::println();
    Com::printSharpLine();
    Com::printF(PSTR("Sollkorrektur: "), SollkorrekturWarm, 0);
    Com::printF(PSTR(" [um] = "), SollkorrekturWarm * 0.001f, 3);
    Com::printFLN(PSTR(" [mm]"));

    //meldung:
    g_ZSchraubenSollKorrekturWarm_mm = SollkorrekturWarm * 0.001f;

    float ZSchraubenDrehungenWarm = SollkorrekturWarm * 0.002f; //[Sollkorrektur in mm] geteilt durch [Gewinde: 0.5 mm/Umdrehung] -> Sollkorrektur / 1000 / 0.5
    Com::printF(PSTR("Sollumdrehungen: "), ZSchraubenDrehungenWarm, 1);
    Com::printF(PSTR(" [U] = "), ZSchraubenDrehungenWarm * 360, 0);
    Com::printF(PSTR(" [Grad]"));

    //meldung:
    g_ZSchraubenSollDrehungenWarm_U = ZSchraubenDrehungenWarm;

    if (ZSchraubenDrehungenWarm > 0)
        Com::printFLN(PSTR(" (+ heisst rausdrehen/linksrum/gegen die Uhr)")); //Bett wird nach unten justiert
    else
        Com::printFLN(PSTR(" (- heisst reindrehen/rechtsrum/im Uhrzeigersinn)")); //Bett wird nach oben justiert

    Com::printFLN(PSTR("Je kaelter der Gesamtdrucker aktuell ist (nach langer Pause frisch angeschaltet), desto besser der Korrekturwert."));
    g_ZSchraubeOk = -1;                                                             //neg -> Matrix negativ -> ok. ausser, wenn:
#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2        //TODO: Prüfen ob das beim RF2000v2 stimmen wird.
    if (-0.5f <= ZSchraubenDrehungenWarm && SollkorrekturWarm < 40.0f /* [um] */) { // < 0.25mm = 0.5Umdrehungen ist mit dem RF2000 nicht machbar.
        Com::printFLN(PSTR(" (Die Z-Schraube ist ok!)"));                           //das ist die Änderung in M3-Regelgewinde-Z-Schrauben-Umdrehungen
        //meldung:
        g_ZSchraubeOk = -1;                                                                        //neg
    } else if (SollkorrekturWarm >= 40.0f /* [um] */) {                                            //dann bin ich rechnerisch um Z = 0 (50um mit 10um toleranz, die ich fordere.)
                                                                                                   //eine korrektur von mehr als +40um heißt, ich bin gerade vermutlich im Z>0
        Com::printFLN(PSTR(" (Die Z-Schraube weiter raus! Das Bett scheint zu hoch zu liegen.)")); //das ist die Änderung in M3-Regelgewinde-Z-Schrauben-Umdrehungen
        //meldung:
        g_ZSchraubeOk = 1; //pos
    }
    Com::printFLN(PSTR(" (RF2000: Minimal eine halbe Schraubendrehung (dZ=0.25mm-Schritte) einstellbar.)")); //das ist die Änderung in M3-Regelgewinde-Z-Schrauben-Umdrehungen
#else                                                                                                        //if MOTHERBOARD == DEVICE_TYPE_RF1000
    if (SollkorrekturWarm >= 40.0f /* [um] */) { //dann bin ich rechnerisch um Z = 0 (50um mit 10um toleranz, die ich fordere.)
        //eine korrektur von mehr als +40um heißt, ich bin gerade vermutlich im Z>0
        Com::printFLN(PSTR(" (Die Z-Schraube weiter raus! Das Bett scheint zu hoch zu liegen.)")); //das ist die Änderung in M3-Regelgewinde-Z-Schrauben-Umdrehungen
        //meldung:
        g_ZSchraubeOk = 1; //pos
    }
#endif
    Com::printSharpLine();
} // calculateZScrewCorrection

/**************************************************************************************************************************************/

/**************************************************************************************************************************************/

void fixKeramikLochInMatrix(void) {
    //Com::printFLN( PSTR( "fixKeramikLochInMatrix(): 1 Init" ) );

    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
        // we load the z compensation matrix before its first usage because this can take some time
        prepareZCompensation();
    }
    if (g_ZCompensationMatrix[0][0] == EEPROM_FORMAT) {
        // search for deepest hole in bed-z-matrix and fix it according to surrounding values

        long peak_hole = 0;
        long peak_x = 0;
        long peak_y = 0;

        long deepness = 0;
        long heights = 0;
        char div = 0;

        for (short x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) { //in der matrix ist alles von 1 an (?) -> also vermutlich anzahl = indexmax.
            for (short y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {

                heights = 0;
                div = 0;

                //circle around matrixposition
                for (short xx = x - 1; xx <= x + 1; xx++) {
                    for (short yy = y - 1; yy <= y + 1; yy++) {                                                     //iterate all points
                        if (xx != x || yy != y) {                                                                   //nicht den punkt in der mitte
                            if (xx <= g_uZMatrixMax[X_AXIS] && xx >= 1 && yy <= g_uZMatrixMax[Y_AXIS] && yy >= 1) { //nicht punkte ausserhalb der matrix
                                heights += (long)g_ZCompensationMatrix[xx][yy];
                                div += 1;
                            }
                        }
                    }
                }
                deepness = (long)((float)heights / div) - g_ZCompensationMatrix[x][y]; //nur täler, negative werte.
                if (deepness > peak_hole && div > 3) {                                 //nicht an ecken, sonst immer das tiefste loch suchen.
                    peak_hole = deepness;
                    peak_x = x;
                    peak_y = y;
                }
            }
        }

        //Com::printFLN( PSTR( "fixKeramikLochInMatrix(): 3 Extremwert" ) );
        //Com::printF( PSTR( "peak_x = " ), peak_x );
        //Com::printF( PSTR( "; peak_y = " ), peak_y );
        //Com::printF( PSTR( "; peak_hole = " ), peak_hole );
        //Com::printFLN( PSTR( "g_ZCompensationMatrix[peak_x,peak_y] =" ), g_ZCompensationMatrix[peak_x][peak_y] );

        if (peak_hole > 100 && peak_x > 0 && peak_y > 0) {
            //loch groß genug
            g_ZCompensationMatrix[peak_x][peak_y] += peak_hole;
            Com::printF(PSTR("Fixed: [peak_x,peak_y]="), g_ZCompensationMatrix[peak_x][peak_y]);
            g_ZMatrixChangedInRam = 1;
        } else {
            Com::printF(PSTR("Fix not needed dh<100 dh="), peak_hole);
        }

    } else {
        Com::printFLN(Com::tError);
    }

    determineCompensationOffsetZ(); //zur sicherheit und zukünftiger kompatibilität (Popel rausrechnen, würde den minimalen Z-Abstand verändern.)

    //Com::printFLN( PSTR( "fixKeramikLochInMatrix(): finished" ) );
    return;
} // fixKeramikLochInMatrix

/**************************************************************************************************************************************/

/**************************************************************************************************************************************/

void setMatrixNull(void) {
    //Damit kann die Matrix eingeebnet werden. Anschließend mit M3902 Zx.x verschoben, dann mit dem M3900/M3901 mit Distance-Schalter ein Auto-Bed-Leveling auf die Alte Art gemacht werden.
    //Man hätte dann eine total ebene Matrix. Die Z-Kompensation ist dann faktisch aus, kann aber weiterlaufen.

    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
        // we load the z compensation matrix before its first usage because this can take some time
        prepareZCompensation();
    }
    if (g_ZCompensationMatrix[0][0] == EEPROM_FORMAT) {
        for (short x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) { //in der matrix ist alles von 1 an (?) -> also vermutlich anzahl = indexmax.
            for (short y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
                g_ZCompensationMatrix[x][y] = 0;
            }
        }
        g_ZMatrixChangedInRam = 1; //man kan die matrix mit diesem marker nun sichern.
        Com::printFLN(PSTR("Matrix set to 0!"));
    } else {
        Com::printFLN(Com::tError);
    }
    determineCompensationOffsetZ(); //zur sicherheit und zukünftiger kompatibilität (Popel rausrechnen, würde den minimalen Z-Abstand verändern.)
    return;
} // setMatrixNull

/**************************************************************************************************************************************/

/**************************************************************************************************************************************/
/**************************************************************************************************************************************/

/**************************************************************************************************************************************/
/* Diese Funktion soll die Step_Size eines Z-Steps einstellbar machen. */
/* Noch ist das statisch über eine Tabelle, aber man könnte das später auch berechnen */

void configureMANUAL_STEPS_Z(int8_t increment) {
    const unsigned short stepsize_table[NUM_ACCEPTABLE_STEP_SIZE_TABLE] PROGMEM = ACCEPTABLE_STEP_SIZE_TABLE;

    int loop = 0;
    //suche die aktuelle Einstellungsposition (oder aufgerundet) in der Tabelle:
    for (loop = 0; loop < NUM_ACCEPTABLE_STEP_SIZE_TABLE; loop++)
        if (stepsize_table[loop] >= g_nManualSteps[Z_AXIS])
            break;

    //ändere die Position in die Wunschposition:
    if (increment >= 0)
        loop += 1;
    else
        loop -= 1;

    //begrenze ringmenge
    if (loop > NUM_ACCEPTABLE_STEP_SIZE_TABLE - 1)
        loop = 0;
    else if (loop < 0)
        loop = NUM_ACCEPTABLE_STEP_SIZE_TABLE - 1;

    //nutze neuen Wert:
    g_nManualSteps[Z_AXIS] = stepsize_table[loop];

    HAL::eprSetInt16(EPR_RF_MOD_Z_STEP_SIZE, g_nManualSteps[Z_AXIS]);
    EEPROM::updateChecksum();
} // configureMANUAL_STEPS_Z()
/**************************************************************************************************************************************/

short testExtruderTemperature(void) //Funktion ist speziell für HBS Scan.
{
    if (Extruder::current->tempControl.targetTemperatureC > MAX_ROOM_TEMPERATURE) {
        // we have to wait until the target temperature is reached
        if ((Extruder::current->tempControl.currentTemperatureC + TEMP_TOLERANCE) < Extruder::current->tempControl.targetTemperatureC) {
            // wait until the extruder has reached its target temperature
            UI_STATUS_UPD(UI_TEXT_HEATING_UP);
            return -1;
        }
        if ((Extruder::current->tempControl.currentTemperatureC - TEMP_TOLERANCE) > Extruder::current->tempControl.targetTemperatureC) {
            // wait until the extruder has reached its target temperature
            UI_STATUS_UPD(UI_TEXT_COOLING_DOWN);
            return -1;
        }
        /* else (inside temp_tolerance), return 0, dont wait anymore */
    } else {
        // we have to wait until the current temperatur is below something which would be too warm
        if (Extruder::current->tempControl.currentTemperatureC > MAX_ROOM_TEMPERATURE + 20 /*orig: 65+*/) {
            // wait until the extruder has reached its target temperature
            UI_STATUS_UPD(UI_TEXT_COOLING_DOWN);
            return -1;
        }
    }

    // at this point we have reached the proper temperature
    return 0;

} // testExtruderTemperature

short testHeatBedTemperature(void) //Funktion ist speziell für HBS Scan.
{
#if HAVE_HEATED_BED
    if (heatedBedController.targetTemperatureC > (MAX_ROOM_TEMPERATURE >= PRECISE_HEAT_BED_SCAN_BED_TEMP_PLA - TEMP_TOLERANCE
                                                      ? PRECISE_HEAT_BED_SCAN_BED_TEMP_PLA - TEMP_TOLERANCE
                                                      : MAX_ROOM_TEMPERATURE))
    /* Evtl. sollte man abfangen, fallst die MAX_ROOM_TEMPERATURE hier höher als die PLA-Scan-TEMP eingestellt ist. Darum der Ternary */
    {
        // we have to wait until the target temperature is reached
        if ((Extruder::getHeatedBedTemperature() + TEMP_TOLERANCE) < heatedBedController.targetTemperatureC) {
            // wait until the heat bed has reached its target temperature
            UI_STATUS_UPD(UI_TEXT_HEATING_UP);
            return -1;
        }
        if ((Extruder::getHeatedBedTemperature() - TEMP_TOLERANCE) > heatedBedController.targetTemperatureC) {
            // wait until the heat bed has reached its target temperature
            UI_STATUS_UPD(UI_TEXT_COOLING_DOWN);
            return -1;
        }
        /* else (inside temp_tolerance), return 0, dont wait anymore */
    } else {
        // we have to wait until the current temperatur is below something which would be too warm
        if (Extruder::getHeatedBedTemperature() > MAX_ROOM_TEMPERATURE + 10 /*orig: 50+*/) {
            // wait until the heat bed has reached its target temperature
            UI_STATUS_UPD(UI_TEXT_COOLING_DOWN);
            return -1;
        }
    }
#endif // HAVE_HEATED_BED

    // at this point we have reached the proper temperature
    return 0;
} // testHeatBedTemperature

long getZMatrixDepth(long x, long y) {
    // find the rectangle which covers the current position of the extruder
    unsigned char nXLeftIndex = 1;
    long nXLeftSteps = (long)((float)g_ZCompensationMatrix[1][0] * Printer::axisStepsPerMM[X_AXIS]);
    unsigned char nXRightIndex = 0;
    long nXRightSteps = 0;
    long i;
    long nTemp;

    for (i = 1; i <= g_uZMatrixMax[X_AXIS]; i++) {
        nTemp = g_ZCompensationMatrix[i][0];
        nTemp = (long)((float)nTemp * Printer::axisStepsPerMM[X_AXIS]);
        if (x <= nTemp) {
            nXRightIndex = i;
            nXRightSteps = nTemp;
            break;
        }
        nXLeftIndex = i;
        nXLeftSteps = nTemp;
    }

    unsigned char nYFrontIndex = 1;
    long nYFrontSteps = (long)((float)g_ZCompensationMatrix[0][1] * Printer::axisStepsPerMM[Y_AXIS]);
    unsigned char nYBackIndex = 0;
    long nYBackSteps = 0;

    for (i = 1; i <= g_uZMatrixMax[Y_AXIS]; i++) {
        nTemp = g_ZCompensationMatrix[0][i];
        nTemp = (long)((float)nTemp * Printer::axisStepsPerMM[Y_AXIS]);
        if (y <= nTemp) {
            nYBackIndex = i;
            nYBackSteps = nTemp;
            break;
        }
        nYFrontIndex = i;
        nYFrontSteps = nTemp;
    }

    long nDeltaX = x - nXLeftSteps;
    long nDeltaY = y - nYFrontSteps;
    long nStepSizeX = nXRightSteps - nXLeftSteps;
    long nStepSizeY = nYBackSteps - nYFrontSteps;

    // we do a linear interpolation in order to find our exact place within the current rectangle
    long nTempXFront = g_ZCompensationMatrix[nXLeftIndex][nYFrontIndex] + (g_ZCompensationMatrix[nXRightIndex][nYFrontIndex] - g_ZCompensationMatrix[nXLeftIndex][nYFrontIndex]) * nDeltaX / nStepSizeX;
    long nTempXBack = g_ZCompensationMatrix[nXLeftIndex][nYBackIndex] + (g_ZCompensationMatrix[nXRightIndex][nYBackIndex] - g_ZCompensationMatrix[nXLeftIndex][nYBackIndex]) * nDeltaX / nStepSizeX;

    long ZMatrixDepth = nTempXFront + (nTempXBack - nTempXFront) * nDeltaY / nStepSizeY; //Das ist hier noch der Zeiger auf die Oberfläche.

    return ZMatrixDepth;
}

long getZMatrixDepth_CurrentXY(void) {
    // We need the current queue steps here. No Direct, No Offsets.
    // We need to insert where the nozzle is, relative to 0 homing coordinates which are always min endstop because we dropped axis offsets.
    // Because our RF2000 Matrix is 180mm width. That is the center of the bed. left and right are spaces. -> Center buildarea starts at 0 gcode coordinates and thats what we want.
    return getZMatrixDepth(
        Printer::currentSteps[X_AXIS],
        Printer::currentSteps[Y_AXIS]);
}

void recalculateHeatBedZCompensation(void) {
    long nNeededZCompensation = 0;
    float nNeededZEPerc = 0.0f;

    if (Printer::doHeatBedZCompensation) {
        long nCurrentPositionStepsZ = Printer::currentSteps[Z_AXIS]; //direct muss hier egal sein, das ist nur das extruder und sonstige manual-offset.

        // Der Z-Kompensation wird das extruderspezifische Z-Offset des jeweiligen Extruders verschwiegen, sodass dieses die Höhen / Limits nicht beeinflusst. Die X- und Y-Offsets werden behalten, denn das korrigiert Düsen- zu Welligkeitsposition nach Extruderwechsel. Das extruderspezifische Z-Offset Extruder::current->zOffset wird beim Toolchange in nCurrentPositionStepsZ eingerechnet und verfahren.
        // Extruder::current->zOffset ist negativ, wenn das hotend weiter heruntergedrückt werden kann als 0. -> Bettfahrt nach unten, um auszuweichen.
        if (nCurrentPositionStepsZ >= 0) {
            // check whether we have to perform a compensation in z-direction
            if (nCurrentPositionStepsZ < g_maxZCompensationSteps) {
                nNeededZCompensation = getZMatrixDepth_CurrentXY(); //Das ist hier der Zeiger auf den interpolierten Z-Matrix-Wert.

                if (nCurrentPositionStepsZ <= g_minZCompensationSteps) {
                    // the printer is very close to the surface - we shall print a layer of exactly the desired thickness
                    if (nCurrentPositionStepsZ == 0) {
                        nNeededZCompensation += 13; //G1 Z0 shall not hit the bed: +5um -> this is better than not compensating at all because it makes tests weired.
                    }
                    //nNeededZEPerc = 0.0f;
                } else {
                    long zl;
                    long div;
                    // Compensate Extrusion within ZCMP: Add and sum up this amount for every step extruded in move interrupt. Extrude the step if we have some full >=1.0 in interrupt.
                    if (Printer::queuePositionZLayerLast < g_minZCompensationSteps && g_minZCompensationSteps < Printer::queuePositionZLayerCurrent) {
                        //geschmälert um Anteil innerhalb CMP-Ausschleichbereich:
                        zl = (g_offsetZCompensationSteps - nNeededZCompensation) * (Printer::queuePositionZLayerCurrent - g_minZCompensationSteps);
                        div = (g_maxZCompensationSteps - g_minZCompensationSteps) * (Printer::queuePositionZLayerCurrent - Printer::queuePositionZLayerLast);
                    } else {
                        zl = (g_offsetZCompensationSteps - nNeededZCompensation);
                        div = (g_maxZCompensationSteps - g_minZCompensationSteps);
                    }
                    if (div && zl) {
                        nNeededZEPerc = float(zl) / float(div);
                    } else {
                        nNeededZEPerc = 0.0f;
                    }
                    // the printer is already a bit away from the surface - do the actual compensation -> Hier ist nNeededZCompensation dann nicht mehr der Zeiger auf die Oberfläche, sondern der Zeiger auf die kompensationshöhe:
                    nNeededZCompensation = ((nNeededZCompensation - g_offsetZCompensationSteps) * (g_maxZCompensationSteps - nCurrentPositionStepsZ))
                        / (g_maxZCompensationSteps - g_minZCompensationSteps);
                    nNeededZCompensation += g_offsetZCompensationSteps;
                }
            } else {
                // after the first layers, only the static offset to the surface must be compensated
                nNeededZCompensation = g_offsetZCompensationSteps;

                if (Printer::queuePositionZLayerLast < g_maxZCompensationSteps && g_maxZCompensationSteps <= Printer::queuePositionZLayerCurrent) {
                    //Volle E-Kompensation: geschmälert um Anteil innerhalb CMP-Ausschleichbereich:
                    long zl = (g_offsetZCompensationSteps - getZMatrixDepth_CurrentXY()) * (g_maxZCompensationSteps - Printer::queuePositionZLayerLast);
                    long div = (g_maxZCompensationSteps - g_minZCompensationSteps) * (Printer::queuePositionZLayerCurrent - Printer::queuePositionZLayerLast);
                    if (div && zl) {
                        nNeededZEPerc = float(zl) / float(div);
                    } else {
                        nNeededZEPerc = 0.0f;
                    }
                    /*}else{
                        nNeededZEPerc = 0.0f;*/
                }
            }
        } else {
            //Gcode Z < 0 soll 5um überhalb des top matrix elements bleiben: diese anweisungen sind generell für uns sinnlos bzw. schädlich.
            nNeededZCompensation = g_offsetZCompensationSteps + 13;
            Printer::compensatedPositionOverPercE = 0.0f;
        }

        nNeededZCompensation += g_staticZSteps;

        //Etwa 5500 digits verursachen 0.05 mm tiefere nozzle: ca. 0.00001 = 1/100.000 mm pro digit.
        //0.00001 ist vermutlich konservativ bis ok.
        //Je höher die Kraft nach unten, desto mehr muss das Bett ausweichen: Z nach oben/+.

        //VORSICHT: Die Messzellen könnten falsch verbaut sein, darum Digits immer positiv nutzen. Negative Digits würden sowieso in die falsche Richtung tunen. Ein kleiner Versatz der Nullposition wäre beim Druck egal.
        //long nNeededDigitZCompensationSteps = (long)(fabs(g_nDigitZCompensationDigits) * (float)Printer::axisStepsPerMM[Z_AXIS] * 0.00001f);
        if (g_nDigitZCompensationDigits_active) {
            long nNeededDigitZCompensationSteps = abs((long)(g_nDigitZCompensationDigits * (float)Printer::axisStepsPerMM[Z_AXIS]));
            nNeededDigitZCompensationSteps >>= 17; // geteilt durch 131072 statt errechnet ca. 110000 .. wäre 18% überkompensiert aber verdammt schnell gerechnet. evtl. ist überkompensation nicht so schlecht... höhere digits höhere schwankungen, das ist sowieso nur eine ganz grob vermessene kompensation, etwas mehr platz kann für die digits eine dämpfende wirkung haben.
            //sign-extension kann hier, da positiv kein problem sein.: unsigned(x) >> y
            //g_nDigitZCompensationDigits -> max. 32768, axisStepsPerMM[Z_AXIS] -> ca. 2560, also passts in long. -> ca. max 640 steps.
            nNeededDigitZCompensationSteps = constrain(nNeededDigitZCompensationSteps, 0, 600);

            nNeededZCompensation += nNeededDigitZCompensationSteps;
            /*
            }else{
                //wie bisher ohne additionszusatz
            */
        }

    } else {
        // Nicht zCMP Offsets nutzen, wenn nur gehomed, weil das alle Scans stören kann, nur weil eine falsche Matrix geladen ist. Security hin oder her.
    }

    InterruptProtectedBlock noInts;
    Printer::compensatedPositionTargetStepsZ = nNeededZCompensation;
    Printer::compensatedPositionOverPercE = nNeededZEPerc;
    noInts.unprotect();
} // recalculateHeatBedZCompensation

void showAbortScanReason(const void* scanName, char abortScanIdentifier) {
    switch (abortScanIdentifier) {
        // Geplantes Canceling
    case SCAN_ABORT_REASON_CANCELED: {
        showError(scanName, PSTR(UI_TEXT_SCAN_ABORTED));
        break;
    }
    case SCAN_ABORT_REASON_PRINTER_LONELY: {
        showError(scanName, PSTR("Unoccupied"), PSTR(UI_TEXT_SCAN_ABORTED));
        break;
    }

    // Kabelbruch, Extruder verspannt, Stecker wackelig, Ingress,...
    case SCAN_ABORT_REASON_START_PRESSURE: {
        showError(scanName, PSTR("Bad start force"));
        break;
    }
    case SCAN_ABORT_REASON_IDLE_PRESSURE: {
        showError(scanName, PSTR("Bad idle force"));
        break;
    }
    case SCAN_ABORT_REASON_AVERAGE_PRESSURE: {
        showError(scanName, PSTR("Bad force signal"));
        break;
    }

    // Hinweise auf Bugs
    case SCAN_ABORT_REASON_SCAN_OVER_START: {
        showError(scanName, PSTR("Bug SOS"));
        break;
    }
    case SCAN_ABORT_REASON_MATRIX_DIMENSION: {
        showError(scanName, PSTR("Bug MDO"));
        break;
    }

    // Löcher im Druckbett, Druckbett verdreckt, Mechanik problem, DDP schlecht geklebt
    case SCAN_ABORT_REASON_SCAN_TOO_UNEVEN: {
        showError(scanName, PSTR("Uneven bed, dirt"), PSTR("hole, bad ground"));
        break;
    }
    case SCAN_ABORT_REASON_SCAN_STEP_DELTA_OUT_OF_RANGE: {
        // Fehler trat auf, als ich die Düse beim HBS ein Blatt Papier mitschleppen hab lassen.
        // Marius1 hatte den Fehler durch ein sehr schräg eingebautes Bett.
        showError(scanName, PSTR("Leftovers, snot"), PSTR("incrustation"));
        break;
    }

    // Z-Schraube falsch eingestellt?
    // _MAX_COMPENSATION und _ENDSTOP_PROTECTION deuten im Printermodus auf dasselbe hin, sind aber 2 unterschiedliche Stellen.
    case SCAN_ABORT_REASON_MIN_ENDSTOP: {
        showError(scanName, PSTR("Min Endstop"));
        break;
    }
    case SCAN_ABORT_REASON_MAX_ENDSTOP: {
        showError(scanName, PSTR("Max Endstop"));
        break;
    }
    case SCAN_ABORT_REASON_REACHED_MAX_COMPENSATION: {
        showError(scanName, PSTR("Reached limit"), PSTR("max compensation"));
        break;
    }
    case SCAN_ABORT_REASON_REACHED_ENDSTOP_PROTECTION: {
        showError(scanName, PSTR("Reached endstop"), PSTR("protection limit"));
        break;
    }

    case SCAN_ABORT_REASON_MISSING_MATRIX: {
        showError(scanName, PSTR("Missing or"), PSTR(UI_TEXT_INVALID_MATRIX));
        break;
    }
    case SCAN_ABORT_REASON_OVERFLOWING_MATRIX: {
        showError(scanName, PSTR("Correction would"), PSTR("overflow matrix"));
        break;
    }
    case SCAN_ABORT_REASON_BAD_HEIGHT_MATRIX: {
        // Matrix hat einen Punkt über Z-Start. Das ist gefährlich hoch, weil die Nozzle theoretisch beim Fahren auf position kollidiert haben könnte.
        // Eine rein positive Matrix melden wir nicht, das ist egal, sofern die Starthöhe immer höher ist. Dafür gibts im Anschluss des Scans die genaue Meldung zur Schraubeneinstellung.
        showError(scanName, PSTR("Badly wrong"), PSTR("Z-Screw/Z-Start"));
        break;
    }
    default:
        showError(scanName, PSTR(UI_TEXT_FAIL));
    }
}

short readIdlePressure(short* pnIdlePressure) {
    short nTempPressure;
    char nTemp;

    // determine the pressure when the heat bed is far away - wait until the measured pressure is rather stable
    nTempPressure = 0;
    if (readAveragePressure(pnIdlePressure)) {
        // we were unable to determine the pressure
        if (Printer::debugErrors()) {
            Com::printFLN(PSTR("readIdlePressure(): the pressure could not be determined"));
        }
        return -1;
    }

    nTemp = 0;
    while (abs(nTempPressure - *pnIdlePressure) > 5) {
        if (Printer::debugInfo()) {
            Com::printF(PSTR("readIdlePressure(): pressure calibration: "), nTempPressure);
            Com::printFLN(PSTR(" / "), *pnIdlePressure);
        }

        nTemp++;
        if (nTemp >= 5) {
            // we are unable to receive stable values - do not hang here forever
            if (Printer::debugErrors()) {
                Com::printF(PSTR("readIdlePressure(): the pressure is not constant: "), nTempPressure);
                Com::printFLN(PSTR(" / "), *pnIdlePressure);
            }
            return -1;
        }

        nTempPressure = *pnIdlePressure;
        if (readAveragePressure(pnIdlePressure)) {
            // we were unable to determine the pressure
            if (Printer::debugErrors()) {
                Com::printFLN(PSTR("readIdlePressure(): the pressure could not be determined"));
            }
            return -1;
        }

        // wait some extra amount of time in case our results were not constant enough
        HAL::delayMilliseconds(500);

        Commands::checkForPeriodicalActions(Processing);
    }

    if (Printer::debugInfo()) {
        Com::printFLN(PSTR("readIdlePressure(): idle pressure: "), *pnIdlePressure);
    }

    if (*pnIdlePressure < g_nScanIdlePressureMin || *pnIdlePressure > g_nScanIdlePressureMax) {
        // the idle pressure is out of range
        if (Printer::debugErrors()) {
            Com::printFLN(PSTR("readIdlePressure(): the idle pressure is out of range"));
        }
        return -1;
    }

    // at this point we know the idle pressure
    return 0;
} // readIdlePressure

short testIdlePressure(void) {
    short nTempPressure;
    if (readAveragePressure(&nTempPressure)) {
        return -1; // some error has occurred
    }
    g_nCurrentIdlePressure = nTempPressure;
    return 0;
} // testIdlePressure

short readAveragePressure(short* pnAveragePressure) {
    short i;
    short nTempPressure;
    short nMinPressure;
    short nMaxPressure;
    long nPressureSum;
    char nTemp;

    nTemp = 0;
    while (1) {
        // we read the strain gauge multiple times and check the variance
        nPressureSum = 0;
        nMinPressure = 32000;
        nMaxPressure = -32000;
        for (i = 0; i < g_nScanPressureReads; i++) {
            HAL::delayMilliseconds(g_nScanPressureReadDelay);
            nTempPressure = readStrainGauge(ACTIVE_STRAIN_GAUGE);
            nPressureSum += nTempPressure;
            if (nTempPressure < nMinPressure)
                nMinPressure = nTempPressure;
            if (nTempPressure > nMaxPressure)
                nMaxPressure = nTempPressure;
        }
        nTempPressure = (short)(nPressureSum / g_nScanPressureReads);

        if ((nMaxPressure - nMinPressure) < g_nScanPressureTolerance) {
            // we have good results
            *pnAveragePressure = nTempPressure;

            return 0;
        }

        Commands::checkForPeriodicalActions(Processing);

        nTemp++;
        if (nTemp >= 5) {
            // we are unable to receive stable values - do not hang here forever
            Com::printF(PSTR("readAveragePressure(): the pressure is not constant: "), nMinPressure);
            Com::printF(PSTR(" / "), nTempPressure);
            Com::printFLN(PSTR(" / "), nMaxPressure);
            break;
        }

        // wait some extra amount of time in case our results were not constant enough
        HAL::delayMilliseconds(100);
        Commands::checkForPeriodicalActions(Processing);
    }

    Com::printFLN(PSTR("readAveragePressure(): the pressure is not plausible"));
    *pnAveragePressure = 0;

    return -1;
} // readAveragePressure

//Spacing Schnell:
short moveZPlusDownFast() {
    short nTempPressure;

    // move the heat bed down so that we won't hit it when we move to the next position
    g_nLastZScanZPosition = g_nZScanZPosition;
    HAL::delayMilliseconds(g_nScanFastStepDelay);

    moveZ(g_nScanHeatBedDownFastSteps);

    Commands::checkForPeriodicalActions(Processing);

    if (readAveragePressure(&nTempPressure)) {
        return -1;
    }

#if DEBUG_HEAT_BED_SCAN
    if (Printer::debugInfo())
        Com::printFLN(PSTR("moveZPlusDownFast(): "), (int)nTempPressure);
#endif // DEBUG_HEAT_BED_SCAN
    return 0;
} // moveZPlusDownFast

//Spacing Langsam:
void moveZPlusDownSlow(uint8_t acuteness) {
    short nTempPressure;
    long startScanZPosition = g_nZScanZPosition;

    // move the heat bed down until we detect the retry pressure (slow speed)
    while (1) {
        HAL::delayMilliseconds(g_nScanSlowStepDelay);
        if (readAveragePressure(&nTempPressure)) {
            // some error has occurred
            g_abortZScan = SCAN_ABORT_REASON_AVERAGE_PRESSURE;
            break;
        }

        if (nTempPressure < g_nMaxPressureRetry && nTempPressure > g_nMinPressureRetry) {
            // we have reached the target pressure
            break;
        }

        moveZ((g_nScanHeatBedDownSlowSteps / acuteness ? g_nScanHeatBedDownSlowSteps / acuteness : 1));

        Commands::checkForPeriodicalActions(Processing);

        if (g_abortZScan) {
            break;
        }

        char error = 0;
        if (g_nZScanZPosition > long(Printer::axisStepsPerMM[Z_AXIS] * g_scanStartZLiftMM) || g_nZScanZPosition > 32767) {
            Com::printFLN(PSTR("Z = "), g_nZScanZPosition * Printer::axisMMPerSteps[Z_AXIS]);
            error = SCAN_ABORT_REASON_SCAN_OVER_START;
        } else if (g_nLastZScanZPosition && abs(g_nZScanZPosition - g_nLastZScanZPosition) > g_nScanHeatBedDownFastSteps * (2 +                                                                                           /*nach wiederholungen etwas mehr zulassen. krumme keramik braucht wohl mehr ... */
                                                                                                                            (g_scanRetries < HEAT_BED_SCAN_RETRIES ?                                                      /* nur beachten bei wiederholung */
                                                                                                                                 (HEAT_BED_SCAN_RETRIES - g_scanRetries <= 2 ? HEAT_BED_SCAN_RETRIES - g_scanRetries : 2) /* nie mehr als 0.2 bzw 2x draufschlagen, das reicht sicher - sonst ist es ein anderer fehler. */
                                                                                                                                                                   : 0))) {
            Com::printFLN(PSTR("dZ_lastpos = "), abs(g_nZScanZPosition - g_nLastZScanZPosition) * Printer::axisMMPerSteps[Z_AXIS]);
            error = SCAN_ABORT_REASON_SCAN_TOO_UNEVEN;
        } else if (abs(startScanZPosition - g_nZScanZPosition) > g_nScanHeatBedDownFastSteps * 2 / acuteness) {
            Com::printFLN(PSTR("dZ_move = "), g_nZScanZPosition * Printer::axisMMPerSteps[Z_AXIS]);
            error = SCAN_ABORT_REASON_SCAN_STEP_DELTA_OUT_OF_RANGE;
        }
        if (error) {
            Com::printFLN(PSTR("moveZPlusDownSlow: out of range "), g_scanRetries);
            if (g_scanRetries)
                g_retryZScan = 1;
            else
                g_abortZScan = error;
            break;
        }
    }
} // moveZPlusDownSlow

//gegen Düse fahren schnell:
void moveZMinusUpFast() {
    short nTempPressure;

    // move the heat bed up until we detect the contact pressure (fast speed)
    while (1) {
        if (g_nZScanZPosition <= -g_nScanZMaxCompensationSteps || g_nZScanZPosition < -32768) {
            Com::printFLN(PSTR("moveZMinusUpFast(): out of range "), (int)g_scanRetries);
            Com::printFLN(PSTR("Z-Endstop Limit:"), Z_ENDSTOP_DRIVE_OVER);
            Com::printFLN(PSTR("Z = "), g_nZScanZPosition * Printer::axisMMPerSteps[Z_AXIS]);

            if (g_scanRetries)
                g_retryZScan = 1;
            else
                g_abortZScan = SCAN_ABORT_REASON_REACHED_MAX_COMPENSATION;
            break;
        }

        Commands::checkForPeriodicalActions(Processing);
        HAL::delayMilliseconds(g_nScanFastStepDelay);

        if (readAveragePressure(&nTempPressure)) {
            g_abortZScan = SCAN_ABORT_REASON_AVERAGE_PRESSURE;
            // some error has occurred
            break;
        }

        if (nTempPressure > g_nMaxPressureContact || nTempPressure < g_nMinPressureContact) {
            // we have reached the target pressure
            break;
        }

        Commands::checkForPeriodicalActions(Processing);
        moveZ(g_nScanHeatBedUpFastSteps);

        if (g_abortZScan) {
            break;
        }
    }
} // moveZMinusUpFast

//Gegen Düse fahren langsam:
void moveZMinusUpSlow(short* pnContactPressure, uint8_t acuteness) {
    short nTempPressure;

    // move the heat bed up until we detect the contact pressure (slow speed)
    while (1) {
        if (g_nZScanZPosition <= -g_nScanZMaxCompensationSteps || g_nZScanZPosition < -32768) {
            Com::printFLN(PSTR("moveZMinusUpSlow(): the z position went out of range, retries = "), g_scanRetries);
            Com::printFLN(PSTR("Z-Endstop Limit:"), Z_ENDSTOP_DRIVE_OVER);
            Com::printFLN(PSTR("Z = "), g_nZScanZPosition * Printer::axisMMPerSteps[Z_AXIS]);

            if (g_scanRetries)
                g_retryZScan = 1;
            else
                g_abortZScan = SCAN_ABORT_REASON_REACHED_MAX_COMPENSATION;
            break;
        }

        Commands::checkForPeriodicalActions(Processing);
        HAL::delayMilliseconds(g_nScanSlowStepDelay);

        if (readAveragePressure(&nTempPressure)) {
            g_abortZScan = SCAN_ABORT_REASON_AVERAGE_PRESSURE;
            // some error has occurred
            break;
        }

        if (nTempPressure > g_nMaxPressureContact || nTempPressure < g_nMinPressureContact) {
            // we have found the proper pressure
            break;
        }

        Commands::checkForPeriodicalActions(Processing);
        moveZ((g_nScanHeatBedUpSlowSteps / acuteness ? g_nScanHeatBedUpSlowSteps / acuteness : 1));

        if (g_abortZScan) {
            break;
        }
    }
    *pnContactPressure = nTempPressure;
} // moveZMinusUpSlow

void moveZ(int nSteps) {
    /*
    Warning 03.11.2017 : Do not try to make more steps than < 10mm in one row. Some printers will get a watchdog reset.
    When choosing 10mm one printer crashed while others still worked.
    We changed Scan PLA/ABS to 2x 5mm and it worked.
    Reason is because we removed watchdog-ping from HAL::delayMicroseconds (which was good!)
    */

    // The scan may pass the Z-min switch up to the configured overtravel.

    int nMaxLoops;
    if (nSteps >= 0)
        nMaxLoops = nSteps;
    else
        nMaxLoops = -nSteps;

    while (Printer::needsCMPwait())
        Commands::checkForPeriodicalActions(Processing);

    // A retreat can make a pending correction reachable again. Keep Z for the scan until this move ends.
    Printer::scanZMoveActive = true;

    // perform the steps
    for (int i = 0; i < nMaxLoops; i++) {
        if (g_abortZScan)
            break; // do not continue here in case the current operation has been cancelled

        if (nSteps < 0 && Printer::isAxisHomed(Z_AXIS) && Printer::isZMinOverrideReached()) {
            g_abortZScan = SCAN_ABORT_REASON_REACHED_MAX_COMPENSATION;
            g_scanRetries = 0;
            Com::printFLN(PSTR("Z-Endstop protection"));
            UI_STATUS_UPD("Z-Endstop protection");
            break;
        }

        if (nSteps >= 0) {
            if (!Printer::getZDirectionIsPos()) {
                Printer::setZDirection(true);
                //Com::printFLN( PSTR( "moveZ: BedDown Z=" ),g_nZScanZPosition );  //kann manchmal verwirrend sein, gleiche richtugnen werden nicht angezeigt. Hoch, etwas runter .... scan ergebnis ... runter für neuanlauf  -> scanergebnis unsichtbar.
            }
        } else {
            if (Printer::getZDirectionIsPos()) {
                Printer::setZDirection(false);
                //Com::printFLN( PSTR( "moveZ: BedUp Z=" ),g_nZScanZPosition );  //kann manchmal verwirrend sein, gleiche richtugnen werden nicht angezeigt. Hoch, etwas runter .... scan ergebnis ... runter für neuanlauf  -> scanergebnis unsichtbar.
            }
        }

        HAL::delayMicroseconds(XYZ_STEPPER_HIGH_DELAY);
        Printer::startZStep();

        HAL::delayMicroseconds(XYZ_STEPPER_LOW_DELAY);
        Printer::endZStep();

        g_nZScanZPosition += (Printer::getZDirectionIsPos() ? 1 : -1);
    }
    Printer::scanZMoveActive = false;
} // moveZ

void restoreDefaultScanParameters(void) {
    // we must restore the default scan parameters for the heat bed scan
    g_nScanXStartSteps = long(Printer::axisStepsPerMM[X_AXIS] * HEAT_BED_SCAN_X_START_MM);
    g_nScanXStepSizeMM = HEAT_BED_SCAN_X_STEP_SIZE_MM;
    g_nScanXStepSizeSteps = long(Printer::axisStepsPerMM[X_AXIS] * HEAT_BED_SCAN_X_STEP_SIZE_MM);
    g_nScanXEndSteps = long(Printer::axisStepsPerMM[X_AXIS] * HEAT_BED_SCAN_X_END_MM);
    g_nScanXMaxPositionSteps = long((Printer::axisLengthMM[X_AXIS] - HEAT_BED_SCAN_X_END_MM) * Printer::axisStepsPerMM[X_AXIS]);

    g_nScanYStartSteps = long(Printer::axisStepsPerMM[Y_AXIS] * HEAT_BED_SCAN_Y_START_MM);
    g_nScanYStepSizeMM = HEAT_BED_SCAN_Y_STEP_SIZE_MM;
    g_nScanYStepSizeSteps = long(Printer::axisStepsPerMM[Y_AXIS] * HEAT_BED_SCAN_Y_STEP_SIZE_MM);
    g_nScanYEndSteps = long(Printer::axisStepsPerMM[Y_AXIS] * HEAT_BED_SCAN_Y_END_MM);
    g_nScanYMaxPositionSteps = long((Printer::axisLengthMM[Y_AXIS] - HEAT_BED_SCAN_Y_END_MM) * Printer::axisStepsPerMM[Y_AXIS]);

    g_nScanHeatBedUpFastSteps = short(Printer::axisStepsPerMM[Z_AXIS] * HEAT_BED_SCAN_UP_FAST_MM);
    g_nScanHeatBedUpSlowSteps = short(Printer::axisStepsPerMM[Z_AXIS] * HEAT_BED_SCAN_UP_SLOW_MM);
    g_nScanHeatBedDownFastSteps = short(Printer::axisStepsPerMM[Z_AXIS] * HEAT_BED_SCAN_DOWN_FAST_MM);
    g_nScanHeatBedDownSlowSteps = short(Printer::axisStepsPerMM[Z_AXIS] * HEAT_BED_SCAN_DOWN_SLOW_MM);

    /* Maximum number of steps to scan after the Z-min switch has been reached. If within these steps the surface has not
       been reached, the scan is retried HEAT_BED_SCAN_RETRIES times and then (if still not found) aborted.
       Note that the head bed scan matrix consists of 16 bit signed values, thus more then 32767 steps will lead to an overflow! */
    g_nScanZMaxCompensationSteps = short(Z_ENDSTOP_DRIVE_OVER * Printer::axisStepsPerMM[Z_AXIS]);

    g_nScanFastStepDelay = HEAT_BED_SCAN_FAST_STEP_DELAY_MS;
    g_nScanSlowStepDelay = HEAT_BED_SCAN_SLOW_STEP_DELAY_MS;
    g_nScanIdleDelay = HEAT_BED_SCAN_IDLE_DELAY_MS;

    g_nScanContactPressureDelta = HEAT_BED_SCAN_CONTACT_PRESSURE_DELTA;
    g_nScanRetryPressureDelta = HEAT_BED_SCAN_RETRY_PRESSURE_DELTA;
    g_nScanIdlePressureDelta = HEAT_BED_SCAN_IDLE_PRESSURE_DELTA;
    g_nScanIdlePressureMin = HEAT_BED_SCAN_IDLE_PRESSURE_MIN;
    g_nScanIdlePressureMax = HEAT_BED_SCAN_IDLE_PRESSURE_MAX;

    g_nScanPressureReads = HEAT_BED_SCAN_PRESSURE_READS;
    g_nScanPressureTolerance = HEAT_BED_SCAN_PRESSURE_TOLERANCE;
    g_nScanPressureReadDelay = HEAT_BED_SCAN_PRESSURE_READ_DELAY_MS;
} // restoreDefaultScanParameters

void outputScanParameters(void) {
    if (Printer::debugInfo()) {
        Com::printFLN(PSTR("outputScanParameters(): current scan parameters:"));

        Com::printF(PSTR(""), Printer::axisStepsPerMM[X_AXIS]);
        Com::printFLN(PSTR(";[steps];axisStepsPerMM[X_AXIS]"));
        Com::printF(PSTR(""), Printer::axisStepsPerMM[Y_AXIS]);
        Com::printFLN(PSTR(";[steps];axisStepsPerMM[Y_AXIS]"));
        Com::printF(PSTR(""), Printer::axisStepsPerMM[Z_AXIS]);
        Com::printFLN(PSTR(";[steps];axisStepsPerMM[Z_AXIS]"));

        Com::printF(PSTR(""), g_nScanXStartSteps);
        Com::printFLN(PSTR(";[steps];g_nScanXStartSteps"));
        Com::printF(PSTR(""), g_nScanXStepSizeSteps);
        Com::printFLN(PSTR(";[steps];g_nScanXStepSizeSteps"));
        Com::printF(PSTR(""), g_nScanXEndSteps);
        Com::printFLN(PSTR(";[steps];g_nScanXEndSteps"));
        Com::printF(PSTR(""), g_nScanXMaxPositionSteps);
        Com::printFLN(PSTR(";[steps];g_nScanXMaxPositionSteps"));

        Com::printF(PSTR(""), g_nScanYStartSteps);
        Com::printFLN(PSTR(";[steps];g_nScanYStartSteps"));
        Com::printF(PSTR(""), g_nScanYStepSizeSteps);
        Com::printFLN(PSTR(";[steps];g_nScanYStepSizeSteps"));
        Com::printF(PSTR(""), g_nScanYEndSteps);
        Com::printFLN(PSTR(";[steps];g_nScanYEndSteps"));
        Com::printF(PSTR(""), g_nScanYMaxPositionSteps);
        Com::printFLN(PSTR(";[steps];g_nScanYMaxPositionSteps"));

        Com::printF(PSTR(""), (int)g_nScanHeatBedUpFastSteps);
        Com::printFLN(PSTR(";[steps];g_nScanHeatBedUpFastSteps"));
        Com::printF(PSTR(""), (int)g_nScanHeatBedUpSlowSteps);
        Com::printFLN(PSTR(";[steps];g_nScanHeatBedUpSlowSteps"));
        Com::printF(PSTR(""), (int)g_nScanHeatBedDownFastSteps);
        Com::printFLN(PSTR(";[steps];g_nScanHeatBedDownFastSteps"));
        Com::printF(PSTR(""), (int)g_nScanHeatBedDownSlowSteps);
        Com::printFLN(PSTR(";[steps];g_nScanHeatBedDownSlowSteps"));
        Com::printF(PSTR(""), (int)g_nScanFastStepDelay);
        Com::printFLN(PSTR(";[ms];g_nScanFastStepDelay"));
        Com::printF(PSTR(""), (int)g_nScanSlowStepDelay);
        Com::printFLN(PSTR(";[ms];g_nScanSlowStepDelay"));
        Com::printF(PSTR(""), (int)g_nScanIdleDelay);
        Com::printFLN(PSTR(";[ms];g_nScanIdleDelay"));

        Com::printF(PSTR(""), (int)g_nScanContactPressureDelta);
        Com::printFLN(PSTR(";[digits];g_nScanContactPressureDelta"));
        Com::printF(PSTR(""), (int)g_nScanRetryPressureDelta);
        Com::printFLN(PSTR(";[digits];g_nScanRetryPressureDelta"));
        Com::printF(PSTR(""), (int)g_nScanIdlePressureDelta);
        Com::printFLN(PSTR(";[digits];g_nScanIdlePressureDelta"));

        Com::printF(PSTR(""), (int)g_nScanPressureReads);
        Com::printFLN(PSTR(";[-];g_nScanPressureReads"));
        Com::printF(PSTR(""), (int)g_nScanPressureTolerance);
        Com::printFLN(PSTR(";[digits];g_nScanPressureTolerance"));
        Com::printF(PSTR(""), (int)g_nScanPressureReadDelay);
        Com::printFLN(PSTR(";[ms];g_nScanPressureReadDelay"));
    }
} // outputScanParameters

void initCompensationMatrix(void) {
    // clear all fields of the compensation matrix
    memset(g_ZCompensationMatrix, 0, COMPENSATION_MATRIX_MAX_X * COMPENSATION_MATRIX_MAX_Y * 2);
} // initCompensationMatrix

void outputCompensationMatrix(char format) {
    short x;
    short y;

    Com::printFLN(PSTR("front left ... front right"));
    Com::printFLN(PSTR("...        ...         ..."));
    Com::printFLN(PSTR("back left  ...  back right"));

    for (y = 0; y <= g_uZMatrixMax[Y_AXIS]; y++) {
        for (x = 0; x <= g_uZMatrixMax[X_AXIS]; x++) {
            if (x == 0 || y == 0) {
                Com::printF(Com::tSemiColon, g_ZCompensationMatrix[x][y]);
            } else {
                if (format) {
                    // output in [mm]
                    Com::printF(Com::tSemiColon, g_ZCompensationMatrix[x][y] / Printer::axisStepsPerMM[Z_AXIS]);
                } else {
                    // output in [steps]
                    Com::printF(Com::tSemiColon, g_ZCompensationMatrix[x][y]);
                }
            }
        }
        Com::printFLN(PSTR(" "));
    }

    Com::printF(PSTR("offset = "), g_offsetZCompensationSteps);
    Com::printF(PSTR(" [steps] (= "), (float)g_offsetZCompensationSteps * Printer::axisMMPerSteps[Z_AXIS]);
    Com::printFLN(PSTR(" [mm])"));

    Com::printF(PSTR("warpage = "), g_ZCompensationMax - g_offsetZCompensationSteps);
    Com::printF(PSTR(" [steps] (= "), float(g_ZCompensationMax - g_offsetZCompensationSteps) * Printer::axisMMPerSteps[Z_AXIS]);
    Com::printFLN(PSTR(" [mm])"));

    Com::printFLN(PSTR("g_uZMatrixMax[X_AXIS] = "), g_uZMatrixMax[X_AXIS]);
    Com::printFLN(PSTR("g_uZMatrixMax[Y_AXIS] = "), g_uZMatrixMax[Y_AXIS]);

    {
        Com::printFLN(PSTR("g_nActiveHeatBed = "), g_nActiveHeatBed);
    }

} // outputCompensationMatrix

char prepareCompensationMatrix(void) {
    short x;
    short y;

    // perform some safety checks first
    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("zCMP: invalid version: "), g_ZCompensationMatrix[0][0]);
            Com::printF(PSTR(" (expected: "), EEPROM_FORMAT);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }

    if (g_uZMatrixMax[X_AXIS] >= COMPENSATION_MATRIX_MAX_X || g_uZMatrixMax[X_AXIS] < 2) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("zCMP: invalid x dimension: "), g_uZMatrixMax[X_AXIS]);
            Com::printF(PSTR(" (max expected: "), COMPENSATION_MATRIX_MAX_X - 1);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }

    if (g_uZMatrixMax[Y_AXIS] >= COMPENSATION_MATRIX_MAX_Y || g_uZMatrixMax[Y_AXIS] < 2) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("zCMP: invalid y dimension: "), g_uZMatrixMax[Y_AXIS]);
            Com::printF(PSTR(" (max expected: "), COMPENSATION_MATRIX_MAX_Y - 1);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }

    if (g_ZCompensationMatrix[2][0] > 0) {
        // we have to fill x[1] with the values of x[2]
        /*      if( Printer::debugInfo() )
        {
            Com::printFLN( PSTR( "prepareCompensationMatrix(): x[2] > 0" ) );
        }
*/
        g_ZCompensationMatrix[1][0] = 0;
        for (y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
            g_ZCompensationMatrix[1][y] = g_ZCompensationMatrix[2][y];
        }
    } else {
        // we have to shift all x columns one index
        for (x = 1; x < g_uZMatrixMax[X_AXIS]; x++) {
            for (y = 0; y <= g_uZMatrixMax[Y_AXIS]; y++) {
                g_ZCompensationMatrix[x][y] = g_ZCompensationMatrix[x + 1][y];
            }
        }

        // we have one x column less now
        g_uZMatrixMax[X_AXIS]--;
    }

    if (g_ZCompensationMatrix[g_uZMatrixMax[X_AXIS]][0] < (short)Printer::axisLengthMM[X_AXIS]) {
        // we have to fill x[g_uZMatrixMax[X_AXIS]] with the values of x[g_uZMatrixMax[X_AXIS]-1]
        g_ZCompensationMatrix[g_uZMatrixMax[X_AXIS] + 1][0] = short(Printer::axisLengthMM[X_AXIS]);
        for (y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
            g_ZCompensationMatrix[g_uZMatrixMax[X_AXIS] + 1][y] = g_ZCompensationMatrix[g_uZMatrixMax[X_AXIS]][y];
        }

        // we have one x column more now
        g_uZMatrixMax[X_AXIS]++;
    } else {
        // there is nothing else to do here
    }

    if (g_ZCompensationMatrix[0][2] > 0) {
        // we have to fill y[1] with the values of y[2]
        g_ZCompensationMatrix[0][1] = 0;
        for (x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) {
            g_ZCompensationMatrix[x][1] = g_ZCompensationMatrix[x][2];
        }
    } else {
        // we have to shift all y columns one index
        for (x = 0; x <= g_uZMatrixMax[X_AXIS]; x++) {
            for (y = 1; y < g_uZMatrixMax[Y_AXIS]; y++) {
                g_ZCompensationMatrix[x][y] = g_ZCompensationMatrix[x][y + 1];
            }
        }

        // we have one y column less now
        g_uZMatrixMax[Y_AXIS]--;
    }

    if (g_ZCompensationMatrix[0][g_uZMatrixMax[Y_AXIS]] < short(Printer::axisLengthMM[Y_AXIS])) {
        // we have to fill y[g_uZMatrixMax[Y_AXIS]] with the values of y[g_uZMatrixMax[Y_AXIS]-1]
        g_ZCompensationMatrix[0][g_uZMatrixMax[Y_AXIS] + 1] = short(Printer::axisLengthMM[Y_AXIS]);
        for (x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) {
            g_ZCompensationMatrix[x][g_uZMatrixMax[Y_AXIS] + 1] = g_ZCompensationMatrix[x][g_uZMatrixMax[Y_AXIS]];
        }

        // we have one y column more now
        g_uZMatrixMax[Y_AXIS]++;
    }

    // determine the minimal distance between extruder and heat bed
    determineCompensationOffsetZ();

    return 0;
} // prepareCompensationMatrix

void determineCompensationOffsetZ(void) {
    short x;
    short y;
    short uMax = -32768;
    short uMin = 32767;

    for (x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) {
        for (y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
            if (g_ZCompensationMatrix[x][y] > uMax) {
                uMax = g_ZCompensationMatrix[x][y];
            }
            if (g_ZCompensationMatrix[x][y] < uMin) {
                uMin = g_ZCompensationMatrix[x][y];
            }
        }
    }
    g_ZCompensationMax = uMin;
    g_offsetZCompensationSteps = uMax;
} // determineCompensationOffsetZ

void adjustCompensationMatrix(short nZ) {
    short x;
    short y;
    short nOffset = getZMatrixDepth_CurrentXY();
    short deltaZ = nZ - nOffset;

    for (x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) {
        for (y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
            g_ZCompensationMatrix[x][y] += deltaZ;
        }
    }

    // determine the minimal distance between extruder and heat bed
    determineCompensationOffsetZ();
} // adjustCompensationMatrix

void saveCompensationMatrix(unsigned int uAddress) {
    unsigned int uOffset;
    short x;
    short y;

    if (uAddress == 0) {
        Com::printFLN(PSTR("saveMatrix(): valid uAddress - aborted!"));

        return;
    }

    if (g_ZCompensationMatrix[0][0] && g_uZMatrixMax[X_AXIS] && g_uZMatrixMax[Y_AXIS]) //valid in RAM means writing ok
    {
        // we have valid compensation values
        if (Printer::debugInfo()) {
            Com::printFLN(PSTR("saveMatrix(): valid data"));
        }

        // write the current header version
        writeWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_OFFSET_HEADER_FORMAT, EEPROM_FORMAT);

        // write the current sector version
        writeWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_SECTOR_FORMAT, EEPROM_FORMAT);

        // write the current x dimension
        writeWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_DIMENSION_X, g_uZMatrixMax[X_AXIS]);

        // write the current y dimension
        writeWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_DIMENSION_Y, g_uZMatrixMax[Y_AXIS]);

        // write the current micro steps
        writeWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_MICRO_STEPS,
#if FEATURE_ADJUSTABLE_MICROSTEPS
                        drv8711ModeValue_2_MicroSteps(Printer::motorMicroStepsModeValue[Z_AXIS])
#else
                        RF_MICRO_STEPS_Z
#endif // FEATURE_ADJUSTABLE_MICROSTEPS
        );

        uOffset = uAddress + EEPROM_OFFSET_MATRIX_START;
        for (x = 0; x <= g_uZMatrixMax[X_AXIS]; x++) {
            for (y = 0; y <= g_uZMatrixMax[Y_AXIS]; y++) {
                writeWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, uOffset, g_ZCompensationMatrix[x][y]);
                uOffset += 2;
            }
            GCode::keepAlive(Processing);
        }
    } else {
        // we do not have valid heat bed compensation values - clear the EEPROM data
        if (Printer::debugErrors()) {
            Com::printF(PSTR("saveMatrix(): invalid data ("), g_ZCompensationMatrix[0][0]);
            Com::printF(PSTR("/"), g_uZMatrixMax[X_AXIS]);
            Com::printF(PSTR("/"), g_uZMatrixMax[Y_AXIS]);
            Com::printFLN(PSTR(")"));
        }

        // write the current version
        writeWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_OFFSET_HEADER_FORMAT, EEPROM_FORMAT);

        // write the current sector version
        writeWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_SECTOR_FORMAT, 0);

        // write the current x dimension
        writeWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_DIMENSION_X, 0);

        // write the current y dimension
        writeWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_DIMENSION_Y, 0);

        // write the current micro steps
        writeWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_MICRO_STEPS, 0);

        uOffset = uAddress + EEPROM_OFFSET_MATRIX_START;
        for (x = 0; x < COMPENSATION_MATRIX_MAX_X; x++) {
            for (y = 0; y < COMPENSATION_MATRIX_MAX_Y; y++) {
                writeWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, uOffset, 0);
                uOffset += 2;
            }
            GCode::keepAlive(Processing);
        }
    }

    determineCompensationOffsetZ();
    g_ZMatrixChangedInRam = 0;
} // saveCompensationMatrix

bool saveActiveHeatBedToExtEEPROM(char newActiveHeatBed) {
    if (newActiveHeatBed < 1 || newActiveHeatBed > EEPROM_MAX_HEAT_BED_SECTORS) {
        return false;
    }

    char oldActiveHeatBed = (char)readWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_OFFSET_ACTIVE_HEAT_BED_Z_MATRIX);
    if (oldActiveHeatBed != newActiveHeatBed) {
        writeWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_OFFSET_ACTIVE_HEAT_BED_Z_MATRIX, newActiveHeatBed);
    }

    return true;
}

bool loadActiveHeatBedFromExtEEPROM() {
    char uTemp = (char)readWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_OFFSET_ACTIVE_HEAT_BED_Z_MATRIX);

    if (uTemp < 1 || uTemp > EEPROM_MAX_HEAT_BED_SECTORS) {
        Com::printFLN(PSTR("Invalid active heat bed within ext. EEPROM detected: "), (int)uTemp);
        if (g_nActiveHeatBed < 1 || g_nActiveHeatBed > EEPROM_MAX_HEAT_BED_SECTORS) {
            g_nActiveHeatBed = 1;
        }
        saveActiveHeatBedToExtEEPROM(g_nActiveHeatBed);

        return false;
    }

    g_nActiveHeatBed = uTemp;
    return true;
}

char loadCompensationMatrix(unsigned int uAddress) {
    if (uAddress == 0) {
        Com::printFLN(PSTR("loadMatrix(): valid uAddress - aborted!"));

        return -1;
    }

    unsigned short uTemp;
    unsigned short uDimensionX;
    unsigned short uDimensionY;
    unsigned short uMicroSteps;
    unsigned int uOffset;
    short nTemp;
    short x;
    short y;
    float fMicroStepCorrection;

    Printer::disableCMPnow(true); // vorher, nicht nachher ausschalten, sonst arbeitet unter Umständen die alte matrix.

    // check the stored header format
    uTemp = readWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_OFFSET_HEADER_FORMAT);

    if (uTemp != EEPROM_FORMAT) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("loadMatrix(): invalid header format: "), (int)uTemp);
            Com::printF(PSTR(" (expected: "), EEPROM_FORMAT);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }


    // check the stored sector format
    uTemp = readWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_SECTOR_FORMAT);

    if (uTemp != EEPROM_FORMAT) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("loadMatrix(): invalid sector format: "), (int)uTemp);
            Com::printF(PSTR(" (expected: "), EEPROM_FORMAT);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }

    // check the stored x dimension
    uDimensionX = readWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_DIMENSION_X);

    if (uDimensionX > COMPENSATION_MATRIX_MAX_X) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("loadMatrix(): invalid x dimension: "), (int)uDimensionX);
            Com::printF(PSTR(" (max expected: "), COMPENSATION_MATRIX_MAX_X);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }

    // check the stored y dimension
    uDimensionY = readWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_DIMENSION_Y);

    if (uDimensionY > COMPENSATION_MATRIX_MAX_Y) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("loadMatrix(): invalid y dimension: "), (int)uDimensionY);
            Com::printF(PSTR(" (max expected: "), COMPENSATION_MATRIX_MAX_Y);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }

    g_uZMatrixMax[X_AXIS] = (unsigned char)uDimensionX;
    g_uZMatrixMax[Y_AXIS] = (unsigned char)uDimensionY;

    // check the stored microsteps
    uMicroSteps = readWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_MICRO_STEPS);

    if (uMicroSteps ==
#if FEATURE_ADJUSTABLE_MICROSTEPS
        drv8711ModeValue_2_MicroSteps(Printer::motorMicroStepsModeValue[Z_AXIS])
#else
        RF_MICRO_STEPS_Z
#endif // FEATURE_ADJUSTABLE_MICROSTEPS
    ) {
        // the current z-compensation matrix has been determined with the current micro step setting, there is nothing to recalculate
        fMicroStepCorrection = 1.0;
    } else {
        // the current z-compensation matrix has been determined with a higher than the current micro step setting, we must divide all z-correction values
        fMicroStepCorrection = (float)
#if FEATURE_ADJUSTABLE_MICROSTEPS
                                   drv8711ModeValue_2_MicroSteps(Printer::motorMicroStepsModeValue[Z_AXIS])
#else
                                   RF_MICRO_STEPS_Z
#endif // FEATURE_ADJUSTABLE_MICROSTEPS
            / (float)uMicroSteps;

        Com::printF(PSTR("loadMatrix(): micro step correction = "), fMicroStepCorrection);
        Com::printF(PSTR(" (stored="), (int)uMicroSteps);
        Com::printF(PSTR(", current="), (int)
#if FEATURE_ADJUSTABLE_MICROSTEPS
                                            drv8711ModeValue_2_MicroSteps(Printer::motorMicroStepsModeValue[Z_AXIS])
#else
                                            RF_MICRO_STEPS_Z
#endif // FEATURE_ADJUSTABLE_MICROSTEPS
        );
        Com::printFLN(PSTR(")"));
    }

    // read out the actual compensation values
    uOffset = uAddress + EEPROM_OFFSET_MATRIX_START;
    for (x = 0; x <= g_uZMatrixMax[X_AXIS]; x++) {
        for (y = 0; y <= g_uZMatrixMax[Y_AXIS]; y++) {
            nTemp = readWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, uOffset);

            if (x == 0 || y == 0) {
                // we must not modify our header row/column
                g_ZCompensationMatrix[x][y] = nTemp;
            } else {
                // we may have to update all z-compensation values
                long transformedMatrixElement = long((float)nTemp * fMicroStepCorrection);
                if (transformedMatrixElement > 32767 || transformedMatrixElement < -32768) {
                    Com::printFLN(PSTR("MicroStep transf.: overflow"));

                    return -1;
                }
                g_ZCompensationMatrix[x][y] = short(transformedMatrixElement);
            }
            uOffset += 2;
        }
        GCode::keepAlive(Processing);
    }

    determineCompensationOffsetZ();

    g_ZMatrixChangedInRam = 0; //Nibbels: Marker, dass die Matrix gespeichert werden kann oder eben nicht, weils unverändert keinen Sinn macht.

    return 0;
} // loadCompensationMatrix

void clearCompensationMatrix(unsigned int uAddress) {
    // clear all fields of the compensation matrix
    initCompensationMatrix();

    // store the cleared compensation matrix to the EEPROM
    saveCompensationMatrix(uAddress);

    if (Printer::debugInfo()) {
        Com::printFLN(PSTR("clearMatrix(): matrix cleared"));
    }
} // clearCompensationMatrix

void clearExternalEEPROM(void) {
    unsigned short i;
    unsigned short uMax = 32768;
    unsigned short uTemp;
    unsigned short uLast = 0;

    if (Printer::debugInfo()) {
        Com::printFLN(PSTR("clearExtEEPROM(): erasing chip memory ..."));
    }

    // the external EEPROM is able to store 262.144 kBit (= 32.768 kByte)
    for (i = 0; i < uMax; i++) {
        writeByte24C256(I2C_ADDRESS_EXTERNAL_EEPROM, i, 0);
        Commands::checkForPeriodicalActions(Processing);

        if (Printer::debugInfo()) {
            uTemp = i / 100;
            if (uTemp != uLast) {
                Com::printF(PSTR("clearExtEEPROM(): "), (int)i);
                Com::printFLN(PSTR(" / "), (long)uMax);
                uLast = uTemp;
            }
        }
    }

    if (Printer::debugInfo()) {
        Com::printFLN(PSTR("clearExtEEPROM(): erasing complete"));
    }
} // clearExternalEEPROM

void writeByte24C256(int addressI2C, unsigned int addressEEPROM, unsigned char data) {
    HAL::delayMilliseconds(EEPROM_DELAY);
    Wire.beginTransmission(addressI2C);
    Wire.write(int(addressEEPROM >> 8));   // MSB
    Wire.write(int(addressEEPROM & 0xFF)); // LSB
    Wire.write(data);
    Wire.endTransmission();
    return;

} // writeByte24C256

void writeWord24C256(int addressI2C, unsigned int addressEEPROM, unsigned short data) {
    unsigned short Temp;

    Temp = byte(data >> 8);
    writeByte24C256(addressI2C, addressEEPROM, Temp);
    Temp = byte(data & 0x00FF);
    writeByte24C256(addressI2C, addressEEPROM + 1, Temp);
    return;

} // writeWord24C256

unsigned char readByte24C256(int addressI2C, unsigned int addressEEPROM) {
    HAL::delayMilliseconds(EEPROM_DELAY);
    Wire.beginTransmission(addressI2C);
    Wire.write(int(addressEEPROM >> 8));   // MSB
    Wire.write(int(addressEEPROM & 0xFF)); // LSB
    Wire.endTransmission();
    Wire.requestFrom(addressI2C, 1);

    return Wire.read();

} // readByte24C256

unsigned short readWord24C256(int addressI2C, unsigned int addressEEPROM) {
    unsigned short data;
    byte Temp;

    Temp = readByte24C256(addressI2C, addressEEPROM);
    data = Temp;
    data = data << 8;
    Temp = readByte24C256(addressI2C, addressEEPROM + 1);

    return data + Temp;

} // readWord24C256

void recalculateZCompensation(void) {
    recalculateHeatBedZCompensation();
} // recalculateZCompensation

void handleStrainGaugeFeatures(millis_t uTime) {
    // Share one scheduled DMS reading among SenseOffset, emergency pause and emergency stop.
    bool i_need_strain_value = 0;

    static millis_t nSensiblePressureTime = 0;
    static long nSensiblePressureSum = 0;
    static char nSensiblePressureChecks = 0;
    static short g_nSensibleLastPressure = 0;
    if ((uTime - nSensiblePressureTime) > SENSIBLE_PRESSURE_INTERVAL) //max. jede 50ms
    {
        i_need_strain_value = 1;
    }

    static millis_t uLastPressureTime = 0;
    static long nPressureSum = 0;
    static char nPressureChecks = 0;
    if ((uTime - uLastPressureTime) > EMERGENCY_PAUSE_INTERVAL) //max. jede 100ms
    {
        i_need_strain_value = 1;
    }

    static millis_t uLastZPressureTime = 0;
    static long nZPressureSum = 0;
    static char nZPressureChecks = 0;
    if ((uTime - uLastZPressureTime) > EMERGENCY_STOP_INTERVAL) //max. jede 10ms
    {
        i_need_strain_value = 1;
    }

    static short pressure = 0;
    if (i_need_strain_value) {
        pressure = readStrainGauge(ACTIVE_STRAIN_GAUGE);
    }

    short nPressure;
    // When bed compensation is disabled, any remaining live SenseOffset is
    // safe to discard, including after M3909 P0 disabled its threshold.
    if (!Printer::doHeatBedZCompensation) {
        bool wasEnabled = g_nSensiblePressureDigits != 0;
        g_nSensiblePressureDigits = 0;
        if (g_nSensiblePressureOffset) {
            Com::printF(PSTR("SensiblePressure(): offset reverted from "), g_nSensiblePressureOffset);
            Com::printFLN(PSTR(" [um] to 0 [um] "));
            // Preserve the EEPROM-backed manual offset.
            g_nSensiblePressureOffset = 0;
            g_staticZSteps = (Printer::ZOffset * Printer::axisStepsPerMM[Z_AXIS]) / 1000;
        }
        if (wasEnabled)
            Com::printFLN(PSTR("SensiblePressure(): now disabled because of no z-CMP. "), g_nSensiblePressureOffset);
    }

    // M3909 P0 keeps the already applied offset while compensation is still
    // active, but its old pressure samples must not leak into a new session.
    if (!g_nSensiblePressureDigits || g_startLineActive) {
        // Priming force belongs to the speed/E brake, not SenseOffset.
        nSensiblePressureSum = 0;
        nSensiblePressureChecks = 0;
        g_nSensibleLastPressure = 0;
        g_nSensiblePressure1stMarke = 0;
    }

    if ((uTime - nSensiblePressureTime) > SENSIBLE_PRESSURE_INTERVAL) {
        nSensiblePressureTime = uTime;
        /* brief: This is for correcting too close Z at first layer // Idee Wessix, coded by Nibbels  */

        if (g_nSensiblePressureDigits && Printer::doHeatBedZCompensation && !g_startLineActive) { //activate feature with G-Code.
            /*
            Still testing: Gesucht ist ein Limit des der Abstands zum Druckbett, also die jeweils richtige Layerbegrenzung::

            #das Extruder::current->zOffset ist negativ und in Steps. Ist ein Extruder weiter unten, per T1-Offset, muss das bedacht werden.
            #Die Höhe über Grund, kompensiert sollte unterhalb g_maxZCompensationSteps sein.
            #currentSteps = Achsenziel + Achsenoffset, aber g_minZCompensationSteps/g_maxZCompensationSteps kennen das Achsenoffset nicht ohne Hilfe: Das gehört hier her, wenn man die Layerhöhe abgleichen will.
            */
            if (Printer::currentSteps[Z_AXIS] <= g_minZCompensationSteps) {
                g_nSensiblePressure1stMarke = 1; //marker für display: wir sind in regelhöhe
                //wenn durch Gcode gefüllt, prüfe, ob Z-Korrektur (weg vom Bett) notwendig ist, in erstem Layer.
                nSensiblePressureSum += pressure;
                nSensiblePressureChecks += 1;
                //jede 1 sekunden, bzw 0.5sekunden. => 100ms * 10 ::
                if (nSensiblePressureChecks >= 10) {

                    nPressure = (short)(nSensiblePressureSum / nSensiblePressureChecks);

                    //half interval, remember old values 50% -> gibt etwas value-trägheit in den regler -> aber verursacht doppelte schrittgeschwindigkeit bei 0,5
                    nSensiblePressureSum *= 0.5;
                    nSensiblePressureChecks *= 0.5;

                    //größer, kleiner, falls Messzellen falschrum eingebaut. -> abs() // später einfacher zu verrechnen.
                    nPressure = abs(nPressure);
                    if ((nPressure > g_nSensiblePressureDigits)) {
                        /* here the action is defined if digits are > g_nSensiblePressureDigits while printing */

                        /*
                        Z-OFFSETs :: Mir sind aktuell folgende Offsets in Z bekannt:
                            long Printer::ZOffset;                                                          == OFFSET IN MENÜ / M3006 in [um]
                            g_staticZSteps = (Printer::ZOffset * Printer::axisStepsPerMM[Z_AXIS]) / 1000;   == OFFSET-STEPS errechnet aus Printer::ZOffset -> Und NEU zusätzlich aus SenseOffset
                            g_offsetZCompensationSteps                                                      == OFFSET AUS zMATRIX (Minimaler Bett-Hotend-Abstand)
                            nNeededZCompensation                                                            == In der Z-Compensations-Funktion: Das ist der Matrixanteil an exaktem Punkt x/y

                            Strategie: Wir haben unser eigenes Sensible-Offset "g_nSensiblePressureOffset" mit Beschränkung und rechnen g_staticZSteps neu aus.
                            g_nSensiblePressureOffset
                        */

                        //wie stark reagieren?
                        //PID -Regler wollen wir uns aber sparen...
                        unsigned char step = 1; //0..255!! standardincrement
                        float inc = 0.0;
                        //mehr offsetincrement je höher digits oberhalb limit. linear
                        if (nPressure > g_nSensiblePressureDigits * 1.1) {
                            //je größer der Fehler, desto beschränkt größer der step
                            inc = (float)nPressure / (float)g_nSensiblePressureDigits - 1; //0.1 ... riesig
                            if (inc > 2.0)
                                inc = 2.0;
                            inc *= 12; // 1,2..24,0
                            step += (short)inc;
                        }

                        if (g_nSensibleLastPressure - nPressure > 0) {
                            //digits sinken gerade -> egal wie hoch, regelspeed raus!
                            step = 1;
                        } else {
                            //digits steigen gerade -> je höher mein aktuelles zusätzliches offset, desto weniger regelspeed.
                            inc = (float)g_nSensiblePressureOffset / (float)g_nSensiblePressureOffsetMax; //0.25 .. 1
                            if (inc >= 0.25) {
                                step = step - (short)(inc * (step - 1)); //über der viertel offsetstrecke, das zusätzliche offset stark ausbremsen.
                            }
                            if (inc >= 0.33) {
                                if (nPressure < g_nSensiblePressureDigits * 1.1)
                                    step = 0; //wenn schon etwas offset da und die kräfte nur um +10% rumpendeln, dann nicht weiter erhöhen.
                            }
                        }

                        if (step > 0) {
                            //um == mikrometer -> Offsetbereich beschränkt auf 0..0,10mm
                            if (g_nSensiblePressureOffset + step <= (long)g_nSensiblePressureOffsetMax) {
                                g_nSensiblePressureOffset += step;
                            } else {
                                g_nSensiblePressureOffset = (long)g_nSensiblePressureOffsetMax;
                            }
                            g_staticZSteps = ((Printer::ZOffset + g_nSensiblePressureOffset) * Printer::axisStepsPerMM[Z_AXIS]) / 1000;
                        }
                    } else {
                        //kleinere digits sind natürlich zugelassen, runtergeregelt wird nicht wegen kleinerer digits bei z.B. retracts
                    }
                    g_nSensibleLastPressure = nPressure; //save last pressure.
                }
            } else {
                g_nSensiblePressure1stMarke = 0; //marker für display: auf dieser Höhe regeln wir nichts mehr oder noch nichts.
                // if sensible not active
                // if z-compensation not active
                if (nSensiblePressureSum > 0) {
                    nSensiblePressureSum = 0; //close down counters if function deactivated.
                    nSensiblePressureChecks = 0;
                }
                //offset muss bleiben! g_nSensiblePressureOffset != 0
            }
        }
    }

    if (g_nEmergencyPauseDigitsMin || g_nEmergencyPauseDigitsMax) {
        if ((uTime - uLastPressureTime) > EMERGENCY_PAUSE_INTERVAL) //jede 100ms -> das macht hier drin wenig sinn.
        {
            uLastPressureTime = uTime;

            if (!g_pauseMode && Printer::isPrinting() && Printer::areAxisHomed()) {
                // this check shall be done only during the printing (for example, it shall not be done in case filament is extruded manually)
                nPressureSum += pressure;
                nPressureChecks += 1;

                if (nPressureChecks == EMERGENCY_PAUSE_CHECKS) {
                    short nPressure = (short)(nPressureSum / nPressureChecks);
                    nPressureSum = 0;
                    nPressureChecks = 0;

                    if ((nPressure < g_nEmergencyPauseDigitsMin) || (nPressure > g_nEmergencyPauseDigitsMax)) {
                        // the pressure is outside the allowed range, we must perform the emergency pause
                        Com::printF(PSTR("emergency pause: "), nPressure);
                        showWarning((void*)ui_text_emergency_pause);

                        requestPrintPause(true);
                    }
                }
            } else {
                nPressureSum = 0;
                nPressureChecks = 0;
            }
        }
    }

    if ((uTime - uLastZPressureTime) > EMERGENCY_STOP_INTERVAL) //jede 10ms -> das macht hier drin überhaupt garkeinen sinn.
    {
        uLastZPressureTime = uTime;

        // this check shall be done only when there is some moving into z-direction in progress and the extruder is not doing anything
        nZPressureSum += pressure; //readStrainGauge( ACTIVE_STRAIN_GAUGE );
        nZPressureChecks += 1;

        if (nZPressureChecks == EMERGENCY_STOP_CHECKS) {
            nPressure = (short)(nZPressureSum / nZPressureChecks);

            nZPressureSum = 0;
            nZPressureChecks = 0;

            if (nPressure < g_nEmergencyStopZAndEMin || g_nEmergencyStopZAndEMax < nPressure) {
                // Forbid Extrusion at high levels
                g_nEmergencyESkip = true;

                // Block if we are driving Z rightnow
                if (Printer::isZMoveActive() && !Extruder::current->stepperDirection) {
                    // the pressure is outside the allowed range, we must perform the emergency stop
                    doEmergencyStop(STOP_BECAUSE_OF_Z_BLOCK);
                }
            } else {
                g_nEmergencyESkip = false;
            }
        }
    }
}

void handlePauseTime(millis_t uTime) {
    if (g_uPauseTime) {
        if (g_pauseStatus == PAUSE_STATUS_PAUSED) //and absolutly not PAUSE_STATUS_HEATING
        {
            if (g_uPauseTime > uTime) {
                return;
            }
            if ((uTime - g_uPauseTime) > (EXTRUDER_CURRENT_PAUSE_DELAY < 30000 ? 30000 : EXTRUDER_CURRENT_PAUSE_DELAY)) //das sind alle 60s
            {
#if NUM_EXTRUDER > 0
                {
                    Com::printFLN(PSTR("Pause: Saving Energy"));
                    // we have paused a few moments ago - reduce the current of the extruder motor in order to avoid unwanted heating of the filament for use cases where the printing is paused for several minutes
                    for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
                        setExtruderCurrent(i, EXTRUDER_CURRENT_PAUSED);
                        if (!extruder[i].tempControl.paused) {
                            const float target = extruder[i].tempControl.targetTemperatureC;
                            const float maxCooldown = constrain(PAUSE_COOLDOWN, 0, 255);
                            const uint8_t cooldown = static_cast<uint8_t>(constrain(target, 0.0f, maxCooldown));
                            Extruder::setTemperatureForExtruder(target - cooldown, i);
                            // Store the drop for resume after the setter clears it.
                            extruder[i].tempControl.paused = cooldown;
                        }
                    }
                    g_pauseStatus = PAUSE_STATUS_PAUSED_STANDBY;
                }
#endif
                g_uPauseTime = 0;
            }
        }
        else if (g_pauseStatus == PAUSE_STATUS_NONE
            || g_pauseStatus > PAUSE_STATUS_PAUSED
            || g_pauseStatus == PAUSE_STATUS_HEATING
            || g_pauseStatus == PAUSE_STATUS_GOTO_CONTINUE) {
            // we are not waiting for the pause event anymore
            g_uPauseTime = 0;
        }
    }
}

void handleScanWorkTasks() {
    if (g_nHeatBedScanStatus) {
        scanHeatBed();
    }
    if (g_nZOSScanStatus) {
        searchZOScan();
    }
#if FEATURE_ALIGN_EXTRUDERS
    if (g_nAlignExtrudersStatus) {
        alignExtruders();
    }
#endif // FEATURE_ALIGN_EXTRUDERS
}

void handleStartPrint() {
    if (PrintLine::linesCount > 2 && !g_uStopTime && !g_uBlockCommands) {
        // this check shall be done only during the printing (for example, it shall not be done in case filament is extruded manually)
        Printer::setPrinting(true);
    }
}

/**
 * Handle Print Stops. Printer::stopPrint()
 * -> g_uStopTime > 0 and g_uBlockCommands == 1
 * -> g_uBlockCommands
 * -> OutputObject
 * -> g_uStartOfIdle
 * -> Printer::setPrinting(false);
 */
void handleStopPrint(millis_t uTime) {
    if (g_uStopTime) {
        if (g_uStopTime > uTime) {
            return;
        }
        GCode::readFromSerial();          //consume gcode buffers but dont put gcodes into queue rightnow, because of g_uBlockCommands
        if ((uTime - g_uStopTime) > 2000) //jede 1 sekunden  wäre standard nach config
        {
            // we have stopped the printing a few moments ago, output the object now
            if (PrintLine::linesCount || PrintLine::direct.task) {
                // wait until all moves are done
                g_uStopTime = uTime;
            } else {
                // there is no printing in progress any more, do all clean-up now
                g_uStopTime = 0;


                // disable all heaters
                Extruder::setHeatedBedTemperature(0);
                Extruder::setTemperatureForAllExtruders(0);

                //unaufgeräumtes beenden: Es wird sowieso der Stepper deaktiviert.
                g_nZOSScanStatus = 0;
                g_nHeatBedScanStatus = 0;
                //g_nAlignExtrudersStatus = 0;


                Com::printFLN(PSTR("Stop complete"));
                Printer::setPrinting(false);

                g_uBlockCommands = uTime;
            }
        }
    }

    if (g_uBlockCommands > 1) //=1 scheint zu blocken, dann muss g_uStopTime aktiv sein und hier drüber erst eine Uhrzeit reinsetzen.
    {
        /*
        Prüfung unnötig, es wird niemals eine Echtzeit reingeschrieben.
        if( g_uBlockCommands > uTime ){
            return;
        }*/
        if ((uTime - g_uBlockCommands) > 1000) //jede 1 sekunden wäre standard nach config
        {
            g_uBlockCommands = 0;
            // output the object
            outputObject(false); //in g_uBlockCommands > 1
        }
    }
}

void handleStartStandby(millis_t uTime) {
    /**
     * Check if the Fan shall be turned off.
     */
#if FEATURE_CASE_FAN && !CASE_FAN_ALWAYS_ON && CASE_FAN_PIN > -1
    if (Printer::prepareFanOff) {
        if (Printer::prepareFanOff > uTime) {
            return;
        }
        if ((uTime - Printer::prepareFanOff) > Printer::fanOffDelay) //60s wäre standard nach config
        {
            // it is time to turn the case fan off
            Printer::prepareFanOff = 0;
            if (!Printer::ignoreFanOn)
                WRITE(CASE_FAN_PIN, 0);
        }
    }
#endif // FEATURE_CASE_FAN && !CASE_FAN_ALWAYS_ON && CASE_FAN_PIN > -1

    /**
     * Check if the Printer should reset the printing flag
     * And show Idle State within the display.
     */
    if (g_uStartOfIdle) {
        if (g_uStartOfIdle > uTime) {
            return;
        }
        if ((uTime - g_uStartOfIdle) > MINIMAL_IDLE_TIME) //500ms nach config
        {
            // show that we are idle for a while already
            showIdle();
            g_uStartOfIdle = 0;
            Printer::setPrinting(false);
        }
    }

    /**
     * Check if the Printer should turn off Steppers or Everything
     */
    if (PrintLine::hasLines() || Printer::isPrinting() || Printer::isMenuMode(MENU_MODE_PAUSED)) {
        previousMillisCmd = uTime; //prevent inactive shutdown of steppers/temps
    } else {
        if (previousMillisCmd > uTime) {
            return;
        }
        uTime -= previousMillisCmd;

        if (maxInactiveTime != 0 && uTime > maxInactiveTime) {
            Printer::switchEverythingOff(); //kill not only steppers
        } else {
            Printer::setAllSwitchedOff(false); // reset if not time to kill: prevents repeated kills
        }

        if (stepperInactiveTime != 0 && uTime > stepperInactiveTime && !Printer::areAllSteppersDisabled()) {
            Printer::disableAllSteppersNow();
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
            Commands::setFanSpeed((uint8_t)0);
#endif // FAN_PIN>-1 && FEATURE_FAN_CONTROL
        }
    }
}

void outputObject(bool showerrors) {
    if (PrintLine::linesCount) {
        if (showerrors)
            showError((void*)ui_text_output_object, (void*)ui_text_operation_denied);
        return;
    }
    if (!Printer::areAxisHomed()) {
        if (showerrors)
            showError((void*)ui_text_output_object, (void*)ui_text_home_unknown);
        return;
    }

    g_uStartOfIdle = 0; //outputobject starts
    UI_STATUS_UPD(UI_TEXT_OUTPUTTING_OBJECT);
    Com::printFLN(PSTR("outputObject"));
    Commands::printCurrentPosition();

#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
    // disable the fan
    Commands::setFanSpeed((uint8_t)0);
#endif // FAN_PIN>-1 && FEATURE_FAN_CONTROL

    {
        GCode::executeFString(Com::tOutputObjectPrint);
    }

    Commands::waitUntilEndOfAllMoves(); //output object

    // disable all steppers
    Printer::disableAllSteppersNow();

    g_uStartOfIdle = HAL::timeInMilliseconds(); //outputobject ends
} // outputObject

void requestPrintPause(bool moveAway) {
    InterruptProtectedBlock noInts;
    if (g_pauseMode || g_uBlockCommands || Printer::blockAll)
        return;

    g_pauseMode = moveAway ? PAUSE_MODE_PAUSED_AND_MOVED : PAUSE_MODE_PAUSED;
    g_pauseStatus = moveAway ? PAUSE_STATUS_GOTO_PAUSE_AND_MOVE : PAUSE_STATUS_GOTO_PAUSE;
}

static long pauseReturnSteps(uint8_t axis) {
    InterruptProtectedBlock noInts;
    return pauseDirectPositionSteps[axis] - Printer::directCurrentSteps[axis];
}

static bool moveDuringPause(long x, long y, long z, long e) {
    Printer::offsetRelativeStepsCoordinates(x, y, z, e);
    return g_pauseStatus == PAUSE_STATUS_PAUSING && !Printer::blockAll;
}

static bool returnToPrintPosition() {
    if (!moveDuringPause(pauseReturnSteps(X_AXIS), pauseReturnSteps(Y_AXIS), 0, 0)
        || pauseReturnSteps(X_AXIS) || pauseReturnSteps(Y_AXIS))
        return false;
    if (!moveDuringPause(0, 0, pauseReturnSteps(Z_AXIS), 0) || pauseReturnSteps(Z_AXIS))
        return false;
    return moveDuringPause(0, 0, 0, g_nContinueExtruderSteps) && !g_nContinueExtruderSteps;
}

static void handlePauseStatus() {
    if (PrintLine::direct.task || Printer::blockAll)
        return;

    const char request = g_pauseStatus;
    switch (request) {
    case PAUSE_STATUS_GOTO_PAUSE:
    case PAUSE_STATUS_GOTO_MOVED:
    case PAUSE_STATUS_GOTO_PAUSE_AND_MOVE:
        g_pauseStatus = PAUSE_STATUS_PAUSING;
        UI_STATUS_UPD(UI_TEXT_PAUSING);
        uid.exitmenu();
        if (request != PAUSE_STATUS_GOTO_MOVED) {
            // Direct motion is idle and the print queue is frozen.
            for (uint8_t axis = X_AXIS; axis <= Z_AXIS; axis++)
                pauseDirectPositionSteps[axis] = Printer::directCurrentSteps[axis];
            g_nContinueExtruderSteps = 0;
            g_uStartOfIdle = 0;
            Printer::setMenuMode(MENU_MODE_PAUSED, true);
            Com::printFLN(PSTR("RequestPause:"));
            if (!moveDuringPause(0, 0, 0, -abs(g_nPauseSteps[E_AXIS])))
                return;
        }
        if (request != PAUSE_STATUS_GOTO_PAUSE) {
            if (!moveDuringPause(0, 0, abs(g_nPauseSteps[Z_AXIS]), 0)
                || !moveDuringPause(g_nPauseSteps[X_AXIS], g_nPauseSteps[Y_AXIS], 0, 0))
                return;
        }
        UI_STATUS_UPD(UI_TEXT_PAUSED);
        break;

    case PAUSE_STATUS_GOTO_CONTINUE:
        g_pauseStatus = PAUSE_STATUS_PAUSING;
        if (returnToPrintPosition()) {
            Com::printFLN(PSTR("RequestContinue:"));
            killPausePrint();
            {
                UI_STATUS_UPD(UI_TEXT_PRINT_POS);
            }
            return;
        }
        if (g_pauseStatus != PAUSE_STATUS_PAUSING || Printer::blockAll)
            return;
        Com::printFLN(PSTR("Pause return incomplete"));
        showWarning(ui_text_pause, ui_text_operation_denied);
        break;

    default:
        return;
    }
    g_uPauseTime = HAL::timeInMilliseconds();
    g_pauseStatus = PAUSE_STATUS_PAUSED;
}

void loopFeatures() //wird so aufgerufen, dass es ein ~100ms Takt sein sollte.
{
    static char nEntered = 0;
    if (!nEntered) {
        nEntered++;

        millis_t uTime = HAL::timeInMilliseconds();
        handleStopPrint(uTime);
        handleStartPrint();

        handleStartStandby(uTime);
        handleStrainGaugeFeatures(uTime);

        handlePauseTime(uTime);

#ifdef RGB_LIGHT_R_PIN
        updateRGBLightStatus();
#endif // RGB_LIGHT_R_PIN
        if (!g_pauseMode) {
            handleScanWorkTasks();
        }

        nEntered--;
    }

    // Direct-move waits can re-enter this loop. Only pending requests start a transition.
    handlePauseStatus();
} // loopFeatures

static void waitForPauseTransition() {
    while (g_pauseMode && !isPauseSettled() && !Printer::blockAll) {
        Commands::checkForPeriodicalActions(Paused);
    }
}

static bool continuePrintLoadTemperatures() {
#if NUM_EXTRUDER > 0
    bool wait = false;
    for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
        setExtruderCurrent(i, Printer::motorCurrent[E_AXIS + i]);
        if (extruder[i].tempControl.paused) {
            // Setze Temperatur zurück auf Vor-Pause-Wert. Config aktuell nur über RFx000.h bei PAUSE_COOLDOWN
            // Wird nicht zurückgesetzt, wenn Temperatur in Pause verändert wurde. dann ist ".pause === 0"
            Extruder::setTemperatureForExtruder((float)extruder[i].tempControl.targetTemperatureC + (float)extruder[i].tempControl.paused, i);
            // extruder[i].tempControl.paused = 0; -> Unnötig, weil setTemperature .paused nullt.
            wait = true;
        }
    }
    if (wait) {
        UI_STATUS_UPD(UI_TEXT_HEATING_UP);
        for (uint8_t i = 0; i < NUM_EXTRUDER && g_pauseStatus == PAUSE_STATUS_HEATING; i++) {
            extruder[i].tempControl.waitForTargetTemperature(ADD_CONTINUE_AFTER_PAUSE_TEMP_TOLERANCE, true);
        }
        return true; // abort continue and wait for next button press because we had to wake up
    }
#endif // NUM_EXTRUDER > 0
    return false;
}

void pausePrint(void) {
    if (g_pauseMode == PAUSE_MODE_NONE) {
        if (!PrintLine::linesCount) // the printing is not paused at the moment
        {
            if (Printer::debugErrors())
                Com::printFLN(PSTR("pausePrint(): nothing is printed"));
            showError((void*)ui_text_pause, (void*)ui_text_operation_denied);

            return;
        }

        if (!Printer::areAxisHomed()) {
            if (Printer::debugErrors())
                Com::printFLN(PSTR("pausePrint(): home position is unknown"));
            showError((void*)ui_text_pause, (void*)ui_text_home_unknown);

            return;
        }

        Com::printFLN(PSTR("pausing..."));
        requestPrintPause(false);
        waitForPauseTransition();
        return;
    }

    if (!isPauseSettled())
        return;

    if (g_pauseMode == PAUSE_MODE_PAUSED) {
        g_pauseMode = PAUSE_MODE_PAUSED_AND_MOVED;
        g_pauseStatus = PAUSE_STATUS_GOTO_MOVED;
        Com::printFLN(PSTR("moving to pause position..."));
        waitForPauseTransition();
        return;
    }

    {
        // 3rd Step: go to standby fast
        if (g_pauseMode == PAUSE_MODE_PAUSED_AND_MOVED && g_pauseStatus == PAUSE_STATUS_PAUSED) {
            g_uStartOfIdle = HAL::timeInMilliseconds() + 30000;                      //hold status msg
            g_uPauseTime = HAL::timeInMilliseconds() - EXTRUDER_CURRENT_PAUSE_DELAY; //drop temps now
            UI_STATUS_UPD(UI_TEXT_COOLING_DOWN);
        }
    }
} // pausePrint

void killPausePrint(void) {
    InterruptProtectedBlock noInts;
    g_uPauseTime = 0;
    g_pauseStatus = PAUSE_STATUS_NONE;
    g_pauseMode = PAUSE_MODE_NONE;
    Printer::setMenuMode(MENU_MODE_PAUSED, false);
}

void continuePrint(void) {
    static char countplays = 1;
    if (!g_pauseMode) {
        if (!g_nHeatBedScanStatus
#if FEATURE_ALIGN_EXTRUDERS
            && !g_nAlignExtrudersStatus
#endif // FEATURE_ALIGN_EXTRUDERS
        ) {
            if (countplays++ >= 10) {
                Com::printFLN(PSTR("LCD re-init"));
                countplays = 1;
                showInformation(PSTR(UI_TEXT_MANUAL), PSTR(UI_TEXT_LCD_CIRCUIT), PSTR(UI_TEXT_RESET));
                uid.initializeLCD();
            }
        }
        return;
    }
    if (!isPauseSettled() || Printer::blockAll) {
        return;
    }

    {
        InterruptProtectedBlock noInts;
        const uint8_t directTask = PrintLine::direct.task;
        if (directTask == DIRECT_PREPARED_STOPPABLE || directTask == DIRECT_RUNNING_STOPPABLE)
            PrintLine::stopDirectMove();
        else if (directTask)
            return;
        g_pauseStatus = PAUSE_STATUS_HEATING;
    }
    countplays = 1;
    g_uPauseTime = 0;   //do not drop temps later
    g_uStartOfIdle = 0; //continueprint
    UI_STATUS_UPD(UI_TEXT_CONTINUING);
    uid.exitmenu();

    if (continuePrintLoadTemperatures()) {
        if (g_pauseStatus == PAUSE_STATUS_HEATING) {
            UI_STATUS_UPD(UI_TEXT_CONTINUING_READY);
            g_uStartOfIdle = HAL::timeInMilliseconds() + 30000;
            g_uPauseTime = HAL::timeInMilliseconds() + 240000;
            g_pauseStatus = PAUSE_STATUS_PAUSED;
        }
        return;
    }

    if (g_pauseStatus == PAUSE_STATUS_HEATING) {
        g_pauseStatus = PAUSE_STATUS_GOTO_CONTINUE;
        waitForPauseTransition();
    }
} // continuePrint

void setExtruderCurrent(uint8_t nr, uint8_t current) {
    if (nr > NUM_EXTRUDER - 1)
        nr = NUM_EXTRUDER - 1; //0 ist T0, 1 ist T1


    // set the current for the extruder motor

    setMotorCurrent(1 + E_AXIS + nr, current); //das ist Extruder-ID+1, also nicht [0,1,2] -> Extruder 4 = [3], sondern [4]. Extruder5 wäre [5]

    if (Printer::debugInfo()) {
        Com::printF(PSTR("Extruder"), (short)nr);
        Com::printFLN(PSTR(" = "), (uint32_t)current);
    }
} // setExtruderCurrent

void processSpecialGCode(GCode* pCommand) {
    long nTemp;

    if (pCommand->hasM()) {
        switch (pCommand->M) {
        case 3000: // M3000 - turn the z-compensation off
        {
            if (Printer::debugInfo()) {
                Com::printFLN(PSTR("M3000: disabling z compensation"));
            }
            queueTask(TASK_DISABLE_Z_COMPENSATION);
            break;
        }
        case 3001: // M3001 - turn the z-compensation on
        {
            if (Printer::doHeatBedZCompensation) {
                Com::printFLN(PSTR("M3001: z compensation is enabled already"));
                break;
            }

            if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
                // we try to load the z compensation matrix before its first usage because this can take some time
                prepareZCompensation();
            }

            if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
                Com::printF(PSTR("M3001: z compensation can not be enabled. Heat bed compensation matrix not valid ( "), g_ZCompensationMatrix[0][0]);
                Com::printF(PSTR(" / "), EEPROM_FORMAT);
                Com::printFLN(PSTR(" )"));
                break;
            }

            if (!Printer::areAxisHomed()) {
                Com::printFLN(PSTR("M3001: z compensation can not be enabled. Home position is unknown"));
                showError((void*)ui_text_z_compensation, (void*)ui_text_home_unknown);
                break;
            }

            queueTask(TASK_ENABLE_Z_COMPENSATION);

            //wenn der nutzer im menü "autostart" gewählt hat, dann senseoffset sofort mit M3001 mitstarten.
            //Die Einstellwerte kommen dann aus dem EEPROM
            if (Printer::g_senseoffset_autostart)
                queueTask(TASK_ENABLE_SENSE_OFFSET);
            break;
        }

        case 3005: // M3005 [S] - enable custom debug outputs
        {
            if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;
                if (nTemp < 0)
                    nTemp = 0;

                g_debugLevel = (char)nTemp;
                if (Printer::debugInfo()) {
                    Com::printFLN(PSTR("M3005: new debug level: "), g_debugLevel);
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }

        case 3006: // M3006 [S] [Z] - configure the static z-offset ( S - units are [um], Z - units are [mm] )
        {
            if (pCommand->hasZ()) {
                // test and take over the specified value
                if (pCommand->Z < -HEAT_BED_Z_COMPENSATION_MAX_MM)
                    pCommand->Z = -HEAT_BED_Z_COMPENSATION_MAX_MM;
                if (pCommand->Z > HEAT_BED_Z_COMPENSATION_MAX_MM)
                    pCommand->Z = HEAT_BED_Z_COMPENSATION_MAX_MM;

                Printer::ZOffset = long(pCommand->Z * 1000);
                // Preserve the live SenseOffset when changing the EEPROM-backed manual offset.
                g_staticZSteps = long(((Printer::ZOffset + g_nSensiblePressureOffset) * Printer::axisStepsPerMM[Z_AXIS]) / 1000);
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3006: new static z-offset: "), pCommand->Z);
                    Com::printF(PSTR(" [mm]"));
                    Com::printF(PSTR(" / "), g_staticZSteps);
                    Com::printFLN(PSTR(" [steps]"));
                }

                if (HAL::eprGetInt32(EPR_RF_Z_OFFSET) != Printer::ZOffset) {
                    HAL::eprSetInt32(EPR_RF_Z_OFFSET, Printer::ZOffset);
                    EEPROM::updateChecksum();
                }
            } else if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;

                if (nTemp < -(HEAT_BED_Z_COMPENSATION_MAX_MM * 1000))
                    nTemp = -(HEAT_BED_Z_COMPENSATION_MAX_MM * 1000);
                if (nTemp > (HEAT_BED_Z_COMPENSATION_MAX_MM * 1000))
                    nTemp = (HEAT_BED_Z_COMPENSATION_MAX_MM * 1000);

                Printer::ZOffset = nTemp;
                // Match the menu and SenseOffset controller's combined correction.
                g_staticZSteps = long(((Printer::ZOffset + g_nSensiblePressureOffset) * Printer::axisStepsPerMM[Z_AXIS]) / 1000);
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3006: new static z-offset: "), nTemp);
                    Com::printF(PSTR(" [um]"));
                    Com::printF(PSTR(" / "), g_staticZSteps);
                    Com::printFLN(PSTR(" [steps]"));
                }

                if (HAL::eprGetInt32(EPR_RF_Z_OFFSET) != Printer::ZOffset) {
                    HAL::eprSetInt32(EPR_RF_Z_OFFSET, Printer::ZOffset);
                    EEPROM::updateChecksum();
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }
        case 3009: // M3009 [S] - get/choose the active heat bed z-compensation matrix
        {
            if (Printer::doHeatBedZCompensation) {
                // do not allow to change the current heat bed z-compensation matrix while the z-compensation is active
                if (Printer::debugErrors()) {
                    Com::printFLN(PSTR("M3009: heat bed z matrix can not be changed while z-compensation is active"));
                }

                showError((void*)ui_text_z_compensation, (void*)ui_text_operation_denied);
                break;
            }

            if (pCommand->hasS()) {
                nTemp = pCommand->S;

                if (nTemp < 1 || nTemp > EEPROM_MAX_HEAT_BED_SECTORS) {
                    if (Printer::debugErrors()) {
                        Com::printF(PSTR("M3009: invalid heat bed z matrix ("), nTemp);
                        Com::printFLN(PSTR(")"));
                    }
                    break;
                }

                if (g_nActiveHeatBed != nTemp) {
                    // we have to switch to another heat bed z-compensation matrix
                    switchActiveHeatBed((char)nTemp);
                }
            }

            if (Printer::debugInfo()) {
                Com::printFLN(PSTR("M3009: currently active heat bed z matrix: "), g_nActiveHeatBed);
            }
            break;
        }
        case 3010: // M3010 [S] - start/abort the heat bed scan
        {
            if (pCommand->hasS()) {
                if (pCommand->S == HEAT_BED_SCAN_MODE_PLA || pCommand->S == HEAT_BED_SCAN_MODE_ABS) {
                    g_nHeatBedScanMode = (char)pCommand->S;
                } else {
                    g_nHeatBedScanMode = 0;
                }
            } else {
                g_nHeatBedScanMode = 0;
            }
            startHeatBedScan();
            break;
        }
        case 3011: // M3011 [S] - clear the specified z-compensation matrix from the EEPROM
        {
            if (Printer::doHeatBedZCompensation) {
                if (Printer::debugErrors()) {
                    Com::printFLN(PSTR("M3011: the heat bed z matrix can not be cleared while the z-compensation is active"));
                }

                showError((void*)ui_text_z_compensation, (void*)ui_text_operation_denied);
                break;
            }
            if (pCommand->hasS()) {
                nTemp = pCommand->S;
            } else {
                // we clear the current z-compensation matrix in case no other z-compensation matrix is specified
                nTemp = g_nActiveHeatBed;
            }

            if (nTemp < 1 || nTemp > EEPROM_MAX_HEAT_BED_SECTORS) {
                if (Printer::debugErrors()) {
                    Com::printF(PSTR("M3011: invalid heat bed z matrix ("), nTemp);
                    Com::printFLN(PSTR(")"));
                }
                break;
            }

            if (saveActiveHeatBedToExtEEPROM((char)nTemp)) {
                // switch to the specified z-compensation matrix
                g_nActiveHeatBed = (char)nTemp;
                clearCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed));
                Com::printFLN(PSTR("M3011: cleared heat bed z matrix: "), nTemp);
            }
            break;
        }
        case 3012: // M3012 - restore the default scan parameters
        {
            restoreDefaultScanParameters();
            break;
        }
        case 3013: // M3013 [S] [P] - output the current heat bed z-compensation matrix
        {
            char format;
            char nOldHeatBed = 127;

            if (pCommand->hasS()) {
                nTemp = pCommand->S;
            } else {
                // we output the current z-compensation matrix in case no other z-compensation matrix is specified
                nTemp = g_nActiveHeatBed;
            }

            if (pCommand->hasP()) {
                format = pCommand->P ? 1 : 0;
            } else {
                format = 0;
            }

            if (nTemp < 1 || nTemp > EEPROM_MAX_HEAT_BED_SECTORS) {
                if (Printer::debugErrors()) {
                    Com::printF(PSTR("M3013: invalid heat bed z matrix ("), nTemp);
                    Com::printFLN(PSTR(")"));
                }
                break;
            }

            if (g_nActiveHeatBed != nTemp) {
                // remember current z-compensation matrix
                nOldHeatBed = g_nActiveHeatBed;
                // we have to switch to another z-compensation matrix
                switchActiveHeatBed((char)nTemp);
            }

            if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
                // we load the z-compensation matrix before its first usage because this can take some time
                prepareZCompensation();
            }

            if (g_ZCompensationMatrix[0][0] == EEPROM_FORMAT) {
                if (Printer::debugInfo()) {
                    Com::printFLN(PSTR("M3013: current heat bed z-compensation matrix: "));
                }
                outputCompensationMatrix(format);
            } else {
                if (Printer::debugErrors()) {
                    Com::printF(PSTR("M3013: the heat bed z-compensation matrix is not valid ( "), g_ZCompensationMatrix[0][0]);
                    Com::printF(PSTR(" / "), EEPROM_FORMAT);
                    Com::printFLN(PSTR(" )"));
                }

                showError((void*)ui_text_z_compensation, (void*)ui_text_invalid_matrix);
            }

            if (nOldHeatBed != 127) {
                // restore z-compensation matrix, in case we switched them
                switchActiveHeatBed(nOldHeatBed);
            }
            break;
        }
        case 3020: // M3020 [S] - configure the x start position for the heat bed scan ( units are [mm] )
        {
            if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;
                if (nTemp < 5)
                    nTemp = 5;
                if (nTemp > (Printer::axisLengthMM[X_AXIS] - 5))
                    nTemp = Printer::axisLengthMM[X_AXIS] - 5;

                g_nScanXStartSteps = (long)((float)nTemp * Printer::axisStepsPerMM[X_AXIS]);
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3020: new x start position: "), nTemp);
                    Com::printF(PSTR(" [mm], "), (int)g_nScanXStartSteps);
                    Com::printFLN(PSTR(" [steps]"));
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }
        case 3021: // M3021 [S] - configure the y start position for the heat bed scan ( units are [mm] )
        {
            if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;
                if (nTemp < 5)
                    nTemp = 5;
                if (nTemp > (Printer::axisLengthMM[Y_AXIS] - 5))
                    nTemp = Printer::axisLengthMM[Y_AXIS] - 5;

                g_nScanYStartSteps = (long)((float)nTemp * Printer::axisStepsPerMM[Y_AXIS]);
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3021: new y start position: "), nTemp);
                    Com::printF(PSTR(" [mm], "), (int)g_nScanYStartSteps);
                    Com::printFLN(PSTR(" [steps]"));
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }
        case 3022: // M3022 [S] - configure the x step size for the heat bed scan ( units are [mm] )
        {
            if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;
                if (nTemp < HEAT_BED_SCAN_X_STEP_SIZE_MIN_MM)
                    nTemp = HEAT_BED_SCAN_X_STEP_SIZE_MIN_MM;
                if (nTemp > 100)
                    nTemp = 100;

                g_nScanXStepSizeSteps = (long)((float)nTemp * Printer::axisStepsPerMM[X_AXIS]);
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3022: new x step size: "), (int)nTemp);
                    Com::printF(PSTR(" [mm], "), (int)g_nScanXStepSizeSteps);
                    Com::printFLN(PSTR(" [steps]"));
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }
        case 3023: // M3023 [S] - configure the y step size for the heat bed scan ( units are [mm] )
        {
            if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;
                if (nTemp < HEAT_BED_SCAN_Y_STEP_SIZE_MIN_MM)
                    nTemp = HEAT_BED_SCAN_Y_STEP_SIZE_MIN_MM;
                if (nTemp > 100)
                    nTemp = 100;

                g_nScanYStepSizeSteps = (long)((float)nTemp * Printer::axisStepsPerMM[Y_AXIS]);
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3023: new y step size: "), (int)nTemp);
                    Com::printF(PSTR(" [mm], "), (int)g_nScanYStepSizeSteps);
                    Com::printFLN(PSTR(" [steps]"));
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }
        case 3024: // M3024 [S] - configure the x end position for the heat bed scan ( units are [mm] )
        {
            if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;
                if (nTemp < 5)
                    nTemp = 5;
                if (nTemp > (Printer::axisLengthMM[X_AXIS] - 5))
                    nTemp = Printer::axisLengthMM[X_AXIS] - 5;

                g_nScanXEndSteps = (long)((float)nTemp * Printer::axisStepsPerMM[X_AXIS]);
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3024: new x end position: "), nTemp);
                    Com::printF(PSTR(" [mm], "), (int)g_nScanXEndSteps);
                    Com::printFLN(PSTR(" [steps]"));
                }

                g_nScanXMaxPositionSteps = long(Printer::axisLengthMM[X_AXIS] * Printer::axisStepsPerMM[X_AXIS] - g_nScanXEndSteps);
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3024: new x max position: "), (int)g_nScanXMaxPositionSteps);
                    Com::printFLN(PSTR(" [steps]"));
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }
        case 3025: // M3025 [S] - configure the y end position for the heat bed scan ( units are [mm] )
        {
            if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;
                if (nTemp < 5)
                    nTemp = 5;
                if (nTemp > (Printer::axisLengthMM[Y_AXIS] - 5))
                    nTemp = Printer::axisLengthMM[Y_AXIS] - 5;

                g_nScanYEndSteps = (long)((float)nTemp * Printer::axisStepsPerMM[Y_AXIS]);
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3025: new y end position: "), nTemp);
                    Com::printF(PSTR(" [mm], "), (int)g_nScanYEndSteps);
                    Com::printFLN(PSTR(" [steps]"));
                }

                g_nScanYMaxPositionSteps = long(Printer::axisLengthMM[Y_AXIS] * Printer::axisStepsPerMM[Y_AXIS] - g_nScanYEndSteps);
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3025: new y max position: "), (int)g_nScanYMaxPositionSteps);
                    Com::printFLN(PSTR(" [steps]"));
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }

#if FEATURE_CHECK_HOME
        case 3028: // M3028
        {
            if (pCommand->hasX()) {
                Printer::checkHome(X_AXIS);
            }
            if (pCommand->hasY()) {
                Printer::checkHome(Y_AXIS);
            }
            if (pCommand->hasZ()) {
                Printer::checkHome(Z_AXIS);
            }
            break;
        }
#endif // FEATURE_CHECK_HOME

#if FEATURE_SEE_DISPLAY
        case 3029: // M3029 [P] - See the display text or send Button press per gcode to the printer via [P]Code.
        {
            if (pCommand->hasP()) {
                switch (pCommand->P) {
                case UI_ACTION_OK:       // 1001
                case UI_ACTION_NEXT:     // 1
                case UI_ACTION_PREVIOUS: // 2
                case UI_ACTION_BACK:     // 1000
                case UI_ACTION_RIGHT:    // 1129

                case UI_ACTION_RF_HEAT_BED_UP:      // 514
                case UI_ACTION_RF_HEAT_BED_DOWN:    // 515
                case UI_ACTION_RF_EXTRUDER_RETRACT: // 517
                case UI_ACTION_RF_EXTRUDER_OUTPUT:  // 516
                case UI_ACTION_RF_CONTINUE:         // 1519
                case UI_ACTION_RF_PAUSE:            // 1518
                {
                    Com::printFLN(PSTR("RequestMenu:Press:"), pCommand->P);
                    uid.executeAction(pCommand->P);
                } break;
                }
            } else {
                extern char displayCache[UI_ROWS][MAX_COLS + 1];
                Com::printF(PSTR("RequestMenu:"), UI_ROWS);

                Com::printF(PSTR(","), MAX_COLS + 1);
                Com::printF(PSTR(":"));
                if (pCommand->hasL()) {
                    //print line by line for human readability
                    Com::println();
                    Com::println();
                }
                // we print out UI_ROWS x MAX_COLS+1 field.
                for (uint8_t row = 0; row < UI_ROWS; row++) {
                    bool nulled = false;
                    for (uint8_t col = 0; col < MAX_COLS + 1; col++) {
                        if (displayCache[row][col] == 0 || nulled) {
                            nulled = true;
                            Com::print(' '); //null as space
                            continue;
                        }

                        if (isprint(displayCache[row][col])) {
                            Com::print(displayCache[row][col]);
                            continue;
                        }

                        switch (displayCache[row][col]) {
                        case 2: {
                            Com::print('\'');
                            break;
                        }
                        case 5: {
                            Com::print('!');
                            break;
                        }
                        case (char)CHAR_SELECTOR: {
                            Com::print('>');
                            break;
                        }
                        case (char)CHAR_SELECTED: {
                            Com::print('*');
                            break;
                        }
                        default:
                            Com::print('_'); //others?
                        }
                    }
                    if (pCommand->hasL()) {
                        //print line by line for human readability
                        Com::println();
                    }
                }
                Com::println();
            }
            break;
        }
#endif // FEATURE_SEE_DISPLAY

        case 3030: // M3030 [S] - configure the fast step size for moving of the heat bed up during the heat-bed scan
        {
            if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;
                if (nTemp > 50)
                    nTemp = 50;
                if (nTemp < 1)
                    nTemp = 1;

                g_nScanHeatBedUpFastSteps = -nTemp;
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3030: new fast step size for moving of the bed up: "), (int)g_nScanHeatBedUpFastSteps);
                    Com::printFLN(PSTR(" [steps]"));
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }
        case 3031: // M3031 [S] - configure the slow step size for moving of the heat bed up during the heat-bed scan
        {
            if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;
                if (nTemp > 50)
                    nTemp = 50;
                if (nTemp < 1)
                    nTemp = 1;

                g_nScanHeatBedUpSlowSteps = -nTemp;
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3031: new slow step size for moving of the bed up: "), (int)g_nScanHeatBedUpSlowSteps);
                    Com::printFLN(PSTR(" [steps]"));
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }
        case 3032: // M3032 [S] - configure the fast step size for moving of the heat bed down during the heat-bed scan
        {
            if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;
                if (nTemp < Printer::axisStepsPerMM[Z_AXIS] / 20)
                    nTemp = Printer::axisStepsPerMM[Z_AXIS] / 20;
                if (nTemp > Printer::axisStepsPerMM[Z_AXIS] * 5)
                    nTemp = Printer::axisStepsPerMM[Z_AXIS] * 5;

                g_nScanHeatBedDownFastSteps = nTemp;
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3032: new fast step size for moving of the bed down: "), (int)g_nScanHeatBedDownFastSteps);
                    Com::printFLN(PSTR(" [steps]"));
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }
        case 3033: // M3033 [S] - configure the slow step size for moving of the heat bed down during the heat-bed scan
        {
            if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;
                if (nTemp < 1)
                    nTemp = 1;
                if (nTemp > 50)
                    nTemp = 50;

                g_nScanHeatBedDownSlowSteps = nTemp;
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3033: new slow step size for moving of the bed down: "), (int)g_nScanHeatBedDownSlowSteps);
                    Com::printFLN(PSTR(" [steps]"));
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }
        case 3040: // M3040 [S] - configure the delay (in ms) between two fast movements during the heat-bed scan
        {
            if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;
                if (nTemp < 1)
                    nTemp = 1;
                if (nTemp > 1000)
                    nTemp = 1000;

                g_nScanFastStepDelay = nTemp;
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3040: new delay between two fast movements: "), (int)g_nScanFastStepDelay);
                    Com::printFLN(PSTR(" [ms]"));
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }
        case 3041: // M3041 [S] - configure the delay (in ms) between two slow movements during the heat-bed scan
        {
            if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;
                if (nTemp < 1)
                    nTemp = 1;
                if (nTemp > 1000)
                    nTemp = 1000;

                g_nScanSlowStepDelay = nTemp;
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3041: new delay between two slow movements: "), (int)g_nScanSlowStepDelay);
                    Com::printFLN(PSTR(" [ms]"));
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }
        case 3042: // M3042 [S] - configure the delay (in ms) between reaching of a new x/y position and the test of the idle pressure
        {
            if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;
                if (nTemp < 1)
                    nTemp = 1;
                if (nTemp > 10000)
                    nTemp = 10000;

                g_nScanIdleDelay = nTemp;
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3042: new idle delay: "), (int)g_nScanIdleDelay);
                    Com::printFLN(PSTR(" [ms]"));
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }
        case 3050: // M3050 [S] - configure the contact pressure delta (in digits)
        {
            if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;
                if (nTemp < 1)
                    nTemp = 1;
                if (nTemp > 1000)
                    nTemp = 1000;

                g_nScanContactPressureDelta = nTemp;
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3050: new contact pressure delta: "), (int)g_nScanContactPressureDelta);
                    Com::printFLN(PSTR(" [digits]"));
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }
        case 3051: // M3051 [S] - configure the retry pressure delta (in digits)
        {
            if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;
                if (nTemp < 1)
                    nTemp = 1;
                if (nTemp > 1000)
                    nTemp = 1000;

                g_nScanRetryPressureDelta = nTemp;
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3051: new retry pressure delta: "), (int)g_nScanRetryPressureDelta);
                    Com::printFLN(PSTR(" [digits]"));
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }
        case 3052: // M3052 [S] - configure the idle pressure tolerance (in digits)
        {
            if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;
                if (nTemp < 1)
                    nTemp = 1;
                if (nTemp > 1000)
                    nTemp = 1000;

                g_nScanIdlePressureDelta = nTemp;
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3052: new idle pressure delta: "), (int)g_nScanIdlePressureDelta);
                    Com::printFLN(PSTR(" [digits]"));
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }
        case 3053: // M3053 [S] - configure the number of A/D converter reads per pressure measurement
        {
            if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;
                if (nTemp < 1)
                    nTemp = 1;
                if (nTemp > 100)
                    nTemp = 100;

                g_nScanPressureReads = nTemp;
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3053: new pressure reads per measurement: "), (int)g_nScanPressureReads);
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }
        case 3054: // M3054 [S] - configure the delay (in ms) between two A/D converter reads
        {
            if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;
                if (nTemp < 1)
                    nTemp = 1;
                if (nTemp > 1000)
                    nTemp = 1000;

                g_nScanPressureReadDelay = nTemp;
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3054: new delay between two pressure reads: "), (int)g_nScanPressureReadDelay);
                    Com::printFLN(PSTR(" [ms]"));
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }
        case 3055: // M3055 [S] - configure the pressure tolerance (in digits) per pressure measurement
        {
            if (pCommand->hasS()) {
                // test and take over the specified value
                nTemp = pCommand->S;
                if (nTemp < 1)
                    nTemp = 1;
                if (nTemp > 1000)
                    nTemp = 1000;

                g_nScanPressureTolerance = nTemp;
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3055: new scan pressure tolerance: "), (int)g_nScanPressureTolerance);
                    Com::printFLN(PSTR(" [digits]"));
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }

        case 3060: // M3060 - output the device type and firmware version
        {
            Com::printFLN(PSTR("Device Type: "), UI_PRINTER_NAME);
            Com::printFLN(PSTR("Firmware Version: "), UI_VERSION_STRING);
            break;
        }

        case 3070: // M3070 [S] - pause the print as if the "Pause" button would have been pressed
        {
            //put pause task into MOVE_CACHE
            if (pCommand->hasS() && pCommand->S >= 2 && Printer::areAxisHomed()) {
                // we shall pause the printing and we shall move away
                queueTask(TASK_PAUSE_PRINT_AND_MOVE);
            } else {
                // we shall pause within print queue and stay where we are
                queueTask(TASK_PAUSE_PRINT);
            }
            Commands::waitUntilEndOfAllMoves(); //M400 (normal gcode wait)
            break;
        }

        case 3075: // M3075 [S] [P] - configure the emergency pause digits
        {
            long nMin = g_nEmergencyPauseDigitsMin;
            long nMax = g_nEmergencyPauseDigitsMax;

            if (pCommand->hasS()) {
                // test and take over the specified value - this is our new min value
                nMin = pCommand->S;
            }
            if (pCommand->hasP()) {
                // test and take over the specified value - this is our new max value
                nMax = pCommand->P;
            }

            if (nMin < EMERGENCY_PAUSE_DIGITS_MIN)
                nMin = EMERGENCY_PAUSE_DIGITS_MIN;
            if (nMax > EMERGENCY_PAUSE_DIGITS_MAX)
                nMax = EMERGENCY_PAUSE_DIGITS_MAX;

            if (nMin == 0 && nMax == 0) {
                g_nEmergencyPauseDigitsMin = 0;
                g_nEmergencyPauseDigitsMax = 0;
                HAL::eprSetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMIN, g_nEmergencyPauseDigitsMin);
                HAL::eprSetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMAX, g_nEmergencyPauseDigitsMax);
                EEPROM::updateChecksum();
                if (Printer::debugInfo()) {
                    Com::printFLN(PSTR("M3075: emergency pause disabled"));
                }
            } else if (nMin < nMax) {
                g_nEmergencyPauseDigitsMin = nMin;
                g_nEmergencyPauseDigitsMax = nMax;
                HAL::eprSetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMIN, g_nEmergencyPauseDigitsMin);
                HAL::eprSetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMAX, g_nEmergencyPauseDigitsMax);
                EEPROM::updateChecksum(); //deshalb die prüfung
                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3075: new min: "), (int)g_nEmergencyPauseDigitsMin);
                    Com::printF(PSTR(" [digits], new max: "), (int)g_nEmergencyPauseDigitsMax);
                    Com::printFLN(PSTR(" [digits]"));
                }
            } else {
                if (Printer::debugErrors()) {
                    Com::printF(PSTR("M3075: min max ("), (int)nMin);
                    Com::printF(Com::tSlash, (int)nMin);
                    Com::printFLN(PSTR(") invalide"));
                }
            }

            break;
        }

        case 3076: // M3076 [S] [P] - configure the emergency stop digits
        {
            long nMin = g_nEmergencyStopZAndEMin;
            long nMax = g_nEmergencyStopZAndEMax;

            if (pCommand->hasS()) {
                // test and take over the specified value - this is our new min value
                nMin = pCommand->S;
            }
            if (pCommand->hasP()) {
                // test and take over the specified value - this is our new max value
                nMax = pCommand->P;
            }

            if (nMin == 0 && nMax == 0) {
                g_nEmergencyStopZAndEMin = 0;
                g_nEmergencyStopZAndEMax = 0;

                if (Printer::debugInfo()) {
                    Com::printFLN(PSTR("M3076: emerg zstop temp. disabled"));
                }
            } else if (nMin < nMax && nMin >= -32768 && nMax <= 32767) {
                g_nEmergencyStopZAndEMin = (short)nMin;
                g_nEmergencyStopZAndEMax = (short)nMax;

                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3076: new min: "), (int)g_nEmergencyStopZAndEMin);
                    Com::printF(PSTR(" [digits], new max: "), (int)g_nEmergencyStopZAndEMax);
                    Com::printFLN(PSTR(" [digits]"));
                }
                HAL::eprSetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMAX, g_nEmergencyStopZAndEMax);
                HAL::eprSetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMIN, g_nEmergencyStopZAndEMin);
                EEPROM::updateChecksum();
            } else {
                if (Printer::debugErrors()) {
                    Com::printF(PSTR("M3076: min max ("), (int)nMin);
                    Com::printF(Com::tSlash, (int)nMax);
                    Com::printFLN(PSTR(") invalide"));
                }
            }

            break;
        }

        case 3079: // M3079 - output the printed object
        {
            outputObject(); //als Gcode direkt
            break;
        }

        // FEATURE_WATCHDOG
        case 3090: // M3090 - test the watchdog (this command resets the firmware)
        {
            if (pCommand->hasS()) {
                if (pCommand->S == 666) {
                    if (g_bPingWatchdog) {
                        HAL::stopWatchdog();
                        Com::printFLN(PSTR("M3090: WARNING the watchdog is disabled now"));
                    } else {
                        HAL::startWatchdog();
                        Com::printFLN(PSTR("M3090: the watchdog has been activated"));
                    }
                }
            } else if (pCommand->hasP()) {
                HAL::delayMilliseconds(10000);
            } else if (pCommand->hasT()) {
                for (int iii = 0; iii < 1000; iii++)
                    HAL::delayMicroseconds(10000);
            } else {
                if (Printer::debugInfo()) {
                    Com::printFLN(PSTR("M3090: the watchdog is going to reset the firmware"));
                }
                HAL::delayMilliseconds(100);
                HAL::testWatchdog();
            }
            break;
        }

        case 3091: // M3091 - erase the external EEPROM
        {
            clearExternalEEPROM();
            break;
        }

        case 3105: // M3105 [X] [Y] [Z] [E] - configure the offset in x, y, z and e direction which shall be applied in case the "Pause" button has been pressed ( units are [mm] )
        {
            if (pCommand->hasNoXYZ() && !pCommand->hasE()) {
                showInvalidSyntax(pCommand->M);
            } else {
                if (pCommand->hasX()) {
                    // test and take over the specified value
                    nTemp = pCommand->X * Printer::axisStepsPerMM[X_AXIS];
                    if (nTemp < -Printer::maxSoftEndstopSteps[X_AXIS])
                        nTemp = -Printer::maxSoftEndstopSteps[X_AXIS];
                    if (nTemp > Printer::maxSoftEndstopSteps[X_AXIS])
                        nTemp = Printer::maxSoftEndstopSteps[X_AXIS];

                    g_nPauseSteps[X_AXIS] = nTemp;
                    Com::printF(PSTR("M3105: new x pause offset: "), nTemp * Printer::axisMMPerSteps[X_AXIS]);
                    Com::printFLN(PSTR(" [mm]"));
                }
                if (pCommand->hasY()) {
                    // test and take over the specified value
                    nTemp = pCommand->Y * Printer::axisStepsPerMM[Y_AXIS];
                    if (nTemp < -Printer::maxSoftEndstopSteps[Y_AXIS])
                        nTemp = -Printer::maxSoftEndstopSteps[Y_AXIS];
                    if (nTemp > Printer::maxSoftEndstopSteps[Y_AXIS])
                        nTemp = Printer::maxSoftEndstopSteps[Y_AXIS];

                    g_nPauseSteps[Y_AXIS] = nTemp;
                    Com::printF(PSTR("M3105: new y pause offset: "), nTemp * Printer::axisMMPerSteps[Y_AXIS]);
                    Com::printFLN(PSTR(" [mm]"));
                }
                if (pCommand->hasZ()) {
                    // test and take over the specified value
                    nTemp = pCommand->Z * Printer::axisStepsPerMM[Z_AXIS];
                    if (nTemp < 0)
                        nTemp = 0; //no pause within printed part.
                    if (nTemp > Printer::maxSoftEndstopSteps[Z_AXIS])
                        nTemp = Printer::maxSoftEndstopSteps[Z_AXIS];

                    g_nPauseSteps[Z_AXIS] = nTemp;
                    Com::printF(PSTR("M3105: new z pause offset: "), nTemp * Printer::axisMMPerSteps[Z_AXIS]);
                    Com::printFLN(PSTR(" [mm]"));
                }
                if (pCommand->hasE()) {
                    // test and take over the specified value
                    nTemp = pCommand->E;
                    if (nTemp < 0)
                        nTemp = 0;
                    if (nTemp > 5)
                        nTemp = 5; //Zahl immer positiv, sie wird abgezogen = Retract!

                    g_nPauseSteps[E_AXIS] = (long)((float)nTemp * Extruder::current->stepsPerMM);
                    Com::printF(PSTR("M3105: new extruder pause offset: "), nTemp);
                    Com::printFLN(PSTR(" [mm]"));
                }
            }
            break;
        }

        case 3115: // M3115 - set the x/y origin to the current x/y position
        {
            Printer::setOrigin(-Printer::destinationMM[X_AXIS], -Printer::destinationMM[Y_AXIS], Printer::originOffsetMM[Z_AXIS]);
            break;
        }
        case 3117: // M3117 - set a status text which is not overwritten by M117
        {
            if (pCommand->hasString()) {
                if (!uid.locked) {
                    // ensure that the current text won't be overwritten
                    uid.lock();
                    if (Printer::debugInfo()) {
                        Com::printFLN(PSTR("M3117: lock"));
                    }
                }
                uid.setStatus(pCommand->text, false, true);
                uid.refreshPage();
            } else {
                if (uid.locked) {
                    // allow to overwrite the current string again
                    // Put the unlock into queue to avoid racetime conditions with prior commands
                    queueTask(TASK_UNLOCK_DISPLAY_MSG);
                    Com::printFLN(PSTR("M3117: queued unlock"));
                }
            }
            break;
        }

#if FEATURE_CASE_FAN && !CASE_FAN_ALWAYS_ON && CASE_FAN_PIN > -1
        case 3120: // M3120 - turn on the case fan
        {
            //disable fan-temp-ignore to original state // Nibbels
            Printer::ignoreFanOn = false;

            // enable the case fan
            Printer::prepareFanOff = 0;
            WRITE(CASE_FAN_PIN, 1);

            if (Printer::debugInfo()) {
                Com::printFLN(PSTR("M3120: fan on"));
            }
            break;
        }

        case 3121: // M3121 - turn off the case fan
        {
            // disable the case fan
            if (pCommand->hasS()) {
                // we shall set a new case fan off delay
                Printer::fanOffDelay = pCommand->S;
                Printer::fanOffDelay *= 1000; // convert from [s] to [ms]
            }

            if (pCommand->hasP()) {
                // the fan should be disabled even if the print starts // temp > fan on temp // Nibbels
                Printer::ignoreFanOn = true;
            }

            if (Printer::fanOffDelay) {
                // we are going to disable the case fan after the delay
                Printer::prepareFanOff = HAL::timeInMilliseconds();

                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3121: fan off in "), pCommand->S);
                    Com::printFLN(PSTR(" [s]"));
                }
            } else {
                // we are going to disable the case fan now
                Printer::prepareFanOff = 0;
                WRITE(CASE_FAN_PIN, 0);

                if (Printer::debugInfo()) {
                    Com::printFLN(PSTR("M3121: fan off"));
                }
            }
            break;
        }
#endif // FEATURE_CASE_FAN && !CASE_FAN_ALWAYS_ON && CASE_FAN_PIN > -1



        case 3200: // M3200 - reserved for test and debug
        {
            if (pCommand->hasP()) {
                switch (pCommand->P) {
                case 1: {
                    Commands::checkFreeMemory();
                    Commands::writeLowestFreeRAM();
                    Com::printFLN(PSTR("lowest free RAM: "), Commands::lowestRAMValue);

                    Com::printFLN(PSTR("z-compensation matrix x: "), COMPENSATION_MATRIX_MAX_X);
                    Com::printFLN(PSTR("z-compensation matrix y: "), COMPENSATION_MATRIX_MAX_Y);
                    break;
                }
                case 2: {
                    Com::printFLN(PSTR("Homing feedrates:"));
                    Com::printF(PSTR("x = "), Printer::homingFeedrate[X_AXIS]);
                    Com::printF(PSTR(", y = "), Printer::homingFeedrate[Y_AXIS]);
                    Com::printFLN(PSTR(", z = "), Printer::homingFeedrate[Z_AXIS]);
                    break;
                }
                case 4: {
                    // simulate blocking of all axes
                    Com::printFLN(PSTR("M3200: block all"));

                    doEmergencyStop(STOP_BECAUSE_OF_Z_BLOCK);
                    break;
                }
                case 5: {
                    // simulate a temp sensor error
                    Com::printFLN(PSTR("M3200: simulating defect sensor"));
                    //ext 0 soll defekt aussehen
                    TemperatureController* act0 = tempController[0];
                    act0->setSensorDefect(true);
                    //ext 1/bett soll decoupled aussehen
                    TemperatureController* act1 = tempController[1];
                    act1->setSensorDecoupled(true);
                    //generelle defekt-einstufung
                    Printer::setSomeTempsensorDefect(true);
                    //fehlermeldung ausgeben
                    reportTempsensorAndHeaterErrors();
                    break;
                }


                case 9: {
                    Com::printFLN(PSTR("debug level: "), Printer::debugLevel);
                    break;
                }

                case 13: {
                    Com::printFLN(PSTR("LCD re-initialization"));
                    uid.initializeLCD();
                }
                }
            }

            break;
        }

#if FEATURE_24V_FET_OUTPUTS
        case 3300: // M3300 [P] [S] - configure the 24V FET outputs ( on/off )
        {
            if (pCommand->hasP()) {
                switch (pCommand->P) {
#if FET1 > -1
                case 1: {
                    if (pCommand->hasS()) {
                        if (pCommand->S == 0) {
                            Printer::enableFET1 = 0;
                            WRITE(FET1, Printer::enableFET1);
                            Com::printFLN(PSTR("24V X42 FET1-output = off"));
                        } else {
                            Printer::enableFET1 = 1;
                            WRITE(FET1, Printer::enableFET1);
                            Com::printFLN(PSTR("24V X42 FET1-output = on"));
                        }
                        HAL::eprSetByte(EPR_RF_FET1_MODE, Printer::enableFET1);
                        EEPROM::updateChecksum();
                    }
                    break;
                }
#endif // FET1
#if FET2 > -1
                case 2: {
                    if (pCommand->hasS()) {
                        if (pCommand->S == 0) {
                            Printer::enableFET2 = 0;
                            WRITE(FET2, Printer::enableFET2);
                            Com::printFLN(PSTR("24V X44 FET2-output = off"));
                        } else {
                            Printer::enableFET2 = 1;
                            WRITE(FET2, Printer::enableFET2);
                            Com::printFLN(PSTR("24V X44 FET2-output = on"));
                        }
                        HAL::eprSetByte(EPR_RF_FET2_MODE, Printer::enableFET2);
                        EEPROM::updateChecksum();
                    }
                    break;
                }
#endif // FET2
#if FET3 > -1
                case 3: {
                    if (pCommand->hasS()) {
                        if (pCommand->S == 0) {
                            Printer::enableFET3 = 0;
                            WRITE(FET3, Printer::enableFET3);
                            Com::printFLN(PSTR("24V X45/BoardFan FET3-output = off"));
                        } else {
                            Printer::enableFET3 = 1;
                            WRITE(FET3, Printer::enableFET3);
                            Com::printFLN(PSTR("24V X45/BoardFan FET3-output = on"));
                        }
                        HAL::eprSetByte(EPR_RF_FET3_MODE, Printer::enableFET3);
                        EEPROM::updateChecksum();
                    }
                    break;
                }
#endif // FET3
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }

            break;
        }
#endif // FEATURE_24V_FET_OUTPUTS

#if FEATURE_230V_OUTPUT
        case 3301: // M3301 [S] - configure the 230V output ( on/off )
        {
            if (pCommand->hasS()) {
                if (pCommand->S == 0) {
                    Printer::enable230VOutput = 0;
                    WRITE(OUTPUT_230V_PIN, Printer::enable230VOutput);
                    Com::printFLN(PSTR("230V output = off"));
                } else {
                    Printer::enable230VOutput = 1;
                    WRITE(OUTPUT_230V_PIN, Printer::enable230VOutput);
                    Com::printFLN(PSTR("230V output = on"));
                }
            } else {
                showInvalidSyntax(pCommand->M);
            }
            break;
        }
#endif // FEATURE_230V_OUTPUT

#ifdef RGB_LIGHT_R_PIN
        case 3303: // M3303 [P] [S] - configure the RGB light effects for heating
        {
            if (pCommand->hasP()) {
                switch (pCommand->P) {
                case 1: // red
                {
                    if (pCommand->hasS()) {
                        if (pCommand->S >= 0 && pCommand->S <= 255) {
                            g_uRGBHeatingR = pCommand->S;

                            HAL::eprSetByte(EPR_RF_RGB_HEATING_R, g_uRGBHeatingR);
                            EEPROM::updateChecksum();

                            if (Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_HEATING) {
                                setRGBTargetColors(g_uRGBHeatingR, g_uRGBHeatingG, g_uRGBHeatingB);
                            }
                            Com::printFLN(PSTR("RGB_HEATING_R = "), g_uRGBHeatingR);
                        } else {
                            Com::printFLN(PSTR("RGB_HEATING_R out of range "));
                        }
                    }
                    break;
                }
                case 2: // green
                {
                    if (pCommand->hasS()) {
                        if (pCommand->S >= 0 && pCommand->S <= 255) {
                            g_uRGBHeatingG = pCommand->S;

                            HAL::eprSetByte(EPR_RF_RGB_HEATING_G, g_uRGBHeatingG);
                            EEPROM::updateChecksum();

                            if (Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_HEATING) {
                                setRGBTargetColors(g_uRGBHeatingR, g_uRGBHeatingG, g_uRGBHeatingB);
                            }
                            Com::printFLN(PSTR("RGB_HEATING_G = "), g_uRGBHeatingG);
                        } else {
                            Com::printFLN(PSTR("RGB_HEATING_G out of range "));
                        }
                    }
                    break;
                }
                case 3: // blue
                {
                    if (pCommand->hasS()) {
                        if (pCommand->S >= 0 && pCommand->S <= 255) {
                            g_uRGBHeatingB = pCommand->S;

                            HAL::eprSetByte(EPR_RF_RGB_HEATING_B, g_uRGBHeatingB);
                            EEPROM::updateChecksum();

                            if (Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_HEATING) {
                                setRGBTargetColors(g_uRGBHeatingR, g_uRGBHeatingG, g_uRGBHeatingB);
                            }
                            Com::printFLN(PSTR("RGB_HEATING_B = "), g_uRGBHeatingB);
                        } else {
                            Com::printFLN(PSTR("RGB_HEATING_B out of range "));
                        }
                    }
                    break;
                }
                }
            }

            break;
        }
        case 3304: // M3304 [P] [S] - configure the RGB light effects for printing
        {
            if (pCommand->hasP()) {
                switch (pCommand->P) {
                case 1: // red
                {
                    if (pCommand->hasS()) {
                        if (pCommand->S >= 0 && pCommand->S <= 255) {
                            g_uRGBPrintingR = pCommand->S;

                            HAL::eprSetByte(EPR_RF_RGB_PRINTING_R, g_uRGBPrintingR);
                            EEPROM::updateChecksum();

                            if (Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_PRINTING) {
                                setRGBTargetColors(g_uRGBPrintingR, g_uRGBPrintingG, g_uRGBPrintingB);
                            }
                            Com::printFLN(PSTR("RGB_PRINTING_R = "), g_uRGBPrintingR);
                        } else {
                            Com::printFLN(PSTR("RGB_PRINTING_R out of range "));
                        }
                    }
                    break;
                }
                case 2: // green
                {
                    if (pCommand->hasS()) {
                        if (pCommand->S >= 0 && pCommand->S <= 255) {
                            g_uRGBPrintingG = pCommand->S;

                            HAL::eprSetByte(EPR_RF_RGB_PRINTING_G, g_uRGBPrintingG);
                            EEPROM::updateChecksum();

                            if (Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_PRINTING) {
                                setRGBTargetColors(g_uRGBPrintingR, g_uRGBPrintingG, g_uRGBPrintingB);
                            }
                            Com::printFLN(PSTR("RGB_PRINTING_G = "), g_uRGBPrintingG);
                        } else {
                            Com::printFLN(PSTR("RGB_PRINTING_G out of range "));
                        }
                    }
                    break;
                }
                case 3: // blue
                {
                    if (pCommand->hasS()) {
                        if (pCommand->S >= 0 && pCommand->S <= 255) {
                            g_uRGBPrintingB = pCommand->S;

                            HAL::eprSetByte(EPR_RF_RGB_PRINTING_B, g_uRGBPrintingB);
                            EEPROM::updateChecksum();

                            if (Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_PRINTING) {
                                setRGBTargetColors(g_uRGBPrintingR, g_uRGBPrintingG, g_uRGBPrintingB);
                            }
                            Com::printFLN(PSTR("RGB_PRINTING_B = "), g_uRGBPrintingB);
                        } else {
                            Com::printFLN(PSTR("RGB_PRINTING_B out of range "));
                        }
                    }
                    break;
                }
                }
            }
            break;
        }

        case 3305: // M3305 [P] [S] - configure the RGB light effects for cooling
        {
            if (pCommand->hasP()) {
                switch (pCommand->P) {
                case 1: // red
                {
                    if (pCommand->hasS()) {
                        if (pCommand->S >= 0 && pCommand->S <= 255) {
                            g_uRGBCoolingR = pCommand->S;

                            HAL::eprSetByte(EPR_RF_RGB_COOLING_R, g_uRGBCoolingR);
                            EEPROM::updateChecksum();

                            if (Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_COOLING) {
                                setRGBTargetColors(g_uRGBCoolingR, g_uRGBCoolingG, g_uRGBCoolingB);
                            }
                            Com::printFLN(PSTR("RGB_COOLING_R = "), g_uRGBCoolingR);
                        } else {
                            Com::printFLN(PSTR("RGB_COOLING_R out of range "));
                        }
                    }
                    break;
                }
                case 2: // green
                {
                    if (pCommand->hasS()) {
                        if (pCommand->S >= 0 && pCommand->S <= 255) {
                            g_uRGBCoolingG = pCommand->S;

                            HAL::eprSetByte(EPR_RF_RGB_COOLING_G, g_uRGBCoolingG);
                            EEPROM::updateChecksum();

                            if (Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_COOLING) {
                                setRGBTargetColors(g_uRGBCoolingR, g_uRGBCoolingG, g_uRGBCoolingB);
                            }
                            Com::printFLN(PSTR("RGB_COOLING_G = "), g_uRGBCoolingG);
                        } else {
                            Com::printFLN(PSTR("RGB_COOLING_G out of range "));
                        }
                    }
                    break;
                }
                case 3: // blue
                {
                    if (pCommand->hasS()) {
                        if (pCommand->S >= 0 && pCommand->S <= 255) {
                            g_uRGBCoolingB = pCommand->S;

                            HAL::eprSetByte(EPR_RF_RGB_COOLING_B, g_uRGBCoolingB);
                            EEPROM::updateChecksum();

                            if (Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_COOLING) {
                                setRGBTargetColors(g_uRGBCoolingR, g_uRGBCoolingG, g_uRGBCoolingB);
                            }
                            Com::printFLN(PSTR("RGB_COOLING_B = "), g_uRGBCoolingB);
                        } else {
                            Com::printFLN(PSTR("RGB_COOLING_B out of range "));
                        }
                    }
                    break;
                }
                }
            }
            break;
        }

        case 3306: // M3306 [P] [S] - configure the RGB light effects for idle
        {
            if (pCommand->hasP()) {
                switch (pCommand->P) {
                case 1: // red
                {
                    if (pCommand->hasS()) {
                        if (pCommand->S >= 0 && pCommand->S <= 255) {
                            g_uRGBIdleR = pCommand->S;

                            HAL::eprSetByte(EPR_RF_RGB_IDLE_R, g_uRGBIdleR);
                            EEPROM::updateChecksum();

                            if (Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_IDLE) {
                                setRGBTargetColors(g_uRGBIdleR, g_uRGBIdleG, g_uRGBIdleB);
                            }
                            Com::printFLN(PSTR("RGB_IDLE_R = "), g_uRGBIdleR);
                        } else {
                            Com::printFLN(PSTR("RGB_IDLE_R out of range "));
                        }
                    }
                    break;
                }
                case 2: // green
                {
                    if (pCommand->hasS()) {
                        if (pCommand->S >= 0 && pCommand->S <= 255) {
                            g_uRGBIdleG = pCommand->S;

                            HAL::eprSetByte(EPR_RF_RGB_IDLE_G, g_uRGBIdleG);
                            EEPROM::updateChecksum();

                            if (Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_IDLE) {
                                setRGBTargetColors(g_uRGBIdleR, g_uRGBIdleG, g_uRGBIdleB);
                            }
                            Com::printFLN(PSTR("RGB_IDLE_G = "), g_uRGBIdleG);
                        } else {
                            Com::printFLN(PSTR("RGB_IDLE_G out of range "));
                        }
                    }
                    break;
                }
                case 3: // blue
                {
                    if (pCommand->hasS()) {
                        if (pCommand->S >= 0 && pCommand->S <= 255) {
                            g_uRGBIdleB = pCommand->S;

                            HAL::eprSetByte(EPR_RF_RGB_IDLE_B, g_uRGBIdleB);
                            EEPROM::updateChecksum();

                            if (Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_IDLE) {
                                setRGBTargetColors(g_uRGBIdleR, g_uRGBIdleG, g_uRGBIdleB);
                            }
                            Com::printFLN(PSTR("RGB_IDLE_B = "), g_uRGBIdleB);
                        } else {
                            Com::printFLN(PSTR("RGB_IDLE_B out of range "));
                        }
                    }
                    break;
                }
                }
            }

            break;
        }
        case 3307: // M3307 [P] [S] - configure the manual RGB light colors
        {
            if (pCommand->hasP()) {
                switch (pCommand->P) {
                case 1: // red
                {
                    if (pCommand->hasS()) {
                        if (pCommand->S >= 0 && pCommand->S <= 255) {
                            g_uRGBManualR = pCommand->S;

                            HAL::eprSetByte(EPR_RF_RGB_MANUAL_R, g_uRGBManualR);
                            EEPROM::updateChecksum();

                            if (Printer::RGBLightMode == RGB_MODE_MANUAL) {
                                setRGBTargetColors(g_uRGBManualR, g_uRGBManualG, g_uRGBManualB);
                            }
                            Com::printFLN(PSTR("RGB_MANUAL_R = "), g_uRGBManualR);
                        } else {
                            Com::printFLN(PSTR("RGB_MANUAL_R out of range "));
                        }
                    }
                    break;
                }
                case 2: // green
                {
                    if (pCommand->hasS()) {
                        if (pCommand->S >= 0 && pCommand->S <= 255) {
                            g_uRGBManualG = pCommand->S;

                            HAL::eprSetByte(EPR_RF_RGB_MANUAL_G, g_uRGBManualG);
                            EEPROM::updateChecksum();

                            if (Printer::RGBLightMode == RGB_MODE_MANUAL) {
                                setRGBTargetColors(g_uRGBManualR, g_uRGBManualG, g_uRGBManualB);
                            }
                            Com::printFLN(PSTR("RGB_MANUAL_G = "), g_uRGBManualG);
                        } else {
                            Com::printFLN(PSTR("RGB_MANUAL_G out of range "));
                        }
                    }
                    break;
                }
                case 3: // blue
                {
                    if (pCommand->hasS()) {
                        if (pCommand->S >= 0 && pCommand->S <= 255) {
                            g_uRGBManualB = pCommand->S;

                            HAL::eprSetByte(EPR_RF_RGB_MANUAL_B, g_uRGBManualB);
                            EEPROM::updateChecksum();

                            if (Printer::RGBLightMode == RGB_MODE_MANUAL) {
                                setRGBTargetColors(g_uRGBManualR, g_uRGBManualG, g_uRGBManualB);
                            }
                            Com::printFLN(PSTR("RGB_MANUAL_B = "), g_uRGBManualB);
                        } else {
                            Com::printFLN(PSTR("RGB_MANUAL_B out of range "));
                        }
                    }
                    break;
                }
                }
            }

            break;
        }
        case 3308: // M3308 [P] - configure the RGB light mode
        {
            if (pCommand->hasP()) {
                switch (pCommand->P) {
                case RGB_MODE_OFF: {
                    Printer::RGBLightStatus = RGB_STATUS_NOT_AUTOMATIC;
                    Printer::RGBLightMode = RGB_MODE_OFF;
                    Printer::RGBLightModeForceWhite = 0;

                    setRGBTargetColors(0, 0, 0);

                    HAL::eprSetByte(EPR_RF_RGB_LIGHT_MODE, Printer::RGBLightMode);
                    EEPROM::updateChecksum();
                    break;
                }
                case RGB_MODE_WHITE: {
                    Printer::RGBLightStatus = RGB_STATUS_NOT_AUTOMATIC;
                    Printer::RGBLightMode = RGB_MODE_WHITE;
                    Printer::RGBLightModeForceWhite = 0;

                    setRGBTargetColors(255, 255, 255);

                    HAL::eprSetByte(EPR_RF_RGB_LIGHT_MODE, Printer::RGBLightMode);
                    EEPROM::updateChecksum();
                    break;
                }
                case RGB_MODE_AUTOMATIC: {
                    Printer::RGBLightStatus = RGB_STATUS_AUTOMATIC;
                    Printer::RGBLightMode = RGB_MODE_AUTOMATIC;
                    Printer::RGBLightModeForceWhite = 0;

                    HAL::eprSetByte(EPR_RF_RGB_LIGHT_MODE, Printer::RGBLightMode);
                    EEPROM::updateChecksum();
                    break;
                }
                case RGB_MODE_MANUAL: {
                    Printer::RGBLightStatus = RGB_STATUS_NOT_AUTOMATIC;
                    Printer::RGBLightMode = RGB_MODE_MANUAL;
                    Printer::RGBLightModeForceWhite = 0;

                    setRGBTargetColors(g_uRGBManualR, g_uRGBManualG, g_uRGBManualB);

                    HAL::eprSetByte(EPR_RF_RGB_LIGHT_MODE, Printer::RGBLightMode);
                    EEPROM::updateChecksum();
                    break;
                }
                }
            }

            break;
        }
#endif // RGB_LIGHT_R_PIN

#if FEATURE_ALIGN_EXTRUDERS
        case 3309: // start/abort to align the two extruders
        {
            startAlignExtruders();
            break;
        }
#endif // FEATURE_ALIGN_EXTRUDERS

        case 3901: // 3901 [X] [Y] - configure the Matrix-Position to Scan, [S] confugure learningrate, [P] configure dist weight || by Nibbels
        case 3900: // 3900 direct preconfig, no break;->next is M3900.
        {
            bool err3900r = false;
            if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
                // we load the z compensation matrix before its first usage because this can take some time
                Com::printFLN(PSTR("M3900/M3901: INFO Die Z-Matrix wurde aus dem EEPROM gelesen."));
                prepareZCompensation();
            }

            if (g_ZCompensationMatrix[0][0] == EEPROM_FORMAT) {
                if (!pCommand->hasR()) { //Normaler ZOS, kein Auto-Matrix-Leveling.
                    if (pCommand->hasX()) {
                        // test and take over the specified value
                        unsigned char nTempUC = (unsigned char)pCommand->X;

                        //Wessix idee: zufalls-Scanpunkt wegen DDP-Platten-Schonung.
                        if (nTempUC == 0)
                            nTempUC = (unsigned char)random((HEAT_BED_SCAN_X_START_MM == 0) ? 1 : 2, g_uZMatrixMax[X_AXIS]); // ergibt min bis max-1

                        //wenn die Startposition nicht 0 ist, wird eine Dummy-Matrix-Linie ergänzt. Mit der sollten wir nicht arbeiten.
                        if (nTempUC < 1 + ((HEAT_BED_SCAN_X_START_MM == 0) ? 0 : 1))
                            nTempUC = 1 + ((HEAT_BED_SCAN_X_START_MM == 0) ? 0 : 1);
                        if (nTempUC > g_uZMatrixMax[X_AXIS] - 1)
                            nTempUC = g_uZMatrixMax[X_AXIS] - 1; //2..n-1

                        g_ZOSTestPoint[X_AXIS] = nTempUC;

                        unsigned char oldval = HAL::eprGetByte(EPR_RF_MOD_ZOS_SCAN_POINT_X);
                        if (oldval != g_ZOSTestPoint[X_AXIS]) {
                            HAL::eprSetByte(EPR_RF_MOD_ZOS_SCAN_POINT_X, g_ZOSTestPoint[X_AXIS]);
                            EEPROM::updateChecksum(); //deshalb die prüfung
                        }

                        if (Printer::debugInfo()) {
                            Com::printF(PSTR("M3900/M3901: CHANGED X ZOS Testposition: "), nTempUC);
                            if (HEAT_BED_SCAN_X_START_MM == 0) {
                                Com::printF(PSTR(" [Z-Matrix index] X={1.."), g_uZMatrixMax[X_AXIS] - 1);
                            } else {
                                Com::printF(PSTR(" [Z-Matrix index] X={2.."), g_uZMatrixMax[X_AXIS] - 1);
                            }
                            Com::printFLN(PSTR("}"));
                        }
                    }

                    if (pCommand->hasY()) {
                        // test and take over the specified value
                        unsigned char nTempUC = (unsigned char)pCommand->Y;

                        //Wessix idee: zufalls-Scanpunkt wegen DDP-Platten-Schonung.
                        if (nTempUC == 0)
                            nTempUC = (unsigned char)random((HEAT_BED_SCAN_Y_START_MM == 0) ? 1 : 2, g_uZMatrixMax[Y_AXIS]); // ergibt min bis max-1

                        if (nTempUC < 1 + ((HEAT_BED_SCAN_Y_START_MM == 0) ? 0 : 1))
                            nTempUC = 1 + ((HEAT_BED_SCAN_Y_START_MM == 0) ? 0 : 1);
                        if (nTempUC > g_uZMatrixMax[Y_AXIS] - 1)
                            nTempUC = g_uZMatrixMax[Y_AXIS] - 1; //2..n-1

                        g_ZOSTestPoint[Y_AXIS] = nTempUC;

                        unsigned char oldval = HAL::eprGetByte(EPR_RF_MOD_ZOS_SCAN_POINT_Y);
                        if (oldval != g_ZOSTestPoint[Y_AXIS]) {
                            HAL::eprSetByte(EPR_RF_MOD_ZOS_SCAN_POINT_Y, g_ZOSTestPoint[Y_AXIS]);
                            EEPROM::updateChecksum(); //deshalb die prüfung
                        }

                        if (Printer::debugInfo()) {
                            Com::printF(PSTR("M3900/M3901: CHANGED Y ZOS Testposition: "), nTempUC);
                            if (HEAT_BED_SCAN_Y_START_MM == 0) {
                                Com::printF(PSTR(" [Z-Matrix index] Y={1.."), g_uZMatrixMax[Y_AXIS] - 1);
                            } else {
                                Com::printF(PSTR(" [Z-Matrix index] Y={2.."), g_uZMatrixMax[Y_AXIS] - 1);
                            }
                            Com::printFLN(PSTR("}"));
                        }
                    }

                    if (pCommand->hasS()) {
                        // M3900 S set learning rate to limit changes caused of z-Offset Scan. This might proof handy for multiple positions scans.
                        if (pCommand->S >= 0 && pCommand->S <= 100) {
                            g_ZOSlearningRate = (float)pCommand->S * 0.01f;
                            Com::printFLN(PSTR("M3900/M3901: CHANGED [S] ZOS learning rate : "), g_ZOSlearningRate);
                        } else {
                            Com::printFLN(PSTR("M3900/M3901: ERROR [S] ZOS learning rate ignored, out of range {0...100}"));
                            err3900r = true;
                        }
                    }

                    if (pCommand->hasP()) {
                        // M3900 P set distance weight: This can be used as something like the auto-bed-leveling (if used in all corners) but technically affects the Z_Matrix
                        if (pCommand->P >= 0 && pCommand->P <= 100) {
                            g_ZOSlearningGradient = (float)pCommand->P * 0.01f;
                            Com::printFLN(PSTR("M3900/M3901: CHANGED [P] ZOS learning linear distance weight : "), g_ZOSlearningGradient);
                        } else {
                            Com::printFLN(PSTR("M3900/M3901: ERROR [P] ZOS learning DistanceWeight, out of range {0...100}"));
                            err3900r = true;
                        }
                    }

                    // Anzeige der aktuellen Settings
                    Com::printFLN(PSTR("M3900/M3901: ### AKTIVE ZOS SETTINGS ###"));
                    Com::printF(PSTR("M3900/M3901: Testposition [X]: "), g_ZOSTestPoint[X_AXIS]);
                    Com::printF(PSTR(" Testposition [Y]: "), g_ZOSTestPoint[Y_AXIS]);
                    Com::printFLN(PSTR(" [Z-Matrix index]"));
                    Com::printFLN(PSTR("M3900/M3901: [S] ZOS learning rate is : "), g_ZOSlearningRate);
                    if (g_ZOSlearningRate == 1.0f) {
                        Com::printFLN(PSTR("M3900/M3901: INFO [S] ZOS::overwrite mode (1.00)"));
                    } else {
                        Com::printFLN(PSTR("M3900/M3901: INFO [S] ZOS::additiv / learning mode (0.00 - 0.99)"));
                    }
                    Com::printFLN(PSTR("M3900/M3901: [P] ZOS learning linear distance weight is : "), g_ZOSlearningGradient);
                    if (g_ZOSlearningGradient == 0.0f) {
                        Com::printFLN(PSTR("M3900/M3901: INFO [P] ZOS::reiner Offset-Scan"));
                    } else if (g_ZOSlearningGradient == 1.0f) {
                        Com::printFLN(PSTR("M3900/M3901: INFO [P] ZOS::rein linear abstandsgewichteter Scan"));
                    } else {
                        Com::printFLN(PSTR("M3900/M3901: INFO [P] ZOS::Das Ergebnis-Offset setzt sich zusammen aus Abstandsgewichtung und Offset"));
                    }

                    if (g_ZOSlearningGradient > 0.0f) {
                        Com::printFLN(PSTR("M3900/M3901: INFO [P] Set 0 => 0.0 for Offset only, set 100 => 1.0 for distance weight only"));
                        Com::printFLN(PSTR("M3900/M3901: INFO [P] Combine linear distance weight with low learning rate and multiple checks at corners (for example) against bed warping!"));
                    }
                    if (g_ZOSlearningRate == 1.0f) {
                        Com::printFLN(PSTR("M3900/M3901: FORMEL Z-Matrix = EEPROM-Matrix + g_ZOSlearningRate*(g_ZOSlearningGradient*weight(x,y)*Offset + (1.0-g_ZOSlearningGradient)*Offset)"));
                    } else {
                        Com::printFLN(PSTR("M3900/M3901: FORMEL Z-Matrix = Z-Matrix + g_ZOSlearningRate*(g_ZOSlearningGradient*weight(x,y)*Offset + (1.0-g_ZOSlearningGradient)*Offset)"));
                    }
                } else { // pCommand->hasR() //Auto-Matrix-Leveling.
                         //Auto-Matrix-Leveling aktiv.
                }
                if (pCommand->M == 3901) {
                    //M3901
                    //M3900 wird nicht gestartet - nur preconfig, kein mhier-Scan!
                    if (pCommand->hasR())
                        Com::printFLN(PSTR("M3901: Use M3900 for Auto-Matrix-Leveling!"));
                } else {
                    if (err3900r) {
                        Com::printFLN(PSTR("M3900/M3901: ERROR in Config ### ZOS SKIPPED ###"));
                    } else {
                        // M3900:
                        // mit R im GCode macht der Scan ein Auto-Matrix-Leveling, anstatt die anderen Schalter zu bedienen.
                        startZOScan(pCommand->hasR());
                        // Warte bis Z-Offset-Scan beendet wurde.
                        while (g_nZOSScanStatus) {
                            Commands::checkForPeriodicalActions(Calibrating);
                        }
                    }
                }
            } else {
                Com::printFLN(PSTR("M3900/M3901: ERROR Matrix Initialisation Error!"));
                Com::printFLN(PSTR("M3900/M3901: INFO Die Z-Matrix konnte nicht aus dem EEPROM gelesen werden."));
                Com::printFLN(PSTR("M3900/M3901: INFO Vermutlich noch nie einen Heat-Bed-Scan gemacht."));
            }
            break;
        }

        case 3902: // M3902 Nibbels Matrix Manipulations "NMM"
        {
            //NMM Funktion 1 - R=Repair
            if (pCommand->hasR()) {
                //search for a hole within the heat beds z-Matrix
                fixKeramikLochInMatrix();
            }

            if (pCommand->hasE()) {
                //completly wipe the matrix-data to zero->flatten the matrix to nothing.
                setMatrixNull();
            }
            //NMM Funktion 2 - Z=Offset manuell nachstellen
            if (pCommand->hasZ()) {
                if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
                    // we load the z compensation matrix before its first usage because this can take some time
                    Com::printFLN(PSTR("M3901: INFO Die Z-Matrix wurde aus dem EEPROM gelesen."));
                    prepareZCompensation();
                }
                if (g_ZCompensationMatrix[0][0] == EEPROM_FORMAT) {
                    //search for a hole within the heat beds z-Matrix
                    float hochrunter = (float)pCommand->Z;
                    if (hochrunter > 0.2f)
                        hochrunter = 0.2f;
                    if (hochrunter < -0.2f)
                        hochrunter = -0.2f;

                    Com::printSharpLine();

                    if (hochrunter < 0.0f) {
                        Com::printFLN(PSTR("M3902: Duese-Bett-Abstand wird kleiner gemacht. "), hochrunter);
                    } else if (hochrunter > 0.0f) {
                        Com::printFLN(PSTR("M3902: Duese-Bett-Abstand wird groesser gemacht. "), hochrunter);
                    } else {
                        Com::printFLN(PSTR("M3902: Duese-Bett-Abstand bleibt gleich. Das Offset wird in die Matrix verrechnet und genullt."), hochrunter);
                    }
                    Com::printFLN(PSTR("M3902: -Z heisst Bett hoch/weniger Abstand, +Z heisst Bett runter/mehr Abstand. "));
                    Com::printFLN(PSTR("M3902: Bei Z=0 ändert sich nichts, doch es wird das aktuelle Offset in die Matrix verrechnet. "));

                    Com::printSharpLine();

                    // determine the minimal distance between extruder and heat bed
                    determineCompensationOffsetZ(); //-> schreibt kleinsten abstand in g_offsetZCompensationSteps, sollte schon drin sein, aber man weiß nie.
                    Com::printFLN(PSTR("M3902: Alt::Min. Bett-Hotend Abstand [Steps] = "), -1 * (int)g_offsetZCompensationSteps);

                    long hochrunterSteps = long(hochrunter * Printer::axisStepsPerMM[Z_AXIS]); //axissteps ist auch float

                    Com::printFLN(PSTR("M3902: Veraenderung Z [mm] = "), hochrunter);
                    Com::printFLN(PSTR("M3902: Veraenderung Z [Steps] = "), hochrunterSteps);

                    if (hochrunter == 0.00f) {
                        //wenn Z=0.0 soll alles so bleibenk aber das Offset ins Matrix-Offset reingerechnet und genullt werden.
                        //das Offset muss negativ eingehen, denn wenn die Matrix "weniger tief" ist, bleibt die Düse weiter weg vom Bett.
                        hochrunterSteps = long((Printer::ZOffset * Printer::axisStepsPerMM[Z_AXIS]) / 1000); //offset-stepps neu berechnen!
                        Com::printFLN(PSTR("M3902: Veraenderung = zOffset --> zMatrix; Offset = 0;"));
                    }

                    //das ist ein negativer wert! Je mehr Abstand, desto negativer. Positiv verboten.
                    if (g_offsetZCompensationSteps + hochrunterSteps > (long(Printer::axisStepsPerMM[Z_AXIS] * g_scanStartZLiftMM) - -1 * g_nScanHeatBedUpFastSteps)) {
                        Com::printF(PSTR("M3902: Fehler::Z-Matrix wuerde positiv werden. Das waere eine Kollision um "), (int)(g_offsetZCompensationSteps + hochrunterSteps));
                        Com::printFLN(PSTR(" [Steps] bzw. "), Printer::axisMMPerSteps[Z_AXIS] * (float)(g_offsetZCompensationSteps + hochrunterSteps), 2);
                        Com::printFLN(PSTR(" [mm]"));
                    } else {
                        Com::printFLN(PSTR("M3902: Neu::Min. Bett-Extruder Restabstand = "), -1 * (int)(g_offsetZCompensationSteps));

                        short x;
                        short y;
                        short deltaZ = (short)hochrunterSteps;
                        bool overflow = false;
                        bool overnull = false;
                        bool overH = false;

                        for (x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) {
                            for (y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
                                if ((long)g_ZCompensationMatrix[x][y] + (long)deltaZ >= 32767) {
                                    overflow = true;
                                } else {
                                    g_ZCompensationMatrix[x][y] += deltaZ;
                                    if (g_ZCompensationMatrix[x][y] > 0)
                                        overnull = true; //overflow oder kollision, kann einfach nicht sein und abfrage ist kurz.
                                    if (g_ZCompensationMatrix[x][y] > (long(Printer::axisStepsPerMM[Z_AXIS] * g_scanStartZLiftMM) - -1 * g_nScanHeatBedUpFastSteps))
                                        overH = true;
                                }
                            }
                        }
                        if (overnull) {
                            Com::printFLN(PSTR("M3901: WARNING::positive Matrix::Please fix Z-Screw"));
                        }
                        if (overH) {
                            Com::printFLN(PSTR("M3901: ERROR::sehr positive Matrix::ReLoading zMatrix from EEPROM to RAM"));
                            if (loadCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed))) {
                                // Error: there is no valid compensation matrix available
                                initCompensationMatrix();
                            }
                        } else if (overflow) {
                            Com::printFLN(PSTR("M3901: ERROR::Overflow in Matrix::ReLoading zMatrix from EEPROM to RAM"));
                            if (loadCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed))) {
                                // Error: there is no valid compensation matrix available
                                initCompensationMatrix();
                            }
                        } else {
                            if (hochrunter == 0.00f) {
                                Printer::ZOffset = 0; //offset um nullen
                                if (HAL::eprGetInt32(EPR_RF_Z_OFFSET) != Printer::ZOffset) {
                                    HAL::eprSetInt32(EPR_RF_Z_OFFSET, Printer::ZOffset);
                                    EEPROM::updateChecksum();
                                }
                                g_staticZSteps = ((Printer::ZOffset + g_nSensiblePressureOffset) * Printer::axisStepsPerMM[Z_AXIS]) / 1000; //offset-stepps neu berechnen
                            }
                            g_ZMatrixChangedInRam = 1;
                        }
                    }

                    // determine the minimal distance between extruder and heat bed
                    determineCompensationOffsetZ();
                    Com::printFLN(PSTR("M3902: Neu::Min. Bett-Extruder Restabstand [Steps] = "), -1 * (int)g_offsetZCompensationSteps);
                    Com::printFLN(PSTR("M3902: Neu::Min. Bett-Extruder Restabstand [mm] = "), -1 * Printer::axisMMPerSteps[Z_AXIS] * (float)g_offsetZCompensationSteps, 2);
                } else {
                    Com::printFLN(PSTR("M3902: ERROR::Matrix Initialisation Error!"));
                    Com::printFLN(PSTR("M3902: INFO Die Z-Matrix konnte nicht aus dem EEPROM gelesen werden."));
                    Com::printFLN(PSTR("M3902: INFO Vermutlich nie einen Heat-Bed-Scan gemacht."));
                }
            }

            //NMM Funktion 3 - S=Save,Sichern der Matrix an spezielle EEPROM-Position
            if (pCommand->hasS()) {
                if (pCommand->S >= 1 && pCommand->S <= EEPROM_MAX_HEAT_BED_SECTORS) {
                    // save the determined values to the EEPROM @ savepoint "pCommand->S" Standard: 1..9
                    unsigned int savepoint = (unsigned int)pCommand->S;
                    saveCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * savepoint)); //g_nActiveHeatBed --> pCommand->S
                    Com::printFLN(PSTR("M3902: Save the Matrix::OK"));
                } else {
                    Com::printFLN(PSTR("M3902: Save the Matrix::ERROR::invalid savepoint, invalid active heatbedmatrix"));
                }
            }
            break;
        }

        case 3909: // M3909 [P]PressureDigits - configure the sensible pressure value threshold || by Wessix and Nibbels
        {
            bool error = false;
            //Statusänderung P10000 ([digits])
            if (pCommand->hasP()) {
                if (pCommand->P > 32767)
                    pCommand->P = 32767;
                if (pCommand->P >= 0 && pCommand->P < EMERGENCY_PAUSE_DIGITS_MAX) {
                    if (pCommand->P) {
                        Com::printFLN(PSTR("M3909: SensiblePressure Pmax="), pCommand->P);
                        short oldval = HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS);
                        if (oldval != pCommand->P) {
                            Com::printFLN(PSTR("M3909: P change"));
                            HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS, pCommand->P);
                            EEPROM::updateChecksum(); //deshalb die prüfung
                        }
                    } else {
                        g_nSensiblePressureDigits = 0;
                        error = true;
                        Com::printFLN(PSTR("M3909: SensiblePressure disabled"));
                    }
                } else {
                    Com::printFLN(PSTR("M3909: wrong [P]"));
                    error = true;
                }
            }

            if (pCommand->hasS()) {
                if (pCommand->S > 0 && pCommand->S <= 300) {
                    //max darf nie 0 werden!! div/0 bei zeile ~5600
                    g_nSensiblePressureOffsetMax = (short)pCommand->S;
                    Com::printFLN(PSTR("M3909: SensiblePressure max. [S]Offset changed to "), g_nSensiblePressureOffsetMax);
                    short oldval = HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX);
                    if (oldval != g_nSensiblePressureOffsetMax) {
                        Com::printFLN(PSTR("M3909: S change"));
                        HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX, g_nSensiblePressureOffsetMax);
                        EEPROM::updateChecksum(); //deshalb die prüfung
                    }
                } else {
                    Com::printFLN(PSTR("M3909: ERROR::>0.3mm=S300 - This function is ment to compensate minimal amounts of too close distance."));
                    Com::printFLN(PSTR("M3909: INFO::If you have to auto-compensate your offset to high, clean your nozzle, rise your temp, lower your speed."));
                    Com::printFLN(PSTR("M3909: INFO::This function should lower the chance of an accidential emergency block on the first layer. It cannot help you to avoid calibration!"));
                    error = true;
                }
            }

            //Statusausgabe per M3909
            if (g_nSensiblePressureDigits) {
                Com::printF(PSTR("M3909: INFO SensiblePressure active. [P]PressureDigit = +-"), g_nSensiblePressureDigits);
                Com::printFLN(PSTR(" [digits] (Standard: use `standard print digits` +20%)"));
                Com::printF(PSTR("M3909: INFO SensiblePressures [S]max. offset is "), g_nSensiblePressureOffsetMax);
                Com::printFLN(PSTR(" [um] (Standard: 180um, Max: 300um)"));
            } else {
                Com::printFLN(PSTR("M3909: INFO SensiblePressure is currently disabled."));
            }

            if (!error)
                queueTask(TASK_ENABLE_SENSE_OFFSET);
            break;
        }

        case 3911: // M3911 [S]Inc/Dec - Testfunction for AtlonXP's DigitFlowCompensation
        {
            if (pCommand->hasS() && pCommand->hasP()) {
                short min = abs(static_cast<short>(pCommand->S));
                short max = abs(static_cast<short>(pCommand->P));
                if (min > max) {
                    min = abs(static_cast<short>(pCommand->P));
                    max = abs(static_cast<short>(pCommand->S));
                }
                if (min < 500)
                    min = 500; //weniger ist in jedem fall sinnfrei, selbst mit ninjaflex und digit homing
                if (max < min)
                    max = min; //geht: flowsprung.
                g_nDigitFlowCompensation_Fmin = min;
                g_nDigitFlowCompensation_Fmax = max;
            }

            if (pCommand->hasE()) {
                int8_t e = static_cast<int8_t>(pCommand->E);
                if (e > 99)
                    e = 99;
                if (e < -99)
                    e = -99;
                g_nDigitFlowCompensation_intense = e;
            }

            if (pCommand->hasF()) {
                int8_t f = static_cast<int8_t>(pCommand->F);
                if (f > 99)
                    f = 99;
                if (f < -90)
                    f = -90;
                g_nDigitFlowCompensation_speed_intense = f;
            }

            Com::printFLN(PSTR("[S|P] Flow CMP min: "), g_nDigitFlowCompensation_Fmin);
            Com::printFLN(PSTR("[S|P] Flow CMP max: "), g_nDigitFlowCompensation_Fmax);
            Com::printFLN(PSTR("[E]   Flow CMP %: "), g_nDigitFlowCompensation_intense);
            Com::printFLN(PSTR("[F]   Feed CMP %: "), g_nDigitFlowCompensation_speed_intense);
            break;
        }

        case 3912: //M3912 lay down automatic startmade which you can abort (when the hotend is moving) by pressing play
        {
            Commands::waitUntilEndOfAllMoves(); //feature startline
            if (Printer::areAxisHomed() && Printer::doHeatBedZCompensation && !g_uBlockCommands) {

                Com::printFLN(PSTR("Auto-Startmade"));
                short min = 1500;
                short max = 4500;

                //dirty hack
                short save_g_nDigitFlowCompensation_Fmin = g_nDigitFlowCompensation_Fmin;
                short save_g_nDigitFlowCompensation_Fmax = g_nDigitFlowCompensation_Fmax;
                short save_g_nDigitFlowCompensation_speed_intense = g_nDigitFlowCompensation_speed_intense;
                short save_g_nDigitFlowCompensation_intense = g_nDigitFlowCompensation_intense;
                bool save_relativeExtruderCoordinateMode = Printer::relativeExtruderCoordinateMode;
                float save_logicalE;
                float save_logicalERemainder;
                float save_extrudeMultiplyErrorSteps;
                float save_compensatedPositionCollectTinyE;
                {
                    InterruptProtectedBlock noInts;
                    save_logicalE = Printer::destinationMM[E_AXIS];
                    save_logicalERemainder = Printer::destinationRemainderMM[E_AXIS];
                    save_extrudeMultiplyErrorSteps = Printer::extrudeMultiplyErrorSteps;
                    save_compensatedPositionCollectTinyE = Printer::compensatedPositionCollectTinyE;
                    Printer::queuePositionZLayerGuessNew = 0;
                    Printer::queuePositionZLayerCurrent = 0;
                    Printer::queuePositionZLayerLast = 0;
                    g_startLineActive = 1;
                }

                if (pCommand->hasS() && pCommand->hasP()) {
                    min = abs(static_cast<short>(pCommand->S));
                    max = abs(static_cast<short>(pCommand->P));
                    if (min > max) {
                        min = abs(static_cast<short>(pCommand->P));
                        max = abs(static_cast<short>(pCommand->S));
                    }
                    if (min < 500)
                        min = 500; //weniger ist in jedem fall sinnfrei, selbst mit ninjaflex und digit homing
                    if (max < min)
                        max = min; //geht: flowsprung.
                }

                uint8_t Extrusion = 20;
                uint8_t Lines = 2;

                if (pCommand->hasE()) {
                    Extrusion = (uint8_t)pCommand->E;
                    if (Extrusion < 10)
                        Extrusion = 10; //sinnvoll war bei mir ca. 10..30 pro linie.
                    if (Extrusion > 30)
                        Extrusion = 30;
                }

                uint8_t lineFeedrate = 15;
                if (pCommand->hasF()) {
                    lineFeedrate = (uint8_t)pCommand->F;
                    if (lineFeedrate < 5)
                        lineFeedrate = 5; //sinnvoll war bei mir ca. 10..30 pro linie.
                    if (lineFeedrate > 50)
                        lineFeedrate = 50;
                }

                if (pCommand->hasI()) {
                    Lines = (uint8_t)pCommand->I;
                    if (Lines < 1)
                        Lines = 1;
                    if (Lines > 5)
                        Lines = 5;
                }

                float y = 23.0f;
                if (pCommand->hasY()) {
                    y = (float)pCommand->Y;
                    if (y < 0.5f)
                        y = 0.5f;
                    // Länge y Achse - halbe Bahnbreite - 1.5 pro Versatz - Schrägversatz nur bei Dual. Übertrieben aber das sollte genau sein.
                    float maxY = Printer::maxSoftEndstopSteps[Y_AXIS] * Printer::axisMMPerSteps[Y_AXIS] - 0.5f - Lines * 1.5f - (NUM_EXTRUDER > 1 ? 5.0f : 0.0f);
                    if (y > maxY)
                        y = maxY;
                }

                //bezogen auf : M3411 S P F-90 -> Flow CMP Speed einstellungen werden kurz substituiert und dann resubstituiert
                g_nDigitFlowCompensation_Fmin = min;
                g_nDigitFlowCompensation_Fmax = max;
                g_nDigitFlowCompensation_speed_intense = -90;
                g_nDigitFlowCompensation_intense = -75;
                //M82:
                Printer::relativeExtruderCoordinateMode = false;

                Printer::setEPositionMM(0); // G92 E0

                //Abstand links und rechts.
                const float spacerX = 15.0f;
                //spacerXd for my personal safety when I flash dual and forget the axis length.
                const float spacerXd = (NUM_EXTRUDER > 1 ? extruder[1].offsetMM[X_AXIS] : 0);

                float x = spacerX;
#if NUM_EXTRUDER > 0
                if (Extruder::current->id != 0) {
                    //if you use T1 then dont make the start line ontop of the startline of T0
                    //shift the y-position according to the extruders number.
                    y += (float)Extruder::current->id; /* times 1mm */
                    ;
                }
#endif                                  //NUM_EXTRUDER > 0
                if (pCommand->hasY()) { //override y with user value
                    y = (float)pCommand->Y;
                    float max = Printer::maxSoftEndstopSteps[Y_AXIS] * Printer::axisMMPerSteps[Y_AXIS] * 0.5;
                    if (y > max)
                        y = max;
                    if (y < 0.0f)
                        y = 0.0f;
                }
                float e = 0.0f;

                bool skip_by_keys = false;

                const float startLineZ = 0.35f;
                if (!g_uBlockCommands)
                    Printer::queueFloatCoordinates(x, y, startLineZ, IGNORE_COORDINATE, RMath::min(Printer::homingFeedrate[X_AXIS], Printer::homingFeedrate[Y_AXIS]));

                for (uint8_t i = 1; i <= Lines && !g_uBlockCommands; i++) {
                    float x_0 = x;
                    float y_0 = y;
                    float e_0 = e;
                    if (i % 2 != 0) { //ungerade zahl.
                        x += (Printer::axisLengthMM[X_AXIS]) - 2 * spacerX - spacerXd;
                        y += (NUM_EXTRUDER > 1 ? 5.0f : 0.0f); //fahre nur bei Dual-Setting schräg.
                        e += (float)Extrusion * ((Printer::axisLengthMM[X_AXIS]) - 2 * spacerX - spacerXd) / 200;
                    } else {
                        x -= (Printer::axisLengthMM[X_AXIS]) - 2 * spacerX - spacerXd;
                        y -= (NUM_EXTRUDER > 1 ? 5.0f : 0.0f); //fahre nur bei Dual-Setting schräg.
                        e += (float)Extrusion * ((Printer::axisLengthMM[X_AXIS]) - 2 * spacerX - spacerXd) / 200;
                    }
                    for (float i = 0.0025f; i <= 1.0f; i += 0.0025f) {
                        Commands::checkForPeriodicalActions(Processing);
                        if (g_uBlockCommands)
                            break; // No further moves after cancellation.
                        if (Printer::checkPlayKey()) {
                            skip_by_keys = true;
                            break;
                        }
                        // Short segments let the DMS brake respond to nozzle load.
                        Printer::queueFloatCoordinates(x_0 + (x - x_0) * i,
                                                       y_0 + (y - y_0) * i,
                                                       IGNORE_COORDINATE,
                                                       e_0 + (e - e_0) * i, lineFeedrate);
                    }
                    if (g_uBlockCommands || skip_by_keys)
                        break; // No extra Y travel after skip/abort.
                    y += 1.5f;
#if NUM_EXTRUDER > 0
                    //if you use T1 then dont make the start line ontop of the startline of T0
                    y += float(NUM_EXTRUDER - 1); /* +1mm spacing for each extruder involved */
                    ;
#endif //NUM_EXTRUDER > 0
                    Printer::queueFloatCoordinates(IGNORE_COORDINATE, y, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Y_AXIS]);
                    if (skip_by_keys)
                        break; //mache die Startmade überspringbar -> weiter zum Druck.
                }
                Commands::waitUntilEndOfAllMoves(); // Prime queue fully drained.
                // Reset only DMS-specific throttling, not M220 or M221.
                Printer::dynamicExtrusionFactor = 1.0f;
                Printer::dynamicFeedrateFactor = 1.0f;
                g_nDigitFlowCompensation_Fmin = save_g_nDigitFlowCompensation_Fmin;
                g_nDigitFlowCompensation_Fmax = save_g_nDigitFlowCompensation_Fmax;
                g_nDigitFlowCompensation_speed_intense = save_g_nDigitFlowCompensation_speed_intense;
                g_nDigitFlowCompensation_intense = save_g_nDigitFlowCompensation_intense;

                {
                    InterruptProtectedBlock noInts;
                    // On AVR, a uint16_t timer modifier must be updated atomically.
                    Printer::interval_mod = 0;
                    if (!g_uBlockCommands) {
                        // Physical filament stays extruded; restore only the
                        // slicer's logical E timeline, including its substeps.
                        Printer::setEPositionMM(save_logicalE, save_logicalERemainder);
                        Printer::extrudeMultiplyErrorSteps = save_extrudeMultiplyErrorSteps;
                        Printer::compensatedPositionCollectTinyE = save_compensatedPositionCollectTinyE;
                    }
                    Printer::queuePositionZLayerGuessNew = 0;
                    Printer::queuePositionZLayerCurrent = 0;
                    Printer::queuePositionZLayerLast = 0;
                    g_startLineActive = 0;
                }
                Printer::relativeExtruderCoordinateMode = save_relativeExtruderCoordinateMode;
            } else {
                Com::printFLN(PSTR("M3912 error missing homing or zCMP"));
            }
            break;
        }

        case 3913: //M3913 input filament until DMS Sensor tells us to stop or Extrusion amount is reached
        {
            Commands::waitUntilEndOfAllMoves(); //loadfilament
            Com::printFLN(PSTR("Load Filament"));
            g_uStartOfIdle = 0; //M3913

                //normalize dms sensor values to 0 if allowed in options and not already done
            if (!Printer::g_pressure_offset)
                Printer::homeDigits();

            uint8_t Extrusion = 65; //max, wenn kein wiederstand. (ca. hotendlänge)
            uint8_t eFeedrate = 1;
            if (pCommand->hasF())
                eFeedrate = constrain(abs(static_cast<uint8_t>(pCommand->F)), 1, 50);
            short maxP = 2000; //to overwrite .. safety.
            if (pCommand->hasP())
                maxP = constrain(abs(static_cast<short>(pCommand->P)), 0, 20000);
            bool save_relativeExtruderCoordinateMode = Printer::relativeExtruderCoordinateMode;
            Printer::relativeExtruderCoordinateMode = false; //M82:
            uint8_t saveunitIsInches = Printer::unitIsInches;
            Printer::unitIsInches = 0; //to save possible G21 .. weiß net ob das so wichtig wäre ^^.

            Printer::setEPositionMM(0); // G92 E0

            float e = 0.0f;
            while (e < float(Extrusion)) {
                Commands::checkForPeriodicalActions(Processing);
                if (g_uBlockCommands || abs(readStrainGauge(ACTIVE_STRAIN_GAUGE)) > maxP /* digits sind soweit gestiegen, dass abbruch.*/) {
                    UI_STATUS_UPD(UI_TEXT_OK);
                    break;
                }
                if (Printer::checkAbortKeys()) {
                    UI_STATUS_UPD(UI_TEXT_ABORT_KEYPRESSED);
                    break;
                }
                e += 0.02; //kleine schritte
                Printer::queueFloatCoordinates(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, e, eFeedrate);
            }

            Printer::setEPositionMM(0); // G92 E0

            Printer::relativeExtruderCoordinateMode = save_relativeExtruderCoordinateMode;
            Printer::unitIsInches = saveunitIsInches;
            g_uStartOfIdle = HAL::timeInMilliseconds() + 5000; // M3913 load filament ends
            break;
        }
        case 3914: //M3914 coldpull filament very gently
        {
            Commands::waitUntilEndOfAllMoves(); //loadfilament
            Com::printFLN(PSTR("Unload Filament"));
            g_uStartOfIdle = 0; //M3914

            //init
            uint8_t outputLength = 100; //9cm max Output, wenn kein übertriebener wiederstand. Unser Hotend ist nicht ganz so lang.
            float eFeedrate = 0.3f;     /*mm/s*/
            short maxForce = 4000;      //3500+ might work well! You should stay underneath the force causing step loss.
            if (pCommand->hasP())
                maxForce = constrain(abs(static_cast<short>(pCommand->P)), 0, EMERGENCY_PAUSE_DIGITS_MAX);
            float t = float(UI_SET_EXTRUDER_MIN_TEMP_UNMOUNT);

            //preheat / precool
            Extruder::setTemperatureForExtruder(t, Extruder::current->id);
            Extruder::current->tempControl.waitForTargetTemperature(10);

            //set stuff
            bool save_relativeExtruderCoordinateMode = Printer::relativeExtruderCoordinateMode;
            Printer::relativeExtruderCoordinateMode = false; //M82:
            uint8_t saveunitIsInches = Printer::unitIsInches;
            Printer::unitIsInches = 0; //to save possible G21 .. weiß net ob das so wichtig wäre ^^.
            Printer::setEPositionMM(0); // G92 E0

                //normalize dms sensor values to 0 if allowed in options and not already done
            if (!Printer::g_pressure_offset)
                Printer::homeDigits();

            //work
            float e = 0.0f;
            float de = 0.005f;
            uint8_t retry = 5;
            while (t < float(UI_SET_EXTRUDER_MAX_TEMP_UNMOUNT) && fabs(e) < float(outputLength)) {
                millis_t time = HAL::timeInMilliseconds() + 2000;
                while (HAL::timeInMilliseconds() <= time) {
                    Commands::checkForPeriodicalActions(WaitHeater);
                    UI_STATUS_UPD(UI_TEXT_UNMOUNT_FILAMENT);
                    if (g_uBlockCommands)
                        break;
                }
                if (g_uBlockCommands)
                    break;
                while (fabs(e) < float(outputLength)) {
                    Commands::checkForPeriodicalActions(Processing);
                    if (g_uBlockCommands)
                        break;
                    if (abs(readStrainGauge(ACTIVE_STRAIN_GAUGE)) > maxForce /* digits sind soweit INS MINUS gestiegen, dass abbruch.*/) {
                        t += 3.33; // +1°K
                        break;
                    }
                    if (Printer::checkAbortKeys()) {
                        e = float(outputLength); //for sure exits because of no retrys
                        retry = 0;               //exits moves
                        UI_STATUS_UPD(UI_TEXT_ABORT_KEYPRESSED);
                        break;
                    }
                    e -= de; //mm - kleine schritte auf Zug
                    if (fabs(e) > 5.0f /*mm*/ && eFeedrate < 10.0f /*mm/s*/) {
                        eFeedrate += (fabs(e) > 10.0f ? 0.01f : 0.001f); // mm/s
                    }
                    if (fabs(e) > 15.0f /*mm*/ && eFeedrate < 20.0f /*mm/s*/) {
                        eFeedrate += 0.01f; // mm/s
                        if (de <= 0.02f)
                            de += 0.0001f;
                    }
                    Printer::queueFloatCoordinates(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, e, eFeedrate);
                }
                if (g_uBlockCommands)
                    break;
                if (t > Extruder::current->tempControl.targetTemperatureC && fabs(e) < 10.0f /*mm*/) {
                    //wenn der extruder nicht weiterkommt und die bisherige wegstrecke unter 10mm ist, dann etwas die Temperatur erhöhen und nochmal ziehen.
                    Extruder::setTemperatureForExtruder(t, Extruder::current->id);
                    Extruder::current->tempControl.waitForTargetTemperature();
                } else if (fabs(e) >= 10.0f /*mm*/) {
                    //das filament kam schon etwas raus, hängt aber nun.
                    if (!retry--) {
                        break;
                    }
                }
                eFeedrate = 0.3f;
            }
            Extruder::setTemperatureForExtruder(0, Extruder::current->id);
            //cleanup

            Printer::setEPositionMM(0); // G92 E0

            Printer::relativeExtruderCoordinateMode = save_relativeExtruderCoordinateMode;
            Printer::unitIsInches = saveunitIsInches;
            g_uStartOfIdle = HAL::timeInMilliseconds() + 5000; // M3914 unload filament ends
            break;
        }

        case 3919: // M3919 [S]mikrometer - Testfunction for Dip-Down-Hotend beim T1: Einstellen des extruderspezifischen Z-Offsets
        {
            /* Eigentlich kann man das mit jedem Extruder machen, aber ich lasse das nur für T1 zu, weil ich nur das testen kann. Der Rechte Extruder kann damit absinken, wenn das Filament ihn runterdrückt. Der Linke bleibt in jedem Fall gleich hoch auf der Höhe des Homings! */
            if (pCommand->hasZ() && (pCommand->Z <= 0 && pCommand->Z >= -2.0f)) {
                if (Printer::debugDryrun())
                    break;
                Commands::waitUntilEndOfAllMoves();

                uint8_t extruderId = 1;

                if (Printer::isAxisHomed(Z_AXIS) && Extruder::current->id == extruder[extruderId].id) {
                    int32_t dz = (pCommand->Z - extruder[extruderId].offsetMM[Z_AXIS]) * Printer::axisStepsPerMM[Z_AXIS];
                    // Shift the extruder-offset negatively to stay at the same point after switch
                    Printer::offsetRelativeStepsCoordinates(0, 0, -dz, 0);
                }
                extruder[extruderId].offsetMM[Z_AXIS] = (float)pCommand->Z;
                Com::printFLN(PSTR("M3919 T1 Spring displace: "), extruder[extruderId].offsetMM[Z_AXIS]);

                HAL::eprSetFloat(EEPROM::getExtruderOffset(extruderId) + EPR_EXTRUDER_Z_OFFSET, extruder[extruderId].offsetMM[Z_AXIS]); //mm negativ
                EEPROM::updateChecksum();
            } else {
                Com::printFLN(PSTR("M3919 Help: Write M3919 T1 Z-0.500 when T1 goes down 500um/0.5mm"));
            }
            break;
        }

#if RESERVE_ANALOG_INPUTS
        case 3941: // 3941 reading optional temperature port X35 - Testfunction || by Nibbels
        {
            //Com::printFLN( PSTR( "M3941 TempReader starting ..." ) );
            TemperatureController* act = &optTempController;
            act->updateCurrentTemperature();
            Com::printFLN(PSTR("Opt Temp: "), act->currentTemperatureC, 2);
            //Com::printFLN( PSTR( "M3941 Ended!" ) );
            break;
        }
#endif // RESERVE_ANALOG_INPUTS

#if FEATURE_READ_STEPPER_STATUS
        case 3987: // M3987 reading motor driver and stall pins - Testfunction || by Nibbels
        {
            Com::printFLN(PSTR("M3987 MotorStatus X-Y-Z-E0-E1 ..."));
            for (uint8_t driver = 1; driver <= 5; driver++) {
                readMotorStatus(driver);
            }
            break;
        }
#endif //FEATURE_READ_STEPPER_STATUS

   
        }
    }
} // processSpecialGCode

void queueTask(char task) {
    PrintLine::waitForXFreeLines(1);
    PrintLine::queueTask(task);
    return;
} // queueTask

bool isAnyScanRunning() {
#if FEATURE_ALIGN_EXTRUDERS
    if (g_nAlignExtrudersStatus)
        return true;
#endif //FEATURE_ALIGN_EXTRUDERS
    if (g_nHeatBedScanStatus)
        return true;
    if (g_nZOSScanStatus)
        return true;

    return false;
}

void moveXAction(int8_t direction) {
    bool moveKosys = Printer::moveKosys;

    if (g_pauseMode) {
        moveKosys = KOSYS_DIRECTOFFSET;
    } else if (PrintLine::linesCount || isAnyScanRunning()) {
        // this operation is not allowed while a printing or scanning is in progress
        return;
    }

    if (direction < 0 && Printer::isXMinEndstopHit()) {
        // we shall move to the left but the x-min-endstop is hit already, so we do nothing
        showInformation((void*)ui_text_x_axis, (void*)ui_text_min_reached);
        return;
    }

    if (direction > 0 && Printer::isXMaxEndstopHit()) {
        // we shall move to the right but the end of the x-axis has been reached already, so we do nothing
        showInformation((void*)ui_text_x_axis, (void*)ui_text_max_reached);
        return;
    }

    // show that we are active
    previousMillisCmd = HAL::timeInMilliseconds(); //prevent inactive shutdown of steppers/temps

    switch (Printer::moveMode[X_AXIS]) {
    case MOVE_MODE_SINGLE_STEPS: {
        int32_t steps = int32_t(g_nManualSteps[X_AXIS]) * direction;

        if (direction < 0 && Printer::currentXSteps + steps < 0 && Printer::isAxisHomed(X_AXIS)) {
            // If we are very close to the homed known Y-Endstop go to 0-Coordinate
            steps = -Printer::currentXSteps;
        }

        if (moveKosys == KOSYS_GCODE) {
            float feedrate = STANDARD_POSITION_FEEDRATE_XY;
            if (Printer::movePositionFeedrateChoice == FEEDRATE_GCODE) {
                feedrate = Printer::feedrate;
            }
            Printer::queueRelativeStepsCoordinates(steps, 0, 0, 0, feedrate, true);
            Commands::printCurrentPosition();
        } else /*if (Printer::moveKosys == KOSYS_DIRECTOFFSET) */
        {
            Printer::offsetRelativeStepsCoordinates(steps, 0, 0, 0, TASK_MOVE_POSITION_MANUAL);
        }

        break;
    }
    case MOVE_MODE_1_MM:
    case MOVE_MODE_10_MM:
    case MOVE_MODE_50_MM: {
        uint8_t distanceMM = 1;
        if (Printer::moveMode[X_AXIS] == MOVE_MODE_10_MM)
            distanceMM = 10;
        if (Printer::moveMode[X_AXIS] == MOVE_MODE_50_MM)
            distanceMM = 50;

        int32_t plannedDestination = Printer::getPlannedDirectAxisSteps(X_AXIS) + (int32_t)(direction * distanceMM * Printer::axisStepsPerMM[X_AXIS]);
        if (plannedDestination > Printer::maxSoftEndstopSteps[X_AXIS]) {
            showInformation((void*)ui_text_x_axis, (void*)ui_text_max_reached);
            return;
        }

        if (moveKosys == KOSYS_GCODE) {
            float feedrate = STANDARD_POSITION_FEEDRATE_XY;
            if (Printer::movePositionFeedrateChoice == FEEDRATE_GCODE) {
                feedrate = Printer::feedrate;
            }
            Printer::queueRelativeMMCoordinates(distanceMM * direction, 0, 0, 0, feedrate, true);
            Commands::printCurrentPosition();
        } else /*if (Printer::moveKosys == KOSYS_DIRECTOFFSET) */
        {
            Printer::offsetRelativeStepsCoordinates(distanceMM * Printer::axisStepsPerMM[Y_AXIS] * direction, 0, 0, 0, TASK_MOVE_POSITION_MANUAL);
        }

        break;
    }
    case MOVE_MODE_SINGLE_MOVE: {
        if (PrintLine::direct.task) {
            // we are moving already, there is nothing more to do
            return;
        }

        int32_t steps;
        //errate homed möglichst den erzwungenen stop-punkt wegen dem bonus der decelleration
        if (direction < 0)
            steps = (Printer::isAxisHomed(X_AXIS) ? -Printer::currentXSteps : -Printer::maxSoftEndstopSteps[X_AXIS]);
        else
            steps = (Printer::isAxisHomed(X_AXIS) ? Printer::maxSoftEndstopSteps[X_AXIS] - Printer::currentXSteps : Printer::maxSoftEndstopSteps[X_AXIS]);

        Printer::offsetRelativeStepsCoordinates(steps, 0, 0, 0, TASK_MOVE_FROM_BUTTON);
        break;
    }
    }
} // moveXAction

void moveYAction(int8_t direction) {
    bool moveKosys = Printer::moveKosys;

    if (g_pauseMode) {
        moveKosys = KOSYS_DIRECTOFFSET;
    } else if (PrintLine::linesCount || isAnyScanRunning()) {
        // this operation is not allowed while a printing or scanning is in progress
        return;
    }

    if (direction < 0 && Printer::isYMinEndstopHit()) {
        // we shall move to the back but the y-min-endstop is hit already, so we do nothing
        showInformation((void*)ui_text_y_axis, (void*)ui_text_min_reached);
        return;
    }

    if (direction > 0 && Printer::isYMaxEndstopHit()) {
        // we shall move to the front but the end of the y-axis has been reached already, so we do nothing
        showInformation((void*)ui_text_y_axis, (void*)ui_text_max_reached);
        return;
    }

    // show that we are active
    previousMillisCmd = HAL::timeInMilliseconds(); //prevent inactive shutdown of steppers/temps

    switch (Printer::moveMode[Y_AXIS]) {
    case MOVE_MODE_SINGLE_STEPS: {
        int32_t steps = int32_t(g_nManualSteps[Y_AXIS]) * direction;

        if (direction < 0 && Printer::currentYSteps + steps < 0 && Printer::isAxisHomed(Y_AXIS)) {
            // If we are very close to the homed known Y-Endstop go to 0-Coordinate
            steps = -Printer::currentYSteps;
        }

        if (moveKosys == KOSYS_GCODE) {
            float feedrate = STANDARD_POSITION_FEEDRATE_XY;
            if (Printer::movePositionFeedrateChoice == FEEDRATE_GCODE) {
                feedrate = Printer::feedrate;
            }
            Printer::queueRelativeStepsCoordinates(0, steps, 0, 0, feedrate, true);
            Commands::printCurrentPosition();
        } else /*if (Printer::moveKosys == KOSYS_DIRECTOFFSET) */
        {
            Printer::offsetRelativeStepsCoordinates(0, steps, 0, 0, TASK_MOVE_POSITION_MANUAL);
        }

        break;
    }
    case MOVE_MODE_1_MM:
    case MOVE_MODE_10_MM:
    case MOVE_MODE_50_MM: {
        uint8_t distanceMM = 1;
        if (Printer::moveMode[Y_AXIS] == MOVE_MODE_10_MM)
            distanceMM = 10;
        if (Printer::moveMode[Y_AXIS] == MOVE_MODE_50_MM)
            distanceMM = 50;

        int32_t plannedDestination = Printer::getPlannedDirectAxisSteps(Y_AXIS) + (int32_t)(direction * distanceMM * Printer::axisStepsPerMM[Y_AXIS]);
        if (plannedDestination > Printer::maxSoftEndstopSteps[Y_AXIS]) {
            showInformation((void*)ui_text_y_axis, (void*)ui_text_max_reached);
            return;
        }

        if (moveKosys == KOSYS_GCODE) {
            float feedrate = STANDARD_POSITION_FEEDRATE_XY;
            if (Printer::movePositionFeedrateChoice == FEEDRATE_GCODE) {
                feedrate = Printer::feedrate;
            }
            Printer::queueRelativeMMCoordinates(0, distanceMM * direction, 0, 0, feedrate, true);
            Commands::printCurrentPosition();
        } else /*if (Printer::moveKosys == KOSYS_DIRECTOFFSET) */
        {
            Printer::offsetRelativeStepsCoordinates(0, distanceMM * Printer::axisStepsPerMM[Y_AXIS] * direction, 0, 0, TASK_MOVE_POSITION_MANUAL);
        }

        break;
    }
    case MOVE_MODE_SINGLE_MOVE: {
        if (PrintLine::direct.task) {
            // we are moving already, there is nothing more to do
            return;
        }

        int32_t steps;
        //errate homed möglichst den erzwungenen stop-punkt wegen dem bonus der decelleration
        if (direction < 0)
            steps = (Printer::isAxisHomed(Y_AXIS) ? -Printer::currentYSteps : -Printer::maxSoftEndstopSteps[Y_AXIS]);
        else
            steps = (Printer::isAxisHomed(Y_AXIS) ? Printer::maxSoftEndstopSteps[Y_AXIS] - Printer::currentYSteps : Printer::maxSoftEndstopSteps[Y_AXIS]);

        Printer::offsetRelativeStepsCoordinates(0, steps, 0, 0, TASK_MOVE_FROM_BUTTON);
        break;
    }
    }
} // moveYAction

void moveZAction(int8_t direction) {
    if (isAnyScanRunning() || (g_pauseMode && !isPauseSettled())) {
        // this operation is not allowed while some scan is in progress
        return;
    }

    // show that we are active
    previousMillisCmd = HAL::timeInMilliseconds(); //prevent inactive shutdown of steppers/temps

    char moveMode = Printer::moveMode[Z_AXIS];

    if (uid.lastButtonAction == UI_ACTION_RF_HEAT_BED_DOWN || uid.lastButtonAction == UI_ACTION_RF_HEAT_BED_UP) {
        //es wäre evtl. cooler die variable durchzuschleifen, aber so gehts auch!
        //VORSICHT: ES könnte evtl. sein dass sich lastbuttonaction während der ausführung ändert. (??)

        //sollte diese Aktion durch die Knöpfe ausgeführt worden sein, dann in der Sprungweite limitieren:
        if (PrintLine::linesCount) {
            // there is some printing in progress at the moment - do not allow single move in this case
            moveMode = MOVE_MODE_SINGLE_STEPS;
        } else {
            moveMode = MOVE_MODE_SINGLE_MOVE;
        }
    }

    if (g_pauseMode) {
        moveMode = MOVE_MODE_SINGLE_MOVE;
    } else if (PrintLine::linesCount) {
        // in case we are printing at the moment, only the single step movements are allowed
        moveMode = MOVE_MODE_SINGLE_STEPS;
    }

    //Limits für die Bewegung in Z
    if (direction > 0 && Printer::isZMaxEndstopHit()) {
        // we shall move to the front but the end of the y-axis has been reached already, so we do nothing
        showInformation((void*)ui_text_z_axis, (void*)ui_text_max_reached);
        return;
    }

    if (direction < 0 && Printer::isZMinEndstopHit()) {
        if (!Printer::isAxisHomed(Z_AXIS)) {
            return;
        } else {
            //fall down to Single Steps @Endstop
            moveMode = MOVE_MODE_SINGLE_STEPS;
        }
    }

    switch (moveMode) {
    case MOVE_MODE_SINGLE_STEPS: {
        int32_t steps = int32_t(g_nManualSteps[Z_AXIS]) * direction;

        if (direction < 0 && Printer::isZMinEndstopHit() && Printer::currentZSteps + steps < -1 * int32_t(Printer::maxZOverrideSteps)) {
            // do not allow to drive the bed into the extruder
            showInformation((void*)ui_text_z_axis, (void*)ui_text_min_reached);
            return;
        }

        if (g_pauseMode && direction < 0) {
            const long returnSteps = pauseReturnSteps(Z_AXIS);
            if (returnSteps >= 0) {
                return;
            }
            steps = RMath::max(steps, returnSteps);
        }

        //Z should be direct, because we have scan origin and z-origin to set coordinates and while printing gcode coordinate system is senseless.

        //if (Printer::moveKosys == KOSYS_GCODE)
        //{
        //	Printer::queueRelativeStepsCoordinates(0, 0, steps, 0, Printer::homingFeedrate[Z_AXIS], true);
        //}
        //else /*if (Printer::moveKosys == KOSYS_DIRECTOFFSET) */
        //{
        Printer::offsetRelativeStepsCoordinates(0, 0, steps, 0, TASK_MOVE_POSITION_MANUAL);
        //}

        break;
    }
    case MOVE_MODE_1_MM:
    case MOVE_MODE_10_MM:
    case MOVE_MODE_50_MM: {
        uint8_t distanceMM = 1;
        if (moveMode == MOVE_MODE_10_MM)
            distanceMM = 10;
        if (moveMode == MOVE_MODE_50_MM)
            distanceMM = 50;

        // Normale Bewegung möglich:
        if (!Printer::isAxisHomed(Z_AXIS)
            || direction > 0
            || Printer::currentZSteps * Printer::axisMMPerSteps[Z_AXIS] >= (float)distanceMM) {
            //#if MENU_POSITION_ALL_DIRECT
            Printer::offsetRelativeStepsCoordinates(0, 0, distanceMM * Printer::axisStepsPerMM[Z_AXIS] * direction, 0, TASK_MOVE_POSITION_MANUAL);
            //#else
            // if enabled fix feedrate
            //				Printer::queueRelativeMMCoordinates(0, 0, distanceMM * direction, 0, Printer::homingFeedrate[Z_AXIS], true);
            //#endif

            return;
        }


        // Wir sind hier knapp über dem Schalter: Nur so weit runterfahren, wie man über 0 ist. Denn da ist beim Printermode homed der Endschalter.
        if (Printer::currentZSteps > 0) {
            //#if MENU_POSITION_ALL_DIRECT
            Printer::offsetRelativeStepsCoordinates(0, 0, -Printer::currentZSteps, 0, TASK_MOVE_POSITION_MANUAL);
            //#else
            // if enabled fix feedrate
            //				Printer::queueRelativeStepsCoordinates(0, 0, -Printer::currentZSteps, 0, Printer::homingFeedrate[Z_AXIS], true);
            //#endif

            return;
        }

        //#if MENU_POSITION_ALL_DIRECT
        // Sonst Stepweise
        Printer::offsetRelativeStepsCoordinates(0, 0, -1 * int32_t(g_nManualSteps[Z_AXIS]), 0, TASK_MOVE_POSITION_MANUAL);
        //#else
        // if enabled fix feedrate
        //			Printer::queueRelativeStepsCoordinates(0, 0, -1 * int32_t(g_nManualSteps[Z_AXIS]), 0, Printer::homingFeedrate[Z_AXIS], true);
        //#endif

        break;
    }
    case MOVE_MODE_SINGLE_MOVE: {
        if (PrintLine::direct.task) {
            // we are moving already, there is nothing more to do
            return;
        }

        int32_t steps;
        //errate homed möglichst den erzwungenen stop-punkt wegen dem bonus der decelleration
        if (direction < 0)
            steps = (Printer::isAxisHomed(Z_AXIS) ? -Printer::currentZSteps : -Printer::maxSoftEndstopSteps[Z_AXIS]);
        else
            steps = (Printer::isAxisHomed(Z_AXIS) ? Printer::maxSoftEndstopSteps[Z_AXIS] - Printer::currentZSteps : Printer::maxSoftEndstopSteps[Z_AXIS]);

        if (g_pauseMode && direction < 0) {
            const long returnSteps = pauseReturnSteps(Z_AXIS);
            if (returnSteps >= 0) {
                return;
            }
            steps = RMath::max(steps, returnSteps);
        }

        Printer::offsetRelativeStepsCoordinates(0, 0, steps, 0, TASK_MOVE_FROM_BUTTON);
        break;
    }
    }
} // moveZAction

#if STEPPER_CURRENT_CONTROL == CURRENT_CONTROL_DRV8711

#if DRV8711_NUM_CHANNELS != 5
#error Wer DRV8711_NUM_CHANNELS ändert, muss sich wirklich reindenken und von den Achszuordnungen bis zu den EEPROM-Speicherplätzen alles überdenken. Normalerweise darf dieser Wert nicht anders als 5 sein. Ausser Conrad baut ein Board mit 3 Extrudersteppern oder ähnlich.
#endif // DRV8711_NUM_CHANNELS != 5

void drv8711Transmit(unsigned short command) {
    // transfer the command (= direction, address and data)
    InterruptProtectedBlock noInts; //HAL::forbidInterrupts();
    for (char i = 15; i >= 0; i--) {
        WRITE(DRV_SDATI, command & (0x01 << i));
        HAL::delayMicroseconds(1);
        WRITE(DRV_SCLK, 1);
        HAL::delayMicroseconds(5);
        WRITE(DRV_SCLK, 0);
    }
    noInts.unprotect(); //HAL::allowInterrupts();

} // drv8711Transmit

unsigned short drv8711Receive(unsigned char address) {
    unsigned short acknowledge = 0;
    unsigned short temp;
    char i;

    if (address > 7)
        return 0;

    acknowledge = address;
    acknowledge = acknowledge << 12;
    acknowledge |= 0x8000;

    // transfer the read request plus the register address (4 bits)
    InterruptProtectedBlock noInts; //HAL::forbidInterrupts();
    for (i = 15; i >= 12; i--) {
        WRITE(DRV_SDATI, acknowledge & (0x01 << i));
        HAL::delayMicroseconds(1);
        WRITE(DRV_SCLK, 1);
        HAL::delayMicroseconds(5);
        WRITE(DRV_SCLK, 0);
    }

    HAL::delayMicroseconds(20);

    // read the acknowledge (12 bits)
    for (i = 11; i >= 0; i--) {
        temp = READ(DRV_SDATO);
        acknowledge = acknowledge | (temp << i);
        WRITE(DRV_SCLK, 1);
        HAL::delayMicroseconds(25);
        WRITE(DRV_SCLK, 0);
        HAL::delayMicroseconds(25);
    }
    noInts.unprotect(); //HAL::allowInterrupts();

    return acknowledge;

} // drv8711Receive

void drv8711EnableAll(void) {
    // enable the chip select of all present DRV8711
    switch (DRV8711_NUM_CHANNELS) {
    case 5: {
        WRITE(O1_SCS_PIN, LOW);
        SET_OUTPUT(O1_SCS_PIN);
        WRITE(O1_SCS_PIN, HIGH);
    } // fall through
    case 4: {
        WRITE(O0_SCS_PIN, LOW);
        SET_OUTPUT(O0_SCS_PIN);
        WRITE(O0_SCS_PIN, HIGH);
    } // fall through
    case 3: {
        WRITE(Z_SCS_PIN, LOW);
        SET_OUTPUT(Z_SCS_PIN);
        WRITE(Z_SCS_PIN, HIGH);
    } // fall through
    case 2: {
        WRITE(Y_SCS_PIN, LOW);
        SET_OUTPUT(Y_SCS_PIN);
        WRITE(Y_SCS_PIN, HIGH);
    } // fall through
    case 1: {
        WRITE(X_SCS_PIN, LOW);
        SET_OUTPUT(X_SCS_PIN);
        WRITE(X_SCS_PIN, HIGH);
    }
    }

} // drv8711EnableAll

void drv8711DisableAll(void) {
    // disable the chip select of all present DRV8711
    switch (DRV8711_NUM_CHANNELS) {
    case 5: {
        WRITE(O1_SCS_PIN, LOW);
    } // fall through
    case 4: {
        WRITE(O0_SCS_PIN, LOW);
    } // fall through
    case 3: {
        WRITE(Z_SCS_PIN, LOW);
    } // fall through
    case 2: {
        WRITE(Y_SCS_PIN, LOW);
    } // fall through
    case 1: {
        WRITE(X_SCS_PIN, LOW);
    }
    }

} // drv8711DisableAll

void drv8711Enable(unsigned char driver) {
    // enable the chip select of the DRV8711
    switch (driver) {
    case 5: {
        WRITE(O1_SCS_PIN, LOW);
        SET_OUTPUT(O1_SCS_PIN);
        WRITE(O1_SCS_PIN, HIGH);
        break;
    }
    case 4: {
        WRITE(O0_SCS_PIN, LOW);
        SET_OUTPUT(O0_SCS_PIN);
        WRITE(O0_SCS_PIN, HIGH);
        break;
    }
    case 3: {
        WRITE(Z_SCS_PIN, LOW);
        SET_OUTPUT(Z_SCS_PIN);
        WRITE(Z_SCS_PIN, HIGH);
        break;
    }
    case 2: {
        WRITE(Y_SCS_PIN, LOW);
        SET_OUTPUT(Y_SCS_PIN);
        WRITE(Y_SCS_PIN, HIGH);
        break;
    }
    case 1: {
        WRITE(X_SCS_PIN, LOW);
        SET_OUTPUT(X_SCS_PIN);
        WRITE(X_SCS_PIN, HIGH);
        break;
    }
    }

} // drv8711Enable

void drv8711Disable(unsigned char driver) {
    // disable the chip select of the DRV8711
    switch (driver) {
    case 5: {
        WRITE(O1_SCS_PIN, LOW);
        break;
    }
    case 4: {
        WRITE(O0_SCS_PIN, LOW);
        break;
    }
    case 3: {
        WRITE(Z_SCS_PIN, LOW);
        break;
    }
    case 2: {
        WRITE(Y_SCS_PIN, LOW);
        break;
    }
    case 1: {
        WRITE(X_SCS_PIN, LOW);
        break;
    }
    }

} // drv8711Disable

uint8_t getmsb(unsigned int x) {
    int r = 0;
    if (x < 1)
        return 0;
    while (x >>= 1)
        r++;
    return r;
}

uint8_t drv8711MicroSteps_2_ModeValue(unsigned short microsteps) { //unknown steps fall back to highest bit!
    return getmsb(microsteps);                                     //highest bit << 1 -> 256->8, 128->7, ... integer log2
} //drv8711MicroSteps_2_ModeValue

unsigned short drv8711Axis_2_InitMicrosteps(uint8_t axis) {
    return (axis == X_AXIS || axis == Y_AXIS ? RF_MICRO_STEPS_XY : (axis == Z_AXIS ? RF_MICRO_STEPS_Z : (axis == E_AXIS || axis == E_AXIS + 1 ? RF_MICRO_STEPS_E : RF_MICRO_STEPS_XY /*error fallback*/
                                                                                                         )));
}

unsigned short drv8711ModeValue_2_MicroSteps(uint8_t modeValue) {
    if (modeValue > 8)
        return 256; //constrain, but ERROR -> avoid this!
    return (1 << modeValue);
} // drv8711ModeValue_2_MicroSteps

unsigned short drv8711MicroSteps_2_Register00Hex(unsigned short microsteps) {

    /* http://www.ti.com/product/DRV8711/datasheet/detailed-description#SLVSC405764
    DTIME
    00: 400 ns dead time
    01: 450 ns dead time
    10: 650 ns dead time
    11: 850 ns dead time

    ISENSE amplifier gain set
    00: Gain of 5
    01: Gain of 10
    10: Gain of 20
    11: Gain of 40

    EXSTALL
    0: Internal stall detect
    1: External stall detect

    MODE
    0000: Full-step, 71% current
    0001: Half step
    0010: 1/4 step
    0011: 1/8 step
    0100: 1/16 step
    0101: 1/32 step
    0110: 1/64 step
    0111: 1/128 step
    1000: 1/256 step
    1001 – 1111: Reserved
    */
    // ADRESS 11..8 7..4 3..0
    unsigned short Register00Hex = 0x0E01; // 0000 1110 0XXX X001: ENBL = 1, RDIR = 0, RSTEP = 0, MODE = XXXX, EXSTALL = 0, ISGAIN = 10, DTIME = 11

    Register00Hex += (drv8711MicroSteps_2_ModeValue(microsteps) << 3); //schiebe zu bit 3..6

    return Register00Hex;
} // drv8711MicroSteps_2_Register00Hex

void drv8711adjustMicroSteps(unsigned char driver) {
    drv8711Enable(driver);
    //X = 1
    //Y = 2
    //Z = 3
    //E0 = 4
    //E1 = 5
#if FEATURE_ADJUSTABLE_MICROSTEPS
    unsigned short reg00bytes = drv8711MicroSteps_2_Register00Hex(drv8711ModeValue_2_MicroSteps(Printer::motorMicroStepsModeValue[driver - 1 /*=axis*/]));
#else
    unsigned short reg00bytes = drv8711MicroSteps_2_Register00Hex(drv8711Axis_2_InitMicrosteps(driver - 1));
#endif // FEATURE_ADJUSTABLE_MICROSTEPS
    if (reg00bytes)
        drv8711Transmit(reg00bytes);
    drv8711Disable(driver);
    HAL::delayMicroseconds(1);
}

void drv8711Init(void) {
    // configure the pins
    WRITE(DRV_RESET1, LOW);
    SET_OUTPUT(DRV_RESET1);

#if DRV_RESET2
    WRITE(DRV_RESET2, LOW);
    SET_OUTPUT(DRV_RESET2);
#endif // DRV_RESET2

    WRITE(DRV_SCLK, LOW);
    SET_OUTPUT(DRV_SCLK);
    WRITE(DRV_SDATI, LOW);
    SET_OUTPUT(DRV_SDATI);

    // configure the following inputs as pullup
    WRITE(DRV_SDATO, HIGH);
    WRITE(DRV_FAULT, HIGH);
    WRITE(X_STALL_PIN, HIGH);
    WRITE(Y_STALL_PIN, HIGH);
    WRITE(Z_STALL_PIN, HIGH);
    WRITE(O0_STALL_PIN, HIGH);
    WRITE(O1_STALL_PIN, HIGH);

    // reset all DRV8711 (active high)
    WRITE(DRV_RESET1, HIGH);

#if DRV_RESET2
    WRITE(DRV_RESET2, HIGH);
#endif // DRV_RESET2

    HAL::delayMicroseconds(5000);
    WRITE(DRV_RESET1, LOW);

#if DRV_RESET2
    WRITE(DRV_RESET2, LOW);
#endif // DRV_RESET2

    HAL::delayMicroseconds(5000);

    // configure all registers except the motor current (= no register 01)

    //DRV8711_REGISTER_00:
#if FEATURE_ADJUSTABLE_MICROSTEPS
    //init to stock settings: update in eeprom init.
    for (uint8_t axis = 0; axis < DRV8711_NUM_CHANNELS; axis++) {
        Printer::motorMicroStepsModeValue[axis] = drv8711MicroSteps_2_ModeValue(drv8711Axis_2_InitMicrosteps(axis)); //init
    }
#endif // FEATURE_ADJUSTABLE_MICROSTEPS
    for (uint8_t driver = 1; driver <= DRV8711_NUM_CHANNELS; driver++) {
        drv8711adjustMicroSteps(driver);
    }
    // ADRESS 11..8 7..4 3..0

// Register 0x02: OFF (Off-Time Register)
// TOFF wurde auf 0x20 (16µs) reduziert, um die Chopping-Frequenz zu erhöhen
// und hörbares Zwitschern zu beseitigen. – Beibehalten.
// PWMMODE = 0 (interner Indexer, Standard für Mikroschritt) – Beibehalten.
#define DRV8711_REGISTER_02 0x2020 // 0010 0000 0010 0000: TOFF = 00100000 (0x20), PWMMODE = 0

// Register 0x03: BLANK (Blanking Time Register) -- intentionally NOT written.
// drv8711Init() hardware-resets the drivers above, leaving the DRV8711 default:
// TBLANK = 0x80 (2.56 us), ABT = 0 (adaptive blanking disabled).
// The previously copied 0xB2D7 has SPI bit 15 set (READ), so the old
// drv8711Transmit(0xB2D7) never applied the intended blanking settings.
// Do not silently replace it with a write such as 0x3102 (1.0 us, ABT on):
// blanking/decay timing can affect motor current, torque and missed steps.
// Keeping BLANK at its reset default preserves the existing motor behavior.

// Register 0x04: DECAY (Decay Mode Register)
// TDECAY wurde von 0x14 (10µs) auf 0x04 (2µs) geändert.
// Eine kürzere Decay-Zeit kann in manchen Fällen zu einer noch glatteren
// Stromwellenform und somit zu einer weiteren Geräuschreduzierung führen.
// DECMOD = 101 (Auto Mixed Decay) – Beibehalten, da dies für leisen Betrieb vorteilhaft ist.
#define DRV8711_REGISTER_04 0x4504 // 0100 0101 0000 0100: TDECAY = 00000100 (0x04), DECMOD = 101

#define DRV8711_REGISTER_05 0x583C // 0101   1000  0011 1100: SDTHR = 00111100, SDCNT = 00, VDIV = 10
#define DRV8711_REGISTER_06 0x60F0 // 0110   0000  1111 0000: OCPTH = 00, OCPDEG = 00, TDRIVEN = 11, TDRIVEP = 11, IDRIVEN = 00, IDRIVEP = 00
#define DRV8711_REGISTER_07 0x7000 // 0111   0000  0000 0000: OTS = 0, AOCP = 0, BOCP = 0, UVLO = 0, APDF = 0, BPDF = 0, STD = 0, STDLAT = 0

    drv8711EnableAll();
    drv8711Transmit(DRV8711_REGISTER_02);
    drv8711DisableAll();
    HAL::delayMicroseconds(1);

    drv8711EnableAll();
    drv8711Transmit(DRV8711_REGISTER_04);
    drv8711DisableAll();
    HAL::delayMicroseconds(1);

    drv8711EnableAll();
    drv8711Transmit(DRV8711_REGISTER_05);
    drv8711DisableAll();
    HAL::delayMicroseconds(1);

    drv8711EnableAll();
    drv8711Transmit(DRV8711_REGISTER_06);
    drv8711DisableAll();
    HAL::delayMicroseconds(1);

    drv8711EnableAll();
    drv8711Transmit(DRV8711_REGISTER_07);
    drv8711DisableAll();

    // set all motor currents
    const unsigned short uMotorCurrentUse[] = MOTOR_CURRENT_NORMAL; //--> {x,y,z,e1,e2} siehe RFx000.h
    Printer::motorCurrent[X_AXIS] = uMotorCurrentUse[X_AXIS];
    Printer::motorCurrent[Y_AXIS] = uMotorCurrentUse[Y_AXIS];
    Printer::motorCurrent[Z_AXIS] = uMotorCurrentUse[Z_AXIS];
    Printer::motorCurrent[E_AXIS + 0] = uMotorCurrentUse[E_AXIS + 0];
    Printer::motorCurrent[E_AXIS + 1] = uMotorCurrentUse[E_AXIS + 1]; //egal ob NUM_EXTRUDER == 1 oder 2

    for (uint8_t axis = 0; axis < DRV8711_NUM_CHANNELS; axis++) {
        if (axis < 3 + NUM_EXTRUDER)
            setMotorCurrent(axis + 1, Printer::motorCurrent[axis]);
    }

} // drv8711Init

void setMotorCurrent(unsigned char driver, uint8_t level) {
    // NOTE: Do not increase the current endlessly. In case the engine reaches its current saturation, the engine and the driver can heat up and loss power.
    // When the saturation is reached, more current causes more heating and more power loss.
    // In case of engines with lower quality, the saturation current may be reached before the nominal current.

    //X = 1
    //Y = 2
    //Z = 3
    //E0 = 4
    //E1 = 5

    // configure the pins
    WRITE(DRV_SCLK, LOW);
    SET_OUTPUT(DRV_SCLK);
    WRITE(DRV_SDATI, LOW);
    SET_OUTPUT(DRV_SDATI);

    drv8711Enable(driver);

    // we have to write to register 01
    unsigned short command = 0x1100 + (short)level;
    drv8711Transmit(command);

    drv8711Disable(driver);

} // setMotorCurrent

#if FEATURE_READ_STEPPER_STATUS
unsigned short readMotorStatus(unsigned char driver) {
    //driver:
    //X = 1
    //Y = 2
    //Z = 3
    //E0 = 4
    //E1 = 5

    /*
0    OTS    1    R/W    0    0: Normal operation
1: Device has entered overtemperature shutdown
Write a 0 to this bit to clear the fault.
Operation automatically resumes when the temperature has fallen to safe levels.
1    AOCP    1    R/W    0    0: Normal operation
1: Channel A overcurrent shutdown
Write a 0 to this bit to clear the fault and resume operation
2    BOCP    1    R/W    0    0: Normal operation
1: Channel B overcurrent shutdown
Write a 0 to this bit to clear the fault and resume operation
3    APDF    1    R/W    0    0: Normal operation
1: Channel A predriver fault
Write a 0 to this bit to clear the fault and resume operation.
4    BPDF    1    R/W    0    0: Normal operation
1: Channel B predriver fault
Write a 0 to this bit to clear the fault and resume operation
5    UVLO    1    R/W    0    0: Normal operation
1: Undervoltage lockout
Write a 0 to this bit to clear the fault. The UVLO bit cannot be cleared in sleep mode. Operation automatically resumes when VM has increased above VUVLO
6    STD    1    R    0    0: Normal operation
1: Stall detected
7    STDLAT    1    R/W    0    0: Normal operation
1: Latched stall detect
Write a 0 to this bit to clear the fault and resume operation
11-8    Reserved    4    -    -    Reserved
*/

    // configure the pins
    WRITE(DRV_SCLK, LOW);
    SET_OUTPUT(DRV_SCLK);
    WRITE(DRV_SDATI, LOW);
    SET_OUTPUT(DRV_SDATI);

    drv8711Enable(driver);
    unsigned short status = drv8711Receive(7); //7.6.9 STATUS Register (Address = 0x07)
    drv8711Disable(driver);

    Com::printF(PSTR("Driver: "), driver);
    switch (driver) {
    case 1: {
        Com::printFLN(PSTR("X"));
        break;
    }
    case 2: {
        Com::printFLN(PSTR("Y"));
        break;
    }
    case 3: {
        Com::printFLN(PSTR("Z"));
        break;
    }
    case 4: {
        Com::printFLN(PSTR("E0"));
        break;
    }
    case 5: {
        Com::printFLN(PSTR("E1"));
        break;
    }
    }

    uint8_t bit = 0;
    for (uint8_t n = 0; n <= 7; n++) {
        bit = status & (1 << n) ? 1 : 0;
        switch (n) {
        case 0: {
            Com::printFLN(PSTR("OTS OverTemp "), bit);
            break;
        }
        case 1: {
            Com::printFLN(PSTR("AOCP Channel A Overcurrent "), bit);
            break;
        }
        case 2: {
            Com::printFLN(PSTR("BOCP Channel B Overcurrent "), bit);
            break;
        }
        case 3: {
            Com::printFLN(PSTR("APDF Channel A predriver fault "), bit);
            break;
        }
        case 4: {
            Com::printFLN(PSTR("BPDF Channel B predriver fault "), bit);
            break;
        }
        case 5: {
            Com::printFLN(PSTR("UVLO Undervoltage lockout "), bit);
            break;
        }
        case 6: {
            Com::printFLN(PSTR("STD Stall detected "), bit);
            break;
        }
        case 7: {
            Com::printFLN(PSTR("STDLAT Latched stall detect "), bit);
            break;
        }
        }
    }

    switch (driver) {
    case 5: {
        bit = READ(O1_STALL_PIN);
        break;
    }
    case 4: {
        bit = READ(O0_STALL_PIN);
        break;
    }
    case 3: {
        bit = READ(Z_STALL_PIN);
        break;
    }
    case 2: {
        bit = READ(Y_STALL_PIN);
        break;
    }
    case 1: {
        bit = READ(X_STALL_PIN);
        break;
    }
    }

    Com::printFLN(PSTR("Stall Pin: "), !bit);
    return (status & (bit << 8));
} // readMotorStatus
#endif // FEATURE_READ_STEPPER_STATUS

#endif // CURRENT_CONTROL_DRV8711

void cleanupEPositions(void) {
    InterruptProtectedBlock noInts;

    Printer::setEPositionMM(0);
    Printer::resetDirectAxis(E_AXIS);

    noInts.unprotect();
} // cleanupEPositions

void switchActiveHeatBed(char newActiveHeatBed) {
    if (!saveActiveHeatBedToExtEEPROM(newActiveHeatBed)) {
        // do not allow not-supported z-compensation matrix
        return;
    }

    g_nActiveHeatBed = newActiveHeatBed;

    if (loadCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed))) {
        // there is no valid z-compensation matrix available
        initCompensationMatrix();
    }
} // switchActiveHeatBed

#ifdef RGB_LIGHT_R_PIN
void setRGBTargetColors(uint8_t R, uint8_t G, uint8_t B) {
    g_uRGBTargetR = R;
    g_uRGBTargetG = G;
    g_uRGBTargetB = B;

} // setRGBTargetColors

void setRGBLEDs(uint8_t R, uint8_t G, uint8_t B) {
    if (R > 0) {
        TCCR4A |= (1 << COM4A1); // R > 0 PWM start
        OCR4A = R * 80;
    } else {
        TCCR4A &= ~(1 << COM4A1); // R = 0 PWM stop
    }

    if (G > 0) {
        TCCR4A |= (1 << COM4B1); // G > 0 PWM start
        OCR4B = G * 80;
    } else {
        TCCR4A &= ~(1 << COM4B1); // G = 0 PWM stop
    }

    if (B > 0) {
        TCCR4A |= (1 << COM4C1); // B > 0 PWM start
        OCR4C = B * 80;
    } else {
        TCCR4A &= ~(1 << COM4C1); // B = 0 PWM stop
    }

    g_uRGBCurrentR = R;
    g_uRGBCurrentG = G;
    g_uRGBCurrentB = B;

} // setRGBLEDs

void updateRGBLightStatus(void) {
    char newStatus = RGB_STATUS_IDLE;

    if (Printer::RGBButtonBackPressed) {
        // toggle the white light
        Printer::RGBLightModeForceWhite = !Printer::RGBLightModeForceWhite;

        if (Printer::RGBLightModeForceWhite) {
            setRGBTargetColors(255, 255, 255);
        } else {
            if (Printer::RGBLightMode == RGB_MODE_OFF) {
                setRGBTargetColors(0, 0, 0);
            } else if (Printer::RGBLightMode == RGB_MODE_AUTOMATIC) {
                Printer::RGBLightStatus = RGB_STATUS_AUTOMATIC;
            } else if (Printer::RGBLightMode == RGB_MODE_MANUAL) {
                setRGBTargetColors(g_uRGBManualR, g_uRGBManualG, g_uRGBManualB);
            }
        }

        Printer::RGBButtonBackPressed = 0;
    }

    if (Printer::RGBLightModeForceWhite) {
        // there is nothing to do in case we shall display the white light
        return;
    }

    if (Printer::RGBLightStatus == RGB_STATUS_NOT_AUTOMATIC) {
        // there is nothing to do in case we shall not change the RGB colors automatically
        return;
    }


    // operating mode print
#if NUM_EXTRUDER >= 1
    for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
        if (extruder[i].tempControl.targetTemperatureC > MAX_ROOM_TEMPERATURE) {
            if (fabs(extruder[i].tempControl.targetTemperatureC - extruder[i].tempControl.currentTemperatureC) < RGB_LIGHT_TEMP_TOLERANCE) {
                // we have reached the target temperature
                newStatus = RGB_STATUS_PRINTING;
            } else if (extruder[i].tempControl.targetTemperatureC > extruder[i].tempControl.currentTemperatureC) {
                // we are still heating
                newStatus = RGB_STATUS_HEATING;
            } else {
                // we end up here in case the target temperature is below the current temperature (this happens typically when the target temperature is reduced after the first layer)
            }
        }
    }
#endif // NUM_EXTRUDER >= 1

    if (heatedBedController.targetTemperatureC > MAX_ROOM_TEMPERATURE) {
        if (fabs(heatedBedController.targetTemperatureC - heatedBedController.currentTemperatureC) < RGB_LIGHT_TEMP_TOLERANCE) {
            // we have reached the target temperature
            if (newStatus == RGB_STATUS_IDLE) {
                newStatus = RGB_STATUS_PRINTING;
            }
        } else if (heatedBedController.targetTemperatureC > heatedBedController.currentTemperatureC) {
            // we are still heating
            newStatus = RGB_STATUS_HEATING;
        } else {
            // we end up here in case the target temperature is below the current temperature (this happens typically when the target temperature is reduced after the first layer)
        }
    }

#if NUM_EXTRUDER >= 1
    for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
        if ((extruder[i].tempControl.currentTemperatureC - extruder[i].tempControl.targetTemperatureC) > COOLDOWN_THRESHOLD) {
            // we shall cool down
            if (newStatus == RGB_STATUS_IDLE) {
                newStatus = RGB_STATUS_COOLING;
            }
        }
    }
#endif // NUM_EXTRUDER >= 1

    if ((heatedBedController.currentTemperatureC - heatedBedController.targetTemperatureC) > COOLDOWN_THRESHOLD) {
        // we shall cool down
        if (newStatus == RGB_STATUS_IDLE) {
            newStatus = RGB_STATUS_COOLING;
        }
    }

    if (newStatus != Printer::RGBLightStatus) {
        if (newStatus == RGB_STATUS_IDLE && Printer::RGBLightStatus == RGB_STATUS_COLOR_CHANGE) {
            // when we are in color change mode already we shall not switch back to idle
        } else {
            Printer::RGBLightStatus = newStatus;
            //          Com::printFLN( PSTR( "new RGB light status: " ), Printer::RGBLightStatus );

            switch (Printer::RGBLightStatus) {
            case RGB_STATUS_PRINTING: {
                setRGBTargetColors(g_uRGBPrintingR, g_uRGBPrintingG, g_uRGBPrintingB);
                break;
            }
            case RGB_STATUS_HEATING: {
                setRGBTargetColors(g_uRGBHeatingR, g_uRGBHeatingG, g_uRGBHeatingB);
                break;
            }
            case RGB_STATUS_COOLING: {
                setRGBTargetColors(g_uRGBCoolingR, g_uRGBCoolingG, g_uRGBCoolingB);
                break;
            }
            case RGB_STATUS_IDLE: // fall through
            default: {
                Printer::RGBLightIdleStart = HAL::timeInMilliseconds();
                Printer::RGBLightStatus = RGB_STATUS_IDLE;
                setRGBTargetColors(g_uRGBIdleR, g_uRGBIdleG, g_uRGBIdleB);
                break;
            }
            }
        }
    }

    if (Printer::RGBLightStatus == RGB_STATUS_IDLE && Printer::RGBLightIdleStart) {
        if ((HAL::timeInMilliseconds() - Printer::RGBLightIdleStart) / 1000 > RGB_LIGHT_COLOR_CHANGE_DELAY) {
            Printer::RGBLightStatus = RGB_STATUS_COLOR_CHANGE;
            Printer::RGBLightIdleStart = 0;

            //          Com::printFLN( PSTR( "new RGB light status: " ), Printer::RGBLightStatus );
            setRGBTargetColors(255, 0, 0);
        }
    }

    if (Printer::RGBLightStatus == RGB_STATUS_COLOR_CHANGE) {
        if (g_uRGBTargetR == g_uRGBCurrentR && g_uRGBTargetG == g_uRGBCurrentG && g_uRGBTargetB == g_uRGBCurrentB) {
            if (g_uRGBTargetR == 255 && g_uRGBTargetG == 0 && g_uRGBTargetB == 0) {
                g_uRGBTargetG = 255;
            } else if (g_uRGBTargetR == 255 && g_uRGBTargetG == 255 && g_uRGBTargetB == 0) {
                g_uRGBTargetR = 0;
            } else if (g_uRGBTargetR == 0 && g_uRGBTargetG == 255 && g_uRGBTargetB == 0) {
                g_uRGBTargetB = 255;
            } else if (g_uRGBTargetR == 0 && g_uRGBTargetG == 255 && g_uRGBTargetB == 255) {
                g_uRGBTargetG = 0;
            } else if (g_uRGBTargetR == 0 && g_uRGBTargetG == 0 && g_uRGBTargetB == 255) {
                g_uRGBTargetR = 255;
            } else if (g_uRGBTargetR == 255 && g_uRGBTargetG == 0 && g_uRGBTargetB == 255) {
                g_uRGBTargetB = 0;
            }
        }
    }

    return;

} // updateRGBLightStatus
#endif // RGB_LIGHT_R_PIN

static void setupForPrinting() {
    Printer::setSomeTempsensorDefect(false);

#if EEPROM_MODE
    // read the settings from the EEPROM
    Printer::homingFeedrate[X_AXIS] = HAL::eprGetFloat(EPR_X_HOMING_FEEDRATE_PRINT);
    Printer::homingFeedrate[Y_AXIS] = HAL::eprGetFloat(EPR_Y_HOMING_FEEDRATE_PRINT);
    Printer::homingFeedrate[Z_AXIS] = HAL::eprGetFloat(EPR_Z_HOMING_FEEDRATE_PRINT);
#else
    // read the settings from Configuration.h
    Printer::homingFeedrate[X_AXIS] = HOMING_FEEDRATE_X_PRINT;
    Printer::homingFeedrate[Y_AXIS] = HOMING_FEEDRATE_Y_PRINT;
    Printer::homingFeedrate[Z_AXIS] = HOMING_FEEDRATE_Z_PRINT;
#endif // EEPROM_MODE

#if EEPROM_MODE
    Printer::axisLengthMM[X_AXIS] = HAL::eprGetFloat(EPR_X_LENGTH);
    if (Printer::axisLengthMM[X_AXIS] <= 0 || Printer::axisLengthMM[X_AXIS] > 245.0f) {
        Printer::axisLengthMM[X_AXIS] = X_MAX_LENGTH_PRINT;
        HAL::eprSetFloat(EPR_X_LENGTH, Printer::axisLengthMM[X_AXIS]);
        EEPROM::updateChecksum();
    }
#else
    Printer::axisLengthMM[X_AXIS] = X_MAX_LENGTH_PRINT;
#endif // EEPROM_MODE

    loadActiveHeatBedFromExtEEPROM();
    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
        // we load the z compensation matrix before its first usage because this can take some time
        // This function needs the corrected axis length for restoreDefaultScanParameters();
        prepareZCompensation();
    }

    g_nPauseSteps[X_AXIS] = long(Printer::axisStepsPerMM[X_AXIS] * DEFAULT_PAUSE_MM_X_PRINT);
    g_nPauseSteps[Y_AXIS] = long(Printer::axisStepsPerMM[Y_AXIS] * DEFAULT_PAUSE_MM_Y_PRINT);
    g_nPauseSteps[Z_AXIS] = long(Printer::axisStepsPerMM[Z_AXIS] * DEFAULT_PAUSE_MM_Z_PRINT);

    Printer::updateDerivedParameter();

    g_staticZSteps = (Printer::ZOffset * Printer::axisStepsPerMM[Z_AXIS]) / 1000;

    g_uStartOfIdle = HAL::timeInMilliseconds(); //setupForPrinting
} // setupForPrinting

void prepareZCompensation(void) {
    if (COMPENSATION_MATRIX_SIZE > EEPROM_SECTOR_SIZE) {
        if (Printer::debugErrors()) {
            Com::printFLN(PSTR("prepareZCompensation(): the size of the compensation matrix is too big"));
        }

        // TODO: show a message at the display in this case
        return;
    }

    // restore the default scan parameters
    restoreDefaultScanParameters();

    // restore the last known compensation matrix
    // this operation must be performed after restoring of the default scan parameters because the values from the EEPROM can overwrite some scan parameters
    if (loadCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed))) {
        // there is no valid compensation matrix available
        initCompensationMatrix();
        Com::printFLN(PSTR("prepareZCompensation(): the compensation matrix is not available"));
    }

} // prepareZCompensation

unsigned char isMovingAllowed(const char* pszCommand, char outputLog) {
    if (!Printer::g_unlock_movement) {
        // A host may keep sending motion commands after a controller reset.
        Com::printF(pszCommand);
        Com::printFLN(PSTR(": movement locked until homing or panel input"));
        return 0;
    }
    if (Printer::blockAll) {
        // do not allow to move in case the movements have been blocked
        return 0;
    }

    if (g_nHeatBedScanStatus || g_nZOSScanStatus
#if FEATURE_ALIGN_EXTRUDERS
        || g_nAlignExtrudersStatus
#endif //FEATURE_ALIGN_EXTRUDERS
    ) {
        // do not allow manual movements while the heat bed scan is in progress
        if (Printer::debugErrors() && outputLog) {
            Com::printF(pszCommand);
            Com::printFLN(PSTR(": command can not be used while scan is in progress"));
        }
        return 0;
    }

    // we allow the manual movements at the moment
    return 1;

} // isMovingAllowed

unsigned char isHomingAllowed(GCode* com, char outputLog) {
#if FEATURE_ALIGN_EXTRUDERS
    if (g_nAlignExtrudersStatus) {
        // do not allow homing while the align is in progress
        if (Printer::debugErrors() && outputLog) {
            Com::printFLN(PSTR("G28: homing can not be performed while alignment is in progress"));
        }
        showError((void*)ui_text_home, (void*)ui_text_operation_denied);

        return 0;
    }
#endif //FEATURE_ALIGN_EXTRUDERS

    if (g_nHeatBedScanStatus || g_nZOSScanStatus) {
        // do not allow homing while the heat bed scan is in progress
        if (Printer::debugErrors() && outputLog) {
            Com::printFLN(PSTR("G28: homing can not be performed while the heat bed scan is in progress"));
        }
        showError((void*)ui_text_home, (void*)ui_text_operation_denied);

        return 0;
    }

    if (!com) {
        // there is nothing more which we can check
        return 1;
    }

    // we allow the homing at the moment
    return 1;
} // isHomingAllowed

void showInvalidSyntax(unsigned int currentMCode) {
    if (Printer::debugErrors()) {
        Com::printF(PSTR("M"), (int)currentMCode);
        Com::printFLN(PSTR(": invalid syntax"));
    }
    return;

} // showInvalidSyntax

void addUInt32(char* pszString, uint32_t uNumber) {
    char szTemp[11];
    char* pszTemp = &szTemp[10];
    uint32_t uTemp;

    *pszTemp = '\0';
    do {
        uTemp = uNumber;
        uNumber /= 10;
        *--pszTemp = '0' + (uTemp - 10 * uNumber);
    } while (uNumber);

    strcpy(pszString, pszTemp);

} // addUInt32

void addFloat(char* pszString, float fNumber, uint8_t uDigits) {
    float fRounding = 0.5;
    uint32_t uNumber;
    float fRemainder;
    uint8_t uRemainder;
    uint8_t i;

    // add at the end of the string
    while (*pszString) {
        pszString++;
    }

    if (isnan(fNumber)) {
        strcpy(pszString, "NAN");
        return;
    }
    if (isinf(fNumber)) {
        strcpy(pszString, "INF");
        return;
    }

    // Handle negative numbers
    if (fNumber < 0.0) {
        *pszString = '-';
        pszString++;
        *pszString = '\0';
        fNumber = -fNumber;
    }

    for (i = 0; i < uDigits; i++) {
        fRounding /= 10.0;
    }

    fNumber += fRounding;

    uNumber = (uint32_t)fNumber;
    fRemainder = fNumber - (float)uNumber;
    addUInt32(pszString, uNumber);

    if (uDigits > 0) {
        while (*pszString) {
            pszString++;
        }

        *pszString = '.';
        pszString++;

        while (uDigits-- > 0) {
            fRemainder *= 10.0;
            uRemainder = (uint8_t)fRemainder;
            *pszString = '0' + uRemainder;
            pszString++;
            fRemainder -= uRemainder;
        }

        *pszString = '\0';
    }

} // addFloat

#if FEATURE_HEAT_BED_TEMP_COMPENSATION
float getHeatBedTemperatureOffset(float temperatureInCelsius) {
    const unsigned char setpointTemperatures[] = BED_SETPOINT_TEMPERATURES;
    const unsigned char measuredTemperatures[] = BED_MEASURED_TEMPERATURES;
    float deltaLow, deltaHigh;
    float temp;
    unsigned char i;

    if (temperatureInCelsius <= setpointTemperatures[0]) {
        // the specified temperature is below our known, measured values
        deltaLow = 0;
        deltaHigh = measuredTemperatures[0] - setpointTemperatures[0];

        temp = (deltaHigh - deltaLow) / float(setpointTemperatures[0]);
        temp *= temperatureInCelsius;
        return temp;
    }

    if (temperatureInCelsius >= setpointTemperatures[BED_TEMP_COMPENSATION_INDEX_MAX]) {
        // the specified temperature is above our known, measured values
        temp = measuredTemperatures[BED_TEMP_COMPENSATION_INDEX_MAX] - setpointTemperatures[BED_TEMP_COMPENSATION_INDEX_MAX];
        return temp;
    }

    for (i = 1; i <= BED_TEMP_COMPENSATION_INDEX_MAX; i++) {
        if (temperatureInCelsius < setpointTemperatures[i]) {
            deltaLow = measuredTemperatures[i - 1] - setpointTemperatures[i - 1];
            deltaHigh = measuredTemperatures[i] - setpointTemperatures[i];

            temp = (deltaHigh - deltaLow) / float(setpointTemperatures[i] - setpointTemperatures[i - 1]);
            temp *= (temperatureInCelsius - setpointTemperatures[i - 1]);
            return (temp + deltaLow);
        }
    }

    // we should never end up here
    return 0;

} // getHeatBedTemperatureOffset
#endif // FEATURE_HEAT_BED_TEMP_COMPENSATION

#if FEATURE_TYPE_EEPROM
void determineHardwareType(void) {
    unsigned short uTemp;

    Printer::wrongType = 0;

    // check the stored header format
    uTemp = readWord24C256(I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_HEADER_FORMAT);

    if (uTemp != TYPE_EEPROM_FORMAT) {
        // we could not read the header format or the header format is wrong
        writeWord24C256(I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_HEADER_FORMAT, TYPE_EEPROM_FORMAT);
        writeWord24C256(I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_BOARD_TYPE, MOTHERBOARD);

        uTemp = readWord24C256(I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_HEADER_FORMAT);

        if (uTemp != TYPE_EEPROM_FORMAT) {
            if (MOTHERBOARD == DEVICE_TYPE_RF1000) {
                // the RF1000 does not provide the type EEPROM, thus we should run at an RF1000 board when we end up here
                return;
            }

            // we end up here in case this firmware is for the RF2000, but the current board does not seem to be an RF2000 board
            Printer::wrongType = 1;
            return;
        }

        if (MOTHERBOARD == DEVICE_TYPE_RF1000) {
            // we end up here in case this firmware is for the RF1000, but the current board seems to be an RF2000 board
            Printer::wrongType = 1;
            return;
        }

        // when we end up here this firmware is for the RF2000 and the current board seems to be an RF2000 board
        return;
    }

    if (MOTHERBOARD == DEVICE_TYPE_RF1000) {
        // we end up here in case this firmware is for the RF1000, but the current board seems to be an RF2000 board
        Printer::wrongType = 1;
        return;
    }

    // when we end up here this firmware is for the RF2000 and the current board seems to be an RF2000 board
    if (readWord24C256(I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_BOARD_TYPE) != MOTHERBOARD) {
        writeWord24C256(I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_BOARD_TYPE, MOTHERBOARD);
    }

    return;

} // determineHardwareType

#endif // FEATURE_TYPE_EEPROM

void showIdle(void) {
    UI_STATUS(UI_TEXT_PRINTER_READY);
} // showIdle

void showError(const void* line2, const void* line3, const void* line4) {
    uid.messageLine1 = (const void*)ui_text_error;
    uid.messageLine2 = line2;
    uid.messageLine3 = line3;
    uid.messageLine4 = line4;

    uid.showMessage(true);
    return;

} // showError

void showWarning(const void* line2, const void* line3, const void* line4) {
    uid.messageLine1 = (const void*)ui_text_warning;
    uid.messageLine2 = line2;
    uid.messageLine3 = line3;
    uid.messageLine4 = line4;

    uid.showMessage(true);
    return;

} // showWarning

void showInformation(const void* line2, const void* line3, const void* line4) {
    uid.messageLine1 = (const void*)ui_text_information;
    uid.messageLine2 = line2;
    uid.messageLine3 = line3;
    uid.messageLine4 = line4;

    uid.showMessage(true);
    return;

} // showInformation

void showMyPage(const void* line1, const void* line2, const void* line3, const void* line4) {
    uid.messageLine1 = line1;
    uid.messageLine2 = line2;
    uid.messageLine3 = line3;
    uid.messageLine4 = line4;

    uid.showMessage(true);
    return;

} // showMyPage

void doEmergencyStop(char reason) {
    // block any further movement
    Printer::blockAll = 1;

    showError((void*)ui_text_emergency_stop);

    Com::printF(PSTR("doEmergencyStop(): block all"));
    if (reason == STOP_BECAUSE_OF_Z_MIN) {
        Com::printFLN(PSTR(" (Z-Min)"));
    } else if (reason == STOP_BECAUSE_OF_Z_BLOCK) {
        Com::printFLN(PSTR(" (Z-Block)"));
    }

    Printer::stopPrint(); //and tell usb senders to stop

    moveZ(int(Printer::axisStepsPerMM[Z_AXIS] * 5));

    // we are not going to perform any further operations until the restart of the firmware
    Printer::switchEverythingOff();
} // doEmergencyStop

void addLong(char* string, long value, char digits) {
    uint8_t dig = 0;
    uint8_t neg = 0;
    uint8_t col = strlen(string);
    char buf[13]; // Assumes 8-bit chars plus zero byte.
    char* str = &buf[12];

    if (value < 0) {
        neg = 1;
        value = -value;
        dig++;
    }

    buf[12] = 0;
    do {
        unsigned long m = value;
        value /= 10;
        char c = m - 10 * value;
        *--str = c + '0';
        dig++;
    } while (value);

    if (neg)
        string[col++] = '-';

    if (digits <= 11) {
        while (dig < digits) {
            *--str = ' ';
            dig++;
        }
    }

    while (*str) {
        string[col++] = *str;
        str++;
    }
    string[col] = 0;

} // addLong

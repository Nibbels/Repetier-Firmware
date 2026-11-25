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

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "HAL.h"

class Com {
public:
    FSTRINGVAR(tDebug)
    FSTRINGVAR(tFirmware)
    FSTRINGVAR(tOk)
    FSTRINGVAR(tNewline)
    FSTRINGVAR(tSemiColon)
    FSTRINGVAR(tNAN)
    FSTRINGVAR(tINF)
    FSTRINGVAR(tError)
    FSTRINGVAR(tFatal)
    FSTRINGVAR(tInfo)
    FSTRINGVAR(tWarning)
    FSTRINGVAR(tResend)
    FSTRINGVAR(tEcho)
    FSTRINGVAR(tOkSpace)
    FSTRINGVAR(tWrongChecksum)
    FSTRINGVAR(tFormatError)
    FSTRINGVAR(tMissingChecksum)
    FSTRINGVAR(tX)
    FSTRINGVAR(tY)
    FSTRINGVAR(tZ)
    FSTRINGVAR(tE)
    FSTRINGVAR(tF)
    FSTRINGVAR(tS)
    FSTRINGVAR(tP)
    FSTRINGVAR(tI)
    FSTRINGVAR(tJ)
    FSTRINGVAR(tR)
    FSTRINGVAR(tD)
    FSTRINGVAR(tC)
    FSTRINGVAR(tH)
    FSTRINGVAR(tA)
    FSTRINGVAR(tB)
    FSTRINGVAR(tK)
    FSTRINGVAR(tL)
    FSTRINGVAR(tO)
    FSTRINGVAR(tExpectedLine)
    FSTRINGVAR(tGot)
    FSTRINGVAR(tSkip)
    FSTRINGVAR(tBLK)
    FSTRINGVAR(tStart)
    FSTRINGVAR(tStartWatchdog)
    FSTRINGVAR(tPowerUp)
    FSTRINGVAR(tExternalReset)
    FSTRINGVAR(tBrownOut)
    FSTRINGVAR(tWatchdog)
    FSTRINGVAR(tSoftwareReset)
    FSTRINGVAR(tUnknownReset)
    FSTRINGVAR(tUnknownCommand)
    FSTRINGVAR(tFreeRAM)
    FSTRINGVAR(tXColon)
    FSTRINGVAR(tSlash)
    FSTRINGVAR(tSpaceSlash)
    FSTRINGVAR(tSpaceXColon)
    FSTRINGVAR(tSpaceYColon)
    FSTRINGVAR(tSpaceZColon)
    FSTRINGVAR(tSpaceEColon)
    FSTRINGVAR(tTColon)
    FSTRINGVAR(tSpaceBColon)
    FSTRINGVAR(tSpaceAtColon)
    FSTRINGVAR(tSpaceT)
    FSTRINGVAR(tSpaceRaw)
    FSTRINGVAR(tSpaceAt)
    FSTRINGVAR(tSpaceBAtColon)
    FSTRINGVAR(tColon)
    FSTRINGVAR(tSpeedMultiply)
    FSTRINGVAR(tFlowMultiply)
    FSTRINGVAR(tPrintingIsInProcessError)
    FSTRINGVAR(tFanspeed)
    FSTRINGVAR(tPrintedFilament)
    FSTRINGVAR(tPrintingTime)
    FSTRINGVAR(tSpacem)
    FSTRINGVAR(tSpaceDaysSpace)
    FSTRINGVAR(tSpaceHoursSpace)
    FSTRINGVAR(tSpaceMin)
    FSTRINGVAR(tComma)
    FSTRINGVAR(tSpace)
    FSTRINGVAR(tYColon)
    FSTRINGVAR(tZColon)
    FSTRINGVAR(tE0Colon)
    FSTRINGVAR(tE1Colon)
    FSTRINGVAR(tMS1MS2Pins)
    FSTRINGVAR(tSetOutputSpace)
    FSTRINGVAR(tSpaceToSpace)
    FSTRINGVAR(tSpaceChamber)
    FSTRINGVAR(tSpaceCAtColon)
    FSTRINGVAR(tHSpace)
    FSTRINGVAR(tLSpace)
    FSTRINGVAR(tSoftDash)
    FSTRINGVAR(tXMinColon)
    FSTRINGVAR(tXMaxColon)
    FSTRINGVAR(tYMinColon)
    FSTRINGVAR(tYMaxColon)
    FSTRINGVAR(tZMinColon)
    FSTRINGVAR(tZMaxColon)
    FSTRINGVAR(tXYJerkColon)
    FSTRINGVAR(tZJerkColon)
    FSTRINGVAR(tLinearStepsColon)
    FSTRINGVAR(tQuadraticStepsColon)
    FSTRINGVAR(tCommaSpeedEqual)
    FSTRINGVAR(tLinearLColon)
    FSTRINGVAR(tQuadraticKColon)
    FSTRINGVAR(tEEPROMUpdated)
    FSTRINGVAR(tscanHeatBed)
    FSTRINGVAR(tAutoMatrixLeveling)


    FSTRINGVAR(tTargetExtr)
    FSTRINGVAR(tTargetBedColon)
    FSTRINGVAR(tPIDAutotuneStart)
    FSTRINGVAR(tAPIDKu)
    FSTRINGVAR(tAPIDTu)
    FSTRINGVAR(tAPIDKp)
    FSTRINGVAR(tAPIDKi)
    FSTRINGVAR(tAPIDKd)
    FSTRINGVAR(tAPIDFailedHigh)
    FSTRINGVAR(tAPIDFailedTimeout)
    FSTRINGVAR(tAPIDFinished)
    FSTRINGVAR(tMTEMPColon)
    FSTRINGVAR(tHeatedBed)
    FSTRINGVAR(tExtruderSpace)
    FSTRINGVAR(tTempSensorDefect)
    FSTRINGVAR(tTempHeaterDefect)
    FSTRINGVAR(tTempSensorWorking)
    FSTRINGVAR(tThermalFaultUntilRestart)

    FSTRINGVAR(tWait)

    FSTRINGVAR(tConfigStoredEEPROM)
    FSTRINGVAR(tConfigLoadedEEPROM)
    FSTRINGVAR(tEPRConfigResetDefaults)
    FSTRINGVAR(tEPRProtocolChanged)
    FSTRINGVAR(tExtrDot)
    FSTRINGVAR(tEPR0)
    FSTRINGVAR(tEPR1)
    FSTRINGVAR(tEPR2)
    FSTRINGVAR(tEPR3)
    FSTRINGVAR(tEPRBaudrate)
    FSTRINGVAR(tEPRFilamentPrinted)
    FSTRINGVAR(tEPRPrinterActive)
    FSTRINGVAR(tEPRMaxInactiveTime)
    FSTRINGVAR(tEPRStopAfterInactivty)
    FSTRINGVAR(tEPRMaxXYJerk)
    FSTRINGVAR(tEPRXMaxLength)
    FSTRINGVAR(tEPRYMaxLength)
    FSTRINGVAR(tEPRZMaxLength)
    FSTRINGVAR(tEPRXBacklash)
    FSTRINGVAR(tEPRYBacklash)
    FSTRINGVAR(tEPRZBacklash)
    FSTRINGVAR(tEPRZAcceleration)
    FSTRINGVAR(tEPRZTravelAcceleration)
    FSTRINGVAR(tEPRZScanStartLift)
    FSTRINGVAR(tEPRZERO_DIGIT_STATE)
    FSTRINGVAR(tEPRZDIGIT_CMP_STATE)
    FSTRINGVAR(tEPRZStepsPerMM)
    FSTRINGVAR(tEPRZMaxFeedrate)
    FSTRINGVAR(tEPRZHomingFeedrate)
    FSTRINGVAR(tEPRMaxZJerk)
    FSTRINGVAR(tEPRXStepsPerMM)
    FSTRINGVAR(tEPRYStepsPerMM)
    FSTRINGVAR(tEPRXMaxFeedrate)
    FSTRINGVAR(tEPRYMaxFeedrate)
    FSTRINGVAR(tEPRXHomingFeedrate)
    FSTRINGVAR(tEPRYHomingFeedrate)
    FSTRINGVAR(tEPRXAcceleration)
    FSTRINGVAR(tEPRYAcceleration)
    FSTRINGVAR(tEPRXTravelAcceleration)
    FSTRINGVAR(tEPRYTravelAcceleration)
    FSTRINGVAR(tEPRBedPIDDriveMax)
    FSTRINGVAR(tEPRBedPIDDriveMin)
    FSTRINGVAR(tEPRBedPGain)
    FSTRINGVAR(tEPRBedIGain)
    FSTRINGVAR(tEPRBedDGain)
    FSTRINGVAR(tEPRBedPIDMaxValue)
    FSTRINGVAR(tEPRStepsPerMM)
    FSTRINGVAR(tEPRMaxFeedrate)
    FSTRINGVAR(tEPReJerk)
    FSTRINGVAR(tEPRAcceleration)
    FSTRINGVAR(tEPRDriveMax)
    FSTRINGVAR(tEPRDriveMin)
    FSTRINGVAR(tEPRPGain)
    FSTRINGVAR(tEPRIGain)
    FSTRINGVAR(tEPRDGain)
    FSTRINGVAR(tEPRPIDMaxValue)
    FSTRINGVAR(tEPRBedsensorType)
    FSTRINGVAR(tEPRsensorType)
    FSTRINGVAR(tEPRXOffset)
    FSTRINGVAR(tEPRYOffset)
    FSTRINGVAR(tEPRZOffset)
    FSTRINGVAR(tEPRZMode)
    FSTRINGVAR(tEPRExtruderCoolerSpeed)
    FSTRINGVAR(tEPRAdvanceK)
    FSTRINGVAR(tEPRAdvanceL)
    FSTRINGVAR(tEPRCaseLightsMode)
    FSTRINGVAR(tEPRRGBLightMode)
    FSTRINGVAR(tEPRFET1Mode)
    FSTRINGVAR(tEPRFET2Mode)
    FSTRINGVAR(tEPRFET3Mode)

    FSTRINGVAR(tEPRPrinterZ_STEP_SIZE)
    FSTRINGVAR(tEPRPrinterMOD_ZOS_SCAN_POINT_X)
    FSTRINGVAR(tEPRPrinterMOD_ZOS_SCAN_POINT_Y)
    FSTRINGVAR(tEPRPrinterMOD_SENSEOFFSET_OFFSET_MAX)
    FSTRINGVAR(tEPRPrinterEPR_RF_MOD_SENSEOFFSET_DIGITS)

    FSTRINGVAR(tEPRPrinterEPR_RF_EmergencyPauseDigitsMin)
    FSTRINGVAR(tEPRPrinterEPR_RF_EmergencyPauseDigitsMax)

    FSTRINGVAR(tEPRPrinterEPR_RF_EmergencyStopAllMin)
    FSTRINGVAR(tEPRPrinterEPR_RF_EmergencyStopAllMax)

    FSTRINGVAR(tEPRPrinter_STEPPER_X)
    FSTRINGVAR(tEPRPrinter_STEPPER_Y)
    FSTRINGVAR(tEPRPrinter_STEPPER_Z)
    FSTRINGVAR(tEPRPrinter_STEPPER_E0)
#if NUM_EXTRUDER > 1
    FSTRINGVAR(tEPRPrinter_STEPPER_E1)
#endif //NUM_EXTRUDER > 1
    FSTRINGVAR(tEPRInterruptSpacingInterval)

    FSTRINGVAR(tEPRPrinter_FAN_MODE)
    FSTRINGVAR(tEPRPrinter_FAN_SPEED)
    FSTRINGVAR(tEPRPrinter_FAN_PART_FAN_PWM_MIN)
    FSTRINGVAR(tEPRPrinter_FAN_PART_FAN_PWM_MAX)

    FSTRINGVAR(tHeaterDecoupledWarning)
    FSTRINGVAR(tOutputObjectPrint)
    FSTRINGVAR(tUnmountFilamentSoft)
    FSTRINGVAR(tUnmountFilamentHard)
    FSTRINGVAR(tMountFilamentSoft)
    FSTRINGVAR(tMountFilamentHard)

    FSTRINGVAR(tCap)
    FSTRINGVAR(tConfig)

    static void cap(FSTRINGPARAM(text));
    static void config(FSTRINGPARAM(text));
    static void config(FSTRINGPARAM(text), int value);
    static void config(FSTRINGPARAM(text), const char* msg);
    static void config(FSTRINGPARAM(text), int32_t value);
    static void config(FSTRINGPARAM(text), uint32_t value);
    static void config(FSTRINGPARAM(text), float value, uint8_t digits = 2);
    static void printNumber(uint32_t n);
    static void printWarningF(FSTRINGPARAM(text));
    static void printInfoF(FSTRINGPARAM(text));
    static void printErrorF(FSTRINGPARAM(text));
    static void printWarningFLN(FSTRINGPARAM(text));
    static void printInfoFLN(FSTRINGPARAM(text));
    static void printErrorFLN(FSTRINGPARAM(text));
    static void printFLN(FSTRINGPARAM(text));
    static void printF(FSTRINGPARAM(ptr));
    static void printF(FSTRINGPARAM(text), int value);
    static void printF(FSTRINGPARAM(text), const char* msg);
    static void printF(FSTRINGPARAM(text), int32_t value);
    static void printF(FSTRINGPARAM(text), uint32_t value);
    static void printF(FSTRINGPARAM(text), float value, uint8_t digits = 2, bool komma_as_dot = false);
    static void printFLN(FSTRINGPARAM(text), int value);
    static void printFLN(FSTRINGPARAM(text), int32_t value);
    static void printFLN(FSTRINGPARAM(text), uint32_t value);
    static void printFLN(FSTRINGPARAM(text), const char* msg);
    static void printFLN(FSTRINGPARAM(text), float value, uint8_t digits = 2, bool komma_as_dot = false);
    static void printArrayFLN(FSTRINGPARAM(text), float* arr, uint8_t n = 4, uint8_t digits = 2);
    static void printArrayFLN(FSTRINGPARAM(text), int32_t* arr, uint8_t n = 4);
    static void printSharpLine();
    static void print(int32_t value);
    static inline void print(uint32_t value) { printNumber(value); }
    static inline void print(int value) { print((int32_t)value); }
    static void print(const char* text);
    static inline void print(char c) {
        HAL::serialWriteByte(c);
    }
    static void printFloat(float number, uint8_t digits, bool komma_as_dot = false);
    static inline void println() {
        HAL::serialWriteByte('\r');
        HAL::serialWriteByte('\n');
    }
}; // Com

#endif // COMMUNICATION_H

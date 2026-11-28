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

#ifndef EEPROM_H
#define EEPROM_H

// Semantic internal EEPROM owner. Physical addresses and field widths
// reside exclusively in EepromZoneLayout.h; the lower bank is unowned.
#include <stdint.h>
struct GCode;

class EEPROM {
public:
    static void init();
    // Settings recovery and local heater commissioning.
    static bool settingsReady();
    static bool recoveryPending();
    static bool acknowledgeRecovery();
    static bool commissionBedCutoff(uint8_t cutoff);
    static bool lastOperationSucceeded();
    static void reportRecovery();
    static const void* recoveryDialogTag();
    // Persist selected pressure limits, preserving the unselected saved
    // value (not necessarily the temporary live stop band).
    static bool storePressureLimits(bool stop, int32_t minimum, int32_t maximum, bool saveMinimum, bool saveMaximum);
    static void storeDriveLimit(uint8_t heater, bool maximum, uint8_t value);
    // Semantic persistence entrypoints. Callers never choose addresses.
    static void storeCaseLight(bool on);
    static void storeFet(uint8_t channel,bool on);
    static void storeJogMode(uint8_t axis,uint8_t mode);
    static void storeZJogSteps(uint16_t steps);
    static void storeJogCoordinates(bool gcode);
    static void storeJogFeedrate(bool configured);
    static void storeZDisplayMode(uint8_t mode);
    static void storeFanMode(uint8_t mode);
    static void storeDigitZeroing(bool enabled);
    static void storeDigitCompensation(bool enabled);
    static void storeSenseAutoStart(bool enabled);
    static void storeScanPoint(uint8_t x,uint8_t y,bool saveX,bool saveY);
    static void storeFanDivisor(uint8_t divisor);
    static void storeFanDuty(bool maximum,uint8_t duty);
    static void storeToolOffsets(uint8_t tool,float x,float y,bool saveX,bool saveY);
    static void storeInactivity(bool stepper,uint32_t duration);
    static void storeScanLift(float lift);
    static void storeWobblePhase(int8_t phase);
    static void storeWobbleAmplitude(uint8_t component,int16_t amplitude);
    static void storeMotorCurrent(uint8_t axis,uint8_t current);
    static void storePackingInterval(uint16_t interval);
    static void storeAxisAcceleration(uint8_t axis,bool travel,float acceleration);
    static void storeAxisJerk(bool z,float jerk);
    static void storeAxisFeedrate(uint8_t axis,bool homing,float feedrate);
    enum class ToolMotionSetting : uint8_t { Steps, Advance, Acceleration, Feedrate, Jerk };
    static void storeToolMotion(uint8_t tool,ToolMotionSetting setting,float value);
    static void storeManualZOffset();
    static void storeSenseOffsetSettings(int16_t digits, int16_t micrometers, bool saveDigits, bool saveMaximum);
    static void storeMicrostepConfiguration(uint8_t changedMask);
    static void storePreheatProfiles();
    static void storePidCalibration(uint8_t heater);
    static void restoreEEPROMSettingsFromConfiguration();
    static void writeSettings();
    static void update(GCode* com);
}; // EEPROM

#endif // EEPROM_H

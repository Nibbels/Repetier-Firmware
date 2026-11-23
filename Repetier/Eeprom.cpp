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

// Persisted signature for the existing internal EEPROM layout.
constexpr uint8_t eepromMagic = 44;

void EEPROM::update(GCode* com) {
    if (com->hasT() && com->hasP())
        switch (com->T) {
        case 0: {
            if (com->hasS())
                HAL::eprSetByte(com->P, (uint8_t)com->S);
            break;
        }
        case 1: {
            if (com->hasS())
                HAL::eprSetInt16(com->P, (int)com->S);
            break;
        }
        case 2: {
            if (com->hasS())
                HAL::eprSetInt32(com->P, (int32_t)com->S);
            break;
        }
        case 3: {
            if (com->hasX())
                HAL::eprSetFloat(com->P, com->X);
            break;
        }
        }
    EEPROM::updateChecksum();

    readDataFromEEPROM();

} // update

void EEPROM::restoreEEPROMExtruderSettingsFromConfiguration(uint8_t extruderId) {
    Extruder* e;

    if (extruderId == 0) {
        e = &extruder[0];
        e->stepsPerMM = EXT0_STEPS_PER_MM;
        e->maxFeedrate = EXT0_MAX_FEEDRATE;
        e->maxEJerk = EXT0_MAX_START_FEEDRATE;
        e->maxAcceleration = EXT0_MAX_ACCELERATION;

        e->tempControl.pidDriveMax = EXT0_HOTEND_PROFILE.pidDriveMax;
        e->tempControl.pidDriveMin = EXT0_HOTEND_PROFILE.pidDriveMin;
        e->tempControl.pidPGain = EXT0_HOTEND_PROFILE.pidPGain;
        e->tempControl.pidIGain = EXT0_HOTEND_PROFILE.pidIGain;
        e->tempControl.pidDGain = EXT0_HOTEND_PROFILE.pidDGain;
        e->tempControl.pidMax = EXT0_HOTEND_PROFILE.pidMax;
        e->tempControl.sensorType = EXT0_TEMPSENSOR_TYPE;

        e->offsetMM[X_AXIS] = EXT0_X_OFFSET_MM;
        e->offsetMM[Y_AXIS] = EXT0_Y_OFFSET_MM;


        e->coolerSpeed = EXT0_EXTRUDER_COOLER_SPEED;
        e->advanceL = EXT0_ADVANCE_L;
    }

#if NUM_EXTRUDER > 1
    if (extruderId == 1) {
        e = &extruder[1];
        e->stepsPerMM = EXT1_STEPS_PER_MM;
        e->maxFeedrate = EXT1_MAX_FEEDRATE;
        e->maxEJerk = EXT1_MAX_START_FEEDRATE;
        e->maxAcceleration = EXT1_MAX_ACCELERATION;

        e->tempControl.pidDriveMax = EXT1_HOTEND_PROFILE.pidDriveMax;
        e->tempControl.pidDriveMin = EXT1_HOTEND_PROFILE.pidDriveMin;
        e->tempControl.pidPGain = EXT1_HOTEND_PROFILE.pidPGain;
        e->tempControl.pidIGain = EXT1_HOTEND_PROFILE.pidIGain;
        e->tempControl.pidDGain = EXT1_HOTEND_PROFILE.pidDGain;
        e->tempControl.pidMax = EXT1_HOTEND_PROFILE.pidMax;
        e->tempControl.sensorType = EXT1_TEMPSENSOR_TYPE;

        e->offsetMM[X_AXIS] = EXT1_X_OFFSET_MM;
        e->offsetMM[Y_AXIS] = EXT1_Y_OFFSET_MM;


        e->coolerSpeed = EXT1_EXTRUDER_COOLER_SPEED;

        e->advanceL = EXT1_ADVANCE_L;
    }
#endif // NUM_EXTRUDER > 1
}

void EEPROM::restoreEEPROMSettingsFromConfiguration() {
    baudrate = BAUDRATE;
    Printer::inactivity.setShutdownTimeout(MAX_INACTIVE_TIME * 1000L);
    Printer::inactivity.setStepperTimeout(STEPPER_INACTIVE_TIME * 1000L);
    Printer::axisStepsPerMM[X_AXIS] = XAXIS_STEPS_PER_MM;
    Printer::axisStepsPerMM[Y_AXIS] = YAXIS_STEPS_PER_MM;
    Printer::axisStepsPerMM[Z_AXIS] = ZAXIS_STEPS_PER_MM;
    Printer::zOffset.refreshSteps(Printer::axisStepsPerMM[Z_AXIS]);
    Printer::axisStepsPerMM[E_AXIS] = 1; //man könnte auch vom current extruder die id auslesen und dann EXT0_STEPS_PER_MM oder EXT1_STEPS_PER_MM ?
                                         // -> ist autokorrigiert wenn man einmal einen extruder auswählt. Siehe unten.
    Printer::maxFeedrate[X_AXIS] = MAX_FEEDRATE_X;
    Printer::maxFeedrate[Y_AXIS] = MAX_FEEDRATE_Y;
    Printer::maxFeedrate[Z_AXIS] = MAX_FEEDRATE_Z;

    //microsteps should be updated but we need to update the driver then.
    bool ms_changed = false;
    for (int ax = 0; ax <= E_AXIS + 1; ax++) {
        if (Printer::motorMicroStepsModeValue[ax] != drv8711MicroSteps_2_ModeValue(drv8711Axis_2_InitMicrosteps(ax))) {
            if (!ms_changed) {
                ms_changed = true;
                Printer::disableAllSteppersNow();               //Stepper und Homing ausmachen.
                                                                //We cannot use the old coordinates anymore.
                HAL::eprSetByte(EPR_RF_MICRO_STEPS_USED, 0x00); //make all Microstep eeprom settings invalid for next boot.
            }
            Printer::motorMicroStepsModeValue[ax] = drv8711MicroSteps_2_ModeValue(drv8711Axis_2_InitMicrosteps(ax));
            drv8711adjustMicroSteps(ax + 1); //adjust driver chip X=1, Y=2 .. ,E1=5 according to motorMicroStepsModeValue[]
        }
    }

        Printer::homingFeedrate[X_AXIS] = HOMING_FEEDRATE_X_PRINT;
        Printer::homingFeedrate[Y_AXIS] = HOMING_FEEDRATE_Y_PRINT;
        Printer::homingFeedrate[Z_AXIS] = HOMING_FEEDRATE_Z_PRINT;

    Printer::maxXYJerk = MAX_JERK;
    Printer::maxZJerk = MAX_ZJERK;

    Printer::moveMode[X_AXIS] = DEFAULT_MOVE_MODE_X;
    Printer::moveMode[Y_AXIS] = DEFAULT_MOVE_MODE_Y;
    Printer::moveMode[Z_AXIS] = DEFAULT_MOVE_MODE_Z;
    Printer::moveKosys = KOSYS_GCODE;
    Printer::movePositionFeedrateChoice = FEEDRATE_DIRECTCONFIG;

    Printer::maxAccelerationMMPerSquareSecond[X_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X;
    Printer::maxAccelerationMMPerSquareSecond[Y_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y;
    Printer::maxAccelerationMMPerSquareSecond[Z_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z;
    Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS] = MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X;
    Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS] = MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y;
    Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS] = MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z;

    heatedBedController.pidDriveMax = HEATED_BED_PID_INTEGRAL_DRIVE_MAX;
    heatedBedController.pidDriveMin = HEATED_BED_PID_INTEGRAL_DRIVE_MIN;
    heatedBedController.pidPGain = HEATED_BED_PID_PGAIN;
    heatedBedController.pidIGain = HEATED_BED_PID_IGAIN;
    heatedBedController.pidDGain = HEATED_BED_PID_DGAIN;
    heatedBedController.pidMax = HEATED_BED_PID_MAX;
    heatedBedController.sensorType = HEATED_BED_SENSOR_TYPE;

        Printer::axisLengthMM[X_AXIS] = X_MAX_LENGTH_PRINT;

    Printer::axisLengthMM[Y_AXIS] = Y_MAX_LENGTH;
    Printer::axisLengthMM[Z_AXIS] = Z_MAX_LENGTH;

    EEPROM::restoreEEPROMExtruderSettingsFromConfiguration(0);
#if NUM_EXTRUDER > 1
    EEPROM::restoreEEPROMExtruderSettingsFromConfiguration(1);
#endif // NUM_EXTRUDER > 1

    /* TODO : Restliche Parameter neu einlesen. ....
    ich glaube gesehen zu haben, dass acceleration und feedrates nicht neu eingelesen werden. will man das?
    */

    Printer::stepsPackingMinInterval = STEP_PACKING_MIN_INTERVAL;

    Printer::ZMode = DEFAULT_Z_SCALE_MODE; //wichtig, weils im Mod einen dritten Mode gibt. Für Zurückmigration

#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    //hier nur config laden nicht mosfets schalten!
    Printer::enable230VOutput = 0;

    //hier nur config laden nicht mosfets schalten!
    Printer::enableFET1 = FET1_DEFAULT_ON;
    Printer::enableFET2 = FET2_DEFAULT_ON;
    Printer::enableFET3 = FET3_DEFAULT_ON;
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2

    g_ZOSTestPoint[X_AXIS] = defaultZOffsetScanIndex;
    g_ZOSTestPoint[Y_AXIS] = defaultZOffsetScanIndex;

    Printer::senseOffsetControl.setMaxMicrometers(SENSIBLE_PRESSURE_MAX_OFFSET);
    Printer::senseOffsetControl.setAutoStart(false);

    Printer::wobbleCorrection.setProfile({});

    Printer::pressureProtection.setPauseLimits({EMERGENCY_PAUSE_DIGITS_MIN, EMERGENCY_PAUSE_DIGITS_MAX});
    Printer::pressureProtection.setStopLimits({EMERGENCY_STOP_DIGITS_MIN, EMERGENCY_STOP_DIGITS_MAX});

#ifdef RGB_LIGHT_R_PIN
    Printer::RGBLightMode = RGB_MODE_AUTOMATIC;
    Printer::RGBLightStatus = RGB_STATUS_AUTOMATIC;
    Printer::RGBLightIdleStart = 0;
#endif // RGB_LIGHT_R_PIN

    const unsigned short uMotorCurrentUse[] = MOTOR_CURRENT_NORMAL; //Standardwert
    for (uint8_t stp = 0; stp < 3 + NUM_EXTRUDER; stp++) {          //0..4 bei 5 steppern.
        Printer::motorCurrent[stp] = uMotorCurrentUse[stp];
        //setMotorCurrent( stp+1, uMotorCurrentUse[stp] ); //driver ist 1-basiert //hier nur config laden
    }

    Printer::pause.restoreDefaultParking();

    Printer::updateDerivedParameter();
    Extruder::applyCurrentSettings();
    Printer::applyAdvanceSettings();
    Extruder::initHeatedBed();

    g_nManualSteps[X_AXIS] = (unsigned short)lroundf(Printer::axisStepsPerMM[X_AXIS] * DEFAULT_MANUAL_MM_X);
    g_nManualSteps[Y_AXIS] = (unsigned short)lroundf(Printer::axisStepsPerMM[Y_AXIS] * DEFAULT_MANUAL_MM_Y);
    g_nManualSteps[Z_AXIS] = (unsigned short)lroundf(Printer::axisStepsPerMM[Z_AXIS] * DEFAULT_MANUAL_MM_Z);
    g_nManualSteps[E_AXIS] = (unsigned short)lroundf(Extruder::current->stepsPerMM * DEFAULT_MANUAL_MM_E);

    Com::printInfoFLN(Com::tEPRConfigResetDefaults);

} // restoreEEPROMSettingsFromConfiguration

void EEPROM::clearEEPROM() {
    millis_t lastTime = HAL::timeInMilliseconds();
    millis_t currentTime;

    for (int i = 0; i < 2048; i++) {
        HAL::eprSetByte(i, 0);

        currentTime = HAL::timeInMilliseconds();
        if ((currentTime - lastTime) > PERIODICAL_ACTIONS_CALL_INTERVAL) {
            Commands::checkForPeriodicalActions(Processing);
            lastTime = currentTime;
        }
    }

} // clearEEPROM

void EEPROM::storeDataIntoEEPROM(uint8_t corrupted) {
    HAL::eprSetByte(EPR_MAGIC_BYTE, eepromMagic);
    HAL::eprSetByte(EPR_CHECK_NUM_EXTRUDERS, NUM_EXTRUDER);
    HAL::eprSetInt32(EPR_BAUDRATE, baudrate);
    HAL::eprSetInt32(EPR_MAX_INACTIVE_TIME, Printer::inactivity.shutdownTimeout());
    HAL::eprSetInt32(EPR_STEPPER_INACTIVE_TIME, Printer::inactivity.stepperTimeout());
    HAL::eprSetFloat(EPR_XAXIS_STEPS_PER_MM, Printer::axisStepsPerMM[X_AXIS]);
    HAL::eprSetFloat(EPR_YAXIS_STEPS_PER_MM, Printer::axisStepsPerMM[Y_AXIS]);
    HAL::eprSetFloat(EPR_ZAXIS_STEPS_PER_MM, Printer::axisStepsPerMM[Z_AXIS]);
    HAL::eprSetFloat(EPR_X_MAX_FEEDRATE, Printer::maxFeedrate[X_AXIS]);
    HAL::eprSetFloat(EPR_Y_MAX_FEEDRATE, Printer::maxFeedrate[Y_AXIS]);
    HAL::eprSetFloat(EPR_Z_MAX_FEEDRATE, Printer::maxFeedrate[Z_AXIS]);
    HAL::eprSetInt32(EPR_RF_Z_OFFSET, Printer::zOffset.manualMicrometers());
    HAL::eprSetByte(EPR_RF_Z_MODE, Printer::ZMode);
    HAL::eprSetByte(EPR_RF_MOVE_MODE_X, Printer::moveMode[X_AXIS]);
    HAL::eprSetByte(EPR_RF_MOVE_MODE_Y, Printer::moveMode[Y_AXIS]);
    HAL::eprSetByte(EPR_RF_MOVE_MODE_Z, Printer::moveMode[Z_AXIS]);
    HAL::eprSetByte(EPR_RF_MOVE_MODE_XY_KOSYS, Printer::moveKosys);
    HAL::eprSetByte(EPR_RF_MOVE_POSITION_FEEDRATE, Printer::movePositionFeedrateChoice);

    HAL::eprSetFloat(EPR_X_HOMING_FEEDRATE_PRINT, Printer::homingFeedrate[X_AXIS]);
    HAL::eprSetFloat(EPR_Y_HOMING_FEEDRATE_PRINT, Printer::homingFeedrate[Y_AXIS]);
    HAL::eprSetFloat(EPR_Z_HOMING_FEEDRATE_PRINT, Printer::homingFeedrate[Z_AXIS]);

    HAL::eprSetFloat(EPR_MAX_XYJERK, Printer::maxXYJerk);
    HAL::eprSetFloat(EPR_MAX_ZJERK, Printer::maxZJerk);

    HAL::eprSetFloat(EPR_X_MAX_ACCEL, Printer::maxAccelerationMMPerSquareSecond[X_AXIS]);
    HAL::eprSetFloat(EPR_Y_MAX_ACCEL, Printer::maxAccelerationMMPerSquareSecond[Y_AXIS]);
    HAL::eprSetFloat(EPR_Z_MAX_ACCEL, Printer::maxAccelerationMMPerSquareSecond[Z_AXIS]);
    HAL::eprSetFloat(EPR_X_MAX_TRAVEL_ACCEL, Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS]);
    HAL::eprSetFloat(EPR_Y_MAX_TRAVEL_ACCEL, Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS]);
    HAL::eprSetFloat(EPR_Z_MAX_TRAVEL_ACCEL, Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS]);

    HAL::eprSetByte(EPR_BED_DRIVE_MAX, heatedBedController.pidDriveMax);
    HAL::eprSetByte(EPR_BED_DRIVE_MIN, heatedBedController.pidDriveMin);
    HAL::eprSetFloat(EPR_BED_PID_PGAIN, heatedBedController.pidPGain);
    HAL::eprSetFloat(EPR_BED_PID_IGAIN, heatedBedController.pidIGain);
    HAL::eprSetFloat(EPR_BED_PID_DGAIN, heatedBedController.pidDGain);
    HAL::eprSetByte(EPR_BED_PID_MAX, heatedBedController.pidMax);
    HAL::eprSetByte(EPR_RF_HEATED_BED_SENSOR_TYPE, heatedBedController.sensorType);

        HAL::eprSetFloat(EPR_X_LENGTH, Printer::axisLengthMM[X_AXIS]);
    HAL::eprSetFloat(EPR_Y_LENGTH, Printer::axisLengthMM[Y_AXIS]);
    HAL::eprSetFloat(EPR_Z_LENGTH, Printer::axisLengthMM[Z_AXIS]);

    // now the extruder
    for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
        storeExtruderDataIntoEEPROM(i);
    }
    HAL::eprSetInt16(EPR_RF_STEP_PACKING_MIN_INTERVAL, Printer::stepsPackingMinInterval);

    HAL::eprSetByte(EPR_RF_FAN_MODE, part_fan_frequency_modulation);

    HAL::eprSetByte(EPR_RF_PART_FAN_SPEED, part_fan_pwm_speed);
    HAL::eprSetByte(EPR_RF_PART_FAN_PWM_MIN, part_fan_pwm_min);
    HAL::eprSetByte(EPR_RF_PART_FAN_PWM_MAX, part_fan_pwm_max);

    if (corrupted) {
        HAL::eprSetInt32(EPR_PRINTING_TIME, 0);
        HAL::eprSetFloat(EPR_PRINTING_DISTANCE, 0);
    }

    HAL::eprSetByte(EPR_RF_CASE_LIGHT_MODE, Printer::enableCaseLight);

#ifdef RGB_LIGHT_R_PIN
    HAL::eprSetByte(EPR_RF_RGB_LIGHT_MODE, Printer::RGBLightMode);
#endif // RGB_LIGHT_R_PIN

#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    HAL::eprSetByte(EPR_RF_FET1_MODE, Printer::enableFET1);
    HAL::eprSetByte(EPR_RF_FET2_MODE, Printer::enableFET2);
    HAL::eprSetByte(EPR_RF_FET3_MODE, Printer::enableFET3);

    // after a power-on, the 230 V plug always shall be turned off - thus, we do not store this setting to the EEPROM
    // HAL::eprSetByte( EPR_RF_230V_OUTPUT_MODE, Printer::enable230VOutput );
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2

    HAL::eprSetInt16(EPR_RF_MOD_Z_STEP_SIZE, g_nManualSteps[Z_AXIS]);
    HAL::eprSetByte(EPR_RF_MOD_ZOS_SCAN_POINT_X, g_ZOSTestPoint[0]);
    HAL::eprSetByte(EPR_RF_MOD_ZOS_SCAN_POINT_Y, g_ZOSTestPoint[1]);
    //Do not update EPR_RF_MOD_SENSEOFFSET_DIGITS here
    HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX, Printer::senseOffsetControl.limits().maxMicrometers);
    HAL::eprSetByte(EPR_RF_MOD_SENSEOFFSET_AUTOSTART, Printer::senseOffsetControl.autoStart());

    const auto& wobble = Printer::wobbleCorrection.profile();
    HAL::eprSetByte(EPR_RF_MOD_WOBBLE_FIX_PHASEXY, wobble.phase);
    HAL::eprSetInt16(EPR_RF_MOD_WOBBLE_FIX_AMPX, wobble.xMicrometers);
    HAL::eprSetInt16(EPR_RF_MOD_WOBBLE_FIX_AMPY1, wobble.yAtXMinMicrometers);
    HAL::eprSetInt16(EPR_RF_MOD_WOBBLE_FIX_AMPY2, wobble.yAtXMaxMicrometers);

    const auto pauseLimits = Printer::pressureProtection.pauseLimits();
    HAL::eprSetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMIN, pauseLimits.minimum);
    HAL::eprSetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMAX, pauseLimits.maximum);

    const auto stopLimits = Printer::pressureProtection.stopLimits();
    HAL::eprSetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMIN, stopLimits.minimum);
    HAL::eprSetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMAX, stopLimits.maximum);

    HAL::eprSetByte(EPR_RF_MOTOR_CURRENT + X_AXIS, Printer::motorCurrent[X_AXIS]);
    HAL::eprSetByte(EPR_RF_MOTOR_CURRENT + Y_AXIS, Printer::motorCurrent[Y_AXIS]);
    HAL::eprSetByte(EPR_RF_MOTOR_CURRENT + Z_AXIS, Printer::motorCurrent[Z_AXIS]);
    HAL::eprSetByte(EPR_RF_MOTOR_CURRENT + E_AXIS + 0, Printer::motorCurrent[E_AXIS + 0]);
#if NUM_EXTRUDER > 1
    HAL::eprSetByte(EPR_RF_MOTOR_CURRENT + E_AXIS + 1, Printer::motorCurrent[E_AXIS + 1]);
#endif //NUM_EXTRUDER > 1

    HAL::eprSetByte(EPR_RF_ZERO_DIGIT_STATE, Printer::strainGauge.zeroingEnabled() ? 1 : 2);
    HAL::eprSetByte(EPR_RF_DIGIT_CMP_STATE, Printer::digitCompensation.digitZEnabled() ? 1 : 2);

    HAL::eprSetFloat(EPR_ZSCAN_START_MM, g_scanStartZLiftMM);

    // Save version and build checksum
    HAL::eprSetByte(EPR_VERSION, EEPROM_PROTOCOL_VERSION);
    EEPROM::updateChecksum();

} // storeDataIntoEEPROM

void EEPROM::storeExtruderDataIntoEEPROM(uint8_t extruderId) {
    int o = EEPROM::getExtruderOffset(extruderId);
    Extruder* e = &extruder[extruderId];
    HAL::eprSetFloat(o + EPR_EXTRUDER_STEPS_PER_MM, e->stepsPerMM);
    HAL::eprSetFloat(o + EPR_EXTRUDER_MAX_FEEDRATE, e->maxFeedrate);
    HAL::eprSetFloat(o + EPR_EXTRUDER_MAX_START_FEEDRATE, e->maxEJerk);
    HAL::eprSetFloat(o + EPR_EXTRUDER_MAX_ACCELERATION, e->maxAcceleration);

    HAL::eprSetByte(o + EPR_EXTRUDER_DRIVE_MAX, e->tempControl.pidDriveMax);
    HAL::eprSetByte(o + EPR_EXTRUDER_DRIVE_MIN, e->tempControl.pidDriveMin);
    HAL::eprSetFloat(o + EPR_EXTRUDER_PID_PGAIN, e->tempControl.pidPGain);
    HAL::eprSetFloat(o + EPR_EXTRUDER_PID_IGAIN, e->tempControl.pidIGain);
    HAL::eprSetFloat(o + EPR_EXTRUDER_PID_DGAIN, e->tempControl.pidDGain);
    HAL::eprSetByte(o + EPR_EXTRUDER_PID_MAX, e->tempControl.pidMax);
    HAL::eprSetByte(o + EPR_EXTRUDER_SENSOR_TYPE, e->tempControl.sensorType);

    HAL::eprSetFloat(o + EPR_EXTRUDER_X_OFFSET, e->offsetMM[X_AXIS]);
    HAL::eprSetFloat(o + EPR_EXTRUDER_Y_OFFSET, e->offsetMM[Y_AXIS]);


    HAL::eprSetByte(o + EPR_EXTRUDER_COOLER_SPEED, e->coolerSpeed);

    HAL::eprSetFloat(o + EPR_EXTRUDER_ADVANCE_L, e->advanceL);
}

void EEPROM::updateChecksum() {
    //ändert sich die checksumme nicht, wird nicht geschrieben. doppelprüfung ist überflüssig.
    HAL::eprSetByte(EPR_INTEGRITY_BYTE, computeChecksum());
} // updateChecksum

void EEPROM::storeManualZOffset() {
    const int32_t manual = Printer::zOffset.manualMicrometers();
    if (HAL::eprGetInt32(EPR_RF_Z_OFFSET) != manual) {
        HAL::eprSetInt32(EPR_RF_Z_OFFSET, manual);
        updateChecksum();
    }
}

void EEPROM::readDataFromEEPROM() {
    bool change = false;

    uint8_t version = HAL::eprGetByte(EPR_VERSION); // This is the saved version. Don't copy data not set in older versions!
    baudrate = HAL::eprGetInt32(EPR_BAUDRATE);
    Printer::inactivity.setShutdownTimeout(HAL::eprGetInt32(EPR_MAX_INACTIVE_TIME));
    Printer::inactivity.setStepperTimeout(HAL::eprGetInt32(EPR_STEPPER_INACTIVE_TIME));
    Printer::axisStepsPerMM[X_AXIS] = HAL::eprGetFloat(EPR_XAXIS_STEPS_PER_MM);
    Printer::axisStepsPerMM[Y_AXIS] = HAL::eprGetFloat(EPR_YAXIS_STEPS_PER_MM);
    Printer::axisStepsPerMM[Z_AXIS] = HAL::eprGetFloat(EPR_ZAXIS_STEPS_PER_MM);

    //check da früher mehr speed erlaubt, was keinen sinn gemacht hat.
    Printer::maxFeedrate[X_AXIS] = MAX_FEEDRATE_X;
    Printer::maxFeedrate[Y_AXIS] = MAX_FEEDRATE_Y;
    Printer::maxFeedrate[Z_AXIS] = MAX_FEEDRATE_Z;
    for (uint8_t axis = X_AXIS; axis <= Z_AXIS; axis++) {
        float tmp = HAL::eprGetFloat(EPR_X_MAX_FEEDRATE + axis * 4); //X dann EPR_Y_MAX_FEEDRATE dann EPR_Z_MAX_FEEDRATE
        if (tmp > Printer::maxFeedrate[axis]) {
            HAL::eprSetFloat(EPR_X_MAX_FEEDRATE + axis * 4, Printer::maxFeedrate[axis]);
            change = true; //update checksum later in this function
        } else {
            Printer::maxFeedrate[axis] = tmp;
        }
    }

    // Reload the saved value in the current scale and keep the live SenseOffset.
    Printer::zOffset.setManualMicrometers(HAL::eprGetInt32(EPR_RF_Z_OFFSET), Printer::axisStepsPerMM[Z_AXIS]);
    Printer::ZMode = HAL::eprGetByte(EPR_RF_Z_MODE);

    Printer::layers.resetFadeBounds(Printer::axisStepsPerMM[Z_AXIS]);

    Printer::moveMode[X_AXIS] = HAL::eprGetByte(EPR_RF_MOVE_MODE_X);
    Printer::moveMode[Y_AXIS] = HAL::eprGetByte(EPR_RF_MOVE_MODE_Y);
    Printer::moveMode[Z_AXIS] = HAL::eprGetByte(EPR_RF_MOVE_MODE_Z);
    Printer::moveKosys = (bool)HAL::eprGetByte(EPR_RF_MOVE_MODE_XY_KOSYS);
    Printer::movePositionFeedrateChoice = (bool)HAL::eprGetByte(EPR_RF_MOVE_POSITION_FEEDRATE);

    Printer::maxXYJerk = HAL::eprGetFloat(EPR_MAX_XYJERK);
    Printer::maxZJerk = HAL::eprGetFloat(EPR_MAX_ZJERK);

    Printer::maxAccelerationMMPerSquareSecond[X_AXIS] = HAL::eprGetFloat(EPR_X_MAX_ACCEL);
    Printer::maxAccelerationMMPerSquareSecond[Y_AXIS] = HAL::eprGetFloat(EPR_Y_MAX_ACCEL);
    Printer::maxAccelerationMMPerSquareSecond[Z_AXIS] = HAL::eprGetFloat(EPR_Z_MAX_ACCEL);
    if (Printer::maxAccelerationMMPerSquareSecond[X_AXIS] < ACCELERATION_MIN_XY || Printer::maxAccelerationMMPerSquareSecond[X_AXIS] > ACCELERATION_MAX_XY) {
        Printer::maxAccelerationMMPerSquareSecond[X_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X;
        HAL::eprSetFloat(EPR_X_MAX_ACCEL, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X);
        change = true; //update checksum later in this function
    }
    if (Printer::maxAccelerationMMPerSquareSecond[Y_AXIS] < ACCELERATION_MIN_XY || Printer::maxAccelerationMMPerSquareSecond[Y_AXIS] > ACCELERATION_MAX_XY) {
        Printer::maxAccelerationMMPerSquareSecond[Y_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y;
        HAL::eprSetFloat(EPR_Y_MAX_ACCEL, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y);
        change = true; //update checksum later in this function
    }
    if (Printer::maxAccelerationMMPerSquareSecond[Z_AXIS] < ACCELERATION_MIN_Z || Printer::maxAccelerationMMPerSquareSecond[Z_AXIS] > ACCELERATION_MAX_Z) {
        Printer::maxAccelerationMMPerSquareSecond[Z_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z;
        HAL::eprSetFloat(EPR_Z_MAX_ACCEL, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z);
        change = true; //update checksum later in this function
    }

    Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS] = HAL::eprGetFloat(EPR_X_MAX_TRAVEL_ACCEL);
    Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS] = HAL::eprGetFloat(EPR_Y_MAX_TRAVEL_ACCEL);
    Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS] = HAL::eprGetFloat(EPR_Z_MAX_TRAVEL_ACCEL);
    if (Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS] < ACCELERATION_MIN_XY || Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS] > ACCELERATION_MAX_XY) {
        Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS] = MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X;
        HAL::eprSetFloat(EPR_X_MAX_TRAVEL_ACCEL, MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X);
        change = true; //update checksum later in this function
    }
    if (Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS] < ACCELERATION_MIN_XY || Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS] > ACCELERATION_MAX_XY) {
        Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS] = MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y;
        HAL::eprSetFloat(EPR_Y_MAX_TRAVEL_ACCEL, MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y);
        change = true; //update checksum later in this function
    }
    if (Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS] < ACCELERATION_MIN_Z || Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS] > ACCELERATION_MAX_Z) {
        Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS] = MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z;
        HAL::eprSetFloat(EPR_Z_MAX_TRAVEL_ACCEL, MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z);
        change = true; //update checksum later in this function
    }

    heatedBedController.pidDriveMax = HAL::eprGetByte(EPR_BED_DRIVE_MAX);
    heatedBedController.pidDriveMin = HAL::eprGetByte(EPR_BED_DRIVE_MIN);
    heatedBedController.pidPGain = HAL::eprGetFloat(EPR_BED_PID_PGAIN);
    heatedBedController.pidIGain = HAL::eprGetFloat(EPR_BED_PID_IGAIN);
    heatedBedController.pidDGain = HAL::eprGetFloat(EPR_BED_PID_DGAIN);
    heatedBedController.pidMax = HAL::eprGetByte(EPR_BED_PID_MAX);
    heatedBedController.sensorType = (HAL::eprGetByte(EPR_RF_HEATED_BED_SENSOR_TYPE) != 0) ? HAL::eprGetByte(EPR_RF_HEATED_BED_SENSOR_TYPE) : HEATED_BED_SENSOR_TYPE;

    Printer::axisLengthMM[X_AXIS] = HAL::eprGetFloat(EPR_X_LENGTH);
    if (Printer::axisLengthMM[X_AXIS] <= 0 || Printer::axisLengthMM[X_AXIS] > 245.0f) {
        Printer::axisLengthMM[X_AXIS] = X_MAX_LENGTH_PRINT;
        HAL::eprSetFloat(EPR_X_LENGTH, Printer::axisLengthMM[X_AXIS]);
        change = true; //update checksum later in this function
    }
    Printer::axisLengthMM[Y_AXIS] = HAL::eprGetFloat(EPR_Y_LENGTH);
    Printer::axisLengthMM[Z_AXIS] = HAL::eprGetFloat(EPR_Z_LENGTH);

    // now the extruder
    for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
        int o = EEPROM::getExtruderOffset(i);
        Extruder* e = &extruder[i];
        float tmp = HAL::eprGetFloat(o + EPR_EXTRUDER_STEPS_PER_MM);
        if (tmp < 5540.0f) { //da hat einer ein komma vergessen ^^ so hohe Werte können kaum sinn machen, ausser wir ändern die CPU. Das wären auch zu viele interrupts pro sekunde.
            e->stepsPerMM = tmp;
        } else {
            HAL::eprSetFloat(o + EPR_EXTRUDER_STEPS_PER_MM, e->stepsPerMM);
            change = true; //update checksum later in this function
        }

        tmp = HAL::eprGetFloat(o + EPR_EXTRUDER_MAX_FEEDRATE);
        if (0 < tmp && tmp < 100) {
            e->maxFeedrate = tmp;
        } else {
            HAL::eprSetFloat(o + EPR_EXTRUDER_MAX_FEEDRATE, e->maxFeedrate);
            change = true; //update checksum later in this function
        }

        tmp = HAL::eprGetFloat(o + EPR_EXTRUDER_MAX_START_FEEDRATE);
        if (0 < tmp && tmp <= e->maxFeedrate) {
            e->maxEJerk = tmp;
        } else {
            e->maxEJerk = RMath::min(e->maxFeedrate, e->maxEJerk);
            HAL::eprSetFloat(o + EPR_EXTRUDER_MAX_START_FEEDRATE, e->maxEJerk);
            change = true; //update checksum later in this function
        }

        tmp = HAL::eprGetFloat(o + EPR_EXTRUDER_MAX_ACCELERATION);
        if (0 < tmp && tmp < 10000) {
            e->maxAcceleration = tmp;
        } else {
            HAL::eprSetFloat(o + EPR_EXTRUDER_MAX_ACCELERATION, e->maxAcceleration);
            change = true; //update checksum later in this function
        }

        e->tempControl.pidDriveMax = HAL::eprGetByte(o + EPR_EXTRUDER_DRIVE_MAX);
        e->tempControl.pidDriveMin = HAL::eprGetByte(o + EPR_EXTRUDER_DRIVE_MIN);
        if ((e->tempControl.pidDriveMin == 40 && e->tempControl.pidDriveMax == 40)
            || (e->tempControl.pidDriveMin == 0 && e->tempControl.pidDriveMax == 0)) {

#if NUM_EXTRUDER > 1
            e->tempControl.pidDriveMin = i == 0 ? EXT0_HOTEND_PROFILE.pidDriveMin : EXT1_HOTEND_PROFILE.pidDriveMin;
            e->tempControl.pidDriveMax = i == 0 ? EXT0_HOTEND_PROFILE.pidDriveMax : EXT1_HOTEND_PROFILE.pidDriveMax;
#else
            e->tempControl.pidDriveMin = EXT0_HOTEND_PROFILE.pidDriveMin;
            e->tempControl.pidDriveMax = EXT0_HOTEND_PROFILE.pidDriveMax;
#endif
            HAL::eprSetByte(o + EPR_EXTRUDER_DRIVE_MIN, e->tempControl.pidDriveMin);
            HAL::eprSetByte(o + EPR_EXTRUDER_DRIVE_MAX, e->tempControl.pidDriveMax);
            change = true; //update checksum later in this function
        }
        e->tempControl.pidPGain = HAL::eprGetFloat(o + EPR_EXTRUDER_PID_PGAIN);
        e->tempControl.pidIGain = HAL::eprGetFloat(o + EPR_EXTRUDER_PID_IGAIN);
        e->tempControl.pidDGain = HAL::eprGetFloat(o + EPR_EXTRUDER_PID_DGAIN);
        if (HAL::eprGetByte(o + EPR_EXTRUDER_PID_MAX) > 0) {
            e->tempControl.pidMax = HAL::eprGetByte(o + EPR_EXTRUDER_PID_MAX);
        } else {
            HAL::eprSetByte(o + EPR_EXTRUDER_PID_MAX, e->tempControl.pidMax);
            change = true; //update checksum later in this function
        }

        uint8_t sensortype_temp = HAL::eprGetByte(o + EPR_EXTRUDER_SENSOR_TYPE);
        if (sensortype_temp > 0 && sensortype_temp <= 100) {
            e->tempControl.sensorType = sensortype_temp;
        }

        e->offsetMM[X_AXIS] = HAL::eprGetFloat(o + EPR_EXTRUDER_X_OFFSET);
        e->offsetMM[Y_AXIS] = HAL::eprGetFloat(o + EPR_EXTRUDER_Y_OFFSET);


        e->advanceL = HAL::eprGetFloat(o + EPR_EXTRUDER_ADVANCE_L);
        if (e->advanceL > 0 && e->advanceL < 20) {
            e->advanceL = 20;
        }

        if (version > 1)
            e->coolerSpeed = HAL::eprGetByte(o + EPR_EXTRUDER_COOLER_SPEED);
    }

    Printer::enableCaseLight = HAL::eprGetByte(EPR_RF_CASE_LIGHT_MODE);

#ifdef RGB_LIGHT_R_PIN
    Printer::RGBLightMode = HAL::eprGetByte(EPR_RF_RGB_LIGHT_MODE);
    if (Printer::RGBLightMode == RGB_MODE_AUTOMATIC) {
        Printer::RGBLightStatus = RGB_STATUS_AUTOMATIC;
        Printer::RGBLightIdleStart = 0;
    } else {
        Printer::RGBLightStatus = RGB_STATUS_NOT_AUTOMATIC;
    }
#endif // RGB_LIGHT_R_PIN

#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    Printer::enableFET1 = HAL::eprGetByte(EPR_RF_FET1_MODE);
    Printer::enableFET2 = HAL::eprGetByte(EPR_RF_FET2_MODE);
#if CASE_FAN_PIN < 0
    // Do not load the FET3 EEPROM state if it is being used as case fan
    Printer::enableFET3 = HAL::eprGetByte(EPR_RF_FET3_MODE);
#endif // CASE_FAN_PIN < 0

    // after a power-on, the 230 V plug always shall be turned off - thus, we do not store this setting to the EEPROM
    // Printer::enable230VOutput = HAL::eprGetByte( EPR_RF_230V_OUTPUT_MODE );
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2

    //reinit for check if eeprom values are too high.
    unsigned int eeprom_homing_feedrate_position;
        eeprom_homing_feedrate_position = EPR_X_HOMING_FEEDRATE_PRINT;
        Printer::homingFeedrate[X_AXIS] = HOMING_FEEDRATE_X_PRINT;
        Printer::homingFeedrate[Y_AXIS] = HOMING_FEEDRATE_Y_PRINT;
        Printer::homingFeedrate[Z_AXIS] = HOMING_FEEDRATE_Z_PRINT;
    //check if values are too high
    //this is no super complete check but if someone uses the printer bad values from old firmwares get corrected after one restart in each mode.
    // and update them to max according configuration consts if too high.
    for (uint8_t axis = X_AXIS; axis <= Z_AXIS; axis++) {
        float tmp = HAL::eprGetFloat(eeprom_homing_feedrate_position + axis * 4); // EPR_X_HOMING_FEEDRATE_PRINT EPR_Y_HOMING_FEEDRATE_PRINT EPR_Z_HOMING_FEEDRATE_PRINT

        //Alle Homing-Geschwindigkeiten höher max-Speed werden mit der Standard-Config-Homing-Speed überschrieben.
        //Das betrifft gerade noch die alten RF-Speeds, die viel zu hoch waren:
        //z.B: https://github.com/RF1000/Repetier-Firmware/blob/0e52693eaed1f606af411e4898030b033304b94f/RF2000/Repetier/RF2000.h#L781
        if (tmp > Printer::maxFeedrate[axis]) {
            HAL::eprSetFloat(eeprom_homing_feedrate_position + axis * 4, Printer::homingFeedrate[axis]);
            change = true; //update checksum later in this function
        } else {
            Printer::homingFeedrate[axis] = tmp;
        }
    }


    g_nManualSteps[X_AXIS] = (unsigned short)lroundf(Printer::axisStepsPerMM[X_AXIS] * DEFAULT_MANUAL_MM_X);
    g_nManualSteps[Y_AXIS] = (unsigned short)lroundf(Printer::axisStepsPerMM[Y_AXIS] * DEFAULT_MANUAL_MM_Y);
    const unsigned short stepsize_table[NUM_ACCEPTABLE_STEP_SIZE_TABLE] PROGMEM = ACCEPTABLE_STEP_SIZE_TABLE;
    g_nManualSteps[Z_AXIS] = (unsigned short)constrain((unsigned short)HAL::eprGetInt16(EPR_RF_MOD_Z_STEP_SIZE), 1, stepsize_table[NUM_ACCEPTABLE_STEP_SIZE_TABLE - 1]); //limit stepsize to value in config.
    g_nManualSteps[E_AXIS] = (unsigned short)lroundf(Extruder::current->stepsPerMM * DEFAULT_MANUAL_MM_E);                                                               //current extruder stepsPerMM weil hier noch kein update für Printer::axisStepsPerMM[E_AXIS] gemacht wurde!

    g_ZOSTestPoint[X_AXIS] = HAL::eprGetByte(EPR_RF_MOD_ZOS_SCAN_POINT_X);
    if (g_ZOSTestPoint[X_AXIS] != 0) { //constrain if not 0 = random.
                                       //wenn die Startposition nicht 0 ist, wird eine Dummy-Matrix-Linie ergänzt. Mit der sollten wir nicht arbeiten.
        if (g_ZOSTestPoint[X_AXIS] < 1 + ((heatBedScanXStartMm == 0) ? 0 : 1))
            g_ZOSTestPoint[X_AXIS] = 1 + ((heatBedScanXStartMm == 0) ? 0 : 1);
        if (g_ZOSTestPoint[X_AXIS] > COMPENSATION_MATRIX_MAX_X - 1)
            g_ZOSTestPoint[X_AXIS] = COMPENSATION_MATRIX_MAX_X - 1; //2..n-1 //COMPENSATION_MATRIX_MAX_X -> g_uZMatrixMax[X_AXIS] erst voll wenn matrix initialisiert, aber die normale ultra-max-grenze reicht hier! constrain nachher.
    }
    g_ZOSTestPoint[Y_AXIS] = HAL::eprGetByte(EPR_RF_MOD_ZOS_SCAN_POINT_Y);
    if (g_ZOSTestPoint[Y_AXIS] != 0) { //constrain if not 0 = random.
                                       //wenn die Startposition nicht 0 ist, wird eine Dummy-Matrix-Linie ergänzt. Mit der sollten wir nicht arbeiten.
        if (g_ZOSTestPoint[Y_AXIS] < 1 + ((heatBedScanYStartMm == 0) ? 0 : 1))
            g_ZOSTestPoint[Y_AXIS] = 1 + ((heatBedScanYStartMm == 0) ? 0 : 1);
        if (g_ZOSTestPoint[Y_AXIS] > COMPENSATION_MATRIX_MAX_Y - 1)
            g_ZOSTestPoint[Y_AXIS] = COMPENSATION_MATRIX_MAX_Y - 1; //2..n-1 //COMPENSATION_MATRIX_MAX_Y -> g_uZMatrixMax[Y_AXIS] erst voll wenn matrix initialisiert, aber die normale ultra-max-grenze reicht hier! constrain nachher.
    }

    //Do not read EPR_RF_MOD_SENSEOFFSET_DIGITS here
    const int16_t savedSenseLimit = HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX);
    Printer::senseOffsetControl.setMaxMicrometers(savedSenseLimit == 0 ? SENSIBLE_PRESSURE_MAX_OFFSET : constrain(savedSenseLimit, 1, 300));
    Printer::senseOffsetControl.setAutoStart(HAL::eprGetByte(EPR_RF_MOD_SENSEOFFSET_AUTOSTART));

    Printer::wobbleCorrection.setProfile({
        int8_t(HAL::eprGetByte(EPR_RF_MOD_WOBBLE_FIX_PHASEXY)),
        HAL::eprGetInt16(EPR_RF_MOD_WOBBLE_FIX_AMPX),
        HAL::eprGetInt16(EPR_RF_MOD_WOBBLE_FIX_AMPY1),
        HAL::eprGetInt16(EPR_RF_MOD_WOBBLE_FIX_AMPY2)
    });

    const int32_t storedPauseMin = HAL::eprGetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMIN);
    const int32_t storedPauseMax = HAL::eprGetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMAX);
    const PressureProtection::Limits pauseLimits = {
        int16_t(constrain(storedPauseMin, EMERGENCY_PAUSE_DIGITS_MIN, EMERGENCY_PAUSE_DIGITS_MAX)),
        int16_t(constrain(storedPauseMax, EMERGENCY_PAUSE_DIGITS_MIN, EMERGENCY_PAUSE_DIGITS_MAX))
    };
    Printer::pressureProtection.setPauseLimits(pauseLimits);

    // Default flow limits depend on the emergency pause limit.
    auto flowProfile = Printer::digitCompensation.flowProfile();
    flowProfile.minDigits = RMath::min(short(abs(pauseLimits.maximum) * 0.7), 4000);
    flowProfile.maxDigits = short(abs(pauseLimits.maximum));
    Printer::digitCompensation.setFlowProfile(flowProfile);

    const int16_t storedStopMin = HAL::eprGetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMIN);
    const int16_t storedStopMax = HAL::eprGetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMAX);
    PressureProtection::Limits stopLimits = {
        int16_t(constrain(storedStopMin, EMERGENCY_STOP_DIGITS_MIN, EMERGENCY_STOP_DIGITS_MAX)),
        int16_t(constrain(storedStopMax, EMERGENCY_STOP_DIGITS_MIN, EMERGENCY_STOP_DIGITS_MAX))
    };
    if (stopLimits.minimum == 0) {
        stopLimits.minimum = EMERGENCY_STOP_DIGITS_MIN;
        HAL::eprSetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMIN, stopLimits.minimum);
        change = true;
    }
    if (stopLimits.maximum == 0) {
        stopLimits.maximum = EMERGENCY_STOP_DIGITS_MAX;
        HAL::eprSetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMAX, stopLimits.maximum);
        change = true;
    }
    Printer::pressureProtection.setStopLimits(stopLimits);

    Printer::strainGauge.setZeroingEnabled(HAL::eprGetByte(EPR_RF_ZERO_DIGIT_STATE) <= 1);
    Printer::digitCompensation.setDigitZEnabled(HAL::eprGetByte(EPR_RF_DIGIT_CMP_STATE) <= 1);

    float tmpss = HAL::eprGetFloat(EPR_ZSCAN_START_MM);
    if (tmpss >= 0.3f && tmpss <= 6.0f) {
        g_scanStartZLiftMM = tmpss;
    } else {
        HAL::eprSetFloat(EPR_ZSCAN_START_MM, defaultHeatBedScanLiftMm);
        change = true;
    }

    const unsigned short uMotorCurrentMax[] = MOTOR_CURRENT_MAX;    //oberes Amperelimit
    const unsigned short uMotorCurrentUse[] = MOTOR_CURRENT_NORMAL; //Standardwert
    uint8_t current = 0;
    for (uint8_t axis = 0; axis < DRV8711_NUM_CHANNELS; axis++) { //0..4 bei 5 steppern.
        current = HAL::eprGetByte(EPR_RF_MOTOR_CURRENT + axis);
        if (MOTOR_CURRENT_MIN <= current && current <= uMotorCurrentMax[axis]) {
            Printer::motorCurrent[axis] = current;
            setMotorCurrent(axis + 1, current); //driver ist 1-basiert
        } else {
            HAL::eprSetByte(EPR_RF_MOTOR_CURRENT + axis, uMotorCurrentUse[axis]); //wenn mist im EEPROM, dann Silent-Wert reinschreiben.
            change = true;
        }
    }
    //does EEPROM have valid values for microsteps?
    if (HAL::eprGetByte(EPR_RF_MICRO_STEPS_USED) == 0xAB) {
        for (uint8_t axis = 0; axis < DRV8711_NUM_CHANNELS; axis++) {
            /* #define EPR_RF_MICRO_STEPS_X              1943 //[1byte]
               #define EPR_RF_MICRO_STEPS_Y              1944 //[1byte]
               #define EPR_RF_MICRO_STEPS_Z              1945 //[1byte]
               #define EPR_RF_MICRO_STEPS_E0             1946 //[1byte]
               #define EPR_RF_MICRO_STEPS_E1             1947 //[1byte] */
            uint8_t epr = HAL::eprGetByte(EPR_RF_MICRO_STEPS_X + axis);
            if (epr <= 8 && Printer::motorMicroStepsModeValue[axis] != epr) {
                Printer::motorMicroStepsModeValue[axis] = epr;
                drv8711adjustMicroSteps(axis + 1);
            }
        }
    }

    if (!HAL::eprGetInt16(EPR_RF_STEP_PACKING_MIN_INTERVAL)) {
        Printer::stepsPackingMinInterval = STEP_PACKING_MIN_INTERVAL;
        HAL::eprSetInt16(EPR_RF_STEP_PACKING_MIN_INTERVAL, STEP_PACKING_MIN_INTERVAL);
        change = true;
    } else {
        Printer::stepsPackingMinInterval = constrain(HAL::eprGetInt16(EPR_RF_STEP_PACKING_MIN_INTERVAL), MIN_STEP_PACKING_MIN_INTERVAL, MAX_STEP_PACKING_MIN_INTERVAL);
    }

    uint8_t temp_min = HAL::eprGetByte(EPR_RF_PART_FAN_PWM_MIN);
    uint8_t temp_max = HAL::eprGetByte(EPR_RF_PART_FAN_PWM_MAX);
    if ((int)temp_max - (int)temp_min >= 16 && temp_min > 0 && temp_max < 255) {
        part_fan_pwm_min = temp_min;
        part_fan_pwm_max = temp_max;
    } else {
        HAL::eprSetByte(EPR_RF_PART_FAN_PWM_MIN, PART_FAN_PWM_MIN);
        HAL::eprSetByte(EPR_RF_PART_FAN_PWM_MAX, PART_FAN_PWM_MAX);
        change = true;
    }

    uint8_t tempfs = HAL::eprGetByte(EPR_RF_PART_FAN_SPEED);
    if (tempfs > 0 && tempfs <= PART_FAN_MODE_MAX) {
        Commands::adjustFanFrequency(tempfs);
    }
    Commands::adjustFanMode((HAL::eprGetByte(EPR_RF_FAN_MODE) == PART_FAN_MODE_PDM ? PART_FAN_MODE_PDM : PART_FAN_MODE_PWM));

    if (change)
        EEPROM::updateChecksum();

    if (version != EEPROM_PROTOCOL_VERSION) {
        Com::printInfoFLN(Com::tEPRProtocolChanged);
        storeDataIntoEEPROM(false); // Store new fields for changed version
    }
    Printer::updateDerivedParameter();
    Extruder::applyCurrentSettings();
    Printer::applyAdvanceSettings();
    Extruder::initHeatedBed();

} // readDataFromEEPROM

void EEPROM::initBaudrate() {
    if (HAL::eprGetByte(EPR_MAGIC_BYTE) == eepromMagic) {
        baudrate = HAL::eprGetInt32(EPR_BAUDRATE);
    }
} // initBaudrate

void EEPROM::init() {
    bool kill_eeprom_because_corrupted = (computeChecksum() != HAL::eprGetByte(EPR_INTEGRITY_BYTE));
    bool kill_eeprom_wrong_version_update = (eepromMagic != HAL::eprGetByte(EPR_MAGIC_BYTE));
    bool kill_eeprom_by_back_ok_play = (READ(ENABLE_KEY_1) == 0 && READ(ENABLE_KEY_4) == 0 && READ(ENABLE_KEY_E5) == 0);

    bool update_eeprom_num_extruders_changed_2 = (NUM_EXTRUDER > HAL::eprGetByte(EPR_CHECK_NUM_EXTRUDERS) && NUM_EXTRUDER == 2);
    bool update_eeprom_num_extruders_changed_1 = (NUM_EXTRUDER < HAL::eprGetByte(EPR_CHECK_NUM_EXTRUDERS) && NUM_EXTRUDER == 1);

    if (kill_eeprom_wrong_version_update || kill_eeprom_because_corrupted || kill_eeprom_by_back_ok_play) {
        EEPROM::clearEEPROM();
        EEPROM::restoreEEPROMSettingsFromConfiguration();
        EEPROM::storeDataIntoEEPROM(kill_eeprom_because_corrupted); //wenn corrupted dann auch die betriebszähler löschen.

        Com::printF(PSTR("EEPROM reset "));

        if (kill_eeprom_wrong_version_update) {
            Com::printFLN(PSTR("wrong version"));
            showInformation(PSTR(UI_TEXT_CONFIGURATION), PSTR(UI_TEXT_UPDATE), PSTR(UI_TEXT_RESTORE_DEFAULTS));

            return;
        }

        if (kill_eeprom_because_corrupted)
            Com::printF(PSTR("corrupted"));
        if (kill_eeprom_by_back_ok_play)
            Com::printF(PSTR("back+ok+play"));
        Com::println();
        showInformation(PSTR(UI_TEXT_CONFIGURATION), PSTR(UI_TEXT_FAIL), PSTR(UI_TEXT_RESTORE_DEFAULTS));

        return;
    }

    if (update_eeprom_num_extruders_changed_2) {
        // prevent stupid bugs because of zeros in extruder 2 configuration
        // add second extruder settings to eeprom
        EEPROM::restoreEEPROMExtruderSettingsFromConfiguration(1);
        EEPROM::storeExtruderDataIntoEEPROM(1);
        // auto-adjust x axis length:
        Printer::axisLengthMM[X_AXIS] = X_MAX_LENGTH_PRINT;
        HAL::eprSetFloat(EPR_X_LENGTH, X_MAX_LENGTH_PRINT);

        HAL::eprSetByte(EPR_CHECK_NUM_EXTRUDERS, NUM_EXTRUDER);
        EEPROM::updateChecksum();

        Com::printF(PSTR("EEPROM update "));
        Com::printFLN(PSTR(UI_TEXT_EXTRUDER1_ADD));
        showInformation(PSTR(UI_TEXT_CONFIGURATION), PSTR(UI_TEXT_EXTRUDER1_ADD), PSTR(UI_TEXT_UPDATE_MATRIX));

        //continue loading data.
    }

    if (update_eeprom_num_extruders_changed_1) {
        // auto-adjust x axis length:
        Printer::axisLengthMM[X_AXIS] = X_MAX_LENGTH_PRINT;
        HAL::eprSetFloat(EPR_X_LENGTH, X_MAX_LENGTH_PRINT);

        HAL::eprSetByte(EPR_CHECK_NUM_EXTRUDERS, NUM_EXTRUDER);
        EEPROM::updateChecksum();

        Com::printF(PSTR("EEPROM update "));
        Com::printFLN(PSTR(UI_TEXT_EXTRUDER1_REMOVED));
        showInformation(PSTR(UI_TEXT_CONFIGURATION), PSTR(UI_TEXT_EXTRUDER1_REMOVED), PSTR(UI_TEXT_UPDATE_MATRIX));

        //continue loading data.
    }

    EEPROM::readDataFromEEPROM();
} // init

void EEPROM::updatePrinterUsage() {
    if (Printer::filamentPrinted == 0 || (Printer::flag2 & PRINTER_FLAG2_RESET_FILAMENT_USAGE) != 0)
        return; // No miles only enabled
    uint32_t seconds = (HAL::timeInMilliseconds() - Printer::msecondsPrinting) / 1000;
    seconds += HAL::eprGetInt32(EPR_PRINTING_TIME);
    HAL::eprSetInt32(EPR_PRINTING_TIME, seconds);
    HAL::eprSetFloat(EPR_PRINTING_DISTANCE, HAL::eprGetFloat(EPR_PRINTING_DISTANCE) + Printer::filamentPrinted * 0.001);

    Printer::flag2 |= PRINTER_FLAG2_RESET_FILAMENT_USAGE;
    Printer::msecondsPrinting = HAL::timeInMilliseconds();
    EEPROM::updateChecksum();
    Commands::reportPrinterUsage();
} // updatePrinterUsage

int EEPROM::getExtruderOffset(uint8_t extruder) {
    return extruder * EEPROM_EXTRUDER_LENGTH + EEPROM_EXTRUDER_OFFSET;
} // getExtruderOffset

/** \brief Writes all eeprom settings to serial console.
For each value stored, this function generates one line with syntax

EPR: pos type value description

With
- pos = Position in EEPROM, the data starts.
- type = Value type: 0 = byte, 1 = int, 2 = long, 3 = float
- value = The value currently stored
- description = Definition of the value
*/
void EEPROM::writeSettings() {

    writeFloat(EPR_X_MAX_FEEDRATE, Com::tEPRXMaxFeedrate);
    writeFloat(EPR_X_HOMING_FEEDRATE_PRINT, Com::tEPRXHomingFeedrate);
    writeFloat(EPR_Y_MAX_FEEDRATE, Com::tEPRYMaxFeedrate);
    writeFloat(EPR_Y_HOMING_FEEDRATE_PRINT, Com::tEPRYHomingFeedrate);
    writeFloat(EPR_Z_MAX_FEEDRATE, Com::tEPRZMaxFeedrate);
    writeFloat(EPR_Z_HOMING_FEEDRATE_PRINT, Com::tEPRZHomingFeedrate);

    writeFloat(EPR_X_MAX_ACCEL, Com::tEPRXAcceleration);
    writeFloat(EPR_X_MAX_TRAVEL_ACCEL, Com::tEPRXTravelAcceleration);
    writeFloat(EPR_Y_MAX_ACCEL, Com::tEPRYAcceleration);
    writeFloat(EPR_Y_MAX_TRAVEL_ACCEL, Com::tEPRYTravelAcceleration);
    writeFloat(EPR_Z_MAX_ACCEL, Com::tEPRZAcceleration);
    writeFloat(EPR_Z_MAX_TRAVEL_ACCEL, Com::tEPRZTravelAcceleration);

    writeFloat(EPR_MAX_XYJERK, Com::tEPRMaxXYJerk);
    writeFloat(EPR_MAX_ZJERK, Com::tEPRMaxZJerk);

    writeByte(EPR_RF_MOTOR_CURRENT + X_AXIS, Com::tEPRPrinter_STEPPER_X);
    writeByte(EPR_RF_MOTOR_CURRENT + Y_AXIS, Com::tEPRPrinter_STEPPER_Y);
    writeByte(EPR_RF_MOTOR_CURRENT + Z_AXIS, Com::tEPRPrinter_STEPPER_Z);
    writeByte(EPR_RF_MOTOR_CURRENT + E_AXIS + 0, Com::tEPRPrinter_STEPPER_E0);
#if NUM_EXTRUDER > 1
    writeByte(EPR_RF_MOTOR_CURRENT + E_AXIS + 1, Com::tEPRPrinter_STEPPER_E1);
#endif //NUM_EXTRUDER > 1

    writeFloat(EPR_XAXIS_STEPS_PER_MM, Com::tEPRXStepsPerMM, 4);
    writeFloat(EPR_YAXIS_STEPS_PER_MM, Com::tEPRYStepsPerMM, 4);
    writeFloat(EPR_ZAXIS_STEPS_PER_MM, Com::tEPRZStepsPerMM, 4);
    writeInt(EPR_RF_STEP_PACKING_MIN_INTERVAL, Com::tEPRInterruptSpacingInterval);

    writeFloat(EPR_X_LENGTH, Com::tEPRXMaxLength);
    writeFloat(EPR_Y_LENGTH, Com::tEPRYMaxLength);
    writeFloat(EPR_Z_LENGTH, Com::tEPRZMaxLength);

    writeLong(EPR_RF_Z_OFFSET, Com::tEPRZOffset);
    writeByte(EPR_RF_Z_MODE, Com::tEPRZMode);

    writeInt(EPR_RF_MOD_Z_STEP_SIZE, Com::tEPRPrinterZ_STEP_SIZE);

    writeByte(EPR_RF_ZERO_DIGIT_STATE, Com::tEPRZERO_DIGIT_STATE);
    writeByte(EPR_RF_DIGIT_CMP_STATE, Com::tEPRZDIGIT_CMP_STATE);

    writeInt(EPR_RF_MOD_SENSEOFFSET_DIGITS, Com::tEPRPrinterEPR_RF_MOD_SENSEOFFSET_DIGITS);
    writeInt(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX, Com::tEPRPrinterMOD_SENSEOFFSET_OFFSET_MAX);

    writeLong(EPR_RF_EMERGENCYPAUSEDIGITSMIN, Com::tEPRPrinterEPR_RF_EmergencyPauseDigitsMin);
    writeLong(EPR_RF_EMERGENCYPAUSEDIGITSMAX, Com::tEPRPrinterEPR_RF_EmergencyPauseDigitsMax);

    writeInt(EPR_RF_EMERGENCYZSTOPDIGITSMIN, Com::tEPRPrinterEPR_RF_EmergencyStopAllMin);
    writeInt(EPR_RF_EMERGENCYZSTOPDIGITSMAX, Com::tEPRPrinterEPR_RF_EmergencyStopAllMax);

    writeByte(EPR_RF_MOD_ZOS_SCAN_POINT_X, Com::tEPRPrinterMOD_ZOS_SCAN_POINT_X);
    writeByte(EPR_RF_MOD_ZOS_SCAN_POINT_Y, Com::tEPRPrinterMOD_ZOS_SCAN_POINT_Y);
    writeFloat(EPR_ZSCAN_START_MM, Com::tEPRZScanStartLift);

    writeByte(EPR_RF_HEATED_BED_SENSOR_TYPE, Com::tEPRBedsensorType);
    writeFloat(EPR_BED_PID_PGAIN, Com::tEPRBedPGain);
    writeFloat(EPR_BED_PID_IGAIN, Com::tEPRBedIGain);
    writeFloat(EPR_BED_PID_DGAIN, Com::tEPRBedDGain);
    writeByte(EPR_BED_PID_MAX, Com::tEPRBedPIDMaxValue);
    writeByte(EPR_BED_DRIVE_MIN, Com::tEPRBedPIDDriveMin);
    writeByte(EPR_BED_DRIVE_MAX, Com::tEPRBedPIDDriveMax);

    // now the extruder
    for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
        int o = EEPROM::getExtruderOffset(i);
        writeFloat(o + EPR_EXTRUDER_STEPS_PER_MM, Com::tEPRStepsPerMM);
        writeFloat(o + EPR_EXTRUDER_MAX_FEEDRATE, Com::tEPRMaxFeedrate);
        writeFloat(o + EPR_EXTRUDER_MAX_ACCELERATION, Com::tEPRAcceleration);
        writeFloat(o + EPR_EXTRUDER_MAX_START_FEEDRATE, Com::tEPReJerk);

        writeFloat(o + EPR_EXTRUDER_ADVANCE_L, Com::tEPRAdvanceL);

        writeByte(o + EPR_EXTRUDER_SENSOR_TYPE, Com::tEPRsensorType);
        writeFloat(o + EPR_EXTRUDER_PID_PGAIN, Com::tEPRPGain, 4);
        writeFloat(o + EPR_EXTRUDER_PID_IGAIN, Com::tEPRIGain, 4);
        writeFloat(o + EPR_EXTRUDER_PID_DGAIN, Com::tEPRDGain, 4);
        writeByte(o + EPR_EXTRUDER_PID_MAX, Com::tEPRPIDMaxValue);
        writeByte(o + EPR_EXTRUDER_DRIVE_MIN, Com::tEPRDriveMin);
        writeByte(o + EPR_EXTRUDER_DRIVE_MAX, Com::tEPRDriveMax);

        writeFloat(o + EPR_EXTRUDER_X_OFFSET, Com::tEPRXOffset);
        writeFloat(o + EPR_EXTRUDER_Y_OFFSET, Com::tEPRYOffset);


        writeByte(o + EPR_EXTRUDER_COOLER_SPEED, Com::tEPRExtruderCoolerSpeed);
    }

    writeByte(EPR_RF_FAN_MODE, Com::tEPRPrinter_FAN_MODE);
    writeByte(EPR_RF_PART_FAN_SPEED, Com::tEPRPrinter_FAN_SPEED);
    writeByte(EPR_RF_PART_FAN_PWM_MIN, Com::tEPRPrinter_FAN_PART_FAN_PWM_MIN);
    writeByte(EPR_RF_PART_FAN_PWM_MAX, Com::tEPRPrinter_FAN_PART_FAN_PWM_MAX);

#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    writeByte(EPR_RF_FET1_MODE, Com::tEPRFET1Mode);
    writeByte(EPR_RF_FET2_MODE, Com::tEPRFET2Mode);
    writeByte(EPR_RF_FET3_MODE, Com::tEPRFET3Mode);
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2

    writeByte(EPR_RF_CASE_LIGHT_MODE, Com::tEPRCaseLightsMode);

#ifdef RGB_LIGHT_R_PIN
    writeByte(EPR_RF_RGB_LIGHT_MODE, Com::tEPRRGBLightMode);
#endif // RGB_LIGHT_R_PIN

    writeLong(EPR_BAUDRATE, Com::tEPRBaudrate);
    writeLong(EPR_MAX_INACTIVE_TIME, Com::tEPRMaxInactiveTime);
    writeLong(EPR_STEPPER_INACTIVE_TIME, Com::tEPRStopAfterInactivty);
    // RF specific

    writeFloat(EPR_PRINTING_DISTANCE, Com::tEPRFilamentPrinted);
    writeLong(EPR_PRINTING_TIME, Com::tEPRPrinterActive);

} // writeSettings

uint8_t EEPROM::computeChecksum() {
    millis_t lastTime = HAL::timeInMilliseconds();
    millis_t currentTime;
    unsigned int i;
    uint8_t checksum = 0;

    for (i = 0; i < 2048; i++) {
        if (i == EEPROM_OFFSET + EPR_INTEGRITY_BYTE)
            continue;
        checksum += HAL::eprGetByte(i);

        currentTime = HAL::timeInMilliseconds();
        if ((currentTime - lastTime) > PERIODICAL_ACTIONS_CALL_INTERVAL) {
            Commands::checkForPeriodicalActions(Processing);
            lastTime = currentTime;
        }
    }

    return checksum;
} // computeChecksum

void EEPROM::writeExtruderPrefix(uint pos) {
    if (pos < EEPROM_EXTRUDER_OFFSET || pos >= 800)
        return;
    int n = (pos - EEPROM_EXTRUDER_OFFSET) / EEPROM_EXTRUDER_LENGTH + 1;
    Com::printF(Com::tExtrDot, n);
    Com::print(' ');

} // writeExtruderPrefix

void EEPROM::writeFloat(uint pos, PGM_P text, uint8_t digits) {
    Com::printF(Com::tEPR3, (int)pos);
    Com::print(' ');
    Com::printFloat(HAL::eprGetFloat(pos), digits);
    Com::print(' ');
    writeExtruderPrefix(pos);
    Com::printFLN(text);
    HAL::delayMilliseconds(4); // reduces somehow transmission errors https://github.com/repetier/Repetier-Firmware/commit/6bdd97e83b656d15cafe51958ba1504e1bfb97de
} // writeFloat

void EEPROM::writeLong(uint pos, PGM_P text) {
    Com::printF(Com::tEPR2, (int)pos);
    Com::print(' ');
    Com::print(HAL::eprGetInt32(pos));
    Com::print(' ');
    writeExtruderPrefix(pos);
    Com::printFLN(text);
    HAL::delayMilliseconds(4); // reduces somehow transmission errors https://github.com/repetier/Repetier-Firmware/commit/6bdd97e83b656d15cafe51958ba1504e1bfb97de
} // writeLong

void EEPROM::writeInt(uint pos, PGM_P text) {
    Com::printF(Com::tEPR1, (int)pos);
    Com::print(' ');
    Com::print(HAL::eprGetInt16(pos));
    Com::print(' ');
    writeExtruderPrefix(pos);
    Com::printFLN(text);
    HAL::delayMilliseconds(4); // reduces somehow transmission errors https://github.com/repetier/Repetier-Firmware/commit/6bdd97e83b656d15cafe51958ba1504e1bfb97de
} // writeInt

void EEPROM::writeByte(uint pos, PGM_P text) {
    Com::printF(Com::tEPR0, (int)pos);
    Com::print(' ');
    Com::print((int)HAL::eprGetByte(pos));
    Com::print(' ');
    writeExtruderPrefix(pos);
    Com::printFLN(text);
    HAL::delayMilliseconds(4); // reduces somehow transmission errors https://github.com/repetier/Repetier-Firmware/commit/6bdd97e83b656d15cafe51958ba1504e1bfb97de
} // writeByte

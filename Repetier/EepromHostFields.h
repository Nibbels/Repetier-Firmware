// Canonical M205/M206 field inventory. Deliberately included with a caller-defined RF_EPR_FIELD.
// Arguments: wire type, start address, report label, float digits, writable.


    RF_EPR_FIELD(3, EPR_X_MAX_FEEDRATE, Com::tEPRXMaxFeedrate, 3, true);
    RF_EPR_FIELD(3, EPR_X_HOMING_FEEDRATE_PRINT, Com::tEPRXHomingFeedrate, 3, true);
    RF_EPR_FIELD(3, EPR_Y_MAX_FEEDRATE, Com::tEPRYMaxFeedrate, 3, true);
    RF_EPR_FIELD(3, EPR_Y_HOMING_FEEDRATE_PRINT, Com::tEPRYHomingFeedrate, 3, true);
    RF_EPR_FIELD(3, EPR_Z_MAX_FEEDRATE, Com::tEPRZMaxFeedrate, 3, true);
    RF_EPR_FIELD(3, EPR_Z_HOMING_FEEDRATE_PRINT, Com::tEPRZHomingFeedrate, 3, true);

    RF_EPR_FIELD(3, EPR_X_MAX_ACCEL, Com::tEPRXAcceleration, 3, true);
    RF_EPR_FIELD(3, EPR_X_MAX_TRAVEL_ACCEL, Com::tEPRXTravelAcceleration, 3, true);
    RF_EPR_FIELD(3, EPR_Y_MAX_ACCEL, Com::tEPRYAcceleration, 3, true);
    RF_EPR_FIELD(3, EPR_Y_MAX_TRAVEL_ACCEL, Com::tEPRYTravelAcceleration, 3, true);
    RF_EPR_FIELD(3, EPR_Z_MAX_ACCEL, Com::tEPRZAcceleration, 3, true);
    RF_EPR_FIELD(3, EPR_Z_MAX_TRAVEL_ACCEL, Com::tEPRZTravelAcceleration, 3, true);

    RF_EPR_FIELD(3, EPR_MAX_XYJERK, Com::tEPRMaxXYJerk, 3, true);
    RF_EPR_FIELD(3, EPR_MAX_ZJERK, Com::tEPRMaxZJerk, 3, true);

    RF_EPR_FIELD(0, EPR_RF_MOTOR_CURRENT + X_AXIS, Com::tEPRPrinter_STEPPER_X, 3, false);
    RF_EPR_FIELD(0, EPR_RF_MOTOR_CURRENT + Y_AXIS, Com::tEPRPrinter_STEPPER_Y, 3, false);
    RF_EPR_FIELD(0, EPR_RF_MOTOR_CURRENT + Z_AXIS, Com::tEPRPrinter_STEPPER_Z, 3, false);
    RF_EPR_FIELD(0, EPR_RF_MOTOR_CURRENT + E_AXIS + 0, Com::tEPRPrinter_STEPPER_E0, 3, false);
#if NUM_EXTRUDER > 1
    RF_EPR_FIELD(0, EPR_RF_MOTOR_CURRENT + E_AXIS + 1, Com::tEPRPrinter_STEPPER_E1, 3, false);
#endif //NUM_EXTRUDER > 1

    RF_EPR_FIELD(3, EPR_XAXIS_STEPS_PER_MM, Com::tEPRXStepsPerMM, 4, true);
    RF_EPR_FIELD(3, EPR_YAXIS_STEPS_PER_MM, Com::tEPRYStepsPerMM, 4, true);
    RF_EPR_FIELD(3, EPR_ZAXIS_STEPS_PER_MM, Com::tEPRZStepsPerMM, 4, true);
    RF_EPR_FIELD(1, EPR_RF_STEP_PACKING_MIN_INTERVAL, Com::tEPRInterruptSpacingInterval, 3, true);

    RF_EPR_FIELD(3, EPR_X_LENGTH, Com::tEPRXMaxLength, 3, true);
    RF_EPR_FIELD(3, EPR_Y_LENGTH, Com::tEPRYMaxLength, 3, true);
    RF_EPR_FIELD(3, EPR_Z_LENGTH, Com::tEPRZMaxLength, 3, true);

    RF_EPR_FIELD(2, EPR_RF_Z_OFFSET, Com::tEPRZOffset, 3, true);
    RF_EPR_FIELD(0, EPR_RF_Z_MODE, Com::tEPRZMode, 3, true);

    RF_EPR_FIELD(1, EPR_RF_MOD_Z_STEP_SIZE, Com::tEPRPrinterZ_STEP_SIZE, 3, true);

    RF_EPR_FIELD(0, EPR_RF_ZERO_DIGIT_STATE, Com::tEPRZERO_DIGIT_STATE, 3, true);
    RF_EPR_FIELD(0, EPR_RF_DIGIT_CMP_STATE, Com::tEPRZDIGIT_CMP_STATE, 3, true);

    RF_EPR_FIELD(1, EPR_RF_MOD_SENSEOFFSET_DIGITS, Com::tEPRPrinterEPR_RF_MOD_SENSEOFFSET_DIGITS, 3, true);
    RF_EPR_FIELD(1, EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX, Com::tEPRPrinterMOD_SENSEOFFSET_OFFSET_MAX, 3, true);

    RF_EPR_FIELD(2, EPR_RF_EMERGENCYPAUSEDIGITSMIN, Com::tEPRPrinterEPR_RF_EmergencyPauseDigitsMin, 3, true);
    RF_EPR_FIELD(2, EPR_RF_EMERGENCYPAUSEDIGITSMAX, Com::tEPRPrinterEPR_RF_EmergencyPauseDigitsMax, 3, true);

    RF_EPR_FIELD(1, EPR_RF_EMERGENCYZSTOPDIGITSMIN, Com::tEPRPrinterEPR_RF_EmergencyStopAllMin, 3, true);
    RF_EPR_FIELD(1, EPR_RF_EMERGENCYZSTOPDIGITSMAX, Com::tEPRPrinterEPR_RF_EmergencyStopAllMax, 3, true);

    RF_EPR_FIELD(0, EPR_RF_MOD_ZOS_SCAN_POINT_X, Com::tEPRPrinterMOD_ZOS_SCAN_POINT_X, 3, true);
    RF_EPR_FIELD(0, EPR_RF_MOD_ZOS_SCAN_POINT_Y, Com::tEPRPrinterMOD_ZOS_SCAN_POINT_Y, 3, true);
    RF_EPR_FIELD(3, EPR_ZSCAN_START_MM, Com::tEPRZScanStartLift, 3, true);

    RF_EPR_FIELD(0, EPR_RF_HEATED_BED_SENSOR_TYPE, Com::tEPRBedsensorType, 3, true);
    RF_EPR_FIELD(3, EPR_BED_PID_PGAIN, Com::tEPRBedPGain, 3, false);
    RF_EPR_FIELD(3, EPR_BED_PID_IGAIN, Com::tEPRBedIGain, 3, false);
    RF_EPR_FIELD(3, EPR_BED_PID_DGAIN, Com::tEPRBedDGain, 3, false);
    RF_EPR_FIELD(0, EPR_BED_PID_MAX, Com::tEPRBedPIDMaxValue, 3, true);
    RF_EPR_FIELD(0, EPR_BED_DRIVE_MIN, Com::tEPRBedPIDDriveMin, 3, true);
    RF_EPR_FIELD(0, EPR_BED_DRIVE_MAX, Com::tEPRBedPIDDriveMax, 3, true);

    // now the extruder
    for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
        int o = EEPROM::getExtruderOffset(i);
        RF_EPR_FIELD(3, o + EPR_EXTRUDER_STEPS_PER_MM, Com::tEPRStepsPerMM, 3, true);
        RF_EPR_FIELD(3, o + EPR_EXTRUDER_MAX_FEEDRATE, Com::tEPRMaxFeedrate, 3, true);
        RF_EPR_FIELD(3, o + EPR_EXTRUDER_MAX_ACCELERATION, Com::tEPRAcceleration, 3, true);
        RF_EPR_FIELD(3, o + EPR_EXTRUDER_MAX_START_FEEDRATE, Com::tEPReJerk, 3, true);

        RF_EPR_FIELD(3, o + EPR_EXTRUDER_ADVANCE_L, Com::tEPRAdvanceL, 3, true);

        RF_EPR_FIELD(0, o + EPR_EXTRUDER_SENSOR_TYPE, Com::tEPRsensorType, 3, true);
        RF_EPR_FIELD(3, o + EPR_EXTRUDER_PID_PGAIN, Com::tEPRPGain, 4, false);
        RF_EPR_FIELD(3, o + EPR_EXTRUDER_PID_IGAIN, Com::tEPRIGain, 4, false);
        RF_EPR_FIELD(3, o + EPR_EXTRUDER_PID_DGAIN, Com::tEPRDGain, 4, false);
        RF_EPR_FIELD(0, o + EPR_EXTRUDER_PID_MAX, Com::tEPRPIDMaxValue, 3, true);
        RF_EPR_FIELD(0, o + EPR_EXTRUDER_DRIVE_MIN, Com::tEPRDriveMin, 3, true);
        RF_EPR_FIELD(0, o + EPR_EXTRUDER_DRIVE_MAX, Com::tEPRDriveMax, 3, true);

        RF_EPR_FIELD(3, o + EPR_EXTRUDER_X_OFFSET, Com::tEPRXOffset, 3, true);
        RF_EPR_FIELD(3, o + EPR_EXTRUDER_Y_OFFSET, Com::tEPRYOffset, 3, true);


        RF_EPR_FIELD(0, o + EPR_EXTRUDER_COOLER_SPEED, Com::tEPRExtruderCoolerSpeed, 3, true);
    }

    RF_EPR_FIELD(0, EPR_RF_FAN_MODE, Com::tEPRPrinter_FAN_MODE, 3, true);
    RF_EPR_FIELD(0, EPR_RF_PART_FAN_SPEED, Com::tEPRPrinter_FAN_SPEED, 3, true);
    RF_EPR_FIELD(0, EPR_RF_PART_FAN_PWM_MIN, Com::tEPRPrinter_FAN_PART_FAN_PWM_MIN, 3, true);
    RF_EPR_FIELD(0, EPR_RF_PART_FAN_PWM_MAX, Com::tEPRPrinter_FAN_PART_FAN_PWM_MAX, 3, true);

#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    RF_EPR_FIELD(0, EPR_RF_FET1_MODE, Com::tEPRFET1Mode, 3, true);
    RF_EPR_FIELD(0, EPR_RF_FET2_MODE, Com::tEPRFET2Mode, 3, true);
    RF_EPR_FIELD(0, EPR_RF_FET3_MODE, Com::tEPRFET3Mode, 3, (CASE_FAN_PIN < 0));
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2

    RF_EPR_FIELD(0, EPR_RF_CASE_LIGHT_MODE, Com::tEPRCaseLightsMode, 3, true);


    RF_EPR_FIELD(2, EPR_MAX_INACTIVE_TIME, Com::tEPRMaxInactiveTime, 3, true);
    RF_EPR_FIELD(2, EPR_STEPPER_INACTIVE_TIME, Com::tEPRStopAfterInactivty, 3, true);
    RF_EPR_FIELD(1, EPR_RF_PREHEAT_A_BED, PSTR("Preheat A bed [C]"), 3, true);
    RF_EPR_FIELD(1, EPR_RF_PREHEAT_A_HOTEND, PSTR("Preheat A hotend [C]"), 3, true);
    RF_EPR_FIELD(1, EPR_RF_PREHEAT_B_BED, PSTR("Preheat B bed [C]"), 3, true);
    RF_EPR_FIELD(1, EPR_RF_PREHEAT_B_HOTEND, PSTR("Preheat B hotend [C]"), 3, true);
    // RF specific

    RF_EPR_FIELD(3, EPR_PRINTING_DISTANCE, Com::tEPRFilamentPrinted, 3, false);
    RF_EPR_FIELD(2, EPR_PRINTING_TIME, Com::tEPRPrinterActive, 3, false);


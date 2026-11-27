// New-schema M205/M206 inventory. Include with RF_ZONE_FIELD defined.
// This becomes the only public field inventory at live-schema cutover.
// Arguments: wire type, physical start address, report label, digits, writable.
// Default to statements for existing report/replay consumers. A table
// consumer defines RF_ZONE_FIELD_END as a comma before including this file.
#ifndef RF_ZONE_FIELD_END
#define RF_ZONE_FIELD_END ;
#define RF_ZONE_DEFAULT_FIELD_END
#endif
#define RF_ZONE_ITEM(type,start,label,digits,writable) \
    RF_ZONE_FIELD(type,start,label,digits,writable) RF_ZONE_FIELD_END



    RF_ZONE_ITEM(3, address(Field::F_X_MAX_FEEDRATE), Com::tEPRXMaxFeedrate, 3, true)
    RF_ZONE_ITEM(3, address(Field::F_X_HOMING_FEEDRATE_PRINT), Com::tEPRXHomingFeedrate, 3, true)
    RF_ZONE_ITEM(3, address(Field::F_Y_MAX_FEEDRATE), Com::tEPRYMaxFeedrate, 3, true)
    RF_ZONE_ITEM(3, address(Field::F_Y_HOMING_FEEDRATE_PRINT), Com::tEPRYHomingFeedrate, 3, true)
    RF_ZONE_ITEM(3, address(Field::F_Z_MAX_FEEDRATE), Com::tEPRZMaxFeedrate, 3, true)
    RF_ZONE_ITEM(3, address(Field::F_Z_HOMING_FEEDRATE_PRINT), Com::tEPRZHomingFeedrate, 3, true)

    RF_ZONE_ITEM(3, address(Field::F_X_MAX_ACCEL), Com::tEPRXAcceleration, 3, true)
    RF_ZONE_ITEM(3, address(Field::F_X_MAX_TRAVEL_ACCEL), Com::tEPRXTravelAcceleration, 3, true)
    RF_ZONE_ITEM(3, address(Field::F_Y_MAX_ACCEL), Com::tEPRYAcceleration, 3, true)
    RF_ZONE_ITEM(3, address(Field::F_Y_MAX_TRAVEL_ACCEL), Com::tEPRYTravelAcceleration, 3, true)
    RF_ZONE_ITEM(3, address(Field::F_Z_MAX_ACCEL), Com::tEPRZAcceleration, 3, true)
    RF_ZONE_ITEM(3, address(Field::F_Z_MAX_TRAVEL_ACCEL), Com::tEPRZTravelAcceleration, 3, true)

    RF_ZONE_ITEM(3, address(Field::F_MAX_XYJERK), Com::tEPRMaxXYJerk, 3, true)
    RF_ZONE_ITEM(3, address(Field::F_MAX_ZJERK), Com::tEPRMaxZJerk, 3, true)

    RF_ZONE_ITEM(0, address(Field::F_RF_MOTOR_CURRENT) + X_AXIS, Com::tEPRPrinter_STEPPER_X, 3, false)
    RF_ZONE_ITEM(0, address(Field::F_RF_MOTOR_CURRENT) + Y_AXIS, Com::tEPRPrinter_STEPPER_Y, 3, false)
    RF_ZONE_ITEM(0, address(Field::F_RF_MOTOR_CURRENT) + Z_AXIS, Com::tEPRPrinter_STEPPER_Z, 3, false)
    RF_ZONE_ITEM(0, address(Field::F_RF_MOTOR_CURRENT) + E_AXIS + 0, Com::tEPRPrinter_STEPPER_E0, 3, false)
#if NUM_EXTRUDER > 1
    RF_ZONE_ITEM(0, address(Field::F_RF_MOTOR_CURRENT) + E_AXIS + 1, Com::tEPRPrinter_STEPPER_E1, 3, false)
#endif //NUM_EXTRUDER > 1

    RF_ZONE_ITEM(3, address(Field::F_XAXIS_STEPS_PER_MM), Com::tEPRXStepsPerMM, 4, true)
    RF_ZONE_ITEM(3, address(Field::F_YAXIS_STEPS_PER_MM), Com::tEPRYStepsPerMM, 4, true)
    RF_ZONE_ITEM(3, address(Field::F_ZAXIS_STEPS_PER_MM), Com::tEPRZStepsPerMM, 4, true)
    RF_ZONE_ITEM(1, address(Field::F_RF_STEP_PACKING_MIN_INTERVAL), Com::tEPRInterruptSpacingInterval, 3, true)

    RF_ZONE_ITEM(3, address(Field::F_X_LENGTH), Com::tEPRXMaxLength, 3, true)
    RF_ZONE_ITEM(3, address(Field::F_Y_LENGTH), Com::tEPRYMaxLength, 3, true)
    RF_ZONE_ITEM(3, address(Field::F_Z_LENGTH), Com::tEPRZMaxLength, 3, true)

    RF_ZONE_ITEM(2, address(Field::F_RF_Z_OFFSET), Com::tEPRZOffset, 3, true)
    RF_ZONE_ITEM(0, address(Field::F_RF_Z_MODE), Com::tEPRZMode, 3, true)

    RF_ZONE_ITEM(1, address(Field::F_RF_MOD_Z_STEP_SIZE), Com::tEPRPrinterZ_STEP_SIZE, 3, true)

    RF_ZONE_ITEM(0, address(Field::F_RF_ZERO_DIGIT_STATE), Com::tEPRZERO_DIGIT_STATE, 3, true)
    RF_ZONE_ITEM(0, address(Field::F_RF_DIGIT_CMP_STATE), Com::tEPRZDIGIT_CMP_STATE, 3, true)

    RF_ZONE_ITEM(1, address(Field::F_RF_MOD_SENSEOFFSET_DIGITS), Com::tEPRPrinterEPR_RF_MOD_SENSEOFFSET_DIGITS, 3, true)
    RF_ZONE_ITEM(1, address(Field::F_RF_MOD_SENSEOFFSET_OFFSET_MAX), Com::tEPRPrinterMOD_SENSEOFFSET_OFFSET_MAX, 3, true)

    RF_ZONE_ITEM(2, address(Field::F_RF_EMERGENCYPAUSEDIGITSMIN), Com::tEPRPrinterEPR_RF_EmergencyPauseDigitsMin, 3, true)
    RF_ZONE_ITEM(2, address(Field::F_RF_EMERGENCYPAUSEDIGITSMAX), Com::tEPRPrinterEPR_RF_EmergencyPauseDigitsMax, 3, true)

    RF_ZONE_ITEM(1, address(Field::F_RF_EMERGENCYZSTOPDIGITSMIN), Com::tEPRPrinterEPR_RF_EmergencyStopAllMin, 3, true)
    RF_ZONE_ITEM(1, address(Field::F_RF_EMERGENCYZSTOPDIGITSMAX), Com::tEPRPrinterEPR_RF_EmergencyStopAllMax, 3, true)

    RF_ZONE_ITEM(0, address(Field::F_RF_MOD_ZOS_SCAN_POINT_X), Com::tEPRPrinterMOD_ZOS_SCAN_POINT_X, 3, true)
    RF_ZONE_ITEM(0, address(Field::F_RF_MOD_ZOS_SCAN_POINT_Y), Com::tEPRPrinterMOD_ZOS_SCAN_POINT_Y, 3, true)
    RF_ZONE_ITEM(3, address(Field::F_ZSCAN_START_MM), Com::tEPRZScanStartLift, 3, true)

    RF_ZONE_ITEM(0, address(Field::F_RF_HEATED_BED_SENSOR_TYPE), Com::tEPRBedsensorType, 3, true)
    RF_ZONE_ITEM(3, address(Field::F_BED_PID_PGAIN), Com::tEPRBedPGain, 3, true)
    RF_ZONE_ITEM(3, address(Field::F_BED_PID_IGAIN), Com::tEPRBedIGain, 3, true)
    RF_ZONE_ITEM(3, address(Field::F_BED_PID_DGAIN), Com::tEPRBedDGain, 3, true)
    RF_ZONE_ITEM(0, address(Field::F_BED_PID_MAX), Com::tEPRBedPIDMaxValue, 3, true)
    RF_ZONE_ITEM(0, address(Field::F_BED_DRIVE_MIN), Com::tEPRBedPIDDriveMin, 3, true)
    RF_ZONE_ITEM(0, address(Field::F_BED_DRIVE_MAX), Com::tEPRBedPIDDriveMax, 3, true)

    // Expand the two possible fitted tools at compile time. The shared
    // inventory can now also initialize immutable tables in Flash.
#define RF_ZONE_TOOL_FIELDS(i) \
    RF_ZONE_ITEM(3, address(Field::F_E0_STEPS_PER_MM) + uint16_t(i)*SlotSize, Com::tEPRStepsPerMM, 3, true) \
    RF_ZONE_ITEM(3, address(Field::F_E0_MAX_FEEDRATE) + uint16_t(i)*SlotSize, Com::tEPRMaxFeedrate, 3, true) \
    RF_ZONE_ITEM(3, address(Field::F_E0_MAX_ACCELERATION) + uint16_t(i)*SlotSize, Com::tEPRAcceleration, 3, true) \
    RF_ZONE_ITEM(3, address(Field::F_E0_MAX_START_FEEDRATE) + uint16_t(i)*SlotSize, Com::tEPReJerk, 3, true) \
    RF_ZONE_ITEM(3, address(Field::F_E0_ADVANCE_L) + uint16_t(i)*SlotSize, Com::tEPRAdvanceL, 3, true) \
    RF_ZONE_ITEM(0, address(Field::F_E0_SENSOR_TYPE) + uint16_t(i)*SlotSize, Com::tEPRsensorType, 3, true) \
    RF_ZONE_ITEM(3, address(Field::F_E0_PID_PGAIN) + uint16_t(i)*SlotSize, Com::tEPRPGain, 4, true) \
    RF_ZONE_ITEM(3, address(Field::F_E0_PID_IGAIN) + uint16_t(i)*SlotSize, Com::tEPRIGain, 4, true) \
    RF_ZONE_ITEM(3, address(Field::F_E0_PID_DGAIN) + uint16_t(i)*SlotSize, Com::tEPRDGain, 4, true) \
    RF_ZONE_ITEM(0, address(Field::F_E0_PID_MAX) + uint16_t(i)*SlotSize, Com::tEPRPIDMaxValue, 3, true) \
    RF_ZONE_ITEM(0, address(Field::F_E0_DRIVE_MIN) + uint16_t(i)*SlotSize, Com::tEPRDriveMin, 3, true) \
    RF_ZONE_ITEM(0, address(Field::F_E0_DRIVE_MAX) + uint16_t(i)*SlotSize, Com::tEPRDriveMax, 3, true) \
    RF_ZONE_ITEM(3, address(Field::F_E0_X_OFFSET) + uint16_t(i)*SlotSize, Com::tEPRXOffset, 3, true) \
    RF_ZONE_ITEM(3, address(Field::F_E0_Y_OFFSET) + uint16_t(i)*SlotSize, Com::tEPRYOffset, 3, true) \
    RF_ZONE_ITEM(0, address(Field::F_E0_COOLER_SPEED) + uint16_t(i)*SlotSize, Com::tEPRExtruderCoolerSpeed, 3, true)
    RF_ZONE_TOOL_FIELDS(0)
#if NUM_EXTRUDER > 1
    RF_ZONE_TOOL_FIELDS(1)
#endif
#undef RF_ZONE_TOOL_FIELDS

    RF_ZONE_ITEM(0, address(Field::F_RF_FAN_MODE), Com::tEPRPrinter_FAN_MODE, 3, true)
    RF_ZONE_ITEM(0, address(Field::F_RF_PART_FAN_SPEED), Com::tEPRPrinter_FAN_SPEED, 3, true)
    RF_ZONE_ITEM(0, address(Field::F_RF_PART_FAN_PWM_MIN), Com::tEPRPrinter_FAN_PART_FAN_PWM_MIN, 3, true)
    RF_ZONE_ITEM(0, address(Field::F_RF_PART_FAN_PWM_MAX), Com::tEPRPrinter_FAN_PART_FAN_PWM_MAX, 3, true)

#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    RF_ZONE_ITEM(0, address(Field::F_RF_FET1_MODE), Com::tEPRFET1Mode, 3, (FET1 >= 0 && !HOTEND_COOLER_USES_PIN(FET1)))
    RF_ZONE_ITEM(0, address(Field::F_RF_FET2_MODE), Com::tEPRFET2Mode, 3, (FET2 >= 0 && !HOTEND_COOLER_USES_PIN(FET2)))
    RF_ZONE_ITEM(0, address(Field::F_RF_FET3_MODE), Com::tEPRFET3Mode, 3, (FET3 >= 0 && CASE_FAN_PIN < 0 && !HOTEND_COOLER_USES_PIN(FET3)))
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2

    RF_ZONE_ITEM(0, address(Field::F_RF_CASE_LIGHT_MODE), Com::tEPRCaseLightsMode, 3, true)

#ifdef RGB_LIGHT_R_PIN
    RF_ZONE_ITEM(0, address(Field::F_RF_RGB_LIGHT_MODE), Com::tEPRRGBLightMode, 3, true)
#endif // RGB_LIGHT_R_PIN

    RF_ZONE_ITEM(2, address(Field::F_BAUDRATE), Com::tEPRBaudrate, 3, true)
    RF_ZONE_ITEM(2, address(Field::F_MAX_INACTIVE_TIME), Com::tEPRMaxInactiveTime, 3, true)
    RF_ZONE_ITEM(2, address(Field::F_STEPPER_INACTIVE_TIME), Com::tEPRStopAfterInactivty, 3, true)
    RF_ZONE_ITEM(1, address(Field::F_RF_PREHEAT_A_BED), zonePreheatABed, 3, true)
    RF_ZONE_ITEM(1, address(Field::F_RF_PREHEAT_A_HOTEND), zonePreheatAHotend, 3, true)
    RF_ZONE_ITEM(1, address(Field::F_RF_PREHEAT_B_BED), zonePreheatBBed, 3, true)
    RF_ZONE_ITEM(1, address(Field::F_RF_PREHEAT_B_HOTEND), zonePreheatBHotend, 3, true)
    // RF specific

    RF_ZONE_ITEM(3, address(Field::F_PRINTING_DISTANCE), Com::tEPRFilamentPrinted, 3, false)
    RF_ZONE_ITEM(2, address(Field::F_PRINTING_TIME), Com::tEPRPrinterActive, 3, false)


    // The limit may be reported; raw M206 is not commissioning authority.
    RF_ZONE_ITEM(0, address(Field::F_NEW_BED_EMERGENCY_C), zoneBedCutoff, 0, false)

#undef RF_ZONE_ITEM
#ifdef RF_ZONE_DEFAULT_FIELD_END
#undef RF_ZONE_DEFAULT_FIELD_END
#undef RF_ZONE_FIELD_END
#endif

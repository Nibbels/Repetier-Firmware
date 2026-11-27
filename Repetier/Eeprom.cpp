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
#include "StepExecutor.h"

#ifndef RF_EEPROM_ZONE_EXPERIMENT

// Persisted signature for the existing internal EEPROM layout.
constexpr uint8_t eepromMagic = 44;

// A changed algorithm requires a new stored tune, not a full settings reset.
// Bed is always slot 2, independent of the number of fitted extruders.
static constexpr uint8_t pidCalibrationTag = 0xA9;
static uint16_t pidCalibrationAddress(uint8_t heater) {
    return EPR_RF_PID_CALIBRATION + 3 * (heater == NUM_EXTRUDER ? 2 : heater);
}
static void writePidCalibrationTag(uint8_t heater) {
    auto& c = *tempController[heater];
    const uint16_t address = pidCalibrationAddress(heater);
    if (!c.pidCalibrationValid()) {
        HAL::eprSetByte(address, 0);
        return;
    }
    HAL::eprSetByte(address + 1, c.tunedSensorType);
    HAL::eprSetByte(address + 2, c.tunedPowerLimit);
    HAL::eprSetByte(address, pidCalibrationTag);
}
static bool readPidCalibrationTag(uint8_t heater, uint8_t version) {
    auto& c = *tempController[heater];
    const uint16_t address = pidCalibrationAddress(heater);
    const uint8_t tag = HAL::eprGetByte(address);
    if (version == EEPROM_PROTOCOL_VERSION && tag == pidCalibrationTag) {
        c.tunedSensorType = HAL::eprGetByte(address + 1);
        c.tunedPowerLimit = HAL::eprGetByte(address + 2);
        if (c.pidCalibrationValid()) {
            c.updateTempControlVars();
            return false;
        }
    }
    c.invalidatePidCalibration();
    c.updateTempControlVars();
    if (tag != 0) {
        HAL::eprSetByte(address, 0);
        return true;
    }
    return false;
}

void EEPROM::storePidCalibration(uint8_t heater) {
    if (heater >= NUM_TEMPERATURE_LOOPS)
        return;
    auto& c = *tempController[heater];
    const uint16_t address = pidCalibrationAddress(heater);
    HAL::eprSetByte(address, 0); // revoke marker before changing coefficients
    if (heater == NUM_EXTRUDER) {
        HAL::eprSetFloat(EPR_BED_PID_PGAIN, c.pidPGain);
        HAL::eprSetFloat(EPR_BED_PID_IGAIN, c.pidIGain);
        HAL::eprSetFloat(EPR_BED_PID_DGAIN, c.pidDGain);
        HAL::eprSetByte(EPR_BED_DRIVE_MAX, c.pidDriveMax);
        HAL::eprSetByte(EPR_BED_DRIVE_MIN, c.pidDriveMin);
        HAL::eprSetByte(EPR_BED_PID_MAX, c.pidMax);
        HAL::eprSetByte(EPR_RF_HEATED_BED_SENSOR_TYPE, c.sensorType);
    } else {
        const int o = getExtruderOffset(heater);
        HAL::eprSetFloat(o + EPR_EXTRUDER_PID_PGAIN, c.pidPGain);
        HAL::eprSetFloat(o + EPR_EXTRUDER_PID_IGAIN, c.pidIGain);
        HAL::eprSetFloat(o + EPR_EXTRUDER_PID_DGAIN, c.pidDGain);
        HAL::eprSetByte(o + EPR_EXTRUDER_DRIVE_MAX, c.pidDriveMax);
        HAL::eprSetByte(o + EPR_EXTRUDER_DRIVE_MIN, c.pidDriveMin);
        HAL::eprSetByte(o + EPR_EXTRUDER_PID_MAX, c.pidMax);
        HAL::eprSetByte(o + EPR_EXTRUDER_SENSOR_TYPE, c.sensorType);
    }
    writePidCalibrationTag(heater);
    updateChecksum();
}

#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
// Identify changed calibration identity/gains. DMIN/DMAX are operator
// fine tuning (same semantics as the LCD menu), not sensor/power identity.
static uint8_t rawThermalSlot(uint16_t pos) {
    if((pos>=EPR_BED_PID_PGAIN&&pos<=EPR_BED_PID_MAX)||
       pos==EPR_RF_HEATED_BED_SENSOR_TYPE)return 4;
    for(uint8_t slot=0;slot<2;++slot) {
        const uint16_t base=EEPROM_EXTRUDER_OFFSET+slot*EEPROM_EXTRUDER_LENGTH;
        if((pos>=base+EPR_EXTRUDER_PID_PGAIN&&pos<=base+EPR_EXTRUDER_PID_MAX)||
           pos==base+EPR_EXTRUDER_SENSOR_TYPE)
            return uint8_t(1u<<slot);
    }
    return 0;
}
// Validate thermal fields before revoking a previous calibration. Zero
// sensor/PWM is an explicit disabled state, not permission to heat.
static bool validHostSensor(int32_t value) {
    switch(value) {
    case 0: case 1: case 2: case 3: case 4: case 8: case 9:
    case 10: case 11: case 12: case 13: case 14: case 53: case 60: case 100:
        return true;
    default: return false;
    }
}
static bool validHostThermalValue(const GCode* com,uint16_t address) {
    if(address==EPR_RF_HEATED_BED_SENSOR_TYPE)return validHostSensor(com->S);
    if(address==EPR_BED_PID_MAX)return com->S==0||com->S>=40;
    if(address==EPR_BED_DRIVE_MIN||address==EPR_BED_DRIVE_MAX)return com->S>=1;
    if(address==EPR_RF_PREHEAT_A_BED||address==EPR_RF_PREHEAT_B_BED)
        return com->S==0||(com->S>=UI_SET_MIN_HEATED_BED_TEMP&&com->S<=UI_SET_MAX_HEATED_BED_TEMP&&com->S<=heatedBedController.maximumTargetC());
    if(address==EPR_RF_PREHEAT_A_HOTEND||address==EPR_RF_PREHEAT_B_HOTEND)
        return com->S==0||(com->S>=UI_SET_MIN_EXTRUDER_TEMP&&com->S<=UI_SET_MAX_EXTRUDER_TEMP);
    for(uint8_t slot=0;slot<NUM_EXTRUDER;++slot) {
        const uint16_t base=EEPROM_EXTRUDER_OFFSET+slot*EEPROM_EXTRUDER_LENGTH;
        if(address==base+EPR_EXTRUDER_SENSOR_TYPE)return validHostSensor(com->S);
        if(address==base+EPR_EXTRUDER_PID_MAX)return com->S==0||com->S>=40;
        if(address==base+EPR_EXTRUDER_DRIVE_MIN||address==base+EPR_EXTRUDER_DRIVE_MAX)return com->S>=1;
    }
    return true; // Nonthermal field-specific checks are handled by their owners.
}
// Use established loader/menu bounds; do not silently repair a proposed
// out-of-range value after writing it. This is not mechanical qualification.
static bool validHostNonthermalValue(const GCode* c,uint16_t p) {
    const float x=c->T==3?c->X:0;
    const int32_t v=c->T==3?0:c->S;
    if(p==EPR_X_MAX_FEEDRATE||p==EPR_Y_MAX_FEEDRATE||p==EPR_Z_MAX_FEEDRATE||
       p==EPR_X_HOMING_FEEDRATE_PRINT||p==EPR_Y_HOMING_FEEDRATE_PRINT||p==EPR_Z_HOMING_FEEDRATE_PRINT) {
        const bool home=p>=EPR_X_HOMING_FEEDRATE_PRINT;
        const uint8_t axis=uint8_t((p-(home?EPR_X_HOMING_FEEDRATE_PRINT:EPR_X_MAX_FEEDRATE))/4);
        const float limit=axis==X_AXIS?MAX_FEEDRATE_X:axis==Y_AXIS?MAX_FEEDRATE_Y:MAX_FEEDRATE_Z;
        if(!(x>0&&x<=limit))return false;
        // Homing is never faster than the proposed/current axis maximum.
        return home?x<=HAL::eprGetFloat(EPR_X_MAX_FEEDRATE+4*axis)
                   :x>=HAL::eprGetFloat(EPR_X_HOMING_FEEDRATE_PRINT+4*axis);
    }
    if(p==EPR_X_MAX_ACCEL||p==EPR_Y_MAX_ACCEL||p==EPR_X_MAX_TRAVEL_ACCEL||p==EPR_Y_MAX_TRAVEL_ACCEL)
        return x>=ACCELERATION_MIN_XY&&x<=ACCELERATION_MAX_XY;
    if(p==EPR_Z_MAX_ACCEL||p==EPR_Z_MAX_TRAVEL_ACCEL)return x>=ACCELERATION_MIN_Z&&x<=ACCELERATION_MAX_Z;
    if(p==EPR_MAX_XYJERK)return x>=1&&x<=33.3f;
    if(p==EPR_MAX_ZJERK)return x>=.05f&&x<=2;
    if(p==EPR_XAXIS_STEPS_PER_MM||p==EPR_YAXIS_STEPS_PER_MM||p==EPR_ZAXIS_STEPS_PER_MM)
        return x>0; // Upper calibration envelope still needs joint timing/travel analysis.
    if(p==EPR_X_LENGTH)return x>0&&x<=245;
    if(p==EPR_Y_LENGTH||p==EPR_Z_LENGTH)return x>0; // Retrofit travel envelope not yet qualified.
    if(p==EPR_MAX_INACTIVE_TIME||p==EPR_STEPPER_INACTIVE_TIME)return v>=0;
    if(p==EPR_RF_STEP_PACKING_MIN_INTERVAL)return v>=MIN_STEP_PACKING_MIN_INTERVAL&&v<=MAX_STEP_PACKING_MIN_INTERVAL;
    if(p==EPR_ZSCAN_START_MM)return x>=.3f&&x<=6;
    if(p==EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX)return v>=1&&v<=300;
    if(p==EPR_RF_MOD_SENSEOFFSET_DIGITS)return v>=500&&v<=EMERGENCY_PAUSE_DIGITS_MAX;
    if(p==EPR_RF_EMERGENCYPAUSEDIGITSMIN)
        return v>=EMERGENCY_PAUSE_DIGITS_MIN&&v<=EMERGENCY_PAUSE_DIGITS_MAX&&v<=HAL::eprGetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMAX);
    if(p==EPR_RF_EMERGENCYPAUSEDIGITSMAX)
        return v>=EMERGENCY_PAUSE_DIGITS_MIN&&v<=EMERGENCY_PAUSE_DIGITS_MAX&&v>=HAL::eprGetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMIN);
    if(p==EPR_RF_EMERGENCYZSTOPDIGITSMIN)
        return v>=EMERGENCY_STOP_DIGITS_MIN&&v<=EMERGENCY_STOP_DIGITS_MAX&&v!=0&&v<=HAL::eprGetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMAX);
    if(p==EPR_RF_EMERGENCYZSTOPDIGITSMAX)
        return v>=EMERGENCY_STOP_DIGITS_MIN&&v<=EMERGENCY_STOP_DIGITS_MAX&&v!=0&&v>=HAL::eprGetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMIN);
    if(p==EPR_RF_FAN_MODE||p==EPR_RF_CASE_LIGHT_MODE||p==EPR_RF_FET1_MODE||p==EPR_RF_FET2_MODE||p==EPR_RF_FET3_MODE)return v<=1;
    if(p==EPR_RF_PART_FAN_SPEED)return v>=1&&v<=PART_FAN_MODE_MAX;
    if(p==EPR_RF_PART_FAN_PWM_MIN)return v>=1&&v+16<=HAL::eprGetByte(EPR_RF_PART_FAN_PWM_MAX);
    if(p==EPR_RF_PART_FAN_PWM_MAX)return v<=254&&v>=16+HAL::eprGetByte(EPR_RF_PART_FAN_PWM_MIN);
    if(p==EPR_RF_ZERO_DIGIT_STATE||p==EPR_RF_DIGIT_CMP_STATE)return v==1||v==2;
    for(uint8_t slot=0;slot<NUM_EXTRUDER;++slot) {
        const uint16_t base=EEPROM_EXTRUDER_OFFSET+slot*EEPROM_EXTRUDER_LENGTH;
        if(p==base+EPR_EXTRUDER_STEPS_PER_MM)return x>0&&x<5540;
        if(p==base+EPR_EXTRUDER_MAX_FEEDRATE)return x>0&&x<100&&x>=HAL::eprGetFloat(base+EPR_EXTRUDER_MAX_START_FEEDRATE);
        if(p==base+EPR_EXTRUDER_MAX_START_FEEDRATE)return x>0&&x<=HAL::eprGetFloat(base+EPR_EXTRUDER_MAX_FEEDRATE);
        if(p==base+EPR_EXTRUDER_MAX_ACCELERATION)return x>0&&x<10000;
        if(p==base+EPR_EXTRUDER_ADVANCE_L)return x==0||(x>=20&&x<=250);
    }
    return true; // Remaining calibration/enum/baud fields need owner-specific audit.
}
static bool prepareRawThermalWrite(GCode* com,uint8_t width) {
    // Caller validated the full signed address and write width first.
    // Preserve no-op M206 transfers from host configuration tools.
    if(!(com->T==3?com->hasX():com->hasS()))return true;
    uint32_t bits=0;
    if(com->T==3) {
        static_assert(sizeof(float)==sizeof(bits),"EEPROM float encoding");
        memcpy(&bits,&com->X,sizeof(bits));
    }else bits=uint32_t(com->S);
    uint8_t slots=0;
    const uint16_t pos=uint16_t(com->P);
    for(uint8_t i=0;i<width;++i) {
        const uint8_t slot=rawThermalSlot(pos+i);
        if(slot&&HAL::eprGetByte(pos+i)!=uint8_t(bits>>(8*i)))slots|=slot;
    }
    // Stop any affected fitted heater before EEPROM mutation; the absent
    // E1 slot still has a persistent calibration marker to revoke.
    for(uint8_t slot=0;slot<3;++slot)if(slots&(1u<<slot)) {
        if(slot==2||slot<NUM_EXTRUDER)
            tempController[slot==2?NUM_EXTRUDER:slot]->invalidatePidCalibration();
    }
    for(uint8_t slot=0;slot<3;++slot)if(slots&(1u<<slot)) {
        const uint16_t marker=EPR_RF_PID_CALIBRATION+3*slot;
        HAL::eprSetByte(marker,0);
        if(HAL::eprGetByte(marker)!=0) {
            Com::printErrorFLN(PSTR("PID calibration revoke failed"));
            return false;
        }
    }
    return true;
}
#endif

void EEPROM::update(GCode* com) {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    // A public settings field must match the exact M205 start and wire type.
    // Validate before narrowing, reading or writing any EEPROM address.
    if(!com->hasP()||!com->hasT()||com->P<0||com->P>=2048||com->T>3||
       !(com->T==3?com->hasX():com->hasS())) {
        Com::printErrorFLN(PSTR("Invalid EEPROM setting command"));
        return;
    }
    const uint16_t address=uint16_t(com->P);
    bool listed=false, writable=false;
#define RF_EPR_FIELD(type, pos, label, digits, access) \
    if(address==(pos)&&com->T==(type)){listed=true;writable=(access);}
#include "EepromHostFields.h"
#undef RF_EPR_FIELD
    if(!listed) {
        Com::printErrorFLN(PSTR("Unknown EEPROM setting or type"));
        return;
    }
    if((com->T==0&&(com->S<0||com->S>255))||
       (com->T==1&&(com->S<-32768L||com->S>32767L))||
       (com->T==3&&!isfinite(com->X))) {
        Com::printErrorFLN(PSTR("Invalid EEPROM value"));
        return;
    }
    const uint8_t bytes=com->T==0?1:com->T==1?2:4;
    uint32_t bits=0;
    if(com->T==3)memcpy(&bits,&com->X,4);
    else bits=uint32_t(com->S);
    bool changed=false;
    for(uint8_t i=0;i<bytes;++i)changed|=HAL::eprGetByte(address+i)!=uint8_t(bits>>(8*i));
    // Exact read-only echoes are harmless; skip checksum/reload for all
    // exact no-ops, including live DMAX adjustment already saved by LCD.
    if(!changed)return;
    if(!writable) {
        Com::printErrorFLN(PSTR("EEPROM setting is read-only"));
        return;
    }
    if(!validHostThermalValue(com,address)||!validHostNonthermalValue(com,address)) {
        Com::printErrorFLN(PSTR("Invalid EEPROM setting value"));
        return;
    }
#endif
    // Calibration authorization is not a user-editable EEPROM setting.
    const int width = com->hasT() && com->T == 0 ? 1 : (com->hasT() && com->T == 1 ? 2 : 4);
#ifndef RF_THERMAL_EMERGENCY_EXPERIMENT
    // The legacy comparison build lacks the exact typed whitelist.
    // Reject narrowing aliases and every overlapping motor-current write.
    if(com->hasP() && (com->P<0 || com->P>2048-width)) {
        Com::printErrorFLN(PSTR("EEPROM address outside settings"));
        return;
    }
    if(com->hasP() && com->P>=EPR_RF_MOTOR_CURRENT-width+1 && com->P<EPR_RF_MOTOR_CURRENT+5) {
        Com::printErrorFLN(PSTR("Motor current is menu-only"));
        return;
    }
    // The retired baud preference is not writable in the raw comparison build.
    if(com->hasP() && com->P<EPR_BAUDRATE+4 && com->P+width>EPR_BAUDRATE) {
        Com::printErrorFLN(PSTR("Firmware serial baudrate is fixed"));
        return;
    }
    // P/I/D are owned by autotune, even in the raw comparison build.
    if(com->hasP()) {
        const int32_t end=com->P+width;
        bool gainWrite=com->P<EPR_BED_PID_DGAIN+4 && end>EPR_BED_PID_PGAIN;
        for(uint8_t tool=0;tool<2;++tool) {
            const uint16_t base=EEPROM_EXTRUDER_OFFSET+tool*EEPROM_EXTRUDER_LENGTH;
            gainWrite|=com->P<base+EPR_EXTRUDER_PID_DGAIN+4 && end>base+EPR_EXTRUDER_PID_PGAIN;
        }
        if(gainWrite) {
            Com::printErrorFLN(PSTR("PID gains are autotune-only"));
            return;
        }
    }
#endif
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    // M206 P is int32_t but HAL takes a 16-bit unsigned AVR address.
    // Reject the whole span before narrowing; upper-bank research records
    // are not part of the existing 2048-byte public settings ABI.
    if(com->hasT()&&com->hasP()&&com->T<=3) {
        if(com->P<0||com->P>2048-width) {
            Com::printErrorFLN(PSTR("EEPROM address outside settings"));
            return;
        }
    }
#endif
    if (com->hasP() && com->P >= EPR_RF_PID_CALIBRATION - width + 1 && com->P < EPR_RF_PID_CALIBRATION + 9) {
        Com::printErrorFLN(PSTR("PID calibration record is read-only"));
        return;
    }
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(com->hasT()&&com->hasP()&&com->T<=3) {
        const int32_t end=com->P+width;
        // Internal version, integrity and fitted-tool metadata must remain
        // owned by EEPROM migration, not be forged with raw writes.
        if(com->P==EPR_MAGIC_BYTE||
           (com->P<=EPR_VERSION&&end>EPR_INTEGRITY_BYTE)||
           (com->P<=EPR_CHECK_NUM_EXTRUDERS&&end>EPR_CHECK_NUM_EXTRUDERS)||
           (com->P<EPR_BAUDRATE+4&&end>EPR_BAUDRATE)) {
            Com::printErrorFLN(PSTR("EEPROM metadata is read-only"));
            return;
        }
        if(!prepareRawThermalWrite(com,uint8_t(width)))return;
    }
#endif
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
        e->tempControl.invalidatePidCalibration();
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
        e->tempControl.invalidatePidCalibration();
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
    Printer::preheatProfiles.restoreDefaults();
    Printer::inactivity.setShutdownTimeout(MAX_INACTIVE_TIME * 1000L);
    Printer::inactivity.setStepperTimeout(STEPPER_INACTIVE_TIME * 1000L);
    Printer::setStepsPerMm(X_AXIS, XAXIS_STEPS_PER_MM);
    Printer::setStepsPerMm(Y_AXIS, YAXIS_STEPS_PER_MM);
    Printer::setStepsPerMm(Z_AXIS, ZAXIS_STEPS_PER_MM);
    Printer::zOffset.refreshSteps(Printer::stepsPerMm(Z_AXIS));
    Printer::setStepsPerMm(E_AXIS, 1); // Man könnte auch vom current extruder die id auslesen und dann EXT0_STEPS_PER_MM oder EXT1_STEPS_PER_MM ?
                                         // -> ist autokorrigiert wenn man einmal einen extruder auswählt. Siehe unten.
    Printer::setAxisMaxFeedrate(X_AXIS, MAX_FEEDRATE_X);
    Printer::setAxisMaxFeedrate(Y_AXIS, MAX_FEEDRATE_Y);
    Printer::setAxisMaxFeedrate(Z_AXIS, MAX_FEEDRATE_Z);

    // Microsteps should be updated but we need to update the driver then.
    bool ms_changed = false;
    for (int ax = 0; ax <= E_AXIS + 1; ax++) {
        if (Drv8711::microstepMode(ax) != drv8711MicroSteps_2_ModeValue(drv8711Axis_2_InitMicrosteps(ax))) {
            if (!ms_changed) {
                ms_changed = true;
                Printer::disableAllSteppersNow();               // Stepper und Homing ausmachen.
                                                                // We cannot use the old coordinates anymore.
                HAL::eprSetByte(EPR_RF_MICRO_STEPS_USED, 0x00); // Make all Microstep eeprom settings invalid for next boot.
            }
            Drv8711::setMicrostepMode(ax, drv8711MicroSteps_2_ModeValue(drv8711Axis_2_InitMicrosteps(ax)));
        }
    }

        Printer::setAxisHomingFeedrate(X_AXIS, HOMING_FEEDRATE_X_PRINT);
        Printer::setAxisHomingFeedrate(Y_AXIS, HOMING_FEEDRATE_Y_PRINT);
        Printer::setAxisHomingFeedrate(Z_AXIS, HOMING_FEEDRATE_Z_PRINT);

    Printer::setXYJerkLimit(MAX_JERK);
    Printer::setZJerkLimit(MAX_ZJERK);

    Printer::manualMotion.setMode(X_AXIS, DEFAULT_MOVE_MODE_X);
    Printer::manualMotion.setMode(Y_AXIS, DEFAULT_MOVE_MODE_Y);
    Printer::manualMotion.setMode(Z_AXIS, DEFAULT_MOVE_MODE_Z);
    Printer::manualMotion.setCoordinateSystem(KOSYS_GCODE);
    Printer::manualMotion.setFeedrateSource(FEEDRATE_DIRECTCONFIG);

    Printer::setPrintAcceleration(X_AXIS, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X);
    Printer::setPrintAcceleration(Y_AXIS, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y);
    Printer::setPrintAcceleration(Z_AXIS, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z);
    Printer::setTravelAcceleration(X_AXIS, MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X);
    Printer::setTravelAcceleration(Y_AXIS, MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y);
    Printer::setTravelAcceleration(Z_AXIS, MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z);

    heatedBedController.invalidatePidCalibration();
    heatedBedController.pidDriveMax = HEATED_BED_PID_INTEGRAL_DRIVE_MAX;
    heatedBedController.pidDriveMin = HEATED_BED_PID_INTEGRAL_DRIVE_MIN;
    heatedBedController.pidPGain = HEATED_BED_PID_PGAIN;
    heatedBedController.pidIGain = HEATED_BED_PID_IGAIN;
    heatedBedController.pidDGain = HEATED_BED_PID_DGAIN;
    heatedBedController.pidMax = HEATED_BED_PID_MAX;
    heatedBedController.sensorType = HEATED_BED_SENSOR_TYPE;

        Printer::setAxisLengthMm(X_AXIS, X_MAX_LENGTH_PRINT);

    Printer::setAxisLengthMm(Y_AXIS, Y_MAX_LENGTH);
    Printer::setAxisLengthMm(Z_AXIS, Z_MAX_LENGTH);

    EEPROM::restoreEEPROMExtruderSettingsFromConfiguration(0);
#if NUM_EXTRUDER > 1
    EEPROM::restoreEEPROMExtruderSettingsFromConfiguration(1);
#endif // NUM_EXTRUDER > 1

    /* TODO : Restliche Parameter neu einlesen. ....
    ich glaube gesehen zu haben, dass acceleration und feedrates nicht neu eingelesen werden. will man das?
    */

    StepExecutor::setPackingInterval(STEP_PACKING_MIN_INTERVAL);

    Printer::restoreZDisplayMode(DEFAULT_Z_SCALE_MODE); //wichtig, weils im Mod einen dritten Mode gibt. Für Zurückmigration

#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    // Hier nur config laden nicht mosfets schalten!
    AccessoryOutputs::stage(AccessoryOutputs::Output::Mains, 0);

    // Hier nur config laden nicht mosfets schalten!
    AccessoryOutputs::stage(AccessoryOutputs::Output::Fet1, FET1_DEFAULT_ON);
    AccessoryOutputs::stage(AccessoryOutputs::Output::Fet2, FET2_DEFAULT_ON);
    AccessoryOutputs::stage(AccessoryOutputs::Output::Fet3, FET3_DEFAULT_ON);
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2

    restoreZOffsetScanPoint({defaultZOffsetScanIndex, defaultZOffsetScanIndex});

    Printer::senseOffsetControl.setMaxMicrometers(SENSIBLE_PRESSURE_MAX_OFFSET);
    Printer::senseOffsetControl.setAutoStart(false);

    Printer::wobbleCorrection.setProfile({});

    Printer::pressureProtection.setPauseLimits({EMERGENCY_PAUSE_DIGITS_MIN, EMERGENCY_PAUSE_DIGITS_MAX});
    Printer::pressureProtection.setStopLimits({EMERGENCY_STOP_DIGITS_MIN, EMERGENCY_STOP_DIGITS_MAX});


    static const unsigned short uMotorCurrentUse[] PROGMEM = MOTOR_CURRENT_NORMAL; //Standardwert
    for (uint8_t stp = 0; stp < 3 + NUM_EXTRUDER; stp++) {          //0..4 bei 5 steppern.
        Drv8711::stageCurrent(stp, pgm_read_word(&uMotorCurrentUse[stp])); // Restore configuration without a register write.
    }

    Printer::pause.restoreDefaultParking();

    Printer::updateDerivedParameter();
    Extruder::applyCurrentSettings();
    Printer::applyAdvanceSettings();
    Extruder::initHeatedBed();

    Printer::manualMotion.setSteps(X_AXIS, (unsigned short)lroundf(Printer::stepsPerMm(X_AXIS) * DEFAULT_MANUAL_MM_X));
    Printer::manualMotion.setSteps(Y_AXIS, (unsigned short)lroundf(Printer::stepsPerMm(Y_AXIS) * DEFAULT_MANUAL_MM_Y));
    Printer::manualMotion.setSteps(Z_AXIS, (unsigned short)lroundf(Printer::stepsPerMm(Z_AXIS) * DEFAULT_MANUAL_MM_Z));
    Printer::manualMotion.setSteps(E_AXIS, (unsigned short)lroundf(Extruder::current->stepsPerMM * DEFAULT_MANUAL_MM_E));

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
    // Frozen legacy comparison image only. Never used to configure UART.
    HAL::eprSetInt32(EPR_BAUDRATE, 115200L);
    HAL::eprSetInt32(EPR_MAX_INACTIVE_TIME, Printer::inactivity.shutdownTimeout());
    HAL::eprSetInt32(EPR_STEPPER_INACTIVE_TIME, Printer::inactivity.stepperTimeout());
    HAL::eprSetFloat(EPR_XAXIS_STEPS_PER_MM, Printer::stepsPerMm(X_AXIS));
    HAL::eprSetFloat(EPR_YAXIS_STEPS_PER_MM, Printer::stepsPerMm(Y_AXIS));
    HAL::eprSetFloat(EPR_ZAXIS_STEPS_PER_MM, Printer::stepsPerMm(Z_AXIS));
    HAL::eprSetFloat(EPR_X_MAX_FEEDRATE, Printer::axisMaxFeedrate(X_AXIS));
    HAL::eprSetFloat(EPR_Y_MAX_FEEDRATE, Printer::axisMaxFeedrate(Y_AXIS));
    HAL::eprSetFloat(EPR_Z_MAX_FEEDRATE, Printer::axisMaxFeedrate(Z_AXIS));
    HAL::eprSetInt32(EPR_RF_Z_OFFSET, Printer::zOffset.manualMicrometers());
    HAL::eprSetByte(EPR_RF_Z_MODE, Printer::zDisplayMode());
    HAL::eprSetByte(EPR_RF_MOVE_MODE_X, Printer::manualMotion.mode(X_AXIS));
    HAL::eprSetByte(EPR_RF_MOVE_MODE_Y, Printer::manualMotion.mode(Y_AXIS));
    HAL::eprSetByte(EPR_RF_MOVE_MODE_Z, Printer::manualMotion.mode(Z_AXIS));
    HAL::eprSetByte(EPR_RF_MOVE_MODE_XY_KOSYS, Printer::manualMotion.coordinateSystem());
    HAL::eprSetByte(EPR_RF_MOVE_POSITION_FEEDRATE, Printer::manualMotion.feedrateSource());

    HAL::eprSetFloat(EPR_X_HOMING_FEEDRATE_PRINT, Printer::axisHomingFeedrate(X_AXIS));
    HAL::eprSetFloat(EPR_Y_HOMING_FEEDRATE_PRINT, Printer::axisHomingFeedrate(Y_AXIS));
    HAL::eprSetFloat(EPR_Z_HOMING_FEEDRATE_PRINT, Printer::axisHomingFeedrate(Z_AXIS));

    HAL::eprSetFloat(EPR_MAX_XYJERK, Printer::xyJerkLimit());
    HAL::eprSetFloat(EPR_MAX_ZJERK, Printer::zJerkLimit());

    HAL::eprSetFloat(EPR_X_MAX_ACCEL, Printer::printAcceleration(X_AXIS));
    HAL::eprSetFloat(EPR_Y_MAX_ACCEL, Printer::printAcceleration(Y_AXIS));
    HAL::eprSetFloat(EPR_Z_MAX_ACCEL, Printer::printAcceleration(Z_AXIS));
    HAL::eprSetFloat(EPR_X_MAX_TRAVEL_ACCEL, Printer::travelAcceleration(X_AXIS));
    HAL::eprSetFloat(EPR_Y_MAX_TRAVEL_ACCEL, Printer::travelAcceleration(Y_AXIS));
    HAL::eprSetFloat(EPR_Z_MAX_TRAVEL_ACCEL, Printer::travelAcceleration(Z_AXIS));

    HAL::eprSetByte(EPR_BED_DRIVE_MAX, heatedBedController.pidDriveMax);
    HAL::eprSetByte(EPR_BED_DRIVE_MIN, heatedBedController.pidDriveMin);
    HAL::eprSetFloat(EPR_BED_PID_PGAIN, heatedBedController.pidPGain);
    HAL::eprSetFloat(EPR_BED_PID_IGAIN, heatedBedController.pidIGain);
    HAL::eprSetFloat(EPR_BED_PID_DGAIN, heatedBedController.pidDGain);
    HAL::eprSetByte(EPR_BED_PID_MAX, heatedBedController.pidMax);
    HAL::eprSetByte(EPR_RF_HEATED_BED_SENSOR_TYPE, heatedBedController.sensorType);
    writePidCalibrationTag(NUM_EXTRUDER);

        HAL::eprSetFloat(EPR_X_LENGTH, Printer::axisLengthMm(X_AXIS));
    HAL::eprSetFloat(EPR_Y_LENGTH, Printer::axisLengthMm(Y_AXIS));
    HAL::eprSetFloat(EPR_Z_LENGTH, Printer::axisLengthMm(Z_AXIS));

    // now the extruder
    for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
        storeExtruderDataIntoEEPROM(i);
    }
    HAL::eprSetInt16(EPR_RF_STEP_PACKING_MIN_INTERVAL, StepExecutor::packingInterval());

    HAL::eprSetByte(EPR_RF_FAN_MODE, HAL::partFanMode());

    HAL::eprSetByte(EPR_RF_PART_FAN_SPEED, HAL::partFanDivisor());
    HAL::eprSetByte(EPR_RF_PART_FAN_PWM_MIN, HAL::partFanMinDuty());
    HAL::eprSetByte(EPR_RF_PART_FAN_PWM_MAX, HAL::partFanMaxDuty());

    if (corrupted) {
        HAL::eprSetInt32(EPR_PRINTING_TIME, 0);
        HAL::eprSetFloat(EPR_PRINTING_DISTANCE, 0);
    }

    HAL::eprSetByte(EPR_RF_CASE_LIGHT_MODE, AccessoryOutputs::state(AccessoryOutputs::Output::CaseLight));


#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    HAL::eprSetByte(EPR_RF_FET1_MODE, AccessoryOutputs::state(AccessoryOutputs::Output::Fet1));
    HAL::eprSetByte(EPR_RF_FET2_MODE, AccessoryOutputs::state(AccessoryOutputs::Output::Fet2));
    HAL::eprSetByte(EPR_RF_FET3_MODE, AccessoryOutputs::state(AccessoryOutputs::Output::Fet3));

    // after a power-on, the 230 V plug always shall be turned off - thus, we do not store this setting to the EEPROM
    // HAL::eprSetByte( EPR_RF_230V_OUTPUT_MODE, AccessoryOutputs::state(AccessoryOutputs::Output::Mains) );
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2

    HAL::eprSetInt16(EPR_RF_MOD_Z_STEP_SIZE, Printer::manualMotion.steps(Z_AXIS));
    const auto scanPoint = zOffsetScanPoint();
    HAL::eprSetByte(EPR_RF_MOD_ZOS_SCAN_POINT_X, scanPoint.x);
    HAL::eprSetByte(EPR_RF_MOD_ZOS_SCAN_POINT_Y, scanPoint.y);
    // Do not update EPR_RF_MOD_SENSEOFFSET_DIGITS here
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

    HAL::eprSetByte(EPR_RF_MOTOR_CURRENT + X_AXIS, Drv8711::configuredCurrent(X_AXIS));
    HAL::eprSetByte(EPR_RF_MOTOR_CURRENT + Y_AXIS, Drv8711::configuredCurrent(Y_AXIS));
    HAL::eprSetByte(EPR_RF_MOTOR_CURRENT + Z_AXIS, Drv8711::configuredCurrent(Z_AXIS));
    HAL::eprSetByte(EPR_RF_MOTOR_CURRENT + E_AXIS + 0, Drv8711::configuredCurrent(E_AXIS + 0));
#if NUM_EXTRUDER > 1
    HAL::eprSetByte(EPR_RF_MOTOR_CURRENT + E_AXIS + 1, Drv8711::configuredCurrent(E_AXIS + 1));
#endif //NUM_EXTRUDER > 1

    HAL::eprSetByte(EPR_RF_ZERO_DIGIT_STATE, Printer::strainGauge.zeroingEnabled() ? 1 : 2);
    HAL::eprSetByte(EPR_RF_DIGIT_CMP_STATE, Printer::digitCompensation.digitZEnabled() ? 1 : 2);

    HAL::eprSetFloat(EPR_ZSCAN_START_MM, scanStartLiftMm());
    writePreheatProfiles();

    // Save version and build checksum
    HAL::eprSetByte(EPR_VERSION, EEPROM_PROTOCOL_VERSION);
    EEPROM::updateChecksum();
    publishSenseOffsetSettings();

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
    writePidCalibrationTag(extruderId);

    HAL::eprSetFloat(o + EPR_EXTRUDER_X_OFFSET, e->offsetMM[X_AXIS]);
    HAL::eprSetFloat(o + EPR_EXTRUDER_Y_OFFSET, e->offsetMM[Y_AXIS]);


    HAL::eprSetByte(o + EPR_EXTRUDER_COOLER_SPEED, e->coolerSpeed);

    HAL::eprSetFloat(o + EPR_EXTRUDER_ADVANCE_L, e->advanceL);
}

void EEPROM::updateChecksum() {
    // Ändert sich die checksumme nicht, wird nicht geschrieben. doppelprüfung ist überflüssig.
    // Do not truncate computeChecksum on a thermal emergency: callers
    // have already written the data, and a partial sum would corrupt the
    // integrity record. The checksum scan continues yielding to the
    // emergency scheduler; ordinary work must unwind after it returns.
    HAL::eprSetByte(EPR_INTEGRITY_BYTE, computeChecksum());
} // updateChecksum

void EEPROM::storeMicrostepConfiguration(uint8_t changedMask) {
    // All driver and motion state is already published by the foreground
    // caller. EEPROM-ready waits belong outside its interrupt guard.
    const bool initialize = HAL::eprGetByte(EPR_RF_MICRO_STEPS_USED) != 0xAB;
    if (!changedMask && !initialize)
        return;

    for (uint8_t axis = 0; axis < DRV8711_NUM_CHANNELS; ++axis) {
        const bool changed = (changedMask & (1U << axis)) != 0;
        if (initialize || changed)
            HAL::eprSetByte(EPR_RF_MICRO_STEPS_X + axis, Drv8711::microstepMode(axis));
        if (!changed)
            continue;
        if (axis < E_AXIS) {
            HAL::eprSetFloat(EPR_XAXIS_STEPS_PER_MM + 4 * axis, Printer::stepsPerMm(axis));
            if (axis == Z_AXIS)
                HAL::eprSetInt16(EPR_RF_MOD_Z_STEP_SIZE, Printer::manualMotion.steps(Z_AXIS));
        } else if (axis - E_AXIS < NUM_EXTRUDER) {
            const uint8_t tool = axis - E_AXIS;
            HAL::eprSetFloat(getExtruderOffset(tool) + EPR_EXTRUDER_STEPS_PER_MM, extruder[tool].stepsPerMM);
        }
    }
    if (initialize)
        HAL::eprSetByte(EPR_RF_MICRO_STEPS_USED, 0xAB);
    updateChecksum();
}

void EEPROM::storeManualZOffset() {
    const int32_t manual = Printer::zOffset.manualMicrometers();
    if (HAL::eprGetInt32(EPR_RF_Z_OFFSET) != manual) {
        HAL::eprSetInt32(EPR_RF_Z_OFFSET, manual);
        updateChecksum();
    }
}

#if defined(__AVR__) && defined(RF_THERMAL_EMERGENCY_EXPERIMENT)
// The experimental watchdog recovery entry is fixed at byte 0x20000.
// Keep this cold, foreground-only settings loader outside the first LTO
// .text block; .text.* is placed after the recovery entry by recovery-link.py.
// No change to near PROGMEM placement or EEPROM contents. Linked evidence
// must still pass all recovery, address and resource checks.
__attribute__((noinline, section(".text.rf_cold_eeprom")))
#endif
void EEPROM::readDataFromEEPROM() {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(Printer::printJob.emergencyLocked())return;
#endif
    bool change = readPreheatProfiles();

    uint8_t version = HAL::eprGetByte(EPR_VERSION); // This is the saved version. Don't copy data not set in older versions!
    Printer::inactivity.setShutdownTimeout(HAL::eprGetInt32(EPR_MAX_INACTIVE_TIME));
    Printer::inactivity.setStepperTimeout(HAL::eprGetInt32(EPR_STEPPER_INACTIVE_TIME));
    Printer::setStepsPerMm(X_AXIS, HAL::eprGetFloat(EPR_XAXIS_STEPS_PER_MM));
    Printer::setStepsPerMm(Y_AXIS, HAL::eprGetFloat(EPR_YAXIS_STEPS_PER_MM));
    Printer::setStepsPerMm(Z_AXIS, HAL::eprGetFloat(EPR_ZAXIS_STEPS_PER_MM));

    // Check da früher mehr speed erlaubt, was keinen sinn gemacht hat.
    Printer::setAxisMaxFeedrate(X_AXIS, MAX_FEEDRATE_X);
    Printer::setAxisMaxFeedrate(Y_AXIS, MAX_FEEDRATE_Y);
    Printer::setAxisMaxFeedrate(Z_AXIS, MAX_FEEDRATE_Z);
    for (uint8_t axis = X_AXIS; axis <= Z_AXIS; axis++) {
        float tmp = HAL::eprGetFloat(EPR_X_MAX_FEEDRATE + axis * 4); //X dann EPR_Y_MAX_FEEDRATE dann EPR_Z_MAX_FEEDRATE
        if (tmp > Printer::axisMaxFeedrate(axis)) {
            HAL::eprSetFloat(EPR_X_MAX_FEEDRATE + axis * 4, Printer::axisMaxFeedrate(axis));
            change = true; // Update checksum later in this function
        } else {
            Printer::setAxisMaxFeedrate(axis, tmp);
        }
    }

    // Reload the saved value in the current scale and keep the live SenseOffset.
    Printer::zOffset.setManualMicrometers(HAL::eprGetInt32(EPR_RF_Z_OFFSET), Printer::stepsPerMm(Z_AXIS));
    Printer::restoreZDisplayMode(HAL::eprGetByte(EPR_RF_Z_MODE));

    Printer::layers.resetFadeBounds(Printer::stepsPerMm(Z_AXIS));

    Printer::manualMotion.setMode(X_AXIS, HAL::eprGetByte(EPR_RF_MOVE_MODE_X));
    Printer::manualMotion.setMode(Y_AXIS, HAL::eprGetByte(EPR_RF_MOVE_MODE_Y));
    Printer::manualMotion.setMode(Z_AXIS, HAL::eprGetByte(EPR_RF_MOVE_MODE_Z));
    Printer::manualMotion.setCoordinateSystem((bool)HAL::eprGetByte(EPR_RF_MOVE_MODE_XY_KOSYS));
    Printer::manualMotion.setFeedrateSource((bool)HAL::eprGetByte(EPR_RF_MOVE_POSITION_FEEDRATE));

    Printer::setXYJerkLimit(HAL::eprGetFloat(EPR_MAX_XYJERK));
    Printer::setZJerkLimit(HAL::eprGetFloat(EPR_MAX_ZJERK));

    Printer::setPrintAcceleration(X_AXIS, HAL::eprGetFloat(EPR_X_MAX_ACCEL));
    Printer::setPrintAcceleration(Y_AXIS, HAL::eprGetFloat(EPR_Y_MAX_ACCEL));
    Printer::setPrintAcceleration(Z_AXIS, HAL::eprGetFloat(EPR_Z_MAX_ACCEL));
    if (Printer::printAcceleration(X_AXIS) < ACCELERATION_MIN_XY || Printer::printAcceleration(X_AXIS) > ACCELERATION_MAX_XY) {
        Printer::setPrintAcceleration(X_AXIS, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X);
        HAL::eprSetFloat(EPR_X_MAX_ACCEL, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X);
        change = true; // Update checksum later in this function
    }
    if (Printer::printAcceleration(Y_AXIS) < ACCELERATION_MIN_XY || Printer::printAcceleration(Y_AXIS) > ACCELERATION_MAX_XY) {
        Printer::setPrintAcceleration(Y_AXIS, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y);
        HAL::eprSetFloat(EPR_Y_MAX_ACCEL, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y);
        change = true; // Update checksum later in this function
    }
    if (Printer::printAcceleration(Z_AXIS) < ACCELERATION_MIN_Z || Printer::printAcceleration(Z_AXIS) > ACCELERATION_MAX_Z) {
        Printer::setPrintAcceleration(Z_AXIS, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z);
        HAL::eprSetFloat(EPR_Z_MAX_ACCEL, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z);
        change = true; // Update checksum later in this function
    }

    Printer::setTravelAcceleration(X_AXIS, HAL::eprGetFloat(EPR_X_MAX_TRAVEL_ACCEL));
    Printer::setTravelAcceleration(Y_AXIS, HAL::eprGetFloat(EPR_Y_MAX_TRAVEL_ACCEL));
    Printer::setTravelAcceleration(Z_AXIS, HAL::eprGetFloat(EPR_Z_MAX_TRAVEL_ACCEL));
    if (Printer::travelAcceleration(X_AXIS) < ACCELERATION_MIN_XY || Printer::travelAcceleration(X_AXIS) > ACCELERATION_MAX_XY) {
        Printer::setTravelAcceleration(X_AXIS, MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X);
        HAL::eprSetFloat(EPR_X_MAX_TRAVEL_ACCEL, MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X);
        change = true; // Update checksum later in this function
    }
    if (Printer::travelAcceleration(Y_AXIS) < ACCELERATION_MIN_XY || Printer::travelAcceleration(Y_AXIS) > ACCELERATION_MAX_XY) {
        Printer::setTravelAcceleration(Y_AXIS, MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y);
        HAL::eprSetFloat(EPR_Y_MAX_TRAVEL_ACCEL, MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y);
        change = true; // Update checksum later in this function
    }
    if (Printer::travelAcceleration(Z_AXIS) < ACCELERATION_MIN_Z || Printer::travelAcceleration(Z_AXIS) > ACCELERATION_MAX_Z) {
        Printer::setTravelAcceleration(Z_AXIS, MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z);
        HAL::eprSetFloat(EPR_Z_MAX_TRAVEL_ACCEL, MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z);
        change = true; // Update checksum later in this function
    }

    heatedBedController.pidDriveMax = HAL::eprGetByte(EPR_BED_DRIVE_MAX);
    heatedBedController.pidDriveMin = HAL::eprGetByte(EPR_BED_DRIVE_MIN);
    heatedBedController.pidPGain = HAL::eprGetFloat(EPR_BED_PID_PGAIN);
    heatedBedController.pidIGain = HAL::eprGetFloat(EPR_BED_PID_IGAIN);
    heatedBedController.pidDGain = HAL::eprGetFloat(EPR_BED_PID_DGAIN);
    heatedBedController.pidMax = HAL::eprGetByte(EPR_BED_PID_MAX);
    heatedBedController.sensorType = (version >= 9 || HAL::eprGetByte(EPR_RF_HEATED_BED_SENSOR_TYPE) != 0) ? HAL::eprGetByte(EPR_RF_HEATED_BED_SENSOR_TYPE) : HEATED_BED_SENSOR_TYPE;
    change |= readPidCalibrationTag(NUM_EXTRUDER, version);

    Printer::setAxisLengthMm(X_AXIS, HAL::eprGetFloat(EPR_X_LENGTH));
    if (Printer::axisLengthMm(X_AXIS) <= 0 || Printer::axisLengthMm(X_AXIS) > 245.0f) {
        Printer::setAxisLengthMm(X_AXIS, X_MAX_LENGTH_PRINT);
        HAL::eprSetFloat(EPR_X_LENGTH, Printer::axisLengthMm(X_AXIS));
        change = true; // Update checksum later in this function
    }
    Printer::setAxisLengthMm(Y_AXIS, HAL::eprGetFloat(EPR_Y_LENGTH));
    Printer::setAxisLengthMm(Z_AXIS, HAL::eprGetFloat(EPR_Z_LENGTH));

    // now the extruder
    for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
        int o = EEPROM::getExtruderOffset(i);
        Extruder* e = &extruder[i];
        float tmp = HAL::eprGetFloat(o + EPR_EXTRUDER_STEPS_PER_MM);
        if (tmp < 5540.0f) { // Da hat einer ein komma vergessen ^^ so hohe Werte können kaum sinn machen, ausser wir ändern die CPU. Das wären auch zu viele interrupts pro sekunde.
            e->stepsPerMM = tmp;
        } else {
            HAL::eprSetFloat(o + EPR_EXTRUDER_STEPS_PER_MM, e->stepsPerMM);
            change = true; // Update checksum later in this function
        }

        tmp = HAL::eprGetFloat(o + EPR_EXTRUDER_MAX_FEEDRATE);
        if (0 < tmp && tmp < 100) {
            e->maxFeedrate = tmp;
        } else {
            HAL::eprSetFloat(o + EPR_EXTRUDER_MAX_FEEDRATE, e->maxFeedrate);
            change = true; // Update checksum later in this function
        }

        tmp = HAL::eprGetFloat(o + EPR_EXTRUDER_MAX_START_FEEDRATE);
        if (0 < tmp && tmp <= e->maxFeedrate) {
            e->maxEJerk = tmp;
        } else {
            e->maxEJerk = RMath::min(e->maxFeedrate, e->maxEJerk);
            HAL::eprSetFloat(o + EPR_EXTRUDER_MAX_START_FEEDRATE, e->maxEJerk);
            change = true; // Update checksum later in this function
        }

        tmp = HAL::eprGetFloat(o + EPR_EXTRUDER_MAX_ACCELERATION);
        if (0 < tmp && tmp < 10000) {
            e->maxAcceleration = tmp;
        } else {
            HAL::eprSetFloat(o + EPR_EXTRUDER_MAX_ACCELERATION, e->maxAcceleration);
            change = true; // Update checksum later in this function
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
            change = true; // Update checksum later in this function
        }
        e->tempControl.pidPGain = HAL::eprGetFloat(o + EPR_EXTRUDER_PID_PGAIN);
        e->tempControl.pidIGain = HAL::eprGetFloat(o + EPR_EXTRUDER_PID_IGAIN);
        e->tempControl.pidDGain = HAL::eprGetFloat(o + EPR_EXTRUDER_PID_DGAIN);
        if (version >= 9 || HAL::eprGetByte(o + EPR_EXTRUDER_PID_MAX) > 0) {
            e->tempControl.pidMax = HAL::eprGetByte(o + EPR_EXTRUDER_PID_MAX);
        } else {
            HAL::eprSetByte(o + EPR_EXTRUDER_PID_MAX, e->tempControl.pidMax);
            change = true; // Update checksum later in this function
        }

        uint8_t sensortype_temp = HAL::eprGetByte(o + EPR_EXTRUDER_SENSOR_TYPE);
        if (version >= 9 || (sensortype_temp > 0 && sensortype_temp <= 100)) {
            e->tempControl.sensorType = sensortype_temp;
        }

        change |= readPidCalibrationTag(i, version);
        e->offsetMM[X_AXIS] = HAL::eprGetFloat(o + EPR_EXTRUDER_X_OFFSET);
        e->offsetMM[Y_AXIS] = HAL::eprGetFloat(o + EPR_EXTRUDER_Y_OFFSET);


        e->advanceL = HAL::eprGetFloat(o + EPR_EXTRUDER_ADVANCE_L);
        if (e->advanceL > 0 && e->advanceL < 20) {
            e->advanceL = 20;
        }

        if (version > 1)
            e->coolerSpeed = HAL::eprGetByte(o + EPR_EXTRUDER_COOLER_SPEED);
    }

    AccessoryOutputs::stage(AccessoryOutputs::Output::CaseLight, HAL::eprGetByte(EPR_RF_CASE_LIGHT_MODE));


#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    AccessoryOutputs::stage(AccessoryOutputs::Output::Fet1, HAL::eprGetByte(EPR_RF_FET1_MODE));
    AccessoryOutputs::stage(AccessoryOutputs::Output::Fet2, HAL::eprGetByte(EPR_RF_FET2_MODE));
#if CASE_FAN_PIN < 0
    // Do not load the FET3 EEPROM state if it is being used as case fan
    AccessoryOutputs::stage(AccessoryOutputs::Output::Fet3, HAL::eprGetByte(EPR_RF_FET3_MODE));
#endif // CASE_FAN_PIN < 0

    // after a power-on, the 230 V plug always shall be turned off - thus, we do not store this setting to the EEPROM
    // AccessoryOutputs::stage(AccessoryOutputs::Output::Mains, HAL::eprGetByte( EPR_RF_230V_OUTPUT_MODE ));
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2

    // Reinit for check if eeprom values are too high.
    unsigned int eeprom_homing_feedrate_position;
        eeprom_homing_feedrate_position = EPR_X_HOMING_FEEDRATE_PRINT;
        Printer::setAxisHomingFeedrate(X_AXIS, HOMING_FEEDRATE_X_PRINT);
        Printer::setAxisHomingFeedrate(Y_AXIS, HOMING_FEEDRATE_Y_PRINT);
        Printer::setAxisHomingFeedrate(Z_AXIS, HOMING_FEEDRATE_Z_PRINT);
    // Check if values are too high
    // This is no super complete check but if someone uses the printer bad values from old firmwares get corrected after one restart in each mode.
    // and update them to max according configuration consts if too high.
    for (uint8_t axis = X_AXIS; axis <= Z_AXIS; axis++) {
        float tmp = HAL::eprGetFloat(eeprom_homing_feedrate_position + axis * 4); // EPR_X_HOMING_FEEDRATE_PRINT EPR_Y_HOMING_FEEDRATE_PRINT EPR_Z_HOMING_FEEDRATE_PRINT

        // Alle Homing-Geschwindigkeiten höher max-Speed werden mit der Standard-Config-Homing-Speed überschrieben.
        // Das betrifft gerade noch die alten RF-Speeds, die viel zu hoch waren:
        //z.B: https://github.com/RF1000/Repetier-Firmware/blob/0e52693eaed1f606af411e4898030b033304b94f/RF2000/Repetier/RF2000.h#L781
        if (tmp > Printer::axisMaxFeedrate(axis)) {
            HAL::eprSetFloat(eeprom_homing_feedrate_position + axis * 4, Printer::axisHomingFeedrate(axis));
            change = true; // Update checksum later in this function
        } else {
            Printer::setAxisHomingFeedrate(axis, tmp);
        }
    }


    Printer::manualMotion.setSteps(X_AXIS, (unsigned short)lroundf(Printer::stepsPerMm(X_AXIS) * DEFAULT_MANUAL_MM_X));
    Printer::manualMotion.setSteps(Y_AXIS, (unsigned short)lroundf(Printer::stepsPerMm(Y_AXIS) * DEFAULT_MANUAL_MM_Y));
    static const unsigned short stepsize_table[NUM_ACCEPTABLE_STEP_SIZE_TABLE] PROGMEM = ACCEPTABLE_STEP_SIZE_TABLE;
    Printer::manualMotion.setSteps(Z_AXIS, (unsigned short)constrain((unsigned short)HAL::eprGetInt16(EPR_RF_MOD_Z_STEP_SIZE), 1, static_cast<unsigned short>(pgm_read_word(&stepsize_table[NUM_ACCEPTABLE_STEP_SIZE_TABLE - 1])))); // Limit stepsize to value in config.
    Printer::manualMotion.setSteps(E_AXIS, (unsigned short)lroundf(Extruder::current->stepsPerMM * DEFAULT_MANUAL_MM_E));                                                               //current extruder stepsPerMM weil hier noch kein update für Printer::stepsPerMm(E_AXIS) gemacht wurde!

    // The matrix is loaded later. Clamp to the compiled grid capacity here
    // and preserve the legacy zero value until the matrix is available.
    const auto storedScanIndex = [](uint8_t index, int minimum, int maximum) {
        return index ? uint8_t(constrain(index, minimum, maximum)) : uint8_t(0);
    };
    restoreZOffsetScanPoint({
        storedScanIndex(HAL::eprGetByte(EPR_RF_MOD_ZOS_SCAN_POINT_X), heatBedScanXStartMm == 0 ? 1 : 2, COMPENSATION_MATRIX_MAX_X - 1),
        storedScanIndex(HAL::eprGetByte(EPR_RF_MOD_ZOS_SCAN_POINT_Y), heatBedScanYStartMm == 0 ? 1 : 2, COMPENSATION_MATRIX_MAX_Y - 1),
    });

    const int16_t savedSenseLimit = HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX);
    Printer::senseOffsetControl.publishStoredLimits(HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS), savedSenseLimit);
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
        setScanStartLiftMm(tmpss);
    } else {
        HAL::eprSetFloat(EPR_ZSCAN_START_MM, defaultHeatBedScanLiftMm);
        change = true;
    }

    static const unsigned short uMotorCurrentMax[] PROGMEM = MOTOR_CURRENT_MAX;    // Oberes Amperelimit
    static const unsigned short uMotorCurrentUse[] PROGMEM = MOTOR_CURRENT_NORMAL; //Standardwert
    uint8_t current = 0;
    for (uint8_t axis = 0; axis < DRV8711_NUM_CHANNELS; axis++) { //0..4 bei 5 steppern.
        current = HAL::eprGetByte(EPR_RF_MOTOR_CURRENT + axis);
        if (MOTOR_CURRENT_MIN <= current && current <= pgm_read_word(&uMotorCurrentMax[axis])) {
            Drv8711::stageCurrent(axis, current);
            Drv8711::applyConfiguredCurrent(axis);
        } else {
            HAL::eprSetByte(EPR_RF_MOTOR_CURRENT + axis, pgm_read_word(&uMotorCurrentUse[axis])); // Wenn mist im EEPROM, dann Silent-Wert reinschreiben.
            change = true;
        }
    }
    // Does EEPROM have valid values for microsteps?
    if (HAL::eprGetByte(EPR_RF_MICRO_STEPS_USED) == 0xAB) {
        for (uint8_t axis = 0; axis < DRV8711_NUM_CHANNELS; axis++) {
            /* #define EPR_RF_MICRO_STEPS_X              1943 //[1byte]
               #define EPR_RF_MICRO_STEPS_Y              1944 //[1byte]
               #define EPR_RF_MICRO_STEPS_Z              1945 //[1byte]
               #define EPR_RF_MICRO_STEPS_E0             1946 //[1byte]
               #define EPR_RF_MICRO_STEPS_E1             1947 //[1byte] */
            uint8_t epr = HAL::eprGetByte(EPR_RF_MICRO_STEPS_X + axis);
            if (epr <= 8 && Drv8711::microstepMode(axis) != epr) {
                Drv8711::setMicrostepMode(axis, epr);
            }
        }
    }

    if (!HAL::eprGetInt16(EPR_RF_STEP_PACKING_MIN_INTERVAL)) {
        StepExecutor::setPackingInterval(STEP_PACKING_MIN_INTERVAL);
        HAL::eprSetInt16(EPR_RF_STEP_PACKING_MIN_INTERVAL, STEP_PACKING_MIN_INTERVAL);
        change = true;
    } else {
        StepExecutor::setPackingInterval(constrain(HAL::eprGetInt16(EPR_RF_STEP_PACKING_MIN_INTERVAL), MIN_STEP_PACKING_MIN_INTERVAL, MAX_STEP_PACKING_MIN_INTERVAL));
    }

    uint8_t temp_min = HAL::eprGetByte(EPR_RF_PART_FAN_PWM_MIN);
    uint8_t temp_max = HAL::eprGetByte(EPR_RF_PART_FAN_PWM_MAX);
    if ((int)temp_max - (int)temp_min >= 16 && temp_min > 0 && temp_max < 255) {
        HAL::setPartFanDutyRange(temp_min, temp_max);
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
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    // Finish an already-started integrity update, but do not continue
    // applying ordinary settings/hardware changes after a terminal fault.
    if(Printer::printJob.emergencyLocked())return;
#endif

    if (version != EEPROM_PROTOCOL_VERSION) {
        Com::printInfoFLN(Com::tEPRProtocolChanged);
        storeDataIntoEEPROM(false); // Store new fields for changed version
    }
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(Printer::printJob.emergencyLocked())return;
#endif
    Printer::updateDerivedParameter();
    Extruder::applyCurrentSettings();
    Printer::applyAdvanceSettings();
    Extruder::initHeatedBed();

} // readDataFromEEPROM

void EEPROM::publishSenseOffsetSettings() {
    const int16_t digits = HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS);
    const int16_t maximum = HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX);
    Printer::senseOffsetControl.publishStoredLimits(digits, maximum);
}

void EEPROM::storeSenseOffsetSettings(int16_t digits, int16_t micrometers, bool saveDigits, bool saveMaximum) {
    bool changed = false;
    if (saveDigits && HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS) != digits) {
        HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS, digits);
        changed = true;
    }
    if (saveMaximum && HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX) != micrometers) {
        HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX, micrometers);
        changed = true;
    }
    if (changed)
        updateChecksum();
    publishSenseOffsetSettings();
}

bool EEPROM::storePressureLimits(bool stop, int32_t minimum, int32_t maximum, bool saveMinimum, bool saveMaximum) {
    bool changed=false;
    if(stop) {
        if(saveMinimum && HAL::eprGetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMIN)!=minimum) {
            HAL::eprSetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMIN,minimum);
            changed=true;
        }
        if(saveMaximum && HAL::eprGetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMAX)!=maximum) {
            HAL::eprSetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMAX,maximum);
            changed=true;
        }
    } else {
        if(saveMinimum && HAL::eprGetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMIN)!=minimum) {
            HAL::eprSetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMIN,minimum);
            changed=true;
        }
        if(saveMaximum && HAL::eprGetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMAX)!=maximum) {
            HAL::eprSetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMAX,maximum);
            changed=true;
        }
    }
    if(changed) updateChecksum();
    return true; // legacy comparison only
}

void EEPROM::storeDriveLimit(uint8_t heater, bool maximum, uint8_t value) {
    if(heater>=NUM_TEMPERATURE_LOOPS) return;
    const uint16_t pos=heater==NUM_EXTRUDER
        ? (maximum?EPR_BED_DRIVE_MAX:EPR_BED_DRIVE_MIN)
        : getExtruderOffset(heater)+(maximum?EPR_EXTRUDER_DRIVE_MAX:EPR_EXTRUDER_DRIVE_MIN);
    if(HAL::eprGetByte(pos)==value) return;
    HAL::eprSetByte(pos,value);
    updateChecksum();
}

namespace {
void saveLegacyByte(uint16_t pos,uint8_t value) {
    if(HAL::eprGetByte(pos)==value)return;
    HAL::eprSetByte(pos,value);
    EEPROM::updateChecksum();
}
}
void EEPROM::storeCaseLight(bool on) {saveLegacyByte(EPR_RF_CASE_LIGHT_MODE,on);}
void EEPROM::storeFet(uint8_t channel,bool on) {
    switch(channel) {
    case 1:saveLegacyByte(EPR_RF_FET1_MODE,on);break;
    case 2:saveLegacyByte(EPR_RF_FET2_MODE,on);break;
    case 3:saveLegacyByte(EPR_RF_FET3_MODE,on);break;
    }
}
void EEPROM::storeJogMode(uint8_t axis,uint8_t mode) {
    switch(axis) {
    case X_AXIS:saveLegacyByte(EPR_RF_MOVE_MODE_X,mode);break;
    case Y_AXIS:saveLegacyByte(EPR_RF_MOVE_MODE_Y,mode);break;
    case Z_AXIS:saveLegacyByte(EPR_RF_MOVE_MODE_Z,mode);break;
    }
}
void EEPROM::storeZJogSteps(uint16_t steps) {
    if(uint16_t(HAL::eprGetInt16(EPR_RF_MOD_Z_STEP_SIZE))==steps)return;
    HAL::eprSetInt16(EPR_RF_MOD_Z_STEP_SIZE,steps);
    updateChecksum();
}
void EEPROM::storeJogCoordinates(bool gcode) {saveLegacyByte(EPR_RF_MOVE_MODE_XY_KOSYS,gcode);}
void EEPROM::storeJogFeedrate(bool configured) {saveLegacyByte(EPR_RF_MOVE_POSITION_FEEDRATE,configured);}
void EEPROM::storeZDisplayMode(uint8_t mode) {saveLegacyByte(EPR_RF_Z_MODE,mode);}
void EEPROM::storeFanMode(uint8_t mode) {saveLegacyByte(EPR_RF_FAN_MODE,mode);}
void EEPROM::storeDigitZeroing(bool enabled) {saveLegacyByte(EPR_RF_ZERO_DIGIT_STATE,enabled?1:2);}
void EEPROM::storeDigitCompensation(bool enabled) {saveLegacyByte(EPR_RF_DIGIT_CMP_STATE,enabled?1:2);}
void EEPROM::storeSenseAutoStart(bool enabled) {saveLegacyByte(EPR_RF_MOD_SENSEOFFSET_AUTOSTART,enabled);}
void EEPROM::storeScanPoint(uint8_t x,uint8_t y,bool saveX,bool saveY) {
    bool changed=false;
    if(saveX && HAL::eprGetByte(EPR_RF_MOD_ZOS_SCAN_POINT_X)!=x) {
        HAL::eprSetByte(EPR_RF_MOD_ZOS_SCAN_POINT_X,x);changed=true;
    }
    if(saveY && HAL::eprGetByte(EPR_RF_MOD_ZOS_SCAN_POINT_Y)!=y) {
        HAL::eprSetByte(EPR_RF_MOD_ZOS_SCAN_POINT_Y,y);changed=true;
    }
    if(changed)updateChecksum();
}

namespace {
void saveLegacyWord(uint16_t pos,uint16_t value) {
    if(uint16_t(HAL::eprGetInt16(pos))==value)return;
    HAL::eprSetInt16(pos,value);
    EEPROM::updateChecksum();
}
void saveLegacyLong(uint16_t pos,uint32_t value) {
    if(uint32_t(HAL::eprGetInt32(pos))==value)return;
    HAL::eprSetInt32(pos,value);
    EEPROM::updateChecksum();
}
void saveLegacyFloat(uint16_t pos,float value) {
    const float old=HAL::eprGetFloat(pos);
    if(memcmp(&old,&value,sizeof(value))==0)return;
    HAL::eprSetFloat(pos,value);
    EEPROM::updateChecksum();
}
}
void EEPROM::storeFanDivisor(uint8_t divisor) {saveLegacyByte(EPR_RF_PART_FAN_SPEED,divisor);}
void EEPROM::storeFanDuty(bool maximum,uint8_t duty) {saveLegacyByte(maximum?EPR_RF_PART_FAN_PWM_MAX:EPR_RF_PART_FAN_PWM_MIN,duty);}
void EEPROM::storeToolOffsets(uint8_t tool,float x,float y,bool saveX,bool saveY) {
    if(tool>=NUM_EXTRUDER)return;
    const uint16_t pos=getExtruderOffset(tool);
    bool changed=false;
    const float oldX=HAL::eprGetFloat(pos+EPR_EXTRUDER_X_OFFSET),oldY=HAL::eprGetFloat(pos+EPR_EXTRUDER_Y_OFFSET);
    if(saveX && memcmp(&oldX,&x,sizeof(x))!=0) {HAL::eprSetFloat(pos+EPR_EXTRUDER_X_OFFSET,x);changed=true;}
    if(saveY && memcmp(&oldY,&y,sizeof(y))!=0) {HAL::eprSetFloat(pos+EPR_EXTRUDER_Y_OFFSET,y);changed=true;}
    if(changed)updateChecksum();
}
void EEPROM::storeInactivity(bool stepper,uint32_t duration) {saveLegacyLong(stepper?EPR_STEPPER_INACTIVE_TIME:EPR_MAX_INACTIVE_TIME,duration);}
void EEPROM::storeScanLift(float lift) {saveLegacyFloat(EPR_ZSCAN_START_MM,lift);}
void EEPROM::storeWobblePhase(int8_t phase) {saveLegacyByte(EPR_RF_MOD_WOBBLE_FIX_PHASEXY,uint8_t(phase));}
void EEPROM::storeWobbleAmplitude(uint8_t component,int16_t amplitude) {
    switch(component) {
    case 0:saveLegacyWord(EPR_RF_MOD_WOBBLE_FIX_AMPX,amplitude);break;
    case 1:saveLegacyWord(EPR_RF_MOD_WOBBLE_FIX_AMPY1,amplitude);break;
    case 2:saveLegacyWord(EPR_RF_MOD_WOBBLE_FIX_AMPY2,amplitude);break;
    }
}
void EEPROM::storeMotorCurrent(uint8_t axis,uint8_t current) {
    if(axis<DRV8711_NUM_CHANNELS)saveLegacyByte(EPR_RF_MOTOR_CURRENT+axis,current);
}
void EEPROM::storePackingInterval(uint16_t interval) {saveLegacyWord(EPR_RF_STEP_PACKING_MIN_INTERVAL,interval);}
void EEPROM::storeAxisAcceleration(uint8_t axis,bool travel,float acceleration) {
    if(axis<3)saveLegacyFloat((travel?EPR_X_MAX_TRAVEL_ACCEL:EPR_X_MAX_ACCEL)+4*axis,acceleration);
}
void EEPROM::storeAxisJerk(bool z,float jerk) {saveLegacyFloat(z?EPR_MAX_ZJERK:EPR_MAX_XYJERK,jerk);}
void EEPROM::storeAxisFeedrate(uint8_t axis,bool homing,float feedrate) {
    if(axis<3)saveLegacyFloat((homing?EPR_X_HOMING_FEEDRATE_PRINT:EPR_X_MAX_FEEDRATE)+4*axis,feedrate);
}
void EEPROM::storeToolMotion(uint8_t tool,ToolMotionSetting setting,float value) {
    if(tool>=NUM_EXTRUDER)return;
    uint16_t relative;
    switch(setting) {
    case ToolMotionSetting::Steps:relative=EPR_EXTRUDER_STEPS_PER_MM;break;
    case ToolMotionSetting::Advance:relative=EPR_EXTRUDER_ADVANCE_L;break;
    case ToolMotionSetting::Acceleration:relative=EPR_EXTRUDER_MAX_ACCELERATION;break;
    case ToolMotionSetting::Feedrate:relative=EPR_EXTRUDER_MAX_FEEDRATE;break;
    case ToolMotionSetting::Jerk:relative=EPR_EXTRUDER_MAX_START_FEEDRATE;break;
    default:return;
    }
    saveLegacyFloat(getExtruderOffset(tool)+relative,value);
}

EEPROM::UsageTotals EEPROM::usageTotals() {
    return {HAL::eprGetInt32(EPR_PRINTING_TIME), HAL::eprGetFloat(EPR_PRINTING_DISTANCE)};
}

void EEPROM::storeUsageTotals(UsageTotals totals) {
    const auto old=usageTotals();
    if(old.seconds==totals.seconds && old.metres==totals.metres) return;
    HAL::eprSetInt32(EPR_PRINTING_TIME,totals.seconds);
    HAL::eprSetFloat(EPR_PRINTING_DISTANCE,totals.metres);
    updateChecksum();
}

void EEPROM::init() {
    bool kill_eeprom_because_corrupted = (computeChecksum() != HAL::eprGetByte(EPR_INTEGRITY_BYTE));
    bool kill_eeprom_wrong_version_update = (eepromMagic != HAL::eprGetByte(EPR_MAGIC_BYTE));
    bool kill_eeprom_by_back_ok_play = (READ(ENABLE_KEY_1) == 0 && READ(ENABLE_KEY_4) == 0 && READ(ENABLE_KEY_E5) == 0);

    bool update_eeprom_num_extruders_changed_2 = (NUM_EXTRUDER > HAL::eprGetByte(EPR_CHECK_NUM_EXTRUDERS) && NUM_EXTRUDER == 2);
    bool update_eeprom_num_extruders_changed_1 = (NUM_EXTRUDER < HAL::eprGetByte(EPR_CHECK_NUM_EXTRUDERS) && NUM_EXTRUDER == 1);

    if (kill_eeprom_wrong_version_update || kill_eeprom_because_corrupted || kill_eeprom_by_back_ok_play) {
        EEPROM::clearEEPROM();
        EEPROM::restoreEEPROMSettingsFromConfiguration();
        EEPROM::storeDataIntoEEPROM(kill_eeprom_because_corrupted); // Wenn corrupted dann auch die betriebszähler löschen.

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
        Printer::setAxisLengthMm(X_AXIS, X_MAX_LENGTH_PRINT);
        HAL::eprSetFloat(EPR_X_LENGTH, X_MAX_LENGTH_PRINT);

        HAL::eprSetByte(EPR_CHECK_NUM_EXTRUDERS, NUM_EXTRUDER);
        EEPROM::updateChecksum();

        Com::printF(PSTR("EEPROM update "));
        Com::printFLN(PSTR(UI_TEXT_EXTRUDER1_ADD));
        showInformation(PSTR(UI_TEXT_CONFIGURATION), PSTR(UI_TEXT_EXTRUDER1_ADD), PSTR(UI_TEXT_UPDATE_MATRIX));

        // Continue loading data.
    }

    if (update_eeprom_num_extruders_changed_1) {
        // auto-adjust x axis length:
        Printer::setAxisLengthMm(X_AXIS, X_MAX_LENGTH_PRINT);
        HAL::eprSetFloat(EPR_X_LENGTH, X_MAX_LENGTH_PRINT);

        HAL::eprSetByte(EPR_CHECK_NUM_EXTRUDERS, NUM_EXTRUDER);
        EEPROM::updateChecksum();

        Com::printF(PSTR("EEPROM update "));
        Com::printFLN(PSTR(UI_TEXT_EXTRUDER1_REMOVED));
        showInformation(PSTR(UI_TEXT_CONFIGURATION), PSTR(UI_TEXT_EXTRUDER1_REMOVED), PSTR(UI_TEXT_UPDATE_MATRIX));

        // Continue loading data.
    }

    EEPROM::readDataFromEEPROM();
} // init

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
#define RF_EPR_REPORT_0(pos, label, digits) writeByte(pos, label)
#define RF_EPR_REPORT_1(pos, label, digits) writeInt(pos, label)
#define RF_EPR_REPORT_2(pos, label, digits) writeLong(pos, label)
#define RF_EPR_REPORT_3(pos, label, digits) writeFloat(pos, label, digits)
#define RF_EPR_FIELD(type, pos, label, digits, writable) RF_EPR_REPORT_##type(pos, label, digits)
#include "EepromHostFields.h"
#undef RF_EPR_FIELD
#undef RF_EPR_REPORT_0
#undef RF_EPR_REPORT_1
#undef RF_EPR_REPORT_2
#undef RF_EPR_REPORT_3
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

void EEPROM::writeExtruderPrefix(uint16_t pos) {
    if (pos < EEPROM_EXTRUDER_OFFSET || pos >= 800)
        return;
    int n = (pos - EEPROM_EXTRUDER_OFFSET) / EEPROM_EXTRUDER_LENGTH + 1;
    Com::printF(Com::tExtrDot, n);
    Com::print(' ');

} // writeExtruderPrefix

void EEPROM::beginReport(uint16_t pos, PGM_P type) {
    Com::printF(type, (int)pos);
    Com::print(' ');
}

void EEPROM::finishReport(uint16_t pos, PGM_P text) {
    Com::print(' ');
    writeExtruderPrefix(pos);
    Com::printFLN(text);
    // Preserve the established serial pacing after each complete EPR line.
    HAL::delayMilliseconds(4);
}

void EEPROM::writeFloat(uint16_t pos, PGM_P text, uint8_t digits) {
    beginReport(pos, Com::tEPR3);
    Com::printFloat(HAL::eprGetFloat(pos), digits);
    finishReport(pos, text);
} // writeFloat

void EEPROM::writeLong(uint16_t pos, PGM_P text) {
    beginReport(pos, Com::tEPR2);
    Com::print(HAL::eprGetInt32(pos));
    finishReport(pos, text);
} // writeLong

void EEPROM::writeInt(uint16_t pos, PGM_P text) {
    beginReport(pos, Com::tEPR1);
    Com::print(HAL::eprGetInt16(pos));
    finishReport(pos, text);
} // writeInt

void EEPROM::writeByte(uint16_t pos, PGM_P text) {
    beginReport(pos, Com::tEPR0);
    Com::print((int)HAL::eprGetByte(pos));
    finishReport(pos, text);
} // writeByte

#endif // RF_EEPROM_ZONE_EXPERIMENT legacy comparison

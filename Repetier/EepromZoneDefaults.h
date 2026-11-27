// Factory provider for the NEW EEPROM schema. Values come from the actual
// selected board configuration, not from historical EEPROM or live RAM.
// Caller includes the resolved BoardConfiguration first. No hardware writes.
#ifndef RF_EEPROM_ZONE_DEFAULTS_H
#define RF_EEPROM_ZONE_DEFAULTS_H
#include "EepromZoneCodec.h"
#include "ManualMotion.h"
#include "DisplayModes.h"
#include <math.h>
#include <stddef.h>

namespace EepromZones {
// Stable identity for the four supported board/retrofit builds. A change to
// an unmodeled connector assignment needs an explicit profile review.
constexpr uint16_t configuredProfile() {
    return (uint16_t(MOTHERBOARD)<<8) | (uint16_t(NUM_EXTRUDER)<<4)
#ifdef X44_PIN
        | (HOTEND_COOLER_USES_PIN(X44_PIN) ? 1 : 0)
#endif
        ;
}

// Fixed-size factory records use the same little-endian binary32 representation
// as the codec on supported AVR and host replay targets. Every member is
// checked against the schema, and the length is checked before copying.
#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error Factory records require a little-endian target
#endif
static_assert(sizeof(float)==4,"factory records require binary32");
#ifdef __AVR__
#define RF_FACTORY_FLASH PROGMEM
#else
#define RF_FACTORY_FLASH
#endif
#define RF_FACTORY_MEMBER(type,field,value) type field;
#define RF_FACTORY_VALUE(type,field,value) type(value),
#define RF_FACTORY_OFFSET(type,field,value) \
    static_assert(offsetof(Record,field)+address(blockId)+PayloadOffset==address(Field::field),"factory field offset");
#define RF_FACTORY_RECORD(block,fields) \
    case block: { \
        constexpr uint8_t blockId=block; \
        struct __attribute__((packed)) Record { fields(RF_FACTORY_MEMBER) }; \
        fields(RF_FACTORY_OFFSET) \
        static constexpr Record record RF_FACTORY_FLASH={fields(RF_FACTORY_VALUE)}; \
        return copyRecord(data,n,record); \
    }

class FactoryDefaults {
    template<class Record> static bool copyRecord(uint8_t* data,uint8_t n,const Record& record) {
        if(n!=sizeof(record))return false;
#ifdef __AVR__
        memcpy_P(data,&record,n);
#else
        memcpy(data,&record,n);
#endif
        return true;
    }
    float scanLift_;
    uint8_t scanIndex_;
    bool gcodeCoordinates_, configuredFeedrate_;
    static constexpr uint8_t mode(unsigned microsteps) {
        return microsteps>1?uint8_t(1+mode(microsteps>>1)):0;
    }
    struct ZeroIdentity {uint8_t bytes[3];};
    static_assert(sizeof(ZeroIdentity)==3,"factory PID identity width");
public:
    FactoryDefaults(float scanLift,uint8_t scanIndex,bool gcodeCoordinates,bool configuredFeedrate)
        :scanLift_(scanLift),scanIndex_(scanIndex),gcodeCoordinates_(gcodeCoordinates),
         configuredFeedrate_(configuredFeedrate) {}
    bool operator()(uint8_t id,uint8_t* data,uint8_t n) const {
        if(!id || id>Last || !data || n!=length(id)) return false;
        BlockEncoder w(id,data,n);
        switch(id) {
        case Geometry:
            w.f32(Field::F_XAXIS_STEPS_PER_MM,XAXIS_STEPS_PER_MM);
            w.f32(Field::F_YAXIS_STEPS_PER_MM,YAXIS_STEPS_PER_MM);
            w.f32(Field::F_ZAXIS_STEPS_PER_MM,ZAXIS_STEPS_PER_MM);
            w.f32(Field::F_X_LENGTH,X_MAX_LENGTH_PRINT);
            w.f32(Field::F_Y_LENGTH,Y_MAX_LENGTH);
            w.f32(Field::F_Z_LENGTH,Z_MAX_LENGTH);
            w.u8(Field::F_RF_MICRO_STEPS_X,mode(RF_MICRO_STEPS_XY));
            w.u8(Field::F_RF_MICRO_STEPS_Y,mode(RF_MICRO_STEPS_XY));
            w.u8(Field::F_RF_MICRO_STEPS_Z,mode(RF_MICRO_STEPS_Z));
            w.u16(Field::F_RF_MOD_Z_STEP_SIZE,uint16_t(lroundf(ZAXIS_STEPS_PER_MM*DEFAULT_MANUAL_MM_Z)));
            break;
#define RF_FACTORY_Feedrates(X) \
            X(float,F_X_MAX_FEEDRATE,MAX_FEEDRATE_X) \
            X(float,F_Y_MAX_FEEDRATE,MAX_FEEDRATE_Y) \
            X(float,F_Z_MAX_FEEDRATE,MAX_FEEDRATE_Z) \
            X(float,F_X_HOMING_FEEDRATE_PRINT,HOMING_FEEDRATE_X_PRINT) \
            X(float,F_Y_HOMING_FEEDRATE_PRINT,HOMING_FEEDRATE_Y_PRINT) \
            X(float,F_Z_HOMING_FEEDRATE_PRINT,HOMING_FEEDRATE_Z_PRINT)
        RF_FACTORY_RECORD(Feedrates,RF_FACTORY_Feedrates)
#undef RF_FACTORY_Feedrates
#define RF_FACTORY_Jerk(X) \
            X(float,F_MAX_XYJERK,MAX_JERK) \
            X(float,F_MAX_ZJERK,MAX_ZJERK)
        RF_FACTORY_RECORD(Jerk,RF_FACTORY_Jerk)
#undef RF_FACTORY_Jerk
#define RF_FACTORY_Acceleration(X) \
            X(float,F_X_MAX_ACCEL,MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X) \
            X(float,F_Y_MAX_ACCEL,MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y) \
            X(float,F_Z_MAX_ACCEL,MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z) \
            X(float,F_X_MAX_TRAVEL_ACCEL,MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X) \
            X(float,F_Y_MAX_TRAVEL_ACCEL,MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y) \
            X(float,F_Z_MAX_TRAVEL_ACCEL,MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z) \
            X(uint16_t,F_RF_STEP_PACKING_MIN_INTERVAL,STEP_PACKING_MIN_INTERVAL)
        RF_FACTORY_RECORD(Acceleration,RF_FACTORY_Acceleration)
#undef RF_FACTORY_Acceleration
        case MotorCurrent: {
            const uint8_t current[5]=MOTOR_CURRENT_NORMAL;
            w.bytes(Field::F_RF_MOTOR_CURRENT,current,5);break;
        }
#define RF_FACTORY_Extruder0Motion(X) \
            X(float,F_E0_STEPS_PER_MM,EXT0_STEPS_PER_MM) \
            X(float,F_E0_MAX_FEEDRATE,EXT0_MAX_FEEDRATE) \
            X(float,F_E0_MAX_START_FEEDRATE,EXT0_MAX_START_FEEDRATE) \
            X(float,F_E0_MAX_ACCELERATION,EXT0_MAX_ACCELERATION) \
            X(float,F_E0_ADVANCE_L,EXT0_ADVANCE_L) \
            X(float,F_E0_X_OFFSET,EXT0_X_OFFSET_MM) \
            X(float,F_E0_Y_OFFSET,EXT0_Y_OFFSET_MM) \
            X(uint8_t,F_RF_MICRO_STEPS_E0,mode(RF_MICRO_STEPS_E))
        RF_FACTORY_RECORD(Extruder0Motion,RF_FACTORY_Extruder0Motion)
#undef RF_FACTORY_Extruder0Motion
#if NUM_EXTRUDER > 1
#define RF_FACTORY_Extruder1Motion(X) \
            X(float,F_E1_STEPS_PER_MM,EXT1_STEPS_PER_MM) \
            X(float,F_E1_MAX_FEEDRATE,EXT1_MAX_FEEDRATE) \
            X(float,F_E1_MAX_START_FEEDRATE,EXT1_MAX_START_FEEDRATE) \
            X(float,F_E1_MAX_ACCELERATION,EXT1_MAX_ACCELERATION) \
            X(float,F_E1_ADVANCE_L,EXT1_ADVANCE_L) \
            X(float,F_E1_X_OFFSET,EXT1_X_OFFSET_MM) \
            X(float,F_E1_Y_OFFSET,EXT1_Y_OFFSET_MM) \
            X(uint8_t,F_RF_MICRO_STEPS_E1,mode(RF_MICRO_STEPS_E))
        RF_FACTORY_RECORD(Extruder1Motion,RF_FACTORY_Extruder1Motion)
#undef RF_FACTORY_Extruder1Motion
#else
#define RF_FACTORY_Extruder1Motion(X) \
            X(float,F_E1_STEPS_PER_MM,0) \
            X(float,F_E1_MAX_FEEDRATE,0) \
            X(float,F_E1_MAX_START_FEEDRATE,0) \
            X(float,F_E1_MAX_ACCELERATION,0) \
            X(float,F_E1_ADVANCE_L,0) \
            X(float,F_E1_X_OFFSET,0) \
            X(float,F_E1_Y_OFFSET,0) \
            X(uint8_t,F_RF_MICRO_STEPS_E1,mode(RF_MICRO_STEPS_E))
        RF_FACTORY_RECORD(Extruder1Motion,RF_FACTORY_Extruder1Motion)
#undef RF_FACTORY_Extruder1Motion
#endif
#define RF_FACTORY_Hotend0(X) \
            X(float,F_E0_PID_PGAIN,EXT0_HOTEND_PROFILE.pidPGain) \
            X(float,F_E0_PID_IGAIN,EXT0_HOTEND_PROFILE.pidIGain) \
            X(float,F_E0_PID_DGAIN,EXT0_HOTEND_PROFILE.pidDGain) \
            X(uint8_t,F_E0_DRIVE_MIN,EXT0_HOTEND_PROFILE.pidDriveMin) \
            X(uint8_t,F_E0_DRIVE_MAX,EXT0_HOTEND_PROFILE.pidDriveMax) \
            X(uint8_t,F_E0_PID_MAX,EXT0_HOTEND_PROFILE.pidMax) \
            X(uint8_t,F_E0_SENSOR_TYPE,EXT0_TEMPSENSOR_TYPE) \
            X(ZeroIdentity,F_PID_CALIBRATION_SLOT_0,ZeroIdentity{}) \
            X(uint8_t,F_E0_COOLER_SPEED,EXT0_EXTRUDER_COOLER_SPEED)
        RF_FACTORY_RECORD(Hotend0,RF_FACTORY_Hotend0)
#undef RF_FACTORY_Hotend0
#if NUM_EXTRUDER > 1
#define RF_FACTORY_Hotend1(X) \
            X(float,F_E1_PID_PGAIN,EXT1_HOTEND_PROFILE.pidPGain) \
            X(float,F_E1_PID_IGAIN,EXT1_HOTEND_PROFILE.pidIGain) \
            X(float,F_E1_PID_DGAIN,EXT1_HOTEND_PROFILE.pidDGain) \
            X(uint8_t,F_E1_DRIVE_MIN,EXT1_HOTEND_PROFILE.pidDriveMin) \
            X(uint8_t,F_E1_DRIVE_MAX,EXT1_HOTEND_PROFILE.pidDriveMax) \
            X(uint8_t,F_E1_PID_MAX,EXT1_HOTEND_PROFILE.pidMax) \
            X(uint8_t,F_E1_SENSOR_TYPE,EXT1_TEMPSENSOR_TYPE) \
            X(ZeroIdentity,F_PID_CALIBRATION_SLOT_1,ZeroIdentity{}) \
            X(uint8_t,F_E1_COOLER_SPEED,EXT1_EXTRUDER_COOLER_SPEED)
        RF_FACTORY_RECORD(Hotend1,RF_FACTORY_Hotend1)
#undef RF_FACTORY_Hotend1
#else
#define RF_FACTORY_Hotend1(X) \
            X(float,F_E1_PID_PGAIN,0) \
            X(float,F_E1_PID_IGAIN,0) \
            X(float,F_E1_PID_DGAIN,0) \
            X(uint8_t,F_E1_DRIVE_MIN,0) \
            X(uint8_t,F_E1_DRIVE_MAX,0) \
            X(uint8_t,F_E1_PID_MAX,0) \
            X(uint8_t,F_E1_SENSOR_TYPE,0) \
            X(ZeroIdentity,F_PID_CALIBRATION_SLOT_1,ZeroIdentity{}) \
            X(uint8_t,F_E1_COOLER_SPEED,0)
        RF_FACTORY_RECORD(Hotend1,RF_FACTORY_Hotend1)
#undef RF_FACTORY_Hotend1
#endif
#define RF_FACTORY_BedPid(X) \
            X(float,F_BED_PID_PGAIN,HEATED_BED_PID_PGAIN) \
            X(float,F_BED_PID_IGAIN,HEATED_BED_PID_IGAIN) \
            X(float,F_BED_PID_DGAIN,HEATED_BED_PID_DGAIN) \
            X(uint8_t,F_BED_DRIVE_MIN,HEATED_BED_PID_INTEGRAL_DRIVE_MIN) \
            X(uint8_t,F_BED_DRIVE_MAX,HEATED_BED_PID_INTEGRAL_DRIVE_MAX) \
            X(uint8_t,F_BED_PID_MAX,HEATED_BED_PID_MAX) \
            X(uint8_t,F_RF_HEATED_BED_SENSOR_TYPE,HEATED_BED_SENSOR_TYPE) \
            X(ZeroIdentity,F_PID_CALIBRATION_SLOT_2,ZeroIdentity{})
        RF_FACTORY_RECORD(BedPid,RF_FACTORY_BedPid)
#undef RF_FACTORY_BedPid
        case BedSafety:
            // UI may offer 120 C for explicit commissioning; no permission
            // to heat is stored on first install or automatic repair.
            w.u8(Field::F_NEW_BED_EMERGENCY_C,0);break;
#define RF_FACTORY_Preheat(X) \
            X(uint16_t,F_RF_PREHEAT_A_BED,PREHEAT_A_BED_C) \
            X(uint16_t,F_RF_PREHEAT_A_HOTEND,PREHEAT_A_HOTEND_C) \
            X(uint16_t,F_RF_PREHEAT_B_BED,PREHEAT_B_BED_C) \
            X(uint16_t,F_RF_PREHEAT_B_HOTEND,PREHEAT_B_HOTEND_C)
        RF_FACTORY_RECORD(Preheat,RF_FACTORY_Preheat)
#undef RF_FACTORY_Preheat
#define RF_FACTORY_ZOffset(X) \
            X(uint32_t,F_RF_Z_OFFSET,0)
        RF_FACTORY_RECORD(ZOffset,RF_FACTORY_ZOffset)
#undef RF_FACTORY_ZOffset
#define RF_FACTORY_SenseOffset(X) \
            X(uint16_t,F_RF_MOD_SENSEOFFSET_OFFSET_MAX,SENSIBLE_PRESSURE_MAX_OFFSET) \
            X(uint16_t,F_RF_MOD_SENSEOFFSET_DIGITS,0) \
            X(uint8_t,F_RF_MOD_SENSEOFFSET_AUTOSTART,0) \
            X(uint8_t,F_RF_ZERO_DIGIT_STATE,1) \
            X(uint8_t,F_RF_DIGIT_CMP_STATE,1)
        RF_FACTORY_RECORD(SenseOffset,RF_FACTORY_SenseOffset)
#undef RF_FACTORY_SenseOffset
#define RF_FACTORY_Pressure(X) \
            X(uint32_t,F_RF_EMERGENCYPAUSEDIGITSMIN,uint32_t(EMERGENCY_PAUSE_DIGITS_MIN)) \
            X(uint32_t,F_RF_EMERGENCYPAUSEDIGITSMAX,EMERGENCY_PAUSE_DIGITS_MAX) \
            X(uint16_t,F_RF_EMERGENCYZSTOPDIGITSMIN,uint16_t(EMERGENCY_STOP_DIGITS_MIN)) \
            X(uint16_t,F_RF_EMERGENCYZSTOPDIGITSMAX,EMERGENCY_STOP_DIGITS_MAX)
        RF_FACTORY_RECORD(Pressure,RF_FACTORY_Pressure)
#undef RF_FACTORY_Pressure
        case Scan:
            w.u8(Field::F_RF_MOD_ZOS_SCAN_POINT_X,scanIndex_);
            w.u8(Field::F_RF_MOD_ZOS_SCAN_POINT_Y,scanIndex_);
            w.f32(Field::F_ZSCAN_START_MM,scanLift_);break;
#define RF_FACTORY_Wobble(X) \
            X(uint8_t,F_RF_MOD_WOBBLE_FIX_PHASEXY,0) \
            X(uint16_t,F_RF_MOD_WOBBLE_FIX_AMPX,0) \
            X(uint16_t,F_RF_MOD_WOBBLE_FIX_AMPY1,0) \
            X(uint16_t,F_RF_MOD_WOBBLE_FIX_AMPY2,0)
        RF_FACTORY_RECORD(Wobble,RF_FACTORY_Wobble)
#undef RF_FACTORY_Wobble
        case Panel:
            w.u8(Field::F_RF_MOVE_MODE_X,DEFAULT_MOVE_MODE_X);
            w.u8(Field::F_RF_MOVE_MODE_Y,DEFAULT_MOVE_MODE_Y);
            w.u8(Field::F_RF_MOVE_MODE_Z,DEFAULT_MOVE_MODE_Z);
            w.u8(Field::F_RF_Z_MODE,DEFAULT_Z_SCALE_MODE);
            w.u8(Field::F_RF_MOVE_MODE_XY_KOSYS,gcodeCoordinates_);
            w.u8(Field::F_RF_MOVE_POSITION_FEEDRATE,configuredFeedrate_);break;
        case Lighting: {
            w.u8(Field::F_RF_CASE_LIGHT_MODE,CASE_LIGHTS_DEFAULT_ON);
#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
            w.u8(Field::F_RF_FET1_MODE,HOTEND_COOLER_USES_PIN(FET1)?0:FET1_DEFAULT_ON);
            w.u8(Field::F_RF_FET2_MODE,HOTEND_COOLER_USES_PIN(FET2)?0:FET2_DEFAULT_ON);
            w.u8(Field::F_RF_FET3_MODE,CASE_FAN_PIN>=0?0:FET3_DEFAULT_ON);
#else
            w.u8(Field::F_RF_FET1_MODE,0);w.u8(Field::F_RF_FET2_MODE,0);w.u8(Field::F_RF_FET3_MODE,0);
#endif
            break;
        }
#define RF_FACTORY_PartFan(X) \
            X(uint8_t,F_RF_FAN_MODE,0) \
            X(uint8_t,F_RF_PART_FAN_SPEED,PART_FAN_DEFAULT_PWM_SPEED_DIVISOR) \
            X(uint8_t,F_RF_PART_FAN_PWM_MIN,PART_FAN_PWM_MIN) \
            X(uint8_t,F_RF_PART_FAN_PWM_MAX,PART_FAN_PWM_MAX)
        RF_FACTORY_RECORD(PartFan,RF_FACTORY_PartFan)
#undef RF_FACTORY_PartFan
#define RF_FACTORY_System(X) \
            X(uint32_t,F_MAX_INACTIVE_TIME,uint32_t(MAX_INACTIVE_TIME)*1000) \
            X(uint32_t,F_STEPPER_INACTIVE_TIME,uint32_t(STEPPER_INACTIVE_TIME)*1000)
        RF_FACTORY_RECORD(System,RF_FACTORY_System)
#undef RF_FACTORY_System
#define RF_FACTORY_Usage(X) \
            X(uint32_t,F_PRINTING_TIME,0) \
            X(float,F_PRINTING_DISTANCE,0)
        RF_FACTORY_RECORD(Usage,RF_FACTORY_Usage)
#undef RF_FACTORY_Usage
        default:return false;
        }
        return w.complete();
    }
};
#undef RF_FACTORY_RECORD
#undef RF_FACTORY_OFFSET
#undef RF_FACTORY_VALUE
#undef RF_FACTORY_MEMBER
#undef RF_FACTORY_FLASH
}
#endif

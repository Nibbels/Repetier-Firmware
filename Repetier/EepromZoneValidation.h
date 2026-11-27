// Shared persisted-settings policy. Validate complete proposed payloads
// before EEPROM commit or publication, never normalize corrupt data in place.
// Bounds come from the established loader, panel, configured board and
// representable motion state; they are not a physical retrofit qualification.
#ifndef RF_EEPROM_ZONE_VALIDATION_H
#define RF_EEPROM_ZONE_VALIDATION_H
#include "EepromZoneCodec.h"
#include "EepromZoneStorage.h"
#include "ManualMotion.h"
#include "DisplayModes.h"
#include "RgbLighting.h"
#include <math.h>
#include <float.h>

namespace EepromZones {
inline bool inRange(float v,float lo,float hi) {return v>=lo && v<=hi;}
// Match the exact persisted field width and signedness. Only typed
// bounds in the same representable domain may be passed to these helpers.
inline bool rangeU8(uint8_t v,uint8_t lo,uint8_t hi) {return v>=lo && v<=hi;}
inline bool rangeU16(uint16_t v,uint16_t lo,uint16_t hi) {return v>=lo && v<=hi;}
inline bool rangeI8(int8_t v,int8_t lo,int8_t hi) {return v>=lo && v<=hi;}
inline bool rangeI16(int16_t v,int16_t lo,int16_t hi) {return v>=lo && v<=hi;}
inline bool positive(float v,float hi) {return v>0 && v<=hi;}
inline Field shifted(Field f,uint8_t bytes) {return static_cast<Field>(address(f)+bytes);}
inline bool sensorSupported(uint8_t v) {
    switch(v) {
    case 0: case 1: case 2: case 3: case 4: case 8: case 9:
    case 10: case 11: case 12: case 13: case 14: case 53: case 60: case 100: return true;
    default:return false;
    }
}
inline bool validBaud(uint32_t v) {
    switch(v) {
    case 9600: case 14400: case 19200: case 28800: case 38400: case 56000:
    case 57600: case 76800: case 111112: case 115200: case 128000:
    case 230400: case 250000: case 256000: case 460800: case 500000:
    case 921600: case 1000000: case 1500000: return true;
    default:return false;
    }
}
inline bool validPreheat(uint16_t v,bool bed) {
    // The stored preference is independent of the currently commissioned
    // cutoff; target admission is a separate, dynamic check.
    return !v || (bed ? v>=UI_SET_MIN_HEATED_BED_TEMP && v<=140 && v<=UI_SET_MAX_HEATED_BED_TEMP
                      : v>=UI_SET_MIN_EXTRUDER_TEMP && v<=UI_SET_MAX_EXTRUDER_TEMP);
}
inline bool validThermal(BlockDecoder& d,Field p,Field i,Field gainD,Field min,
                         Field max,Field power,Field sensor,Field tag) {
    const float pg=d.f32(p), ig=d.f32(i), dg=d.f32(gainD);
    const uint8_t pwm=d.u8(power), type=d.u8(sensor);
    const uint8_t marker=d.u8(tag), tunedSensor=d.u8(shifted(tag,1)), tunedPower=d.u8(shifted(tag,2));
    return positive(pg,1000) && positive(ig,1000) && inRange(dg,0,100000)
        && d.u8(min)>0 && d.u8(max)>0 && (!pwm || pwm>=40) && sensorSupported(type)
        && ((marker==0 && !tunedSensor && !tunedPower)
            || (marker==0xA9 && type && pwm>=40 && tunedSensor==type && tunedPower==pwm));
}
inline bool validBlock(uint8_t id,const uint8_t* data,uint8_t n) {
    if(!id || id>Last || !data || n!=length(id)) return false;
    BlockDecoder d(id,data,n);
    bool ok=false;
    switch(id) {
    case Geometry: {
        const uint16_t jog=d.u16(Field::F_RF_MOD_Z_STEP_SIZE);
        const uint16_t stepSizes[]=ACCEPTABLE_STEP_SIZE_TABLE;
        ok=jog>=1 && jog<=stepSizes[NUM_ACCEPTABLE_STEP_SIZE_TABLE-1];
        for(uint8_t a=0;a<3;++a) {
            const float steps=d.f32(shifted(Field::F_XAXIS_STEPS_PER_MM,4*a));
            const float travel=d.f32(shifted(Field::F_X_LENGTH,4*a));
            const uint8_t mode=d.u8(shifted(Field::F_RF_MICRO_STEPS_X,a));
            // Guard numeric conversions and arithmetic; retain retrofit
            // travel flexibility for Y/Z pending mechanical qualification.
            const float maxAccel=a==2?ACCELERATION_MAX_Z:ACCELERATION_MAX_XY;
            // Check the actual products before conversion. ZMotion casts
            // steps/mm and overtravel to uint16_t; X/Y default jog uses
            // lroundf then uint16_t. Positive subnormal steps/mm can also
            // overflow the reciprocal used by updateDerivedParameter().
            const bool narrowSafe=a==2
                ? steps<65536.0f && steps*Z_ENDSTOP_DRIVE_OVER<65536.0f
                : steps*(a==0?DEFAULT_MANUAL_MM_X:DEFAULT_MANUAL_MM_Y)<65535.5f;
            ok &= steps>=FLT_MIN && steps*maxAccel<=4294967040.0f
                && positive(travel,a==0?245.0f:2147483520.0f)
                && steps*travel<=2147483520.0f && mode<=8 && narrowSafe;
        }
        break;
    }
    case Feedrates:
        ok=true;
        for(uint8_t a=0;a<3;++a) {
            const float speed=d.f32(shifted(Field::F_X_MAX_FEEDRATE,4*a));
            const float home=d.f32(shifted(Field::F_X_HOMING_FEEDRATE_PRINT,4*a));
            const float limit=a==0?MAX_FEEDRATE_X:a==1?MAX_FEEDRATE_Y:MAX_FEEDRATE_Z;
            ok &= positive(speed,limit) && positive(home,speed);
        }
        break;
    case Jerk:
        ok=inRange(d.f32(Field::F_MAX_XYJERK),1,33.3f) && inRange(d.f32(Field::F_MAX_ZJERK),.05f,2);
        break;
    case Acceleration:
        ok=rangeU16(d.u16(Field::F_RF_STEP_PACKING_MIN_INTERVAL),MIN_STEP_PACKING_MIN_INTERVAL,MAX_STEP_PACKING_MIN_INTERVAL);
        for(uint8_t a=0;a<6;++a)
            ok &= inRange(d.f32(shifted(Field::F_X_MAX_ACCEL,4*a)),a%3==2?ACCELERATION_MIN_Z:ACCELERATION_MIN_XY,
                a%3==2?ACCELERATION_MAX_Z:ACCELERATION_MAX_XY);
        break;
    case MotorCurrent: {
        const uint8_t maxima[]=MOTOR_CURRENT_MAX;
        ok=true;
        for(uint8_t a=0;a<5;++a)
            ok &= rangeU8(d.u8(shifted(Field::F_RF_MOTOR_CURRENT,a)),MOTOR_CURRENT_MIN,maxima[a]);
        break;
    }
    case Extruder0Motion: case Extruder1Motion: {
        const uint8_t delta=(id-Extruder0Motion)*SlotSize;
        const uint8_t micro=d.u8(shifted(Field::F_RF_MICRO_STEPS_E0,delta));
        if(id==Extruder1Motion && NUM_EXTRUDER==1) {
            ok=micro<=8;
            for(uint8_t a=0;a<n-1;++a) ok &= data[a]==0;
            break;
        }
        const float feed=d.f32(shifted(Field::F_E0_MAX_FEEDRATE,delta));
        const float advance=d.f32(shifted(Field::F_E0_ADVANCE_L,delta));
        // Historical loader/host allowed <5540 steps/mm, <100 mm/s
        // and <10000 acceleration. Use the same envelope on new writes.
        ok=positive(d.f32(shifted(Field::F_E0_STEPS_PER_MM,delta)),5539.999f)
            && positive(feed,99.99999f) && positive(d.f32(shifted(Field::F_E0_MAX_START_FEEDRATE,delta)),feed)
            && positive(d.f32(shifted(Field::F_E0_MAX_ACCELERATION,delta)),10000)
            && (advance==0 || inRange(advance,20,250)) && micro<=8
            && isfinite(d.f32(shifted(Field::F_E0_X_OFFSET,delta)))
            && isfinite(d.f32(shifted(Field::F_E0_Y_OFFSET,delta)));
        break;
    }
    case Hotend0: case Hotend1: {
        if(id==Hotend1 && NUM_EXTRUDER==1) {
            ok=true;for(uint8_t a=0;a<n;++a) ok &= data[a]==0;break;
        }
        const uint8_t delta=(id-Hotend0)*SlotSize;
        ok=validThermal(d,shifted(Field::F_E0_PID_PGAIN,delta),shifted(Field::F_E0_PID_IGAIN,delta),
            shifted(Field::F_E0_PID_DGAIN,delta),shifted(Field::F_E0_DRIVE_MIN,delta),
            shifted(Field::F_E0_DRIVE_MAX,delta),shifted(Field::F_E0_PID_MAX,delta),
            shifted(Field::F_E0_SENSOR_TYPE,delta),shifted(Field::F_PID_CALIBRATION_SLOT_0,delta));
        break;
    }
    case BedPid:
        ok=validThermal(d,Field::F_BED_PID_PGAIN,Field::F_BED_PID_IGAIN,Field::F_BED_PID_DGAIN,
            Field::F_BED_DRIVE_MIN,Field::F_BED_DRIVE_MAX,Field::F_BED_PID_MAX,
            Field::F_RF_HEATED_BED_SENSOR_TYPE,Field::F_PID_CALIBRATION_SLOT_2);
        break;
    case BedSafety: {
        const uint8_t v=d.u8(Field::F_NEW_BED_EMERGENCY_C);ok=!v || (v>=40 && v<=150);break;
    }
    case Preheat:
        ok=validPreheat(d.u16(Field::F_RF_PREHEAT_A_BED),true)
            && validPreheat(d.u16(Field::F_RF_PREHEAT_B_BED),true)
            && validPreheat(d.u16(Field::F_RF_PREHEAT_A_HOTEND),false)
            && validPreheat(d.u16(Field::F_RF_PREHEAT_B_HOTEND),false);
        break;
    case ZOffset: {
        const int32_t v=int32_t(d.u32(Field::F_RF_Z_OFFSET));
        ok=v>=-10000 && v<=10000;break;
    }
    case SenseOffset:
        ok=rangeU16(d.u16(Field::F_RF_MOD_SENSEOFFSET_OFFSET_MAX),1,300)
            && d.u16(Field::F_RF_MOD_SENSEOFFSET_DIGITS)<=EMERGENCY_PAUSE_DIGITS_MAX
            && d.u8(Field::F_RF_MOD_SENSEOFFSET_AUTOSTART)<=1
            && rangeU8(d.u8(Field::F_RF_ZERO_DIGIT_STATE),1,2)
            && rangeU8(d.u8(Field::F_RF_DIGIT_CMP_STATE),1,2);
        break;
    case Pressure: {
        const int32_t pmin=int32_t(d.u32(Field::F_RF_EMERGENCYPAUSEDIGITSMIN));
        const int32_t pmax=int32_t(d.u32(Field::F_RF_EMERGENCYPAUSEDIGITSMAX));
        const int16_t smin=int16_t(d.u16(Field::F_RF_EMERGENCYZSTOPDIGITSMIN));
        const int16_t smax=int16_t(d.u16(Field::F_RF_EMERGENCYZSTOPDIGITSMAX));
        ok=pmin>=EMERGENCY_PAUSE_DIGITS_MIN && pmax<=EMERGENCY_PAUSE_DIGITS_MAX && pmin<=pmax
            && smin>=EMERGENCY_STOP_DIGITS_MIN && smax<=EMERGENCY_STOP_DIGITS_MAX && smin<=smax && smin && smax;
        break;
    }
    case Scan: {
        // Same grid capacity formulas as RF.h; a zero index is the existing
        // unselected sentinel. Actual matrix extents must be checked later.
        const int xmax=(int(X_MAX_LENGTH_PRINT)-(NUM_EXTRUDER==2?0:20))/10+4;
        const int ymax=(int(Y_MAX_LENGTH)-35)/10+4;
        const uint8_t x=d.u8(Field::F_RF_MOD_ZOS_SCAN_POINT_X), y=d.u8(Field::F_RF_MOD_ZOS_SCAN_POINT_Y);
        ok=(x==0 || (x>=(NUM_EXTRUDER==2?1:2) && x<xmax)) && (y==0 || (y>=2 && y<ymax))
            && inRange(d.f32(Field::F_ZSCAN_START_MM),.3f,6);
        break;
    }
    case Wobble:
        ok=rangeI8(int8_t(d.u8(Field::F_RF_MOD_WOBBLE_FIX_PHASEXY)),-100,101)
            && rangeI16(int16_t(d.u16(Field::F_RF_MOD_WOBBLE_FIX_AMPX)),-995,995)
            && rangeI16(int16_t(d.u16(Field::F_RF_MOD_WOBBLE_FIX_AMPY1)),-995,995)
            && rangeI16(int16_t(d.u16(Field::F_RF_MOD_WOBBLE_FIX_AMPY2)),-995,995);
        break;
    case Panel:
        ok=rangeU8(d.u8(Field::F_RF_MOVE_MODE_X),MOVE_MODE_SINGLE_STEPS,MOVE_MODE_SINGLE_MOVE)
            && rangeU8(d.u8(Field::F_RF_MOVE_MODE_Y),MOVE_MODE_SINGLE_STEPS,MOVE_MODE_SINGLE_MOVE)
            && rangeU8(d.u8(Field::F_RF_MOVE_MODE_Z),MOVE_MODE_SINGLE_STEPS,MOVE_MODE_SINGLE_MOVE)
            && rangeU8(d.u8(Field::F_RF_Z_MODE),1,3)
            && d.u8(Field::F_RF_MOVE_MODE_XY_KOSYS)<=1 && d.u8(Field::F_RF_MOVE_POSITION_FEEDRATE)<=1;
        break;
    case Lighting:
        ok=d.u8(Field::F_RF_CASE_LIGHT_MODE)<=1;
#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
        ok &= d.u8(Field::F_RF_FET1_MODE)<=((FET1>=0 && !HOTEND_COOLER_USES_PIN(FET1))?1:0)
            && d.u8(Field::F_RF_FET2_MODE)<=((FET2>=0 && !HOTEND_COOLER_USES_PIN(FET2))?1:0)
            && d.u8(Field::F_RF_FET3_MODE)<=((FET3>=0 && CASE_FAN_PIN<0 && !HOTEND_COOLER_USES_PIN(FET3))?1:0);
#else
        ok &= !d.u8(Field::F_RF_FET1_MODE) && !d.u8(Field::F_RF_FET2_MODE) && !d.u8(Field::F_RF_FET3_MODE);
#endif
#ifdef RGB_LIGHT_R_PIN
        ok &= d.u8(Field::F_RF_RGB_LIGHT_MODE)<=RGB_MODE_MANUAL;
#else
        for(uint8_t a=4;a<n;++a) ok &= data[a]==0;
#endif
        break;
    case PartFan: {
        const uint8_t min=d.u8(Field::F_RF_PART_FAN_PWM_MIN),max=d.u8(Field::F_RF_PART_FAN_PWM_MAX);
        ok=d.u8(Field::F_RF_FAN_MODE)<=1 && rangeU8(d.u8(Field::F_RF_PART_FAN_SPEED),1,7)
            && min && max<255 && int(max)-int(min)>=16;break;
    }
    case System:
        ok=validBaud(d.u32(Field::F_BAUDRATE)) && d.u32(Field::F_MAX_INACTIVE_TIME)<=0x7FFFFFFFUL
            && d.u32(Field::F_STEPPER_INACTIVE_TIME)<=0x7FFFFFFFUL;break;
    case Usage:
        ok=d.u32(Field::F_PRINTING_TIME)<=0x7FFFFFFFUL && isfinite(d.f32(Field::F_PRINTING_DISTANCE))
            && d.f32(Field::F_PRINTING_DISTANCE)>=0;break;
    default:return false;
    }
    return ok && d.good();
}

// Only CRC-valid blocks are considered for semantic failure classification.
// Do not count the same corrupt block twice, or trust a partial read.
template<class Store> uint32_t semanticInvalid(Store& store,const Inspection& state) {
    uint32_t mask=0;
    if(!state.headerValid) return mask;
    uint8_t data[MaxPayload];
    for(uint8_t id=1;id<=Last;++id) {
        if((state.invalid|state.uncertain)&blockBit(id)) continue;
        const uint8_t n=length(id);
        if(!store.readBlock(id,data,n) || !validBlock(id,data,n)) mask|=blockBit(id);
    }
    return mask;
}
}
#endif

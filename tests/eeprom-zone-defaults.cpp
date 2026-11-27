// Run with the same four board/retrofit flags as the legacy storage replay.
#include <cassert>
#include <cstdio>
#include <array>
#include <cstdint>
#define __AVR_ATmega2560__ 1
#include "BoardConfiguration.h"
#include "EepromZoneDefaults.h"
#include "EepromZoneStorage.h"
#include "EepromZoneHostAccess.h"
#include "EepromZoneValidation.h"
using namespace EepromZones;
struct Memory {
    std::array<uint8_t,4096> bytes;
    Memory(){bytes.fill(0xff);}
    uint8_t read(uint16_t pos){assert(pos>=Base && pos<End);return bytes[pos];}
    void update(uint16_t pos,uint8_t v){assert(pos>=Base && pos<End);bytes[pos]=v;}
};
int main(){
    FactoryDefaults defaults(1.0f,5,true,false);
    Memory io;Storage<Memory> store(io,configuredProfile());
    assert(store.initialize(defaults));
    const auto state=store.inspect();
    assert(state.headerValid && state.masterValid && state.clean && !state.invalid && state.alerts==bit(BedSafety));
    for(uint8_t id=1;id<=Last;++id){
        const uint8_t n=length(id);
        uint8_t guard[MaxPayload+2];for(auto& b:guard)b=0xa7;
        assert(defaults(id,guard+1,n));
        assert(guard[0]==0xa7 && guard[n+1]==0xa7);
        uint8_t actual[MaxPayload];
        assert(store.readBlock(id,actual,n));
        for(uint8_t i=0;i<n;++i) assert(actual[i]==guard[i+1]);
        assert(validBlock(id,actual,n));
    }
    uint8_t block[MaxPayload];
    assert(store.readBlock(Geometry,block,length(Geometry)));
    BlockDecoder geom(Geometry,block,length(Geometry));
    assert(geom.f32(Field::F_XAXIS_STEPS_PER_MM)==float(XAXIS_STEPS_PER_MM));
    assert(geom.f32(Field::F_Z_LENGTH)==float(Z_MAX_LENGTH));
    assert(geom.u16(Field::F_RF_MOD_Z_STEP_SIZE)==uint16_t(lroundf(ZAXIS_STEPS_PER_MM*DEFAULT_MANUAL_MM_Z)));
    assert(geom.good());
    assert(store.readBlock(Hotend0,block,length(Hotend0)));
    BlockDecoder h0(Hotend0,block,length(Hotend0));
    assert(h0.f32(Field::F_E0_PID_PGAIN)==EXT0_HOTEND_PROFILE.pidPGain);
    assert(h0.u8(Field::F_E0_DRIVE_MAX)==EXT0_HOTEND_PROFILE.pidDriveMax);
    assert(h0.u8(Field::F_E0_SENSOR_TYPE)==EXT0_TEMPSENSOR_TYPE);
    assert(h0.u8(Field::F_PID_CALIBRATION_SLOT_0)==0);
    assert(store.readBlock(Hotend1,block,length(Hotend1)));
    BlockDecoder h1(Hotend1,block,length(Hotend1));
#if NUM_EXTRUDER > 1
    assert(h1.u8(Field::F_E1_SENSOR_TYPE)==EXT1_TEMPSENSOR_TYPE);
    assert(h1.f32(Field::F_E1_PID_PGAIN)==EXT1_HOTEND_PROFILE.pidPGain);
#else
    for(uint8_t i=0;i<length(Hotend1);++i)assert(block[i]==0);
#endif
    assert(store.readBlock(BedSafety,block,1) && block[0]==0 && !state.eligible(BedSafety));
    assert(store.readBlock(Preheat,block,length(Preheat)));
    BlockDecoder preheat(Preheat,block,length(Preheat));
    assert(preheat.u16(Field::F_RF_PREHEAT_B_BED)==PREHEAT_B_BED_C);
    assert(store.readBlock(System,block,length(System)));
    BlockDecoder system(System,block,length(System));
    assert(system.u32(Field::F_BAUDRATE)==BAUDRATE);
    assert(system.u32(Field::F_STEPPER_INACTIVE_TIME)==uint32_t(STEPPER_INACTIVE_TIME)*1000);
    assert(store.readBlock(Lighting,block,length(Lighting)));
    BlockDecoder light(Lighting,block,length(Lighting));
#ifdef RGB_LIGHT_R_PIN
    assert(light.u8(Field::F_RF_RGB_LIGHT_MODE)==RGB_LIGHT_DEFAULT_MODE);
    assert(light.u8(Field::F_RF_RGB_PRINTING_R)==RGB_PRINTING_R);
    assert(light.u8(Field::F_RF_RGB_MANUAL_B)==RGB_MANUAL_B);
#endif
#ifdef X44_PIN
    if(HOTEND_COOLER_USES_PIN(X44_PIN)) {
        assert(configuredProfile()&1);assert(light.u8(Field::F_RF_FET2_MODE)==0);
    }
#endif
    // Even CRC-valid values are not automatically safe. Test the actual
    // policy using complete board-specific payloads and an altered field.
    const auto rejected=[&](uint8_t id,Field field,uint32_t bits,uint8_t width) {
        uint8_t data[MaxPayload];
        const uint8_t n=length(id);
        assert(defaults(id,data,n) && validBlock(id,data,n));
        const uint8_t offset=address(field)-address(id)-PayloadOffset;
        assert(offset+width<=n);
        for(uint8_t i=0;i<width;++i) data[offset+i]=uint8_t(bits>>(8*i));
        assert(!validBlock(id,data,n));
    };
    rejected(Geometry,Field::F_XAXIS_STEPS_PER_MM,0,4);
    rejected(Geometry,Field::F_YAXIS_STEPS_PER_MM,0x7FC00000UL,4);
    rejected(Geometry,Field::F_Z_LENGTH,0x7F800000UL,4);
    rejected(Geometry,Field::F_RF_MICRO_STEPS_X,9,1);
    rejected(Feedrates,Field::F_X_HOMING_FEEDRATE_PRINT,0x447A0000UL,4);
    rejected(Jerk,Field::F_MAX_ZJERK,0,4);
    rejected(Acceleration,Field::F_RF_STEP_PACKING_MIN_INTERVAL,0,2);
    rejected(MotorCurrent,Field::F_RF_MOTOR_CURRENT,255,1);
    rejected(Extruder0Motion,Field::F_E0_STEPS_PER_MM,0xBF800000UL,4);
    rejected(Extruder0Motion,Field::F_E0_ADVANCE_L,0x41200000UL,4);
    rejected(Hotend0,Field::F_E0_SENSOR_TYPE,255,1);
    rejected(Hotend0,Field::F_E0_PID_IGAIN,0,4);
    rejected(Hotend0,Field::F_PID_CALIBRATION_SLOT_0,0xA9,1);
    rejected(BedPid,Field::F_BED_PID_MAX,39,1);
    rejected(BedSafety,Field::F_NEW_BED_EMERGENCY_C,151,1);
    rejected(Preheat,Field::F_RF_PREHEAT_A_BED,141,2);
    rejected(ZOffset,Field::F_RF_Z_OFFSET,10001,4);
    rejected(SenseOffset,Field::F_RF_MOD_SENSEOFFSET_OFFSET_MAX,301,2);
    rejected(Pressure,Field::F_RF_EMERGENCYZSTOPDIGITSMAX,0,2);
    rejected(Pressure,Field::F_RF_EMERGENCYPAUSEDIGITSMIN,32767,4);
    rejected(Scan,Field::F_RF_MOD_ZOS_SCAN_POINT_Y,255,1);
    rejected(Wobble,Field::F_RF_MOD_WOBBLE_FIX_AMPX,1000,2);
    rejected(Panel,Field::F_RF_MOVE_MODE_Z,0,1);
    rejected(Lighting,Field::F_RF_CASE_LIGHT_MODE,2,1);
    rejected(PartFan,Field::F_RF_PART_FAN_PWM_MIN,254,1);
    rejected(System,Field::F_BAUDRATE,0,4);
    rejected(Usage,Field::F_PRINTING_DISTANCE,0x7FC00000UL,4);
    assert(validPreheat(100,true)); // a preference, not heater permission
    // Store can contain a CRC-correct but semantically invalid payload.
    // The higher-level policy must discover this before applying it.
    struct InvalidProvider {
        FactoryDefaults& defaults;
        bool operator()(uint8_t id,uint8_t* data,uint8_t n) {
            if(!defaults(id,data,n)) return false;
            if(id==Feedrates) for(uint8_t a=0;a<4;++a) data[a]=0;
            return true;
        }
    } invalidProvider{defaults};
    assert(store.commit(bit(Feedrates),invalidProvider));
    const auto structural=store.inspect();
    assert(!structural.invalid && structural.masterValid);
    assert(semanticInvalid(store,structural)==bit(Feedrates));
    assert(store.repair(bit(Feedrates),defaults));
    assert(!semanticInvalid(store,store.inspect()));
    // Invalid block length and field/overlap are rejected, not a partial
    // provider silently completing an otherwise CRC-valid record.
    assert(!defaults(Admin,block,1) && !defaults(Geometry,block,1));
    BlockEncoder invalid(Geometry,block,length(Geometry));
    invalid.u32(Field::F_PRINTING_TIME,0);assert(!invalid.complete());
    BlockEncoder overlap(BedSafety,block,1);
    overlap.u8(Field::F_NEW_BED_EMERGENCY_C,0);assert(overlap.complete());
    overlap.u8(Field::F_NEW_BED_EMERGENCY_C,0);assert(!overlap.complete());
    // M205/M206 use one inventory, including actual fitted-tool and
    // output ownership restrictions. No metadata, padding, old address,
    // alternate wire type or interior byte is accepted.
    uint8_t typeAt[4096]={};
    uint8_t accessAt[4096]={};
    unsigned fields=0;
#define RF_ZONE_FIELD(type,start,label,digits,writable) do { \
        const uint16_t pos=(start);const uint8_t id=blockOf(pos); \
        const uint8_t width=(type)==0?1:(type)==1?2:4; \
        assert(id && id<=Last && pos>=address(id)+PayloadOffset); \
        assert(pos+width<=address(id)+PayloadOffset+length(id)); \
        assert(!typeAt[pos]);typeAt[pos]=(type)+1; \
        accessAt[pos]=uint8_t((writable)?HostAccess::Writable:HostAccess::ReadOnly); \
        ++fields; \
    } while(false)
#include "EepromZoneHostFields.h"
#undef RF_ZONE_FIELD
    for(int32_t pos=0;pos<4096;++pos)
        for(uint8_t wire=0;wire<5;++wire) {
            const auto expected=typeAt[pos]==wire+1
                ? static_cast<HostAccess>(accessAt[pos]):HostAccess::Unknown;
            assert(hostAccess(pos,wire)==expected);
        }
    assert(hostAccess(-1,0)==HostAccess::Unknown);
    assert(hostAccess(65536L+address(Field::F_NEW_BED_EMERGENCY_C),0)==HostAccess::Unknown);
    assert(hostAccess(address(Field::F_NEW_BED_EMERGENCY_C),0)==HostAccess::ReadOnly);
    assert(hostAccess(address(Field::F_PRINTING_TIME),2)==HostAccess::ReadOnly);
    assert(hostAccess(address(Field::F_PID_CALIBRATION_SLOT_0),0)==HostAccess::Unknown);
#if NUM_EXTRUDER == 1
    assert(hostAccess(address(Field::F_E1_STEPS_PER_MM),3)==HostAccess::Unknown);
#endif
#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    if(HOTEND_COOLER_USES_PIN(FET2))
        assert(hostAccess(address(Field::F_RF_FET2_MODE),0)==HostAccess::ReadOnly);
#endif
    printf("New EEPROM host whitelist: %u fields, 20480 address/type cases; PASS\\n",fields);
    printf("EEPROM zone factory profile %04x: all 22 blocks initialized; PASS\n",unsigned(configuredProfile()));
}

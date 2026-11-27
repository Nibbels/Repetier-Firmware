// Run with the same four board/retrofit flags as the legacy storage replay.
#include <cassert>
#include <cstdio>
#include <array>
#include <cstdint>
#define __AVR_ATmega2560__ 1
#include "BoardConfiguration.h"
#include "EepromZoneDefaults.h"
#include "EepromZoneStorage.h"
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
    // Invalid block length and field/overlap are rejected, not a partial
    // provider silently completing an otherwise CRC-valid record.
    assert(!defaults(Admin,block,1) && !defaults(Geometry,block,1));
    BlockEncoder invalid(Geometry,block,length(Geometry));
    invalid.u32(Field::F_PRINTING_TIME,0);assert(!invalid.complete());
    BlockEncoder overlap(BedSafety,block,1);
    overlap.u8(Field::F_NEW_BED_EMERGENCY_C,0);assert(overlap.complete());
    overlap.u8(Field::F_NEW_BED_EMERGENCY_C,0);assert(!overlap.complete());
    printf("EEPROM zone factory profile %04x: all 22 blocks initialized; PASS\n",unsigned(configuredProfile()));
}

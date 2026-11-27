// Run with the same four board/retrofit flags as the legacy storage replay.
#include <cassert>
#include <cstdio>
#include <array>
#include <cstdint>
#include <initializer_list>
#define __AVR_ATmega2560__ 1
#include "BoardConfiguration.h"
#include "EepromZoneDefaults.h"
#include "EepromZoneStorage.h"
#include "EepromZoneHostAccess.h"
#include "EepromZoneValidation.h"
#include "EepromZoneSession.h"
using namespace EepromZones;
struct Memory {
    std::array<uint8_t,4096> bytes;
    Memory(){bytes.fill(0xff);}
    uint8_t read(uint16_t pos){assert(pos>=Base && pos<End);return bytes[pos];}
    void update(uint16_t pos,uint8_t v){assert(pos>=Base && pos<End);bytes[pos]=v;}
};
struct BusyDevice {
    std::array<uint8_t,4096> bytes;
    bool irq=true, stuck=false;
    bool& owner;
    uint32_t clock=0xfffffff0UL, readyAt=clock;
    unsigned programs=0, services=0, nestedRejected=0;
    explicit BusyDevice(bool& busy):owner(busy) {bytes.fill(0xff);}
    bool foreground() const {return irq;}
    bool ready() const {return !stuck && int32_t(clock-readyAt)>=0;}
    uint32_t now() const {return clock;}
    uint8_t readReady(uint16_t pos){assert(ready() && irq);return bytes[pos];}
    void updateReady(uint16_t pos,uint8_t v){
        assert(ready() && irq && owner && bytes[pos]!=v);
        bytes[pos]=v;++programs;readyAt=clock+4;
    }
    void service() {
        assert(owner && irq);
        SessionGuard nested(owner);
        assert(!nested.entered() && owner);++nestedRejected;
        ++clock;++services;
    }
};
static void sessionReplay(FactoryDefaults& defaults) {
    bool busy=false;
    BusyDevice dev(busy);
    Session<BusyDevice> session(dev,busy,configuredProfile());
    Inspection state;
    assert(session.restore(defaults,state)==Result::Ok);
    assert(!busy && state.alerts==AllAlerts && dev.services==dev.nestedRejected && dev.programs>0);
    // Cold start crosses the uint32_t millisecond rollover.
    assert(dev.clock<0xfffffff0UL);
    unsigned count=0;
    auto consumer=[&](uint8_t id,const uint8_t* data,uint8_t n,uint32_t alerts){
        assert(busy && alerts==AllAlerts && validBlock(id,data,n));++count;return true;
    };
    assert(session.load(consumer,state)==Result::Ok && count==Last && !busy);
    unsigned writes=dev.programs;
    assert(session.save(blockBit(Feedrates),defaults)==Result::Ok && dev.programs==writes);
    struct Edit {
        FactoryDefaults& base;
        uint8_t mode;
        bool operator()(uint8_t id,uint8_t* data,uint8_t n) {
            if(!base(id,data,n) || id!=Panel)return false;
            data[0]=mode;return true;
        }
    } edit{defaults,0};
    assert(session.save(blockBit(Panel),edit)==Result::Invalid && dev.programs==writes);
    edit.mode=MOVE_MODE_1_MM;
    assert(session.save(blockBit(Panel),edit)==Result::Ok && dev.programs>writes);
    // A field edit starts from the committed payload, not from defaults or
    // unrelated live settings. Keep the previously saved X move mode.
    const Patch patches[]={Patch::byte(Field::F_RF_MOVE_MODE_Z,MOVE_MODE_1_MM),
                           Patch::word(Field::F_RF_MOD_SENSEOFFSET_OFFSET_MAX,120)};
    assert(session.patch(patches,2)==Result::Ok);
    auto patched=[&](uint8_t id,const uint8_t* data,uint8_t n,uint32_t){
        BlockDecoder d(id,data,n);
        if(id==Panel) {
            assert(d.u8(Field::F_RF_MOVE_MODE_X)==MOVE_MODE_1_MM);
            assert(d.u8(Field::F_RF_MOVE_MODE_Z)==MOVE_MODE_1_MM);
        }
        if(id==SenseOffset) assert(d.u16(Field::F_RF_MOD_SENSEOFFSET_OFFSET_MAX)==120);
        return true;
    };
    assert(session.load(patched,state)==Result::Ok);
    writes=dev.programs;
    assert(session.patch(patches,2)==Result::Ok && dev.programs==writes);
    const Patch wrong[]={Patch::word(Field::F_RF_MOVE_MODE_Z,1)};
    const Patch duplicate[]={patches[0],patches[0]};
    const Patch bed[]={Patch::byte(Field::F_NEW_BED_EMERGENCY_C,120)};
    const Patch bad[]={Patch::byte(Field::F_RF_MOVE_MODE_Z,0)};
    assert(session.patch(wrong,1)==Result::Invalid);
    assert(session.patch(duplicate,2)==Result::Invalid);
    assert(session.patch(bed,1)==Result::Invalid);
    assert(session.patch(bad,1)==Result::Invalid);
    assert(session.save(blockBit(BedSafety),defaults)==Result::Invalid);
    assert(dev.programs==writes);
    assert(session.acknowledge(AllAlerts)==Result::Ok);
    assert(session.commission(90)==Result::Ok);
    auto commissioned=[&](uint8_t id,const uint8_t* data,uint8_t n,uint32_t alerts){
        assert(busy && !alerts && validBlock(id,data,n));
        if(id==BedSafety) assert(data[0]==90);
        return true;
    };
    assert(session.load(commissioned,state)==Result::Ok && !busy);
    // Early UART selection is read-only and validates the complete system
    // block and global commit state. An unrelated raw payload error is
    // not silently blessed; full boot inspection still finds it.
    uint8_t readback[MaxPayload];uint32_t alerts=AllAlerts;
    writes=dev.programs;
    assert(session.read(System,readback,length(System),alerts)==Result::Ok && !alerts);
    BlockDecoder systemRead(System,readback,length(System));
    assert(systemRead.u32(Field::F_BAUDRATE)==BAUDRATE && dev.programs==writes);
    dev.bytes[address(System)+PayloadOffset]^=1;
    assert(session.read(System,readback,length(System),alerts)==Result::NeedsRecovery);
    dev.bytes[address(System)+PayloadOffset]^=1;
    const Patch limits[]={Patch::byte(Field::F_E0_DRIVE_MIN,55),Patch::byte(Field::F_E0_DRIVE_MAX,66)};
    unsigned publications=0;
    auto publish=[&](uint8_t id,const uint8_t* data,uint8_t n,uint32_t pending,const Patch* p,uint8_t count){
        assert(busy && id==Hotend0 && !pending && p==limits && count==2);
        BlockDecoder decoder(id,data,n);
        assert(decoder.u8(Field::F_E0_DRIVE_MIN)==55 && decoder.u8(Field::F_E0_DRIVE_MAX)==66);
        assert(dev.ready());
        ++publications;return true;
    };
    assert(session.patchAndPublish(limits,2,publish)==Result::Ok && publications==1);
    writes=dev.programs;
    assert(session.patchAndPublish(limits,2,publish)==Result::Ok && publications==2 && dev.programs==writes);
    const Patch invalidLimit[]={Patch::byte(Field::F_E0_DRIVE_MIN,0)};
    assert(session.patchAndPublish(invalidLimit,1,publish)==Result::Invalid && publications==2 && dev.programs==writes);
    auto refuse=[](uint8_t,const uint8_t*,uint8_t,uint32_t,const Patch*,uint8_t){return false;};
    assert(session.patchAndPublish(limits,2,refuse)==Result::PublishRejected);
    assert(!busy);
    // Real M206 transaction semantics: exact typed address, no-op echo,
    // no mutation until admission, atomic revoke of tuning identity.
    struct Tuned {
        FactoryDefaults& base;
        bool operator()(uint8_t id,uint8_t* d,uint8_t n) {
            if(id!=Hotend0 || !base(id,d,n))return false;
            BlockDecoder decoder(id,d,n);
            const uint8_t sensor=decoder.u8(Field::F_E0_SENSOR_TYPE);
            const uint8_t power=decoder.u8(Field::F_E0_PID_MAX);
            const uint8_t tag=address(Field::F_PID_CALIBRATION_SLOT_0)-address(id)-PayloadOffset;
            d[tag]=0xA9;d[tag+1]=sensor;d[tag+2]=power;return true;
        }
    } tuned{defaults};
    assert(session.save(blockBit(Hotend0),tuned)==Result::Ok);
    unsigned admissions=0,applies=0;
    bool allow=true;
    auto admit=[&](uint8_t id,uint32_t pending,const Patch* p,uint8_t count){
        assert(busy && id==Hotend0 && !pending && p && count>=1 && count<=2);
        ++admissions;return allow;
    };
    auto hostPublish=[&](uint8_t id,const uint8_t* data,uint8_t n,uint32_t pending,const Patch* p,uint8_t count){
        assert(busy && id==Hotend0 && !pending && p && dev.ready());
        BlockDecoder decoder(id,data,n);
        if(count==2) {
            assert(p[1].pos==address(Field::F_PID_CALIBRATION_SLOT_0));
            assert(decoder.u8(Field::F_PID_CALIBRATION_SLOT_0)==0);
        } else {
            assert(p[0].pos==address(Field::F_E0_DRIVE_MAX));
            assert(decoder.u8(Field::F_PID_CALIBRATION_SLOT_0)==0xA9);
        }
        ++applies;return true;
    };
    const int32_t dmaxPos=address(Field::F_E0_DRIVE_MAX);
    const int32_t gainPos=address(Field::F_E0_PID_PGAIN);
    const int32_t bedPos=address(Field::F_NEW_BED_EMERGENCY_C);
    assert(session.hostEdit(dmaxPos,0,73,0,admit,hostPublish)==Result::Ok && admissions==1 && applies==1);
    writes=dev.programs;
    assert(session.hostEdit(dmaxPos,0,73,0,admit,hostPublish)==Result::Ok);
    assert(session.hostEdit(bedPos,0,90,0,admit,hostPublish)==Result::Ok);
    assert(session.hostEdit(bedPos,0,120,0,admit,hostPublish)==Result::ReadOnly);
    assert(session.hostEdit(dmaxPos,0,256,0,admit,hostPublish)==Result::Invalid);
    assert(session.hostEdit(dmaxPos,0,0,0,admit,hostPublish)==Result::Invalid);
    assert(session.hostEdit(dmaxPos,256,73,0,admit,hostPublish)==Result::Invalid);
    assert(session.hostEdit(dmaxPos+65536L,0,73,0,admit,hostPublish)==Result::Invalid);
    assert(session.hostEdit(gainPos,3,0,NAN,admit,hostPublish)==Result::Invalid);
    assert(dev.programs==writes && admissions==1 && applies==1);
    allow=false;
    assert(session.hostEdit(gainPos,3,0,12.345f,admit,hostPublish)==Result::AdmissionRejected);
    assert(dev.programs==writes && admissions==2 && applies==1);
    allow=true;
    assert(session.hostEdit(gainPos,3,0,12.345f,admit,hostPublish)==Result::Ok && admissions==3 && applies==2);
    assert(session.read(Hotend0,readback,length(Hotend0),alerts)==Result::Ok);
    BlockDecoder finalThermal(Hotend0,readback,length(Hotend0));
    assert(finalThermal.f32(Field::F_E0_PID_PGAIN)==12.345f && !finalThermal.u8(Field::F_PID_CALIBRATION_SLOT_0));
    assert(finalThermal.u8(Field::F_E0_DRIVE_MAX)==73);
    // Physical busy fault cannot hang or be mistaken for successful readback.
    dev.stuck=true;const uint32_t started=dev.clock;writes=dev.programs;
    assert(session.restore(defaults,state)==Result::TransportFault);
    assert(dev.clock-started==20 && dev.programs==writes && !busy);
    dev.stuck=false;
    dev.irq=false;
    assert(session.restore(defaults,state)==Result::TransportFault && dev.programs==writes);
    dev.irq=true;
    {
        SessionGuard outer(busy);
        assert(outer.entered() && session.restore(defaults,state)==Result::Busy);
        assert(busy);
    }
    assert(!busy);
    ReadyByteIO<BusyDevice> out(dev);
    assert(out.read(0)==0xff && !out.good()); // never access legacy region
    printf("EEPROM ready wait / checked session: %u services, recursion blocked; PASS\\n",dev.services);
}
// Byte-exact baseline captured from the pre-compaction factory provider,
// including all constant blocks and varying runtime-provided defaults.
// Update the pinned profile digest only for an intentional settings change.
static void factoryImageReplay() {
    uint32_t digest=2166136261UL;
    unsigned bytes=0;
    for(float lift:{1.0f,0.3f,6.0f})
        for(unsigned idx:{0u,5u})
            for(bool xy:{false,true})
                for(bool fr:{false,true}) {
                    FactoryDefaults defaults(lift,idx,xy,fr);
                    for(uint8_t id=1;id<=Last;++id) {
                        uint8_t guard[MaxPayload+2];
                        for(auto& b:guard)b=0xa7;
                        const uint8_t n=length(id);
                        assert(defaults(id,guard+1,n));
                        assert(guard[0]==0xa7 && guard[n+1]==0xa7);
                        for(uint8_t i=1;i<=n;++i) {
                            digest=(digest^guard[i])*16777619UL;
                            ++bytes;
                        }
                    }
                }
    uint32_t expected=0;
    switch(configuredProfile()) {
    case 0x0d10:expected=0xd4905079UL;break;
    case 0x0e20:expected=0x941b89c9UL;break;
    case 0x0f20:expected=0x6e0c6269UL;break;
    case 0x0e11:expected=0xa3e79259UL;break;
    default:assert(false);
    }
    assert(bytes==7296 && digest==expected);
    printf("Factory payload baseline: %u bytes match; PASS\n",bytes);
}
template<class T,class L,class H> static unsigned compareIntegerRange(int first,int last,L lo,H hi,bool (*range)(T,T,T)) {
    assert(float(T(lo))==float(lo) && float(T(hi))==float(hi));
    unsigned n=0;
    for(int i=first;i<=last;++i) {
        const T v=static_cast<T>(i);
        assert(range(v,T(lo),T(hi))==inRange(v,lo,hi));
        ++n;
    }
    return n;
}
static void integerRangeReplay() {
    unsigned n=0;
    n+=compareIntegerRange<uint16_t>(0,65535,MIN_STEP_PACKING_MIN_INTERVAL,MAX_STEP_PACKING_MIN_INTERVAL,rangeU16);
    n+=compareIntegerRange<uint16_t>(0,65535,1,300,rangeU16);
    n+=compareIntegerRange<int16_t>(-32768,32767,-995,995,rangeI16);
    n+=compareIntegerRange<int8_t>(-128,127,-100,101,rangeI8);
    const uint8_t maxima[]=MOTOR_CURRENT_MAX;
    for(auto m:maxima)n+=compareIntegerRange<uint8_t>(0,255,MOTOR_CURRENT_MIN,m,rangeU8);
    n+=compareIntegerRange<uint8_t>(0,255,1,2,rangeU8);
    n+=compareIntegerRange<uint8_t>(0,255,MOVE_MODE_SINGLE_STEPS,MOVE_MODE_SINGLE_MOVE,rangeU8);
    n+=compareIntegerRange<uint8_t>(0,255,1,3,rangeU8);
    n+=compareIntegerRange<uint8_t>(0,255,1,7,rangeU8);
    assert(n==199168);
    printf("Integral settings bounds: %u cases match historical float comparison; PASS\n",n);
}
// Independent address/width oracle: preserve the old codec's byte order,
// full address domain and sticky invalid-read state, including padding.
static void codecReplay() {
    unsigned long reads=0;
    for(uint8_t id=1;id<=Last;++id) {
        uint8_t data[MaxPayload];
        for(uint8_t i=0;i<MaxPayload;++i)data[i]=uint8_t(137U+53U*i+17U*id);
        const uint8_t n=length(id);
        const uint32_t first=address(id)+PayloadOffset;
        for(uint32_t pos=0;pos<65536UL;++pos) {
            for(uint8_t width:{1,2,4}) {
                const bool valid=pos>=first && pos+width<=first+n;
                uint32_t expected=0;
                if(valid)for(uint8_t b=0;b<width;++b)expected|=uint32_t(data[pos-first+b])<<(8*b);
                BlockDecoder decoder(id,data,n);
                const Field f=static_cast<Field>(pos);
                const uint32_t actual=width==1?decoder.u8(f):width==2?decoder.u16(f):decoder.u32(f);
                assert(actual==expected && decoder.good()==valid);
                if(!valid) {
                    assert(decoder.u8(static_cast<Field>(first))==0);
                    assert(!decoder.good());
                }
                ++reads;
            }
        }
    }
    assert(reads==4325376UL);
    // Every 16-bit payload, with an independent byte pattern in the upper
    // word, covers all signed/unsigned bit patterns and fixed shifts.
    for(uint32_t value=0;value<65536UL;++value) {
        uint8_t data[4]={uint8_t(value),uint8_t(value>>8),uint8_t(~value),uint8_t((~value)>>8)};
        BlockDecoder decoder(ZOffset,data,4);
        const auto f=Field::F_RF_Z_OFFSET;
        assert(decoder.u8(f)==uint8_t(value));
        assert(decoder.u16(f)==uint16_t(value));
        assert(decoder.u32(f)==(value|((~value&0xFFFFUL)<<16)));
        assert(decoder.good());
    }
    puts("EEPROM codec: 4325376 address/width checks and 65536 payload checks; PASS");
}
int main(){
    codecReplay();
    factoryImageReplay();
    integerRangeReplay();
    FactoryDefaults defaults(1.0f,5,true,false);
    sessionReplay(defaults);
    Memory io;Storage<Memory> store(io,configuredProfile());
    assert(store.initialize(defaults));
    const auto state=store.inspect();
    assert(state.headerValid && state.masterValid && state.clean && !state.invalid && state.alerts==blockBit(BedSafety));
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
    rejected(Geometry,Field::F_XAXIS_STEPS_PER_MM,1,4); // subnormal binary32
    const auto floatBits=[](float v){uint32_t bits;memcpy(&bits,&v,4);return bits;};
    rejected(Geometry,Field::F_XAXIS_STEPS_PER_MM,floatBits(700000.0f),4);
    rejected(Geometry,Field::F_YAXIS_STEPS_PER_MM,floatBits(700000.0f),4);
    rejected(Geometry,Field::F_ZAXIS_STEPS_PER_MM,floatBits(65536.0f),4);
    rejected(Geometry,Field::F_ZAXIS_STEPS_PER_MM,floatBits(65536.0f/Z_ENDSTOP_DRIVE_OVER+1.0f),4);
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
    assert(store.commit(blockBit(Feedrates),invalidProvider));
    const auto structural=store.inspect();
    assert(!structural.invalid && structural.masterValid);
    assert(semanticInvalid(store,structural)==blockBit(Feedrates));
    assert(store.repair(blockBit(Feedrates),defaults));
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

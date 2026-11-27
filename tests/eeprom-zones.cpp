// Host replay executes the actual candidate storage engine, not a separate
// mathematical implementation. The backend models a byte as the physical
// write unit and can cut power before/after or during any byte programming.
#include "EepromZoneRecovery.h"
#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>
#include <stdexcept>
using namespace EepromZones;
struct PowerCut {};
struct Memory {
    std::array<uint8_t,4096> bytes;
    unsigned reads=0, writes=0, cut=0;
    int torn=-1;
    bool stuck=false;
    Memory() {
        for(unsigned i=0;i<bytes.size();++i) bytes[i]=uint8_t(i*37U+19U);
        for(uint16_t p=Base;p<End;++p) bytes[p]=0xff;
    }
    uint8_t read(uint16_t pos) {
        assert(pos>=Base && pos<End && blockOf(pos)<=Last);
        ++reads;return bytes[pos];
    }
    void update(uint16_t pos,uint8_t v) {
        assert(pos>=Base && pos<End && blockOf(pos)<=Last);
        assert(bytes[pos]!=v);
        ++writes;
        if(cut==writes) {
            if(stuck) return;
            if(torn>=0) bytes[pos]=uint8_t(torn);
            throw PowerCut{};
        }
        bytes[pos]=v;
    }
    void resetCounts() { reads=writes=cut=0;torn=-1;stuck=false; }
};
struct Values {
    uint8_t salt=0;
    uint8_t bed=120;
    bool operator()(uint8_t id,uint8_t* dst,uint8_t n) {
        assert(n==length(id));
        for(uint8_t i=0;i<n;++i) dst[i]=uint8_t(id*7+i+salt);
        if(id==BedSafety) dst[0]=bed;
        return true;
    }
};
constexpr uint16_t Profile=0x1004;
using Store=Storage<Memory>;
static uint8_t cutoff(Store& s) {
    const auto state=s.inspect();
    uint8_t c=0;
    return state.eligible(BedSafety) && s.readBlock(BedSafety,&c,1) ? c : 0;
}
static void assertOutside(const Memory& a,const Memory& b) {
    for(unsigned p=0;p<a.bytes.size();++p) {
        if(p<Base || p>=End || blockOf(p)>Last)
            assert(a.bytes[p]==b.bytes[p]);
        if(p>=address(1) && p<address(Last+1)
           && (p-address(blockOf(p)))>=unsigned(PayloadOffset+length(blockOf(p))))
            assert(a.bytes[p]==b.bytes[p]);
    }
}
static bool sameBlock(Store& s,uint8_t id,Values& expected) {
    uint8_t a[MaxPayload],b[MaxPayload];const uint8_t n=length(id);
    return s.readBlock(id,a,n) && expected(id,b,n) && !memcmp(a,b,n);
}
static unsigned long cases=0;
template<class Op,class Check> static void cuts(const Memory& base,Op op,Check check) {
    Memory complete=base;complete.resetCounts();Store store(complete,Profile);
    assert(op(store)); const unsigned total=complete.writes;
    assert(total>0); assertOutside(base,complete); check(complete,true);
    // Every write interrupted before programming (-1), after programming
    // (the intended value is among 0..255), or at any possible torn byte.
    for(unsigned at=1;at<=total;++at) {
        for(int torn=-1;torn<256;++torn) {
            Memory m=base;m.resetCounts();m.cut=at;m.torn=torn;
            Store s(m,Profile);
            try { (void)op(s);assert(false); } catch(const PowerCut&) {}
            assertOutside(base,m);m.resetCounts();check(m,false);++cases;
        }
        Memory m=base;m.resetCounts();m.cut=at;m.stuck=true;Store s(m,Profile);
        // The rejected update is caught by readback, not a successful commit.
        assert(!op(s));assertOutside(base,m);m.resetCounts();check(m,false);++cases;
    }
}
int main() {
    struct FieldWidth {Field field;uint8_t width;};
    const FieldWidth fields[]={
    {Field::F_XAXIS_STEPS_PER_MM, 4},
    {Field::F_YAXIS_STEPS_PER_MM, 4},
    {Field::F_ZAXIS_STEPS_PER_MM, 4},
    {Field::F_X_LENGTH, 4},
    {Field::F_Y_LENGTH, 4},
    {Field::F_Z_LENGTH, 4},
    {Field::F_RF_MICRO_STEPS_X, 1},
    {Field::F_RF_MICRO_STEPS_Y, 1},
    {Field::F_RF_MICRO_STEPS_Z, 1},
    {Field::F_RF_MOD_Z_STEP_SIZE, 2},
    {Field::F_X_MAX_FEEDRATE, 4},
    {Field::F_Y_MAX_FEEDRATE, 4},
    {Field::F_Z_MAX_FEEDRATE, 4},
    {Field::F_X_HOMING_FEEDRATE_PRINT, 4},
    {Field::F_Y_HOMING_FEEDRATE_PRINT, 4},
    {Field::F_Z_HOMING_FEEDRATE_PRINT, 4},
    {Field::F_MAX_XYJERK, 4},
    {Field::F_MAX_ZJERK, 4},
    {Field::F_X_MAX_ACCEL, 4},
    {Field::F_Y_MAX_ACCEL, 4},
    {Field::F_Z_MAX_ACCEL, 4},
    {Field::F_X_MAX_TRAVEL_ACCEL, 4},
    {Field::F_Y_MAX_TRAVEL_ACCEL, 4},
    {Field::F_Z_MAX_TRAVEL_ACCEL, 4},
    {Field::F_RF_STEP_PACKING_MIN_INTERVAL, 2},
    {Field::F_RF_MOTOR_CURRENT, 5},
    {Field::F_E0_STEPS_PER_MM, 4},
    {Field::F_E0_MAX_FEEDRATE, 4},
    {Field::F_E0_MAX_START_FEEDRATE, 4},
    {Field::F_E0_MAX_ACCELERATION, 4},
    {Field::F_E0_ADVANCE_L, 4},
    {Field::F_E0_X_OFFSET, 4},
    {Field::F_E0_Y_OFFSET, 4},
    {Field::F_RF_MICRO_STEPS_E0, 1},
    {Field::F_E1_STEPS_PER_MM, 4},
    {Field::F_E1_MAX_FEEDRATE, 4},
    {Field::F_E1_MAX_START_FEEDRATE, 4},
    {Field::F_E1_MAX_ACCELERATION, 4},
    {Field::F_E1_ADVANCE_L, 4},
    {Field::F_E1_X_OFFSET, 4},
    {Field::F_E1_Y_OFFSET, 4},
    {Field::F_RF_MICRO_STEPS_E1, 1},
    {Field::F_E0_PID_PGAIN, 4},
    {Field::F_E0_PID_IGAIN, 4},
    {Field::F_E0_PID_DGAIN, 4},
    {Field::F_E0_DRIVE_MIN, 1},
    {Field::F_E0_DRIVE_MAX, 1},
    {Field::F_E0_PID_MAX, 1},
    {Field::F_E0_SENSOR_TYPE, 1},
    {Field::F_PID_CALIBRATION_SLOT_0, 3},
    {Field::F_E0_COOLER_SPEED, 1},
    {Field::F_E1_PID_PGAIN, 4},
    {Field::F_E1_PID_IGAIN, 4},
    {Field::F_E1_PID_DGAIN, 4},
    {Field::F_E1_DRIVE_MIN, 1},
    {Field::F_E1_DRIVE_MAX, 1},
    {Field::F_E1_PID_MAX, 1},
    {Field::F_E1_SENSOR_TYPE, 1},
    {Field::F_PID_CALIBRATION_SLOT_1, 3},
    {Field::F_E1_COOLER_SPEED, 1},
    {Field::F_BED_PID_PGAIN, 4},
    {Field::F_BED_PID_IGAIN, 4},
    {Field::F_BED_PID_DGAIN, 4},
    {Field::F_BED_DRIVE_MIN, 1},
    {Field::F_BED_DRIVE_MAX, 1},
    {Field::F_BED_PID_MAX, 1},
    {Field::F_RF_HEATED_BED_SENSOR_TYPE, 1},
    {Field::F_PID_CALIBRATION_SLOT_2, 3},
    {Field::F_NEW_BED_EMERGENCY_C, 1},
    {Field::F_RF_PREHEAT_A_BED, 2},
    {Field::F_RF_PREHEAT_A_HOTEND, 2},
    {Field::F_RF_PREHEAT_B_BED, 2},
    {Field::F_RF_PREHEAT_B_HOTEND, 2},
    {Field::F_RF_Z_OFFSET, 4},
    {Field::F_RF_MOD_SENSEOFFSET_OFFSET_MAX, 2},
    {Field::F_RF_MOD_SENSEOFFSET_DIGITS, 2},
    {Field::F_RF_MOD_SENSEOFFSET_AUTOSTART, 1},
    {Field::F_RF_ZERO_DIGIT_STATE, 1},
    {Field::F_RF_DIGIT_CMP_STATE, 1},
    {Field::F_RF_EMERGENCYPAUSEDIGITSMIN, 4},
    {Field::F_RF_EMERGENCYPAUSEDIGITSMAX, 4},
    {Field::F_RF_EMERGENCYZSTOPDIGITSMIN, 2},
    {Field::F_RF_EMERGENCYZSTOPDIGITSMAX, 2},
    {Field::F_RF_MOD_ZOS_SCAN_POINT_X, 1},
    {Field::F_RF_MOD_ZOS_SCAN_POINT_Y, 1},
    {Field::F_ZSCAN_START_MM, 4},
    {Field::F_RF_MOD_WOBBLE_FIX_PHASEXY, 1},
    {Field::F_RF_MOD_WOBBLE_FIX_AMPX, 2},
    {Field::F_RF_MOD_WOBBLE_FIX_AMPY1, 2},
    {Field::F_RF_MOD_WOBBLE_FIX_AMPY2, 2},
    {Field::F_RF_MOVE_MODE_X, 1},
    {Field::F_RF_MOVE_MODE_Y, 1},
    {Field::F_RF_MOVE_MODE_Z, 1},
    {Field::F_RF_Z_MODE, 1},
    {Field::F_RF_MOVE_MODE_XY_KOSYS, 1},
    {Field::F_RF_MOVE_POSITION_FEEDRATE, 1},
    {Field::F_RF_CASE_LIGHT_MODE, 1},
    {Field::F_RF_FET1_MODE, 1},
    {Field::F_RF_FET2_MODE, 1},
    {Field::F_RF_FET3_MODE, 1},
    {Field::F_RF_RGB_LIGHT_MODE, 1},
    {Field::F_RF_RGB_PRINTING_R, 1},
    {Field::F_RF_RGB_PRINTING_G, 1},
    {Field::F_RF_RGB_PRINTING_B, 1},
    {Field::F_RF_RGB_HEATING_R, 1},
    {Field::F_RF_RGB_HEATING_G, 1},
    {Field::F_RF_RGB_HEATING_B, 1},
    {Field::F_RF_RGB_COOLING_R, 1},
    {Field::F_RF_RGB_COOLING_G, 1},
    {Field::F_RF_RGB_COOLING_B, 1},
    {Field::F_RF_RGB_IDLE_R, 1},
    {Field::F_RF_RGB_IDLE_G, 1},
    {Field::F_RF_RGB_IDLE_B, 1},
    {Field::F_RF_RGB_MANUAL_R, 1},
    {Field::F_RF_RGB_MANUAL_G, 1},
    {Field::F_RF_RGB_MANUAL_B, 1},
    {Field::F_RF_FAN_MODE, 1},
    {Field::F_RF_PART_FAN_SPEED, 1},
    {Field::F_RF_PART_FAN_PWM_MIN, 1},
    {Field::F_RF_PART_FAN_PWM_MAX, 1},
    {Field::F_BAUDRATE, 4},
    {Field::F_MAX_INACTIVE_TIME, 4},
    {Field::F_STEPPER_INACTIVE_TIME, 4},
    {Field::F_PRINTING_TIME, 4},
    {Field::F_PRINTING_DISTANCE, 4},
    };
    bool occupied[1024]={};unsigned payload=0;
    for(const auto& f:fields) {
        const uint16_t start=address(f.field);const uint8_t id=blockOf(start);
        assert(id && id<=Last && start>=address(id)+PayloadOffset);
        assert(start+f.width<=address(id)+PayloadOffset+length(id));
        for(uint8_t i=0;i<f.width;++i) {
            assert(!occupied[start+i-Base]);occupied[start+i-Base]=true;++payload;
        }
    }
    assert(payload==304 && sizeof(fields)/sizeof(fields[0])==125);
    uint16_t c16=0xffff;uint32_t c32=0xffffffff;
    for(char c:"123456789") if(c) {c16=crc16Byte(c16,c);c32=crc32Byte(c32,c);}
    assert(c16==0x29b1 && (c32^0xffffffffUL)==0xcbf43926UL);
    Values factory,changed;changed.salt=31;changed.bed=90;
    Memory blank;Store blankStore(blank,Profile);assert(!blankStore.inspect().headerValid);
    cuts(blank,[&](Store& s){return s.initialize(factory);},[&](Memory& m,bool done){
        Store s(m,Profile);const auto i=s.inspect();assert(cutoff(s)==0);
        if(i.headerValid) {assert(i.clean && i.masterValid && !i.invalid && !i.uncertain && i.alerts==blockBit(BedSafety));}
        if(done) assert(i.headerValid);
    });
    Memory good=blank;Store g(good,Profile);assert(g.initialize(factory));
    assert(g.commit(blockBit(BedSafety),factory));assert(g.acknowledge(blockBit(BedSafety)));assert(cutoff(g)==120);
    const auto initial=g.inspect();assert(initial.clean && initial.masterValid && !initial.alerts && !initial.invalid);
    // No-op: exact image and no physical programming.
    good.resetCounts();assert(g.commit(blockBit(Geometry)|blockBit(BedSafety),factory));assert(!good.writes);
    const unsigned noOpReads=good.reads;
    assert(noOpReads<300); // no 2048-byte scan
    const uint32_t pairMask=blockBit(Geometry)|blockBit(Extruder0Motion)|blockBit(BedSafety);
    cuts(good,[&](Store& s){return s.commit(pairMask,changed);},[&](Memory& m,bool done){
        Store s(m,Profile);const auto i=s.inspect();const uint8_t c=cutoff(s);
        assert(i.headerValid);
        assert(c==0 || c==120 || c==90);
        if(i.eligible(Geometry) && i.eligible(Extruder0Motion) && i.eligible(BedSafety)) {
            const bool old=sameBlock(s,Geometry,factory) && sameBlock(s,Extruder0Motion,factory) && c==120;
            const bool fresh=sameBlock(s,Geometry,changed) && sameBlock(s,Extruder0Motion,changed) && c==90;
            assert(old || fresh);
        }
        if(done) assert(c==90 && !i.invalid && !i.uncertain && i.clean);
        assert(sameBlock(s,Lighting,factory));
    });
    Memory low=good;Store l(low,Profile);assert(l.commit(blockBit(BedSafety),changed));assert(cutoff(l)==90);
    // A single flipped bit anywhere in the defined KiB never raises the
    // committed lower cutoff. Spare bytes/padding are intentionally ignored.
    for(uint16_t p=Base;p<End;++p) for(uint8_t b=0;b<8;++b) {
        Memory m=low;m.bytes[p]^=1U<<b;Store s(m,Profile);
        assert(cutoff(s)<=90);++cases;
    }
    // Master alone cannot see a changed data byte; inspect must recompute
    // the actual block CRCs before use.
    Memory local=low;local.bytes[address(Lighting)+PayloadOffset]^=1;
    Store ls(local,Profile);auto i=ls.inspect();
    assert(i.masterValid && i.invalid==blockBit(Lighting) && i.eligible(BedSafety));
    cuts(local,[&](Store& s){return s.repair(blockBit(Lighting),factory);},[&](Memory& m,bool done){
        Store s(m,Profile);const auto st=s.inspect();assert(cutoff(s)<=90);
        if(done) assert(st.clean && st.masterValid && !st.invalid && (st.alerts&blockBit(Lighting)) && cutoff(s)==90);
    });
    Memory bad=low;bad.bytes[address(BedSafety)+PayloadOffset]^=1;Store bs(bad,Profile);
    assert(!bs.commit(blockBit(BedSafety),factory));assert(cutoff(bs)==0);
    cuts(bad,[&](Store& s){return s.repair(blockBit(BedSafety),factory);},[&](Memory& m,bool done){
        Store s(m,Profile);assert(cutoff(s)==0);
        if(done) {const auto st=s.inspect();assert(st.clean && st.masterValid && !st.invalid && (st.alerts&blockBit(BedSafety)));}
    });
    Memory master=low;master.bytes[Header::Master]^=1;Store ms(master,Profile);
    i=ms.inspect();assert(!i.masterValid && !i.invalid && !i.uncertain);
    assert(ms.repair(0,factory));assert(cutoff(ms)==90 && ms.inspect().alerts==blockBit(Admin));
    assert(ms.acknowledge(blockBit(Admin)));assert(!ms.inspect().alerts);
    Memory multi=low;multi.bytes[address(Geometry)+3]^=1;multi.bytes[address(Jerk)+3]^=1;
    Store mm(multi,Profile);assert(mm.inspect().invalid==(blockBit(Geometry)|blockBit(Jerk)));
    assert(!mm.repair(blockBit(Geometry),factory));
    cuts(low,[&](Store& s){return s.initialize(factory);},[&](Memory& m,bool done){
        Store s(m,Profile);assert(cutoff(s)<=90);
        if(done) assert(cutoff(s)==0 && s.inspect().headerValid);
    });
    // Wrong board/profile cannot consume even structurally valid settings.
    Store wrong(low,Profile+1);assert(!wrong.inspect().headerValid && cutoff(wrong)==0);
    // Alert acknowledgment may be interrupted; no implicit authorization.
    Memory pending=bad;Store ps(pending,Profile);assert(ps.repair(blockBit(BedSafety),factory));
    cuts(pending,[&](Store& s){return s.acknowledge(blockBit(BedSafety));},[&](Memory& m,bool){
        Store s(m,Profile);assert(cutoff(s)==0);
    });
    // A previous explicit change may have completed all payload/CRC/master
    // writes but not committed its state pair. Repair must NEVER hide its
    // uncertainty, even when a second power cut interrupts the repair.
    Values higher=changed; higher.bed=120; higher.salt=99;
    Memory trial=low;trial.resetCounts();Store ts(trial,Profile);
    assert(ts.commit(pairMask,higher));const unsigned commitWrites=trial.writes;
    Memory interrupted=low;interrupted.resetCounts();
    interrupted.cut=commitWrites; // before final state-complement byte
    Store is(interrupted,Profile);
    try {(void)is.commit(pairMask,higher);assert(false);}catch(const PowerCut&){}
    interrupted.resetCounts();
    auto interruptedInfo=is.inspect();
    assert(interruptedInfo.headerValid && !interruptedInfo.clean && interruptedInfo.uncertain);
    assert(cutoff(is)==0);
    const uint32_t recoveryMask=interruptedInfo.invalid|interruptedInfo.uncertain;
    cuts(interrupted,[&](Store& s){return s.repair(recoveryMask,factory);},[&](Memory& m,bool done){
        Store s(m,Profile);assert(cutoff(s)<=90);
        const auto st=s.inspect();
        if(!st.clean || !st.masterValid || !st.alertsValid || st.invalid || st.uncertain) {
            assert(s.repair(st.invalid|st.uncertain,factory));
        }
        assert(cutoff(s)<=90);
        uint8_t c=255;
        assert(s.readBlock(BedSafety,&c,1) && c==0);
        const auto after=s.inspect();
        assert(after.clean && after.masterValid && !after.invalid && !after.uncertain);
        assert(after.alerts&blockBit(BedSafety));
        if(done) assert(st.clean && st.masterValid);
    });
    // Interrupted transaction with fully valid individual payload CRCs is
    // still uncertain. Repair must cover the entire durable intent.
    Memory uncertain=low;Store us(uncertain,Profile);
    uncertain.resetCounts();uncertain.cut=12; // interruption during intent/data sequence
    try {(void)us.commit(pairMask,changed);}catch(const PowerCut&){}
    uncertain.resetCounts();auto u=us.inspect();
    if(u.uncertain) {
        assert(!us.repair(0,factory));
        assert(us.repair(u.invalid|u.uncertain,factory));
        assert(cutoff(us)==0 || cutoff(us)==90);
    }
    // Recovery policy: only independently confirmed invalid blocks trigger
    // full reset. Master metadata alone or an interrupted write is repair.
    assert(recoveryPlan(mm.inspect()).kind==RecoveryKind::Reset);
    assert(recoveryPlan(ls.inspect()).kind==RecoveryKind::Repair);
    assert(recoveryPlan(ms.inspect()).kind==RecoveryKind::None);
    Inspection unresolved=initial;
    unresolved.clean=false;unresolved.invalid=blockBit(Geometry)|blockBit(Jerk);
    unresolved.uncertain=unresolved.invalid;
    assert(recoveryPlan(unresolved).kind==RecoveryKind::Repair);
    unresolved.uncertain=0;
    assert(recoveryPlan(unresolved).kind==RecoveryKind::Reset);
    assert(recoveryPlan(initial,blockBit(Geometry)|blockBit(Jerk)).kind==RecoveryKind::Reset);
    assert(recoveryPlan(initial,blockBit(Geometry)).mask==blockBit(Geometry));
    cuts(multi,[&](Store& s){return recover(s,factory);},[&](Memory& m,bool done){
        Store s(m,Profile);assert(cutoff(s)<=90);
        if(done) {
            const auto st=s.inspect();
            assert(st.headerValid && st.clean && st.masterValid && !st.invalid);
            assert(st.alerts==AllAlerts && cutoff(s)==0);
            assert(sameBlock(s,Geometry,factory) && sameBlock(s,Jerk,factory));
        }
    });
    Memory reset=multi;Store rs(reset,Profile);assert(recover(rs,factory));
    assert(acknowledgeOrdinary(rs,AllAlerts));
    assert(rs.inspect().alerts==blockBit(BedSafety) && cutoff(rs)==0);
    assert(!commissionBed(rs,39) && !commissionBed(rs,151));
    cuts(reset,[&](Store& s){return commissionBed(s,85);},[&](Memory& m,bool done){
        Store s(m,Profile);assert(cutoff(s)==0 || cutoff(s)==85);
        if(done) assert(cutoff(s)==85 && !s.inspect().alerts);
    });
    printf("EEPROM zones: %lu cut/torn-byte/bit-flip cases, 125 fields, %u no-op reads; PASS\n",cases,noOpReads);
}

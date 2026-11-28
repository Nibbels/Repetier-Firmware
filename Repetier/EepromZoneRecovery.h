// Recovery and commissioning policy, separate from the byte transaction engine.
// A caller must first inhibit heaters and movement as appropriate; no live
// settings are applied here. Only independent invalid blocks count as a
// multiple-block failure. An interrupted write alone is not proof of damage.
#ifndef RF_EEPROM_ZONE_RECOVERY_H
#define RF_EEPROM_ZONE_RECOVERY_H
#include "EepromZoneStorage.h"

namespace EepromZones {
enum class RecoveryKind : uint8_t { None, Repair, Reset };
struct RecoveryPlan {
    RecoveryKind kind;
    uint32_t mask;
};
inline RecoveryPlan recoveryPlan(const Inspection& state, uint32_t semanticInvalid=0) {
    if(!state.headerValid) return {RecoveryKind::Reset, Active};
    const uint32_t invalid=(state.invalid | semanticInvalid) & Active;
    const uint32_t independent=invalid & ~state.uncertain;
    if(independent & (independent-1)) return {RecoveryKind::Reset, Active};
    const uint32_t repair=invalid | state.uncertain;
    if(repair || !state.masterValid || !state.clean || !state.alertsValid)
        return {RecoveryKind::Repair, repair};
    return {RecoveryKind::None, 0};
}
template<class Store,class Defaults>
bool recover(Store& store,Defaults& defaults,uint32_t semanticInvalid=0) {
    const auto plan=recoveryPlan(store.inspect(),semanticInvalid);
    switch(plan.kind) {
    case RecoveryKind::None: return true;
    case RecoveryKind::Repair: return store.repair(plan.mask,defaults);
    case RecoveryKind::Reset: return store.initialize(defaults,AllAlerts);
    }
    return false;
}
// Generic Alert+OK is never bed commissioning, even if the UI passes all
// bits. The caller must separately validate each other safety dependency.
template<class Store> bool acknowledgeOrdinary(Store& store,uint32_t mask) {
    if(!mask || (mask & ~AllAlerts)) return false;
    mask &= ~blockBit(BedSafety);
    return !mask || store.acknowledge(mask);
}
// Only the explicitly confirmed cutoff path may call this operation; it
// never invents a default or raises an existing cutoff during recovery.
// Revalidate the actual persisted payload after committing, before clearing
// its commissioning alert. A cut before acknowledgment leaves heat inhibited.
template<class Store> bool commissionBed(Store& store,uint8_t cutoff) {
    if(cutoff<40 || cutoff>150) return false;
    const auto before=store.inspect();
    if(!before.headerValid || !before.masterValid || !before.clean
       || !before.alertsValid || before.invalid || before.uncertain) return false;
    struct Provider {
        uint8_t cutoff;
        bool operator()(uint8_t id,uint8_t* data,uint8_t n) const {
            if(id!=BedSafety || !data || n!=1) return false;
            data[0]=cutoff;return true;
        }
    } proposed{cutoff};
    if(!store.commit(blockBit(BedSafety),proposed)) return false;
    uint8_t actual=0;
    if(!store.readBlock(BedSafety,&actual,1) || actual!=cutoff) return false;
    if(before.alerts & blockBit(BedSafety)) return store.acknowledge(blockBit(BedSafety));
    return true;
}
}
#endif

// Transactional, fixed-slot EEPROM storage. This implementation deliberately
// does not own printer state, UI, heater permission, or a full EEPROM mirror.
// Backend: uint8_t read(uint16_t), void update(uint16_t,uint8_t). update must
// finish or allow read() to wait until the programmed byte is stable.
// One non-reentrant foreground owner; provider data must stay immutable
// for the transaction. Real AVR service/yield integration is a separate gate.
#ifndef RF_EEPROM_ZONE_STORAGE_H
#define RF_EEPROM_ZONE_STORAGE_H
#include "EepromZoneLayout.h"
#include "EepromZoneIntegrity.h"

namespace EepromZones {
struct Inspection {
    uint32_t invalid;
    uint32_t uncertain;
    uint32_t alerts;
    bool headerValid;
    bool masterValid;
    bool alertsValid;
    bool clean;

    // AVR LTO emitted duplicate anonymous 16-byte RAM initializer images
    // for the fail-closed defaults in different partitions. Keep one
    // constructor body rather than cloning the constant into every caller.
#ifdef __AVR__
    __attribute__((noinline,noclone))
#endif
    Inspection() : invalid(Active), uncertain(Active), alerts(AllAlerts),
                   headerValid(false), masterValid(false), alertsValid(false), clean(false) {}
    // In addition to this storage gate, the caller MUST validate semantics,
    // dependent blocks, calibration and live motion/heater admission.
    bool eligible(uint8_t id) const {
        return id && id <= Last && headerValid && masterValid && clean
            && alertsValid && !((invalid | uncertain | alerts) & blockBit(id));
    }
};

template<class Backend> class Storage {
    Backend& io_;
    uint16_t profile_;

    uint8_t get(uint16_t pos) { return io_.read(pos); }
    bool put(uint16_t pos, uint8_t value) {
        if (pos < Base || pos >= End) return false;
        if (get(pos) == value) return true;
        io_.update(pos, value);
        return get(pos) == value;
    }
    uint16_t get16(uint16_t pos) {
        uint16_t value = get(pos);
        return value | (uint16_t(get(pos+1)) << 8);
    }
    uint32_t get32(uint16_t pos) {
        uint32_t value = 0;
        // Preserve ascending physical reads and use fixed byte shifts.
        value = get(pos);
        value |= uint32_t(get(pos+1)) << 8;
        value |= uint32_t(get(pos+2)) << 16;
        value |= uint32_t(get(pos+3)) << 24;
        return value;
    }
    bool put16(uint16_t pos, uint16_t value) {
        return put(pos,uint8_t(value)) && put(pos+1,uint8_t(value>>8));
    }
    bool put32(uint16_t pos, uint32_t value) {
        for (uint8_t i=0; i<4; ++i) {
            if (!put(pos+i,uint8_t(value))) return false;
            value>>=8;
        }
        return true;
    }
    uint8_t identity(uint8_t i) const {
        if (i<4) return uint8_t(Header::Signature >> (8*i));
        if (i<6) return uint8_t(Schema >> (8*(i-4)));
        return uint8_t(profile_ >> (8*(i-6)));
    }
    uint16_t headerCrc() const {
        uint16_t crc=0xFFFF;
        for (uint8_t i=0; i<8; ++i) crc=crc16Byte(crc,identity(i));
        return crc;
    }
    bool headerValid() {
        for (uint8_t i=0; i<8; ++i)
            if (get(Base+i) != identity(i)) return false;
        return get16(Header::Crc) == headerCrc();
    }
    uint16_t blockSeed(uint8_t id, uint8_t n) const {
        uint16_t crc=0xFFFF;
        for (uint8_t i=0; i<8; ++i) crc=crc16Byte(crc,identity(i));
        return crc16Byte(crc16Byte(crc,id),n);
    }
    uint16_t payloadCrc(uint8_t id, const uint8_t* data, uint8_t n) const {
        uint16_t crc=blockSeed(id,n);
        for (uint8_t i=0; i<n; ++i) crc=crc16Byte(crc,data[i]);
        return crc;
    }
    uint32_t masterCrc() {
        uint32_t crc=0xFFFFFFFFUL;
        for (uint8_t i=0; i<8; ++i) crc=crc32Byte(crc,identity(i));
        for (uint8_t id=1; id<=Last; ++id) {
            crc=crc32Byte(crc,id);
            crc=crc32Byte(crc,get(address(id)));
            crc=crc32Byte(crc,get(address(id)+1));
        }
        return crc ^ 0xFFFFFFFFUL;
    }
    bool pair(uint16_t pos, uint32_t allowed, uint32_t& value) {
        value=get32(pos);
        return !(value & ~allowed) && get32(pos+4)==~value;
    }
    bool putPair(uint16_t pos, uint32_t value) {
        return put32(pos,value) && put32(pos+4,~value);
    }
    uint8_t state() {
        const uint8_t value=get(Header::State);
        return get(Header::StateInverse)==uint8_t(~value) ? value : 0;
    }
    bool putState(uint8_t value) {
        return put(Header::State,value) && put(Header::StateInverse,uint8_t(~value));
    }
    bool begin(uint32_t mask) {
        // Preparing is durable before intent bytes change. No feature byte
        // changes until the complete intent and Writing state pair read back.
        return mask && !(mask & ~Active)
            && putState(Header::Preparing)
            && putPair(Header::Intent,mask)
            && putState(Header::Writing);
    }
    bool beginRepair(uint32_t mask,bool wasClean) {
        if(wasClean) return begin(mask);
        // Never replace an unresolved Writing state with Preparing: doing
        // so could hide a CRC-valid but incomplete multi-block transaction.
        // Old uncertain bits are already in the alert mask; a torn intent
        // pair itself means all active blocks are uncertain.
        return mask && !(mask & ~Active)
            && putPair(Header::Intent,mask)
            && putState(Header::Writing);
    }
    bool finish() {
        return put32(Header::Master,masterCrc())
            && putState(Header::Clean);
    }
    // Validate the old payload and compare proposed bytes in one pass.
    // Avoid reading all changed block bytes twice just to detect a no-op.
    bool compareBlock(uint8_t id,const uint8_t* data,uint8_t n,bool& different) {
        if(!validPayload(id,data,n)) return false;
        const uint16_t base=address(id);
        if(get(base+2)!=n) return false;
        uint16_t crc=blockSeed(id,n);
        different=false;
        for(uint8_t i=0;i<n;++i) {
            const uint8_t old=get(base+PayloadOffset+i);
            if(id==BedSafety && old!=0 && (old<40 || old>150)) return false;
            crc=crc16Byte(crc,old);
            different |= old!=data[i];
        }
        return get16(base)==crc;
    }
    bool validPayload(uint8_t id, const uint8_t* data, uint8_t n) const {
        if (!id || id>Last || !data || n!=length(id)) return false;
        // Generic store knows only the representation/range of bed cutoff.
        // Explicit authorization and other semantic checks live above it.
        return id!=BedSafety || data[0]==0 || (data[0]>=40 && data[0]<=150);
    }
    bool writeBlock(uint8_t id,const uint8_t* data,uint8_t n) {
        if (!validPayload(id,data,n)) return false;
        const uint16_t base=address(id);
        for(uint8_t i=0;i<n;++i)
            if(!put(base+PayloadOffset+i,data[i])) return false;
        if(!put(base+2,n) || !put16(base,payloadCrc(id,data,n))) return false;
        return readBlock(id,nullptr,n);
    }
public:
    Storage(Backend& io,uint16_t profile):io_(io),profile_(profile) {}

    // CRC-valid bytes only; does not by itself authorize use. Even on
    // false, dst may have been filled, so callers must check the return.
    bool readBlock(uint8_t id,uint8_t* dst,uint8_t n) {
        if(!id || id>Last || n!=length(id)) return false;
        const uint16_t base=address(id);
        if(get(base+2)!=n) return false; // never trust a persisted length
        uint16_t crc=blockSeed(id,n);
        for(uint8_t i=0;i<n;++i) {
            const uint8_t v=get(base+PayloadOffset+i);
            if(dst) dst[i]=v;
            if(id==BedSafety && v!=0 && (v<40 || v>150)) return false;
            crc=crc16Byte(crc,v);
        }
        return get16(base)==crc;
    }

    // Single-block access for early baud selection and targeted edits.
    // The global commit state and directory CRC must agree; verify actual
    // requested payload and return pending alerts without clearing them.
    bool readCommittedBlock(uint8_t id,uint8_t* data,uint8_t n,uint32_t& alerts) {
        return headerValid() && state()==Header::Clean
            && pair(Header::Alerts,AllAlerts,alerts)
            && get32(Header::Master)==masterCrc() && readBlock(id,data,n);
    }

    Inspection inspect() {
        Inspection result;
        if(!headerValid()) return result;
        result.headerValid=true;
        result.invalid=0;
        for(uint8_t id=1;id<=Last;++id)
            if(!readBlock(id,nullptr,length(id))) result.invalid |= blockBit(id);
        result.masterValid=get32(Header::Master)==masterCrc();
        result.alertsValid=pair(Header::Alerts,AllAlerts,result.alerts);
        if(!result.alertsValid) result.alerts=AllAlerts;
        const uint8_t status=state();
        result.clean=status==Header::Clean;
        if(result.clean || status==Header::Preparing) {
            // Preparing never writes feature bytes; during recovery any old
            // uncertain blocks were already put in the durable alert mask.
            result.uncertain=0;
        } else if(status==Header::Writing) {
            uint32_t mask;
            if(pair(Header::Intent,Active,mask) && mask) result.uncertain=mask;
        }
        return result;
    }

    // First install or explicitly authorized full reset. Never import legacy
    // bytes, never touch spare slots, never automatically commission a bed.
    // Provider signature: bool operator()(uint8_t id,uint8_t* data,uint8_t n).
    template<class Provider> bool initialize(Provider& source, uint32_t alerts=blockBit(BedSafety)) {
        if(alerts & ~AllAlerts) return false;
        alerts |= blockBit(BedSafety);
        // Invalidating the first signature byte must precede every change.
        if(!put(Header::Magic,0)) return false;
        for(uint8_t i=1;i<8;++i)
            if(!put(Base+i,identity(i))) return false;
        if(!put16(Header::Crc,headerCrc())) return false;
        uint8_t data[MaxPayload];
        for(uint8_t id=1;id<=Last;++id) {
            const uint8_t n=length(id);
            if(!source(id,data,n)) return false;
            if(id==BedSafety) data[0]=0;
            if(!writeBlock(id,data,n)) return false;
        }
        if(!putPair(Header::Intent,0) || !putPair(Header::Alerts,alerts)
           || !finish()) return false;
        // Publish the identity last, only after all data and metadata.
        return put(Header::Magic,identity(0));
    }

    // Normal foreground save. Validate old target blocks before blessing
    // their new bytes. Fast path never reads unrelated feature payloads.
    // No programming of data, intent, CRCs or master on a no-op.
    template<class Provider> bool commit(uint32_t mask,Provider& source) {
        uint32_t alerts;
        if(!mask || (mask & ~Active) || !headerValid()
           || state()!=Header::Clean
           || !pair(Header::Alerts,AllAlerts,alerts)
           || get32(Header::Master)!=masterCrc()) return false;
        uint8_t data[MaxPayload];
        uint32_t changed=0;
        for(uint8_t id=1;id<=Last;++id) {
            if(!(mask & blockBit(id))) continue;
            const uint8_t n=length(id);
            bool different=false;
            if(!source(id,data,n) || !compareBlock(id,data,n,different)) return false;
            if(different) changed |= blockBit(id);
        }
        if(!changed) return true;
        if(!begin(changed)) return false;
        for(uint8_t id=1;id<=Last;++id) {
            if(!(changed & blockBit(id))) continue;
            const uint8_t n=length(id);
            if(!source(id,data,n) || !writeBlock(id,data,n)) return false;
        }
        return finish();
    }

    // Recovery is an explicit policy operation, not a read-side effect.
    // Caller selects which defaults to restore, including all invalid and
    // uncertain blocks. An invalid master alone does NOT mean multiple
    // feature failures. Preserve a durable alert before replacing intent.
    template<class Provider> bool repair(uint32_t mask,Provider& defaults) {
        const Inspection before=inspect();
        const uint32_t required=before.invalid | before.uncertain;
        if(!before.headerValid || (mask & ~Active) || (required & ~mask)) return false;
        if(!mask && before.clean && before.masterValid && before.alertsValid) return true;
        const uint32_t alerts=before.alerts | mask
            | ((!before.masterValid || !before.clean || !before.alertsValid) ? blockBit(Admin) : 0);
        if(!putPair(Header::Alerts,alerts)) return false;
        if(mask) {
            if(!beginRepair(mask,before.clean)) return false;
            uint8_t data[MaxPayload];
            for(uint8_t id=1;id<=Last;++id) {
                if(!(mask & blockBit(id))) continue;
                const uint8_t n=length(id);
                if(!defaults(id,data,n)) return false;
                if(id==BedSafety) data[0]=0; // never restore higher limit
                if(!writeBlock(id,data,n)) return false;
            }
        }
        return finish();
    }

    // UI must distinguish an ordinary reset acknowledgment from explicit
    // recommissioning/validation of safety-critical settings.
    bool acknowledge(uint32_t mask) {
        if(!mask || (mask & ~AllAlerts)) return false;
        const Inspection current=inspect();
        if(!current.headerValid || !current.clean || !current.masterValid
           || !current.alertsValid || current.invalid || current.uncertain) return false;
        return putPair(Header::Alerts,current.alerts & ~mask);
    }
};
}
#endif

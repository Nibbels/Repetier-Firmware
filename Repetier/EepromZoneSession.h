// Checked, non-reentrant settings transactions over an arbitrary backend.
// A caller owns hardware admission and publishing; this layer never applies
// a corrupt/partial payload and never touches old EEPROM addresses.
#ifndef RF_EEPROM_ZONE_SESSION_H
#define RF_EEPROM_ZONE_SESSION_H
#include "EepromZoneByteIO.h"
#include "EepromZoneRecovery.h"
#include "EepromZoneValidation.h"
#include "EepromZonePatch.h"

namespace EepromZones {
enum class Result : uint8_t { Ok, Busy, Invalid, TransportFault, NeedsRecovery, PublishRejected };

// Provider fills complete blocks with immutable proposed values. Validation
// wraps both normal live saves and factory reset/recovery.
template<class Provider> struct CheckedProvider {
    Provider& source;
    bool operator()(uint8_t id,uint8_t* data,uint8_t n) {
        return source(id,data,n) && validBlock(id,data,n);
    }
};

template<class Device> class Session {
    bool& busy_;
    uint16_t profile_;
    Device& device_;
    using IO=ReadyByteIO<Device>;
    using Store=Storage<IO>;
public:
    Session(Device& device,bool& busy,uint16_t profile)
        :busy_(busy),profile_(profile),device_(device) {}

private:
    template<class Provider> Result saveChecked(IO& io,Store& store,uint32_t mask,Provider& provider) {
        CheckedProvider<Provider> checked{provider};
        if(!mask || (mask & ~Active) || (mask & blockBit(BedSafety))) return Result::Invalid;
        uint8_t data[MaxPayload];
        for(uint8_t id=1;id<=Last;++id)
            if((mask&blockBit(id)) && !checked(id,data,length(id)))
                return io.good()?Result::Invalid:Result::TransportFault;
        const bool ok=store.commit(mask,checked);
        if(!io.good()) return Result::TransportFault;
        return ok?Result::Ok:Result::NeedsRecovery;
    }
public:
    template<class Provider> Result save(uint32_t mask,Provider& provider) {
        SessionGuard guard(busy_);
        if(!guard.entered()) return Result::Busy;
        IO io(device_);Store store(io,profile_);
        return saveChecked(io,store,mask,provider);
    }
    Result patch(const Patch* edits,uint8_t count) {
        SessionGuard guard(busy_);
        if(!guard.entered()) return Result::Busy;
        const uint32_t mask=patchMask(edits,count);
        if(!mask) return Result::Invalid;
        IO io(device_);Store store(io,profile_);
        PatchProvider<Store> proposed{store,edits,count};
        return saveChecked(io,store,mask,proposed);
    }
    // Read one checked block without recovery, unrelated payload reads or
    // write side effects. In particular early UART setup can fall back to
    // the configured baud rather than trust a corrupt/foreign image.
    Result read(uint8_t id,uint8_t* data,uint8_t n,uint32_t& alerts) {
        SessionGuard guard(busy_);
        if(!guard.entered()) return Result::Busy;
        if(!id || id>Last || !data || n!=length(id)) return Result::Invalid;
        IO io(device_);Store store(io,profile_);
        const bool ok=store.readCommittedBlock(id,data,n,alerts);
        if(!io.good()) return Result::TransportFault;
        return ok && validBlock(id,data,n)?Result::Ok:Result::NeedsRecovery;
    }

    // Only after verified durable commit invoke the field-specific owner.
    // The guard spans publication too. The caller must preflight live
    // admission before entering and recheck terminal faults in publish.
    // A publish rejection is not a rollback: caller must keep outputs
    // inhibited if required and report the durable/live discrepancy.
    template<class Consumer> Result patchAndPublish(const Patch* edits,uint8_t count,Consumer& publish) {
        SessionGuard guard(busy_);
        if(!guard.entered()) return Result::Busy;
        const uint32_t mask=patchMask(edits,count);
        if(!mask) return Result::Invalid;
        IO io(device_);Store store(io,profile_);
        PatchProvider<Store> proposed{store,edits,count};
        const Result saved=saveChecked(io,store,mask,proposed);
        if(saved!=Result::Ok) return saved;
        uint8_t data[MaxPayload];
        for(uint8_t id=1;id<=Last;++id) {
            if(!(mask&blockBit(id))) continue;
            uint32_t alerts=AllAlerts;
            const uint8_t n=length(id);
            if(!store.readCommittedBlock(id,data,n,alerts) || !validBlock(id,data,n) || !io.good())
                return io.good()?Result::NeedsRecovery:Result::TransportFault;
            if(!publish(id,data,n,alerts,edits,count)) return Result::PublishRejected;
        }
        return Result::Ok;
    }

    template<class Defaults> Result restore(Defaults& defaults,Inspection& state,bool fullReset=false) {
        SessionGuard guard(busy_);
        if(!guard.entered()) return Result::Busy;
        IO io(device_);Store store(io,profile_);
        CheckedProvider<Defaults> checked{defaults};
        state=store.inspect();
        if(!io.good()) return Result::TransportFault;
        const uint32_t invalid=semanticInvalid(store,state);
        if(!io.good()) return Result::TransportFault;
        const bool ok=fullReset?store.initialize(checked,AllAlerts):recover(store,checked,invalid);
        if(!io.good()) return Result::TransportFault;
        if(!ok) return Result::NeedsRecovery;
        state=store.inspect();
        const uint32_t finalInvalid=semanticInvalid(store,state);
        if(!io.good()) return Result::TransportFault;
        return state.headerValid && state.masterValid && state.clean && state.alertsValid
            && !state.invalid && !state.uncertain && !finalInvalid?Result::Ok:Result::NeedsRecovery;
    }

    // Consumer may stage live values only after full image inspection and
    // semantic preflight. It must not call a persistence entry recursively.
    template<class Consumer> Result load(Consumer& consumer,Inspection& state) {
        SessionGuard guard(busy_);
        if(!guard.entered()) return Result::Busy;
        IO io(device_);Store store(io,profile_);
        state=store.inspect();
        const uint32_t invalid=semanticInvalid(store,state);
        if(!io.good()) return Result::TransportFault;
        if(!state.headerValid || !state.masterValid || !state.clean || !state.alertsValid ||
           state.invalid || state.uncertain || invalid) return Result::NeedsRecovery;
        uint8_t data[MaxPayload];
        for(uint8_t id=1;id<=Last;++id) {
            const uint8_t n=length(id);
            if(!store.readBlock(id,data,n) || !io.good()) return io.good()?Result::NeedsRecovery:Result::TransportFault;
            if(!validBlock(id,data,n) || !consumer(id,data,n,state.alerts)) return Result::Invalid;
        }
        return Result::Ok;
    }

    Result commission(uint8_t cutoff) {
        SessionGuard guard(busy_);
        if(!guard.entered()) return Result::Busy;
        IO io(device_);Store store(io,profile_);
        const auto state=store.inspect();
        const uint32_t invalid=semanticInvalid(store,state);
        if(!io.good())return Result::TransportFault;
        if(invalid)return Result::NeedsRecovery;
        const bool ok=commissionBed(store,cutoff);
        if(!io.good())return Result::TransportFault;
        return ok?Result::Ok:Result::Invalid;
    }

    Result acknowledge(uint32_t mask) {
        SessionGuard guard(busy_);
        if(!guard.entered()) return Result::Busy;
        IO io(device_);Store store(io,profile_);
        const auto state=store.inspect();
        const uint32_t invalid=semanticInvalid(store,state);
        if(!io.good())return Result::TransportFault;
        if(invalid)return Result::NeedsRecovery;
        const bool ok=acknowledgeOrdinary(store,mask);
        if(!io.good())return Result::TransportFault;
        return ok?Result::Ok:Result::Invalid;
    }
};
}
#endif

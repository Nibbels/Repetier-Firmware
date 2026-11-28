#pragma once
#include <stdint.h>

namespace EventLog {
// Diagnostic-only wire format; never used for heater or motion admission.
// Storage owns a disjoint EEPROM region and serializes access with settings.
// ready() must be false while a write is in flight OR another owner holds it.
// read(offset) is nonblocking after ready(); startWrite(offset,value) either
// launches ONE byte and returns true, or makes no change and returns false.
// All methods run in one non-reentrant foreground context, never in an ISR.
constexpr uint8_t EntryBytes=32, Committed=0xA7, Version=1;
enum class Channel : uint8_t { E0, E1, Bed, System };
constexpr uint8_t eventId(Channel ch,uint8_t reason) {
    return reason>0 && reason<64 ? uint8_t((uint8_t(ch)<<6)|reason) : 0;
}
inline bool validId(uint8_t id) { return (id&63)!=0; }
inline bool newer(uint32_t a,uint32_t b) {
    const uint32_t distance=a-b;
    return distance!=0 && distance<0x80000000UL;
}
inline uint8_t byteOf(uint32_t value,uint8_t offset) {
    return uint8_t(value>>(8*offset));
}
// Common payload layout, explicit little-endian, no C++ ABI dependence.
// Evidence fields are defined by event family. Units and validity are part
// of that family's decoder, not a basis for live safety decisions.
struct Payload {
    uint32_t uptime=0, window=0;
    int16_t values[4]{};
    uint16_t raw=0;
    uint8_t context[5]{};
    uint16_t valid=0;
    uint8_t id=0;
    uint8_t byte(uint8_t offset) const {
        if(offset<4)return byteOf(uptime,offset);
        if(offset<8)return byteOf(window,offset-4);
        if(offset<16)return byteOf(uint16_t(values[(offset-8)/2]),(offset-8)%2);
        if(offset<18)return byteOf(raw,offset-16);
        if(offset<23)return context[offset-18];
        if(offset<25)return byteOf(valid,offset-23);
        return offset==25?id:0;
    }
};
enum class Progress : uint8_t { Idle, Busy, Advanced, CommittedEntry, Failed };

// Source is immutable from begin() until commit/failure and supplies
// byte(0..25). The writer retains NO copy or permanent staging buffer.
// A caller can use existing first-fault evidence directly. Do not pass an
// ephemeral stack object and do not replace the source mid-append.
template<uint8_t Slots> class Ring {
    static_assert(Slots>=2 && Slots<=64,"bounded EEPROM diagnostic ring");
    uint32_t latest_=0;
    // Slots is an impossible physical slot and represents an empty log.
    uint8_t latestSlot_=Slots;
    // Scan and append are disjoint lifetimes; reuse the same bounded cursor.
    uint8_t offset_=0;
    enum class WriteState : uint8_t { Scanning, Idle, Ready, Waiting, Failed };
    WriteState writeState_=WriteState::Scanning;

    static constexpr uint16_t address(uint8_t slot,uint8_t offset) {
        return uint16_t(slot)*EntryBytes+offset;
    }
    template<class Store> static uint32_t sequence(Store& s,uint8_t slot) {
        uint32_t result=0;
        for(uint8_t i=0;i<4;++i)
            result|=uint32_t(s.read(address(slot,2+i)))<<(8*i);
        return result;
    }
    template<class Source> uint8_t expected(const Source& source) const {
        if(offset_==0)return 0;
        if(offset_==EntryBytes)return Committed;
        if(offset_==1)return Version;
        if(offset_<6)return byteOf(latest_+uint32_t(1),offset_-2);
        return source.byte(offset_-6);
    }
    uint8_t physicalOffset() const { return offset_==EntryBytes?0:offset_; }
    // The last committed slot cannot change while an append is active.
    // Derive the next slot rather than retaining a second cursor.
    uint8_t nextSlot() const { return hasEntries()?uint8_t((latestSlot_+1)%Slots):0; }
    Progress advance() {
        writeState_=WriteState::Ready;
        if(offset_++!=EntryBytes)return Progress::Advanced;
        ++latest_;
        latestSlot_=nextSlot();
        writeState_=WriteState::Idle;
        return Progress::CommittedEntry;
    }
public:
    static constexpr uint16_t Bytes=uint16_t(Slots)*EntryBytes;
    bool initialized() const {return writeState_!=WriteState::Scanning;}
    bool pending() const {return writeState_==WriteState::Ready || writeState_==WriteState::Waiting;}
    bool failed() const {return writeState_==WriteState::Failed;}
    bool hasEntries() const {return latestSlot_<Slots;}
    uint32_t latestSequence() const {return latest_;}
    uint8_t latestSlot() const {return hasEntries()?latestSlot_:0;}

    // One bounded slot-header inspection per foreground visit. Ignore
    // unknown versions/invalid IDs. Corruption may still look plausible:
    // this is an intentionally checksum-free, best-effort log.
    template<class Store> static bool header(Store& s,uint8_t slot,uint32_t& seq) {
        if(slot>=Slots || !s.ready())return false;
        if(s.read(address(slot,0))!=Committed || s.read(address(slot,1))!=Version ||
           !validId(s.read(address(slot,31))))return false;
        seq=sequence(s,slot);
        return true;
    }
    template<class Store> Progress scanStep(Store& s) {
        if(initialized())return Progress::Idle;
        if(!s.ready())return Progress::Busy;
        uint32_t seq=0;
        if(header(s,offset_,seq)&&(!hasEntries()||newer(seq,latest_))) {
            latest_=seq;latestSlot_=offset_;
        }
        if(++offset_==Slots)writeState_=WriteState::Idle;
        return Progress::Advanced;
    }
    template<class Source> bool begin(const Source& source) {
        if(!initialized()||writeState_!=WriteState::Idle||!validId(source.byte(25)))return false;
        offset_=0;writeState_=WriteState::Ready;
        return true;
    }
    template<class Store,class Source> Progress step(Store& s,const Source& source) {
        if(failed())return Progress::Failed;
        if(!pending())return Progress::Idle;
        if(!s.ready())return Progress::Busy;
        const uint16_t addr=address(nextSlot(),physicalOffset());
        const uint8_t value=expected(source);
        if(writeState_==WriteState::Waiting) {
            if(s.read(addr)!=value) {
                writeState_=WriteState::Failed;
                return Progress::Failed;
            }
            return advance();
        }
        // Skip equal bytes, including already invalid virgin slots.
        // Old committed record is invalidated before any body write.
        if(s.read(addr)==value)return advance();
        if(!s.startWrite(addr,value))return Progress::Busy;
        writeState_=WriteState::Waiting;
        return Progress::Advanced;
    }
    // A read cursor may call this when idle; never read partial records.
    // Read one complete entry in a foreground visit. The caller must not
    // release shared EEPROM ownership or yield during these bounded reads.
    template<class Store> bool readEntry(Store& s,uint8_t slot,uint8_t (&bytes)[EntryBytes]) const {
        if(pending()||!initialized()||!s.ready())return false;
        uint32_t seq;
        if(!header(s,slot,seq))return false;
        bytes[0]=Committed;bytes[1]=Version;
        for(uint8_t i=0;i<4;++i)bytes[2+i]=byteOf(seq,i);
        for(uint8_t i=6;i<EntryBytes;++i)bytes[i]=s.read(address(slot,i));
        return true;
    }

    // Caller controls bounded output and host backpressure. No full mirror.
    template<class Store> bool readByte(Store& s,uint8_t slot,uint8_t offset,uint8_t& value) const {
        if(pending()||!initialized()||slot>=Slots||offset>=EntryBytes||!s.ready())return false;
        uint32_t seq;
        if(!header(s,slot,seq))return false;
        value=s.read(address(slot,offset));
        return true;
    }
};
}

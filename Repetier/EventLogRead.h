#pragma once
#include "EventLog.h"

namespace EventLog {
inline uint32_t le(const uint8_t* p,uint8_t count) {
    uint32_t value=0;
    for(uint8_t i=0;i<count;++i)value|=uint32_t(p[i])<<(8*i);
    return value;
}
struct EntryView {
    const uint8_t (&bytes)[EntryBytes];
    uint32_t sequence() const{return le(bytes+2,4);}
    uint32_t uptime() const{return le(bytes+6,4);}
    uint32_t window() const{return le(bytes+10,4);}
    int16_t value(uint8_t k) const {
        if(k>=4)return 0;
        const uint16_t u=uint16_t(le(bytes+14+2*k,2));
        return int16_t(u<0x8000u?int32_t(u):int32_t(u)-65536L);
    }
    uint16_t raw() const{return uint16_t(le(bytes+22,2));}
    uint8_t context(uint8_t k) const{return k<5?bytes[24+k]:0;}
    uint16_t valid() const{return uint16_t(le(bytes+29,2));}
    Channel channel() const{return Channel(bytes[31]>>6);}
    uint8_t reason() const{return bytes[31]&63;}
};
enum class ReadProgress : uint8_t { Idle, Busy, Skipped, Sent, Done, Changed };
// Newest-first, one record per visit. No array of sequence numbers and no
// full-history mirror. A full ring traversal is bounded by Slots; invalid
// slots skipped. Restart explicitly if another append changed the history.
// Sink consumes/copies the entire record synchronously or returns false;
// never retain the temporary EntryView or pointer. Serial formatting may
// instead use a field cursor with same snapshot/ownership contract.
template<uint8_t Slots> class Reader {
    uint32_t newest_=0;
    uint8_t slot_=0,remaining_=0;
    bool running_=false,failedAtStart_=false;
public:
    bool begin(const Ring<Slots>& ring) {
        if(!ring.initialized()||ring.pending())return false;
        newest_=ring.latestSequence();slot_=ring.latestSlot();
        failedAtStart_=ring.failed();
        remaining_=ring.hasEntries()?Slots:0;
        running_=true;return true;
    }
    template<class Store,class Sink>
    ReadProgress step(const Ring<Slots>& ring,Store& store,Sink& sink) {
        if(!running_)return ReadProgress::Idle;
        if(ring.pending()||!store.ready())return ReadProgress::Busy;
        if(newest_!=ring.latestSequence()||failedAtStart_!=ring.failed()){
            running_=false;return ReadProgress::Changed;
        }
        if(!remaining_){running_=false;return ReadProgress::Done;}
        uint8_t bytes[EntryBytes];
        ReadProgress result=ReadProgress::Skipped;
        if(ring.readEntry(store,slot_,bytes)){
            EntryView view{bytes};
            // Exclude impossible sequence jumps from partially corrupted
            // data; diagnostic plausibility only, no corruption guarantee.
            if(uint32_t(newest_-view.sequence())<Slots){
                if(!sink.tryEntry(view))return ReadProgress::Busy;
                result=ReadProgress::Sent;
            }
        }
        slot_=slot_?uint8_t(slot_-1):uint8_t(Slots-1);
        --remaining_;
        return result;
    }
};
}

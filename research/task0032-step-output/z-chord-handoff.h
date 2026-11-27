#pragma once
#include "z-feedforward-dda.h"

namespace ZFeedforwardResearch {
// One active chord and one foreground-prepared successor. Single-core AVR
// SPSC protocol, NOT a C++ multithreaded queue. Producer is foreground only;
// consumer is Timer1 only. No interrupt-masked bulk copy, no allocation,
// no per-PrintLine growth. Tags refer to the immutable execution context.
// A move/context ID must not be reused while any old record can survive.
struct Tag {
    uint32_t move,first;
    uint16_t context;
    bool equals(const Tag& b) const {
        return move==b.move&&first==b.first&&context==b.context;
    }
};
class Handoff {
    struct Slot {Chord chord;Tag tag;};
    Slot slots_[2]{};
    volatile uint8_t active_=0,ready_=0;
    bool valid_=false; // Timer1 only
    static void barrier(){asm volatile("" ::: "memory");}
public:
    enum class Take:uint8_t {Accepted,Missing,Stale,Busy,Discontinuous};
    // Producer cannot be re-entered. Hook is only a deterministic
    // interrupt-injection seam for the host test; normal offer has none.
    template<class Hook>
    bool offer(const Tag& tag,int32_t first,int32_t last,uint32_t length,Hook hook) {
        if(ready_)return false;
        barrier();
        auto& next=slots_[active_^1];
        hook();
        if(!next.chord.prepare(first,last,length))return false;
        hook();
        next.tag=tag;
        hook();
        barrier();
        ready_=1; // publication is a single byte
        return true;
    }
    bool offer(const Tag& tag,int32_t first,int32_t last,uint32_t length){
        return offer(tag,first,last,length,[]{});
    }
    // Consumer at a boundary. A missing/stale record is NOT permission
    // to execute uncorrected steps, repeat old chord or stop abruptly.
    // Admission must guarantee a stopping horizon outside this class.
    Take take(const Tag& expected,int32_t expectedStart){
        if(valid_&&slots_[active_].chord.remaining())return Take::Busy;
        if(!ready_)return Take::Missing;
        barrier();
        const uint8_t next=active_^1;
        auto& slot=slots_[next];
        if(!slot.tag.equals(expected)){
            barrier();ready_=0;return Take::Stale;
        }
        if(slot.chord.value()!=expectedStart){
            barrier();ready_=0;return Take::Discontinuous;
        }
        active_=next;valid_=true;
        barrier();ready_=0; // only now can producer reuse the old slot
        return Take::Accepted;
    }
    bool advance(){return valid_&&slots_[active_].chord.advance();}
    uint32_t remaining() const{return valid_?slots_[active_].chord.remaining():0;}
    int32_t value() const{return slots_[active_].chord.value();}
    bool valid() const{return valid_;}
    // Abort is consumer-side; never change active_ or clear an unpublished
    // producer slot. Ready stale records will be rejected by new context.
    // Physical correction and motor ledger are deliberately untouched.
    void invalidate(){valid_=false;}
};
#ifdef __AVR__
static_assert(sizeof(Tag)==10,"AVR tag size");
static_assert(sizeof(Handoff)==73,"two active/prepared records + flags");
#endif
}

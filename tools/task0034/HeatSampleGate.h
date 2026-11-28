// Research-only exact ADC batch reservation. Not linked into firmware.
// One serialized foreground reservation per channel; publication in ADC ISR.
// All operations (including Ticket construction/destruction and consume+grant)
// must run with IRQs masked. Conversion itself runs outside the lock.
// Clock is a free-running 16-bit counter with a sticky wrap flag; exclusive
// wrap-flag ownership is required. Any wrap invalidates outstanding samples.
// The clock must continue; this is not an independent physical shutdown.
#pragma once
#include <stdint.h>

template<class Clock, uint8_t Channels, uint16_t MaxAge>
class HeatSampleGate {
    static_assert(Channels > 0 && Channels <= 8, "ADC channels");
    static_assert(MaxAge > 0 && MaxAge < 32768, "age range");
    enum : uint8_t { Fresh=1, Busy=2, Match=4 };
    Clock& clock_;
    volatile uint16_t raw_[Channels] = {};
    volatile uint16_t published_[Channels] = {};
    volatile uint8_t state_[Channels] = {};

    void synchronize() {
        if (clock_.wrapped()) {
            invalidateAll();
            clock_.clearWrap();
        }
    }
    bool live(uint8_t c) {
        synchronize();
        if (!(state_[c]&Fresh)) return false;
        const uint16_t now=clock_.now();
        // A wrap during observation must not allow an old timestamp to alias.
        if (clock_.wrapped()) {
            synchronize();
            return false;
        }
        if (uint16_t(now-published_[c])>MaxAge) {
            state_[c] &= Busy;
            return false;
        }
        return true;
    }
    bool reserve(uint8_t c,uint16_t& raw) {
        if (c>=Channels || (state_[c]&Busy) || !live(c)) return false;
        state_[c] |= Busy|Match;
        raw=raw_[c];
        return true;
    }
    void release(uint8_t c) { state_[c] &= Fresh; }
    bool consume(uint8_t c) {
        if (!live(c) || state_[c]!=(Fresh|Busy|Match)) return false;
        state_[c]=Busy; // consume exactly once; owner persists until release
        return true;
    }
public:
    explicit HeatSampleGate(Clock& clock):clock_(clock){}
    void invalidateAll() {
        // Never free a reservation still owned by foreground code.
        for(uint8_t c=0;c<Channels;++c) state_[c] &= Busy;
    }
    void publish(uint8_t c,uint16_t raw) {
        if(c>=Channels)return;
        synchronize();
        const uint16_t stamp=clock_.now();
        if(clock_.wrapped()) { synchronize(); return; }
        raw_[c]=raw;
        published_[c]=stamp;
        // Even an equal-valued batch invalidates a previous reservation.
        state_[c]=(state_[c]&Busy)|Fresh;
    }

    // No generation counter, no stale-copy ABA. Non-copyable, non-movable;
    // reservation remains occupied through any number of intervening batches.
    class Ticket {
        HeatSampleGate& gate_;
        const uint8_t channel_;
        uint16_t raw_=0;
        const bool owns_;
    public:
        Ticket(HeatSampleGate& gate,uint8_t c)
            :gate_(gate),channel_(c),owns_(gate_.reserve(c,raw_)){}
        ~Ticket(){if(owns_)gate_.release(channel_);}
        Ticket(const Ticket&)=delete;
        Ticket& operator=(const Ticket&)=delete;
        bool captured() const{return owns_;}
        uint16_t raw() const{return raw_;}
        bool consume(){return owns_&&gate_.consume(channel_);}
    };
};

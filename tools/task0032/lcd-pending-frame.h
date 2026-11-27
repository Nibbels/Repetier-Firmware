#pragma once
#include <stdint.h>

// Candidate normal-DDRAM transport. The caller owns a single desired
// framebuffer and must only call from one non-reentrant foreground context.
// A cleared dirty bit means that the corresponding desired byte was sent
// (not physically read back; the LCD R/W line is tied low).
template<uint8_t Columns>
class LcdPendingFrame {
    static_assert(Columns==16 || Columns==20, "RFx000 panel width");
    static constexpr uint8_t bytesPerRow=(Columns+7)/8;
    uint8_t dirty_[4][bytesPerRow]{};
    uint8_t dirtyRows_=0; // O(1) no-work check on every foreground visit
    uint32_t completedAt_=0;
    uint8_t nextAddress_=0xff;
    bool waiting_=false;

    void mark(uint8_t row,uint8_t col) {
        dirty_[row][col/8]|=uint8_t(1u<<(col%8));
    }
    void clear(uint8_t row,uint8_t col) {
        dirty_[row][col/8]&=uint8_t(~(1u<<(col%8)));
        for(uint8_t byte=0;byte<bytesPerRow;++byte)
            if(dirty_[row][byte]) return;
        dirtyRows_&=uint8_t(~(1u<<row));
    }
    bool find(uint8_t row,uint8_t& col) const {
        if(!(dirtyRows_&(1u<<row))) return false;
        for(uint8_t byte=0;byte<bytesPerRow;++byte) {
            uint8_t bits=dirty_[row][byte];
            if(!bits) continue;
            uint8_t bit=0;
            while(!(bits&1)) {bits>>=1;++bit;}
            col=byte*8+bit;
            return true;
        }
        return false;
    }
public:
    static constexpr uint32_t minimumClockDeltaUs=324;
    enum class Result:uint8_t {Idle,Waiting,Address,Data};

    bool pending() const { return dirtyRows_!=0; }

    // Only after an exclusive synchronous operation has actually emitted
    // every logical cell (or cleared DDRAM). Even then wait after its last
    // edge before the first normal byte; cursor may now point into CGRAM.
    void afterSynchronous(uint32_t now) {
        for(uint8_t row=0;row<4;++row)
            for(uint8_t byte=0;byte<bytesPerRow;++byte)
                dirty_[row][byte]=0;
        dirtyRows_=0;
        nextAddress_=0xff;
        completedAt_=now;
        waiting_=true;
    }
    void afterClear(char (&frame)[4][Columns+1],uint32_t now) {
        for(uint8_t row=0;row<4;++row) {
            for(uint8_t col=0;col<Columns;++col) frame[row][col]=' ';
            frame[row][Columns]=0;
        }
        afterSynchronous(now);
    }

    // Rendered text is a C string. NUL is padding, not a normal glyph;
    // boot slot zero remains the exclusive synchronous owner's business.
    void stage(uint8_t row,const char* text,char (&frame)[4][Columns+1]) {
        if(row>=4) return;
        bool changed=false;
        for(uint8_t col=0;col<Columns;++col) {
            const char c=*text?*text++:' ';
            if(frame[row][col]!=c) {
                frame[row][col]=c;
                mark(row,col);
                changed=true;
            }
        }
        frame[row][Columns]=0;
        if(changed) dirtyRows_|=uint8_t(1u<<row);
    }

    // Emergency prioritizes title/reason/action before live temperature.
    // Always read current logical content, never queue a stale data byte.
    // Sink writes exactly one raw LCD byte, then samples the clock.
    template<class Sink>
    Result service(const char (&frame)[4][Columns+1],bool emergency,Sink& sink) {
        uint8_t row=0,col=0;
        for(;row<4;++row) {
            const uint8_t candidate=emergency && row>=2?uint8_t(5-row):row;
            if(find(candidate,col)) {row=candidate;break;}
        }
        if(row==4) return Result::Idle;
        if(waiting_) {
            const uint32_t elapsed=uint32_t(sink.now()-completedAt_);
            // A small backwards clock jump is not permission to send.
            // Requires a qualified monotonic modulo-32-bit timebase and
            // pending service cadence well below half a clock wrap.
            if(elapsed<minimumClockDeltaUs || elapsed>=0x80000000UL)
                return Result::Waiting;
        }
        const uint8_t base=(row&1?0x40:0)+(row>=2?Columns:0);
        const uint8_t address=base+col;
        Result result;
        if(nextAddress_!=address) {
            sink.write(uint8_t(0x80|address),false);
            nextAddress_=address;
            result=Result::Address;
        } else {
            sink.write(uint8_t(frame[row][col]),true);
            clear(row,col);
            nextAddress_=col+1==Columns?0xff:uint8_t(address+1);
            result=Result::Data;
        }
        completedAt_=sink.now();
        waiting_=true;
        return result;
    }
};

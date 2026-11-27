#include "lcd-pending-frame.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <initializer_list>

struct Sink {
    char memory[128];
    uint32_t t=0,last=0;
    uint8_t address=0xff;
    unsigned writes=0,commands=0,data=0;
    bool started=false;
    Sink() { memset(memory,' ',sizeof(memory)); }
    uint32_t now() const { return t&~uint32_t(3); }
    void write(uint8_t v,bool rs) {
        if(started) assert(uint32_t(t-last)>=320);
        if(rs) {assert(address<128 && v);memory[address++]=char(v);++data;}
        else {assert(v&128);address=v&127;++commands;}
        ++writes;t+=7;last=t;started=true;
    }
    void ready() { t=(last&~uint32_t(3))+324; }
};

template<uint8_t C>
using Frame=char[4][C];

template<uint8_t C>
void verify(const Frame<C>& frame,const Sink& sink) {
    for(uint8_t row=0;row<4;++row) {
        uint8_t base=(row&1?64:0)+(row>=2?C:0);
        assert(memcmp(frame[row],sink.memory+base,C)==0);
    }
}

template<uint8_t C>
void drain(LcdPendingFrame<C>& f,Frame<C>& frame,Sink& sink,bool emergency=false) {
    using R=typename LcdPendingFrame<C>::Result;
    for(unsigned i=0;f.pending();++i) {
        assert(i<8*C+1);
        unsigned writes=sink.writes;
        auto r=f.service(frame,emergency,sink);
        if(r==R::Waiting) {
            assert(sink.writes==writes);
            sink.ready();
            r=f.service(frame,emergency,sink);
        }
        assert(r==R::Address || r==R::Data);
        assert(sink.writes==writes+1);
    }
    const unsigned before=sink.writes;
    auto r=f.service(frame,emergency,sink);
    if(r==R::Waiting){sink.ready();r=f.service(frame,emergency,sink);}
    assert(r==R::Idle && !f.needsService() && sink.writes==before);
    verify<C>(frame,sink);
}

template<uint8_t C>
void exhaustive() {
    unsigned long long bytes=0;
    for(uint8_t target=0;target<4;++target) for(uint32_t mask=0;mask<(uint32_t(1)<<C);++mask) {
        LcdPendingFrame<C> f;
        Frame<C> frame{};
        Sink sink;
        sink.t=0xffffff00u;
        sink.last=sink.t;
        f.afterClear(frame,sink.now());
        char row[C+1]{};
        unsigned data=0,runs=0;
        for(uint8_t col=0;col<C;++col) {
            const bool dirty=(mask>>col)&1;
            row[col]=dirty?'B':' ';
            if(dirty) {++data;if(!col || !((mask>>(col-1))&1))++runs;}
        }
        f.stage(target,row,frame);
        f.stage(target,row,frame);
        drain<C>(f,frame,sink);
        assert(sink.data==data && sink.commands==runs);
        bytes+=sink.writes;
    }
    std::printf("width %u: %lu dirty masks, %llu emitted bytes\n",
                unsigned(C),static_cast<unsigned long>(uint32_t(4)<<C),bytes);
}

template<uint8_t C>
void transitions() {
    using R=typename LcdPendingFrame<C>::Result;
    LcdPendingFrame<C> f;
    Frame<C> frame{};
    Sink sink;
    sink.t=sink.last=0xfffffff0u;
    f.afterClear(frame,sink.now());
    f.stage(0,"OLD",frame);
    assert(f.pending());
    assert(f.service(frame,false,sink)==R::Waiting);
    sink.t-=20; // backwards clock observation cannot authorize a write
    assert(f.service(frame,false,sink)==R::Waiting);
    sink.ready();
    assert(f.service(frame,false,sink)==R::Address);
    f.stage(0,"NEW",frame);
    sink.t=(sink.last&~uint32_t(3))+320;
    assert(f.service(frame,false,sink)==R::Waiting);
    sink.ready();
    assert(f.service(frame,false,sink)==R::Data);
    assert(sink.memory[0]=='N');
    drain<C>(f,frame,sink);
    f.stage(0,"TEMP",frame);
    f.stage(0,"NEW",frame); // reverted cells stay conservatively dirty
    drain<C>(f,frame,sink);

    // Normal row 2 had already started; the emergency row 0 takes over.
    f.stage(2,"OLD STATUS",frame);
    sink.ready();
    assert(f.service(frame,false,sink)==R::Address);
    f.stage(0,"FIRE WARNING",frame);
    f.stage(1,"BED:OVER TEMP",frame);
    f.stage(2,"BED:200/90 C",frame);
    f.stage(3,"POWER OFF! CHECK",frame);
    sink.ready();
    assert(f.service(frame,true,sink)==R::Address);
    assert(sink.address==0);
    sink.ready();
    assert(f.service(frame,true,sink)==R::Data);
    assert(sink.memory[0]=='F');
    // While emergency row 2 changes repeatedly, row 3 is selected first.
    while(memcmp(sink.memory+64+C,frame[3],C)!=0) {
        const unsigned before=sink.writes;
        sink.ready();
        auto r=f.service(frame,true,sink);
        assert(r==R::Data || r==R::Address);
        assert(sink.writes==before+1);
        assert(sink.memory[C]==' '); // no old or live status data sent yet
    }
    drain<C>(f,frame,sink,true);
    f.stage(0,"",frame);
    f.stage(1,"",frame);
    f.stage(2,"",frame);
    f.stage(3,"",frame);
    drain<C>(f,frame,sink,true);
    f.stage(0,"stale",frame);
    // Exclusive clear/glyph operation has really finished; suppress old work.
    memset(sink.memory,' ',sizeof(sink.memory));
    sink.t+=2000;sink.last=sink.t;
    f.afterClear(frame,sink.now());
    assert(!f.pending());
    f.stage(3,"boot finished",frame);
    assert(f.service(frame,false,sink)==R::Waiting);
    drain<C>(f,frame,sink);
    assert(!f.needsService());
    // >half a 32-bit micros wrap with completely unchanged text.
    sink.t+=0x90000000UL;
    sink.last=sink.t-1000u;
    f.stage(0,"long idle",frame);
    assert(f.needsService());
    drain<C>(f,frame,sink);
    std::printf("width %u: overwrite, revert, emergency ordering, wrap, clock regression, clear, long idle PASS\n",unsigned(C));
}

// Every pair of 4-bit high-column masks in both shared bitmap bytes.
// Exercise the neighbour while a row is partially drained and restaged.
void sharedNibbles() {
    for(uint8_t pair=0;pair<2;++pair) {
        for(unsigned masks=0;masks<256;++masks) {
            for(bool emergency:{false,true}) {
                LcdPendingFrame<20> f;
                Frame<20> frame{};
                Sink sink;
                f.afterClear(frame,sink.now());
                char a[21],b[21];
                memset(a,' ',20);memset(b,' ',20);a[20]=b[20]=0;
                unsigned count=0,runs=0;
                for(uint8_t i=0;i<8;++i) {
                    if(masks&(1u<<i)) {
                        (i<4?a:b)[16+i%4]='X';
                        ++count;
                        if(i%4==0 || !(masks&(1u<<(i-1)))) ++runs;
                    }
                }
                f.stage(pair*2,a,frame);
                f.stage(pair*2+1,b,frame);
                drain<20>(f,frame,sink,emergency);
                assert(sink.data==count&&sink.commands==runs);
            }
        }
    }
    puts("1024 adjacent-row shared-nibble patterns PASS");
}

// Deterministic interleaved frame changes and variable service cadence.
// This proves state consistency on the sampled transitions, not a CPU
// scheduling or hardware timing guarantee.
static uint32_t randomState=0x9e3779b9u;
static uint32_t randomWord() {
    randomState^=randomState<<13;
    randomState^=randomState>>17;
    randomState^=randomState<<5;
    return randomState;
}

template<uint8_t C>
void interleaved() {
    using R=typename LcdPendingFrame<C>::Result;
    LcdPendingFrame<C> f;
    Frame<C> frame{};
    Sink sink;
    sink.t=sink.last=0xffff0000u;
    f.afterClear(frame,sink.now());
    unsigned stages=0, visits=0, sends=0, waits=0, drains=0;
    const uint8_t bases[]={0,64,C,uint8_t(64+C)};
    for(unsigned n=0;n<200000;++n) {
        const auto choice=randomWord();
        if((choice&7)<3) {
            uint8_t row=uint8_t((choice>>3)&3);
            char text[C+1];
            const unsigned length=randomWord()%(C+1);
            for(unsigned col=0;col<length;++col)
                text[col]=char(1+randomWord()%255);
            text[length]=0;
            f.stage(row,text,frame);
            ++stages;
        } else {
            // Variable intervals, all clock quantization phases, wrap
            // and repeated too-early visits. No asynchronous GPIO owner.
            sink.t+=randomWord()%1001;
            const bool emergency=choice&8;
            const unsigned count=sink.writes;
            const auto priorAddress=sink.address;
            const auto r=f.service(frame,emergency,sink);
            ++visits;
            if(r==R::Data || r==R::Address) {
                assert(sink.writes==count+1);
                ++sends;
                const uint8_t target=r==R::Address?sink.address:priorAddress;
                int row=-1;
                for(uint8_t k=0;k<4;++k)
                    if(target>=bases[k]&&target<bases[k]+C) row=k;
                assert(row>=0);
                if(r==R::Data)
                    assert(sink.memory[target]==frame[row][target-bases[row]]);
                const uint8_t priority[]={0,1,uint8_t(emergency?3:2),uint8_t(emergency?2:3)};
                for(uint8_t k:priority) {
                    if(k==row) break;
                    // A dirty logical difference in a higher-priority
                    // row cannot be bypassed by a lower-priority byte.
                    assert(memcmp(frame[k],sink.memory+bases[k],C)==0);
                }
            } else {
                assert(sink.writes==count);
                if(r==R::Waiting) ++waits;
            }
        }
        if(!f.pending()) verify<C>(frame,sink);
        if(n%509==0) {drain<C>(f,frame,sink,choice&8);++drains;}
    }
    drain<C>(f,frame,sink);
    assert(stages>60000&&visits>100000&&sends>10000&&waits>10000);
    std::printf("width %u: 200000 interleaved transitions, %u stage, %u visits, %u sends, %u waits, %u drains PASS\n",
                unsigned(C),stages,visits,sends,waits,drains);
}

int main() {
    exhaustive<16>(); exhaustive<20>();
    transitions<16>(); transitions<20>();
    sharedNibbles();
    interleaved<16>(); interleaved<20>();
}

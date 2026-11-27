#include "lcd-pending-frame.h"
#include <cassert>
#include <cstdio>
#include <cstring>

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
using Frame=char[4][C+1];

template<uint8_t C>
void verify(const Frame<C>& frame,const Sink& sink) {
    for(uint8_t row=0;row<4;++row) {
        uint8_t base=(row&1?64:0)+(row>=2?C:0);
        assert(memcmp(frame[row],sink.memory+base,C)==0);
        assert(frame[row][C]==0);
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
    for(uint32_t mask=0;mask<(uint32_t(1)<<C);++mask) {
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
        f.stage(0,row,frame);
        f.stage(0,row,frame);
        drain<C>(f,frame,sink);
        assert(sink.data==data && sink.commands==runs);
        bytes+=sink.writes;
    }
    std::printf("width %u: %lu dirty masks, %llu emitted bytes\n",
                unsigned(C),static_cast<unsigned long>(uint32_t(1)<<C),bytes);
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

int main() {
    exhaustive<16>(); exhaustive<20>();
    transitions<16>(); transitions<20>();
}

// Task-private replay: exact candidate, simulated DDRAM and clock.
#include "lcd-byte-service.h"
#include <cassert>
#include <cstdio>
#include <cstring>

using Task0032Lcd::ByteService;
using Task0032Lcd::Result;
struct Sink {
    char memory[128];
    uint32_t t, lastEdge=0;
    uint8_t address=0xff;
    unsigned commands=0, data=0;
    bool wrote=false;
    explicit Sink(uint32_t start=0):t(start) { memset(memory,'A',sizeof(memory)); }
    uint32_t now() const { return t & ~uint32_t(3); }
    void write(uint8_t value,bool rs) {
        if(wrote) assert(uint32_t(t-lastEdge)>=320);
        if(rs) {
            assert(address<128 && value!=0);
            memory[address++]=char(value);
            ++data;
        } else {
            assert(value&0x80);
            address=value&0x7f;
            ++commands;
        }
        // Model the completed edge, not the beginning of the transfer.
        t+=7;
        lastEdge=t;
        wrote=true;
    }
    unsigned count() const { return commands+data; }
    void ready() {
        if(wrote) t=(lastEdge&~uint32_t(3))+ByteService::minimumClockDeltaUs;
    }
};

template<uint8_t C>
Result visit(ByteService& s,Sink& sink,uint8_t base,const char (&desired)[C],char (&shadow)[C]) {
    const unsigned before=sink.count();
    const uint32_t start=sink.t;
    const Result result=s.service(base,desired,shadow,sink.now(),sink);
    const bool emitted=result==Result::Address || result==Result::Data;
    assert(sink.count()==before+unsigned(emitted));
    assert(uint32_t(sink.t-start)==(emitted?7u:0u));
    for(unsigned i=0;i<C;++i)
        if(shadow[i]!=0) assert(shadow[i]==sink.memory[base+i]);
    return result;
}

template<uint8_t C>
void drain(ByteService& s,Sink& sink,uint8_t base,const char (&desired)[C],char (&shadow)[C]) {
    for(unsigned i=0;i<=2*C;++i) {
        if(sink.wrote) {
            sink.t=(sink.lastEdge&~uint32_t(3))+320u;
            const auto early=visit(s,sink,base,desired,shadow);
            if(early==Result::Idle) break;
            assert(early==Result::Waiting);
            sink.ready();
        }
        const auto result=visit(s,sink,base,desired,shadow);
        if(result==Result::Idle) break;
        assert(result==Result::Data || result==Result::Address);
    }
    assert(memcmp(desired,shadow,C)==0);
    assert(memcmp(desired,sink.memory+base,C)==0);
}

template<uint8_t C>
void allMasks() {
    unsigned long long bytes=0;
    for(uint32_t mask=0;mask<(uint32_t(1)<<C);++mask) {
        ByteService s;
        Sink sink(mask&1?0xffffff00u:0u);
        char desired[C],shadow[C];
        unsigned count=0,runs=0;
        for(unsigned i=0;i<C;++i) {
            const bool dirty=(mask>>i)&1;
            desired[i]=dirty?'B':'A'; shadow[i]='A';
            if(dirty) {
                ++count;
                if(i==0 || !(mask&(uint32_t(1)<<(i-1)))) ++runs;
            }
        }
        drain(s,sink,0,desired,shadow);
        assert(sink.data==count && sink.commands==runs);
        assert(visit(s,sink,0,desired,shadow)==Result::Idle);
        bytes+=sink.count();
    }
    std::printf("width %u: %lu exhaustive dirty masks, %llu exact bytes\n",
                unsigned(C),static_cast<unsigned long>(uint32_t(1)<<C),bytes);
}

template<uint8_t C>
void transitions() {
    ByteService s;
    Sink sink(0xfffffff0u);
    char shadow[4][C],desired[4][C];
    const uint8_t bases[]={0,0x40,C,uint8_t(0x40+C)};
    for(unsigned row=0;row<4;++row) {
        memset(shadow[row],'A',C);
        memset(desired[row],'A',C);
    }
    // Address issued, desired changes before the data byte. Do not send stale B.
    desired[0][0]='B';
    assert(visit(s,sink,bases[0],desired[0],shadow[0])==Result::Address);
    desired[0][0]='A'; desired[0][C-1]='C';
    sink.ready();
    assert(visit(s,sink,bases[0],desired[0],shadow[0])==Result::Address);
    sink.ready();
    assert(visit(s,sink,bases[0],desired[0],shadow[0])==Result::Data);
    assert(sink.memory[0]=='A' && sink.memory[C-1]=='C');
    // New row supersedes an unfinished ordinary row (e.g. emergency priority).
    desired[1][0]='D';
    sink.ready();
    assert(visit(s,sink,bases[1],desired[1],shadow[1])==Result::Address);
    desired[3][2]='E';
    sink.ready();
    assert(visit(s,sink,bases[3],desired[3],shadow[3])==Result::Address);
    sink.ready();
    assert(visit(s,sink,bases[3],desired[3],shadow[3])==Result::Data);
    assert(sink.memory[bases[1]]=='A' && sink.memory[bases[3]+2]=='E');
    drain(s,sink,bases[1],desired[1],shadow[1]);

    // Hardware clear / CGRAM / reinitialization: no shadow reuse, wait and
    // exclusive operation are owned by caller, never escape ordinary delay.
    sink.ready();
    memset(sink.memory,' ',sizeof(sink.memory));
    sink.address=0xff;
    s.afterExclusiveOperation();
    for(unsigned row=0;row<4;++row) {
        ByteService::invalidate(shadow[row]);
        memset(desired[row],' ',C);
        desired[row][row]=char(row+1);
    }
    const unsigned before=sink.count();
    for(unsigned row=0;row<4;++row) drain(s,sink,bases[row],desired[row],shadow[row]);
    assert(sink.count()-before==4*(C+1));
    for(unsigned row=0;row<4;++row) assert(visit(s,sink,bases[row],desired[row],shadow[row])==Result::Idle);
    std::printf("width %u: rollover, supersession, row mapping, full invalidation passed\n",unsigned(C));
}

int main() {
    allMasks<16>(); allMasks<20>();
    transitions<16>(); transitions<20>();
}

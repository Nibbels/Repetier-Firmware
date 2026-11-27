#!/usr/bin/env python3
"""Isolated shared-state reset/normal-DDRAM composition (not shipping code)."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

HERE=Path(__file__).resolve().parent

METHODS=r'''
    // Reuse the otherwise suspended DDRAM cursor and last-edge clock.
    // 0xfe is a request sentinel, outside both DDRAM and reset phases.
    bool resetting() const { return (state_&ResetBit)!=0; }
    bool requestReset() {
        if(resetting()) return false;
        state_|=ResetBit;
        nextAddress_=0xfe;
        return true;
    }

    template<class Sink>
    bool serviceReset(Sink& sink) {
        if(!resetting()) return false;
        if(nextAddress_==0xfe) {
            // Complete the previous normal byte before changing pin modes.
            if(state_&WaitingBit) {
                const uint32_t elapsed=uint32_t(sink.now()-completedAt_);
                if(elapsed<minimumClockDeltaUs || elapsed>=0x80000000UL)
                    return false;
            }
            state_&=uint8_t(~WaitingBit);
            LcdResetSequence::start(nextAddress_,completedAt_,sink);
            return true;
        }
        if(!LcdResetSequence::advance(nextAddress_,completedAt_,sink)) return false;
        if(nextAddress_==LcdResetSequence::inactive) {
            // Physical DDRAM was cleared, but the desired frame may have
            // changed during the reset. Force *all* cells to be repainted.
            for(uint8_t i=0;i<sizeof(dirty_);++i) dirty_[i]=0xff;
            state_=RowMask;
            nextAddress_=0xff;
        }
        return true;
    }
'''

FIXTURE=r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include "lcd-pending-frame.h"
#include "lcd-shared-reset-candidate.h"

struct Sink {
    char memory[128];
    uint32_t clock=0, last=0;
    bool initial=false, hasEdge=false;
    uint8_t address=0xff;
    unsigned writes=0, inputsCount=0, outputsCount=0, nibbles=0, initBytes=0, normalBytes=0;
    Sink(){memset(memory,' ',sizeof memory);}
    uint32_t now()const{return clock&~uint32_t(3);}
    void edge(){
        if(hasEdge) assert(uint32_t(clock-last)>=320 || initial);
        clock+=7;last=clock;hasEdge=true;++writes;
    }
    void inputs(){
        if(hasEdge) assert(uint32_t(clock-last)>=320);
        initial=true;++inputsCount;edge();
    }
    void outputs(){assert(initial);++outputsCount;edge();}
    void nibble(uint8_t v){
        assert(initial && v==(nibbles%4==3?2:3));++nibbles;edge();
    }
    uint8_t glyph(uint8_t slot,uint8_t row){assert(initial&&slot>=1&&slot<=7&&row<8);return (slot+row)&31;}
    void write(uint8_t v,bool rs){
        edge();
        if(initial){
            ++initBytes;
            if(!rs&&v==1)memset(memory,' ',sizeof memory);
            return;
        }
        ++normalBytes;
        if(rs){assert(address<128&&v);memory[address++]=char(v);}
        else {assert(v&128);address=v&127;}
    }
};

template<uint8_t C>
void exercise(uint32_t start) {
    using Frame=LcdPendingResetFrame<C>;
    using R=typename Frame::Result;
    static_assert(sizeof(Frame)==sizeof(LcdPendingFrame<C>),"reset added state");
    Frame f;
    char desired[4][C]{};
    Sink sink;
    sink.clock=start;
    f.afterClear(desired,sink.now());
    for(uint8_t row=0;row<4;++row)f.stage(row,"NORMAL OLD",desired);
    sink.clock+=324;assert(f.service(desired,false,sink)==R::Address);
    sink.clock+=324;assert(f.service(desired,false,sink)==R::Data);
    assert(sink.memory[0]=='N');
    assert(f.requestReset()&&f.resetting()&&f.pending()&&f.needsService());
    assert(!f.requestReset());
    // Must not touch GPIO modes while the last DDRAM byte is executing.
    const auto old=sink.writes;
    assert(!f.serviceReset(sink)&&sink.writes==old);
    assert(f.service(desired,false,sink)==R::Waiting&&sink.writes==old);
    sink.clock+=324;
    assert(f.serviceReset(sink)&&sink.inputsCount==1);
    // New normal and then emergency frames arrive during 500 ms settle.
    f.stage(0,"NORMAL NEW",desired);
    f.stage(1,"NEW MENU",desired);
    f.stage(0,"FIRE WARNING",desired);
    f.stage(1,"BED:OVER TEMP",desired);
    f.stage(2,"BED:200/90 C",desired);
    f.stage(3,"POWER OFF! CHECK",desired);
    unsigned visits=0;
    while(f.resetting()){
        assert(++visits<3000);
        const auto n=sink.writes;
        sink.clock+=1000;
        assert(f.service(desired,true,sink)==R::Waiting&&sink.writes==n);
        if(!f.serviceReset(sink))assert(sink.writes==n);
        assert(sink.writes<=n+1);
        assert(f.pending());
        if(f.resetting())assert(!f.requestReset());
    }
    assert(sink.outputsCount==1&&sink.nibbles==4&&sink.initBytes==67);
    assert(f.pending()&&f.needsService());
    sink.initial=false;
    const auto before=sink.normalBytes;
    // Cursor was in CGRAM: first operation MUST be a new DDRAM address.
    assert(f.service(desired,true,sink)==R::Address&&sink.address==0);
    // At most one byte per visit, prioritizing latest warning and action.
    while(f.pending()){
        assert(++visits<4000);
        sink.clock+=324;
        const auto n=sink.writes;
        auto r=f.service(desired,true,sink);
        assert(r==R::Data||r==R::Address);
        assert(sink.writes==n+1);
        if(sink.memory[C]!=' ')assert(memcmp(sink.memory+64+C,desired[3],C)==0);
    }
    assert(sink.normalBytes-before==4*(C+1));
    for(uint8_t r=0;r<4;++r)
        assert(memcmp(sink.memory+(r&1?64:0)+(r>=2?C:0),desired[r],C)==0);
    sink.clock+=324;
    assert(f.service(desired,true,sink)==R::Idle&&!f.needsService());
    // A second reset after a long completely idle period must not consult
    // the already retired old deadline or silently preserve stale DDRAM.
    sink.clock+=0x90000000UL;
    assert(f.requestReset()&&f.serviceReset(sink)&&sink.inputsCount==2);
}

int main(){
    for(uint32_t start:{0u,1u,2u,3u,0xfffffff0u,0xfffffff1u,0xfffffff2u,0xfffffff3u}){
        exercise<16>(start);exercise<20>(start);
    }
    puts("16 shared-state reset compositions: no new persistent member; latest-frame repaint and emergency priority PASS");
}
'''

def once(s,old,new):
    assert s.count(old)==1,(old,s.count(old))
    return s.replace(old,new)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out",type=Path,required=True)
    a=p.parse_args()
    source=(HERE/"lcd-pending-frame.h").read_text()
    candidate=source.replace("class LcdPendingFrame {","class LcdPendingResetFrame {")
    assert candidate!=source
    candidate=once(candidate,"#include <stdint.h>","#include <stdint.h>\n#include \"lcd-runtime-reset.h\"")
    candidate=once(candidate,"RowMask=0x0f, WaitingBit=0x10","RowMask=0x0f, WaitingBit=0x10, ResetBit=0x20")
    candidate=once(candidate,"bool pending() const { return (state_&RowMask)!=0; }",
                   "bool pending() const { return (state_&(RowMask|ResetBit))!=0; }")
    candidate=once(candidate,"    bool needsService() const { return state_!=0; }",
                   "    bool needsService() const { return state_!=0; }\n"+METHODS)
    candidate=once(candidate,"Result service(const char (&frame)[4][Columns],bool emergency,Sink& sink) {",
                   "Result service(const char (&frame)[4][Columns],bool emergency,Sink& sink) {\n        if(resetting()) return Result::Waiting;")
    a.out.mkdir(parents=True,exist_ok=True)
    (a.out/"lcd-shared-reset-candidate.h").write_text(candidate)
    (a.out/"lcd-shared-reset-fixture.cpp").write_text(FIXTURE)
    with tempfile.TemporaryDirectory(prefix="task0032-shared-reset-") as directory:
        exe=Path(directory)/"replay"
        subprocess.run(["c++","-std=c++11","-O1","-Wall","-Wextra","-Werror",
                        "-fsanitize=undefined","-fno-sanitize-recover=all","-I",str(HERE),
                        "-I",str(a.out),a.out/"lcd-shared-reset-fixture.cpp","-o",exe],check=True)
        result=subprocess.check_output([exe],text=True)
    print(result,end="")
    (a.out/"lcd-shared-reset-results.json").write_text(json.dumps({
        "base_sha256":hashlib.sha256(source.encode()).hexdigest(),
        "candidate_sha256":hashlib.sha256(candidate.encode()).hexdigest(),
        "reset_sha256":hashlib.sha256((HERE/"lcd-runtime-reset.h").read_bytes()).hexdigest(),
        "result":result.strip(),
        "qualification":"isolated host composition; no additional state members, not linked AVR resource or shipping safety acceptance"
    },indent=2)+"\n")

if __name__=="__main__":
    main()

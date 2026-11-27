#!/usr/bin/env python3
"""Task0034 task-private event-log core replay; no firmware wiring or EEPROM address."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
source=r"""
#include "EventLog.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <array>
using namespace EventLog;
constexpr uint8_t Slots=4;
using R=Ring<Slots>;
struct Store {
    std::array<uint8_t,R::Bytes> data;
    bool busy=false,otherOwner=false,deny=false,ignore=false;
    uint16_t address=0;
    uint8_t value=0;
    unsigned launches=0,reads=0;
    Store(){data.fill(255);}
    bool ready()const{return !busy&&!otherOwner;}
    uint8_t read(uint16_t a){assert(ready()&&a<data.size());++reads;return data[a];}
    bool startWrite(uint16_t a,uint8_t v){
        assert(ready()&&a<data.size());
        if(deny)return false;
        busy=true;address=a;value=v;++launches;return true;
    }
    void complete(){assert(busy);if(!ignore)data[address]=value;busy=false;}
    void cut(uint8_t v){assert(busy);data[address]=v;busy=false;}
};
Payload record(uint32_t n) {
    Payload p;
    p.id=eventId(Channel::Bed,6);p.uptime=n;p.window=0x12345678;
    p.values[0]=-123;p.values[1]=3000;p.values[2]=42;p.values[3]=-1;
    p.raw=4091;p.context[0]=255;p.context[1]=17;p.context[2]=3;
    p.context[3]=190;p.context[4]=2;p.valid=0x0fff;return p;
}
void scan(R& r,Store& s){
    for(uint8_t i=0;i<Slots;++i)assert(r.scanStep(s)==Progress::Advanced);
    assert(r.initialized()&&r.scanStep(s)==Progress::Idle);
}
void append(R& r,Store& s,const Payload& p) {
    assert(r.begin(p));assert(!r.begin(p));
    unsigned visits=0;
    while(r.pending()){
        assert(++visits<150);
        unsigned w=s.launches;
        if(s.busy){assert(r.step(s,p)==Progress::Busy);assert(w==s.launches);s.complete();}
        auto x=r.step(s,p);
        assert(x!=Progress::Failed&&s.launches-w<=1);
    }
    assert(!s.busy&&r.hasEntries());
}
void verify(R& r,Store& s,uint8_t slot,uint32_t seq,const Payload& p){
    uint32_t got=0;assert(R::header(s,slot,got)&&got==seq);
    for(uint8_t i=0;i<EntryBytes;++i){
        uint8_t v=0;assert(r.readByte(s,slot,i,v));
        uint8_t want=i==0?Committed:i==1?Version:i<6?byteOf(seq,i-2):p.byte(i-6);
        assert(v==want);
    }
}
void inject(Store& s,uint8_t slot,uint32_t seq,const Payload& p){
    auto a=slot*EntryBytes;s.data[a]=Committed;s.data[a+1]=Version;
    for(uint8_t i=0;i<4;++i)s.data[a+2+i]=byteOf(seq,i);
    for(uint8_t i=0;i<26;++i)s.data[a+6+i]=p.byte(i);
}
int main(){
    static_assert(R::Bytes==128,"region size");
    assert(eventId(Channel::System,0)==0&&eventId(Channel::System,64)==0);
    assert(newer(0,0xffffffffu)&&!newer(0xffffffffu,0)&&!newer(5,5));
    Store s;R r;
    s.otherOwner=true;assert(r.scanStep(s)==Progress::Busy);s.otherOwner=false;
    scan(r,s);assert(!r.hasEntries());
    Payload p=record(1);
    assert(p.byte(8)==0x85&&p.byte(9)==0xff&&p.byte(16)==0xfb&&p.byte(17)==0x0f);
    assert(!r.begin(Payload{}));
    for(uint32_t n=1;n<=40;++n){
        p=record(n);append(r,s,p);
        assert(r.latestSequence()==n&&r.latestSlot()==(n-1)%Slots);
        R reboot;scan(reboot,s);
        assert(reboot.latestSequence()==n&&reboot.latestSlot()==(n-1)%Slots);
        for(uint32_t k=n>Slots?n-Slots+1:1;k<=n;++k)
            verify(reboot,s,uint8_t((k-1)%Slots),k,record(k));
    }
    // Writer is not allowed to perform a read or launch while settings owns it.
    p=record(41);assert(r.begin(p));s.otherOwner=true;
    auto reads=s.reads,writes=s.launches;
    for(unsigned i=0;i<10;++i)assert(r.step(s,p)==Progress::Busy);
    assert(reads==s.reads&&writes==s.launches);
    s.otherOwner=false;s.deny=true;
    assert(r.step(s,p)==Progress::Busy&&!s.busy);s.deny=false;
    assert(r.step(s,p)==Progress::Advanced&&s.busy);
    // Ignored invalidation is detected before any payload mutation.
    s.ignore=true;s.complete();assert(r.step(s,p)==Progress::Failed);
    assert(r.failed()&&!r.pending()&&!r.begin(p));
    R recover;scan(recover,s);assert(recover.latestSequence()==40);
    s.ignore=false;
    // Power cut at every physical byte launch, every possible torn value.
    // The target slot can contain old record, complete new record, or no
    // valid record. Other slots remain unchanged. No CRC means unrelated
    // arbitrary corruption is not claimed detectable.
    unsigned cuts=0;
    Store base=s;
    for(unsigned cutAt=0;cutAt<33;++cutAt){
        for(unsigned v=0;v<256;++v){
            Store trial=base;R writer;scan(writer,trial);
            assert(writer.begin(p));unsigned index=0;
            bool cut=false;
            while(writer.pending()){
                auto x=writer.step(trial,p);assert(x!=Progress::Failed);
                if(trial.busy){
                    if(index++==cutAt){trial.cut(uint8_t(v));cut=true;break;}
                    trial.complete();
                }
            }
            if(!cut)break;
            ++cuts;
            R after;scan(after,trial);
            uint8_t target=0;
            for(uint8_t slot=1;slot<Slots;++slot)
                for(uint8_t k=0;k<EntryBytes;++k)
                    assert(trial.data[slot*EntryBytes+k]==base.data[slot*EntryBytes+k]);
            uint32_t seq=0;
            if(R::header(trial,target,seq)){
                assert(seq==37||seq==41);
                verify(after,trial,target,seq,record(seq));
            }
            assert(after.latestSequence()==40||after.latestSequence()==41);
            // Can continue appending after restart regardless of torn slot.
            append(after,trial,record(42));
        }
    }
    assert(cuts>0);
    Store wrap;
    inject(wrap,0,0xfffffffeu,record(1));
    inject(wrap,1,0xffffffffu,record(2));
    inject(wrap,2,0,record(3));
    inject(wrap,3,1,record(4));
    R wr;scan(wr,wrap);assert(wr.latestSequence()==1&&wr.latestSlot()==3);
    append(wr,wrap,record(5));verify(wr,wrap,0,2,record(5));
    // Reject unsupported versions and empty reason codes; not a CRC.
    wrap.data[1]=99;wrap.data[3*EntryBytes+31]=0;
    R bad;scan(bad,wrap);assert(bad.latestSequence()==0&&bad.latestSlot()==2);
    std::printf("Event log: 40 appends, %u torn-byte cuts, wrap, busy owner, ignored writes PASS\n",cuts);
}
"""
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"event-log.cpp"
    exe=Path(d)/"event-log"
    p.write_text(source)
    subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror","-I",
                    str(root/"tools/task0034/event-log"),str(p),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

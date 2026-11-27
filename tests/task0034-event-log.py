#!/usr/bin/env python3
"""Task0034 task-private event-log core replay; no firmware wiring or EEPROM address."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
source=r"""
#include "EventLog.h"
#include "EventLogRead.h"
#include "EventLogText.h"
#include "EventLogService.h"
#include <string>
#include "ThermalEventSource.h"
#include <limits>
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
// Model uses the actual public record interface from the pinned Task0034
// ledger; integrated replay must also instantiate with the production type.
enum class Cause : uint8_t {
    None, SensorInvalid, StaleSample, NoRise, ExcessiveRise, HoldFailure,
    OverTemperature, HeatingWhileOff, LeaseExpired, WatchdogReset,
    CalibrationMismatch, InterruptedHeating
};
struct Evidence {
    uint32_t uptimeMs=0,windowMs=0;
    float measuredC=0,targetC=0,deltaC=0,threshold=0;
    uint16_t rawAdc=0;
    uint8_t requestedDuty=0,admittedDuty=0,sensorType=0,powerLimit=0,profile=0;
    uint16_t valid=0;
    enum : uint16_t {
        Measured=1,Target=2,Delta=4,Threshold=8,Raw=16,Requested=32,
        Admitted=64,Profile=128,Uptime=256,Window=512,SensorType=1024,PowerLimit=2048
    };
};
struct Record {Evidence evidence;Cause cause=Cause::None;uint8_t channel=0;};
struct Ledger {
    Record initial,escalation;
    bool active()const{return initial.cause!=Cause::None;}
    const Record& first()const{return initial;}
    const Record& firstFire()const{return escalation;}
};
struct Sink {
    bool blocked=false;
    uint32_t sequences[Slots]{};
    uint8_t reasons[Slots]{},channels[Slots]{},count=0;
    bool tryEntry(const EntryView& v){
        if(blocked)return false;
        assert(count<Slots);
        sequences[count]=v.sequence();reasons[count]=v.reason();
        channels[count]=uint8_t(v.channel());++count;return true;
    }
};
void readAll(R& ring,Store& store,Sink& sink) {
    Reader<Slots> reader;assert(reader.begin(ring));
    for(unsigned i=0;i<Slots+1;++i){
        auto x=reader.step(ring,store,sink);
        if(x==ReadProgress::Done)return;
        assert(x==ReadProgress::Sent||x==ReadProgress::Skipped);
    }
    assert(false);
}
struct Line {
    std::string buffer;
    void character(char c){buffer+=c;}
    void text(const char* s){buffer+=s;}
    void number(uint32_t n){buffer+=std::to_string(n);}
    void unknown(){text("unknown");}
    void cause(Cause c){
        switch(c){
        case Cause::SensorInvalid:text("SENSOR INVALID");break;
        case Cause::OverTemperature:text("OVER TEMP");break;
        case Cause::CalibrationMismatch:text("CALIBRATION MISMATCH");break;
        case Cause::HeatingWhileOff:text("HEATING WHILE OFF");break;
        default:text("OTHER THERMAL ERROR");break;
        }
    }
    bool finish(){text("\r\n");return buffer.size()<=63;}
    const char* data()const{return buffer.data();}
    uint8_t size()const{return uint8_t(buffer.size());}
};
struct Serial {
    bool blocked=false;
    std::string all;
    unsigned lines=0;
    bool tryWriteSpan(const char* bytes,uint8_t count){
        if(blocked)return false;
        assert(count<=63);all.append(bytes,count);++lines;return true;
    }
};
void textTest(R& ring,Store& store) {
    Reader<Slots> reader;assert(reader.begin(ring));
    Serial serial;TextSink<Line,Cause,Serial> output(serial);
    serial.blocked=true;
    assert(reader.step(ring,store,output)==ReadProgress::Busy&&serial.lines==0);
    serial.blocked=false;
    for(unsigned i=0;i<100;++i){
        unsigned prior=serial.lines;
        auto p=reader.step(ring,store,output);
        assert(serial.lines-prior<=1&&!output.failed());
        if(p==ReadProgress::Done)break;
    }
    assert(serial.lines==12);
    assert(serial.all.find("LOG #2 BED OVER TEMP")!=std::string::npos);
    assert(serial.all.find("LOG #1 E0 SENSOR INVALID")!=std::string::npos);
    assert(serial.all.find("measured_C:unknown target_C:240.3")!=std::string::npos);
    assert(serial.all.find("raw_ADC:0 req_PWM:unknown admitted_PWM:217")!=std::string::npos);
    // Worst representable fields, longest labels, and unknown reason.
    uint8_t bytes[EntryBytes];std::memset(bytes,255,sizeof(bytes));
    bytes[31]=eventId(Channel::System,10);
    for(uint8_t i=0;i<4;++i){bytes[14+2*i]=0;bytes[15+2*i]=128;}
    EntryView v{bytes};
    for(uint8_t i=0;i<rows(v);++i){
        Line line;entryLine<Line,Cause>(line,v,i);assert(line.finish());
    }
    bytes[31]=eventId(Channel::System,63);
    Line line;entryLine<Line,Cause>(line,v,0);assert(rows(v)==1&&line.finish());
    assert(line.buffer.find("REASON:63")!=std::string::npos);
}
void serviceTest(){
    using S=Service<Slots,2,Line,Cause,Serial>;
    Store store;Serial serial;S service(serial);Ledger ledger;
    for(unsigned i=0;i<100;++i)service.service(store,ledger);
    assert(store.launches==0&&store.reads==0&&serial.lines==0);
    service.requestHistory();service.requestHistory();
    for(unsigned i=0;i<100&&service.historyPending();++i)
        service.service(store,ledger);
    assert(!service.historyPending()&&serial.all=="LOG BEGIN\r\nLOG END\r\n");
    ledger.initial.cause=Cause::WatchdogReset;ledger.initial.channel=2;
    ledger.initial.evidence.valid=0;
    for(unsigned i=0;i<200;++i){
        service.service(store,ledger);
        if(store.busy)store.complete();
    }
    assert(!service.logFailed()&&service.ring().latestSequence()==1);
    // Cold boot: history retrieved with no active fault. Logging cannot
    // manufacture a live fault or trigger heating decisions.
    Serial after;S restarted(after);Ledger normal;
    restarted.requestHistory();
    for(unsigned i=0;i<100&&restarted.historyPending();++i)
        restarted.service(store,normal);
    assert(!restarted.historyPending()&&!normal.active());
    assert(after.all.find("LOG #1 SYSTEM")!=std::string::npos);
    assert(after.all.find("measured_C:unknown target_C:unknown")!=std::string::npos);
    assert(after.all.find("LOG END")!=std::string::npos);
}
void captureTest(){
    int16_t v=0;
    assert(deciC(-3276.8f,v)&&v==-32768);
    assert(deciC(3276.7f,v)&&v==32767);
    assert(deciC(-12.35f,v)&&v==-124);
    assert(!deciC(std::numeric_limits<float>::quiet_NaN(),v));
    assert(!deciC(std::numeric_limits<float>::infinity(),v));
    Ledger l;
    l.initial.cause=Cause::SensorInvalid;l.initial.channel=0;
    auto& e=l.initial.evidence;
    e.targetC=240.3f;e.measuredC=std::numeric_limits<float>::quiet_NaN();
    e.deltaC=5000.f;e.threshold=-20.f;e.admittedDuty=217;e.rawAdc=0;
    e.uptimeMs=0x12345678;e.valid=Evidence::Target|Evidence::Measured|
        Evidence::Delta|Evidence::Threshold|Evidence::Admitted|Evidence::Raw|Evidence::Uptime;
    ThermalSource<Record,2> src(l.initial);
    assert(src.byte(25)==eventId(Channel::E0,1));
    assert(src.byte(23)==uint8_t(e.valid&~(Evidence::Measured|Evidence::Delta)));
    assert(src.byte(24)==uint8_t(e.valid>>8));
    Store store;R ring;ThermalCapture<2> capture;
    // Escalate while first record is still being written.
    for(unsigned i=0;i<Slots+3;++i){
        capture.service(ring,store,l);
        if(store.busy)store.complete();
    }
    assert(ring.pending());
    l.escalation.cause=Cause::OverTemperature;l.escalation.channel=1;
    l.escalation.evidence.measuredC=280.f;
    l.escalation.evidence.valid=Evidence::Measured;
    for(unsigned i=0;i<200;++i){
        auto x=capture.service(ring,store,l);assert(x!=Progress::Failed);
        if(store.busy)store.complete();
    }
    assert(ring.latestSequence()==2&&!ring.pending());
    uint8_t bytes[EntryBytes];assert(ring.readEntry(store,0,bytes));
    EntryView first{bytes};
    assert(first.value(0)==0&&first.value(1)==2403&&first.value(2)==0&&first.value(3)==-200);
    assert(first.uptime()==0x12345678&&first.raw()==0&&first.context(1)==217);
    assert((first.valid()&Evidence::Raw)&&!(first.valid()&Evidence::Measured));
    assert(ring.readEntry(store,1,bytes));
    EntryView fire{bytes};
    assert(fire.channel()==Channel::Bed&&fire.reason()==6&&fire.value(0)==2800);
    Sink sink;readAll(ring,store,sink);
    assert(sink.count==2&&sink.sequences[0]==2&&sink.sequences[1]==1);
    textTest(ring,store);
    // Initial fire is the same event, not a second duplicate append.
    Ledger lf;lf.initial=l.escalation;lf.escalation=l.escalation;
    Store sf;R rf;ThermalCapture<2> cf;
    for(unsigned i=0;i<200;++i){
        assert(cf.service(rf,sf,lf)!=Progress::Failed);
        if(sf.busy)sf.complete();
    }
    assert(rf.latestSequence()==1);
    // Stable physical channel identity, independent of fitted heater count.
    Record rr;rr.cause=Cause::LeaseExpired;rr.channel=3;
    assert((ThermalSource<Record,3>(rr).byte(25)==eventId(Channel::System,8)));
    rr.channel=1;assert((ThermalSource<Record,3>(rr).byte(25)==eventId(Channel::E1,8)));
    rr.channel=2;assert((ThermalSource<Record,3>(rr).byte(25)==eventId(Channel::Bed,8)));
}
int main(){
    captureTest();
    serviceTest();
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
    Sink listed;readAll(r,s,listed);
    assert(listed.count==Slots&&listed.sequences[0]==40&&listed.sequences[3]==37);
    Reader<Slots> reader;Sink blocked;blocked.blocked=true;assert(reader.begin(r));
    assert(reader.step(r,s,blocked)==ReadProgress::Busy&&blocked.count==0);
    blocked.blocked=false;assert(reader.step(r,s,blocked)==ReadProgress::Sent&&blocked.sequences[0]==40);
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
    // Force differences in every payload byte rather than just uptime.
    p=record(41);p.uptime=0xfefdfcfb;p.window=0xabcdef01;
    p.values[0]=0x1234;p.values[1]=-2;p.values[2]=0x4321;p.values[3]=0x1234;
    p.raw=0x1234;p.context[0]=11;p.context[1]=22;p.context[2]=33;
    p.context[3]=44;p.context[4]=55;p.valid=0x1234;p.id=eventId(Channel::E1,7);
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
                verify(after,trial,target,seq,seq==41?p:record(seq));
            }
            assert(after.latestSequence()==40||after.latestSequence()==41);
            // Can continue appending after restart regardless of torn slot.
            append(after,trial,record(42));
        }
    }
    assert(cuts>=25*256);
    Store wrap;
    inject(wrap,0,0xfffffffeu,record(1));
    inject(wrap,1,0xffffffffu,record(2));
    inject(wrap,2,0,record(3));
    inject(wrap,3,1,record(4));
    R wr;scan(wr,wrap);assert(wr.latestSequence()==1&&wr.latestSlot()==3);
    append(wr,wrap,record(5));verify(wr,wrap,0,2,record(5));
    Sink wrapped;readAll(wr,wrap,wrapped);
    assert(wrapped.count==Slots&&wrapped.sequences[0]==2&&wrapped.sequences[3]==0xffffffffu);
    Reader<Slots> changed;assert(changed.begin(wr));
    append(wr,wrap,record(6));
    Sink unused;assert(changed.step(wr,wrap,unused)==ReadProgress::Changed);
    // Restore the prior four entries for the corruption test.
    inject(wrap,1,0xffffffffu,record(2));
    // Reject unsupported versions and empty reason codes; not a CRC.
    wrap.data[1]=99;wrap.data[3*EntryBytes+31]=0;
    R bad;scan(bad,wrap);assert(bad.latestSequence()==0&&bad.latestSlot()==2);
    std::printf("Event log: 40 appends, %u torn-byte cuts, wrap, busy owner, ignored writes, thermal evidence/escalation, read order and bounded serial output PASS\n",cuts);
}
"""
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"event-log.cpp"
    exe=Path(d)/"event-log"
    p.write_text(source)
    subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror","-I",
                    str(root/"tools/task0034/event-log"),str(p),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

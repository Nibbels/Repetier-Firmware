#!/usr/bin/env python3
"""Replay the actual foreground history binding with real production ledger/format.
Host EEPROM simulates asynchronous byte completion, not electrical timing.
"""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
fw=root/"Repetier"
source=(fw/"EepromZoneAvr.cpp").read_text()
binding=source[source.index("namespace DiagnosticHistory {"):source.rindex("#endif")]
commands=(fw/"Commands.cpp").read_text()
assert commands.count("DiagnosticHistory::service();")==2
assert commands.index("DiagnosticHistory::service();")<commands.index("} // commandLoop")
assert commands.rindex("DiagnosticHistory::service();")<commands.index("} // commandLoop")
assert "case DiagnosticHistory::ReadCommand: return DiagnosticHistory::request();" in (fw/"GCodeCommands.cpp").read_text()
receiver=(fw/"GCodeInput.cpp").read_text()
assert "command.M==DiagnosticHistory::ReadCommand" in receiver
assert receiver.index("DiagnosticHistory::request();")<receiver.index("if (command.hasM()) {",receiver.index("DiagnosticHistory::request();"))
stub=r'''
#include <cassert>
#include <array>
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <string>
#define RF_THERMAL_EMERGENCY_EXPERIMENT
#define E2END 4095
#include "DiagnosticHistory.h"
#include "EepromZoneLayout.h"
#include "ThermalEmergencyReport.h"
#include "EventLogService.h"
bool ownerBusy=false,irq=true,programming=false,txBlocked=false;
unsigned reads=0,launches=0;
uint16_t writePos=0;
uint8_t writeValue=0;
std::array<uint8_t,E2END+1> cells;
std::string tx;
unsigned conditions=0;
namespace EepromZones {
struct AvrDevice {
    static bool logMayAccess(){return !ownerBusy&&irq;}
};
}
namespace EventLog {
template<uint16_t Base,uint8_t Slots,class Owner> struct AvrStore {
    const Owner& owner;
    explicit AvrStore(const Owner& o):owner(o){}
    bool ready()const{return owner.logMayAccess()&&!programming;}
    uint8_t read(uint16_t off)const {
        assert(ready()&&off<Ring<Slots>::Bytes);
        ++reads;return cells[Base+off];
    }
    bool startWrite(uint16_t off,uint8_t value)const {
        assert(ready()&&off<Ring<Slots>::Bytes);
        ++launches;programming=true;writePos=Base+off;writeValue=value;
        return true;
    }
};
}
struct UsbSerial {
    bool tryWriteSpan(const char* bytes,uint8_t size) {
        assert(size<=63);
        if(txBlocked)return false;
        tx.append(bytes,size);return true;
    }
};
namespace ThermalEmergency {
Ledger<NUM_TEMPERATURE_LOOPS> live;
const Ledger<NUM_TEMPERATURE_LOOPS>& faultLedger(){return live;}
}
namespace HAL { uint32_t timeInMilliseconds(){return 123456;} }
struct Printer {
    struct Job {bool isBusy()const{return conditions&1;}} static printJob;
    struct Pause {bool isPaused()const{return conditions&2;}} static pause;
    static bool needsCMPwait(){return conditions&16;}
};
Printer::Job Printer::printJob;
Printer::Pause Printer::pause;
struct PrintLine {
    static bool hasLines(){return conditions&4;}
    static bool directMoveActive(){return conditions&8;}
};
bool isAnyScanRunning(){return conditions&32;}
struct TemperatureController {static bool autotuneActive(){return conditions&64;}};
'''
test=r'''
void complete() {
    if(programming){cells[writePos]=writeValue;programming=false;}
}
void visit(bool allowComplete=true) {
    unsigned before=launches;
    DiagnosticHistory::service();
    assert(launches-before<=1);
    if(allowComplete)complete();
}
void drain(unsigned n=600) {
    for(unsigned i=0;i<n;++i)visit();
}
int main() {
    using namespace DiagnosticHistory;
    using namespace ThermalEmergency;
    using EventLog::Channel;
    cells.fill(0x5a);
    std::fill(cells.begin()+0x0e00,cells.end(),0xff);
    drain(10);assert(reads==0&&launches==0);
    recordScan(ScanAbort::Canceled,ScanKind::HeatBed);
    drain(10);assert(reads==0&&launches==0);
    recordScan(ScanAbort::DmsOverload,ScanKind::HeatBed);
    recordScan(ScanAbort::MissingMatrix,ScanKind::ZOffset); // one compact slot
    assert(reads==0&&launches==0);
    for(unsigned bit=1;bit<=64;bit<<=1){
        conditions=bit;drain(3);assert(reads==0&&launches==0);
    }
    conditions=0;ownerBusy=true;drain(3);assert(reads==0&&launches==0);
    ownerBusy=false;irq=false;drain(3);assert(reads==0&&launches==0);
    irq=true;
    for(unsigned i=0;i<100&&!programming;++i)visit(false);
    assert(programming&&launches==1);
    unsigned r=reads,w=launches;
    for(unsigned i=0;i<10;++i)visit(false);
    assert(reads==r&&launches==w);
    // Escalate while a scan record is physically in flight. All idle
    // gates must be irrelevant in the terminal state, but ownership stays.
    Evidence first{};first.targetC=217.5f;first.valid=Evidence::Target;
    assert(live.record(0,Cause::SensorInvalid,first));
    Evidence fire{};fire.measuredC=288.5f;fire.valid=Evidence::Measured;
    assert(live.record(NUM_TEMPERATURE_LOOPS-1,Cause::OverTemperature,fire));
    complete();conditions=127;ownerBusy=true;
    drain(10);assert(reads==r&&launches==w);
    ownerBusy=false;drain();
    assert(history.ring().latestSequence()==3);
    assert(cells[0xe00]==EventLog::Committed&&cells[0xe20]==EventLog::Committed&&
           cells[0xe40]==EventLog::Committed);
    assert(cells[0xe1f]==EventLog::eventId(Channel::System,27));
    assert(cells[0xe3f]==EventLog::eventId(Channel::E0,1));
    assert(cells[0xe5f]==EventLog::eventId(Channel::Bed,6));
    assert(std::all_of(cells.begin(),cells.begin()+0xe00,[](uint8_t x){return x==0x5a;}));
    w=launches;request();txBlocked=true;drain(30);assert(tx.empty()&&launches==w);
    txBlocked=false;drain();
    auto fireLine=tx.find("LOG #3 BED OVER TEMP");
    auto firstLine=tx.find("LOG #2 E0 SENSOR INVALID");
    auto scanLine=tx.find("LOG #1 HBS CODE:27");
    assert(fireLine<firstLine&&firstLine<scanLine&&tx.find("LOG END")!=std::string::npos);
    assert(tx.find("measured_C:288.5")!=std::string::npos);
    assert(tx.find("target_C:217.5")!=std::string::npos);
    assert(launches==w&&live.active());
    // Simulate fresh controller state after reset, retaining physical bytes.
    UsbSerial newSink;
    EventLog::Service<16,NUM_TEMPERATURE_LOOPS,ReportLine,Cause,UsbSerial> reboot(newSink);
    Ledger<NUM_TEMPERATURE_LOOPS> inactive;
    tx.clear();reboot.requestHistory();
    for(unsigned i=0;i<600;++i)reboot.service(store,inactive);
    assert(tx.find("LOG #3 BED OVER TEMP")!=std::string::npos&&tx.find("LOG END")!=std::string::npos);
    assert(launches==w&&!inactive.active());
    std::printf("Actual diagnostic binding: %d heaters, idle/owner/IRQ exclusion, one byte, fault escalation, immutable settings, reset history PASS\n",NUM_TEMPERATURE_LOOPS);
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"history.cpp";exe=Path(d)/"history"
    p.write_text(stub+binding+test)
    for n in (2,3):
        subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror",
                        f"-DNUM_TEMPERATURE_LOOPS={n}","-I",str(fw),str(p),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True,timeout=10)

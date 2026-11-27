#!/usr/bin/env python3
"""Exercise actual transformed UI emergency and periodic scheduler functions."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile

def section(s,first,last):
    i=s.index(first)
    return s[i:s.index(last,i+len(first))]

FIXTURE=r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "LcdPendingFrame.h"
#include "ThermalEmergencyPanel.h"
#define UI_ROWS 4
#define RF_THERMAL_EMERGENCY_EXPERIMENT 1
#define LCD_CLEAR 1
using millis_t=uint32_t;
using namespace ThermalEmergency;
static Ledger<HEATERS> ledger;
namespace ThermalEmergency {
const Ledger<HEATERS>& faultLedger(){return ledger;}
bool liveTemperatureValid(uint8_t ch){return ch<HEATERS;}
}
static struct Heater {float currentTemperatureC=145;} heaters[HEATERS];
static Heater* tempController[HEATERS];
static std::vector<std::string> events;
namespace HAL {
uint32_t now=0;
bool scan=false,comp=false,feature=false,temp=false;
uint32_t timeInMicroseconds(){return now;}
millis_t timeInMilliseconds(){return now/1000;}
void delayMilliseconds(unsigned ms){now+=ms*1000;}
bool takeScanTick(){bool b=scan;scan=false;return b;}
bool takeCompensationTick(){bool b=comp;comp=false;return b;}
bool takeFeatureTick(){bool b=feature;feature=false;return b;}
bool takeTemperatureTick(){bool b=temp;temp=false;return b;}
bool temperatureTickPending(){return temp;}
void tellWatchdogOk(){events.push_back("watchdog");}
}
namespace Printer {
struct {bool locked=false;bool emergencyLocked()const{return locked;}} printJob;
struct {void service(millis_t){events.push_back("strain");}} strainGauge;
}
static bool faultInThermal=false;
namespace Extruder {
void manageTemperatures(){
    events.push_back("thermal");
    if(faultInThermal)Printer::printJob.locked=true;
}
}
namespace Lighting { void serviceAlarm(millis_t){events.push_back("alarm");} }
enum FirmwareState {NotBusy,Processing,WaitHeater};
namespace GCodeInput {void keepAlive(FirmwareState){events.push_back("keepalive");}}
namespace Commands {
void checkForPeriodicalActions(FirmwareState);
void printTemperatures(){events.push_back("temperature-report");}
}
void recalculateZCompensation(){events.push_back("compensation");}
void loopFeatures(){events.push_back("features");}
void handleScanWorkTasks(bool){events.push_back("scan");}
static char memory[128];
static unsigned transfers=0, syncWrites=0;
static uint8_t address=0xff;
void lcdCommand(uint8_t c){
    assert(c==LCD_CLEAR);memset(memory,' ',sizeof memory);++syncWrites;
}
void lcdWriteByteNow(uint8_t v,uint8_t rs){
    ++transfers;events.push_back("lcd");
    if(rs){assert(address<128&&v);memory[address++]=char(v);}
    else {assert(v&128);address=v&127;}
}
struct UIDisplay {
    bool normalCharacters_=true;
    uint8_t emergencyFrameRevision_=0xff;
    millis_t lastDisplayCompletedMs_=0;
    char displayCache_[4][UI_COLS+1]{};
    LcdPendingFrame<UI_COLS> lcdTransport_;
    unsigned refreshes=0;
    void resetDisplayAfterClear();
    void serviceDisplay();
    void refreshPage();
    void loadNormalCharacters(){normalCharacters_=true;}
    void renderNormalPage(){
        ++refreshes;
        for(uint8_t r=0;r<4;++r)lcdTransport_.stage(r,"NORMAL OLD TEXT",displayCache_);
    }
    void slowAction(){events.push_back("keys");}
    millis_t lastButtonTime()const{return HAL::timeInMilliseconds();}
} uid;
'''

MAIN=r'''
std::string row(unsigned r){
    return std::string(memory+(r&1?64:0)+(r>=2?UI_COLS:0),UI_COLS);
}
std::string logical(unsigned r){return uid.displayCache_[r];}
std::string padded(std::string s){s.resize(UI_COLS,' ');return s;}
void service(){
    auto n=transfers;HAL::now+=324;
    uid.serviceDisplay();
    assert(transfers<=n+1);
}
void drain(){
    for(unsigned n=0;uid.lcdTransport_.pending();++n){assert(n<8*UI_COLS+1);service();}
    for(unsigned r=0;r<4;++r)assert(row(r)==logical(r));
}
int main(){
    for(unsigned i=0;i<HEATERS;++i)tempController[i]=&heaters[i];
    memset(memory,' ',sizeof memory);
    uid.resetDisplayAfterClear();
    uid.refreshPage();
    assert(transfers==0&&uid.lcdTransport_.pending());
    service(); // address of first ordinary byte
    auto n=transfers;
    Evidence e{};e.targetC=210;e.valid=Evidence::Target;
    assert(ledger.record(0,Cause::NoRise,e));
    Printer::printJob.locked=true;
    service(); // latch before refresh: no stale ordinary byte
    assert(transfers==n);
    uid.refreshPage();
    assert(transfers==n && logical(0)==padded("THERMAL ERROR"));
    service(); // start primary warning
    service();
    e.targetC=90;
    assert(ledger.record(HEATERS-1,Cause::OverTemperature,e));
    n=transfers;
    service(); // newer revision blocks stale previous warning
    assert(transfers==n);
    uid.refreshPage();
    assert(logical(0)==padded("FIRE WARNING"));
    assert(logical(1)==padded("BED:OVER TEMP"));
    assert(logical(3)==padded("POWER OFF! CHECK"));
    drain();
    n=transfers;
    uid.refreshPage();service();assert(transfers==n);
    heaters[HEATERS-1].currentTemperatureC=151;
    uid.refreshPage();drain();
    assert(row(2)==padded("BED:151/90 C"));

    // Normal pump (separate simulated machine lifetime). The real
    // scheduler must run priority work before the one-byte service.
    Printer::printJob.locked=false;
    uid.refreshPage();
    events.clear();HAL::now+=324;
    HAL::scan=HAL::comp=HAL::feature=HAL::temp=true;
    n=transfers;Commands::checkForPeriodicalActions(Processing);
    assert(transfers==n+1);
    const std::vector<std::string> expected={"keepalive","watchdog","compensation","thermal","keys","strain","features","scan","lcd"};
    assert(events==expected);

    // Latch during thermal service, no normal features or scan after it.
    events.clear();HAL::now+=324;HAL::temp=HAL::feature=true;
    faultInThermal=true;n=transfers;
    Commands::checkForPeriodicalActions(Processing);
    assert(Printer::printJob.locked && transfers<=n+1);
    assert(events.size()>=2 && events[0]=="keepalive" && events[1]=="thermal");
    for(auto& e:events)assert(e!="scan"&&e!="features"&&e!="strain"&&e!="keys");
    // Locked pass without a thermal tick can still drain one LCD byte.
    events.clear();HAL::now+=324;n=transfers;
    Commands::checkForPeriodicalActions(NotBusy);
    assert(transfers==n+1 && events==std::vector<std::string>{"lcd"});
    events.clear();HAL::now+=324;HAL::temp=true;
    Commands::checkForPeriodicalActions(NotBusy);
    assert(events.size()>=2&&events[0]=="thermal"&&events[1]=="alarm");
    assert(syncWrites==0);
    std::printf("width %d heaters %d: actual emergency revision supersession and periodic service order PASS\n",UI_COLS,HEATERS);
}
'''

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source",type=Path,required=True)
    a=p.parse_args()
    fw=a.source/"Repetier"
    ui=(fw/"ui.cpp").read_text()
    commands=(fw/"Commands.cpp").read_text()
    extracted=(section(ui,"void UIDisplay::resetDisplayAfterClear()","void UIDisplay::printRow(")
               +section(ui,"void UIDisplay::refreshPage()","void UIDisplay::showMessage("))
    scheduler=section(commands,"static void serviceThermalEmergency()","bool Commands::waitForMotion(")
    # The extraction starts inside the outer feature #ifdef. Retain all
    # source preprocessor guards, including its closing endif.
    scheduler="#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\n"+scheduler
    with tempfile.TemporaryDirectory(prefix="task0032-lcd-emergency-") as directory:
        d=Path(directory);cpp=d/"test.cpp";exe=d/"test"
        cpp.write_text(FIXTURE+extracted+scheduler+MAIN)
        for cols,heaters in ((16,2),(20,3),(20,2)):
            subprocess.run([os.environ.get("CXX","c++"),"-std=c++11","-O1","-Wall","-Wextra","-Werror",
                            "-fsanitize=address,undefined","-fno-sanitize-recover=all","-fno-pie","-no-pie",
                            "-I",str(fw),f"-DUI_COLS={cols}",f"-DHEATERS={heaters}",cpp,"-o",exe],check=True)
            subprocess.run([exe],check=True,env={**os.environ,"ASAN_OPTIONS":"detect_leaks=0"})

if __name__=="__main__":
    main()

#!/usr/bin/env python3
"""Replay terminal dispatch/report/panel on exact P0-derived candidate bodies.

Transport loss must not synthesize thermal evidence, ack corrupt input, or
starve the existing thermal service. Real sensor/actuator timing is outside
this host fixture. No shipping source changes.
"""
from pathlib import Path
import argparse
import hashlib
import json
import runpy
import subprocess
import tempfile

base=runpy.run_path(str(Path(__file__).with_name("transport-terminal-preflight.py")))
once=base["once"]
region=base["region"]
P0=base["P0"]

def candidate(source):
    files=base["candidate"](source)
    t=files["TemperatureControl.cpp"]
    t=once(t,"    if(!ledger.active())return;\n    // At most one whole nonblocking span",
        "    if(!ledger.active()&&!Printer::printJob.transportEmergency())return;\n    // At most one whole nonblocking span")
    t=once(t,"    if(cancellation.service(true,sink)!=ReportProgress::Idle)return;\n    const auto progress=report.service(ledger,sink);",
        """    // Keep cancellation shared across transport->thermal promotion.
    if(cancellation.service(true,sink)!=ReportProgress::Idle)return;
    if(Printer::printJob.transportEmergency()) {
        static bool sent=false;
        if(!sent) {
            ReportLine line;
            line.text(PSTR("!! COMMUNICATION LOST"));
            if(line.finish()&&sink.tryWriteSpan(line.data(),line.size()))sent=true;
        }
        return;
    }
    const auto progress=report.service(ledger,sink);""")
    files["TemperatureControl.cpp"]=t
    u=(source/"ui.cpp").read_text()
    marker="    const auto& ledger=ThermalEmergency::faultLedger();"
    u=once(u,marker,
        """    if(Printer::printJob.transportEmergency()) {
        // Fixed four-row page. No synthetic thermal record, no menu actions.
        for(uint8_t row=0;row<UI_ROWS;++row) {
            char line[UI_COLS+1];
            ThermalEmergency::PanelLine<UI_COLS> out(line);
            switch(row) {
            case 0: out.text(PSTR("COMMUNICATION"));break;
            case 1: out.text(PSTR("PRINT STOPPED"));break;
            case 2: out.text(PSTR("CHECK SERIAL"));break;
            case 3: out.text(PSTR("RESTART PRINTER"));break;
            }
            printRow(row,line,nullptr,UI_COLS);
        }
        return;
    }
"""+marker)
    files["ui.cpp"]=u
    g=(source/"GCodeInput.cpp").read_text()
    # Do not parse the quarantined transport stream or grant host credit.
    # After real thermal promotion existing thermal-only diagnostic policy
    # applies; receiver lifetime/ISR quarantine remains a later integration.
    g=once(g,"void GCodeInput::readFromSerial() {\n#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT",
        "void GCodeInput::readFromSerial() {\n#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\n"
        "    if(Printer::printJob.transportEmergency())return;")
    g=once(g,"void GCodeInput::requestResend() {\n#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT",
        "void GCodeInput::requestResend() {\n#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\n"
        "    if(Printer::printJob.transportEmergency())return;")
    g=once(g,"void GCodeInput::checkAndPushCommand(GCode& command) {\n#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT",
        "void GCodeInput::checkAndPushCommand(GCode& command) {\n#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\n"
        "    if(Printer::printJob.transportEmergency())return;")
    files["GCodeInput.cpp"]=g
    return files

PRELUDE=r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <string>
#include <vector>
#define RF_THERMAL_EMERGENCY_EXPERIMENT 1
#define RF_EEPROM_ZONE_EXPERIMENT 1
#define PSTR(x) x
#define NUM_TEMPERATURE_LOOPS HEATERS
#define NUM_EXTRUDER (HEATERS-1)
#define UI_COLS WIDTH
#define UI_ROWS 4
#define LCD_CLEAR 0
#include "PrintJob.h"
#include "ThermalEmergencyReport.h"
#include "ThermalEmergencyPanel.h"
struct Printer {static PrintJob printJob;};
PrintJob Printer::printJob;
static unsigned attempts,freeBytes,thermalVisits,alarmVisits,normalPages;
static unsigned scanTicks,compTicks,featureTicks,temperatureTicks,oldRetires,readVisits,historyVisits;
static bool tick, promote;
static uint32_t clockMs;
static std::vector<std::string> wire,rows;
struct UsbSerial {
    static bool tryWriteSpan(const char* s,uint8_t n) {
        ++attempts;
        assert(n<=63);
        if(n>freeBytes)return false;
        freeBytes-=n;
        wire.emplace_back(s,n);return true;
    }
};
namespace HAL {
uint32_t timeInMilliseconds(){return clockMs;}
void delayMilliseconds(unsigned){}
bool takeScanTick(){++scanTicks;return true;}
bool takeCompensationTick(){++compTicks;return true;}
bool takeFeatureTick(){++featureTicks;return true;}
bool takeTemperatureTick(){++temperatureTicks;bool value=tick;tick=false;return value;}
}
struct Temp {uint8_t pwmIndex=0;float currentTemperatureC=42;};
static Temp controllers[NUM_TEMPERATURE_LOOPS];
static Temp* tempController[NUM_TEMPERATURE_LOOPS];
struct Extruder {
    static Extruder* current;
    Temp tempControl;
    static void manageTemperatures();
};
Extruder extruder;
Extruder* Extruder::current=&extruder;
namespace ThermalEmergency {
Ledger<NUM_TEMPERATURE_LOOPS> ledger;
ReportCursor<NUM_TEMPERATURE_LOOPS> report;
CancellationCursor cancellation;
bool liveRequested=false;
uint32_t lastLiveReport=0;
uint8_t liveChannel=0;
bool liveTemperatureValid(uint8_t channel){assert(channel<NUM_TEMPERATURE_LOOPS);return true;}
const Ledger<NUM_TEMPERATURE_LOOPS>& faultLedger(){return ledger;}
REPORT
}
void Extruder::manageTemperatures() {
    ++thermalVisits;
    if(promote) {
        promote=false;
        assert(ThermalEmergency::ledger.record(0,ThermalEmergency::Cause::SensorInvalid,{}));
        assert(!Printer::printJob.beginThermalEmergency()); // already terminal
    }
    ThermalEmergency::serviceReport();
}
namespace Lighting {void serviceAlarm(uint32_t){++alarmVisits;}}
struct UIDisplay {
    bool normalCharacters_=true;
    void lcdCommand(unsigned){}
    void loadNormalCharacters(){normalCharacters_=true;}
    void printRow(uint8_t row,const char* text,const char*,unsigned width) {
        assert(row==rows.size()%4 && std::string(text).size()<=width);
        rows.emplace_back(text);
    }
    void refreshEmergencyPage();
    void refreshPage();
    void refreshNormalPage(){++normalPages;}
} uid;
PANEL
PUMP
enum FirmwareState {NotBusy,Processing};
namespace GCodeInput {
void readFromSerial(){++readVisits;if(Printer::printJob.transportEmergency())return;}
void popCurrentCommand(){++oldRetires;}
}
namespace DiagnosticHistory {void service(){++historyVisits;}}
namespace Commands {
void checkForPeriodicalActions(FirmwareState) {assert(Printer::printJob.emergencyLocked());serviceThermalEmergency();}
void commandLoop() {
OUTER
    assert(false); // not reachable in any terminal fixture
}
}
'''

SCENARIOS=r'''
int main(int argc,char** argv){
    assert(argc==2);
    const std::string mode=argv[1];
    for(uint8_t i=0;i<NUM_TEMPERATURE_LOOPS;++i){
        controllers[i].pwmIndex=i;tempController[i]=&controllers[i];
    }
    assert(Printer::printJob.beginTransportEmergency());
    // Continuous backpressure cannot suppress real thermal passes or UI.
    for(unsigned n=0;n<100;++n) {
        const auto old=attempts;
        tick=true;clockMs=n*100;freeBytes=0;
        Commands::commandLoop();
        assert(attempts==old+1 && thermalVisits==n+1 && alarmVisits==n+1);
        assert(wire.empty() && rows.size()==4*(n+1) && !normalPages);
        assert(rows[4*n]=="COMMUNICATION" && rows[4*n+1]=="PRINT STOPPED"
            && rows[4*n+2]=="CHECK SERIAL" && rows[4*n+3]=="RESTART PRINTER");
        assert(!ThermalEmergency::ledger.active());
    }
    assert(scanTicks==100 && compTicks==100 && featureTicks==100);
    assert(oldRetires==100 && readVisits==100 && historyVisits==100);
    if(mode=="promote-early")promote=true;
    // At most one whole response per thermal visit, no spurious ACK.
    for(unsigned n=0;n<5;++n){
        freeBytes=63;tick=true;clockMs+=100;auto old=wire.size();
        Commands::commandLoop();assert(wire.size()<=old+1);
    }
    assert(wire.size()>=2 && wire[0]=="RequestStop:\r\n" && wire[1]=="// action:cancel\r\n");
    if(mode!="promote-early"){
        assert(wire.size()==3 && wire[2]=="!! COMMUNICATION LOST\r\n");
        assert(!ThermalEmergency::ledger.active() && !Printer::printJob.thermalEmergency());
        if(mode=="promote-late")promote=true;
    }
    // Genuine subsequent fault retains actual thermal diagnostic priority.
    if(mode!="transport-only"){
        for(unsigned n=0;n<60;++n){
            freeBytes=63;tick=true;clockMs+=100;auto old=wire.size();
            Commands::commandLoop();assert(wire.size()<=old+1);
        }
        assert(Printer::printJob.thermalEmergency() && ThermalEmergency::ledger.active());
        assert(rows[rows.size()-4]=="THERMAL ERROR");
        unsigned requests=0,cancels=0,thermal=0;
        for(const auto& w:wire){
            requests+=w=="RequestStop:\r\n";cancels+=w=="// action:cancel\r\n";
            thermal+=w=="Error: THERMAL EMERGENCY\r\n";
            assert(w.rfind("ok",0)!=0);
        }
        assert(requests==1 && cancels==1 && thermal==1);
    }
    tick=false;const unsigned old=thermalVisits;
    Commands::commandLoop();assert(thermalVisits==old && !normalPages);
    std::printf("Terminal dispatch %s: %u cols, %u heaters, %u thermal passes, whole-span reports and panel passed\n",
        argv[1],UI_COLS,NUM_TEMPERATURE_LOOPS,thermalVisits);
}
'''

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",type=Path,required=True)
    parser.add_argument("--out",type=Path,required=True)
    args=parser.parse_args()
    assert subprocess.check_output(["git","-C",str(args.source),"rev-parse","HEAD"],text=True).strip()==P0
    source=args.source/"Repetier"
    args.out.mkdir(parents=True,exist_ok=True)
    files=candidate(source)
    for name,s in files.items():(args.out/name).write_text(s)
    t=region(files["TemperatureControl.cpp"],"static void serviceReport()","\n}\n}\n#endif")+ "\n}"
    panel=region(files["ui.cpp"],"void UIDisplay::refreshPage()","__attribute__((noinline)) void UIDisplay::refreshNormalPage()")
    c=(source/"Commands.cpp").read_text()
    pump=region(c,"static void serviceThermalEmergency()","\n#endif")
    outer=region(c,"void Commands::commandLoop() {","\n    GCodeInput::readFromSerial();")
    outer=outer[outer.index("\n")+1:]
    test=PRELUDE.replace("REPORT",t).replace("PANEL",panel).replace("PUMP",pump).replace("OUTER",outer)+SCENARIOS
    # Three exact generated receiver guards: compile only the early admission
    # decisions here; full parser/ISR composition remains a separate fixture.
    for name in ("readFromSerial()","requestResend()","checkAndPushCommand(GCode& command)"):
        assert "if(Printer::printJob.transportEmergency())return;" in files["GCodeInput.cpp"].split("void GCodeInput::"+name+" {",1)[1][:150]
    (args.out/"dispatch.cpp").write_text(test)
    for width in (16,20):
        for heaters in (2,3):
            with tempfile.TemporaryDirectory() as d:
                exe=Path(d)/"dispatch"
                subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror",
                    "-fsanitize=undefined","-fno-sanitize-recover=all",
                    f"-DWIDTH={width}",f"-DHEATERS={heaters}","-I",str(source),str(args.out/"dispatch.cpp"),"-o",str(exe)],check=True)
                for mode in ("transport-only","promote-early","promote-late"):
                    print(subprocess.check_output([str(exe),mode],text=True),end="")
    (args.out/"summary.json").write_text(json.dumps({"p0":P0,
        "candidate_sha256":{n:hashlib.sha256((args.out/n).read_bytes()).hexdigest() for n in files},
        "limits":"actual extracted report, panel, terminal pump/outer branch; stubbed hardware, receiver guards inspected, not full production integration"},indent=2)+"\n")

if __name__=="__main__":
    main()

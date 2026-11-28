#!/usr/bin/env python3
"""Compose a reachable fatal TX-write trigger with terminal-only P0 candidate.

Actual source-derived commandLoop and periodic scheduler replays with
fault injection at foreground boundaries. No new ISR or nested scheduler
call from the transport. This is not the final combined RX-resend receiver.
"""
from pathlib import Path
import argparse
import runpy
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[2]
dispatch=runpy.run_path(str(Path(__file__).with_name("transport-terminal-dispatch.py")))
writer=runpy.run_path(str(Path(__file__).with_name("tx-blocking-preflight.py")))
once=dispatch["once"]
region=dispatch["region"]
P0=dispatch["P0"]

def candidate(source):
    files=dispatch["candidate"](source)
    h,s=writer["bounded_writer"]((source/"UsbSerial.h").read_text(),(source/"UsbSerial.cpp").read_text())
    files["UsbSerial.h"]=h;files["UsbSerial.cpp"]=s
    c=(source/"Commands.cpp").read_text()
    c=once(c,"void Commands::commandLoop() {",
        """#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
static bool serviceTransportFailure() {
    if(!Printer::printJob.emergencyLocked() && UsbSerial::blockingTxFailed())
        Printer::stopForTransportFault();
    return Printer::printJob.emergencyLocked();
}
#endif

void Commands::commandLoop() {""")
    # Use common transition at all existing terminal guard boundaries
    # inside outer dispatch and periodic service only; don't change other
    # P0 functions or the terminal thermal service itself.
    part=region(c,"void Commands::commandLoop() {","bool Commands::waitForMotion")
    revised=part.replace("if(Printer::printJob.emergencyLocked())","if(serviceTransportFailure())")
    revised=once(revised,"    GCodeInput::readFromSerial();\n\n    GCode* code",
        """    GCodeInput::readFromSerial();
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    serviceTransportFailure();
#endif

    GCode* code""")
    revised=once(revised,"        GCodeInput::keepAlive(state);\n    }",
        """        GCodeInput::keepAlive(state);
    }
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(serviceTransportFailure()){serviceThermalEmergency();return;}
#endif""")
    revised=once(revised,"        recalculateZCompensation();\n    }",
        """        recalculateZCompensation();
    }
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(serviceTransportFailure()){serviceThermalEmergency();return;}
#endif""")
    revised=once(revised,"            Commands::printTemperatures(); // Selfcontrolling timediff\n        uid.slowAction();",
        """            Commands::printTemperatures(); // Selfcontrolling timediff
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        if(serviceTransportFailure()){uid.refreshEmergencyPage();return;}
#endif
        uid.slowAction();""")
    revised=once(revised,"    Printer::strainGauge.service(HAL::timeInMilliseconds());",
        """    Printer::strainGauge.service(HAL::timeInMilliseconds());
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(serviceTransportFailure()){serviceThermalEmergency();return;}
#endif""")
    revised=once(revised,"    handleScanWorkTasks(scanTickDue);\n} // checkForPeriodicalActions",
        """#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(serviceTransportFailure()){serviceThermalEmergency();return;}
#endif
    handleScanWorkTasks(scanTickDue);
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    serviceTransportFailure();
#endif
} // checkForPeriodicalActions""")
    c=once(c,part,revised);files["Commands.cpp"]=c
    g=files["GCodeInput.cpp"]
    guard="if(Printer::printJob.transportEmergency())return;"
    assert g.count(guard)==3
    g=g.replace(guard,
        "if(Printer::printJob.transportEmergency() ||\n"
        "       (UsbSerial::blockingTxFailed() && !Printer::printJob.thermalEmergency()))return;")
    g=once(g,"    // Consume at most one RX ring's usable capacity per foreground visit.",
        """#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(UsbSerial::blockingTxFailed() && !Printer::printJob.thermalEmergency())return;
#endif
    // Consume at most one RX ring's usable capacity per foreground visit.""")
    g=once(g,"            if(Printer::printJob.emergencyLocked())return;\n#endif\n        }\n    } while (--budget",
        "            if(Printer::printJob.emergencyLocked() || UsbSerial::blockingTxFailed())return;\n#endif\n        }\n    } while (--budget")
    files["GCodeInput.cpp"]=g
    return files

PRELUDE=r'''
#include <cassert>
#include <cstdio>
#include <cstdint>
#include <string>
#define RF_THERMAL_EMERGENCY_EXPERIMENT 1
#define RF_EEPROM_ZONE_EXPERIMENT 1
#include "PrintJob.h"
using millis_t=uint32_t;
enum FirmwareState {NotBusy,Processing,Paused,WaitHeater};
enum Point {None,Read,Execute,KeepAlive,CompUi,Comp,Temperature,PrintTemperature,TempUi,Dms,Features,Scan};
static Point inject;
static bool txFailed,queued,temperatureDue,compDue,featureDue;
static unsigned stops,ordinary,executes,retired,thermalVisits,panelVisits,historyVisits,slowVisits;
static void visit(Point p){
    assert(!txFailed);++ordinary;
    if(inject==p)txFailed=true;
}
struct UsbSerial {static bool blockingTxFailed(){return txFailed;}};
struct Printer {
    static PrintJob printJob;
    static void stopForTransportFault(){assert(txFailed && printJob.beginTransportEmergency());++stops;}
    struct Pause {bool isPaused(){return false;}};static Pause pause;
    struct DmsService {void service(uint32_t){visit(Dms);}};static DmsService strainGauge;
};
PrintJob Printer::printJob;Printer::Pause Printer::pause;Printer::DmsService Printer::strainGauge;
void PrintJob::servicePendingOutput(uint32_t){assert(emergencyLocked());}
struct GCode{} code;
namespace GCodeInput {
void readFromSerial(){if(Printer::printJob.emergencyLocked())return;visit(Read);}
GCode* peekCurrentCommand(){return queued?&code:nullptr;}
void popCurrentCommand(){if(queued){queued=false;++retired;}}
void keepAlive(FirmwareState){visit(KeepAlive);}
}
namespace HAL {
uint32_t timeInMilliseconds(){return 10;}
bool takeScanTick(){return true;}
void tellWatchdogOk(){assert(!txFailed);}
bool takeCompensationTick(){bool r=compDue;compDue=false;return r;}
bool temperatureTickPending(){return temperatureDue;}
bool takeTemperatureTick(){bool r=temperatureDue;temperatureDue=false;return r;}
bool takeFeatureTick(){bool r=featureDue;featureDue=false;return r;}
}
struct Extruder {
    static void manageTemperatures(){
        ++thermalVisits;
        if(!Printer::printJob.emergencyLocked())visit(Temperature);
    }
};
namespace Lighting {void serviceAlarm(uint32_t){}}
struct Display {
    uint32_t lastButtonTime(){return 0;}
    void slowAction(){++slowVisits;visit(compDue?CompUi:(thermalVisits?TempUi:CompUi));}
    void refreshEmergencyPage(){assert(Printer::printJob.emergencyLocked());++panelVisits;}
} uid;
void recalculateZCompensation(){visit(Comp);}
void loopFeatures(){visit(Features);}
void handleScanWorkTasks(bool){visit(Scan);}
namespace DiagnosticHistory{void service(){++historyVisits;}}
namespace Commands {
void commandLoop();
void checkForPeriodicalActions(FirmwareState);
void executeGCode(GCode*){++executes;visit(Execute);}
void printTemperatures(){visit(PrintTemperature);}
}
'''
SCENARIOS=r'''
int main(int argc,char** argv){
    assert(argc==2);std::string mode=argv[1];
    if(mode=="before-read")txFailed=true;
    else if(mode=="read")inject=Read;
    else if(mode=="execute")inject=Execute;
    else if(mode=="keepalive")inject=KeepAlive;
    else if(mode=="comp-ui"){inject=CompUi;compDue=true;}
    else if(mode=="comp"){inject=Comp;compDue=true;}
    else if(mode=="temperature"){inject=Temperature;temperatureDue=true;}
    else if(mode=="print-temperature"){inject=PrintTemperature;temperatureDue=true;}
    else if(mode=="temp-ui"){inject=TempUi;temperatureDue=true;}
    else if(mode=="dms")inject=Dms;
    else if(mode=="features"){inject=Features;featureDue=true;}
    else if(mode=="scan")inject=Scan;
    else assert(false);
    queued=true;
    if(mode=="before-read"||mode=="read"||mode=="execute") {
        Commands::commandLoop();
        assert(executes==(mode=="execute"?1u:0u) && retired==1);
    }else Commands::checkForPeriodicalActions(mode=="print-temperature"?WaitHeater:Processing);
    assert(stops==1 && txFailed && Printer::printJob.transportEmergency());
    const unsigned old=ordinary,thermal=thermalVisits;
    for(unsigned n=0;n<100;++n){temperatureDue=true;Commands::commandLoop();}
    assert(stops==1 && ordinary==old && thermalVisits==thermal+100 && !queued && retired==1);
    std::printf("TX trigger %s: one terminal transition, no subsequent ordinary work, thermal service continues\n",argv[1]);
}
'''

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",type=Path,required=True)
    parser.add_argument("--out",type=Path,required=True)
    args=parser.parse_args()
    assert subprocess.check_output(["git","-C",str(args.source),"rev-parse","HEAD"],text=True).strip()==P0
    args.out.mkdir(parents=True,exist_ok=True)
    files=candidate(args.source/"Repetier")
    for name,s in files.items():(args.out/name).write_text(s)
    code=region(files["Commands.cpp"],"#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\nstatic bool serviceTransportFailure()","bool Commands::waitForMotion")
    test=PRELUDE+code+SCENARIOS
    (args.out/"trigger.cpp").write_text(test)
    with tempfile.TemporaryDirectory() as d:
        exe=Path(d)/"trigger"
        subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror",
            "-fsanitize=undefined","-fno-sanitize-recover=all",str(args.out/"trigger.cpp"),"-o",str(exe)],check=True)
        for mode in ("before-read","read","execute","keepalive","comp-ui","comp","temperature","print-temperature","temp-ui","dms","features","scan"):
            print(subprocess.check_output([str(exe),mode],text=True),end="")
if __name__=="__main__":
    main()

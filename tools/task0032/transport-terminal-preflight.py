#!/usr/bin/env python3
"""Task-private terminal transport transition based on exact P0 sources.

No shipping edits. Generate candidate bodies in a disposable tree and
exercise real PrintJob/Printer/TemperatureControl code with instrumented
actuator stubs. The scheduler/receiver/panel integration is separate.
"""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import tempfile

P0 = "962e9b6b5152e2c3972896f5a93981bdaed61422"

def once(s, old, new):
    assert s.count(old)==1,old
    return s.replace(old,new)

def region(s, start, end):
    a=s.index(start)
    return s[a:s.index(end,a)]

def candidate(source):
    h=(source/"PrintJob.h").read_text()
    h=once(h,"|| phase == Phase::ThermalEmergency","|| phase == Phase::ThermalEmergency || phase == Phase::TransportEmergency")
    h=once(h,"    bool emergencyLocked() const { return phase_ == Phase::ThermalEmergency; }",
        """    // Common actuator/admission guard, not a thermal diagnostic predicate.
    bool emergencyLocked() const {
        const Phase phase=phase_;
        return phase==Phase::ThermalEmergency || phase==Phase::TransportEmergency;
    }
    bool thermalEmergency() const { return phase_==Phase::ThermalEmergency; }
    bool transportEmergency() const { return phase_==Phase::TransportEmergency; }
    // Caller holds interrupt lock, publish terminal admission before power revoke.
    bool beginTransportEmergency() {
        if(emergencyLocked())return false;
        phase_=Phase::TransportEmergency;
        return true;
    }""")
    h=once(h,"        if(emergencyLocked()) return false;\n        phase_=Phase::ThermalEmergency;\n        return true;",
        """        // Promote a real subsequent thermal fault without repeating motion
        // cleanup. The thermal caller already revoked duty and latched evidence.
        if(thermalEmergency())return false;
        const bool first=!emergencyLocked();
        phase_=Phase::ThermalEmergency;
        return first;""")
    h=once(h,"        , ThermalEmergency","        , ThermalEmergency, TransportEmergency")
    j=(source/"PrintJob.cpp").read_text()
    j=once(j,"    case Phase::ThermalEmergency:","    case Phase::TransportEmergency:\n    case Phase::ThermalEmergency:")
    ph=(source/"Printer.h").read_text()
    ph=once(ph,"    static bool heatingInhibited() {","    static bool heatingInhibited() {\n#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\n        if(printJob.emergencyLocked())return true;\n#endif")
    ph=once(ph,"    static void stopForThermalFault();",
        """    static void stopForThermalFault();
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    static void stopForTransportFault();
private:
    static void stopTerminalMotionLocked();
    static void revokeHeaterOutputsLocked();
    static void clearHeaterTargets();
public:
#endif""")
    t=(source/"TemperatureControl.cpp").read_text()
    # Extract the exact existing power-revoke loop and target clear loop.
    revoke=region(t,"        extrusionOutput.inhibit(ExtrusionOutput::Inhibit::SafetyLock);","\n    }\n    stopAllHeaters();")
    t=once(t,revoke,"        revokeHeaterOutputsLocked();")
    # Keep common heater helper available for the legacy caller too.
    ph=once(ph,"    static void revokeHeaterOutputsLocked();\n    static void clearHeaterTargets();\npublic:\n#endif",
        "public:\n#endif\nprivate:\n    static void revokeHeaterOutputsLocked();\n    static void clearHeaterTargets();\npublic:")
    t=once(t,"static void stopAllHeaters() {","void Printer::clearHeaterTargets() {")
    # No other stopAllHeaters callers may accidentally disappear.
    assert t.count("stopAllHeaters();")==1
    t=once(t,"    stopAllHeaters();","    clearHeaterTargets();")
    t=once(t,"void Printer::latchOutputInhibit(bool thermalFault) {",
        "void Printer::revokeHeaterOutputsLocked() {\n"+revoke+"\n}\n\nvoid Printer::latchOutputInhibit(bool thermalFault) {")
    p=(source/"Printer.cpp").read_text()
    motion=region(p,"    blockAll=1;","\n    noInts.unprotect();")
    p=once(p,motion,"    stopTerminalMotionLocked();")
    start="void Printer::stopForThermalFault() {"
    p=once(p,start,
        """#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
void Printer::stopTerminalMotionLocked() {
"""+motion+"""
}
void Printer::stopForTransportFault() {
    InterruptProtectedBlock noInts;
    if(!printJob.beginTransportEmergency())return;
    revokeHeaterOutputsLocked();
    stopTerminalMotionLocked();
    noInts.unprotect();
    clearHeaterTargets();
    discardScans();
}
#endif

"""+start)
    return {"PrintJob.h":h,"PrintJob.cpp":j,"Printer.h":ph,"Printer.cpp":p,"TemperatureControl.cpp":t}

PRELUDE=r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <vector>
#define RF_THERMAL_EMERGENCY_EXPERIMENT 1
#define PSTR(x) x
#define NUM_TEMPERATURE_LOOPS HEATERS
#define PRINTER_FLAG0_TEMPSENSOR_DEFECT 1
#define PRINTER_FLAG0_LEGACY_SIMULATION_REJECTED 2
#include "PrintJob.h"
static unsigned irqDepth;
enum Effect {PowerOff, SafetyInhibit, MotionLock, IdleCancel, ExtrusionInhibit,
             PauseCancel, DirectAbort, QueueDiscard, TargetClear, ScanDiscard,
             NormalEffect};
static std::vector<Effect> effects;
struct InterruptProtectedBlock {
    bool owns=true;
    InterruptProtectedBlock(){++irqDepth;}
    void unprotect(){assert(owns && irqDepth);--irqDepth;owns=false;}
    ~InterruptProtectedBlock(){if(owns)unprotect();}
};
struct GCodeInput {static void readFromSerial(){effects.push_back(NormalEffect);}};
struct PrintLine {
    static bool hasLines(){return false;}
    static bool directMoveActive(){return false;}
    static void abortDirectMoveForEmergency(){assert(irqDepth);effects.push_back(DirectAbort);}
    static void discardQueuedMoves(){assert(irqDepth);effects.push_back(QueueDiscard);}
};
struct Extruder {
    static void setHeatedBedTemperature(int){effects.push_back(NormalEffect);}
    static void setTemperatureForAllExtruders(int){effects.push_back(NormalEffect);}
};
static void discardScans(){assert(!irqDepth);effects.push_back(ScanDiscard);}
struct Com {static void printFLN(const char*){effects.push_back(NormalEffect);}};
struct ExtrusionOutput {
    enum class Inhibit {PrintStop, SafetyLock, Emergency};
    void allow(Inhibit){effects.push_back(NormalEffect);}
    void inhibit(Inhibit cause){
        assert(irqDepth);
        effects.push_back(cause==Inhibit::SafetyLock?SafetyInhibit:ExtrusionInhibit);
    }
};
struct Idle {void cancel(){assert(irqDepth);effects.push_back(IdleCancel);}};
struct Pause {void cancel(){assert(irqDepth);effects.push_back(PauseCancel);}};
struct Printer {
    static PrintJob printJob;
    static ExtrusionOutput extrusionOutput;
    static Idle idleStatus;
    static Pause pause;
    static unsigned flag0;
    static volatile char blockAll;
    static void lockMovement(){assert(irqDepth);effects.push_back(MotionLock);}
    static void stopForThermalFault();
    static void stopForTransportFault();
    static void stopTerminalMotionLocked();
    static void revokeHeaterOutputsLocked();
    static void clearHeaterTargets();
    HEATING_INHIBIT
};
ExtrusionOutput Printer::extrusionOutput;
Idle Printer::idleStatus;
Pause Printer::pause;
unsigned Printer::flag0;
volatile char Printer::blockAll;
static void outputObject(bool){effects.push_back(NormalEffect);}
namespace HAL {
void setHeaterDuty(uint8_t channel,uint8_t duty){
    assert(irqDepth && !duty && Printer::heatingInhibited());
    assert(channel<NUM_TEMPERATURE_LOOPS);
    effects.push_back(PowerOff);
}
}
struct Controller {
    uint8_t pwmIndex;
    void clearTargetTemperature(){
        assert(!irqDepth && Printer::heatingInhibited());
        effects.push_back(TargetClear);
    }
};
static Controller controllers[NUM_TEMPERATURE_LOOPS];
static Controller* tempController[NUM_TEMPERATURE_LOOPS];
'''
SCENARIOS=r'''
static void reset(unsigned phase,uint32_t start) {
    Printer::printJob=PrintJob();
    Printer::flag0=0;Printer::blockAll=0;
    assert(!irqDepth);
    effects.clear();
    auto& j=Printer::printJob;
    if(phase==1 || phase>=3)j.admitPrintMotion();
    if(phase==2)assert(j.beginFilamentService());
    if(phase>=3)assert(j.beginStop(start));
    if(phase==4)j.service(start+2001);
    effects.clear();
}
static void assertTerminal(bool thermal) {
    auto& j=Printer::printJob;
    assert(j.emergencyLocked() && j.commandsBlocked() && !j.isBusy() && !j.isIdle());
    assert(j.thermalEmergency()==thermal && j.transportEmergency()!=thermal);
    assert(Printer::heatingInhibited() && Printer::blockAll && !irqDepth);
}
static void assertNoAdmission(uint32_t now) {
    auto& j=Printer::printJob;
    auto old=effects;
    for(unsigned n=0;n<100;++n) {
        j.admitPrintMotion();j.finishPrintMotion();
        assert(!j.beginFilamentService());j.finishFilamentService();
        assert(!j.beginStop(now+n));
        j.service(now+n*1000);j.servicePendingOutput(now+n*1000);
        Printer::stopForTransportFault();
        assert(effects==old);
    }
}
int main() {
    for(unsigned i=0;i<NUM_TEMPERATURE_LOOPS;++i){
        controllers[i].pwmIndex=i;tempController[i]=&controllers[i];
    }
    unsigned count=0;
    for(unsigned phase=0;phase<5;++phase)
    for(uint32_t start:{0u,0xfffffff0u})
    for(unsigned order=0;order<3;++order) {
        reset(phase,start);
        if(order!=1){
            Printer::stopForTransportFault();
            assertTerminal(false);
            assert(Printer::flag0==0); // no fabricated sensor/legacy cause
            std::vector<Effect> expected{SafetyInhibit};
            expected.insert(expected.end(),NUM_TEMPERATURE_LOOPS,PowerOff);
            for(auto e:{MotionLock,IdleCancel,ExtrusionInhibit,PauseCancel,DirectAbort,QueueDiscard})expected.push_back(e);
            expected.insert(expected.end(),NUM_TEMPERATURE_LOOPS,TargetClear);
            expected.push_back(ScanDiscard);
            assert(effects==expected);
            assertNoAdmission(start);
        }
        if(order!=0){
            effects.clear();
            Printer::latchOutputInhibit(true);
            Printer::stopForThermalFault();
            assertTerminal(true);
            assert(Printer::flag0==PRINTER_FLAG0_TEMPSENSOR_DEFECT);
            std::vector<Effect> expected{SafetyInhibit};
            expected.insert(expected.end(),NUM_TEMPERATURE_LOOPS,PowerOff);
            expected.insert(expected.end(),NUM_TEMPERATURE_LOOPS,TargetClear);
            if(order==1)for(auto e:{MotionLock,IdleCancel,ExtrusionInhibit,PauseCancel,DirectAbort,QueueDiscard,ScanDiscard})expected.push_back(e);
            assert(effects==expected); // promotion does not repeat cleanup
            assertNoAdmission(start);
            auto old=effects;
            Printer::stopForThermalFault();assert(effects==old);
        }
        ++count;
    }
    std::printf("Terminal transport: %u initial phase/time/cause-order cases, %u heater channels, 4000 blocked transition visits\n",count,NUM_TEMPERATURE_LOOPS);
}
'''

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",type=Path,required=True)
    parser.add_argument("--out",type=Path,required=True)
    args=parser.parse_args()
    assert subprocess.check_output(["git","-C",str(args.source),"rev-parse","HEAD"],text=True).strip()==P0
    args.out.mkdir(parents=True,exist_ok=True)
    source=args.source/"Repetier"
    files=candidate(source)
    for name,s in files.items():(args.out/name).write_text(s)
    heat=region(files["Printer.h"],"    static void latchOutputInhibit(bool thermalFault);","    static INLINE bool isManualMoveMode")
    body=region(files["Printer.cpp"],"#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\nvoid Printer::stopTerminalMotionLocked()","void Printer::stopPrint()")
    temp=region(files["TemperatureControl.cpp"],"void Printer::clearHeaterTargets()","#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\nnamespace ThermalEmergency")
    job=files["PrintJob.cpp"][files["PrintJob.cpp"].index("PrintJob Printer::printJob;"):]
    test=PRELUDE.replace("HEATING_INHIBIT",heat)+temp+body+job+SCENARIOS
    (args.out/"terminal-replay.cpp").write_text(test)
    for heaters in (2,3):
        for mode,flags in [("O2",["-O2"]),("ubsan",["-O1","-fsanitize=undefined","-fno-sanitize-recover=all"])]:
            with tempfile.TemporaryDirectory() as d:
                exe=Path(d)/"test"
                subprocess.run(["c++","-std=c++11","-Wall","-Wextra","-Werror",
                    *flags,f"-DHEATERS={heaters}",str(args.out/"terminal-replay.cpp"),"-o",str(exe)],check=True)
                output=subprocess.check_output([str(exe)],text=True)
                (args.out/f"{heaters}-{mode}.log").write_text(output)
                print(output,end="")
    report={"p0":P0,"source_sha256":{n:hashlib.sha256((source/n).read_bytes()).hexdigest() for n in files},
            "candidate_sha256":{n:hashlib.sha256((args.out/n).read_bytes()).hexdigest() for n in files},
            "limits":"actual extracted transition and cleanup; simulated actuators, no full scheduler/UI/receiver or physical timing"}
    (args.out/"summary.json").write_text(json.dumps(report,indent=2)+"\n")

if __name__=="__main__":
    main()

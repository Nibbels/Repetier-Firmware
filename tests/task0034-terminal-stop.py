#!/usr/bin/env python3
"""Actual terminal motion-stop body: ordering, idempotence and saved I state."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
src=(root/"Repetier/Printer.cpp").read_text()
body=src[src.index("void Printer::stopForThermalFault()"):src.index("void Printer::stopPrint()")]
assert "Printer::stopForThermalFault();" in (root/"Repetier/HAL.cpp").read_text()
assert "Printer::stopPrint();" not in (root/"Repetier/TemperatureControl.cpp").read_text()

harness=r'''
#include <cassert>
#include <cstdint>
#include <initializer_list>
#include "PrintJob.h"

static bool irq, entryI, heatersOff, motorsEnabled;
static unsigned sequence;
struct InterruptProtectedBlock {
    bool saved=irq;
    InterruptProtectedBlock(){irq=false;}
    ~InterruptProtectedBlock(){irq=saved;}
    void unprotect(){irq=saved;}
};
static void stage(unsigned n) {
    assert(!irq && heatersOff && motorsEnabled && sequence+1==n);
    sequence=n;
}
namespace PrintLine {
    void abortDirectMoveForEmergency(){stage(5);}
    void discardQueuedMoves(){stage(6);}
}
namespace Printer {
    PrintJob printJob;
    bool blockAll=false, movementLocked=false;
    void lockMovement(){assert(blockAll);stage(1);movementLocked=true;}
    struct Idle {void cancel(){stage(2);}} idleStatus;
    struct ExtrusionOutput {
        enum class Inhibit { Emergency };
        void inhibit(Inhibit){stage(3);}
    } extrusionOutput;
    struct Pause {void cancel(){stage(4);}} pause;
    void stopForThermalFault();
}
using Printer::ExtrusionOutput;
static void discardScans(){
    assert(irq==entryI && sequence==6 && heatersOff && motorsEnabled);
    assert(Printer::printJob.emergencyLocked());
    sequence=7;
}
''' + body + r'''
int main(){
    for(bool enabled:{false,true})for(unsigned phase:{0u,1u,2u,3u}){
        Printer::printJob=PrintJob{};
        if(phase==1||phase==3)Printer::printJob.admitPrintMotion();
        if(phase==2)Printer::printJob.beginFilamentService();
        if(phase==3)assert(Printer::printJob.beginStop(1));
        Printer::blockAll=Printer::movementLocked=false;
        irq=entryI=enabled;heatersOff=motorsEnabled=true;sequence=0;
        Printer::stopForThermalFault();
        assert(sequence==7 && irq==enabled && motorsEnabled);
        assert(Printer::blockAll && Printer::movementLocked);
        assert(Printer::printJob.emergencyLocked() && Printer::printJob.commandsBlocked());
        Printer::stopForThermalFault();
        assert(sequence==7 && irq==enabled && motorsEnabled);
    }
}
'''
with tempfile.TemporaryDirectory() as d:
    cpp=Path(d)/"test.cpp";exe=Path(d)/"test"
    cpp.write_text(harness)
    subprocess.run(["g++","-std=c++11","-Wall","-Wextra","-Werror",
                    "-I",str(root/"Repetier"),str(cpp),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print("Task0034 terminal motion stop: four phases, both entry I states, repeat call PASS")

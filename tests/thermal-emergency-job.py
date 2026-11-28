#!/usr/bin/env python3
"""Replay the actual PrintJob terminal transition and service."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
src=(root/"Repetier/PrintJob.cpp").read_text()
body=src[src.index("void PrintJob::service("):]
code=r"""
#include <cassert>
#include <cstdint>
#include <cstdio>
#define PSTR(x) x
#include "PrintJob.h"
static unsigned effects;
struct InterruptProtectedBlock {};
struct GCodeInput {static void readFromSerial(){++effects;}};
struct PrintLine {
    static bool hasLines(){++effects;return false;}
    static bool directMoveActive(){++effects;return false;}
};
struct Extruder {
    static void setHeatedBedTemperature(int){++effects;}
    static void setTemperatureForAllExtruders(int){++effects;}
};
static void discardScans(){++effects;}
struct Com {static void printFLN(const char*){++effects;}};
struct ExtrusionOutput {
    enum class Inhibit {PrintStop};
    void allow(Inhibit){++effects;}
};
struct Printer {static ExtrusionOutput extrusionOutput; static bool blockAll;};
bool Printer::blockAll=false;
ExtrusionOutput Printer::extrusionOutput;
static void outputObject(bool){++effects;}
"""
end=r"""
int main(){
    unsigned count=0;
    for(unsigned phase=0;phase<5;++phase)
    for(uint32_t start:{0u,0xfffffff0u}){
        PrintJob j;
        if(phase==1||phase>=3)j.admitPrintMotion();
        if(phase==2)j.beginFilamentService();
        if(phase>=3)assert(j.beginStop(start));
        if(phase==4)j.service(start+2001);
        assert(j.beginThermalEmergency());
        effects=0;
        for(unsigned n=0;n<100;++n){
            assert(j.emergencyLocked()&&j.commandsBlocked()&&!j.isPrinting());
            assert(!j.beginThermalEmergency()&&!j.beginStop(start+n));
            j.admitPrintMotion();j.finishPrintMotion();
            j.beginFilamentService();j.finishFilamentService();
            j.service(start+n*1000);
            j.servicePendingOutput(start+n*1000);
            assert(effects==0);
        }
        ++count;
    }
    // A mechanical stop must not release the print-stop inhibit or start
    // outputObject, even when the previous job is already waiting.
    PrintJob mechanical;
    mechanical.admitPrintMotion();
    assert(mechanical.beginStop(0));
    mechanical.service(2001);
    Printer::blockAll=true;
    effects=0;
    mechanical.servicePendingOutput(5000);
    assert(effects==0 && mechanical.commandsBlocked());
    std::printf("Thermal job terminal: %u initial phase/time cases, 1000 attempted transitions; no output, no side effects\n",count);
}
"""
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"job.cpp";exe=Path(d)/"job"
    p.write_text("#include <initializer_list>\n"+code+body+end)
    subprocess.run(["c++","-std=c++11","-O2","-I",str(root/"Repetier"),str(p),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

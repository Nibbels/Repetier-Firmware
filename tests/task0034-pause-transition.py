#!/usr/bin/env python3
"""Replay real pause dispatch across reentry, cancellation and pending requests."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
src=(root/"Repetier/PrintPause.cpp").read_text()
header=(root/"Repetier/PrintPause.h").read_text()
assert "bool servicingMotion_ = false;" in header
cancel=src[src.index("void PrintPause::cancel()"):src.index("void PrintPause::serviceMotion()")]
assert "servicingMotion_" not in cancel
start=src.index("void PrintPause::serviceMotion()")
body=src[start:src.index("void PrintPause::serviceStandby(",start)]
source=r'''
#include <cassert>
#include <cstdint>
namespace PrintLine {
    bool active=false;
    bool directMoveActive(){return active;}
}
namespace Printer { bool blockAll=false; }
static bool scanActive=false;
static bool isAnyScanRunning(){return scanActive;}
struct PrintPause {
    enum class Phase : uint8_t { Running,RetractRequested,RetractAndParkRequested,ParkRequested,ReturnRequested,Parking };
    enum class Destination : uint8_t { InPlace,Park };
    Phase phase_=Phase::Running;
    bool servicingMotion_=false;
    bool isPaused() const {return phase_!=Phase::Running;}
    int calls=0, last=0;
    bool inject=true;
    void serviceMotion();
    void dispatchPendingMotion();
    void transition(int n){
        assert(servicingMotion_);
        ++calls;last=n;
        if(inject){
            inject=false;
            phase_=Phase::Parking;
            serviceMotion(); // ordinary nested periodic pump
            assert(calls==1);
            // Simulate cancellation followed by another admitted request
            // while the prior transition still owns its wait frame.
            phase_=Phase::Running;
            phase_=Phase::ReturnRequested;
            serviceMotion();
            assert(calls==1 && servicingMotion_);
        }else phase_=Phase::Running;
    }
    void retract(Destination d){transition(d==Destination::InPlace?1:2);}
    void park(){transition(3);}
    void returnToPrint(){transition(4);}
};
''' + body + r'''
int main(){
    using P=PrintPause::Phase;
    for(int i=1;i<=4;++i){
        PrintPause p;
        p.phase_=i==1?P::RetractRequested:i==2?P::RetractAndParkRequested:i==3?P::ParkRequested:P::ReturnRequested;
        PrintLine::active=true;p.serviceMotion();assert(!p.calls&&!p.servicingMotion_);
        PrintLine::active=false;Printer::blockAll=true;p.serviceMotion();assert(!p.calls&&!p.servicingMotion_);
        Printer::blockAll=false;
        scanActive=true;p.serviceMotion();assert(!p.calls&&!p.servicingMotion_);
        scanActive=false;p.serviceMotion();
        assert(p.calls==1&&p.last==i&&!p.servicingMotion_&&p.phase_==P::ReturnRequested);
        p.serviceMotion();assert(p.calls==2&&p.last==4&&!p.servicingMotion_);
        p.serviceMotion();assert(p.calls==2);
    }
}
'''
with tempfile.TemporaryDirectory() as d:
    path=Path(d)/"test.cpp";exe=Path(d)/"test"
    path.write_text(source)
    subprocess.run(["g++","-std=c++11","-Wall","-Wextra","-Werror",str(path),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print("Task0034 pause transition exclusion: four dispatch paths, cancellation and reentry PASS")

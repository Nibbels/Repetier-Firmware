#!/usr/bin/env python3
"""Replay actual candidate scan presentation and shared panel homing admission."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
firmware=root/"Repetier"
probe=(firmware/"ScanProbe.cpp").read_text()
ui=(firmware/"UiActions.cpp").read_text()
render=probe[probe.index("void showAbortScanReason("):probe.index("\nbool moveZ(",probe.index("void showAbortScanReason("))]
home=ui[ui.index("    case UI_ACTION_HOME_ALL:"):ui.index("    case UI_ACTION_DEBUG_ECHO:")]
assert render.count("showError(")==1
assert home.count("showError(")==1 and home.count("Printer::homeAxis(")==1
# Compile the exact production function/dispatch body. Keep independent
# golden texts for old cases to detect unintended presentation drift.
source=r"""
#include "CalibrationScan.h"
#include <cassert>
#include <cstring>
#include <cstdio>
#define PSTR(x) (x)
#define UI_TEXT_FAIL "Fail"
#define UI_TEXT_SCAN_ABORTED "Scan aborted"
static unsigned messages;
static const char* head;
static const char* line1;
static const char* line2;
void showError(const void* a,const void* b=nullptr,const void* c=nullptr){
    ++messages;head=(const char*)a;line1=(const char*)b;line2=(const char*)c;
}
"""+render+r"""
enum {UI_ACTION_HOME_ALL=23,UI_ACTION_HOME_X=97,UI_ACTION_HOME_Y=54,UI_ACTION_HOME_Z=18};
const char* ui_text_home="Home";
const char* ui_text_operation_denied="Operation denied";
bool busy,debug,allowed;
unsigned serial,admission,exits,homes,axes;
struct PrintLine { static bool hasLines(){return busy;} };
struct Printer {
    static bool debugErrors(){return debug;}
    static void homeAxis(bool x,bool y,bool z){++homes;axes=unsigned(x)|unsigned(y)*2|unsigned(z)*4;}
};
struct Com {
    static constexpr int tPrintingIsInProcessError=1;
    static void printFLN(int n){assert(n==1);++serial;}
};
bool isHomingAllowed(void* p,int mode){assert(!p&&mode==1);++admission;return allowed;}
void exitmenu(){++exits;}
void dispatch(int action){switch(action){
"""+home+r"""
default:assert(false);
}}
void check(ScanAbort reason,const char* first,const char* second=nullptr){
    unsigned before=messages;
    showAbortScanReason("HBS",reason);
    assert(messages==before+1&&std::strcmp(head,"HBS")==0);
    assert(std::strcmp(line1,first)==0&&std::strlen(line1)<=16);
    if(second)assert(line2&&std::strcmp(line2,second)==0&&std::strlen(line2)<=16);
    else assert(!line2);
}
int main(){
    check(ScanAbort::None,"Fail");
    check(ScanAbort::Canceled,"Scan aborted");
    check(ScanAbort::StartPressure,"Bad start force");
    check(ScanAbort::IdlePressure,"Bad idle force");
    check(ScanAbort::AveragePressure,"Bad force signal");
    check(ScanAbort::AboveStart,"Bug SOS");
    check(ScanAbort::StepDeltaOutOfRange,"Leftovers, snot","incrustation");
    check(ScanAbort::MatrixDimensions,"Bug MDO");
    check(ScanAbort::UnevenBed,"Uneven bed, dirt","hole, bad ground");
    check(ScanAbort::MinEndstop,"Min Endstop");
    check(ScanAbort::MaxEndstop,"Max Endstop");
    check(ScanAbort::MaxCompensation,"Reached limit","max compensation");
    check(ScanAbort::EndstopProtection,"Reached endstop","protection limit");
    check(ScanAbort::MissingMatrix,"Missing or","Invalid Matrix");
    check(ScanAbort::MatrixOverflow,"Correction would","overflow matrix");
    check(ScanAbort::MatrixHeight,"Badly wrong","Z-Screw/Z-Start");
    check(ScanAbort::DmsUnavailable,"DMS read failed");
    check(ScanAbort::DmsOverload,"DMS overload");
    check(ScanAbort::HeaterTargetRejected,"Bed target","not admitted");
    check(ScanAbort::HeaterTargetChanged,"Bed target","changed");
    check(ScanAbort(255),"Fail");
    const int actions[]={UI_ACTION_HOME_ALL,UI_ACTION_HOME_X,UI_ACTION_HOME_Y,UI_ACTION_HOME_Z};
    const unsigned masks[]={7,1,2,4};
    unsigned cases=0;
    for(unsigned a=0;a<4;++a)
        for(unsigned b=0;b<2;++b)for(unsigned d=0;d<2;++d)for(unsigned h=0;h<2;++h){
            busy=b;debug=d;allowed=h;
            messages=serial=admission=exits=homes=axes=0;
            dispatch(actions[a]);
            assert(messages==b&&serial==unsigned(b&&(a==0||d)));
            assert(admission==unsigned(!b));
            assert(exits==unsigned(!b&&h)&&homes==exits);
            assert(axes==(!b&&h?masks[a]:0));
            if(b)assert(std::strcmp(head,"Home")==0&&std::strcmp(line1,"Operation denied")==0);
            ++cases;
        }
    std::printf("Error consolidation: 21 exact scan displays, %u homing admission/axis cases PASS\n",cases);
}
"""
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"error.cpp";exe=Path(d)/"error"
    p.write_text(source)
    subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror",
                    "-I",str(firmware),str(p),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

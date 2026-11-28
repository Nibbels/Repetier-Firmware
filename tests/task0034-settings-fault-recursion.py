#!/usr/bin/env python3
"""Exercise actual settings-failure latch and diagnostic read entry.
A failed usage read during LCD rendering must not recurse through refreshPage.
"""
from pathlib import Path
import subprocess
import tempfile
fw=Path(__file__).resolve().parents[1]/"Repetier"
source=(fw/"EepromZoneRuntime.cpp").read_text()
def extract(a,b):
    start=source.index(a)
    return source[start:source.index(b,start)]
actual=extract("void failSettings(Result result)","bool persist(")
actual+=extract("bool readBlockChecked(","bool driveOnly(")
pre=r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
enum class Result {Ok, Busy, TransportFault, NeedsRecovery};
bool lastOk=true,settingsFaultLatched=false,loaded=true,outputsAdmitted=true;
unsigned revokes=0,errors=0,notices=0,refreshes=0,reads=0;
namespace Printer {bool blockAll=false;}
void failSettings(Result);
void revokeOutputs(){
    assert(settingsFaultLatched&&Printer::blockAll&&!lastOk);
    ++revokes;loaded=false;outputsAdmitted=false;
    // Even a nested notification cannot repeat revocation or re-admit.
    failSettings(Result::Busy);
}
#define PSTR(x) x
namespace Com {
void printErrorFLN(const char*){assert(!outputsAdmitted);++errors;}
void printFLN(const char*,int){assert(!outputsAdmitted);++notices;}
}
void status(const char*){assert(!outputsAdmitted);}
#define UI_STATUS(x) status(x)
#define UI_STATUS_UPD(x) do { status(x); ++refreshes; failSettings(Result::Busy); } while(0)
Result next=Result::Ok;
uint8_t length(uint8_t){return 8;}
struct Session {
    Result read(uint8_t,uint8_t*,uint8_t,uint32_t&){++reads;return next;}
};
Session session(){return Session{};}
'''
main=r'''
int main(){
    uint8_t bytes[8]{};uint32_t alerts=0;
    loaded=false;
    assert(!readBlockChecked(1,bytes,alerts)&&reads==0);
    loaded=true;assert(readBlockChecked(1,bytes,alerts)&&reads==1);
    next=Result::TransportFault;
    assert(!readBlockChecked(1,bytes,alerts)&&reads==2);
    assert(settingsFaultLatched&&Printer::blockAll&&!outputsAdmitted&&!loaded&&!lastOk);
    for(unsigned i=0;i<1000;++i){
        assert(!readBlockChecked(1,bytes,alerts));
        assert(!accept(Result::Ok));
        failSettings(Result::NeedsRecovery);
    }
    assert(reads==2&&revokes==1&&errors==1&&notices==1&&refreshes==0&&!lastOk);
    std::puts("Settings read fault: terminal admission, no recursive LCD render, no repeated IO PASS");
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"settings-fault.cpp";exe=Path(d)/"settings-fault"
    p.write_text(pre+actual+main)
    subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror",str(p),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=10)

#!/usr/bin/env python3
"""Primary live emergency notice; full secondary details remain in serial report."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
code=r"""
#include "ThermalEmergencyPanel.h"
#include "ThermalEmergencyReport.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>
using namespace ThermalEmergency;
template<uint8_t C,uint8_t R,uint8_t N>
std::string row(const Ledger<N>& l,uint8_t r,bool valid=true,float value=145) {
    char text[C+1];
    panelRow<C,R>(text,r,l,valid,value);
    assert(std::strlen(text)<=C);
    return text;
}
struct Sink {
    std::string text;
    bool tryWriteSpan(const char* p,uint8_t n){assert(n<=63);text.append(p,n);return true;}
};
template<uint8_t C,uint8_t R,uint8_t N> void test(){
    Ledger<N> l;
    Evidence e{};e.targetC=210;e.valid=Evidence::Target;
    assert(l.record(0,Cause::NoRise,e));
    assert((row<C,R>(l,0)=="THERMAL ERROR"));
    assert((row<C,R>(l,1)=="E0:NO RISE"));
    assert((row<C,R>(l,3)=="CHECK SERIAL LOG"));
    assert((row<C,R>(l,2)=="E0:145/210 C"));
    assert((row<C,R>(l,2,false)=="E0:?/210 C"));
    // Fire escalation always replaces the primary notice immediately.
    e.targetC=90;
    assert(l.record(N-1,Cause::OverTemperature,e));
    assert((row<C,R>(l,0)=="FIRE WARNING"));
    assert((row<C,R>(l,1)=="BED:OVER TEMP"));
    assert((row<C,R>(l,3)=="POWER OFF! CHECK"));
    assert((row<C,R>(l,2)=="BED:145/90 C"));
    // Later secondary reasons cannot displace the first fire warning.
    assert(l.record(0,Cause::SensorInvalid,Evidence{}));
    assert((row<C,R>(l,1)=="BED:OVER TEMP"));
    assert(l.first().cause==Cause::NoRise);
    assert(l.reasons(0)&causeBit(Cause::SensorInvalid));
    // The existing full serial report remains the source of all affected
    // reasons, including those no longer cycled on the live LCD.
    ReportCursor<N> report;Sink sink;
    for(unsigned i=0;i<200;++i) {
        const auto p=report.service(l,sink);
        assert(p!=ReportProgress::Blocked&&p!=ReportProgress::Overflow);
        if(p==ReportProgress::Idle)break;
    }
    assert(sink.text.find("Thermal first E0 NO RISE\r\n")!=std::string::npos);
    assert(sink.text.find("Thermal fire BED OVER TEMP\r\n")!=std::string::npos);
    assert(sink.text.find("Thermal affected E0 SENSOR INVALID\r\n")!=std::string::npos);
    // Each reason/channel as the primary, including all system causes.
    for(uint8_t ch=0;ch<=N;++ch)for(uint8_t k=1;k<uint8_t(Cause::Count);++k){
        Ledger<N> x;assert(x.record(ch,Cause(k),Evidence{}));
        assert(primaryFault(x).channel==ch&&primaryFault(x).cause==Cause(k));
        const auto reason=row<C,R>(x,1);
        const auto prefix=ch==N?"SYS:":ch==N-1?"BED:":ch==0?"E0:":"E1:";
        assert(reason.find(prefix)==0&&reason.size()>std::strlen(prefix));
        for(uint8_t r=0;r<R;++r){
            assert((row<C,R>(x,r).size()<=C));
            assert((row<C,R>(x,r,false).size()<=C));
        }
    }
    Ledger<N> reset;
    assert(reset.record(N,Cause::WatchdogReset,Evidence{}));
    assert((row<C,R>(reset,0)=="WATCHDOG RESET"));
    assert((row<C,R>(reset,1)=="SYS:WDT RESET"));
    Ledger<N> interrupted;
    assert(interrupted.record(N,Cause::InterruptedHeating,Evidence{}));
    assert((row<C,R>(interrupted,0)=="THERMAL ERROR"));
    Ledger<N> empty;
    assert((row<C,R>(empty,1)=="SYS:LOCKED"));
}
int main(){
    test<16,4,2>();test<20,4,3>();test<20,4,2>();
    std::puts("Emergency LCD: primary severity, all 16x4/20x4 causes/channels, secondary serial evidence PASS");
}
"""
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"panel.cpp";exe=Path(d)/"panel"
    p.write_text(code)
    subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror","-I",str(root/"Repetier"),str(p),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

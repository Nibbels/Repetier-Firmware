#!/usr/bin/env python3
"""Actual compact LCD renderer for RF1000 16x4 and RF2000 20x4 (plus 16x2 renderer fixture)."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
code=r"""
#include "ThermalEmergencyPanel.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>
using namespace ThermalEmergency;
template<uint8_t C,uint8_t R,uint8_t N>
std::string row(const Ledger<N>& l,const PanelFault& f,uint8_t r,bool temp,bool valid=true,float value=145) {
    char text[C+1];
    panelRow<C,R>(text,r,l,f,temp,valid,value);
    assert(std::strlen(text)<=C);
    return text;
}
template<uint8_t C,uint8_t R,uint8_t N> void test(){
    Ledger<N> l;
    Evidence e{};e.targetC=210;e.valid=Evidence::Target;
    assert(l.record(0,Cause::NoRise,e));
    auto f=panelFault(l,0);
    assert((row<C,R>(l,f,0,false)=="THERMAL ERROR"));
    assert((row<C,R>(l,f,1,false)=="E0:NO RISE"));
    if(R==4)assert((row<C,R>(l,f,3,false)=="CHECK SERIAL LOG"));
    assert((row<C,R>(l,f,R==2?1:2,true)=="E0:145/210 C"));
    assert((row<C,R>(l,f,R==2?1:2,true,false)=="E0:?/210 C"));
    // Fire escalation selects the actual fire cause, not the earlier
    // no-rise condition, but original error remains reachable.
    e.targetC=90;
    assert(l.record(N-1,Cause::OverTemperature,e));
    f=panelFault(l,0);
    assert((row<C,R>(l,f,0,false)=="FIRE WARNING"));
    assert((row<C,R>(l,f,1,false)=="BED:OVER TEMP"));
    if(R==4)assert((row<C,R>(l,f,3,false)=="POWER OFF! CHECK"));
    assert((row<C,R>(l,f,R==2?1:2,true)=="BED:145/90 C"));
    f=panelFault(l,1);
    assert(f.channel==0&&f.cause==Cause::NoRise);
    assert((row<C,R>(l,f,1,false)=="E0:NO RISE"));
    // Every reason on every channel, including system/unknown, must be
    // reachable and fit with full channel and cause in both LCD widths.
    for(uint8_t ch=0;ch<=N;++ch)
        for(uint8_t k=1;k<uint8_t(Cause::Count);++k)l.record(ch,Cause(k),Evidence{});
    bool seen[N+1][uint8_t(Cause::Count)]{};
    for(unsigned page=0;page<(N+1)*(uint8_t(Cause::Count)-1);++page){
        auto x=panelFault(l,page);
        assert(!seen[x.channel][uint8_t(x.cause)]);
        seen[x.channel][uint8_t(x.cause)]=true;
        auto reason=row<C,R>(l,x,1,false);
        const auto prefix=x.channel==N?"SYS:":x.channel==N-1?"BED:":x.channel==0?"E0:":"E1:";
        assert(reason.find(prefix)==0&&reason.size()>std::strlen(prefix));
        for(uint8_t r=0;r<R;++r) {
            assert(row<C,R>(l,x,r,false).size()<=C);
            assert(row<C,R>(l,x,r,true,false).size()<=C);
        }
    }
    Ledger<N> reset;
    assert(reset.record(N,Cause::WatchdogReset,Evidence{}));
    f=panelFault(reset,0xffffffffu);
    assert((row<C,R>(reset,f,0,false)=="WATCHDOG RESET"));
    assert((row<C,R>(reset,f,1,false)=="SYS:WDT RESET"));
    Ledger<N> interrupted;
    assert(interrupted.record(N,Cause::InterruptedHeating,Evidence{}));
    assert((row<C,R>(interrupted,panelFault(interrupted,0),0,false)=="THERMAL ERROR"));
    Ledger<N> empty;
    assert((row<C,R>(empty,panelFault(empty,0),1,false)=="SYS:LOCKED"));
}
int main(){
    test<16,2,2>();test<16,4,2>();test<20,4,3>();test<20,4,2>();
    std::puts("Emergency LCD: 16x2, 16x4 and 20x4, every cause/channel, severity escalation, unknown temperatures and reset distinctions passed");
}
"""
# assert is a macro; template argument commas need extra parentheses.
code=code.replace("assert(row<C,R>(l,x,r,false).size()<=C);","assert((row<C,R>(l,x,r,false).size()<=C));").replace("assert(row<C,R>(l,x,r,true,false).size()<=C);","assert((row<C,R>(l,x,r,true,false).size()<=C));")
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"panel.cpp";exe=Path(d)/"panel"
    p.write_text(code)
    subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror","-I",str(root/"Repetier"),str(p),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

#!/usr/bin/env python3
"""Actual-source absolute cutoff classification and evidence, isolated from peripherals."""
from pathlib import Path
import subprocess
import tempfile
fw=Path(__file__).resolve().parents[1]/"Repetier"
s=(fw/"TemperatureControl.cpp").read_text()
def extract(a,b):
    start=s.index(a);return s[start:s.index(b,start)]
parts=extract("static bool validTemperatureSample(","static void stopAllHeaters(")
parts+='namespace ThermalEmergency {\n'+extract("static Evidence capture(","// Called only after")
parts+=extract("static bool confirm(","void latchSystemFault(")+"}\n"
prelude=r'''
#include "ThermalEmergency.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>
#include <limits>
using millis_t=uint32_t;
#define MIN_DEFECT_TEMPERATURE -10
#define MAX_DEFECT_TEMPERATURE 300
struct TemperatureController {uint8_t pwmIndex,sensorType,pidMax;float currentTemperatureC,targetTemperatureC;uint16_t heaterCutoffC;uint16_t effectiveCutoffC() const{return heaterCutoffC;}};
static TemperatureController acts[3];static uint8_t duties[3];
static bool locked,autotuneAbortRequested;static uint8_t autotuneIndex,autotuneMethod;
static float autotuneTargetC;static unsigned latches,stops,marks;
namespace HAL {uint8_t outputDuty(uint8_t c){return duties[c];}millis_t timeInMilliseconds(){return 12345;}}
namespace Printer {
bool isAnyTempsensorDefect(){return locked;}
void latchOutputInhibit(bool thermal){assert(thermal&&!locked);locked=true;++latches;for(int i=0;i<3;++i){duties[i]=0;acts[i].targetTemperatureC=0;}}
void markTemperatureReadingsAvailable(){++marks;}
void stopPrint(){assert(locked);for(int i=0;i<3;++i)assert(!duties[i]&&!acts[i].targetTemperatureC);++stops;}
}
namespace ThermalEmergency {Ledger<3> ledger;}
'''
main=r'''
int main(){using namespace ThermalEmergency;unsigned cases=0;
    for(uint8_t ch=0;ch<3;++ch)for(uint16_t cutoff:{uint16_t(275),uint16_t(280)})
    for(float t:{-11.f,-10.f,25.f,float(cutoff)-.25f,float(cutoff),float(cutoff)+.25f,300.f,301.f,
                 std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()})
    for(bool supported:{false,true})for(bool raw:{false,true})for(bool enabled:{false,true})
    for(bool previous:{false,true})for(bool tuning:{false,true})for(bool idle:{false,true}){
        ledger=Ledger<3>{};locked=previous;latches=stops=marks=0;autotuneAbortRequested=false;
        if(previous)ledger.record(3,Cause::LeaseExpired,Evidence{});
        autotuneIndex=tuning?ch:255;autotuneTargetC=150;autotuneMethod=ch==2?3:1;
        for(uint8_t i=0;i<3;++i){acts[i]={i,60,128,25,previous||idle?0.f:100.f,cutoff};duties[i]=previous||idle?0:128;}
        auto& a=acts[ch];a.currentTemperatureC=t;if(!enabled)a.sensorType=0;
        const float target=a.targetTemperatureC;const uint8_t duty=duties[ch];
        SensorObservation observation;observation.observe(raw,1195);
        const bool valid=validTemperatureSample(supported,t);
        const bool fire=valid&&raw&&enabled&&t>cutoff;
        assert(checkAbsoluteCutoff(ch,a,valid,observation)==fire);
        assert(autotuneAbortRequested==fire);
        assert(latches==unsigned(fire&&!previous)&&stops==latches&&marks==latches);
        if(fire){assert(locked&&ledger.level()==Severity::FireWarning);
            const auto& r=ledger.firstFire();const auto& e=r.evidence;
            assert(r.channel==ch&&r.cause==Cause::OverTemperature&&e.threshold==cutoff&&e.deltaC==t-cutoff);
            assert(e.measuredC==t&&e.admittedDuty==duty&&e.rawAdc==1195&&(e.valid&Evidence::Raw));
            assert(e.targetC==(tuning?150.f:target)&&bool(e.valid&Evidence::Profile)==tuning);
            if(tuning)assert(e.profile==autotuneMethod);
            assert(ledger.first().cause==(previous?Cause::LeaseExpired:Cause::OverTemperature));
        }else{assert(locked==previous&&ledger.level()!=Severity::FireWarning);assert(duties[ch]==duty&&a.targetTemperatureC==target);}
        ++cases;
    }
    std::printf("Absolute cutoff actual-source classification: %u cases passed\n",cases);
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/'cutoff.cpp';exe=Path(d)/'cutoff';p.write_text(prelude+parts+main)
    subprocess.run(['c++','-std=c++11','-O2','-Wall','-Wextra','-Werror','-I',str(fw),str(p),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

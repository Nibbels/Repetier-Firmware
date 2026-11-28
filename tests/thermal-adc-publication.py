#!/usr/bin/env python3
"""Actual HAL ADC snapshot + diagnostic publication identity (host fixture)."""
from pathlib import Path
import subprocess
import tempfile
fw=Path(__file__).resolve().parents[1]/"Repetier"
s=(fw/"HAL.cpp").read_text()
a=s.index("bool HAL::temperatureAdcSample(")
b=s.index("#ifdef RF_HEATER_LEASE_EXPERIMENT\nHAL::HeaterSample::~HeaterSample",a)
pre=r'''
#include "ThermalRelayProfile.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>
constexpr uint8_t TemperatureAdcCount=3;
constexpr uint32_t temperatureAdcMaxAgeMs=100;
static uint16_t temperatureAdcValues[3];
static uint32_t temperatureAdcPublishedAt[3],now;
static bool temperatureAdcReady[3],inject;
static uint8_t channel;
struct InterruptProtectedBlock {
    ~InterruptProtectedBlock(){if(inject){inject=false;temperatureAdcValues[channel]=5678;temperatureAdcPublishedAt[channel]=now;temperatureAdcReady[channel]=true;}}
};
struct HAL {
    static uint32_t timeInMilliseconds(){return now;}
    static bool temperatureAdcSample(uint8_t,uint16_t&,uint32_t* publication=nullptr);
};
'''
main=r'''
int main(){unsigned snapshots=0,tracks=0;
    for(uint32_t start:{0u,0xffffff9cu,0xffffffffu})for(uint8_t c=0;c<3;++c)
    for(unsigned age=0;age<=101;++age)for(bool ready:{false,true})for(bool race:{false,true}){
        channel=c;now=start+age;temperatureAdcPublishedAt[c]=start;temperatureAdcValues[c]=1234;
        temperatureAdcReady[c]=ready;inject=race;
        uint16_t raw=0;uint32_t pub=0xdeadbeef;
        const bool ok=HAL::temperatureAdcSample(c,raw,&pub);
        assert(!inject&&raw==1234&&ok==(ready&&age<=100));
        assert(pub==(ok?start:0xdeadbeefu));
        assert(temperatureAdcReady[c]==(race||(ready&&age<=100)));
        if(race){assert(temperatureAdcValues[c]==5678&&temperatureAdcPublishedAt[c]==now);}
        ++snapshots;
    }
    for(uint8_t c:{uint8_t(3),uint8_t(255)}){uint16_t raw=1234;uint32_t pub=0xdeadbeef;
        assert(!HAL::temperatureAdcSample(c,raw,&pub)&&raw==0&&pub==0xdeadbeefu);++snapshots;}
    using namespace ThermalRelayProfile;
    for(uint32_t start:{0u,1u,0xfffffb00u,0xffffffffu}){
        for(uint32_t delta=0;delta<=1200;++delta){PublicationTracker tracker(start);
            assert(tracker.next(start+delta)==(delta>0&&delta<=1100));++tracks;}
        PublicationTracker tracker(start);assert(!tracker.next(start-1));++tracks;
    }
    // Equal ADC values are not duplicates when their publication times advance.
    PublicationTracker tracker(100);assert(tracker.next(200));assert(!tracker.next(200));
    Recorder r;r.begin(0,25,255,255);r.invalidate();
    for(uint32_t n=100;n<10000;n+=100){r.sample(n,25,255);r.afterDecision(n,25,255);}
    assert(!r.completedProfile());
    std::printf("Coherent ADC snapshot: %u cases; publication identity: %u cases passed\n",snapshots,tracks);
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/'snapshot.cpp';exe=Path(d)/'snapshot';p.write_text(pre+s[a:b]+main)
    subprocess.run(['c++','-std=c++11','-O2','-Wall','-Wextra','-Werror','-I',str(fw),str(p),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

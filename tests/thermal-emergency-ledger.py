#!/usr/bin/env python3
"""Actual compact diagnostic ledger, not the physical shutdown or UI."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
code=r"""
#include "ThermalEmergency.h"
#include <cassert>
#include <cstdio>
using namespace ThermalEmergency;
constexpr uint8_t count=uint8_t(Cause::Count);
static_assert(sizeof(Ledger<3>)<=84,"unexpected host storage growth");
static_assert(severity(Cause::WatchdogReset)==Severity::ThermalError,"reset is not fire");
static_assert(severity(Cause::OverTemperature)==Severity::FireWarning,"overtemperature");
static_assert(severity(Cause::ExcessiveRise)==Severity::FireWarning,"excessive rise");
static_assert(severity(Cause::HeatingWhileOff)==Severity::FireWarning,"off-state heating");
static_assert(bit(Cause::None)==0 && bit(Cause::Count)==0,"invalid masks");

Evidence evidence(unsigned seed) {
    return {uint32_t(0xfffffff0u+seed),12000u+seed,float(seed)+0.25f,
            210.f+seed,0.5f+seed,1.f, uint16_t(seed),255,128,3,200,1,
            uint8_t(Evidence::Measured|Evidence::Target|Evidence::Delta|
            Evidence::Threshold|Evidence::Raw|Evidence::Requested|Evidence::Admitted|Evidence::Profile)};
}
void equal(const Evidence& a,const Evidence& b) {
    assert(a.uptimeMs==b.uptimeMs&&a.windowMs==b.windowMs);
    assert(a.measuredC==b.measuredC&&a.targetC==b.targetC&&a.deltaC==b.deltaC&&a.threshold==b.threshold);
    assert(a.rawAdc==b.rawAdc&&a.requestedDuty==b.requestedDuty&&a.admittedDuty==b.admittedDuty);
    assert(a.sensorType==b.sensorType&&a.powerLimit==b.powerLimit&&a.profile==b.profile&&a.valid==b.valid);
}
template<uint8_t N> unsigned replay() {
    unsigned cases=0;
    for(uint8_t a=1;a<count;++a)
    for(uint8_t b=1;b<count;++b)
    for(uint8_t x=0;x<=N;++x)
    for(uint8_t y=0;y<=N;++y) {
        Ledger<N> log;
        assert(!log.active()&&log.level()==Severity::None);
        auto original=evidence(0),later=evidence(32);
        assert(!log.record(N+1,Cause(a),original));
        assert(!log.record(x,Cause::None,original));
        assert(!log.record(x,Cause(255),original));
        assert(log.record(x,Cause(a),original));
        assert(log.active()&&log.first().cause==Cause(a)&&log.first().channel==x);
        // Caller can immediately zero its live targets and duty; the ledger
        // is an independent copy. Raw ADC=0 is a valid observation.
        original.targetC=0;original.requestedDuty=0;
        assert(!log.record(x,Cause(a),later));
        assert(log.record(y,Cause(b),later)==(a!=b||x!=y));
        equal(log.first().evidence,evidence(0));
        assert(log.first().cause==Cause(a)&&log.first().channel==x);
        const bool fireA=severity(Cause(a))==Severity::FireWarning;
        const bool fireB=severity(Cause(b))==Severity::FireWarning;
        assert(log.level()==(fireA||fireB?Severity::FireWarning:Severity::ThermalError));
        if(fireA||fireB) {
            assert(log.firstFire().cause==Cause(fireA?a:b));
            assert(log.firstFire().channel==(fireA?x:y));
            equal(log.firstFire().evidence,evidence(fireA?0:32));
        }else assert(log.firstFire().cause==Cause::None);
        for(uint8_t c=0;c<=N;++c)
            assert(log.reasons(c)==((c==x?bit(Cause(a)):0)|(c==y?bit(Cause(b)):0)));
        assert(log.reasons(255)==0);
        // All causes on all channels, including a late fire escalation,
        // leave first evidence untouched.
        for(uint8_t c=0;c<=N;++c)for(uint8_t k=1;k<count;++k)
            log.record(c,Cause(k),later);
        equal(log.first().evidence,evidence(0));
        assert(log.level()==Severity::FireWarning);
        for(uint8_t c=0;c<=N;++c)assert(log.reasons(c)==uint16_t((1u<<(count-1))-1));
        ++cases;
    }
    // Unknown snapshot (e.g. bootloader-cleared reset cause): no invented
    // target, measurement, ADC value or physical heater assignment.
    Ledger<N> reset;
    Evidence unknown{};
    assert(reset.record(Ledger<N>::System,Cause::WatchdogReset,unknown));
    assert(reset.first().evidence.valid==0&&reset.level()==Severity::ThermalError);
    return cases;
}
int main() {
    const auto cases=replay<2>()+replay<3>();
    std::printf("Thermal emergency ledger: %u ordered cause/channel pairs; host storage %zu/%zu bytes\n",
                cases,sizeof(Ledger<2>),sizeof(Ledger<3>));
}
"""
with tempfile.TemporaryDirectory() as temp:
    cpp=Path(temp)/"ledger.cpp"
    exe=Path(temp)/"ledger"
    cpp.write_text(code)
    subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror",
                    "-I",str(root/"Repetier"),str(cpp),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

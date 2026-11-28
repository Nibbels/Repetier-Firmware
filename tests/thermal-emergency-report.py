#!/usr/bin/env python3
"""Replay real bounded fault report generation without an active printer."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
code=r"""
#include "ThermalEmergencyReport.h"
#include <cassert>
#include <cstdio>
#include <string>
#include <limits>
#include <vector>
using namespace ThermalEmergency;
struct Sink {
    bool block=false;
    unsigned attempts=0;
    std::string last;
    std::vector<std::string> lines;
    bool tryWriteSpan(const char* p,uint8_t n) {
        ++attempts;
        assert(n>=2&&n<=63&&p[n-2]=='\r'&&p[n-1]=='\n');
        last.assign(p,n);
        assert(last.find('\r')==size_t(n-2)&&last.find('\n')==size_t(n-1));
        if(block)return false;
        lines.push_back(last);return true;
    }
    unsigned count(const std::string& line) const {
        unsigned n=0;
        for(const auto& l:lines)if(l==line+"\r\n")++n;
        return n;
    }
};
template<uint8_t N> void drain(const Ledger<N>& log,ReportCursor<N>& report,Sink& sink) {
    for(unsigned i=0;i<200;++i) {
        unsigned before=sink.attempts;
        auto r=report.service(log,sink);
        assert(sink.attempts-before<=1);
        assert(r!=ReportProgress::Blocked&&r!=ReportProgress::Overflow);
        if(r==ReportProgress::Idle)return;
    }
    assert(false);
}
Evidence known() {
    return {0xffffffffu,12000,145.25f,210.f,-0.25f,1.f,0,255,128,3,200,1,4095};
}
int main() {
    Ledger<3> log;
    ReportCursor<3> report;
    Sink sink;
    CancellationCursor cancel;
    Sink cancelSink;
    assert(cancel.service(false,cancelSink)==ReportProgress::Idle);
    cancelSink.block=true;
    for(unsigned i=0;i<1000;++i)
        assert(cancel.service(true,cancelSink)==ReportProgress::Blocked);
    assert(cancelSink.lines.empty());
    cancelSink.block=false;
    assert(cancel.service(true,cancelSink)==ReportProgress::Sent);
    cancelSink.block=true;
    assert(cancel.service(true,cancelSink)==ReportProgress::Blocked);
    cancelSink.block=false;
    assert(cancel.service(true,cancelSink)==ReportProgress::Sent);
    for(unsigned i=0;i<100;++i)
        assert(cancel.service(true,cancelSink)==ReportProgress::Idle);
    assert(cancelSink.count("RequestStop:")==1&&cancelSink.count("// action:cancel")==1);
    assert(report.service(log,sink)==ReportProgress::Idle&&sink.attempts==0);
    assert(log.record(0,Cause::NoRise,known()));
    sink.block=true;
    for(unsigned i=0;i<1000;++i){
        assert(report.service(log,sink)==ReportProgress::Blocked);
        assert(sink.last=="Error: THERMAL EMERGENCY\r\n"&&sink.lines.empty());
    }
    sink.block=false;
    assert(report.service(log,sink)==ReportProgress::Sent);
    assert(report.service(log,sink)==ReportProgress::Sent);
    sink.block=true;
    assert(report.service(log,sink)==ReportProgress::Blocked);
    assert(sink.last=="Thermal first uptime_ms: 4294967295\r\n");
    // Fire escalation while a prior report is blocked. First report must
    // retain its snapshot, then a second complete report carries fire.
    auto severe=known();severe.measuredC=301.f;
    assert(log.record(2,Cause::OverTemperature,severe));
    assert(log.record(1,Cause::SensorInvalid,Evidence{}));
    assert(report.service(log,sink)==ReportProgress::Blocked);
    assert(sink.last=="Thermal first uptime_ms: 4294967295\r\n");
    sink.block=false;
    drain(log,report,sink);
    assert(sink.count("Error: THERMAL EMERGENCY")==1);
    assert(sink.count("Error: FIRE WARNING")==1);
    assert(sink.count("Thermal first E0 NO RISE")==2);
    assert(sink.count("Thermal first target_C: 210.0")==2);
    assert(sink.count("Thermal first measured_C: 145.3")==2);
    assert(sink.count("Thermal first delta_C: -0.3")==2);
    assert(sink.count("Thermal first raw_ADC: 0")==2);
    assert(sink.count("Thermal first admitted_PWM: 128")==2);
    assert(sink.count("Thermal fire BED OVER TEMP")==1);
    assert(sink.count("Thermal fire measured_C: 301.0")==1);
    assert(sink.count("Thermal affected BED OVER TEMP")==1);
    assert(sink.count("Thermal affected E1 SENSOR INVALID")==1);
    assert(report.service(log,sink)==ReportProgress::Idle);
    report.requestReplay();report.requestReplay();
    drain(log,report,sink);
    assert(sink.count("Error: FIRE WARNING")==2);
    assert(sink.count("Thermal first E0 NO RISE")==3);
    Ledger<2> reset;
    ReportCursor<2> resetReport;
    Sink resetSink;
    assert(reset.record(Ledger<2>::System,Cause::WatchdogReset,Evidence{}));
    drain(reset,resetReport,resetSink);
    assert(resetSink.count("Error: WATCHDOG RESET")==1);
    assert(resetSink.count("Error: FIRE WARNING")==0);
    assert(resetSink.count("Thermal first SYSTEM WATCHDOG RESET")==1);
    assert(resetSink.count("Thermal first measured_C: unknown")==1);
    assert(resetSink.count("Thermal first requested_PWM: unknown")==1);
    assert(resetSink.count("Thermal first uptime_ms: unknown")==1);
    // All causes/channels, worst valid numeric lengths, non-finite inputs.
    for(uint8_t cause=1;cause<uint8_t(Cause::Count);++cause) {
        for(uint8_t channel=0;channel<=3;++channel) {
            Ledger<3> l;ReportCursor<3> r;Sink s;
            auto extreme=known();
            extreme.windowMs=0xffffffffu;extreme.measuredC=-10000.f;
            extreme.targetC=std::numeric_limits<float>::quiet_NaN();
            extreme.deltaC=std::numeric_limits<float>::infinity();
            extreme.threshold=10000.f;extreme.rawAdc=65535;
            assert(l.record(channel,Cause(cause),extreme));
            for(uint8_t c=0;c<=3;++c)for(uint8_t k=1;k<uint8_t(Cause::Count);++k)
                l.record(c,Cause(k),extreme);
            drain(l,r,s);
            assert(s.count("Thermal first measured_C: -10000.0")==1);
            assert(s.count("Thermal first target_C: unknown")==1);
            assert(s.count("Thermal first delta_C: unknown")==1);
            assert(s.count("Thermal first threshold: 10000.0")==1);
            assert(s.count("Thermal first raw_ADC: 65535")==1);
        }
    }
    std::puts("Emergency report: blocked TX, exact line retry, escalation, replay, unknown reset fields, all cause/channel combinations passed");
}
"""
with tempfile.TemporaryDirectory() as temp:
    cpp=Path(temp)/"report.cpp"
    exe=Path(temp)/"report"
    cpp.write_text(code)
    subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror",
                    "-I",str(root/"Repetier"),str(cpp),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

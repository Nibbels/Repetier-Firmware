#!/usr/bin/env python3
"""Exhaustive stable diagnostic wire IDs for native and reordered enums."""
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory
fw=Path(__file__).resolve().parents[1]/"Repetier"
source=r'''
#include <cassert>
#include "ThermalEmergency.h"
#include "ThermalEventSource.h"
enum class Shuffled : uint8_t {
    SensorInvalid=99,
    StaleSample=18,
    NoRise=42,
    ExcessiveRise=3,
    HoldFailure=200,
    OverTemperature=7,
    HeatingWhileOff=90,
    LeaseExpired=255,
    WatchdogReset=1,
    CalibrationMismatch=6,
    InterruptedHeating=54
};
template<class C> void check() {
    const C labels[]={C::SensorInvalid,C::StaleSample,C::NoRise,C::ExcessiveRise,C::HoldFailure,C::OverTemperature,C::HeatingWhileOff,C::LeaseExpired,C::WatchdogReset,C::CalibrationMismatch,C::InterruptedHeating};
    for(unsigned raw=0; raw<256; ++raw){
        unsigned expected=0;
        for(unsigned i=0;i<11;++i)
            if(C(raw)==labels[i])expected=i+1;
        assert(EventLog::thermalReason(C(raw))==expected);
    }
}
int main(){check<ThermalEmergency::Cause>();check<Shuffled>();}
'''
with TemporaryDirectory() as d:
    p=Path(d)/"wire.cpp"
    p.write_text(source)
    exe=Path(d)/"wire"
    subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror","-I",str(fw),str(p),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print("EventLog reason wire IDs: 512 values, native and shuffled enums PASS")

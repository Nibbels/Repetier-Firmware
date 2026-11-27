#!/usr/bin/env python3
"""Replay actual usage publication and persistence on the new settings owner."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
fw=root/"Repetier"
source=(fw/"EepromZoneRuntime.cpp").read_text()
def extract(first,last):
    a=source.index(first)
    return source[a:source.index(last,a)]
stage=extract("bool stageUsageBlock(","bool loadFull(")
usage=extract("EEPROM::UsageTotals EEPROM::usageTotals()","#endif // RF_EEPROM_ZONE_EXPERIMENT")
pre=r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#define __AVR_ATmega2560__ 1
#include "BoardConfiguration.h"
#include "EepromZonePatch.h"
#include "EepromZoneValidation.h"
using namespace EepromZones;
struct EEPROM {
    struct UsageTotals {int32_t seconds;float metres;};
    static UsageTotals usageTotals();
    static void storeUsageTotals(UsageTotals);
};
bool loaded=false,settingsFaultLatched=false,failWrite=false;
EEPROM::UsageTotals committedUsage{0,0};
unsigned writes=0;
bool persist(const Patch* edits,uint8_t count){
    assert(count==2&&edits[0].pos==address(Field::F_PRINTING_TIME)&&edits[1].pos==address(Field::F_PRINTING_DISTANCE));
    ++writes;
    if(failWrite){settingsFaultLatched=true;loaded=false;return false;}
    return loaded&&!settingsFaultLatched;
}
'''
main=r'''
int main(){
    uint8_t bytes[8]={};
    BlockEncoder encoder(Usage,bytes,sizeof bytes);
    encoder.u32(Field::F_PRINTING_TIME,1234);
    encoder.f32(Field::F_PRINTING_DISTANCE,12.5f);
    assert(encoder.complete());
    assert(stageUsageBlock(Usage,bytes,sizeof bytes));
    assert(!EEPROM::usageTotals().seconds&&!EEPROM::usageTotals().metres);
    loaded=true;
    for(unsigned i=0;i<1000;++i){
        auto t=EEPROM::usageTotals();
        assert(t.seconds==1234&&t.metres==12.5f);
    }
    assert(!writes);
    EEPROM::storeUsageTotals({1500,14.25f});
    assert(writes==1&&EEPROM::usageTotals().seconds==1500&&EEPROM::usageTotals().metres==14.25f);
    failWrite=true;EEPROM::storeUsageTotals({1800,17.0f});
    assert(writes==2&&committedUsage.seconds==1500&&committedUsage.metres==14.25f);
    assert(!EEPROM::usageTotals().seconds&&!EEPROM::usageTotals().metres);
    // Restore stages new counters but does not publish on failed load.
    memset(bytes,0,sizeof bytes);
    assert(stageUsageBlock(Usage,bytes,sizeof bytes));
    assert(!EEPROM::usageTotals().seconds&&!EEPROM::usageTotals().metres);
    assert(!stageUsageBlock(Usage,bytes,7));
    assert(!stageUsageBlock(0,bytes,8));
    memset(bytes,0xff,sizeof bytes);
    assert(!stageUsageBlock(Usage,bytes,8));
    std::puts("Usage snapshot: validated load, IO-free reads, commit-before-publication, terminal failure PASS");
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"usage.cpp";exe=Path(d)/"usage"
    p.write_text(pre+stage+usage+main)
    for flags in (["-DMOTHERBOARD=DEVICE_TYPE_RF1000"],
                  ["-DMOTHERBOARD=DEVICE_TYPE_RF2000"],
                  ["-DMOTHERBOARD=DEVICE_TYPE_RF2000v2"],
                  ["-DMOTHERBOARD=DEVICE_TYPE_RF2000","-DNUM_EXTRUDER=1","-DEXT0_EXTRUDER_COOLER_PIN=X44_PIN"]):
        subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror",*flags,"-I",str(fw),str(p),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True,timeout=10)

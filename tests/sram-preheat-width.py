#!/usr/bin/env python3
"""Exhaustive stored/adjusted bed-profile ranges, using the actual owner."""
from pathlib import Path
import re, subprocess
from tempfile import TemporaryDirectory
fw=Path(__file__).resolve().parents[1]/"Repetier"
config=(fw/"Configuration.h").read_text()
defines="\n".join(re.findall(r"^#define (?:PREHEAT_\w+|UI_SET_(?:MIN|MAX)_\w+_TEMP)\s+[^\n]+",config,re.M))
owner=(fw/"PreheatProfiles.cpp").read_text()
src=r'''
#include <cassert>
#include "PreheatProfiles.h"
struct BedFixture {int16_t maximum=0; int16_t maximumTargetC()const{return maximum;}} heatedBedController;
'''
checks=r'''
using P=PreheatProfiles;
int main(){
    static_assert(sizeof(P)==6,"compact storage must not rely on alignment");
    constexpr int min=UI_SET_MIN_HEATED_BED_TEMP;
    constexpr int max=UI_SET_MAX_HEATED_BED_TEMP<140?UI_SET_MAX_HEATED_BED_TEMP:140;
    unsigned long cases=0;
    for(int p=0;p<2;++p){
        const auto profile=P::Profile(p);
        const int factory=p?PREHEAT_B_BED_C:PREHEAT_A_BED_C;
        const int hot=p?PREHEAT_B_HOTEND_C:PREHEAT_A_HOTEND_C;
        P owner;
        for(int raw=-32768;raw<=32767;++raw){
            const int expected=raw==0||(raw>=min&&raw<=max)?raw:
                (factory==0||(factory>=min&&factory<=max)?factory:0);
            assert(owner.readStored(profile,{int16_t(raw),int16_t(hot)})==(expected!=raw));
            const auto v=owner.get(profile);
            assert(v.bed==expected&&v.hotend==hot);
            ++cases;
        }
        // Include starts above the persisted-load cap, reachable by menu
        // adjustments when the live limit permits them. Preserve them on a
        // subsequent reduction or disable of that live limit.
        constexpr int uiMax=UI_SET_MAX_HEATED_BED_TEMP;
        for(int start=0;start<=uiMax;++start){
            if(start&&start<min)continue;
            P seed;
            const int loaded=start>max?max:start;
            assert(!seed.readStored(profile,{int16_t(loaded),int16_t(hot)}));
            heatedBedController.maximum=uiMax;
            for(int i=loaded;i<start;++i)
                assert(seed.adjust(profile,P::Heater::Bed,1));
            assert(seed.get(profile).bed==start);
            for(int cap=-1;cap<=uiMax+1;++cap){
                heatedBedController.maximum=cap;
                const int effective=cap>uiMax?uiMax:cap;
                for(int inc=-128;inc<=127;++inc){
                    P trial=seed;
                    int next=start;
                    if(effective>=min&&inc){
                        next+=inc;
                        if(next<min)next=inc>0?min:0;
                        if(next>effective)next=effective;
                    }
                    assert(trial.adjust(profile,P::Heater::Bed,int8_t(inc))==(next!=start));
                    const auto v=trial.get(profile);
                    assert(v.bed==next&&v.hotend==hot);
                    ++cases;
                }
            }
        }
        owner.restoreDefaults();
        assert(owner.get(profile).bed==factory&&owner.get(profile).hotend==hot);
    }
    assert(cases>5000000);
}
'''
with TemporaryDirectory() as d:
    p=Path(d)/"width.cpp"
    p.write_text(defines+"\n"+src+owner[owner.index("namespace {"):]+checks)
    exe=Path(d)/"width"
    subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror","-I",str(fw),str(p),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print("Preheat compact bed storage: exhaustive int16 loads and all reachable start/cap/increment combinations PASS")

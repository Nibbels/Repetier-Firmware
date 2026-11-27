#!/usr/bin/env python3
"""Replay LCD RMW interleavings against Timer1 motion writes on PORTK/PORTL."""
from pathlib import Path
import os
import re
import subprocess
import tempfile

fw=Path(__file__).resolve().parents[1]/"Repetier"
source=(fw/"ui.cpp").read_text()
assert not re.search(r"\bWRITE\(UI_DISPLAY_",source), "LCD write bypasses the shared-port helper"
start=source.index("#define LCD_WRITE(pin, value)")
end=source.index("void UIDisplay::loadStartupCharacters()",start)
transport=source[start:end]
init=source[source.index("void UIDisplay::initializeLCD("):source.index("#undef LCD_WRITE")]
assert init.count("LCD_WRITE(UI_DISPLAY_RS_PIN, LOW)")==2
assert init.count("LCD_WRITE(UI_DISPLAY_ENABLE_PIN, LOW)")==1

stub=r'''
#include <cassert>
#include <cstdint>
#include <utility>
#include <vector>
#define __AVR_ATmega2560__ 1
#include "BoardConfiguration.h"
constexpr bool HIGH=true, LOW=false;
bool irq=true;
bool pending[2]{};
uint8_t latch[2]{};
unsigned locks=0, depth=0, serviced[2]{}, delayCount=0;
using Event=std::pair<int,bool>;
std::vector<Event> writes;
constexpr uint8_t motionMask[]={1,2}; // PK0 Z-enable, PL1 Z-dir
int groupFor(int pin){
    if(pin>=63 && pin<=66 && (pin!=66 || MOTHERBOARD==DEVICE_TYPE_RF1000))return 0;
    if(MOTHERBOARD==DEVICE_TYPE_RF1000 && pin==44)return 1;
    return -1;
}
uint8_t lcdMask(int group,int pin){return uint8_t(1u<<(group==0?pin-62:5));}
void servicePending(){
    if(!irq)return;
    for(int g=0;g<2;++g)if(pending[g]){
        pending[g]=false;latch[g]^=motionMask[g];++serviced[g];
    }
}
struct InterruptProtectedBlock {
    bool old=irq;
    InterruptProtectedBlock(){assert(depth==0);++depth;++locks;irq=false;}
    ~InterruptProtectedBlock(){assert(depth==1);--depth;irq=old;servicePending();}
};
void fakeWrite(int pin,bool value){
    writes.emplace_back(pin,value);
    int g=groupFor(pin);
    if(g<0){assert(depth==0);return;}
    uint8_t mask=lcdMask(g,pin);
    uint8_t old=latch[g]; // AVR extended-I/O read
    // Inject a motion update at exactly the dangerous read/store boundary.
    pending[g]=true;servicePending();
    latch[g]=value?uint8_t(old|mask):uint8_t(old&~mask); // AVR store
}
#define WRITE(pin,value) fakeWrite(pin,bool(value))
namespace HAL {
void delayMicroseconds(unsigned us){assert(us==320 && depth==0);++delayCount;}
}
void reset(bool enabled,uint8_t initial){
    irq=enabled;pending[0]=pending[1]=false;latch[0]=latch[1]=initial;
    locks=depth=serviced[0]=serviced[1]=delayCount=0;writes.clear();
}
'''
checks=r'''
// Exercise the real macro on each LCD output, preserving entry interrupt state.
void one(int pin,bool value,bool enabled,uint8_t initial){
    reset(enabled,initial);
    switch(pin){
    case UI_DISPLAY_RS_PIN: LCD_WRITE(UI_DISPLAY_RS_PIN,value);break;
    case UI_DISPLAY_D4_PIN: LCD_WRITE(UI_DISPLAY_D4_PIN,value);break;
    case UI_DISPLAY_D5_PIN: LCD_WRITE(UI_DISPLAY_D5_PIN,value);break;
    case UI_DISPLAY_D6_PIN: LCD_WRITE(UI_DISPLAY_D6_PIN,value);break;
    case UI_DISPLAY_D7_PIN: LCD_WRITE(UI_DISPLAY_D7_PIN,value);break;
    case UI_DISPLAY_ENABLE_PIN: LCD_WRITE(UI_DISPLAY_ENABLE_PIN,value);break;
    default:assert(false);
    }
    assert(irq==enabled && depth==0 && writes.size()==1);
    int g=groupFor(pin);
    assert(locks==unsigned(g>=0));
    if(g<0){assert(latch[0]==initial&&latch[1]==initial);return;}
    if(!enabled){assert(pending[g]&&serviced[g]==0);irq=true;servicePending();}
    assert(serviced[g]==1&&!pending[g]);
    uint8_t mask=lcdMask(g,pin), expected=uint8_t(initial^motionMask[g]);
    expected=value?uint8_t(expected|mask):uint8_t(expected&~mask);
    assert(latch[g]==expected&&latch[1-g]==initial);
}
void appendNibble(std::vector<Event>& want,unsigned v){
    want.emplace_back(UI_DISPLAY_D4_PIN,bool(v&1));
    want.emplace_back(UI_DISPLAY_D5_PIN,bool(v&2));
    want.emplace_back(UI_DISPLAY_D6_PIN,bool(v&4));
    want.emplace_back(UI_DISPLAY_D7_PIN,bool(v&8));
    want.emplace_back(UI_DISPLAY_ENABLE_PIN,true);
    want.emplace_back(UI_DISPLAY_ENABLE_PIN,false);
}
void checkSequence(const std::vector<Event>& want,unsigned delays){
    assert(writes==want&&irq&&depth==0&&delayCount==delays);
    unsigned shared[2]{};
    for(auto e:want)if(groupFor(e.first)>=0)++shared[groupFor(e.first)];
    assert(locks==shared[0]+shared[1]);
    for(int g=0;g<2;++g){
        assert(serviced[g]==shared[g]&&!pending[g]);
        assert(bool(latch[g]&motionMask[g])==bool(shared[g]&1));
    }
}
int main(){
    const int pins[]={UI_DISPLAY_RS_PIN,UI_DISPLAY_D4_PIN,UI_DISPLAY_D5_PIN,
                      UI_DISPLAY_D6_PIN,UI_DISPLAY_D7_PIN,UI_DISPLAY_ENABLE_PIN};
    for(int pin:pins)for(unsigned initial=0;initial<256;++initial)
        for(bool enabled:{false,true})for(bool value:{false,true})
            one(pin,value,enabled,uint8_t(initial));
    for(unsigned v=0;v<16;++v){
        reset(true,0);std::vector<Event> want;appendNibble(want,v);
        lcdWriteNibble(v);checkSequence(want,0);
    }
    for(unsigned v=0;v<256;++v)for(bool rs:{false,true}){
        reset(true,0);std::vector<Event> want{{UI_DISPLAY_RS_PIN,rs}};
        appendNibble(want,v>>4);appendNibble(want,v&15);
        lcdWriteByte(v,rs);checkSequence(want,1);
    }
}
'''
variants=[
    ["-DMOTHERBOARD=DEVICE_TYPE_RF1000"],
    ["-DMOTHERBOARD=DEVICE_TYPE_RF2000"],
    ["-DMOTHERBOARD=DEVICE_TYPE_RF2000v2"],
    ["-DMOTHERBOARD=DEVICE_TYPE_RF2000","-DNUM_EXTRUDER=1","-DEXT0_EXTRUDER_COOLER_PIN=X44_PIN"],
]
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"lcd.cpp";exe=Path(d)/"lcd"
    p.write_text(stub+transport+checks)
    for flags in variants:
        subprocess.run([os.environ.get("CXX","c++"),"-std=c++11","-O2","-Wall","-Wextra","-Werror",
                        "-I",str(fw),*flags,str(p),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True,timeout=10)
        print("LCD shared-port interleavings, entry I state, all nibble/byte writes PASS",flags)

#!/usr/bin/env python3
"""Actual experimental M206 method, signed address and thermal revoke replay."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
fw=root/"Repetier"
source=(fw/"Eeprom.cpp").read_text()
a=source.index("#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\n// Identify actual changed bytes")
b=source.index("void EEPROM::restoreEEPROMExtruderSettingsFromConfiguration",a)
header=(fw/"Eeprom.h").read_text()
gcode=(fw/"gcode.h").read_text()
assert "int32_t P;" in gcode and "uint8_t T;" in gcode
pre=r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>
using PGM_P=const char*;
#define PSTR(x) x
#include "Eeprom.h"
struct GCode {
    int32_t P=0,S=0;uint8_t T=0;float X=0;
    bool p=true,t=true,s=true,x=true;
    bool hasP()const{return p;}bool hasT()const{return t;}
    bool hasS()const{return s;}bool hasX()const{return x;}
};
static uint8_t bytes[2048];
static unsigned writes,loads,checks,errors,invalidated;
static int ignore=-1;
static bool requireRevoke=false;
static uint8_t expectedSlots=0;
static uint8_t slotFor(unsigned pos){
    if((pos>=107&&pos<=120)||pos==124||pos==1922)return 4;
    if((pos>=217&&pos<=230)||pos==245||pos==294)return 1;
    if((pos>=317&&pos<=330)||pos==345||pos==394)return 2;
    return 0;
}
struct HAL {
    static uint8_t eprGetByte(unsigned p){assert(p<sizeof bytes);return bytes[p];}
    static void put(unsigned p,uint8_t v){
        assert(p<sizeof bytes);++writes;
        if(requireRevoke&&bytes[p]!=v&&slotFor(p)){
            const auto slot=slotFor(p);
            for(unsigned k=0;k<3;++k)if(slot&(1u<<k)){
                assert(bytes[1978+3*k]==0);
                if(k==2||k<NUM_EXTRUDER)assert(invalidated&(1u<<k));
            }
        }
        if(int(p)!=ignore)bytes[p]=v;
    }
    static void eprSetByte(unsigned p,uint8_t v){put(p,v);}
    static void eprSetInt16(unsigned p,int16_t v){for(unsigned i=0;i<2;++i)put(p+i,uint16_t(v)>>(8*i));}
    static void eprSetInt32(unsigned p,int32_t v){for(unsigned i=0;i<4;++i)put(p+i,uint32_t(v)>>(8*i));}
    static void eprSetFloat(unsigned p,float v){uint32_t raw;memcpy(&raw,&v,4);eprSetInt32(p,raw);}
};
struct Controller {uint8_t slot;void invalidatePidCalibration(){invalidated|=1u<<slot;}};
static Controller e0{0},bed{2};
#if NUM_EXTRUDER==2
static Controller e1{1};
static Controller* tempController[]={&e0,&e1,&bed};
#else
static Controller* tempController[]={&e0,&bed};
#endif
struct Com {static void printErrorFLN(const char*){++errors;}};
void EEPROM::updateChecksum(){++checks;}
void EEPROM::readDataFromEEPROM(){++loads;}
'''
# GCode forward declaration is normally supplied by Repetier.h.
pre=pre.replace('#include "Eeprom.h"','struct GCode;\n#include "Eeprom.h"')
main=r'''
static void reset(){
    memset(bytes,0xa5,sizeof bytes);
    writes=loads=checks=errors=invalidated=0;ignore=-1;requireRevoke=true;
}
static bool protectedByte(int32_t p){
    return p==0||p==104||p==105||p==141||(p>=1978&&p<1987);
}
int main(){
    unsigned count=0,noops=0,rejected=0,revocations=0;
    for(uint8_t t=0;t<4;++t){
        const int width=t==0?1:t==1?2:4;
        for(int32_t p=-4;p<=2052;++p)for(bool identical:{false,true}){
            reset();GCode c;c.P=p;c.T=t;c.S=identical?int32_t(0xa5a5a5a5u):0;
            const uint32_t bits=identical?0xa5a5a5a5u:0;
            memcpy(&c.X,&bits,4);
            bool deny=p<0||p>2048-width;
            if(!deny)for(int i=0;i<width;++i)deny|=protectedByte(p+i);
            uint8_t slots=0;
            if(!deny&&!identical)for(int i=0;i<width;++i)slots|=slotFor(p+i);
            EEPROM::update(&c);
            if(deny){assert(errors==1&&writes==0&&checks==0&&loads==0&&invalidated==0);++rejected;}
            else{
                assert(errors==0&&checks==1&&loads==1);
                const uint8_t fitted=NUM_EXTRUDER==2?7:5;
                assert(invalidated==(slots&fitted));
                for(unsigned slot=0;slot<3;++slot)assert(bytes[1978+3*slot]==((slots&(1u<<slot))?0:0xa5));
                for(unsigned i=0;i<sizeof bytes;++i){
                    bool marker=false;
                    for(unsigned slot=0;slot<3;++slot)marker|=(slots&(1u<<slot))&&i==1978+3*slot;
                    const bool written=i>=unsigned(p)&&i<unsigned(p+width);
                    assert(bytes[i]==((marker||(written&&!identical))?0:0xa5));
                }
                if(identical)++noops;
                if(slots)++revocations;
            }
            ++count;
        }
        for(int32_t p:{INT32_MIN,INT32_MAX,-65536,-65535,65536,65536+217,65536+1978}){
            reset();GCode c;c.P=p;c.T=t;EEPROM::update(&c);
            assert(errors==1&&writes==0&&checks==0&&loads==0&&invalidated==0);++count;++rejected;
        }
    }
    // Marker readback failure must stop the write, even on an unfitted E1.
    for(unsigned slot=0;slot<3;++slot){
        reset();GCode c;c.P=slot==0?217:slot==1?317:107;c.S=0;
        ignore=1978+3*slot;EEPROM::update(&c);
        assert(errors==1&&writes==1&&checks==0&&loads==0&&bytes[c.P]==0xa5);
        assert(invalidated==((slot==2||slot<NUM_EXTRUDER)?(1u<<slot):0));++count;
    }
    // Missing payload keeps existing no-write reload behavior.
    reset();GCode c;c.P=217;c.s=false;EEPROM::update(&c);
    assert(!writes&&!invalidated&&!errors&&loads==1&&checks==1);++count;
    reset();c.T=3;c.x=false;EEPROM::update(&c);
    assert(!writes&&!invalidated&&!errors&&loads==1&&checks==1);++count;
    std::printf("M206 actual-source: %u cases, %u rejected, %u identical, %u thermal revocations, %d extruders\n",count,rejected,noops,revocations,NUM_EXTRUDER);
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"m206.cpp";exe=Path(d)/"m206"
    p.write_text(pre+source[a:b]+main)
    for n in (1,2):
        subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror",
                        "-DRF_THERMAL_EMERGENCY_EXPERIMENT=1",f"-DNUM_EXTRUDER={n}",
                        "-I",str(fw),str(p),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True)

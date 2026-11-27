#!/usr/bin/env python3
"""Actual experimental M206 method, signed address and thermal revoke replay."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
fw=root/"Repetier"
source=(fw/"Eeprom.cpp").read_text()
a=source.index("#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\n// Identify changed calibration identity/gains")
b=source.index("void EEPROM::restoreEEPROMExtruderSettingsFromConfiguration",a)
gcode=(fw/"gcode.h").read_text()
assert "int32_t P;" in gcode and "uint8_t T;" in gcode
pre=r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <limits>
using std::isfinite;
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3
#define DEVICE_TYPE_RF1000 1
#define DEVICE_TYPE_RF2000 2
#define DEVICE_TYPE_RF2000v2 3
#define UI_SET_MIN_HEATED_BED_TEMP 50
#define UI_SET_MAX_HEATED_BED_TEMP 160
#define UI_SET_MIN_EXTRUDER_TEMP 70
#define UI_SET_MAX_EXTRUDER_TEMP 270
#define MAX_FEEDRATE_X 150
#define MAX_FEEDRATE_Y 150
#define MAX_FEEDRATE_Z 9
#define ACCELERATION_MIN_XY 100
#define ACCELERATION_MAX_XY 6000
#define ACCELERATION_MIN_Z 5
#define ACCELERATION_MAX_Z 100
#define MIN_STEP_PACKING_MIN_INTERVAL 3300
#define MAX_STEP_PACKING_MIN_INTERVAL 5000
#define EMERGENCY_PAUSE_DIGITS_MIN -11000
#define EMERGENCY_PAUSE_DIGITS_MAX 11000
#define EMERGENCY_STOP_DIGITS_MIN -12500
#define EMERGENCY_STOP_DIGITS_MAX 12500
#define MOTOR_CURRENT_MIN 32
#define MOTOR_CURRENT_MAX {126,126,126,126,126}
#define PART_FAN_MODE_MAX 7
#define PROGMEM
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
static uint8_t slotFor(unsigned pos){
    if((pos>=108&&pos<=120)||pos==1922)return 4;
    if((pos>=218&&pos<=230)||pos==294)return 1;
    if((pos>=318&&pos<=330)||pos==394)return 2;
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
    static int32_t eprGetInt32(unsigned p){uint32_t v=0;for(unsigned i=0;i<4;++i)v|=uint32_t(eprGetByte(p+i))<<(8*i);return int32_t(v);}
    static int16_t eprGetInt16(unsigned p){return int16_t(uint16_t(eprGetByte(p))|(uint16_t(eprGetByte(p+1))<<8));}
    static float eprGetFloat(unsigned p){int32_t v=eprGetInt32(p);float f;memcpy(&f,&v,4);return f;}
    static void eprSetByte(unsigned p,uint8_t v){put(p,v);}
    static void eprSetInt16(unsigned p,int16_t v){for(unsigned i=0;i<2;++i)put(p+i,uint16_t(v)>>(8*i));}
    static void eprSetInt32(unsigned p,int32_t v){for(unsigned i=0;i<4;++i)put(p+i,uint32_t(v)>>(8*i));}
    static void eprSetFloat(unsigned p,float v){uint32_t raw;memcpy(&raw,&v,4);eprSetInt32(p,raw);}
};
struct Controller {uint8_t slot;void invalidatePidCalibration(){invalidated|=1u<<slot;}int maximumTargetC()const{return 110;}};
static Controller e0{0},bed{2};
static Controller& heatedBedController=bed;
#if NUM_EXTRUDER==2
static Controller e1{1};
static Controller* tempController[]={&e0,&e1,&bed};
#else
static Controller* tempController[]={&e0,&bed};
#endif
struct Com {static void printErrorFLN(const char*){++errors;}};
int EEPROM::getExtruderOffset(uint8_t i){return 200+100*i;}
void EEPROM::updateChecksum(){++checks;}
void EEPROM::readDataFromEEPROM(){++loads;}
'''
# GCode forward declaration is normally supplied by Repetier.h.
pre=pre.replace('#include "Eeprom.h"','struct GCode;\n#include "Eeprom.h"')
main=r'''
static void reset(){
    memset(bytes,0,sizeof bytes);
    writes=loads=checks=errors=invalidated=0;ignore=-1;requireRevoke=true;
}
static void denied(GCode c){
    uint8_t before[2048];memcpy(before,bytes,sizeof before);
    EEPROM::update(&c);
    assert(errors==1&&!writes&&!loads&&!checks&&!invalidated);
    assert(!memcmp(before,bytes,sizeof before));
}
static void admitted(GCode c,unsigned slots){
    EEPROM::update(&c);
    assert(!errors&&loads==1&&checks==1&&invalidated==slots);
}
int main(){
    unsigned count=0,noops=0,rejected=0;
    for(uint8_t type=0;type<=4;++type)for(int32_t pos=-4;pos<=2052;++pos){
        reset();GCode c;c.P=pos;c.T=type;
        bool listed=false;
        for(auto f:fields)if(f.pos==pos&&f.type==type)listed=true;
        if(listed){EEPROM::update(&c);assert(!errors&&!writes&&!loads&&!checks&&!invalidated);++noops;}
        else{denied(c);++rejected;}
        ++count;
    }
    for(uint8_t type=0;type<4;++type)for(int32_t pos:{INT32_MIN,INT32_MAX,-65536,-65535,65536,65536+217,65536+1978}){
        reset();GCode c;c.P=pos;c.T=type;denied(c);++count;++rejected;
    }
    for(unsigned missing=0;missing<4;++missing){
        reset();GCode c;c.P=107;c.T=0;c.S=90;
        if(missing==0)c.p=false;
        if(missing==1)c.t=false;
        if(missing==2)c.s=false;
        if(missing==3){c.P=108;c.T=3;c.x=false;}
        denied(c);++count;
    }
    for(unsigned slot=0;slot<3;++slot){
        if(slot==1&&NUM_EXTRUDER==1)continue;
        const unsigned base=slot==0?200:slot==1?300:0;
        const unsigned drive=slot==2?107:base+17,p=slot==2?108:base+18;
        const unsigned sensor=slot==2?1922:base+94,cap=slot==2?120:base+30;
        const unsigned marker=1978+slot*3;
        for(int32_t value:{-1,0,1,80,120,255,256,65536}){
            reset();bytes[marker]=0xa9;bytes[drive]=100;GCode c;c.P=drive;c.S=value;
            if(value<1||value>255)denied(c);else{admitted(c,0);assert(bytes[marker]==0xa9&&bytes[drive]==value);}
            ++count;
        }
        for(int32_t value:{-1,0,3,7,8,13,39,53,60,100,101,255,256}){
            reset();bytes[marker]=0xa9;bytes[sensor]=14;GCode c;c.P=sensor;c.S=value;
            const bool valid=value==0||value==3||value==8||value==13||value==53||value==60||value==100;
            if(valid){admitted(c,1u<<slot);assert(bytes[marker]==0&&bytes[sensor]==value);}else denied(c);
            ++count;
        }
        for(int32_t value:{-1,0,1,39,40,200,255,256}){
            reset();bytes[marker]=0xa9;bytes[cap]=128;GCode c;c.P=cap;c.S=value;
            if(value==0||(value>=40&&value<=255)){admitted(c,1u<<slot);assert(bytes[marker]==0&&bytes[cap]==value);}else denied(c);
            ++count;
        }
        for(float value:{-1.f,0.f,.1f,1000.f,1001.f,std::numeric_limits<float>::infinity(),std::numeric_limits<float>::quiet_NaN()}){
            reset();bytes[marker]=0xa9;GCode c;c.P=p;c.T=3;c.X=value;
            if(value==0){EEPROM::update(&c);assert(!errors&&!writes&&!loads&&!invalidated);}
            else if(value>0&&value<=1000){admitted(c,1u<<slot);assert(bytes[marker]==0);}
            else denied(c);
            ++count;
        }
        reset();bytes[marker]=0xa9;GCode c;c.P=sensor;c.S=3;ignore=marker;
        EEPROM::update(&c);assert(errors==1&&writes==1&&!checks&&!loads&&!bytes[sensor]&&invalidated==(1u<<slot));++count;
    }
    reset();GCode c;c.P=125;c.T=2;c.S=1;denied(c);++count;
    reset();c.P=129;c.T=3;c.X=1;denied(c);++count;
    for(int32_t value:{-1,0,49,50,100,110,111,160,161,32768}){
        reset();c=GCode{};c.P=1970;c.T=1;c.S=value;
        if(value==0){EEPROM::update(&c);assert(!errors&&!writes&&!loads);}
        else if(value>=50&&value<=110)admitted(c,0);else denied(c);
        ++count;
    }
    // Real cross-field fixtures, not the all-zero schema inventory.
    for(float value:{-1.f,1.f,20.f,100.f,150.f,151.f}){
        reset();HAL::eprSetFloat(15,100);HAL::eprSetFloat(27,20);writes=0;
        c=GCode{};c.P=15;c.T=3;c.X=value;
        if(value==100){EEPROM::update(&c);assert(!errors&&!writes&&!loads);}
        else if(value>=20&&value<=150)admitted(c,0);else denied(c);
        ++count;
    }
    for(int32_t value:{0,1,16,84,85,100,255}){
        reset();bytes[1961]=10;bytes[1962]=100;
        c=GCode{};c.P=1961;c.S=value;
        if(value>=1&&value<=84)admitted(c,0);else denied(c);
        ++count;
    }
    for(int32_t value:{-1,0,3299,3300,5000,5001}){
        reset();bytes[1964]=1;
        c=GCode{};c.P=1964;c.T=1;c.S=value;
        if(value>=3300&&value<=5000)admitted(c,0);else denied(c);
        ++count;
    }
    for(int32_t value:{-11001,-11000,-1,0,1,11000,11001}){
        reset();HAL::eprSetInt32(1910,-100);HAL::eprSetInt32(1914,100);writes=0;
        c=GCode{};c.P=1910;c.T=2;c.S=value;
        if(value>=-11000&&value<=100)admitted(c,0);else denied(c);
        ++count;
    }
    std::printf("M206 exact fields: %u cases, %u absent/wrong-type denied, %u exact no-ops, %d extruders\n",count,rejected,noops,NUM_EXTRUDER);
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"m206.cpp";exe=Path(d)/"m206"
    golden=(root/"tests/eeprom-report-golden.tsv").read_text().splitlines()
    for model,n,board in (("RF1000",1,1),("RF2000",2,2),("RF2000",2,3),("RF2000-X44",1,2)):
        rows=[line.split("\t") for line in golden if line.startswith(model+"\t")]
        fixture="struct Field {int pos,type;}; static const Field fields[]={"+",".join("{"+r[1]+","+r[2]+"}" for r in rows)+"};\n"
        assert rows
        p.write_text(pre+source[a:b]+fixture+main)
        flags=[f"-DNUM_EXTRUDER={n}",f"-DMOTHERBOARD={board}","-DCASE_FAN_PIN=5"]
        if board!=1:flags.append("-DRGB_LIGHT_R_PIN=1")
        subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror",
                        "-DRF_THERMAL_EMERGENCY_EXPERIMENT=1",*flags,
                        "-I",str(fw),str(p),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True)

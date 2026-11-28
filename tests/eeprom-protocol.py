#!/usr/bin/env python3
"""Replay the standard M205/M206 entrypoints using the real checked session.

The storage transport and live hardware owners are modeled; the session,
typed fields, codec, report iteration and admission/publication callbacks
are actual production code. Communication-format.py tests real Com formatting.
"""
from pathlib import Path
import re
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
fw=root/"Repetier"
runtime=(fw/"EepromZoneRuntime.cpp").read_text()
comm=(fw/"Communication.cpp").read_text()
inventory=(fw/"EepromZoneHostFields.h").read_text()
def section(source,a,b):
    start=source.index(a)
    return source[start:source.index(b,start+len(a))]
labels=set(re.findall(r"Com::(t\w+)",inventory))|{"tEPR0","tEPR1","tEPR2","tEPR3","tExtrDot"}
strings=dict(re.findall(r'^FSTRINGVALUE\(Com::(t\w+), ("(?:[^"\\]|\\.)*")\)',comm,re.M))
assert labels<=strings.keys(), labels-strings.keys()
definitions="\n".join(f"const char {k}[]={strings[k]};" for k in sorted(labels))
report=section(runtime,"const char zonePreheatABed","}\nbool EEPROM::settingsReady()")
report+=section(runtime,"void EEPROM::writeSettings()","void EEPROM::storePidCalibration(")
report=report.replace("reinterpret_cast<PGM_P>(pgm_read_word(&field.label))","pgm_read_word(&field.label)")
owners=section(runtime,"bool driveOnly(","const char zonePreheatABed")
pre=r'''
#include <cassert>
#include <array>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <limits>
#include <initializer_list>
#define __AVR_ATmega2560__ 1
#include "BoardConfiguration.h"
#include "EepromZoneDefaults.h"
#include "EepromZoneSession.h"
#undef str
#undef xstr
using namespace EepromZones;
#define PROGMEM
#define PSTR(x) x
using PGM_P=const char*;
template<class T>T pgm_read_word(const T* p){return *p;}
uint8_t pgm_read_byte(const uint8_t* p){return *p;}
struct Device {
    std::array<uint8_t,4096> bytes;
    unsigned programs=0, reads=0;
    uint32_t clock=0;
    bool foreground()const{return true;}
    bool ready()const{return true;}
    uint32_t now()const{return clock;}
    uint8_t readReady(uint16_t pos){assert(pos>=Base&&pos<End);++reads;return bytes[pos];}
    void updateReady(uint16_t pos,uint8_t v){assert(pos>=Base&&pos<End);assert(bytes[pos]!=v);++programs;bytes[pos]=v;}
    void service(){++clock;}
} dev;
bool busy=false,loaded=true,lastOk=false,settingsFaultLatched=false;
Session<Device> session(){return Session<Device>(dev,busy,configuredProfile());}
bool accept(Result r){return lastOk=r==Result::Ok;}
bool readBlockChecked(uint8_t id,uint8_t* data,uint32_t& alerts){
    return loaded&&!settingsFaultLatched&&accept(session().read(id,data,length(id),alerts));
}
static std::string output;
static unsigned progressCalls=0,errors=0,admissions=0,applies=0,driveApplies=0,revocations=0,finishes=0;
static bool idleAllowed=true,cold=true,autotune=false,terminal=false;
void progress(){++progressCalls;}
namespace Com {
LABELS
template<class T> void print(T v){output+=std::to_string(v);}
void print(char c){output+=c;}
template<class T>void printF(const char* label,T v){output+=label;print(v);}
void printFloat(float v,uint8_t digits){char b[80];snprintf(b,sizeof b,"%.*f",int(digits),double(v));output+=b;}
void printFLN(const char* label){output+=label;output+="\r\n";}
void printFLN(const char*,int){}
void printErrorFLN(const char*){++errors;}
}
namespace Printer {
bool blockAll=false;
struct Job{bool emergencyLocked(){return terminal;}}printJob;
void disableAllSteppersNow(){assert(idleAllowed&&cold);++admissions;}
void setHomed(bool x,bool y,bool z){assert(!x&&!y&&!z);}
}
struct TemperatureController{
    static bool autotuneActive(){return autotune;}
    void invalidatePidCalibration(){++revocations;}
} controllers[3];
TemperatureController& controller(uint8_t id){return controllers[id==BedPid?NUM_EXTRUDER:id-Hotend0];}
bool idle(){return idleAllowed;}
bool heatersOff(){return cold;}
bool invalidateMatrices(){return true;}
namespace AccessoryOutputs {
void initializeBoardOutputs(){}
void initializeCaseLight(){}
}
void failSettings(Result){settingsFaultLatched=true;++errors;}
struct LiveSettings{
    bool applyDriveLimits(uint8_t,const uint8_t*,uint8_t,uint32_t)=delete;
    bool applyDriveLimits(uint8_t id,const uint8_t* d,uint8_t n,const Patch* p,uint8_t count){
        assert(busy&&count==1&&p&&validBlock(id,d,n));++driveApplies;return true;
    }
    bool applyBlock(uint8_t id,const uint8_t* d,uint8_t n,uint32_t){
        assert(busy&&validBlock(id,d,n)&&!terminal);++applies;return true;
    }
    void finishMotion(){++finishes;}
};
struct GCode{
    int32_t P=0,S=0;uint8_t T=0;float X=0;
    bool p=true,t=true,s=true,x=true;
    bool hasP()const{return p;}bool hasT()const{return t;}bool hasS()const{return s;}bool hasX()const{return x;}
};
struct EEPROM{static void writeSettings();static void update(GCode*);};
'''
main=r'''
struct Expected {uint8_t wire;uint16_t pos;const char* label;uint8_t digits;bool writable;};
static const Expected fields[]={
#define RF_ZONE_FIELD_END ,
#define RF_ZONE_FIELD(type,start,label,digits,writable) {type,start,label,digits,writable}
#include "EepromZoneHostFields.h"
#undef RF_ZONE_FIELD
#undef RF_ZONE_FIELD_END
};
static void verifyReport(FactoryDefaults& defaults) {
    std::ostringstream expected;
    unsigned count=0;
    for(const auto& f:fields){
        const uint8_t id=blockOf(f.pos);
        uint8_t data[MaxPayload];assert(defaults(id,data,length(id)));
        const uint8_t* p=data+f.pos-address(id)-PayloadOffset;
        expected<<"EPR:"<<unsigned(f.wire)<<" "<<f.pos<<" ";
        if(f.wire==0)expected<<unsigned(*p);
        else if(f.wire==1){int16_t v;memcpy(&v,p,2);expected<<v;}
        else if(f.wire==2){int32_t v;memcpy(&v,p,4);expected<<v;}
        else {float v;memcpy(&v,p,4);expected<<std::fixed<<std::setprecision(f.digits)<<v;}
        expected<<" ";
        if(id==Extruder0Motion||id==Extruder1Motion||id==Hotend0||id==Hotend1)
            expected<<"Extr."<<unsigned(id>=Hotend0?id-Hotend0+1:id-Extruder0Motion+1)<<" ";
        expected<<f.label<<"\r\n";
        ++count;
    }
    auto before=dev.bytes;
    unsigned writes=dev.programs;
    EEPROM::writeSettings();
    if(output!=expected.str()){
        fprintf(stderr,"M205 expected:\n%s\nactual:\n%s",expected.str().c_str(),output.c_str());abort();
    }
    assert(progressCalls==count&&dev.bytes==before&&dev.programs==writes);
    printf("M205 exact standard report: %u fields PASS\n",count);
}
static void rejected(GCode c){
    auto old=dev.bytes;unsigned w=dev.programs,a=applies,d=driveApplies,r=revocations;
    EEPROM::update(&c);
    assert(!lastOk&&dev.bytes==old&&dev.programs==w&&applies==a&&driveApplies==d&&revocations==r);
}
int main(){
    dev.bytes.fill(0xff);FactoryDefaults defaults(1.0f,5,true,false);
    Inspection state;assert(session().restore(defaults,state)==Result::Ok);
    verifyReport(defaults);
    assert(session().acknowledge(AllAlerts&~blockBit(BedSafety))==Result::Ok);
    GCode c;
    unsigned denied=0;
    for(int32_t pos=0;pos<4096;++pos)for(uint8_t wire=0;wire<5;++wire){
        if(hostAccess(pos,wire)!=HostAccess::Unknown)continue;
        c.P=pos;c.T=wire;rejected(c);++denied;
    }
    c.T=0;
    for(int32_t pos:{-1,65536+int(address(Field::F_E0_SENSOR_TYPE)),INT32_MAX,INT32_MIN}){
        c.P=pos;rejected(c);++denied;
    }
    c.P=address(Field::F_E0_DRIVE_MAX);c.S=73;
    idleAllowed=false;cold=false;
    EEPROM::update(&c);assert(lastOk&&driveApplies==1&&applies==0&&revocations==0);
    auto writes=dev.programs;
    EEPROM::update(&c);assert(lastOk&&driveApplies==1&&dev.programs==writes);
    // Pick a different value even if a future factory profile changes.
    uint8_t data[MaxPayload];uint32_t alerts=AllAlerts;
    assert(session().read(Hotend0,data,length(Hotend0),alerts)==Result::Ok);
    c.P=address(Field::F_E0_PID_MAX);
    c.S=BlockDecoder(Hotend0,data,length(Hotend0)).u8(Field::F_E0_PID_MAX)==80?81:80;
    rejected(c);
    idleAllowed=cold=true;
    // Test actual M206 wrapper's sensor/cap calibration revocation path.
    EEPROM::update(&c);assert(lastOk&&applies==1&&revocations==1);
    c.S=82;terminal=true;rejected(c);terminal=false;
    autotune=true;rejected(c);autotune=false;
    c.T=3;c.P=address(Field::F_E0_PID_PGAIN);
    c.X=std::numeric_limits<float>::quiet_NaN();rejected(c);
    assert(session().read(Hotend0,data,length(Hotend0),alerts)==Result::Ok);
    c.X=BlockDecoder(Hotend0,data,length(Hotend0)).f32(Field::F_E0_PID_PGAIN)==1.5f?2.f:1.5f;rejected(c);
    c.T=0;c.P=address(Field::F_RF_MOTOR_CURRENT);c.S=1;rejected(c);
    c.P=address(Field::F_NEW_BED_EMERGENCY_C);c.S=100;rejected(c);
    c.P=address(Field::F_E0_PID_MAX);c.S=83;c.s=false;rejected(c);
    assert(!settingsFaultLatched);
    printf("M206 standard wrapper: %u unknown/wrong-type address cases, idle/hot/terminal/read-only admission PASS\n",denied);
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"protocol.cpp";exe=Path(d)/"protocol"
    p.write_text(pre.replace("LABELS",definitions)+owners+report+main)
    for model,extra in (("RF1000",[]),("RF2000",[]),("RF2000v2",[]),("RF2000",["-DNUM_EXTRUDER=1","-DEXT0_EXTRUDER_COOLER_PIN=X44_PIN"])):
        subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror","-I",str(fw),
                        "-DMOTHERBOARD=DEVICE_TYPE_"+model,*extra,str(p),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True,timeout=20)

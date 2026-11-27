#!/usr/bin/env python3
"""Exercise actual checksum and post-checksum configuration continuations.
Storage writes before the checksum and peripheral operations are mocked.
"""
from pathlib import Path
import subprocess
import tempfile

fw=Path(__file__).resolve().parents[1]/"Repetier"
def extract(name,a,b):
    s=(fw/name).read_text()
    start=s.index(a)
    return s[start:s.index(b,start)]

checksum=extract("Eeprom.cpp","void EEPROM::updateChecksum()","void EEPROM::storeMicrostepConfiguration(")
checksum+=extract("Eeprom.cpp","uint8_t EEPROM::computeChecksum()","void EEPROM::writeExtruderPrefix(")
loader=extract("Eeprom.cpp","void EEPROM::readDataFromEEPROM()","    bool change = readPreheatProfiles();")
loader+="    bool change=needsUpdate; uint8_t version=storedVersion;\n"
loader+=extract("Eeprom.cpp","    if (change)\n        EEPROM::updateChecksum();","} // readDataFromEEPROM")+"}\n"
commands=extract("GCodeSystem.cpp","void restoreConfigurationDefaults()","void accessPanel(")
prelude=r"""
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
using millis_t=uint32_t;
#define PERIODICAL_ACTIONS_CALL_INTERVAL 10
#define EEPROM_OFFSET 0
#define EPR_INTEGRITY_BYTE 104
#define EEPROM_PROTOCOL_VERSION 44
#define PSTR(x) x
#define UI_TEXT_CONFIGURATION "configuration"
#define UI_TEXT_RESTORE_DEFAULTS "defaults"
#define UI_TEXT_OK "ok"
#define UI_STATUS(x) normalOutput()
enum {Processing};
static uint8_t mem[2048],storedVersion;
static bool locked,needsUpdate;
static uint32_t now;
static unsigned visits,injectAt,reads,integrityWrites,applies,outputs,stores,restores;
static void normalOutput(){assert(!locked);++outputs;}
namespace HAL {
uint32_t timeInMilliseconds(){return now;}
uint8_t eprGetByte(unsigned i){assert(i<2048);++reads;++now;return mem[i];}
void eprSetByte(unsigned i,uint8_t v){assert(i==104);++integrityWrites;mem[i]=v;}
}
namespace Commands {
enum class MotionWait{AllOutputs}; enum class PauseWait{Resume};
bool waitForMotion(MotionWait,PauseWait){return !locked;}
void checkForPeriodicalActions(int){if(++visits==injectAt)locked=true;}
}
namespace Printer {
struct Job{bool emergencyLocked()const{return locked;}}printJob;
bool debugInfo(){return true;}
void updateDerivedParameter(){assert(!locked);++applies;}
void applyAdvanceSettings(){assert(!locked);++applies;}
}
namespace Extruder {
void applyCurrentSettings(){assert(!locked);++applies;}
void initHeatedBed(){assert(!locked);++applies;}
}
namespace Com {
const char *tEPRProtocolChanged="protocol",*tConfigStoredEEPROM="stored",*tConfigLoadedEEPROM="loaded";
void printInfoF(const char*){normalOutput();}
void printInfoFLN(const char*){normalOutput();}
}
void showInformation(const char*,const char*,const char*){normalOutput();}
struct EEPROM {
static uint8_t computeChecksum();static void updateChecksum();static void readDataFromEEPROM();
static void storeDataIntoEEPROM(bool){assert(!locked);++stores;updateChecksum();}
static void restoreEEPROMSettingsFromConfiguration(){assert(!locked);++restores;}
};
"""
main=r"""
static void reset(unsigned n,uint32_t clock,bool change=true,uint8_t version=44){
    now=clock;locked=false;injectAt=n;needsUpdate=change;storedVersion=version;
    visits=reads=integrityWrites=applies=outputs=stores=restores=0;
    for(unsigned i=0;i<2048;++i)mem[i]=uint8_t(i*17+3);
    mem[104]=0;
}
static void validChecksum(){
    uint8_t sum=0;for(unsigned i=0;i<2048;++i)if(i!=104)sum+=mem[i];
    assert(mem[104]==sum&&reads==2047&&integrityWrites==1);
}
int main(){
    unsigned faults=0;
    for(uint32_t clock:{0u,0xffffff00u}){
        const unsigned n=visits;
        reset(0,clock);restoreConfigurationDefaults();validChecksum();assert(outputs==2&&restores==1&&stores==1);
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        for(unsigned i=1;i<=n;++i){
            reset(i,clock);restoreConfigurationDefaults();validChecksum();assert(locked&&outputs==0&&restores==1&&stores==1);++faults;
        }
        reset(0,clock);locked=true;EEPROM::readDataFromEEPROM();assert(!reads&&!applies&&!stores&&!outputs);
#endif
        assert(n>0);
    }
    std::printf("EEPROM checksum and configuration unwind: %u injected faults; healthy paths passed\n",faults);
}
"""
with tempfile.TemporaryDirectory() as d:
    src=Path(d)/"replay.cpp";exe=Path(d)/"replay"
    src.write_text(prelude+checksum+loader+commands+main)
    for flags in ([],["-DRF_THERMAL_EMERGENCY_EXPERIMENT=1"]):
        subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror",*flags,str(src),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True)

#!/usr/bin/env python3
"""Actual standard EEPROM load/reset continuation and thermal unwind.

The storage session transport is modeled at its boundaries here. The real
checked session, power cuts and codecs are independently exercised by
eeprom-storage.py. No successful publication may follow an injected fault.
"""
from pathlib import Path
import subprocess
import tempfile
fw=Path(__file__).resolve().parents[1]/"Repetier"
def extract(file,a,b):
    s=(fw/file).read_text()
    start=s.index(a)
    return s[start:s.index(b,start)]
actual=extract("EepromZoneRuntime.cpp","bool idle() {","bool persist(const Patch*")
actual+=extract("EepromZoneRuntime.cpp","bool loadFull(","bool readBlockChecked(")
actual+=extract("EepromZoneRuntime.cpp","bool EEPROM::settingsReady()","void EEPROM::reportRecovery()")
actual+=extract("EepromZoneRuntime.cpp","void EEPROM::restoreEEPROMSettingsFromConfiguration()","void EEPROM::writeSettings()")
actual+=extract("GCodeSystem.cpp","void restoreConfigurationDefaults()","void accessPanel(")
pre=r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#define PSTR(x) x
#define UI_TEXT_CONFIGURATION "configuration"
#define UI_TEXT_RESTORE_DEFAULTS "defaults"
#define UI_TEXT_OK "ok"
static bool terminal=false,block=false,resetWithAlerts=true;
static unsigned injection=0,steps=0,applied=0,motionFinished=0,diagnostics=0,normal=0,cleared=0,initOutputs=0,restoreCalls=0,loadCalls=0,matrixInvalidations=0;
static void faultPoint(){if(++steps==injection)terminal=true;}
static void normalOutput(){assert(!terminal);++normal;}
#define UI_STATUS(x) do { if(!terminal) ++diagnostics; } while(0)
enum class Result {Ok,Invalid,TransportFault};
constexpr uint8_t Geometry=1,BedSafety=11,Last=21,NUM_TEMPERATURE_LOOPS=3;
constexpr uint32_t blockBit(uint8_t id){return uint32_t(1)<<id;}
constexpr uint32_t AllAlerts=(blockBit(Last+1)-1),OrdinaryAlerts=AllAlerts&~blockBit(BedSafety);
constexpr float defaultHeatBedScanLiftMm=1;
constexpr int defaultZOffsetScanIndex=5;
constexpr bool KOSYS_GCODE=true,FEEDRATE_DIRECTCONFIG=false;
struct Inspection{uint32_t alerts=0;};
struct FactoryDefaults{FactoryDefaults(float,int,bool,bool){}};
static bool loaded,outputsAdmitted,lastOk,settingsFaultLatched;
static uint32_t pending;
namespace Printer {
bool& blockAll=block;
struct Job {bool isBusy(){return false;}bool commandsBlocked(){return terminal;}bool emergencyLocked(){return terminal;}}printJob;
struct Pause{bool isPaused(){return false;}}pause;
bool needsCMPwait(){return false;}
void lockMovement(){}
void disableAllSteppersNow(){assert(!terminal);}
void setHomed(bool x,bool y,bool z){assert(!x&&!y&&!z&&!terminal);}
}
namespace PrintLine {bool hasLines(){return false;}bool directMoveActive(){return false;}}
bool isAnyScanRunning(){return false;}
struct TemperatureController{
    float targetTemperatureC=0;
    uint8_t cutoff=0;
    void setTargetTemperature(float v){assert(v==0);targetTemperatureC=v;++cleared;}
    bool setBedEmergencyCutoff(uint8_t v){
        if(v&&terminal)return false;
        cutoff=v;return true;
    }
    static bool autotuneActive(){return false;}
} controllers[3];
TemperatureController* tempController[]={&controllers[0],&controllers[1],&controllers[2]};
TemperatureController& heatedBedController=controllers[2];
namespace HAL {uint8_t outputDuty(uint8_t){return 0;}}
namespace Com {
void printErrorFLN(const char*){++diagnostics;}
void printFLN(const char*,int){++diagnostics;}
}
namespace AccessoryOutputs {
void initializeBoardOutputs(){assert(!terminal&&!outputsAdmitted);++initOutputs;}
void initializeCaseLight(){assert(!terminal&&!outputsAdmitted);++initOutputs;}
}
struct LiveSettings{
    bool applyBlock(uint8_t,const uint8_t*,uint8_t,uint32_t){assert(!terminal&&!outputsAdmitted);++applied;return true;}
    void finishMotion(){assert(!terminal&&!outputsAdmitted);++motionFinished;faultPoint();}
    uint8_t bedCutoff(){return resetWithAlerts?0:100;}
};
struct Session{
    Result restore(FactoryDefaults&,Inspection& s,bool reset){
        ++restoreCalls;assert(!outputsAdmitted&&!loaded);
        s.alerts=reset&&resetWithAlerts?AllAlerts:0;
        for(unsigned i=0;i<5;++i)faultPoint();
        return Result::Ok;
    }
    template<class Consumer> Result load(Consumer& consumer,Inspection& s){
        ++loadCalls;
        for(uint8_t id=1;id<=Last;++id){
            faultPoint();
            const uint8_t data=0;
            if(!consumer(id,&data,1,s.alerts))return Result::Invalid;
        }
        return Result::Ok;
    }
};
Session session(){return {};}
bool invalidateMatrices(){++matrixInvalidations;faultPoint();return true;}
struct EEPROM{
    static bool settingsReady();static bool recoveryPending();static bool lastOperationSucceeded();
    static const void* recoveryDialogTag();static void restoreEEPROMSettingsFromConfiguration();
    static void reportRecovery(){assert(!terminal);++diagnostics;}
};
namespace Commands {
enum class MotionWait{AllOutputs};enum class PauseWait{Resume};
bool waitForMotion(MotionWait,PauseWait){return !terminal;}
}
void showInformation(const char*,const char*,const char*){normalOutput();}
'''
# UI_STATUS in failSettings is a diagnostic even if terminal; the wrapper's
# normal success status must be suppressed. Track the success string exactly.
pre=pre.replace('#define UI_STATUS(x) do { if(!terminal) ++diagnostics; } while(0)',
'''#include <cstring>
#define UI_STATUS(x) do { if(!std::strcmp(x,UI_TEXT_RESTORE_DEFAULTS))normalOutput();else ++diagnostics; } while(0)''')
main=r'''
static void reset(unsigned at,bool alerts){
    terminal=block=false;loaded=outputsAdmitted=lastOk=true;settingsFaultLatched=false;
    pending=0;injection=at;steps=applied=motionFinished=diagnostics=normal=cleared=initOutputs=restoreCalls=loadCalls=matrixInvalidations=0;
    resetWithAlerts=alerts;heatedBedController.cutoff=100;
}
int main(){
    reset(0,false);restoreConfigurationDefaults();
    assert(EEPROM::settingsReady()&&lastOk&&normal==2&&applied==Last&&motionFinished==1&&initOutputs==2);
    const unsigned noAlerts=steps;
    for(unsigned at=1;at<=noAlerts;++at){
        reset(at,false);restoreConfigurationDefaults();
        assert(terminal&&!outputsAdmitted&&!loaded&&!lastOk&&!normal&&!initOutputs);
        assert(heatedBedController.cutoff==0);
    }
    reset(0,true);restoreConfigurationDefaults();
    assert(!EEPROM::settingsReady()&&lastOk&&EEPROM::recoveryPending()&&!normal&&matrixInvalidations==1);
    const unsigned withAlerts=steps;
    for(unsigned at=1;at<=withAlerts;++at){
        reset(at,true);restoreConfigurationDefaults();
        assert(terminal&&!outputsAdmitted&&!loaded&&!lastOk&&!normal&&!initOutputs);
        assert(heatedBedController.cutoff==0);
    }
    reset(0,false);terminal=true;restoreConfigurationDefaults();assert(!restoreCalls&&!normal);
    reset(0,false);terminal=true;EEPROM::restoreEEPROMSettingsFromConfiguration();assert(!restoreCalls&&!normal&&!lastOk);
    printf("Standard EEPROM configuration unwind: %u fault injection points, recovery and success paths PASS\n",noAlerts+withAlerts);
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"unwind.cpp";exe=Path(d)/"unwind"
    p.write_text(pre+actual+main)
    subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror",str(p),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

// Host-only zoned EEPROM owner replay, with actual LiveSettings and preheat methods.
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <array>
#include <string>
#include "Constants.h"
#include "Configuration.h"
#if MOTHERBOARD == DEVICE_TYPE_RF1000
#include "RF1000.h"
#else
#include "RF2000.h"
#endif
#define EXT0_HOTEND_PROFILE DEFAULT_HOTEND_PROFILE
#define EXT1_HOTEND_PROFILE DEFAULT_HOTEND_PROFILE
#define EXT0_TEMPSENSOR_TYPE DEFAULT_HOTEND_SENSOR_TYPE
#define EXT1_TEMPSENSOR_TYPE DEFAULT_HOTEND_SENSOR_TYPE
#define __AVR_ATmega2560__ 1
#include "pins.h"
#include "ManualMotion.h"
#include "MachineInactivity.h"
#include "PreheatProfiles.h"
#include "ZOffset.h"
#include "WobbleCorrection.h"
#include "PressureProtection.h"
using millis_t = uint32_t;
using PGM_P = const char*;
#define PROGMEM
static uint16_t pgm_read_word(const uint16_t* p) { return *p; }
#define PSTR(s) s
#define EEPROM_OFFSET 0
#define DRV8711_NUM_CHANNELS 5
#define PART_FAN_MODE_MAX 7
#define PART_FAN_MODE_PWM 0
#define PART_FAN_MODE_PDM 1
#define KOSYS_GCODE true
#define FEEDRATE_DIRECTCONFIG false
#define COMPENSATION_MATRIX_MAX_X long((X_MAX_LENGTH_PRINT - heatBedScanXStartMm - heatBedScanXEndMm) / 10 + 4)
#define COMPENSATION_MATRIX_MAX_Y long((Y_MAX_LENGTH - 30 - 5) / 10 + 4)
constexpr int heatBedScanXStartMm = NUM_EXTRUDER == 2 ? 0 : 15, heatBedScanXEndMm = NUM_EXTRUDER == 2 ? 0 : 5, heatBedScanYStartMm = 30;
constexpr float defaultHeatBedScanLiftMm = 1.0f;
constexpr uint8_t defaultZOffsetScanIndex = 5;
#define constrain(x,a,b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
namespace RMath { template<class A,class B> auto min(A a,B b) -> decltype(a+b) {return a<b?a:b;} }
struct GCode { int T=0,P=0; int32_t S=0; float X=0; bool hasT(){return true;} bool hasP(){return true;} bool hasS(){return true;} bool hasX(){return true;} };
#include "Eeprom.h"
static std::array<uint8_t,2048> image{};
static bool keys=false;
#define READ(pin) (!keys)
namespace HAL {
    template<class T> T get(int p) {assert(p>=0 && size_t(p)+sizeof(T)<=image.size()); T v; memcpy(&v,image.data()+p,sizeof v); return v;}
    template<class T> void set(int p,T v){assert(p>=0 && size_t(p)+sizeof(T)<=image.size());memcpy(image.data()+p,&v,sizeof v);}
    uint8_t eprGetByte(int p){return get<uint8_t>(p);} int16_t eprGetInt16(int p){return get<int16_t>(p);} int32_t eprGetInt32(int p){return get<int32_t>(p);} float eprGetFloat(int p){return get<float>(p);}
    void eprSetByte(int p,uint8_t v){set(p,v);} void eprSetInt16(int p,int16_t v){set(p,v);} void eprSetInt32(int p,int32_t v){set(p,v);} void eprSetFloat(int p,float v){set(p,v);}
    millis_t timeInMilliseconds(){static millis_t t=0;return ++t;}
    uint8_t mode=0,divisor=PART_FAN_DEFAULT_PWM_SPEED_DIVISOR,lo=PART_FAN_PWM_MIN,hi=PART_FAN_PWM_MAX;
    uint8_t partFanMode(){return mode;} uint8_t partFanDivisor(){return divisor;} uint8_t partFanMinDuty(){return lo;} uint8_t partFanMaxDuty(){return hi;} void setPartFanDutyRange(uint8_t l,uint8_t h){lo=l;hi=h;}
}
constexpr int Processing=0;
namespace Commands {void checkForPeriodicalActions(int){} void adjustFanFrequency(uint8_t v){HAL::divisor=v;} void adjustFanMode(uint8_t v){HAL::mode=v;}}
namespace Com { const char* tEPRConfigResetDefaults="defaults"; const char* tEPRProtocolChanged="version"; void printInfoFLN(const char*){} void printErrorFLN(const char*){} void printF(const char*){} void printFLN(const char*){} void println(){} }
#define UI_TEXT_CONFIGURATION "config"
#define UI_TEXT_UPDATE "update"
#define UI_TEXT_RESTORE_DEFAULTS "defaults"
#define UI_TEXT_FAIL "fail"
#define UI_TEXT_EXTRUDER1_ADD "add"
#define UI_TEXT_EXTRUDER1_REMOVED "remove"
#define UI_TEXT_UPDATE_MATRIX "matrix"
void showInformation(const char*,const char*,const char*){}
struct Controller {
    uint8_t pidDriveMax=0,pidDriveMin=0,pidMax=0,sensorType=0,tunedSensorType=0,tunedPowerLimit=0;
    float pidPGain=0,pidIGain=0,pidDGain=0;
    bool pidCalibrationValid() const {
        return sensorType != 0 && tunedSensorType == sensorType && tunedPowerLimit == pidMax
            && pidMax >= 40 && pidPGain > 0 && pidPGain <= 1000 && pidIGain > 0 && pidIGain <= 1000 && pidDGain >= 0 && pidDGain <= 100000;
    }
    void invalidatePidCalibration(){tunedSensorType=tunedPowerLimit=0;}
    float target=0;
    void setTargetTemperature(float v){target=v;}
    void updateTempControlVars(){}
} heatedBedController;
using TemperatureController=Controller;
struct Extruder {float stepsPerMM=0,maxFeedrate=0,maxEJerk=0,maxAcceleration=0,offsetMM[2]{},advanceL=0; uint8_t coolerSpeed=0; Controller tempControl; static Extruder* current; static void applyCurrentSettings(){} static void initHeatedBed(){} } extruder[2]; Extruder* Extruder::current=extruder;
#define NUM_TEMPERATURE_LOOPS (NUM_EXTRUDER+1)
Controller* tempController[NUM_TEMPERATURE_LOOPS] = {&extruder[0].tempControl,
#if NUM_EXTRUDER>1
    &extruder[1].tempControl,
#endif
    &heatedBedController};
struct Point {uint8_t x,y;}; Point point{}; float lift=defaultHeatBedScanLiftMm; void restoreZOffsetScanPoint(Point p){point=p;} Point zOffsetScanPoint(){return point;} float scanStartLiftMm(){return lift;} void setScanStartLiftMm(float v){lift=v;}
uint8_t drv8711Axis_2_InitMicrosteps(int){return 16;} uint8_t drv8711MicroSteps_2_ModeValue(int){return 4;}
namespace Drv8711 {
    uint8_t modes[5]{}, currents[5]{};
    uint8_t microstepMode(uint8_t i){return modes[i];}
    void setMicrostepMode(uint8_t i,uint8_t v){modes[i]=v;}
    uint8_t configuredCurrent(uint8_t i){return currents[i];}
    void stageCurrent(uint8_t i,uint8_t v){currents[i]=v;}
    void applyConfiguredCurrent(uint8_t){}
}
namespace Printer {
    PreheatProfiles preheatProfiles; MachineInactivity inactivity(0,0); ManualMotion manualMotion(0,0,0,0); ZOffset zOffset; WobbleCorrection wobbleCorrection; PressureProtection pressureProtection;
    struct Sense {struct Limits{int16_t maxMicrometers;};Limits l{};bool a=false; void setMaxMicrometers(int16_t v){l.maxMicrometers=v;} int16_t savedDigits=0,savedMaximum=0; void publishStoredLimits(int16_t d,int16_t m){savedDigits=d;savedMaximum=m;} struct Stored{int16_t thresholdDigits,maxMicrometers;}; Stored storedLimits() const{return {savedDigits,savedMaximum};} Limits limits(){return l;} bool autoStart(){return a;} void setAutoStart(bool v){a=v;}} senseOffsetControl;
    struct Gauge {bool v=true;bool zeroingEnabled(){return v;}void setZeroingEnabled(bool x){v=x;}} strainGauge;
    struct Digit {struct Flow{int16_t minDigits,maxDigits;};Flow p{};bool v=true;Flow flowProfile(){return p;}void setFlowProfile(Flow x){p=x;}bool digitZEnabled(){return v;}void setDigitZEnabled(bool x){v=x;}} digitCompensation;
    struct Pause {void restoreDefaultParking(){}} pause; struct Layers{void resetFadeBounds(float){}} layers;
    uint8_t ZMode=0;
    uint8_t zDisplayMode(){ return ZMode; }
    void restoreZDisplayMode(uint8_t value){ ZMode=value; }
    float xy=0,z=0;
    #define AXIS_STORAGE(getter,setter) float getter##Values[4]{}; float getter(int i){return getter##Values[i];} void setter(int i,float v){getter##Values[i]=v;}
    AXIS_STORAGE(stepsPerMm,setStepsPerMm) AXIS_STORAGE(axisMaxFeedrate,setAxisMaxFeedrate) AXIS_STORAGE(axisHomingFeedrate,setAxisHomingFeedrate) AXIS_STORAGE(printAcceleration,setPrintAcceleration) AXIS_STORAGE(travelAcceleration,setTravelAcceleration) AXIS_STORAGE(axisLengthMm,setAxisLengthMm)
    float xyJerkLimit(){return xy;} void setXYJerkLimit(float v){xy=v;} float zJerkLimit(){return z;} void setZJerkLimit(float v){z=v;}
    void disableAllSteppersNow(){} void updateDerivedParameter(){} void applyAdvanceSettings(){}
}
namespace StepExecutor {
    uint16_t interval=0;
    uint16_t packingInterval(){return interval;}
    void setPackingInterval(uint16_t v){interval=v;}
}
namespace AccessoryOutputs {
    enum class Output : uint8_t { CaseLight, Mains, Fet1, Fet2, Fet3 };
    char states[5]{};
    char state(Output o){return states[static_cast<uint8_t>(o)];}
    void stage(Output o,char v){states[static_cast<uint8_t>(o)]=v;}
}
#include "preheat-owner.inc"
#include "EepromZoneDefaults.h"
#include "live-settings.inc"
// The new binding must neither read nor write the unowned lower bank.
static void zoneOwnerRoundTrip(){
    using namespace EepromZones;
    FactoryDefaults defaults(defaultHeatBedScanLiftMm,defaultZOffsetScanIndex,KOSYS_GCODE,FEEDRATE_DIRECTCONFIG);
    LiveSettings live;
    const auto untouched=image;
    uint8_t before[Last+1][MaxPayload]={};
    for(uint8_t id=1;id<=Last;++id){
        const uint8_t n=length(id);
        assert(defaults(id,before[id],n));
        assert(validBlock(id,before[id],n));
        assert(live.applyBlock(id,before[id],n,AllAlerts));
    }
    live.finishMotion();
    assert(live.bedCutoff()==0);
    for(uint8_t id=1;id<=Last;++id){
        if(!(LiveSettings::BulkMask&blockBit(id)))continue;
        uint8_t out[MaxPayload]={};
        const uint8_t n=length(id);
        assert(live(id,out,n));
        assert(validBlock(id,out,n));
        if(memcmp(before[id],out,n)!=0){
            fprintf(stderr,"new settings roundtrip mismatch block %u\n",unsigned(id));
            abort();
        }
    }
    // Preserve a deliberately tuned drive pair including the historic
    // 40/40 combination. Calibration identity and coefficients are exact.
    auto& c=extruder[0].tempControl;
    c.pidDriveMin=c.pidDriveMax=40;
    c.pidPGain=12.5f;c.pidIGain=.5f;c.pidDGain=22.f;
    c.tunedSensorType=c.sensorType;c.tunedPowerLimit=c.pidMax;
    c.target=180;
    uint8_t out[MaxPayload];
    assert(live(Hotend0,out,length(Hotend0)));
    c.pidDriveMin=10;c.pidDriveMax=20;c.invalidatePidCalibration();
    assert(live.applyBlock(Hotend0,out,length(Hotend0),0));
    assert(c.pidDriveMin==40 && c.pidDriveMax==40 && c.pidCalibrationValid() && c.pidPGain==12.5f && c.target==0);
    // Simulate ordinary post-autotune DMAX fine tuning: update only the
    // selected drive limit while a target is active. The other saved value
    // must not overwrite an independent live value.
    c.target=180;
    c.pidDriveMin=27;
    const Patch dmax=Patch::byte(Field::F_E0_DRIVE_MAX,75);
    out[address(Field::F_E0_DRIVE_MAX)-address(Hotend0)-PayloadOffset]=75;
    assert(live.applyDriveLimits(Hotend0,out,length(Hotend0),&dmax,1));
    assert(c.pidDriveMax==75 && c.pidDriveMin==27 && c.target==180);
    assert(c.pidCalibrationValid() && c.pidPGain==12.5f);
    const Patch wrong=Patch::byte(Field::F_E0_SENSOR_TYPE,c.sensorType);
    assert(!live.applyDriveLimits(Hotend0,out,length(Hotend0),&wrong,1));
    assert(c.target==180 && c.pidCalibrationValid());
    // Stored/active SenseOffset remain distinct; no takeover/reset of the
    // live offset is introduced by this binding.
    Printer::senseOffsetControl.publishStoredLimits(3200,120);
    assert(live(SenseOffset,out,length(SenseOffset)));
    Printer::senseOffsetControl.publishStoredLimits(0,25);
    assert(live.applyBlock(SenseOffset,out,length(SenseOffset),0));
    assert(Printer::senseOffsetControl.savedDigits==3200 && Printer::senseOffsetControl.savedMaximum==120);
    const uint8_t cutoff=90;
    assert(live.applyBlock(BedSafety,&cutoff,1,blockBit(BedSafety)) && live.bedCutoff()==0);
    assert(live.applyBlock(BedSafety,&cutoff,1,0) && live.bedCutoff()==90);
    assert(!live(BedSafety,out,1) && !live(22,out,length(22)));
    // No malformed block may alter an owner.
    before[Geometry][0]=0; before[Geometry][1]=0;before[Geometry][2]=0;before[Geometry][3]=0;
    const float steps=Printer::stepsPerMm(X_AXIS);
    assert(!live.applyBlock(Geometry,before[Geometry],length(Geometry),0));
    assert(Printer::stepsPerMm(X_AXIS)==steps);
    assert(image==untouched);
}
int main(){
    for(size_t p=0;p<image.size();++p) image[p]=uint8_t(p*13U+7U);
    zoneOwnerRoundTrip();
    puts("New EEPROM live settings owner roundtrip PASS");
}

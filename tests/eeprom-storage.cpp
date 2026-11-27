// Host-only EEPROM replay. Real persistence methods; model settings/hardware adapters.
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
#include "RgbLighting.h"
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
    void updateTempControlVars(){}
} heatedBedController;
struct Extruder {float stepsPerMM=0,maxFeedrate=0,maxEJerk=0,maxAcceleration=0,offsetMM[2]{},advanceL=0; uint8_t coolerSpeed=0; Controller tempControl; static Extruder* current; static void applyCurrentSettings(){} static void initHeatedBed(){} } extruder[2]; Extruder* Extruder::current=extruder;
#define NUM_TEMPERATURE_LOOPS (NUM_EXTRUDER+1)
Controller* tempController[NUM_TEMPERATURE_LOOPS] = {&extruder[0].tempControl,
#if NUM_EXTRUDER>1
    &extruder[1].tempControl,
#endif
    &heatedBedController};
struct Point {uint8_t x,y;}; Point point{}; float lift=defaultHeatBedScanLiftMm; void restoreZOffsetScanPoint(Point p){point=p;} Point zOffsetScanPoint(){return point;} float scanStartLiftMm(){return lift;} void setScanStartLiftMm(float v){lift=v;}
namespace RgbLighting {char m=0;void restoreMode(char v){m=v;}char mode(){return m;}}
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
    struct Sense {struct Limits{int16_t maxMicrometers;};Limits l{};bool a=false; void setMaxMicrometers(int16_t v){l.maxMicrometers=v;} int16_t savedDigits=0,savedMaximum=0; void publishStoredLimits(int16_t d,int16_t m){savedDigits=d;savedMaximum=m;} Limits limits(){return l;} bool autoStart(){return a;} void setAutoStart(bool v){a=v;}} senseOffsetControl;
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
#include "storage-source.inc"
static void checksum(){unsigned c=0;for(size_t i=0;i<image.size();++i)if(i!=EPR_INTEGRITY_BYTE)c+=image[i];assert(uint8_t(c)==image[EPR_INTEGRITY_BYTE]);}
static void snapshot(const char* name){checksum();printf("%s ",name);for(auto b:image)printf("%02x",b);puts("");}
int main(){
    image.fill(0xff); EEPROM::init(); assert(image[EPR_MAGIC_BYTE]==44 && image[EPR_VERSION]==EEPROM_PROTOCOL_VERSION); snapshot("factory");
    const auto factory=image; assert(image[EPR_CHECK_NUM_EXTRUDERS]==NUM_EXTRUDER);
    // Existing settings, counters and legacy/reserved cells survive normal load/store.
    HAL::eprSetInt32(EPR_PRINTING_TIME,123456); HAL::eprSetFloat(EPR_PRINTING_DISTANCE,42.25f); HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS,1234); image[1000]=0x9a; image[EPR_RF_230V_OUTPUT_MODE]=1;
    for(int i=0;i<NUM_EXTRUDER;++i){int p=EEPROM::getExtruderOffset(i);HAL::eprSetFloat(p+EPR_EXTRUDER_STEPS_PER_MM,100.25f+i);HAL::eprSetFloat(p+EPR_EXTRUDER_X_OFFSET,1.25f+i);}
    HAL::eprSetInt32(EPR_RF_Z_OFFSET,-123); EEPROM::updateChecksum(); EEPROM::init(); EEPROM::storeDataIntoEEPROM();
    assert(Printer::zOffset.manualMicrometers()==-123 && image[1000]==0x9a && image[EPR_RF_230V_OUTPUT_MODE]==1 && HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS)==1234 && HAL::eprGetInt32(EPR_PRINTING_TIME)==123456);
    for(int i=0;i<NUM_EXTRUDER;++i){assert(extruder[i].stepsPerMM==100.25f+i && extruder[i].offsetMM[0]==1.25f+i);} snapshot("roundtrip");
    // Valid checksum but invalid values: repair paths and retained RAM defaults.
    HAL::eprSetFloat(EPR_X_MAX_FEEDRATE,99999); HAL::eprSetFloat(EPR_X_LENGTH,-1); HAL::eprSetFloat(EPR_Z_MAX_ACCEL,-1); HAL::eprSetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMIN,0); HAL::eprSetFloat(EPR_ZSCAN_START_MM,99); image[EPR_RF_PART_FAN_PWM_MIN]=255; image[EPR_RF_PART_FAN_PWM_MAX]=0;
    for(int i=0;i<NUM_EXTRUDER;++i){int p=EEPROM::getExtruderOffset(i);HAL::eprSetFloat(p+EPR_EXTRUDER_STEPS_PER_MM,6000);HAL::eprSetFloat(p+EPR_EXTRUDER_MAX_FEEDRATE,-1);HAL::eprSetByte(p+EPR_EXTRUDER_DRIVE_MAX,40);HAL::eprSetByte(p+EPR_EXTRUDER_DRIVE_MIN,40);}
    EEPROM::updateChecksum(); EEPROM::init(); assert(HAL::eprGetFloat(EPR_X_MAX_FEEDRATE)==MAX_FEEDRATE_X && HAL::eprGetFloat(EPR_X_LENGTH)==X_MAX_LENGTH_PRINT && HAL::eprGetFloat(EPR_Z_MAX_ACCEL)==MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z); snapshot("repaired");
    image=factory; image[EPR_VERSION]=1; HAL::eprSetInt32(EPR_RF_PREHEAT_PROFILE_TAG,0); image[1000]=0x9a; EEPROM::updateChecksum(); EEPROM::init(); assert(image[EPR_VERSION]==EEPROM_PROTOCOL_VERSION && image[1000]==0x9a && HAL::eprGetInt32(EPR_RF_PREHEAT_PROFILE_TAG)==0x50524831L); snapshot("v1-migration");
    image=factory; image[EPR_CHECK_NUM_EXTRUDERS]=NUM_EXTRUDER==1?2:1; HAL::eprSetFloat(EPR_X_LENGTH,100); image[EEPROM::getExtruderOffset(1)+99]=0xa5; EEPROM::updateChecksum(); EEPROM::init(); assert(image[EPR_CHECK_NUM_EXTRUDERS]==NUM_EXTRUDER && HAL::eprGetFloat(EPR_X_LENGTH)==X_MAX_LENGTH_PRINT && image[EEPROM::getExtruderOffset(1)+99]==0xa5); snapshot("tool-migration");
    image=factory; image[1000]^=1; EEPROM::init(); assert(image[1000]==0); snapshot("bad-checksum");
    image=factory; image[EPR_MAGIC_BYTE]=0; EEPROM::updateChecksum(); EEPROM::init(); assert(image[EPR_MAGIC_BYTE]==44); snapshot("bad-magic");
    image=factory; image[1000]=0xa5; EEPROM::updateChecksum(); keys=true; EEPROM::init(); assert(image[1000]==0); snapshot("key-reset");
    keys=false;
    // Upgrade a version-8 image containing apparently valid gains: no heater authorized.
    image=factory; image[EPR_VERSION]=8;
    for (int i=0;i<9;++i) image[EPR_RF_PID_CALIBRATION+i]=0xA9;
    image[1000]=0x5a; EEPROM::updateChecksum(); EEPROM::init();
    assert(image[1000]==0x5a && image[EPR_VERSION]==9);
    for (auto* c:tempController) assert(!c->pidCalibrationValid());
    // Successful tune of a single physical heater persists only its thermal fields.
    for (uint8_t i=0;i<NUM_TEMPERATURE_LOOPS;++i) {
        auto& c=*tempController[i];
        c.pidPGain=12.5f; c.pidIGain=.5f; c.pidDGain=22.0f;
        c.pidDriveMax=c.pidMax; c.pidDriveMin=20;
        c.tunedSensorType=c.sensorType; c.tunedPowerLimit=c.pidMax;
        EEPROM::storePidCalibration(i); checksum();
        assert(c.pidCalibrationValid());
        assert(image[EPR_RF_PID_CALIBRATION+3*(i==NUM_EXTRUDER?2:i)]==0xA9);
        for (uint8_t j=i+1;j<NUM_TEMPERATURE_LOOPS;++j) assert(!tempController[j]->pidCalibrationValid());
    }
    const auto calibrated=image;
    for (auto* c:tempController) c->invalidatePidCalibration();
    EEPROM::init();
    for (auto* c:tempController) assert(c->pidCalibrationValid() && c->pidPGain==12.5f);
    // A sensor or PWM change revokes exactly that heater, including after reboot.
    HAL::eprSetByte(EPR_BED_PID_MAX, 100); EEPROM::updateChecksum(); EEPROM::init();
    assert(!heatedBedController.pidCalibrationValid() && extruder[0].tempControl.pidCalibrationValid());
    HAL::eprSetByte(EPR_BED_PID_MAX, calibrated[EPR_BED_PID_MAX]); EEPROM::updateChecksum(); EEPROM::init();
    assert(!heatedBedController.pidCalibrationValid());
    // Non-finite raw EEPROM gain cannot authorize a heater.
    HAL::eprSetFloat(EEPROM::getExtruderOffset(0)+EPR_EXTRUDER_PID_PGAIN, NAN);
    EEPROM::updateChecksum(); EEPROM::init(); assert(!extruder[0].tempControl.pidCalibrationValid());
    EEPROM::restoreEEPROMSettingsFromConfiguration(); EEPROM::storeDataIntoEEPROM(); EEPROM::init();
    for (auto* c:tempController) assert(!c->pidCalibrationValid());

}

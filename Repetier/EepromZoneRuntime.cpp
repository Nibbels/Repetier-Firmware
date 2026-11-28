// Sole internal EEPROM settings owner. The upper-bank schema is mandatory.
#include "Repetier.h"
#ifndef RF_THERMAL_EMERGENCY_EXPERIMENT
#error The new schema requires thermal emergency admission
#endif
#include "EepromZoneAvr.h"
#include "EepromZoneSession.h"
#include "EepromZoneDefaults.h"
#include "EepromZoneLive.h"
#include "BedMatrix.h"
#include "GCodeInput.h"

using namespace EepromZones;
namespace {
AvrDevice device;
bool loaded=false;
volatile bool outputsAdmitted=false;
bool lastOk=false;
bool settingsFaultLatched=false;
uint32_t pending=AllAlerts;
constexpr uint32_t OrdinaryAlerts=AllAlerts&~blockBit(BedSafety);
using ZoneSession=Session<AvrDevice>;
ZoneSession session(){return ZoneSession(device,AvrDevice::settingsBusy(),configuredProfile());}
Field at(Field field,uint8_t delta){return shifted(field,delta);}
uint8_t thermalBlock(uint8_t heater){return heater==NUM_EXTRUDER?BedPid:Hotend0+heater;}
TemperatureController& controller(uint8_t block){return *tempController[block==BedPid?NUM_EXTRUDER:block-Hotend0];}
bool idle() {
    return !Printer::blockAll && !Printer::printJob.isBusy()
        && !Printer::printJob.commandsBlocked() && !Printer::pause.isPaused()
        && !PrintLine::hasLines() && !PrintLine::directMoveActive() && !Printer::needsCMPwait()
        && !isAnyScanRunning() && !TemperatureController::autotuneActive();
}
bool heatersOff() {
    for(uint8_t h=0;h<NUM_TEMPERATURE_LOOPS;++h)
        if(tempController[h]->targetTemperatureC>0 || HAL::outputDuty(h))return false;
    return !TemperatureController::autotuneActive();
}
void revokeOutputs() {
    outputsAdmitted=false;
    loaded=false;
    for(uint8_t h=0;h<NUM_TEMPERATURE_LOOPS;++h)tempController[h]->setTargetTemperature(0);
    heatedBedController.setBedEmergencyCutoff(0);
    Printer::lockMovement();
}
void failSettings(Result result) {
    lastOk=false;
    if(settingsFaultLatched)return;
    // Latch admission before any cleanup or diagnostic output. A failed
    // statistics-page read must not recursively render the same page.
    settingsFaultLatched=true;
    Printer::blockAll=1;
    revokeOutputs();
    // Not a sensor fault: separate settings failure, terminal this boot.
    Com::printErrorFLN(PSTR("EEPROM settings fault"));
    Com::printFLN(PSTR("EEPROM result: "),int(result));
    UI_STATUS("EEPROM fault"); // deferred render, never re-enter EEPROM here
}
bool accept(Result result) {
    if(settingsFaultLatched){lastOk=false;return false;}
    if(result==Result::Ok){lastOk=true;return true;}
    failSettings(result);return false;
}
bool persist(const Patch* edits,uint8_t count) {
    if(!loaded){lastOk=false;return false;}
    auto s=session();
    return accept(s.patch(edits,count));
}
bool persist(Patch p){return persist(&p,1);}
bool saveLive(uint32_t mask) {
    if(!loaded){lastOk=false;return false;}
    LiveSettings provider;
    auto s=session();
    return accept(s.save(mask,provider));
}
void progress(){device.service();GCodeInput::keepAlive(Processing);}
bool invalidateMatrices() {
    if(!BedMatrix::invalidateAllStored(progress)) {
        failSettings(Result::TransportFault);return false;
    }
    return true;
}
bool loadFull(bool reset,bool startup) {
    lastOk=false;
    if(!startup && (!idle() || !heatersOff())) {
        Com::printErrorFLN(PSTR("EEPROM load requires idle cold targets"));
        return false;
    }
    revokeOutputs();
    if(!startup) {
        Printer::disableAllSteppersNow();
        Printer::setHomed(false,false,false);
    }
    FactoryDefaults defaults(defaultHeatBedScanLiftMm,defaultZOffsetScanIndex,KOSYS_GCODE,FEEDRATE_DIRECTCONFIG);
    auto s=session();Inspection state;
    if(!accept(s.restore(defaults,state,reset)))return false;
    // Keep recovery alerts durable until all external matrix markers have
    // been invalidated. A power cut here repeats this work on next boot.
    if((state.alerts&blockBit(Geometry)) && !invalidateMatrices())return false;
    LiveSettings live;
    auto consumer=[&](uint8_t id,const uint8_t* data,uint8_t n,uint32_t alerts){
        if(Printer::printJob.emergencyLocked())return false;
        return live.applyBlock(id,data,n,alerts);
    };
    if(!accept(s.load(consumer,state)))return false;
    if(Printer::printJob.emergencyLocked()){lastOk=false;return false;}
    live.finishMotion();
    if(Printer::printJob.emergencyLocked()){lastOk=false;return false;}
    pending=state.alerts;
    const uint8_t cutoff=live.bedCutoff();
    if(cutoff && !heatedBedController.setBedEmergencyCutoff(cutoff)){
        failSettings(Result::Invalid);return false;
    }
    // Runtime output pins are already initialized; at boot Printer::setup
    // owns their initialization after the settings have been applied.
    if(!startup) {
        AccessoryOutputs::initializeBoardOutputs();
        AccessoryOutputs::initializeCaseLight();
    }
    loaded=true;lastOk=true;
    outputsAdmitted=!(pending&OrdinaryAlerts) && !Printer::blockAll;
    EEPROM::reportRecovery();
    return true;
}
bool readBlockChecked(uint8_t id,uint8_t* data,uint32_t& alerts) {
    // At startup and after terminal failure there is no published settings
    // snapshot. Repeated diagnostics must not access the failed transport.
    if(!loaded || settingsFaultLatched)return false;
    auto s=session();
    return accept(s.read(id,data,length(id),alerts));
}
bool driveOnly(uint8_t id,const Patch* edits,uint8_t count) {
    if(count!=1 || edits[0].width!=1 || (id!=BedPid && (id<Hotend0 || id>=Hotend0+NUM_EXTRUDER)))return false;
    const uint8_t offset=edits[0].pos-address(id)-PayloadOffset;
    return offset==12 || offset==13;
}
bool motionBlock(uint8_t id) {
    return id>=Geometry && id<=Extruder1Motion;
}
const char zonePreheatABed[] PROGMEM="Preheat A bed [C]";
const char zonePreheatAHotend[] PROGMEM="Preheat A hotend [C]";
const char zonePreheatBBed[] PROGMEM="Preheat B bed [C]";
const char zonePreheatBHotend[] PROGMEM="Preheat B hotend [C]";
const char zoneBedCutoff[] PROGMEM="Bed emergency cutoff [C]";
struct Report {
    uint8_t data[MaxPayload];
    uint8_t cached=0;
    bool ok=true;
    void field(uint8_t wire,uint16_t pos,PGM_P label,uint8_t digits) {
        if(!ok)return;
        const uint8_t id=blockOf(pos);
        if(id!=cached) {
            uint32_t alerts;
            if(!readBlockChecked(id,data,alerts)){ok=false;return;}
            cached=id;
        }
        BlockDecoder d(id,data,length(id));
        const auto f=static_cast<Field>(pos);
        switch(wire) {
        case 0:Com::printF(Com::tEPR0,int(pos));Com::print(' ');Com::print(int(d.u8(f)));break;
        case 1:Com::printF(Com::tEPR1,int(pos));Com::print(' ');Com::print(int16_t(d.u16(f)));break;
        case 2:Com::printF(Com::tEPR2,int(pos));Com::print(' ');Com::print(int32_t(d.u32(f)));break;
        case 3:Com::printF(Com::tEPR3,int(pos));Com::print(' ');Com::printFloat(d.f32(f),digits);break;
        }
        Com::print(' ');
        if(id==Extruder0Motion || id==Extruder1Motion || id==Hotend0 || id==Hotend1){
            Com::printF(Com::tExtrDot,int(id>=Hotend0?id-Hotend0+1:id-Extruder0Motion+1));
            Com::print(' ');
        }
        Com::printFLN(label);
        progress();
    }
};
}
bool EEPROM::settingsReady(){return outputsAdmitted && !Printer::blockAll;}
namespace EepromZones {bool machineSettingsReady(){return EEPROM::settingsReady();}}
bool EEPROM::recoveryPending(){return loaded && (pending&OrdinaryAlerts);}
bool EEPROM::lastOperationSucceeded(){return lastOk;}
const void* EEPROM::recoveryDialogTag(){return PSTR("EEPROM recovery");}
void EEPROM::reportRecovery() {
    if(pending&OrdinaryAlerts) {
        Com::printErrorFLN(PSTR("EEPROM recovery: verify defaults; acknowledge settings"));
        Com::printFLN(PSTR("EEPROM alert mask: "),pending);
        UI_STATUS_UPD("EEPROM check/OK");
        showInformation(recoveryDialogTag(),PSTR("Verify defaults"),PSTR("OK to confirm"));
    }
    if(pending&blockBit(BedSafety)) {
        Com::printFLN(PSTR("Bed cutoff unset; Configuration > Bed cutoff"));
        if(!(pending&OrdinaryAlerts))UI_STATUS_UPD("Bed cutoff unset");
    }
}
bool EEPROM::acknowledgeRecovery() {
    if(!loaded || !idle() || !heatersOff())return false;
    auto s=session();
    const uint32_t mask=pending&OrdinaryAlerts;
    if(mask && !accept(s.acknowledge(mask)))return false;
    uint8_t cutoff=0;uint32_t alerts=AllAlerts;
    if(!accept(s.read(BedSafety,&cutoff,1,alerts)))return false;
    pending=alerts;
    // The byte transport can service a terminal thermal fault while
    // committing the operator's acknowledgment. Never reopen admission.
    if(!idle() || !heatersOff()) {
        lastOk=false;outputsAdmitted=false;return false;
    }
    outputsAdmitted=loaded && !(pending&OrdinaryAlerts) && !Printer::blockAll;
    reportRecovery();
    return !(pending&OrdinaryAlerts);
}
bool EEPROM::commissionBedCutoff(uint8_t cutoff) {
    if(!settingsReady() || !idle() || !heatersOff() || cutoff<40 || cutoff>150)return false;
    // Revoke before changing the stored permission, including lower limits.
    heatedBedController.setBedEmergencyCutoff(0);
    auto s=session();
    if(!accept(s.commission(cutoff)))return false;
    uint8_t actual=0;uint32_t alerts=AllAlerts;
    if(!accept(s.read(BedSafety,&actual,1,alerts)))return false;
    pending=alerts;
    // A durable write is not authorization to re-arm after a terminal
    // thermal fault during the EEPROM wait/readback.
    if(!idle() || !heatersOff()) {
        lastOk=false;outputsAdmitted=false;return false;
    }
    if(actual!=cutoff || (alerts&AllAlerts) || !heatedBedController.setBedEmergencyCutoff(actual)) {
        failSettings(Result::NeedsRecovery);return false;
    }
    return true;
}
void EEPROM::init() {
    const bool reset=READ(ENABLE_KEY_1)==0 && READ(ENABLE_KEY_4)==0 && READ(ENABLE_KEY_E5)==0;
    loadFull(reset,true);
}
void EEPROM::restoreEEPROMSettingsFromConfiguration(){loadFull(true,false);}
void EEPROM::writeSettings() {
    // All report metadata is immutable and remains in near Flash.
    struct ReportField {uint16_t key;PGM_P label;uint8_t digits;};
    static const ReportField fields[] PROGMEM = {
#define RF_ZONE_FIELD_END ,
#define RF_ZONE_FIELD(type,start,label,digits,writable) {hostFieldKey(start,type),label,digits}
#include "EepromZoneHostFields.h"
#undef RF_ZONE_FIELD
#undef RF_ZONE_FIELD_END
    };
    Report report;
    for(const auto& field:fields) {
        const uint16_t key=pgm_read_word(&field.key);
        const auto label=reinterpret_cast<PGM_P>(pgm_read_word(&field.label));
        report.field(uint8_t(key>>10),uint16_t(Base+(key&0x03FF)),label,pgm_read_byte(&field.digits));
        if(!report.ok)break;
    }
}
void EEPROM::update(GCode* com) {
    lastOk=false;
    if(!loaded || !com->hasP() || !com->hasT() || !(com->T==3?com->hasX():com->hasS()))return;
    LiveSettings live;
    auto admit=[&](uint8_t id,uint32_t,const Patch* edits,uint8_t count){
        if(Printer::blockAll || Printer::printJob.emergencyLocked() || TemperatureController::autotuneActive())return false;
        if(driveOnly(id,edits,count))return true;
        if(!idle() || !heatersOff())return false;
        if(id==Geometry && !invalidateMatrices())return false;
        if(count==2 && (id==BedPid || id==Hotend0 || id==Hotend1))controller(id).invalidatePidCalibration();
        if(motionBlock(id)) {
            Printer::disableAllSteppersNow();
            Printer::setHomed(false,false,false);
        }
        return true;
    };
    auto publish=[&](uint8_t id,const uint8_t* data,uint8_t n,uint32_t alerts,const Patch* edits,uint8_t count){
        if(Printer::blockAll || Printer::printJob.emergencyLocked())return false;
        if(driveOnly(id,edits,count))return live.applyDriveLimits(id,data,n,edits,count);
        if(!live.applyBlock(id,data,n,alerts))return false;
        if(motionBlock(id))live.finishMotion();
        if(id==EepromZones::Lighting) {
            AccessoryOutputs::initializeBoardOutputs();
            AccessoryOutputs::initializeCaseLight();
        }
        return true;
    };
    auto s=session();
    const auto result=s.hostEdit(com->P,com->T,com->hasS()?com->S:0,com->hasX()?com->X:0,admit,publish);
    if(result==Result::Ok){lastOk=!settingsFaultLatched;return;}
    if(result==Result::Invalid || result==Result::ReadOnly || result==Result::AdmissionRejected || result==Result::Busy) {
        Com::printErrorFLN(PSTR("EEPROM edit rejected"));
        Com::printFLN(PSTR("EEPROM result: "),int(result));return;
    }
    failSettings(result);
}

void EEPROM::storePidCalibration(uint8_t heater) {
    if(heater>=NUM_TEMPERATURE_LOOPS)return;
    if(!saveLive(blockBit(thermalBlock(heater))))tempController[heater]->invalidatePidCalibration();
}
void EEPROM::storeDriveLimit(uint8_t heater,bool maximum,uint8_t value) {
    if(heater>=NUM_TEMPERATURE_LOOPS)return;
    const Field f=heater==NUM_EXTRUDER?(maximum?Field::F_BED_DRIVE_MAX:Field::F_BED_DRIVE_MIN)
        :at(maximum?Field::F_E0_DRIVE_MAX:Field::F_E0_DRIVE_MIN,heater*SlotSize);
    persist(Patch::byte(f,value));
}
void EEPROM::storePreheatProfiles(){saveLive(blockBit(Preheat));}
void EEPROM::storeManualZOffset(){persist(Patch::dword(Field::F_RF_Z_OFFSET,Printer::zOffset.manualMicrometers()));}
void EEPROM::storeSenseOffsetSettings(int16_t digits,int16_t micrometers,bool saveDigits,bool saveMaximum) {
    Patch p[2];uint8_t n=0;
    if(saveDigits)p[n++]=Patch::word(Field::F_RF_MOD_SENSEOFFSET_DIGITS,digits);
    if(saveMaximum)p[n++]=Patch::word(Field::F_RF_MOD_SENSEOFFSET_OFFSET_MAX,micrometers);
    if(!n || !persist(p,n))return;
    const auto old=Printer::senseOffsetControl.storedLimits();
    Printer::senseOffsetControl.publishStoredLimits(saveDigits?digits:old.thresholdDigits,saveMaximum?micrometers:old.maxMicrometers);
}
bool EEPROM::storePressureLimits(bool stop,int32_t min,int32_t max,bool saveMin,bool saveMax) {
    lastOk=false;
    if(!loaded || Printer::blockAll || Printer::printJob.emergencyLocked())return false;
    if(!saveMin && !saveMax){lastOk=true;return true;}
    // The other live pause endpoint can be a temporary M3075 override.
    // Validate the proposed DURABLE pair before publishing a menu change.
    uint8_t data[MaxPayload];uint32_t alerts=AllAlerts;
    auto s=session();
    if(!accept(s.read(Pressure,data,length(Pressure),alerts)))return false;
    BlockDecoder d(Pressure,data,length(Pressure));
    const int32_t lo=saveMin?min:(stop?int32_t(int16_t(d.u16(Field::F_RF_EMERGENCYZSTOPDIGITSMIN))):int32_t(d.u32(Field::F_RF_EMERGENCYPAUSEDIGITSMIN)));
    const int32_t hi=saveMax?max:(stop?int32_t(int16_t(d.u16(Field::F_RF_EMERGENCYZSTOPDIGITSMAX))):int32_t(d.u32(Field::F_RF_EMERGENCYPAUSEDIGITSMAX)));
    if(lo>hi || (stop?(lo<EMERGENCY_STOP_DIGITS_MIN || hi>EMERGENCY_STOP_DIGITS_MAX || !lo || !hi):
                          (lo<EMERGENCY_PAUSE_DIGITS_MIN || hi>EMERGENCY_PAUSE_DIGITS_MAX))) {
        lastOk=false;
        Com::printErrorFLN(PSTR("EEPROM pressure band rejected"));
        UI_STATUS_UPD("Invalid digit band");
        return false;
    }
    Patch p[2];uint8_t n=0;
    if(stop){
        if(saveMin)p[n++]=Patch::word(Field::F_RF_EMERGENCYZSTOPDIGITSMIN,uint16_t(lo));
        if(saveMax)p[n++]=Patch::word(Field::F_RF_EMERGENCYZSTOPDIGITSMAX,uint16_t(hi));
    }else{
        if(saveMin)p[n++]=Patch::dword(Field::F_RF_EMERGENCYPAUSEDIGITSMIN,uint32_t(lo));
        if(saveMax)p[n++]=Patch::dword(Field::F_RF_EMERGENCYPAUSEDIGITSMAX,uint32_t(hi));
    }
    if(!accept(s.patch(p,n)))return false;
    if(Printer::printJob.emergencyLocked()){lastOk=false;return false;}
    return true;
}
void EEPROM::storeMicrostepConfiguration(uint8_t mask) {
    if(!mask)return;
    uint32_t blocks=0;
    if(mask&7) {
        if(!invalidateMatrices())return;
        blocks|=blockBit(Geometry);
    }
    if(mask&(1<<E_AXIS))blocks|=blockBit(Extruder0Motion);
    if(mask&(1<<(E_AXIS+1)))blocks|=blockBit(Extruder1Motion);
    if(blocks)saveLive(blocks);
}
void EEPROM::storeCaseLight(bool on){persist(Patch::byte(Field::F_RF_CASE_LIGHT_MODE,on));}
void EEPROM::storeFet(uint8_t channel,bool on) {
    if(channel<1 || channel>3)return;
#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    if((channel==1 && HOTEND_COOLER_USES_PIN(FET1)) ||
       (channel==2 && HOTEND_COOLER_USES_PIN(FET2)) ||
       (channel==3 && (CASE_FAN_PIN>=0 || HOTEND_COOLER_USES_PIN(FET3))))return;
    persist(Patch::byte(at(Field::F_RF_FET1_MODE,channel-1),on));
#else
    (void)on;
#endif
}
void EEPROM::storeJogMode(uint8_t axis,uint8_t mode){if(axis<3)persist(Patch::byte(at(Field::F_RF_MOVE_MODE_X,axis),mode));}
void EEPROM::storeZJogSteps(uint16_t steps){persist(Patch::word(Field::F_RF_MOD_Z_STEP_SIZE,steps));}
void EEPROM::storeJogCoordinates(bool gcode){persist(Patch::byte(Field::F_RF_MOVE_MODE_XY_KOSYS,gcode));}
void EEPROM::storeJogFeedrate(bool configured){persist(Patch::byte(Field::F_RF_MOVE_POSITION_FEEDRATE,configured));}
void EEPROM::storeZDisplayMode(uint8_t mode){persist(Patch::byte(Field::F_RF_Z_MODE,mode));}
void EEPROM::storeFanMode(uint8_t mode){persist(Patch::byte(Field::F_RF_FAN_MODE,mode));}
void EEPROM::storeDigitZeroing(bool on){persist(Patch::byte(Field::F_RF_ZERO_DIGIT_STATE,on?1:2));}
void EEPROM::storeDigitCompensation(bool on){persist(Patch::byte(Field::F_RF_DIGIT_CMP_STATE,on?1:2));}
void EEPROM::storeSenseAutoStart(bool on){persist(Patch::byte(Field::F_RF_MOD_SENSEOFFSET_AUTOSTART,on));}
void EEPROM::storeScanPoint(uint8_t x,uint8_t y,bool saveX,bool saveY) {
    Patch p[2];uint8_t n=0;
    if(saveX)p[n++]=Patch::byte(Field::F_RF_MOD_ZOS_SCAN_POINT_X,x);
    if(saveY)p[n++]=Patch::byte(Field::F_RF_MOD_ZOS_SCAN_POINT_Y,y);
    if(n)persist(p,n);
}
void EEPROM::storeFanDivisor(uint8_t divisor){persist(Patch::byte(Field::F_RF_PART_FAN_SPEED,divisor));}
void EEPROM::storeFanDuty(bool maximum,uint8_t duty){persist(Patch::byte(maximum?Field::F_RF_PART_FAN_PWM_MAX:Field::F_RF_PART_FAN_PWM_MIN,duty));}
void EEPROM::storeToolOffsets(uint8_t tool,float x,float y,bool saveX,bool saveY) {
    if(tool>=NUM_EXTRUDER)return;
    Patch p[2];uint8_t n=0;
    if(saveX)p[n++]=Patch::real(at(Field::F_E0_X_OFFSET,tool*SlotSize),x);
    if(saveY)p[n++]=Patch::real(at(Field::F_E0_Y_OFFSET,tool*SlotSize),y);
    if(n)persist(p,n);
}
void EEPROM::storeInactivity(bool stepper,uint32_t duration){persist(Patch::dword(stepper?Field::F_STEPPER_INACTIVE_TIME:Field::F_MAX_INACTIVE_TIME,duration));}
void EEPROM::storeScanLift(float lift){persist(Patch::real(Field::F_ZSCAN_START_MM,lift));}
void EEPROM::storeWobblePhase(int8_t phase){persist(Patch::byte(Field::F_RF_MOD_WOBBLE_FIX_PHASEXY,phase));}
void EEPROM::storeWobbleAmplitude(uint8_t component,int16_t amplitude){if(component<3)persist(Patch::word(at(Field::F_RF_MOD_WOBBLE_FIX_AMPX,2*component),amplitude));}
void EEPROM::storeMotorCurrent(uint8_t axis,uint8_t current){if(axis<DRV8711_NUM_CHANNELS)persist(Patch::byte(at(Field::F_RF_MOTOR_CURRENT,axis),current));}
void EEPROM::storePackingInterval(uint16_t interval){persist(Patch::word(Field::F_RF_STEP_PACKING_MIN_INTERVAL,interval));}
void EEPROM::storeAxisAcceleration(uint8_t axis,bool travel,float v){if(axis<3)persist(Patch::real(at(travel?Field::F_X_MAX_TRAVEL_ACCEL:Field::F_X_MAX_ACCEL,4*axis),v));}
void EEPROM::storeAxisJerk(bool z,float v){persist(Patch::real(z?Field::F_MAX_ZJERK:Field::F_MAX_XYJERK,v));}
void EEPROM::storeAxisFeedrate(uint8_t axis,bool homing,float v){if(axis<3)persist(Patch::real(at(homing?Field::F_X_HOMING_FEEDRATE_PRINT:Field::F_X_MAX_FEEDRATE,4*axis),v));}
void EEPROM::storeToolMotion(uint8_t tool,ToolMotionSetting setting,float v) {
    if(tool>=NUM_EXTRUDER)return;
    Field f;
    switch(setting){
    case ToolMotionSetting::Steps:f=Field::F_E0_STEPS_PER_MM;break;
    case ToolMotionSetting::Advance:f=Field::F_E0_ADVANCE_L;break;
    case ToolMotionSetting::Acceleration:f=Field::F_E0_MAX_ACCELERATION;break;
    case ToolMotionSetting::Feedrate:f=Field::F_E0_MAX_FEEDRATE;break;
    case ToolMotionSetting::Jerk:f=Field::F_E0_MAX_START_FEEDRATE;break;
    default:return;
    }
    persist(Patch::real(at(f,tool*SlotSize),v));
}

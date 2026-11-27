#include "Repetier.h"
#include "EepromZoneLive.h"
#include "EepromZoneCodec.h"
#include "EepromZoneValidation.h"
#include "StepExecutor.h"

namespace EepromZones {
namespace {
Field at(Field field,uint8_t delta) {return shifted(field,delta);}
void encodeThermal(BlockEncoder& w,const TemperatureController& t,Field p,Field i,Field d,
                   Field min,Field max,Field power,Field sensor,Field tag) {
    w.f32(p,t.pidPGain);w.f32(i,t.pidIGain);w.f32(d,t.pidDGain);
    w.u8(min,t.pidDriveMin);w.u8(max,t.pidDriveMax);w.u8(power,t.pidMax);w.u8(sensor,t.sensorType);
    uint8_t identity[3]={};
    if(t.pidCalibrationValid()) {identity[0]=0xA9;identity[1]=t.tunedSensorType;identity[2]=t.tunedPowerLimit;}
    w.bytes(tag,identity,3);
}
void applyThermal(BlockDecoder& d,TemperatureController& t,Field p,Field i,Field gainD,
                  Field min,Field max,Field power,Field sensor,Field tag) {
    t.setTargetTemperature(0);
    t.invalidatePidCalibration();
    t.pidPGain=d.f32(p);t.pidIGain=d.f32(i);t.pidDGain=d.f32(gainD);
    t.pidDriveMin=d.u8(min);t.pidDriveMax=d.u8(max);t.pidMax=d.u8(power);t.sensorType=d.u8(sensor);
    if(d.u8(tag)==0xA9) {
        t.tunedSensorType=d.u8(at(tag,1));t.tunedPowerLimit=d.u8(at(tag,2));
    }
    t.updateTempControlVars();
}
}
bool LiveSettings::operator()(uint8_t id,uint8_t* data,uint8_t n) const {
    BlockEncoder w(id,data,n);
    if(!id || id>Last || !(BulkMask&blockBit(id)))return false;
    switch(id) {
    case Geometry:
        for(uint8_t a=0;a<3;++a) {
            w.f32(at(Field::F_XAXIS_STEPS_PER_MM,4*a),Printer::stepsPerMm(a));
            w.f32(at(Field::F_X_LENGTH,4*a),Printer::axisLengthMm(a));
            w.u8(at(Field::F_RF_MICRO_STEPS_X,a),Drv8711::microstepMode(a));
        }
        w.u16(Field::F_RF_MOD_Z_STEP_SIZE,Printer::manualMotion.steps(Z_AXIS));break;
    case Feedrates:
        for(uint8_t a=0;a<3;++a) {
            w.f32(at(Field::F_X_MAX_FEEDRATE,4*a),Printer::axisMaxFeedrate(a));
            w.f32(at(Field::F_X_HOMING_FEEDRATE_PRINT,4*a),Printer::axisHomingFeedrate(a));
        }break;
    case Jerk:
        w.f32(Field::F_MAX_XYJERK,Printer::xyJerkLimit());w.f32(Field::F_MAX_ZJERK,Printer::zJerkLimit());break;
    case Acceleration:
        for(uint8_t a=0;a<3;++a) {
            w.f32(at(Field::F_X_MAX_ACCEL,4*a),Printer::printAcceleration(a));
            w.f32(at(Field::F_X_MAX_TRAVEL_ACCEL,4*a),Printer::travelAcceleration(a));
        }
        w.u16(Field::F_RF_STEP_PACKING_MIN_INTERVAL,StepExecutor::packingInterval());break;
    case MotorCurrent:
        for(uint8_t a=0;a<5;++a)w.u8(at(Field::F_RF_MOTOR_CURRENT,a),Drv8711::configuredCurrent(a));
        break;
    case Extruder0Motion: case Extruder1Motion: {
        const uint8_t tool=id-Extruder0Motion, delta=tool*SlotSize;
        if(tool<NUM_EXTRUDER) {
            const auto& e=extruder[tool];
            w.f32(at(Field::F_E0_STEPS_PER_MM,delta),e.stepsPerMM);
            w.f32(at(Field::F_E0_MAX_FEEDRATE,delta),e.maxFeedrate);
            w.f32(at(Field::F_E0_MAX_START_FEEDRATE,delta),e.maxEJerk);
            w.f32(at(Field::F_E0_MAX_ACCELERATION,delta),e.maxAcceleration);
            w.f32(at(Field::F_E0_ADVANCE_L,delta),e.advanceL);
            w.f32(at(Field::F_E0_X_OFFSET,delta),e.offsetMM[X_AXIS]);
            w.f32(at(Field::F_E0_Y_OFFSET,delta),e.offsetMM[Y_AXIS]);
        } else {
            const uint8_t zeros[28]={};w.bytes(at(Field::F_E0_STEPS_PER_MM,delta),zeros,28);
        }
        w.u8(at(Field::F_RF_MICRO_STEPS_E0,delta),Drv8711::microstepMode(E_AXIS+tool));break;
    }
    case Hotend0: case Hotend1: {
        const uint8_t tool=id-Hotend0, delta=tool*SlotSize;
        if(tool<NUM_EXTRUDER) {
            const auto& e=extruder[tool];
            encodeThermal(w,e.tempControl,at(Field::F_E0_PID_PGAIN,delta),at(Field::F_E0_PID_IGAIN,delta),
                at(Field::F_E0_PID_DGAIN,delta),at(Field::F_E0_DRIVE_MIN,delta),at(Field::F_E0_DRIVE_MAX,delta),
                at(Field::F_E0_PID_MAX,delta),at(Field::F_E0_SENSOR_TYPE,delta),at(Field::F_PID_CALIBRATION_SLOT_0,delta));
            w.u8(at(Field::F_E0_COOLER_SPEED,delta),e.coolerSpeed);
        } else {
            const uint8_t zeros[20]={};w.bytes(at(Field::F_E0_PID_PGAIN,delta),zeros,20);
        }break;
    }
    case BedPid:
        encodeThermal(w,heatedBedController,Field::F_BED_PID_PGAIN,Field::F_BED_PID_IGAIN,Field::F_BED_PID_DGAIN,
            Field::F_BED_DRIVE_MIN,Field::F_BED_DRIVE_MAX,Field::F_BED_PID_MAX,Field::F_RF_HEATED_BED_SENSOR_TYPE,
            Field::F_PID_CALIBRATION_SLOT_2);break;
    case Preheat: {
        const auto a=Printer::preheatProfiles.get(PreheatProfiles::Profile::A);
        const auto b=Printer::preheatProfiles.get(PreheatProfiles::Profile::B);
        w.u16(Field::F_RF_PREHEAT_A_BED,a.bed);w.u16(Field::F_RF_PREHEAT_A_HOTEND,a.hotend);
        w.u16(Field::F_RF_PREHEAT_B_BED,b.bed);w.u16(Field::F_RF_PREHEAT_B_HOTEND,b.hotend);break;
    }
    case ZOffset:w.u32(Field::F_RF_Z_OFFSET,Printer::zOffset.manualMicrometers());break;
    case SenseOffset: {
        const auto saved=Printer::senseOffsetControl.storedLimits();
        w.u16(Field::F_RF_MOD_SENSEOFFSET_OFFSET_MAX,saved.maxMicrometers);
        w.u16(Field::F_RF_MOD_SENSEOFFSET_DIGITS,saved.thresholdDigits);
        w.u8(Field::F_RF_MOD_SENSEOFFSET_AUTOSTART,Printer::senseOffsetControl.autoStart());
        w.u8(Field::F_RF_ZERO_DIGIT_STATE,Printer::strainGauge.zeroingEnabled()?1:2);
        w.u8(Field::F_RF_DIGIT_CMP_STATE,Printer::digitCompensation.digitZEnabled()?1:2);break;
    }
    case Pressure: {
        const auto p=Printer::pressureProtection.pauseLimits(),s=Printer::pressureProtection.stopLimits();
        w.u32(Field::F_RF_EMERGENCYPAUSEDIGITSMIN,p.minimum);w.u32(Field::F_RF_EMERGENCYPAUSEDIGITSMAX,p.maximum);
        w.u16(Field::F_RF_EMERGENCYZSTOPDIGITSMIN,s.minimum);w.u16(Field::F_RF_EMERGENCYZSTOPDIGITSMAX,s.maximum);break;
    }
    case Scan: {
        const auto p=zOffsetScanPoint();w.u8(Field::F_RF_MOD_ZOS_SCAN_POINT_X,p.x);
        w.u8(Field::F_RF_MOD_ZOS_SCAN_POINT_Y,p.y);w.f32(Field::F_ZSCAN_START_MM,scanStartLiftMm());break;
    }
    case Wobble: {
        const auto& p=Printer::wobbleCorrection.profile();w.u8(Field::F_RF_MOD_WOBBLE_FIX_PHASEXY,p.phase);
        w.u16(Field::F_RF_MOD_WOBBLE_FIX_AMPX,p.xMicrometers);w.u16(Field::F_RF_MOD_WOBBLE_FIX_AMPY1,p.yAtXMinMicrometers);
        w.u16(Field::F_RF_MOD_WOBBLE_FIX_AMPY2,p.yAtXMaxMicrometers);break;
    }
    case Panel:
        for(uint8_t a=0;a<3;++a)w.u8(at(Field::F_RF_MOVE_MODE_X,a),Printer::manualMotion.mode(a));
        w.u8(Field::F_RF_Z_MODE,Printer::zDisplayMode());w.u8(Field::F_RF_MOVE_MODE_XY_KOSYS,Printer::manualMotion.coordinateSystem());
        w.u8(Field::F_RF_MOVE_POSITION_FEEDRATE,Printer::manualMotion.feedrateSource());break;
    case Lighting: {
        w.u8(Field::F_RF_CASE_LIGHT_MODE,AccessoryOutputs::state(AccessoryOutputs::Output::CaseLight));
#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
        w.u8(Field::F_RF_FET1_MODE,HOTEND_COOLER_USES_PIN(FET1)?0:AccessoryOutputs::state(AccessoryOutputs::Output::Fet1));
        w.u8(Field::F_RF_FET2_MODE,HOTEND_COOLER_USES_PIN(FET2)?0:AccessoryOutputs::state(AccessoryOutputs::Output::Fet2));
        w.u8(Field::F_RF_FET3_MODE,(CASE_FAN_PIN>=0 || HOTEND_COOLER_USES_PIN(FET3))?0:AccessoryOutputs::state(AccessoryOutputs::Output::Fet3));
#else
        w.u8(Field::F_RF_FET1_MODE,0);w.u8(Field::F_RF_FET2_MODE,0);w.u8(Field::F_RF_FET3_MODE,0);
#endif
        break;
    }
    case PartFan:
        w.u8(Field::F_RF_FAN_MODE,HAL::partFanMode());w.u8(Field::F_RF_PART_FAN_SPEED,HAL::partFanDivisor());
        w.u8(Field::F_RF_PART_FAN_PWM_MIN,HAL::partFanMinDuty());w.u8(Field::F_RF_PART_FAN_PWM_MAX,HAL::partFanMaxDuty());break;
    case System:
        w.u32(Field::F_BAUDRATE,firmwareSerialBaudrate);
        w.u32(Field::F_MAX_INACTIVE_TIME,Printer::inactivity.shutdownTimeout());
        w.u32(Field::F_STEPPER_INACTIVE_TIME,Printer::inactivity.stepperTimeout());break;
    default:return false;
    }
    return w.complete();
}

bool LiveSettings::applyBlock(uint8_t id,const uint8_t* data,uint8_t n,uint32_t alerts) {
    if(!validBlock(id,data,n))return false;
    BlockDecoder d(id,data,n);
    switch(id) {
    case Geometry:
        for(uint8_t a=0;a<3;++a) {
            Printer::setStepsPerMm(a,d.f32(at(Field::F_XAXIS_STEPS_PER_MM,4*a)));
            Printer::setAxisLengthMm(a,d.f32(at(Field::F_X_LENGTH,4*a)));
            const uint8_t mode=d.u8(at(Field::F_RF_MICRO_STEPS_X,a));
            if(Drv8711::microstepMode(a)!=mode)Drv8711::setMicrostepMode(a,mode);
        }
        Printer::manualMotion.setSteps(Z_AXIS,d.u16(Field::F_RF_MOD_Z_STEP_SIZE));break;
    case Feedrates:
        for(uint8_t a=0;a<3;++a) {
            Printer::setAxisMaxFeedrate(a,d.f32(at(Field::F_X_MAX_FEEDRATE,4*a)));
            Printer::setAxisHomingFeedrate(a,d.f32(at(Field::F_X_HOMING_FEEDRATE_PRINT,4*a)));
        }break;
    case Jerk:
        Printer::setXYJerkLimit(d.f32(Field::F_MAX_XYJERK));Printer::setZJerkLimit(d.f32(Field::F_MAX_ZJERK));break;
    case Acceleration:
        for(uint8_t a=0;a<3;++a) {
            Printer::setPrintAcceleration(a,d.f32(at(Field::F_X_MAX_ACCEL,4*a)));
            Printer::setTravelAcceleration(a,d.f32(at(Field::F_X_MAX_TRAVEL_ACCEL,4*a)));
        }
        StepExecutor::setPackingInterval(d.u16(Field::F_RF_STEP_PACKING_MIN_INTERVAL));break;
    case MotorCurrent:
        for(uint8_t a=0;a<5;++a) {
            Drv8711::stageCurrent(a,d.u8(at(Field::F_RF_MOTOR_CURRENT,a)));Drv8711::applyConfiguredCurrent(a);
        }break;
    case Extruder0Motion: case Extruder1Motion: {
        const uint8_t tool=id-Extruder0Motion,delta=tool*SlotSize;
        const uint8_t mode=d.u8(at(Field::F_RF_MICRO_STEPS_E0,delta));
        if(Drv8711::microstepMode(E_AXIS+tool)!=mode)Drv8711::setMicrostepMode(E_AXIS+tool,mode);
        if(tool<NUM_EXTRUDER) {
            auto& e=extruder[tool];
            e.stepsPerMM=d.f32(at(Field::F_E0_STEPS_PER_MM,delta));e.maxFeedrate=d.f32(at(Field::F_E0_MAX_FEEDRATE,delta));
            e.maxEJerk=d.f32(at(Field::F_E0_MAX_START_FEEDRATE,delta));e.maxAcceleration=d.f32(at(Field::F_E0_MAX_ACCELERATION,delta));
            e.advanceL=d.f32(at(Field::F_E0_ADVANCE_L,delta));e.offsetMM[X_AXIS]=d.f32(at(Field::F_E0_X_OFFSET,delta));
            e.offsetMM[Y_AXIS]=d.f32(at(Field::F_E0_Y_OFFSET,delta));
        }break;
    }
    case Hotend0: case Hotend1: {
        const uint8_t tool=id-Hotend0,delta=tool*SlotSize;
        if(tool<NUM_EXTRUDER) {
            auto& e=extruder[tool];
            applyThermal(d,e.tempControl,at(Field::F_E0_PID_PGAIN,delta),at(Field::F_E0_PID_IGAIN,delta),
                at(Field::F_E0_PID_DGAIN,delta),at(Field::F_E0_DRIVE_MIN,delta),at(Field::F_E0_DRIVE_MAX,delta),
                at(Field::F_E0_PID_MAX,delta),at(Field::F_E0_SENSOR_TYPE,delta),at(Field::F_PID_CALIBRATION_SLOT_0,delta));
            e.coolerSpeed=d.u8(at(Field::F_E0_COOLER_SPEED,delta));
        }break;
    }
    case BedPid:
        applyThermal(d,heatedBedController,Field::F_BED_PID_PGAIN,Field::F_BED_PID_IGAIN,Field::F_BED_PID_DGAIN,
            Field::F_BED_DRIVE_MIN,Field::F_BED_DRIVE_MAX,Field::F_BED_PID_MAX,Field::F_RF_HEATED_BED_SENSOR_TYPE,
            Field::F_PID_CALIBRATION_SLOT_2);break;
    case BedSafety:bedCutoff_=(alerts&blockBit(BedSafety))?0:d.u8(Field::F_NEW_BED_EMERGENCY_C);break;
    case Preheat:
        Printer::preheatProfiles.readStored(PreheatProfiles::Profile::A,{int16_t(d.u16(Field::F_RF_PREHEAT_A_BED)),int16_t(d.u16(Field::F_RF_PREHEAT_A_HOTEND))});
        Printer::preheatProfiles.readStored(PreheatProfiles::Profile::B,{int16_t(d.u16(Field::F_RF_PREHEAT_B_BED)),int16_t(d.u16(Field::F_RF_PREHEAT_B_HOTEND))});break;
    case ZOffset:Printer::zOffset.setManualMicrometers(int32_t(d.u32(Field::F_RF_Z_OFFSET)),Printer::stepsPerMm(Z_AXIS));break;
    case SenseOffset: {
        const int16_t digits=d.u16(Field::F_RF_MOD_SENSEOFFSET_DIGITS),limit=d.u16(Field::F_RF_MOD_SENSEOFFSET_OFFSET_MAX);
        Printer::senseOffsetControl.publishStoredLimits(digits,limit);
        Printer::senseOffsetControl.setMaxMicrometers(limit);Printer::senseOffsetControl.setAutoStart(d.u8(Field::F_RF_MOD_SENSEOFFSET_AUTOSTART));
        Printer::strainGauge.setZeroingEnabled(d.u8(Field::F_RF_ZERO_DIGIT_STATE)==1);
        Printer::digitCompensation.setDigitZEnabled(d.u8(Field::F_RF_DIGIT_CMP_STATE)==1);break;
    }
    case Pressure: {
        const PressureProtection::Limits p={int16_t(int32_t(d.u32(Field::F_RF_EMERGENCYPAUSEDIGITSMIN))),int16_t(int32_t(d.u32(Field::F_RF_EMERGENCYPAUSEDIGITSMAX)))};
        Printer::pressureProtection.setPauseLimits(p);
        auto flow=Printer::digitCompensation.flowProfile();flow.minDigits=RMath::min(short(abs(p.maximum)*0.7),4000);
        flow.maxDigits=short(abs(p.maximum));Printer::digitCompensation.setFlowProfile(flow);
        Printer::pressureProtection.setStopLimits({int16_t(d.u16(Field::F_RF_EMERGENCYZSTOPDIGITSMIN)),int16_t(d.u16(Field::F_RF_EMERGENCYZSTOPDIGITSMAX))});break;
    }
    case Scan:
        restoreZOffsetScanPoint({d.u8(Field::F_RF_MOD_ZOS_SCAN_POINT_X),d.u8(Field::F_RF_MOD_ZOS_SCAN_POINT_Y)});
        setScanStartLiftMm(d.f32(Field::F_ZSCAN_START_MM));break;
    case Wobble:
        Printer::wobbleCorrection.setProfile({int8_t(d.u8(Field::F_RF_MOD_WOBBLE_FIX_PHASEXY)),
            int16_t(d.u16(Field::F_RF_MOD_WOBBLE_FIX_AMPX)),int16_t(d.u16(Field::F_RF_MOD_WOBBLE_FIX_AMPY1)),
            int16_t(d.u16(Field::F_RF_MOD_WOBBLE_FIX_AMPY2))});break;
    case Panel:
        for(uint8_t a=0;a<3;++a)Printer::manualMotion.setMode(a,d.u8(at(Field::F_RF_MOVE_MODE_X,a)));
        Printer::restoreZDisplayMode(d.u8(Field::F_RF_Z_MODE));
        Printer::manualMotion.setCoordinateSystem(d.u8(Field::F_RF_MOVE_MODE_XY_KOSYS));
        Printer::manualMotion.setFeedrateSource(d.u8(Field::F_RF_MOVE_POSITION_FEEDRATE));break;
    case Lighting:
        AccessoryOutputs::stage(AccessoryOutputs::Output::CaseLight,d.u8(Field::F_RF_CASE_LIGHT_MODE));
#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
        if(!HOTEND_COOLER_USES_PIN(FET1))AccessoryOutputs::stage(AccessoryOutputs::Output::Fet1,d.u8(Field::F_RF_FET1_MODE));
        if(!HOTEND_COOLER_USES_PIN(FET2))AccessoryOutputs::stage(AccessoryOutputs::Output::Fet2,d.u8(Field::F_RF_FET2_MODE));
        if(CASE_FAN_PIN<0 && !HOTEND_COOLER_USES_PIN(FET3))AccessoryOutputs::stage(AccessoryOutputs::Output::Fet3,d.u8(Field::F_RF_FET3_MODE));
#endif
        break;
    case PartFan:
        HAL::setPartFanDutyRange(d.u8(Field::F_RF_PART_FAN_PWM_MIN),d.u8(Field::F_RF_PART_FAN_PWM_MAX));
        Commands::adjustFanFrequency(d.u8(Field::F_RF_PART_FAN_SPEED));Commands::adjustFanMode(d.u8(Field::F_RF_FAN_MODE));break;
    case System:
        Printer::inactivity.setShutdownTimeout(d.u32(Field::F_MAX_INACTIVE_TIME));
        Printer::inactivity.setStepperTimeout(d.u32(Field::F_STEPPER_INACTIVE_TIME));break;
    case Usage:
        // Accounting is intentionally handled by its own owner at cutover.
        break;
    default:return false;
    }
    return d.good();
}

bool LiveSettings::applyDriveLimits(uint8_t id,const uint8_t* data,uint8_t n,
                                    const Patch* edits,uint8_t count) {
    if(!edits || !count || !validBlock(id,data,n)) return false;
    TemperatureController* controller;
    Field min,max;
    if(id==BedPid) {
        controller=&heatedBedController;
        min=Field::F_BED_DRIVE_MIN;max=Field::F_BED_DRIVE_MAX;
    } else if(id>=Hotend0 && id<Hotend0+NUM_EXTRUDER) {
        const uint8_t tool=id-Hotend0, delta=tool*SlotSize;
        controller=&extruder[tool].tempControl;
        min=at(Field::F_E0_DRIVE_MIN,delta);max=at(Field::F_E0_DRIVE_MAX,delta);
    } else return false;
    bool setMin=false,setMax=false;
    for(uint8_t a=0;a<count;++a) {
        const auto& edit=edits[a];
        if(blockOf(edit.pos)!=id) continue;
        if(edit.width!=1) return false;
        if(edit.pos==address(min)) setMin=true;
        else if(edit.pos==address(max)) setMax=true;
        else return false;
    }
    if(!setMin && !setMax) return false;
    BlockDecoder d(id,data,n);
    const uint8_t low=d.u8(min),high=d.u8(max);
    if(!d.good()) return false;
    if(setMin)controller->pidDriveMin=low;
    if(setMax)controller->pidDriveMax=high;
    controller->updateTempControlVars();
    return true;
}

void LiveSettings::finishMotion() {
    // Caller has already inhibited motion, revoked heater permission and
    // performed complete load. Never invoke for a partial/failed load.
    Printer::manualMotion.setSteps(X_AXIS,uint16_t(lroundf(Printer::stepsPerMm(X_AXIS)*DEFAULT_MANUAL_MM_X)));
    Printer::manualMotion.setSteps(Y_AXIS,uint16_t(lroundf(Printer::stepsPerMm(Y_AXIS)*DEFAULT_MANUAL_MM_Y)));
    Printer::manualMotion.setSteps(E_AXIS,uint16_t(lroundf(Extruder::current->stepsPerMM*DEFAULT_MANUAL_MM_E)));
    Printer::zOffset.refreshSteps(Printer::stepsPerMm(Z_AXIS));
    Printer::layers.resetFadeBounds(Printer::stepsPerMm(Z_AXIS));
    Printer::updateDerivedParameter();
    Extruder::applyCurrentSettings();
    Printer::applyAdvanceSettings();
    Extruder::initHeatedBed();
}
}

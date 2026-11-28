#!/usr/bin/env python3
"""Task0010: actual sensor, controller, PWM and scheduler safety replays.

GAP marks unresolved sensor validity behavior, not a safety requirement.
Software duty requests and simulated pin transitions are distinct; this is
not physical sensor/relay or AVR timing certification.
"""
import os
from pathlib import Path
import re
import subprocess
from tempfile import TemporaryDirectory

FW = Path(__file__).resolve().parents[1] / "Repetier"


def section(source, start, end):
    a = source.index(start)
    return source[a:source.index(end, a)]


def macro(source, name):
    return re.search(r"(?m)^\s*#define\s+" + name + r"\s+([^/\n]+)", source)[1].strip()


HARNESS = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <algorithm>
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
#include "ThermalEmergency.h"
#include "BedTemperatureLimit.h"
// Test transport for actual report calls; no blocking dependency.
static bool diagnosticTxBlocked=false;
static unsigned diagnosticLines=0;
struct UsbSerial {
    static bool tryWriteSpan(const char*,uint8_t count) {
        assert(count<=63);
        if(diagnosticTxBlocked)return false;
        ++diagnosticLines;return true;
    }
};
#endif
using millis_t = uint32_t;
#define PROGMEM
#define TEMP_INT_TO_FLOAT(t) ((float)(t) / (float)(1 << CELSIUS_EXTRA_BITS))
#define constrain(x,a,b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
#define LOW 0
#define HIGH 1
#define LED_PIN -1
#define EXT0_HEATER_PIN 0
#define EXT1_HEATER_PIN 1
#define HEATED_BED_HEATER_PIN NUM_EXTRUDER
#define EXT0_EXTRUDER_COOLER_PIN -1
#define EXT1_EXTRUDER_COOLER_PIN -1
#define FAN_BOARD_PIN -1
#define WATCHDOG_PIN 5
#define NUM_TEMPERATURE_LOOPS (NUM_EXTRUDER + 1)
#define PRINTER_FLAG2_GOT_TEMPS 1
#define PSTR(s) s
#define UI_STATUS_UPD(x) do { if (::delayAutotuneAdmission) { ::now += 101; ::delayAutotuneAdmission = false; } } while (0)
static bool delayAutotuneAdmission;
static uint8_t pins[8];
#define WRITE(pin,value) (pins[pin] = (value))
#define READ(pin) (pins[pin])
static uint16_t PWM_OCR;
static millis_t now;
constexpr uint8_t TemperatureAdcCount = NUM_TEMPERATURE_LOOPS;
#include "adc-state.inc"
static int stalledChannel = -1;
static void publishAdc() {
    for (uint8_t i = 0; i < TemperatureAdcCount; ++i) {
        if (i != stalledChannel) {
            temperatureAdcPublishedAt[i] = now;
            temperatureAdcReady[i] = true;
        }
    }
}
struct InterruptProtectedBlock {};
static uint8_t pgm_read_byte(const uint8_t* p) { return *p; }
static float pgm_read_float(const float* p) { return *p; }
static int16_t pgm_read_word(const int16_t* p) { return *p; }
static const int16_t* pgm_read_word(const int16_t* const* p) { return *p; }
struct HAL {
    static millis_t timeInMilliseconds() { return now; }
    static uint16_t temperatureAdcSample(uint8_t);
    static bool temperatureAdcSample(uint8_t, uint16_t&
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
                                     ,uint32_t* publication=nullptr
#endif
    );
    static void setHeaterDuty(uint8_t, uint8_t);
    static uint8_t outputDuty(uint8_t);
    static void tellWatchdogOk();
    static bool takeScanTick();
    static bool takeCompensationTick();
    static bool takeFeatureTick();
    static bool takeTemperatureTick();
    static bool temperatureTickPending();
    static bool watchdogEnabled();
};
#include "controller.inc"
struct Extruder {
    TemperatureController tempControl;
    uint8_t coolerPWM, coolerSpeed;
    static Extruder* current;
    static bool allHeatersCalibrated();
    static void manageTemperatures();
    static void setHeatedBedTemperature(float);
};
static Extruder extruder[] = {
    {{0, 13, 0, 280}, 0, 255},
#if NUM_EXTRUDER > 1
    {{1, 13, 1, 280}, 0, 255},
#endif
};
Extruder* Extruder::current = &extruder[0];
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
static TemperatureController bed{NUM_EXTRUDER, 13, NUM_EXTRUDER, TemperatureController::BedAbsoluteCeilingC};
#else
static TemperatureController bed{NUM_EXTRUDER, 13, NUM_EXTRUDER, BED_MEASURED_CUTOFF};
#endif
static TemperatureController& heatedBedController = bed;
static TemperatureController* tempController[NUM_TEMPERATURE_LOOPS] = {
    &extruder[0].tempControl,
#if NUM_EXTRUDER > 1
    &extruder[1].tempControl,
#endif
    &bed
};
#define INLINE inline
#define EXTRUDER_TIMSK advanceTimerMask
#define EXTRUDER_OCIE 1
#define _BV(n) (1U << (n))
static uint8_t advanceTimerMask;
#include "ExtrusionOutput.h"
void ExtrusionOutput::startTimer(int16_t pending, uint8_t mask) {
    if (pending) advanceTimerMask = mask | _BV(EXTRUDER_OCIE);
}
void ExtrusionOutput::stopTimer() { advanceTimerMask = 0; }

namespace Printer {
    static uint8_t flag0;
    static ExtrusionOutput extrusionOutput;
    #include "printer-safety.inc"
    static unsigned stops;
    static struct {
        bool busy = false;
        bool isBusy() const { return busy; }
        bool isIdle() const { return !busy; }
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        bool emergencyLocked() const { return isAnyTempsensorDefect(); }
#endif
    } printJob;
    static unsigned flag2, debugLevel;
    static void markTemperatureReadingsAvailable() { flag2 |= PRINTER_FLAG2_GOT_TEMPS; }
    static void stopForThermalFault() {
        for (auto* t : tempController)
            assert(t->targetTemperatureC == 0 && HAL::outputDuty(t->pwmIndex) == 0);
        ++stops;
    }
    static bool debugInfo() { return false; }
    struct { void cancel() {} void schedule(millis_t) {} } idleStatus;
    struct { bool owned = false; bool isReheating() { return true; } bool blocksCalibration() const { return owned; } } pause;
    struct { void service(millis_t) {} } strainGauge;
}
namespace PrintLine { bool hasLines() { return false; } bool directMoveActive() { return false; } }
static bool scanActive = false;
static bool isAnyScanRunning() { return scanActive; }
static unsigned relayReports,relayUnavailable;
namespace Com {
    #include "com.inc"
    constexpr int tTargetBedColon = 0;
    template<class... T> void printErrorFLN(T...) {}
    template<class... T> void printInfoFLN(T...) {}
    template<class... T> void printF(T...) {}
    template<class... T> void printFLN(T...) {}
    void printFLN(const char* msg) {
        if(!std::strcmp(msg,"Thermal relay observation (diagnostic only)")) {
            for(auto* c:tempController)assert(c->targetTemperatureC==0&&HAL::outputDuty(c->pwmIndex)==0);
            ++relayReports;
        }
        if(!std::strcmp(msg,"Thermal relay observation unavailable"))++relayUnavailable;
    }
    void println() {}
}
static unsigned stores;
static TemperatureController* tuned;
namespace EEPROM {
    void storePidCalibration(uint8_t heater) {
        assert(tuned == tempController[heater] && tuned->pidCalibrationValid());
        assert(tuned && tuned->targetTemperatureC == 0 && HAL::outputDuty(tuned->pwmIndex) == 0);
        ++stores;
    }
}
namespace RMath { using std::min; using std::max; }
static int ui_text_temperature_manager, ui_text_sensor_error, ui_text_heater_error;
static int ui_text_autodetect_pid, ui_text_temperature_wrong, ui_text_timeout;
static void showError(void*, void*) {
    if (tuned) assert(HAL::outputDuty(tuned->pwmIndex) == 0);
}
enum FirmwareState { NotBusy, WaitHeater };
struct Commands {
    static void checkForPeriodicalActions(FirmwareState);
    static void printTemperatures() {}
};
static void reportTempsensorAndHeaterErrors();
#include "conversion.inc"
#include "hal.inc"
#include "control.inc"
#include "bed-setter.inc"
static unsigned visits;
static unsigned diagnosticRefreshes, ordinaryServices;
static unsigned observedPeakDuty;
static millis_t stride = 10;
enum class Injection { None, Reach, LateUnsupported, LateLow, Transient, High, TransientHigh, Cutoff, Cycles, ProfileCycles, Unstable, PowerChange, OtherFault, Cancel, Reentrant, Stale };
static Injection injection;
static bool postTemperature = true;
namespace GCodeInput {
    bool scriptActive = false;
    bool flashScriptActive() { return scriptActive; }
    void keepAlive(FirmwareState) {
        ++visits;
        if (tuned) observedPeakDuty = std::max(observedPeakDuty, unsigned(HAL::outputDuty(tuned->pwmIndex)));
        assert(visits < 100000);
        now += stride;
        if (injection == Injection::Stale) stalledChannel = tuned->sensorPin;
        publishAdc();
        scanTick = true;
        if (postTemperature) temperatureTick = true;
        if (injection == Injection::Reach && visits == 3) {
            extruder[0].tempControl.sensorType = 60;
            temperatureAdcValues[0] = 102;
        }
        if (injection == Injection::LateUnsupported || injection == Injection::Transient)
            tuned->sensorType = 255;
        if (injection == Injection::LateLow) {
            tuned->sensorType = 14;
            temperatureAdcValues[tuned->sensorPin] = 4092;
        }
        if (injection == Injection::High || injection == Injection::TransientHigh)
            temperatureAdcValues[tuned->sensorPin] =
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
                tuned==&bed?476:
#endif
                820; // relative overshoot; bed fixture ~108..116 C below absolute 120 C
        if (injection == Injection::Cutoff)
            temperatureAdcValues[tuned->sensorPin] = 1195; // valid converted range, above 280 C cutoff
        if (injection == Injection::Cancel) {
            if (tuned == &bed)
                Extruder::setHeatedBedTemperature(0);
            else
                tuned->setTargetTemperature(0);
            assert(HAL::outputDuty(tuned->pwmIndex) == 0);
        }
        if (injection == Injection::Reentrant && visits == 1) {
            const auto oldDuty = HAL::outputDuty(tuned->pwmIndex);
            tuned->autotunePID(100, tuned->pwmIndex, 5, false, 0);
            assert(autotuneIndex == tuned->pwmIndex && HAL::outputDuty(tuned->pwmIndex) == oldDuty);
            tuned->setTargetTemperature(0); // terminate original tune after checking guard
        }
        if (injection == Injection::Cycles)
            temperatureAdcValues[tuned->sensorPin] = visits % 2 ? 470 : 330;
        if (injection == Injection::ProfileCycles || injection == Injection::Unstable)
            temperatureAdcValues[tuned->sensorPin] = (visits - 1) % 4 < 2
                ? ((injection == Injection::Unstable && ((visits - 1)/4)%2) ? 465 : 445) : 390;
        if (injection == Injection::PowerChange) tuned->pidMax = 10;
        if (injection == Injection::OtherFault)
            assert(tuned != &bed); // bed already has 10 consecutive invalid samples

    }
}
static struct {
    millis_t lastButtonTime() { return 0; }
    void refreshPage() {++diagnosticRefreshes;}
    void refreshEmergencyPage() {++diagnosticRefreshes;}
    void slowAction() {
        ++ordinaryServices;
        if (injection == Injection::Transient || injection == Injection::TransientHigh) {
            assert(HAL::outputDuty(tuned->pwmIndex) == 0);
            tuned->sensorType = 60; // valid again before autotune's next read
            temperatureAdcValues[tuned->sensorPin] = 330;
        }
    }
} uid;
static void recalculateZCompensation() {++ordinaryServices;}
static void loopFeatures() {++ordinaryServices;}
static void handleScanWorkTasks(bool) {++ordinaryServices;}
namespace Lighting { void serviceAlarm(uint32_t) {} }
#include "scheduler.inc"

static void service() {
    now += 100;
    publishAdc();
    Extruder::manageTemperatures();
}
static void allOff() {
    assert(Printer::isAnyTempsensorDefect() && Printer::stops == 1 && Printer::heatingInhibited());
    assert(Printer::debugLevel == 0 && Printer::extrusionOutput.blocked());
    assert(Printer::extrusionOutput.pendingSteps() == 0 && advanceTimerMask == 0);
    Printer::extrusionOutput.allow(ExtrusionOutput::Inhibit::PrintStop);
    Printer::extrusionOutput.allow(ExtrusionOutput::Inhibit::MotorsDisabled);
    Printer::extrusionOutput.allow(ExtrusionOutput::Inhibit::Pressure);
    assert(Printer::extrusionOutput.blocked());
    for (auto* c : tempController)
        assert(c->targetTemperatureC == 0 && HAL::outputDuty(c->pwmIndex) == 0);
}
int main(int argc, char** argv) {
    assert(argc == 2);
    const char* name = argv[1];
    auto& c = extruder[0].tempControl;
    for (auto* t : tempController) {
        t->sensorType = 60;
        temperatureAdcValues[t->sensorPin] = 102; // plausible room-temperature sample
        t->pidMax = 255;
        t->targetTemperatureC = 100;
        t->pidPGain = 20;
        t->pidIGain = 0.5;
        t->pidDGain = 3;
        t->pidDriveMax = 255;
        t->pidDriveMin = 20;
        t->tunedSensorType = t->sensorType;
        t->tunedPowerLimit = t->pidMax;
        t->updateTempControlVars();
        t->decoupleTestPeriod = EXT0_DECOUPLE_TEST_PERIOD;
    }
    publishAdc();
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if (!std::strcmp(name,"bed-limit")) {
        assert(bed.heaterCutoffC==150&&bed.effectiveCutoffC()==120&&bed.maximumTargetC()==110);
        bed.setTargetTemperature(110);
        assert(bed.targetTemperatureC==110);
        bed.setTargetTemperature(111);
        assert(!bed.targetTemperatureC&&!HAL::outputDuty(bed.pwmIndex));
        Extruder::setHeatedBedTemperature(180);
        assert(bed.targetTemperatureC==110);
        bed.targetTemperatureC=111;
        service();
        assert(!bed.targetTemperatureC&&!HAL::outputDuty(bed.pwmIndex));
        assert(bed.setBedEmergencyCutoff(90));
        assert(bed.effectiveCutoffC()==90 && bed.maximumTargetC()==80);
        bed.setTargetTemperature(80);
        assert(bed.targetTemperatureC==80);
        HAL::setHeaterDuty(bed.pwmIndex,255);
        assert(bed.setBedEmergencyCutoff(0));
        assert(!bed.targetTemperatureC && !HAL::outputDuty(bed.pwmIndex));
        assert(bed.effectiveCutoffC()==90 && bed.maximumTargetC()==0);
        Extruder::setHeatedBedTemperature(100);
        assert(!bed.targetTemperatureC);
        assert(!bed.setBedEmergencyCutoff(151));
        assert(bed.effectiveCutoffC()==90 && bed.maximumTargetC()==0);
        assert(!c.setBedEmergencyCutoff(120)); // cannot publish via hotend
        assert(bed.setBedEmergencyCutoff(150));
        assert(bed.effectiveCutoffC()==150 && bed.maximumTargetC()==140);
        assert(!bed.targetTemperatureC && !HAL::outputDuty(bed.pwmIndex));
    } else if (!std::strncmp(name,"absolute-",9)) {
        using namespace ThermalEmergency;
        const bool bedCase=std::strcmp(name,"absolute-hotend")!=0;
        auto& act=bedCase?bed:c;
        const bool escalate=!std::strcmp(name,"absolute-escalation");
        const bool idle=!std::strcmp(name,"absolute-idle");
        SensorObservation valid;valid.observe(true,1195);
        // Existing strict boundary, not a new numerical threshold.
        act.currentTemperatureC=act.effectiveCutoffC();
        assert(!checkAbsoluteCutoff(act.pwmIndex,act,true,valid));
        service();
        if(idle)for(auto* h:tempController)h->setTargetTemperature(0);
        if(escalate) {
            c.sensorType=255;
            for(unsigned i=0;i<11;++i)service();
            assert(faultLedger().first().cause==Cause::SensorInvalid);
        }
        const float oldTarget=act.targetTemperatureC;
        const uint8_t oldDuty=HAL::outputDuty(act.pwmIndex);
        temperatureAdcValues[act.sensorPin]=1195;
        service();
        allOff();
        assert(faultLedger().level()==Severity::FireWarning);
        const auto& fire=faultLedger().firstFire();
        assert(fire.cause==Cause::OverTemperature&&fire.channel==act.pwmIndex);
        assert(fire.evidence.measuredC>act.effectiveCutoffC()&&fire.evidence.measuredC<=MAX_DEFECT_TEMPERATURE);
        assert(fire.evidence.threshold==act.effectiveCutoffC()&&fire.evidence.deltaC>0);
        assert(fire.evidence.rawAdc==1195&&(fire.evidence.valid&Evidence::Raw));
        assert(fire.evidence.targetC==oldTarget&&fire.evidence.admittedDuty==oldDuty);
        if(escalate)assert(faultLedger().first().cause==Cause::SensorInvalid);
        else assert(faultLedger().first().cause==Cause::OverTemperature);
        temperatureAdcValues[act.sensorPin]=102;
        act.setTargetTemperature(100);
        service();
        allOff();
    } else if (!std::strcmp(name, "emergency-invalid-episode")) {
        using namespace ThermalEmergency;
        service();
        assert(HAL::outputDuty(0)==255);
        c.sensorType=14;temperatureAdcValues[0]=4092;
        service();
        assert(HAL::outputDuty(0)==0&&!faultLedger().active());
        // A recovered episode must not be reported as a later confirmed
        // fault. The second episode has distinct type/target/time/duty.
        c.sensorType=60;temperatureAdcValues[0]=102;
        service();
        assert(HAL::outputDuty(0)==255&&!faultLedger().active());
        c.setTargetTemperature(140);
        c.sensorType=14;temperatureAdcValues[0]=0;
        const auto started=now+100;
        service();
        assert(HAL::outputDuty(0)==0&&!faultLedger().active());
        c.setTargetTemperature(120);
        diagnosticTxBlocked=true;
        for(unsigned i=0;i<11&&!Printer::heatingInhibited();++i)service();
        assert(diagnosticLines==0);
        for(unsigned i=0;i<10;++i)service(); // blocked TX cannot stall thermal pass
        assert(diagnosticLines==0);
        diagnosticTxBlocked=false;
        for(unsigned i=0;i<100;++i)service();
        assert(diagnosticLines>2);
        allOff();
        const auto& first=faultLedger().first();
        assert(first.cause==Cause::SensorInvalid&&first.channel==0);
        const auto& e=first.evidence;
        assert(e.uptimeMs==started&&e.targetC==140&&e.admittedDuty==255);
        assert(e.rawAdc==0&&(e.valid&Evidence::Raw));
        assert(!(e.valid&Evidence::Measured)&&!(e.valid&Evidence::Requested));
        assert(e.sensorType==14&&e.powerLimit==255);
        // Additional sensor fault after global latch: original evidence
        // survives, affected heater mask grows, no new stop/heat.
        bed.sensorType=255;
        for(unsigned i=0;i<15;++i)service();
        assert(faultLedger().reasons(NUM_EXTRUDER)&causeBit(Cause::SensorInvalid));
        assert(faultLedger().first().evidence.uptimeMs==started);
        assert(faultLedger().first().evidence.targetC==140);
        assert(faultLedger().level()==Severity::ThermalError);
        for(unsigned h=0;h<NUM_EXTRUDER;++h)assert(extruder[h].coolerPWM==255);
        allOff();
    } else if (!std::strcmp(name, "emergency-stale-episode")) {
        using namespace ThermalEmergency;
        service();
        assert(HAL::outputDuty(0)==255);
        stalledChannel=0;
        // First stale batch is diagnosed before clearing its duty.
        now+=101;
        const auto started=now+100;
        for(unsigned i=0;i<11;++i)service();
        allOff();
        const auto& r=faultLedger().first();
        assert(r.cause==Cause::StaleSample&&r.channel==0);
        assert(r.evidence.uptimeMs==started&&r.evidence.admittedDuty==255);
        assert(!(r.evidence.valid&(Evidence::Raw|Evidence::Measured)));
    } else if (!std::strcmp(name, "emergency-scheduler")) {
        using namespace ThermalEmergency;
        latchSystemFault(Cause::WatchdogReset);
        const auto initialVisits=visits,initialOrdinary=ordinaryServices,initialRefresh=diagnosticRefreshes;
        diagnosticTxBlocked=true;
        for(unsigned n=0;n<100;++n) {
            now+=100;publishAdc();
            scanTick=compensationTick=featureTick=temperatureTick=true;
            Commands::checkForPeriodicalActions(WaitHeater);
            assert(!scanTick&&!compensationTick&&!featureTick&&!temperatureTick);
            assert(visits==initialVisits&&ordinaryServices==initialOrdinary);
            assert(diagnosticRefreshes==initialRefresh+n+1);
            for(auto* h:tempController)assert(HAL::outputDuty(h->pwmIndex)==0);
            for(unsigned h=0;h<NUM_EXTRUDER;++h)assert(extruder[h].coolerPWM==255);
        }
        assert(diagnosticLines==0);
        diagnosticTxBlocked=false;
        for(unsigned n=0;n<100;++n) {
            now+=100;publishAdc();temperatureTick=true;
            Commands::checkForPeriodicalActions(NotBusy);
        }
        assert(diagnosticLines>2&&ordinaryServices==initialOrdinary);
    } else if (!std::strcmp(name, "emergency-recovery")) {
        using namespace ThermalEmergency;
        latchSystemFault(Cause::InterruptedHeating);
        assert(Printer::heatingInhibited()&&faultLedger().active());
        assert(faultLedger().first().cause==Cause::InterruptedHeating);
        assert(faultLedger().first().channel==NUM_TEMPERATURE_LOOPS);
        assert(faultLedger().first().evidence.valid==0);
        latchSystemFault(Cause::WatchdogReset);
        assert(faultLedger().first().cause==Cause::InterruptedHeating);
        assert(faultLedger().level()==Severity::ThermalError);
        for(auto* t:tempController)assert(t->targetTemperatureC==0&&HAL::outputDuty(t->pwmIndex)==0);
    } else
#endif
    if (!std::strcmp(name, "pid-components")) {
        for (auto* t : tempController) t->setTargetTemperature(0);
        temperatureAdcValues[c.sensorPin] = 330;
        assert(c.updateCurrentTemperature());
        const float measured = c.currentTemperatureC;
        c.pidDriveMax = 100;
        c.pidDriveMin = 30;
        c.pidPGain = 4;
        c.pidIGain = 2;
        c.pidDGain = 3;
        c.updateTempControlVars();
        c.setTargetTemperature(measured + 5);
        c.pidHistoryValid = true;
        c.previousTemperature = measured + 3;
        c.filteredSlope = 0;
        c.lastTemperatureUpdate = now;
        c.tempIState = 0;
        service();
        assert(HAL::outputDuty(c.pwmIndex) == 36); // P=20, I=1, filtered D=15
        assert(std::fabs(c.tempIState - 1) < 0.001f);
        // 200 ms elapsed: integrate actual time, not a hard-coded 100 ms.
        now += 100;
        c.previousTemperature = measured;
        c.filteredSlope = 0;
        service();
        assert(HAL::outputDuty(c.pwmIndex) == 23);
        assert(std::fabs(c.tempIState - 3) < 0.001f);
        c.setTargetTemperature(measured + 50);
        c.pidPGain = 100;
        c.filteredSlope = 0;
        service();
        assert(HAL::outputDuty(c.pwmIndex) == c.pidMax && std::fabs(c.tempIState - 3) < 0.001f);
        c.setTargetTemperature(measured - 5);
        service();
        assert(HAL::outputDuty(c.pwmIndex) == 0);
        c.setTargetTemperature(0);
        service();
        assert(HAL::outputDuty(c.pwmIndex) == 0 && c.tempIState == 0);
    } else if (!std::strcmp(name, "untuned") || !std::strcmp(name, "bed-untuned")) {
        auto& t = !std::strcmp(name, "bed-untuned") ? bed : c;
        assert(Extruder::allHeatersCalibrated());
        t.invalidatePidCalibration();
        assert(!Extruder::allHeatersCalibrated());
        assert(!t.pidCalibrationValid());
        t.setTargetTemperature(100);
        service();
        assert(t.targetTemperatureC == 0 && HAL::outputDuty(t.pwmIndex) == 0);
        // A raw target write cannot bypass the regulator's authorization.
        t.targetTemperatureC = 100;
        service();
        assert(t.targetTemperatureC == 0 && HAL::outputDuty(t.pwmIndex) == 0);
    } else if (!std::strcmp(name, "invalid-gains")) {
        assert(!TemperatureController::validPidGains(NAN, 1, 1));
        assert(!TemperatureController::validPidGains(1, INFINITY, 1));
        assert(!TemperatureController::validPidGains(1, 1, -1));
        assert(!TemperatureController::validPidGains(0, 1, 0));
        c.pidPGain = NAN;
        service();
        assert(!c.pidCalibrationValid() && HAL::outputDuty(c.pwmIndex) == 0);
    } else if (!std::strcmp(name, "stale") || !std::strcmp(name, "bed-stale")) {
        auto& affected = !std::strcmp(name, "bed-stale") ? bed : c;
        auto& other = &affected == &bed ? c : bed;
        service();
        assert(HAL::outputDuty(affected.pwmIndex) == 255);
        stalledChannel = affected.sensorPin;
        ++now;
        const auto oldSampleTime = affected.lastTemperatureUpdate;
        service();
        assert(HAL::outputDuty(affected.pwmIndex) == 0 && !Printer::isAnyTempsensorDefect());
        assert(HAL::outputDuty(other.pwmIndex) == 255 && affected.lastTemperatureUpdate == oldSampleTime && !affected.pidHistoryValid);
        stalledChannel = -1;
        service();
        assert(HAL::outputDuty(affected.pwmIndex) == 255 && !Printer::isAnyTempsensorDefect());
    } else if (!std::strcmp(name, "unpublished")) {
        temperatureAdcReady[c.sensorPin] = false;
        stalledChannel = c.sensorPin;
        HAL::setHeaterDuty(c.pwmIndex, 255);
        service();
        assert(HAL::outputDuty(c.pwmIndex) == 0 && HAL::outputDuty(bed.pwmIndex) == 255);
        assert(!Printer::isAnyTempsensorDefect() && !c.pidHistoryValid);
    } else if (!std::strcmp(name, "rail")) {
        c.sensorType = 13;
        temperatureAdcValues[0] = 4092;
        service();
        // The numeric conversion is preserved, but rail samples cannot heat.
        assert(c.currentTemperatureC == 0 && !Printer::isAnyTempsensorDefect());
        assert(HAL::outputDuty(0) == 0);
        c.setTargetTemperature(0);
        assert(HAL::outputDuty(0) == 0);
    } else if (!std::strcmp(name, "invalid-low")) {
        c.sensorType = 14;
        temperatureAdcValues[0] = 4092;
        for (int i = 0; i < 10; ++i) {
            HAL::setHeaterDuty(0, 255);
            service();
            assert(c.currentTemperatureC < MIN_DEFECT_TEMPERATURE);
            assert(!Printer::isAnyTempsensorDefect() && HAL::outputDuty(0) == 0);
            assert(HAL::outputDuty(bed.pwmIndex) == 255);
            assert(!c.pidHistoryValid); // invalid readings never enter the PID history
        }
        service();
        assert(c.isSensorDefect());
        allOff();
    } else if (!std::strcmp(name, "recovery")) {
        service();
        assert(HAL::outputDuty(0) == 255);
        c.sensorType = 14;
        temperatureAdcValues[0] = 4092;
        service();
        assert(HAL::outputDuty(0) == 0 && !Printer::isAnyTempsensorDefect());
        c.sensorType = 60;
        temperatureAdcValues[0] = 102;
        service();
        assert(HAL::outputDuty(0) == 255 && !Printer::isAnyTempsensorDefect());
    } else if (!std::strcmp(name, "latched-fault-wait")) {
        service();
        assert(HAL::outputDuty(c.pwmIndex) == 255 && HAL::outputDuty(bed.pwmIndex) == 255);
        Printer::extrusionOutput.addSteps(20);
        assert(advanceTimerMask != 0);
        bed.sensorType = 255;
        for (int i = 0; i < 10; ++i) service();
        assert(!Printer::isAnyTempsensorDefect());
        c.waitForTargetTemperature();
        assert(visits == 1 && watchdogHeartbeat == now);
        allOff();
        for (auto mask : {0, 7, 255}) {
            Printer::debugLevel = mask;
            c.setTargetTemperature(180);
            Extruder::setHeatedBedTemperature(60);
            c.waitForTargetTemperature();
            assert(visits == 1);
            service();
            assert(Printer::isAnyTempsensorDefect() && Printer::extrusionOutput.blocked());
            for (auto* t : tempController)
                assert(t->targetTemperatureC == 0 && HAL::outputDuty(t->pwmIndex) == 0);
        }
    } else if (!std::strcmp(name, "unconditional-clear")) {
        c.tempIState = 17;
        c.pidHistoryValid = true;
        c.setDecoupleFull();
        c.invalidatePidCalibration();
        c.targetTemperatureC = 180;
        HAL::setHeaterDuty(c.pwmIndex, 180);
        autotuneIndex = c.pwmIndex;
        autotuneAbortRequested = false;
        delayAutotuneAdmission = true;
        const auto before = now;
        c.clearTargetTemperature();
        assert(c.targetTemperatureC == 0 && HAL::outputDuty(c.pwmIndex) == 0);
        assert(c.tempIState == 0 && !c.pidHistoryValid);
        assert(!c.isDecoupleFull() && !c.isDecoupleHold());
        assert(autotuneAbortRequested);
        assert(delayAutotuneAdmission && now == before);
        autotuneIndex = 255;
        delayAutotuneAdmission = false;
    } else if (!std::strcmp(name, "legacy-output-inhibit")) {
        service();
        pwmSlice();
        for (auto* t : tempController) assert(pins[t->pwmIndex] == HIGH);
        Printer::extrusionOutput.addSteps(20);
        Printer::latchOutputInhibit(false);
        for (auto* t : tempController) assert(pins[t->pwmIndex] == LOW);
        assert(!Printer::isAnyTempsensorDefect() && Printer::legacySimulationRejected());
        assert(Printer::extrusionOutput.blocked() && Printer::extrusionOutput.pendingSteps() == 0 && advanceTimerMask == 0);
        for (auto* t : tempController) {
            assert(t->targetTemperatureC == 0 && HAL::outputDuty(t->pwmIndex) == 0);
            t->setTargetTemperature(180);
        }
        Extruder::setHeatedBedTemperature(60);
        c.waitForTargetTemperature();
        assert(visits == 0);
        service();
        for (auto* t : tempController)
            assert(t->targetTemperatureC == 0 && HAL::outputDuty(t->pwmIndex) == 0);
        // A real thermal fault remains independent of the legacy rejection.
        Printer::latchOutputInhibit(true);
        assert(Printer::isAnyTempsensorDefect() && Printer::legacySimulationRejected());
    } else if (!std::strcmp(name, "disabled")) {
        c.sensorType = 0;
        bed.sensorType = 0;
        HAL::setHeaterDuty(0, 255);
        HAL::setHeaterDuty(bed.pwmIndex, 255);
        service();
        assert(!Printer::isAnyTempsensorDefect() && HAL::outputDuty(0) == 0 && HAL::outputDuty(bed.pwmIndex) == 0);
    } else if (!std::strcmp(name, "invalid-high")) {
        c.sensorType = 60;
        temperatureAdcValues[0] = 4092;
        HAL::setHeaterDuty(0, 255);
        service();
        assert(c.currentTemperatureC > MAX_DEFECT_TEMPERATURE);
        assert(!Printer::isAnyTempsensorDefect() && HAL::outputDuty(0) == 0 && !c.pidHistoryValid);
    } else if (!std::strcmp(name, "bed-invalid")) {
        bed.sensorType = 14;
        temperatureAdcValues[bed.sensorPin] = 4092;
        HAL::setHeaterDuty(bed.pwmIndex, 255);
        service();
        assert(HAL::outputDuty(bed.pwmIndex) == 0 && HAL::outputDuty(0) == 255 && !Printer::isAnyTempsensorDefect());
    } else if (!std::strcmp(name, "dual-invalid")) {
#if NUM_EXTRUDER > 1
        auto& second = extruder[1].tempControl;
        second.sensorType = 14;
        temperatureAdcValues[second.sensorPin] = 4092;
        HAL::setHeaterDuty(second.pwmIndex, 255);
        service();
        assert(HAL::outputDuty(second.pwmIndex) == 0 && HAL::outputDuty(0) == 255
            && HAL::outputDuty(bed.pwmIndex) == 255 && !Printer::isAnyTempsensorDefect());
#else
        assert(false);
#endif
    } else if (!std::strcmp(name, "hold-drift")) {
        for (auto* other : tempController) if (other != &c) other->setTargetTemperature(0);
        temperatureAdcValues[0] = 410; // ~100 C in both calibrations
        service();
        assert(c.isDecoupleHold());
        temperatureAdcValues[0] = 318; // drift >20 C but within the +/-30 C PID band
        now += EXT0_DECOUPLE_TEST_PERIOD;
        for (int i = 0; i < 10; ++i) {
            service();
            assert(!Printer::isAnyTempsensorDefect());
        }
        service();
        assert(c.isSensorDecoupled());
        allOff();
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        {
            using namespace ThermalEmergency;
            const auto& first=faultLedger().first();
            assert(first.cause==Cause::HoldFailure&&first.channel==0);
            assert(first.evidence.targetC==100&&first.evidence.threshold==DECOUPLING_TEST_MAX_HOLD_VARIANCE);
            assert(first.evidence.deltaC<-DECOUPLING_TEST_MAX_HOLD_VARIANCE);
        }
#endif
    } else if (!std::strcmp(name, "unsupported")) {
        c.sensorType = 255;
        HAL::setHeaterDuty(0, 255);
        for (int i = 0; i < 10; ++i) {
            service();
            assert(HAL::outputDuty(0) == 0 && !Printer::isAnyTempsensorDefect());
        }
        service();
        assert(c.isSensorDefect());
        allOff();
    } else if (!std::strcmp(name, "no-rise")) {
        for (auto* t : tempController) if (t != &c) t->setTargetTemperature(0);
        service();
        assert(c.isDecoupleFull() && HAL::outputDuty(0) == 255);
        now += EXT0_DECOUPLE_TEST_PERIOD;
        for (int i = 0; i < 10; ++i) {
            service();
            assert(!Printer::isAnyTempsensorDefect() && HAL::outputDuty(0) == 255);
        }
        service();
        assert(c.isSensorDecoupled());
        allOff();
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        {
            using namespace ThermalEmergency;
            const auto& first=faultLedger().first();
            assert(first.cause==Cause::NoRise&&first.channel==0);
            assert(first.evidence.targetC==100&&first.evidence.admittedDuty==255);
            assert(first.evidence.deltaC==0&&first.evidence.threshold==DECOUPLING_TEST_MIN_TEMP_RISE);
            assert(first.evidence.rawAdc==102&&(first.evidence.valid&Evidence::Raw));
            assert(first.evidence.windowMs>EXT0_DECOUPLE_TEST_PERIOD);
        }
#endif
    } else if (!std::strcmp(name, "pwm")) {
        for (auto* t : tempController) HAL::setHeaterDuty(t->pwmIndex, 255);
        pwmSlice();
        for (auto* t : tempController) {
            assert(pins[t->pwmIndex] == HIGH);
            t->setTargetTemperature(0);
            assert(HAL::outputDuty(t->pwmIndex) == 0);
        }
        // Revocation is synchronous, even with the previous full-duty frame latched.
        for (auto* t : tempController) assert(pins[t->pwmIndex] == LOW);
        for (int i = 0; i < 128; ++i) {
            pwmSlice();
            for (auto* t : tempController) assert(pins[t->pwmIndex] == LOW);
        }
    } else if (!std::strcmp(name, "watchdog")) {
        watchdogActive = true;
        now = 100;
        Commands::checkForPeriodicalActions(NotBusy);
        assert(watchdogHeartbeat == 0);
        scanTick = true;
        Commands::checkForPeriodicalActions(NotBusy);
        assert(watchdogHeartbeat == now && !temperatureTick);
        pingWatchdog();
        assert(pins[WATCHDOG_PIN] == 1);
        now += WATCHDOG_MAIN_LOOP_TIMEOUT;
        pingWatchdog();
        assert(pins[WATCHDOG_PIN] == 1);
    } else if (!std::strcmp(name, "wait")) {
        c.sensorType = 13;
        temperatureAdcValues[0] = 4092;
        c.currentTemperatureC = 0;
        c.setTargetTemperature(25);
        injection = Injection::Reach;
        c.waitForTargetTemperature();
        assert(visits == 3 && watchdogHeartbeat == now);
        assert(std::fabs(c.currentTemperatureC - 25) <= TEMP_TOLERANCE && HAL::outputDuty(0) == 0);
    } else if (!std::strncmp(name, "autotune-", 9)) {
        const bool useBed = std::strstr(name, "-bed-") != nullptr;
        tuned = useBed ? &bed : &c;
        auto& t = *tuned;
        for (auto* other : tempController) other->setTargetTemperature(0);
        t.setTargetTemperature(100); // must not restart normal heating after autotune
        HAL::setHeaterDuty(t.pwmIndex, 255);
        t.sensorType = 60;
        temperatureAdcValues[t.sensorPin] = 330;
        float target = 100;
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        if(!std::strcmp(name,"autotune-bed-high"))target=90;
#endif
        bool store = false;
        const float oldP = t.pidPGain, oldI = t.pidIGain, oldD = t.pidDGain;
        const bool entry = std::strstr(name, "entry") != nullptr;
        if (!std::strcmp(name, "autotune-entry-unsupported")) t.sensorType = 255;
        else if (!std::strcmp(name, "autotune-entry-low")) {
            t.sensorType = 14;
            temperatureAdcValues[t.sensorPin] = 4092;
        } else if (!std::strcmp(name, "autotune-entry-stale")) now += 101;
        else if (std::strstr(name, "-entry-admission-stale")) delayAutotuneAdmission = true;
        else if (!std::strcmp(name, "autotune-entry-disabled")) t.sensorType = 0;
        else if (!std::strcmp(name, "autotune-entry-latched")) Printer::latchOutputInhibit(true);
        else if (!std::strcmp(name, "autotune-entry-decoupled")) t.setSensorDecoupled(true);
        else if (!std::strcmp(name, "autotune-entry-legacy-rejected")) Printer::latchOutputInhibit(false);
        else if (!std::strcmp(name, "autotune-entry-high")) temperatureAdcValues[t.sensorPin] = 820;
        else if (!std::strcmp(name, "autotune-entry-absolute")) temperatureAdcValues[t.sensorPin] = 1195;
        else if (!std::strcmp(name, "autotune-entry-target")) target = 500;
        else if (!std::strcmp(name, "autotune-bed-entry-target")) target = HEATED_BED_MAX_TEMP + 1;
        else if (!std::strcmp(name, "autotune-entry-controller")) {}
        else if (std::strstr(name, "-late")) {
            injection = Injection::LateUnsupported;
            postTemperature = false; // fault between periodic temperature services
        } else if (std::strstr(name, "-stale")) {
            injection = Injection::Stale;
            stride = 101;
            postTemperature = false;
        } else if (!std::strcmp(name, "autotune-low")) {
            injection = Injection::LateLow;
            postTemperature = false;
        } else if (!std::strcmp(name, "autotune-transient")) injection = Injection::Transient;
        else if (!std::strcmp(name, "autotune-transient-high")) injection = Injection::TransientHigh;
        else if (!std::strcmp(name, "autotune-cancel") || !std::strcmp(name, "autotune-bed-cancel")) injection = Injection::Cancel;
        else if (!std::strcmp(name, "autotune-reentrant")) injection = Injection::Reentrant;
        else if (!std::strcmp(name, "autotune-high") || !std::strcmp(name, "autotune-bed-high")) {
            injection = Injection::High;
            postTemperature = false;
        } else if (!std::strcmp(name, "autotune-cutoff") || !std::strcmp(name, "autotune-cutoff-periodic")) {
            target = 270;
            injection = Injection::Cutoff;
            postTemperature = std::strcmp(name,"autotune-cutoff-periodic")==0;
        } else if (std::strstr(name, "-power-change")) injection = Injection::PowerChange;
        else if (std::strstr(name, "-timeout")) {
            temperatureAdcValues[t.sensorPin] = 445; // cooling phase, no crossing
            stride = 1000;
        } else if (std::strstr(name, "-no-rise")) stride = 1000;
        else if (std::strstr(name, "-unstable")) { injection = Injection::Unstable; stride = 1000; store = true; }
        else if (std::strstr(name, "-entry-profile")) {}
        else if (std::strstr(name, "-entry-busy")) Printer::printJob.busy = true;
        else if (std::strstr(name, "-entry-pause")) Printer::pause.owned = true;
        else if (std::strstr(name, "-entry-scan")) scanActive = true;
        else if (std::strstr(name, "-entry-script")) GCodeInput::scriptActive = true;
        else if (std::strstr(name, "-success")) {
            injection = Injection::ProfileCycles;
            stride = 1000;
            store = std::strstr(name, "-store") != nullptr;
        } else if (!std::strcmp(name, "autotune-other-fault")) {
            bed.sensorType = 255;
            for (int i = 0; i < 10; ++i) service();
            assert(!Printer::isAnyTempsensorDefect());
            injection = Injection::OtherFault;
        } else assert(false);
        const uint8_t id = !std::strcmp(name, "autotune-entry-controller") ? NUM_TEMPERATURE_LOOPS : t.pwmIndex;
        const char* methodSuffix = std::strstr(name, "-j");
        const int method = methodSuffix ? methodSuffix[2] - '0' : (useBed ? 3 : 1);
        const char* limitSuffix = std::strstr(name, "-limit");
        if (limitSuffix) {
            unsigned limit = 0;
            assert(std::sscanf(limitSuffix + 6, "%u", &limit) == 1 && limit <= 255);
            t.pidMax = uint8_t(limit);
        }
        const bool lowMax = t.pidMax < 40;
        const bool wrongProfile = !TemperatureController::autotuneProfileAllowed(t.pwmIndex, method);
        const bool busy = Printer::printJob.busy || Printer::pause.owned || scanActive || GCodeInput::scriptActive;
        const bool expectSuccess = std::strstr(name, "-success") && !lowMax && !wrongProfile;
        float high = 0, low = 0;
        if (expectSuccess) {
            temperatureAdcValues[t.sensorPin] = 445;
            assert(t.updateCurrentTemperature());
            high = t.currentTemperatureC;
            temperatureAdcValues[t.sensorPin] = 390;
            assert(t.updateCurrentTemperature());
            low = t.currentTemperatureC;
            assert(high > target && low < target);
            t.invalidatePidCalibration();
        }
        if (wrongProfile || busy) t.setTargetTemperature(0);
        t.autotunePID(target, id, 5, store, method);
        assert(autotuneIndex == 255 && HAL::outputDuty(t.pwmIndex) == 0 && t.targetTemperatureC == 0);
        if (entry || lowMax || wrongProfile || busy) assert(visits == 0);
        else assert(visits > 0 && watchdogHeartbeat == now);
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        assert(relayReports==unsigned(expectSuccess)&&relayUnavailable==0);
#else
        assert(relayReports==0&&relayUnavailable==0);
#endif
        if (expectSuccess) {
            assert(visits == 19 && !Printer::isAnyTempsensorDefect());
            assert(stores == unsigned(store));
            assert(t.pidCalibrationValid() == store);
            if (store) {
                const float p[] = {0.45f, 1.0f/2.2f, 0.65f, 0.45f};
                const float ti[] = {1.6f, 2.2f, 1.6f, 3.0f};
                const float td[] = {0.08f, 1.0f/6.3f, 0.16f, 0.08f};
                // Full PWM relay (low=0), 2 s high and 2 s low.
                const float ku = 4.0f * t.pidMax / (3.14159265f * (high - low));
                const float expectedP = p[method] * ku;
                assert(std::fabs(t.pidPGain - expectedP) < 0.001f);
                assert(std::fabs(t.pidIGain - expectedP / (4 * ti[method])) < 0.001f);
                assert(std::fabs(t.pidDGain - expectedP * 4 * td[method]) < 0.01f);
                assert(t.tempIStateLimitMax == t.pidMax && t.tempIStateLimitMin == -20);
                assert(t.tempIState == 0 && !t.pidHistoryValid);
                std::printf("rule J%d: P=%f I=%f D=%f\n", method, t.pidPGain, t.pidIGain, t.pidDGain);
            }
            assert(observedPeakDuty <= t.pidMax);
        } else assert(stores == 0);
        if (lowMax || wrongProfile) assert(observedPeakDuty == 0 && stores == 0);
        if (!expectSuccess || !store) assert(t.pidPGain == oldP && t.pidIGain == oldI && t.pidDGain == oldD);
        if (std::strstr(name, "-no-rise")) {
            assert(t.isSensorDecoupled()); allOff();
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
            using namespace ThermalEmergency;
            const auto& first=faultLedger().first();
            assert(first.cause==Cause::NoRise&&first.channel==t.pwmIndex);
            assert(first.evidence.targetC==target&&first.evidence.admittedDuty>0);
            assert(first.evidence.valid&Evidence::Profile);
            assert(first.evidence.windowMs>=uint32_t(t.pwmIndex==NUM_EXTRUDER?60000:20000));
#endif
        }
        if (injection == Injection::OtherFault) allOff();
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        if(injection==Injection::Cutoff||!std::strcmp(name,"autotune-entry-absolute")) {
            using namespace ThermalEmergency;
            allOff();
            const auto& e=faultLedger().firstFire().evidence;
            assert(faultLedger().firstFire().cause==Cause::OverTemperature&&e.threshold==t.effectiveCutoffC());
            if(injection==Injection::Cutoff)assert(e.targetC==target&&e.profile==method&&(e.valid&Evidence::Profile)&&e.admittedDuty>0);
            else assert(e.targetC==0&&e.admittedDuty==0);
        }
        if(injection==Injection::High||injection==Injection::TransientHigh)
            assert(ThermalEmergency::faultLedger().level()!=ThermalEmergency::Severity::FireWarning);
#endif
        // One subsequent normal service must not re-enable the old target.
        service();
        assert(HAL::outputDuty(t.pwmIndex) == 0);
    } else {
        assert(false);
    }
    std::printf("%s: passed\n", name);
}
'''


def main():
    header = (FW / "Extruder.h").read_text()
    control = (FW / "TemperatureControl.cpp").read_text()
    conversion = (FW / "TemperatureSensors.cpp").read_text()
    hal = (FW / "HAL.cpp").read_text()
    config = (FW / "Configuration.h").read_text()
    extruder = (FW / "Extruder.cpp").read_text()
    commands = (FW / "Commands.cpp").read_text()
    include = '#include "Repetier.h"'
    printer = (FW / "Printer.h").read_text()
    fragments = {
        "printer-safety.inc": "\n".join(re.findall(r"^#define PRINTER_FLAG0_(?:LEGACY_SIMULATION_REJECTED|TEMPSENSOR_DEFECT)[^\n]+", printer, re.M))
            + "\n" + section(printer, "    static void latchOutputInhibit(", "\n    static INLINE bool isManualMoveMode()"),
        "adc-state.inc": section(hal, "volatile uint16_t temperatureAdcValues", "inline __attribute__((always_inline)) void serviceTemperatureAdc"),
        "controller.inc": section(header, "#define TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL", "}; // TemperatureController") + "};",
        "conversion.inc": conversion.split(include, 1)[1],
        "control.inc": control.split(include, 1)[1].replace('#include "UsbSerial.h"', '').replace('#include "GCodeInput.h"', ''),
        "bed-setter.inc": section(extruder, "void Extruder::setHeatedBedTemperature(", "float Extruder::getHeatedBedTemperature("),
        "scheduler.inc": section(commands, "#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\nstatic void serviceThermalEmergency()", "bool Commands::waitForMotion("),
        "com.inc": "\n".join("constexpr int " + n + " = 0;" for n in sorted(set(re.findall(r"Com::(t[A-Z]\w*)", control)))),
        "hal.inc": (
            "namespace {\n" + section(hal, "#ifndef RF_HEATER_RESET_WDT_EXPERIMENT\nvolatile unsigned long watchdogHeartbeat", "HAL::HAL()")
            + section(hal, "uint16_t HAL::temperatureAdcSample(", "void HAL::analogStart(")
            + section(hal, "void HAL::tellWatchdogOk()", "\n}") + "\n}\n"
            + section(hal, "constexpr uint8_t heaterPwmStep", "constexpr uint8_t partFanPwmStep")
            + "static volatile uint8_t requestedDuty[NUM_EXTRUDER + 3];\n"
            + section(hal, "uint8_t HAL::outputDuty(", "void HAL::setBoardFan(")
            + "void pwmSlice() {\n"
            + section(hal, "    static uint8_t pwm_count_heater", "    pwm_count_cooler += coolerPwmStep;")
            + "    pwm_count_cooler += coolerPwmStep;\n}\n"
        ),
    }
    scenarios = (
        "unconditional-clear", "rail", "invalid-low", "invalid-high", "bed-invalid", "recovery", "disabled", "latched-fault-wait", "legacy-output-inhibit",
        "stale", "bed-stale", "unpublished", "autotune-entry-stale", "autotune-entry-admission-stale", "autotune-bed-entry-admission-stale", "autotune-stale", "autotune-bed-stale",
        "unsupported", "no-rise", "hold-drift", "pwm", "watchdog", "wait",
        "autotune-entry-unsupported", "autotune-entry-low", "autotune-entry-disabled",
        "autotune-entry-latched", "autotune-entry-decoupled", "autotune-entry-legacy-rejected", "autotune-entry-high",
        "autotune-entry-target", "autotune-bed-entry-target", "autotune-entry-controller", "autotune-late",
        "autotune-low", "autotune-transient", "autotune-transient-high", "autotune-high", "autotune-cutoff", "autotune-timeout",
        "autotune-cancel", "autotune-reentrant", "autotune-other-fault", "autotune-success", "autotune-success-store",
        "autotune-bed-late", "autotune-bed-timeout", "autotune-bed-high", "autotune-bed-cancel", "autotune-bed-success-store",
        "pid-components", "untuned", "bed-untuned", "invalid-gains",
        "autotune-power-change", "autotune-bed-power-change",
        "autotune-no-rise", "autotune-bed-no-rise", "autotune-unstable", "autotune-bed-unstable",
        "autotune-entry-busy", "autotune-bed-entry-busy",
        "autotune-entry-pause", "autotune-bed-entry-pause",
        "autotune-entry-scan", "autotune-bed-entry-scan",
        "autotune-entry-script", "autotune-bed-entry-script",
        *(f"autotune-success-store-limit{n}" for n in (0, 10, 39, 40, 41, 255)),
        *(f"autotune-bed-success-store-limit{n}" for n in (0, 10, 39, 40, 41, 255)),
        *(f"autotune-success-store-j{j}" for j in range(6)),
        *(f"autotune-bed-success-store-j{j}" for j in range(6)),
    )
    with TemporaryDirectory(prefix="rfx-temperature-control-") as tmp:
        work = Path(tmp)
        for name, data in fragments.items():
            (work / name).write_text(data)
        (work / "replay.cpp").write_text(HARNESS)
        for board, tools in (("RF1000", 1), ("RF2000", 2), ("RF2000", 1)):
            profile = (FW / (board + ".h")).read_text()
            defines = {n: macro(profile, n) for n in (
                "SENSOR_TEMPERATURE_OFFSETS_C", "MIN_DEFECT_TEMPERATURE", "MAX_DEFECT_TEMPERATURE", "HEATED_BED_MAX_TEMP")}
            defines["BED_MEASURED_CUTOFF"] = re.search(r"constexpr uint16_t bedHeaterCutoffC = (\d+);", (FW / "Extruder.cpp").read_text())[1]
            defines.update({n: macro(config, n) for n in (
                "EXTRUDER_FAN_COOL_TEMP", "DECOUPLING_TEST_MAX_HOLD_VARIANCE",
                "DECOUPLING_TEST_MIN_TEMP_RISE", "EXT0_DECOUPLE_TEST_PERIOD",
                "MAX_ROOM_TEMPERATURE", "WATCHDOG_MAIN_LOOP_TIMEOUT",
                "TEMP_TOLERANCE")})
            defines["ANALOG_REDUCE_BITS"] = macro((FW / "HAL.h").read_text(), "ANALOG_REDUCE_BITS")
            defines["CELSIUS_EXTRA_BITS"] = macro(header, "CELSIUS_EXTRA_BITS")
            defines["NUM_EXTRUDER"] = str(tools)
            for emergency in (False, True):
                exe = work / "replay"
                extra = ["-DRF_THERMAL_EMERGENCY_EXPERIMENT=1"] if emergency else []
                subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-O2", *extra,
                                *[f"-D{k}={v}" for k, v in defines.items()],
                                "-I", str(FW), str(work / "replay.cpp"), "-o", str(exe)], check=True)
                selected = scenarios + (("dual-invalid",) if tools > 1 else ())
                if emergency:
                    selected += ("emergency-invalid-episode", "emergency-stale-episode", "emergency-recovery", "emergency-scheduler",
                                 "absolute-hotend", "absolute-bed", "absolute-idle", "absolute-escalation", "bed-limit",
                                 "autotune-entry-absolute", "autotune-cutoff-periodic")
                for scenario in selected:
                    subprocess.run([str(exe), scenario], check=True, timeout=10)
                print(f"{board} / {tools} hotend(s) / emergency={emergency}: {len(selected)} cases passed", flush=True)


if __name__ == "__main__":
    main()

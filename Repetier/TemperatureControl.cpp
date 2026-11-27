/*
    This file is part of the Repetier-Firmware for RF devices from Conrad Electronic SE.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/

// Temperature regulation, thermal fault latching, heater waits and autotune; ADC conversion is separate.

#include "CaseFan.h"
#include "Repetier.h"
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
#include "ThermalEmergencyReport.h"
#include "ThermalRelayProfile.h"
#include "UsbSerial.h"
#endif

static uint8_t autotuneIndex = 255;
#ifdef RF_EEPROM_ZONE_EXPERIMENT
bool TemperatureController::autotuneActive(){return autotuneIndex!=255;}
#endif
static bool autotuneAbortRequested = false;
static float autotuneTemperatureLimit;
static uint8_t autotuneSensorType, autotunePowerLimit;
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
static float autotuneTargetC;
static uint8_t autotuneMethod;
#endif

static bool validTemperatureSample(bool supported, float temperature) {
    return supported && temperature >= MIN_DEFECT_TEMPERATURE && temperature <= MAX_DEFECT_TEMPERATURE;
}

static void stopAllHeaters() {
    for (uint8_t controller = 0; controller < NUM_TEMPERATURE_LOOPS; controller++)
        tempController[controller]->setTargetTemperature(0);
}

void Printer::latchOutputInhibit(bool thermalFault) {
    {
        InterruptProtectedBlock lock;
        flag0 |= thermalFault ? PRINTER_FLAG0_TEMPSENSOR_DEFECT : PRINTER_FLAG0_LEGACY_SIMULATION_REJECTED;
        extrusionOutput.inhibit(ExtrusionOutput::Inhibit::SafetyLock);
        // Revoke every physical heater output before releasing the safety lock.
        // Target and controller cleanup below may run with interrupts enabled.
        for (uint8_t controller = 0; controller < NUM_TEMPERATURE_LOOPS; ++controller)
            HAL::setHeaterDuty(tempController[controller]->pwmIndex, 0);
    }
    stopAllHeaters();
}

#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
namespace ThermalEmergency {
namespace {
Ledger<NUM_TEMPERATURE_LOOPS> ledger;
ReportCursor<NUM_TEMPERATURE_LOOPS> report;
CancellationCursor cancellation;
uint8_t liveValid=0;
bool liveRequested=false,replayRequestedOnce=false;
uint32_t lastLiveReport=0,lastReplayRequest=0;
uint8_t liveChannel=0;
// Preserve the start of a contiguous invalid-sample episode across the
// existing debounce. No normal valid-sample copies or history allocation.
// Raw ADC and freshness come from the SAME conversion; do not pretend
// that a converted invalid value or software PWM is a valid measurement
// of temperature or actual electrical power.
struct InvalidEpisode {
    uint32_t started;
    float target;
    uint16_t raw;
    uint8_t admitted, sensor, limit;
    Cause cause;
    bool rawKnown, active;
};
InvalidEpisode invalid[NUM_TEMPERATURE_LOOPS]{};
}
const Ledger<NUM_TEMPERATURE_LOOPS>& faultLedger() { return ledger; }

static Evidence capture(const TemperatureController& act, bool measured, millis_t now) {
    Evidence e{};
    e.uptimeMs=now;
    e.targetC=act.targetTemperatureC;
    e.admittedDuty=HAL::outputDuty(act.pwmIndex);
    e.sensorType=act.sensorType;
    e.powerLimit=act.pidMax;
    e.valid=Evidence::Uptime|Evidence::Target|Evidence::Admitted|Evidence::SensorType|Evidence::PowerLimit;
    if(measured) {e.measuredC=act.currentTemperatureC;e.valid|=Evidence::Measured;}
    return e;
}
// Called only after the offending output was revoked. Duty is the
// pre-revoke software value, sampled by the caller immediately before.
static void observeInvalid(uint8_t channel, const TemperatureController& act, uint8_t priorDuty,
                           const SensorObservation& observation) {
    auto& p=invalid[channel];
    if(!p.active) {
        p.started=HAL::timeInMilliseconds();
        p.target=act.targetTemperatureC;
        p.admitted=priorDuty;
        p.sensor=act.sensorType;
        p.limit=act.pidMax;
        p.raw=observation.raw;p.rawKnown=observation.rawKnown;p.cause=observation.invalidCause;
        p.active=true;
    }
}
static Evidence invalidEvidence(uint8_t channel) {
    const auto& p=invalid[channel];
    Evidence e{};
    if(p.active) {
        e.uptimeMs=p.started;e.targetC=p.target;e.admittedDuty=p.admitted;
        e.sensorType=p.sensor;e.powerLimit=p.limit;
        e.valid=Evidence::Uptime|Evidence::Target|Evidence::Admitted|Evidence::SensorType|Evidence::PowerLimit;
        if(p.rawKnown) {e.rawAdc=p.raw;e.valid|=Evidence::Raw;}
    }
    return e;
}
// Snapshot is supplied by the caller. Revoke/latch before copying to
// persistent-in-this-session diagnostic storage; no formatting here.
static bool confirm(uint8_t channel, Cause cause, const Evidence& e) {
    const bool first=!Printer::isAnyTempsensorDefect();
    if(first) Printer::latchOutputInhibit(true);
    ledger.record(channel,cause,e);
    return first;
}
// The existing absolute measured cutoff is independent of learned
// profiles and the RELATIVE autotune overshoot envelope. A fresh,
// supported, plausible reading above the absolute limit is a terminal
// thermal fault even with a zero setpoint. Invalid/rail/stale readings
// retain their separate sensor-fault cause, not a fictional temperature.
static bool checkAbsoluteCutoff(uint8_t channel, TemperatureController& act,
                                bool sampleValid, const SensorObservation& observation) {
    if(!sampleValid||!act.sensorType||!observation.rawKnown||
       !(act.currentTemperatureC>act.effectiveCutoffC()))return false;
    auto e=capture(act,true,HAL::timeInMilliseconds());
    observation.apply(e);
    e.threshold=float(act.effectiveCutoffC());
    e.deltaC=act.currentTemperatureC-e.threshold;
    e.valid|=Evidence::Threshold|Evidence::Delta;
    if(channel==autotuneIndex) {
        e.targetC=autotuneTargetC;
        e.profile=autotuneMethod;
        e.valid|=Evidence::Profile;
    }
    const bool first=confirm(channel,Cause::OverTemperature,e);
    autotuneAbortRequested=true;
    if(first) {
        Printer::markTemperatureReadingsAvailable();
        Printer::stopPrint();
    }
    return true;
}
void latchSystemFault(Cause cause) {
    confirm(Ledger<NUM_TEMPERATURE_LOOPS>::System,cause,Evidence{});
}
bool liveTemperatureValid(uint8_t channel) {
    return channel<NUM_TEMPERATURE_LOOPS && (liveValid&(1u<<channel));
}
void requestLiveReport() {liveRequested=true;}
void requestDiagnosticReplay() {
    const uint32_t now=HAL::timeInMilliseconds();
    if(!replayRequestedOnce || uint32_t(now-lastReplayRequest)>=5000) {
        replayRequestedOnce=true;
        lastReplayRequest=now;
        report.requestReplay();
    }
}
static void serviceReport() {
    if(!ledger.active())return;
    // At most one whole nonblocking span after the thermal pass. A live
    // request cannot starve the retained diagnosis: cancellation first,
    // then one report line, then a periodic/live reading when idle.
    UsbSerial sink;
    if(cancellation.service(true,sink)!=ReportProgress::Idle)return;
    const auto progress=report.service(ledger,sink);
    if(progress!=ReportProgress::Idle)return;
    const uint32_t now=HAL::timeInMilliseconds();
    if(!liveRequested&&uint32_t(now-lastLiveReport)<1000)return;
    // Individual channel lines keep even a 3-heater report <=63 bytes and
    // never claim an invalid measurement as 0 C. These diagnostic lines
    // are not a replacement for measured electrical current.
    ReportLine line;
    Evidence e{};
    // First a conventional Repetier temperature line for valid readings,
    // then explicit per-channel evidence (including unknown readings).
    if(liveChannel==0) {
        const uint8_t selected=Extruder::current->tempControl.pwmIndex;
        if(liveTemperatureValid(selected)) {
            e.valid=Evidence::Measured;
            line.text(PSTR("T:"));
            line.optionalDecimal(e,Evidence::Measured,tempController[selected]->currentTemperatureC);
            line.text(PSTR(" /0 @:0 "));
        }
        if(liveTemperatureValid(NUM_EXTRUDER)) {
            e.valid=Evidence::Measured;
            line.text(PSTR("B:"));
            line.optionalDecimal(e,Evidence::Measured,tempController[NUM_EXTRUDER]->currentTemperatureC);
            line.text(PSTR(" /0 B@:0"));
        }
        if(!line.size())line.text(PSTR("Thermal live: no valid T/B reading"));
    }else {
        const uint8_t channel=liveChannel-1;
        line.text(PSTR("Thermal live "));
        line.channel<NUM_TEMPERATURE_LOOPS>(channel);
        line.text(PSTR(" C: "));
        if(liveTemperatureValid(channel))e.valid=Evidence::Measured;
        line.optionalDecimal(e,Evidence::Measured,tempController[channel]->currentTemperatureC);
        line.text(PSTR(" commanded PWM:0"));
    }
    if(line.finish()&&sink.tryWriteSpan(line.data(),line.size())) {
        if(++liveChannel>NUM_TEMPERATURE_LOOPS) {
            liveChannel=0;liveRequested=false;lastLiveReport=now;
        }else liveRequested=true;
    }
}
}
#endif

bool Extruder::allHeatersCalibrated() {
    for (uint8_t h = 0; h < NUM_TEMPERATURE_LOOPS; ++h)
        if (!tempController[h]->pidCalibrationValid()) return false;
    return true;
}

#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
// Ordinary post-tune diagnostics, only after the heaters have been revoked.
// No per-sample serial output and no safety threshold derived from this data.
static void reportRelayObservation(const ThermalRelayProfile::Summary* p, uint8_t sensor,
                                   uint8_t cap, float target, uint8_t method) {
    for(uint8_t c=0;c<NUM_TEMPERATURE_LOOPS;++c)
        if(HAL::outputDuty(tempController[c]->pwmIndex)||tempController[c]->targetTemperatureC>0)return;
    if(!p) {
        Com::printFLN(PSTR("Thermal relay observation unavailable"));
        return;
    }
    Com::printFLN(PSTR("Thermal relay observation (diagnostic only)"));
    Com::printF(PSTR("Sensor: "),int(sensor));
    Com::printF(PSTR(" PWM cap: "),int(cap));
    Com::printF(PSTR(" J: "),int(method));
    Com::printFLN(PSTR(" target C: "),target);
    // Order: first heating 1s/5s, later heating 1s/5s, cooling 1s/5s.
    Com::printF(PSTR("Observed C/s [first1,first5,later1,later5,cool1,cool5]:"));
    for(uint8_t i=0;i<6;++i) {
        Com::printF(PSTR(" "));
        if(p->mask&(1u<<i))Com::printF(PSTR(""),p->rates[i]);
        else Com::printF(PSTR("?"));
    }
    Com::println();
    Com::printFLN(PSTR("Completed off phases: "),int(p->completed));
    Com::printFLN(PSTR("Observed max off rebound C: "),p->rebound);
    Com::printFLN(PSTR("Observed max time to peak ms: "),p->peakDelayMs);
    Com::printF(PSTR("Observed off duration ms min: "),p->minOffMs);
    Com::printFLN(PSTR(" max: "),p->maxOffMs);
}
#endif

static void finishAutotune(TemperatureController& controller) {
    // Revoke the duty request and normal setpoint before reporting or storing results.
    controller.setTargetTemperature(0);
    autotuneIndex = 255;
#if defined(CASE_FAN_PIN) && (MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2) && CASE_FAN_PIN > -1
    CaseFan::updateFromHotendTargets();
#endif
}

/** Makes updates to temperatures and heater state every call.
Is called every 100ms.
*/
void Extruder::manageTemperatures() {
    static uint8_t extruderTempErrors = 0;
#ifdef RF_HEATER_LEASE_EXPERIMENT
    // Expiry ISR has already removed power. Defer the print/UI/serial work.
    if (HAL::heaterLeaseFault() && !Printer::isAnyTempsensorDefect()) {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        ThermalEmergency::latchSystemFault(ThermalEmergency::Cause::LeaseExpired);
#else
        Printer::latchOutputInhibit(true);
#endif
        Printer::markTemperatureReadingsAvailable();
#ifndef RF_THERMAL_EMERGENCY_EXPERIMENT
        Com::printErrorFLN(PSTR("Heater authorization expired"));
#endif
        Printer::stopPrint();
        autotuneAbortRequested = true;
    }
#endif

    uint8_t errorDetected = 0;

    for (uint8_t controller = 0; controller < NUM_TEMPERATURE_LOOPS; controller++) {
        TemperatureController* act = tempController[controller];
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        ThermalEmergency::SensorObservation observation;
#endif

#ifdef RF_HEATER_LEASE_EXPERIMENT
        HAL::HeaterSample sample;
        const uint8_t checkedType=act->sensorType, checkedLimit=act->pidMax;
        const bool sensorSupported = act->updateCurrentTemperature(&sample
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
            , &observation
#endif
        );
#else
        const bool sensorSupported = act->updateCurrentTemperature(
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
            &observation
#endif
        );
#endif
        const bool sampleValid = validTemperatureSample(sensorSupported, act->currentTemperatureC);
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        if(sampleValid&&act->sensorType) ThermalEmergency::liveValid |= uint8_t(1u<<controller);
        else ThermalEmergency::liveValid &= uint8_t(~(1u<<controller));
#endif
        if (!sampleValid || act->sensorType == 0) {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
            const uint8_t priorDuty=HAL::outputDuty(act->pwmIndex);
#endif
            HAL::setHeaterDuty(act->pwmIndex, 0);
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
            if(!sampleValid) ThermalEmergency::observeInvalid(controller,*act,priorDuty,observation);
            else ThermalEmergency::invalid[controller].active=false;
#endif
            act->pidHistoryValid = false;
            act->tempIState = 0;
        }
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        else {
            ThermalEmergency::invalid[controller].active=false;
            if(ThermalEmergency::checkAbsoluteCutoff(controller,*act,true,observation))continue;
        }
#endif
        if (controller == autotuneIndex) {
            // Remember even a transient invalid sample observed by the periodic service.
            if (!sampleValid || act->sensorType == 0 || act->currentTemperatureC > autotuneTemperatureLimit
                || act->sensorType != autotuneSensorType || act->pidMax != autotunePowerLimit
                || Printer::heatingInhibited()) {
                HAL::setHeaterDuty(act->pwmIndex, 0);
                autotuneAbortRequested = true;
            }
            continue; // Autotune owns this heater's duty request.
        }

        if (controller < NUM_EXTRUDER) {
#if NUM_EXTRUDER >= 2 && EXT0_EXTRUDER_COOLER_PIN == EXT1_EXTRUDER_COOLER_PIN && EXT0_EXTRUDER_COOLER_PIN >= 0
            if (controller == 1 && autotuneIndex != 0 && autotuneIndex != 1)
                if (tempController[0]->currentTemperatureC < EXTRUDER_FAN_COOL_TEMP && tempController[0]->targetTemperatureC < EXTRUDER_FAN_COOL_TEMP && tempController[1]->currentTemperatureC < EXTRUDER_FAN_COOL_TEMP && tempController[1]->targetTemperatureC < EXTRUDER_FAN_COOL_TEMP)
                    extruder[0].coolerPWM = 0;
                else
                    extruder[0].coolerPWM = extruder[0].coolerSpeed;
            if (controller > 1)
#endif // NUM_EXTRUDER >= 2 && EXT0_EXTRUDER_COOLER_PIN == EXT1_EXTRUDER_COOLER_PIN && EXT0_EXTRUDER_COOLER_PIN >= 0

                if (act->currentTemperatureC < EXTRUDER_FAN_COOL_TEMP && act->targetTemperatureC < EXTRUDER_FAN_COOL_TEMP)
                    extruder[controller].coolerPWM = 0;
                else
                    extruder[controller].coolerPWM = extruder[controller].coolerSpeed;
        }

        if (!act->isSensorDefect() && !sampleValid) // Count faults separately from immediate duty inhibition.
        {
            extruderTempErrors++; // Pro sensor. max +10+1+1+1.... bleibt also < 255
            errorDetected = 1;

            if (extruderTempErrors > 10) // Ignore short temporary failures
            {
                act->setSensorDefect(true);
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
                if (ThermalEmergency::confirm(controller,ThermalEmergency::invalid[controller].cause,
                                             ThermalEmergency::invalidEvidence(controller))) {
#else
                if (!Printer::isAnyTempsensorDefect()) {
                    Printer::latchOutputInhibit(true);
#endif
                    Printer::markTemperatureReadingsAvailable(); // We are not waiting for first temp measurements anymore.

#ifndef RF_THERMAL_EMERGENCY_EXPERIMENT
                    reportTempsensorAndHeaterErrors();
#endif
                    Printer::stopPrint();

#ifndef RF_THERMAL_EMERGENCY_EXPERIMENT
                    showError((void*)ui_text_temperature_manager, (void*)ui_text_sensor_error);
#endif
                }
            }
        }
        if (!sampleValid || act->sensorType == 0 || Printer::heatingInhibited())
            continue; // Never feed an invalid or disabled sensor sample into PID.

#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        // No raw/internal target assignment may bypass the heater-specific
        // target ceiling, even if a future caller forgets the public setter.
        if(!(act->targetTemperatureC>=0&&act->targetTemperatureC<=act->maximumTargetC())) {
            act->setTargetTemperature(0);
            continue;
        }
#endif
        if (act->targetTemperatureC >= MAX_ROOM_TEMPERATURE && !act->pidCalibrationValid()) {
            act->invalidatePidCalibration();
            continue;
        }
        uint8_t on = act->currentTemperatureC >= act->targetTemperatureC ? LOW : HIGH;

        millis_t time = HAL::timeInMilliseconds(); // compare time for decouple tests
        // Run test if heater and sensor are decoupled
        bool decoupleTestRequired = !errorDetected && act->decoupleTestPeriod > 0 && (time - act->lastDecoupleTest) > act->decoupleTestPeriod; // time enough for temperature change?
        if (decoupleTestRequired) {                                                                                                            // Only test when powered
            if (act->isDecoupleFull()) {                                                                                                       // Phase 1: Heating fully until target range is reached
                if (act->currentTemperatureC - act->lastDecoupleTemp < DECOUPLING_TEST_MIN_TEMP_RISE) {                                        // failed test
                    extruderTempErrors++;
                    errorDetected = 1;

                    if (extruderTempErrors > 10) { // Ignore short temporary failures
                        act->setSensorDecoupled(true);
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
                        auto evidence=ThermalEmergency::capture(*act,true,time);
                        observation.apply(evidence);
                        evidence.windowMs=time-act->lastDecoupleTest;
                        evidence.deltaC=act->currentTemperatureC-act->lastDecoupleTemp;
                        evidence.threshold=DECOUPLING_TEST_MIN_TEMP_RISE;
                        evidence.valid|=ThermalEmergency::Evidence::Window|ThermalEmergency::Evidence::Delta|ThermalEmergency::Evidence::Threshold;
                        if (ThermalEmergency::confirm(controller,ThermalEmergency::Cause::NoRise,evidence)) {
#else
                        if (!Printer::isAnyTempsensorDefect()) {
                            Printer::latchOutputInhibit(true);
#endif

#ifndef RF_THERMAL_EMERGENCY_EXPERIMENT
                            Com::printErrorFLN(Com::tHeaterDecoupledWarning);
                            Com::printF(PSTR("Error:Temp. raised to slow. Rise = "), act->currentTemperatureC - act->lastDecoupleTemp);
                            Com::printF(PSTR(" after "), (int32_t)(time - act->lastDecoupleTest));
                            Com::printFLN(PSTR(" ms"));

                            reportTempsensorAndHeaterErrors();
#endif
                            Printer::stopPrint();

#ifndef RF_THERMAL_EMERGENCY_EXPERIMENT
                            showError((void*)ui_text_temperature_manager, (void*)ui_text_heater_error);
#endif
                        }
                    }
                } else {
                    // Wir ziehen hier die zeit und die temperatur hoch: starten einen neuen test auf einer neuen stufe.
                    act->stopDecouple();              // Without stop no clean start.
                    act->startFullDecoupleTest(time); // Set new time and current temp
                }
            } else if (act->isDecoupleHold()) {                                                                     // Phase 2: Holding temperature inside a target corridor
                if (fabs(act->currentTemperatureC - act->targetTemperatureC) > DECOUPLING_TEST_MAX_HOLD_VARIANCE) { // failed test
                    extruderTempErrors++;
                    errorDetected = 1;

                    if (extruderTempErrors > 10) { // Ignore short temporary failures
                        act->setSensorDecoupled(true);
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
                        auto evidence=ThermalEmergency::capture(*act,true,time);
                        observation.apply(evidence);
                        evidence.windowMs=time-act->lastDecoupleTest;
                        evidence.deltaC=act->currentTemperatureC-act->targetTemperatureC;
                        evidence.threshold=DECOUPLING_TEST_MAX_HOLD_VARIANCE;
                        evidence.valid|=ThermalEmergency::Evidence::Window|ThermalEmergency::Evidence::Delta|ThermalEmergency::Evidence::Threshold;
                        if (ThermalEmergency::confirm(controller,ThermalEmergency::Cause::HoldFailure,evidence)) {
#else
                        if (!Printer::isAnyTempsensorDefect()) {
                            Printer::latchOutputInhibit(true);
#endif

#ifndef RF_THERMAL_EMERGENCY_EXPERIMENT
                            Com::printErrorFLN(Com::tHeaterDecoupledWarning);
                            Com::printF(PSTR("Error:Could not hold temperature "), act->lastDecoupleTemp);
                            Com::printF(PSTR(" measured "), act->currentTemperatureC);
                            Com::printFLN(Com::tC);

                            reportTempsensorAndHeaterErrors();
#endif
                            Printer::stopPrint();

#ifndef RF_THERMAL_EMERGENCY_EXPERIMENT
                            showError((void*)ui_text_temperature_manager, (void*)ui_text_heater_error);
#endif
                        }
                    }
                } else {
                    // This test ignores decoupleTestPeriod in configuration: "- act->decoupleTestPeriod" and runs every second. But not at first hold-start-time.
                    act->lastDecoupleTest = time - act->decoupleTestPeriod + 1000; // once running test every second
                }
            }
        }
        if (Printer::heatingInhibited())
            continue;

        // Elapsed time and derivative on measurement with a 0.5 s low-pass filter.
        // Do not integrate unobserved error after a missed service or invalid sample.
        const millis_t elapsed = time - act->lastTemperatureUpdate;
        const float dt = act->pidHistoryValid && elapsed > 0 && elapsed <= 1000 ? elapsed * 0.001f : 0;
        if (dt > 0) {
            const float slope = (act->currentTemperatureC - act->previousTemperature) / dt;
            act->filteredSlope += dt / (0.5f + dt) * (slope - act->filteredSlope);
        } else {
            act->filteredSlope = 0;
        }
        act->previousTemperature = act->currentTemperatureC;
        act->lastTemperatureUpdate = time;
        act->pidHistoryValid = true;

        uint8_t output = 0;
        const float error = act->targetTemperatureC - act->currentTemperatureC;
        if (act->targetTemperatureC < MAX_ROOM_TEMPERATURE) {
            act->stopDecouple();
            act->tempIState = 0;
        } else {
            const float pd = act->pidPGain * error - act->pidDGain * act->filteredSlope;
            const float candidate = constrain(act->tempIState + act->pidIGain * error * dt,
                                              act->tempIStateLimitMin, act->tempIStateLimitMax);
            // Integrate only when unsaturated or when unwinding saturation.
            if ((pd + candidate <= act->pidMax || error < 0)
                && (pd + candidate >= 0 || error > 0))
                act->tempIState = candidate;
            // Clamp in float before converting to the AVR's narrow integer type.
            const float duty = constrain(pd + act->tempIState, 0.0f, float(act->pidMax));
            output = static_cast<uint8_t>(duty);
            if (!act->isDecoupleHold() && error > DECOUPLING_TEST_MAX_HOLD_VARIANCE && output > 0)
                act->startFullDecoupleTest(time);
            else
                act->startHoldDecoupleTest(time);
        }
        if (act->currentTemperatureC > act->effectiveCutoffC())
            output = 0;
#ifdef RF_HEATER_LEASE_EXPERIMENT
        if(act->sensorType!=checkedType || act->pidMax!=checkedLimit) output=0;
        HAL::setHeaterDuty(act->pwmIndex, output, &sample);
#else
        HAL::setHeaterDuty(act->pwmIndex, output);
#endif

#if LED_PIN > -1
        if (act == &Extruder::current->tempControl)
            WRITE(LED_PIN, on);
#endif // LED_PIN>-1
    }  // for controller

    if (errorDetected == 0 && extruderTempErrors > 0)
        extruderTempErrors--;

    if (!Printer::isAnyTempsensorDefect() && !errorDetected)
        Printer::markTemperatureReadingsAvailable();
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(Printer::isAnyTempsensorDefect()) {
#if defined(THERMAL_ALARM_PIN) && THERMAL_ALARM_PIN >= 0
        // Provisional slow active-HIGH alarm gate, not a qualified tone.
        // No continuous DC request, blocking delay or extra timer. Passive
        // transducers may need a different carrier; verify the fitted driver.
        WRITE(THERMAL_ALARM_PIN, (HAL::timeInMilliseconds()%1000)<250 ? HIGH : LOW);
#endif
        // Do not use an invalid sensor's apparent cold value to stop
        // heatsink cooling. Keep named cooler outputs on in this state.
        for(uint8_t h=0;h<NUM_EXTRUDER;++h)extruder[h].coolerPWM=255;
#if defined(CASE_FAN_PIN) && (MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2) && CASE_FAN_PIN > -1
        CaseFan::turnOn();
#endif
        // Revoke again before the completed watchdog pass, independently
        // of any controller/auto-tune branch above.
        for(uint8_t h=0;h<NUM_TEMPERATURE_LOOPS;++h)
            HAL::setHeaterDuty(tempController[h]->pwmIndex,0);
    }
#endif
#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
    // A momentarily zero PID duty does not mean the heating session ended.
    // Retain reset evidence across the whole target/autotune session.
    bool heatDemand=autotuneIndex!=255;
    for(uint8_t c=0;c<NUM_TEMPERATURE_LOOPS;++c)
        heatDemand=heatDemand || tempController[c]->targetTemperatureC>=MAX_ROOM_TEMPERATURE;
    HAL::completeThermalWatchdogPass(heatDemand);
#endif
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    ThermalEmergency::serviceReport();
#endif

} // manageTemperatures

void TemperatureController::updateTempControlVars() {
    tempIStateLimitMax = RMath::min(int(pidDriveMax), int(pidMax));
    tempIStateLimitMin = -float(RMath::min(int(pidDriveMin), int(pidMax)));
    tempIState = constrain(tempIState, tempIStateLimitMin, tempIStateLimitMax);
}

void TemperatureController::invalidatePidCalibration() {
    tunedSensorType = tunedPowerLimit = 0;
    setTargetTemperature(0);
}

#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
BedTemperatureLimit TemperatureController::bedLimit_{BedTemperatureLimit::DefaultC};
bool TemperatureController::setBedEmergencyCutoff(uint8_t verifiedCutoff) {
    if(pwmIndex!=NUM_EXTRUDER) return false;
    // Revoke any current target, lease and pending autotune before publishing
    // a different cutoff or removing authorization. No stale rearming.
    if(!verifiedCutoff || !BedTemperatureLimit::validSetting(verifiedCutoff) ||
       verifiedCutoff!=bedLimit_.commissioned()) setTargetTemperature(0);
    return bedLimit_.publish(verifiedCutoff);
}
#endif

void TemperatureController::setTargetTemperature(float target) {
    if (!(target >= 0 && target <= maximumTargetC()) || Printer::heatingInhibited())
        target = 0;
    if (target >= MAX_ROOM_TEMPERATURE && autotuneIndex != 255) {
        Com::printErrorFLN(PSTR("PID autotune active"));
        target = 0;
    }
    if (target >= MAX_ROOM_TEMPERATURE && !pidCalibrationValid()) {
        Com::printErrorFLN(PSTR("PID tune required; use menu"));
        UI_STATUS_UPD("PID tune needed");
        target = 0;
    }
    if (target <= 0) {
        tempIState = 0;
        pidHistoryValid = false;
    }
    if (target <= 0.0f && autotuneIndex < NUM_TEMPERATURE_LOOPS && tempController[autotuneIndex] == this)
        autotuneAbortRequested = true;
    if (target <= 0.0f || target < targetTemperatureC)
        HAL::setHeaterDuty(pwmIndex, 0);
    targetTemperatureC = target;
    stopDecouple();

    // Do not set back temperatures to "not idle temperatures" after pause, if they have been changed manually within the pause.
    paused = 0;
} // setTargetTemperature

void TemperatureController::waitForTargetTemperature(uint8_t plus_temp_tolerance, bool resumingPause) {
    if (Printer::heatingInhibited() || (targetTemperatureC >= MAX_ROOM_TEMPERATURE && !pidCalibrationValid()))
        return;
    bool dirRising = targetTemperatureC > currentTemperatureC;
    if (dirRising) {
        UI_STATUS_UPD(UI_TEXT_HEATING_UP);
    } else {
        UI_STATUS_UPD(UI_TEXT_COOLING_DOWN);
    }
    Printer::idleStatus.cancel();
    while (true) {
        Commands::checkForPeriodicalActions(WaitHeater);
        if (Printer::heatingInhibited() || (dirRising && targetTemperatureC <= 0) || (resumingPause && !Printer::pause.isReheating()))
            return;
        if (fabs(targetTemperatureC - currentTemperatureC) <= TEMP_TOLERANCE + plus_temp_tolerance)
            break;
        if (!dirRising && currentTemperatureC < MAX_ROOM_TEMPERATURE)
            break;
    }
    Printer::idleStatus.schedule(HAL::timeInMilliseconds());
}

// Fixed duty levels for every measured cycle. The asymmetric rectangular
// relay fundamental accounts for unequal on/off times; span is peak-to-peak.
// J0/J1 hotend dynamic/robust, J2/J3 bed dynamic/robust. No legacy scaling.
// Keep the large autotune frame out of ordinary menu/G-code dispatch.
__attribute__((noinline)) void TemperatureController::autotunePID(float temp, uint8_t controllerId, int maxCycles, bool storeValues, int method) {
    if (autotuneIndex != 255) {
        Com::printErrorFLN(PSTR("PID autotune already active"));
        return;
    }
    if (controllerId >= NUM_TEMPERATURE_LOOPS || tempController[controllerId] != this) {
        setTargetTemperature(0);
        Com::printErrorFLN(PSTR("PID autotune: invalid heater"));
        return;
    }
    if (Printer::printJob.isBusy() || PrintLine::hasLines() || PrintLine::directMoveActive() || isAnyScanRunning()) {
        Com::printErrorFLN(PSTR("PID autotune requires idle printer"));
        return;
    }
    if (!autotuneProfileAllowed(controllerId, method)) {
        Com::printErrorFLN(PSTR("PID autotune: invalid profile for heater"));
        return;
    }
    setTargetTemperature(0);
    const bool bed = controllerId == NUM_EXTRUDER;
    const float hysteresis = 0.375f;
    const float cutoff = RMath::min(float(effectiveCutoffC()), temp + (bed ? 15.0f : 20.0f));
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    ThermalEmergency::SensorObservation initialObservation;
    const bool initialSampleValid = updateCurrentTemperature(
#ifdef RF_HEATER_LEASE_EXPERIMENT
        nullptr,
#endif
        &initialObservation);
    if(ThermalEmergency::checkAbsoluteCutoff(controllerId,*this,
        validTemperatureSample(initialSampleValid,currentTemperatureC),initialObservation))return;
#else
    const bool initialSampleValid = updateCurrentTemperature();
#endif
    if (Printer::heatingInhibited() || isSensorDefect() || isSensorDecoupled()
        || sensorType == 0 || pidMax < 40
        || !(temp > MAX_ROOM_TEMPERATURE + hysteresis && temp + hysteresis < cutoff)
        || (bed && (temp > HEATED_BED_MAX_TEMP || temp > maximumTargetC()))
        || !validTemperatureSample(initialSampleValid, currentTemperatureC)
        || currentTemperatureC > cutoff) {
        Com::printErrorFLN(PSTR("PID autotune: heater, target, sensor or PWM limit invalid"));
        return;
    }
    // No other heater may perturb the experiment.
    stopAllHeaters();
    Printer::idleStatus.cancel();
    UI_STATUS_UPD(UI_TEXT_PID);
    Com::printInfoFLN(Com::tPIDAutotuneStart);
    Com::printFLN(PSTR("Profile J: "), method);
    autotuneAbortRequested = false;
    autotuneTemperatureLimit = cutoff;
    autotuneSensorType = sensorType;
    autotunePowerLimit = pidMax;
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    autotuneTargetC=temp;
    autotuneMethod=uint8_t(method);
#endif
    autotuneIndex = controllerId;
    stopDecouple();
    // Admission diagnostics can take time. Do not use the earlier sample
    // to authorize the first positive power request.
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    ThermalEmergency::SensorObservation firstObservation;
#endif
#ifdef RF_HEATER_LEASE_EXPERIMENT
    HAL::HeaterSample firstSample;
    const bool firstSampleValid = updateCurrentTemperature(&firstSample
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        , &firstObservation
#endif
    );
#else
    const bool firstSampleValid = updateCurrentTemperature(
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        &firstObservation
#endif
    );
#endif
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(ThermalEmergency::checkAbsoluteCutoff(controllerId,*this,
        validTemperatureSample(firstSampleValid,currentTemperatureC),firstObservation)) {
        finishAutotune(*this);
        return;
    }
#endif
    if (autotuneAbortRequested || Printer::heatingInhibited()
        || sensorType != autotuneSensorType || pidMax != autotunePowerLimit
        || isSensorDefect() || isSensorDecoupled()
        || !validTemperatureSample(firstSampleValid, currentTemperatureC)
        || currentTemperatureC > cutoff) {
        finishAutotune(*this);
        Com::printErrorFLN(PSTR("PID autotune: initial authorization lost"));
        return;
    }
    const millis_t started = HAL::timeInMilliseconds();
    millis_t phaseStart = started, cycleStart = started, riseStart = started, lastSample = started;
    float riseTemp = currentTemperatureC, low = currentTemperatureC, high = currentTemperatureC;
    float kuSum = 0, tuSum = 0, prevKu = 0, prevTu = 0;
    millis_t onTime = 0;
    unsigned cycles = 0, stable = 0;
    const unsigned limit = constrain(maxCycles, 5, 20);
    bool heating = currentTemperatureC < temp - hysteresis;
    bool completeCycle = heating;
#ifdef RF_HEATER_LEASE_EXPERIMENT
    HAL::setHeaterDuty(pwmIndex, heating ? autotunePowerLimit : 0, &firstSample);
    firstSample.release();
    if(heating && !HAL::outputDuty(pwmIndex)) {
        finishAutotune(*this);
        Com::printErrorFLN(PSTR("PID autotune: initial sample not authorized"));
        return;
    }
#else
    HAL::setHeaterDuty(pwmIndex, heating ? autotunePowerLimit : 0);
#endif
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    ThermalRelayProfile::Recorder relayObservation;
    ThermalRelayProfile::PublicationTracker relayPublication(firstObservation.publishedAt);
    relayObservation.begin(started,currentTemperatureC,autotunePowerLimit,HAL::outputDuty(pwmIndex));
    if(!firstObservation.rawKnown)relayObservation.invalidate();
#endif
    if (!bed) {
#if defined(CASE_FAN_PIN) && (MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2) && CASE_FAN_PIN > -1
        CaseFan::turnOn();
#endif
        extruder[controllerId].coolerPWM = extruder[controllerId].coolerSpeed;
        extruder[0].coolerPWM = extruder[0].coolerSpeed;
    }

    for (;;) {
        Commands::checkForPeriodicalActions(WaitHeater);
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        if(Printer::printJob.emergencyLocked()) {
            finishAutotune(*this);
            return;
        }
        ThermalEmergency::SensorObservation observation;
#endif
#ifdef RF_HEATER_LEASE_EXPERIMENT
        HAL::HeaterSample sample;
        const bool supported = updateCurrentTemperature(&sample
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
            , &observation
#endif
        );
#else
        const bool supported = updateCurrentTemperature(
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
            &observation
#endif
        );
#endif
        const float measured = currentTemperatureC;
        const millis_t now = HAL::timeInMilliseconds();
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        if(ThermalEmergency::checkAbsoluteCutoff(controllerId,*this,
            validTemperatureSample(supported,measured),observation)) {
            finishAutotune(*this);
            return;
        }
#endif
        if (autotuneAbortRequested || Printer::heatingInhibited()
            || Printer::printJob.isBusy() || PrintLine::hasLines() || PrintLine::directMoveActive() || isAnyScanRunning()
            || sensorType != autotuneSensorType || pidMax != autotunePowerLimit
            || isSensorDefect() || isSensorDecoupled() || !validTemperatureSample(supported, measured)) {
            finishAutotune(*this);
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
            if(ThermalEmergency::faultLedger().active())return;
#endif
            Com::printErrorFLN(PSTR("PID autotune aborted: sensor, settings, fault or cancellation"));
            showError((void*)ui_text_autodetect_pid, (void*)ui_text_temperature_wrong);
            break;
        }
        if (measured > cutoff) {
            finishAutotune(*this);
            Com::printErrorFLN(Com::tAPIDFailedHigh);
            showError((void*)ui_text_autodetect_pid, (void*)ui_text_temperature_wrong);
            break;
        }
        if (now - started > (bed ? 3600000UL : 1200000UL)
            || now - phaseStart > (bed ? 600000UL : 180000UL)) {
            finishAutotune(*this);
            Com::printErrorFLN(Com::tAPIDFailedTimeout);
            showError((void*)ui_text_autodetect_pid, (void*)ui_text_timeout);
            break;
        }
        // An excessively late sample invalidates timing/identification, even
        // if the ADC itself is still fresh.
        if (now - lastSample > 1000) {
            finishAutotune(*this);
            Com::printErrorFLN(PSTR("PID autotune: temperature service delayed"));
            break;
        }
        if (now - lastSample < 100)
            continue;
        lastSample = now;
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        if(!observation.rawKnown||!relayPublication.next(observation.publishedAt))
            relayObservation.invalidate();
        relayObservation.sample(now,measured,HAL::outputDuty(pwmIndex));
#endif
        low = RMath::min(low, measured);
        high = RMath::max(high, measured);
        if (heating && measured >= temp + hysteresis) {
            HAL::setHeaterDuty(pwmIndex, 0);
            heating = false;
            onTime = now - cycleStart;
            phaseStart = now;
        } else if (heating && now - riseStart >= (bed ? 60000UL : 20000UL)) {
            if (measured - riseTemp < DECOUPLING_TEST_MIN_TEMP_RISE) {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
                auto evidence=ThermalEmergency::capture(*this,true,now);
                observation.apply(evidence);
                evidence.targetC=temp; // relay tune uses its own target, not normal setpoint
                evidence.deltaC=measured-riseTemp;
                evidence.windowMs=now-riseStart;
                evidence.threshold=DECOUPLING_TEST_MIN_TEMP_RISE;
                evidence.profile=uint8_t(method);
                evidence.valid|=ThermalEmergency::Evidence::Delta|ThermalEmergency::Evidence::Window
                    |ThermalEmergency::Evidence::Threshold|ThermalEmergency::Evidence::Profile;
#endif
                finishAutotune(*this);
                setSensorDecoupled(true);
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
                ThermalEmergency::confirm(controllerId,ThermalEmergency::Cause::NoRise,evidence);
#else
                Printer::latchOutputInhibit(true);
#endif
#ifndef RF_THERMAL_EMERGENCY_EXPERIMENT
                Com::printErrorFLN(PSTR("PID autotune: no temperature rise"));
#endif
                Printer::stopPrint();
#ifndef RF_THERMAL_EMERGENCY_EXPERIMENT
                showError((void*)ui_text_temperature_manager, (void*)ui_text_heater_error);
#endif
                break;
            }
            riseStart = now;
            riseTemp = measured;
        } else if (!heating && measured <= temp - hysteresis) {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
            relayObservation.completeOff(now);
#endif
            if (completeCycle) {
                ++cycles;
                const millis_t period = now - cycleStart;
                const float fraction = float(onTime) / float(period);
                const float span = high - low;
                // Discard heat-up and the next transient cycle.
                if (cycles > 2 && period >= 1000 && span >= 0.75f
                    && fraction >= 0.05f && fraction <= 0.95f) {
                    const float ku = (4.0f * autotunePowerLimit * sinf(3.14159265f * fraction)) / (3.14159265f * span);
                    const float tu = period * 0.001f;
                    if (ku > 0 && ku <= 1000 && tu > 0) {
                        if (stable && (fabsf(ku - prevKu) > 0.15f * prevKu || fabsf(tu - prevTu) > 0.15f * prevTu)) {
                            stable = 0;
                            kuSum = tuSum = 0;
                        }
                        if (stable == 0) {
                            prevKu = ku;
                            prevTu = tu;
                        }
                        ++stable;
                        kuSum += ku;
                        tuSum += tu;
                    } else {
                        stable = 0;
                        kuSum = tuSum = 0;
                    }
                } else {
                    stable = 0;
                    kuSum = tuSum = 0;
                }
                if (stable >= 3 && cycles >= limit) {
                    const float ku = kuSum / stable, tu = tuSum / stable;
                    // Immutable tuning rules; read once after the measured cycles.
                    static const float rules[4][3] PROGMEM = {
                        {0.45f, 1.6f, 0.08f},
                        {1.0f / 2.2f, 2.2f, 1.0f / 6.3f},
                        {0.65f, 1.6f, 0.16f},
                        {0.45f, 3.0f, 0.08f}
                    };
                    // Admission above restricts method to 0..3.
                    const float p = pgm_read_float(&rules[method][0]);
                    const float ti = pgm_read_float(&rules[method][1]);
                    const float td = pgm_read_float(&rules[method][2]);
                    const float kp = p * ku, ki = kp / (ti * tu), kd = kp * td * tu;
                    finishAutotune(*this);
                    if (!validPidGains(kp, ki, kd)) {
                        Com::printErrorFLN(PSTR("PID autotune: invalid gains"));
                        break;
                    }
                    Com::printF(Com::tAPIDKu, ku);
                    Com::printFLN(Com::tAPIDTu, tu);
                    Com::printFLN(Com::tAPIDKp, kp);
                    Com::printFLN(Com::tAPIDKi, ki);
                    Com::printFLN(Com::tAPIDKd, kd);
                    if (storeValues) {
                        pidPGain = kp;
                        pidIGain = ki;
                        pidDGain = kd;
                        pidDriveMax = pidMax;
                        pidDriveMin = RMath::min(20, int(pidMax));
                        tunedSensorType = sensorType;
                        tunedPowerLimit = pidMax;
                        tempIState = 0;
                        pidHistoryValid = false;
                        updateTempControlVars();
                        EEPROM::storePidCalibration(controllerId);
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
                        if(Printer::printJob.emergencyLocked())return;
#endif
                    }
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
                    reportRelayObservation(relayObservation.completedProfile(),
                                           autotuneSensorType,autotunePowerLimit,temp,uint8_t(method));
#endif
                    Com::printInfoFLN(Com::tAPIDFinished);
                    UI_STATUS_UPD(UI_TEXT_AUTODETECT_PID_DONE);
                    Printer::idleStatus.schedule(HAL::timeInMilliseconds() + 30000);
                    return;
                }
                if (cycles >= limit) {
                    finishAutotune(*this);
                    Com::printErrorFLN(PSTR("PID autotune: oscillation did not converge"));
                    break;
                }
            }
            completeCycle = true;
            heating = true;
            phaseStart = cycleStart = riseStart = now;
            riseTemp = low = high = measured;
        }
        // Renew only after the full sampled autotune decision, including
        // cutoff, cancellation, timing and rise checks. No ISR renewal.
        if (heating) {
#ifdef RF_HEATER_LEASE_EXPERIMENT
            HAL::setHeaterDuty(pwmIndex, autotunePowerLimit, &sample);
            if(!HAL::outputDuty(pwmIndex)) {
                finishAutotune(*this);
                Com::printErrorFLN(PSTR("PID autotune: sample not authorized"));
                break;
            }
#else
            HAL::setHeaterDuty(pwmIndex, autotunePowerLimit);
#endif
        }
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        relayObservation.afterDecision(now,measured,HAL::outputDuty(pwmIndex));
#endif
#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
        // Complete the second half of the watchdog handshake only after
        // the entire sampled relay decision and its safety checks finished.
        HAL::completeAutotuneWatchdogPass();
#endif
    }
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(Printer::printJob.emergencyLocked())return;
#endif
    Printer::idleStatus.schedule(HAL::timeInMilliseconds());
} // autotunePID

void reportTempsensorAndHeaterErrors() {
    // Write full status list to console output
    for (uint8_t i = 0; i < NUM_TEMPERATURE_LOOPS; i++) {
        if (i == NUM_EXTRUDER)
            Com::printF(Com::tHeatedBed);
        else
            Com::printF(Com::tExtruderSpace, i);

        if (tempController[i]->isSensorDefect()) {
            Com::printF(Com::tTempSensorDefect);
        }
        if (tempController[i]->isSensorDecoupled()) {
            Com::printF(Com::tTempHeaterDefect);
        }
        if (!tempController[i]->isSensorDecoupled() && !tempController[i]->isSensorDefect()) {
            Com::printF(Com::tTempSensorWorking);
        }
        Com::println();
    }
    Com::printErrorFLN(Com::tThermalFaultUntilRestart);
} // reportTempsensorAndHeaterErrors

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

#include "Repetier.h"

static uint8_t autotuneIndex = 255;
static bool autotuneAbortRequested = false;
static float autotuneTemperatureLimit;

static bool validTemperatureSample(bool supported, float temperature) {
    return supported && temperature >= MIN_DEFECT_TEMPERATURE && temperature <= MAX_DEFECT_TEMPERATURE;
}

static void stopAllHeaters() {
    for (uint8_t controller = 0; controller < NUM_TEMPERATURE_LOOPS; controller++)
        tempController[controller]->setTargetTemperature(0);
}

static void finishAutotune(TemperatureController& controller) {
    // Revoke the duty request and normal setpoint before reporting or storing results.
    controller.setTargetTemperature(0);
    autotuneIndex = 255;
}

/** Makes updates to temperatures and heater state every call.
Is called every 100ms.
*/
void Extruder::manageTemperatures() {
    static uint8_t extruderTempErrors = 0;

    uint8_t errorDetected = 0;

    for (uint8_t controller = 0; controller < NUM_TEMPERATURE_LOOPS; controller++) {
        TemperatureController* act = tempController[controller];

        const bool sensorSupported = act->updateCurrentTemperature();
        const bool sampleValid = validTemperatureSample(sensorSupported, act->currentTemperatureC);
        if (!sampleValid || act->sensorType == 0)
            HAL::setHeaterDuty(act->pwmIndex, 0);
        if (controller == autotuneIndex) {
            // Remember even a transient invalid sample observed by the periodic service.
            if (!sampleValid || act->sensorType == 0 || act->currentTemperatureC > autotuneTemperatureLimit
                || Printer::isAnyTempsensorDefect() || Printer::debugDryrun()) {
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
            extruderTempErrors++; //pro sensor. max +10+1+1+1.... bleibt also < 255
            errorDetected = 1;

            if (extruderTempErrors > 10) // Ignore short temporary failures
            {
                act->setSensorDefect(true);
                if (!Printer::isAnyTempsensorDefect()) {
                    Printer::setSomeTempsensorDefect(true);
                    stopAllHeaters();
                    Printer::markTemperatureReadingsAvailable(); //we are not waiting for first temp measurements anymore.

                    reportTempsensorAndHeaterErrors();
                    Printer::stopPrint();

                    showError((void*)ui_text_temperature_manager, (void*)ui_text_sensor_error);
                }
            }
        }
        if (!sampleValid || act->sensorType == 0 || Printer::isAnyTempsensorDefect())
            continue; // Never feed an invalid or disabled sensor sample into PID.

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
                        if (!Printer::isAnyTempsensorDefect()) {
                            Printer::setSomeTempsensorDefect(true);
                            stopAllHeaters();

                            Com::printErrorFLN(Com::tHeaterDecoupledWarning);
                            Com::printF(PSTR("Error:Temp. raised to slow. Rise = "), act->currentTemperatureC - act->lastDecoupleTemp);
                            Com::printF(PSTR(" after "), (int32_t)(time - act->lastDecoupleTest));
                            Com::printFLN(PSTR(" ms"));

                            reportTempsensorAndHeaterErrors();
                            Printer::stopPrint();

                            showError((void*)ui_text_temperature_manager, (void*)ui_text_heater_error);
                        }
                    }
                } else {
                    //wir ziehen hier die zeit und die temperatur hoch: starten einen neuen test auf einer neuen stufe.
                    act->stopDecouple();              //without stop no clean start.
                    act->startFullDecoupleTest(time); //set new time and current temp
                }
            } else if (act->isDecoupleHold()) {                                                                     // Phase 2: Holding temperature inside a target corridor
                if (fabs(act->currentTemperatureC - act->targetTemperatureC) > DECOUPLING_TEST_MAX_HOLD_VARIANCE) { // failed test
                    extruderTempErrors++;
                    errorDetected = 1;

                    if (extruderTempErrors > 10) { // Ignore short temporary failures
                        act->setSensorDecoupled(true);
                        if (!Printer::isAnyTempsensorDefect()) {
                            Printer::setSomeTempsensorDefect(true);
                            stopAllHeaters();

                            Com::printErrorFLN(Com::tHeaterDecoupledWarning);
                            Com::printF(PSTR("Error:Could not hold temperature "), act->lastDecoupleTemp);
                            Com::printF(PSTR(" measured "), act->currentTemperatureC);
                            Com::printFLN(Com::tC);

                            reportTempsensorAndHeaterErrors();
                            Printer::stopPrint();

                            showError((void*)ui_text_temperature_manager, (void*)ui_text_heater_error);
                        }
                    }
                } else {
                    //this test ignores decoupleTestPeriod in configuration: "- act->decoupleTestPeriod" and runs every second. But not at first hold-start-time.
                    act->lastDecoupleTest = time - act->decoupleTestPeriod + 1000; // once running test every second
                }
            }
        }
        if (Printer::isAnyTempsensorDefect())
            continue;

        act->tempArray[act->tempPointer++] = act->currentTemperatureC; //ist in jedem fall voll mit gültigen temperaturen, wenn der regelbereich erreicht wird.
        //act->tempPointer &= 3; // 3 = springe von 4 = 100b auf 0 zurück,    wenn 3. -> 1/300ms  -> 3.33 = reciproke     !!tempArray needs [4] ...
        //act->tempPointer &= 7; // 7 = springe von 8 = 1000b auf 0 zurück,   wenn 7. -> 1/700ms  -> 1.42 = reciproke     !!tempArray needs [8] ...
        act->tempPointer &= 15; // 15 = springe von 16 = 10000b zurück auf 0, wenn 15 -> 1/1500ms -> 0.666 = reciproke    !!tempArray needs [16] ...

        uint8_t output;
        float error = act->targetTemperatureC - act->currentTemperatureC;

        if (act->targetTemperatureC < 20.0f || act->targetTemperatureC < MAX_ROOM_TEMPERATURE) {
            //targetTemperature is off
            //we never set heating on if the commanded temperature might be lower than max room temperature.
            output = 0; // off is off, even if damping term wants a heat peak!
            act->stopDecouple();
            act->tempIState = 0;
        } else if (error > PID_CONTROL_RANGE) {
            //heating up to targetTemperature
            output = act->pidMax;
            act->startFullDecoupleTest(time);
            act->tempIState = 0;
        } else if (error < -PID_CONTROL_RANGE) {
            //cooling down to targetTemperature
            output = 0;
            act->tempIState = 0;
        } else {
            act->startHoldDecoupleTest(time);

            float pidTerm = 0;
            float pgain = act->pidPGain * error;
            pidTerm += pgain;
            act->tempIState = constrain(act->tempIState + error, act->tempIStateLimitMin, act->tempIStateLimitMax);
            float igain = act->pidIGain * act->tempIState * 0.1; // 0.1 = 10Hz
            pidTerm += igain;
            float dgain = act->pidDGain * (act->tempArray[act->tempPointer] - act->currentTemperatureC) * 0.666f; // raising dT/dt, 0.666 = reciproke of time interval (1500 ms) -> temparray greift weiter zurück als letzte messung.
            pidTerm += dgain;


            output = constrain((int)pidTerm, 0, act->pidMax);
        }
        if (act->currentTemperatureC > act->heaterCutoffC)
            output = 0;
        HAL::setHeaterDuty(act->pwmIndex, output);

#if LED_PIN > -1
        if (act == &Extruder::current->tempControl)
            WRITE(LED_PIN, on);
#endif // LED_PIN>-1
    }  // for controller

    if (errorDetected == 0 && extruderTempErrors > 0)
        extruderTempErrors--;

    if (Printer::isAnyTempsensorDefect()) {
        stopAllHeaters();
        Printer::debugLevel |= 8; // Go into dry mode
    } else {
        if (!errorDetected)
            Printer::markTemperatureReadingsAvailable(); //we are not waiting for first temp measurements anymore.
    }

} // manageTemperatures

void TemperatureController::updateTempControlVars() {
    if (pidIGain != 0) // prevent division by zero
    {
        tempIStateLimitMax = (float)pidDriveMax * PID_CONTROL_DRIVE_MAX_LIMIT_FACTOR / pidIGain;
        tempIStateLimitMin = (float)pidDriveMin * PID_CONTROL_DRIVE_MIN_LIMIT_FACTOR / pidIGain; //Bisher hatte der PID-Regler keinen negativen I-Anteil, weil die Limits nicht ins Negative dürfen. Jetzt schon. Es ist das Minus in der Config, das die Stabilität bringt.
    }
} // updateTempControlVars

void TemperatureController::setTargetTemperature(float target) {
    if (target <= 0.0f && autotuneIndex < NUM_TEMPERATURE_LOOPS && tempController[autotuneIndex] == this)
        autotuneAbortRequested = true;
    if (target <= 0.0f || target < targetTemperatureC)
        HAL::setHeaterDuty(pwmIndex, 0);
    targetTemperatureC = target;
    stopDecouple();

    //do not set back temperatures to "not idle temperatures" after pause, if they have been changed manually within the pause.
    paused = 0;
} // setTargetTemperature

void TemperatureController::waitForTargetTemperature(uint8_t plus_temp_tolerance, bool resumingPause) {
    if (Printer::debugDryrun())
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
        if (resumingPause && !Printer::pause.isReheating())
            return;
        if (fabs(targetTemperatureC - currentTemperatureC) <= TEMP_TOLERANCE + plus_temp_tolerance)
            break;
        if (!dirRising && currentTemperatureC < MAX_ROOM_TEMPERATURE)
            break;
    }
    Printer::idleStatus.schedule(HAL::timeInMilliseconds());
}

void TemperatureController::autotunePID(float temp, uint8_t controllerId, int maxCycles, bool storeValues, int method) {
    if (autotuneIndex != 255) {
        Com::printErrorFLN(PSTR("PID autotune already active"));
        return; // Do not interfere with the existing autotune owner.
    }
    setTargetTemperature(0);
    if (controllerId >= NUM_TEMPERATURE_LOOPS || tempController[controllerId] != this
        || Printer::isAnyTempsensorDefect() || Printer::debugDryrun()
        || isSensorDefect() || isSensorDecoupled()
        || !(temp >= MAX_ROOM_TEMPERATURE && temp <= heaterCutoffC)
        || (controllerId == NUM_EXTRUDER && temp > HEATED_BED_MAX_TEMP)) {
        Com::printErrorFLN(PSTR("PID autotune failed: heater unavailable or target invalid"));
        return;
    }
    const bool sensorSupported = updateCurrentTemperature();
    if (!sensorSupported) {
        Com::printErrorFLN(PSTR("PID autotune failed: unsupported or stale temperature sensor"));
        return;
    }
    if (!validTemperatureSample(sensorSupported, currentTemperatureC) || sensorType == 0
        || currentTemperatureC > heaterCutoffC || currentTemperatureC > temp + 40) {
        Com::printErrorFLN(PSTR("PID autotune failed: invalid temperature"));
        return;
    }
    Printer::idleStatus.cancel();
    UI_STATUS_UPD(UI_TEXT_PID);
    float currentTemp;
    int cycles = 0;
    bool heating = true;

    uint32_t temp_millis = HAL::timeInMilliseconds();
    uint32_t t1 = temp_millis;
    uint32_t t2 = temp_millis;
    int32_t t_high = 0;
    int32_t t_low = 0;

    int32_t bias = pidMax >> 1;
    int32_t d = pidMax >> 1;
    float Ku, Tu;
    float Kp = 0, Ki = 0, Kd = 0;
    float maxTemp = 20, minTemp = 20;
    if (maxCycles < 5) {
        maxCycles = 5;
    }
    if (maxCycles > 20) {
        maxCycles = 20;
    }
    Com::printInfoFLN(Com::tPIDAutotuneStart);
    Com::printF(PSTR("Ruleset: "), (int)method);
    switch (method) {
    case 4: //Tyreus-Lyben
    {
        Com::printFLN(Com::tAPIDTyreusLyben);
        break;
    }
    case 3: //PID no overshoot
    {
        Com::printFLN(Com::tAPIDNone);
        break;
    }
    case 2: //PID some overshoot
    {
        Com::printFLN(Com::tAPIDSome);
        break;
    }
    case 1: //PID Pessen Integral Rule
    {
        Com::printFLN(Com::tAPIDPessen);
        break;
    }
    default: //PID classic Ziegler-Nichols
    {
        Com::printFLN(Com::tAPIDClassic);
        method = 0; //filter not available methods.
    }
    }

    autotuneAbortRequested = false;
    autotuneTemperatureLimit = temp + 40;
    if (autotuneTemperatureLimit > heaterCutoffC)
        autotuneTemperatureLimit = heaterCutoffC;
    autotuneIndex = controllerId;
    stopDecouple(); // Also valid when this controller is the heated bed.
    HAL::setHeaterDuty(pwmIndex, pidMax);
    if (controllerId < NUM_EXTRUDER) {
        extruder[controllerId].coolerPWM = extruder[controllerId].coolerSpeed;
        extruder[0].coolerPWM = extruder[0].coolerSpeed;
    }

    for (;;) {
        Commands::checkForPeriodicalActions(WaitHeater); // update heaters etc. https://github.com/repetier/Repetier-Firmware/commit/241c550ac004023842d6886c6e0db15a1f6b56d7
        const bool supported = updateCurrentTemperature();
        currentTemp = currentTemperatureC;
        if (autotuneAbortRequested || Printer::isAnyTempsensorDefect() || Printer::debugDryrun()
            || isSensorDefect() || isSensorDecoupled() || sensorType == 0
            || !validTemperatureSample(supported, currentTemp)) {
            finishAutotune(*this);
            Com::printErrorFLN(PSTR("PID autotune aborted: invalid sensor, heater fault or cancellation"));
            showError((void*)ui_text_autodetect_pid, (void*)ui_text_temperature_wrong);
            break;
        }

        millis_t time = HAL::timeInMilliseconds();
        if (currentTemp > autotuneTemperatureLimit) {
            finishAutotune(*this);
            Com::printErrorFLN(Com::tAPIDFailedHigh);
            showError((void*)ui_text_autodetect_pid, (void*)ui_text_temperature_wrong);
            break;
        }
        if (((time - t1) + (time - t2)) > (10L * 60L * 1000L * 2L)) {
            finishAutotune(*this);
            Com::printErrorFLN(Com::tAPIDFailedTimeout);
            showError((void*)ui_text_autodetect_pid, (void*)ui_text_timeout);
            break;
        }
        maxTemp = RMath::max(maxTemp, currentTemp);
        minTemp = RMath::min(minTemp, currentTemp);
        if (heating == true && currentTemp > temp) // switch heating -> off
        {
            if (time - t2 > (controllerId < NUM_EXTRUDER ? 2500 : 1500)) {
                heating = false;
                HAL::setHeaterDuty(pwmIndex, (bias - d));
                t1 = time;
                t_high = t1 - t2;
                maxTemp = temp;
            }
        }
        if (heating == false && currentTemp < temp) {
            if (time - t1 > (controllerId < NUM_EXTRUDER ? 5000 : 3000)) {
                heating = true;
                t2 = time;
                t_low = t2 - t1; // half wave length
                if (cycles > 0) {
                    bias += (d * (t_high - t_low)) / (t_low + t_high);
                    bias = constrain(bias, 20, pidMax - 20);
                    if (bias > pidMax / 2)
                        d = pidMax - 1 - bias;
                    else
                        d = bias;

                    Com::printF(Com::tAPIDBias, bias);
                    Com::printF(Com::tAPIDD, d);
                    Com::printF(Com::tAPIDMin, minTemp);
                    Com::printFLN(Com::tAPIDMax, maxTemp);
                    if (cycles > 2) {
                        // Parameter according Ziegler¡§CNichols method: http://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
                        Ku = (4.0 * d) / (3.14159 * (maxTemp - minTemp));
                        Tu = static_cast<float>(t_low + t_high) / 1000.0;
                        Com::printF(Com::tAPIDKu, Ku);
                        Com::printFLN(Com::tAPIDTu, Tu);
                        /**
                        https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
                        KP = KP
                        KI = KP / Ti
                        KD = KP * Td
                        wegen Formel u(t) = KP*( rest )

                        Ti = Tu / Maßzahl lt. Tabelle
                        Td = Tu / Maßzahl lt. Tabelle
                        KP = Ku * Maßzahl lt. Tabelle

                        see also: http://www.mstarlabs.com/control/znrule.html
                        */
                        switch (method) {
                        case 4: {                //PID Tyreus-Lyben ("lazy--" -> Heated Bed!)
                            Kp = 0.4545f * Ku;   //1/2.2 KRkrit
                            Ki = Kp / Tu / 2.2f; //2.2 Tkrit
                            Kd = Kp * Tu / 6.3f; //1/6.3 Tkrit[/code]
                            Com::printFLN(Com::tAPIDTyreusLyben);
                            break;
                        }
                        case 3: {                //PID no overshoot ("lazy-" -> Heated Bed!)
                            Kp = 0.2f * Ku;      //0.2 KRkrit
                            Ki = 2.0f * Kp / Tu; //0.5 Tkrit
                            Kd = Kp * Tu / 3.0f; //0.333 Tkrit
                            Com::printFLN(Com::tAPIDNone);
                            break;
                        }
                        case 2: {                //PID some overshoot
                            Kp = 0.33f * Ku;     //0.33 KRkrit
                            Ki = 2.0f * Kp / Tu; //0.5 Tkrit
                            Kd = Kp * Tu / 3.0f; //0.333 Tkrit
                            Com::printFLN(Com::tAPIDSome);
                            break;
                        }
                        case 1: {                        //PID Pessen Integral Rule ("dynamic++" -> fast Hotend!)
                            Kp = 0.7f * Ku;              //0.7 KRkrit
                            Ki = 2.5f * Kp / Tu;         //0.4 Tkrit
                            Kd = Kp * Tu * 3.0f / 20.0f; //0.15 Tkrit
                            Com::printFLN(Com::tAPIDPessen);
                            break;
                        }
                        default: {               //PID classic Ziegler-Nichols  ("dynamic+" -> Hotend!)
                            Kp = 0.6f * Ku;      //0.6 KRkrit
                            Ki = 2.0f * Kp / Tu; //0.5 Tkrit
                            Kd = Kp * Tu / 8.0f; //0.125 Tkrit
                            Com::printFLN(Com::tAPIDClassic);
                        }
                        }
                        Com::printFLN(Com::tAPIDKp, Kp);
                        Com::printFLN(Com::tAPIDKi, Ki);
                        Com::printFLN(Com::tAPIDKd, Kd);
                    }
                }
                cycles++;
                if (cycles <= maxCycles)
                    HAL::setHeaterDuty(pwmIndex, (bias + d));
                minTemp = temp;
            }
        }
        if (cycles > maxCycles) {
            finishAutotune(*this);
            Com::printInfoFLN(Com::tAPIDFinished);
            UI_STATUS_UPD(UI_TEXT_AUTODETECT_PID_DONE);
            if (storeValues) {
                pidPGain = Kp;
                pidIGain = Ki;
                pidDGain = Kd;
                EEPROM::storeDataIntoEEPROM();
            }
            Printer::idleStatus.schedule(HAL::timeInMilliseconds() + 30000);
            return;
        }
    }
    Printer::idleStatus.schedule(HAL::timeInMilliseconds());
} // autotunePID

void reportTempsensorAndHeaterErrors() {
    //Write full status list to console output
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
    Com::printErrorFLN(Com::tDryModeUntilRestart);
} // reportTempsensorAndHeaterErrors

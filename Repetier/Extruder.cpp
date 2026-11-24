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
#include "pins_arduino.h"

#include "Eeprom.h"

// Preserve the heated bed's existing measured-temperature cutoff. Its setpoint has its own limit.
constexpr uint16_t bedHeaterCutoffC = 275;

void Extruder::initHeatedBed() {
    heatedBedController.updateTempControlVars();
} // initHeatedBed


/** \brief Initalizes all extruder.
Updates the pin configuration needed for the extruder and activates extruder 0.
Starts a interrupt based analog input reader, which is used by simple thermistors
for temperature reading.
*/
void Extruder::initExtruder() {
    uint8_t i;
    Extruder::current = &extruder[0];




#if defined(EXT0_STEP_PIN) && EXT0_STEP_PIN > -1
    SET_OUTPUT(EXT0_DIR_PIN);
    SET_OUTPUT(EXT0_STEP_PIN);
#endif // defined(EXT0_STEP_PIN) && EXT0_STEP_PIN>-1

#if defined(EXT1_STEP_PIN) && EXT1_STEP_PIN > -1 && NUM_EXTRUDER > 1
    SET_OUTPUT(EXT1_DIR_PIN);
    SET_OUTPUT(EXT1_STEP_PIN);
#endif // defined(EXT1_STEP_PIN) && EXT1_STEP_PIN>-1 && NUM_EXTRUDER>1

    for (i = 0; i < NUM_EXTRUDER; ++i) {
        Extruder* act = &extruder[i];
        if (act->enablePin > -1) {
            HAL::pinMode(act->enablePin, OUTPUT);
            if (!act->enableOn)
                HAL::digitalWrite(act->enablePin, HIGH);
        }
        act->tempControl.lastTemperatureUpdate = HAL::timeInMilliseconds();
        act->tempControl.updateTempControlVars();
    }
} // initExtruder

bool Extruder::selectExtruderById(uint8_t extruderId, bool waitThroughPause) {
    const auto pauseWait = waitThroughPause ? Commands::PauseWait::Resume : Commands::PauseWait::Cancel;
    if (!Commands::waitForMotion(Commands::MotionWait::AllOutputs, pauseWait))
        return false;

#if NUM_EXTRUDER > 1
    if (extruderId >= NUM_EXTRUDER || extruderId == current->id)
        return true;

    // Correct X and Y on homed axes. Homing positions the other axes later.
    int32_t dx = 0;
    int32_t dy = 0;
    if (Printer::isAxisHomed(X_AXIS))
        dx = (extruder[extruderId].offsetMM[X_AXIS] - current->offsetMM[X_AXIS]) * Printer::stepsPerMm(X_AXIS);
    if (Printer::isAxisHomed(Y_AXIS))
        dy = (extruder[extruderId].offsetMM[Y_AXIS] - current->offsetMM[Y_AXIS]) * Printer::stepsPerMm(Y_AXIS);

    {
        InterruptProtectedBlock lock;
        current->enabled = 0;
        current = &extruder[extruderId];
        Printer::extrusionOutput.resetDirection();
    }
    applyCurrentSettings();
    Printer::offsetRelativeStepsCoordinates(-dx, -dy, 0, 0);

    uid.setStatusP(current->id == 0 ? PSTR("Extruder 0") : PSTR("Extruder 1"));
    uid.refreshPage();
#else
    (void)extruderId;
#endif
    return true;
}

void Extruder::applyCurrentSettings() {
    Printer::applyToolMotionSettings(Extruder::current->stepsPerMM,
                                    Extruder::current->maxFeedrate, Extruder::current->maxAcceleration);

    float minimumESpeed = 1.41 * Extruder::current->maxAcceleration * sqrt(2.0f / (Extruder::current->stepsPerMM * Extruder::current->maxAcceleration));
    if (minimumESpeed > Extruder::current->maxEJerk) {
        Extruder::current->maxEJerk = minimumESpeed;
        Com::printFLN(PSTR("E jerk was too low, setting to "), minimumESpeed);
    }

    Printer::manualMotion.setSteps(E_AXIS, (unsigned short)lroundf(Printer::stepsPerMm(E_AXIS) * DEFAULT_MANUAL_MM_E));
    Printer::pause.setParkSteps(E_AXIS, long(Printer::stepsPerMm(E_AXIS) * SCRIPT_RETRACT_MM));

    uint8_t stepInterval = (uint8_t)floor(HAL::maxExtruderTimerFrequency() / (Extruder::current->maxFeedrate * Extruder::current->stepsPerMM));
    if (stepInterval > 15)
        stepInterval = 15;
    Printer::extrusionOutput.setStepInterval(stepInterval);
    float fmax = HAL::maxExtruderTimerFrequency() / (float(stepInterval) * Printer::stepsPerMm(E_AXIS));
    // Limit feedrate to interrupt speed
    if (fmax < Printer::axisMaxFeedrate(E_AXIS))
        Printer::setAxisMaxFeedrate(E_AXIS, fmax);
    Extruder::current->tempControl.updateTempControlVars();
}

void Extruder::setTemperatureForExtruder(float temperatureInCelsius, uint8_t extr) {
    if (extr >= NUM_EXTRUDER) {
        // do not set the temperature for an extruder which is not present - this attempt could heat up the extruder without any control and could significantly overheat the extruder
        if (temperatureInCelsius > 0)
            Com::printFLN(PSTR("setTemperatureForExtruder(): cant set Temp for Extr. T"), extr);
        return;
    }

    bool alloffs = true;
    for (uint8_t i = 0; i < NUM_EXTRUDER; i++)
        if (tempController[i]->targetTemperatureC > 0)
            alloffs = false;

    TemperatureController* tc = tempController[extr];
    if (temperatureInCelsius > tc->heaterCutoffC)
        temperatureInCelsius = tc->heaterCutoffC;
    if (temperatureInCelsius < 0)
        temperatureInCelsius = 0;
    if (tc->isSensorDefect() || tc->isSensorDecoupled())
        temperatureInCelsius = 0;
    if (tc->sensorType == 0)
        temperatureInCelsius = 0;
    tc->setTargetTemperature(temperatureInCelsius);
    tc->updateTempControlVars();

    if (temperatureInCelsius >= EXTRUDER_FAN_COOL_TEMP)
        extruder[extr].coolerPWM = extruder[extr].coolerSpeed;

    if (Printer::debugInfo()) {
        Com::printF(Com::tTargetExtr, extr, 0);
        Com::printFLN(Com::tColon, temperatureInCelsius, 0);
    }

#if (MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2) && CASE_FAN_PIN > -1
    if (Printer::ignoreFanOn) {
        //ignore the case fan whenever there is another cooling solution available // Nibbels
        //the fan should still be connected within the rf2000 but might be avoided to suppress noise.
        //the ignore-flag has to be set at runtime to prevent unlearned persons to risk overheat having the wrong startcode.
        //enable and disable the fan with M3120 or M3121 or M3300 P3 S{1,0}
        Printer::prepareFanOff = 0;
        Printer::fanOffDelay = 0;
    } else {
        bool isheating = false;
        for (uint8_t i = 0; i < NUM_EXTRUDER; i++)
            if (tempController[i]->targetTemperatureC > CASE_FAN_ON_TEMPERATURE)
                isheating = true;
        if (isheating) {
            // enable the case fan in case any extruder is turned on
            Printer::prepareFanOff = 0;
            WRITE(CASE_FAN_PIN, 1);
        } else {
            // disable the case fan in case the extruder is turned off
            if (Printer::fanOffDelay) {
                // we are going to disable the case fan after the delay
                Printer::prepareFanOff = HAL::timeInMilliseconds();
            } else {
                // we are going to disable the case fan now
                Printer::prepareFanOff = 0;
                WRITE(CASE_FAN_PIN, 0);
            }
        }
    }
#endif // (MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2) && CASE_FAN_PIN > -1

    bool alloff = true;
    for (uint8_t i = 0; i < NUM_EXTRUDER; i++)
        if (tempController[i]->targetTemperatureC > 0)
            alloff = false;

    if (alloff && !alloffs) // All heaters are now switched off?
    {
        EEPROM::updatePrinterUsage();
    }

    if (alloffs && !alloff) // heaters are turned on, start measuring printing time
    {
        Printer::msecondsPrinting = HAL::timeInMilliseconds();
        Printer::filamentPrinted = 0; // new print, new counter
        Printer::flag2 &= ~PRINTER_FLAG2_RESET_FILAMENT_USAGE;
    }
} // setTemperatureForExtruder

void Extruder::setTemperatureForAllExtruders(float temperatureInCelsius) {
    for (uint8_t extrNr = 0; extrNr < NUM_EXTRUDER; extrNr++)
        Extruder::setTemperatureForExtruder(temperatureInCelsius, extrNr);
}

void Extruder::setHeatedBedTemperature(float temperatureInCelsius) {
    if (temperatureInCelsius > HEATED_BED_MAX_TEMP)
        temperatureInCelsius = HEATED_BED_MAX_TEMP;
    if (temperatureInCelsius < 0)
        temperatureInCelsius = 0;
    if (heatedBedController.isSensorDefect() || heatedBedController.isSensorDecoupled())
        temperatureInCelsius = 0;

    if (heatedBedController.targetTemperatureC == temperatureInCelsius)
        return; // don't flood log with messages if killed

    heatedBedController.setTargetTemperature(temperatureInCelsius);

    if (Printer::debugInfo()) {
        Com::printFLN(Com::tTargetBedColon, heatedBedController.targetTemperatureC, 0);
    }
} // setHeatedBedTemperature

float Extruder::getHeatedBedTemperature() {
    return heatedBedController.currentTemperatureC;
} // getHeatedBedTemperature

void Extruder::disableAllExtruders() {
    {
        InterruptProtectedBlock lock;
        const int16_t discarded = Printer::extrusionOutput.inhibit(ExtrusionOutput::Inhibit::MotorsDisabled);
        Printer::pause.discardPendingExtrusion(discarded);
        for (uint8_t i = 0; i < NUM_EXTRUDER; ++i) {
            auto& tool = extruder[i];
            if (tool.enablePin > -1)
                HAL::digitalWrite(tool.enablePin, !tool.enableOn);

            tool.enabled = 0;
        }
    }

    cleanupEPositions();
} // disableAllExtruders

Extruder* Extruder::current;

Extruder extruder[NUM_EXTRUDER] = {
    { 0,
      { EXT0_X_OFFSET_MM, EXT0_Y_OFFSET_MM },
      EXT0_STEPS_PER_MM,
      EXT0_ENABLE_PIN,
      EXT0_ENABLE_ON,
      EXT0_MAX_FEEDRATE,
      EXT0_MAX_ACCELERATION,
      EXT0_MAX_START_FEEDRATE,
      0,
      EXT0_ADVANCE_L,
      {
          0, EXT0_TEMPSENSOR_TYPE, Hotend0AdcIndex, EXT0_HOTEND_PROFILE.maxTemperatureC, 0, 0, 0,
          0,
          0,
          EXT0_HOTEND_PROFILE.pidDriveMax,
          EXT0_HOTEND_PROFILE.pidDriveMin,
          EXT0_HOTEND_PROFILE.pidPGain,
          EXT0_HOTEND_PROFILE.pidIGain,
          EXT0_HOTEND_PROFILE.pidDGain,
          EXT0_HOTEND_PROFILE.pidMax,
          0,
          0,
          0,
          { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
          0,
          0,
          0,
          EXT0_DECOUPLE_TEST_PERIOD,
          0 //uint8_t paused
      },
      EXT0_EXTRUDER_COOLER_SPEED,
      0,
      0
    }

#if NUM_EXTRUDER > 1
    ,
    { 1,
      { EXT1_X_OFFSET_MM, EXT1_Y_OFFSET_MM },
      EXT1_STEPS_PER_MM,
      EXT1_ENABLE_PIN,
      EXT1_ENABLE_ON,
      EXT1_MAX_FEEDRATE,
      EXT1_MAX_ACCELERATION,
      EXT1_MAX_START_FEEDRATE,
      0,
      EXT1_ADVANCE_L,
      {
          1, EXT1_TEMPSENSOR_TYPE, Hotend1AdcIndex, EXT1_HOTEND_PROFILE.maxTemperatureC, 0, 0, 0,
          0,
          0,
          EXT1_HOTEND_PROFILE.pidDriveMax,
          EXT1_HOTEND_PROFILE.pidDriveMin,
          EXT1_HOTEND_PROFILE.pidPGain,
          EXT1_HOTEND_PROFILE.pidIGain,
          EXT1_HOTEND_PROFILE.pidDGain,
          EXT1_HOTEND_PROFILE.pidMax,
          0,
          0,
          0,
          { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
          0,
          0,
          0,
          EXT1_DECOUPLE_TEST_PERIOD,
          0 //uint8_t paused
      },
      EXT1_EXTRUDER_COOLER_SPEED,
      0,
      0
    }
#endif // NUM_EXTRUDER>1
};

TemperatureController heatedBedController = {
    NUM_EXTRUDER, HEATED_BED_SENSOR_TYPE, BedAdcIndex, bedHeaterCutoffC, 0, 0, 0,
    0,
    0,
    HEATED_BED_PID_INTEGRAL_DRIVE_MAX,
    HEATED_BED_PID_INTEGRAL_DRIVE_MIN,
    HEATED_BED_PID_PGAIN,
    HEATED_BED_PID_IGAIN,
    HEATED_BED_PID_DGAIN,
    HEATED_BED_PID_MAX,
    0,
    0,
    0,
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    0,
    0,
    0,
    BED_DECOUPLE_TEST_PERIOD,
    0 //uint8_t paused
};

#ifdef X35_ADC_CHANNEL
/** \brief This is an optional Temperature Sensor. //Nibbels
There is no Controller involved! It is not hooked into tempController
Do not try to do anything other than updateCurrentTemperature and reading the Temps.
TODO: Making a totally clean class, without the logic to controll something.
*/
TemperatureController optTempController = {
    0, RESERVE_ANALOG_SENSOR_TYPE, ChamberAdcIndex, 0, 0, 0, 0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    0,
    0,
    0,
    0 /*0 = no decouple test*/,
    0 //uint8_t paused
};
#endif // X35_ADC_CHANNEL

TemperatureController* tempController[NUM_TEMPERATURE_LOOPS] = {
    &extruder[0].tempControl

#if NUM_EXTRUDER > 1
    ,
    &extruder[1].tempControl
#endif // NUM_EXTRUDER>1

    ,
    &heatedBedController
};

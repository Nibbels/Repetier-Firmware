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

// Tool selection, E calibration and heater target interfaces; regulation and sensor conversion are in TemperatureControl.cpp and TemperatureSensors.cpp.

#include "Repetier.h"
#include "pins_arduino.h"

#include "Eeprom.h"
#include "CaseFan.h"

// Absolute ceiling is independent of the sensor conversion range.
// The separately commissioned bed limit can only lower this ceiling.
constexpr uint16_t bedHeaterCutoffC = TemperatureController::BedAbsoluteCeilingC;

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

// Cold settings calculation, not part of the timer ISR. Bound the ratio
// before conversion: an interval of zero divides by zero below, while a
// ratio above 255 is not representable as uint8_t. The existing upper
// limit is 15 ticks; negative/NaN input cannot define a valid rate.
static uint8_t boundedExtruderStepInterval(float ratio) {
    if (!(ratio >= 0.0f) || ratio >= 15.0f)
        return 15;
    return ratio < 1.0f ? 1 : uint8_t(ratio);
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

    const uint8_t stepInterval = boundedExtruderStepInterval(HAL::maxExtruderTimerFrequency() /
        (Extruder::current->maxFeedrate * Extruder::current->stepsPerMM));
    Printer::extrusionOutput.setStepInterval(stepInterval);
    float fmax = HAL::maxExtruderTimerFrequency() / (float(stepInterval) * Printer::stepsPerMm(E_AXIS));
    // Limit feedrate to interrupt speed
    if (fmax < Printer::axisMaxFeedrate(E_AXIS))
        Printer::setAxisMaxFeedrate(E_AXIS, fmax);
    Extruder::current->tempControl.updateTempControlVars();
}

bool Extruder::hasActiveHotendTarget() {
    for (uint8_t i = 0; i < NUM_EXTRUDER; ++i)
        if (tempController[i]->targetTemperatureC > 0)
            return true;
    return false;
}

void Extruder::setTemperatureForExtruder(float temperatureInCelsius, uint8_t extr) {
    if (extr >= NUM_EXTRUDER) {
        // do not set the temperature for an extruder which is not present - this attempt could heat up the extruder without any control and could significantly overheat the extruder
        if (temperatureInCelsius > 0)
            Com::printFLN(PSTR("setTemperatureForExtruder(): cant set Temp for Extr. T"), extr);
        return;
    }

    TemperatureController* tc = tempController[extr];
    if (temperatureInCelsius > tc->heaterCutoffC)
        temperatureInCelsius = tc->heaterCutoffC;
    if (temperatureInCelsius < 0)
        temperatureInCelsius = 0;
    if (Printer::heatingInhibited() || tc->isSensorDefect() || tc->isSensorDecoupled())
        temperatureInCelsius = 0;
    if (tc->sensorType == 0)
        temperatureInCelsius = 0;
    tc->setTargetTemperature(temperatureInCelsius);
    temperatureInCelsius = tc->targetTemperatureC;
    tc->updateTempControlVars();

    if (temperatureInCelsius >= EXTRUDER_FAN_COOL_TEMP)
        extruder[extr].coolerPWM = extruder[extr].coolerSpeed;

    Com::printF(Com::tTargetExtr, extr, 0);
    Com::printFLN(Com::tColon, temperatureInCelsius, 0);

#if (MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2) && CASE_FAN_PIN > -1
    CaseFan::updateFromHotendTargets();
#endif // (MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2) && CASE_FAN_PIN > -1

} // setTemperatureForExtruder

void Extruder::setTemperatureForAllExtruders(float temperatureInCelsius) {
    for (uint8_t extrNr = 0; extrNr < NUM_EXTRUDER; extrNr++)
        Extruder::setTemperatureForExtruder(temperatureInCelsius, extrNr);
}

void Extruder::setHeatedBedTemperature(float temperatureInCelsius) {
    const float limit = RMath::min(float(HEATED_BED_MAX_TEMP), float(heatedBedController.maximumTargetC()));
    if (temperatureInCelsius > limit)
        temperatureInCelsius = limit;
    if (temperatureInCelsius < 0)
        temperatureInCelsius = 0;
    if (Printer::heatingInhibited() || heatedBedController.isSensorDefect() || heatedBedController.isSensorDecoupled())
        temperatureInCelsius = 0;

    if (heatedBedController.targetTemperatureC == temperatureInCelsius) {
        // Autotune holds a zero normal target while driving its own PWM.
        // An explicit off command must still revoke duty and cancel that tune.
        if (temperatureInCelsius == 0)
            heatedBedController.setTargetTemperature(0);
        return; // don't flood log with messages if killed
    }

    heatedBedController.setTargetTemperature(temperatureInCelsius);

    Com::printFLN(Com::tTargetBedColon, heatedBedController.targetTemperatureC, 0);
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
          0, 0,
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
          0, 0,
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
    0, 0,
    0,
    0,
    0,
    BED_DECOUPLE_TEST_PERIOD,
    0 //uint8_t paused
};

TemperatureController* tempController[NUM_TEMPERATURE_LOOPS] = {
    &extruder[0].tempControl

#if NUM_EXTRUDER > 1
    ,
    &extruder[1].tempControl
#endif // NUM_EXTRUDER>1

    ,
    &heatedBedController
};

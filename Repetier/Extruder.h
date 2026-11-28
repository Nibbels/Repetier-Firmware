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

#ifndef EXTRUDER_H
#define EXTRUDER_H

#include "HAL.h"
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
#include "ThermalEmergency.h"
#endif

#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
#include "BedTemperatureLimit.h"
#endif

#define CELSIUS_EXTRA_BITS 3

// Report latched sensor/heater faults; regulation is in TemperatureControl.cpp.
extern void reportTempsensorAndHeaterErrors();

#define TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL 2  ///< Full heating enabled
#define TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD 4  ///< Holding target temperature
#define TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT 8     ///< Indicating sensor defect
#define TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED 16 ///< Indicating sensor decoupling
/** TemperatureController represents one heater/sensor loop.
The supported RF builds use one or two hotends plus the heated bed.

*/
class TemperatureController {
public:
    uint8_t pwmIndex;           ///< pwm index for output control. 0-2 = Extruder, 3 = Fan, 4 = Heated Bed
    uint8_t sensorType;         ///< Type of temperature sensor.
    uint8_t sensorPin;          ///< Index into the ADC sample array.
    const uint16_t heaterCutoffC; ///< Immutable measured-temperature ceiling.
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    enum : uint16_t { BedAbsoluteCeilingC=BedTemperatureLimit::AbsoluteC,
                      BedDefaultCutoffC=BedTemperatureLimit::DefaultC,
                      BedTargetMarginC=BedTemperatureLimit::MarginC };
    uint16_t effectiveCutoffC() const {
        return pwmIndex==NUM_EXTRUDER && heaterCutoffC>bedLimit_.monitor()
            ? uint16_t(bedLimit_.monitor()) : heaterCutoffC;
    }
    uint16_t maximumTargetC() const {
        if(pwmIndex!=NUM_EXTRUDER) return heaterCutoffC;
        if(!bedLimit_.commissioned()) return 0;
        const uint16_t ceiling=effectiveCutoffC();
        return ceiling>BedTargetMarginC ? ceiling-BedTargetMarginC : 0;
    }
    // Foreground settings owner only. Zero revokes bed permission.
    bool setBedEmergencyCutoff(uint8_t verifiedCutoff);
    static uint8_t commissionedBedCutoff() {return bedLimit_.commissioned();}
private:
    static BedTemperatureLimit bedLimit_;
public:
#else
    uint16_t effectiveCutoffC() const { return heaterCutoffC; }
    uint16_t maximumTargetC() const { return heaterCutoffC; }
#endif
    int16_t currentTemperature; ///< Currenttemperature value read from sensor.
    float currentTemperatureC;  ///< Current temperature in degC.
    float targetTemperatureC;   ///< Target temperature in degC.

    uint32_t lastTemperatureUpdate; ///< Time in millis of the last temperature update.

    float tempIState;    ///< Integral contribution in PWM counts, not accumulated error.
    uint8_t pidDriveMax; ///< Used for windup in PID calculation.
    uint8_t pidDriveMin; ///< Used for windup in PID calculation.
    float pidPGain;      ///< Pgain (proportional gain) for parallel PID (PWM counts, Celsius, seconds).
    float pidIGain;      ///< Igain (integral) for parallel PID (PWM counts, Celsius, seconds).
    float pidDGain;      ///< Dgain (damping) for parallel PID (PWM counts, Celsius, seconds).
    uint8_t pidMax;      ///< Maximum PWM value, the heater should be set.
    float tempIStateLimitMax;
    float tempIStateLimitMin;
    bool pidHistoryValid;
    float previousTemperature;
    float filteredSlope;

    uint8_t flags;

    millis_t lastDecoupleTest;   ///< Last time of decoupling sensor-heater test
    float lastDecoupleTemp;      ///< Temperature on last test
    millis_t decoupleTestPeriod; ///< Time between setting and testing decoupling.

    uint8_t paused;
    // A saved tune is tied to the actual sensor and power limit.
    uint8_t tunedSensorType;
    uint8_t tunedPowerLimit;

    static bool validPidGains(float p, float i, float d) {
        return p > 0 && p <= 1000 && i > 0 && i <= 1000 && d >= 0 && d <= 100000;
    }
    bool pidCalibrationValid() const {
        return sensorType != 0 && tunedSensorType == sensorType && tunedPowerLimit == pidMax
            && pidMax >= 40 && validPidGains(pidPGain, pidIGain, pidDGain);
    }
    void invalidatePidCalibration();
    static bool autotuneProfileAllowed(uint8_t controller, int method) {
        return controller <= NUM_EXTRUDER
            && (controller == NUM_EXTRUDER ? method == 2 || method == 3 : method == 0 || method == 1);
    }

    void clearTargetTemperature();
    void setTargetTemperature(float target);
#ifdef RF_HEATER_LEASE_EXPERIMENT
    bool updateCurrentTemperature(HAL::HeaterSample* sample = nullptr
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        , ThermalEmergency::SensorObservation* observation = nullptr
#endif
    );
#else
    bool updateCurrentTemperature(
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        ThermalEmergency::SensorObservation* observation = nullptr
#endif
    );
#endif
    // ///< False for an unsupported sensor type or stale/unpublished ADC sample.
    void updateTempControlVars();
    void waitForTargetTemperature(uint8_t plus_temp_tolerance = 0, bool resumingPause = false);
    void autotunePID(float temp, uint8_t controllerId, int maxCycles, bool storeResult, int method);
private:
    void autotunePIDPrepared(float temp, uint8_t controllerId, int maxCycles, bool storeResult, int method);
public:
    static bool autotuneActive();

    // Code für den Decouple-Test der Heizkreisläufe:
    inline bool isDecoupleFull() {
        return flags & TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL;
    }
    inline void setDecoupleFull() {
        flags &= ~(TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD); // Hold aus
        flags |= TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL;    // Full an
    }
    inline bool isDecoupleHold() {
        return flags & TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD;
    }
    inline void setDecoupleHold() {
        flags &= ~(TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL); // Full aus
        flags |= TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD;    // Hold an
    }
    inline void stopDecouple() {
        flags &= ~(TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL | TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD);
    }

    // Phase 1: wir stehen auf einer Stufe und merken uns die Zeit und die Temperatur. Das wird später mit der neuen Temperatur verglichen und dann muss die Temperatur gestiegen sein.
    inline void startFullDecoupleTest(millis_t& t) {
        if (isDecoupleFull())
            return;
        lastDecoupleTest = t;
        lastDecoupleTemp = currentTemperatureC;
        // Switch active test method to "temperature has to rise":
        setDecoupleFull();
    }

    // Phase 2: Wir sind voll aufgeheizt und prüfen ob die Temperatur ohne Änderung der Zieltemperatur abdriftet.
    //         Schlägt an, wenn die Heizkartusche kaputt geht oder rausrutscht. Oder auch wenn der Sensorwert willkürlich wandert.
    //         Ist der Sensor ausserhalb der bekannten MIN-MAX Limits wird das auch erkannt.
    //
    //         ???:
    //         Der Regler steuert, wenn der Fehler anfängt, soweit es geht noch nach. Driftet der Sensorwert wegen schleichendem Sensordefekt !langsam! nach unten, heizt der PID-Regler um so mehr.
    //         Solange der PID-Regler den Drift kompensieren kann und das System glaubt, die Temperaturen sind im Rahmen von MIN-MAX könnte der Heizblock heißer sein als gewollt!
    //         Allerdings sollte der Thermistor > 300°C schnell das Zeitliche segnen völligen Mist anzeigen und dann greifen die Checks wieder.
    inline void startHoldDecoupleTest(millis_t& t) {
        // Nur starten, wenn es nicht schon läuft.
        if (isDecoupleHold())
            return;
        // Nicht starten, wenn die temperatur "noch" ausserhalb der decoupling varianz liegt. kann z.b. sein, dass die pid-control-range 30 grad ist und das hier 20.
        if (fabs(currentTemperatureC - targetTemperatureC) + 1 > DECOUPLING_TEST_MAX_HOLD_VARIANCE)
            return;
        // Beim ersten start setzt man den timer auf "jetzt". Es dauert also bis zum ersten check nicht 1 sekunde sondern so wie die config eingestellt ist.
        // Erst danach macht der code jede sekunde den "Wackelcheck".
        lastDecoupleTest = t;
        lastDecoupleTemp = targetTemperatureC;
        // Switch active test method to "temperature has to stay where it was":
        setDecoupleHold();
    }

    //4 Funktionen um die Sensorstatus Flags in den einzelnen TemperatureController zu schreiben.
    inline void setSensorDefect(bool on) {
        if (on)
            flags |= TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT;
        else
            flags &= ~TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT;
    }
    inline bool isSensorDefect() {
        return flags & TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT;
    }
    inline void setSensorDecoupled(bool on) {
        if (on)
            flags |= TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED;
        else
            flags &= ~TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED;
    }
    inline bool isSensorDecoupled() {
        return flags & TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED;
    }
}; // TemperatureController

class Extruder;
extern Extruder extruder[];

/** \brief Data to drive one extruder.

This structure contains all definitions for an extruder and all
current state variables, like current temperature, feeder position etc.
*/
class Extruder
{
public:
    static Extruder* current;

    uint8_t id;
    float offsetMM[2]; ///< X/Y tool offsets in mm.
    float stepsPerMM; ///< Steps per mm.
    int8_t enablePin; ///< Pin to enable extruder stepper motor.
    uint8_t enableOn;
    float maxFeedrate;              ///< Maximum feedrate in mm/s.
    float maxAcceleration;          ///< Maximum acceleration in mm/s^2.
    float maxEJerk;                 ///< Maximum start feedrate in mm/s.
    volatile int8_t stepperDirection;

    float advanceL;

    TemperatureController tempControl;
    uint8_t coolerSpeed; ///< Speed to use when enabled
    uint8_t coolerPWM;   ///< current PWM setting

    char enabled;

    /** \brief Sends the high-signal to the stepper for next extruder step.
    Call this function only, if interrupts are disabled.
    */
    static INLINE void step() {
#if NUM_EXTRUDER == 1
        WRITE(EXT0_STEP_PIN, HIGH);
#else
        switch (Extruder::current->id) {
        case 0: {
            WRITE(EXT0_STEP_PIN, HIGH);

            break;
        }

#if defined(EXT1_STEP_PIN) && NUM_EXTRUDER > 1
        case 1: {
            WRITE(EXT1_STEP_PIN, HIGH);
            break;
        }
#endif // defined(EXT1_STEP_PIN) && NUM_EXTRUDER>1
        }
#endif
    } // step

    /** \brief Sets stepper signal to low for current extruder.
    Call this function only, if interrupts are disabled.
    */
    static INLINE void unstep() {
#if NUM_EXTRUDER == 1
        WRITE(EXT0_STEP_PIN, LOW);
#else
        switch (Extruder::current->id) {
        case 0: {
            WRITE(EXT0_STEP_PIN, LOW);

            break;
        }

#if defined(EXT1_STEP_PIN) && NUM_EXTRUDER > 1
        case 1: {
            WRITE(EXT1_STEP_PIN, LOW);
            break;
        }
#endif // defined(EXT1_STEP_PIN) && NUM_EXTRUDER>1
        }
#endif // NUM_EXTRUDER==1

    } // unstep

    // Select the tool before either edge of the timed Pressure Advance pulse.
    static INLINE void timedStep(volatile int16_t& pending, int16_t remaining) {
#if NUM_EXTRUDER == 1
        STEP_WITH_COUNT(EXT0_STEP_PIN, pending, remaining);
#else
        switch (Extruder::current->id) {
        case 0:
            STEP_WITH_COUNT(EXT0_STEP_PIN, pending, remaining);
            break;
#ifdef EXT1_STEP_PIN
        case 1:
            STEP_WITH_COUNT(EXT1_STEP_PIN, pending, remaining);
            break;
#endif
        default:
            pending = remaining;
            break;
        }
#endif
    }

    /** \brief Activates the extruder stepper and sets the direction. */
    static inline void setDirection(uint8_t dir) {
#if NUM_EXTRUDER == 1
        if (dir) {
            if (Extruder::current->stepperDirection != 1) {
                WRITE(EXT0_DIR_PIN, !EXT0_INVERSE);
                Extruder::current->stepperDirection = 1;
            }
        } else {
            if (Extruder::current->stepperDirection != -1) {
                WRITE(EXT0_DIR_PIN, EXT0_INVERSE);
                Extruder::current->stepperDirection = -1;
            }
        }
#else
        switch (Extruder::current->id) {
        case 0: {
            if (dir)
                WRITE(EXT0_DIR_PIN, !EXT0_INVERSE);
            else
                WRITE(EXT0_DIR_PIN, EXT0_INVERSE);

            break;
        }

#if defined(EXT1_DIR_PIN) && NUM_EXTRUDER > 1
        case 1: {
            if (dir)
                WRITE(EXT1_DIR_PIN, !EXT1_INVERSE);
            else
                WRITE(EXT1_DIR_PIN, EXT1_INVERSE);
            break;
        }
#endif // defined(EXT1_DIR_PIN) && NUM_EXTRUDER>1
        }
#endif // NUM_EXTRUDER>0

    } // setDirection

    static inline void enable() {
#if NUM_EXTRUDER == 1
#if EXT0_ENABLE_PIN > -1
        WRITE(EXT0_ENABLE_PIN, EXT0_ENABLE_ON);
#endif // EXT0_ENABLE_PIN>-1
#else
        if (Extruder::current->enablePin > -1)
            HAL::digitalWrite(Extruder::current->enablePin, Extruder::current->enableOn);

#endif // NUM_EXTRUDER==1

        if (!Extruder::current->enabled) {
            Extruder::current->enabled = 1;
            HAL::delayMilliseconds(stepperEnableDelayMs);
        }

    } // enable
    static bool allHeatersCalibrated();
    static void manageTemperatures();
    static bool hasActiveHotendTarget();
    static void disableAllExtruders();
    static bool selectExtruderById(uint8_t extruderId, bool waitThroughPause = false);
    static void applyCurrentSettings();
    static void initExtruder();
    static void initHeatedBed();
    static void setHeatedBedTemperature(float temperatureInCelsius);
    static float getHeatedBedTemperature();
    static void setTemperatureForExtruder(float temperatureInCelsius, uint8_t extr);
    static void setTemperatureForAllExtruders(float temperatureInCelsius);
}; // Extruder

#define NUM_TEMPERATURE_LOOPS (NUM_EXTRUDER + 1)
extern TemperatureController heatedBedController;

#ifdef X35_ADC_CHANNEL
extern TemperatureController optTempController;
#endif // X35_ADC_CHANNEL

#define TEMP_INT_TO_FLOAT(temp) ((float)(temp) / (float)(1 << CELSIUS_EXTRA_BITS))
#define TEMP_FLOAT_TO_INT(temp) ((int)((temp) * (1 << CELSIUS_EXTRA_BITS)))

extern TemperatureController* tempController[NUM_TEMPERATURE_LOOPS];
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
namespace ThermalEmergency {
    const Ledger<NUM_TEMPERATURE_LOOPS>& faultLedger();
    // Foreground only, after safe GPIO initialization.
    void latchSystemFault(Cause cause);
    bool liveTemperatureValid(uint8_t channel);
    void requestLiveReport();
    void requestDiagnosticReplay();
}
#endif

#endif // EXTRUDER_H

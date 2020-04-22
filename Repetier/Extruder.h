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

#define CELSIUS_EXTRA_BITS 3

// Updates the temperature of all extruders and heated bed if it's time.
// Toggels the heater power if necessary.
extern void reportTempsensorAndHeaterErrors(); ///< Report defect sensors
extern uint8_t manageMonitor;

#define TEMPERATURE_CONTROLLER_FLAG_ALARM 1
#define TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL 2  ///< Full heating enabled
#define TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD 4  ///< Holding target temperature
#define TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT 8     ///< Indicating sensor defect
#define TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED 16 ///< Indicating sensor decoupling
/** TemperatureController manages one heater-temperature sensore loop. You can have up to
4 loops allowing pid for up to 3 extruder and the heated bed.

*/
class TemperatureController {
public:
    uint8_t pwmIndex;           ///< pwm index for output control. 0-2 = Extruder, 3 = Fan, 4 = Heated Bed
    uint8_t sensorType;         ///< Type of temperature sensor.
    uint8_t sensorPin;          ///< Pin to read extruder temperature.
    int16_t currentTemperature; ///< Currenttemperature value read from sensor.
    float currentTemperatureC;  ///< Current temperature in degC.
    float targetTemperatureC;   ///< Target temperature in degC.

#if FEATURE_HEAT_BED_TEMP_COMPENSATION
    float offsetC; ///< offset in degC.
#endif             // FEATURE_HEAT_BED_TEMP_COMPENSATION

    uint32_t lastTemperatureUpdate; ///< Time in millis of the last temperature update.

    float tempIState;    ///< Temp. var. for PID computation.
    uint8_t pidDriveMax; ///< Used for windup in PID calculation.
    uint8_t pidDriveMin; ///< Used for windup in PID calculation.
    float pidPGain;      ///< Pgain (proportional gain) for PID temperature control [0,01 Units].
    float pidIGain;      ///< Igain (integral) for PID temperature control [0,01 Units].
    float pidDGain;      ///< Dgain (damping) for PID temperature control [0,01 Units].
    uint8_t pidMax;      ///< Maximum PWM value, the heater should be set.
    float tempIStateLimitMax;
    float tempIStateLimitMin;
    uint8_t tempPointer;
    float tempArray[16];

    uint8_t flags;

    millis_t lastDecoupleTest;   ///< Last time of decoupling sensor-heater test
    float lastDecoupleTemp;      ///< Temperature on last test
    millis_t decoupleTestPeriod; ///< Time between setting and testing decoupling.

    uint8_t paused;

    void setTargetTemperature(float target, float offset);
    void updateCurrentTemperature();
    void updateTempControlVars();
    void waitForTargetTemperature(uint8_t plus_temp_tolerance = 0);
    void autotunePID(float temp, uint8_t controllerId, int maxCycles, bool storeResult, int method);

    inline bool isAlarm() {
        return flags & TEMPERATURE_CONTROLLER_FLAG_ALARM;
    }
    inline void setAlarm(bool on) {
        if (on)
            flags |= TEMPERATURE_CONTROLLER_FLAG_ALARM;
        else
            flags &= ~TEMPERATURE_CONTROLLER_FLAG_ALARM;
    }

    //Code für den Decouple-Test der Heizkreisläufe:
    inline bool isDecoupleFull() {
        return flags & TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL;
    }
    inline void setDecoupleFull() {
        flags &= ~(TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD); //hold aus
        flags |= TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL;    //full an
    }
    inline bool isDecoupleHold() {
        return flags & TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD;
    }
    inline void setDecoupleHold() {
        flags &= ~(TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL); //full aus
        flags |= TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD;    //hold an
    }
    inline void stopDecouple() {
        flags &= ~(TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL | TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD);
    }

    //Phase 1: wir stehen auf einer Stufe und merken uns die Zeit und die Temperatur. Das wird später mit der neuen Temperatur verglichen und dann muss die Temperatur gestiegen sein.
    inline void startFullDecoupleTest(millis_t& t) {
        if (isDecoupleFull())
            return;
        lastDecoupleTest = t;
        lastDecoupleTemp = currentTemperatureC;
        //switch active test method to "temperature has to rise":
        setDecoupleFull();
    }

    //Phase 2: Wir sind voll aufgeheizt und prüfen ob die Temperatur ohne Änderung der Zieltemperatur abdriftet.
    //         Schlägt an, wenn die Heizkartusche kaputt geht oder rausrutscht. Oder auch wenn der Sensorwert willkürlich wandert.
    //         Ist der Sensor ausserhalb der bekannten MIN-MAX Limits wird das auch erkannt.
    //
    //         ???:
    //         Der Regler steuert, wenn der Fehler anfängt, soweit es geht noch nach. Driftet der Sensorwert wegen schleichendem Sensordefekt !langsam! nach unten, heizt der PID-Regler um so mehr.
    //         Solange der PID-Regler den Drift kompensieren kann und das System glaubt, die Temperaturen sind im Rahmen von MIN-MAX könnte der Heizblock heißer sein als gewollt!
    //         Allerdings sollte der Thermistor > 300°C schnell das Zeitliche segnen völligen Mist anzeigen und dann greifen die Checks wieder.
    inline void startHoldDecoupleTest(millis_t& t) {
        //nur starten, wenn es nicht schon läuft.
        if (isDecoupleHold())
            return;
        //nicht starten, wenn die temperatur "noch" ausserhalb der decoupling varianz liegt. kann z.b. sein, dass die pid-control-range 30 grad ist und das hier 20.
        if (fabs(currentTemperatureC - targetTemperatureC) + 1 > DECOUPLING_TEST_MAX_HOLD_VARIANCE)
            return;
        //beim ersten start setzt man den timer auf "jetzt". Es dauert also bis zum ersten check nicht 1 sekunde sondern so wie die config eingestellt ist.
        //erst danach macht der code jede sekunde den "Wackelcheck".
        lastDecoupleTest = t;
        lastDecoupleTemp = targetTemperatureC;
        //switch active test method to "temperature has to stay where it was":
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
class Extruder // Size: 12*1 Byte+12*4 Byte+4*2Byte = 68 Byte
{
public:
    static Extruder* current;

#if FEATURE_DITTO_PRINTING
    static uint8_t dittoMode;
#endif // FEATURE_DITTO_PRINTING

    uint8_t id;
    float offsetMM[3];
    float stepsPerMM; ///< Steps per mm.
    int8_t enablePin; ///< Pin to enable extruder stepper motor.
    uint8_t enableOn;
    float maxFeedrate;              ///< Maximum feedrate in mm/s.
    float maxAcceleration;          ///< Maximum acceleration in mm/s^2.
    float maxEJerk;                 ///< Maximum start feedrate in mm/s.
    int16_t waitRetractTemperature; ///< Temperature to retract the filament when waiting for heatup
    int16_t waitRetractUnits;       ///< Units to retract the filament when waiting for heatup
    volatile int8_t stepperDirection;

#if USE_ADVANCE
    float advanceL;
#endif // USE_ADVANCE

    TemperatureController tempControl;
    const char* PROGMEM selectCommands;
    const char* PROGMEM deselectCommands;
    uint8_t coolerSpeed; ///< Speed to use when enabled
    uint8_t coolerPWM;   ///< current PWM setting

#if STEPPER_ON_DELAY
    char enabled;
#endif // STEPPER_ON_DELAY

    /** \brief Sends the high-signal to the stepper for next extruder step.
    Call this function only, if interrupts are disabled.
    */
    static INLINE void step() {
#if NUM_EXTRUDER == 1
        WRITE(EXT0_STEP_PIN, HIGH);
#else
        switch (Extruder::current->id) {
        case 0: {
#if NUM_EXTRUDER > 0
            WRITE(EXT0_STEP_PIN, HIGH);

#if FEATURE_DITTO_PRINTING
            if (Extruder::dittoMode) {
                WRITE(EXT1_STEP_PIN, HIGH);
            }
#endif // FEATURE_DITTO_PRINTING
#endif // NUM_EXTRUDER>0
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
#if NUM_EXTRUDER > 0
            WRITE(EXT0_STEP_PIN, LOW);

#if FEATURE_DITTO_PRINTING
            if (Extruder::dittoMode) {
                WRITE(EXT1_STEP_PIN, LOW);
            }
#endif // FEATURE_DITTO_PRINTING
#endif // NUM_EXTRUDER>0
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
#if NUM_EXTRUDER > 0
        case 0: {
            if (dir)
                WRITE(EXT0_DIR_PIN, !EXT0_INVERSE);
            else
                WRITE(EXT0_DIR_PIN, EXT0_INVERSE);

#if FEATURE_DITTO_PRINTING
            if (Extruder::dittoMode) {
                if (dir)
                    WRITE(EXT1_DIR_PIN, !EXT1_INVERSE);
                else
                    WRITE(EXT1_DIR_PIN, EXT1_INVERSE);
            }
#endif // FEATURE_DITTO_PRINTING
            break;
        }
#endif // NUM_EXTRUDER>0

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

#if FEATURE_DITTO_PRINTING
        if (Extruder::dittoMode) {
            if (extruder[1].enablePin > -1)
                HAL::digitalWrite(extruder[1].enablePin, extruder[1].enableOn);
        }
#endif // FEATURE_DITTO_PRINTING
#endif // NUM_EXTRUDER==1

#if STEPPER_ON_DELAY
        if (!Extruder::current->enabled) {
            Extruder::current->enabled = 1;
            HAL::delayMilliseconds(STEPPER_ON_DELAY);
        }
#endif // STEPPER_ON_DELAY

    } // enable
    static void manageTemperatures();
    static void disableAllExtruders();
    static void selectExtruderById(uint8_t extruderId);
    static void initExtruder();
    static void initHeatedBed();
    static void setHeatedBedTemperature(float temperatureInCelsius, bool beep = false);
    static float getHeatedBedTemperature();
    static void setTemperatureForExtruder(float temperatureInCelsius, uint8_t extr, bool beep = false);
    static void setTemperatureForAllExtruders(float temperatureInCelsius, bool beep);
}; // Extruder

#if HAVE_HEATED_BED
#define NUM_TEMPERATURE_LOOPS NUM_EXTRUDER + 1
extern TemperatureController heatedBedController;
#else
#define NUM_TEMPERATURE_LOOPS NUM_EXTRUDER
#endif // HAVE_HEATED_BED

#if RESERVE_ANALOG_INPUTS
extern TemperatureController optTempController;
#endif // RESERVE_ANALOG_INPUTS

#define TEMP_INT_TO_FLOAT(temp) ((float)(temp) / (float)(1 << CELSIUS_EXTRA_BITS))
#define TEMP_FLOAT_TO_INT(temp) ((int)((temp) * (1 << CELSIUS_EXTRA_BITS)))

extern TemperatureController* tempController[NUM_TEMPERATURE_LOOPS];
extern uint8_t autotuneIndex;

#endif // EXTRUDER_H

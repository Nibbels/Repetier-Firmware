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
#include "GCodeInput.h"
#include "UiMenus.h"

const long baudrates[] PROGMEM = { 9600, 14400, 19200, 28800, 38400, 56000, 57600, 76800, 111112, 115200, 128000, 230400, 250000, 256000,
                                   460800, 500000, 921600, 1000000, 1500000, 0 };

namespace {
constexpr double modMenuSenseLimit = EMERGENCY_PAUSE_DIGITS_MAX * 0.8;
constexpr int16_t modMenuSenseMax = modMenuSenseLimit < 32767 ? int16_t(modMenuSenseLimit) : 32767;

void adjustModMenuSenseOffset(int16_t digits) {
    Printer::senseOffsetControl.setThresholdDigits(digits);
    // The Mod Menu remembers ordinary adjustments from 500 digits upwards.
    if (digits >= 500 && HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS) != digits) {
        HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS, digits);
        EEPROM::updateChecksum();
    }
}
}

void UIDisplay::senseOffsetUpAction() {
    auto& controller = Printer::senseOffsetControl;
    const int16_t digits = controller.limits().thresholdDigits;
    if (digits == modMenuSenseLimit || digits == 32767) {
        controller.setThresholdDigits(0);
    } else if (digits > modMenuSenseLimit - 250 || digits > 32767 - 250) {
        controller.setThresholdDigits(modMenuSenseMax);
    } else {
        const int16_t step = digits >= 2000 ? 250 : digits >= 500 ? 100 : 50;
        adjustModMenuSenseOffset(digits + step);
    }
}

void UIDisplay::senseOffsetDownAction() {
    auto& controller = Printer::senseOffsetControl;
    const int16_t digits = controller.limits().thresholdDigits;
    if (digits == 0) {
        controller.setThresholdDigits(modMenuSenseMax);
    } else if (digits < 50) {
        controller.setThresholdDigits(0);
    } else {
        const int16_t step = digits >= 2000 ? 250 : digits >= 500 ? 100 : 50;
        adjustModMenuSenseOffset(digits - step);
    }
}

void UIDisplay::zOffsetPlusAction() {
    Printer::zOffset.adjustWithBedButton(Z_OFFSET_BUTTON_STEPS, maxManualZOffsetMicrometers, Printer::stepsPerMm(Z_AXIS));
    EEPROM::storeManualZOffset();
}

void UIDisplay::zOffsetMinusAction() {
    Printer::zOffset.adjustWithBedButton(-Z_OFFSET_BUTTON_STEPS, maxManualZOffsetMicrometers, Printer::stepsPerMm(Z_AXIS));
    EEPROM::storeManualZOffset();
}

void UIDisplay::finishAction(int action) {
    switch (action) {
    case UI_ACTION_RF_RESET_ACK: {
        if (confirmation_ != 1) {
            // continue only in case the user has chosen "Yes"
            break;
        }
        Com::printFLN(PSTR("Reset via Menu"));
        HAL::delayMilliseconds(100);
        Commands::emergencyStop();
        break;
    }

    case UI_ACTION_STOP_ACK: {
        if (confirmation_ != 1) {
            // continue only in case the user has chosen "Yes"
            break;
        }
        exitmenu();
        Printer::stopPrint();
        break;
    }

    case UI_ACTION_RESTORE_DEFAULTS: {
        if (confirmation_ != 1 || !Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Cancel)) {
            // continue only in case the user has chosen "Yes"
            break;
        }
        EEPROM::restoreEEPROMSettingsFromConfiguration();
        EEPROM::storeDataIntoEEPROM(false);

        exitmenu();
        UI_STATUS(UI_TEXT_RESTORE_DEFAULTS);
        break;
    }

    case UI_ACTION_CHOOSE_LESSERINTEGRAL:
    case UI_ACTION_CHOOSE_CLASSICPID:
    case UI_ACTION_CHOOSE_NO:
    case UI_ACTION_CHOOSE_TYREUS_LYBEN: {
        if (confirmation_ != 1) {
            // continue only in case the user has chosen "Yes"
            break;
        }
        unsigned char heater = menuPos[menuLevel - 2]; // The bed follows the hotends in the PID menu.
        int method = menuPos[menuLevel - 1];           //0..1..2..3 passt nicht mehr zum J-Listing des M303

        //Esthetics: Switch method for better order in menu. I want the menu to start with pessen integral, because it is the most responsive one. We dont need some overshoot within menu because it has no known use for us.
        /*
                                             responsiveness of tunings: J1pessen++ J0classic+ J2someovershoot J3noovershoot- J4tyreuslyben--
                                             -- is for slow beds
                                             ++ is for fast hotends
                                             */
        if (method == 0) {
            method = 1; //J1
        } else if (method == 1) {
            method = 0; //J0
        } else if (method == 2) {
            method = 3; //J3
        } else if (method == 3) {
            method = 4; //J4
        }

        int temperature = heatBedScanPlaNozzleC;
        int cycles = 10;
        if (NUM_EXTRUDER == heater) {
            //Das ist das Heizbett und kein Extruder
            temperature = heatBedScanPlaBedC; //Bett nicht so hoch testen, wie Extruder.
            cycles = 16;                                      //Bett ist erfahrungsgemäß viel träger. Mehr Zyklen erhöhen die Genauigkeit des Ergebnis.
        }
        bool writeeeprom = true;
        if (heater >= NUM_TEMPERATURE_LOOPS)
            heater = NUM_TEMPERATURE_LOOPS - 1;
        //show menu and message to user: He cant do anything until autotune is over.
        exitmenu();
        menuPos[0] = 3; //show temps
        tempController[heater]->autotunePID(temperature, heater, cycles, writeeeprom, method);
        break;
    }
    }
} // finishAction

//increment ist ne variable, global -> Knopfrichtung.
//#define INCREMENT_MIN_MAX(a,steps,_min,_max) if ( (increment<0) && (_min>=0) && (a<_min-increment*steps) ) {a=_min;} else { a+=increment*steps; if(a<_min) a=_min; else if(a>_max) a=_max;};
// this version not have single byte variable rollover bug
#define INCREMENT_MIN_MAX(a, steps, _min, _max) a = constrain((a + increment * steps), _min, _max);

#define INCREMENT_MAX(a, steps, _max) \
    if (increment < 0 && a <= -1 * increment * steps) \
        a = 0; \
    else { \
        a += increment * steps; \
        if (a > _max) \
            a = _max; \
    };
//#define INCREMENT_MAX(a,steps,_max) a = constrain((a + increment * steps), 0, _max); //Nibbels: ist für unsigned variablen gegen warning. 2x vorkommen

void UIDisplay::changeSwitchCase(int action, int8_t increment) {
    switch (action) {
    case UI_ACTION_PREHEAT_A_BED:
    case UI_ACTION_PREHEAT_A_HOTEND:
    case UI_ACTION_PREHEAT_B_BED:
    case UI_ACTION_PREHEAT_B_HOTEND: {
        using Profile = PreheatProfiles::Profile;
        using Heater = PreheatProfiles::Heater;
        const bool profileA = action == UI_ACTION_PREHEAT_A_BED || action == UI_ACTION_PREHEAT_A_HOTEND;
        const bool bed = action == UI_ACTION_PREHEAT_A_BED || action == UI_ACTION_PREHEAT_B_BED;
        if (Printer::preheatProfiles.adjust(profileA ? Profile::A : Profile::B,
                                           bed ? Heater::Bed : Heater::Hotend, increment))
            EEPROM::storePreheatProfiles();
        break;
    }
    case UI_ACTION_FANSPEED: {
        int speed = (int)Printer::getFanSpeed() + (int)increment;
        if (speed > 255)
            speed = 0;
        if (speed < 0)
            speed = 255;
        Commands::setFanSpeed((uint8_t)speed);
        break;
    }
    case UI_ACTION_FAN_HZ: {
        if (increment > 0) {
            Commands::adjustFanFrequency((HAL::partFanDivisor() == PART_FAN_MODE_MAX ? 1 : HAL::partFanDivisor() + 1));
        } else {
            Commands::adjustFanFrequency((HAL::partFanDivisor() == 1 ? PART_FAN_MODE_MAX : HAL::partFanDivisor() - 1));
        }
        HAL::eprSetByte(EPR_RF_PART_FAN_SPEED, HAL::partFanDivisor());
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_FAN_PART_FAN_PWM_MIN: {
        int temp = HAL::partFanMinDuty();
        INCREMENT_MIN_MAX(temp, 1, 1, 239);
        if (temp <= (int)HAL::partFanMaxDuty() - 16) {
            HAL::setPartFanDutyRange(temp, HAL::partFanMaxDuty());
        }
        //recalculate active pwm value out of HAL::partFanSpeed() for easy tune-in.
        //(Tune-In: set fan to 1% and rise minimum until it starts.)
        Commands::setFanSpeed(HAL::partFanSpeed(), true);
        HAL::eprSetByte(EPR_RF_PART_FAN_PWM_MIN, HAL::partFanMinDuty());
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_FAN_PART_FAN_PWM_MAX: {
        int temp = HAL::partFanMaxDuty();
        INCREMENT_MIN_MAX(temp, 1, 16, 255);
        if (temp >= HAL::partFanMinDuty() + 16) {
            HAL::setPartFanDutyRange(HAL::partFanMinDuty(), temp);
        }
        //recalculate active pwm value out of HAL::partFanSpeed() for easy tune-in.
        //(Tune-In: set fan to 100% and decrease maximum until fan slows down slightly.)
        Commands::setFanSpeed(HAL::partFanSpeed(), true);
        HAL::eprSetByte(EPR_RF_PART_FAN_PWM_MAX, HAL::partFanMaxDuty());
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_XPOSITION: {
        /*
        XYZ_POSITION_BUTTON_DIRECTION = -1 : This fits to you if you want more intuitivity when choosing the Up-Down-Buttons.
        XYZ_POSITION_BUTTON_DIRECTION = 1 : This fits more if you stick to coordinates direction.
        see configuration.h
        */
        moveXAction(XYZ_POSITION_BUTTON_DIRECTION * increment);
        break;
    }
    case UI_ACTION_YPOSITION: {
        /*
        XYZ_POSITION_BUTTON_DIRECTION = -1 : This fits to you if you want more intuitivity when choosing the Up-Down-Buttons.
        XYZ_POSITION_BUTTON_DIRECTION = 1 : This fits more if you stick to coordinates direction.
        see configuration.h
        */
        moveYAction(XYZ_POSITION_BUTTON_DIRECTION * increment);
        break;
    }
    case UI_ACTION_ZPOSITION: {
        /*
        XYZ_POSITION_BUTTON_DIRECTION = -1 : This fits to you if you want more intuitivity when choosing the Up-Down-Buttons.
        XYZ_POSITION_BUTTON_DIRECTION = 1 : This fits more if you stick to coordinates direction.
        see configuration.h
        */
        moveZAction(XYZ_POSITION_BUTTON_DIRECTION * increment);
        break;
    }
    case UI_ACTION_ZOFFSET: {
        int32_t manual = Printer::zOffset.manualMicrometers();
        INCREMENT_MIN_MAX(manual, Z_OFFSET_MENU_STEPS, -maxManualZOffsetMicrometers, maxManualZOffsetMicrometers);
        Printer::zOffset.setManualMicrometers(manual, Printer::stepsPerMm(Z_AXIS));
        EEPROM::storeManualZOffset();
        break;
    }
    case UI_ACTION_EPOSITION: {
        Printer::queueRelativeMMCoordinates(0, 0, 0, (float)increment / Printer::menuExtrusionFactor, UI_SET_EXTRUDER_FEEDRATE, true); //klappt nicht in Pause!!
        Commands::printCurrentPosition();
        break;
    }
    case UI_ACTION_HEATED_BED_TEMP: {
        int tmp = (int)heatedBedController.targetTemperatureC;
        if (tmp < UI_SET_MIN_HEATED_BED_TEMP)
            tmp = 0;
        tmp += increment;
        if (tmp == 1)
            tmp = UI_SET_MIN_HEATED_BED_TEMP;
        if (tmp < UI_SET_MIN_HEATED_BED_TEMP)
            tmp = 0;
        else if (tmp > UI_SET_MAX_HEATED_BED_TEMP)
            tmp = UI_SET_MAX_HEATED_BED_TEMP;
        Extruder::setHeatedBedTemperature(tmp);
        break;
    }
    case UI_ACTION_EXTRUDER0_TEMP: {
        int tmp = (int)extruder[0].tempControl.targetTemperatureC;
        if (tmp < UI_SET_MIN_EXTRUDER_TEMP)
            tmp = 0;
        tmp += increment;
        if (tmp == 1)
            tmp = UI_SET_MIN_EXTRUDER_TEMP;
        if (tmp < UI_SET_MIN_EXTRUDER_TEMP)
            tmp = 0;
        else if (tmp > UI_SET_MAX_EXTRUDER_TEMP)
            tmp = UI_SET_MAX_EXTRUDER_TEMP;
        Extruder::setTemperatureForExtruder(tmp, 0);
        break;
    }
    case UI_ACTION_EXTRUDER1_TEMP: {
#if NUM_EXTRUDER > 1
        int tmp = (int)extruder[1].tempControl.targetTemperatureC;
        tmp += increment;
        if (tmp == 1)
            tmp = UI_SET_MIN_EXTRUDER_TEMP;
        if (tmp < UI_SET_MIN_EXTRUDER_TEMP)
            tmp = 0;
        else if (tmp > UI_SET_MAX_EXTRUDER_TEMP)
            tmp = UI_SET_MAX_EXTRUDER_TEMP;
        Extruder::setTemperatureForExtruder(tmp, 1);
#endif // NUM_EXTRUDER>1
        break;
    }

#if NUM_EXTRUDER > 1
    case UI_ACTION_EXTRUDER_OFFSET_X: {
        float oldXOffset = extruder[1].offsetMM[X_AXIS];
        INCREMENT_MIN_MAX(extruder[1].offsetMM[X_AXIS], 0.25, 32, 36);

        if (Printer::isAxisHomed(X_AXIS) && Extruder::current->id == extruder[1].id) {
            // Shift the extruder-offset negatively to stay at the same point after switch
            int32_t dx = (extruder[1].offsetMM[X_AXIS] - oldXOffset) * Printer::stepsPerMm(X_AXIS);
            Printer::offsetRelativeStepsCoordinates(-dx, 0, 0, 0);
        }

        HAL::eprSetFloat(EEPROM::getExtruderOffset(1) + EPR_EXTRUDER_X_OFFSET, extruder[1].offsetMM[X_AXIS]);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_EXTRUDER_OFFSET_Y: {
        float oldYOffset = extruder[1].offsetMM[Y_AXIS];
        INCREMENT_MIN_MAX(extruder[1].offsetMM[Y_AXIS], 0.25, -2, 2);

        if (Printer::isAxisHomed(Y_AXIS) && Extruder::current->id == extruder[1].id) {
            int32_t dy = (extruder[1].offsetMM[Y_AXIS] - oldYOffset) * Printer::stepsPerMm(Y_AXIS);
            // Shift the extruder-offset negatively to stay at the same point after switch
            Printer::offsetRelativeStepsCoordinates(0, -dy, 0, 0);
        }

        HAL::eprSetFloat(EEPROM::getExtruderOffset(1) + EPR_EXTRUDER_Y_OFFSET, extruder[1].offsetMM[Y_AXIS]);
        EEPROM::updateChecksum();
        break;
    }
#endif // NUM_EXTRUDER>1

    case UI_ACTION_FEEDRATE_MULTIPLY: {
        int fr = Printer::feedrateMultiply;
        INCREMENT_MIN_MAX(fr, 1, 25, 500);
        Commands::changeFeedrateMultiply(fr);
        break;
    }
    case UI_ACTION_FLOWRATE_MULTIPLY: {
        float eF = Printer::menuExtrusionFactor;
        INCREMENT_MIN_MAX(eF, 0.01f, 0.25f, 2.0f);
        Commands::changeFlowrateMultiply(eF);
        break;
    }
    case UI_ACTION_STEPPER_INACTIVE: {
        millis_t duration = Printer::inactivity.stepperTimeout();
        duration -= duration % 1000;
        INCREMENT_MAX(duration, 60000UL, 10080000UL);
        Printer::inactivity.setStepperTimeout(duration);
        HAL::eprSetInt32(EPR_STEPPER_INACTIVE_TIME, duration);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_MAX_INACTIVE: {
        millis_t duration = Printer::inactivity.shutdownTimeout();
        duration -= duration % 1000;
        INCREMENT_MAX(duration, 60000UL, 10080000UL);
        Printer::inactivity.setShutdownTimeout(duration);
        HAL::eprSetInt32(EPR_MAX_INACTIVE_TIME, duration);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_PRINT_ACCEL_X: {
        float value = Printer::printAcceleration(X_AXIS);
        INCREMENT_MIN_MAX(value, ACCELERATION_MENU_CHANGE_XY, ACCELERATION_MIN_XY, ACCELERATION_MAX_XY);
        Printer::setPrintAcceleration(X_AXIS, value);
        Printer::updateDerivedParameter();

        HAL::eprSetFloat(EPR_X_MAX_ACCEL, Printer::printAcceleration(X_AXIS));
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_PRINT_ACCEL_Y: {
        float value = Printer::printAcceleration(Y_AXIS);
        INCREMENT_MIN_MAX(value, ACCELERATION_MENU_CHANGE_XY, ACCELERATION_MIN_XY, ACCELERATION_MAX_XY);
        Printer::setPrintAcceleration(Y_AXIS, value);
        Printer::updateDerivedParameter();

        HAL::eprSetFloat(EPR_Y_MAX_ACCEL, Printer::printAcceleration(Y_AXIS));
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_PRINT_ACCEL_Z: {
        float value = Printer::printAcceleration(Z_AXIS);
        INCREMENT_MIN_MAX(value, ACCELERATION_MENU_CHANGE_Z, ACCELERATION_MIN_Z, ACCELERATION_MAX_Z);
        Printer::setPrintAcceleration(Z_AXIS, value);
        Printer::updateDerivedParameter();

        HAL::eprSetFloat(EPR_Z_MAX_ACCEL, Printer::printAcceleration(Z_AXIS));
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_MOVE_ACCEL_X: {
        float value = Printer::travelAcceleration(X_AXIS);
        INCREMENT_MIN_MAX(value, ACCELERATION_MENU_CHANGE_XY, ACCELERATION_MIN_XY, ACCELERATION_MAX_XY);
        Printer::setTravelAcceleration(X_AXIS, value);
        Printer::updateDerivedParameter();

        HAL::eprSetFloat(EPR_X_MAX_TRAVEL_ACCEL, Printer::travelAcceleration(X_AXIS));
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_MOVE_ACCEL_Y: {
        float value = Printer::travelAcceleration(Y_AXIS);
        INCREMENT_MIN_MAX(value, ACCELERATION_MENU_CHANGE_XY, ACCELERATION_MIN_XY, ACCELERATION_MAX_XY);
        Printer::setTravelAcceleration(Y_AXIS, value);
        Printer::updateDerivedParameter();

        HAL::eprSetFloat(EPR_Y_MAX_TRAVEL_ACCEL, Printer::travelAcceleration(Y_AXIS));
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_MOVE_ACCEL_Z: {
        float value = Printer::travelAcceleration(Z_AXIS);
        INCREMENT_MIN_MAX(value, ACCELERATION_MENU_CHANGE_Z, ACCELERATION_MIN_Z, ACCELERATION_MAX_Z);
        Printer::setTravelAcceleration(Z_AXIS, value);
        Printer::updateDerivedParameter();

        HAL::eprSetFloat(EPR_Z_MAX_TRAVEL_ACCEL, Printer::travelAcceleration(Z_AXIS));
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_MAX_JERK: {
        float value = Printer::xyJerkLimit();
        INCREMENT_MIN_MAX(value, 0.1f, 1.0f, 33.3f);
        Printer::setXYJerkLimit(value); //RFx000: Limit 33.3 sind willkürlich grob faktor 3 von normalwert.

        HAL::eprSetFloat(EPR_MAX_XYJERK, Printer::xyJerkLimit());
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_MAX_ZJERK: {
        float value = Printer::zJerkLimit();
        INCREMENT_MIN_MAX(value, 0.05f, 0.05f, 2.0f);
        Printer::setZJerkLimit(value); //RFx000: Limit 2 sind Max XY-Jerk / Stepratenverhältnis XY->Z von ~16.8 gibt etwas unter 2.

        HAL::eprSetFloat(EPR_MAX_ZJERK, Printer::zJerkLimit());
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_HOMING_FEEDRATE_X: {
        float value = Printer::axisHomingFeedrate(X_AXIS);
        INCREMENT_MIN_MAX(value, 5, 5, MAX_FEEDRATE_X);
        Printer::setAxisHomingFeedrate(X_AXIS, value);

        HAL::eprSetFloat(EPR_X_HOMING_FEEDRATE_PRINT, Printer::axisHomingFeedrate(X_AXIS));
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_HOMING_FEEDRATE_Y: {
        float value = Printer::axisHomingFeedrate(Y_AXIS);
        INCREMENT_MIN_MAX(value, 5, 5, MAX_FEEDRATE_Y);
        Printer::setAxisHomingFeedrate(Y_AXIS, value);

        HAL::eprSetFloat(EPR_Y_HOMING_FEEDRATE_PRINT, Printer::axisHomingFeedrate(Y_AXIS));
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_HOMING_FEEDRATE_Z: {
        float value = Printer::axisHomingFeedrate(Z_AXIS);
        INCREMENT_MIN_MAX(value, 1, 1, MAX_FEEDRATE_Z);
        Printer::setAxisHomingFeedrate(Z_AXIS, value);

        HAL::eprSetFloat(EPR_Z_HOMING_FEEDRATE_PRINT, Printer::axisHomingFeedrate(Z_AXIS));
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_MAX_FEEDRATE_X: {
        float value = Printer::axisMaxFeedrate(X_AXIS);
        INCREMENT_MIN_MAX(value, 5, 1, MAX_FEEDRATE_X);
        Printer::setAxisMaxFeedrate(X_AXIS, value);

        HAL::eprSetFloat(EPR_X_MAX_FEEDRATE, Printer::axisMaxFeedrate(X_AXIS));
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_MAX_FEEDRATE_Y: {
        float value = Printer::axisMaxFeedrate(Y_AXIS);
        INCREMENT_MIN_MAX(value, 5, 1, MAX_FEEDRATE_Y);
        Printer::setAxisMaxFeedrate(Y_AXIS, value);

        HAL::eprSetFloat(EPR_Y_MAX_FEEDRATE, Printer::axisMaxFeedrate(Y_AXIS));
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_MAX_FEEDRATE_Z: {
        float value = Printer::axisMaxFeedrate(Z_AXIS);
        INCREMENT_MIN_MAX(value, 1, 1, MAX_FEEDRATE_Z);
        Printer::setAxisMaxFeedrate(Z_AXIS, value);

        HAL::eprSetFloat(EPR_Z_MAX_FEEDRATE, Printer::axisMaxFeedrate(Z_AXIS));
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_BAUDRATE: {
        unsigned char p = 0;
        int32_t rate;
        do {
            rate = pgm_read_dword(&(baudrates[p]));
            if (rate == baudrate)
                break;
            p++;

        } while (rate != 0);

        if (rate == 0 && p >= 2)
            p -= 2;
        else if (rate == 0 && p == 1)
            p = 0;

        if (p + increment >= 0)
            p += increment;
        else if (p + increment <= 0)
            p = 0;

        //if(p<0) p = 0;
        rate = pgm_read_dword(&(baudrates[p]));
        if (rate == 0 && p >= 1)
            p--;
        baudrate = pgm_read_dword(&(baudrates[p]));

        HAL::eprSetInt32(EPR_BAUDRATE, baudrate);
        EEPROM::updateChecksum();
        break;
    }

#ifdef RGB_LIGHT_R_PIN
    case UI_ACTION_RGB_LIGHT_MODE: {
        char mode = RgbLighting::mode();
        INCREMENT_MIN_MAX(mode, 1, 0, 3);
        RgbLighting::selectMode(mode);
        break;
    }
#endif // RGB_LIGHT_R_PIN

    case UI_ACTION_EXTR_STEPS_E0:
    case UI_ACTION_EXTR_STEPS_E1: {
        if (!Printer::isMenuMode(MENU_MODE_PAUSED) && !Printer::printJob.isBusy()) {
#if NUM_EXTRUDER > 1
            uint8_t eNr = (UI_ACTION_EXTR_STEPS_E1 == action) ? 1 : 0;
#else
            uint8_t eNr = 0;
#endif //NUM_EXTRUDER > 1
            INCREMENT_MIN_MAX(extruder[eNr].stepsPerMM, 1, 1, 9999);
            if (eNr == Extruder::current->id)
                Extruder::applyCurrentSettings();

            HAL::eprSetFloat(EEPROM::getExtruderOffset(eNr) + EPR_EXTRUDER_STEPS_PER_MM, extruder[eNr].stepsPerMM);
            EEPROM::updateChecksum();
        }
        break;
    }

    case UI_ACTION_ADVANCE_L_E0:
    case UI_ACTION_ADVANCE_L_E1: {
        if (!Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Cancel))
            break;
#if NUM_EXTRUDER > 1
        uint8_t eNr = (UI_ACTION_ADVANCE_L_E1 == action) ? 1 : 0;
#else
        uint8_t eNr = 0;
#endif //NUM_EXTRUDER > 1
        float step = (extruder[eNr].advanceL < 30.0f) ? 1.0f : ((extruder[eNr].advanceL < 50.0f) ? 5.0f : 10.0f);
        INCREMENT_MIN_MAX(extruder[eNr].advanceL, step, 0.0f, 250.0f);
        // advance < 20 might be problematic
        // https://github.com/repetier/Repetier-Firmware/issues/837#issuecomment-455852008
        if (extruder[eNr].advanceL < 20.0f) {
            if (increment > 0) {
                extruder[eNr].advanceL = 20;
            } else {
                extruder[eNr].advanceL = 0;
            }
        }
        Printer::applyAdvanceSettings();

        HAL::eprSetFloat(EEPROM::getExtruderOffset(eNr) + EPR_EXTRUDER_ADVANCE_L, extruder[eNr].advanceL);
        EEPROM::updateChecksum();
        break;
    }

    case UI_ACTION_EXTR_ACCELERATION_E0:
    case UI_ACTION_EXTR_ACCELERATION_E1: {
#if NUM_EXTRUDER > 1
        uint8_t eNr = (UI_ACTION_EXTR_ACCELERATION_E1 == action) ? 1 : 0;
#else
        uint8_t eNr = 0;
#endif //NUM_EXTRUDER > 1
        INCREMENT_MIN_MAX(extruder[eNr].maxAcceleration, 200, 200, 10000);
        if (eNr == Extruder::current->id)
            Extruder::applyCurrentSettings();

        HAL::eprSetFloat(EEPROM::getExtruderOffset(eNr) + EPR_EXTRUDER_MAX_ACCELERATION, extruder[eNr].maxAcceleration);
        EEPROM::updateChecksum();
        break;
    }

    case UI_ACTION_EXTR_MAX_FEEDRATE_E0:
    case UI_ACTION_EXTR_MAX_FEEDRATE_E1: {
#if NUM_EXTRUDER > 1
        uint8_t eNr = (UI_ACTION_EXTR_MAX_FEEDRATE_E1 == action) ? 1 : 0;
#else
        uint8_t eNr = 0;
#endif //NUM_EXTRUDER > 1
        INCREMENT_MIN_MAX(extruder[eNr].maxFeedrate, 1, extruder[eNr].maxEJerk, 60);
        if (eNr == Extruder::current->id)
            Extruder::applyCurrentSettings();

        HAL::eprSetFloat(EEPROM::getExtruderOffset(eNr) + EPR_EXTRUDER_MAX_FEEDRATE, extruder[eNr].maxFeedrate);
        EEPROM::updateChecksum();
        break;
    }

    case UI_ACTION_EXTR_START_FEEDRATE_E0:
    case UI_ACTION_EXTR_START_FEEDRATE_E1: {
#if NUM_EXTRUDER > 1
        uint8_t eNr = (UI_ACTION_EXTR_START_FEEDRATE_E1 == action) ? 1 : 0;
#else
        uint8_t eNr = 0;
#endif //NUM_EXTRUDER > 1
        INCREMENT_MIN_MAX(extruder[eNr].maxEJerk, 1, 1, extruder[eNr].maxFeedrate);
        if (eNr == Extruder::current->id)
            Extruder::applyCurrentSettings();

        HAL::eprSetFloat(EEPROM::getExtruderOffset(eNr) + EPR_EXTRUDER_MAX_START_FEEDRATE, extruder[eNr].maxEJerk);
        EEPROM::updateChecksum();
        break;
    }

    case UI_ACTION_RF_SET_Z_MATRIX_HEAT_BED: {
        if (Printer::doHeatBedZCompensation) {
            // do not allow to change the current heat bed z-compensation matrix while the z-compensation is active
            showError((void*)ui_text_z_compensation, (void*)ui_text_operation_denied);
            break;
        }

        char nextHeatBed = activeBedMatrixSlot();
        INCREMENT_MIN_MAX(nextHeatBed, 1, 1, EEPROM_MAX_HEAT_BED_SECTORS);
        switchActiveHeatBed(nextHeatBed);
        break;
    }

    case UI_ACTION_RF_SCAN_START_HEIGHT: {
        float liftMm = scanStartLiftMm();
        INCREMENT_MIN_MAX(liftMm, 0.1f, 0.3f, 6.0f);
        setScanStartLiftMm(liftMm);
        HAL::eprSetFloat(EPR_ZSCAN_START_MM, scanStartLiftMm()); //mm zlift vor den scans.
        EEPROM::updateChecksum();
        break;
    }

    case UI_ACTION_RF_RESET_ACK:
    case UI_ACTION_STOP_ACK:
    case UI_ACTION_RESTORE_DEFAULTS:
    case UI_ACTION_CHOOSE_LESSERINTEGRAL:
    case UI_ACTION_CHOOSE_CLASSICPID:
    case UI_ACTION_CHOOSE_NO:
    case UI_ACTION_CHOOSE_TYREUS_LYBEN: {
        INCREMENT_MIN_MAX(confirmation_, 1, 0, 1);
        break;
    }

    case UI_ACTION_EMERGENCY_PAUSE_MIN: {
        auto limits = Printer::pressureProtection.pauseLimits();
        long minimum = limits.minimum;
        INCREMENT_MIN_MAX(minimum, 200, EMERGENCY_PAUSE_DIGITS_MIN, limits.maximum);
        limits.minimum = minimum;
        Printer::pressureProtection.setPauseLimits(limits);
        HAL::eprSetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMIN, minimum);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_EMERGENCY_PAUSE_MAX: {
        auto limits = Printer::pressureProtection.pauseLimits();
        long maximum = limits.maximum;
        INCREMENT_MIN_MAX(maximum, 200, limits.minimum, EMERGENCY_PAUSE_DIGITS_MAX);
        limits.maximum = maximum;
        Printer::pressureProtection.setPauseLimits(limits);
        HAL::eprSetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMAX, maximum);
        EEPROM::updateChecksum();
        break;
    }

    case UI_ACTION_EMERGENCY_ZSTOP_MIN: {
        auto limits = Printer::pressureProtection.stopLimits();
        INCREMENT_MIN_MAX(limits.minimum, 200, EMERGENCY_STOP_DIGITS_MIN, limits.maximum);
        Printer::pressureProtection.setStopLimits(limits);
        HAL::eprSetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMIN, limits.minimum);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_EMERGENCY_ZSTOP_MAX: {
        auto limits = Printer::pressureProtection.stopLimits();
        INCREMENT_MIN_MAX(limits.maximum, 200, limits.minimum, EMERGENCY_STOP_DIGITS_MAX);
        Printer::pressureProtection.setStopLimits(limits);
        HAL::eprSetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMAX, limits.maximum);
        EEPROM::updateChecksum();
        break;
    }

    case UI_ACTION_SENSEOFFSET_DIGITS: {
        short oldval = HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS);
        INCREMENT_MIN_MAX(oldval, 100, 500, EMERGENCY_PAUSE_DIGITS_MAX);
        HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS, oldval);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_SENSEOFFSET_MAX: {
        short oldval = HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX);
        INCREMENT_MIN_MAX(oldval, 10, 10, 300);
        HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX, oldval);
        EEPROM::updateChecksum();
        break;
    }

    case UI_ACTION_WOBBLE_FIX_PHASEXY: {
        auto phase = Printer::wobbleCorrection.profile().phase;
        INCREMENT_MIN_MAX(phase, 1, -100, 101);
        if (phase == 101) {
            phase = -99;
        }
        if (phase == -100) {
            phase = 100;
        }
        Printer::wobbleCorrection.setPhase(phase);
        HAL::eprSetByte(EPR_RF_MOD_WOBBLE_FIX_PHASEXY, phase);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_WOBBLE_FIX_AMPX: {
        auto amplitude = Printer::wobbleCorrection.profile().xMicrometers;
        INCREMENT_MIN_MAX(amplitude, 5, -995, 995);
        Printer::wobbleCorrection.setXAmplitude(amplitude);
        HAL::eprSetInt16(EPR_RF_MOD_WOBBLE_FIX_AMPX, amplitude);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_WOBBLE_FIX_AMPY1: {
        auto amplitude = Printer::wobbleCorrection.profile().yAtXMinMicrometers;
        INCREMENT_MIN_MAX(amplitude, 5, -995, 995);
        Printer::wobbleCorrection.setYMinAmplitude(amplitude);
        HAL::eprSetInt16(EPR_RF_MOD_WOBBLE_FIX_AMPY1, amplitude);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_WOBBLE_FIX_AMPY2: {
        auto amplitude = Printer::wobbleCorrection.profile().yAtXMaxMicrometers;
        INCREMENT_MIN_MAX(amplitude, 5, -995, 995);
        Printer::wobbleCorrection.setYMaxAmplitude(amplitude);
        HAL::eprSetInt16(EPR_RF_MOD_WOBBLE_FIX_AMPY2, amplitude);
        EEPROM::updateChecksum();
        break;
    }

    case UI_ACTION_CHOOSE_DMIN: {
        if (menuLevel == 4) {                        //identifikation des temperaturzyklus anhand der position im menü. Das ist nicht 100% sauber, aber funktioniert.
            uint8_t heater = menuPos[menuLevel - 1]; // The bed follows the hotends in the PID menu.
            if (heater < NUM_TEMPERATURE_LOOPS) {
                int drive = tempController[heater]->pidDriveMin;
                INCREMENT_MIN_MAX(drive, 1, 1, 255);
                tempController[heater]->pidDriveMin = drive;
                if (NUM_EXTRUDER == heater) {
                    //Das ist das Heizbett
                    HAL::eprSetByte(EPR_BED_DRIVE_MIN, (uint8_t)drive);
                    EEPROM::updateChecksum();
                } else {
                    //Extruder
                    if (heater <= NUM_EXTRUDER - 1) { //paranoid doublecheck
                        HAL::eprSetByte(EEPROM::getExtruderOffset(heater) + EPR_EXTRUDER_DRIVE_MIN, (uint8_t)drive);
                        EEPROM::updateChecksum();
                    }
                }
            }
        }
        break;
    }
    case UI_ACTION_CHOOSE_DMAX: {
        if (menuLevel == 4) {                        //identifikation des temperaturzyklus anhand der position im menü. Das ist nicht 100% sauber, aber funktioniert.
            uint8_t heater = menuPos[menuLevel - 1]; // The bed follows the hotends in the PID menu.
            if (heater < NUM_TEMPERATURE_LOOPS) {
                int drive = tempController[heater]->pidDriveMax;
                INCREMENT_MIN_MAX(drive, 1, 1, 255);
                tempController[heater]->pidDriveMax = drive;
                if (NUM_EXTRUDER == heater) {
                    //Das ist das Heizbett
                    HAL::eprSetByte(EPR_BED_DRIVE_MAX, (uint8_t)drive);
                    EEPROM::updateChecksum();
                } else {
                    //Extruder
                    if (heater <= NUM_EXTRUDER - 1) { //paranoid doublecheck
                        HAL::eprSetByte(EEPROM::getExtruderOffset(heater) + EPR_EXTRUDER_DRIVE_MAX, (uint8_t)drive);
                        EEPROM::updateChecksum();
                    }
                }
            }
        }
        break;
    }
    case UI_ACTION_CHOOSE_PIDMAX: {
        if (menuLevel == 4) {                        //identifikation des temperaturzyklus anhand der position im menü. Das ist nicht 100% sauber, aber funktioniert.
            uint8_t heater = menuPos[menuLevel - 1]; // The bed follows the hotends in the PID menu.
            if (heater < NUM_TEMPERATURE_LOOPS) {
                int drive = tempController[heater]->pidMax;
                INCREMENT_MIN_MAX(drive, 1, 1, 255);
                tempController[heater]->pidMax = drive;
                if (NUM_EXTRUDER == heater) {
                    //Das ist das Heizbett
                    HAL::eprSetByte(EPR_BED_PID_MAX, (uint8_t)drive);
                    EEPROM::updateChecksum();
                } else {
                    //Extruder
                    if (heater <= NUM_EXTRUDER - 1) { //paranoid doublecheck
                        HAL::eprSetByte(EEPROM::getExtruderOffset(heater) + EPR_EXTRUDER_PID_MAX, (uint8_t)drive);
                        EEPROM::updateChecksum();
                    }
                }
            }
        }
        break;
    }
    case UI_ACTION_CHOOSE_SENSOR: {
        if (menuLevel == 4) {                        //identifikation des temperaturzyklus anhand der position im menü. Das ist nicht 100% sauber, aber funktioniert.
            uint8_t heater = menuPos[menuLevel - 1]; // The bed follows the hotends in the PID menu.
            if (heater < NUM_TEMPERATURE_LOOPS) {
                int drive = tempController[heater]->sensorType;
                if (NUM_EXTRUDER == heater) {
                    //Das ist das Heizbett
                    switch (drive) {
                    case 3: {
                        drive = 4;
                        break;
                    } //4 ist der 10K Thermistor vom hersteller der matten (? rf1k_mhj11)
                    default: {
                        drive = 3;
                        break;
                    }
                    }
                    tempController[heater]->sensorType = drive;
                    HAL::eprSetByte(EPR_RF_HEATED_BED_SENSOR_TYPE, (uint8_t)drive);
                    EEPROM::updateChecksum();
                } else {
                    //Extruder
                    if (increment > 0) {
                        switch (drive) {
                        case 3: {
                            drive = 8;
                            break;
                        }
                        case 8: {
                            drive = 13;
                            break;
                        } //add more sensors for menu-tweaking here, those are the most common for RFx000
                        case 13: {
                            drive = 1;
                            break;
                        }
                        default: {
                            drive = 3;
                            break;
                        }
                        }
                    } else { //== 0 gibts nicht, soweit ich weiß
                        switch (drive) {
                        case 3: {
                            drive = 1;
                            break;
                        }
                        case 1: {
                            drive = 13;
                            break;
                        }
                        case 13: {
                            drive = 8;
                            break;
                        } //add more sensors for menu-tweaking here, those are the most common for RFx000
                        default: {
                            drive = 3;
                            break;
                        }
                        }
                    }
                    tempController[heater]->sensorType = drive;
                    if (heater <= NUM_EXTRUDER - 1) { //paranoid doublecheck
                        HAL::eprSetByte(EEPROM::getExtruderOffset(heater) + EPR_EXTRUDER_SENSOR_TYPE, (uint8_t)drive);
                        EEPROM::updateChecksum();
                    }
                }
            }
        }
        break;
    }
    case UI_ACTION_CHOOSE_MOTOR_X:
    case UI_ACTION_CHOOSE_MOTOR_Y:
    case UI_ACTION_CHOOSE_MOTOR_Z:
    case UI_ACTION_CHOOSE_MOTOR_E0:
    case UI_ACTION_CHOOSE_MOTOR_E1: {
        uint8_t steppernr = menuPos[menuLevel];
        if (steppernr == 6)
            steppernr = 4;   //das ist etwas stümperhaft, aber ich brauche die Zeilennummer um auszuwählen und die Einstellungen für die Extruderstepper gehören drüber, also muss Extruder 2 zwei Zeilen runter...
        if (steppernr < 5) { // aktuell gibts nur 5
            int drive = Printer::motorCurrent[steppernr];
            const short uMotorCurrentMax[] = MOTOR_CURRENT_MAX;
            INCREMENT_MIN_MAX(drive, 1, MOTOR_CURRENT_MIN + 1, uMotorCurrentMax[steppernr]); //von 40 bis maximal das was in der config steht.
            Printer::motorCurrent[steppernr] = drive;
            HAL::eprSetByte(EPR_RF_MOTOR_CURRENT + steppernr, (uint8_t)drive);
            EEPROM::updateChecksum();
            setMotorCurrent(steppernr + 1, drive);
            Com::printF(PSTR("Stepper"), steppernr + 1);
            Com::printFLN(PSTR(" = "), drive);
        }
        break;
    }
    case UI_ACTION_FLOW_MIN: {
        auto profile = Printer::digitCompensation.flowProfile();
        INCREMENT_MIN_MAX(profile.minDigits, 200, 0, profile.maxDigits);
        Printer::digitCompensation.setFlowProfile(profile);
        break;
    }
    case UI_ACTION_FLOW_MAX: {
        const int16_t pauseMax = Printer::pressureProtection.pauseLimits().maximum;
        const int16_t max = pauseMax ? abs(pauseMax) : abs(EMERGENCY_PAUSE_DIGITS_MAX);
        auto profile = Printer::digitCompensation.flowProfile();
        INCREMENT_MIN_MAX(profile.maxDigits, 200, profile.minDigits, max);
        Printer::digitCompensation.setFlowProfile(profile);
        break;
    }
    case UI_ACTION_FLOW_DF: {
        auto profile = Printer::digitCompensation.flowProfile();
        INCREMENT_MIN_MAX(profile.extrusionPercent, 1, -99, 99);
        Printer::digitCompensation.setFlowProfile(profile);
        break;
    }
    case UI_ACTION_FLOW_DV: {
        auto profile = Printer::digitCompensation.flowProfile();
        INCREMENT_MIN_MAX(profile.speedPercent, 1, -90, 99);
        Printer::digitCompensation.setFlowProfile(profile);
        break;
    }
    case UI_ACTION_SHIFT_INTERVAL: {
        INCREMENT_MIN_MAX(Printer::stepsPackingMinInterval, -100, MIN_STEP_PACKING_MIN_INTERVAL, MAX_STEP_PACKING_MIN_INTERVAL);
        HAL::eprSetInt16(EPR_RF_STEP_PACKING_MIN_INTERVAL, Printer::stepsPackingMinInterval);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_MICROSTEPS_XY:
    case UI_ACTION_MICROSTEPS_Z:
    case UI_ACTION_MICROSTEPS_E: {
        if (!Printer::printJob.isBusy() && !PrintLine::hasLines() && Printer::pause.allowsQueueAdvance()) {
            Printer::disableAllSteppersNow(); //Stepper und Homing ausmachen.
                                              //We cannot use the old coordinates anymore.

            InterruptProtectedBlock noInts;
            bool changed[5] = { false, false, false, false, false };
            switch (action) {
            case UI_ACTION_MICROSTEPS_XY: {
                INCREMENT_MIN_MAX(Printer::motorMicroStepsModeValue[0], 1, 4, 6);
                if (Printer::motorMicroStepsModeValue[1] != Printer::motorMicroStepsModeValue[0]) { //only adjust, when changed.
                    Printer::motorMicroStepsModeValue[1] = Printer::motorMicroStepsModeValue[0];    //sync x and y
                    drv8711adjustMicroSteps(1);                                                     //adjust driver chip X=1
                    drv8711adjustMicroSteps(2);                                                     //adjust driver chip Y=2
                    changed[0] = changed[1] = true;
                }
                break;
            }
            case UI_ACTION_MICROSTEPS_Z: {
                uint8_t temp = Printer::motorMicroStepsModeValue[2];
                INCREMENT_MIN_MAX(Printer::motorMicroStepsModeValue[2], 1, 4, 5);
                if (temp != Printer::motorMicroStepsModeValue[2]) { //only adjust, when changed.
                    drv8711adjustMicroSteps(3);                     //adjust driver chip Z=3
                    changed[2] = true;
                }
                break;
            }
            case UI_ACTION_MICROSTEPS_E: {
                INCREMENT_MIN_MAX(Printer::motorMicroStepsModeValue[3], 1, 4, 7);
                if (Printer::motorMicroStepsModeValue[4] != Printer::motorMicroStepsModeValue[3]) { //only adjust, when changed.
                    Printer::motorMicroStepsModeValue[4] = Printer::motorMicroStepsModeValue[3];    //sync E0 and E1
                    drv8711adjustMicroSteps(4);                                                     //adjust driver chip E0=4
                    drv8711adjustMicroSteps(5);                                                     //adjust driver chip E1=5
                    changed[3] = changed[4] = true;
                }
                break;
            }
            }

            bool updateall = false;
            if (HAL::eprGetByte(EPR_RF_MICRO_STEPS_USED) != 0xAB)
                updateall = true;
            float stepsmm_korrekturfactor = (increment > 0 ? 2.0f : 0.5f);
            bool updatederived = false;
            bool updateextruder = false;

            //anpassen der eeprom-werte und anpassen der steps/mm sodass die geschwindigkeit weiterhin passt.
            for (int i = 0; i < DRV8711_NUM_CHANNELS; i++) {
                if ((!changed[i] && updateall) || changed[i]) { //erstes oder veränderndes schreiben
                    HAL::eprSetByte(EPR_RF_MICRO_STEPS_X + i, Printer::motorMicroStepsModeValue[i]);
                }
                if (changed[i]) { //nur dann die axis-steps anpassen, wenn wirklich was geändert wurde.
                    switch (i) {
                    case X_AXIS:
                    case Y_AXIS:
                    case Z_AXIS: {
                        Printer::setStepsPerMm(i, Printer::stepsPerMm(i) * stepsmm_korrekturfactor);
                        Printer::pause.scaleParkSteps(i, stepsmm_korrekturfactor);
                        if (i == Z_AXIS) {
                            Printer::zMotion.rescaleMotorReference(stepsmm_korrekturfactor);
                            Printer::layers.rescaleFadeBounds(stepsmm_korrekturfactor);
                            Printer::manualMotion.rescaleSteps(Z_AXIS, stepsmm_korrekturfactor);
                            HAL::eprSetInt16(EPR_RF_MOD_Z_STEP_SIZE, Printer::manualMotion.steps(Z_AXIS));
                            invalidateBedMatrixForReload();
                                                                             //korrektur der aktiven kompensation/zoffset ist durch fehlendes homing unterbunden.
                        }
                        updatederived = true; //übernehmen der werte in offsets und infaxissteps, accel usw..
                        HAL::eprSetFloat(EPR_XAXIS_STEPS_PER_MM + 4 * i, Printer::stepsPerMm(i));
                        break;
                    }
                    case E_AXIS:
                    case E_AXIS + 1: {
                        //i-3 ist hier 0 oder 1
                        uint8_t etr = i - 3; //3-3 =0 oder 4-1 =1
                        extruder[etr].stepsPerMM *= stepsmm_korrekturfactor;
                        HAL::eprSetFloat(EEPROM::getExtruderOffset(etr) + EPR_EXTRUDER_STEPS_PER_MM, extruder[etr].stepsPerMM);
                        if (etr == Extruder::current->id)
                            updateextruder = true; //übernehmen der werte in offsets und infaxissteps, accel usw..
                        break;
                    }
                    }
                }
            }
            //übernehmen der werte in offsets und infaxissteps, accel usw..
            if (updatederived)
                Printer::updateDerivedParameter();
            if (updateextruder)
                Extruder::applyCurrentSettings();
            if (updateall)
                HAL::eprSetByte(EPR_RF_MICRO_STEPS_USED, 0xAB); //erstes schreiben markiert eepromwerte als gültig
            noInts.unprotect();
            if (changed[Z_AXIS])
                Printer::zOffset.refreshSteps(Printer::stepsPerMm(Z_AXIS));
            EEPROM::updateChecksum();
        }
        break;
    }
    }
} //changeSwitchCase

void UIDisplay::pressPlay() {
    static uint8_t idlePlayCount = 0;
    if (Printer::pause.isPaused()) {
        if (Printer::pause.resume())
            idlePlayCount = 0;
        return;
    }

    if (!heatBedScanRunning() && ++idlePlayCount >= 10) {
        idlePlayCount = 0;
        Com::printFLN(PSTR("LCD re-init"));
        showInformation(PSTR(UI_TEXT_MANUAL), PSTR(UI_TEXT_LCD_CIRCUIT), PSTR(UI_TEXT_RESET));
        initializeLCD();
    }
}

void UIDisplay::mainSwitchCase(int action) {
    switch (action) {
        /*
        * HARDWARE BUTTONS
        */
    case UI_ACTION_OK: {
        okAction();
        confirmation_ = 0;
        break;
    }
    case UI_ACTION_BACK: {
        backAction();
        break;
    }
    case UI_ACTION_NEXT: {
        nextPreviousAction(1);
        break;
    }
    case UI_ACTION_RIGHT: {
        rightAction();
        break;
    }
    case UI_ACTION_PREVIOUS: {
        nextPreviousAction(-1);
        break;
    }
    case UI_ACTION_RF_HEAT_BED_UP: {
        // show that we are active
        Printer::inactivity.recordActivity(HAL::timeInMilliseconds());

        //DO NOT MOVE Z: ALTER Z-OFFSET
        if (uid.menuLevel == 0 && uid.menuPos[0] == 1) {
            //wenn im Mod-Menü für Z-Offset/Matrix Sense-Offset/Limiter, dann anders!
            zOffsetMinusAction();
        } //ELSE DO MOVE Z:
        else {
            moveZAction(-1);
        }
        break;
    }
    case UI_ACTION_RF_HEAT_BED_DOWN: {
        // show that we are active
        Printer::inactivity.recordActivity(HAL::timeInMilliseconds());
                                                         //DO NOT MOVE Z: ALTER Z-OFFSET
        if (uid.menuLevel == 0 && uid.menuPos[0] == 1) { //wenn im Mod-Menü für Z-Offset/Matrix Z-Offset/Matrix Sense-Offset/Limiter, dann anders!
            zOffsetPlusAction();
        } //ELSE DO MOVE Z:
        else {
            moveZAction(1);
        }
        break;
    }
    case UI_ACTION_RF_EXTRUDER_OUTPUT:
    case UI_ACTION_RF_EXTRUDER_RETRACT: {
        Printer::inactivity.recordActivity(HAL::timeInMilliseconds());
        const int8_t direction = action == UI_ACTION_RF_EXTRUDER_OUTPUT ? 1 : -1;
        if (uid.menuLevel == 0 && uid.menuPos[0] == 1)
            Commands::changeFeedrateMultiply(Printer::feedrateMultiply + direction);
        else
            Printer::pause.extrudeManually(int32_t(Printer::manualMotion.steps(E_AXIS)) * direction);
        break;
    }
    case UI_ACTION_RF_PAUSE: {
        Printer::pause.pressPause();
        break;
    }
    case UI_ACTION_RF_CONTINUE: {
        pressPlay();
        break;
    }

    /*
    * FUNCTIONS
    */
    case UI_ACTION_EMERGENCY_STOP: {
        Commands::emergencyStop();
        break;
    }
    case UI_ACTION_HOME_ALL: {
        if (PrintLine::hasLines()) {
            // do not allow homing via the menu while we are printing
            Com::printFLN(Com::tPrintingIsInProcessError);
            showError((void*)ui_text_home, (void*)ui_text_operation_denied);
            break;
        }
        if (!isHomingAllowed(NULL, 1)) {
            break;
        }
        exitmenu();
        Printer::homeAxis(true, true, true);
        break;
    }
    case UI_ACTION_HOME_X: {
        if (PrintLine::hasLines()) {
            // do not allow homing via the menu while we are printing
            if (Printer::debugErrors()) {
                Com::printFLN(Com::tPrintingIsInProcessError);
            }

            showError((void*)ui_text_home, (void*)ui_text_operation_denied);
            break;
        }
        if (!isHomingAllowed(NULL, 1)) {
            break;
        }
        exitmenu();
        Printer::homeAxis(true, false, false);
        break;
    }
    case UI_ACTION_HOME_Y: {
        if (PrintLine::hasLines()) {
            // do not allow homing via the menu while we are printing
            if (Printer::debugErrors()) {
                Com::printFLN(Com::tPrintingIsInProcessError);
            }

            showError((void*)ui_text_home, (void*)ui_text_operation_denied);
            break;
        }
        if (!isHomingAllowed(NULL, 1)) {
            break;
        }
        exitmenu();
        Printer::homeAxis(false, true, false);
        break;
    }
    case UI_ACTION_HOME_Z: {
        if (PrintLine::hasLines()) {
            // do not allow homing via the menu while we are printing
            if (Printer::debugErrors()) {
                Com::printFLN(Com::tPrintingIsInProcessError);
            }
            showError((void*)ui_text_home, (void*)ui_text_operation_denied);
            break;
        }
        if (!isHomingAllowed(NULL, 1)) {
            break;
        }
        exitmenu();
        Printer::homeAxis(false, false, true);
        break;
    }
    case UI_ACTION_DEBUG_ECHO: {
        if (Printer::debugEcho())
            Printer::debugLevel -= 1;
        else
            Printer::debugLevel += 1;
        break;
    }
    case UI_ACTION_DEBUG_INFO: {
        if (Printer::debugInfo())
            Printer::debugLevel -= 2;
        else
            Printer::debugLevel += 2;
        break;
    }
    case UI_ACTION_DEBUG_ERROR: {
        if (Printer::debugErrors())
            Printer::debugLevel -= 4;
        else
            Printer::debugLevel += 4;
        break;
    }
    case UI_ACTION_DEBUG_DRYRUN: {
        if (Printer::debugDryrun())
            Printer::debugLevel -= 8;
        else
            Printer::debugLevel += 8;
        if (Printer::debugDryrun()) // simulate movements without printing
        {
            Extruder::setTemperatureForAllExtruders(0);
            Extruder::setHeatedBedTemperature(0);
        }
        break;
    }

#if !HOTEND_COOLER_USES_PIN(CASE_LIGHT_PIN)
    case UI_ACTION_LIGHTS_ONOFF: {
        if (Printer::enableCaseLight)
            Printer::enableCaseLight = 0;
        else
            Printer::enableCaseLight = 1;
        WRITE(CASE_LIGHT_PIN, Printer::enableCaseLight);

        HAL::eprSetByte(EPR_RF_CASE_LIGHT_MODE, Printer::enableCaseLight);
        EEPROM::updateChecksum();

        break;
    }
#endif

#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    case UI_ACTION_230V_OUTPUT: {
        if (Printer::enable230VOutput)
            Printer::enable230VOutput = 0;
        else
            Printer::enable230VOutput = 1;
        WRITE(OUTPUT_230V_PIN, Printer::enable230VOutput);

        // after a power-on, the 230 V plug always shall be turned off - thus, we do not store this setting to the EEPROM
        // HAL::eprSetByte( EPR_RF_230V_OUTPUT_MODE, Printer::enable230VOutput );
        // EEPROM::updateChecksum();

        break;
    }

#if FET1 > -1 && !HOTEND_COOLER_USES_PIN(FET1)
    case UI_ACTION_FET1_OUTPUT: {
        if (Printer::enableFET1)
            Printer::enableFET1 = 0;
        else
            Printer::enableFET1 = 1;
        WRITE(FET1, Printer::enableFET1);

        HAL::eprSetByte(EPR_RF_FET1_MODE, Printer::enableFET1);
        EEPROM::updateChecksum();
        break;
    }
#endif // FET1 accessory output
#if FET2 > -1 && !HOTEND_COOLER_USES_PIN(FET2)
    case UI_ACTION_FET2_OUTPUT: {
        if (Printer::enableFET2)
            Printer::enableFET2 = 0;
        else
            Printer::enableFET2 = 1;
        WRITE(FET2, Printer::enableFET2);

        HAL::eprSetByte(EPR_RF_FET2_MODE, Printer::enableFET2);
        EEPROM::updateChecksum();
        break;
    }
#endif // FET2 accessory output
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2

    case UI_ACTION_CONFIG_SINGLE_STEPS: {
        configureMANUAL_STEPS_Z(1);
        break;
    }

    case UI_ACTION_CONFIG_SINGLE_STEPS_KOSYS: {
        Printer::manualMotion.setCoordinateSystem(!Printer::manualMotion.coordinateSystem());

        HAL::eprSetByte(EPR_RF_MOVE_MODE_XY_KOSYS, Printer::manualMotion.coordinateSystem());
        EEPROM::updateChecksum();
        break;
    }

    case UI_ACTION_CONFIG_POSITION_FEEDRATE: {
        Printer::manualMotion.setFeedrateSource(!Printer::manualMotion.feedrateSource());

        HAL::eprSetByte(EPR_RF_MOVE_POSITION_FEEDRATE, Printer::manualMotion.feedrateSource());
        EEPROM::updateChecksum();
        break;
    }

    case UI_ACTION_ZMODE: {
        if (Printer::ZMode == 1)
            Printer::ZMode = 2;
        else
            Printer::ZMode = 1;

        HAL::eprSetByte(EPR_RF_Z_MODE, Printer::ZMode);
        EEPROM::updateChecksum();
        break;
    }


    case UI_ACTION_PREHEAT_A:
    case UI_ACTION_PREHEAT_B: {
        const bool profileA = action == UI_ACTION_PREHEAT_A;
        const auto temperatures = Printer::preheatProfiles.get(profileA ? PreheatProfiles::Profile::A : PreheatProfiles::Profile::B);
        Printer::idleStatus.schedule(HAL::timeInMilliseconds() + 10000);
        if (profileA) {
            UI_STATUS_UPD(UI_TEXT_PREHEAT_A);
        } else {
            UI_STATUS_UPD(UI_TEXT_PREHEAT_B);
        }
        Extruder::setTemperatureForAllExtruders(temperatures.hotend);
        Extruder::setHeatedBedTemperature(temperatures.bed);
        break;
    }
    case UI_ACTION_COOLDOWN: {
        UI_STATUS_UPD(UI_TEXT_COOLDOWN);
        Extruder::setTemperatureForAllExtruders(0);
        Extruder::setHeatedBedTemperature(0);
        break;
    }
    case UI_ACTION_HEATED_BED_OFF: {
        Extruder::setHeatedBedTemperature(0);
        break;
    }
    case UI_ACTION_EXTRUDER0_OFF: {
        Extruder::setTemperatureForExtruder(0, 0);
        break;
    }
    case UI_ACTION_EXTRUDER1_OFF: {
#if NUM_EXTRUDER > 1
        Extruder::setTemperatureForExtruder(0, 1);
#endif // NUM_EXTRUDER>1
        break;
    }
    case UI_ACTION_DISABLE_STEPPER: {
        Printer::disableAllSteppersNow();
        break;
    }
    case UI_ACTION_MOUNT_FILAMENT_SOFT:
    case UI_ACTION_MOUNT_FILAMENT_HARD:
    case UI_ACTION_UNMOUNT_FILAMENT_SOFT:
    case UI_ACTION_UNMOUNT_FILAMENT_HARD: {
        Printer::idleStatus.cancel();
        while (Printer::checkAbortKeys())
            Commands::checkForPeriodicalActions(); //dont quit script by holding the ok longer than 1ms if no temp is involved. -> min einmal OK loslassen.
        bool unmount = (action == UI_ACTION_UNMOUNT_FILAMENT_SOFT || action == UI_ACTION_UNMOUNT_FILAMENT_HARD);
        exitmenu();

        if (unmount) {
            UI_STATUS_UPD(UI_TEXT_UNMOUNT_FILAMENT);
            if (action == UI_ACTION_UNMOUNT_FILAMENT_SOFT) {
                GCodeInput::executeFlashScript(Com::tUnmountFilamentSoft);
            } else {
                GCodeInput::executeFlashScript(Com::tUnmountFilamentHard);
            }
        } else {
            UI_STATUS_UPD(UI_TEXT_MOUNT_FILAMENT);
            if (action == UI_ACTION_MOUNT_FILAMENT_SOFT) {
                /* Diese PLA Temp ist bei ~180 °C, die meisten Filamente werden gerade so weich. Wenn man keine mindesetens "weiche" Temperatur eingestellt hat, soll geheizt werden.*/
                if (Extruder::current->tempControl.targetTemperatureC < UI_SET_EXTRUDER_TEMP_MOUNT) {
                    Extruder::setTemperatureForExtruder(UI_SET_EXTRUDER_TEMP_MOUNT, Extruder::current->id);
                    Extruder::current->tempControl.waitForTargetTemperature();
                    GCodeInput::executeFlashScript(Com::tMountFilamentSoft);
                    Extruder::setTemperatureForExtruder(0, Extruder::current->id);
                } else {
                    GCodeInput::executeFlashScript(Com::tMountFilamentSoft);
                }
            } else {
                GCodeInput::executeFlashScript(Com::tMountFilamentHard);
            }
        }
        Printer::idleStatus.schedule(HAL::timeInMilliseconds());
        break;
    }
    case UI_ACTION_SET_E_ORIGIN: {
        Printer::setEPositionMM(0); // G92 E0
        break;
    }
    case UI_ACTION_EXTRUDER_RELATIVE: {
        Printer::gcodeModes.setRelativeEOverride(!Printer::gcodeModes.relativeEOverride());
        break;
    }
    case UI_ACTION_SELECT_EXTRUDER0: {
        Extruder::selectExtruderById(0);
        break;
    }
    case UI_ACTION_SELECT_EXTRUDER1: {
#if NUM_EXTRUDER > 1
        Extruder::selectExtruderById(1);
#endif // NUM_EXTRUDER>1
        break;
    }
    case UI_ACTION_ACTIVE_EXTRUDER: {
        if (Extruder::current->id == 0)
            Extruder::selectExtruderById(1);
        else
            Extruder::selectExtruderById(0);
        break;
    }


    case UI_ACTION_FAN_OFF: {
        Commands::setFanSpeed((uint8_t)0);
        break;
    }
    case UI_ACTION_FAN_25: {
        Commands::setFanSpeed((uint8_t)64);
        break;
    }
    case UI_ACTION_FAN_50: {
        Commands::setFanSpeed((uint8_t)128);
        break;
    }
    case UI_ACTION_FAN_75: {
        Commands::setFanSpeed((uint8_t)192);
        break;
    }
    case UI_ACTION_FAN_FULL: {
        Commands::setFanSpeed((uint8_t)255);
        break;
    }
    case UI_ACTION_FAN_MODE: {
        Commands::adjustFanMode((HAL::partFanMode() ? PART_FAN_MODE_PWM : PART_FAN_MODE_PDM)); //0 = pwm, 1 = pdm
        HAL::eprSetByte(EPR_RF_FAN_MODE, HAL::partFanMode());
        EEPROM::updateChecksum();
        break;
    }

    case UI_ACTION_MENU_XPOS: {
        pushMenu((void*)&ui_menu_xpos, false);
        break;
    }
    case UI_ACTION_MENU_YPOS: {
        pushMenu((void*)&ui_menu_ypos, false);
        break;
    }
    case UI_ACTION_MENU_ZPOS: {
        pushMenu((void*)&ui_menu_zpos, false);
        break;
    }
    case UI_ACTION_MENU_QUICKSETTINGS: {
        pushMenu((void*)&ui_menu_quick, false);
        break;
    }
    case UI_ACTION_MENU_EXTRUDER: {
        pushMenu((void*)&ui_menu_extruder, false);
        break;
    }
    case UI_ACTION_MENU_POSITIONS: {
        pushMenu((void*)&ui_menu_positions, false);
        break;
    }

    case UI_ACTION_DIGIT_HOMING: {
        const bool enabled = !Printer::strainGauge.zeroingEnabled();
        Printer::strainGauge.setZeroingEnabled(enabled);
        HAL::eprSetByte(EPR_RF_ZERO_DIGIT_STATE, enabled ? 1 : 2);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_DIGIT_COMPENSATION: {
        const bool enabled = !Printer::digitCompensation.digitZEnabled();
        Printer::digitCompensation.setDigitZEnabled(enabled);
        HAL::eprSetByte(EPR_RF_DIGIT_CMP_STATE, enabled ? 1 : 2);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_SENSEOFFSET_AUTOSTART: {
        const bool enabled = !Printer::senseOffsetControl.autoStart();
        Printer::senseOffsetControl.setAutoStart(enabled);
        HAL::eprSetByte(EPR_RF_MOD_SENSEOFFSET_AUTOSTART, enabled);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_RF_DO_MHIER_BED_SCAN: {
        //macht an, wenn an, macht aus:
        startZOScan();
        //gehe zurück und zeige dem User was passiert.
        exitmenu();
        break;
    }
    case UI_ACTION_RF_DO_MHIER_AUTO_MATRIX_LEVELING: {
        //macht an, wenn an, macht aus:
        startZOScan(true); //Scan aber an vielen Punkten und Gewichtet.
                           //gehe zurück und zeige dem User was passiert.
        exitmenu();
        break;
    }
    case UI_ACTION_RF_DO_SAVE_ACTIVE_ZMATRIX: {
        // save the determined values to the EEPROM
        if (bedMatrixHasUnsavedChanges()) {
            exitmenu();
            if (!saveActiveBedMatrix()) {
                showError(ui_text_manual, PSTR("Matrix save failed"));
                break;
            }
            if (Printer::debugInfo()) {
                Com::printFLN(PSTR("Manual Input: the heat bed compensation matrix has been saved"));
            }
            showInformation((void*)ui_text_manual, (void*)ui_text_saving_success);
        } else {
            showInformation((void*)ui_text_manual, (void*)ui_text_saving_needless);
        }
        break;
    }

    case UI_ACTION_RF_SCAN_HEAT_BED: {
        startHeatBedScan(HeatBedScanMode::Standard);
        //gehe zurück und zeige dem User was passiert.
        uid.exitmenu();
        break;
    }
    case UI_ACTION_RF_SCAN_HEAT_BED_PLA: {
        startHeatBedScan(HeatBedScanMode::Pla);
        //gehe zurück und zeige dem User was passiert.
        uid.exitmenu();
        break;
    }
    case UI_ACTION_RF_SCAN_HEAT_BED_ABS: {
        startHeatBedScan(HeatBedScanMode::Abs);
        //gehe zurück und zeige dem User was passiert.
        uid.exitmenu();
        break;
    }

    case UI_ACTION_RF_OUTPUT_OBJECT: {
        outputObject(); //als UI_ACTION_RF_OUTPUT_OBJECT
        break;
    }
    }

} //mainSwitchCase

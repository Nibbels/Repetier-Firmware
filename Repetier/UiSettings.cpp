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

// Panel setting edits, confirmation and value validation; navigation and rendering are separate.

#include "Repetier.h"
#include "StepExecutor.h"

const long baudrates[] PROGMEM = { 9600, 14400, 19200, 28800, 38400, 56000, 57600, 76800, 111112, 115200, 128000, 230400, 250000, 256000,
                                   460800, 500000, 921600, 1000000, 1500000, 0 };

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
#ifdef RF_EEPROM_ZONE_EXPERIMENT
        if(!EEPROM::lastOperationSucceeded())return;
        if(EEPROM::recoveryPending()) {
            exitmenu();
            EEPROM::reportRecovery();
            return;
        }
#else
        EEPROM::storeDataIntoEEPROM(false);
#endif
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        if(Printer::printJob.emergencyLocked())return;
#endif

        exitmenu();
        UI_STATUS(UI_TEXT_RESTORE_DEFAULTS);
        break;
    }

#ifdef RF_EEPROM_ZONE_EXPERIMENT
    case UI_ACTION_BED_CUTOFF:
        if(!EEPROM::commissionBedCutoff(bedCutoffCandidate_)) {
            Com::printErrorFLN(PSTR("Bed cutoff rejected; idle cold targets required"));
            UI_STATUS_UPD("Cutoff rejected");
        } else {
            UI_STATUS_UPD("Cutoff saved");
        }
        break;
#endif
    case UI_ACTION_CHOOSE_PID_ROBUST:
    case UI_ACTION_CHOOSE_PID_DYNAMIC: {
        if (confirmation_ != 1) {
            // continue only in case the user has chosen "Yes"
            break;
        }
        const uint8_t heater = menuPos[menuLevel - 2];
        if (heater >= NUM_TEMPERATURE_LOOPS)
            break;
        const bool bed = heater == NUM_EXTRUDER;
        const int method = (bed ? 2 : 0) + (action == UI_ACTION_CHOOSE_PID_ROBUST ? 1 : 0);
        const int temperature = bed ? heatBedScanPlaBedC : heatBedScanPlaNozzleC;
        const int cycles = bed ? 16 : 10;
        bool writeeeprom = true;
        // Show menu and message to user: He cant do anything until autotune is over.
        exitmenu();
        menuPos[0] = 3; // Show temps
        tempController[heater]->autotunePID(temperature, heater, cycles, writeeeprom, method);
        break;
    }
    }
} // finishAction

// Increment ist ne variable, global -> Knopfrichtung.
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
    case UI_ACTION_EPOSITION: {
        Printer::queueRelativeMMCoordinates(0, 0, 0, (float)increment / Printer::flowMultiplier(), UI_SET_EXTRUDER_FEEDRATE, true); // Klappt nicht in Pause!!
        Commands::printCurrentPosition();
        break;
    }
#ifdef RF_EEPROM_ZONE_EXPERIMENT
    case UI_ACTION_BED_CUTOFF: {
        const int candidate=int(bedCutoffCandidate_)+increment;
        bedCutoffCandidate_=constrain(candidate,int(BedTemperatureLimit::MinimumC),int(BedTemperatureLimit::AbsoluteC));
        break;
    }
#endif
    case UI_ACTION_RF_RESET_ACK:
    case UI_ACTION_STOP_ACK:
    case UI_ACTION_RESTORE_DEFAULTS:
    case UI_ACTION_CHOOSE_PID_ROBUST:
    case UI_ACTION_CHOOSE_PID_DYNAMIC: {
        INCREMENT_MIN_MAX(confirmation_, 1, 0, 1);
        break;
    }

    default:
        changeSetting(action, increment);
        break;
    }
} // changeSwitchCase

// Actual setting changes, separate from motion commands and dialog selection.
void UIDisplay::changeSetting(int action, int8_t increment) {
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
        EEPROM::storeFanDivisor(HAL::partFanDivisor());
        break;
    }
    case UI_ACTION_FAN_PART_FAN_PWM_MIN: {
        int temp = HAL::partFanMinDuty();
        INCREMENT_MIN_MAX(temp, 1, 1, 239);
        if (temp <= (int)HAL::partFanMaxDuty() - 16) {
            HAL::setPartFanDutyRange(temp, HAL::partFanMaxDuty());
        }
        // Recalculate active pwm value out of HAL::partFanSpeed() for easy tune-in.
        //(Tune-In: set fan to 1% and rise minimum until it starts.)
        Commands::setFanSpeed(HAL::partFanSpeed(), true);
        EEPROM::storeFanDuty(false,HAL::partFanMinDuty());
        break;
    }
    case UI_ACTION_FAN_PART_FAN_PWM_MAX: {
        int temp = HAL::partFanMaxDuty();
        INCREMENT_MIN_MAX(temp, 1, 16, 255);
        if (temp >= HAL::partFanMinDuty() + 16) {
            HAL::setPartFanDutyRange(HAL::partFanMinDuty(), temp);
        }
        // Recalculate active pwm value out of HAL::partFanSpeed() for easy tune-in.
        //(Tune-In: set fan to 100% and decrease maximum until fan slows down slightly.)
        Commands::setFanSpeed(HAL::partFanSpeed(), true);
        EEPROM::storeFanDuty(true,HAL::partFanMaxDuty());
        break;
    }
    case UI_ACTION_ZOFFSET: {
        int32_t manual = Printer::zOffset.manualMicrometers();
        INCREMENT_MIN_MAX(manual, Z_OFFSET_MENU_STEPS, -maxManualZOffsetMicrometers, maxManualZOffsetMicrometers);
        Printer::zOffset.setManualMicrometers(manual, Printer::stepsPerMm(Z_AXIS));
        EEPROM::storeManualZOffset();
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
        else {
            const int maximum=RMath::min(int(UI_SET_MAX_HEATED_BED_TEMP), int(heatedBedController.maximumTargetC()));
            if(maximum<UI_SET_MIN_HEATED_BED_TEMP)tmp=0;
            else if(tmp>maximum)tmp=maximum;
        }
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

        EEPROM::storeToolOffsets(1,extruder[1].offsetMM[X_AXIS],0,true,false);
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

        EEPROM::storeToolOffsets(1,0,extruder[1].offsetMM[Y_AXIS],false,true);
        break;
    }
#endif // NUM_EXTRUDER>1

    case UI_ACTION_FEEDRATE_MULTIPLY: {
        int fr = Printer::feedrateMultiplier();
        INCREMENT_MIN_MAX(fr, 1, 25, 500);
        Commands::changeFeedrateMultiply(fr);
        break;
    }
    case UI_ACTION_FLOWRATE_MULTIPLY: {
        float eF = Printer::flowMultiplier();
        INCREMENT_MIN_MAX(eF, 0.01f, 0.25f, 2.0f);
        Commands::changeFlowrateMultiply(eF);
        break;
    }
    case UI_ACTION_STEPPER_INACTIVE: {
        millis_t duration = Printer::inactivity.stepperTimeout();
        duration -= duration % 1000;
        INCREMENT_MAX(duration, 60000UL, 10080000UL);
        Printer::inactivity.setStepperTimeout(duration);
        EEPROM::storeInactivity(true,duration);
        break;
    }
    case UI_ACTION_MAX_INACTIVE: {
        millis_t duration = Printer::inactivity.shutdownTimeout();
        duration -= duration % 1000;
        INCREMENT_MAX(duration, 60000UL, 10080000UL);
        Printer::inactivity.setShutdownTimeout(duration);
        EEPROM::storeInactivity(false,duration);
        break;
    }
    case UI_ACTION_PRINT_ACCEL_X:
    case UI_ACTION_PRINT_ACCEL_Y:
    case UI_ACTION_PRINT_ACCEL_Z:
    case UI_ACTION_MOVE_ACCEL_X:
    case UI_ACTION_MOVE_ACCEL_Y:
    case UI_ACTION_MOVE_ACCEL_Z:
    case UI_ACTION_MAX_JERK:
    case UI_ACTION_MAX_ZJERK:
    case UI_ACTION_HOMING_FEEDRATE_X:
    case UI_ACTION_HOMING_FEEDRATE_Y:
    case UI_ACTION_HOMING_FEEDRATE_Z:
    case UI_ACTION_MAX_FEEDRATE_X:
    case UI_ACTION_MAX_FEEDRATE_Y:
    case UI_ACTION_MAX_FEEDRATE_Z:
        changeMotionSetting(action, increment);
        break;

    case UI_ACTION_BAUDRATE: {
        unsigned char p = 0;
        int32_t rate;
        do {
            rate = pgm_read_dword(&(baudrates[p]));
            if (rate == EEPROM::configuredBaudrate())
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
        EEPROM::setConfiguredBaudrate(pgm_read_dword(&(baudrates[p])));

        EEPROM::storeBaudrate();
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
    case UI_ACTION_EXTR_STEPS_E1:
    case UI_ACTION_ADVANCE_L_E0:
    case UI_ACTION_ADVANCE_L_E1:
    case UI_ACTION_EXTR_ACCELERATION_E0:
    case UI_ACTION_EXTR_ACCELERATION_E1:
    case UI_ACTION_EXTR_MAX_FEEDRATE_E0:
    case UI_ACTION_EXTR_MAX_FEEDRATE_E1:
    case UI_ACTION_EXTR_START_FEEDRATE_E0:
    case UI_ACTION_EXTR_START_FEEDRATE_E1:
        changeExtruderSetting(action, increment);
        break;

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
        EEPROM::storeScanLift(scanStartLiftMm());
        break;
    }

    case UI_ACTION_EMERGENCY_PAUSE_MIN:
    case UI_ACTION_EMERGENCY_PAUSE_MAX:
    case UI_ACTION_EMERGENCY_ZSTOP_MIN:
    case UI_ACTION_EMERGENCY_ZSTOP_MAX:
    case UI_ACTION_SENSEOFFSET_DIGITS:
    case UI_ACTION_SENSEOFFSET_MAX:
        changePressureSetting(action, increment);
        break;

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
        EEPROM::storeWobblePhase(phase);
        break;
    }
    case UI_ACTION_WOBBLE_FIX_AMPX: {
        auto amplitude = Printer::wobbleCorrection.profile().xMicrometers;
        INCREMENT_MIN_MAX(amplitude, 5, -995, 995);
        Printer::wobbleCorrection.setXAmplitude(amplitude);
        EEPROM::storeWobbleAmplitude(0,amplitude);
        break;
    }
    case UI_ACTION_WOBBLE_FIX_AMPY1: {
        auto amplitude = Printer::wobbleCorrection.profile().yAtXMinMicrometers;
        INCREMENT_MIN_MAX(amplitude, 5, -995, 995);
        Printer::wobbleCorrection.setYMinAmplitude(amplitude);
        EEPROM::storeWobbleAmplitude(1,amplitude);
        break;
    }
    case UI_ACTION_WOBBLE_FIX_AMPY2: {
        auto amplitude = Printer::wobbleCorrection.profile().yAtXMaxMicrometers;
        INCREMENT_MIN_MAX(amplitude, 5, -995, 995);
        Printer::wobbleCorrection.setYMaxAmplitude(amplitude);
        EEPROM::storeWobbleAmplitude(2,amplitude);
        break;
    }

    case UI_ACTION_CHOOSE_DMIN:
    case UI_ACTION_CHOOSE_DMAX:
    case UI_ACTION_CHOOSE_PIDMAX:
    case UI_ACTION_CHOOSE_SENSOR:
        changeHeaterSetting(action, increment);
        break;

    case UI_ACTION_CHOOSE_MOTOR_X:
    case UI_ACTION_CHOOSE_MOTOR_Y:
    case UI_ACTION_CHOOSE_MOTOR_Z:
    case UI_ACTION_CHOOSE_MOTOR_E0:
    case UI_ACTION_CHOOSE_MOTOR_E1: {
        uint8_t steppernr = menuPos[menuLevel];
        if (steppernr == 6)
            steppernr = 4;   // Das ist etwas stümperhaft, aber ich brauche die Zeilennummer um auszuwählen und die Einstellungen für die Extruderstepper gehören drüber, also muss Extruder 2 zwei Zeilen runter...
        if (steppernr < 5) { // aktuell gibts nur 5
            int drive = Drv8711::configuredCurrent(steppernr);
            static const short uMotorCurrentMax[] PROGMEM = MOTOR_CURRENT_MAX;
            INCREMENT_MIN_MAX(drive, 1, MOTOR_CURRENT_MIN + 1, static_cast<short>(pgm_read_word(&uMotorCurrentMax[steppernr]))); // Von 40 bis maximal das was in der config steht.
            Drv8711::stageCurrent(steppernr, drive);
            EEPROM::storeMotorCurrent(steppernr,uint8_t(drive));
            Drv8711::applyConfiguredCurrent(steppernr);
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
        uint16_t interval = StepExecutor::packingInterval();
        INCREMENT_MIN_MAX(interval, -100, MIN_STEP_PACKING_MIN_INTERVAL, MAX_STEP_PACKING_MIN_INTERVAL);
        StepExecutor::setPackingInterval(interval);
        EEPROM::storePackingInterval(StepExecutor::packingInterval());
        break;
    }
    case UI_ACTION_MICROSTEPS_XY:
    case UI_ACTION_MICROSTEPS_Z:
    case UI_ACTION_MICROSTEPS_E: {
        if (!Printer::printJob.isBusy() && !PrintLine::hasLines() && Printer::pause.allowsQueueAdvance()) {
            Printer::disableAllSteppersNow(); // Invalidate homing before changing physical resolution.

            const uint8_t firstAxis = action == UI_ACTION_MICROSTEPS_XY ? X_AXIS
                                    : action == UI_ACTION_MICROSTEPS_Z ? Z_AXIS : E_AXIS;
            const uint8_t lastAxis = firstAxis == Z_AXIS ? Z_AXIS : firstAxis + 1;
            const uint8_t maxMode = firstAxis == X_AXIS ? 6 : firstAxis == Z_AXIS ? 5 : 7;
            uint8_t mode = Drv8711::microstepMode(firstAxis);
            INCREMENT_MIN_MAX(mode, 1, 4, maxMode);

            uint8_t changedMask = 0;
            {
                InterruptProtectedBlock noInts;
                bool updatederived = false;
                bool updateextruder = false;
                for (uint8_t axis = firstAxis; axis <= lastAxis; ++axis) {
                    const uint8_t previous = Drv8711::microstepMode(axis);
                    if (previous == mode)
                        continue;

                    // Rescale each actual driver change, including previously
                    // mismatched pairs and a clamped/multi-step menu increment.
                    const float factor = float(drv8711ModeValue_2_MicroSteps(mode))
                                       / drv8711ModeValue_2_MicroSteps(previous);
                    Drv8711::setMicrostepMode(axis, mode);
                    changedMask |= 1U << axis;
                    if (axis < E_AXIS) {
                        Printer::setStepsPerMm(axis, Printer::stepsPerMm(axis) * factor);
                        Printer::pause.scaleParkSteps(axis, factor);
                        if (axis == Z_AXIS) {
                            Printer::zMotion.rescaleMotorReference(factor);
                            Printer::layers.rescaleFadeBounds(factor);
                            Printer::manualMotion.rescaleSteps(Z_AXIS, factor);
                            invalidateBedMatrixForReload();
                        }
                        updatederived = true;
                    } else if (axis - E_AXIS < NUM_EXTRUDER) {
                        // Five physical driver channels exist even on a
                        // single-extruder build. Never index the absent tool.
                        const uint8_t tool = axis - E_AXIS;
                        extruder[tool].stepsPerMM *= factor;
                        if (tool == Extruder::current->id)
                            updateextruder = true;
                    }
                }
                if (updatederived)
                    Printer::updateDerivedParameter();
                if (updateextruder)
                    Extruder::applyCurrentSettings();
            }
            if (changedMask & (1U << Z_AXIS))
                Printer::zOffset.refreshSteps(Printer::stepsPerMm(Z_AXIS));

            // Persistence is deliberately outside the interrupt guard.
            // No EEPROM read or write is needed for the atomic RAM update.
            EEPROM::storeMicrostepConfiguration(changedMask);
        }
        break;
    }
    }
}

void UIDisplay::changeMotionSetting(int action, int8_t increment) {
    switch (action) {
    case UI_ACTION_PRINT_ACCEL_X: {
        float value = Printer::printAcceleration(X_AXIS);
        INCREMENT_MIN_MAX(value, ACCELERATION_MENU_CHANGE_XY, ACCELERATION_MIN_XY, ACCELERATION_MAX_XY);
        Printer::setPrintAcceleration(X_AXIS, value);
        Printer::updateDerivedParameter();

        EEPROM::storeAxisAcceleration(X_AXIS,false,Printer::printAcceleration(X_AXIS));
        break;
    }
    case UI_ACTION_PRINT_ACCEL_Y: {
        float value = Printer::printAcceleration(Y_AXIS);
        INCREMENT_MIN_MAX(value, ACCELERATION_MENU_CHANGE_XY, ACCELERATION_MIN_XY, ACCELERATION_MAX_XY);
        Printer::setPrintAcceleration(Y_AXIS, value);
        Printer::updateDerivedParameter();

        EEPROM::storeAxisAcceleration(Y_AXIS,false,Printer::printAcceleration(Y_AXIS));
        break;
    }
    case UI_ACTION_PRINT_ACCEL_Z: {
        float value = Printer::printAcceleration(Z_AXIS);
        INCREMENT_MIN_MAX(value, ACCELERATION_MENU_CHANGE_Z, ACCELERATION_MIN_Z, ACCELERATION_MAX_Z);
        Printer::setPrintAcceleration(Z_AXIS, value);
        Printer::updateDerivedParameter();

        EEPROM::storeAxisAcceleration(Z_AXIS,false,Printer::printAcceleration(Z_AXIS));
        break;
    }
    case UI_ACTION_MOVE_ACCEL_X: {
        float value = Printer::travelAcceleration(X_AXIS);
        INCREMENT_MIN_MAX(value, ACCELERATION_MENU_CHANGE_XY, ACCELERATION_MIN_XY, ACCELERATION_MAX_XY);
        Printer::setTravelAcceleration(X_AXIS, value);
        Printer::updateDerivedParameter();

        EEPROM::storeAxisAcceleration(X_AXIS,true,Printer::travelAcceleration(X_AXIS));
        break;
    }
    case UI_ACTION_MOVE_ACCEL_Y: {
        float value = Printer::travelAcceleration(Y_AXIS);
        INCREMENT_MIN_MAX(value, ACCELERATION_MENU_CHANGE_XY, ACCELERATION_MIN_XY, ACCELERATION_MAX_XY);
        Printer::setTravelAcceleration(Y_AXIS, value);
        Printer::updateDerivedParameter();

        EEPROM::storeAxisAcceleration(Y_AXIS,true,Printer::travelAcceleration(Y_AXIS));
        break;
    }
    case UI_ACTION_MOVE_ACCEL_Z: {
        float value = Printer::travelAcceleration(Z_AXIS);
        INCREMENT_MIN_MAX(value, ACCELERATION_MENU_CHANGE_Z, ACCELERATION_MIN_Z, ACCELERATION_MAX_Z);
        Printer::setTravelAcceleration(Z_AXIS, value);
        Printer::updateDerivedParameter();

        EEPROM::storeAxisAcceleration(Z_AXIS,true,Printer::travelAcceleration(Z_AXIS));
        break;
    }
    case UI_ACTION_MAX_JERK: {
        float value = Printer::xyJerkLimit();
        INCREMENT_MIN_MAX(value, 0.1f, 1.0f, 33.3f);
        Printer::setXYJerkLimit(value); //RFx000: Limit 33.3 sind willkürlich grob faktor 3 von normalwert.

        EEPROM::storeAxisJerk(false,Printer::xyJerkLimit());
        break;
    }
    case UI_ACTION_MAX_ZJERK: {
        float value = Printer::zJerkLimit();
        INCREMENT_MIN_MAX(value, 0.05f, 0.05f, 2.0f);
        Printer::setZJerkLimit(value); //RFx000: Limit 2 sind Max XY-Jerk / Stepratenverhältnis XY->Z von ~16.8 gibt etwas unter 2.

        EEPROM::storeAxisJerk(true,Printer::zJerkLimit());
        break;
    }
    case UI_ACTION_HOMING_FEEDRATE_X: {
        float value = Printer::axisHomingFeedrate(X_AXIS);
        INCREMENT_MIN_MAX(value, 5, 5, MAX_FEEDRATE_X);
        Printer::setAxisHomingFeedrate(X_AXIS, value);

        EEPROM::storeAxisFeedrate(X_AXIS,true,Printer::axisHomingFeedrate(X_AXIS));
        break;
    }
    case UI_ACTION_HOMING_FEEDRATE_Y: {
        float value = Printer::axisHomingFeedrate(Y_AXIS);
        INCREMENT_MIN_MAX(value, 5, 5, MAX_FEEDRATE_Y);
        Printer::setAxisHomingFeedrate(Y_AXIS, value);

        EEPROM::storeAxisFeedrate(Y_AXIS,true,Printer::axisHomingFeedrate(Y_AXIS));
        break;
    }
    case UI_ACTION_HOMING_FEEDRATE_Z: {
        float value = Printer::axisHomingFeedrate(Z_AXIS);
        INCREMENT_MIN_MAX(value, 1, 1, MAX_FEEDRATE_Z);
        Printer::setAxisHomingFeedrate(Z_AXIS, value);

        EEPROM::storeAxisFeedrate(Z_AXIS,true,Printer::axisHomingFeedrate(Z_AXIS));
        break;
    }
    case UI_ACTION_MAX_FEEDRATE_X: {
        float value = Printer::axisMaxFeedrate(X_AXIS);
        INCREMENT_MIN_MAX(value, 5, 1, MAX_FEEDRATE_X);
        Printer::setAxisMaxFeedrate(X_AXIS, value);

        EEPROM::storeAxisFeedrate(X_AXIS,false,Printer::axisMaxFeedrate(X_AXIS));
        break;
    }
    case UI_ACTION_MAX_FEEDRATE_Y: {
        float value = Printer::axisMaxFeedrate(Y_AXIS);
        INCREMENT_MIN_MAX(value, 5, 1, MAX_FEEDRATE_Y);
        Printer::setAxisMaxFeedrate(Y_AXIS, value);

        EEPROM::storeAxisFeedrate(Y_AXIS,false,Printer::axisMaxFeedrate(Y_AXIS));
        break;
    }
    case UI_ACTION_MAX_FEEDRATE_Z: {
        float value = Printer::axisMaxFeedrate(Z_AXIS);
        INCREMENT_MIN_MAX(value, 1, 1, MAX_FEEDRATE_Z);
        Printer::setAxisMaxFeedrate(Z_AXIS, value);

        EEPROM::storeAxisFeedrate(Z_AXIS,false,Printer::axisMaxFeedrate(Z_AXIS));
        break;
    }
    }
}

void UIDisplay::changeExtruderSetting(int action, int8_t increment) {
    switch (action) {
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

            EEPROM::storeToolMotion(eNr,EEPROM::ToolMotionSetting::Steps,extruder[eNr].stepsPerMM);
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

        EEPROM::storeToolMotion(eNr,EEPROM::ToolMotionSetting::Advance,extruder[eNr].advanceL);
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

        EEPROM::storeToolMotion(eNr,EEPROM::ToolMotionSetting::Acceleration,extruder[eNr].maxAcceleration);
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

        EEPROM::storeToolMotion(eNr,EEPROM::ToolMotionSetting::Feedrate,extruder[eNr].maxFeedrate);
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

        EEPROM::storeToolMotion(eNr,EEPROM::ToolMotionSetting::Jerk,extruder[eNr].maxEJerk);
        break;
    }

    }
}

void UIDisplay::changePressureSetting(int action, int8_t increment) {
    switch (action) {
    case UI_ACTION_EMERGENCY_PAUSE_MIN: {
        auto limits = Printer::pressureProtection.pauseLimits();
        long minimum = limits.minimum;
        INCREMENT_MIN_MAX(minimum, 200, EMERGENCY_PAUSE_DIGITS_MIN, limits.maximum);
        limits.minimum = minimum;
        Printer::pressureProtection.setPauseLimits(limits);
        EEPROM::storePressureLimits(false,minimum,0,true,false);
        break;
    }
    case UI_ACTION_EMERGENCY_PAUSE_MAX: {
        auto limits = Printer::pressureProtection.pauseLimits();
        long maximum = limits.maximum;
        INCREMENT_MIN_MAX(maximum, 200, limits.minimum, EMERGENCY_PAUSE_DIGITS_MAX);
        limits.maximum = maximum;
        Printer::pressureProtection.setPauseLimits(limits);
        EEPROM::storePressureLimits(false,0,maximum,false,true);
        break;
    }

    case UI_ACTION_EMERGENCY_ZSTOP_MIN: {
        auto limits = Printer::pressureProtection.stopLimits();
        INCREMENT_MIN_MAX(limits.minimum, 200, EMERGENCY_STOP_DIGITS_MIN, limits.maximum);
        Printer::pressureProtection.setStopLimits(limits);
        EEPROM::storePressureLimits(true,limits.minimum,0,true,false);
        break;
    }
    case UI_ACTION_EMERGENCY_ZSTOP_MAX: {
        auto limits = Printer::pressureProtection.stopLimits();
        INCREMENT_MIN_MAX(limits.maximum, 200, limits.minimum, EMERGENCY_STOP_DIGITS_MAX);
        Printer::pressureProtection.setStopLimits(limits);
        EEPROM::storePressureLimits(true,0,limits.maximum,false,true);
        break;
    }

    case UI_ACTION_SENSEOFFSET_DIGITS: {
        short oldval = Printer::senseOffsetControl.storedLimits().thresholdDigits;
        INCREMENT_MIN_MAX(oldval, 100, 500, EMERGENCY_PAUSE_DIGITS_MAX);
        EEPROM::storeSenseOffsetSettings(oldval, 0, true, false);
        break;
    }
    case UI_ACTION_SENSEOFFSET_MAX: {
        short oldval = Printer::senseOffsetControl.storedLimits().maxMicrometers;
        INCREMENT_MIN_MAX(oldval, 10, 10, 300);
        EEPROM::storeSenseOffsetSettings(0, oldval, false, true);
        break;
    }

    }
}

void UIDisplay::changeHeaterSetting(int action, int8_t increment) {
#ifdef RF_EEPROM_ZONE_EXPERIMENT
    if(TemperatureController::autotuneActive())return;
#endif
    switch (action) {
    case UI_ACTION_CHOOSE_DMIN: {
        if (menuLevel == 4) {                        // Identifikation des temperaturzyklus anhand der position im menü. Das ist nicht 100% sauber, aber funktioniert.
            uint8_t heater = menuPos[menuLevel - 1]; // The bed follows the hotends in the PID menu.
            if (heater < NUM_TEMPERATURE_LOOPS) {
                int drive = tempController[heater]->pidDriveMin;
                INCREMENT_MIN_MAX(drive, 1, 1, 255);
                tempController[heater]->pidDriveMin = drive;
                tempController[heater]->updateTempControlVars();
                EEPROM::storeDriveLimit(heater,false,uint8_t(drive));
            }
        }
        break;
    }
    case UI_ACTION_CHOOSE_DMAX: {
        if (menuLevel == 4) {                        // Identifikation des temperaturzyklus anhand der position im menü. Das ist nicht 100% sauber, aber funktioniert.
            uint8_t heater = menuPos[menuLevel - 1]; // The bed follows the hotends in the PID menu.
            if (heater < NUM_TEMPERATURE_LOOPS) {
                int drive = tempController[heater]->pidDriveMax;
                INCREMENT_MIN_MAX(drive, 1, 1, 255);
                tempController[heater]->pidDriveMax = drive;
                tempController[heater]->updateTempControlVars();
                EEPROM::storeDriveLimit(heater,true,uint8_t(drive));
            }
        }
        break;
    }
    case UI_ACTION_CHOOSE_PIDMAX: {
        if (menuLevel == 4) {                        // Identifikation des temperaturzyklus anhand der position im menü. Das ist nicht 100% sauber, aber funktioniert.
            uint8_t heater = menuPos[menuLevel - 1]; // The bed follows the hotends in the PID menu.
            if (heater < NUM_TEMPERATURE_LOOPS) {
                int drive = tempController[heater]->pidMax;
                INCREMENT_MIN_MAX(drive, 1, 1, 255);
                if (tempController[heater]->pidMax == drive)
                    break;
                tempController[heater]->invalidatePidCalibration();
                tempController[heater]->pidMax = drive;
                tempController[heater]->updateTempControlVars();
                EEPROM::storePidCalibration(heater);
            }
        }
        break;
    }
    case UI_ACTION_CHOOSE_SENSOR: {
        if (menuLevel == 4) {                        // Identifikation des temperaturzyklus anhand der position im menü. Das ist nicht 100% sauber, aber funktioniert.
            uint8_t heater = menuPos[menuLevel - 1]; // The bed follows the hotends in the PID menu.
            if (heater < NUM_TEMPERATURE_LOOPS) {
                int drive = tempController[heater]->sensorType;
                if (NUM_EXTRUDER == heater) {
                    // Das ist das Heizbett
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
                    if (tempController[heater]->sensorType == drive)
                        break;
                    tempController[heater]->invalidatePidCalibration();
                    tempController[heater]->sensorType = drive;
                    EEPROM::storePidCalibration(heater);

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
                        } // Add more sensors for menu-tweaking here, those are the most common for RFx000
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
                        } // Add more sensors for menu-tweaking here, those are the most common for RFx000
                        default: {
                            drive = 3;
                            break;
                        }
                        }
                    }
                    if (tempController[heater]->sensorType == drive)
                        break;
                    tempController[heater]->invalidatePidCalibration();
                    tempController[heater]->sensorType = drive;
                    EEPROM::storePidCalibration(heater);

                }
            }
        }
        break;
    }
    }
}



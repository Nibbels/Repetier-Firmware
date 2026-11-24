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
            Commands::changeFeedrateMultiply(Printer::feedrateMultiplier() + direction);
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
        Printer::toggleDebugFlag(1);
        break;
    }
    case UI_ACTION_DEBUG_INFO: {
        Printer::toggleDebugFlag(2);
        break;
    }
    case UI_ACTION_DEBUG_ERROR: {
        Printer::toggleDebugFlag(4);
        break;
    }
    case UI_ACTION_DEBUG_DRYRUN: {
        Printer::toggleDebugFlag(8);
        if (Printer::debugDryrun()) // simulate movements without printing
        {
            Extruder::setTemperatureForAllExtruders(0);
            Extruder::setHeatedBedTemperature(0);
        }
        break;
    }

#if !HOTEND_COOLER_USES_PIN(CASE_LIGHT_PIN)
    case UI_ACTION_LIGHTS_ONOFF: {
        AccessoryOutputs::toggle(AccessoryOutputs::Output::CaseLight);

        HAL::eprSetByte(EPR_RF_CASE_LIGHT_MODE, AccessoryOutputs::state(AccessoryOutputs::Output::CaseLight));
        EEPROM::updateChecksum();

        break;
    }
#endif

#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    case UI_ACTION_230V_OUTPUT: {
        AccessoryOutputs::toggle(AccessoryOutputs::Output::Mains);

        // after a power-on, the 230 V plug always shall be turned off - thus, we do not store this setting to the EEPROM
        // HAL::eprSetByte( EPR_RF_230V_OUTPUT_MODE, AccessoryOutputs::state(AccessoryOutputs::Output::Mains) );
        // EEPROM::updateChecksum();

        break;
    }

#if FET1 > -1 && !HOTEND_COOLER_USES_PIN(FET1)
    case UI_ACTION_FET1_OUTPUT: {
        AccessoryOutputs::toggle(AccessoryOutputs::Output::Fet1);

        HAL::eprSetByte(EPR_RF_FET1_MODE, AccessoryOutputs::state(AccessoryOutputs::Output::Fet1));
        EEPROM::updateChecksum();
        break;
    }
#endif // FET1 accessory output
#if FET2 > -1 && !HOTEND_COOLER_USES_PIN(FET2)
    case UI_ACTION_FET2_OUTPUT: {
        AccessoryOutputs::toggle(AccessoryOutputs::Output::Fet2);

        HAL::eprSetByte(EPR_RF_FET2_MODE, AccessoryOutputs::state(AccessoryOutputs::Output::Fet2));
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
        Printer::toggleZDisplayMode();

        HAL::eprSetByte(EPR_RF_Z_MODE, Printer::zDisplayMode());
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

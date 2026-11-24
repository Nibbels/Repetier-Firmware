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

int Commands::lowestRAMValue = MAX_RAM;
int Commands::lowestRAMValueSend = MAX_RAM;

void Commands::commandLoop() {
    GCodeInput::readFromSerial();

    GCode* code = GCodeInput::peekCurrentCommand();
    if (code) {
        if (!Printer::printJob.commandsBlocked())
            Commands::executeGCode(code);
        GCodeInput::popCurrentCommand();
        Commands::checkForPeriodicalActions(Processing); //check heater and other stuff every n milliseconds
    } else {
        enum FirmwareState state = NotBusy;
        if (Printer::pause.isPaused()) {
            state = Paused;
        }

        Commands::checkForPeriodicalActions(state); //check heater and other stuff every n milliseconds
    }
} // commandLoop

void Commands::checkForPeriodicalActions(enum FirmwareState state) {
    /*
     * The execute variables are set by PWM-Timer. This timer ticks with about 3910 Hz and the ms clocks are made with software counter dividors.
     * Except this 16ms execute variable which is set by internal watchdog timer.
     */
    if (state != NotBusy) {
        GCodeInput::keepAlive(state);
    }

    const bool scanTickDue = HAL::takeScanTick();
    if (scanTickDue) {
        // Dieses freigabesignal sollte aus dem PWM-Timer kommen, denn dann ist klar, dass auch der noch läuft.
        // Dann laufen für den Watchdogreset der Timer und checkForPeriodicalActions().
        HAL::tellWatchdogOk();
    }

    if (HAL::takeCompensationTick()) {
        if (!HAL::temperatureTickPending()) {
            const millis_t lastButtonAt = uid.lastButtonTime();
            if (HAL::timeInMilliseconds() - lastButtonAt < 20000)
                uid.slowAction();
        }
        recalculateZCompensation();
    }

    if (HAL::takeTemperatureTick()) {
        Extruder::manageTemperatures();
        if (state == WaitHeater)
            Commands::printTemperatures(); //selfcontrolling timediff
        uid.slowAction();
    }

    Printer::strainGauge.service(HAL::timeInMilliseconds());

    if (HAL::takeFeatureTick()) {
        loopFeatures();
    }

    handleScanWorkTasks(scanTickDue);
} // checkForPeriodicalActions

bool Commands::waitForMotion(MotionWait motion, PauseWait pause) {
    const bool drainOutputs = motion == MotionWait::AllOutputs;
    // The periodic pump can run panel commands. Only one caller may drain Advance.
    static bool drainingOutputs = false;
    if (drainOutputs) {
        if (drainingOutputs)
            return false;
        drainingOutputs = true;
    }

    bool settled = false;
    auto& advance = Printer::extrusionOutput;
    while (!Printer::blockAll && !Printer::printJob.commandsBlocked()) {
        bool paused;
        {
            InterruptProtectedBlock lock;
            paused = Printer::pause.isPaused();
            const bool queueAndZSettled = !PrintLine::hasLines() && !Printer::needsCMPwait();
            if (drainOutputs) {
                if (queueAndZSettled && !paused && !PrintLine::directMoveActive())
                    settled = advance.drain();
                else
                    advance.cancelDrain();
            } else {
                settled = queueAndZSettled;
            }
        }
        if (paused && pause == PauseWait::Cancel) {
            settled = false;
            break;
        }
        if (settled)
            break;
        Commands::checkForPeriodicalActions(drainOutputs && paused ? Paused : Processing);
    }

    if (drainOutputs) {
        advance.cancelDrain();
        drainingOutputs = false;
    }
    return settled;
}

void Commands::printCurrentPosition() {
    // Wir zeigen dem Repetier-Host wo wir gerade stehen.
    // Hier ist die reine GCode-Position interessant, sonst würde das Einrechnen des GCode-Origin auch keinen Sinn machen.
    float x = Printer::xyMotion.nominalSteps(X_AXIS) * Printer::mmPerStep(X_AXIS);
    float y = Printer::xyMotion.nominalSteps(Y_AXIS) * Printer::mmPerStep(Y_AXIS);
    // Z bleibt ein Sonderfall. Normalerweise auch hier GCode-Position
    float z = Printer::currentZPositionMM();

    x += Printer::moveTarget.workOffsetMM(X_AXIS);
    y += Printer::moveTarget.workOffsetMM(Y_AXIS);
    z += Printer::moveTarget.workOffsetMM(Z_AXIS);

    Com::printF(Com::tXColon, x * (Printer::gcodeModes.inches() ? 0.03937 : 1), 2);
    Com::printF(Com::tSpaceYColon, y * (Printer::gcodeModes.inches() ? 0.03937 : 1), 2);
    Com::printF(Com::tSpaceZColon, z * (Printer::gcodeModes.inches() ? 0.03937 : 1), 2);
    Com::printFLN(Com::tSpaceEColon, Printer::moveTarget.machineMM(E_AXIS) * (Printer::gcodeModes.inches() ? 0.03937 : 1), 2);
} // printCurrentPosition

void Commands::printTemperatures(bool showRaw) {
    static millis_t lastTemperatureSignal = 0;
    millis_t now = HAL::timeInMilliseconds();
    if ((now - lastTemperatureSignal) > 1000) {
        lastTemperatureSignal = now;

        Com::printF(Com::tTColon, Extruder::current->tempControl.currentTemperatureC, 1);
        Com::printF(Com::tSpaceSlash, Extruder::current->tempControl.targetTemperatureC, 0);
        // Show output of autotune when tuning!
        Com::printF(Com::tSpaceAtColon, (HAL::outputDuty(Extruder::current->id)));

        Com::printF(Com::tSpaceBColon, Extruder::getHeatedBedTemperature(), 1);
        Com::printF(Com::tSpaceSlash, heatedBedController.targetTemperatureC, 0);

        if (showRaw) {
            Com::printF(Com::tSpaceRaw, (int)NUM_EXTRUDER);
            Com::printF(Com::tColon, (1023 << (2 - ANALOG_REDUCE_BITS)) - heatedBedController.currentTemperature);
        }
        Com::printF(Com::tSpaceBAtColon, (HAL::outputDuty(heatedBedController.pwmIndex))); // Show output of autotune when tuning!

#if NUM_EXTRUDER > 1
        for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
            Com::printF(Com::tSpaceT, (int)i);
            Com::printF(Com::tColon, extruder[i].tempControl.currentTemperatureC, 1);
            Com::printF(Com::tSpaceSlash, extruder[i].tempControl.targetTemperatureC, 0);

            Com::printF(Com::tSpaceAt, (int)i);
            Com::printF(Com::tColon, HAL::outputDuty(extruder[i].tempControl.pwmIndex)); // Show output of autotune when tuning!

            if (showRaw) {
                Com::printF(Com::tSpaceRaw, (int)i);
                Com::printF(Com::tColon, (1023 << (2 - ANALOG_REDUCE_BITS)) - extruder[i].tempControl.currentTemperature);
            }
        }
#endif // NUM_EXTRUDER

#ifdef X35_ADC_CHANNEL
        //Act as heated chamber ambient temperature for Repetier-Server 0.86.2+ ---> Letter C
        TemperatureController* act = &optTempController;
        act->updateCurrentTemperature();
        Com::printF(Com::tSpaceChamber);
        Com::printF(Com::tColon, act->currentTemperatureC, 1); //temp
        Com::printF(Com::tSpaceSlash, 0, 0);                   // ziel-temp
        Com::printF(Com::tSpaceCAtColon, 0);                   // leistung
#endif // X35_ADC_CHANNEL

        Com::printF(Com::tSpace);
        Com::printF(Com::tF);
        Com::printF(Com::tColon, (int)Printer::strainGauge.lastKnownDigits()); //Kraft
        Com::printF(Com::tSpaceSlash, (int)Printer::strainGauge.effectiveZeroOffset());
        Com::printF(Com::tSpaceAtColon, 0); //Ziel ^^, nein ich halte mich nur an die PWM-Syntax
        Com::println();
    }
} // printTemperatures

void Commands::changeFeedrateMultiply(int factor) {
    factor = constrain(factor, 25, 500);
    Printer::feedrate *= (float)factor / (float)Printer::feedrateMultiply;
    Printer::feedrateMultiply = factor;

    Com::printFLN(Com::tSpeedMultiply, factor);
} // changeFeedrateMultiply

void Commands::changeFlowrateMultiply(float newExtrusionFactor) {
    Printer::menuExtrusionFactor = constrain(newExtrusionFactor, 0.1f, 2.0f);

    Com::printFLN(Com::tFlowMultiply, (int)lroundf(100 * newExtrusionFactor));
} // changeFlowrateMultiply

void Commands::setFanSpeed(uint8_t speed, bool recalc) {
    if (HAL::partFanSpeed() == speed && !recalc)
        return;

    Printer::setMenuMode(MENU_MODE_FAN_RUNNING, speed > 0);
    // Repetier-Host rounds a reported value of 1 down to zero percent.
    Com::printFLN(Com::tFanspeed, speed == 1 ? 2 : speed);
    HAL::setPartFanSpeed(speed);
}

void Commands::adjustFanFrequency(uint8_t speed_divisor = PART_FAN_DEFAULT_PWM_SPEED_DIVISOR) {
    HAL::setPartFanDivisor(speed_divisor);
}

void Commands::adjustFanMode(uint8_t output_mode) {
    HAL::setPartFanMode(output_mode);
    Printer::setMenuMode(MENU_MODE_FAN_MODE_PDM, HAL::partFanMode() == PART_FAN_MODE_PDM);
}

void Commands::reportPrinterUsage() {
    if (!Printer::debugInfo())
        return;
    Com::printF(Com::tPrintedFilament, Printer::printUsage.filamentMetres(), 2);
    Com::printF(Com::tSpacem);

    int32_t seconds = Printer::printUsage.heatedSeconds();
    const int32_t days = seconds / 86400;
    seconds -= days * 86400;
    const int32_t hours = seconds / 3600;
    seconds -= hours * 3600;
    Com::printF(Com::tPrintingTime, days);
    Com::printF(Com::tSpaceDaysSpace, hours);
    Com::printF(Com::tSpaceHoursSpace, seconds / 60);
    Com::printFLN(Com::tSpaceMin);
}

void Commands::emergencyStop() {
    HAL::resetHardware(); //RF2000 / RF1000!
} // emergencyStop

void Commands::checkFreeMemory() {
    int newfree = HAL::getFreeRam();
    if (newfree < lowestRAMValue) {
        lowestRAMValue = newfree;
    }
} // checkFreeMemory

void Commands::writeLowestFreeRAM() {
    if (lowestRAMValueSend > lowestRAMValue) {
        lowestRAMValueSend = lowestRAMValue;

        if (true) //Printer::debugInfo()
        {
            Com::printFLN(Com::tFreeRAM, lowestRAMValue);
        }
    }
} // writeLowestFreeRAM

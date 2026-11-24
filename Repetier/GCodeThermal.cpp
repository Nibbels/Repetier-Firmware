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
#include "GCodeCommands.h"

namespace GCodeCommands {

void setHotendTemperature(GCode* com) {
    if (Printer::isAnyTempsensorDefect()) {
        reportTempsensorAndHeaterErrors();
        return;
    }
    Printer::inactivity.recordActivity(HAL::timeInMilliseconds());
    if (Printer::debugDryrun())
        return;

    //TODO man müsste das in den Movecache legen!
    const bool heaterOff = com->hasS() && com->S == 0;
    if ((com->hasP() || heaterOff) && !Commands::waitForMotion(Commands::MotionWait::QueueAndZ, Commands::PauseWait::Resume) && !heaterOff)
        return;

    if (com->hasS()) {
        if (com->hasT())
            Extruder::setTemperatureForExtruder(com->S, com->T);
        else
            Extruder::setTemperatureForExtruder(com->S, Extruder::current->id);
    }
}

void reportTemperature(GCode* com) {
    Commands::printTemperatures(com->hasX());
}

void startPartFan(GCode* com) {
    Commands::setFanSpeed(com->hasS() ? (uint8_t)constrain(com->S, 0, 255) : (uint8_t)255);
}

void stopPartFan() {
    Commands::setFanSpeed(0);
}

void waitForHotend(GCode* com) {
    Printer::inactivity.recordActivity(HAL::timeInMilliseconds());
    if (Printer::isAnyTempsensorDefect()) {
        reportTempsensorAndHeaterErrors();
        return;
    }
    if (Printer::debugDryrun()) {
        return;
    }

    Printer::idleStatus.cancel();
    if (!Commands::waitForMotion(Commands::MotionWait::QueueAndZ, Commands::PauseWait::Resume))
        return;

    Extruder* actExtruder = Extruder::current;
    if (com->hasT() && com->T < NUM_EXTRUDER)
        actExtruder = &extruder[com->T];
    if (com->hasS())
        Extruder::setTemperatureForExtruder(com->S, actExtruder->id);

    if (fabs(actExtruder->tempControl.targetTemperatureC - actExtruder->tempControl.currentTemperatureC) < TEMP_TOLERANCE) {
        return;
    }

    millis_t currentTime = HAL::timeInMilliseconds();
    bool isTempReached;
    bool dirRising = true;     // random init
    float settarget = -1;      // random init in °C

    do {
        Commands::checkForPeriodicalActions(WaitHeater);

        //Anpassung an die neue Situation falls der Bediener am Display-Menü des Druckers während Aufheizzeit was umstellt.
        if (settarget != actExtruder->tempControl.targetTemperatureC) {
            settarget = actExtruder->tempControl.targetTemperatureC; //in °C
            dirRising = (actExtruder->tempControl.targetTemperatureC > actExtruder->tempControl.currentTemperatureC);
            if (dirRising) {
                UI_STATUS_UPD(UI_TEXT_HEATING_EXTRUDER);
            } else {
                UI_STATUS_UPD(UI_TEXT_COOLING_DOWN);
            }
        }

        isTempReached = fabs(actExtruder->tempControl.targetTemperatureC - actExtruder->tempControl.currentTemperatureC) < TEMP_TOLERANCE;

        if (!dirRising) {
            if (actExtruder->tempControl.currentTemperatureC <= MAX_ROOM_TEMPERATURE) {
                break;
                //never wait longer than reaching lowest allowed temperature.
                //This might still be a long-run-bug if you have heated chamber/hot summer and wrong settings in MAX_ROOM_TEMPERATURE!
            }
        }
        if (abs(HAL::timeInMilliseconds() - currentTime) > 300000) {
            /* Aufheizen dauert nie länger als 5 Minuten */
            break;
        }
        if (Printer::isAnyTempsensorDefect()) {
            /* Temp sensor decoupled oder defekt? abort. */
            break;
        }
    } while (!isTempReached);


    Printer::idleStatus.schedule(HAL::timeInMilliseconds());
}

void waitForTemperatures() {
    if (Printer::debugDryrun())
        return;
    {
        bool allReached = false;
        while (!allReached) {
            allReached = true;
            Commands::checkForPeriodicalActions(WaitHeater);

            for (uint8_t h = 0; h < NUM_TEMPERATURE_LOOPS; h++) {
                TemperatureController* act = tempController[h];
                if (act->targetTemperatureC > MAX_ROOM_TEMPERATURE && fabs(act->targetTemperatureC - act->currentTemperatureC) > TEMP_TOLERANCE) {
                    allReached = false;
                }
            }
        }
    }
}

void setBedTemperature(GCode* com) {
    if (Printer::isAnyTempsensorDefect()) {
        reportTempsensorAndHeaterErrors();
        return;
    }
    Printer::inactivity.recordActivity(HAL::timeInMilliseconds());
    if (Printer::debugDryrun())
        return;
    if (com->hasS())
        Extruder::setHeatedBedTemperature(com->S);
}

void waitForBed(GCode* com) {
    Printer::inactivity.recordActivity(HAL::timeInMilliseconds());
    if (Printer::isAnyTempsensorDefect()) {
        reportTempsensorAndHeaterErrors();
        return;
    }
    if (Printer::debugDryrun()) {
        return;
    }
    Printer::idleStatus.cancel();
    if (!Commands::waitForMotion(Commands::MotionWait::QueueAndZ, Commands::PauseWait::Resume))
        return;
    if (com->hasS())
        Extruder::setHeatedBedTemperature(com->S);

    bool dirRising = true; // random init
    float settarget = -1;  // random init in °C

    while (fabs(heatedBedController.currentTemperatureC - heatedBedController.targetTemperatureC) > TEMP_TOLERANCE) {
        //Init und Anpassung an die neue Situation falls der Bediener am Display-Menü des Druckers während Aufheizzeit was umstellt.
        if (settarget != heatedBedController.targetTemperatureC) {
            settarget = heatedBedController.targetTemperatureC; //in °C
            dirRising = (heatedBedController.targetTemperatureC > heatedBedController.currentTemperatureC);
            if (dirRising) {
                UI_STATUS_UPD(UI_TEXT_HEATING_BED);
            } else {
                UI_STATUS_UPD(UI_TEXT_COOLING_DOWN);
            }
        }

        Commands::checkForPeriodicalActions(WaitHeater);
        if (!dirRising && heatedBedController.currentTemperatureC <= MAX_ROOM_TEMPERATURE)
            break;
        if (Printer::isAnyTempsensorDefect())
            break;
    }

    Printer::idleStatus.schedule(HAL::timeInMilliseconds() + 5000);
}

void setPidGains(GCode* com) {
    TemperatureController* temp = &Extruder::current->tempControl;
    if (com->hasS()) {
        if (com->S < 0)
            return;
        if (com->S < NUM_EXTRUDER)
            temp = &extruder[com->S].tempControl;
        else
            temp = &heatedBedController;
    }
    if (com->hasX())
        temp->pidPGain = com->X;
    if (com->hasY())
        temp->pidIGain = com->Y;
    if (com->hasZ())
        temp->pidDGain = com->Z;
    temp->updateTempControlVars();
}

void tunePid(GCode* com) {
    int temp = 150;
    int cont = 0;
    int cycles = 10;
    int method = 0; //0 = Classic PID
    if (com->hasS())
        temp = com->S; //Verwechsle ich immer, weil T wie Temperatur, aber T ist 0..255
    if (com->hasP())
        cont = com->P;
    if (com->hasR())
        cycles = static_cast<int>(com->R);
    if (com->hasJ())
        method = static_cast<int>(com->J); //original Repetier used hasC, we dont have that in this version of repetier.
    if (cont >= NUM_TEMPERATURE_LOOPS)
        cont = NUM_TEMPERATURE_LOOPS - 1;
    if (cont < 0)
        cont = 0;
    tempController[cont]->autotunePID(temp, cont, cycles, com->hasX(), method);
}

void setCaseLight(GCode* com) {
#if HOTEND_COOLER_USES_PIN(CASE_LIGHT_PIN)
    (void)com;
    Com::printErrorFLN(PSTR("M355: X19 is assigned to a hotend cooler"));
#else
    // Idee und Teilcode und Vorarbeit von WESSIX
    // Code schaltet X19, das ist nicht zwingend das Licht!
    if (com->hasS()) {
        if (com->S == 1 || com->S == 0) {
            Printer::enableCaseLight = com->S;
        } else {
            Com::printFLN(PSTR("M355 Error S=0||1"));
        }
    } else {
        if (Printer::enableCaseLight)
            Printer::enableCaseLight = 0;
        else
            Printer::enableCaseLight = 1;
    }
    WRITE(CASE_LIGHT_PIN, Printer::enableCaseLight);
    Com::printFLN(PSTR("M355: X19 set to "), Printer::enableCaseLight);
#endif
}

#if (MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2) && CASE_FAN_PIN > -1
void turnOnCaseFan() {
    //disable fan-temp-ignore to original state // Nibbels
    Printer::ignoreFanOn = false;

    // enable the case fan
    Printer::prepareFanOff = 0;
    WRITE(CASE_FAN_PIN, 1);

    if (Printer::debugInfo()) {
        Com::printFLN(PSTR("M3120: fan on"));
    }
}

void scheduleCaseFanOff(GCode* pCommand) {
    // disable the case fan
    if (pCommand->hasS()) {
        // we shall set a new case fan off delay
        Printer::fanOffDelay = pCommand->S;
        Printer::fanOffDelay *= 1000; // convert from [s] to [ms]
    }

    if (pCommand->hasP()) {
        // the fan should be disabled even if the print starts // temp > fan on temp // Nibbels
        Printer::ignoreFanOn = true;
    }

    if (Printer::fanOffDelay) {
        // we are going to disable the case fan after the delay
        Printer::prepareFanOff = HAL::timeInMilliseconds();

        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3121: fan off in "), pCommand->S);
            Com::printFLN(PSTR(" [s]"));
        }
    } else {
        // we are going to disable the case fan now
        Printer::prepareFanOff = 0;
        WRITE(CASE_FAN_PIN, 0);

        if (Printer::debugInfo()) {
            Com::printFLN(PSTR("M3121: fan off"));
        }
    }
}
#endif

#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
void setAccessoryFet(GCode* pCommand) {
    if (pCommand->hasP()) {
        if ((pCommand->P == 1 && FET1 > -1 && HOTEND_COOLER_USES_PIN(FET1))
            || (pCommand->P == 2 && FET2 > -1 && HOTEND_COOLER_USES_PIN(FET2))) {
            Com::printErrorFLN(PSTR("M3300: output is assigned to a hotend cooler"));
            return;
        }
        switch (pCommand->P) {
#if FET1 > -1 && !HOTEND_COOLER_USES_PIN(FET1)
        case 1: {
            if (pCommand->hasS()) {
                if (pCommand->S == 0) {
                    Printer::enableFET1 = 0;
                    WRITE(FET1, Printer::enableFET1);
                    Com::printFLN(PSTR("24V X42 FET1-output = off"));
                } else {
                    Printer::enableFET1 = 1;
                    WRITE(FET1, Printer::enableFET1);
                    Com::printFLN(PSTR("24V X42 FET1-output = on"));
                }
                HAL::eprSetByte(EPR_RF_FET1_MODE, Printer::enableFET1);
                EEPROM::updateChecksum();
            }
            break;
        }
#endif // FET1 accessory output
#if FET2 > -1 && !HOTEND_COOLER_USES_PIN(FET2)
        case 2: {
            if (pCommand->hasS()) {
                if (pCommand->S == 0) {
                    Printer::enableFET2 = 0;
                    WRITE(FET2, Printer::enableFET2);
                    Com::printFLN(PSTR("24V X44 FET2-output = off"));
                } else {
                    Printer::enableFET2 = 1;
                    WRITE(FET2, Printer::enableFET2);
                    Com::printFLN(PSTR("24V X44 FET2-output = on"));
                }
                HAL::eprSetByte(EPR_RF_FET2_MODE, Printer::enableFET2);
                EEPROM::updateChecksum();
            }
            break;
        }
#endif // FET2 accessory output
#if FET3 > -1
        case 3: {
            if (pCommand->hasS()) {
                if (pCommand->S == 0) {
                    Printer::enableFET3 = 0;
                    WRITE(FET3, Printer::enableFET3);
                    Com::printFLN(PSTR("24V X45/BoardFan FET3-output = off"));
                } else {
                    Printer::enableFET3 = 1;
                    WRITE(FET3, Printer::enableFET3);
                    Com::printFLN(PSTR("24V X45/BoardFan FET3-output = on"));
                }
                HAL::eprSetByte(EPR_RF_FET3_MODE, Printer::enableFET3);
                EEPROM::updateChecksum();
            }
            break;
        }
#endif // FET3
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setMainsOutput(GCode* pCommand) {
    if (pCommand->hasS()) {
        if (pCommand->S == 0) {
            Printer::enable230VOutput = 0;
            WRITE(OUTPUT_230V_PIN, Printer::enable230VOutput);
            Com::printFLN(PSTR("230V output = off"));
        } else {
            Printer::enable230VOutput = 1;
            WRITE(OUTPUT_230V_PIN, Printer::enable230VOutput);
            Com::printFLN(PSTR("230V output = on"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}
#endif

static bool prepareFilamentProgram() {
    if (Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Cancel)) {
        Printer::printJob.beginFilamentService();
        return true;
    }
    Com::printFLN(PSTR("Filament program requires settled, unpaused motion"));
    return false;
}

void loadFilament(GCode* pCommand) {
    if (!prepareFilamentProgram())
        return;
    Com::printFLN(PSTR("Load Filament"));
    Printer::idleStatus.cancel();

        //normalize dms sensor values to 0 if allowed in options and not already done
    if (!Printer::strainGauge.zeroOffset())
        Printer::homeDigits();

    uint8_t Extrusion = 65; //max, wenn kein wiederstand. (ca. hotendlänge)
    uint8_t eFeedrate = 1;
    if (pCommand->hasF())
        eFeedrate = constrain(abs(static_cast<uint8_t>(pCommand->F)), 1, 50);
    short maxP = 2000; //to overwrite .. safety.
    if (pCommand->hasP())
        maxP = constrain(abs(static_cast<short>(pCommand->P)), 0, 20000);
    const bool savedRelativeE = Printer::gcodeModes.relativeEOverride();
    Printer::gcodeModes.setRelativeEOverride(false); //M82:
    const bool savedInches = Printer::gcodeModes.inches();
    Printer::gcodeModes.setInches(0); //to save possible G21 .. weiß net ob das so wichtig wäre ^^.

    Printer::setEPositionMM(0); // G92 E0

    float e = 0.0f;
    while (e < float(Extrusion)) {
        Commands::checkForPeriodicalActions(Processing);
        int16_t digits;
        const bool haveDigits = !Printer::printJob.commandsBlocked()
            && Printer::strainGauge.readDigits(HAL::timeInMilliseconds(), digits);
        if (haveDigits)
            updateDigitCompensation(digits);
        if (!haveDigits || abs(int32_t(digits)) > maxP) {
            UI_STATUS_UPD(UI_TEXT_OK);
            break;
        }
        if (Printer::checkAbortKeys()) {
            UI_STATUS_UPD(UI_TEXT_ABORT_KEYPRESSED);
            break;
        }
        e += 0.02; //kleine schritte
        Printer::queueFloatCoordinates(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, e, eFeedrate);
    }

    Printer::setEPositionMM(0); // G92 E0

    Printer::gcodeModes.setRelativeEOverride(savedRelativeE);
    Printer::gcodeModes.setInches(savedInches);
    Printer::printJob.finishFilamentService();
    Printer::idleStatus.schedule(HAL::timeInMilliseconds() + 5000);
}

void unloadFilament(GCode* pCommand) {
    if (!prepareFilamentProgram())
        return;
    Com::printFLN(PSTR("Unload Filament"));
    Printer::idleStatus.cancel();

    //init
    uint8_t outputLength = 100; //9cm max Output, wenn kein übertriebener wiederstand. Unser Hotend ist nicht ganz so lang.
    float eFeedrate = 0.3f;     /*mm/s*/
    short maxForce = 4000;      //3500+ might work well! You should stay underneath the force causing step loss.
    if (pCommand->hasP())
        maxForce = constrain(abs(static_cast<short>(pCommand->P)), 0, EMERGENCY_PAUSE_DIGITS_MAX);
    float t = float(UI_SET_EXTRUDER_MIN_TEMP_UNMOUNT);

    //preheat / precool
    Extruder::setTemperatureForExtruder(t, Extruder::current->id);
    Extruder::current->tempControl.waitForTargetTemperature(10);

    //set stuff
    const bool savedRelativeE = Printer::gcodeModes.relativeEOverride();
    Printer::gcodeModes.setRelativeEOverride(false); //M82:
    const bool savedInches = Printer::gcodeModes.inches();
    Printer::gcodeModes.setInches(0); //to save possible G21 .. weiß net ob das so wichtig wäre ^^.
    Printer::setEPositionMM(0); // G92 E0

        //normalize dms sensor values to 0 if allowed in options and not already done
    if (!Printer::strainGauge.zeroOffset())
        Printer::homeDigits();

    //work
    float e = 0.0f;
    float de = 0.005f;
    uint8_t retry = 5;
    while (t < float(UI_SET_EXTRUDER_MAX_TEMP_UNMOUNT) && fabs(e) < float(outputLength)) {
        millis_t time = HAL::timeInMilliseconds() + 2000;
        while (HAL::timeInMilliseconds() <= time) {
            Commands::checkForPeriodicalActions(WaitHeater);
            UI_STATUS_UPD(UI_TEXT_UNMOUNT_FILAMENT);
            if (Printer::printJob.commandsBlocked())
                break;
        }
        if (Printer::printJob.commandsBlocked())
            break;
        while (fabs(e) < float(outputLength)) {
            Commands::checkForPeriodicalActions(Processing);
            if (Printer::printJob.commandsBlocked())
                break;
            int16_t digits;
            if (!Printer::strainGauge.readDigits(HAL::timeInMilliseconds(), digits)) {
                e = outputLength;
                retry = 0;
                break;
            }
            updateDigitCompensation(digits);
            if (abs(int32_t(digits)) > maxForce) {
                t += 3.33; // +1°K
                break;
            }
            if (Printer::checkAbortKeys()) {
                e = float(outputLength); //for sure exits because of no retrys
                retry = 0;               //exits moves
                UI_STATUS_UPD(UI_TEXT_ABORT_KEYPRESSED);
                break;
            }
            e -= de; //mm - kleine schritte auf Zug
            if (fabs(e) > 5.0f /*mm*/ && eFeedrate < 10.0f /*mm/s*/) {
                eFeedrate += (fabs(e) > 10.0f ? 0.01f : 0.001f); // mm/s
            }
            if (fabs(e) > 15.0f /*mm*/ && eFeedrate < 20.0f /*mm/s*/) {
                eFeedrate += 0.01f; // mm/s
                if (de <= 0.02f)
                    de += 0.0001f;
            }
            Printer::queueFloatCoordinates(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, e, eFeedrate);
        }
        if (Printer::printJob.commandsBlocked())
            break;
        if (t > Extruder::current->tempControl.targetTemperatureC && fabs(e) < 10.0f /*mm*/) {
            //wenn der extruder nicht weiterkommt und die bisherige wegstrecke unter 10mm ist, dann etwas die Temperatur erhöhen und nochmal ziehen.
            Extruder::setTemperatureForExtruder(t, Extruder::current->id);
            Extruder::current->tempControl.waitForTargetTemperature();
        } else if (fabs(e) >= 10.0f /*mm*/) {
            //das filament kam schon etwas raus, hängt aber nun.
            if (!retry--) {
                break;
            }
        }
        eFeedrate = 0.3f;
    }
    Extruder::setTemperatureForExtruder(0, Extruder::current->id);
    //cleanup

    Printer::setEPositionMM(0); // G92 E0

    Printer::gcodeModes.setRelativeEOverride(savedRelativeE);
    Printer::gcodeModes.setInches(savedInches);
    Printer::printJob.finishFilamentService();
    Printer::idleStatus.schedule(HAL::timeInMilliseconds() + 5000);
}

#ifdef X35_ADC_CHANNEL
void readOptionalTemperature() {
    //Com::printFLN( PSTR( "M3941 TempReader starting ..." ) );
    TemperatureController* act = &optTempController;
    act->updateCurrentTemperature();
    Com::printFLN(PSTR("Opt Temp: "), act->currentTemperatureC, 2);
    //Com::printFLN( PSTR( "M3941 Ended!" ) );
}
#endif

} // namespace GCodeCommands

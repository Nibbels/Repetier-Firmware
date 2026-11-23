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

int Commands::lowestRAMValue = MAX_RAM;
int Commands::lowestRAMValueSend = MAX_RAM;

void Commands::commandLoop() {
    GCode::readFromSerial();

    GCode* code = GCode::peekCurrentCommand();
    if (code) {
        Commands::executeGCode(code);
        code->popCurrentCommand();
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
        GCode::keepAlive(state);
    }

    if (execute10msPeriodical) {
        execute10msPeriodical = 0;
        // Dieses freigabesignal sollte aus dem PWM-Timer kommen, denn dann ist klar, dass auch der noch läuft.
        // Dann laufen für den Watchdogreset der Timer und checkForPeriodicalActions().
        HAL::tellWatchdogOk();
    }

    if (execute16msPeriodical) {
        execute16msPeriodical = 0;
        bool buttonSpeedBoost = (!execute100msPeriodical && HAL::timeInMilliseconds() - uid.lastButtonStart < 20000);
        if (buttonSpeedBoost)
            UI_SLOW;
        recalculateZCompensation();
    }

    if (execute100msPeriodical) {
        execute100msPeriodical = 0;
        Extruder::manageTemperatures();
        if (state == WaitHeater)
            Commands::printTemperatures(); //selfcontrolling timediff
        UI_SLOW;
    }

    if (execute50msPeriodical) {
        execute50msPeriodical = 0;
        loopFeatures();
    }

} // checkForPeriodicalActions

/** \brief Waits until movement cache is empty.
    Some commands expect no movement, before they can execute. This function
    waits, until the steppers are stopped. In the meanwhile it buffers incoming
    commands and manages temperatures. */
void Commands::waitUntilEndOfAllMoves() {
    bool bWait = false;

    if (PrintLine::hasLines())
        bWait = true;

    bWait = (Printer::needsCMPwait() ? true : bWait);

    while (bWait) {
        Commands::checkForPeriodicalActions(Processing);

        bWait = false;
        if (PrintLine::hasLines())
            bWait = true;

        bWait = (Printer::needsCMPwait() ? true : bWait);
    }
} // waitUntilEndOfAllMoves

void Commands::printCurrentPosition() {
    // Wir zeigen dem Repetier-Host wo wir gerade stehen.
    // Hier ist die reine GCode-Position interessant, sonst würde das Einrechnen des GCode-Origin auch keinen Sinn machen.
    float x = Printer::currentXYSteps[X_AXIS] * Printer::axisMMPerSteps[X_AXIS];
    float y = Printer::currentXYSteps[Y_AXIS] * Printer::axisMMPerSteps[Y_AXIS];
    // Z bleibt ein Sonderfall. Normalerweise auch hier GCode-Position
    float z = Printer::currentZPositionMM();

    x += Printer::originOffsetMM[X_AXIS];
    y += Printer::originOffsetMM[Y_AXIS];
    z += Printer::originOffsetMM[Z_AXIS];

    Com::printF(Com::tXColon, x * (Printer::unitIsInches ? 0.03937 : 1), 2);
    Com::printF(Com::tSpaceYColon, y * (Printer::unitIsInches ? 0.03937 : 1), 2);
    Com::printF(Com::tSpaceZColon, z * (Printer::unitIsInches ? 0.03937 : 1), 2);
    Com::printFLN(Com::tSpaceEColon, Printer::destinationMM[E_AXIS] * (Printer::unitIsInches ? 0.03937 : 1), 2);
} // printCurrentPosition

void Commands::printTemperatures(bool showRaw) {
    static millis_t lastTemperatureSignal = 0;
    millis_t now = HAL::timeInMilliseconds();
    if ((now - lastTemperatureSignal) > 1000) {
        lastTemperatureSignal = now;

        Com::printF(Com::tTColon, Extruder::current->tempControl.currentTemperatureC, 1);
        Com::printF(Com::tSpaceSlash, Extruder::current->tempControl.targetTemperatureC, 0);
        // Show output of autotune when tuning!
        Com::printF(Com::tSpaceAtColon, (pwm_pos[Extruder::current->id]));

        Com::printF(Com::tSpaceBColon, Extruder::getHeatedBedTemperature(), 1);
        Com::printF(Com::tSpaceSlash, heatedBedController.targetTemperatureC, 0);

        if (showRaw) {
            Com::printF(Com::tSpaceRaw, (int)NUM_EXTRUDER);
            Com::printF(Com::tColon, (1023 << (2 - ANALOG_REDUCE_BITS)) - heatedBedController.currentTemperature);
        }
        Com::printF(Com::tSpaceBAtColon, (pwm_pos[heatedBedController.pwmIndex])); // Show output of autotune when tuning!

#if NUM_EXTRUDER > 1
        for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
            Com::printF(Com::tSpaceT, (int)i);
            Com::printF(Com::tColon, extruder[i].tempControl.currentTemperatureC, 1);
            Com::printF(Com::tSpaceSlash, extruder[i].tempControl.targetTemperatureC, 0);

            Com::printF(Com::tSpaceAt, (int)i);
            Com::printF(Com::tColon, (pwm_pos[extruder[i].tempControl.pwmIndex])); // Show output of autotune when tuning!

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
        Com::printF(Com::tColon, (int)Printer::digitCompensation.latestDigits()); //Kraft
        Com::printF(Com::tSpaceSlash, (int)(Printer::g_pressure_offset_active ? Printer::g_pressure_offset : 0)); //Offset
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

uint8_t fanKickstart = 0;
void Commands::setFanSpeed(uint8_t speed, bool recalc) {
    //speed = constrain(speed,0,255); nichts anderes passt in uint8_t hinein.

    //do nothing if the fan speed does not change at all
    if (fanSpeed == speed && !recalc)
        return;

    //the new fan speed value is being remembered unchanged
    //the output of the speed within display etc. stays unscaled
    fanSpeed = speed;

    //output the new setting unscaled to repetier-host/server UI
    Printer::setMenuMode(MENU_MODE_FAN_RUNNING, (fanSpeed > 0));
    Com::printFLN(Com::tFanspeed, (fanSpeed == 1) ? 2 : fanSpeed); //bei 1 zeigt repetierserver / repetierhost 0% an, was nicht stimmt. Das ist etwas Pfusch, aber nun funktionierts.

    //wenn speed > 0 und < 255, dann wird der wertebereich eingegrenzt, sonst === 0 oder full power
    if (speed > 0 && speed < 255) {
        //scale the input speed value within MIN and MAX
        long temp = speed;
        temp *= (part_fan_pwm_max - part_fan_pwm_min);
        temp /= 255;
        temp += part_fan_pwm_min;
        speed = constrain(temp, 0, 255);
        /*
        from here "speed" is scaled to a set boundary. It will be the same scale like pwm_pos[NUM_EXTRUDER+2] has.
        Commanded (and user ui) speed is "fanSpeed", which stays unscaled.
        */
    }

    //if specified calculate a kickstart time when the fan speed is commanded to rise
    //use fanSpeed (not "speed") to compare with PART_FAN_KICKSTART_THRESHOLD so we dont have to recalculate the threshold according min-max.
    if (fanKickstart == 0 && speed > pwm_pos[NUM_EXTRUDER + 2] && fanSpeed < PART_FAN_KICKSTART_THRESHOLD) {
        if (speed)
            fanKickstart = PART_FAN_KICKSTART_TIME_BOOST / 10;
        else
            fanKickstart = PART_FAN_KICKSTART_TIME_OFF_ON / 10;
    }

    pwm_pos[NUM_EXTRUDER + 2] = speed;
} // setFanSpeed

void Commands::adjustFanFrequency(uint8_t speed_divisor = PART_FAN_DEFAULT_PWM_SPEED_DIVISOR) { //1 = ~15.3hz, ~2=7.62hz, ...
    InterruptProtectedBlock noInts;
    if (!speed_divisor)
        speed_divisor = 1;
    part_fan_pwm_speed = (speed_divisor <= PART_FAN_MODE_MAX ? speed_divisor : PART_FAN_DEFAULT_PWM_SPEED_DIVISOR);
}
void Commands::adjustFanMode(uint8_t output_mode) { //0 = pwm, 1 = pdm
    part_fan_frequency_modulation = (output_mode ? PART_FAN_MODE_PDM : PART_FAN_MODE_PWM);
    Printer::setMenuMode(MENU_MODE_FAN_MODE_PDM, (bool)part_fan_frequency_modulation);
}

void Commands::reportPrinterUsage() {
    if (Printer::debugInfo()) {
        float dist = Printer::filamentPrinted * 0.001 + HAL::eprGetFloat(EPR_PRINTING_DISTANCE);
        Com::printF(Com::tPrintedFilament, dist, 2);
        Com::printF(Com::tSpacem);
    }
    bool alloff = true;

    for (uint8_t i = 0; i < NUM_EXTRUDER; i++)
        if (tempController[i]->targetTemperatureC > 0)
            alloff = false;

    if (Printer::debugInfo()) {
        int32_t seconds = (alloff ? 0 : (HAL::timeInMilliseconds() - Printer::msecondsPrinting) / 1000) + HAL::eprGetInt32(EPR_PRINTING_TIME);
        int32_t tmp = seconds / 86400;
        seconds -= tmp * 86400;

        Com::printF(Com::tPrintingTime, tmp);
        tmp = seconds / 3600;
        Com::printF(Com::tSpaceDaysSpace, tmp);
        seconds -= tmp * 3600;
        tmp = seconds / 60;

        Com::printF(Com::tSpaceHoursSpace, tmp);
        Com::printFLN(Com::tSpaceMin);
    }
} // reportPrinterUsage

/** \brief Execute the command stored in com. */
void Commands::executeGCode(GCode* com) {
    if (com->hasG()) {
        Commands::processGCode(com);
    } else if (com->hasM()) // Process M Code
    {
        Commands::processMCode(com);
    } else if (com->hasT()) // Process T code
    {
        Commands::waitUntilEndOfAllMoves(); //Tn-Code (change Extruder)
        Extruder::selectExtruderById(com->T);
    } else if (com->hasF()) // Process F code http://linuxcnc.org/docs/html/gcode/other-code.html#sec:set-feed-rate
    {
        Printer::setFeedrate(com->F);
    } else {
        if (Printer::debugErrors()) {
            Com::printF(Com::tUnknownCommand);
            com->printCommand();
        }
    }
} // executeGCode

/**
\brief Execute the G command stored in com.
*/
void Commands::processGCode(GCode* com) {
    switch (com->G) {
    case 0: // G0 -> G1
    case 1: // G1
    {
        if (isMovingAllowed(PSTR("G0/1"))) {
            Printer::queueGCodeCoordinates(com); // For X Y Z E F
        }

        break;
    }

    case 4: // G4 dwell
    {
        Commands::waitUntilEndOfAllMoves(); //G4
        uint32_t codenum = 0;
        if (com->hasP())
            codenum = com->P; // milliseconds to wait
        if (com->hasS())
            codenum = com->S * 1000;          // seconds to wait
        codenum += HAL::timeInMilliseconds(); // keep track of when we started waiting

        while ((uint32_t)(codenum - HAL::timeInMilliseconds()) < 2000000000) {
            Commands::checkForPeriodicalActions(Processing);
        }
        break;
    }
    case 20: // G20 - Units to inches
    {
        Printer::unitIsInches = 1;
        break;
    }
    case 21: // G21 - Units to mm
    {
        Printer::unitIsInches = 0;
        break;
    }
    case 28: // G28 - Home all axes one at a time
    {
        if (!isHomingAllowed(com)) {
            break;
        }
        uint8_t home_all_axis = (com->hasNoXYZ() && !com->hasE());
        if (com->hasE()) {
            Printer::setEPositionMM(0); // G92 E0
        }
        if (home_all_axis || !com->hasNoXYZ())
            Printer::homeAxis(home_all_axis || com->hasX(), home_all_axis || com->hasY(), home_all_axis || com->hasZ());
    } break;


    case 90: // G90
    {
        Printer::relativeCoordinateMode = false;
        if (com->internalCommand)
            Com::printInfoFLN(PSTR("Absolute positioning"));
        break;
    }
    case 91: // G91
    {
        Printer::relativeCoordinateMode = true;
        if (com->internalCommand)
            Com::printInfoFLN(PSTR("Relative positioning"));
        break;
    }
    case 92: // G92
    {
        if (!com->hasNoXYZ()) {
            // set the origin only in case we got any x, y and/or z offset
            float xOff = Printer::originOffsetMM[X_AXIS];
            float yOff = Printer::originOffsetMM[Y_AXIS];
            float zOff = Printer::originOffsetMM[Z_AXIS];

            if (com->hasX())
                xOff = Printer::convertToMM(com->X) - Printer::destinationMM[X_AXIS];
            if (com->hasY())
                yOff = Printer::convertToMM(com->Y) - Printer::destinationMM[Y_AXIS];
            if (com->hasZ())
                zOff = Printer::convertToMM(com->Z) - Printer::destinationMM[Z_AXIS];

            Printer::setOrigin(xOff, yOff, zOff);
        }
        if (com->hasE()) {
            Printer::setEPositionMM(Printer::convertToMM(com->E));
        }
        break;
    }
    }
    previousMillisCmd = HAL::timeInMilliseconds(); //prevent inactive shutdown of steppers/temps
}

/**
\brief Execute the M command stored in com.
*/
void Commands::processMCode(GCode* com) {
    switch (com->M) {
    case 104: // M104 - set extruder temp
    {
        if (Printer::isAnyTempsensorDefect()) {
            reportTempsensorAndHeaterErrors();
            break;
        }
        previousMillisCmd = HAL::timeInMilliseconds(); //prevent inactive shutdown of steppers/temps
        if (Printer::debugDryrun())
            break;

        //TODO man müsste das in den Movecache legen!
        if (com->hasP() || (com->hasS() && com->S == 0))
            Commands::waitUntilEndOfAllMoves(); //M104

        if (com->hasS()) {
            if (com->hasT())
                Extruder::setTemperatureForExtruder(com->S, com->T);
            else
                Extruder::setTemperatureForExtruder(com->S, Extruder::current->id);
        }
        break;
    }
    case 105: // M105 - get temperature. Always returns the current temperature, doesn't wait until move stopped
    {
        printTemperatures(com->hasX());
        break;
    }
    case 140: // M140 - set bed temp
    {
        if (Printer::isAnyTempsensorDefect()) {
            reportTempsensorAndHeaterErrors();
            break;
        }
        previousMillisCmd = HAL::timeInMilliseconds(); //prevent inactive shutdown of steppers/temps
        if (Printer::debugDryrun())
            break;
        if (com->hasS())
            Extruder::setHeatedBedTemperature(com->S);
        break;
    }
    case 109: // M109 - Wait for extruder heater to reach target.
    {
        previousMillisCmd = HAL::timeInMilliseconds(); //prevent inactive shutdown of steppers/temps
        if (Printer::isAnyTempsensorDefect()) {
            reportTempsensorAndHeaterErrors();
            break;
        }
        if (Printer::debugDryrun()) {
            break;
        }

        g_uStartOfIdle = 0;                 //M109
        Commands::waitUntilEndOfAllMoves(); //M109

        Extruder* actExtruder = Extruder::current;
        if (com->hasT() && com->T < NUM_EXTRUDER)
            actExtruder = &extruder[com->T];
        if (com->hasS())
            Extruder::setTemperatureForExtruder(com->S, actExtruder->id);

        if (fabs(actExtruder->tempControl.targetTemperatureC - actExtruder->tempControl.currentTemperatureC) < TEMP_TOLERANCE) {
            break;
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


        g_uStartOfIdle = HAL::timeInMilliseconds(); //end of M109
        break;
    }
    case 190: // M190 - Wait bed for heater to reach target.
    {
        previousMillisCmd = HAL::timeInMilliseconds(); //prevent inactive shutdown of steppers/temps
        if (Printer::isAnyTempsensorDefect()) {
            reportTempsensorAndHeaterErrors();
            break;
        }
        if (Printer::debugDryrun()) {
            break;
        }
        g_uStartOfIdle = 0;                 //M190
        Commands::waitUntilEndOfAllMoves(); //M190
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

        g_uStartOfIdle = HAL::timeInMilliseconds() + 5000; //end of M190
        break;
    }
    case 116: // M116 - Wait for temperatures to reach target temperature
    {
        if (Printer::debugDryrun())
            break;
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
        break;
    }

    case 303: // M303
    {
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
        break;
    }

    case 106: // M106 - Fan On
    {
        setFanSpeed(com->hasS() ? (uint8_t)constrain(com->S, 0, 255) : (uint8_t)255);
        break;
    }
    case 107: // M107 - Fan Off
    {
        setFanSpeed(0);
        break;
    }

    case 82: // M82
    {
        Printer::relativeExtruderCoordinateMode = false;
        break;
    }
    case 83: // M83
    {
        Printer::relativeExtruderCoordinateMode = true;
        break;
    }
    case 84: // M84
    {
        if (com->hasS()) {
            stepperInactiveTime = com->S * 1000;
        } else {
            Commands::waitUntilEndOfAllMoves(); //M84
            Printer::disableAllSteppersNow();
        }
        break;
    }
    case 85: // M85
    {
        if (com->hasS())
            maxInactiveTime = (long)com->S * 1000;
        else
            maxInactiveTime = 0;
        break;
    }
    case 99: // M99 S<time>
             //Nibbels: 050118 Ich halte den Befehl für tendentiell gefährlich. Man sollte nicht abschalten, sondern Strom senken, oder hat das einen sinn? Vermutlich muss danach Homing und CMP deaktiviert werden!
    {
        millis_t wait = 10000L;
        if (com->hasS())
            wait = 1000 * com->S;
        if (com->hasX())
            Printer::disableXStepper();
        if (com->hasY())
            Printer::disableYStepper();
        if (com->hasZ())
            Printer::disableZStepper();
        wait += HAL::timeInMilliseconds();

        while (wait - HAL::timeInMilliseconds() < 100000L) {
            Commands::checkForPeriodicalActions(Paused); //check heater and other stuff every n milliseconds
        }
        if (com->hasX())
            Printer::enableXStepper();
        if (com->hasY())
            Printer::enableYStepper();
        if (com->hasZ())
            Printer::enableZStepper();
        break;
    }
    case 111: // M111
    {
        if (com->hasS()) {
            Printer::debugLevel = com->S;
        }
        if (Printer::debugDryrun()) // simulate movements without printing
        {
            Extruder::setTemperatureForAllExtruders(0);
        }
        break;
    }
    case 114: // M114
    {
        printCurrentPosition();
        break;
    }
    case 115: // M115
    {
        if (Printer::debugInfo()) {
            Com::printFLN(Com::tFirmware);
        }
        //		Com::cap(PSTR("EEPROM:1"));
        //#else
        //		Com::cap(PSTR("EEPROM:0"));
        //#endif

        //		Com::cap(PSTR("EMERGENCY_PARSER:0"));
        reportPrinterUsage();
        break;
    }
    case 117: // M117 - message to lcd
    {
        if (com->hasString()) {
            UI_STATUS_UPD_RAM(com->text);
        }
        break;
    }
    case 119: // M119
    {
        if (!Printer::debugInfo()) {
            break;
        }

        Commands::waitUntilEndOfAllMoves(); //M119

        Com::printF(Com::tXMinColon);
        Com::printF(Printer::isXMinEndstopHit() ? Com::tHSpace : Com::tLSpace);

        Com::printF(Com::tSoftDash);
        Com::printF(Com::tXMaxColon);
        Com::printF(Printer::isXMaxEndstopHit() ? Com::tHSpace : Com::tLSpace);

        Com::printF(Com::tYMinColon);
        Com::printF(Printer::isYMinEndstopHit() ? Com::tHSpace : Com::tLSpace);

        Com::printF(Com::tSoftDash);
        Com::printF(Com::tYMaxColon);
        Com::printF(Printer::isYMaxEndstopHit() ? Com::tHSpace : Com::tLSpace);

        Com::printF(Com::tZMinColon);
        Com::printF(Printer::isZMinEndstopHit() ? Com::tHSpace : Com::tLSpace);

#if MOTHERBOARD == DEVICE_TYPE_RF1000
        Com::printF(Com::tSoftDash);
#endif
        Com::printF(Com::tZMaxColon);
        Com::printF(Printer::isZMaxEndstopHit() ? Com::tHSpace : Com::tLSpace);

        Com::println();
        break;
    }

    case 201: // M201
    {
        if (com->hasX())
            Printer::maxAccelerationMMPerSquareSecond[X_AXIS] = constrain(com->X, ACCELERATION_MIN_XY, ACCELERATION_MAX_XY);
        if (com->hasY())
            Printer::maxAccelerationMMPerSquareSecond[Y_AXIS] = constrain(com->Y, ACCELERATION_MIN_XY, ACCELERATION_MAX_XY);
        if (com->hasZ())
            Printer::maxAccelerationMMPerSquareSecond[Z_AXIS] = constrain(com->Z, ACCELERATION_MIN_Z, ACCELERATION_MAX_Z);
        if (com->hasE())
            Printer::maxAccelerationMMPerSquareSecond[E_AXIS] = com->E;
        Printer::updateDerivedParameter();
        break;
    }
    case 202: // M202
    {
        if (com->hasX())
            Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS] = constrain(com->X, ACCELERATION_MIN_XY, ACCELERATION_MAX_XY);
        if (com->hasY())
            Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS] = constrain(com->Y, ACCELERATION_MIN_XY, ACCELERATION_MAX_XY);
        if (com->hasZ())
            Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS] = constrain(com->Z, ACCELERATION_MIN_Z, ACCELERATION_MAX_Z);
        if (com->hasE())
            Printer::maxTravelAccelerationMMPerSquareSecond[E_AXIS] = com->E;
        Printer::updateDerivedParameter();
        break;
    }

    case 203: // M203 - Temperature monitor
    {
        if (com->hasS()) {
            if (com->S < NUM_EXTRUDER)
                manageMonitor = com->S;
            else
                manageMonitor = NUM_EXTRUDER; // Set 100 to heated bed
        }
        break;
    }
    case 204: // M204
    {
        TemperatureController* temp = &Extruder::current->tempControl;
        if (com->hasS()) {
            if (com->S < 0)
                break;
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
        break;
    }
    case 503: // M503 (fall through)
    case 205: // M205 - Show EEPROM settings
    {
        EEPROM::writeSettings();
        break;
    }
    case 206: // M206 - T[type] P[pos] [Sint(long] [Xfloat]  Set eeprom value
    {
        EEPROM::update(com);
        break;
    }
    case 207: // M207 - X<XY jerk> Z<Z Jerk> E<E Jerk>
    {
        // Die Minimalwerte für XYZ sind hier eigentlich egal. Denn sie werden durch die Acceleration innerhalb updateDerivedParameter() auf ein Minimum angehoben.
        if (com->hasX()) {
            Printer::maxXYJerk = constrain(com->X, 0.001f, 33.3f);
            Com::printFLN(Com::tXYJerkColon, Printer::maxXYJerk);
            Printer::updateDerivedParameter();
        }
        if (com->hasZ()) {
            Printer::maxZJerk = constrain(com->Z, 0.001f, 2.0f);
            Com::printFLN(Com::tZJerkColon, Printer::maxZJerk);
            Printer::updateDerivedParameter();
        }

        // Das ist die E-Startgeschwindigkeit. Quasi dasselbe wie Jerk für den Extruder.
        if (com->hasE()) {
            Extruder::current->maxEJerk = constrain(com->E, 1.0f, Extruder::current->maxFeedrate);
            Extruder::selectExtruderById(Extruder::current->id);
        }

        break;
    }
    case 218: {
        // New MCode with https://github.com/repetier/Repetier-Firmware/commit/e5db16080d0c98776ae82f543e2bc6ef643a63c7
        // I added this MCode for compatibility-Reasons with Repetier and Marlin.

        int extId = 0;
        if (com->hasT()) {
            extId = com->T;
        }
        if (extId >= 0 && extId < NUM_EXTRUDER) {
            Commands::waitUntilEndOfAllMoves(); //M218
            int32_t dx = 0;
            int32_t dy = 0;
            if (com->hasX()) {
                if (Printer::isAxisHomed(X_AXIS))
                    dx = (com->X - extruder[extId].offsetMM[X_AXIS]) * Printer::axisStepsPerMM[X_AXIS];
                extruder[extId].offsetMM[X_AXIS] = com->X;
            }
            if (com->hasY()) {
                if (Printer::isAxisHomed(Y_AXIS))
                    dy = (com->Y - extruder[extId].offsetMM[Y_AXIS]) * Printer::axisStepsPerMM[Y_AXIS];
                extruder[extId].offsetMM[Y_AXIS] = com->Y;
            }

            Printer::offsetRelativeStepsCoordinates(-dx, -dy, 0, 0);

            if (com->hasS() && com->S > 0) {
                if (com->hasX())
                    HAL::eprSetFloat(EEPROM::getExtruderOffset(extId) + EPR_EXTRUDER_X_OFFSET, com->X);
                if (com->hasY())
                    HAL::eprSetFloat(EEPROM::getExtruderOffset(extId) + EPR_EXTRUDER_Y_OFFSET, com->Y);
                EEPROM::updateChecksum();
            }
        }
        break;
    }
    case 220: // M220 - S<Feedrate multiplier in percent>
    {
        changeFeedrateMultiply(com->getS(100));
        break;
    }
    case 221: // M221 - S<Extrusion flow multiplier in percent>
    {
        if (com->hasS() && com->S > 0) {
            changeFlowrateMultiply(static_cast<float>(com->S) * 0.01f);
        } else {
            changeFlowrateMultiply(1.0f);
        }
        break;
    }

    case 223: // M223 - Extruder interrupt test
    {
        if (com->hasS()) {
            InterruptProtectedBlock noInts; //BEGIN_INTERRUPT_PROTECTED
            Printer::extruderStepsNeeded += com->S;
        }
        break;
    }
    case 232: // M232
    {
        if (Printer::debugInfo()) {
            Com::printF(Com::tLinearStepsColon, maxadv2);
            Com::printFLN(Com::tCommaSpeedEqual, maxadvspeed);
        }
        maxadv2 = 0;
        maxadvspeed = 0;
        break;
    }
    case 233: // M233
    {
        if (com->hasY())
            Extruder::current->advanceL = com->Y;
        Com::printF(Com::tLinearLColon, Extruder::current->advanceL);
        Com::println();
        Printer::updateAdvanceActivated();
        break;
    }

    case 360: // M360 - show configuration
        Printer::showConfiguration();
        break;

    case 355: // M355  - Turn case light on/off / Turn X19 on and off.
    {
#if HOTEND_COOLER_USES_PIN(CASE_LIGHT_PIN)
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
        break;
    }
    // Ende Idee und Teilcode von WESSIX
    case 400: // M400 - Finish all moves
    {
        Commands::waitUntilEndOfAllMoves(); //M400 (normal gcode wait)
        break;
    }

    case 908: // M908 - Control digital trimpot directly.
    {
        if (com->hasS()) {
            uint8_t current = com->S;
            if (current > 150) {
                break;
            }
            if (current < MOTOR_CURRENT_MIN) {
                break;
            }

            uint8_t steppernr = 255; //fails
            if (com->hasP()) {
                steppernr = (uint8_t)com->P;
            }
            if (com->hasX()) {
                steppernr = 0;
            }
            if (com->hasY()) {
                steppernr = 1;
            }
            if (com->hasZ()) {
                steppernr = 2;
            }
            if (com->hasE() && com->E == 0) { //E0
                steppernr = 4;
            }
            if (com->hasE() && com->E == 1) { //E1
                steppernr = 5;
            }

            if (steppernr > 3 + NUM_EXTRUDER) {
                break; // Axis number too high for setup
            }

            setMotorCurrent(steppernr, current);
        }
        break;
    }
    case 500: // M500
    {
        EEPROM::storeDataIntoEEPROM(false);
        Com::printInfoF(Com::tConfigStoredEEPROM);
        break;
    }
    case 501: // M501
    {
        EEPROM::readDataFromEEPROM();
        Extruder::selectExtruderById(Extruder::current->id);

        if (Printer::debugInfo()) {
            Com::printInfoF(Com::tConfigLoadedEEPROM);
        }
        break;
    }
    case 502: // M502
    {
        EEPROM::restoreEEPROMSettingsFromConfiguration();
        EEPROM::storeDataIntoEEPROM(false);

        UI_STATUS(UI_TEXT_RESTORE_DEFAULTS);
        showInformation(PSTR(UI_TEXT_CONFIGURATION), PSTR(UI_TEXT_RESTORE_DEFAULTS), PSTR(UI_TEXT_OK));
        break;
    }

    default: {
        // we may have to process RF specific commands
        processSpecialGCode(com);
        break;
    }
    }
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

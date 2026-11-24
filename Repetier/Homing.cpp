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

int8_t Printer::anyHomeDir(uint8_t axis) {
    return axis <= Z_AXIS ? -1 : 0;
}

bool Printer::anyEndstop(uint8_t axis) {
    int endstop = 0;
    for (uint8_t i = 1; i <= 3; i++) {
        switch (axis) {
        case X_AXIS: {
            endstop += (Printer::isXMinEndstopHit() || Printer::isXMaxEndstopHit());
            break;
        }
        case Y_AXIS: {
            endstop += (Printer::isYMinEndstopHit() || Printer::isYMaxEndstopHit());
            break;
        }
        case Z_AXIS: {
            endstop += (Printer::isZMinEndstopHit() || Printer::isZMaxEndstopHit());
            break;
        }
        }
    }
    return (bool)(endstop > 1); //endstop muss sauber gedrückt sein.
}

void Printer::changeAxisDirection(uint8_t axis, int8_t direction) {
    switch (axis) {
    case X_AXIS: {
        Printer::setXDirection((direction > 0 ? true : false));
        break;
    }
    case Y_AXIS: {
        Printer::setYDirection((direction > 0 ? true : false));
        break;
    }
    case Z_AXIS: {
        ZMotion::setDirection(direction > 0);
        break;
    }
    }
}

void Printer::stepAxisStep(uint8_t axis, uint8_t slower) {
    if (axis == Z_AXIS) {
        zMotion.stepHomeCheck(slower);
        return;
    }
    HAL::delayMicroseconds(XYZ_STEPPER_HIGH_DELAY * slower);
    if (axis == X_AXIS)
        startXStep();
    else if (axis == Y_AXIS)
        startYStep();
    HAL::delayMicroseconds(XYZ_STEPPER_LOW_DELAY * slower);
    if (axis == X_AXIS)
        endXStep();
    else if (axis == Y_AXIS)
        endYStep();
}

int8_t Printer::checkHome(int8_t axis) //X_AXIS 0, Y_AXIS 1, Z_AXIS 2
{
    //do not allow Z jet because we have to check compensation behaviour
    Com::printFLN(PSTR("checkHome: start axis"), axis);
    //if(axis == Z_AXIS) return -1; //noch verboten, weil solange das läuft der emergency-Stop nicht funktioniert. funktion wird evtl. wie HBS oder ZOS geteilt.

    if (axis > Z_AXIS)
        return -1;

    //do not allow checkHome without Home
    if (!Printer::isAxisHomed(axis))
        return -1;
    Com::printFLN(PSTR("ishomed!"));

    //do not allow checkHome when some endstop is already reached
    if (Printer::anyEndstop(axis))
        return -1; //knopf gedrückt = kein sinnvoller rückfahrweg bzw. nicht vertrauenswürdige coordinaten.

    //get the right homeDirection for moving -> 0 is none.
    char nHomeDir = Printer::anyHomeDir(axis);
    Com::printFLN(PSTR("nHomeDir="), nHomeDir);
    if (!nHomeDir)
        return -1;

    //set stepper direction as homing direction
    Printer::changeAxisDirection(axis, nHomeDir);

    //previous motion state:
    long oldCurrentSteps;
    if (axis == Z_AXIS)
        oldCurrentSteps = zMotion.motorSteps();
    else if (axis == Y_AXIS)
        oldCurrentSteps = Printer::xyMotion.motorSteps(Y_AXIS);
    else
        oldCurrentSteps = Printer::xyMotion.motorSteps(X_AXIS);
    //write previous motion state to log
    Com::printFLN(PSTR("Current position steps="), oldCurrentSteps);

    //drive axis towards endstop:
    long returnSteps = 0;
    bool finish = false;
    int mmLoops = Printer::stepsPerMm(axis); //fahre in mm-Blöcken.

    while (1) {
        Commands::checkForPeriodicalActions(Calibrating);

        for (int i = 0; i < mmLoops; i++) {
            Printer::stepAxisStep(axis);
            if (Printer::anyEndstop(axis)) {
                finish = true;
            } else {
                returnSteps += nHomeDir;
            }
            //verbot unendlich weiterzufahren, wenn endstop kaputt
            if (abs(returnSteps) > abs(softwareEndstopSteps(axis)) * 110 / 100)
                finish = true;

            if (finish)
                break;
        }
        if (finish)
            break;
    }

    //steps to log
    Com::printFLN(PSTR("Return steps="), returnSteps);

    Com::printFLN(PSTR("Recheck endstop:"));
    //check ob endstop da (oder defekt oder nicht erreicht: return)
    if (!Printer::anyEndstop(axis))
        return -1;

    //change stepper direction to drive against homing direction
    Printer::changeAxisDirection(axis, -1 * nHomeDir);
    //Ein paar steps zurück, bis schalter wieder losgelassen wurde:
    finish = false; // Neue Abbruchbedingung
    for (int i = 0; i < mmLoops; i++) {
        Printer::stepAxisStep(axis, 20 /*20x langsamer*/);
        //end if button is not pressed anymore.
        if (!Printer::anyEndstop(axis)) {
            finish = true;
        } else {
            returnSteps += -1 * nHomeDir;
        }
        if (finish)
            break;
    }

    //check ob endstop nach 1mm immernoch gedrückt:
    if (Printer::anyEndstop(axis))
        return -1;

    //merke schalter-aus-punkt für hysteresemessung:
    long hysterese = returnSteps;

    //Neu und langsamer in den Schalter fahren:
    //change stepper direction to drive against homing direction
    Printer::changeAxisDirection(axis, nHomeDir);
    finish = false; // Neue Abbruchbedingung
    for (int i = 0; i < mmLoops; i++) {
        Printer::stepAxisStep(axis, 20 /*20x langsamer*/);
        //end if button is not pressed anymore.
        if (Printer::anyEndstop(axis)) {
            finish = true;
        } else {
            returnSteps += nHomeDir;
        }
        if (finish)
            break;
    }
    //check ob endstop nach 1mm immernoch nicht gedrückt:
    if (!Printer::anyEndstop(axis))
        return -1;

    hysterese -= returnSteps;
    Com::printFLN(PSTR("aus-ein-hysterese="), hysterese);

    //check ob steps verloren gehen:
    long delta;
    if (axis == Z_AXIS)
        delta = zMotion.motorSteps();
    else if (axis == Y_AXIS)
        delta = Printer::xyMotion.motorSteps(Y_AXIS);
    else
        delta = Printer::xyMotion.motorSteps(X_AXIS);

    Com::printFLN(PSTR("Steps delta="), delta);

    //fahre zurück an Startpunkt:
    Printer::changeAxisDirection(axis, -1 * nHomeDir);

    returnSteps = abs(returnSteps);
    while (returnSteps > 0) {
        Commands::checkForPeriodicalActions(Calibrating);

        if (returnSteps >= mmLoops) {
            returnSteps -= mmLoops;
        } else {
            mmLoops = returnSteps;
            returnSteps = 0;
        }

        for (int i = 0; i < mmLoops; i++) {
            Printer::stepAxisStep(axis);
        }
    }

    return 1;
} // checkHome

void Printer::homeXAxis() {
    char nHomeDir = Printer::anyHomeDir(X_AXIS);

    if (nHomeDir) {
        UI_STATUS_UPD(UI_TEXT_HOME_X);
        Printer::resetDirectAxis(X_AXIS);

        int32_t offX = 0;
#if NUM_EXTRUDER > 1
        // Reposition extruder that way, that all extruders can be selected at home pos.
        for (uint8_t i = 0; i < NUM_EXTRUDER; i++)
            offX = (nHomeDir == -1)
                ? RMath::max(offX, int32_t(extruder[i].offsetMM[X_AXIS] * Printer::stepsPerMm(X_AXIS)))
                : RMath::min(offX, int32_t(extruder[i].offsetMM[X_AXIS] * Printer::stepsPerMm(X_AXIS)));
#endif // NUM_EXTRUDER>1


        if (!Printer::queueRelativeStepsCoordinates(long(2 * abs(maxSoftEndstopSteps_[X_AXIS] + 2 * offX) * nHomeDir), 0, 0, 0, homingFeedrate_[X_AXIS], false, true))
            return;
        Printer::queueRelativeMMCoordinates(-1 * ENDSTOP_X_BACK_MOVE * nHomeDir, 0, 0, 0, homingFeedrate_[X_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, false);
        if (!Printer::queueRelativeMMCoordinates(2 * ENDSTOP_X_BACK_MOVE * nHomeDir, 0, 0, 0, homingFeedrate_[X_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, true)) {
            setHomed(false, -1, -1);
            return;
        }

        Printer::xyMotion.setMotorSteps(X_AXIS, (nHomeDir == -1) ? 0 : maxSoftEndstopSteps_[X_AXIS]);

#if NUM_EXTRUDER > 1
        if (offX) {
            //Wir halten z.B. von Links soviel Abstand wie der größte Extruder-Offset. Wir fahren aber nur so weit, dass das aktuelle Hotend dort steht.
            Printer::offsetRelativeStepsCoordinates(-1 * nHomeDir * (offX * axisMMPerSteps_[X_AXIS] - Extruder::current->offsetMM[X_AXIS]) * Printer::stepsPerMm(X_AXIS), 0, 0, 0);
        }
#endif // NUM_EXTRUDER>1

        // Set axis value here.
        Printer::setXAxisSteps((nHomeDir == -1) ? 0 : maxSoftEndstopSteps_[X_AXIS]);

        // show that we are active
        Printer::inactivity.recordActivity(HAL::timeInMilliseconds());
        setHomed(true, -1, -1);
        UI_STATUS_UPD("");
    }
} // homeXAxis

void Printer::homeYAxis() {
    char nHomeDir = Printer::anyHomeDir(Y_AXIS);

    if (nHomeDir) {
        UI_STATUS_UPD(UI_TEXT_HOME_Y);
        Printer::resetDirectAxis(Y_AXIS);

        int32_t offY = 0;
#if NUM_EXTRUDER > 1
        // Reposition extruder that way, that all extruders can be selected at home pos.
        for (uint8_t i = 0; i < NUM_EXTRUDER; i++)
            offY = (nHomeDir == -1)
                ? RMath::max(offY, int32_t(extruder[i].offsetMM[Y_AXIS] * Printer::stepsPerMm(Y_AXIS)))
                : RMath::min(offY, int32_t(extruder[i].offsetMM[Y_AXIS] * Printer::stepsPerMm(Y_AXIS)));
#endif // NUM_EXTRUDER>1


        if (!Printer::queueRelativeStepsCoordinates(0, long(2 * abs(maxSoftEndstopSteps_[Y_AXIS] + 2 * offY) * nHomeDir), 0, 0, homingFeedrate_[Y_AXIS], false, true))
            return;
        Printer::queueRelativeMMCoordinates(0, -1 * ENDSTOP_Y_BACK_MOVE * nHomeDir, 0, 0, homingFeedrate_[Y_AXIS] / ENDSTOP_Y_RETEST_REDUCTION_FACTOR, false);
        if (!Printer::queueRelativeMMCoordinates(0, 2 * ENDSTOP_Y_BACK_MOVE * nHomeDir, 0, 0, homingFeedrate_[Y_AXIS] / ENDSTOP_Y_RETEST_REDUCTION_FACTOR, true, true)) {
            setHomed(-1, false, -1);
            return;
        }

        Printer::xyMotion.setMotorSteps(Y_AXIS, (nHomeDir == -1) ? 0 : maxSoftEndstopSteps_[Y_AXIS]);
#if NUM_EXTRUDER > 1
        if (offY) {
            //Wir halten z.B. von Links soviel Abstand wie der größte Extruder-Offset. Wir fahren aber nur so weit, dass das aktuelle Hotend dort steht.
            Printer::offsetRelativeStepsCoordinates(0, -1 * nHomeDir * (offY * axisMMPerSteps_[Y_AXIS] - Extruder::current->offsetMM[Y_AXIS]) * Printer::stepsPerMm(Y_AXIS), 0, 0);
        }
#endif // NUM_EXTRUDER>1

        // Set axis value here.
        Printer::setYAxisSteps((nHomeDir == -1) ? 0 : maxSoftEndstopSteps_[Y_AXIS]);

        // show that we are active
        Printer::inactivity.recordActivity(HAL::timeInMilliseconds());
        setHomed(-1, true, -1);
        UI_STATUS_UPD("");
    }
} // homeYAxis

void Printer::homeZAxis() {
    char nHomeDir = Printer::anyHomeDir(Z_AXIS);

    if (nHomeDir) {
        UI_STATUS_UPD(UI_TEXT_HOME_Z);

        //homing ausschalten und zCMP (...) auch.
        setHomed(-1, -1, false);

        // Establish the first-layer defaults for the print that follows this Z home.
        layers.resetFadeBounds(axisStepsPerMM_[Z_AXIS]);
        layers.resetTracking();
        zOffset.clearSenseOffset(axisStepsPerMM_[Z_AXIS]);
        senseOffsetControl.setThresholdDigits(0);
        senseOffsetControl.resetWindow();

        InterruptProtectedBlock noInts;
        extrusionCompensation.reset();
        zMotion.resetDirectPosition();

        // Homing replaces the reference, including any correction that could not unwind at a limit.
        zMotion.clearScanAndCorrection();
        noInts.unprotect();

        //1. Schnelles Fahren bis zum Schalterkontakt:
        //Ist der Schalter gedrückt, wird sofort geskipped.
        if (!Printer::queueRelativeStepsCoordinates(0, 0, (zMotion.maxSteps() + 2 * int32_t(zMotion.minOvertravel())) * nHomeDir, 0, homingFeedrate_[Z_AXIS], true, true))
            return;

        //2. in jedem Fall Freifahren vom Schalterkontakt:
        //ENDSTOP_Z_BACK_MOVE größer als 32768 ist eigentlich nicht möglich, nicht sinnvoll und würde, da das überfahren bei 32microsteps von der z-matrix >-12,7mm abhängig ist verboten sein.
        //darum ist uint16_t in jedem fall ohne overflow.
        for (uint16_t step = 0; step < uint16_t(axisStepsPerMM_[Z_AXIS] * ENDSTOP_Z_BACK_MOVE); step += uint16_t(0.1f * axisStepsPerMM_[Z_AXIS])) {
            //faktor *2 und *5 : doppelt/5x so schnell beim Zurücksetzen als nachher beim langsamst hinfahren. Sonst dauert das ewig.
            if (Printer::isZMinEndstopHit()) {
                //schalter noch gedrückt, wir müssen weiter aus dem schalter rausfahren, aber keinesfalls mehr als ENDSTOP_Z_BACK_MOVE
                if (!Printer::queueRelativeMMCoordinates(0, 0, 0.1f * (-1 * nHomeDir), 0, float(homingFeedrate_[Z_AXIS] / ENDSTOP_Z_RETEST_REDUCTION_FACTOR * 5.0f), true))
                    return;
            } else { //wir sind aus dem schalterbereich raus, müssten also nicht weiter zurücksetzen:
                     //1) egal ob der schalter zu anfang überfahren war oder nicht: etwas zurücksetzen, nachdem der schalter angefahren wurde.
                     //2) hier wird in jedem Fall etwas weiter weggefahren, sodass man wieder neu auf Z anfahren kann.
                if (!Printer::queueRelativeMMCoordinates(0, 0, Z_ENDSTOP_MAX_HYSTERESIS * (-1 * nHomeDir), 0, float(homingFeedrate_[Z_AXIS] / ENDSTOP_Z_RETEST_REDUCTION_FACTOR * 2.0f), true))
                    return;
                break;
            }
        }

        //3. langsames Fahren bis zum Schalterkontakt:
        if (!Printer::queueRelativeMMCoordinates(0, 0, (0.1f + ENDSTOP_Z_BACK_MOVE) * nHomeDir, 0, float(homingFeedrate_[Z_AXIS] / ENDSTOP_Z_RETEST_REDUCTION_FACTOR), true, true))
            return;


        // Set the motor position and the nominal axis origin to the switch coordinate.
        const int32_t homeSteps = nHomeDir < 0 ? 0 : zMotion.maxSteps();
        {
            InterruptProtectedBlock noInts;
            zMotion.setMotorReference(homeSteps);
            setZAxisSteps(homeSteps);
        }

        //6. Korrektur der Flags
        setHomed(-1, -1, true);
        // show that we are active
        Printer::inactivity.recordActivity(HAL::timeInMilliseconds());
        UI_STATUS_UPD("");
    }
} // homeZAxis

void Printer::homeDigits() {
    if (!strainGauge.zeroingEnabled())
        return;

    // Base the new zero on uncorrected readings.
    strainGauge.clearZeroOffset();
    short average = 0;
    if (readAveragePressure(&average)) {
        Com::printFLN(PSTR("DigitOffset failed reading "));
    } else if (strainGauge.setZeroFromAverage(average)) {
        Com::printFLN(PSTR("DigitOffset = "), average);
    } else {
        Com::printFLN(PSTR("DigitOffset failed "), average);
    }
}

void Printer::homeAxis(bool xaxis, bool yaxis, bool zaxis) // home non-delta printer
{
    Printer::idleStatus.cancel();

    //Bei beliebiger user interaktion oder Homing soll G1 etc. erlaubt werden. Dann ist der Drucker nicht abgestürzt, sondern bedient worden.
    allowMovement();

    // Clear a positive bed matrix before homing Z when Y was omitted.
    if (!yaxis && zaxis && !Printer::isZHomeSafe())
        homeYAxis();

    if (xaxis)
        homeXAxis();
    if (yaxis)
        homeYAxis();
    if (zaxis)
        homeZAxis();

    if (xaxis && yaxis && zaxis)
        Printer::homeDigits();

    Printer::idleStatus.schedule(HAL::timeInMilliseconds());
    Commands::printCurrentPosition();
} // homeAxis

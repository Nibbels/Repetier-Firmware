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
#include "ScanProbe.h"

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
    return (bool)(endstop > 1); // endstop muss sauber gedrückt sein.
}

__attribute__((noinline)) int8_t Printer::checkHome(int8_t axis) {
    UIDisplay::ScopedActionExclusion panelExclusion(uid);
    if (axis < X_AXIS || axis > Z_AXIS || Printer::blockAll ||
        Printer::printJob.commandsBlocked() || !Printer::isAxisHomed(axis) ||
        Printer::anyEndstop(axis))
        return -1;

    // Preserve the historical nominal 5 kstep/s and 250 step/s requests,
    // but let the common planner enforce the configured speed and ramp.
    constexpr uint16_t fastRate = 1000000UL / (XYZ_STEPPER_HIGH_DELAY + XYZ_STEPPER_LOW_DELAY);
    constexpr uint16_t slowRate = fastRate / 20;
    static_assert(slowRate > 0, "Invalid homing test speed");
    const int32_t origin = PrintLine::guardedAxisPosition(axis);
    const int32_t travel = Printer::softwareEndstopSteps(axis);
    const float stepsPerMillimeter = Printer::stepsPerMm(axis);
    // Keep every absolute target representable, including on legacy
    // configurations, before converting the one-millimetre search cap.
    if (travel <= 0 || travel > INT32_MAX/4 || origin < -travel || origin > travel ||
        !(stepsPerMillimeter >= 1 && stepsPerMillimeter <= travel))
        return -1;
    const int32_t oneMM = static_cast<int32_t>(stepsPerMillimeter);

    Com::printFLN(PSTR("checkHome: start axis"), axis);
    Com::printFLN(PSTR("Current position steps="), origin);
    const auto first = PrintLine::runGuardedMoveTo(axis, origin - travel - travel/10,
                                                    fastRate, GuardedStop::Closed);
    if (first.status != GuardedMoveStatus::SwitchReached)
        return -1;
    Com::printFLN(PSTR("Return steps="), first.positionSteps - origin);

    const auto release = PrintLine::runGuardedMoveTo(axis, first.positionSteps + oneMM,
                                                      slowRate, GuardedStop::Open);
    if (release.status != GuardedMoveStatus::SwitchReached)
        return -1;

    const auto retest = PrintLine::runGuardedMoveTo(axis, release.positionSteps - oneMM,
                                                     slowRate, GuardedStop::Closed);
    if (retest.status != GuardedMoveStatus::SwitchReached)
        return -1;

    // Measurements from the actual executor's motor ledger, not inferred
    // from requested distances. The return move uses the captured origin.
    Com::printFLN(PSTR("aus-ein-hysterese="), release.positionSteps - retest.positionSteps);
    Com::printFLN(PSTR("Steps delta="), retest.positionSteps - first.positionSteps);
    const auto returned = PrintLine::runGuardedMoveTo(axis, origin, fastRate, GuardedStop::Distance);
    return returned.status == GuardedMoveStatus::PositionReached ? 1 : -1;
} // checkHome

void Printer::homeXAxis() {
    char nHomeDir = Printer::anyHomeDir(X_AXIS);

    if (nHomeDir) {
        UI_STATUS_UPD(UI_TEXT_HOME_X);
        setHomed(false, -1, -1);
        Printer::resetDirectAxis(X_AXIS);

        int32_t offX = 0;
#if NUM_EXTRUDER > 1
        // Reposition extruder that way, that all extruders can be selected at home pos.
        for (uint8_t i = 0; i < NUM_EXTRUDER; i++)
            offX = (nHomeDir == -1)
                ? RMath::max(offX, int32_t(extruder[i].offsetMM[X_AXIS] * Printer::stepsPerMm(X_AXIS)))
                : RMath::min(offX, int32_t(extruder[i].offsetMM[X_AXIS] * Printer::stepsPerMm(X_AXIS)));
#endif // NUM_EXTRUDER>1


        if (!Printer::queueRelativeStepsCoordinates(long(2 * abs(maxSoftEndstopSteps_[X_AXIS] + 2 * offX) * nHomeDir), 0, 0, 0, homingFeedrate_[X_AXIS], true, true) || !Printer::isXMinEndstopHit())
            return;
        if (!Printer::queueRelativeMMCoordinates(-1 * ENDSTOP_X_BACK_MOVE * nHomeDir, 0, 0, 0, homingFeedrate_[X_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, true) || Printer::isXMinEndstopHit())
            return;
        if (!Printer::queueRelativeMMCoordinates(2 * ENDSTOP_X_BACK_MOVE * nHomeDir, 0, 0, 0, homingFeedrate_[X_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, true) || !Printer::isXMinEndstopHit()) {
            setHomed(false, -1, -1);
            return;
        }

        const int32_t homeSteps = nHomeDir < 0 ? 0 : maxSoftEndstopSteps_[X_AXIS];
        {
            InterruptProtectedBlock lock;
            if (blockAll || printJob.commandsBlocked() || !enabledStepper[X_AXIS] || !Printer::isXMinEndstopHit())
                return;
            xyMotion.setMotorSteps(X_AXIS, homeSteps);
            setXAxisSteps(homeSteps);
        }

#if NUM_EXTRUDER > 1
        if (offX) {
            // Wir halten z.B. von Links soviel Abstand wie der größte Extruder-Offset. Wir fahren aber nur so weit, dass das aktuelle Hotend dort steht.
            const int32_t delta = -1 * nHomeDir * (offX * axisMMPerSteps_[X_AXIS] - Extruder::current->offsetMM[X_AXIS]) * Printer::stepsPerMm(X_AXIS);
            Printer::offsetRelativeStepsCoordinates(delta, 0, 0, 0);
            if (PrintLine::guardedAxisPosition(X_AXIS) != homeSteps + delta)
                return;
        }
#endif // NUM_EXTRUDER>1

        // A stopped tool-offset move must not publish a homed axis.
        if (blockAll || printJob.commandsBlocked() || !enabledStepper[X_AXIS])
            return;

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
        setHomed(-1, false, -1);
        Printer::resetDirectAxis(Y_AXIS);

        int32_t offY = 0;
#if NUM_EXTRUDER > 1
        // Reposition extruder that way, that all extruders can be selected at home pos.
        for (uint8_t i = 0; i < NUM_EXTRUDER; i++)
            offY = (nHomeDir == -1)
                ? RMath::max(offY, int32_t(extruder[i].offsetMM[Y_AXIS] * Printer::stepsPerMm(Y_AXIS)))
                : RMath::min(offY, int32_t(extruder[i].offsetMM[Y_AXIS] * Printer::stepsPerMm(Y_AXIS)));
#endif // NUM_EXTRUDER>1


        if (!Printer::queueRelativeStepsCoordinates(0, long(2 * abs(maxSoftEndstopSteps_[Y_AXIS] + 2 * offY) * nHomeDir), 0, 0, homingFeedrate_[Y_AXIS], true, true) || !Printer::isYMinEndstopHit())
            return;
        if (!Printer::queueRelativeMMCoordinates(0, -1 * ENDSTOP_Y_BACK_MOVE * nHomeDir, 0, 0, homingFeedrate_[Y_AXIS] / ENDSTOP_Y_RETEST_REDUCTION_FACTOR, true, true) || Printer::isYMinEndstopHit())
            return;
        if (!Printer::queueRelativeMMCoordinates(0, 2 * ENDSTOP_Y_BACK_MOVE * nHomeDir, 0, 0, homingFeedrate_[Y_AXIS] / ENDSTOP_Y_RETEST_REDUCTION_FACTOR, true, true) || !Printer::isYMinEndstopHit()) {
            setHomed(-1, false, -1);
            return;
        }

        const int32_t homeSteps = nHomeDir < 0 ? 0 : maxSoftEndstopSteps_[Y_AXIS];
        {
            InterruptProtectedBlock lock;
            if (blockAll || printJob.commandsBlocked() || !enabledStepper[Y_AXIS] || !Printer::isYMinEndstopHit())
                return;
            xyMotion.setMotorSteps(Y_AXIS, homeSteps);
            setYAxisSteps(homeSteps);
        }
#if NUM_EXTRUDER > 1
        if (offY) {
            // Wir halten z.B. von Links soviel Abstand wie der größte Extruder-Offset. Wir fahren aber nur so weit, dass das aktuelle Hotend dort steht.
            const int32_t delta = -1 * nHomeDir * (offY * axisMMPerSteps_[Y_AXIS] - Extruder::current->offsetMM[Y_AXIS]) * Printer::stepsPerMm(Y_AXIS);
            Printer::offsetRelativeStepsCoordinates(0, delta, 0, 0);
            if (PrintLine::guardedAxisPosition(Y_AXIS) != homeSteps + delta)
                return;
        }
#endif // NUM_EXTRUDER>1

        // A stopped tool-offset move must not publish a homed axis.
        if (blockAll || printJob.commandsBlocked() || !enabledStepper[Y_AXIS])
            return;

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

        // Homing ausschalten und zCMP (...) auch.
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
        // Ist der Schalter gedrückt, wird sofort geskipped.
        if (!Printer::queueRelativeStepsCoordinates(0, 0, (zMotion.maxSteps() + 2 * int32_t(zMotion.minOvertravel())) * nHomeDir, 0, homingFeedrate_[Z_AXIS], true, true) || !Printer::isZMinEndstopHit())
            return;

        // Count bounded release attempts in millimetres, not a truncated
        // uint16_t step increment (which could become zero or wrap).
        const uint16_t releaseAttempts = static_cast<uint16_t>(ceilf(ENDSTOP_Z_BACK_MOVE / 0.1f));
        for (uint16_t attempt = 0; attempt < releaseAttempts; ++attempt) {
            // Faktor *2 und *5 : doppelt/5x so schnell beim Zurücksetzen als nachher beim langsamst hinfahren. Sonst dauert das ewig.
            if (Printer::isZMinEndstopHit()) {
                // Schalter noch gedrückt, wir müssen weiter aus dem schalter rausfahren, aber keinesfalls mehr als ENDSTOP_Z_BACK_MOVE
                if (!Printer::queueRelativeMMCoordinates(0, 0, 0.1f * (-1 * nHomeDir), 0, float(homingFeedrate_[Z_AXIS] / ENDSTOP_Z_RETEST_REDUCTION_FACTOR * 5.0f), true))
                    return;
            } else { // Wir sind aus dem schalterbereich raus, müssten also nicht weiter zurücksetzen:
                     //1) egal ob der schalter zu anfang überfahren war oder nicht: etwas zurücksetzen, nachdem der schalter angefahren wurde.
                     //2) hier wird in jedem Fall etwas weiter weggefahren, sodass man wieder neu auf Z anfahren kann.
                if (!Printer::queueRelativeMMCoordinates(0, 0, Z_ENDSTOP_MAX_HYSTERESIS * (-1 * nHomeDir), 0, float(homingFeedrate_[Z_AXIS] / ENDSTOP_Z_RETEST_REDUCTION_FACTOR * 2.0f), true))
                    return;
                break;
            }
        }

        // A release must have actually opened the switch before retesting.
        if (Printer::isZMinEndstopHit())
            return;

        //3. langsames Fahren bis zum Schalterkontakt:
        if (!Printer::queueRelativeMMCoordinates(0, 0, (0.1f + ENDSTOP_Z_BACK_MOVE) * nHomeDir, 0, float(homingFeedrate_[Z_AXIS] / ENDSTOP_Z_RETEST_REDUCTION_FACTOR), true, true) || !Printer::isZMinEndstopHit())
            return;


        // Set the motor position and the nominal axis origin to the switch coordinate.
        const int32_t homeSteps = nHomeDir < 0 ? 0 : zMotion.maxSteps();
        {
            InterruptProtectedBlock noInts;
            if (blockAll || printJob.commandsBlocked() || !zStepperEnabled() || !Printer::isZMinEndstopHit())
                return;
            zMotion.setMotorReference(homeSteps);
            setZAxisSteps(homeSteps);
            // Publish the homed bit in the same verified reference transaction.
            zMotion.setHomed(true);
        }

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

__attribute__((noinline)) void Printer::homeAxis(bool xaxis, bool yaxis, bool zaxis) // home non-delta printer
{
#ifdef RF_EEPROM_ZONE_EXPERIMENT
    if(!EEPROM::settingsReady())return;
#endif
    // Host and scan homing have the same menu exclusion as panel homing.
    // Periodic thermal service and raw key sampling remain available.
    UIDisplay::ScopedActionExclusion panelExclusion(uid);
    Printer::idleStatus.cancel();

    // Drain queued, direct and Advance output before invalidating references.
    // Do not wait for a paused print to resume inside homing.
    bool ok = Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Cancel);
    if (ok)
        allowMovement();

    // Stop the sequence at the first unverified reference. In particular,
    // do not approach Z when the preceding clearance-related Y home failed.
    const bool requireY = !yaxis && zaxis && !Printer::isZHomeSafe();
    if (ok && requireY) {
        homeYAxis();
        ok = isAxisHomed(Y_AXIS);
    }
    if (ok && xaxis) {
        homeXAxis();
        ok = isAxisHomed(X_AXIS);
    }
    if (ok && yaxis) {
        homeYAxis();
        ok = isAxisHomed(Y_AXIS);
    }
    if (ok && zaxis) {
        homeZAxis();
        ok = isAxisHomed(Z_AXIS);
    }

    if (!ok) {
        lockMovement();
        if (Calibration::scanSession.dispatching())
            Calibration::scanSession.requestAbort(ScanAbort::EndstopProtection);
        Com::printFLN(PSTR("Error: Homing switch sequence failed"));
    } else if (xaxis && yaxis && zaxis) {
        Printer::homeDigits();
    }

    Printer::idleStatus.schedule(HAL::timeInMilliseconds());
    Commands::printCurrentPosition();
} // homeAxis

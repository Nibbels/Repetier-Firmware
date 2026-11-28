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

// Motion and extrusion G-code handlers; planning and pulse execution are delegated to motion owners.

#include "Repetier.h"
#include "GCodeCommands.h"
#include "MotionPlanner.h"

namespace GCodeCommands {

void queueLinearMove(GCode* com) {
    if (isMovingAllowed(PSTR("G0/1"))) {
        Printer::queueGCodeCoordinates(com); // For X Y Z E F
    }

}

void dwellMilliseconds(uint32_t duration) {
    if (!Commands::waitForMotion(Commands::MotionWait::QueueAndZ, Commands::PauseWait::Resume))
        return;
    const uint32_t codenum = duration + HAL::timeInMilliseconds();

    while ((uint32_t)(codenum - HAL::timeInMilliseconds()) < 2000000000) {
        Commands::checkForPeriodicalActions(Processing);
        if(Printer::printJob.emergencyLocked())return;
    }
}

void dwell(GCode* com) {
    uint32_t duration = 0;
    if (com->hasP())
        duration = com->P;
    if (com->hasS())
        duration = uint32_t(com->S) * 1000UL;
    dwellMilliseconds(duration);
}

void homeAxes(GCode* com) {
    if (!isHomingAllowed(com)) {
        return;
    }
    uint8_t home_all_axis = (com->hasNoXYZ() && !com->hasE());
    if (com->hasE()) {
        Printer::setEPositionMM(0); // G92 E0
    }
    if (home_all_axis || !com->hasNoXYZ())
        Printer::homeAxis(home_all_axis || com->hasX(), home_all_axis || com->hasY(), home_all_axis || com->hasZ());
}

void selectAbsolutePositioningWithSource(bool internal) {
    Printer::gcodeModes.setRelativeXYZ(false);
    if (internal)
        Com::printInfoFLN(PSTR("Absolute positioning"));
}

void selectAbsolutePositioning(GCode* com) {
    selectAbsolutePositioningWithSource(com->internalCommand);
}

void selectRelativePositioning(GCode* com) {
    Printer::gcodeModes.setRelativeXYZ(true);
    if (com->internalCommand)
        Com::printInfoFLN(PSTR("Relative positioning"));
}

void setLogicalPosition(GCode* com) {
    if (!com->hasNoXYZ()) {
        // set the origin only in case we got any x, y and/or z offset
        float xOff = Printer::moveTarget.workOffsetMM(X_AXIS);
        float yOff = Printer::moveTarget.workOffsetMM(Y_AXIS);
        float zOff = Printer::moveTarget.workOffsetMM(Z_AXIS);

        if (com->hasX())
            xOff = com->X - Printer::moveTarget.machineMM(X_AXIS);
        if (com->hasY())
            yOff = com->Y - Printer::moveTarget.machineMM(Y_AXIS);
        if (com->hasZ())
            zOff = com->Z - Printer::moveTarget.machineMM(Z_AXIS);

        Printer::setOrigin(xOff, yOff, zOff);
    }
    if (com->hasE()) {
        Printer::setEPositionMM(com->E);
    }
}

void selectAbsoluteExtrusion() {
    Printer::gcodeModes.setRelativeEOverride(false);
}

void selectRelativeExtrusion() {
    Printer::gcodeModes.setRelativeEOverride(true);
}

void configureStepperShutdown(GCode* com) {
    if (com->hasS()) {
        Printer::inactivity.setStepperTimeout(com->S * 1000);
    } else {
        if (Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Resume))
            Printer::disableAllSteppersNow();
    }
}

void setPrintAcceleration(GCode* com) {
    if (com->hasX())
        Printer::setPrintAcceleration(X_AXIS, constrain(com->X, ACCELERATION_MIN_XY, ACCELERATION_MAX_XY));
    if (com->hasY())
        Printer::setPrintAcceleration(Y_AXIS, constrain(com->Y, ACCELERATION_MIN_XY, ACCELERATION_MAX_XY));
    if (com->hasZ())
        Printer::setPrintAcceleration(Z_AXIS, constrain(com->Z, ACCELERATION_MIN_Z, ACCELERATION_MAX_Z));
    if (com->hasE())
        Printer::setPrintAcceleration(E_AXIS, com->E);
    Printer::updateDerivedParameter();
}

void setTravelAcceleration(GCode* com) {
    if (com->hasX())
        Printer::setTravelAcceleration(X_AXIS, constrain(com->X, ACCELERATION_MIN_XY, ACCELERATION_MAX_XY));
    if (com->hasY())
        Printer::setTravelAcceleration(Y_AXIS, constrain(com->Y, ACCELERATION_MIN_XY, ACCELERATION_MAX_XY));
    if (com->hasZ())
        Printer::setTravelAcceleration(Z_AXIS, constrain(com->Z, ACCELERATION_MIN_Z, ACCELERATION_MAX_Z));
    if (com->hasE())
        Printer::setTravelAcceleration(E_AXIS, com->E);
    Printer::updateDerivedParameter();
}

void setFeedrateMultiplier(GCode* com) {
    Commands::changeFeedrateMultiply(com->getS(100));
}

void setFlowMultiplier(GCode* com) {
    if (com->hasS() && com->S > 0) {
        Commands::changeFlowrateMultiply(static_cast<float>(com->S) * 0.01f);
    } else {
        Commands::changeFlowrateMultiply(1.0f);
    }
}

void configureAdvance(GCode* com) {
    if (com->hasY() && com->Y != Extruder::current->advanceL && Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Resume)) {
        Extruder::current->advanceL = com->Y;
        Printer::applyAdvanceSettings();
    }
    Com::printF(Com::tLinearLColon, Extruder::current->advanceL);
    Com::println();
}

void finishMotion() {
    Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Resume);
}

void setManualZOffset(GCode* pCommand) {
    long nTemp;
    // S in micrometers, Z in millimeters
    const bool inMillimeters = pCommand->hasZ();
    if (inMillimeters) {
        constexpr float maxOffsetMm = maxManualZOffsetMicrometers / 1000.0f;
        pCommand->Z = constrain(pCommand->Z, -maxOffsetMm, maxOffsetMm);
        nTemp = long(pCommand->Z * 1000);
    } else if (pCommand->hasS()) {
        nTemp = constrain(pCommand->S, -maxManualZOffsetMicrometers, maxManualZOffsetMicrometers);
    } else {
        showInvalidSyntax(pCommand->M);
        return;
    }

    Printer::zOffset.setManualMicrometers(nTemp, Printer::stepsPerMm(Z_AXIS));
    EEPROM::storeManualZOffset();
}

void testHomingRepeatability(GCode* pCommand) {
    // M3028 emits foreground STEP pulses, not queued motion. Do not
    // compete with a scan, queued/direct moves or a pending Z correction.
    if (!isHomingAllowed(pCommand))
        return;
    if (Printer::blockAll || !Printer::printJob.isIdle() || Printer::pause.blocksCalibration() ||
        PrintLine::hasLines() || PrintLine::directMoveActive() ||
        Printer::doHeatBedZCompensation || !Printer::zMotion.correctionCleared()) {
        Com::printErrorFLN(PSTR("M3028 requires idle motion and cleared Z compensation"));
        return;
    }
    if (pCommand->hasX()) {
        Printer::checkHome(X_AXIS);
    }
    if (pCommand->hasY()) {
        Printer::checkHome(Y_AXIS);
    }
    if (pCommand->hasZ()) {
        Printer::checkHome(Z_AXIS);
    }
}

void requestPrintPause(GCode* pCommand) {
    const bool park = pCommand->hasS() && pCommand->S >= 2 && Printer::areAxisHomed();
    Printer::pause.queueRequest(park ? PrintPause::Destination::Park : PrintPause::Destination::InPlace);
}

void setPauseParkingOffsets(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasNoXYZ() && !pCommand->hasE()) {
        showInvalidSyntax(pCommand->M);
    } else {
        if (pCommand->hasX()) {
            // test and take over the specified value
            nTemp = pCommand->X * Printer::stepsPerMm(X_AXIS);
            if (nTemp < -Printer::xyMaxSteps(X_AXIS))
                nTemp = -Printer::xyMaxSteps(X_AXIS);
            if (nTemp > Printer::xyMaxSteps(X_AXIS))
                nTemp = Printer::xyMaxSteps(X_AXIS);

            Printer::pause.setParkSteps(X_AXIS, nTemp);
            Com::printF(PSTR("M3105: new x pause offset: "), nTemp * Printer::mmPerStep(X_AXIS));
            Com::printFLN(PSTR(" [mm]"));
        }
        if (pCommand->hasY()) {
            // test and take over the specified value
            nTemp = pCommand->Y * Printer::stepsPerMm(Y_AXIS);
            if (nTemp < -Printer::xyMaxSteps(Y_AXIS))
                nTemp = -Printer::xyMaxSteps(Y_AXIS);
            if (nTemp > Printer::xyMaxSteps(Y_AXIS))
                nTemp = Printer::xyMaxSteps(Y_AXIS);

            Printer::pause.setParkSteps(Y_AXIS, nTemp);
            Com::printF(PSTR("M3105: new y pause offset: "), nTemp * Printer::mmPerStep(Y_AXIS));
            Com::printFLN(PSTR(" [mm]"));
        }
        if (pCommand->hasZ()) {
            // test and take over the specified value
            nTemp = pCommand->Z * Printer::stepsPerMm(Z_AXIS);
            if (nTemp < 0)
                nTemp = 0; // No pause within printed part.
            if (nTemp > Printer::zMotion.maxSteps())
                nTemp = Printer::zMotion.maxSteps();

            Printer::pause.setParkSteps(Z_AXIS, nTemp);
            Com::printF(PSTR("M3105: new z pause offset: "), nTemp * Printer::mmPerStep(Z_AXIS));
            Com::printFLN(PSTR(" [mm]"));
        }
        if (pCommand->hasE()) {
            // test and take over the specified value
            nTemp = pCommand->E;
            if (nTemp < 0)
                nTemp = 0;
            if (nTemp > 5)
                nTemp = 5; // Zahl immer positiv, sie wird abgezogen = Retract!

            Printer::pause.setParkSteps(E_AXIS, long(nTemp * Extruder::current->stepsPerMM));
            Com::printF(PSTR("M3105: new extruder pause offset: "), nTemp);
            Com::printFLN(PSTR(" [mm]"));
        }
    }
}

void setEmergencyPauseDigits(GCode* pCommand) {
    const auto limits = Printer::pressureProtection.pauseLimits();
    long nMin = limits.minimum;
    long nMax = limits.maximum;

    if (pCommand->hasS()) {
        // test and take over the specified value - this is our new min value
        nMin = pCommand->S;
    }
    if (pCommand->hasP()) {
        // test and take over the specified value - this is our new max value
        nMax = pCommand->P;
    }

    if (nMin < EMERGENCY_PAUSE_DIGITS_MIN)
        nMin = EMERGENCY_PAUSE_DIGITS_MIN;
    if (nMax > EMERGENCY_PAUSE_DIGITS_MAX)
        nMax = EMERGENCY_PAUSE_DIGITS_MAX;

    const bool disable = nMin == 0 && nMax == 0;
    if (disable || nMin < nMax) {
        Printer::pressureProtection.setPauseLimits({int16_t(nMin), int16_t(nMax)});
    } else {
        Com::printErrorFLN(PSTR("M3075: invalid pause band"));
    }
}

void setSenseOffsetParameters(GCode* pCommand) {
    auto& controller = Printer::senseOffsetControl;
    const auto stored = controller.storedLimits();
    int16_t digits = stored.thresholdDigits, maximum = stored.maxMicrometers;
    bool saveDigits = false, saveMaximum = false;
    bool error = false;
    // Statusänderung P10000 ([digits])
    if (pCommand->hasP()) {
        if (pCommand->P > 32767)
            pCommand->P = 32767;
        if (pCommand->P >= 0 && pCommand->P < EMERGENCY_PAUSE_DIGITS_MAX) {
            if (pCommand->P) {
                Com::printFLN(PSTR("M3909: SensiblePressure Pmax="), pCommand->P);
                digits = int16_t(pCommand->P);
                saveDigits = true;
                if (stored.thresholdDigits != digits)
                    Com::printFLN(PSTR("M3909: P change"));
            } else {
                controller.setThresholdDigits(0);
                error = true;
                Com::printFLN(PSTR("M3909: SensiblePressure disabled"));
            }
        } else {
            Com::printFLN(PSTR("M3909: wrong [P]"));
            error = true;
        }
    }

    if (pCommand->hasS()) {
        if (pCommand->S > 0 && pCommand->S <= 300) {
            const int16_t maxMicrometers = pCommand->S;
            controller.setMaxMicrometers(maxMicrometers);
            Com::printFLN(PSTR("M3909: SensiblePressure max. [S]Offset changed to "), maxMicrometers);
            maximum = maxMicrometers;
            saveMaximum = true;
            if (stored.maxMicrometers != maximum)
                Com::printFLN(PSTR("M3909: S change"));
        } else {
            Com::printFLN(PSTR("M3909: ERROR::>0.3mm=S300 - This function is ment to compensate minimal amounts of too close distance."));
            Com::printFLN(PSTR("M3909: INFO::If you have to auto-compensate your offset to high, clean your nozzle, rise your temp, lower your speed."));
            Com::printFLN(PSTR("M3909: INFO::This function should lower the chance of an accidential emergency block on the first layer. It cannot help you to avoid calibration!"));
            error = true;
        }
    }

    if (saveDigits || saveMaximum)
        EEPROM::storeSenseOffsetSettings(digits, maximum, saveDigits, saveMaximum);
    if (Printer::printJob.emergencyLocked()) return;

    // Statusausgabe per M3909
    const auto limits = controller.limits();
    if (limits.thresholdDigits) {
        Com::printF(PSTR("M3909: INFO SensiblePressure active. [P]PressureDigit = +-"), limits.thresholdDigits);
        Com::printFLN(PSTR(" [digits] (Standard: use `standard print digits` +20%)"));
        Com::printF(PSTR("M3909: INFO SensiblePressures [S]max. offset is "), limits.maxMicrometers);
        Com::printFLN(PSTR(" [um] (Standard: 180um, Max: 300um)"));
    } else {
        Com::printFLN(PSTR("M3909: INFO SensiblePressure is currently disabled."));
    }

    if (!error)
        queueTask(QueueTask::EnableSenseOffset);
}

void setDigitFlowParameters(GCode* pCommand) {
    auto profile = Printer::digitCompensation.flowProfile();
    if (pCommand->hasS() && pCommand->hasP()) {
        short min = abs(static_cast<short>(pCommand->S));
        short max = abs(static_cast<short>(pCommand->P));
        if (min > max) {
            min = abs(static_cast<short>(pCommand->P));
            max = abs(static_cast<short>(pCommand->S));
        }
        if (min < 500)
            min = 500; // Weniger ist in jedem fall sinnfrei, selbst mit ninjaflex und digit homing
        if (max < min)
            max = min; // Geht: flowsprung.
        profile.minDigits = min;
        profile.maxDigits = max;
    }

    if (pCommand->hasE()) {
        int8_t e = static_cast<int8_t>(pCommand->E);
        if (e > 99)
            e = 99;
        if (e < -99)
            e = -99;
        profile.extrusionPercent = e;
    }

    if (pCommand->hasF()) {
        int8_t f = static_cast<int8_t>(pCommand->F);
        if (f > 99)
            f = 99;
        if (f < -90)
            f = -90;
        profile.speedPercent = f;
    }

    Printer::digitCompensation.setFlowProfile(profile);
    Com::printFLN(PSTR("[S|P] Flow CMP min: "), profile.minDigits);
    Com::printFLN(PSTR("[S|P] Flow CMP max: "), profile.maxDigits);
    Com::printFLN(PSTR("[E]   Flow CMP %: "), profile.extrusionPercent);
    Com::printFLN(PSTR("[F]   Feed CMP %: "), profile.speedPercent);
}

} // namespace GCodeCommands

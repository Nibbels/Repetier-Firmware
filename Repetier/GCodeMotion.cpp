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

void dwell(GCode* com) {
    if (!Commands::waitForMotion(Commands::MotionWait::QueueAndZ, Commands::PauseWait::Resume))
        return;
    uint32_t codenum = 0;
    if (com->hasP())
        codenum = com->P; // milliseconds to wait
    if (com->hasS())
        codenum = com->S * 1000;          // seconds to wait
    codenum += HAL::timeInMilliseconds(); // keep track of when we started waiting

    while ((uint32_t)(codenum - HAL::timeInMilliseconds()) < 2000000000) {
        Commands::checkForPeriodicalActions(Processing);
    }
}

void selectInches() {
    Printer::gcodeModes.setInches(1);
}

void selectMillimeters() {
    Printer::gcodeModes.setInches(0);
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

void selectAbsolutePositioning(GCode* com) {
    Printer::gcodeModes.setRelativeXYZ(false);
    if (com->internalCommand)
        Com::printInfoFLN(PSTR("Absolute positioning"));
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
            xOff = Printer::convertToMM(com->X) - Printer::moveTarget.machineMM(X_AXIS);
        if (com->hasY())
            yOff = Printer::convertToMM(com->Y) - Printer::moveTarget.machineMM(Y_AXIS);
        if (com->hasZ())
            zOff = Printer::convertToMM(com->Z) - Printer::moveTarget.machineMM(Z_AXIS);

        Printer::setOrigin(xOff, yOff, zOff);
    }
    if (com->hasE()) {
        Printer::setEPositionMM(Printer::convertToMM(com->E));
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

void configureInactivityShutdown(GCode* com) {
    if (com->hasS())
        Printer::inactivity.setShutdownTimeout((long)com->S * 1000);
    else
        Printer::inactivity.setShutdownTimeout(0);
}

// Temporarily releasing an axis can lose its position. Review this command
// alongside homing and compensation when auditing supported G-code.
void suspendAxes(GCode* com) {
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
        Commands::checkForPeriodicalActions(Paused); // Check heater and other stuff every n milliseconds
    }
    if (com->hasX())
        Printer::enableXStepper();
    if (com->hasY())
        Printer::enableYStepper();
    if (com->hasZ())
        Printer::enableZStepper();
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

void setJerkLimits(GCode* com) {
    // Die Minimalwerte für XYZ sind hier eigentlich egal. Denn sie werden durch die Acceleration innerhalb updateDerivedParameter() auf ein Minimum angehoben.
    if (com->hasX()) {
        Printer::setXYJerkLimit(constrain(com->X, 0.001f, 33.3f));
        Com::printFLN(Com::tXYJerkColon, Printer::xyJerkLimit());
        Printer::updateDerivedParameter();
    }
    if (com->hasZ()) {
        Printer::setZJerkLimit(constrain(com->Z, 0.001f, 2.0f));
        Com::printFLN(Com::tZJerkColon, Printer::zJerkLimit());
        Printer::updateDerivedParameter();
    }

    // Das ist die E-Startgeschwindigkeit. Quasi dasselbe wie Jerk für den Extruder.
    if (com->hasE()) {
        Extruder::current->maxEJerk = constrain(com->E, 1.0f, Extruder::current->maxFeedrate);
        Extruder::applyCurrentSettings();
    }

}

void setToolOffset(GCode* com) {
    // New MCode with https://github.com/repetier/Repetier-Firmware/commit/e5db16080d0c98776ae82f543e2bc6ef643a63c7
    // I added this MCode for compatibility-Reasons with Repetier and Marlin.

    int extId = 0;
    if (com->hasT()) {
        extId = com->T;
    }
    if (extId >= 0 && extId < NUM_EXTRUDER) {
        if (!Commands::waitForMotion(Commands::MotionWait::QueueAndZ, Commands::PauseWait::Resume))
            return;
        int32_t dx = 0;
        int32_t dy = 0;
        if (com->hasX()) {
            if (Printer::isAxisHomed(X_AXIS))
                dx = (com->X - extruder[extId].offsetMM[X_AXIS]) * Printer::stepsPerMm(X_AXIS);
            extruder[extId].offsetMM[X_AXIS] = com->X;
        }
        if (com->hasY()) {
            if (Printer::isAxisHomed(Y_AXIS))
                dy = (com->Y - extruder[extId].offsetMM[Y_AXIS]) * Printer::stepsPerMm(Y_AXIS);
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

void testAdvanceSteps(GCode* com) {
    if (com->hasS() && Printer::extrusionOutput.advanceEnabled() && !Printer::pause.isPaused()
        && !Printer::extrusionOutput.blocked()) {
        InterruptProtectedBlock lock;
        Printer::extrusionOutput.addSteps(com->S);
    }
}

void reportAdvancePeak() {
    const auto peak = MotionPlanner::takeAdvancePeak();
    if (Printer::debugInfo()) {
        Com::printF(Com::tLinearStepsColon, peak.steps);
        Com::printFLN(Com::tCommaSpeedEqual, peak.extrusionSpeed);
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

void setStepperCurrent(GCode* com) {
    if (com->hasS()) {
        uint8_t current = com->S;
        if (current > 150) {
            return;
        }
        if (current < MOTOR_CURRENT_MIN) {
            return;
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
            return; // Axis number too high for setup
        }

        setMotorCurrent(steppernr, current);
    }
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
    if (Printer::debugInfo()) {
        if (inMillimeters) {
            Com::printF(PSTR("M3006: new static z-offset: "), pCommand->Z);
            Com::printF(PSTR(" [mm]"));
        } else {
            Com::printF(PSTR("M3006: new static z-offset: "), nTemp);
            Com::printF(PSTR(" [um]"));
        }
        Com::printF(PSTR(" / "), Printer::zOffset.effectiveSteps());
        Com::printFLN(PSTR(" [steps]"));
    }
    EEPROM::storeManualZOffset();
}

void testHomingRepeatability(GCode* pCommand) {
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

void setXYOrigin() {
    Printer::setOrigin(-Printer::moveTarget.machineMM(X_AXIS), -Printer::moveTarget.machineMM(Y_AXIS),
                       Printer::moveTarget.workOffsetMM(Z_AXIS));
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
        HAL::eprSetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMIN, nMin);
        HAL::eprSetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMAX, nMax);
        EEPROM::updateChecksum();
        if (Printer::debugInfo()) {
            if (disable) {
                Com::printFLN(PSTR("M3075: emergency pause disabled"));
            } else {
                Com::printF(PSTR("M3075: new min: "), (int)nMin);
                Com::printF(PSTR(" [digits], new max: "), (int)nMax);
                Com::printFLN(PSTR(" [digits]"));
            }
        }
    } else {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("M3075: min max ("), (int)nMin);
            Com::printF(Com::tSlash, (int)nMin);
            Com::printFLN(PSTR(") invalide"));
        }
    }
}

void setEmergencyStopDigits(GCode* pCommand) {
    const auto limits = Printer::pressureProtection.stopLimits();
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

    if (nMin == 0 && nMax == 0) {
        Printer::pressureProtection.setStopLimits({0, 0});

        if (Printer::debugInfo()) {
            Com::printFLN(PSTR("M3076: temporary stop band [0, 0] digits"));
        }
    } else if (nMin < nMax && nMin >= -32768 && nMax <= 32767) {
        Printer::pressureProtection.setStopLimits({int16_t(nMin), int16_t(nMax)});

        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3076: new min: "), (int)nMin);
            Com::printF(PSTR(" [digits], new max: "), (int)nMax);
            Com::printFLN(PSTR(" [digits]"));
        }
        HAL::eprSetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMAX, nMax);
        HAL::eprSetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMIN, nMin);
        EEPROM::updateChecksum();
    } else {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("M3076: min max ("), (int)nMin);
            Com::printF(Com::tSlash, (int)nMax);
            Com::printFLN(PSTR(") invalide"));
        }
    }
}

void setSenseOffsetParameters(GCode* pCommand) {
    auto& controller = Printer::senseOffsetControl;
    bool error = false;
    // Statusänderung P10000 ([digits])
    if (pCommand->hasP()) {
        if (pCommand->P > 32767)
            pCommand->P = 32767;
        if (pCommand->P >= 0 && pCommand->P < EMERGENCY_PAUSE_DIGITS_MAX) {
            if (pCommand->P) {
                Com::printFLN(PSTR("M3909: SensiblePressure Pmax="), pCommand->P);
                short oldval = HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS);
                if (oldval != pCommand->P) {
                    Com::printFLN(PSTR("M3909: P change"));
                    HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS, pCommand->P);
                    EEPROM::updateChecksum(); // Deshalb die prüfung
                }
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
            short oldval = HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX);
            if (oldval != maxMicrometers) {
                Com::printFLN(PSTR("M3909: S change"));
                HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX, maxMicrometers);
                EEPROM::updateChecksum(); // Deshalb die prüfung
            }
        } else {
            Com::printFLN(PSTR("M3909: ERROR::>0.3mm=S300 - This function is ment to compensate minimal amounts of too close distance."));
            Com::printFLN(PSTR("M3909: INFO::If you have to auto-compensate your offset to high, clean your nozzle, rise your temp, lower your speed."));
            Com::printFLN(PSTR("M3909: INFO::This function should lower the chance of an accidential emergency block on the first layer. It cannot help you to avoid calibration!"));
            error = true;
        }
    }

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

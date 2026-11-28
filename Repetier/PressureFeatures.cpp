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

void updateDigitCompensation(int16_t digits) {
    Printer::digitCompensation.observe(digits, Printer::doHeatBedZCompensation, [] {
        // The force brake also runs during M3912, regardless of the learned first-layer boundary.
        return Printer::zMotion.nominalSteps() > Printer::layers.fadeStartSteps() || Printer::startLine.active();
    });
}

static void serviceSenseOffset(bool sampleDue, short pressure, const PrintPause::SenseOffsetSample& pauseSample) {
    auto& controller = Printer::senseOffsetControl;
    auto& offset = Printer::zOffset;

    if (!Printer::doHeatBedZCompensation) {
        const bool wasEnabled = controller.disable();
        if (offset.senseMicrometers()) {
            Com::printF(PSTR("SensiblePressure(): offset reverted from "), offset.senseMicrometers());
            Com::printFLN(PSTR(" [um] to 0 [um] "));
            offset.clearSenseOffset(Printer::stepsPerMm(Z_AXIS));
        }
        if (wasEnabled)
            Com::printFLN(PSTR("SensiblePressure(): now disabled because of no z-CMP. "), offset.senseMicrometers());
    }

    const auto limits = controller.limits();
    const bool canLearn = limits.thresholdDigits && Printer::doHeatBedZCompensation
        && !Printer::startLine.active() && !pauseSample.wasPaused && Printer::pause.allowsSenseOffsetLearning();
    if (!canLearn || pauseSample.resetWindow)
        controller.resetWindow();
    if (!sampleDue || !canLearn)
        return;

    if (Printer::zMotion.nominalSteps() > Printer::layers.fadeStartSteps()) {
        controller.leaveLearningRange();
        return;
    }

    const uint8_t step = controller.observe(pressure, offset.senseMicrometers(), limits);
    if (!step)
        return;

    const auto increment = offset.prepareSenseIncrement(step, limits.maxMicrometers, Printer::stepsPerMm(Z_AXIS));
    // Finish the conversion before masking step interrupts.
    asm volatile("" : : "m"(increment));
    InterruptProtectedBlock lock;
    // A queued pause or settings change can arrive while calculating the increment.
    if (Printer::pause.allowsSenseOffsetLearning() && Printer::doHeatBedZCompensation && controller.limitsMatch(limits))
        offset.applySenseIncrement(increment);
}

void handleStrainGaugeFeatures(millis_t uTime) {
    const auto pauseSample = Printer::pause.beginSenseOffsetSample();
    auto& protection = Printer::pressureProtection;

    // Share one cached DMS value among the checks due on this tick.
    const bool senseOffsetDue = Printer::senseOffsetControl.takeSampleSlot(uTime);
    const bool pauseDue = protection.pauseSampleDue(uTime);
    const bool stopDue = protection.stopSampleDue(uTime);
    int16_t pressure = 0;
    if (senseOffsetDue || pauseDue || stopDue) {
        if (!Printer::strainGauge.readDigits(HAL::timeInMilliseconds(), pressure)) {
            protection.readFailed(uTime, pauseDue, stopDue);
            Printer::senseOffsetControl.resetWindow();
            serviceSenseOffset(false, 0, pauseSample);
            return;
        }
        updateDigitCompensation(pressure);
    }

    serviceSenseOffset(senseOffsetDue, pressure, pauseSample);

    if (pauseDue && protection.pauseEnabled()) {
        const bool printingHomed = !Printer::pause.isPaused() && Printer::printJob.isPrinting() && Printer::areAxisHomed();
        int16_t average;
        if (protection.checkPause(uTime, pressure, printingHomed, average)) {
            Com::printF(PSTR("emergency pause: "), average);
            showWarning((void*)ui_text_emergency_pause);
            Printer::pause.request(PrintPause::Destination::Park);
        }
    }

    if (!stopDue)
        return;
    const auto decision = protection.sampleStop(uTime, pressure);
    if (decision == PressureProtection::StopDecision::Sampling)
        return;

    const bool blocked = decision == PressureProtection::StopDecision::LimitExceeded;
    bool stopZ = false;
    {
        InterruptProtectedBlock lock;
        auto& output = Printer::extrusionOutput;
        if (blocked) {
            const int16_t discarded = output.inhibit(ExtrusionOutput::Inhibit::Pressure);
            Printer::pause.discardPendingExtrusion(discarded);
            const auto move = PrintLine::activeMove();
            stopZ = move.usesAxis(Z_AXIS) && !move.usesAxis(E_AXIS);
        } else {
            output.allow(ExtrusionOutput::Inhibit::Pressure);
        }
    }
    if (stopZ)
        doEmergencyStop(StopReason::ZBlock);
}

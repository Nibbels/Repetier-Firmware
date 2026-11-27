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

namespace Calibration {

constexpr int16_t scanOverloadDigits = 10000;

ScanSession scanSession;
ScanPressure scanPressure;
ScanProbe scanProbe;

static PressureMeasurement::Settings pressureMeasurementSettings() {
    return {static_cast<uint8_t>(probeSettings().pressureReadCount), probeSettings().pressureReadDelayMs,
            static_cast<uint16_t>(probeSettings().pressureTolerance)};
}

static ScanAbort servicePressureMeasurement(PressureMeasurement& measurement, millis_t now, bool scanning = false) {
    const auto& gauge = Printer::strainGauge;
    StrainGauge::Sample sample;
    const auto availability = gauge.latest(now, sample);
    if (availability == StrainGauge::Availability::Waiting)
        return ScanAbort::None;
    if (availability == StrainGauge::Availability::Fault) {
        measurement.readFailed();
        return ScanAbort::DmsUnavailable;
    }
    const int16_t digits = gauge.correctedDigits(sample);
    if (scanning && abs(static_cast<int32_t>(digits)) >= scanOverloadDigits) {
        measurement.readFailed();
        return ScanAbort::DmsOverload;
    }
    if (measurement.record(sample.startedAtMs, sample.readAtMs, digits))
        updateDigitCompensation(digits);
    return ScanAbort::None;
}

static void reportPressureReadFailure(const PressureMeasurement& measurement) {
    if (measurement.result() == PressureMeasurement::Result::Unstable) {
        Com::printF(PSTR("readAveragePressure(): the pressure is not constant: "), measurement.minimum());
        Com::printF(PSTR(" / "), measurement.average());
        Com::printFLN(PSTR(" / "), measurement.maximum());
    }
    Com::printFLN(PSTR("readAveragePressure(): the pressure is not plausible"));
}

PressureResult ScanPressure::average(short& digits, uint16_t settleMs) {
    return measure(digits, settleMs, pressureMeasurementSettings());
}

PressureResult ScanPressure::afterStep(short& digits, uint16_t settleMs) {
    return measure(digits, settleMs, {1, 0, 1});
}

PressureResult ScanPressure::idle(short& digits) {
    const auto result = average(digits, idleComparisons_ ? 500 : 0);
    if (result != PressureResult::Stable) {
        if (result != PressureResult::Pending)
            reset();
        return result;
    }

    if (abs(static_cast<int32_t>(digits) - previousIdle_) <= 5) {
        reset();
        if (digits >= probeSettings().idlePressureMin && digits <= probeSettings().idlePressureMax)
            return PressureResult::Stable;
        return PressureResult::Unstable;
    }
    if (++idleComparisons_ >= 5) {
        reset();
        return PressureResult::Unstable;
    }
    previousIdle_ = digits;
    return PressureResult::Pending;
}

bool ScanPressure::service(millis_t now) {
    if (measurement_.result() != PressureResult::Pending)
        return false;
    const auto fault = servicePressureMeasurement(measurement_, now, true);
    if (fault != ScanAbort::None)
        scanSession.requestAbort(fault);
    return measurement_.result() != PressureResult::Pending;
}

void ScanPressure::reset() {
    measurement_.reset();
    idleComparisons_ = 0;
    previousIdle_ = 0;
}

PressureResult ScanPressure::measure(short& digits, uint16_t settleMs, PressureMeasurement::Settings settings) {
    if (measurement_.result() == PressureResult::Idle)
        measurement_.begin(settings, HAL::timeInMilliseconds(), settleMs);

    const auto result = measurement_.result();
    if (result == PressureResult::Pending)
        return result;

    if (result == PressureResult::Stable) {
        digits = measurement_.average();
    } else {
        reportPressureReadFailure(measurement_);
        if (result == PressureResult::ReadError)
            scanSession.requestAbort(ScanAbort::DmsUnavailable);
    }
    measurement_.reset();
    return result;
}

ProbeProgress ScanProbe::measurePoint() {
    switch (pointPhase_) {
    case PointPhase::ApproachCoarse:
        return advancePoint(approach(false), PointPhase::ReleaseCoarse);
    case PointPhase::ReleaseCoarse:
        return advancePoint(release(1), PointPhase::ApproachFine);
    case PointPhase::ApproachFine:
        return advancePoint(approach(true), PointPhase::ReleaseFine);
    case PointPhase::ReleaseFine: {
        const auto progress = release(8);
        if (progress == ProbeProgress::Done) {
            measuredZ_ = Printer::zMotion.scanSteps();
            pointPhase_ = PointPhase::ApproachCoarse;
        }
        return progress;
    }
    }
    return ProbeProgress::Failed;
}

ProbeProgress ScanProbe::advancePoint(ProbeProgress progress, PointPhase next) {
    if (progress == ProbeProgress::Done) {
        pointPhase_ = next;
        return ProbeProgress::Pending;
    }
    return progress;
}

ProbeProgress ScanProbe::approach(bool fine) {
    const long currentZ = Printer::zMotion.scanSteps();
    const long limit = RMath::max(-static_cast<long>(probeSettings().maxTravelSteps), -32768L);
    if (currentZ <= limit) {
        Com::printFLN(PSTR("Probe reached the Z scan limit: "), currentZ);
        return travelLimit(ScanAbort::MaxCompensation);
    }

    short digits;
    const auto result = scanPressure.afterStep(digits, fine ? probeSettings().slowStepDelayMs : probeSettings().fastStepDelayMs);
    if (result == PressureResult::Pending)
        return ProbeProgress::Pending;
    if (result != PressureResult::Stable) {
        scanSession.requestAbort(ScanAbort::AveragePressure);
        return ProbeProgress::Failed;
    }
    if (digits > contactMax_ || digits < contactMin_)
        return ProbeProgress::Done;

    const int step = fine ? probeSettings().slowApproachSteps : probeSettings().fastApproachSteps;
    if (step >= 0)
        return travelLimit(ScanAbort::MaxCompensation);
    return moveZ(RMath::max(static_cast<long>(step), limit - currentZ))
        ? ProbeProgress::Pending : ProbeProgress::Failed;
}

ProbeProgress ScanProbe::release(uint8_t divisor) {
    if (!releaseStarted_) {
        releaseStartZ_ = Printer::zMotion.scanSteps();
        releaseStarted_ = true;
    }

    short digits;
    const auto result = scanPressure.afterStep(digits, probeSettings().slowStepDelayMs);
    if (result == PressureResult::Pending)
        return ProbeProgress::Pending;
    if (result != PressureResult::Stable) {
        scanSession.requestAbort(ScanAbort::AveragePressure);
        return ProbeProgress::Failed;
    }
    if (digits < releaseMax_ && digits > releaseMin_) {
        releaseStarted_ = false;
        return ProbeProgress::Done;
    }

    const int step = RMath::max(1, probeSettings().slowRetreatSteps / divisor);
    const long nextZ = Printer::zMotion.scanSteps() + step;
    const long startHeight = static_cast<long>(Printer::stepsPerMm(Z_AXIS) * probeSettings().startLiftMm);
    if (nextZ > startHeight || nextZ > 32767)
        return travelLimit(ScanAbort::AboveStart);

    const int retryAllowance = constrain(heatBedScanRetries - scanSession.retriesRemaining(), 0, 2);
    const long fastRetreatSteps = probeSettings().fastRetreatSteps;
    if (lastProbeZ_ && abs(nextZ - lastProbeZ_) > fastRetreatSteps * (2 + retryAllowance))
        return travelLimit(ScanAbort::UnevenBed);
    if (abs(nextZ - releaseStartZ_) > fastRetreatSteps * 2 / divisor)
        return travelLimit(ScanAbort::StepDeltaOutOfRange);

    return moveZ(step) ? ProbeProgress::Pending : ProbeProgress::Failed;
}

ProbeProgress ScanProbe::retreat() {
    const millis_t now = HAL::timeInMilliseconds();
    switch (retreatPhase_) {
    case RetreatPhase::Idle:
        lastProbeZ_ = Printer::zMotion.scanSteps();
        retreatStartedAt_ = now;
        retreatPhase_ = RetreatPhase::BeforeMove;
        return ProbeProgress::Pending;
    case RetreatPhase::BeforeMove:
        if (now - retreatStartedAt_ < probeSettings().fastStepDelayMs)
            return ProbeProgress::Pending;
        if (!moveZ(probeSettings().fastRetreatSteps))
            return ProbeProgress::Failed;
        retreatPhase_ = RetreatPhase::AfterMove;
        return ProbeProgress::Pending;
    case RetreatPhase::AfterMove:
        short digits;
        const auto result = scanPressure.average(digits);
        if (result == PressureResult::Pending)
            return ProbeProgress::Pending;
        retreatPhase_ = RetreatPhase::Idle;
        if (result == PressureResult::Stable)
            return ProbeProgress::Done;
        scanSession.requestAbort(ScanAbort::AveragePressure);
        return ProbeProgress::Failed;
    }
    return ProbeProgress::Failed;
}

void showAbortScanReason(const void* scanName, ScanAbort reason) {
    // All scan failures use the same presentation path. Keep the specific
    // reason and hints until persistent history is actually available.
    const void* first = PSTR(UI_TEXT_FAIL);
    const void* second = nullptr;
    switch (reason) {
    case ScanAbort::Canceled:
        first = PSTR(UI_TEXT_SCAN_ABORTED);
        break;
#define RF_SCAN_PANEL(name, value, code, shortText, hint) \
    case ScanAbort::name: \
        first = PSTR(shortText); \
        if (sizeof(hint) > 1) second = PSTR(hint); \
        break;
    RF_SCAN_FAULTS(RF_SCAN_PANEL)
#undef RF_SCAN_PANEL
    default:
        break;
    }
    showError(scanName, first, second);
}

bool moveZ(int nSteps, bool emergencyRetreat) {
    const auto interrupted = [emergencyRetreat] {
        return !emergencyRetreat && (scanSession.aborting() || Printer::blockAll || Printer::printJob.commandsBlocked());
    };
    if (interrupted())
        return false;
    while (Printer::needsCMPwait()) {
        Commands::checkForPeriodicalActions(Processing);
        if (interrupted())
            return false;
    }

    // Retreat can make a pending correction reachable. Reserve Z for this scan move.
    const uint16_t pulses = abs(static_cast<int32_t>(nSteps));
    ZMotion::ScanMove scan(Printer::zMotion, nSteps < 0 ? -1 : 1);
    for (uint16_t i = 0; i < pulses; ++i) {
        if (interrupted())
            return false;
        if (!scan.step()) {
            if (!emergencyRetreat)
                scanSession.requestAbort(ScanAbort::MaxCompensation);
            Com::printFLN(PSTR("Z-Endstop protection"));
            UI_STATUS_UPD("Z-Endstop protection");
            return false;
        }
        if ((i & 15) == 15)
            Commands::checkForPeriodicalActions(Calibrating);
    }
    return !interrupted();
} // moveZ

} // namespace Calibration

short readAveragePressure(short* average) {
    using namespace Calibration;
    PressureMeasurement measurement;
    measurement.begin(pressureMeasurementSettings(), HAL::timeInMilliseconds());
    while (measurement.result() == PressureMeasurement::Result::Pending) {
        servicePressureMeasurement(measurement, HAL::timeInMilliseconds());
        if (measurement.result() != PressureMeasurement::Result::Pending)
            break;
        Commands::checkForPeriodicalActions(Processing);
        if (Printer::blockAll || Printer::printJob.commandsBlocked())
            measurement.readFailed();
    }

    if (measurement.result() == PressureMeasurement::Result::Stable) {
        *average = measurement.average();
        return 0;
    }
    reportPressureReadFailure(measurement);
    *average = 0;
    return -1;
} // readAveragePressure

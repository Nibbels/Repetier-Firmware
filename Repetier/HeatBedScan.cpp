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
#include "DiagnosticHistory.h"
#include "BedMatrix.h"
#include "ScanProbe.h"
#include "GCodeInput.h"

constexpr uint32_t preciseBedScanWarmupSeconds = 600;
constexpr uint32_t preciseBedScanCalibrationSeconds = 600;
constexpr int heatBedScanProbeNozzleC = 100;
constexpr int heatBedScanAbsBedC = 100;
constexpr int heatBedScanAbsNozzleC = 240;

constexpr int heatBedScanCalibrationPointMm = 100;
constexpr int heatBedScanDelayMs = 1000;

static short testExtruderTemperature(void) // Funktion ist speziell für HBS Scan.
{
    if (Extruder::current->tempControl.targetTemperatureC > MAX_ROOM_TEMPERATURE) {
        // we have to wait until the target temperature is reached
        if ((Extruder::current->tempControl.currentTemperatureC + TEMP_TOLERANCE) < Extruder::current->tempControl.targetTemperatureC) {
            // wait until the extruder has reached its target temperature
            UI_STATUS_UPD(UI_TEXT_HEATING_UP);
            return -1;
        }
        if ((Extruder::current->tempControl.currentTemperatureC - TEMP_TOLERANCE) > Extruder::current->tempControl.targetTemperatureC) {
            // wait until the extruder has reached its target temperature
            UI_STATUS_UPD(UI_TEXT_COOLING_DOWN);
            return -1;
        }
        /* else (inside temp_tolerance), return 0, dont wait anymore */
    } else {
        // we have to wait until the current temperatur is below something which would be too warm
        if (Extruder::current->tempControl.currentTemperatureC > MAX_ROOM_TEMPERATURE + 20 /*orig: 65+*/) {
            // wait until the extruder has reached its target temperature
            UI_STATUS_UPD(UI_TEXT_COOLING_DOWN);
            return -1;
        }
    }

    // at this point we have reached the proper temperature
    return 0;

} // testExtruderTemperature

static short testHeatBedTemperature(void) // Funktion ist speziell für HBS Scan.
{
    if (heatedBedController.targetTemperatureC > (MAX_ROOM_TEMPERATURE >= heatBedScanPlaBedC - TEMP_TOLERANCE
                                                      ? heatBedScanPlaBedC - TEMP_TOLERANCE
                                                      : MAX_ROOM_TEMPERATURE))
    /* Evtl. sollte man abfangen, fallst die MAX_ROOM_TEMPERATURE hier höher als die PLA-Scan-TEMP eingestellt ist. Darum der Ternary */
    {
        // we have to wait until the target temperature is reached
        if ((Extruder::getHeatedBedTemperature() + TEMP_TOLERANCE) < heatedBedController.targetTemperatureC) {
            // wait until the heat bed has reached its target temperature
            UI_STATUS_UPD(UI_TEXT_HEATING_UP);
            return -1;
        }
        if ((Extruder::getHeatedBedTemperature() - TEMP_TOLERANCE) > heatedBedController.targetTemperatureC) {
            // wait until the heat bed has reached its target temperature
            UI_STATUS_UPD(UI_TEXT_COOLING_DOWN);
            return -1;
        }
        /* else (inside temp_tolerance), return 0, dont wait anymore */
    } else {
        // we have to wait until the current temperatur is below something which would be too warm
        if (Extruder::getHeatedBedTemperature() > MAX_ROOM_TEMPERATURE + 10 /*orig: 50+*/) {
            // wait until the heat bed has reached its target temperature
            UI_STATUS_UPD(UI_TEXT_COOLING_DOWN);
            return -1;
        }
    }

    // at this point we have reached the proper temperature
    return 0;
} // testHeatBedTemperature

static void addLong(char* string, long value, char digits) {
    uint8_t dig = 0;
    uint8_t neg = 0;
    uint8_t col = strlen(string);
    char buf[13]; // Assumes 8-bit chars plus zero byte.
    char* str = &buf[12];

    if (value < 0) {
        neg = 1;
        value = -value;
        dig++;
    }

    buf[12] = 0;
    do {
        unsigned long m = value;
        value /= 10;
        char c = m - 10 * value;
        *--str = c + '0';
        dig++;
    } while (value);

    if (neg)
        string[col++] = '-';

    if (digits <= 11) {
        while (dig < digits) {
            *--str = ' ';
            dig++;
        }
    }

    while (*str) {
        string[col++] = *str;
        str++;
    }
    string[col] = 0;
} // addLong

namespace {

using Calibration::PressureResult;
using Calibration::ProbeProgress;

class HeatBedScan {
public:
    bool active() const { return phase_ != Phase::Idle; }
    void startOrCancel(HeatBedScanMode mode);
    void tick();
    void discard();

private:
    enum class Phase : uint8_t {
        Idle,
        Initialize,
        PrepareProbeTemperature,
        WaitForBedTemperature,
        WaitForProbeTemperature,
        WarmupAndHome,
        MoveToMeshStart,
        MeasureInitialPressure,
        InitializeGrid,
        AdvanceColumn,
        StartColumn,
        RetryHomeY,
        RetryHomeZ,
        RetryReturnToPoint,
        RetryMeasureBaseline,
        ArmPointRetries,
        CheckPointPressure,
        MeasureMeshPoint,
        RecordMeshPoint,
        AdvanceRow,
        HomeAfterMesh,
        ReportRawMatrix,
        YieldBeforeMatrixPreparation,
        PrepareMatrix,
        CheckFinalPressure,
        MoveToThermalProbe,
        LiftForThermalProbe,
        ExtendThermalClearance,
        HeatNozzleForCalibration,
        WaitForCalibrationTemperature,
        StabilizeCalibrationTemperature,
        ThermalRetryHomeY,
        ThermalRetryHomeZ,
        ThermalRetryReturnToPoint,
        ThermalRetryMeasureBaseline,
        CheckThermalProbePressure,
        MeasureThermalProbe,
        ApplyThermalOffset,
        HomeAfterThermalProbe,
        SwitchOffAfterScan,
        SaveMatrix,
        Aborting,
    };

    void abort();
    void finish();
    void showStatus();
    void initialize();
    void prepareProbeTemperature();
    void waitForBedTemperature();
    void waitForProbeTemperature();
    void warmupAndHome();
    void moveToMeshStart();
    void measureInitialPressure();
    void initializeGrid();
    void advanceColumn();
    void startColumn();
    void retryHomeY();
    void retryHomeZ();
    void retryReturnToPoint();
    void retryMeasureBaseline();
    void armPointRetries();
    void checkProbePressure(Phase next);
    void measureMeshPoint();
    void recordMeshPoint();
    void advanceRow();
    void homeAfterMesh();
    void reportRawMatrix();
    void prepareMatrix();
    void checkFinalPressure();
    void moveToThermalProbe();
    void liftForThermalProbe();
    void extendThermalClearance();
    void heatNozzleForCalibration();
    void waitForCalibrationTemperature();
    void stabilizeCalibrationTemperature();
    void thermalRetryHomeY();
    void thermalRetryHomeZ();
    void thermalRetryReturnToPoint();
    void thermalRetryMeasureBaseline();
    void measureThermalProbe();
    void applyThermalOffset();
    void homeAfterThermalProbe();
    void switchOffAfterScan();
    void saveMatrix();



    volatile Phase phase_ = Phase::Idle;
    Phase retryPhase_ = Phase::Idle;
    HeatBedScanMode mode_ = HeatBedScanMode::Standard;
    millis_t lastCheckMs_ = 0;
    long xSteps_ = 0;
    long ySteps_ = 0;
    long yStrideSteps_ = 0;
    uint8_t xIndex_ = 0;
    uint8_t yIndex_ = 0;
    int8_t yIndexDirection_ = 0;
    short baselineDigits_ = 0;
};

HeatBedScan heatBedScan;

void HeatBedScan::startOrCancel(HeatBedScanMode mode) {
    if (phase_ == Phase::Aborting)
        return;
    if (active()) {
        Calibration::scanSession.requestAbort(ScanAbort::Canceled);
        return;
    }

    if (Printer::printJob.isBusy()) {
        Com::printFLN(Com::tPrintingIsInProcessError);
        showError((void*)ui_text_heat_bed_scan, (void*)ui_text_operation_denied);
        return;
    }

    if (mode != HeatBedScanMode::Standard && !Extruder::allHeatersCalibrated()) {
        Com::printFLN(PSTR("Error: PID tune required before heated scan"));
        return;
    }
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(mode!=HeatBedScanMode::Standard) {
        const int requested=mode==HeatBedScanMode::Pla?heatBedScanPlaBedC:heatBedScanAbsBedC;
        if(requested>heatedBedController.maximumTargetC()) {
            Com::printErrorFLN(PSTR("Bed safety limit too low for heated scan"));
            return;
        }
    }
#endif
    Calibration::scanSession.reset();
    mode_ = mode;
    phase_ = Phase::Initialize;
}

void HeatBedScan::tick() {
    if (!active())
        return;
    if (Calibration::scanSession.aborting()) {
        abort();
        return;
    }

    Printer::inactivity.recordActivity(HAL::timeInMilliseconds());
    GCodeInput::keepAlive(Calibrating);
    showStatus();
    if (Calibration::scanSession.takeRetryRequest()) {
        Calibration::scanProbe.reset();
        phase_ = retryPhase_;
    }

    // A single phase runs before the foreground services the firmware again.
    switch (phase_) {
    case Phase::Idle:
    case Phase::Aborting: break;
    case Phase::Initialize: initialize(); break;
    case Phase::PrepareProbeTemperature: prepareProbeTemperature(); break;
    case Phase::WaitForBedTemperature: waitForBedTemperature(); break;
    case Phase::WaitForProbeTemperature: waitForProbeTemperature(); break;
    case Phase::WarmupAndHome: warmupAndHome(); break;
    case Phase::MoveToMeshStart: moveToMeshStart(); break;
    case Phase::MeasureInitialPressure: measureInitialPressure(); break;
    case Phase::InitializeGrid: initializeGrid(); break;
    case Phase::AdvanceColumn: advanceColumn(); break;
    case Phase::StartColumn: startColumn(); break;
    case Phase::RetryHomeY: retryHomeY(); break;
    case Phase::RetryHomeZ: retryHomeZ(); break;
    case Phase::RetryReturnToPoint: retryReturnToPoint(); break;
    case Phase::RetryMeasureBaseline: retryMeasureBaseline(); break;
    case Phase::ArmPointRetries: armPointRetries(); break;
    case Phase::CheckPointPressure: checkProbePressure(Phase::MeasureMeshPoint); break;
    case Phase::MeasureMeshPoint: measureMeshPoint(); break;
    case Phase::RecordMeshPoint: recordMeshPoint(); break;
    case Phase::AdvanceRow: advanceRow(); break;
    case Phase::HomeAfterMesh: homeAfterMesh(); break;
    case Phase::ReportRawMatrix: reportRawMatrix(); break;
    case Phase::YieldBeforeMatrixPreparation: phase_ = Phase::PrepareMatrix; break;
    case Phase::PrepareMatrix: prepareMatrix(); break;
    case Phase::CheckFinalPressure: checkFinalPressure(); break;
    case Phase::MoveToThermalProbe: moveToThermalProbe(); break;
    case Phase::LiftForThermalProbe: liftForThermalProbe(); break;
    case Phase::ExtendThermalClearance: extendThermalClearance(); break;
    case Phase::HeatNozzleForCalibration: heatNozzleForCalibration(); break;
    case Phase::WaitForCalibrationTemperature: waitForCalibrationTemperature(); break;
    case Phase::StabilizeCalibrationTemperature: stabilizeCalibrationTemperature(); break;
    case Phase::ThermalRetryHomeY: thermalRetryHomeY(); break;
    case Phase::ThermalRetryHomeZ: thermalRetryHomeZ(); break;
    case Phase::ThermalRetryReturnToPoint: thermalRetryReturnToPoint(); break;
    case Phase::ThermalRetryMeasureBaseline: thermalRetryMeasureBaseline(); break;
    case Phase::CheckThermalProbePressure: checkProbePressure(Phase::MeasureThermalProbe); break;
    case Phase::MeasureThermalProbe: measureThermalProbe(); break;
    case Phase::ApplyThermalOffset: applyThermalOffset(); break;
    case Phase::HomeAfterThermalProbe: homeAfterThermalProbe(); break;
    case Phase::SwitchOffAfterScan: switchOffAfterScan(); break;
    case Phase::SaveMatrix: saveMatrix(); break;
    }
}

void HeatBedScan::showStatus() {
    switch (phase_) {
    case Phase::WaitForBedTemperature:
    case Phase::WaitForProbeTemperature:
    case Phase::WarmupAndHome:
    case Phase::WaitForCalibrationTemperature:
    case Phase::StabilizeCalibrationTemperature:
        return; // Keep the heater or countdown message during these phases.
    default:
        UI_STATUS(UI_TEXT_HEAT_BED_SCAN);
    }
}

void HeatBedScan::abort() {
    phase_ = Phase::Aborting;
    const auto reason = Calibration::scanSession.takeAbort();
#if defined(RF_THERMAL_EMERGENCY_EXPERIMENT) && defined(RF_EEPROM_ZONE_EXPERIMENT)
    DiagnosticHistory::recordScan(reason,DiagnosticHistory::ScanKind::HeatBed);
#endif
    Extruder::setHeatedBedTemperature(0);
    Extruder::setTemperatureForAllExtruders(0);
    Calibration::showAbortScanReason((void*)ui_text_heat_bed_scan, reason);

    const bool retreated = Calibration::moveZ(int(Printer::stepsPerMm(Z_AXIS) * 5));
    if (retreated && !ScanSession::retreatOnly(reason))
        Printer::homeAxis(true, true, true);
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    // A nested move/homing wait can latch the terminal stop. Let the
    // outer scan dispatcher perform the deferred discard without releasing torque.
    if(Printer::printJob.emergencyLocked())return;
#endif
    Printer::disableAllSteppersNow();


    Printer::idleStatus.schedule(HAL::timeInMilliseconds() + 30000);
    if (!BedMatrix::reloadActive())
        BedMatrix::clearWorking();

    finish();
}

void HeatBedScan::discard() {
    // An admitted scan has replaced the RAM matrix once initialization ran.
    if (active() && phase_ != Phase::Initialize)
        BedMatrix::clearWorking();
    finish();
}

void HeatBedScan::finish() {
    phase_ = Phase::Idle;
    Calibration::scanProbe.reset();
    Calibration::scanProbe.forgetPreviousPoint();
    Calibration::scanSession.reset();
    retryPhase_ = Phase::Idle;
}

void HeatBedScan::initialize() {
    Printer::idleStatus.cancel();


    // Suspend compensation before replacing its working matrix.
    Printer::disableCMPnow(false);
    BedMatrix::clearWorking();


    phase_ = Phase::PrepareProbeTemperature;
}

void HeatBedScan::prepareProbeTemperature() {
    // the scan is performed with the left extruder
    if (!Extruder::selectExtruderById(0))
        return;

    if (mode_ == HeatBedScanMode::Pla) {
        Extruder::setHeatedBedTemperature(heatBedScanPlaBedC);
        Extruder::setTemperatureForAllExtruders(heatBedScanProbeNozzleC);
    } else if (mode_ == HeatBedScanMode::Abs) {
        Extruder::setHeatedBedTemperature(heatBedScanAbsBedC);
        Extruder::setTemperatureForAllExtruders(heatBedScanProbeNozzleC);
    }

#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(mode_!=HeatBedScanMode::Standard) {
        const int requested=mode_==HeatBedScanMode::Pla?heatBedScanPlaBedC:heatBedScanAbsBedC;
        if(heatedBedController.targetTemperatureC!=requested) {
            Com::printErrorFLN(PSTR("Heated scan bed target not admitted"));
            Calibration::scanSession.requestAbort(ScanAbort::HeaterTargetRejected);
            return;
        }
    }
#endif
    phase_ = Phase::WaitForBedTemperature;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::waitForBedTemperature() {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(mode_!=HeatBedScanMode::Standard) {
        const int requested=mode_==HeatBedScanMode::Pla?heatBedScanPlaBedC:heatBedScanAbsBedC;
        if(heatedBedController.targetTemperatureC!=requested) {
            Com::printErrorFLN(PSTR("Heated scan bed target changed"));
            Calibration::scanSession.requestAbort(ScanAbort::HeaterTargetChanged);
            return;
        }
    }
#endif
    if ((HAL::timeInMilliseconds() - lastCheckMs_) < heatBedScanDelayMs) {
        // do not check too often
        return;
    }

    if (testHeatBedTemperature()) {
        // we did not reach the proper temperature
        lastCheckMs_ = HAL::timeInMilliseconds();
        return;
    }
    phase_ = Phase::WaitForProbeTemperature;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::waitForProbeTemperature() {
    if ((HAL::timeInMilliseconds() - lastCheckMs_) < heatBedScanDelayMs) {
        // do not check too often
        return;
    }

    if (testExtruderTemperature()) {
        // we did not reach the proper temperature
        lastCheckMs_ = HAL::timeInMilliseconds();
        return;
    }
    phase_ = Phase::WarmupAndHome;
    lastCheckMs_ = HAL::timeInMilliseconds();

}

void HeatBedScan::warmupAndHome() {
    if (mode_ != HeatBedScanMode::Standard) {
        // wait some time so that the desired target temperature is reached in all parts of our components
        unsigned long uRemainingSeconds;

        uRemainingSeconds = (HAL::timeInMilliseconds() - lastCheckMs_) / 1000;
        if (uRemainingSeconds < preciseBedScanWarmupSeconds) {
            char szStatus[32];
            strcpy_P(szStatus, PSTR(UI_TEXT_HEATING));
            addLong(szStatus, preciseBedScanWarmupSeconds - uRemainingSeconds, 3);
            strcat_P(szStatus, PSTR("[s]"));
            UI_STATUS_UPD_RAM(szStatus);
            return;
        }
    }

    // start at the home position
    Printer::homeAxis(true, true, true);
    if (!Commands::waitForMotion(Commands::MotionWait::QueueAndZ, Commands::PauseWait::Resume))
        return;

    // move a bit away from the heat bed in order to achieve better measurements in case of hardware configurations where the extruder is very close to the heat bed after the z-homing
    if (!Calibration::moveZ(long(Printer::stepsPerMm(Z_AXIS) * Calibration::probeSettings().startLiftMm)))
        return;

    phase_ = Phase::MoveToMeshStart;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::moveToMeshStart() {
    // move to the first position
    Printer::queueRelativeStepsCoordinates(Calibration::probeSettings().x.startSteps, 0, 0, 0, Printer::axisHomingFeedrate(X_AXIS), false, true);
    Printer::queueRelativeStepsCoordinates(0, Calibration::probeSettings().y.startSteps, 0, 0, Printer::axisHomingFeedrate(Y_AXIS), true, true);

    phase_ = Phase::MeasureInitialPressure;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::measureInitialPressure() {
    if (HAL::timeInMilliseconds() - lastCheckMs_ < heatBedScanDelayMs)
        return;

    const auto result = Calibration::scanPressure.idle(baselineDigits_);
    if (result == PressureResult::Pending)
        return;
    if (result != PressureResult::Stable) {
        Calibration::scanSession.requestAbort(ScanAbort::AveragePressure);
        return;
    }

    phase_ = Phase::InitializeGrid;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::initializeGrid() {
    xSteps_ = Calibration::probeSettings().x.startSteps;
    ySteps_ = Calibration::probeSettings().y.startSteps;
    yStrideSteps_ = Calibration::probeSettings().y.strideSteps; // we start to move the heat bed from the back to the front
    yIndexDirection_ = 1;
    xIndex_ = 2;
    yIndex_ = 2;

    // store also the version of this heat bed compensation matrix
    BedMatrix::beginScanGrid();

    phase_ = Phase::StartColumn;
}

void HeatBedScan::advanceColumn() {
    const long nextPosition = xSteps_ + Calibration::probeSettings().x.strideSteps;
    if (nextPosition > Calibration::probeSettings().x.maxPositionSteps) {
        // we end up here when the scan is complete
        phase_ = Phase::HomeAfterMesh;

        return;
    }

    // move to the next x-position
    Printer::queueRelativeStepsCoordinates(Calibration::probeSettings().x.strideSteps, 0, 0, 0, Printer::axisHomingFeedrate(X_AXIS), true, true);
    xSteps_ += Calibration::probeSettings().x.strideSteps;
    xIndex_++;

    if (xIndex_ >= COMPENSATION_MATRIX_MAX_X) {
        Calibration::scanSession.requestAbort(ScanAbort::MatrixDimensions);
        return;
    }

    if (yStrideSteps_ > 0) {
        // we were moving from the front to the back during this column, so we have to move from the back to the front during the next column
        yStrideSteps_ = -Calibration::probeSettings().y.strideSteps; // we start to move the heat bed from the back to the front
        yIndexDirection_ = -1;
    } else {
        // we were moving from the back to the front during this column, so we have to move from the front to the back during the next column
        yStrideSteps_ = Calibration::probeSettings().y.strideSteps; // we start to move the heat bed from the back to the front
        yIndexDirection_ = 1;
    }

    phase_ = Phase::StartColumn;
}

void HeatBedScan::startColumn() {
    // safety checks
    if (xSteps_ <= Calibration::probeSettings().x.maxPositionSteps) {
        // remember also the exact x-position of this row/column
        BedMatrix::recordScanColumn(xIndex_, (short)((float)xSteps_ / Printer::stepsPerMm(X_AXIS) + 0.5)); // convert to mm

        phase_ = Phase::ArmPointRetries;
        lastCheckMs_ = HAL::timeInMilliseconds();

        return;
    }

    // we end up here when the scan is complete
    phase_ = Phase::HomeAfterMesh;
}

void HeatBedScan::retryHomeY() {
    if (!Calibration::moveZ(long(Printer::stepsPerMm(Z_AXIS) * Calibration::probeSettings().startLiftMm)))
        return;
    Printer::homeAxis(false, true, false);                             //Home Y

    Calibration::scanSession.consumeRetry();
    phase_ = Phase::RetryHomeZ;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::retryHomeZ() {
    // home the z-axis in order to find the starting point again
    Printer::homeAxis(false, false, true);                             // Neben Bett: Home Z
    if (!Calibration::moveZ(long(Printer::stepsPerMm(Z_AXIS) * Calibration::probeSettings().startLiftMm)))
        return;
    Calibration::scanProbe.forgetPreviousPoint();

    phase_ = Phase::RetryReturnToPoint;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::retryReturnToPoint() {
    Printer::queueRelativeStepsCoordinates(0, ySteps_, 0, 0, Printer::axisHomingFeedrate(Y_AXIS), true, true);

    phase_ = Phase::RetryMeasureBaseline;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::retryMeasureBaseline() {
    if (HAL::timeInMilliseconds() - lastCheckMs_ < heatBedScanDelayMs)
        return;

    const auto result = Calibration::scanPressure.idle(baselineDigits_);
    if (result == PressureResult::Pending)
        return;
    if (result != PressureResult::Stable) {
        Calibration::scanSession.requestAbort(ScanAbort::AveragePressure);
        return;
    }
    phase_ = Phase::CheckPointPressure;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::armPointRetries() {
    Calibration::scanSession.armRetries(Calibration::heatBedScanRetries);
    retryPhase_ = Phase::RetryHomeY;
    phase_ = Phase::CheckPointPressure;
}

void HeatBedScan::checkProbePressure(Phase next) {
    if (HAL::timeInMilliseconds() - lastCheckMs_ < Calibration::probeSettings().idleDelayMs)
        return;

    short baseline;
    const auto result = Calibration::scanPressure.average(baseline);
    if (result == PressureResult::Pending || Calibration::scanSession.aborting())
        return;
    if (result != PressureResult::Stable) {
        Calibration::scanSession.requestRetryOrAbort(ScanAbort::IdlePressure);
        return;
    }

    Calibration::scanProbe.setPressureLimits(baseline, Calibration::probeSettings().contactPressureDelta, Calibration::probeSettings().retryPressureDelta);
    if (next == Phase::MeasureThermalProbe)
        Calibration::scanProbe.forgetPreviousPoint();
    phase_ = next;
}

void HeatBedScan::measureMeshPoint() {
    if (Calibration::scanProbe.measurePoint() == ProbeProgress::Done)
        phase_ = Phase::RecordMeshPoint;
}

void HeatBedScan::recordMeshPoint() {
    // remember the z-position and the exact y-position of this row/column
    const short yMm = (short)((float)ySteps_ / Printer::stepsPerMm(Y_AXIS) + 0.5);
    BedMatrix::recordScanPoint(xIndex_, yIndex_, yMm, (short)Calibration::scanProbe.measuredZ());

    phase_ = Phase::AdvanceRow;
}

void HeatBedScan::advanceRow() {
    if (Calibration::scanProbe.retreat() != ProbeProgress::Done)
        return;

    const long nextPosition = ySteps_ + yStrideSteps_;
    const bool columnComplete = yStrideSteps_ > 0 ? nextPosition > Calibration::probeSettings().y.maxPositionSteps
                                                  : nextPosition < Calibration::probeSettings().y.startSteps;
    if (columnComplete) {
        phase_ = Phase::AdvanceColumn;
        return;
    }

    // move to the next y-position
    Printer::queueRelativeStepsCoordinates(0, yStrideSteps_, 0, 0, Printer::axisHomingFeedrate(Y_AXIS), true, true);
    ySteps_ += yStrideSteps_;
    yIndex_ += yIndexDirection_;

    if (yIndex_ >= COMPENSATION_MATRIX_MAX_Y) {
        Calibration::scanSession.requestAbort(ScanAbort::MatrixDimensions);
        return;
    }

    phase_ = Phase::ArmPointRetries;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::homeAfterMesh() {
    // avoid to crash the extruder against the heat bed during the following homing
    if (!Calibration::moveZ(int(Printer::stepsPerMm(Z_AXIS) * 5)))
        return;

    // move back to the home position
    Printer::homeAxis(true, true, true);

    if (mode_ == HeatBedScanMode::Standard) {
        // disable all heaters
        Extruder::setHeatedBedTemperature(0);
        Extruder::setTemperatureForAllExtruders(0);
    }

    phase_ = Phase::ReportRawMatrix;
}

void HeatBedScan::reportRawMatrix() {
    BedMatrix::updateExtents();

    phase_ = Phase::YieldBeforeMatrixPreparation;
}

void HeatBedScan::prepareMatrix() {


    // prepare the heat bed compensation matrix for fast usage during the actual printing
    if (BedMatrix::prepareWorking()) {
        Calibration::scanSession.requestAbort(ScanAbort::MatrixDimensions);
        return;
    }


    phase_ = Phase::CheckFinalPressure;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::checkFinalPressure() {
    short pressure;
    if ((HAL::timeInMilliseconds() - lastCheckMs_) < heatBedScanDelayMs) {
        // do not check too early
        return;
    }

    // compare the idle pressure at the beginning and at the end
    const auto result = Calibration::scanPressure.average(pressure);
    if (result == PressureResult::Pending)
        return;
    if (result != PressureResult::Stable) {
        Calibration::scanSession.requestAbort(ScanAbort::AveragePressure);
        return;
    }


    if (mode_ != HeatBedScanMode::Standard) {
        // we have to determine the z-offset which is caused by different extruder temperatures
        phase_ = Phase::MoveToThermalProbe;

    } else {
        // we are done
        phase_ = Phase::SwitchOffAfterScan;

    }
}

void HeatBedScan::moveToThermalProbe() {
    // we are homed at the moment - move to the position where we shall determine the length offset which is caused by the heated up extruder
    Printer::queueRelativeMMCoordinates(heatBedScanCalibrationPointMm, heatBedScanCalibrationPointMm, 0, 0, RMath::min(Printer::axisHomingFeedrate(X_AXIS), Printer::axisHomingFeedrate(Y_AXIS)), true, true);

    lastCheckMs_ = HAL::timeInMilliseconds();
    phase_ = Phase::LiftForThermalProbe;
}

void HeatBedScan::liftForThermalProbe() {
    // move the heat bed 5mm down
    if (!Calibration::moveZ(int(Printer::stepsPerMm(Z_AXIS) * 5)))
        return;
    lastCheckMs_ = HAL::timeInMilliseconds();
    phase_ = Phase::ExtendThermalClearance;
}

void HeatBedScan::extendThermalClearance() {
    // move the heat bed 5mm down
    if (!Calibration::moveZ(int(Printer::stepsPerMm(Z_AXIS) * 5)))
        return;
    lastCheckMs_ = HAL::timeInMilliseconds();
    phase_ = Phase::HeatNozzleForCalibration;
}

void HeatBedScan::heatNozzleForCalibration() {
    lastCheckMs_ = HAL::timeInMilliseconds();
    // at this point we are homed and we are above the x/y position at which we shall perform the measurement of the z-offset with the hot extruder(s)
    if (mode_ == HeatBedScanMode::Pla) {
        Extruder::setTemperatureForAllExtruders(heatBedScanPlaNozzleC);
    } else if (mode_ == HeatBedScanMode::Abs) {
        Extruder::setTemperatureForAllExtruders(heatBedScanAbsNozzleC);
    }

    phase_ = Phase::WaitForCalibrationTemperature;
}

void HeatBedScan::waitForCalibrationTemperature() {
    if ((HAL::timeInMilliseconds() - lastCheckMs_) < heatBedScanDelayMs) {
        // do not check too often
        return;
    }

    if (testExtruderTemperature()) {
        // we did not reach the proper temperature
        lastCheckMs_ = HAL::timeInMilliseconds();
        return;
    }

    phase_ = Phase::StabilizeCalibrationTemperature;
    lastCheckMs_ = HAL::timeInMilliseconds();


}

void HeatBedScan::stabilizeCalibrationTemperature() {
    if (mode_ != HeatBedScanMode::Standard) {
        // wait some time so that the desired target temperature is reached in all parts of our components
        unsigned long uRemainingSeconds;

        uRemainingSeconds = (HAL::timeInMilliseconds() - lastCheckMs_) / 1000;
        if (uRemainingSeconds < preciseBedScanCalibrationSeconds) {
            char szStatus[32];
            strcpy_P(szStatus, PSTR(UI_TEXT_HEATING));
            addLong(szStatus, preciseBedScanCalibrationSeconds - uRemainingSeconds, 3);
            strcat_P(szStatus, PSTR("[s]"));
            UI_STATUS_UPD_RAM(szStatus);
            return;
        }
    }

    Calibration::scanSession.armRetries(Calibration::heatBedScanRetries);
    retryPhase_ = Phase::ThermalRetryHomeY;
    phase_ = Phase::CheckThermalProbePressure;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::thermalRetryHomeY() {
    if (!Calibration::moveZ(long(Printer::stepsPerMm(Z_AXIS) * Calibration::probeSettings().startLiftMm)))
        return;
    Printer::homeAxis(false, true, false);                             //Home Y

    Calibration::scanSession.consumeRetry();
    phase_ = Phase::ThermalRetryHomeZ;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::thermalRetryHomeZ() {
    // home the z-axis in order to find the starting point again
    Printer::homeAxis(false, false, true);                             // Neben Bett: Home Z
    if (!Calibration::moveZ(long(Printer::stepsPerMm(Z_AXIS) * Calibration::probeSettings().startLiftMm)))
        return;
    Calibration::scanProbe.forgetPreviousPoint();

    phase_ = Phase::ThermalRetryReturnToPoint;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::thermalRetryReturnToPoint() {
    Printer::queueRelativeMMCoordinates(0, heatBedScanCalibrationPointMm, 0, 0, Printer::axisHomingFeedrate(Y_AXIS), true, true);

    phase_ = Phase::ThermalRetryMeasureBaseline;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::thermalRetryMeasureBaseline() {
    if (HAL::timeInMilliseconds() - lastCheckMs_ < heatBedScanDelayMs)
        return;

    short baseline;
    const auto result = Calibration::scanPressure.idle(baseline);
    if (result == PressureResult::Pending)
        return;
    if (result != PressureResult::Stable) {
        Calibration::scanSession.requestAbort(ScanAbort::AveragePressure);
        return;
    }
    phase_ = Phase::CheckThermalProbePressure;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::measureThermalProbe() {
    if (Calibration::scanProbe.measurePoint() == ProbeProgress::Done)
        phase_ = Phase::ApplyThermalOffset;
}

void HeatBedScan::applyThermalOffset() {
    // adjust the current z-position to the compensation matrix in order to consider the different length of the extruder at higher temperatures
    BedMatrix::applyThermalOffset((short)Calibration::scanProbe.measuredZ());


    phase_ = Phase::HomeAfterThermalProbe;
}

void HeatBedScan::homeAfterThermalProbe() {
    // avoid to crash the extruder against the heat bed during the following homing
    if (!Calibration::moveZ(int(Printer::stepsPerMm(Z_AXIS) * 5)))
        return;

    // move back to the home position
    Printer::homeAxis(true, true, true);
    phase_ = Phase::SwitchOffAfterScan;
}

void HeatBedScan::switchOffAfterScan() {
    // turn off the engines
    Printer::disableAllSteppersNow();

    // disable all heaters
    Extruder::setHeatedBedTemperature(0);
    Extruder::setTemperatureForAllExtruders(0);

    phase_ = Phase::SaveMatrix;
}

void HeatBedScan::saveMatrix() {
    // save the determined values to the EEPROM
    const bool saved = BedMatrix::saveScannedGrid();
    Printer::idleStatus.schedule(HAL::timeInMilliseconds());
    if (!saved) {
        showError(ui_text_heat_bed_scan, PSTR("Matrix save failed"));
        finish();
        return;
    }

    showInformation(PSTR(UI_TEXT_HEAT_BED_SCAN_DONE), (void*)ui_text_saving_success, PSTR(UI_TEXT_OK)); // Tell user the scan was a success

    finish();
}

} // namespace

bool heatBedScanRunning() {
    return heatBedScan.active();
}

void startHeatBedScan(HeatBedScanMode mode) {
#ifdef RF_EEPROM_ZONE_EXPERIMENT
    if(!EEPROM::settingsReady())return;
#endif
    if (!Calibration::zOffsetScanRunning())
        heatBedScan.startOrCancel(mode);
}

void Calibration::tickHeatBedScan() { heatBedScan.tick(); }
void Calibration::discardHeatBedScan() { heatBedScan.discard(); }

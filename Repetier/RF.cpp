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

#include <Arduino.h> // capital A so it is error prone on case-sensitive filesystems
#include "Repetier.h"
#include "PressureMeasurement.h"
#include "ScanSettings.h"
#include "GCodeInput.h"
#include "BedCompensation.h"
#include "TwiTransfer.h"

FSTRINGVALUE(ui_text_error, UI_TEXT_ERROR)
FSTRINGVALUE(ui_text_warning, UI_TEXT_WARNING)
FSTRINGVALUE(ui_text_information, UI_TEXT_INFORMATION)
FSTRINGVALUE(ui_text_heat_bed_scan, UI_TEXT_HEAT_BED_SCAN)
FSTRINGVALUE(ui_text_output_object, UI_TEXT_OUTPUT_OBJECT)
FSTRINGVALUE(ui_text_set_origin, UI_TEXT_SET_ORIGIN)
FSTRINGVALUE(ui_text_pause, UI_TEXT_PAUSE)
FSTRINGVALUE(ui_text_home, UI_TEXT_HOME)
FSTRINGVALUE(ui_text_z_compensation, UI_TEXT_Z_COMPENSATION)
FSTRINGVALUE(ui_text_x_axis, UI_TEXT_X_AXIS)
FSTRINGVALUE(ui_text_y_axis, UI_TEXT_Y_AXIS)
FSTRINGVALUE(ui_text_z_axis, UI_TEXT_Z_AXIS)
FSTRINGVALUE(ui_text_extruder, UI_TEXT_EXTRUDER)
FSTRINGVALUE(ui_text_autodetect_pid, UI_TEXT_AUTODETECT_PID)
FSTRINGVALUE(ui_text_temperature_manager, UI_TEXT_TEMPERATURE_MANAGER)
FSTRINGVALUE(ui_text_home_unknown, UI_TEXT_HOME_UNKNOWN)
FSTRINGVALUE(ui_text_saving_needless, UI_TEXT_SAVING_NEEDLESS)
FSTRINGVALUE(ui_text_operation_denied, UI_TEXT_OPERATION_DENIED)
FSTRINGVALUE(ui_text_emergency_pause, UI_TEXT_EMERGENCY_PAUSE)
FSTRINGVALUE(ui_text_emergency_stop, UI_TEXT_EMERGENCY_STOP)
FSTRINGVALUE(ui_text_invalid_matrix, UI_TEXT_INVALID_MATRIX)
FSTRINGVALUE(ui_text_min_reached, UI_TEXT_MIN_REACHED)
FSTRINGVALUE(ui_text_min_reached_unhomed, UI_TEXT_MIN_REACHED_UNHOMED)
FSTRINGVALUE(ui_text_max_reached, UI_TEXT_MAX_REACHED)
FSTRINGVALUE(ui_text_temperature_wrong, UI_TEXT_TEMPERATURE_WRONG)
FSTRINGVALUE(ui_text_timeout, UI_TEXT_TIMEOUT)
FSTRINGVALUE(ui_text_sensor_error, UI_TEXT_SENSOR_ERROR)
FSTRINGVALUE(ui_text_heater_error, UI_TEXT_HEATER_ERROR)
FSTRINGVALUE(ui_text_heat_bed_zoffset_search_status, UI_TEXT_HEAT_BED_ZOFFSET_SEARCH_STATUS)
FSTRINGVALUE(ui_text_heat_bed_zoffset_fix_z1, UI_TEXT_HEAT_BED_ZOFFSET_FIX_Z1)
FSTRINGVALUE(ui_text_question, UI_TEXT_UNKNOWN)
FSTRINGVALUE(ui_text_heat_bed_zoffset_fix_z2, UI_TEXT_HEAT_BED_ZOFFSET_FIX_Z2)
FSTRINGVALUE(ui_text_statusmsg, UI_TEXT_STATUSMSG)

FSTRINGVALUE(ui_text_saving_success, UI_TEXT_SAVING_SUCCESS)

constexpr int heatBedScanStepMm = 20;
constexpr int heatBedScanCalibrationPointMm = 100;
constexpr int heatBedScanContactPressureDelta = 10;
constexpr int heatBedScanRetryPressureDelta = 5;
constexpr int heatBedScanIdlePressureDelta = 0;
constexpr int heatBedScanIdlePressureMin = -7500;
constexpr int heatBedScanIdlePressureMax = 7500;
constexpr float heatBedScanUpFastMm = -0.02f;
constexpr float heatBedScanUpSlowMm = -0.004f;
constexpr float heatBedScanDownSlowMm = 0.004f;
constexpr int heatBedScanFastStepDelayMs = 1;
constexpr int heatBedScanSlowStepDelayMs = 100;
constexpr int heatBedScanIdleDelayMs = 250;
constexpr int heatBedScanDelayMs = 1000;
constexpr int heatBedScanRetries = 5;
constexpr int heatBedScanPressureReads = 3;
constexpr int heatBedScanPressureTolerance = 15;
constexpr int heatBedScanPressureReadDelayMs = 15;

constexpr uint32_t preciseBedScanWarmupSeconds = 600;
constexpr uint32_t preciseBedScanCalibrationSeconds = 600;
constexpr int heatBedScanProbeNozzleC = 100;
constexpr int heatBedScanAbsBedC = 100;
constexpr int heatBedScanAbsNozzleC = 240;

constexpr int zOffsetScanContactPressureDelta = 33;
constexpr int zOffsetScanRetryPressureDelta = 30;


static long g_offsetZCompensationSteps = 0;
static short g_ZCompensationMax = 0;
static char g_nActiveHeatBed = 1;

// g_ZMatrixChangedInRam soll 1 werden, wenn ZOS, Offsetänderung der Matrix etc. Sonst wäre Sichern der Matrix unnötig.
static volatile unsigned char g_ZMatrixChangedInRam = 0;

static ScanSettings scanSettings(defaultHeatBedScanLiftMm);

float scanStartLiftMm() { return scanSettings.startLiftMm; }
void setScanStartLiftMm(float liftMm) { scanSettings.startLiftMm = liftMm; }
long scanStrideMm(uint8_t axis) { return axis == X_AXIS ? scanSettings.x.strideMm : scanSettings.y.strideMm; }

static short g_ZCompensationMatrix[COMPENSATION_MATRIX_MAX_X][COMPENSATION_MATRIX_MAX_Y];
static unsigned char g_uZMatrixMax[2] = { 0, 0 };

uint8_t activeBedMatrixSlot() { return g_nActiveHeatBed; }
bool bedMatrixAvailable() { return g_ZCompensationMatrix[0][0] == EEPROM_FORMAT; }
bool bedMatrixHasUnsavedChanges() { return g_ZMatrixChangedInRam; }
long bedMatrixHighestZSteps() { return g_offsetZCompensationSteps; }
short bedMatrixLowestZSteps() { return g_ZCompensationMax; }
void invalidateBedMatrixForReload() { g_ZCompensationMatrix[0][0] = EEPROM_FORMAT - 1; }

char g_debugLevel = 0;
char g_debugLog = 0;

#ifdef RGB_LIGHT_R_PIN
unsigned char g_uRGBHeatingR = RGB_HEATING_R;
unsigned char g_uRGBHeatingG = RGB_HEATING_G;
unsigned char g_uRGBHeatingB = RGB_HEATING_B;
unsigned char g_uRGBPrintingR = RGB_PRINTING_R;
unsigned char g_uRGBPrintingG = RGB_PRINTING_G;
unsigned char g_uRGBPrintingB = RGB_PRINTING_B;
unsigned char g_uRGBCoolingR = RGB_COOLING_R;
unsigned char g_uRGBCoolingG = RGB_COOLING_G;
unsigned char g_uRGBCoolingB = RGB_COOLING_B;
unsigned char g_uRGBIdleR = RGB_IDLE_R;
unsigned char g_uRGBIdleG = RGB_IDLE_G;
unsigned char g_uRGBIdleB = RGB_IDLE_B;
unsigned char g_uRGBManualR = RGB_MANUAL_R;
unsigned char g_uRGBManualG = RGB_MANUAL_G;
unsigned char g_uRGBManualB = RGB_MANUAL_B;
volatile unsigned char g_uRGBCurrentR = 0;
volatile unsigned char g_uRGBCurrentG = 0;
volatile unsigned char g_uRGBCurrentB = 0;
unsigned char g_uRGBTargetR = 0;
unsigned char g_uRGBTargetG = 0;
unsigned char g_uRGBTargetB = 0;
#endif // RGB_LIGHT_R_PIN

static void setupForPrinting();
static void initCompensationMatrix();
static void outputCompensationMatrix(char format = 0);
static char prepareCompensationMatrix();
static void determineCompensationOffsetZ();
static void adjustCompensationMatrix(short steps);
static char loadCompensationMatrix(unsigned int address);
static unsigned short readWord24C256(int addressI2C, unsigned int addressEEPROM);
static bool writeWord24C256(int addressI2C, unsigned int addressEEPROM, unsigned short data);
static bool readWord24C256(int addressI2C, unsigned int addressEEPROM, unsigned short& data);
static bool writeByte24C256(int addressI2C, unsigned int addressEEPROM, unsigned char data);
static bool saveWorkingMatrix(unsigned int address);
static void showAbortScanReason(const void* scanName, ScanAbort reason);
static bool moveZ(int nSteps, bool emergencyRetreat = false);

void initRF(void) {
    Printer::strainGauge.begin(HAL::timeInMilliseconds());

    Printer::disableCMPnow(true);
    setupForPrinting();
} // initRF

static void updateDigitCompensation(int16_t digits) {
    Printer::digitCompensation.observe(digits, Printer::doHeatBedZCompensation, [] {
        // The force brake also runs during M3912, regardless of the learned first-layer boundary.
        return Printer::zMotion.nominalSteps() > Printer::layers.fadeStartSteps() || Printer::startLine.active();
    });
}

static PressureMeasurement::Settings pressureMeasurementSettings();
static ScanAbort servicePressureMeasurement(PressureMeasurement& measurement, millis_t now, bool scanning = false);
static void reportPressureReadFailure(const PressureMeasurement& measurement);

namespace {

using PressureResult = PressureMeasurement::Result;
constexpr int16_t scanOverloadDigits = 10000;

ScanSession scanSession;

class ScanPressure {
public:
    PressureResult average(short& digits, uint16_t settleMs = 0) {
        return measure(digits, settleMs, pressureMeasurementSettings());
    }

    PressureResult afterStep(short& digits, uint16_t settleMs) {
        return measure(digits, settleMs, {1, 0, 1});
    }

    PressureResult idle(short& digits) {
        const auto result = average(digits, idleComparisons_ ? 500 : 0);
        if (result != PressureResult::Stable) {
            if (result != PressureResult::Pending)
                reset();
            return result;
        }

        if (abs(static_cast<int32_t>(digits) - previousIdle_) <= 5) {
            reset();
            if (digits >= scanSettings.idlePressureMin && digits <= scanSettings.idlePressureMax)
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

    bool service(millis_t now) {
        if (measurement_.result() != PressureResult::Pending)
            return false;
        const auto fault = servicePressureMeasurement(measurement_, now, true);
        if (fault != ScanAbort::None)
            scanSession.requestAbort(fault);
        return measurement_.result() != PressureResult::Pending;
    }

    void reset() {
        measurement_.reset();
        idleComparisons_ = 0;
        previousIdle_ = 0;
    }

private:
    PressureResult measure(short& digits, uint16_t settleMs, PressureMeasurement::Settings settings) {
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

    PressureMeasurement measurement_;
    uint8_t idleComparisons_ = 0;
    short previousIdle_ = 0;
};

ScanPressure scanPressure;

enum class ProbeProgress : uint8_t { Pending, Done, Failed };

class ScanProbe {
public:
    // The same measurement is used for a mesh cell, hot-nozzle calibration and Z-offset correction.
    ProbeProgress measurePoint();
    ProbeProgress retreat();
    long measuredZ() const { return measuredZ_; }

    void setPressureLimits(short baseline, uint16_t contactDelta, uint16_t retryDelta) {
        contactMin_ = baseline - contactDelta;
        contactMax_ = baseline + contactDelta;
        releaseMin_ = baseline - retryDelta;
        releaseMax_ = baseline + retryDelta;
    }

    void forgetPreviousPoint() { lastProbeZ_ = 0; }

    void reset() {
        scanPressure.reset();
        releaseStarted_ = false;
        pointPhase_ = PointPhase::ApproachCoarse;
        retreatPhase_ = RetreatPhase::Idle;
    }

private:
    enum class PointPhase : uint8_t { ApproachCoarse, ReleaseCoarse, ApproachFine, ReleaseFine };
    enum class RetreatPhase : uint8_t { Idle, BeforeMove, AfterMove };

    ProbeProgress approach(bool fine);
    ProbeProgress release(uint8_t divisor);
    ProbeProgress advancePoint(ProbeProgress progress, PointPhase next);
    ProbeProgress travelLimit(ScanAbort reason) const {
        scanSession.requestRetryOrAbort(reason);
        return ProbeProgress::Failed;
    }

    long lastProbeZ_ = 0;
    long releaseStartZ_ = 0;
    long measuredZ_ = 0;
    short contactMin_ = 0;
    short contactMax_ = 0;
    short releaseMin_ = 0;
    short releaseMax_ = 0;
    PointPhase pointPhase_ = PointPhase::ApproachCoarse;
    millis_t retreatStartedAt_ = 0;
    bool releaseStarted_ = false;
    RetreatPhase retreatPhase_ = RetreatPhase::Idle;
};

ScanProbe scanProbe;

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
    const long limit = RMath::max(-static_cast<long>(scanSettings.maxTravelSteps), -32768L);
    if (currentZ <= limit) {
        Com::printFLN(PSTR("Probe reached the Z scan limit: "), currentZ);
        return travelLimit(ScanAbort::MaxCompensation);
    }

    short digits;
    const auto result = scanPressure.afterStep(digits, fine ? scanSettings.slowStepDelayMs : scanSettings.fastStepDelayMs);
    if (result == PressureResult::Pending)
        return ProbeProgress::Pending;
    if (result != PressureResult::Stable) {
        scanSession.requestAbort(ScanAbort::AveragePressure);
        return ProbeProgress::Failed;
    }
    if (digits > contactMax_ || digits < contactMin_)
        return ProbeProgress::Done;

    const int step = fine ? scanSettings.slowApproachSteps : scanSettings.fastApproachSteps;
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
    const auto result = scanPressure.afterStep(digits, scanSettings.slowStepDelayMs);
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

    const int step = RMath::max(1, scanSettings.slowRetreatSteps / divisor);
    const long nextZ = Printer::zMotion.scanSteps() + step;
    const long startHeight = static_cast<long>(Printer::axisStepsPerMM[Z_AXIS] * scanSettings.startLiftMm);
    if (nextZ > startHeight || nextZ > 32767)
        return travelLimit(ScanAbort::AboveStart);

    const int retryAllowance = constrain(heatBedScanRetries - scanSession.retriesRemaining(), 0, 2);
    const long fastRetreatSteps = scanSettings.fastRetreatSteps;
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
        if (now - retreatStartedAt_ < scanSettings.fastStepDelayMs)
            return ProbeProgress::Pending;
        if (!moveZ(scanSettings.fastRetreatSteps))
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

} // namespace

static bool zOffsetScanRunning();

namespace {

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
    millis_t startedAtMs_ = 0;
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
        if (Printer::debugInfo())
            Com::printFLN(PSTR("HBS: cancelled"));
        scanSession.requestAbort(ScanAbort::Canceled);
        return;
    }

    if (Printer::printJob.isBusy()) {
        if (Printer::debugErrors())
            Com::printFLN(Com::tPrintingIsInProcessError);
        showError((void*)ui_text_heat_bed_scan, (void*)ui_text_operation_denied);
        return;
    }

    scanSession.reset();
    mode_ = mode;
    phase_ = Phase::Initialize;
}

void HeatBedScan::tick() {
    if (!active())
        return;
    if (scanSession.aborting()) {
        abort();
        return;
    }

    Printer::inactivity.recordActivity(HAL::timeInMilliseconds());
    GCodeInput::keepAlive(Calibrating);
    showStatus();
    if (scanSession.takeRetryRequest()) {
        scanProbe.reset();
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
    const auto reason = scanSession.takeAbort();
    Extruder::setHeatedBedTemperature(0);
    Extruder::setTemperatureForAllExtruders(0);
    showAbortScanReason((void*)ui_text_heat_bed_scan, reason);

    const bool retreated = moveZ(int(Printer::axisStepsPerMM[Z_AXIS] * 5));
    if (retreated && !ScanSession::retreatOnly(reason))
        Printer::homeAxis(true, true, true);
    Printer::disableAllSteppersNow();

    if (Printer::debugInfo()) {
        Com::printF(Com::tscanHeatBed);
        Com::printFLN(PSTR("aborted"));
    }

    Printer::idleStatus.schedule(HAL::timeInMilliseconds() + 30000);
    if (loadCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed)))
        initCompensationMatrix();

    finish();
}

void HeatBedScan::discard() {
    // An admitted scan has replaced the RAM matrix once initialization ran.
    if (active() && phase_ != Phase::Initialize)
        initCompensationMatrix();
    finish();
}

void HeatBedScan::finish() {
    phase_ = Phase::Idle;
    scanProbe.reset();
    scanProbe.forgetPreviousPoint();
    scanSession.reset();
    retryPhase_ = Phase::Idle;
}

void HeatBedScan::initialize() {
    Printer::idleStatus.cancel();
    startedAtMs_ = HAL::timeInMilliseconds();

    if (Printer::debugInfo()) {
        Com::printF(Com::tscanHeatBed);
        Com::printFLN(PSTR("scan started"));
    }

    // Suspend compensation before replacing its working matrix.
    Printer::disableCMPnow(false);
    initCompensationMatrix();

    // output the currently used scan parameters
    outputScanParameters();

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

    phase_ = Phase::WaitForBedTemperature;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::waitForBedTemperature() {
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

    if (mode_ != HeatBedScanMode::Standard) {
        if (Printer::debugInfo()) {
            Com::printF(Com::tscanHeatBed);
            Com::printFLN(PSTR("warmup delay [s] = "), preciseBedScanWarmupSeconds);
        }
    }
}

void HeatBedScan::warmupAndHome() {
    if (mode_ != HeatBedScanMode::Standard) {
        // wait some time so that the desired target temperature is reached in all parts of our components
        unsigned long uRemainingSeconds;

        uRemainingSeconds = (HAL::timeInMilliseconds() - lastCheckMs_) / 1000;
        if (uRemainingSeconds < preciseBedScanWarmupSeconds) {
            char szStatus[32];
            strcpy(szStatus, UI_TEXT_HEATING);
            addLong(szStatus, preciseBedScanWarmupSeconds - uRemainingSeconds, 3);
            strcat(szStatus, "[s]");
            UI_STATUS_UPD_RAM(szStatus);
            return;
        }
    }

    // start at the home position
    Printer::homeAxis(true, true, true);
    if (!Commands::waitForMotion(Commands::MotionWait::QueueAndZ, Commands::PauseWait::Resume))
        return;

    // move a bit away from the heat bed in order to achieve better measurements in case of hardware configurations where the extruder is very close to the heat bed after the z-homing
    if (!moveZ(long(Printer::axisStepsPerMM[Z_AXIS] * scanSettings.startLiftMm)))
        return;

    phase_ = Phase::MoveToMeshStart;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::moveToMeshStart() {
    // move to the first position
    Printer::queueRelativeStepsCoordinates(scanSettings.x.startSteps, 0, 0, 0, Printer::homingFeedrate[X_AXIS], false, true);
    Printer::queueRelativeStepsCoordinates(0, scanSettings.y.startSteps, 0, 0, Printer::homingFeedrate[Y_AXIS], true, true);

    phase_ = Phase::MeasureInitialPressure;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::measureInitialPressure() {
    if (HAL::timeInMilliseconds() - lastCheckMs_ < heatBedScanDelayMs)
        return;

    const auto result = scanPressure.idle(baselineDigits_);
    if (result == PressureResult::Pending)
        return;
    if (result != PressureResult::Stable) {
        scanSession.requestAbort(ScanAbort::AveragePressure);
        return;
    }

    phase_ = Phase::InitializeGrid;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::initializeGrid() {
    xSteps_ = scanSettings.x.startSteps;
    ySteps_ = scanSettings.y.startSteps;
    yStrideSteps_ = scanSettings.y.strideSteps; // we start to move the heat bed from the back to the front
    yIndexDirection_ = 1;
    xIndex_ = 2;
    yIndex_ = 2;

    // store also the version of this heat bed compensation matrix
    g_ZCompensationMatrix[0][0] = EEPROM_FORMAT;

    phase_ = Phase::StartColumn;
}

void HeatBedScan::advanceColumn() {
    const long nextPosition = xSteps_ + scanSettings.x.strideSteps;
    if (nextPosition > scanSettings.x.maxPositionSteps) {
        // we end up here when the scan is complete
        phase_ = Phase::HomeAfterMesh;

        return;
    }

    // move to the next x-position
    Printer::queueRelativeStepsCoordinates(scanSettings.x.strideSteps, 0, 0, 0, Printer::homingFeedrate[X_AXIS], true, true);
    xSteps_ += scanSettings.x.strideSteps;
    xIndex_++;

    if (xIndex_ >= COMPENSATION_MATRIX_MAX_X) {
        scanSession.requestAbort(ScanAbort::MatrixDimensions);
        return;
    }

    if (yStrideSteps_ > 0) {
        // we were moving from the front to the back during this column, so we have to move from the back to the front during the next column
        yStrideSteps_ = -scanSettings.y.strideSteps; // we start to move the heat bed from the back to the front
        yIndexDirection_ = -1;
    } else {
        // we were moving from the back to the front during this column, so we have to move from the front to the back during the next column
        yStrideSteps_ = scanSettings.y.strideSteps; // we start to move the heat bed from the back to the front
        yIndexDirection_ = 1;
    }

    phase_ = Phase::StartColumn;
}

void HeatBedScan::startColumn() {
    // safety checks
    if (xSteps_ <= scanSettings.x.maxPositionSteps) {
        // remember also the exact x-position of this row/column
        g_ZCompensationMatrix[xIndex_][0] = (short)((float)xSteps_ / Printer::axisStepsPerMM[X_AXIS] + 0.5); // convert to mm

        phase_ = Phase::ArmPointRetries;
        lastCheckMs_ = HAL::timeInMilliseconds();

        return;
    }

    // we end up here when the scan is complete
    phase_ = Phase::HomeAfterMesh;
}

void HeatBedScan::retryHomeY() {
    if (!moveZ(long(Printer::axisStepsPerMM[Z_AXIS] * scanSettings.startLiftMm)))
        return;
    Printer::homeAxis(false, true, false);                             //Home Y

    scanSession.consumeRetry();
    phase_ = Phase::RetryHomeZ;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::retryHomeZ() {
    // home the z-axis in order to find the starting point again
    Printer::homeAxis(false, false, true);                             //Neben Bett: Home Z
    if (!moveZ(long(Printer::axisStepsPerMM[Z_AXIS] * scanSettings.startLiftMm)))
        return;
    scanProbe.forgetPreviousPoint();

    phase_ = Phase::RetryReturnToPoint;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::retryReturnToPoint() {
    Printer::queueRelativeStepsCoordinates(0, ySteps_, 0, 0, Printer::homingFeedrate[Y_AXIS], true, true);

    phase_ = Phase::RetryMeasureBaseline;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::retryMeasureBaseline() {
    if (HAL::timeInMilliseconds() - lastCheckMs_ < heatBedScanDelayMs)
        return;

    const auto result = scanPressure.idle(baselineDigits_);
    if (result == PressureResult::Pending)
        return;
    if (result != PressureResult::Stable) {
        scanSession.requestAbort(ScanAbort::AveragePressure);
        return;
    }
    phase_ = Phase::CheckPointPressure;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::armPointRetries() {
    scanSession.armRetries(heatBedScanRetries);
    retryPhase_ = Phase::RetryHomeY;
    phase_ = Phase::CheckPointPressure;
}

void HeatBedScan::checkProbePressure(Phase next) {
    if (HAL::timeInMilliseconds() - lastCheckMs_ < scanSettings.idleDelayMs)
        return;

    short baseline;
    const auto result = scanPressure.average(baseline);
    if (result == PressureResult::Pending || scanSession.aborting())
        return;
    if (result != PressureResult::Stable) {
        scanSession.requestRetryOrAbort(ScanAbort::IdlePressure);
        return;
    }

    scanProbe.setPressureLimits(baseline, scanSettings.contactPressureDelta, scanSettings.retryPressureDelta);
    if (next == Phase::MeasureThermalProbe)
        scanProbe.forgetPreviousPoint();
    phase_ = next;
}

void HeatBedScan::measureMeshPoint() {
    if (scanProbe.measurePoint() == ProbeProgress::Done)
        phase_ = Phase::RecordMeshPoint;
}

void HeatBedScan::recordMeshPoint() {
    // remember the z-position and the exact y-position of this row/column
    g_ZCompensationMatrix[xIndex_][yIndex_] = (short)scanProbe.measuredZ();
    g_ZCompensationMatrix[0][yIndex_] = (short)((float)ySteps_ / Printer::axisStepsPerMM[Y_AXIS] + 0.5); // convert to mm

    if (xIndex_ > g_uZMatrixMax[X_AXIS]) {
        g_uZMatrixMax[X_AXIS] = xIndex_;
    }

    if (yIndex_ > g_uZMatrixMax[Y_AXIS]) {
        g_uZMatrixMax[Y_AXIS] = yIndex_;
    }

    phase_ = Phase::AdvanceRow;
}

void HeatBedScan::advanceRow() {
    if (scanProbe.retreat() != ProbeProgress::Done)
        return;

    const long nextPosition = ySteps_ + yStrideSteps_;
    const bool columnComplete = yStrideSteps_ > 0 ? nextPosition > scanSettings.y.maxPositionSteps
                                                  : nextPosition < scanSettings.y.startSteps;
    if (columnComplete) {
        phase_ = Phase::AdvanceColumn;
        return;
    }

    // move to the next y-position
    Printer::queueRelativeStepsCoordinates(0, yStrideSteps_, 0, 0, Printer::homingFeedrate[Y_AXIS], true, true);
    ySteps_ += yStrideSteps_;
    yIndex_ += yIndexDirection_;

    if (yIndex_ >= COMPENSATION_MATRIX_MAX_Y) {
        scanSession.requestAbort(ScanAbort::MatrixDimensions);
        return;
    }

    phase_ = Phase::ArmPointRetries;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::homeAfterMesh() {
    // avoid to crash the extruder against the heat bed during the following homing
    if (!moveZ(int(Printer::axisStepsPerMM[Z_AXIS] * 5)))
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
    if (Printer::debugInfo()) {
        // output the determined compensation
        Com::printF(Com::tscanHeatBed);
        Com::printFLN(PSTR("raw heat bed z matrix: "));
        determineCompensationOffsetZ();
        outputCompensationMatrix();
    }

    phase_ = Phase::YieldBeforeMatrixPreparation;
}

void HeatBedScan::prepareMatrix() {
    if (Printer::debugInfo()) {
        // output the pure scan time
        Com::printF(Com::tscanHeatBed);
        Com::printF(PSTR("total scan time: "), long((HAL::timeInMilliseconds() - startedAtMs_) / 1000));
        Com::printFLN(PSTR(" [s]"));
    }

    if (Printer::debugInfo()) {
        Com::printF(Com::tscanHeatBed);
        Com::printFLN(PSTR("g_uZMatrixMax[Y_AXIS].1 = "), (int)g_uZMatrixMax[Y_AXIS]);
    }

    // prepare the heat bed compensation matrix for fast usage during the actual printing
    if (prepareCompensationMatrix()) {
        scanSession.requestAbort(ScanAbort::MatrixDimensions);
        return;
    }

    if (Printer::debugInfo()) {
        Com::printF(Com::tscanHeatBed);
        Com::printFLN(PSTR("g_uZMatrixMax[Y_AXIS].2 = "), (int)g_uZMatrixMax[Y_AXIS]);

        // output the converted heat bed compensation matrix
        Com::printF(Com::tscanHeatBed);
        Com::printFLN(PSTR("converted heat bed z matrix: "));
        outputCompensationMatrix();
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
    const auto result = scanPressure.average(pressure);
    if (result == PressureResult::Pending)
        return;
    if (result != PressureResult::Stable) {
        scanSession.requestAbort(ScanAbort::AveragePressure);
        return;
    }

    if (Printer::debugInfo()) {
        Com::printF(Com::tscanHeatBed);
        Com::printFLN(PSTR("idle pressure at start: "), baselineDigits_);
        Com::printF(Com::tscanHeatBed);
        Com::printFLN(PSTR("idle pressure at stop: "), pressure);
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
    Printer::queueRelativeMMCoordinates(heatBedScanCalibrationPointMm, heatBedScanCalibrationPointMm, 0, 0, RMath::min(Printer::homingFeedrate[X_AXIS], Printer::homingFeedrate[Y_AXIS]), true, true);

    lastCheckMs_ = HAL::timeInMilliseconds();
    phase_ = Phase::LiftForThermalProbe;
}

void HeatBedScan::liftForThermalProbe() {
    // move the heat bed 5mm down
    if (!moveZ(int(Printer::axisStepsPerMM[Z_AXIS] * 5)))
        return;
    lastCheckMs_ = HAL::timeInMilliseconds();
    phase_ = Phase::ExtendThermalClearance;
}

void HeatBedScan::extendThermalClearance() {
    // move the heat bed 5mm down
    if (!moveZ(int(Printer::axisStepsPerMM[Z_AXIS] * 5)))
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


    if (Printer::debugInfo()) {
        Com::printF(Com::tscanHeatBed);
        Com::printFLN(PSTR("calibration delay [s] =  "), (uint32_t)preciseBedScanCalibrationSeconds);
    }
}

void HeatBedScan::stabilizeCalibrationTemperature() {
    if (mode_ != HeatBedScanMode::Standard) {
        // wait some time so that the desired target temperature is reached in all parts of our components
        unsigned long uRemainingSeconds;

        uRemainingSeconds = (HAL::timeInMilliseconds() - lastCheckMs_) / 1000;
        if (uRemainingSeconds < preciseBedScanCalibrationSeconds) {
            char szStatus[32];
            strcpy(szStatus, UI_TEXT_HEATING);
            addLong(szStatus, preciseBedScanCalibrationSeconds - uRemainingSeconds, 3);
            strcat(szStatus, "[s]");
            UI_STATUS_UPD_RAM(szStatus);
            return;
        }
    }

    scanSession.armRetries(heatBedScanRetries);
    retryPhase_ = Phase::ThermalRetryHomeY;
    phase_ = Phase::CheckThermalProbePressure;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::thermalRetryHomeY() {
    if (!moveZ(long(Printer::axisStepsPerMM[Z_AXIS] * scanSettings.startLiftMm)))
        return;
    Printer::homeAxis(false, true, false);                             //Home Y

    scanSession.consumeRetry();
    phase_ = Phase::ThermalRetryHomeZ;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::thermalRetryHomeZ() {
    // home the z-axis in order to find the starting point again
    Printer::homeAxis(false, false, true);                             //Neben Bett: Home Z
    if (!moveZ(long(Printer::axisStepsPerMM[Z_AXIS] * scanSettings.startLiftMm)))
        return;
    scanProbe.forgetPreviousPoint();

    phase_ = Phase::ThermalRetryReturnToPoint;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::thermalRetryReturnToPoint() {
    Printer::queueRelativeMMCoordinates(0, heatBedScanCalibrationPointMm, 0, 0, Printer::homingFeedrate[Y_AXIS], true, true);

    phase_ = Phase::ThermalRetryMeasureBaseline;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::thermalRetryMeasureBaseline() {
    if (HAL::timeInMilliseconds() - lastCheckMs_ < heatBedScanDelayMs)
        return;

    short baseline;
    const auto result = scanPressure.idle(baseline);
    if (result == PressureResult::Pending)
        return;
    if (result != PressureResult::Stable) {
        scanSession.requestAbort(ScanAbort::AveragePressure);
        return;
    }
    phase_ = Phase::CheckThermalProbePressure;
    lastCheckMs_ = HAL::timeInMilliseconds();
}

void HeatBedScan::measureThermalProbe() {
    if (scanProbe.measurePoint() == ProbeProgress::Done)
        phase_ = Phase::ApplyThermalOffset;
}

void HeatBedScan::applyThermalOffset() {
    // adjust the current z-position to the compensation matrix in order to consider the different length of the extruder at higher temperatures
    adjustCompensationMatrix((short)scanProbe.measuredZ());

    if (Printer::debugInfo()) {
        // output the converted heat bed compensation matrix
        Com::printF(Com::tscanHeatBed);
        Com::printFLN(PSTR("adjusted heat bed z matrix: "));
        outputCompensationMatrix();
    }

    phase_ = Phase::HomeAfterThermalProbe;
}

void HeatBedScan::homeAfterThermalProbe() {
    // avoid to crash the extruder against the heat bed during the following homing
    if (!moveZ(int(Printer::axisStepsPerMM[Z_AXIS] * 5)))
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
    g_ZMatrixChangedInRam = 1;
    const bool saved = saveWorkingMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed));
    Printer::idleStatus.schedule(HAL::timeInMilliseconds());
    if (!saved) {
        showError(ui_text_heat_bed_scan, PSTR("Matrix save failed"));
        finish();
        return;
    }
    if (Printer::debugInfo()) {
        Com::printF(Com::tscanHeatBed);
        Com::printFLN(PSTR("the heat bed z matrix has been saved"));
    }

    if (Printer::debugInfo()) {
        Com::printF(Com::tscanHeatBed);
        Com::printFLN(PSTR("the scan has been completed"));
    }
    showInformation(PSTR(UI_TEXT_HEAT_BED_SCAN_DONE), (void*)ui_text_saving_success, PSTR(UI_TEXT_OK)); //tell user the scan was a success

    finish();
}

} // namespace

bool heatBedScanRunning() {
    return heatBedScan.active();
}

void startHeatBedScan(HeatBedScanMode mode) {
    if (!zOffsetScanRunning())
        heatBedScan.startOrCancel(mode);
}

namespace {

class ZOffsetScan {
public:
    struct Settings {
        ZOffsetScanPoint point{defaultZOffsetScanIndex, defaultZOffsetScanIndex};
        float learningRate = 1.0f;
        float learningGradient = 0.0f;
    };

    Settings settings() const { return settings_; }
    void configure(const Settings& settings) { settings_ = settings; }
    void restorePoint(ZOffsetScanPoint point) { settings_.point = point; }

    bool active() const { return phase_ != Phase::Idle; }
    void startOrCancel(bool levelMatrix);
    void tick();
    void discard() {
        phase_ = Phase::Idle;
        levelingPass_ = LevelingPass::None;
        scanProbe.reset();
        scanProbe.forgetPreviousPoint();
        scanSession.reset();
    }

private:
    enum class Phase : uint8_t {
        Idle,
        Initialize,
        HomeForProbe,
        LiftForProbe,
        LoadMatrix,
        PositionProbe,
        ArmPressureRetries,
        RetreatForRetry,
        MeasureBaseline,
        MeasureSurface,
        ApplyMatrixCorrection,
        ContinueLeveling,
        RetreatForFinish,
        ReportScrewAdjustment,
        Aborting,
    };
    enum class LevelingPass : uint8_t {
        None, Center, MinXMinY, MaxXMaxY, CenterYMinusOne, MinXMaxY, MaxXMinY, CenterYPlusOne
    };

    void abort(bool reloadMatrix);
    void selectLevelingPoint(unsigned short maxX, unsigned short maxY);
    void initialize();
    void homeForProbe();
    void liftForProbe();
    void loadMatrix();
    void positionProbe();
    void armPressureRetries();
    void beginRetryWait();
    void retreatForRetry();
    void measureBaseline();
    void measureSurface();
    void applyMatrixCorrection();
    void continueLeveling();
    void retreatForFinish();
    void reportScrewAdjustment();





    volatile Phase phase_ = Phase::Idle;
    LevelingPass levelingPass_ = LevelingPass::None;
    Settings settings_;
    void retryProbe(ScanAbort failure);
    long probeZSteps_ = 0;
    millis_t retryWaitStartedMs_ = 0;
};

ZOffsetScan zOffsetScan;

void ZOffsetScan::startOrCancel(bool levelMatrix) {
    if (phase_ == Phase::Aborting)
        return;
    if (active()) {
        Com::printFLN(PSTR("ZOS cancelled"));
        scanSession.requestAbort(ScanAbort::Canceled);
        return;
    }

    Com::printFLN(PSTR("ZOS started"));
    scanSession.reset();
    levelingPass_ = levelMatrix ? LevelingPass::Center : LevelingPass::None;
    phase_ = Phase::Initialize;
}

void ZOffsetScan::tick() {
    if (!active())
        return;

    Printer::inactivity.recordActivity(HAL::timeInMilliseconds());
    GCodeInput::keepAlive(Calibrating);
    if (scanSession.aborting()) {
        abort(false);
        return;
    }

    // Run one phase per service call. Calibration waits still pump the foreground.
    switch (phase_) {
    case Phase::Idle:
    case Phase::Aborting: break;
    case Phase::Initialize: initialize(); break;
    case Phase::HomeForProbe: homeForProbe(); break;
    case Phase::LiftForProbe: liftForProbe(); break;
    case Phase::LoadMatrix: loadMatrix(); break;
    case Phase::PositionProbe: positionProbe(); break;
    case Phase::ArmPressureRetries: armPressureRetries(); break;
    case Phase::RetreatForRetry: retreatForRetry(); break;
    case Phase::MeasureBaseline: measureBaseline(); break;
    case Phase::MeasureSurface: measureSurface(); break;
    case Phase::ApplyMatrixCorrection: applyMatrixCorrection(); break;
    case Phase::ContinueLeveling: continueLeveling(); break;
    case Phase::RetreatForFinish: retreatForFinish(); break;
    case Phase::ReportScrewAdjustment: reportScrewAdjustment(); break;
    }
}

void ZOffsetScan::selectLevelingPoint(unsigned short maxX, unsigned short maxY) {
    const LevelingPass pass = levelingPass_;
    switch (pass) {
    case LevelingPass::None: return;
    case LevelingPass::Center:
        settings_.point = {uint8_t(maxX / 2), uint8_t(maxY / 2)};
        levelingPass_ = LevelingPass::MinXMinY;
        break;
    case LevelingPass::MinXMinY:
        settings_.point = {1, 1};
        levelingPass_ = LevelingPass::MaxXMaxY;
        break;
    case LevelingPass::MaxXMaxY:
        settings_.point = {uint8_t(maxX), uint8_t(maxY)};
        levelingPass_ = LevelingPass::CenterYMinusOne;
        break;
    case LevelingPass::CenterYMinusOne:
        settings_.point = {uint8_t(maxX / 2), uint8_t(maxY / 2 - 1)};
        levelingPass_ = LevelingPass::MinXMaxY;
        break;
    case LevelingPass::MinXMaxY:
        settings_.point = {1, uint8_t(maxY)};
        levelingPass_ = LevelingPass::MaxXMinY;
        break;
    case LevelingPass::MaxXMinY:
        settings_.point = {uint8_t(maxX), 1};
        levelingPass_ = LevelingPass::CenterYPlusOne;
        break;
    case LevelingPass::CenterYPlusOne:
        settings_.point = {uint8_t(maxX / 2), uint8_t(maxY / 2 + 1)};
        levelingPass_ = LevelingPass::None;
        break;
    }

    settings_.learningRate = pass == LevelingPass::Center ? 1.0f : 0.8f;
    settings_.learningGradient = pass == LevelingPass::Center ? 0.0f : 1.0f;
    probeZSteps_ = long(Printer::axisStepsPerMM[Z_AXIS] * scanSettings.startLiftMm);
    Com::printF(Com::tAutoMatrixLeveling, settings_.point.x);
    Com::printFLN(PSTR(" Y:"), settings_.point.y);
}

void ZOffsetScan::initialize() {
    Printer::idleStatus.cancel();
    Com::printFLN(PSTR("ZOS init"));
    phase_ = Phase::HomeForProbe;
    probeZSteps_ = long(Printer::axisStepsPerMM[Z_AXIS] * scanSettings.startLiftMm); //nur nutzen wenn kleiner.
}

void ZOffsetScan::homeForProbe() {
    //bissel übertrieben, sollte aber jede eventualität abfangen: Wir brauchen die maximale Matrix-Dimension auch schon hier (ganz grob) und wollen nicht so lange warten.
    unsigned short uDimensionX, uDimensionY;
    if (g_ZCompensationMatrix[0][0] == EEPROM_FORMAT) {
        uDimensionX = g_uZMatrixMax[X_AXIS] - 1;
        uDimensionY = g_uZMatrixMax[Y_AXIS] - 1;
    } else {
        uDimensionX = (unsigned char)readWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_SECTOR_SIZE * g_nActiveHeatBed + EEPROM_OFFSET_DIMENSION_X) - 1;
        uDimensionY = (unsigned char)readWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_SECTOR_SIZE * g_nActiveHeatBed + EEPROM_OFFSET_DIMENSION_Y) - 1;
    }
    if (uDimensionX > COMPENSATION_MATRIX_MAX_X - 1)
        uDimensionX = (unsigned char)(COMPENSATION_MATRIX_MAX_X - 1);
    if (uDimensionY > COMPENSATION_MATRIX_MAX_Y - 1)
        uDimensionY = (unsigned char)(COMPENSATION_MATRIX_MAX_Y - 1);

    selectLevelingPoint(uDimensionX, uDimensionY);

    // start at the home position
    Printer::homeAxis(true, true, true); //home z resets ZCMP
    phase_ = Phase::LiftForProbe;
}

void ZOffsetScan::liftForProbe() {
    // Keep the lift in scan coordinates. The probe uses its own Z travel counter.
    if (!moveZ(probeZSteps_))
        return;
    UI_STATUS_UPD(UI_TEXT_ZCALIB);
    phase_ = Phase::LoadMatrix;
}

void ZOffsetScan::loadMatrix() {
    // load the unaltered compensation matrix from the EEPROM
    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT || settings_.learningRate == 1.0) {
        Com::printFLN(PSTR("Loading zMatrix from EEPROM"));
        if (loadCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed))) {
            // Error: there is no valid compensation matrix available
            initCompensationMatrix();
        }
    } else {
        Com::printFLN(PSTR("Reusing existing zMatrix"));
    }

    // safety check on the current matrix
    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
        Com::printFLN(PSTR("ERROR::prev. matrix invalid!"));
        scanSession.requestAbort(ScanAbort::MissingMatrix);
        abort(false);
        return;
    }
    phase_ = Phase::PositionProbe;
}

void ZOffsetScan::positionProbe() {
    // Prefer measured points inside the padded edges whenever the grid allows it.
    const int minX = heatBedScanXStartMm == 0 ? 1 : 2;
    const int minY = heatBedScanYStartMm == 0 ? 1 : 2;
    auto& point = settings_.point;
    point.x = RMath::min(RMath::max(int(point.x), minX), g_uZMatrixMax[X_AXIS] - 1);
    point.y = RMath::min(RMath::max(int(point.y), minY), g_uZMatrixMax[Y_AXIS] - 1);
    float xScanPosition = g_ZCompensationMatrix[point.x][0];
    float yScanPosition = g_ZCompensationMatrix[0][point.y];

    constexpr long scanPositionJitterMm = 3;
    xScanPosition += random(-scanPositionJitterMm, scanPositionJitterMm);
    yScanPosition += random(-scanPositionJitterMm, scanPositionJitterMm);

    Printer::queueRelativeMMCoordinates(xScanPosition, yScanPosition, 0, 0, RMath::min(Printer::homingFeedrate[X_AXIS], Printer::homingFeedrate[Y_AXIS]), true, true);
    phase_ = Phase::ArmPressureRetries;
}

void ZOffsetScan::armPressureRetries() {
    scanSession.armRetries(20);
    phase_ = Phase::MeasureBaseline;
}

void ZOffsetScan::beginRetryWait() {
    scanProbe.reset();
    retryWaitStartedMs_ = HAL::timeInMilliseconds();
    phase_ = Phase::RetreatForRetry;
}

void ZOffsetScan::retreatForRetry() {
    if (HAL::timeInMilliseconds() - retryWaitStartedMs_ < heatBedScanDelayMs)
        return;
    if (scanProbe.retreat() == ProbeProgress::Done)
        phase_ = Phase::MeasureBaseline;
}

void ZOffsetScan::measureBaseline() {
    short baseline;
    const auto result = scanPressure.idle(baseline);
    if (result == PressureResult::Pending)
        return;
    if (result != PressureResult::Stable) {
        retryProbe(ScanAbort::IdlePressure);
        return;
    }

    scanProbe.setPressureLimits(baseline, zOffsetScanContactPressureDelta, zOffsetScanRetryPressureDelta);

    Com::printFLN(PSTR("Approaching HeatBed"));
    scanProbe.forgetPreviousPoint();
    phase_ = Phase::MeasureSurface;
}

void ZOffsetScan::retryProbe(ScanAbort failure) {
    if (scanSession.consumeRetry()) {
        beginRetryWait();
        return;
    }
    scanSession.requestAbort(failure);
}

void ZOffsetScan::measureSurface() {
    const auto progress = scanProbe.measurePoint();
    if (progress == ProbeProgress::Pending)
        return;
    if (progress == ProbeProgress::Failed) {
        retryProbe(ScanAbort::AveragePressure);
        return;
    }

    const long measuredZ = scanProbe.measuredZ();
    if (measuredZ < probeZSteps_)
        probeZSteps_ = measuredZ;
    Com::printFLN(PSTR("Z = "), measuredZ * Printer::axisMMPerSteps[Z_AXIS], 4);
    phase_ = Phase::ApplyMatrixCorrection;
}

void ZOffsetScan::applyMatrixCorrection() {
    // compute number of steps we need to shift the entire matrix by
    long matrixwert = getZMatrixDepth_CurrentXY();
    long nZ = probeZSteps_ - matrixwert;

    Com::printFLN(PSTR("Matrix-Wert Z = "), matrixwert);
    Com::printF(PSTR("Minimum Z = "), probeZSteps_);
    Com::printFLN(PSTR(" dZ = "), nZ);

    // update the matrix: shift by nZ and check for integer overflow
    bool overflow = false;
    bool overH = false;
    bool overnull = false;

    //Nibbels: scaling nZ according to learning Rate for additional corrective scans
    nZ = (long)((float)nZ * settings_.learningRate);

    //Nibbels: weight change because of distance. lerne bettwinkelausgleich.
    const int xIntervals = RMath::max(1, g_uZMatrixMax[X_AXIS] - 2);
    const int yIntervals = RMath::max(1, g_uZMatrixMax[Y_AXIS] - 2);
    const float x_bed_len_quadrat = xIntervals * xIntervals;
    const float y_bed_len_quadrat = yIntervals * yIntervals;
    float x_dist = 0;
    float y_dist = 0;
    float xy_weight = 0;
    long weighted_nZ = 0;
    long newValue = 0;

    for (short x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) {
        for (short y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
            x_dist = (settings_.point.x - x) * (settings_.point.x - x) / x_bed_len_quadrat; //normierter indexabstand
            y_dist = (settings_.point.y - y) * (settings_.point.y - y) / y_bed_len_quadrat; //normierter indexabstand
            x_dist = sqrt(x_dist);                                                                    //Linearisieren. für Auto-Matrix-Leveling
            y_dist = sqrt(y_dist);                                                                    //Linearisieren. für Auto-Matrix-Leveling
            //das ist nur ein kreisabstand, wenn die messpunkte quadratisch angeordnet sind, ist aber nicht so?
            // evtl. todo: achse faktor skalieren, sodass kreis x/y=(10/13)
            xy_weight = 1 - sqrt(x_dist * x_dist + y_dist * y_dist); //linear gewichtet
            if (xy_weight < 0.0)
                xy_weight = 0;
            if (xy_weight > 1.0)
                xy_weight = 1.0; //kann aber nicht wirklich vorkommen.
            weighted_nZ = (long)(settings_.learningGradient * xy_weight * (float)nZ + (1.0 - settings_.learningGradient) * (float)nZ);
            newValue = (long)g_ZCompensationMatrix[x][y] + weighted_nZ;
            if (newValue > 32767 || newValue < -32768)
                overflow = true;
            if (newValue > 0)
                overnull = true; //darf nicht positiv werden. //darf nicht über limit sein.
            if (newValue > (long(Printer::axisStepsPerMM[Z_AXIS] * scanSettings.startLiftMm) + scanSettings.fastApproachSteps))
                overH = true; //darf nicht positiv werden. //darf nicht über limit sein.
            g_ZCompensationMatrix[x][y] = (short)newValue;
        }
    }
    // fail if overflow occurred
    if (overflow) {
        // load the unaltered compensation matrix from the EEPROM since the current in-memory matrix is invalid
        Com::printFLN(PSTR("Matrix Overflow!"));
        scanSession.requestAbort(ScanAbort::MatrixOverflow);
        abort(true);
        return;
    }
    // fail if z>(Starthöhe - ein bisschen) occurred
    if (overH) {
        // load the unaltered compensation matrix from the EEPROM since the current in-memory matrix is bigger than z=zero
        Com::printFLN(PSTR("ERROR::Z-Matrix höher Start-Z!"));
        scanSession.requestAbort(ScanAbort::MatrixHeight);
        abort(true);
        return;
    }
    // fail if z>0 occurred
    if (overnull) {
        // load the unaltered compensation matrix from the EEPROM since the current in-memory matrix is bigger than z=zero
        Com::printFLN(PSTR("FIX::Clean Hotend-Nozzle"));
        Com::printFLN(PSTR("FIX::Fix Z-Schraube"));
    }
    // determine the minimal distance between extruder and heat bed
    determineCompensationOffsetZ();
    g_ZMatrixChangedInRam = 1; //man kan die matrix mit diesem marker nun sichern.

    phase_ = Phase::ContinueLeveling;
}

void ZOffsetScan::continueLeveling() {
    if (levelingPass_ == LevelingPass::None) {
        phase_ = Phase::RetreatForFinish;
        return;
    }
    phase_ = Phase::HomeForProbe;
    if (!moveZ(Printer::axisStepsPerMM[Z_AXIS]))
        return;
    Printer::homeAxis(true, true, false);
    if (!moveZ(-Printer::zMotion.scanSteps()))
        return;
    outputCompensationMatrix(1);
}

void ZOffsetScan::retreatForFinish() {
    if (!moveZ(Printer::axisStepsPerMM[Z_AXIS]))
        return;
    Printer::homeAxis(false, true, false);
    if (!moveZ(-Printer::zMotion.scanSteps()))
        return;
    phase_ = Phase::ReportScrewAdjustment;    // No AUTO_MATRIX_LEVELING
}

void ZOffsetScan::reportScrewAdjustment() {
    discard();
    calculateZScrewCorrection();

    showMyPage((void*)ui_text_heat_bed_zoffset_search_status, (void*)ui_text_heat_bed_zoffset_fix_z1, (void*)ui_text_heat_bed_zoffset_fix_z2, (void*)ui_text_statusmsg);
    uid.scheduleMessageReturn(HAL::timeInMilliseconds() + 30000);
    Printer::idleStatus.schedule(HAL::timeInMilliseconds() + 30000);

    //Printer::zMotion.scanSteps() is 0 now.
    Com::printFLN(PSTR("ZOS finished"));
}

void ZOffsetScan::abort(bool reloadMatrix) {
    phase_ = Phase::Aborting;
    const auto reason = scanSession.takeAbort();
    showAbortScanReason(PSTR(UI_TEXT_DO_MHIER_BED_SCAN), reason);

    if (reloadMatrix) {
        if (loadCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed))) {
            // Error: there is no valid compensation matrix available
            initCompensationMatrix();
        }
    }
    // the search has been aborted
    Com::printFLN(PSTR("ZOS aborted"));

    if (ScanSession::retreatOnly(reason)) {
        moveZ(int(Printer::axisStepsPerMM[Z_AXIS] * 5));
    } else if (Printer::queueRelativeMMCoordinates(0, 0, 5, 0, Printer::homingFeedrate[Z_AXIS], true, true)) {
        Printer::homeAxis(true, true, true);
    }

    // turn off all steppers and extruders
    Printer::disableAllSteppersNow();

    discard();
    Printer::idleStatus.schedule(HAL::timeInMilliseconds() + 30000);
}

} // namespace

void startZOScan(bool autoLevelMatrix) {
    if (!heatBedScan.active())
        zOffsetScan.startOrCancel(autoLevelMatrix);
}

static bool zOffsetScanRunning() {
    return zOffsetScan.active();
}

ZOffsetScanPoint zOffsetScanPoint() {
    return zOffsetScan.settings().point;
}

void restoreZOffsetScanPoint(ZOffsetScanPoint point) {
    zOffsetScan.restorePoint(point);
}

void discardScans() {
    // A homing or move wait can service the print stop while a scan phase is on
    // the stack. Complete the discard when that phase returns.
    if (scanSession.deferDiscard())
        return;
    heatBedScan.discard();
    zOffsetScan.discard();
}

float g_ZSchraubenSollDrehungenWarm_U = 0;
float g_ZSchraubenSollKorrekturWarm_mm = 0;
char g_ZSchraubeOk = 0;

void calculateZScrewCorrection(void) {
    Com::printFLN(PSTR(" "));
    Com::printFLN(PSTR("Z-Schrauben-Helper: "));

    /*IDEAL ist es, wenn mand den Drucker "kalt" und frisch aufgeheizt einstellt.*/

    //Gerade eben muss ein Z-Scan die Matrix korrigiert haben.
    //Der Extruder darf sich seither nicht verändert (T0 -> T1) und nicht abgekühlt haben
    //Dann wird pro °C des aktiven Extruders ~0.001mm Längung angenommen.
    //Dann wird pro °C des Heizbettes ~0.0015mm Längung angenommen.

    //Wegen der unbekannten Nachlängung werden 0.15mm Puffer auf Z=0 angestrebt.
    //Einfach so: wird ein Puffer von 0.05 angestrebt. Die ideale Matrix-Verschiebung wird auf 260°C Hotend / 120°C Bett, was ein Extremwert darstellen soll auf -0.05mm angepeilt.

    //Config
    const float maxExtruderTemperature = Extruder::current->tempControl.heaterCutoffC;
    float maxBedTemperature = 120.0f; //120°C ist ok... mit 180 zu rechnen wäre übertrieben.
    float BedThermalExplansionInMikrons = 1.5f;
    float ExtruderThermalExpansionInMikrons = 0.95f;
    float maxNachdehnungInMikrons = 150.0f;
    float RestAbstandInMikrons = 50.0f;

    //Ist-Abstand.
    float MatrixMaximumInMikrons = (float)g_offsetZCompensationSteps * Printer::axisMMPerSteps[Z_AXIS] * 1000.0f;

    float ExtruderTemperature = Extruder::current->tempControl.currentTemperatureC;
    if (ExtruderTemperature < 20.0f)
        ExtruderTemperature = 20.0f; //Wenn zu kalt oder undefiniert dann Standardbedingungen annehmen.

    float BedTemperature = Extruder::getHeatedBedTemperature();

    //Umrechnung des aktuellen Zustandes auf die heißesten Werte:
    float MinDistanceInMikronsKalt = MatrixMaximumInMikrons
        + (maxExtruderTemperature - ExtruderTemperature) * ExtruderThermalExpansionInMikrons
        + (maxBedTemperature - BedTemperature) * BedThermalExplansionInMikrons;
    float MinDistanceInMikronsWarm = MinDistanceInMikronsKalt + maxNachdehnungInMikrons;

    Com::printFLN(PSTR("- Alle Werte in Mikrometern / Einheit [um] -"));
    Com::printFLN(PSTR("Matrix-Minimum: "), MatrixMaximumInMikrons);
    Com::printFLN(PSTR("Weitere Extruderausdehnung maximal: "), (maxExtruderTemperature - ExtruderTemperature) * ExtruderThermalExpansionInMikrons);
    Com::printFLN(PSTR("Weitere Heizbettausdehnung maximal: "), (maxBedTemperature - BedTemperature) * BedThermalExplansionInMikrons);
    Com::printFLN(PSTR("Maximalwert-zMatrix bei Maximaltemperaturen (kalter Drucker): "), MinDistanceInMikronsKalt);
    Com::printFLN(PSTR("Maximalwert-zMatrix bei Maximaltemperaturen (durchgewaermter Drucker): "), MinDistanceInMikronsWarm);

    //z.B. -200 <-- um diesen Wert dürfte man korrigieren, wenn der Drucker zum Messzeitpunkt voll durchgewärmt wäre. Weiß er aber nicht!
    //Ein vorgewärmter Drucker justiert das Heizbett eher auf -0.2, ein kalter Drucker justiert es eher auf -0.05 bei Spitzentemperaturen. Beides ist ok.
    //float SollkorrekturKalt = (MinDistanceInMikronsKalt + RestAbstandInMikrons);
    //z.B.  -50 <-- diesen Wert darf man in jedem Fall korrigieren.
    float SollkorrekturWarm = (MinDistanceInMikronsWarm + RestAbstandInMikrons);

    /*
    Wenn ich den Test mit einem bereits warmen Drucker mache, plane ich unnötig eine Sicherheit ein, die ich nur einrechne, weil der Drucker aktuell kalt sein könnte.
    Also warmer Drucker: Bett weiter hoch justieren, also Schraube weiter rein, also Drehsinn Minus.
    */
    // |+0-|..Puffer..|.......Nachdehnung........|Kalt-Soll-Einstellung|
    // |+0-|..Puffer..|Warm-Soll-Einstellung|

    /* TIPP: -> Schraube bis maxNachdehnungInMikrons ~ 150um weiter runter(=Bett weiter hoch =Drehsinn Minus) empfehlen, wenn der Drucker aktuell "mehr druchgewärmt" wäre. */

    Com::println();
    Com::printSharpLine();
    Com::printF(PSTR("Sollkorrektur: "), SollkorrekturWarm, 0);
    Com::printF(PSTR(" [um] = "), SollkorrekturWarm * 0.001f, 3);
    Com::printFLN(PSTR(" [mm]"));

    //meldung:
    g_ZSchraubenSollKorrekturWarm_mm = SollkorrekturWarm * 0.001f;

    float ZSchraubenDrehungenWarm = SollkorrekturWarm * 0.002f; //[Sollkorrektur in mm] geteilt durch [Gewinde: 0.5 mm/Umdrehung] -> Sollkorrektur / 1000 / 0.5
    Com::printF(PSTR("Sollumdrehungen: "), ZSchraubenDrehungenWarm, 1);
    Com::printF(PSTR(" [U] = "), ZSchraubenDrehungenWarm * 360, 0);
    Com::printF(PSTR(" [Grad]"));

    //meldung:
    g_ZSchraubenSollDrehungenWarm_U = ZSchraubenDrehungenWarm;

    if (ZSchraubenDrehungenWarm > 0)
        Com::printFLN(PSTR(" (+ heisst rausdrehen/linksrum/gegen die Uhr)")); //Bett wird nach unten justiert
    else
        Com::printFLN(PSTR(" (- heisst reindrehen/rechtsrum/im Uhrzeigersinn)")); //Bett wird nach oben justiert

    Com::printFLN(PSTR("Je kaelter der Gesamtdrucker aktuell ist (nach langer Pause frisch angeschaltet), desto besser der Korrekturwert."));
    g_ZSchraubeOk = -1;                                                             //neg -> Matrix negativ -> ok. ausser, wenn:
#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2        //TODO: Prüfen ob das beim RF2000v2 stimmen wird.
    if (-0.5f <= ZSchraubenDrehungenWarm && SollkorrekturWarm < 40.0f /* [um] */) { // < 0.25mm = 0.5Umdrehungen ist mit dem RF2000 nicht machbar.
        Com::printFLN(PSTR(" (Die Z-Schraube ist ok!)"));                           //das ist die Änderung in M3-Regelgewinde-Z-Schrauben-Umdrehungen
        //meldung:
        g_ZSchraubeOk = -1;                                                                        //neg
    } else if (SollkorrekturWarm >= 40.0f /* [um] */) {                                            //dann bin ich rechnerisch um Z = 0 (50um mit 10um toleranz, die ich fordere.)
                                                                                                   //eine korrektur von mehr als +40um heißt, ich bin gerade vermutlich im Z>0
        Com::printFLN(PSTR(" (Die Z-Schraube weiter raus! Das Bett scheint zu hoch zu liegen.)")); //das ist die Änderung in M3-Regelgewinde-Z-Schrauben-Umdrehungen
        //meldung:
        g_ZSchraubeOk = 1; //pos
    }
    Com::printFLN(PSTR(" (RF2000: Minimal eine halbe Schraubendrehung (dZ=0.25mm-Schritte) einstellbar.)")); //das ist die Änderung in M3-Regelgewinde-Z-Schrauben-Umdrehungen
#else                                                                                                        //if MOTHERBOARD == DEVICE_TYPE_RF1000
    if (SollkorrekturWarm >= 40.0f /* [um] */) { //dann bin ich rechnerisch um Z = 0 (50um mit 10um toleranz, die ich fordere.)
        //eine korrektur von mehr als +40um heißt, ich bin gerade vermutlich im Z>0
        Com::printFLN(PSTR(" (Die Z-Schraube weiter raus! Das Bett scheint zu hoch zu liegen.)")); //das ist die Änderung in M3-Regelgewinde-Z-Schrauben-Umdrehungen
        //meldung:
        g_ZSchraubeOk = 1; //pos
    }
#endif
    Com::printSharpLine();
} // calculateZScrewCorrection

/**************************************************************************************************************************************/

/**************************************************************************************************************************************/

void fixKeramikLochInMatrix(void) {
    //Com::printFLN( PSTR( "fixKeramikLochInMatrix(): 1 Init" ) );

    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
        // we load the z compensation matrix before its first usage because this can take some time
        prepareZCompensation();
    }
    if (g_ZCompensationMatrix[0][0] == EEPROM_FORMAT) {
        // search for deepest hole in bed-z-matrix and fix it according to surrounding values

        long peak_hole = 0;
        long peak_x = 0;
        long peak_y = 0;

        long deepness = 0;
        long heights = 0;
        char div = 0;

        for (short x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) { //in der matrix ist alles von 1 an (?) -> also vermutlich anzahl = indexmax.
            for (short y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {

                heights = 0;
                div = 0;

                //circle around matrixposition
                for (short xx = x - 1; xx <= x + 1; xx++) {
                    for (short yy = y - 1; yy <= y + 1; yy++) {                                                     //iterate all points
                        if (xx != x || yy != y) {                                                                   //nicht den punkt in der mitte
                            if (xx <= g_uZMatrixMax[X_AXIS] && xx >= 1 && yy <= g_uZMatrixMax[Y_AXIS] && yy >= 1) { //nicht punkte ausserhalb der matrix
                                heights += (long)g_ZCompensationMatrix[xx][yy];
                                div += 1;
                            }
                        }
                    }
                }
                deepness = (long)((float)heights / div) - g_ZCompensationMatrix[x][y]; //nur täler, negative werte.
                if (deepness > peak_hole && div > 3) {                                 //nicht an ecken, sonst immer das tiefste loch suchen.
                    peak_hole = deepness;
                    peak_x = x;
                    peak_y = y;
                }
            }
        }

        //Com::printFLN( PSTR( "fixKeramikLochInMatrix(): 3 Extremwert" ) );
        //Com::printF( PSTR( "peak_x = " ), peak_x );
        //Com::printF( PSTR( "; peak_y = " ), peak_y );
        //Com::printF( PSTR( "; peak_hole = " ), peak_hole );
        //Com::printFLN( PSTR( "g_ZCompensationMatrix[peak_x,peak_y] =" ), g_ZCompensationMatrix[peak_x][peak_y] );

        if (peak_hole > 100 && peak_x > 0 && peak_y > 0) {
            //loch groß genug
            g_ZCompensationMatrix[peak_x][peak_y] += peak_hole;
            Com::printF(PSTR("Fixed: [peak_x,peak_y]="), g_ZCompensationMatrix[peak_x][peak_y]);
            g_ZMatrixChangedInRam = 1;
        } else {
            Com::printF(PSTR("Fix not needed dh<100 dh="), peak_hole);
        }

    } else {
        Com::printFLN(Com::tError);
    }

    determineCompensationOffsetZ(); //zur sicherheit und zukünftiger kompatibilität (Popel rausrechnen, würde den minimalen Z-Abstand verändern.)

    //Com::printFLN( PSTR( "fixKeramikLochInMatrix(): finished" ) );
    return;
} // fixKeramikLochInMatrix

/**************************************************************************************************************************************/

/**************************************************************************************************************************************/

void setMatrixNull(void) {
    //Damit kann die Matrix eingeebnet werden. Anschließend mit M3902 Zx.x verschoben, dann mit dem M3900/M3901 mit Distance-Schalter ein Auto-Bed-Leveling auf die Alte Art gemacht werden.
    //Man hätte dann eine total ebene Matrix. Die Z-Kompensation ist dann faktisch aus, kann aber weiterlaufen.

    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
        // we load the z compensation matrix before its first usage because this can take some time
        prepareZCompensation();
    }
    if (g_ZCompensationMatrix[0][0] == EEPROM_FORMAT) {
        for (short x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) { //in der matrix ist alles von 1 an (?) -> also vermutlich anzahl = indexmax.
            for (short y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
                g_ZCompensationMatrix[x][y] = 0;
            }
        }
        g_ZMatrixChangedInRam = 1; //man kan die matrix mit diesem marker nun sichern.
        Com::printFLN(PSTR("Matrix set to 0!"));
    } else {
        Com::printFLN(Com::tError);
    }
    determineCompensationOffsetZ(); //zur sicherheit und zukünftiger kompatibilität (Popel rausrechnen, würde den minimalen Z-Abstand verändern.)
    return;
} // setMatrixNull

/**************************************************************************************************************************************/

/**************************************************************************************************************************************/
/**************************************************************************************************************************************/

/**************************************************************************************************************************************/
/* Diese Funktion soll die Step_Size eines Z-Steps einstellbar machen. */
/* Noch ist das statisch über eine Tabelle, aber man könnte das später auch berechnen */

void configureMANUAL_STEPS_Z(int8_t increment) {
    const unsigned short stepsize_table[NUM_ACCEPTABLE_STEP_SIZE_TABLE] PROGMEM = ACCEPTABLE_STEP_SIZE_TABLE;

    int loop = 0;
    //suche die aktuelle Einstellungsposition (oder aufgerundet) in der Tabelle:
    for (loop = 0; loop < NUM_ACCEPTABLE_STEP_SIZE_TABLE; loop++)
        if (stepsize_table[loop] >= Printer::manualMotion.steps(Z_AXIS))
            break;

    //ändere die Position in die Wunschposition:
    if (increment >= 0)
        loop += 1;
    else
        loop -= 1;

    //begrenze ringmenge
    if (loop > NUM_ACCEPTABLE_STEP_SIZE_TABLE - 1)
        loop = 0;
    else if (loop < 0)
        loop = NUM_ACCEPTABLE_STEP_SIZE_TABLE - 1;

    //nutze neuen Wert:
    Printer::manualMotion.setSteps(Z_AXIS, stepsize_table[loop]);

    HAL::eprSetInt16(EPR_RF_MOD_Z_STEP_SIZE, Printer::manualMotion.steps(Z_AXIS));
    EEPROM::updateChecksum();
} // configureMANUAL_STEPS_Z()
/**************************************************************************************************************************************/

short testExtruderTemperature(void) //Funktion ist speziell für HBS Scan.
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

short testHeatBedTemperature(void) //Funktion ist speziell für HBS Scan.
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

namespace {
struct MatrixSpan {
    uint8_t low, high;
    long distance, width;

    long depth(long lowDepth, long highDepth) const {
        return width ? lowDepth + (highDepth - lowDepth) * distance / width : lowDepth;
    }
};

MatrixSpan matrixSpan(uint8_t axis, long position) {
    const float stepsPerMm = Printer::axisStepsPerMM[axis];
    const auto coordinate = [axis, stepsPerMm](uint8_t index) {
        const short mm = axis == X_AXIS ? g_ZCompensationMatrix[index][0] : g_ZCompensationMatrix[0][index];
        return long(mm * stepsPerMm);
    };

    uint8_t low = 1;
    long start = coordinate(low);
    if (position > start) {
        for (uint8_t high = 2; high <= g_uZMatrixMax[axis]; ++high) {
            const long end = coordinate(high);
            if (position <= end)
                return {low, high, position - start, end - start};
            low = high;
            start = end;
        }
    }
    return {low, low, 0, 0};
}
} // namespace

long getZMatrixDepth(long x, long y) {
    const auto column = matrixSpan(X_AXIS, x);
    const auto row = matrixSpan(Y_AXIS, y);
    const long front = column.depth(g_ZCompensationMatrix[column.low][row.low], g_ZCompensationMatrix[column.high][row.low]);
    const long back = column.depth(g_ZCompensationMatrix[column.low][row.high], g_ZCompensationMatrix[column.high][row.high]);
    return row.depth(front, back);
}

long getZMatrixDepth_CurrentXY(void) {
    // We need the current queue steps here. No Direct, No Offsets.
    // We need to insert where the nozzle is, relative to 0 homing coordinates which are always min endstop because we dropped axis offsets.
    // Because our RF2000 Matrix is 180mm width. That is the center of the bed. left and right are spaces. -> Center buildarea starts at 0 gcode coordinates and thats what we want.
    return getZMatrixDepth(
        Printer::xyMotion.nominalSteps(X_AXIS),
        Printer::xyMotion.nominalSteps(Y_AXIS));
}

void recalculateZCompensation(void) {
    BedCompensation::Result result{0, 0.0f};
    if (Printer::doHeatBedZCompensation) {
        const auto& layers = Printer::layers;
        const BedCompensation::Geometry geometry{
            Printer::zMotion.nominalSteps(),
            layers.fadeStartSteps(),
            layers.fadeEndSteps(),
            layers.previousLayerSteps(),
            layers.currentLayerSteps(),
            g_offsetZCompensationSteps,
        };
        result = BedCompensation::calculate(geometry, [] { return getZMatrixDepth_CurrentXY(); });
        result.zCorrectionSteps += Printer::zOffset.effectiveSteps();
        if (Printer::digitCompensation.digitZEnabled())
            result.zCorrectionSteps += BedCompensation::digitZSteps(Printer::digitCompensation.filteredDigits(), Printer::axisStepsPerMM[Z_AXIS]);
    }

    const float ratio = ExtrusionCompensation::prepareRatio(result.extraExtrusionPerStep);
    // Finish quantization before disabling step interrupts.
    asm volatile("" : : "g"(ratio));
    InterruptProtectedBlock lock;
    Printer::zMotion.requestCorrection(result.zCorrectionSteps);
    Printer::extrusionCompensation.publishRatio(ratio);
}

static void showAbortScanReason(const void* scanName, ScanAbort reason) {
    switch (reason) {
        // Geplantes Canceling
    case ScanAbort::Canceled: {
        showError(scanName, PSTR(UI_TEXT_SCAN_ABORTED));
        break;
    }

    // Kabelbruch, Extruder verspannt, Stecker wackelig, Ingress,...
    case ScanAbort::StartPressure: {
        showError(scanName, PSTR("Bad start force"));
        break;
    }
    case ScanAbort::IdlePressure: {
        showError(scanName, PSTR("Bad idle force"));
        break;
    }
    case ScanAbort::DmsOverload:
        showError(scanName, PSTR("DMS overload"));
        break;
    case ScanAbort::DmsUnavailable:
        showError(scanName, PSTR("DMS read failed"));
        break;
    case ScanAbort::AveragePressure: {
        showError(scanName, PSTR("Bad force signal"));
        break;
    }

    // Hinweise auf Bugs
    case ScanAbort::AboveStart: {
        showError(scanName, PSTR("Bug SOS"));
        break;
    }
    case ScanAbort::MatrixDimensions: {
        showError(scanName, PSTR("Bug MDO"));
        break;
    }

    // Löcher im Druckbett, Druckbett verdreckt, Mechanik problem, DDP schlecht geklebt
    case ScanAbort::UnevenBed: {
        showError(scanName, PSTR("Uneven bed, dirt"), PSTR("hole, bad ground"));
        break;
    }
    case ScanAbort::StepDeltaOutOfRange: {
        // Fehler trat auf, als ich die Düse beim HBS ein Blatt Papier mitschleppen hab lassen.
        // Marius1 hatte den Fehler durch ein sehr schräg eingebautes Bett.
        showError(scanName, PSTR("Leftovers, snot"), PSTR("incrustation"));
        break;
    }

    // Z-Schraube falsch eingestellt?
    // _MAX_COMPENSATION und _ENDSTOP_PROTECTION deuten im Printermodus auf dasselbe hin, sind aber 2 unterschiedliche Stellen.
    case ScanAbort::MinEndstop: {
        showError(scanName, PSTR("Min Endstop"));
        break;
    }
    case ScanAbort::MaxEndstop: {
        showError(scanName, PSTR("Max Endstop"));
        break;
    }
    case ScanAbort::MaxCompensation: {
        showError(scanName, PSTR("Reached limit"), PSTR("max compensation"));
        break;
    }
    case ScanAbort::EndstopProtection: {
        showError(scanName, PSTR("Reached endstop"), PSTR("protection limit"));
        break;
    }

    case ScanAbort::MissingMatrix: {
        showError(scanName, PSTR("Missing or"), PSTR(UI_TEXT_INVALID_MATRIX));
        break;
    }
    case ScanAbort::MatrixOverflow: {
        showError(scanName, PSTR("Correction would"), PSTR("overflow matrix"));
        break;
    }
    case ScanAbort::MatrixHeight: {
        // Matrix hat einen Punkt über Z-Start. Das ist gefährlich hoch, weil die Nozzle theoretisch beim Fahren auf position kollidiert haben könnte.
        // Eine rein positive Matrix melden wir nicht, das ist egal, sofern die Starthöhe immer höher ist. Dafür gibts im Anschluss des Scans die genaue Meldung zur Schraubeneinstellung.
        showError(scanName, PSTR("Badly wrong"), PSTR("Z-Screw/Z-Start"));
        break;
    }
    default:
        showError(scanName, PSTR(UI_TEXT_FAIL));
    }
}

static PressureMeasurement::Settings pressureMeasurementSettings() {
    return {static_cast<uint8_t>(scanSettings.pressureReadCount), scanSettings.pressureReadDelayMs,
            static_cast<uint16_t>(scanSettings.pressureTolerance)};
}

static ScanAbort servicePressureMeasurement(PressureMeasurement& measurement, millis_t now, bool scanning) {
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

short readAveragePressure(short* average) {
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

static bool moveZ(int nSteps, bool emergencyRetreat) {
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

void restoreDefaultScanParameters(void) {
    // we must restore the default scan parameters for the heat bed scan
    scanSettings.x.startSteps = long(Printer::axisStepsPerMM[X_AXIS] * heatBedScanXStartMm);
    scanSettings.x.strideMm = heatBedScanStepMm;
    scanSettings.x.strideSteps = long(Printer::axisStepsPerMM[X_AXIS] * heatBedScanStepMm);
    scanSettings.x.endMarginSteps = long(Printer::axisStepsPerMM[X_AXIS] * heatBedScanXEndMm);
    scanSettings.x.maxPositionSteps = long((Printer::axisLengthMM[X_AXIS] - heatBedScanXEndMm) * Printer::axisStepsPerMM[X_AXIS]);

    scanSettings.y.startSteps = long(Printer::axisStepsPerMM[Y_AXIS] * heatBedScanYStartMm);
    scanSettings.y.strideMm = heatBedScanStepMm;
    scanSettings.y.strideSteps = long(Printer::axisStepsPerMM[Y_AXIS] * heatBedScanStepMm);
    scanSettings.y.endMarginSteps = long(Printer::axisStepsPerMM[Y_AXIS] * heatBedScanYEndMm);
    scanSettings.y.maxPositionSteps = long((Printer::axisLengthMM[Y_AXIS] - heatBedScanYEndMm) * Printer::axisStepsPerMM[Y_AXIS]);

    scanSettings.fastApproachSteps = short(Printer::axisStepsPerMM[Z_AXIS] * heatBedScanUpFastMm);
    scanSettings.slowApproachSteps = short(Printer::axisStepsPerMM[Z_AXIS] * heatBedScanUpSlowMm);
    scanSettings.fastRetreatSteps = short(Printer::axisStepsPerMM[Z_AXIS] * HEAT_BED_SCAN_DOWN_FAST_MM);
    scanSettings.slowRetreatSteps = short(Printer::axisStepsPerMM[Z_AXIS] * heatBedScanDownSlowMm);

    /* Maximum number of steps to scan after the Z-min switch has been reached. If the probe cannot find the surface
       within this travel, it retries before aborting.
       Note that the head bed scan matrix consists of 16 bit signed values, thus more then 32767 steps will lead to an overflow! */
    scanSettings.maxTravelSteps = short(Z_ENDSTOP_DRIVE_OVER * Printer::axisStepsPerMM[Z_AXIS]);

    scanSettings.fastStepDelayMs = heatBedScanFastStepDelayMs;
    scanSettings.slowStepDelayMs = heatBedScanSlowStepDelayMs;
    scanSettings.idleDelayMs = heatBedScanIdleDelayMs;

    scanSettings.contactPressureDelta = heatBedScanContactPressureDelta;
    scanSettings.retryPressureDelta = heatBedScanRetryPressureDelta;
    scanSettings.idlePressureDelta = heatBedScanIdlePressureDelta;
    scanSettings.idlePressureMin = heatBedScanIdlePressureMin;
    scanSettings.idlePressureMax = heatBedScanIdlePressureMax;

    scanSettings.pressureReadCount = heatBedScanPressureReads;
    scanSettings.pressureTolerance = heatBedScanPressureTolerance;
    scanSettings.pressureReadDelayMs = heatBedScanPressureReadDelayMs;
} // restoreDefaultScanParameters

void outputScanParameters(void) {
    if (Printer::debugInfo()) {
        Com::printFLN(PSTR("outputScanParameters(): current scan parameters:"));

        Com::printF(PSTR(""), Printer::axisStepsPerMM[X_AXIS]);
        Com::printFLN(PSTR(";[steps];axisStepsPerMM[X_AXIS]"));
        Com::printF(PSTR(""), Printer::axisStepsPerMM[Y_AXIS]);
        Com::printFLN(PSTR(";[steps];axisStepsPerMM[Y_AXIS]"));
        Com::printF(PSTR(""), Printer::axisStepsPerMM[Z_AXIS]);
        Com::printFLN(PSTR(";[steps];axisStepsPerMM[Z_AXIS]"));

        Com::printF(PSTR(""), scanSettings.x.startSteps);
        Com::printFLN(PSTR(";[steps];g_nScanXStartSteps"));
        Com::printF(PSTR(""), scanSettings.x.strideSteps);
        Com::printFLN(PSTR(";[steps];g_nScanXStepSizeSteps"));
        Com::printF(PSTR(""), scanSettings.x.endMarginSteps);
        Com::printFLN(PSTR(";[steps];g_nScanXEndSteps"));
        Com::printF(PSTR(""), scanSettings.x.maxPositionSteps);
        Com::printFLN(PSTR(";[steps];g_nScanXMaxPositionSteps"));

        Com::printF(PSTR(""), scanSettings.y.startSteps);
        Com::printFLN(PSTR(";[steps];g_nScanYStartSteps"));
        Com::printF(PSTR(""), scanSettings.y.strideSteps);
        Com::printFLN(PSTR(";[steps];g_nScanYStepSizeSteps"));
        Com::printF(PSTR(""), scanSettings.y.endMarginSteps);
        Com::printFLN(PSTR(";[steps];g_nScanYEndSteps"));
        Com::printF(PSTR(""), scanSettings.y.maxPositionSteps);
        Com::printFLN(PSTR(";[steps];g_nScanYMaxPositionSteps"));

        Com::printF(PSTR(""), (int)scanSettings.fastApproachSteps);
        Com::printFLN(PSTR(";[steps];g_nScanHeatBedUpFastSteps"));
        Com::printF(PSTR(""), (int)scanSettings.slowApproachSteps);
        Com::printFLN(PSTR(";[steps];g_nScanHeatBedUpSlowSteps"));
        Com::printF(PSTR(""), (int)scanSettings.fastRetreatSteps);
        Com::printFLN(PSTR(";[steps];g_nScanHeatBedDownFastSteps"));
        Com::printF(PSTR(""), (int)scanSettings.slowRetreatSteps);
        Com::printFLN(PSTR(";[steps];g_nScanHeatBedDownSlowSteps"));
        Com::printF(PSTR(""), (int)scanSettings.fastStepDelayMs);
        Com::printFLN(PSTR(";[ms];g_nScanFastStepDelay"));
        Com::printF(PSTR(""), (int)scanSettings.slowStepDelayMs);
        Com::printFLN(PSTR(";[ms];g_nScanSlowStepDelay"));
        Com::printF(PSTR(""), (int)scanSettings.idleDelayMs);
        Com::printFLN(PSTR(";[ms];g_nScanIdleDelay"));

        Com::printF(PSTR(""), (int)scanSettings.contactPressureDelta);
        Com::printFLN(PSTR(";[digits];g_nScanContactPressureDelta"));
        Com::printF(PSTR(""), (int)scanSettings.retryPressureDelta);
        Com::printFLN(PSTR(";[digits];g_nScanRetryPressureDelta"));
        Com::printF(PSTR(""), (int)scanSettings.idlePressureDelta);
        Com::printFLN(PSTR(";[digits];g_nScanIdlePressureDelta"));

        Com::printF(PSTR(""), (int)scanSettings.pressureReadCount);
        Com::printFLN(PSTR(";[-];g_nScanPressureReads"));
        Com::printF(PSTR(""), (int)scanSettings.pressureTolerance);
        Com::printFLN(PSTR(";[digits];g_nScanPressureTolerance"));
        Com::printF(PSTR(""), (int)scanSettings.pressureReadDelayMs);
        Com::printFLN(PSTR(";[ms];g_nScanPressureReadDelay"));
    }
} // outputScanParameters

static void initCompensationMatrix(void) {
    // clear all fields of the compensation matrix
    memset(g_ZCompensationMatrix, 0, sizeof(g_ZCompensationMatrix));
    g_uZMatrixMax[X_AXIS] = g_uZMatrixMax[Y_AXIS] = 0;
    g_offsetZCompensationSteps = 0;
    g_ZCompensationMax = 0;
    g_ZMatrixChangedInRam = 0;
} // initCompensationMatrix

static void outputCompensationMatrix(char format) {
    short x;
    short y;

    Com::printFLN(PSTR("front left ... front right"));
    Com::printFLN(PSTR("...        ...         ..."));
    Com::printFLN(PSTR("back left  ...  back right"));

    for (y = 0; y <= g_uZMatrixMax[Y_AXIS]; y++) {
        for (x = 0; x <= g_uZMatrixMax[X_AXIS]; x++) {
            if (x == 0 || y == 0) {
                Com::printF(Com::tSemiColon, g_ZCompensationMatrix[x][y]);
            } else {
                if (format) {
                    // output in [mm]
                    Com::printF(Com::tSemiColon, g_ZCompensationMatrix[x][y] / Printer::axisStepsPerMM[Z_AXIS]);
                } else {
                    // output in [steps]
                    Com::printF(Com::tSemiColon, g_ZCompensationMatrix[x][y]);
                }
            }
        }
        Com::printFLN(PSTR(" "));
    }

    Com::printF(PSTR("offset = "), g_offsetZCompensationSteps);
    Com::printF(PSTR(" [steps] (= "), (float)g_offsetZCompensationSteps * Printer::axisMMPerSteps[Z_AXIS]);
    Com::printFLN(PSTR(" [mm])"));

    Com::printF(PSTR("warpage = "), g_ZCompensationMax - g_offsetZCompensationSteps);
    Com::printF(PSTR(" [steps] (= "), float(g_ZCompensationMax - g_offsetZCompensationSteps) * Printer::axisMMPerSteps[Z_AXIS]);
    Com::printFLN(PSTR(" [mm])"));

    Com::printFLN(PSTR("g_uZMatrixMax[X_AXIS] = "), g_uZMatrixMax[X_AXIS]);
    Com::printFLN(PSTR("g_uZMatrixMax[Y_AXIS] = "), g_uZMatrixMax[Y_AXIS]);

    {
        Com::printFLN(PSTR("g_nActiveHeatBed = "), g_nActiveHeatBed);
    }

} // outputCompensationMatrix

static char prepareCompensationMatrix(void) {
    short x;
    short y;

    // perform some safety checks first
    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("zCMP: invalid version: "), g_ZCompensationMatrix[0][0]);
            Com::printF(PSTR(" (expected: "), EEPROM_FORMAT);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }

    if (g_uZMatrixMax[X_AXIS] >= COMPENSATION_MATRIX_MAX_X || g_uZMatrixMax[X_AXIS] < 2) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("zCMP: invalid x dimension: "), g_uZMatrixMax[X_AXIS]);
            Com::printF(PSTR(" (max expected: "), COMPENSATION_MATRIX_MAX_X - 1);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }

    if (g_uZMatrixMax[Y_AXIS] >= COMPENSATION_MATRIX_MAX_Y || g_uZMatrixMax[Y_AXIS] < 2) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("zCMP: invalid y dimension: "), g_uZMatrixMax[Y_AXIS]);
            Com::printF(PSTR(" (max expected: "), COMPENSATION_MATRIX_MAX_Y - 1);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }

    const bool padLeft = g_ZCompensationMatrix[2][0] > 0;
    const bool padFront = g_ZCompensationMatrix[0][2] > 0;
    const bool padRight = g_ZCompensationMatrix[g_uZMatrixMax[X_AXIS]][0] < short(Printer::axisLengthMM[X_AXIS]);
    const bool padBack = g_ZCompensationMatrix[0][g_uZMatrixMax[Y_AXIS]] < short(Printer::axisLengthMM[Y_AXIS]);
    const int lastX = g_uZMatrixMax[X_AXIS] - !padLeft + padRight;
    const int lastY = g_uZMatrixMax[Y_AXIS] - !padFront + padBack;
    if (lastX < 2 || lastX >= COMPENSATION_MATRIX_MAX_X || lastY < 2 || lastY >= COMPENSATION_MATRIX_MAX_Y) {
        Com::printFLN(PSTR("zCMP: prepared matrix dimensions out of range"));
        return -1;
    }

    if (padLeft) {
        // we have to fill x[1] with the values of x[2]
        /*      if( Printer::debugInfo() )
        {
            Com::printFLN( PSTR( "prepareCompensationMatrix(): x[2] > 0" ) );
        }
*/
        g_ZCompensationMatrix[1][0] = 0;
        for (y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
            g_ZCompensationMatrix[1][y] = g_ZCompensationMatrix[2][y];
        }
    } else {
        // we have to shift all x columns one index
        for (x = 1; x < g_uZMatrixMax[X_AXIS]; x++) {
            for (y = 0; y <= g_uZMatrixMax[Y_AXIS]; y++) {
                g_ZCompensationMatrix[x][y] = g_ZCompensationMatrix[x + 1][y];
            }
        }

        // we have one x column less now
        g_uZMatrixMax[X_AXIS]--;
    }

    if (padRight) {
        // we have to fill x[g_uZMatrixMax[X_AXIS]] with the values of x[g_uZMatrixMax[X_AXIS]-1]
        g_ZCompensationMatrix[g_uZMatrixMax[X_AXIS] + 1][0] = short(Printer::axisLengthMM[X_AXIS]);
        for (y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
            g_ZCompensationMatrix[g_uZMatrixMax[X_AXIS] + 1][y] = g_ZCompensationMatrix[g_uZMatrixMax[X_AXIS]][y];
        }

        // we have one x column more now
        g_uZMatrixMax[X_AXIS]++;
    } else {
        // there is nothing else to do here
    }

    if (padFront) {
        // we have to fill y[1] with the values of y[2]
        g_ZCompensationMatrix[0][1] = 0;
        for (x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) {
            g_ZCompensationMatrix[x][1] = g_ZCompensationMatrix[x][2];
        }
    } else {
        // we have to shift all y columns one index
        for (x = 0; x <= g_uZMatrixMax[X_AXIS]; x++) {
            for (y = 1; y < g_uZMatrixMax[Y_AXIS]; y++) {
                g_ZCompensationMatrix[x][y] = g_ZCompensationMatrix[x][y + 1];
            }
        }

        // we have one y column less now
        g_uZMatrixMax[Y_AXIS]--;
    }

    if (padBack) {
        // we have to fill y[g_uZMatrixMax[Y_AXIS]] with the values of y[g_uZMatrixMax[Y_AXIS]-1]
        g_ZCompensationMatrix[0][g_uZMatrixMax[Y_AXIS] + 1] = short(Printer::axisLengthMM[Y_AXIS]);
        for (x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) {
            g_ZCompensationMatrix[x][g_uZMatrixMax[Y_AXIS] + 1] = g_ZCompensationMatrix[x][g_uZMatrixMax[Y_AXIS]];
        }

        // we have one y column more now
        g_uZMatrixMax[Y_AXIS]++;
    }

    // determine the minimal distance between extruder and heat bed
    determineCompensationOffsetZ();

    return 0;
} // prepareCompensationMatrix

static void determineCompensationOffsetZ(void) {
    short x;
    short y;
    short uMax = -32768;
    short uMin = 32767;

    for (x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) {
        for (y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
            if (g_ZCompensationMatrix[x][y] > uMax) {
                uMax = g_ZCompensationMatrix[x][y];
            }
            if (g_ZCompensationMatrix[x][y] < uMin) {
                uMin = g_ZCompensationMatrix[x][y];
            }
        }
    }
    g_ZCompensationMax = uMin;
    g_offsetZCompensationSteps = uMax;
} // determineCompensationOffsetZ

static void adjustCompensationMatrix(short nZ) {
    short x;
    short y;
    short nOffset = getZMatrixDepth_CurrentXY();
    short deltaZ = nZ - nOffset;

    for (x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) {
        for (y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
            g_ZCompensationMatrix[x][y] += deltaZ;
        }
    }

    // determine the minimal distance between extruder and heat bed
    determineCompensationOffsetZ();
} // adjustCompensationMatrix

static bool writeMatrixSector(unsigned int address, bool clear) {
    const uint8_t eeprom = I2C_ADDRESS_EXTERNAL_EEPROM;
    const unsigned int formatAddress = address + EEPROM_OFFSET_SECTOR_FORMAT;
    const unsigned short format = clear ? 0 : EEPROM_FORMAT;
    const uint8_t lastX = clear ? COMPENSATION_MATRIX_MAX_X - 1 : g_uZMatrixMax[X_AXIS];
    const uint8_t lastY = clear ? COMPENSATION_MATRIX_MAX_Y - 1 : g_uZMatrixMax[Y_AXIS];
    const unsigned short microsteps = clear ? 0 : drv8711ModeValue_2_MicroSteps(Printer::motorMicroStepsModeValue[Z_AXIS]);

    // Invalidate the sector before replacing any payload. A reset mid-save
    // must leave this sector unavailable to the loader.
    unsigned short storedFormat;
    if (!writeWord24C256(eeprom, formatAddress, 0)
        || !readWord24C256(eeprom, formatAddress, storedFormat) || storedFormat != 0
        || !writeWord24C256(eeprom, EEPROM_OFFSET_HEADER_FORMAT, EEPROM_FORMAT)
        || !writeWord24C256(eeprom, address + EEPROM_OFFSET_DIMENSION_X, clear ? 0 : lastX)
        || !writeWord24C256(eeprom, address + EEPROM_OFFSET_DIMENSION_Y, clear ? 0 : lastY)
        || !writeWord24C256(eeprom, address + EEPROM_OFFSET_MICRO_STEPS, microsteps)) {
        return false;
    }

    unsigned int offset = address + EEPROM_OFFSET_MATRIX_START;
    for (uint8_t x = 0; x <= lastX; ++x) {
        for (uint8_t y = 0; y <= lastY; ++y) {
            if (!writeWord24C256(eeprom, offset, clear ? 0 : g_ZCompensationMatrix[x][y]))
                return false;
            offset += 2;
        }
        GCodeInput::keepAlive(Processing);
    }

    // A read after the marker write also waits for the last EEPROM write cycle.
    return (clear || writeWord24C256(eeprom, formatAddress, format))
        && readWord24C256(eeprom, formatAddress, storedFormat) && storedFormat == format;
}

static bool saveWorkingMatrix(unsigned int address) {
    if (!address || g_ZCompensationMatrix[0][0] != EEPROM_FORMAT
        || g_uZMatrixMax[X_AXIS] < 2 || g_uZMatrixMax[X_AXIS] >= COMPENSATION_MATRIX_MAX_X
        || g_uZMatrixMax[Y_AXIS] < 2 || g_uZMatrixMax[Y_AXIS] >= COMPENSATION_MATRIX_MAX_Y) {
        Com::printFLN(PSTR("saveMatrix(): invalid matrix"));
        return false;
    }
    if (!writeMatrixSector(address, false)) {
        Com::printFLN(PSTR("saveMatrix(): EEPROM write failed"));
        return false;
    }

    determineCompensationOffsetZ();
    if (address == static_cast<unsigned int>(EEPROM_SECTOR_SIZE * g_nActiveHeatBed))
        g_ZMatrixChangedInRam = 0;
    return true;
}

static bool saveCompensationMatrix(unsigned int address) {
    if (isAnyScanRunning()) {
        Com::printFLN(PSTR("saveMatrix(): scan in progress"));
        return false;
    }
    return saveWorkingMatrix(address);
}

bool saveActiveBedMatrix() {
    return saveCompensationMatrix(static_cast<unsigned int>(EEPROM_SECTOR_SIZE * g_nActiveHeatBed));
}

bool saveActiveHeatBedToExtEEPROM(char newActiveHeatBed) {
    if (newActiveHeatBed < 1 || newActiveHeatBed > EEPROM_MAX_HEAT_BED_SECTORS) {
        return false;
    }

    const uint8_t device = I2C_ADDRESS_EXTERNAL_EEPROM;
    const unsigned int address = EEPROM_OFFSET_ACTIVE_HEAT_BED_Z_MATRIX;
    unsigned short stored;
    if (readWord24C256(device, address, stored) && stored == static_cast<unsigned short>(newActiveHeatBed))
        return true;
    if (writeWord24C256(device, address, newActiveHeatBed)
        && readWord24C256(device, address, stored) && stored == static_cast<unsigned short>(newActiveHeatBed))
        return true;
    Com::printFLN(PSTR("selectMatrix(): EEPROM write failed"));
    return false;
}

bool loadActiveHeatBedFromExtEEPROM() {
    const unsigned short slot = readWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_OFFSET_ACTIVE_HEAT_BED_Z_MATRIX);

    if (slot < 1 || slot > EEPROM_MAX_HEAT_BED_SECTORS) {
        Com::printFLN(PSTR("Invalid active heat bed within ext. EEPROM detected: "), uint32_t(slot));
        if (g_nActiveHeatBed < 1 || g_nActiveHeatBed > EEPROM_MAX_HEAT_BED_SECTORS) {
            g_nActiveHeatBed = 1;
        }
        saveActiveHeatBedToExtEEPROM(g_nActiveHeatBed);

        return false;
    }

    g_nActiveHeatBed = slot;
    return true;
}

static char loadCompensationMatrix(unsigned int uAddress) {
    if (uAddress == 0) {
        Com::printFLN(PSTR("loadMatrix(): valid uAddress - aborted!"));

        return -1;
    }

    unsigned short uTemp;
    unsigned short uDimensionX;
    unsigned short uDimensionY;
    unsigned short uMicroSteps;
    unsigned int uOffset;
    short x;
    short y;
    float fMicroStepCorrection;

    Printer::disableCMPnow(true); // vorher, nicht nachher ausschalten, sonst arbeitet unter Umständen die alte matrix.
    // The payload is read into the working buffer. Publish the format marker
    // and dimensions only after the whole sector has been accepted.
    g_ZCompensationMatrix[0][0] = 0;

    // check the stored header format
    if (!readWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_OFFSET_HEADER_FORMAT, uTemp))
        return -1;
    if (uTemp != EEPROM_FORMAT) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("loadMatrix(): invalid header format: "), (int)uTemp);
            Com::printF(PSTR(" (expected: "), EEPROM_FORMAT);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }


    // check the stored sector format
    if (!readWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_SECTOR_FORMAT, uTemp))
        return -1;
    if (uTemp != EEPROM_FORMAT) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("loadMatrix(): invalid sector format: "), (int)uTemp);
            Com::printF(PSTR(" (expected: "), EEPROM_FORMAT);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }

    // check the stored x dimension
    if (!readWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_DIMENSION_X, uDimensionX))
        return -1;
    if (uDimensionX < 2 || uDimensionX >= COMPENSATION_MATRIX_MAX_X) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("loadMatrix(): invalid x dimension: "), (int)uDimensionX);
            Com::printF(PSTR(" (max expected: "), COMPENSATION_MATRIX_MAX_X - 1);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }

    // check the stored y dimension
    if (!readWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_DIMENSION_Y, uDimensionY))
        return -1;
    if (uDimensionY < 2 || uDimensionY >= COMPENSATION_MATRIX_MAX_Y) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("loadMatrix(): invalid y dimension: "), (int)uDimensionY);
            Com::printF(PSTR(" (max expected: "), COMPENSATION_MATRIX_MAX_Y - 1);
            Com::printFLN(PSTR(")"));
        }
        return -1;
    }

    // check the stored microsteps
    if (!readWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_MICRO_STEPS, uMicroSteps))
        return -1;
    if (!uMicroSteps || uMicroSteps > 256 || (uMicroSteps & (uMicroSteps - 1))) {
        Com::printFLN(PSTR("loadMatrix(): invalid microstep count: "), uint32_t(uMicroSteps));
        return -1;
    }

    if (uMicroSteps ==
        drv8711ModeValue_2_MicroSteps(Printer::motorMicroStepsModeValue[Z_AXIS])
    ) {
        // the current z-compensation matrix has been determined with the current micro step setting, there is nothing to recalculate
        fMicroStepCorrection = 1.0;
    } else {
        // the current z-compensation matrix has been determined with a higher than the current micro step setting, we must divide all z-correction values
        fMicroStepCorrection = (float)
                                   drv8711ModeValue_2_MicroSteps(Printer::motorMicroStepsModeValue[Z_AXIS])
            / (float)uMicroSteps;

        Com::printF(PSTR("loadMatrix(): micro step correction = "), fMicroStepCorrection);
        Com::printF(PSTR(" (stored="), (int)uMicroSteps);
        Com::printF(PSTR(", current="), (int)
                                            drv8711ModeValue_2_MicroSteps(Printer::motorMicroStepsModeValue[Z_AXIS])
        );
        Com::printFLN(PSTR(")"));
    }

    // read out the actual compensation values
    uOffset = uAddress + EEPROM_OFFSET_MATRIX_START;
    for (x = 0; x <= uDimensionX; x++) {
        for (y = 0; y <= uDimensionY; y++) {
            if (!readWord24C256(I2C_ADDRESS_EXTERNAL_EEPROM, uOffset, uTemp)) {
                Com::printFLN(PSTR("loadMatrix(): EEPROM read failed"));
                return -1;
            }
            const int16_t value = static_cast<int16_t>(uTemp);
            if (x == 0 && y == 0) {
                if (value != EEPROM_FORMAT) {
                    Com::printFLN(PSTR("loadMatrix(): invalid matrix format: "), value);
                    return -1;
                }
            } else if (x == 0 || y == 0) {
                // we must not modify our header row/column
                g_ZCompensationMatrix[x][y] = value;
            } else {
                // we may have to update all z-compensation values
                long transformedMatrixElement = long((float)value * fMicroStepCorrection);
                if (transformedMatrixElement > 32767 || transformedMatrixElement < -32768) {
                    Com::printFLN(PSTR("MicroStep transf.: overflow"));

                    return -1;
                }
                g_ZCompensationMatrix[x][y] = short(transformedMatrixElement);
            }
            uOffset += 2;
        }
        GCodeInput::keepAlive(Processing);
    }

    g_uZMatrixMax[X_AXIS] = uDimensionX;
    g_uZMatrixMax[Y_AXIS] = uDimensionY;
    determineCompensationOffsetZ();
    g_ZCompensationMatrix[0][0] = EEPROM_FORMAT;
    g_ZMatrixChangedInRam = 0; //Nibbels: Marker, dass die Matrix gespeichert werden kann oder eben nicht, weils unverändert keinen Sinn macht.

    return 0;
} // loadCompensationMatrix

static bool clearCompensationMatrix(unsigned int address) {
    // The caller has selected this sector. Invalidate the RAM copy even if the
    // sector could only be partially cleared.
    initCompensationMatrix();
    if (!writeMatrixSector(address, true)) {
        Com::printFLN(PSTR("clearMatrix(): EEPROM write failed"));
        return false;
    }
    if (Printer::debugInfo())
        Com::printFLN(PSTR("clearMatrix(): matrix cleared"));
    return true;
}

static void clearExternalEEPROM(void) {
    unsigned short i;
    unsigned short uMax = 32768;
    unsigned short uTemp;
    unsigned short uLast = 0;

    if (Printer::debugInfo()) {
        Com::printFLN(PSTR("clearExtEEPROM(): erasing chip memory ..."));
    }

    // the external EEPROM is able to store 262.144 kBit (= 32.768 kByte)
    for (i = 0; i < uMax; i++) {
        if (!writeByte24C256(I2C_ADDRESS_EXTERNAL_EEPROM, i, 0)) {
            Com::printFLN(PSTR("clearExtEEPROM(): EEPROM write failed"));
            return;
        }
        Commands::checkForPeriodicalActions(Processing);

        if (Printer::debugInfo()) {
            uTemp = i / 100;
            if (uTemp != uLast) {
                Com::printF(PSTR("clearExtEEPROM(): "), (int)i);
                Com::printFLN(PSTR(" / "), (long)uMax);
                uLast = uTemp;
            }
        }
    }

    if (Printer::debugInfo()) {
        Com::printFLN(PSTR("clearExtEEPROM(): erasing complete"));
    }
} // clearExternalEEPROM

static bool writeByte24C256(int addressI2C, unsigned int addressEEPROM, unsigned char data) {
    TwiTransfer::finishActive();
    HAL::delayMilliseconds(EEPROM_DELAY);
    const uint8_t bytes[] = {byte(addressEEPROM >> 8), byte(addressEEPROM), data};
    TwiTransfer transfer;
    return transfer.write(addressI2C, bytes, sizeof(bytes), HAL::timeInMilliseconds()) && transfer.wait();
}

static bool writeWord24C256(int addressI2C, unsigned int addressEEPROM, unsigned short data) {
    return writeByte24C256(addressI2C, addressEEPROM, byte(data >> 8))
        && writeByte24C256(addressI2C, addressEEPROM + 1, byte(data));
}

static bool readByte24C256(int addressI2C, unsigned int addressEEPROM, uint8_t& data) {
    TwiTransfer::finishActive();
    HAL::delayMilliseconds(EEPROM_DELAY);
    const uint8_t bytes[] = {byte(addressEEPROM >> 8), byte(addressEEPROM)};
    TwiTransfer transfer;
    if (!transfer.write(addressI2C, bytes, sizeof(bytes), HAL::timeInMilliseconds()) || !transfer.wait()
        || !transfer.read(addressI2C, 1, HAL::timeInMilliseconds()) || !transfer.wait())
        return false;
    data = transfer.byteAt(0);
    return true;
} // readByte24C256

static bool readWord24C256(int addressI2C, unsigned int addressEEPROM, unsigned short& data) {
    uint8_t high, low;
    if (!readByte24C256(addressI2C, addressEEPROM, high) || !readByte24C256(addressI2C, addressEEPROM + 1, low))
        return false;
    data = (uint16_t(high) << 8) | low;
    return true;
} // readWord24C256

static unsigned short readWord24C256(int addressI2C, unsigned int addressEEPROM) {
    unsigned short data;
    return readWord24C256(addressI2C, addressEEPROM, data) ? data : 0xffff;
}

static void serviceSenseOffset(bool sampleDue, short pressure, const PrintPause::SenseOffsetSample& pauseSample) {
    auto& controller = Printer::senseOffsetControl;
    auto& offset = Printer::zOffset;

    if (!Printer::doHeatBedZCompensation) {
        const bool wasEnabled = controller.disable();
        if (offset.senseMicrometers()) {
            Com::printF(PSTR("SensiblePressure(): offset reverted from "), offset.senseMicrometers());
            Com::printFLN(PSTR(" [um] to 0 [um] "));
            offset.clearSenseOffset(Printer::axisStepsPerMM[Z_AXIS]);
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

    const auto increment = offset.prepareSenseIncrement(step, limits.maxMicrometers, Printer::axisStepsPerMM[Z_AXIS]);
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
        doEmergencyStop(STOP_BECAUSE_OF_Z_BLOCK);
}

void handleScanWorkTasks(bool tickDue) {
    if (scanSession.dispatching() || !isAnyScanRunning())
        return;
    if (Printer::blockAll || Printer::printJob.commandsBlocked()) {
        discardScans();
        return;
    }
    if (Printer::pause.isPaused())
        return;

    scanSession.beginDispatch();
    const bool measurementReady = !scanSession.aborting() && scanPressure.service(HAL::timeInMilliseconds());
    if (tickDue || measurementReady) {
        if (heatBedScan.active())
            heatBedScan.tick();
        else
            zOffsetScan.tick();
    }
    if (scanSession.finishDispatch() || Printer::blockAll || Printer::printJob.commandsBlocked())
        discardScans();
}

void handleStartStandby(millis_t uTime) {
    /**
     * Check if the Fan shall be turned off.
     */
#if (MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2) && CASE_FAN_PIN > -1
    if (Printer::prepareFanOff) {
        if (Printer::prepareFanOff > uTime) {
            return;
        }
        if ((uTime - Printer::prepareFanOff) > Printer::fanOffDelay) //60s wäre standard nach config
        {
            // it is time to turn the case fan off
            Printer::prepareFanOff = 0;
            if (!Printer::ignoreFanOn)
                WRITE(CASE_FAN_PIN, 0);
        }
    }
#endif // (MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2) && CASE_FAN_PIN > -1

    if (!Printer::pause.isPaused() && !Printer::startLine.active()) {
        const auto idleTransition = Printer::idleStatus.poll(uTime, MINIMAL_IDLE_TIME);
        if (idleTransition == IdleStatus::Transition::Deferred)
            return;
        if (idleTransition == IdleStatus::Transition::Ready && !PrintLine::hasLines() && !PrintLine::directMoveActive()) {
            Printer::printJob.finishPrintMotion();
            showIdle();
        }
    }

    /**
     * Check if the Printer should turn off Steppers or Everything
     */
    // A preceding service may have recorded activity after the initial clock sample.
    const millis_t inactivityTime = HAL::timeInMilliseconds();
    if (PrintLine::hasLines() || Printer::printJob.isBusy() || Printer::pause.isPaused()) {
        Printer::inactivity.recordActivity(inactivityTime);
    } else {
        if (Printer::inactivity.shutdownTimeoutExpired(inactivityTime)) {
            Printer::switchEverythingOff(); //kill not only steppers
        } else {
            Printer::setAllSwitchedOff(false); // reset if not time to kill: prevents repeated kills
        }

        if (Printer::inactivity.stepperTimeoutExpired(inactivityTime) && !Printer::areAllSteppersDisabled()) {
            Printer::disableAllSteppersNow();
            Commands::setFanSpeed((uint8_t)0);
        }
    }
}

void outputObject(bool showerrors) {
    if (PrintLine::hasLines() || !Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Cancel)) {
        if (showerrors)
            showError((void*)ui_text_output_object, (void*)ui_text_operation_denied);
        return;
    }
    if (!Printer::areAxisHomed()) {
        if (showerrors)
            showError((void*)ui_text_output_object, (void*)ui_text_home_unknown);
        return;
    }

    Printer::idleStatus.cancel();
    UI_STATUS_UPD(UI_TEXT_OUTPUTTING_OBJECT);
    Com::printFLN(PSTR("outputObject"));
    Commands::printCurrentPosition();

    // disable the fan
    Commands::setFanSpeed((uint8_t)0);

    {
        GCodeInput::executeFlashScript(Com::tOutputObjectPrint);
    }

    if (!Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Cancel))
        return;

    // disable all steppers
    Printer::disableAllSteppersNow();

    Printer::idleStatus.schedule(HAL::timeInMilliseconds());
} // outputObject

void loopFeatures() //wird so aufgerufen, dass es ein ~100ms Takt sein sollte.
{
    static char nEntered = 0;
    if (!nEntered) {
        nEntered++;

        millis_t uTime = HAL::timeInMilliseconds();
        Printer::printJob.service(uTime);

        handleStartStandby(uTime);
        handleStrainGaugeFeatures(uTime);

        Printer::pause.serviceStandby(uTime);

#ifdef RGB_LIGHT_R_PIN
        updateRGBLightStatus();
#endif // RGB_LIGHT_R_PIN

        nEntered--;
    }

    // Direct-move waits can re-enter this loop. Only pending requests start a transition.
    Printer::pause.serviceMotion();
} // loopFeatures

void setExtruderCurrent(uint8_t nr, uint8_t current) {
    if (nr > NUM_EXTRUDER - 1)
        nr = NUM_EXTRUDER - 1; //0 ist T0, 1 ist T1


    // set the current for the extruder motor

    setMotorCurrent(1 + E_AXIS + nr, current); //das ist Extruder-ID+1, also nicht [0,1,2] -> Extruder 4 = [3], sondern [4]. Extruder5 wäre [5]

    if (Printer::debugInfo()) {
        Com::printF(PSTR("Extruder"), (short)nr);
        Com::printFLN(PSTR(" = "), (uint32_t)current);
    }
} // setExtruderCurrent

static bool prepareFilamentProgram() {
    if (Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Cancel)) {
        Printer::printJob.beginFilamentService();
        return true;
    }
    Com::printFLN(PSTR("Filament program requires settled, unpaused motion"));
    return false;
}

namespace {

// Bed compensation and scan parameters.
void disableBedCompensation() {
    if (Printer::debugInfo()) {
        Com::printFLN(PSTR("M3000: disabling z compensation"));
    }
    queueTask(TASK_DISABLE_Z_COMPENSATION);
}

void enableBedCompensation() {
    if (isAnyScanRunning()) {
        Com::printFLN(PSTR("M3001: z compensation is unavailable during a scan"));
        return;
    }
    if (Printer::doHeatBedZCompensation) {
        Com::printFLN(PSTR("M3001: z compensation is enabled already"));
        return;
    }

    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
        // we try to load the z compensation matrix before its first usage because this can take some time
        prepareZCompensation();
    }

    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
        Com::printF(PSTR("M3001: z compensation can not be enabled. Heat bed compensation matrix not valid ( "), g_ZCompensationMatrix[0][0]);
        Com::printF(PSTR(" / "), EEPROM_FORMAT);
        Com::printFLN(PSTR(" )"));
        return;
    }

    if (!Printer::areAxisHomed()) {
        Com::printFLN(PSTR("M3001: z compensation can not be enabled. Home position is unknown"));
        showError((void*)ui_text_z_compensation, (void*)ui_text_home_unknown);
        return;
    }

    queueTask(TASK_ENABLE_Z_COMPENSATION);

    //wenn der nutzer im menü "autostart" gewählt hat, dann senseoffset sofort mit M3001 mitstarten.
    //Die Einstellwerte kommen dann aus dem EEPROM
    if (Printer::senseOffsetControl.autoStart())
        queueTask(TASK_ENABLE_SENSE_OFFSET);
}

void configureRfDebug(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 0)
            nTemp = 0;

        g_debugLevel = (char)nTemp;
        if (Printer::debugInfo()) {
            Com::printFLN(PSTR("M3005: new debug level: "), g_debugLevel);
        }
    } else {
        showInvalidSyntax(pCommand->M);
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

    Printer::zOffset.setManualMicrometers(nTemp, Printer::axisStepsPerMM[Z_AXIS]);
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

void selectBedMatrix(GCode* pCommand) {
    long nTemp;
    if (Printer::doHeatBedZCompensation) {
        // do not allow to change the current heat bed z-compensation matrix while the z-compensation is active
        if (Printer::debugErrors()) {
            Com::printFLN(PSTR("M3009: heat bed z matrix can not be changed while z-compensation is active"));
        }

        showError((void*)ui_text_z_compensation, (void*)ui_text_operation_denied);
        return;
    }

    if (pCommand->hasS()) {
        nTemp = pCommand->S;

        if (nTemp < 1 || nTemp > EEPROM_MAX_HEAT_BED_SECTORS) {
            if (Printer::debugErrors()) {
                Com::printF(PSTR("M3009: invalid heat bed z matrix ("), nTemp);
                Com::printFLN(PSTR(")"));
            }
            return;
        }

        if (g_nActiveHeatBed != nTemp) {
            // we have to switch to another heat bed z-compensation matrix
            switchActiveHeatBed((char)nTemp);
        }
    }

    if (Printer::debugInfo()) {
        Com::printFLN(PSTR("M3009: currently active heat bed z matrix: "), g_nActiveHeatBed);
    }
}

void requestHeatBedScan(GCode* pCommand) {
    if (pCommand->hasS() && (pCommand->S == int(HeatBedScanMode::Pla) || pCommand->S == int(HeatBedScanMode::Abs))) {
        startHeatBedScan(static_cast<HeatBedScanMode>(pCommand->S));
        return;
    }
    startHeatBedScan(HeatBedScanMode::Standard);
}

void clearBedMatrix(GCode* pCommand) {
    long nTemp;
    if (isAnyScanRunning()) {
        Com::printFLN(PSTR("M3011: scan in progress"));
        return;
    }
    if (Printer::doHeatBedZCompensation) {
        if (Printer::debugErrors()) {
            Com::printFLN(PSTR("M3011: the heat bed z matrix can not be cleared while the z-compensation is active"));
        }

        showError((void*)ui_text_z_compensation, (void*)ui_text_operation_denied);
        return;
    }
    if (pCommand->hasS()) {
        nTemp = pCommand->S;
    } else {
        // we clear the current z-compensation matrix in case no other z-compensation matrix is specified
        nTemp = g_nActiveHeatBed;
    }

    if (nTemp < 1 || nTemp > EEPROM_MAX_HEAT_BED_SECTORS) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("M3011: invalid heat bed z matrix ("), nTemp);
            Com::printFLN(PSTR(")"));
        }
        return;
    }

    if (saveActiveHeatBedToExtEEPROM((char)nTemp)) {
        // switch to the specified z-compensation matrix
        g_nActiveHeatBed = (char)nTemp;
        if (clearCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed)))
            Com::printFLN(PSTR("M3011: cleared heat bed z matrix: "), nTemp);
    }
}

void reportBedMatrix(GCode* pCommand) {
    long nTemp;
    char format;
    char nOldHeatBed = 127;

    if (pCommand->hasS()) {
        nTemp = pCommand->S;
    } else {
        // we output the current z-compensation matrix in case no other z-compensation matrix is specified
        nTemp = g_nActiveHeatBed;
    }

    if (pCommand->hasP()) {
        format = pCommand->P ? 1 : 0;
    } else {
        format = 0;
    }

    if (nTemp < 1 || nTemp > EEPROM_MAX_HEAT_BED_SECTORS) {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("M3013: invalid heat bed z matrix ("), nTemp);
            Com::printFLN(PSTR(")"));
        }
        return;
    }

    if (g_nActiveHeatBed != nTemp) {
        // remember current z-compensation matrix
        nOldHeatBed = g_nActiveHeatBed;
        // we have to switch to another z-compensation matrix
        if (!switchActiveHeatBed((char)nTemp))
            return;
    }

    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
        // we load the z-compensation matrix before its first usage because this can take some time
        prepareZCompensation();
    }

    if (g_ZCompensationMatrix[0][0] == EEPROM_FORMAT) {
        if (Printer::debugInfo()) {
            Com::printFLN(PSTR("M3013: current heat bed z-compensation matrix: "));
        }
        outputCompensationMatrix(format);
    } else {
        if (Printer::debugErrors()) {
            Com::printF(PSTR("M3013: the heat bed z-compensation matrix is not valid ( "), g_ZCompensationMatrix[0][0]);
            Com::printF(PSTR(" / "), EEPROM_FORMAT);
            Com::printFLN(PSTR(" )"));
        }

        showError((void*)ui_text_z_compensation, (void*)ui_text_invalid_matrix);
    }

    if (nOldHeatBed != 127) {
        // restore z-compensation matrix, in case we switched them
        switchActiveHeatBed(nOldHeatBed);
    }
}

void setScanXStart(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 5)
            nTemp = 5;
        if (nTemp > (Printer::axisLengthMM[X_AXIS] - 5))
            nTemp = Printer::axisLengthMM[X_AXIS] - 5;

        scanSettings.x.startSteps = (long)((float)nTemp * Printer::axisStepsPerMM[X_AXIS]);
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3020: new x start position: "), nTemp);
            Com::printF(PSTR(" [mm], "), (int)scanSettings.x.startSteps);
            Com::printFLN(PSTR(" [steps]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setScanYStart(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 5)
            nTemp = 5;
        if (nTemp > (Printer::axisLengthMM[Y_AXIS] - 5))
            nTemp = Printer::axisLengthMM[Y_AXIS] - 5;

        scanSettings.y.startSteps = (long)((float)nTemp * Printer::axisStepsPerMM[Y_AXIS]);
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3021: new y start position: "), nTemp);
            Com::printF(PSTR(" [mm], "), (int)scanSettings.y.startSteps);
            Com::printFLN(PSTR(" [steps]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setScanXStep(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < heatBedScanMinStepMm)
            nTemp = heatBedScanMinStepMm;
        if (nTemp > 100)
            nTemp = 100;

        scanSettings.x.strideSteps = (long)((float)nTemp * Printer::axisStepsPerMM[X_AXIS]);
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3022: new x step size: "), (int)nTemp);
            Com::printF(PSTR(" [mm], "), (int)scanSettings.x.strideSteps);
            Com::printFLN(PSTR(" [steps]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setScanYStep(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < heatBedScanMinStepMm)
            nTemp = heatBedScanMinStepMm;
        if (nTemp > 100)
            nTemp = 100;

        scanSettings.y.strideSteps = (long)((float)nTemp * Printer::axisStepsPerMM[Y_AXIS]);
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3023: new y step size: "), (int)nTemp);
            Com::printF(PSTR(" [mm], "), (int)scanSettings.y.strideSteps);
            Com::printFLN(PSTR(" [steps]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setScanXEnd(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 5)
            nTemp = 5;
        if (nTemp > (Printer::axisLengthMM[X_AXIS] - 5))
            nTemp = Printer::axisLengthMM[X_AXIS] - 5;

        scanSettings.x.endMarginSteps = (long)((float)nTemp * Printer::axisStepsPerMM[X_AXIS]);
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3024: new x end position: "), nTemp);
            Com::printF(PSTR(" [mm], "), (int)scanSettings.x.endMarginSteps);
            Com::printFLN(PSTR(" [steps]"));
        }

        scanSettings.x.maxPositionSteps = long(Printer::axisLengthMM[X_AXIS] * Printer::axisStepsPerMM[X_AXIS] - scanSettings.x.endMarginSteps);
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3024: new x max position: "), (int)scanSettings.x.maxPositionSteps);
            Com::printFLN(PSTR(" [steps]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setScanYEnd(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 5)
            nTemp = 5;
        if (nTemp > (Printer::axisLengthMM[Y_AXIS] - 5))
            nTemp = Printer::axisLengthMM[Y_AXIS] - 5;

        scanSettings.y.endMarginSteps = (long)((float)nTemp * Printer::axisStepsPerMM[Y_AXIS]);
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3025: new y end position: "), nTemp);
            Com::printF(PSTR(" [mm], "), (int)scanSettings.y.endMarginSteps);
            Com::printFLN(PSTR(" [steps]"));
        }

        scanSettings.y.maxPositionSteps = long(Printer::axisLengthMM[Y_AXIS] * Printer::axisStepsPerMM[Y_AXIS] - scanSettings.y.endMarginSteps);
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3025: new y max position: "), (int)scanSettings.y.maxPositionSteps);
            Com::printFLN(PSTR(" [steps]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

// Panel and homing diagnostics.
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

void accessPanel(GCode* pCommand) {
    if (pCommand->hasP()) {
        switch (pCommand->P) {
        case UI_ACTION_OK:       // 1001
        case UI_ACTION_NEXT:     // 1
        case UI_ACTION_PREVIOUS: // 2
        case UI_ACTION_BACK:     // 1000
        case UI_ACTION_RIGHT:    // 1129

        case UI_ACTION_RF_HEAT_BED_UP:      // 514
        case UI_ACTION_RF_HEAT_BED_DOWN:    // 515
        case UI_ACTION_RF_EXTRUDER_RETRACT: // 517
        case UI_ACTION_RF_EXTRUDER_OUTPUT:  // 516
        case UI_ACTION_RF_CONTINUE:         // 1519
        case UI_ACTION_RF_PAUSE:            // 1518
        {
            Com::printFLN(PSTR("RequestMenu:Press:"), pCommand->P);
            uid.executeAction(pCommand->P);
        } break;
        }
    } else {
        Com::printF(PSTR("RequestMenu:"), UI_ROWS);

        Com::printF(PSTR(","), MAX_COLS + 1);
        Com::printF(PSTR(":"));
        if (pCommand->hasL()) {
            //print line by line for human readability
            Com::println();
            Com::println();
        }
        // we print out UI_ROWS x MAX_COLS+1 field.
        for (uint8_t row = 0; row < UI_ROWS; row++) {
            const char* line = uid.cachedRow(row);
            bool nulled = false;
            for (uint8_t col = 0; col < MAX_COLS + 1; col++) {
                const char character = line[col];
                if (character == 0 || nulled) {
                    nulled = true;
                    Com::print(' '); //null as space
                    continue;
                }

                if (isprint(character)) {
                    Com::print(character);
                    continue;
                }

                switch (character) {
                case 2: {
                    Com::print('\'');
                    break;
                }
                case 5: {
                    Com::print('!');
                    break;
                }
                case (char)CHAR_SELECTOR: {
                    Com::print('>');
                    break;
                }
                case (char)CHAR_SELECTED: {
                    Com::print('*');
                    break;
                }
                default:
                    Com::print('_'); //others?
                }
            }
            if (pCommand->hasL()) {
                //print line by line for human readability
                Com::println();
            }
        }
        Com::println();
    }
}

// Probe settings.
void setFastApproachStep(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp > 50)
            nTemp = 50;
        if (nTemp < 1)
            nTemp = 1;

        scanSettings.fastApproachSteps = -nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3030: new fast step size for moving of the bed up: "), (int)scanSettings.fastApproachSteps);
            Com::printFLN(PSTR(" [steps]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setSlowApproachStep(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp > 50)
            nTemp = 50;
        if (nTemp < 1)
            nTemp = 1;

        scanSettings.slowApproachSteps = -nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3031: new slow step size for moving of the bed up: "), (int)scanSettings.slowApproachSteps);
            Com::printFLN(PSTR(" [steps]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setFastRetreatStep(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < Printer::axisStepsPerMM[Z_AXIS] / 20)
            nTemp = Printer::axisStepsPerMM[Z_AXIS] / 20;
        if (nTemp > Printer::axisStepsPerMM[Z_AXIS] * 5)
            nTemp = Printer::axisStepsPerMM[Z_AXIS] * 5;

        scanSettings.fastRetreatSteps = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3032: new fast step size for moving of the bed down: "), (int)scanSettings.fastRetreatSteps);
            Com::printFLN(PSTR(" [steps]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setSlowRetreatStep(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 1)
            nTemp = 1;
        if (nTemp > 50)
            nTemp = 50;

        scanSettings.slowRetreatSteps = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3033: new slow step size for moving of the bed down: "), (int)scanSettings.slowRetreatSteps);
            Com::printFLN(PSTR(" [steps]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setFastScanDelay(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 1)
            nTemp = 1;
        if (nTemp > 1000)
            nTemp = 1000;

        scanSettings.fastStepDelayMs = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3040: new delay between two fast movements: "), (int)scanSettings.fastStepDelayMs);
            Com::printFLN(PSTR(" [ms]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setSlowScanDelay(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 1)
            nTemp = 1;
        if (nTemp > 1000)
            nTemp = 1000;

        scanSettings.slowStepDelayMs = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3041: new delay between two slow movements: "), (int)scanSettings.slowStepDelayMs);
            Com::printFLN(PSTR(" [ms]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setScanIdleDelay(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 1)
            nTemp = 1;
        if (nTemp > 10000)
            nTemp = 10000;

        scanSettings.idleDelayMs = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3042: new idle delay: "), (int)scanSettings.idleDelayMs);
            Com::printFLN(PSTR(" [ms]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setContactPressureDelta(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 1)
            nTemp = 1;
        if (nTemp > 1000)
            nTemp = 1000;

        scanSettings.contactPressureDelta = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3050: new contact pressure delta: "), (int)scanSettings.contactPressureDelta);
            Com::printFLN(PSTR(" [digits]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setRetryPressureDelta(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 1)
            nTemp = 1;
        if (nTemp > 1000)
            nTemp = 1000;

        scanSettings.retryPressureDelta = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3051: new retry pressure delta: "), (int)scanSettings.retryPressureDelta);
            Com::printFLN(PSTR(" [digits]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setIdlePressureTolerance(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 1)
            nTemp = 1;
        if (nTemp > 1000)
            nTemp = 1000;

        scanSettings.idlePressureDelta = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3052: new idle pressure delta: "), (int)scanSettings.idlePressureDelta);
            Com::printFLN(PSTR(" [digits]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setPressureReadCount(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 1)
            nTemp = 1;
        if (nTemp > 100)
            nTemp = 100;

        scanSettings.pressureReadCount = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3053: new pressure reads per measurement: "), (int)scanSettings.pressureReadCount);
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setPressureReadDelay(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 1)
            nTemp = 1;
        if (nTemp > 1000)
            nTemp = 1000;

        scanSettings.pressureReadDelayMs = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3054: new delay between two pressure reads: "), (int)scanSettings.pressureReadDelayMs);
            Com::printFLN(PSTR(" [ms]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

void setPressureReadTolerance(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasS()) {
        // test and take over the specified value
        nTemp = pCommand->S;
        if (nTemp < 1)
            nTemp = 1;
        if (nTemp > 1000)
            nTemp = 1000;

        scanSettings.pressureTolerance = nTemp;
        if (Printer::debugInfo()) {
            Com::printF(PSTR("M3055: new scan pressure tolerance: "), (int)scanSettings.pressureTolerance);
            Com::printFLN(PSTR(" [digits]"));
        }
    } else {
        showInvalidSyntax(pCommand->M);
    }
}

// Printer operation and safety commands.
void reportDeviceVersion() {
    Com::printFLN(PSTR("Device Type: "), UI_PRINTER_NAME);
    Com::printFLN(PSTR("Firmware Version: "), UI_VERSION_STRING);
}

void requestPrintPause(GCode* pCommand) {
    const bool park = pCommand->hasS() && pCommand->S >= 2 && Printer::areAxisHomed();
    Printer::pause.queueRequest(park ? PrintPause::Destination::Park : PrintPause::Destination::InPlace);
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

void testWatchdog(GCode* pCommand) {
    if (pCommand->hasS()) {
        if (pCommand->S == 666) {
            if (g_bPingWatchdog) {
                HAL::stopWatchdog();
                Com::printFLN(PSTR("M3090: WARNING the watchdog is disabled now"));
            } else {
                HAL::startWatchdog();
                Com::printFLN(PSTR("M3090: the watchdog has been activated"));
            }
        }
    } else if (pCommand->hasP()) {
        HAL::delayMilliseconds(10000);
    } else if (pCommand->hasT()) {
        for (int iii = 0; iii < 1000; iii++)
            HAL::delayMicroseconds(10000);
    } else {
        if (Printer::debugInfo()) {
            Com::printFLN(PSTR("M3090: the watchdog is going to reset the firmware"));
        }
        HAL::delayMilliseconds(100);
        HAL::testWatchdog();
    }
}

void eraseMatrixEeprom() {
    if (isAnyScanRunning()) {
        Com::printFLN(PSTR("clearExtEEPROM(): scan in progress"));
        return;
    }
    clearExternalEEPROM();
}

void setPauseParkingOffsets(GCode* pCommand) {
    long nTemp;
    if (pCommand->hasNoXYZ() && !pCommand->hasE()) {
        showInvalidSyntax(pCommand->M);
    } else {
        if (pCommand->hasX()) {
            // test and take over the specified value
            nTemp = pCommand->X * Printer::axisStepsPerMM[X_AXIS];
            if (nTemp < -Printer::maxSoftEndstopSteps[X_AXIS])
                nTemp = -Printer::maxSoftEndstopSteps[X_AXIS];
            if (nTemp > Printer::maxSoftEndstopSteps[X_AXIS])
                nTemp = Printer::maxSoftEndstopSteps[X_AXIS];

            Printer::pause.setParkSteps(X_AXIS, nTemp);
            Com::printF(PSTR("M3105: new x pause offset: "), nTemp * Printer::axisMMPerSteps[X_AXIS]);
            Com::printFLN(PSTR(" [mm]"));
        }
        if (pCommand->hasY()) {
            // test and take over the specified value
            nTemp = pCommand->Y * Printer::axisStepsPerMM[Y_AXIS];
            if (nTemp < -Printer::maxSoftEndstopSteps[Y_AXIS])
                nTemp = -Printer::maxSoftEndstopSteps[Y_AXIS];
            if (nTemp > Printer::maxSoftEndstopSteps[Y_AXIS])
                nTemp = Printer::maxSoftEndstopSteps[Y_AXIS];

            Printer::pause.setParkSteps(Y_AXIS, nTemp);
            Com::printF(PSTR("M3105: new y pause offset: "), nTemp * Printer::axisMMPerSteps[Y_AXIS]);
            Com::printFLN(PSTR(" [mm]"));
        }
        if (pCommand->hasZ()) {
            // test and take over the specified value
            nTemp = pCommand->Z * Printer::axisStepsPerMM[Z_AXIS];
            if (nTemp < 0)
                nTemp = 0; //no pause within printed part.
            if (nTemp > Printer::zMotion.maxSteps())
                nTemp = Printer::zMotion.maxSteps();

            Printer::pause.setParkSteps(Z_AXIS, nTemp);
            Com::printF(PSTR("M3105: new z pause offset: "), nTemp * Printer::axisMMPerSteps[Z_AXIS]);
            Com::printFLN(PSTR(" [mm]"));
        }
        if (pCommand->hasE()) {
            // test and take over the specified value
            nTemp = pCommand->E;
            if (nTemp < 0)
                nTemp = 0;
            if (nTemp > 5)
                nTemp = 5; //Zahl immer positiv, sie wird abgezogen = Retract!

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

void setPanelStatusOverride(GCode* pCommand) {
    if (pCommand->hasString()) {
        if (!uid.locked) {
            // ensure that the current text won't be overwritten
            uid.lock();
            if (Printer::debugInfo()) {
                Com::printFLN(PSTR("M3117: lock"));
            }
        }
        uid.setStatus(pCommand->text, false, true);
        uid.refreshPage();
    } else {
        if (uid.locked) {
            // allow to overwrite the current string again
            // Put the unlock into queue to avoid racetime conditions with prior commands
            queueTask(TASK_UNLOCK_DISPLAY_MSG);
            Com::printFLN(PSTR("M3117: queued unlock"));
        }
    }
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

#endif // (MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2) && CASE_FAN_PIN > -1
void runRfDiagnostic(GCode* pCommand) {
    if (pCommand->hasP()) {
        switch (pCommand->P) {
        case 1: {
            Commands::checkFreeMemory();
            Commands::writeLowestFreeRAM();
            Com::printFLN(PSTR("lowest free RAM: "), Commands::lowestRAMValue);

            Com::printFLN(PSTR("z-compensation matrix x: "), COMPENSATION_MATRIX_MAX_X);
            Com::printFLN(PSTR("z-compensation matrix y: "), COMPENSATION_MATRIX_MAX_Y);
            break;
        }
        case 2: {
            Com::printFLN(PSTR("Homing feedrates:"));
            Com::printF(PSTR("x = "), Printer::homingFeedrate[X_AXIS]);
            Com::printF(PSTR(", y = "), Printer::homingFeedrate[Y_AXIS]);
            Com::printFLN(PSTR(", z = "), Printer::homingFeedrate[Z_AXIS]);
            break;
        }
        case 4: {
            // simulate blocking of all axes
            Com::printFLN(PSTR("M3200: block all"));

            doEmergencyStop(STOP_BECAUSE_OF_Z_BLOCK);
            break;
        }
        case 5: {
            // simulate a temp sensor error
            Com::printFLN(PSTR("M3200: simulating defect sensor"));
            //ext 0 soll defekt aussehen
            TemperatureController* act0 = tempController[0];
            act0->setSensorDefect(true);
            //ext 1/bett soll decoupled aussehen
            TemperatureController* act1 = tempController[1];
            act1->setSensorDecoupled(true);
            //generelle defekt-einstufung
            Printer::setSomeTempsensorDefect(true);
            //fehlermeldung ausgeben
            reportTempsensorAndHeaterErrors();
            break;
        }


        case 9: {
            Com::printFLN(PSTR("debug level: "), Printer::debugLevel);
            break;
        }

        case 13: {
            Com::printFLN(PSTR("LCD re-initialization"));
            uid.initializeLCD();
        }
        }
    }
}

#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
// RF2000 family outputs.
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

#endif // MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
#ifdef RGB_LIGHT_R_PIN
struct RgbComponent {
    uint8_t& level;
    unsigned int address;
};

struct RgbProfile {
    RgbComponent components[3];
    PGM_P messagePrefix;
    bool active;
};

void setRgbComponent(GCode& command, const RgbProfile& profile) {
    if (!command.hasP() || !command.hasS() || command.P < 1 || command.P > 3)
        return;

    const uint8_t channel = command.P - 1;
    const char channelName = channel == 0 ? 'R' : channel == 1 ? 'G' : 'B';
    if (command.S < 0 || command.S > 255) {
        Com::printF(profile.messagePrefix);
        Com::print(channelName);
        Com::printFLN(PSTR(" out of range "));
        return;
    }

    const auto& component = profile.components[channel];
    component.level = command.S;
    HAL::eprSetByte(component.address, component.level);
    EEPROM::updateChecksum();
    if (profile.active)
        setRGBTargetColors(profile.components[0].level, profile.components[1].level, profile.components[2].level);
    Com::printF(profile.messagePrefix);
    Com::print(channelName);
    Com::printFLN(PSTR(" = "), component.level);
}

// RGB lighting.
void setHeatingLight(GCode* pCommand) {
    const bool active = Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_HEATING;
    setRgbComponent(*pCommand, {{{g_uRGBHeatingR, EPR_RF_RGB_HEATING_R},
                                 {g_uRGBHeatingG, EPR_RF_RGB_HEATING_G},
                                 {g_uRGBHeatingB, EPR_RF_RGB_HEATING_B}},
                                PSTR("RGB_HEATING_"), active});
}

void setPrintingLight(GCode* pCommand) {
    const bool active = Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_PRINTING;
    setRgbComponent(*pCommand, {{{g_uRGBPrintingR, EPR_RF_RGB_PRINTING_R},
                                 {g_uRGBPrintingG, EPR_RF_RGB_PRINTING_G},
                                 {g_uRGBPrintingB, EPR_RF_RGB_PRINTING_B}},
                                PSTR("RGB_PRINTING_"), active});
}

void setCoolingLight(GCode* pCommand) {
    const bool active = Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_COOLING;
    setRgbComponent(*pCommand, {{{g_uRGBCoolingR, EPR_RF_RGB_COOLING_R},
                                 {g_uRGBCoolingG, EPR_RF_RGB_COOLING_G},
                                 {g_uRGBCoolingB, EPR_RF_RGB_COOLING_B}},
                                PSTR("RGB_COOLING_"), active});
}

void setIdleLight(GCode* pCommand) {
    const bool active = Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_IDLE;
    setRgbComponent(*pCommand, {{{g_uRGBIdleR, EPR_RF_RGB_IDLE_R},
                                 {g_uRGBIdleG, EPR_RF_RGB_IDLE_G},
                                 {g_uRGBIdleB, EPR_RF_RGB_IDLE_B}},
                                PSTR("RGB_IDLE_"), active});
}

void setManualLight(GCode* pCommand) {
    const bool active = Printer::RGBLightMode == RGB_MODE_MANUAL;
    setRgbComponent(*pCommand, {{{g_uRGBManualR, EPR_RF_RGB_MANUAL_R},
                                 {g_uRGBManualG, EPR_RF_RGB_MANUAL_G},
                                 {g_uRGBManualB, EPR_RF_RGB_MANUAL_B}},
                                PSTR("RGB_MANUAL_"), active});
}

void setRgbLightMode(GCode* pCommand) {
    if (pCommand->hasP()) {
        switch (pCommand->P) {
        case RGB_MODE_OFF: {
            Printer::RGBLightStatus = RGB_STATUS_NOT_AUTOMATIC;
            Printer::RGBLightMode = RGB_MODE_OFF;
            Printer::RGBLightModeForceWhite = 0;

            setRGBTargetColors(0, 0, 0);

            HAL::eprSetByte(EPR_RF_RGB_LIGHT_MODE, Printer::RGBLightMode);
            EEPROM::updateChecksum();
            break;
        }
        case RGB_MODE_WHITE: {
            Printer::RGBLightStatus = RGB_STATUS_NOT_AUTOMATIC;
            Printer::RGBLightMode = RGB_MODE_WHITE;
            Printer::RGBLightModeForceWhite = 0;

            setRGBTargetColors(255, 255, 255);

            HAL::eprSetByte(EPR_RF_RGB_LIGHT_MODE, Printer::RGBLightMode);
            EEPROM::updateChecksum();
            break;
        }
        case RGB_MODE_AUTOMATIC: {
            Printer::RGBLightStatus = RGB_STATUS_AUTOMATIC;
            Printer::RGBLightMode = RGB_MODE_AUTOMATIC;
            Printer::RGBLightModeForceWhite = 0;

            HAL::eprSetByte(EPR_RF_RGB_LIGHT_MODE, Printer::RGBLightMode);
            EEPROM::updateChecksum();
            break;
        }
        case RGB_MODE_MANUAL: {
            Printer::RGBLightStatus = RGB_STATUS_NOT_AUTOMATIC;
            Printer::RGBLightMode = RGB_MODE_MANUAL;
            Printer::RGBLightModeForceWhite = 0;

            setRGBTargetColors(g_uRGBManualR, g_uRGBManualG, g_uRGBManualB);

            HAL::eprSetByte(EPR_RF_RGB_LIGHT_MODE, Printer::RGBLightMode);
            EEPROM::updateChecksum();
            break;
        }
        }
    }
}

#endif // RGB_LIGHT_R_PIN
// Z-offset scan, matrix correction and filament control.
void configureZOffsetScan(GCode* pCommand) {
    if (heatBedScan.active()) {
        Com::printFLN(PSTR("M3900/M3901: bed scan in progress"));
        return;
    }
    bool err3900r = false;
    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
        // we load the z compensation matrix before its first usage because this can take some time
        Com::printFLN(PSTR("M3900/M3901: INFO Die Z-Matrix wurde aus dem EEPROM gelesen."));
        prepareZCompensation();
    }

    if (g_ZCompensationMatrix[0][0] == EEPROM_FORMAT) {
        if (!pCommand->hasR()) { //Normaler ZOS, kein Auto-Matrix-Leveling.
            auto settings = zOffsetScan.settings();
            if (pCommand->hasX()) {
                // test and take over the specified value
                unsigned char nTempUC = (unsigned char)pCommand->X;

                //Wessix idee: zufalls-Scanpunkt wegen DDP-Platten-Schonung.
                if (nTempUC == 0)
                    nTempUC = (unsigned char)random((heatBedScanXStartMm == 0) ? 1 : 2, g_uZMatrixMax[X_AXIS]); // ergibt min bis max-1

                //wenn die Startposition nicht 0 ist, wird eine Dummy-Matrix-Linie ergänzt. Mit der sollten wir nicht arbeiten.
                if (nTempUC < 1 + ((heatBedScanXStartMm == 0) ? 0 : 1))
                    nTempUC = 1 + ((heatBedScanXStartMm == 0) ? 0 : 1);
                if (nTempUC > g_uZMatrixMax[X_AXIS] - 1)
                    nTempUC = g_uZMatrixMax[X_AXIS] - 1; //2..n-1

                settings.point.x = nTempUC;

                unsigned char oldval = HAL::eprGetByte(EPR_RF_MOD_ZOS_SCAN_POINT_X);
                if (oldval != settings.point.x) {
                    HAL::eprSetByte(EPR_RF_MOD_ZOS_SCAN_POINT_X, settings.point.x);
                    EEPROM::updateChecksum(); //deshalb die prüfung
                }

                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3900/M3901: CHANGED X ZOS Testposition: "), nTempUC);
                    if (heatBedScanXStartMm == 0) {
                        Com::printF(PSTR(" [Z-Matrix index] X={1.."), g_uZMatrixMax[X_AXIS] - 1);
                    } else {
                        Com::printF(PSTR(" [Z-Matrix index] X={2.."), g_uZMatrixMax[X_AXIS] - 1);
                    }
                    Com::printFLN(PSTR("}"));
                }
            }

            if (pCommand->hasY()) {
                // test and take over the specified value
                unsigned char nTempUC = (unsigned char)pCommand->Y;

                //Wessix idee: zufalls-Scanpunkt wegen DDP-Platten-Schonung.
                if (nTempUC == 0)
                    nTempUC = (unsigned char)random((heatBedScanYStartMm == 0) ? 1 : 2, g_uZMatrixMax[Y_AXIS]); // ergibt min bis max-1

                if (nTempUC < 1 + ((heatBedScanYStartMm == 0) ? 0 : 1))
                    nTempUC = 1 + ((heatBedScanYStartMm == 0) ? 0 : 1);
                if (nTempUC > g_uZMatrixMax[Y_AXIS] - 1)
                    nTempUC = g_uZMatrixMax[Y_AXIS] - 1; //2..n-1

                settings.point.y = nTempUC;

                unsigned char oldval = HAL::eprGetByte(EPR_RF_MOD_ZOS_SCAN_POINT_Y);
                if (oldval != settings.point.y) {
                    HAL::eprSetByte(EPR_RF_MOD_ZOS_SCAN_POINT_Y, settings.point.y);
                    EEPROM::updateChecksum(); //deshalb die prüfung
                }

                if (Printer::debugInfo()) {
                    Com::printF(PSTR("M3900/M3901: CHANGED Y ZOS Testposition: "), nTempUC);
                    if (heatBedScanYStartMm == 0) {
                        Com::printF(PSTR(" [Z-Matrix index] Y={1.."), g_uZMatrixMax[Y_AXIS] - 1);
                    } else {
                        Com::printF(PSTR(" [Z-Matrix index] Y={2.."), g_uZMatrixMax[Y_AXIS] - 1);
                    }
                    Com::printFLN(PSTR("}"));
                }
            }

            if (pCommand->hasS()) {
                // M3900 S set learning rate to limit changes caused of z-Offset Scan. This might proof handy for multiple positions scans.
                if (pCommand->S >= 0 && pCommand->S <= 100) {
                    settings.learningRate = (float)pCommand->S * 0.01f;
                    Com::printFLN(PSTR("M3900/M3901: CHANGED [S] ZOS learning rate : "), settings.learningRate);
                } else {
                    Com::printFLN(PSTR("M3900/M3901: ERROR [S] ZOS learning rate ignored, out of range {0...100}"));
                    err3900r = true;
                }
            }

            if (pCommand->hasP()) {
                // M3900 P set distance weight: This can be used as something like the auto-bed-leveling (if used in all corners) but technically affects the Z_Matrix
                if (pCommand->P >= 0 && pCommand->P <= 100) {
                    settings.learningGradient = (float)pCommand->P * 0.01f;
                    Com::printFLN(PSTR("M3900/M3901: CHANGED [P] ZOS learning linear distance weight : "), settings.learningGradient);
                } else {
                    Com::printFLN(PSTR("M3900/M3901: ERROR [P] ZOS learning DistanceWeight, out of range {0...100}"));
                    err3900r = true;
                }
            }

            if (pCommand->hasX() || pCommand->hasY() || pCommand->hasS() || pCommand->hasP())
                zOffsetScan.configure(settings);

            // Anzeige der aktuellen Settings
            Com::printFLN(PSTR("M3900/M3901: ### AKTIVE ZOS SETTINGS ###"));
            Com::printF(PSTR("M3900/M3901: Testposition [X]: "), settings.point.x);
            Com::printF(PSTR(" Testposition [Y]: "), settings.point.y);
            Com::printFLN(PSTR(" [Z-Matrix index]"));
            Com::printFLN(PSTR("M3900/M3901: [S] ZOS learning rate is : "), settings.learningRate);
            if (settings.learningRate == 1.0f) {
                Com::printFLN(PSTR("M3900/M3901: INFO [S] ZOS::overwrite mode (1.00)"));
            } else {
                Com::printFLN(PSTR("M3900/M3901: INFO [S] ZOS::additiv / learning mode (0.00 - 0.99)"));
            }
            Com::printFLN(PSTR("M3900/M3901: [P] ZOS learning linear distance weight is : "), settings.learningGradient);
            if (settings.learningGradient == 0.0f) {
                Com::printFLN(PSTR("M3900/M3901: INFO [P] ZOS::reiner Offset-Scan"));
            } else if (settings.learningGradient == 1.0f) {
                Com::printFLN(PSTR("M3900/M3901: INFO [P] ZOS::rein linear abstandsgewichteter Scan"));
            } else {
                Com::printFLN(PSTR("M3900/M3901: INFO [P] ZOS::Das Ergebnis-Offset setzt sich zusammen aus Abstandsgewichtung und Offset"));
            }

            if (settings.learningGradient > 0.0f) {
                Com::printFLN(PSTR("M3900/M3901: INFO [P] Set 0 => 0.0 for Offset only, set 100 => 1.0 for distance weight only"));
                Com::printFLN(PSTR("M3900/M3901: INFO [P] Combine linear distance weight with low learning rate and multiple checks at corners (for example) against bed warping!"));
            }
            if (settings.learningRate == 1.0f) {
                Com::printFLN(PSTR("M3900/M3901: FORMEL Z-Matrix = EEPROM-Matrix + g_ZOSlearningRate*(g_ZOSlearningGradient*weight(x,y)*Offset + (1.0-g_ZOSlearningGradient)*Offset)"));
            } else {
                Com::printFLN(PSTR("M3900/M3901: FORMEL Z-Matrix = Z-Matrix + g_ZOSlearningRate*(g_ZOSlearningGradient*weight(x,y)*Offset + (1.0-g_ZOSlearningGradient)*Offset)"));
            }
        }
        if (pCommand->M == 3901) {
            //M3901
            //M3900 wird nicht gestartet - nur preconfig, kein mhier-Scan!
            if (pCommand->hasR())
                Com::printFLN(PSTR("M3901: Use M3900 for Auto-Matrix-Leveling!"));
        } else {
            if (err3900r) {
                Com::printFLN(PSTR("M3900/M3901: ERROR in Config ### ZOS SKIPPED ###"));
            } else {
                // M3900:
                // mit R im GCode macht der Scan ein Auto-Matrix-Leveling, anstatt die anderen Schalter zu bedienen.
                startZOScan(pCommand->hasR());
                // Warte bis Z-Offset-Scan beendet wurde.
                while (zOffsetScan.active()) {
                    Commands::checkForPeriodicalActions(Calibrating);
                }
            }
        }
    } else {
        Com::printFLN(PSTR("M3900/M3901: ERROR Matrix Initialisation Error!"));
        Com::printFLN(PSTR("M3900/M3901: INFO Die Z-Matrix konnte nicht aus dem EEPROM gelesen werden."));
        Com::printFLN(PSTR("M3900/M3901: INFO Vermutlich noch nie einen Heat-Bed-Scan gemacht."));
    }
}

void adjustBedMatrix(GCode* pCommand) {
    if (isAnyScanRunning()) {
        Com::printFLN(PSTR("M3902: scan in progress"));
        return;
    }
    //NMM Funktion 1 - R=Repair
    if (pCommand->hasR()) {
        //search for a hole within the heat beds z-Matrix
        fixKeramikLochInMatrix();
    }

    if (pCommand->hasE()) {
        //completly wipe the matrix-data to zero->flatten the matrix to nothing.
        setMatrixNull();
    }
    //NMM Funktion 2 - Z=Offset manuell nachstellen
    if (pCommand->hasZ()) {
        if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
            // we load the z compensation matrix before its first usage because this can take some time
            Com::printFLN(PSTR("M3901: INFO Die Z-Matrix wurde aus dem EEPROM gelesen."));
            prepareZCompensation();
        }
        if (g_ZCompensationMatrix[0][0] == EEPROM_FORMAT) {
            //search for a hole within the heat beds z-Matrix
            float hochrunter = (float)pCommand->Z;
            if (hochrunter > 0.2f)
                hochrunter = 0.2f;
            if (hochrunter < -0.2f)
                hochrunter = -0.2f;

            Com::printSharpLine();

            if (hochrunter < 0.0f) {
                Com::printFLN(PSTR("M3902: Duese-Bett-Abstand wird kleiner gemacht. "), hochrunter);
            } else if (hochrunter > 0.0f) {
                Com::printFLN(PSTR("M3902: Duese-Bett-Abstand wird groesser gemacht. "), hochrunter);
            } else {
                Com::printFLN(PSTR("M3902: Duese-Bett-Abstand bleibt gleich. Das Offset wird in die Matrix verrechnet und genullt."), hochrunter);
            }
            Com::printFLN(PSTR("M3902: -Z heisst Bett hoch/weniger Abstand, +Z heisst Bett runter/mehr Abstand. "));
            Com::printFLN(PSTR("M3902: Bei Z=0 ändert sich nichts, doch es wird das aktuelle Offset in die Matrix verrechnet. "));

            Com::printSharpLine();

            // determine the minimal distance between extruder and heat bed
            determineCompensationOffsetZ(); //-> schreibt kleinsten abstand in g_offsetZCompensationSteps, sollte schon drin sein, aber man weiß nie.
            Com::printFLN(PSTR("M3902: Alt::Min. Bett-Hotend Abstand [Steps] = "), -1 * (int)g_offsetZCompensationSteps);

            long hochrunterSteps = long(hochrunter * Printer::axisStepsPerMM[Z_AXIS]); //axissteps ist auch float

            Com::printFLN(PSTR("M3902: Veraenderung Z [mm] = "), hochrunter);
            Com::printFLN(PSTR("M3902: Veraenderung Z [Steps] = "), hochrunterSteps);

            if (hochrunter == 0.00f) {
                //wenn Z=0.0 soll alles so bleibenk aber das Offset ins Matrix-Offset reingerechnet und genullt werden.
                //das Offset muss negativ eingehen, denn wenn die Matrix "weniger tief" ist, bleibt die Düse weiter weg vom Bett.
                hochrunterSteps = Printer::zOffset.manualSteps(Printer::axisStepsPerMM[Z_AXIS]);
                Com::printFLN(PSTR("M3902: Veraenderung = zOffset --> zMatrix; Offset = 0;"));
            }

            //das ist ein negativer wert! Je mehr Abstand, desto negativer. Positiv verboten.
            if (g_offsetZCompensationSteps + hochrunterSteps > (long(Printer::axisStepsPerMM[Z_AXIS] * scanSettings.startLiftMm) - -1 * scanSettings.fastApproachSteps)) {
                Com::printF(PSTR("M3902: Fehler::Z-Matrix wuerde positiv werden. Das waere eine Kollision um "), (int)(g_offsetZCompensationSteps + hochrunterSteps));
                Com::printFLN(PSTR(" [Steps] bzw. "), Printer::axisMMPerSteps[Z_AXIS] * (float)(g_offsetZCompensationSteps + hochrunterSteps), 2);
                Com::printFLN(PSTR(" [mm]"));
            } else {
                Com::printFLN(PSTR("M3902: Neu::Min. Bett-Extruder Restabstand = "), -1 * (int)(g_offsetZCompensationSteps));

                short x;
                short y;
                short deltaZ = (short)hochrunterSteps;
                bool overflow = false;
                bool overnull = false;
                bool overH = false;

                for (x = 1; x <= g_uZMatrixMax[X_AXIS]; x++) {
                    for (y = 1; y <= g_uZMatrixMax[Y_AXIS]; y++) {
                        if ((long)g_ZCompensationMatrix[x][y] + (long)deltaZ >= 32767) {
                            overflow = true;
                        } else {
                            g_ZCompensationMatrix[x][y] += deltaZ;
                            if (g_ZCompensationMatrix[x][y] > 0)
                                overnull = true; //overflow oder kollision, kann einfach nicht sein und abfrage ist kurz.
                            if (g_ZCompensationMatrix[x][y] > (long(Printer::axisStepsPerMM[Z_AXIS] * scanSettings.startLiftMm) - -1 * scanSettings.fastApproachSteps))
                                overH = true;
                        }
                    }
                }
                if (overnull) {
                    Com::printFLN(PSTR("M3901: WARNING::positive Matrix::Please fix Z-Screw"));
                }
                if (overH) {
                    Com::printFLN(PSTR("M3901: ERROR::sehr positive Matrix::ReLoading zMatrix from EEPROM to RAM"));
                    if (loadCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed))) {
                        // Error: there is no valid compensation matrix available
                        initCompensationMatrix();
                    }
                } else if (overflow) {
                    Com::printFLN(PSTR("M3901: ERROR::Overflow in Matrix::ReLoading zMatrix from EEPROM to RAM"));
                    if (loadCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed))) {
                        // Error: there is no valid compensation matrix available
                        initCompensationMatrix();
                    }
                } else {
                    if (hochrunter == 0.00f) {
                        Printer::zOffset.setManualMicrometers(0, Printer::axisStepsPerMM[Z_AXIS]);
                        EEPROM::storeManualZOffset();
                    }
                    g_ZMatrixChangedInRam = 1;
                }
            }

            // determine the minimal distance between extruder and heat bed
            determineCompensationOffsetZ();
            Com::printFLN(PSTR("M3902: Neu::Min. Bett-Extruder Restabstand [Steps] = "), -1 * (int)g_offsetZCompensationSteps);
            Com::printFLN(PSTR("M3902: Neu::Min. Bett-Extruder Restabstand [mm] = "), -1 * Printer::axisMMPerSteps[Z_AXIS] * (float)g_offsetZCompensationSteps, 2);
        } else {
            Com::printFLN(PSTR("M3902: ERROR::Matrix Initialisation Error!"));
            Com::printFLN(PSTR("M3902: INFO Die Z-Matrix konnte nicht aus dem EEPROM gelesen werden."));
            Com::printFLN(PSTR("M3902: INFO Vermutlich nie einen Heat-Bed-Scan gemacht."));
        }
    }

    //NMM Funktion 3 - S=Save,Sichern der Matrix an spezielle EEPROM-Position
    if (pCommand->hasS()) {
        if (pCommand->S >= 1 && pCommand->S <= EEPROM_MAX_HEAT_BED_SECTORS) {
            // save the determined values to the EEPROM @ savepoint "pCommand->S" Standard: 1..9
            unsigned int savepoint = (unsigned int)pCommand->S;
            if (saveCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * savepoint)))
                Com::printFLN(PSTR("M3902: Save the Matrix::OK"));
            else
                Com::printFLN(PSTR("M3902: Save the Matrix::ERROR"));
        } else {
            Com::printFLN(PSTR("M3902: Save the Matrix::ERROR::invalid savepoint, invalid active heatbedmatrix"));
        }
    }
}

void setSenseOffsetParameters(GCode* pCommand) {
    auto& controller = Printer::senseOffsetControl;
    bool error = false;
    //Statusänderung P10000 ([digits])
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
                    EEPROM::updateChecksum(); //deshalb die prüfung
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
                EEPROM::updateChecksum(); //deshalb die prüfung
            }
        } else {
            Com::printFLN(PSTR("M3909: ERROR::>0.3mm=S300 - This function is ment to compensate minimal amounts of too close distance."));
            Com::printFLN(PSTR("M3909: INFO::If you have to auto-compensate your offset to high, clean your nozzle, rise your temp, lower your speed."));
            Com::printFLN(PSTR("M3909: INFO::This function should lower the chance of an accidential emergency block on the first layer. It cannot help you to avoid calibration!"));
            error = true;
        }
    }

    //Statusausgabe per M3909
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
        queueTask(TASK_ENABLE_SENSE_OFFSET);
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
            min = 500; //weniger ist in jedem fall sinnfrei, selbst mit ninjaflex und digit homing
        if (max < min)
            max = min; //geht: flowsprung.
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

#endif // X35_ADC_CHANNEL

} // namespace

void processSpecialGCode(GCode* pCommand) {
    if (!pCommand->hasM())
        return;

    switch (pCommand->M) {
    case 3000: disableBedCompensation(); return;
    case 3001: enableBedCompensation(); return;
    case 3005: configureRfDebug(pCommand); return;
    case 3006: setManualZOffset(pCommand); return;
    case 3009: selectBedMatrix(pCommand); return;
    case 3010: requestHeatBedScan(pCommand); return;
    case 3011: clearBedMatrix(pCommand); return;
    case 3012: restoreDefaultScanParameters(); return;
    case 3013: reportBedMatrix(pCommand); return;
    case 3020: setScanXStart(pCommand); return;
    case 3021: setScanYStart(pCommand); return;
    case 3022: setScanXStep(pCommand); return;
    case 3023: setScanYStep(pCommand); return;
    case 3024: setScanXEnd(pCommand); return;
    case 3025: setScanYEnd(pCommand); return;
    case 3028: testHomingRepeatability(pCommand); return;
    case 3029: accessPanel(pCommand); return;
    case 3030: setFastApproachStep(pCommand); return;
    case 3031: setSlowApproachStep(pCommand); return;
    case 3032: setFastRetreatStep(pCommand); return;
    case 3033: setSlowRetreatStep(pCommand); return;
    case 3040: setFastScanDelay(pCommand); return;
    case 3041: setSlowScanDelay(pCommand); return;
    case 3042: setScanIdleDelay(pCommand); return;
    case 3050: setContactPressureDelta(pCommand); return;
    case 3051: setRetryPressureDelta(pCommand); return;
    case 3052: setIdlePressureTolerance(pCommand); return;
    case 3053: setPressureReadCount(pCommand); return;
    case 3054: setPressureReadDelay(pCommand); return;
    case 3055: setPressureReadTolerance(pCommand); return;
    case 3060: reportDeviceVersion(); return;
    case 3070: requestPrintPause(pCommand); return;
    case 3075: setEmergencyPauseDigits(pCommand); return;
    case 3076: setEmergencyStopDigits(pCommand); return;
    case 3079: outputObject(); return;
    case 3090: testWatchdog(pCommand); return;
    case 3091: eraseMatrixEeprom(); return;
    case 3105: setPauseParkingOffsets(pCommand); return;
    case 3115: setXYOrigin(); return;
    case 3117: setPanelStatusOverride(pCommand); return;
#if (MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2) && CASE_FAN_PIN > -1
    case 3120: turnOnCaseFan(); return;
    case 3121: scheduleCaseFanOff(pCommand); return;
#endif // (MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2) && CASE_FAN_PIN > -1
    case 3200: runRfDiagnostic(pCommand); return;
#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
    case 3300: setAccessoryFet(pCommand); return;
    case 3301: setMainsOutput(pCommand); return;
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
#ifdef RGB_LIGHT_R_PIN
    case 3303: setHeatingLight(pCommand); return;
    case 3304: setPrintingLight(pCommand); return;
    case 3305: setCoolingLight(pCommand); return;
    case 3306: setIdleLight(pCommand); return;
    case 3307: setManualLight(pCommand); return;
    case 3308: setRgbLightMode(pCommand); return;
#endif // RGB_LIGHT_R_PIN
    case 3901:
    case 3900: configureZOffsetScan(pCommand); return;
    case 3902: adjustBedMatrix(pCommand); return;
    case 3909: setSenseOffsetParameters(pCommand); return;
    case 3911: setDigitFlowParameters(pCommand); return;
    case 3912: Printer::startLine.run(*pCommand); return;
    case 3913: loadFilament(pCommand); return;
    case 3914: unloadFilament(pCommand); return;
#ifdef X35_ADC_CHANNEL
    case 3941: readOptionalTemperature(); return;
#endif // X35_ADC_CHANNEL
    }
}


void queueTask(char task) {
    if (PrintLine::waitForXFreeLines(1))
        PrintLine::queueTask(task);
} // queueTask

bool isAnyScanRunning() {
    return heatBedScan.active() || zOffsetScan.active();
}

void moveXAction(int8_t direction) {
    bool moveKosys = Printer::manualMotion.coordinateSystem();

    if (Printer::pause.isPaused()) {
        moveKosys = KOSYS_DIRECTOFFSET;
    } else if (PrintLine::hasLines() || isAnyScanRunning()) {
        // this operation is not allowed while a printing or scanning is in progress
        return;
    }

    if (direction < 0 && Printer::isXMinEndstopHit()) {
        // we shall move to the left but the x-min-endstop is hit already, so we do nothing
        showInformation((void*)ui_text_x_axis, (void*)ui_text_min_reached);
        return;
    }

    if (direction > 0 && Printer::isXMaxEndstopHit()) {
        // we shall move to the right but the end of the x-axis has been reached already, so we do nothing
        showInformation((void*)ui_text_x_axis, (void*)ui_text_max_reached);
        return;
    }

    // show that we are active
    Printer::inactivity.recordActivity(HAL::timeInMilliseconds());

    switch (Printer::manualMotion.mode(X_AXIS)) {
    case MOVE_MODE_SINGLE_STEPS: {
        int32_t steps = int32_t(Printer::manualMotion.steps(X_AXIS)) * direction;

        if (direction < 0 && Printer::xyMotion.motorSteps(X_AXIS) + steps < 0 && Printer::isAxisHomed(X_AXIS)) {
            // If we are very close to the homed known Y-Endstop go to 0-Coordinate
            steps = -Printer::xyMotion.motorSteps(X_AXIS);
        }

        if (moveKosys == KOSYS_GCODE) {
            float feedrate = STANDARD_POSITION_FEEDRATE_XY;
            if (Printer::manualMotion.feedrateSource() == FEEDRATE_GCODE) {
                feedrate = Printer::feedrate;
            }
            Printer::queueRelativeStepsCoordinates(steps, 0, 0, 0, feedrate, true);
            Commands::printCurrentPosition();
        } else /*if (Printer::manualMotion.coordinateSystem() == KOSYS_DIRECTOFFSET) */
        {
            Printer::offsetRelativeStepsCoordinates(steps, 0, 0, 0, TASK_MOVE_POSITION_MANUAL);
        }

        break;
    }
    case MOVE_MODE_1_MM:
    case MOVE_MODE_10_MM:
    case MOVE_MODE_50_MM: {
        uint8_t distanceMM = 1;
        if (Printer::manualMotion.mode(X_AXIS) == MOVE_MODE_10_MM)
            distanceMM = 10;
        if (Printer::manualMotion.mode(X_AXIS) == MOVE_MODE_50_MM)
            distanceMM = 50;

        int32_t plannedDestination = Printer::getPlannedDirectAxisSteps(X_AXIS) + (int32_t)(direction * distanceMM * Printer::axisStepsPerMM[X_AXIS]);
        if (plannedDestination > Printer::maxSoftEndstopSteps[X_AXIS]) {
            showInformation((void*)ui_text_x_axis, (void*)ui_text_max_reached);
            return;
        }

        if (moveKosys == KOSYS_GCODE) {
            float feedrate = STANDARD_POSITION_FEEDRATE_XY;
            if (Printer::manualMotion.feedrateSource() == FEEDRATE_GCODE) {
                feedrate = Printer::feedrate;
            }
            Printer::queueRelativeMMCoordinates(distanceMM * direction, 0, 0, 0, feedrate, true);
            Commands::printCurrentPosition();
        } else /*if (Printer::manualMotion.coordinateSystem() == KOSYS_DIRECTOFFSET) */
        {
            Printer::offsetRelativeStepsCoordinates(distanceMM * Printer::axisStepsPerMM[Y_AXIS] * direction, 0, 0, 0, TASK_MOVE_POSITION_MANUAL);
        }

        break;
    }
    case MOVE_MODE_SINGLE_MOVE: {
        if (PrintLine::directMoveActive()) {
            // we are moving already, there is nothing more to do
            return;
        }

        int32_t steps;
        //errate homed möglichst den erzwungenen stop-punkt wegen dem bonus der decelleration
        if (direction < 0)
            steps = (Printer::isAxisHomed(X_AXIS) ? -Printer::xyMotion.motorSteps(X_AXIS) : -Printer::maxSoftEndstopSteps[X_AXIS]);
        else
            steps = (Printer::isAxisHomed(X_AXIS) ? Printer::maxSoftEndstopSteps[X_AXIS] - Printer::xyMotion.motorSteps(X_AXIS) : Printer::maxSoftEndstopSteps[X_AXIS]);

        Printer::offsetRelativeStepsCoordinates(steps, 0, 0, 0, TASK_MOVE_FROM_BUTTON);
        break;
    }
    }
} // moveXAction

void moveYAction(int8_t direction) {
    bool moveKosys = Printer::manualMotion.coordinateSystem();

    if (Printer::pause.isPaused()) {
        moveKosys = KOSYS_DIRECTOFFSET;
    } else if (PrintLine::hasLines() || isAnyScanRunning()) {
        // this operation is not allowed while a printing or scanning is in progress
        return;
    }

    if (direction < 0 && Printer::isYMinEndstopHit()) {
        // we shall move to the back but the y-min-endstop is hit already, so we do nothing
        showInformation((void*)ui_text_y_axis, (void*)ui_text_min_reached);
        return;
    }

    if (direction > 0 && Printer::isYMaxEndstopHit()) {
        // we shall move to the front but the end of the y-axis has been reached already, so we do nothing
        showInformation((void*)ui_text_y_axis, (void*)ui_text_max_reached);
        return;
    }

    // show that we are active
    Printer::inactivity.recordActivity(HAL::timeInMilliseconds());

    switch (Printer::manualMotion.mode(Y_AXIS)) {
    case MOVE_MODE_SINGLE_STEPS: {
        int32_t steps = int32_t(Printer::manualMotion.steps(Y_AXIS)) * direction;

        if (direction < 0 && Printer::xyMotion.motorSteps(Y_AXIS) + steps < 0 && Printer::isAxisHomed(Y_AXIS)) {
            // If we are very close to the homed known Y-Endstop go to 0-Coordinate
            steps = -Printer::xyMotion.motorSteps(Y_AXIS);
        }

        if (moveKosys == KOSYS_GCODE) {
            float feedrate = STANDARD_POSITION_FEEDRATE_XY;
            if (Printer::manualMotion.feedrateSource() == FEEDRATE_GCODE) {
                feedrate = Printer::feedrate;
            }
            Printer::queueRelativeStepsCoordinates(0, steps, 0, 0, feedrate, true);
            Commands::printCurrentPosition();
        } else /*if (Printer::manualMotion.coordinateSystem() == KOSYS_DIRECTOFFSET) */
        {
            Printer::offsetRelativeStepsCoordinates(0, steps, 0, 0, TASK_MOVE_POSITION_MANUAL);
        }

        break;
    }
    case MOVE_MODE_1_MM:
    case MOVE_MODE_10_MM:
    case MOVE_MODE_50_MM: {
        uint8_t distanceMM = 1;
        if (Printer::manualMotion.mode(Y_AXIS) == MOVE_MODE_10_MM)
            distanceMM = 10;
        if (Printer::manualMotion.mode(Y_AXIS) == MOVE_MODE_50_MM)
            distanceMM = 50;

        int32_t plannedDestination = Printer::getPlannedDirectAxisSteps(Y_AXIS) + (int32_t)(direction * distanceMM * Printer::axisStepsPerMM[Y_AXIS]);
        if (plannedDestination > Printer::maxSoftEndstopSteps[Y_AXIS]) {
            showInformation((void*)ui_text_y_axis, (void*)ui_text_max_reached);
            return;
        }

        if (moveKosys == KOSYS_GCODE) {
            float feedrate = STANDARD_POSITION_FEEDRATE_XY;
            if (Printer::manualMotion.feedrateSource() == FEEDRATE_GCODE) {
                feedrate = Printer::feedrate;
            }
            Printer::queueRelativeMMCoordinates(0, distanceMM * direction, 0, 0, feedrate, true);
            Commands::printCurrentPosition();
        } else /*if (Printer::manualMotion.coordinateSystem() == KOSYS_DIRECTOFFSET) */
        {
            Printer::offsetRelativeStepsCoordinates(0, distanceMM * Printer::axisStepsPerMM[Y_AXIS] * direction, 0, 0, TASK_MOVE_POSITION_MANUAL);
        }

        break;
    }
    case MOVE_MODE_SINGLE_MOVE: {
        if (PrintLine::directMoveActive()) {
            // we are moving already, there is nothing more to do
            return;
        }

        int32_t steps;
        //errate homed möglichst den erzwungenen stop-punkt wegen dem bonus der decelleration
        if (direction < 0)
            steps = (Printer::isAxisHomed(Y_AXIS) ? -Printer::xyMotion.motorSteps(Y_AXIS) : -Printer::maxSoftEndstopSteps[Y_AXIS]);
        else
            steps = (Printer::isAxisHomed(Y_AXIS) ? Printer::maxSoftEndstopSteps[Y_AXIS] - Printer::xyMotion.motorSteps(Y_AXIS) : Printer::maxSoftEndstopSteps[Y_AXIS]);

        Printer::offsetRelativeStepsCoordinates(0, steps, 0, 0, TASK_MOVE_FROM_BUTTON);
        break;
    }
    }
} // moveYAction

void moveZAction(int8_t direction) {
    if (isAnyScanRunning() || !Printer::pause.allowsManualMove()) {
        // this operation is not allowed while some scan is in progress
        return;
    }

    // show that we are active
    Printer::inactivity.recordActivity(HAL::timeInMilliseconds());

    char moveMode = Printer::manualMotion.mode(Z_AXIS);

    if (uid.lastButtonAction == UI_ACTION_RF_HEAT_BED_DOWN || uid.lastButtonAction == UI_ACTION_RF_HEAT_BED_UP) {
        //es wäre evtl. cooler die variable durchzuschleifen, aber so gehts auch!
        //VORSICHT: ES könnte evtl. sein dass sich lastbuttonaction während der ausführung ändert. (??)

        //sollte diese Aktion durch die Knöpfe ausgeführt worden sein, dann in der Sprungweite limitieren:
        if (PrintLine::hasLines()) {
            // there is some printing in progress at the moment - do not allow single move in this case
            moveMode = MOVE_MODE_SINGLE_STEPS;
        } else {
            moveMode = MOVE_MODE_SINGLE_MOVE;
        }
    }

    if (Printer::pause.isPaused()) {
        moveMode = MOVE_MODE_SINGLE_MOVE;
    } else if (PrintLine::hasLines()) {
        // in case we are printing at the moment, only the single step movements are allowed
        moveMode = MOVE_MODE_SINGLE_STEPS;
    }

    //Limits für die Bewegung in Z
    if (direction > 0 && Printer::isZMaxEndstopHit()) {
        // we shall move to the front but the end of the y-axis has been reached already, so we do nothing
        showInformation((void*)ui_text_z_axis, (void*)ui_text_max_reached);
        return;
    }

    if (direction < 0 && Printer::isZMinEndstopHit()) {
        if (!Printer::isAxisHomed(Z_AXIS)) {
            return;
        } else {
            //fall down to Single Steps @Endstop
            moveMode = MOVE_MODE_SINGLE_STEPS;
        }
    }

    switch (moveMode) {
    case MOVE_MODE_SINGLE_STEPS: {
        int32_t steps = int32_t(Printer::manualMotion.steps(Z_AXIS)) * direction;

        if (Printer::zMotion.manualStepExceedsMinimum(steps)) {
            showInformation((void*)ui_text_z_axis, (void*)ui_text_min_reached);
            return;
        }

        steps = Printer::pause.limitZJog(steps);

        Printer::offsetRelativeStepsCoordinates(0, 0, steps, 0, TASK_MOVE_POSITION_MANUAL);
        break;
    }
    case MOVE_MODE_1_MM:
    case MOVE_MODE_10_MM:
    case MOVE_MODE_50_MM: {
        uint8_t distanceMM = 1;
        if (moveMode == MOVE_MODE_10_MM)
            distanceMM = 10;
        if (moveMode == MOVE_MODE_50_MM)
            distanceMM = 50;

        const long motorZ = Printer::zMotion.motorSteps();
        if (!Printer::isAxisHomed(Z_AXIS)
            || direction > 0
            || motorZ * Printer::axisMMPerSteps[Z_AXIS] >= distanceMM) {
            Printer::offsetRelativeStepsCoordinates(0, 0, distanceMM * Printer::axisStepsPerMM[Z_AXIS] * direction, 0, TASK_MOVE_POSITION_MANUAL);
            return;
        }

        // Approach the switch coordinate first, then allow small single moves.
        const long steps = motorZ > 0 ? -motorZ : -int32_t(Printer::manualMotion.steps(Z_AXIS));
        Printer::offsetRelativeStepsCoordinates(0, 0, steps, 0, TASK_MOVE_POSITION_MANUAL);
        break;
    }
    case MOVE_MODE_SINGLE_MOVE: {
        if (PrintLine::directMoveActive()) {
            // we are moving already, there is nothing more to do
            return;
        }

        // Plan to the estimated stop so a homed jog decelerates near it.
        long steps = direction < 0 ? -Printer::zMotion.maxSteps() : Printer::zMotion.maxSteps();
        if (Printer::isAxisHomed(Z_AXIS))
            steps = (direction < 0 ? 0L : Printer::zMotion.maxSteps()) - Printer::zMotion.motorSteps();

        steps = Printer::pause.limitZJog(steps);

        Printer::offsetRelativeStepsCoordinates(0, 0, steps, 0, TASK_MOVE_FROM_BUTTON);
        break;
    }
    }
} // moveZAction


#if DRV8711_NUM_CHANNELS != 5
#error Wer DRV8711_NUM_CHANNELS ändert, muss sich wirklich reindenken und von den Achszuordnungen bis zu den EEPROM-Speicherplätzen alles überdenken. Normalerweise darf dieser Wert nicht anders als 5 sein. Ausser Conrad baut ein Board mit 3 Extrudersteppern oder ähnlich.
#endif // DRV8711_NUM_CHANNELS != 5

void drv8711Transmit(unsigned short command) {
    // transfer the command (= direction, address and data)
    InterruptProtectedBlock noInts; //HAL::forbidInterrupts();
    for (char i = 15; i >= 0; i--) {
        WRITE(DRV_SDATI, command & (0x01 << i));
        HAL::delayMicroseconds(1);
        WRITE(DRV_SCLK, 1);
        HAL::delayMicroseconds(5);
        WRITE(DRV_SCLK, 0);
    }
    noInts.unprotect(); //HAL::allowInterrupts();

} // drv8711Transmit

void drv8711EnableAll(void) {
    // enable the chip select of all present DRV8711
    switch (DRV8711_NUM_CHANNELS) {
    case 5: {
        WRITE(O1_SCS_PIN, LOW);
        SET_OUTPUT(O1_SCS_PIN);
        WRITE(O1_SCS_PIN, HIGH);
    } // fall through
    case 4: {
        WRITE(O0_SCS_PIN, LOW);
        SET_OUTPUT(O0_SCS_PIN);
        WRITE(O0_SCS_PIN, HIGH);
    } // fall through
    case 3: {
        WRITE(Z_SCS_PIN, LOW);
        SET_OUTPUT(Z_SCS_PIN);
        WRITE(Z_SCS_PIN, HIGH);
    } // fall through
    case 2: {
        WRITE(Y_SCS_PIN, LOW);
        SET_OUTPUT(Y_SCS_PIN);
        WRITE(Y_SCS_PIN, HIGH);
    } // fall through
    case 1: {
        WRITE(X_SCS_PIN, LOW);
        SET_OUTPUT(X_SCS_PIN);
        WRITE(X_SCS_PIN, HIGH);
    }
    }

} // drv8711EnableAll

void drv8711DisableAll(void) {
    // disable the chip select of all present DRV8711
    switch (DRV8711_NUM_CHANNELS) {
    case 5: {
        WRITE(O1_SCS_PIN, LOW);
    } // fall through
    case 4: {
        WRITE(O0_SCS_PIN, LOW);
    } // fall through
    case 3: {
        WRITE(Z_SCS_PIN, LOW);
    } // fall through
    case 2: {
        WRITE(Y_SCS_PIN, LOW);
    } // fall through
    case 1: {
        WRITE(X_SCS_PIN, LOW);
    }
    }

} // drv8711DisableAll

void drv8711Enable(unsigned char driver) {
    // enable the chip select of the DRV8711
    switch (driver) {
    case 5: {
        WRITE(O1_SCS_PIN, LOW);
        SET_OUTPUT(O1_SCS_PIN);
        WRITE(O1_SCS_PIN, HIGH);
        break;
    }
    case 4: {
        WRITE(O0_SCS_PIN, LOW);
        SET_OUTPUT(O0_SCS_PIN);
        WRITE(O0_SCS_PIN, HIGH);
        break;
    }
    case 3: {
        WRITE(Z_SCS_PIN, LOW);
        SET_OUTPUT(Z_SCS_PIN);
        WRITE(Z_SCS_PIN, HIGH);
        break;
    }
    case 2: {
        WRITE(Y_SCS_PIN, LOW);
        SET_OUTPUT(Y_SCS_PIN);
        WRITE(Y_SCS_PIN, HIGH);
        break;
    }
    case 1: {
        WRITE(X_SCS_PIN, LOW);
        SET_OUTPUT(X_SCS_PIN);
        WRITE(X_SCS_PIN, HIGH);
        break;
    }
    }

} // drv8711Enable

void drv8711Disable(unsigned char driver) {
    // disable the chip select of the DRV8711
    switch (driver) {
    case 5: {
        WRITE(O1_SCS_PIN, LOW);
        break;
    }
    case 4: {
        WRITE(O0_SCS_PIN, LOW);
        break;
    }
    case 3: {
        WRITE(Z_SCS_PIN, LOW);
        break;
    }
    case 2: {
        WRITE(Y_SCS_PIN, LOW);
        break;
    }
    case 1: {
        WRITE(X_SCS_PIN, LOW);
        break;
    }
    }

} // drv8711Disable

uint8_t getmsb(unsigned int x) {
    int r = 0;
    if (x < 1)
        return 0;
    while (x >>= 1)
        r++;
    return r;
}

uint8_t drv8711MicroSteps_2_ModeValue(unsigned short microsteps) { //unknown steps fall back to highest bit!
    return getmsb(microsteps);                                     //highest bit << 1 -> 256->8, 128->7, ... integer log2
} //drv8711MicroSteps_2_ModeValue

unsigned short drv8711Axis_2_InitMicrosteps(uint8_t axis) {
    return (axis == X_AXIS || axis == Y_AXIS ? RF_MICRO_STEPS_XY : (axis == Z_AXIS ? RF_MICRO_STEPS_Z : (axis == E_AXIS || axis == E_AXIS + 1 ? RF_MICRO_STEPS_E : RF_MICRO_STEPS_XY /*error fallback*/
                                                                                                         )));
}

unsigned short drv8711ModeValue_2_MicroSteps(uint8_t modeValue) {
    if (modeValue > 8)
        return 256; //constrain, but ERROR -> avoid this!
    return (1 << modeValue);
} // drv8711ModeValue_2_MicroSteps

unsigned short drv8711MicroSteps_2_Register00Hex(unsigned short microsteps) {

    /* http://www.ti.com/product/DRV8711/datasheet/detailed-description#SLVSC405764
    DTIME
    00: 400 ns dead time
    01: 450 ns dead time
    10: 650 ns dead time
    11: 850 ns dead time

    ISENSE amplifier gain set
    00: Gain of 5
    01: Gain of 10
    10: Gain of 20
    11: Gain of 40

    EXSTALL
    0: Internal stall detect
    1: External stall detect

    MODE
    0000: Full-step, 71% current
    0001: Half step
    0010: 1/4 step
    0011: 1/8 step
    0100: 1/16 step
    0101: 1/32 step
    0110: 1/64 step
    0111: 1/128 step
    1000: 1/256 step
    1001 – 1111: Reserved
    */
    // ADRESS 11..8 7..4 3..0
    unsigned short Register00Hex = 0x0E01; // 0000 1110 0XXX X001: ENBL = 1, RDIR = 0, RSTEP = 0, MODE = XXXX, EXSTALL = 0, ISGAIN = 10, DTIME = 11

    Register00Hex += (drv8711MicroSteps_2_ModeValue(microsteps) << 3); //schiebe zu bit 3..6

    return Register00Hex;
} // drv8711MicroSteps_2_Register00Hex

void drv8711adjustMicroSteps(unsigned char driver) {
    drv8711Enable(driver);
    //X = 1
    //Y = 2
    //Z = 3
    //E0 = 4
    //E1 = 5
    unsigned short reg00bytes = drv8711MicroSteps_2_Register00Hex(drv8711ModeValue_2_MicroSteps(Printer::motorMicroStepsModeValue[driver - 1 /*=axis*/]));
    if (reg00bytes)
        drv8711Transmit(reg00bytes);
    drv8711Disable(driver);
    HAL::delayMicroseconds(1);
}

void drv8711Init(void) {
    // configure the pins
    WRITE(DRV_RESET1, LOW);
    SET_OUTPUT(DRV_RESET1);


    WRITE(DRV_SCLK, LOW);
    SET_OUTPUT(DRV_SCLK);
    WRITE(DRV_SDATI, LOW);
    SET_OUTPUT(DRV_SDATI);

    // configure the following inputs as pullup
    WRITE(DRV_SDATO, HIGH);
    WRITE(DRV_FAULT, HIGH);
    WRITE(X_STALL_PIN, HIGH);
    WRITE(Y_STALL_PIN, HIGH);
    WRITE(Z_STALL_PIN, HIGH);
    WRITE(O0_STALL_PIN, HIGH);
    WRITE(O1_STALL_PIN, HIGH);

    // reset all DRV8711 (active high)
    WRITE(DRV_RESET1, HIGH);


    HAL::delayMicroseconds(5000);
    WRITE(DRV_RESET1, LOW);


    HAL::delayMicroseconds(5000);

    // configure all registers except the motor current (= no register 01)

    //DRV8711_REGISTER_00:
    //init to stock settings: update in eeprom init.
    for (uint8_t axis = 0; axis < DRV8711_NUM_CHANNELS; axis++) {
        Printer::motorMicroStepsModeValue[axis] = drv8711MicroSteps_2_ModeValue(drv8711Axis_2_InitMicrosteps(axis)); //init
    }
    for (uint8_t driver = 1; driver <= DRV8711_NUM_CHANNELS; driver++) {
        drv8711adjustMicroSteps(driver);
    }
    // ADRESS 11..8 7..4 3..0

// Register 0x02: OFF (Off-Time Register)
// TOFF wurde auf 0x20 (16µs) reduziert, um die Chopping-Frequenz zu erhöhen
// und hörbares Zwitschern zu beseitigen. – Beibehalten.
// PWMMODE = 0 (interner Indexer, Standard für Mikroschritt) – Beibehalten.
#define DRV8711_REGISTER_02 0x2020 // 0010 0000 0010 0000: TOFF = 00100000 (0x20), PWMMODE = 0

// Register 0x03: BLANK (Blanking Time Register) -- intentionally NOT written.
// drv8711Init() hardware-resets the drivers above, leaving the DRV8711 default:
// TBLANK = 0x80 (2.56 us), ABT = 0 (adaptive blanking disabled).
// The previously copied 0xB2D7 has SPI bit 15 set (READ), so the old
// drv8711Transmit(0xB2D7) never applied the intended blanking settings.
// Do not silently replace it with a write such as 0x3102 (1.0 us, ABT on):
// blanking/decay timing can affect motor current, torque and missed steps.
// Keeping BLANK at its reset default preserves the existing motor behavior.

// Register 0x04: DECAY (Decay Mode Register)
// TDECAY wurde von 0x14 (10µs) auf 0x04 (2µs) geändert.
// Eine kürzere Decay-Zeit kann in manchen Fällen zu einer noch glatteren
// Stromwellenform und somit zu einer weiteren Geräuschreduzierung führen.
// DECMOD = 101 (Auto Mixed Decay) – Beibehalten, da dies für leisen Betrieb vorteilhaft ist.
#define DRV8711_REGISTER_04 0x4504 // 0100 0101 0000 0100: TDECAY = 00000100 (0x04), DECMOD = 101

#define DRV8711_REGISTER_05 0x583C // 0101   1000  0011 1100: SDTHR = 00111100, SDCNT = 00, VDIV = 10
#define DRV8711_REGISTER_06 0x60F0 // 0110   0000  1111 0000: OCPTH = 00, OCPDEG = 00, TDRIVEN = 11, TDRIVEP = 11, IDRIVEN = 00, IDRIVEP = 00
#define DRV8711_REGISTER_07 0x7000 // 0111   0000  0000 0000: OTS = 0, AOCP = 0, BOCP = 0, UVLO = 0, APDF = 0, BPDF = 0, STD = 0, STDLAT = 0

    drv8711EnableAll();
    drv8711Transmit(DRV8711_REGISTER_02);
    drv8711DisableAll();
    HAL::delayMicroseconds(1);

    drv8711EnableAll();
    drv8711Transmit(DRV8711_REGISTER_04);
    drv8711DisableAll();
    HAL::delayMicroseconds(1);

    drv8711EnableAll();
    drv8711Transmit(DRV8711_REGISTER_05);
    drv8711DisableAll();
    HAL::delayMicroseconds(1);

    drv8711EnableAll();
    drv8711Transmit(DRV8711_REGISTER_06);
    drv8711DisableAll();
    HAL::delayMicroseconds(1);

    drv8711EnableAll();
    drv8711Transmit(DRV8711_REGISTER_07);
    drv8711DisableAll();

    // set all motor currents
    const unsigned short uMotorCurrentUse[] = MOTOR_CURRENT_NORMAL; //--> {x,y,z,e1,e2} siehe RFx000.h
    Printer::motorCurrent[X_AXIS] = uMotorCurrentUse[X_AXIS];
    Printer::motorCurrent[Y_AXIS] = uMotorCurrentUse[Y_AXIS];
    Printer::motorCurrent[Z_AXIS] = uMotorCurrentUse[Z_AXIS];
    Printer::motorCurrent[E_AXIS + 0] = uMotorCurrentUse[E_AXIS + 0];
    Printer::motorCurrent[E_AXIS + 1] = uMotorCurrentUse[E_AXIS + 1]; //egal ob NUM_EXTRUDER == 1 oder 2

    for (uint8_t axis = 0; axis < DRV8711_NUM_CHANNELS; axis++) {
        if (axis < 3 + NUM_EXTRUDER)
            setMotorCurrent(axis + 1, Printer::motorCurrent[axis]);
    }

} // drv8711Init

void setMotorCurrent(unsigned char driver, uint8_t level) {
    // NOTE: Do not increase the current endlessly. In case the engine reaches its current saturation, the engine and the driver can heat up and loss power.
    // When the saturation is reached, more current causes more heating and more power loss.
    // In case of engines with lower quality, the saturation current may be reached before the nominal current.

    //X = 1
    //Y = 2
    //Z = 3
    //E0 = 4
    //E1 = 5

    // configure the pins
    WRITE(DRV_SCLK, LOW);
    SET_OUTPUT(DRV_SCLK);
    WRITE(DRV_SDATI, LOW);
    SET_OUTPUT(DRV_SDATI);

    drv8711Enable(driver);

    // we have to write to register 01
    unsigned short command = 0x1100 + (short)level;
    drv8711Transmit(command);

    drv8711Disable(driver);

} // setMotorCurrent


void cleanupEPositions(void) {
    InterruptProtectedBlock noInts;

    Printer::setEPositionMM(0);
    Printer::resetDirectAxis(E_AXIS);

    noInts.unprotect();
} // cleanupEPositions

bool switchActiveHeatBed(char newActiveHeatBed) {
    if (isAnyScanRunning()) {
        Com::printFLN(PSTR("selectMatrix(): scan in progress"));
        return false;
    }
    if (!saveActiveHeatBedToExtEEPROM(newActiveHeatBed)) {
        // do not allow not-supported z-compensation matrix
        return false;
    }

    g_nActiveHeatBed = newActiveHeatBed;

    if (loadCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed))) {
        // there is no valid z-compensation matrix available
        initCompensationMatrix();
    }
    return true;
} // switchActiveHeatBed

#ifdef RGB_LIGHT_R_PIN
void setRGBTargetColors(uint8_t R, uint8_t G, uint8_t B) {
    g_uRGBTargetR = R;
    g_uRGBTargetG = G;
    g_uRGBTargetB = B;

} // setRGBTargetColors

void setRGBLEDs(uint8_t R, uint8_t G, uint8_t B) {
    if (R > 0) {
        TCCR4A |= (1 << COM4A1); // R > 0 PWM start
        OCR4A = R * 80;
    } else {
        TCCR4A &= ~(1 << COM4A1); // R = 0 PWM stop
    }

    if (G > 0) {
        TCCR4A |= (1 << COM4B1); // G > 0 PWM start
        OCR4B = G * 80;
    } else {
        TCCR4A &= ~(1 << COM4B1); // G = 0 PWM stop
    }

    if (B > 0) {
        TCCR4A |= (1 << COM4C1); // B > 0 PWM start
        OCR4C = B * 80;
    } else {
        TCCR4A &= ~(1 << COM4C1); // B = 0 PWM stop
    }

    g_uRGBCurrentR = R;
    g_uRGBCurrentG = G;
    g_uRGBCurrentB = B;

} // setRGBLEDs

void updateRGBLightStatus(void) {
    char newStatus = RGB_STATUS_IDLE;

    if (Printer::RGBButtonBackPressed) {
        // toggle the white light
        Printer::RGBLightModeForceWhite = !Printer::RGBLightModeForceWhite;

        if (Printer::RGBLightModeForceWhite) {
            setRGBTargetColors(255, 255, 255);
        } else {
            if (Printer::RGBLightMode == RGB_MODE_OFF) {
                setRGBTargetColors(0, 0, 0);
            } else if (Printer::RGBLightMode == RGB_MODE_AUTOMATIC) {
                Printer::RGBLightStatus = RGB_STATUS_AUTOMATIC;
            } else if (Printer::RGBLightMode == RGB_MODE_MANUAL) {
                setRGBTargetColors(g_uRGBManualR, g_uRGBManualG, g_uRGBManualB);
            }
        }

        Printer::RGBButtonBackPressed = 0;
    }

    if (Printer::RGBLightModeForceWhite) {
        // there is nothing to do in case we shall display the white light
        return;
    }

    if (Printer::RGBLightStatus == RGB_STATUS_NOT_AUTOMATIC) {
        // there is nothing to do in case we shall not change the RGB colors automatically
        return;
    }


    // operating mode print
    for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
        if (extruder[i].tempControl.targetTemperatureC > MAX_ROOM_TEMPERATURE) {
            if (fabs(extruder[i].tempControl.targetTemperatureC - extruder[i].tempControl.currentTemperatureC) < RGB_LIGHT_TEMP_TOLERANCE) {
                // we have reached the target temperature
                newStatus = RGB_STATUS_PRINTING;
            } else if (extruder[i].tempControl.targetTemperatureC > extruder[i].tempControl.currentTemperatureC) {
                // we are still heating
                newStatus = RGB_STATUS_HEATING;
            } else {
                // we end up here in case the target temperature is below the current temperature (this happens typically when the target temperature is reduced after the first layer)
            }
        }
    }

    if (heatedBedController.targetTemperatureC > MAX_ROOM_TEMPERATURE) {
        if (fabs(heatedBedController.targetTemperatureC - heatedBedController.currentTemperatureC) < RGB_LIGHT_TEMP_TOLERANCE) {
            // we have reached the target temperature
            if (newStatus == RGB_STATUS_IDLE) {
                newStatus = RGB_STATUS_PRINTING;
            }
        } else if (heatedBedController.targetTemperatureC > heatedBedController.currentTemperatureC) {
            // we are still heating
            newStatus = RGB_STATUS_HEATING;
        } else {
            // we end up here in case the target temperature is below the current temperature (this happens typically when the target temperature is reduced after the first layer)
        }
    }

    for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
        if ((extruder[i].tempControl.currentTemperatureC - extruder[i].tempControl.targetTemperatureC) > COOLDOWN_THRESHOLD) {
            // we shall cool down
            if (newStatus == RGB_STATUS_IDLE) {
                newStatus = RGB_STATUS_COOLING;
            }
        }
    }

    if ((heatedBedController.currentTemperatureC - heatedBedController.targetTemperatureC) > COOLDOWN_THRESHOLD) {
        // we shall cool down
        if (newStatus == RGB_STATUS_IDLE) {
            newStatus = RGB_STATUS_COOLING;
        }
    }

    if (newStatus != Printer::RGBLightStatus) {
        if (newStatus == RGB_STATUS_IDLE && Printer::RGBLightStatus == RGB_STATUS_COLOR_CHANGE) {
            // when we are in color change mode already we shall not switch back to idle
        } else {
            Printer::RGBLightStatus = newStatus;
            //          Com::printFLN( PSTR( "new RGB light status: " ), Printer::RGBLightStatus );

            switch (Printer::RGBLightStatus) {
            case RGB_STATUS_PRINTING: {
                setRGBTargetColors(g_uRGBPrintingR, g_uRGBPrintingG, g_uRGBPrintingB);
                break;
            }
            case RGB_STATUS_HEATING: {
                setRGBTargetColors(g_uRGBHeatingR, g_uRGBHeatingG, g_uRGBHeatingB);
                break;
            }
            case RGB_STATUS_COOLING: {
                setRGBTargetColors(g_uRGBCoolingR, g_uRGBCoolingG, g_uRGBCoolingB);
                break;
            }
            case RGB_STATUS_IDLE: // fall through
            default: {
                Printer::RGBLightIdleStart = HAL::timeInMilliseconds();
                Printer::RGBLightStatus = RGB_STATUS_IDLE;
                setRGBTargetColors(g_uRGBIdleR, g_uRGBIdleG, g_uRGBIdleB);
                break;
            }
            }
        }
    }

    if (Printer::RGBLightStatus == RGB_STATUS_IDLE && Printer::RGBLightIdleStart) {
        if ((HAL::timeInMilliseconds() - Printer::RGBLightIdleStart) / 1000 > RGB_LIGHT_COLOR_CHANGE_DELAY) {
            Printer::RGBLightStatus = RGB_STATUS_COLOR_CHANGE;
            Printer::RGBLightIdleStart = 0;

            //          Com::printFLN( PSTR( "new RGB light status: " ), Printer::RGBLightStatus );
            setRGBTargetColors(255, 0, 0);
        }
    }

    if (Printer::RGBLightStatus == RGB_STATUS_COLOR_CHANGE) {
        if (g_uRGBTargetR == g_uRGBCurrentR && g_uRGBTargetG == g_uRGBCurrentG && g_uRGBTargetB == g_uRGBCurrentB) {
            if (g_uRGBTargetR == 255 && g_uRGBTargetG == 0 && g_uRGBTargetB == 0) {
                g_uRGBTargetG = 255;
            } else if (g_uRGBTargetR == 255 && g_uRGBTargetG == 255 && g_uRGBTargetB == 0) {
                g_uRGBTargetR = 0;
            } else if (g_uRGBTargetR == 0 && g_uRGBTargetG == 255 && g_uRGBTargetB == 0) {
                g_uRGBTargetB = 255;
            } else if (g_uRGBTargetR == 0 && g_uRGBTargetG == 255 && g_uRGBTargetB == 255) {
                g_uRGBTargetG = 0;
            } else if (g_uRGBTargetR == 0 && g_uRGBTargetG == 0 && g_uRGBTargetB == 255) {
                g_uRGBTargetR = 255;
            } else if (g_uRGBTargetR == 255 && g_uRGBTargetG == 0 && g_uRGBTargetB == 255) {
                g_uRGBTargetB = 0;
            }
        }
    }

    return;

} // updateRGBLightStatus
#endif // RGB_LIGHT_R_PIN

static void setupForPrinting() {
    Printer::setSomeTempsensorDefect(false);

    // read the settings from the EEPROM
    Printer::homingFeedrate[X_AXIS] = HAL::eprGetFloat(EPR_X_HOMING_FEEDRATE_PRINT);
    Printer::homingFeedrate[Y_AXIS] = HAL::eprGetFloat(EPR_Y_HOMING_FEEDRATE_PRINT);
    Printer::homingFeedrate[Z_AXIS] = HAL::eprGetFloat(EPR_Z_HOMING_FEEDRATE_PRINT);

    Printer::axisLengthMM[X_AXIS] = HAL::eprGetFloat(EPR_X_LENGTH);
    if (Printer::axisLengthMM[X_AXIS] <= 0 || Printer::axisLengthMM[X_AXIS] > 245.0f) {
        Printer::axisLengthMM[X_AXIS] = X_MAX_LENGTH_PRINT;
        HAL::eprSetFloat(EPR_X_LENGTH, Printer::axisLengthMM[X_AXIS]);
        EEPROM::updateChecksum();
    }

    loadActiveHeatBedFromExtEEPROM();
    if (g_ZCompensationMatrix[0][0] != EEPROM_FORMAT) {
        // we load the z compensation matrix before its first usage because this can take some time
        // This function needs the corrected axis length for restoreDefaultScanParameters();
        prepareZCompensation();
    }

    Printer::pause.restoreDefaultParking();

    Printer::updateDerivedParameter();

    Printer::idleStatus.schedule(HAL::timeInMilliseconds());
} // setupForPrinting

void prepareZCompensation(void) {
    if (COMPENSATION_MATRIX_SIZE > EEPROM_SECTOR_SIZE) {
        if (Printer::debugErrors()) {
            Com::printFLN(PSTR("prepareZCompensation(): the size of the compensation matrix is too big"));
        }

        // TODO: show a message at the display in this case
        return;
    }

    // restore the default scan parameters
    restoreDefaultScanParameters();

    // restore the last known compensation matrix
    // this operation must be performed after restoring of the default scan parameters because the values from the EEPROM can overwrite some scan parameters
    if (loadCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed))) {
        // there is no valid compensation matrix available
        initCompensationMatrix();
        Com::printFLN(PSTR("prepareZCompensation(): the compensation matrix is not available"));
    }

} // prepareZCompensation

unsigned char isMovingAllowed(const char* pszCommand, char outputLog) {
    if (!Printer::g_unlock_movement) {
        // A host may keep sending motion commands after a controller reset.
        Com::printF(pszCommand);
        Com::printFLN(PSTR(": movement locked until homing or panel input"));
        return 0;
    }
    if (Printer::blockAll) {
        // do not allow to move in case the movements have been blocked
        return 0;
    }

    if (isAnyScanRunning()) {
        // do not allow manual movements while the heat bed scan is in progress
        if (Printer::debugErrors() && outputLog) {
            Com::printF(pszCommand);
            Com::printFLN(PSTR(": command can not be used while scan is in progress"));
        }
        return 0;
    }

    // we allow the manual movements at the moment
    return 1;

} // isMovingAllowed

unsigned char isHomingAllowed(GCode* com, char outputLog) {

    if (isAnyScanRunning()) {
        // do not allow homing while the heat bed scan is in progress
        if (Printer::debugErrors() && outputLog) {
            Com::printFLN(PSTR("G28: homing can not be performed while the heat bed scan is in progress"));
        }
        showError((void*)ui_text_home, (void*)ui_text_operation_denied);

        return 0;
    }

    if (!com) {
        // there is nothing more which we can check
        return 1;
    }

    // we allow the homing at the moment
    return 1;
} // isHomingAllowed

void showInvalidSyntax(unsigned int currentMCode) {
    if (Printer::debugErrors()) {
        Com::printF(PSTR("M"), (int)currentMCode);
        Com::printFLN(PSTR(": invalid syntax"));
    }
    return;

} // showInvalidSyntax

void addUInt32(char* pszString, uint32_t uNumber) {
    char szTemp[11];
    char* pszTemp = &szTemp[10];
    uint32_t uTemp;

    *pszTemp = '\0';
    do {
        uTemp = uNumber;
        uNumber /= 10;
        *--pszTemp = '0' + (uTemp - 10 * uNumber);
    } while (uNumber);

    strcpy(pszString, pszTemp);

} // addUInt32

void addFloat(char* pszString, float fNumber, uint8_t uDigits) {
    float fRounding = 0.5;
    uint32_t uNumber;
    float fRemainder;
    uint8_t uRemainder;
    uint8_t i;

    // add at the end of the string
    while (*pszString) {
        pszString++;
    }

    if (isnan(fNumber)) {
        strcpy(pszString, "NAN");
        return;
    }
    if (isinf(fNumber)) {
        strcpy(pszString, "INF");
        return;
    }

    // Handle negative numbers
    if (fNumber < 0.0) {
        *pszString = '-';
        pszString++;
        *pszString = '\0';
        fNumber = -fNumber;
    }

    for (i = 0; i < uDigits; i++) {
        fRounding /= 10.0;
    }

    fNumber += fRounding;

    uNumber = (uint32_t)fNumber;
    fRemainder = fNumber - (float)uNumber;
    addUInt32(pszString, uNumber);

    if (uDigits > 0) {
        while (*pszString) {
            pszString++;
        }

        *pszString = '.';
        pszString++;

        while (uDigits-- > 0) {
            fRemainder *= 10.0;
            uRemainder = (uint8_t)fRemainder;
            *pszString = '0' + uRemainder;
            pszString++;
            fRemainder -= uRemainder;
        }

        *pszString = '\0';
    }

} // addFloat

void determineHardwareType(void) {
    unsigned short uTemp;

    Printer::wrongType = 0;

    // check the stored header format
    uTemp = readWord24C256(I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_HEADER_FORMAT);

    if (uTemp != TYPE_EEPROM_FORMAT) {
        // we could not read the header format or the header format is wrong
        writeWord24C256(I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_HEADER_FORMAT, TYPE_EEPROM_FORMAT);
        writeWord24C256(I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_BOARD_TYPE, MOTHERBOARD);

        uTemp = readWord24C256(I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_HEADER_FORMAT);

        if (uTemp != TYPE_EEPROM_FORMAT) {
            if (MOTHERBOARD == DEVICE_TYPE_RF1000) {
                // the RF1000 does not provide the type EEPROM, thus we should run at an RF1000 board when we end up here
                return;
            }

            // we end up here in case this firmware is for the RF2000, but the current board does not seem to be an RF2000 board
            Printer::wrongType = 1;
            return;
        }

        if (MOTHERBOARD == DEVICE_TYPE_RF1000) {
            // we end up here in case this firmware is for the RF1000, but the current board seems to be an RF2000 board
            Printer::wrongType = 1;
            return;
        }

        // when we end up here this firmware is for the RF2000 and the current board seems to be an RF2000 board
        return;
    }

    if (MOTHERBOARD == DEVICE_TYPE_RF1000) {
        // we end up here in case this firmware is for the RF1000, but the current board seems to be an RF2000 board
        Printer::wrongType = 1;
        return;
    }

    // when we end up here this firmware is for the RF2000 and the current board seems to be an RF2000 board
    if (readWord24C256(I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_BOARD_TYPE) != MOTHERBOARD) {
        writeWord24C256(I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_BOARD_TYPE, MOTHERBOARD);
    }

    return;

} // determineHardwareType

void showIdle(void) {
    UI_STATUS(UI_TEXT_PRINTER_READY);
} // showIdle

void showError(const void* line2, const void* line3, const void* line4) {
    uid.showMessage((const void*)ui_text_error, line2, line3, line4);

} // showError

void showWarning(const void* line2, const void* line3, const void* line4) {
    uid.showMessage((const void*)ui_text_warning, line2, line3, line4);

} // showWarning

void showInformation(const void* line2, const void* line3, const void* line4) {
    uid.showMessage((const void*)ui_text_information, line2, line3, line4);

} // showInformation

void showMyPage(const void* line1, const void* line2, const void* line3, const void* line4) {
    uid.showMessage(line1, line2, line3, line4);

} // showMyPage

void doEmergencyStop(char reason) {
    {
        InterruptProtectedBlock lock;
        Printer::blockAll = 1;
        Printer::extrusionOutput.inhibit(ExtrusionOutput::Inhibit::Emergency);
    }

    showError((void*)ui_text_emergency_stop);

    Com::printF(PSTR("doEmergencyStop(): block all"));
    if (reason == STOP_BECAUSE_OF_Z_MIN) {
        Com::printFLN(PSTR(" (Z-Min)"));
    } else if (reason == STOP_BECAUSE_OF_Z_BLOCK) {
        Com::printFLN(PSTR(" (Z-Block)"));
    }

    Printer::stopPrint(); //and tell usb senders to stop

    moveZ(int(Printer::axisStepsPerMM[Z_AXIS] * 5), true);

    // we are not going to perform any further operations until the restart of the firmware
    Printer::switchEverythingOff();
} // doEmergencyStop

void addLong(char* string, long value, char digits) {
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

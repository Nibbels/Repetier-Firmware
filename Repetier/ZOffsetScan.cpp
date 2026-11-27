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
#include "ZScrewAdjustment.h"
#include "GCodeCommands.h"
#include "GCodeInput.h"

FSTRINGVALUE(ui_text_heat_bed_zoffset_search_status, UI_TEXT_HEAT_BED_ZOFFSET_SEARCH_STATUS)
FSTRINGVALUE(ui_text_heat_bed_zoffset_fix_z1, UI_TEXT_HEAT_BED_ZOFFSET_FIX_Z1)
FSTRINGVALUE(ui_text_heat_bed_zoffset_fix_z2, UI_TEXT_HEAT_BED_ZOFFSET_FIX_Z2)
FSTRINGVALUE(ui_text_statusmsg, UI_TEXT_STATUSMSG)

constexpr int zOffsetScanContactPressureDelta = 33;
constexpr int zOffsetScanRetryPressureDelta = 30;
constexpr int zOffsetScanRetryDelayMs = 1000;

namespace {

using Calibration::PressureResult;
using Calibration::ProbeProgress;

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
        Calibration::scanProbe.reset();
        Calibration::scanProbe.forgetPreviousPoint();
        Calibration::scanSession.reset();
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
        Calibration::scanSession.requestAbort(ScanAbort::Canceled);
        return;
    }

    // Like a mesh scan, Z-offset calibration must not begin inside a
    // print, filament program or the pending post-stop output phase.
    if (Printer::printJob.isBusy() || Printer::printJob.commandsBlocked()) {
        Com::printFLN(PSTR("ZOS requires idle print job"));
        return;
    }
    Com::printFLN(PSTR("ZOS started"));
    Calibration::scanSession.reset();
    levelingPass_ = levelMatrix ? LevelingPass::Center : LevelingPass::None;
    phase_ = Phase::Initialize;
}

void ZOffsetScan::tick() {
    if (!active())
        return;

    Printer::inactivity.recordActivity(HAL::timeInMilliseconds());
    GCodeInput::keepAlive(Calibrating);
    if (Calibration::scanSession.aborting()) {
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
    probeZSteps_ = long(Printer::stepsPerMm(Z_AXIS) * Calibration::probeSettings().startLiftMm);
    Com::printF(Com::tAutoMatrixLeveling, settings_.point.x);
    Com::printFLN(PSTR(" Y:"), settings_.point.y);
}

void ZOffsetScan::initialize() {
    Printer::idleStatus.cancel();
    Com::printFLN(PSTR("ZOS init"));
    phase_ = Phase::HomeForProbe;
    probeZSteps_ = long(Printer::stepsPerMm(Z_AXIS) * Calibration::probeSettings().startLiftMm); // Nur nutzen wenn kleiner.
}

void ZOffsetScan::homeForProbe() {
    // Bissel übertrieben, sollte aber jede eventualität abfangen: Wir brauchen die maximale Matrix-Dimension auch schon hier (ganz grob) und wollen nicht so lange warten.
    unsigned short uDimensionX, uDimensionY;
    if (bedMatrixAvailable()) {
        uDimensionX = BedMatrix::maxIndex(X_AXIS) - 1;
        uDimensionY = BedMatrix::maxIndex(Y_AXIS) - 1;
    } else {
        uDimensionX = BedMatrix::storedMaxIndex(X_AXIS) - 1;
        uDimensionY = BedMatrix::storedMaxIndex(Y_AXIS) - 1;
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
    if (!Calibration::moveZ(probeZSteps_))
        return;
    UI_STATUS_UPD(UI_TEXT_ZCALIB);
    phase_ = Phase::LoadMatrix;
}

void ZOffsetScan::loadMatrix() {
    // load the unaltered compensation matrix from the EEPROM
    if (!bedMatrixAvailable() || settings_.learningRate == 1.0) {
        Com::printFLN(PSTR("Loading zMatrix from EEPROM"));
        if (!BedMatrix::reloadActive()) {
            // Error: there is no valid compensation matrix available
            BedMatrix::clearWorking();
        }
    } else {
        Com::printFLN(PSTR("Reusing existing zMatrix"));
    }

    // safety check on the current matrix
    if (!bedMatrixAvailable()) {
        Com::printFLN(PSTR("ERROR::prev. matrix invalid!"));
        Calibration::scanSession.requestAbort(ScanAbort::MissingMatrix);
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
    point.x = RMath::min(RMath::max(int(point.x), minX), BedMatrix::maxIndex(X_AXIS) - 1);
    point.y = RMath::min(RMath::max(int(point.y), minY), BedMatrix::maxIndex(Y_AXIS) - 1);
    float xScanPosition = BedMatrix::entry(point.x, 0);
    float yScanPosition = BedMatrix::entry(0, point.y);

    constexpr long scanPositionJitterMm = 3;
    xScanPosition += random(-scanPositionJitterMm, scanPositionJitterMm);
    yScanPosition += random(-scanPositionJitterMm, scanPositionJitterMm);

    Printer::queueRelativeMMCoordinates(xScanPosition, yScanPosition, 0, 0, RMath::min(Printer::axisHomingFeedrate(X_AXIS), Printer::axisHomingFeedrate(Y_AXIS)), true, true);
    phase_ = Phase::ArmPressureRetries;
}

void ZOffsetScan::armPressureRetries() {
    Calibration::scanSession.armRetries(20);
    phase_ = Phase::MeasureBaseline;
}

void ZOffsetScan::beginRetryWait() {
    Calibration::scanProbe.reset();
    retryWaitStartedMs_ = HAL::timeInMilliseconds();
    phase_ = Phase::RetreatForRetry;
}

void ZOffsetScan::retreatForRetry() {
    if (HAL::timeInMilliseconds() - retryWaitStartedMs_ < zOffsetScanRetryDelayMs)
        return;
    if (Calibration::scanProbe.retreat() == ProbeProgress::Done)
        phase_ = Phase::MeasureBaseline;
}

void ZOffsetScan::measureBaseline() {
    short baseline;
    const auto result = Calibration::scanPressure.idle(baseline);
    if (result == PressureResult::Pending)
        return;
    if (result != PressureResult::Stable) {
        retryProbe(ScanAbort::IdlePressure);
        return;
    }

    Calibration::scanProbe.setPressureLimits(baseline, zOffsetScanContactPressureDelta, zOffsetScanRetryPressureDelta);

    Com::printFLN(PSTR("Approaching HeatBed"));
    Calibration::scanProbe.forgetPreviousPoint();
    phase_ = Phase::MeasureSurface;
}

void ZOffsetScan::retryProbe(ScanAbort failure) {
    if (Calibration::scanSession.consumeRetry()) {
        beginRetryWait();
        return;
    }
    Calibration::scanSession.requestAbort(failure);
}

void ZOffsetScan::measureSurface() {
    const auto progress = Calibration::scanProbe.measurePoint();
    if (progress == ProbeProgress::Pending)
        return;
    if (progress == ProbeProgress::Failed) {
        retryProbe(ScanAbort::AveragePressure);
        return;
    }

    const long measuredZ = Calibration::scanProbe.measuredZ();
    if (measuredZ < probeZSteps_)
        probeZSteps_ = measuredZ;
    Com::printFLN(PSTR("Z = "), measuredZ * Printer::mmPerStep(Z_AXIS), 4);
    phase_ = Phase::ApplyMatrixCorrection;
}

void ZOffsetScan::applyMatrixCorrection() {
    // compute number of steps we need to shift the entire matrix by
    long matrixwert = getZMatrixDepth_CurrentXY();
    long nZ = probeZSteps_ - matrixwert;

    Com::printFLN(PSTR("Matrix-Wert Z = "), matrixwert);
    Com::printF(PSTR("Minimum Z = "), probeZSteps_);
    Com::printFLN(PSTR(" dZ = "), nZ);

    nZ = (long)((float)nZ * settings_.learningRate);
    const auto& settings = Calibration::probeSettings();
    const long maximumHeight = long(Printer::stepsPerMm(Z_AXIS) * settings.startLiftMm) + settings.fastApproachSteps;
    const auto result = BedMatrix::applyZOffset(nZ, settings_.point, settings_.learningGradient, maximumHeight);

    // fail if overflow occurred
    if (result.overflow) {
        // load the unaltered compensation matrix from the EEPROM since the current in-memory matrix is invalid
        Com::printFLN(PSTR("Matrix Overflow!"));
        Calibration::scanSession.requestAbort(ScanAbort::MatrixOverflow);
        abort(true);
        return;
    }
    // fail if z>(Starthöhe - ein bisschen) occurred
    if (result.tooHigh) {
        // load the unaltered compensation matrix from the EEPROM since the current in-memory matrix is bigger than z=zero
        Com::printFLN(PSTR("ERROR::Z-Matrix höher Start-Z!"));
        Calibration::scanSession.requestAbort(ScanAbort::MatrixHeight);
        abort(true);
        return;
    }
    // fail if z>0 occurred
    if (result.aboveZero) {
        // load the unaltered compensation matrix from the EEPROM since the current in-memory matrix is bigger than z=zero
        Com::printFLN(PSTR("FIX::Clean Hotend-Nozzle"));
        Com::printFLN(PSTR("FIX::Fix Z-Schraube"));
    }
    // determine the minimal distance between extruder and heat bed
    BedMatrix::finishZOffset();

    phase_ = Phase::ContinueLeveling;
}

void ZOffsetScan::continueLeveling() {
    if (levelingPass_ == LevelingPass::None) {
        phase_ = Phase::RetreatForFinish;
        return;
    }
    phase_ = Phase::HomeForProbe;
    if (!Calibration::moveZ(Printer::stepsPerMm(Z_AXIS)))
        return;
    Printer::homeAxis(true, true, false);
    if (!Calibration::moveZ(-Printer::zMotion.scanSteps()))
        return;
    BedMatrix::report(1);
}

void ZOffsetScan::retreatForFinish() {
    if (!Calibration::moveZ(Printer::stepsPerMm(Z_AXIS)))
        return;
    Printer::homeAxis(false, true, false);
    if (!Calibration::moveZ(-Printer::zMotion.scanSteps()))
        return;
    phase_ = Phase::ReportScrewAdjustment;    // No AUTO_MATRIX_LEVELING
}

void ZOffsetScan::reportScrewAdjustment() {
    discard();
    ZScrewAdjustment::calculate();

    showMyPage((void*)ui_text_heat_bed_zoffset_search_status, (void*)ui_text_heat_bed_zoffset_fix_z1, (void*)ui_text_heat_bed_zoffset_fix_z2, (void*)ui_text_statusmsg);
    uid.scheduleMessageReturn(HAL::timeInMilliseconds() + 30000);
    Printer::idleStatus.schedule(HAL::timeInMilliseconds() + 30000);

    //Printer::zMotion.scanSteps() is 0 now.
    Com::printFLN(PSTR("ZOS finished"));
}

void ZOffsetScan::abort(bool reloadMatrix) {
    phase_ = Phase::Aborting;
    const auto reason = Calibration::scanSession.takeAbort();
#if defined(RF_THERMAL_EMERGENCY_EXPERIMENT) && defined(RF_EEPROM_ZONE_EXPERIMENT)
    DiagnosticHistory::recordScan(reason,DiagnosticHistory::ScanKind::ZOffset);
#endif
    Calibration::showAbortScanReason(PSTR(UI_TEXT_DO_MHIER_BED_SCAN), reason);

    if (reloadMatrix) {
        if (!BedMatrix::reloadActive()) {
            // Error: there is no valid compensation matrix available
            BedMatrix::clearWorking();
        }
    }
    // the search has been aborted
    Com::printFLN(PSTR("ZOS aborted"));

    if (ScanSession::retreatOnly(reason)) {
        Calibration::moveZ(int(Printer::stepsPerMm(Z_AXIS) * 5));
    } else if (Printer::queueRelativeMMCoordinates(0, 0, 5, 0, Printer::axisHomingFeedrate(Z_AXIS), true, true)) {
        Printer::homeAxis(true, true, true);
    }

    // turn off all steppers and extruders
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    // A nested wait may have latched the terminal stop. Deferred discard
    // belongs to the scan dispatcher, not the normal mechanical abort.
    if(Printer::printJob.emergencyLocked())return;
#endif
    Printer::disableAllSteppersNow();

    discard();
    Printer::idleStatus.schedule(HAL::timeInMilliseconds() + 30000);
}

} // namespace

void startZOScan(bool autoLevelMatrix) {
    if (!heatBedScanRunning())
        zOffsetScan.startOrCancel(autoLevelMatrix);
}

bool Calibration::zOffsetScanRunning() {
    return zOffsetScan.active();
}

ZOffsetScanPoint zOffsetScanPoint() {
    return zOffsetScan.settings().point;
}

void restoreZOffsetScanPoint(ZOffsetScanPoint point) {
    zOffsetScan.restorePoint(point);
}

void Calibration::tickZOffsetScan() { zOffsetScan.tick(); }
void Calibration::discardZOffsetScan() { zOffsetScan.discard(); }

namespace GCodeCommands {
namespace Calibration {

void configureZOffsetScan(GCode* pCommand) {
#ifdef RF_EEPROM_ZONE_EXPERIMENT
    if(!EEPROM::settingsReady())return;
#endif
    if (heatBedScanRunning()) {
        Com::printFLN(PSTR("M3900/M3901: bed scan in progress"));
        return;
    }
    bool err3900r = false;
    if (!bedMatrixAvailable()) {
        // we load the z compensation matrix before its first usage because this can take some time
        Com::printFLN(PSTR("M3900/M3901: INFO Die Z-Matrix wurde aus dem EEPROM gelesen."));
        prepareZCompensation();
    }

    if (bedMatrixAvailable()) {
        if (!pCommand->hasR()) { // Normaler ZOS, kein Auto-Matrix-Leveling.
            auto settings = zOffsetScan.settings();
            if (pCommand->hasX()) {
                // test and take over the specified value
                unsigned char nTempUC = (unsigned char)pCommand->X;

                // Wessix idee: zufalls-Scanpunkt wegen DDP-Platten-Schonung.
                if (nTempUC == 0)
                    nTempUC = (unsigned char)random((heatBedScanXStartMm == 0) ? 1 : 2, BedMatrix::maxIndex(X_AXIS)); // ergibt min bis max-1

                // Wenn die Startposition nicht 0 ist, wird eine Dummy-Matrix-Linie ergänzt. Mit der sollten wir nicht arbeiten.
                if (nTempUC < 1 + ((heatBedScanXStartMm == 0) ? 0 : 1))
                    nTempUC = 1 + ((heatBedScanXStartMm == 0) ? 0 : 1);
                if (nTempUC > BedMatrix::maxIndex(X_AXIS) - 1)
                    nTempUC = BedMatrix::maxIndex(X_AXIS) - 1; //2..n-1

                settings.point.x = nTempUC;


            }

            if (pCommand->hasY()) {
                // test and take over the specified value
                unsigned char nTempUC = (unsigned char)pCommand->Y;

                // Wessix idee: zufalls-Scanpunkt wegen DDP-Platten-Schonung.
                if (nTempUC == 0)
                    nTempUC = (unsigned char)random((heatBedScanYStartMm == 0) ? 1 : 2, BedMatrix::maxIndex(Y_AXIS)); // ergibt min bis max-1

                if (nTempUC < 1 + ((heatBedScanYStartMm == 0) ? 0 : 1))
                    nTempUC = 1 + ((heatBedScanYStartMm == 0) ? 0 : 1);
                if (nTempUC > BedMatrix::maxIndex(Y_AXIS) - 1)
                    nTempUC = BedMatrix::maxIndex(Y_AXIS) - 1; //2..n-1

                settings.point.y = nTempUC;


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

            if(pCommand->hasX() || pCommand->hasY())
                EEPROM::storeScanPoint(settings.point.x,settings.point.y,pCommand->hasX(),pCommand->hasY());
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
                while (::Calibration::zOffsetScanRunning()) {
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

} // namespace Calibration
} // namespace GCodeCommands

#include "ScanSettings.h"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include "CalibrationScan.h"
#include "PressureMeasurement.h"
#include "scan-strain-gauge.h"

namespace {
using millis_t = uint32_t;
enum { X_AXIS, Y_AXIS, Z_AXIS };
enum { EEPROM_FORMAT = 42, COMPENSATION_MATRIX_MAX_X = 8, COMPENSATION_MATRIX_MAX_Y = 8 };
enum { EEPROM_SECTOR_SIZE = 1536 };
template<class T>
T constrain(T value, T minimum, T maximum) { return std::min(std::max(value, minimum), maximum); }

constexpr int heatBedScanDelayMs = 1000, heatBedScanCalibrationPointMm = 100;
constexpr int heatBedScanPlaBedC = 60, heatBedScanPlaNozzleC = 210, heatBedScanAbsBedC = 100,
              heatBedScanAbsNozzleC = 240, heatBedScanProbeNozzleC = 100;
constexpr millis_t preciseBedScanWarmupSeconds = 600, preciseBedScanCalibrationSeconds = 600;
#define PSTR(text) text
#define strcpy_P(destination, source) std::strcpy(destination, source)
#define strcat_P(destination, source) std::strcat(destination, source)
#define UI_STATUS(text) ((void)(text))
#define UI_STATUS_UPD(text) ((void)(text))
#define UI_STATUS_UPD_RAM(text) checkHeatingStatus(text)
constexpr char UI_TEXT_HEATING[] = "Heating...", UI_TEXT_HEAT_BED_SCAN[] = "Scan",
               UI_TEXT_HEAT_BED_SCAN_DONE[] = "Done", UI_TEXT_OK[] = "OK";
constexpr char ui_text_heat_bed_scan[] = "Scan", ui_text_operation_denied[] = "Busy", ui_text_saving_success[] = "Saved";

void checkHeatingStatus(const char* text) {
    const auto prefix = std::strlen(UI_TEXT_HEATING);
    assert(std::strncmp(text, UI_TEXT_HEATING, prefix) == 0);
    const char* end = text + std::strlen(text) - 3;
    assert(end > text + prefix && std::strcmp(end, "[s]") == 0);
    unsigned seconds = 0;
    for (auto p = text + prefix; p < end; ++p) {
        assert(*p >= '0' && *p <= '9');
        seconds = seconds * 10 + unsigned(*p - '0');
    }
    assert(seconds >= 1 && seconds <= 600);
}

char g_nActiveHeatBed = 2;
bool g_ZMatrixChangedInRam = false;
ScanSettings scanSettings = [] {
    ScanSettings settings(1.0f);
    settings.fastApproachSteps = -4;
    settings.slowApproachSteps = -2;
    settings.slowRetreatSteps = 2;
    settings.fastRetreatSteps = 20;
    settings.maxTravelSteps = 600;
    settings.idlePressureMin = -7500;
    settings.idlePressureMax = 7500;
    settings.contactPressureDelta = 10;
    settings.retryPressureDelta = 5;
    settings.idleDelayMs = 250;
    settings.fastStepDelayMs = 1;
    settings.slowStepDelayMs = 100;
    settings.pressureReadCount = 3;
    settings.pressureReadDelayMs = 15;
    settings.pressureTolerance = 15;
    settings.x.startSteps = 100;
    settings.y.startSteps = 100;
    settings.x.strideSteps = 100;
    settings.y.strideSteps = 100;
    settings.x.maxPositionSteps = 300;
    settings.y.maxPositionSteps = 300;
    return settings;
}();
short g_ZCompensationMatrix[COMPENSATION_MATRIX_MAX_X][COMPENSATION_MATRIX_MAX_Y];
uint8_t g_uZMatrixMax[2] = {};

bool injectNoise = false, injectDisconnection = false, injectOverload = false, invalidGrid = false;
bool matrixPrepared = false, matrixSaved = false, saveFailed = false;
unsigned keepAlives = 0, homedXY = 0, homedZ = 0, pressureReads = 0, negativeMoves = 0,
         disabled = 0, loaded = 0, thermalOffsets = 0, completed = 0, warmupMessages = 0, errors = 0;
ScanAbort aborted = ScanAbort::None;
uint32_t lastMoveAt = 0, acceptedAt = 0;
bool moved = false;
int bedTarget = 0;
std::vector<int> nozzleTargets;

bool injectPauseAtAdmission = false;
struct InterruptProtectedBlock { InterruptProtectedBlock(); };

namespace HAL {
millis_t now = 0;
millis_t timeInMilliseconds() { return now; }
}

namespace Printer {
ScanStrainGauge strainGauge;
float axisStepsPerMM[3] = {100, 100, 100}, homingFeedrate[3] = {60, 60, 5};
float stepsPerMm(uint8_t axis) { return axisStepsPerMM[axis]; }
float axisHomingFeedrate(uint8_t axis) { return homingFeedrate[axis]; }
struct { bool busy=false, blocked=false; bool isIdle() const { return !busy && !blocked; } bool isBusy() const { return busy; } bool commandsBlocked() const { return blocked; } } printJob;
struct { bool paused=false, servicing=false; bool isPaused() const { return paused; } bool blocksCalibration() const { return paused || servicing; } } pause;
struct { void recordActivity(millis_t) {} } inactivity;
struct { void cancel() {} void schedule(millis_t) {} } idleStatus;
struct {
    long position = 0;
    long scanSteps() const { return position; }
} zMotion;
void disableCMPnow(bool wait) { assert(!wait); }
void homeAxis(bool x, bool y, bool z) {
    assert(x || y || z);
    if (y)
        ++homedXY;
    if (z) {
        ++homedZ;
        zMotion.position = 0;
    }
}
void disableAllSteppersNow() { ++disabled; }
bool queueRelativeStepsCoordinates(long, long, long z, long e, float, bool, bool endstop) {
    assert(!z && !e && endstop);
    return true;
}
bool queueRelativeMMCoordinates(float x, float y, float z, float e, float, bool wait, bool endstop) {
    assert((x == heatBedScanCalibrationPointMm || !x) && y == heatBedScanCalibrationPointMm);
    assert(!z && !e && wait && endstop);
    return true;
}
}

namespace Commands {
enum class MotionWait { QueueAndZ, AllOutputs };
enum class PauseWait { Resume, Cancel };
bool waitForMotion(MotionWait motion, PauseWait pause) {
    assert(motion == MotionWait::QueueAndZ && pause == PauseWait::Resume);
    return true;
}
}
InterruptProtectedBlock::InterruptProtectedBlock() {
    // Simulate Timer1 publishing a pause just before the scan takes its
    // admission lock, after the optimistic initial phase check.
    if (injectPauseAtAdmission) {
        injectPauseAtAdmission = false;
        Printer::pause.paused = true;
    }
}

namespace Extruder {
bool calibrated = true;
bool allHeatersCalibrated() { return calibrated; }
bool selectExtruderById(unsigned int tool) { return tool == 0; }
void setHeatedBedTemperature(int target) { bedTarget = target; }
void setTemperatureForAllExtruders(int target) { nozzleTargets.push_back(target); }
}
namespace Com {
const char* tPrintingIsInProcessError = "Busy";
template<typename... T> void printF(T...) {}
template<typename... T> void printFLN(T...) {}
}
enum { Calibrating };
namespace GCodeInput {
void keepAlive(int state) { assert(state == Calibrating); ++keepAlives; }
}
namespace RMath { using std::min; using std::max; }

void showError(const void*, const void*) { ++errors; }
void showAbortScanReason(const void*, ScanAbort reason) { aborted = reason; }
void showInformation(const void*, const void*, const void*) { ++completed; }
void initCompensationMatrix() {
    std::memset(g_ZCompensationMatrix, 0, sizeof(g_ZCompensationMatrix));
    g_uZMatrixMax[X_AXIS] = g_uZMatrixMax[Y_AXIS] = 0;
    g_ZMatrixChangedInRam = false;
}
void outputCompensationMatrix(char) {}
char prepareCompensationMatrix() {
    matrixPrepared = !invalidGrid;
    return invalidGrid ? -1 : 0;
}
void determineCompensationOffsetZ() {}
void adjustCompensationMatrix(short steps) { assert(steps == -123); ++thermalOffsets; }
int loadCompensationMatrix(unsigned int address) {
    assert(address == static_cast<unsigned int>(EEPROM_SECTOR_SIZE * g_nActiveHeatBed));
    ++loaded;
    return 0;
}
bool saveWorkingMatrix(unsigned int address) {
    assert(matrixPrepared && address == static_cast<unsigned int>(EEPROM_SECTOR_SIZE * g_nActiveHeatBed));
    if (saveFailed)
        return false;
    matrixSaved = true;
    g_ZMatrixChangedInRam = false;
    return true;
}
int testHeatBedTemperature() { return 0; }
int testExtruderTemperature() { return 0; }
bool moveZ(long distance) {
    if (distance < 0) {
        assert(!Printer::strainGauge.disconnected && (!injectOverload || !negativeMoves));
        assert(!moved || static_cast<int32_t>(acceptedAt - lastMoveAt) > 0);
        ++negativeMoves;
        if (injectDisconnection)
            Printer::strainGauge.disconnected = true;
    } else if (Printer::strainGauge.disconnected || (injectOverload && negativeMoves)) {
        assert(bedTarget == 0 && !nozzleTargets.empty() && nozzleTargets.back() == 0);
    }
    lastMoveAt = HAL::now;
    moved = true;
    Printer::zMotion.position += distance;
    return true;
}
void updateDigitCompensation(int16_t) {
    ++pressureReads;
    acceptedAt = Printer::strainGauge.latestStart();
}
int16_t probeDigits() {
    if (injectOverload && negativeMoves)
        return 10000;
    if (injectNoise && pressureReads >= 6 && pressureReads < 21)
        return pressureReads % 2 ? 700 : 500;
    return Printer::zMotion.scanSteps() <= -124 ? 600 : 500;
}
void addLong(char* text, long value, int) {
    ++warmupMessages;
    std::sprintf(text + std::strlen(text), "%ld", value);
}
} // namespace

#include "scan-pressure-firmware.inc"
#include "bed-matrix-scan-firmware.inc"

const ScanSettings& Calibration::probeSettings() { return scanSettings; }
bool Calibration::moveZ(int steps, bool) { return ::moveZ(steps); }
void Calibration::showAbortScanReason(const void* text, ScanAbort reason) { ::showAbortScanReason(text, reason); }

#include "heat-bed-scan-firmware.inc"

int main(int argc, char** argv) {
    assert(argc == 2);
    if (!std::strcmp(argv[1], "busy-admission")) {
        Printer::printJob.busy=true;
        heatBedScan.startOrCancel(HeatBedScanMode::Standard);
        assert(!heatBedScan.active() && !homedZ && !homedXY);
        Printer::printJob.busy=false;
        Printer::printJob.blocked=true;
        heatBedScan.startOrCancel(HeatBedScanMode::Standard);
        assert(!heatBedScan.active() && !homedZ && !homedXY);
        Printer::printJob.blocked=false;
        Printer::pause.paused=true;
        heatBedScan.startOrCancel(HeatBedScanMode::Standard);
        assert(!heatBedScan.active() && !homedZ && !homedXY);
        Printer::pause.paused=false;
        Printer::pause.servicing=true;
        heatBedScan.startOrCancel(HeatBedScanMode::Standard);
        assert(!heatBedScan.active() && !homedZ && !homedXY);
        Printer::pause.servicing=false;
        injectPauseAtAdmission=true;
        heatBedScan.startOrCancel(HeatBedScanMode::Standard);
        assert(!injectPauseAtAdmission && Printer::pause.paused);
        assert(!heatBedScan.active() && !homedZ && !homedXY);
        Printer::pause.paused=false;
        heatBedScan.startOrCancel(HeatBedScanMode::Standard);
        assert(heatBedScan.active());
        Printer::printJob.busy=true;
        heatBedScan.startOrCancel(HeatBedScanMode::Standard);
        assert(Calibration::scanSession.aborting()); // cancel remains allowed
        return 0;
    }
    HeatBedScanMode mode = HeatBedScanMode::Standard;
    if (!std::strcmp(argv[1], "pla"))
        mode = HeatBedScanMode::Pla;
    if (!std::strcmp(argv[1], "abs"))
        mode = HeatBedScanMode::Abs;
    const bool retry = !std::strcmp(argv[1], "retry");
    const bool cancel = !std::strcmp(argv[1], "cancel");
    const bool cancelOnOverload = !std::strcmp(argv[1], "overload-and-cancel");
    const bool stop = !std::strcmp(argv[1], "stop-mid-grid");
    injectNoise = retry;
    injectDisconnection = !std::strcmp(argv[1], "disconnected");
    injectOverload = cancelOnOverload || !std::strcmp(argv[1], "overload");
    invalidGrid = !std::strcmp(argv[1], "invalid-grid");
    saveFailed = !std::strcmp(argv[1], "save-failed");

    Extruder::calibrated = false;
    heatBedScan.startOrCancel(HeatBedScanMode::Pla);
    assert(!heatBedScan.active());
    heatBedScan.startOrCancel(HeatBedScanMode::Abs);
    assert(!heatBedScan.active());
    Extruder::calibrated = true;
    heatBedScan.startOrCancel(mode);
    unsigned ticks = 0;
    while (heatBedScan.active()) {
        assert(++ticks <= 5000);
        Printer::strainGauge.service(HAL::now, probeDigits());
        if (!Calibration::scanSession.aborting())
            Calibration::scanPressure.service(HAL::now);
        if (cancelOnOverload && Calibration::scanSession.aborting())
            heatBedScan.startOrCancel(HeatBedScanMode::Abs);
        heatBedScan.tick();
        if (cancel && ticks == 3)
            heatBedScan.startOrCancel(HeatBedScanMode::Abs);
        if (stop && negativeMoves) {
            assert(g_ZCompensationMatrix[0][0] == EEPROM_FORMAT);
            heatBedScan.discard();
        }
        HAL::now += 200000;
    }
    if (injectDisconnection || injectOverload) {
        assert(aborted == (injectOverload ? ScanAbort::DmsOverload : ScanAbort::DmsUnavailable) && negativeMoves == 1);
        assert(homedZ == 1 && disabled == 1 && loaded == 1 && !matrixSaved);
    } else if (invalidGrid) {
        assert(aborted == ScanAbort::MatrixDimensions && loaded == 1 && disabled == 1);
        assert(!matrixPrepared && !matrixSaved && !completed);
    } else if (saveFailed) {
        assert(aborted == ScanAbort::None && errors == 1 && disabled == 1);
        assert(matrixPrepared && !matrixSaved && !completed && !loaded && g_ZMatrixChangedInRam);
    } else if (stop) {
        assert(aborted == ScanAbort::None && !g_ZCompensationMatrix[0][0] && !matrixSaved);
    } else if (cancel) {
        assert(aborted == ScanAbort::Canceled && loaded == 1 && disabled == 1);
        assert(!matrixSaved && !completed && ticks == 4 && keepAlives == 3);
    } else {
        assert(aborted == ScanAbort::None && !errors && !loaded && matrixSaved && completed == 1 && disabled == 1);
        assert(g_uZMatrixMax[X_AXIS] == 4 && g_uZMatrixMax[Y_AXIS] == 4);
        for (int x = 2; x <= 4; ++x) {
            assert(g_ZCompensationMatrix[x][0] == x - 1);
            for (int y = 2; y <= 4; ++y)
                assert(g_ZCompensationMatrix[x][y] == -123);
        }
        assert(keepAlives == ticks && bedTarget == 0 && negativeMoves > 9);
        if (mode != HeatBedScanMode::Standard) {
            assert(nozzleTargets == std::vector<int>({100, mode == HeatBedScanMode::Pla ? 210 : 240, 0}));
            assert(thermalOffsets == 1 && warmupMessages == 4 && homedXY == 3 && homedZ == 3);
        } else {
            assert(nozzleTargets == std::vector<int>({0, 0}));
            assert(!thermalOffsets && !warmupMessages && homedXY == (retry ? 3u : 2u) && homedZ == homedXY);
        }
    }
    std::printf("Heat bed scan replay passed (%s, %u ticks)\n", argv[1], ticks);
}

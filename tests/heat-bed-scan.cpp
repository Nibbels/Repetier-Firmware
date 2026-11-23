#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include "PressureMeasurement.h"

namespace {
using millis_t = uint32_t;
enum { X_AXIS, Y_AXIS, Z_AXIS };
enum { EEPROM_FORMAT = 42, COMPENSATION_MATRIX_MAX_X = 8, COMPENSATION_MATRIX_MAX_Y = 8 };
enum { HEAT_BED_SCAN_MODE_PLA = 1, HEAT_BED_SCAN_MODE_ABS = 2, EEPROM_SECTOR_SIZE = 1536 };
enum { SCAN_ABORT_REASON_CANCELED = 1, SCAN_ABORT_REASON_AVERAGE_PRESSURE, SCAN_ABORT_REASON_IDLE_PRESSURE,
       SCAN_ABORT_REASON_MATRIX_DIMENSION, SCAN_ABORT_REASON_DMS_UNAVAILABLE, SCAN_ABORT_REASON_DMS_OVERLOAD };
enum {
    SCAN_ABORT_REASON_REACHED_MAX_COMPENSATION = 30, SCAN_ABORT_REASON_SCAN_OVER_START,
    SCAN_ABORT_REASON_SCAN_TOO_UNEVEN, SCAN_ABORT_REASON_SCAN_STEP_DELTA_OUT_OF_RANGE
};
template<class T>
T constrain(T value, T minimum, T maximum) { return std::min(std::max(value, minimum), maximum); }

constexpr int heatBedScanDelayMs = 1000, heatBedScanRetries = 5, heatBedScanCalibrationPointMm = 100;
constexpr int heatBedScanPlaBedC = 60, heatBedScanPlaNozzleC = 210, heatBedScanAbsBedC = 100,
              heatBedScanAbsNozzleC = 240, heatBedScanProbeNozzleC = 100;
constexpr millis_t preciseBedScanWarmupSeconds = 600, preciseBedScanCalibrationSeconds = 600;
#define PSTR(text) text
#define UI_STATUS(text) ((void)(text))
#define UI_STATUS_UPD(text) ((void)(text))
#define UI_STATUS_UPD_RAM(text) ((void)(text))
constexpr char UI_TEXT_HEATING[] = "Heating", UI_TEXT_HEAT_BED_SCAN[] = "Scan",
               UI_TEXT_HEAT_BED_SCAN_DONE[] = "Done", UI_TEXT_OK[] = "OK";
constexpr char ui_text_heat_bed_scan[] = "Scan", ui_text_operation_denied[] = "Busy", ui_text_saving_success[] = "Saved";

char g_abortZScan = 0, g_retryZScan = 0, g_scanRetries = 0, g_nHeatBedScanMode = 0, g_nActiveHeatBed = 2;
short g_nScanHeatBedUpFastSteps = -4, g_nScanHeatBedUpSlowSteps = -2, g_nScanHeatBedDownSlowSteps = 2;
short g_nScanHeatBedDownFastSteps = 20, g_nScanZMaxCompensationSteps = 600;
short g_nScanIdlePressureMin = -7500, g_nScanIdlePressureMax = 7500;
short g_nMinPressureContact = 0, g_nMaxPressureContact = 0, g_nMinPressureRetry = 0, g_nMaxPressureRetry = 0;
short g_nFirstIdlePressure = 500, g_nCurrentIdlePressure = 500;
long g_nLastZScanZPosition = 0;
short g_ZCompensationMatrix[COMPENSATION_MATRIX_MAX_X][COMPENSATION_MATRIX_MAX_Y];
uint8_t g_uZMatrixMax[2] = {};
float g_scanStartZLiftMM = 1.0f;
uint16_t g_nScanIdleDelay = 250, g_nScanFastStepDelay = 1, g_nScanSlowStepDelay = 100;
uint8_t g_nScanPressureReads = 3;
uint16_t g_nScanPressureReadDelay = 15, g_nScanPressureTolerance = 15;
long g_nScanXStartSteps = 100, g_nScanYStartSteps = 100, g_nScanXStepSizeSteps = 100, g_nScanYStepSizeSteps = 100;
long g_nScanXMaxPositionSteps = 300, g_nScanYMaxPositionSteps = 300;

bool injectNoise = false, injectDisconnection = false, injectOverload = false, disconnected = false, matrixPrepared = false, matrixSaved = false;
unsigned keepAlives = 0, homedXY = 0, homedZ = 0, pressureReads = 0, negativeMoves = 0,
         disabled = 0, loaded = 0, thermalOffsets = 0, completed = 0, warmupMessages = 0;
char aborted = 0;
int bedTarget = 0;
std::vector<int> nozzleTargets;

namespace HAL {
millis_t now = 0;
millis_t timeInMilliseconds() { return now; }
}

namespace Printer {
float axisStepsPerMM[3] = {100, 100, 100}, homingFeedrate[3] = {60, 60, 5};
struct { bool isBusy() const { return false; } } printJob;
struct { void recordActivity(millis_t) {} } inactivity;
struct { void cancel() {} void schedule(millis_t) {} } idleStatus;
struct {
    long position = 0;
    long scanSteps() const { return position; }
} zMotion;
bool debugErrors() { return false; }
bool debugInfo() { return false; }
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
namespace Extruder {
bool selectExtruderById(unsigned int tool) { return tool == 0; }
void setHeatedBedTemperature(int target) { bedTarget = target; }
void setTemperatureForAllExtruders(int target) { nozzleTargets.push_back(target); }
}
namespace Com {
const char* tPrintingIsInProcessError = "Busy";
const char* tscanHeatBed = "Scan";
template<typename... T> void printF(T...) {}
template<typename... T> void printFLN(T...) {}
}
enum { Calibrating };
namespace GCodeInput {
void keepAlive(int state) { assert(state == Calibrating); ++keepAlives; }
}
namespace RMath { using std::min; using std::max; }

void showError(void*, void*) { assert(false); }
void showAbortScanReason(const void*, char reason) { aborted = reason; }
void showInformation(const void*, const void*, const void*) { ++completed; }
void initCompensationMatrix() {
    std::memset(g_ZCompensationMatrix, 0, sizeof(g_ZCompensationMatrix));
    g_uZMatrixMax[X_AXIS] = g_uZMatrixMax[Y_AXIS] = 0;
}
void outputScanParameters() {}
void outputCompensationMatrix() {}
void prepareCompensationMatrix() { matrixPrepared = true; }
void determineCompensationOffsetZ() {}
void adjustCompensationMatrix(short steps) { assert(steps == -123); ++thermalOffsets; }
int loadCompensationMatrix(unsigned int address) {
    assert(address == static_cast<unsigned int>(EEPROM_SECTOR_SIZE * g_nActiveHeatBed));
    ++loaded;
    return 0;
}
void saveCompensationMatrix(unsigned int address) {
    assert(matrixPrepared && address == static_cast<unsigned int>(EEPROM_SECTOR_SIZE * g_nActiveHeatBed));
    matrixSaved = true;
}
int testHeatBedTemperature() { return 0; }
int testExtruderTemperature() { return 0; }
bool moveZ(long distance) {
    if (distance < 0) {
        assert(!disconnected && (!injectOverload || !negativeMoves));
        ++negativeMoves;
        if (injectDisconnection)
            disconnected = true;
    } else if (disconnected || (injectOverload && negativeMoves)) {
        assert(bedTarget == 0 && !nozzleTargets.empty() && nozzleTargets.back() == 0);
    }
    Printer::zMotion.position += distance;
    return true;
}
void adjustPressureLimits(short baseline) {
    g_nMinPressureContact = baseline - 10;
    g_nMaxPressureContact = baseline + 10;
    g_nMinPressureRetry = baseline - 5;
    g_nMaxPressureRetry = baseline + 5;
}
bool sampleStrainGauge(int16_t& digits) {
    ++pressureReads;
    if (disconnected)
        return false;
    if (injectOverload && negativeMoves) {
        digits = 10000;
        return true;
    }
    if (injectNoise && pressureReads > 6 && pressureReads <= 21) {
        digits = pressureReads % 2 ? 500 : 700;
        return true;
    }
    digits = Printer::zMotion.scanSteps() <= -124 ? 600 : 500;
    return true;
}
void addLong(char* text, long value, int) {
    ++warmupMessages;
    std::sprintf(text + std::strlen(text), "%ld", value);
}
} // namespace

#include "scan-pressure-firmware.inc"
#include "heat-bed-scan-firmware.inc"

int main(int argc, char** argv) {
    assert(argc == 2);
    if (!std::strcmp(argv[1], "pla"))
        g_nHeatBedScanMode = HEAT_BED_SCAN_MODE_PLA;
    if (!std::strcmp(argv[1], "abs"))
        g_nHeatBedScanMode = HEAT_BED_SCAN_MODE_ABS;
    const bool retry = !std::strcmp(argv[1], "retry");
    const bool cancel = !std::strcmp(argv[1], "cancel");
    injectNoise = retry;
    injectDisconnection = !std::strcmp(argv[1], "disconnected");
    injectOverload = !std::strcmp(argv[1], "overload");

    heatBedScan.startOrCancel();
    unsigned ticks = 0;
    while (heatBedScan.active()) {
        assert(++ticks <= 5000);
        if (!g_abortZScan)
            scanPressure.service(HAL::now);
        heatBedScan.tick();
        if (cancel && ticks == 3)
            heatBedScan.startOrCancel();
        HAL::now += 200000;
    }
    if (injectDisconnection || injectOverload) {
        assert(aborted == (injectOverload ? SCAN_ABORT_REASON_DMS_OVERLOAD : SCAN_ABORT_REASON_DMS_UNAVAILABLE) && negativeMoves == 1);
        assert(homedZ == 1 && disabled == 1 && loaded == 1 && !matrixSaved);
    } else if (cancel) {
        assert(aborted == SCAN_ABORT_REASON_CANCELED && loaded == 1 && disabled == 1);
        assert(!matrixSaved && !completed && ticks == 4 && keepAlives == 3);
    } else {
        assert(!aborted && !loaded && matrixSaved && completed == 1 && disabled == 1);
        assert(g_uZMatrixMax[X_AXIS] == 4 && g_uZMatrixMax[Y_AXIS] == 4);
        for (int x = 2; x <= 4; ++x) {
            assert(g_ZCompensationMatrix[x][0] == x - 1);
            for (int y = 2; y <= 4; ++y)
                assert(g_ZCompensationMatrix[x][y] == -123);
        }
        assert(keepAlives == ticks && bedTarget == 0 && negativeMoves > 9);
        if (g_nHeatBedScanMode) {
            assert(nozzleTargets == std::vector<int>({100, g_nHeatBedScanMode == HEAT_BED_SCAN_MODE_PLA ? 210 : 240, 0}));
            assert(thermalOffsets == 1 && warmupMessages == 4 && homedXY == 3 && homedZ == 3);
        } else {
            assert(nozzleTargets == std::vector<int>({0, 0}));
            assert(!thermalOffsets && !warmupMessages && homedXY == (retry ? 3u : 2u) && homedZ == homedXY);
        }
    }
    std::printf("Heat bed scan replay passed (%s, %u ticks)\n", argv[1], ticks);
}

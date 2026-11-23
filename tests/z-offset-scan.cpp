#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <utility>
#include <vector>
#include "CalibrationScan.h"
#include "PressureMeasurement.h"
#include "scan-strain-gauge.h"

namespace {
using millis_t = uint32_t;
enum { X_AXIS, Y_AXIS, Z_AXIS };
enum { EEPROM_FORMAT = 42, COMPENSATION_MATRIX_MAX_X = 10, COMPENSATION_MATRIX_MAX_Y = 10 };
enum { EEPROM_SECTOR_SIZE = 4096, EEPROM_OFFSET_DIMENSION_X = 2, EEPROM_OFFSET_DIMENSION_Y = 4 };
template<class T>
T constrain(T value, T minimum, T maximum) { return std::min(std::max(value, minimum), maximum); }

constexpr int heatBedScanRetries = 5;
constexpr uint8_t I2C_ADDRESS_EXTERNAL_EEPROM = 0x50;
constexpr int heatBedScanXStartMm = 15, heatBedScanYStartMm = 30, heatBedScanDelayMs = 1000;
constexpr int zOffsetScanContactPressureDelta = 33, zOffsetScanRetryPressureDelta = 30;
#define PSTR(text) text
#define UI_STATUS_UPD(text) ((void)(text))
constexpr char UI_TEXT_ZCALIB[] = "z calibrate", UI_TEXT_DO_MHIER_BED_SCAN[] = "scan";
constexpr char ui_text_heat_bed_zoffset_search_status[] = "status", ui_text_heat_bed_zoffset_fix_z1[] = "z1",
               ui_text_heat_bed_zoffset_fix_z2[] = "z2", ui_text_statusmsg[] = "message";

char g_nActiveHeatBed = 1;
short g_nScanHeatBedUpFastSteps = -4, g_nScanHeatBedUpSlowSteps = -2;
short g_nScanHeatBedDownSlowSteps = 2, g_nScanHeatBedDownFastSteps = 20, g_nScanZMaxCompensationSteps = 600;
short g_nScanIdlePressureMin = -7500, g_nScanIdlePressureMax = 7500;
uint8_t g_nScanPressureReads = 2;
uint16_t g_nScanPressureReadDelay = 15, g_nScanPressureTolerance = 15;
uint16_t g_nScanSlowStepDelay = 100, g_nScanFastStepDelay = 1;
float g_scanStartZLiftMM = 1.0f;
float g_ZOSlearningRate = 1.0f, g_ZOSlearningGradient = 0.0f;
uint8_t g_ZOSTestPoint[2] = {3, 4}, g_uZMatrixMax[2] = {9, 9}, g_ZMatrixChangedInRam = 0;
short g_ZCompensationMatrix[COMPENSATION_MATRIX_MAX_X][COMPENSATION_MATRIX_MAX_Y];
bool g_nAutoReturnMessage = false;
uint32_t g_nAutoReturnTime = 0;

unsigned homedZ = 0, homedXY = 0, reported = 0, positiveRetryRetreats = 0, loaded = 0, shutdowns = 0, serviced = 0;
unsigned corrected = 0, pressureSamples = 0, tickCount = 0, negativeMoves = 0;
bool injectNoise = false, injectDisconnection = false, injectOverload = false;
uint32_t lastUnstableAt = 0;
uint32_t lastMoveAt = 0, acceptedAt = 0;
bool moved = false, returningToZero = false;
ScanAbort abortedWith = ScanAbort::None;
bool matrixAvailable = true;
void (*onHome)() = nullptr;
std::vector<std::pair<float, float>> probePoints;

namespace HAL {
uint32_t now = 0;
uint32_t timeInMilliseconds() { return now; }
}

namespace Printer {
ScanStrainGauge strainGauge;
bool blockAll = false;
struct { bool commandsBlocked() const { return false; } } printJob;
struct { bool isPaused() const { return false; } } pause;
struct { void cancel() {} void schedule(uint32_t) {} } idleStatus;
struct { void recordActivity(uint32_t) {} } inactivity;
struct {
    long position = 0;
    long scanSteps() const { return position; }
} zMotion;
float axisStepsPerMM[3] = {160, 160, 100};
float axisMMPerSteps[3] = {1.0f / 160, 1.0f / 160, 1.0f / 100};
float homingFeedrate[3] = {60, 60, 5};
void homeAxis(bool x, bool y, bool z) {
    assert(y);
    if (onHome)
        onHome();
    if (z) {
        assert(x);
        ++homedZ;
        zMotion.position = 0;
    } else {
        ++homedXY;
        returningToZero = true;
    }
}
bool queueRelativeMMCoordinates(float x, float y, float z, float e, float, bool wait, bool endstop) {
    assert(wait && endstop && !e);
    if (z)
        assert(!x && !y && z == 5);
    else
        probePoints.emplace_back(x, y);
    return true;
}
void disableAllSteppersNow() { ++shutdowns; }
}

constexpr int Calibrating = 1;
namespace GCodeInput {
void keepAlive(int state) {
    assert(state == Calibrating);
    ++serviced;
}
}
namespace RMath {
using std::min;
using std::max;
}
namespace Com {
const char* tAutoMatrixLeveling = "Auto";
template<class... Args> void printF(Args...) {}
template<class... Args> void printFLN(Args...) {}
}

void showAbortScanReason(const char*, ScanAbort reason) { abortedWith = reason; }
void showMyPage(void*, void*, void*, void*) {}
void initCompensationMatrix() { g_ZCompensationMatrix[0][0] = 0; }
int loadCompensationMatrix(unsigned int address) {
    assert(address == EEPROM_SECTOR_SIZE);
    ++loaded;
    if (!matrixAvailable)
        return 1;
    g_ZCompensationMatrix[0][0] = EEPROM_FORMAT;
    for (unsigned x = 1; x <= g_uZMatrixMax[X_AXIS]; ++x) {
        g_ZCompensationMatrix[x][0] = x * 10;
        for (unsigned y = 1; y <= g_uZMatrixMax[Y_AXIS]; ++y) {
            g_ZCompensationMatrix[0][y] = y * 10;
            g_ZCompensationMatrix[x][y] = -100;
        }
    }
    return 0;
}
uint16_t readWord24C256(int address, unsigned int offset) {
    assert(address == I2C_ADDRESS_EXTERNAL_EEPROM);
    assert(offset == EEPROM_SECTOR_SIZE + EEPROM_OFFSET_DIMENSION_X || offset == EEPROM_SECTOR_SIZE + EEPROM_OFFSET_DIMENSION_Y);
    return 9;
}
long random(long min, long max) {
    assert(min == -3 && max == 3);
    return 0;
}
bool moveZ(long steps) {
    if (steps < 0 && !returningToZero) {
        assert(!Printer::strainGauge.disconnected && (!injectOverload || !negativeMoves));
        assert(!moved || static_cast<int32_t>(acceptedAt - lastMoveAt) > 0);
        ++negativeMoves;
        if (injectDisconnection)
            Printer::strainGauge.disconnected = true;
    } else if (!returningToZero && injectNoise && steps == g_nScanHeatBedDownFastSteps) {
        assert(HAL::now - lastUnstableAt >= heatBedScanDelayMs);
        ++positiveRetryRetreats;
    }
    returningToZero = false;
    lastMoveAt = HAL::now;
    moved = true;
    Printer::zMotion.position += steps;
    return true;
}
void updateDigitCompensation(int16_t) {
    acceptedAt = Printer::strainGauge.latestStart();
    if (++pressureSamples <= 10 && injectNoise)
        lastUnstableAt = HAL::now;
}
int16_t probeDigits() {
    if (injectOverload && negativeMoves)
        return -10000;
    if (injectNoise && pressureSamples < 10)
        return Printer::strainGauge.conversionsStarted() % 2 ? 700 : 500;
    return Printer::zMotion.scanSteps() <= -124 ? 600 : 500;
}
long getZMatrixDepth_CurrentXY() {
    return g_ZCompensationMatrix[g_ZOSTestPoint[X_AXIS]][g_ZOSTestPoint[Y_AXIS]];
}
void determineCompensationOffsetZ() { ++corrected; }
void outputCompensationMatrix(int mode) { assert(mode == 1); }
void calculateZScrewCorrection() {
    assert(Printer::zMotion.scanSteps() == 0);
    ++reported;
}
} // namespace

#include "scan-pressure-firmware.inc"
#include "z-offset-scan-firmware.inc"

namespace {
struct {
    bool active() const { return false; }
    void tick() { assert(false); }
    void discard() {}
} heatBedScan;
bool isAnyScanRunning() { return zOffsetScan.active(); }
}

#include "scan-dispatch-firmware.inc"

int main(int argc, char** argv) {
    assert(argc == 2);
    const bool level = !std::strcmp(argv[1], "level");
    const bool retry = !std::strcmp(argv[1], "retry");
    const bool missing = !std::strcmp(argv[1], "missing-matrix");
    const bool cancel = !std::strcmp(argv[1], "cancel-during-home");
    const bool stop = !std::strcmp(argv[1], "stop-during-home");
    const bool tinyMatrix = !std::strcmp(argv[1], "tiny-matrix");
    if (tinyMatrix)
        g_uZMatrixMax[X_AXIS] = g_uZMatrixMax[Y_AXIS] = 2;
    if (cancel) {
        onHome = [] {
            onHome = nullptr;
            zOffsetScan.startOrCancel(false);
            assert(zOffsetScan.active() && abortedWith == ScanAbort::None);
        };
    }
    if (stop) {
        onHome = [] {
            onHome = nullptr;
            Printer::blockAll = true;
            discardScans();
            assert(zOffsetScan.active());
        };
    }
    matrixAvailable = !missing;
    injectNoise = retry;
    injectDisconnection = !std::strcmp(argv[1], "disconnected");
    injectOverload = !std::strcmp(argv[1], "overload");
    HAL::now = retry ? UINT32_MAX - 900 : 0;
    zOffsetScan.startOrCancel(level);
    while (zOffsetScan.active()) {
        assert(++tickCount <= 15000);
        Printer::strainGauge.service(HAL::now, probeDigits());
        handleScanWorkTasks(true);
        assert(serviced == tickCount);
        HAL::now += 10;
    }
    if (injectDisconnection || injectOverload) {
        assert(abortedWith == (injectOverload ? ScanAbort::DmsOverload : ScanAbort::DmsUnavailable) && shutdowns == 1);
        assert(negativeMoves == 1 && homedZ == 1 && !corrected && !reported);
    } else if (cancel) {
        assert(abortedWith == ScanAbort::Canceled && tickCount == 3 && homedZ == 2);
        assert(shutdowns == 1 && probePoints.empty() && !corrected && !reported);
    } else if (stop) {
        assert(abortedWith == ScanAbort::None && tickCount == 2 && homedZ == 1);
        assert(probePoints.empty() && !negativeMoves && !corrected && !reported);
    } else if (missing) {
        assert(abortedWith == ScanAbort::MissingMatrix && shutdowns == 1);
        assert(tickCount == 4 && loaded == 1 && homedZ == 2 && !reported && !corrected);
    } else {
        assert(abortedWith == ScanAbort::None && reported == 1 && g_nAutoReturnMessage && g_ZMatrixChangedInRam);
        assert(homedZ == (level ? 7u : 1u) && corrected == homedZ && homedXY == homedZ);
        assert(loaded == 1 && negativeMoves > 0);
        const int checkIndex = tinyMatrix ? 2 : 4;
        assert(g_ZCompensationMatrix[checkIndex][checkIndex] == -123);
        assert(positiveRetryRetreats == static_cast<unsigned>(retry));
        const std::vector<std::pair<float, float>> expected = level
            ? std::vector<std::pair<float, float>>{{40, 40}, {20, 20}, {80, 80}, {40, 30}, {20, 80}, {80, 20}, {40, 50}}
            : tinyMatrix ? std::vector<std::pair<float, float>>{{10, 10}}
                         : std::vector<std::pair<float, float>>{{30, 40}};
        assert(probePoints == expected);
    }
    std::printf("Z-offset scan replay passed (%s)\n", argv[1]);
}

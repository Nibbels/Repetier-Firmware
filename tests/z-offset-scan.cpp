#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <utility>
#include <vector>

namespace {
using millis_t = uint32_t;
enum { X_AXIS, Y_AXIS, Z_AXIS };
enum { EEPROM_FORMAT = 42, COMPENSATION_MATRIX_MAX_X = 10, COMPENSATION_MATRIX_MAX_Y = 10 };
enum { EEPROM_SECTOR_SIZE = 4096, EEPROM_OFFSET_DIMENSION_X = 2, EEPROM_OFFSET_DIMENSION_Y = 4 };
enum { SCAN_ABORT_REASON_CANCELED = 1, SCAN_ABORT_REASON_MISSING_MATRIX, SCAN_ABORT_REASON_AVERAGE_PRESSURE,
       SCAN_ABORT_REASON_IDLE_PRESSURE, SCAN_ABORT_REASON_OVERFLOWING_MATRIX, SCAN_ABORT_REASON_BAD_HEIGHT_MATRIX };
constexpr uint8_t I2C_ADDRESS_EXTERNAL_EEPROM = 0x50;
constexpr int heatBedScanXStartMm = 15, heatBedScanYStartMm = 30, heatBedScanDelayMs = 1000;
constexpr int zOffsetScanContactPressureDelta = 33, zOffsetScanRetryPressureDelta = 30, zOffsetScanIdlePressureDelta = 0;
#define PSTR(text) text
#define UI_STATUS_UPD(text) ((void)(text))
constexpr char UI_TEXT_ZCALIB[] = "z calibrate", UI_TEXT_DO_MHIER_BED_SCAN[] = "scan";
constexpr char ui_text_heat_bed_zoffset_search_status[] = "status", ui_text_heat_bed_zoffset_fix_z1[] = "z1",
               ui_text_heat_bed_zoffset_fix_z2[] = "z2", ui_text_statusmsg[] = "message";

char g_retryZScan = 0, g_scanRetries = 0, g_abortZScan = 0, g_nActiveHeatBed = 1;
short g_nCurrentIdlePressure = 0, g_nMinPressureContact = 0, g_nMaxPressureContact = 0;
short g_nMinPressureRetry = 0, g_nMaxPressureRetry = 0, g_nMinPressureIdle = 0, g_nMaxPressureIdle = 0;
short g_nScanHeatBedUpFastSteps = -4, g_nScanHeatBedUpSlowSteps = -2;
uint16_t g_nScanSlowStepDelay = 100;
float g_scanStartZLiftMM = 1.0f;
float g_ZOSlearningRate = 1.0f, g_ZOSlearningGradient = 0.0f;
uint8_t g_ZOSTestPoint[2] = {3, 4}, g_uZMatrixMax[2] = {9, 9}, g_ZMatrixChangedInRam = 0;
short g_ZCompensationMatrix[COMPENSATION_MATRIX_MAX_X][COMPENSATION_MATRIX_MAX_Y];
long g_nLastZScanZPosition = 0;
bool g_nAutoReturnMessage = false;
uint32_t g_nAutoReturnTime = 0;

unsigned homedZ = 0, homedXY = 0, reported = 0, retreats = 0, loaded = 0, shutdowns = 0, serviced = 0;
unsigned corrected = 0, pressureSamples = 0, tickCount = 0, idleFailures = 0;
char abortedWith = 0;
bool matrixAvailable = true;
void (*onHome)() = nullptr;
std::vector<std::pair<float, float>> probePoints;

namespace HAL {
uint32_t now = 0;
uint32_t timeInMilliseconds() { return now; }
void delayMilliseconds(uint32_t delta) { now += delta; }
}

namespace Printer {
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
}
namespace Com {
const char* tAutoMatrixLeveling = "Auto";
template<class... Args> void printF(Args...) {}
template<class... Args> void printFLN(Args...) {}
}

void showAbortScanReason(const char*, char reason) { abortedWith = reason; }
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
void moveZ(long steps) { Printer::zMotion.position += steps; }
bool moveZPlusDownFast() { ++retreats; return false; }
void moveZMinusUpFast() { moveZ(-150); }
void moveZMinusUpSlow(short* digits, int) { *digits = 600; moveZ(-2); }
void moveZPlusDownSlow(int) { moveZ(2); }
bool readIdlePressure(short* digits) {
    if (idleFailures) {
        --idleFailures;
        return true;
    }
    *digits = 200;
    return false;
}
bool readAveragePressure(short* digits) {
    *digits = ++pressureSamples & 1 ? 600 : 200;
    return false;
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

#include "z-offset-scan-firmware.inc"

int main(int argc, char** argv) {
    assert(argc == 2);
    const bool level = !std::strcmp(argv[1], "level");
    const bool retry = !std::strcmp(argv[1], "retry");
    const bool missing = !std::strcmp(argv[1], "missing-matrix");
    const bool cancel = !std::strcmp(argv[1], "cancel-during-home");
    if (cancel) {
        onHome = [] {
            onHome = nullptr;
            zOffsetScan.startOrCancel(false);
            assert(zOffsetScan.active() && !abortedWith);
        };
    }
    matrixAvailable = !missing;
    idleFailures = retry ? 1 : 0;
    zOffsetScan.startOrCancel(level);
    while (zOffsetScan.active()) {
        assert(++tickCount <= 75);
        if (retry && tickCount == 7)
            HAL::now = UINT32_MAX - 500;
        if (retry && tickCount >= 8 && tickCount <= 11)
            HAL::now += 250;
        zOffsetScan.tick();
        assert(serviced == tickCount);
        if (retry && tickCount >= 7 && tickCount <= 11)
            assert(retreats == (tickCount == 11 ? 1u : 0u));
    }
    if (cancel) {
        assert(abortedWith == SCAN_ABORT_REASON_CANCELED && tickCount == 3 && homedZ == 2);
        assert(shutdowns == 1 && probePoints.empty() && !corrected && !reported);
    } else if (missing) {
        assert(abortedWith == SCAN_ABORT_REASON_MISSING_MATRIX && shutdowns == 1);
        assert(tickCount == 4 && loaded == 1 && homedZ == 2 && !reported && !corrected);
    } else {
        assert(!abortedWith && reported == 1 && g_nAutoReturnMessage && g_ZMatrixChangedInRam);
        assert(homedZ == (level ? 7u : 1u) && corrected == homedZ && homedXY == homedZ);
        assert(tickCount == (level ? 73u : retry ? 18u : 13u) && loaded == 1);
        assert(retreats == static_cast<unsigned>(retry));
        const std::vector<std::pair<float, float>> expected = level
            ? std::vector<std::pair<float, float>>{{40, 40}, {20, 20}, {80, 80}, {40, 30}, {20, 80}, {80, 20}, {40, 50}}
            : std::vector<std::pair<float, float>>{{30, 40}};
        assert(probePoints == expected);
    }
    std::printf("Z-offset scan replay passed (%s)\n", argv[1]);
}

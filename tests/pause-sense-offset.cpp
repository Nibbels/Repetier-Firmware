#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "Constants.h"
#undef uint
#define MOTHERBOARD DEVICE_TYPE_RF2000
#include "Configuration.h"
#include "ZOffset.h"

// The replay compiles the live firmware routines. These shims supply DMS bytes,
// a bed profile, a clock and the motor displacements completed during each wait.
using millis_t = uint32_t;
constexpr int NUM_EXTRUDER = 1;
constexpr float zStepsPerMm = 1280.0f;
constexpr int16_t initialClearance = 20;
constexpr int16_t printingForce = 6000;
constexpr int16_t senseThreshold = 3000;

millis_t clockMs = 0, g_uStartOfIdle = 0, previousMillisCmd = 0;
volatile millis_t g_uBlockCommands = 0;
volatile uint8_t g_startLineActive = 0;
char g_nHeatBedScanStatus = 0;
unsigned short g_nManualSteps[4] = { 1, 1, 1, 28 };
long g_minZCompensationSteps = 256, g_maxZCompensationSteps = 12800;
long g_offsetZCompensationSteps = 250;
short forceDigits = 0;
bool pollDuringMoves = true;

void requestPrintPause(bool moveAway);
void killPausePrint();
void poll();

namespace HAL {
millis_t timeInMilliseconds() { return clockMs; }
}

bool interruptsEnabled = true;
unsigned pauseBeforeLock = 0;

class InterruptProtectedBlock {
    const bool previousState = interruptsEnabled;

public:
    InterruptProtectedBlock() {
        if (interruptsEnabled && pauseBeforeLock && --pauseBeforeLock == 0)
            requestPrintPause(true);
        interruptsEnabled = false;
    }
    ~InterruptProtectedBlock() { unprotect(); }
    void unprotect() { interruptsEnabled = previousState; }
};

#define PSTR(text) (text)
#define UI_STATUS_UPD(text) ((void)0)
#define UI_TEXT_MANUAL ""
#define UI_TEXT_LCD_CIRCUIT ""
#define UI_TEXT_RESET ""
const char ui_text_pause[] = "", ui_text_operation_denied[] = "", ui_text_home_unknown[] = "";
const char ui_text_z_axis[] = "", ui_text_max_reached[] = "", ui_text_min_reached[] = "";
const char ui_text_emergency_pause[] = "";

struct Com {
    template<class... Args> static void printF(Args...) {}
    template<class... Args> static void printFLN(Args...) {}
};

template<class... Args> void showWarning(Args...) {}
template<class... Args> void showError(Args...) { assert(false); }
template<class... Args> void showInformation(Args...) { assert(false); }
void doEmergencyStop(int) { assert(false); }
bool isAnyScanRunning() { return false; }
void setExtruderCurrent(int, int) {}

struct WireInput {
    uint8_t byteIndex = 0;
    void beginTransmission(uint8_t address) const { assert(address == I2C_ADDRESS_STRAIN_GAUGE); }
    void requestFrom(uint8_t address, uint8_t length) {
        assert(address == I2C_ADDRESS_STRAIN_GAUGE && length == 3);
        byteIndex = 0;
    }
    void endTransmission() const { assert(byteIndex == 3); }
    int read() {
        const uint16_t digits = static_cast<uint16_t>(forceDigits);
        switch (byteIndex++) {
        case 0: return digits >> 8;
        case 1: return digits & 0xff;
        default: return 0;
        }
    }
} Wire;

namespace Printer {
ZOffset zOffset;
long currentSteps[3] = { 6400, 12800, 256 };
long directCurrentSteps[4] = { 0, 16, 8, 0 };
long currentZSteps = currentSteps[Z_AXIS] + directCurrentSteps[Z_AXIS];
long compensatedPositionTargetStepsZ = 0;
long maxSoftEndstopSteps[3] = { 100000, 100000, 200000 };
long maxZOverrideSteps = 1000;
long queuePositionZLayerLast = 0, queuePositionZLayerCurrent = 256;
float axisStepsPerMM[3] = { 160.0f, 160.0f, zStepsPerMm };
float axisMMPerSteps[3] = { 1.0f / 160, 1.0f / 160, 1.0f / zStepsPerMm };
float compensatedPositionOverPercE = 0.0f;
float dynamicFeedrateFactor = 1.0f, dynamicExtrusionFactor = 1.0f;
unsigned short interval_mod = 0;
bool blockAll = false, doHeatBedZCompensation = true, g_pressure_offset_active = false;
short g_pressure_offset = 0;
int feedrateMultiply = 100, motorCurrent[4] = {};
char moveMode[3] = { MOVE_MODE_SINGLE_STEPS, MOVE_MODE_SINGLE_STEPS, MOVE_MODE_SINGLE_STEPS };

bool debugErrors() { return false; }
bool isPrinting() { return true; }
bool areAxisHomed() { return true; }
bool isAxisHomed(int) { return true; }
bool isZMoveActive() { return false; }
bool isZMaxEndstopHit() { return currentZSteps >= maxSoftEndstopSteps[Z_AXIS]; }
bool isZMinEndstopHit() { return currentZSteps <= 0; }
void setMenuMode(int, bool) {}
void offsetRelativeStepsCoordinates(long x, long y, long z, long e, int task = TASK_NO_TASK);
}

namespace RMath {
long max(long a, long b) { return std::max(a, b); }
}

template<class T, class A, class B>
T constrain(T value, A low, B high) {
    return value < low ? low : value > high ? high : value;
}

struct Extruder {
    struct {
        float targetTemperatureC = 200.0f;
        uint8_t paused = 0;
        void waitForTargetTemperature(int, bool) { poll(); }
    } tempControl;
    int8_t stepperDirection = 0;

    static Extruder* current;
    static void setTemperatureForExtruder(float temperature, int tool);
};

Extruder extruder[NUM_EXTRUDER];
Extruder* Extruder::current = extruder;

void Extruder::setTemperatureForExtruder(float temperature, int tool) {
    extruder[tool].tempControl.targetTemperatureC = temperature;
    extruder[tool].tempControl.paused = 0;
}

struct PrintLine {
    static struct Direct { int task = TASK_NO_TASK; } direct;
    static int linesCount;
    static void stopDirectMove() { direct.task = TASK_NO_TASK; }
};

PrintLine::Direct PrintLine::direct;
int PrintLine::linesCount = 3;

struct {
    int lastButtonAction = 0, menuLevel = 1, menuPos[1] = {};
    void exitmenu() {}
    void initializeLCD() {}
} uid;

enum FirmwareState { Paused };
namespace Commands {
void checkForPeriodicalActions(FirmwareState);
void changeFeedrateMultiply(int) { assert(false); }
}

long getZMatrixDepth(long x, long y) { return 37 + x / 32 - y / 64; }

#include "pause-sense-offset-firmware.inc"

void poll() {
    clockMs += EMERGENCY_PAUSE_INTERVAL + 1;
    handleStrainGaugeFeatures(clockMs);
    recalculateHeatBedZCompensation();
}

void sample(short force, int count = 1) {
    forceDigits = force;
    for (int i = 0; i < count; ++i)
        poll();
}

void Commands::checkForPeriodicalActions(FirmwareState) { handlePauseStatus(); }

void Printer::offsetRelativeStepsCoordinates(long x, long y, long z, long e, int task) {
    // A short jog toward the print point travels one mm before key release.
    if (task == TASK_MOVE_FROM_BUTTON)
        z = std::max(z, -static_cast<long>(zStepsPerMm));
    if (x || y || z || e) {
        if (pollDuringMoves)
            poll();
        directCurrentSteps[X_AXIS] += x;
        directCurrentSteps[Y_AXIS] += y;
        directCurrentSteps[Z_AXIS] += z;
        directCurrentSteps[E_AXIS] += e;
        currentZSteps += z;
        g_nContinueExtruderSteps -= e;
    }
}

void parkAndResume() {
    sample(printingForce, 9);
    assert(Printer::zOffset.senseMicrometers() == initialClearance && g_nSensiblePressure1stMarke);
    const long firstTarget = Printer::compensatedPositionTargetStepsZ;
    const long originalY = Printer::directCurrentSteps[Y_AXIS];
    const long originalZ = Printer::directCurrentSteps[Z_AXIS];

    requestPrintPause(true);
    assert(g_pauseStatus == PAUSE_STATUS_GOTO_PAUSE_AND_MOVE);
    sample(0);
    assert(Printer::zOffset.senseMicrometers() == initialClearance && !g_nSensiblePressure1stMarke);
    handlePauseStatus();
    assert(isPauseSettled());
    assert(Printer::directCurrentSteps[Y_AXIS] == originalY + g_nPauseSteps[Y_AXIS]);
    assert(Printer::directCurrentSteps[Z_AXIS] == originalZ + g_nPauseSteps[Z_AXIS]);

    sample(0, 40);
    const long zeroForceTarget = Printer::compensatedPositionTargetStepsZ;
    const long printMatrix = getZMatrixDepth(Printer::currentSteps[X_AXIS], Printer::currentSteps[Y_AXIS]);
    assert(zeroForceTarget == printMatrix + Printer::zOffset.effectiveSteps());

    sample(printingForce, 40);
    assert(Printer::compensatedPositionTargetStepsZ > zeroForceTarget);
    assert(Printer::zOffset.senseMicrometers() == initialClearance);
    const long parkedZ = Printer::directCurrentSteps[Z_AXIS];
    moveZAction(-1);
    assert(Printer::directCurrentSteps[Z_AXIS] == parkedZ - static_cast<long>(zStepsPerMm));

    sample(EMERGENCY_STOP_DIGITS_MAX + 1, 2 * EMERGENCY_STOP_CHECKS);
    assert(g_nEmergencyESkip);
    sample(printingForce, 50);
    assert(!g_nEmergencyESkip && g_pauseStatus == PAUSE_STATUS_PAUSED);
    extrudeFromPanel();
    const long manualExtruderPosition = -g_nPauseSteps[E_AXIS] + g_nManualSteps[E_AXIS];
    assert(Printer::directCurrentSteps[E_AXIS] == manualExtruderPosition && !g_nContinueExtruderSteps);

    const auto& offset = Printer::zOffset;
    const long firstManual = offset.manualMicrometers();
    Printer::zOffset.adjustWithBedButton(Z_OFFSET_BUTTON_STEPS, 10000, zStepsPerMm);
    Printer::zOffset.adjustWithBedButton(-Z_OFFSET_BUTTON_STEPS, 10000, zStepsPerMm);
    assert(offset.manualMicrometers() == firstManual + Z_OFFSET_BUTTON_STEPS);
    assert(offset.senseMicrometers() == initialClearance - Z_OFFSET_BUTTON_STEPS);

    g_pauseStatus = PAUSE_STATUS_PAUSED_STANDBY;
    Extruder::current->tempControl.paused = 40;
    continuePrint();
    assert(g_pauseMode && isPauseSettled());
    continuePrint();
    assert(!g_pauseMode && g_pauseStatus == PAUSE_STATUS_NONE);
    assert(Printer::directCurrentSteps[Y_AXIS] == originalY && Printer::directCurrentSteps[Z_AXIS] == originalZ);
    assert(Printer::directCurrentSteps[E_AXIS] == manualExtruderPosition);
    assert(Printer::compensatedPositionTargetStepsZ == firstTarget);

    sample(printingForce, 9);
    assert(offset.senseMicrometers() == initialClearance - Z_OFFSET_BUTTON_STEPS);
    sample(printingForce);
    assert(offset.senseMicrometers() > initialClearance - Z_OFFSET_BUTTON_STEPS);
}

void pauseBetweenPolls() {
    sample(printingForce, 9);
    pollDuringMoves = false;
    requestPrintPause(false);
    handlePauseStatus();
    continuePrint();
    assert(!g_pauseMode);

    sample(printingForce, 9);
    assert(Printer::zOffset.senseMicrometers() == initialClearance);
    sample(printingForce);
    assert(Printer::zOffset.senseMicrometers() > initialClearance);
}

void interruptDuringLearning() {
    sample(printingForce, 9);
    // Let the poll capture its pause state and read Digit-Z. Enter a queued
    // pause just before the lock that would publish the SenseOffset update.
    pauseBeforeLock = 3;
    sample(printingForce);
    assert(!pauseBeforeLock && g_pauseMode == PAUSE_MODE_PAUSED_AND_MOVED);
    assert(Printer::zOffset.senseMicrometers() == initialClearance);
}

void initializeOffsets() {
    Printer::zOffset.setManualMicrometers(50, zStepsPerMm);
    g_nSensiblePressureDigits = senseThreshold;
    g_nSensiblePressureOffsetMax = initialClearance;
    sample(3 * senseThreshold, 10);
    assert(Printer::zOffset.senseMicrometers() == initialClearance);
    g_nSensiblePressureDigits = 0;
    sample(printingForce, 40);
    g_nSensiblePressureOffsetMax = SENSIBLE_PRESSURE_MAX_OFFSET;
    g_nSensiblePressureDigits = senseThreshold;
}

int main(int argc, char** argv) {
    assert(argc == 2);
    initializeOffsets();
    g_nPauseSteps[Y_AXIS] = 200 * 160;
    g_nPauseSteps[Z_AXIS] = 30 * static_cast<long>(zStepsPerMm);
    g_nPauseSteps[E_AXIS] = 280;

    if (!std::strcmp(argv[1], "park"))
        parkAndResume();
    else if (!std::strcmp(argv[1], "short-pause"))
        pauseBetweenPolls();
    else if (!std::strcmp(argv[1], "interrupt"))
        interruptDuringLearning();
    else
        return 1;
    std::printf("SenseOffset pause replay passed (%s)\n", argv[1]);
}

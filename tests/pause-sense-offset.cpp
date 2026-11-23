#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "Constants.h"
#undef uint
#define MOTHERBOARD DEVICE_TYPE_RF2000
#define NUM_EXTRUDER 1
#include "Configuration.h"
#include "RF2000.h"
#include "ZOffset.h"
#include "PrintLayers.h"
#include "BedCompensation.h"
#include "PrintPause.h"
#include "PrintStop.h"

// The replay compiles the live firmware routines. These shims supply DMS bytes,
// a bed profile, a clock and the motor displacements completed during each wait.
using millis_t = uint32_t;
constexpr float zStepsPerMm = 1280.0f;
constexpr int16_t initialClearance = 20;
constexpr int16_t printingForce = 6000;
constexpr int16_t senseThreshold = 3000;
constexpr long parkY = 200 * 160L, parkZ = 30 * static_cast<long>(zStepsPerMm), parkE = 280;

millis_t clockMs = 0, g_uStartOfIdle = 0, previousMillisCmd = 0;
volatile uint8_t g_startLineActive = 0;
char g_nHeatBedScanStatus = 0, g_nZOSScanStatus = 0;
unsigned short g_nManualSteps[4] = { 1, 1, 1, 28 };
int32_t g_offsetZCompensationSteps = 250;
short g_ZCompensationMax = 200;
short forceDigits = 0;
bool pollDuringMoves = true;
bool clipReturn[4] = {};
int incompleteReturns = 0;
int incomingReads = 0, objectOutputs = 0;
unsigned matrixDepthReads = 0;
unsigned strainGaugeReads = 0;
void (*duringMove)(long, long, long, long) = nullptr;
void (*duringHeatWait)() = nullptr;
void (*duringPeriodicActions)() = nullptr;
unsigned primePolls = 0, primeMoves = 0, primeWaits = 0, skipPrimeAt = 0, abortPrimeAt = 0;
bool observedPrimeBrake = false;

void requestInterruptPause();
void poll();

namespace HAL {
millis_t timeInMilliseconds() { return clockMs; }
}

bool interruptsEnabled = true;
unsigned pauseBeforeLock = 0;
unsigned locksUntilCompensationPublish = 0;
float extrusionBeforePublish = 0.0f;

namespace Printer {
extern float compensatedPositionOverPercE;
}

class InterruptProtectedBlock {
    const bool previousState = interruptsEnabled;

public:
    InterruptProtectedBlock() {
        if (interruptsEnabled && pauseBeforeLock && --pauseBeforeLock == 0)
            requestInterruptPause();
        if (interruptsEnabled && locksUntilCompensationPublish && --locksUntilCompensationPublish == 0)
            extrusionBeforePublish = Printer::compensatedPositionOverPercE;
        interruptsEnabled = false;
    }
    ~InterruptProtectedBlock() { unprotect(); }
    void unprotect() { interruptsEnabled = previousState; }
};

enum { Z_MIN_PIN, Z_MAX_PIN, Z_DIR_PIN, Z_STEP_PIN };
constexpr bool LOW = false, HIGH = true;
#define INLINE inline
bool READ(int pin);
void WRITE(int pin, bool high);
#include "ZMotion.h"
#include "DigitCompensation.h"

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

template<class... Args> void showWarning(Args...) { ++incompleteReturns; }
template<class... Args> void showError(Args...) { assert(false); }
template<class... Args> void showInformation(Args...) { assert(false); }
void doEmergencyStop(int) { assert(false); }
bool isAnyScanRunning();
void setExtruderCurrent(int, int) {}

struct WireInput {
    uint8_t byteIndex = 0;
    void beginTransmission(uint8_t address) const { assert(address == I2C_ADDRESS_STRAIN_GAUGE); }
    void requestFrom(uint8_t address, uint8_t length) {
        assert(address == I2C_ADDRESS_STRAIN_GAUGE && length == 3);
        ++strainGaugeReads;
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
extern PrintPause pause;
extern PrintStop printStop;
ZOffset zOffset;
ZMotion zMotion(zStepsPerMm);
PrintLayers layers{zStepsPerMm};
DigitCompensation digitCompensation{
    short(std::abs(EMERGENCY_PAUSE_DIGITS_MAX) * 0.7), short(std::abs(EMERGENCY_PAUSE_DIGITS_MAX))
};
long currentXYSteps[2] = { 6400, 12800 };
long directCurrentSteps[4] = { 0, 16, 8, 0 };
struct DirectDisplacement {
    long displacement;
    long steps() const { return displacement; }
};
DirectDisplacement directPosition(uint8_t axis) { return {directCurrentSteps[axis]}; }
int32_t queuedPositionSteps[3] = {};
float axisStepsPerMM[3] = { 160.0f, 160.0f, zStepsPerMm };
float axisMMPerSteps[3] = { 1.0f / 160, 1.0f / 160, 1.0f / zStepsPerMm };
float homingFeedrate[3] = { 60.0f, 60.0f, 5.0f };
float axisLengthMM[3] = { 170.0f, 245.0f, 200.0f };
long maxSoftEndstopSteps[2] = { 170 * 160L, 245 * 160L };
float destinationMM[4] = {}, destinationRemainderMM[4] = {};
float extrudeMultiplyErrorSteps = 0.0f, menuExtrusionFactor = 1.0f;
bool relativeExtruderCoordinateMode = false;
float compensatedPositionOverPercE = 0.0f;
float compensatedPositionCollectTinyE = 0.0f;
bool blockAll = false, g_pressure_offset_active = false;
volatile char doHeatBedZCompensation = true;
bool printing = true, menuPaused = false;
short g_pressure_offset = 0;
int feedrateMultiply = 100, motorCurrent[4] = {};
char moveMode[3] = { MOVE_MODE_SINGLE_STEPS, MOVE_MODE_SINGLE_STEPS, MOVE_MODE_SINGLE_STEPS };

bool debugErrors() { return false; }
bool isPrinting() { return printing; }
void setPrinting(bool active) { printing = active; }
bool areAxisHomed() { return true; }
bool isAxisHomed(int) { return true; }
bool isZMoveActive() { return false; }
bool isZMaxEndstopHit() { return zMotion.maxEndstopHit(); }
bool isZMinEndstopHit() { return ZMotion::minSwitchHit(); }
bool needsCMPwait() { return false; }
void setMenuMode(int mode, bool active) {
    assert(mode == MENU_MODE_PAUSED);
    menuPaused = active;
}
void stopPrint();
void homeZAxis();
char anyHomeDir(uint8_t axis) {
    assert(axis == Z_AXIS);
    return -1;
}
void setHomed(int8_t x, int8_t y, int8_t z) {
    assert(x == -1 && y == -1);
    zMotion.setHomed(z);
    if (!z)
        doHeatBedZCompensation = false;
}
// Model the endstop contact and backoff used by homeZAxis().
void queueRelativeStepsCoordinates(long, long, long z, long, float, bool, bool checkEndstop) {
    assert(z < 0 && checkEndstop);
    zMotion.setMotorReference(0);
}
void queueRelativeMMCoordinates(float, float, float z, float, float, bool, bool checkEndstop = false) {
    const int32_t steps = zMotion.motorSteps() + int32_t(z * axisStepsPerMM[Z_AXIS]);
    zMotion.setMotorReference(checkEndstop ? std::max(int32_t(0), steps) : steps);
}
void setXAxisSteps(long) {}
void setYAxisSteps(long) {}
void setZAxisSteps(long steps) { zMotion.setNominalSteps(steps); }
void setEPositionMM(float value, float remainder = 0) {
    destinationMM[E_AXIS] = value;
    destinationRemainderMM[E_AXIS] = remainder;
}
void queueFloatCoordinates(float, float, float z, float e, float) {
    assert(g_startLineActive && !printStop.commandsBlocked());
    ++primeMoves;
    if (z != IGNORE_COORDINATE)
        zMotion.setNominalSteps(std::lround(z * axisStepsPerMM[Z_AXIS]));
    if (e != IGNORE_COORDINATE) {
        setEPositionMM(e);
        extrudeMultiplyErrorSteps = 0.375f;
        compensatedPositionCollectTinyE = 0.25f;
    }
}
bool checkPlayKey() { return skipPrimeAt && primePolls >= skipPrimeAt; }
void offsetRelativeStepsCoordinates(long x, long y, long z, long e, int task = TASK_NO_TASK);
}

void requestInterruptPause() {
    Printer::pause.request(PrintPause::Destination::Park);
}

bool READ(int pin) {
    const int32_t motor = Printer::zMotion.motorSteps();
    if (pin == Z_MIN_PIN)
        return motor > 0;
    assert(pin == Z_MAX_PIN);
    return motor < 200000;
}

template<class T, class A, class B>
T constrain(T value, A low, B high) {
    return value < low ? low : value > high ? high : value;
}

namespace RMath {
float min(float a, float b) { return std::min(a, b); }
}

struct Extruder {
    struct {
        float targetTemperatureC = 200.0f;
        uint8_t paused = 0;
        void waitForTargetTemperature(int, bool resumingPause) {
            assert(resumingPause && Printer::pause.isReheating());
            if (duringHeatWait)
                duringHeatWait();
            poll();
        }
    } tempControl;
    int8_t stepperDirection = 0;
    uint8_t id = 0;
    float offsetMM[2] = {};

    static Extruder* current;
    static void setTemperatureForExtruder(float temperature, int tool);
    static void setHeatedBedTemperature(float temperature) { bedTemperature = temperature; }
    static void setTemperatureForAllExtruders(float temperature) { setTemperatureForExtruder(temperature, 0); }
    static float bedTemperature;
};

Extruder extruder[NUM_EXTRUDER];
Extruder* Extruder::current = extruder;
float Extruder::bedTemperature = 60.0f;

void Extruder::setTemperatureForExtruder(float temperature, int tool) {
    extruder[tool].tempControl.targetTemperatureC = temperature;
    extruder[tool].tempControl.paused = 0;
}

struct PrintLine {
    bool moveZ = false;
    bool positiveE = false;
    bool moveXY = false;
    bool isZMove() const { return moveZ; }
    bool isEPositiveMove() const { return positiveE; }
    bool isXOrYMove() const { return moveXY; }

    static struct Direct { int task = TASK_NO_TASK; } direct;
    static PrintLine* cur;
    static int linesCount;
    static int queuedTask, stopRequests;
    static bool hasLines() { return linesCount; }
    static bool hasStoppableDirectMove();
    static void stopDirectMove() { ++stopRequests; }
    static void resetPathPlanner() { linesCount = 0; queuedTask = TASK_NO_TASK; }
    static void queueTask(int task) {
        assert(!interruptsEnabled && linesCount < MOVE_CACHE_SIZE);
        queuedTask = task;
        ++linesCount;
    }
};

PrintLine::Direct PrintLine::direct;
PrintLine* PrintLine::cur = nullptr;
int PrintLine::linesCount = 3;
int PrintLine::queuedTask = TASK_NO_TASK;
int PrintLine::stopRequests = 0;

struct UIDisplay {
    int lastButtonAction = 0, menuLevel = 1, menuPos[1] = {};
    int flags = 0, jogButtonAction = 0;
    int16_t nextKey = 0;
    millis_t lastButtonStart = 0;
    void exitmenu() {}
    void initializeLCD() {}
    void ui_check_keys(int16_t& action) { action = nextKey; }
    void pressPlay();
    void fastAction();
} uid;

enum FirmwareState { Paused, Processing };
namespace Commands {
void checkForPeriodicalActions(FirmwareState);
void waitUntilEndOfAllMoves() { ++primeWaits; PrintLine::linesCount = 0; }
void changeFeedrateMultiply(int) { assert(false); }
void printCurrentPosition() {}
}

struct GCode {
    long S = 0, P = 0;
    float E = 0, F = 0, I = 1, Y = 0;
    bool hasS() const { return S != 0; }
    bool hasP() const { return P != 0; }
    bool hasE() const { return E != 0; }
    bool hasF() const { return F != 0; }
    bool hasI() const { return I != 0; }
    bool hasY() const { return Y != 0; }

    static void readFromSerial() {
        assert(Printer::printStop.commandsBlocked());
        ++incomingReads;
    }
};
void outputObject(bool showerrors) {
    assert(!showerrors && !PrintLine::hasLines() && !PrintLine::direct.task);
    assert(!Printer::printStop.commandsBlocked());
    ++objectOutputs;
}

long getZMatrixDepth(long x, long y) {
    ++matrixDepthReads;
    return 37 + x / 32 - y / 64;
}

#include "pause-sense-offset-firmware.inc"

void poll() {
    clockMs += EMERGENCY_PAUSE_INTERVAL + 1;
    handleStrainGaugeFeatures(clockMs);
    recalculateZCompensation();
}

void sample(short force, int count = 1) {
    forceDigits = force;
    for (int i = 0; i < count; ++i)
        poll();
}

bool pauseDuringQueueWait = false;
void Commands::checkForPeriodicalActions(FirmwareState state) {
    if (duringPeriodicActions)
        duringPeriodicActions();
    if (state == Processing && Printer::pause.allowsQueueAdvance() && PrintLine::hasLines()) {
        if (pauseDuringQueueWait) {
            pauseDuringQueueWait = false;
            Printer::pause.request(PrintPause::Destination::InPlace);
        } else if (--PrintLine::linesCount == 0 && PrintLine::queuedTask != TASK_NO_TASK) {
            const auto destination = PrintLine::queuedTask == TASK_PAUSE_PRINT_AND_MOVE
                ? PrintPause::Destination::Park : PrintPause::Destination::InPlace;
            PrintLine::queuedTask = TASK_NO_TASK;
            Printer::pause.request(destination);
        }
    }
    if (PrintLine::direct.task && PrintLine::stopRequests) {
        const long stopTail = -32;
        Printer::directCurrentSteps[Z_AXIS] += stopTail;
        Printer::zMotion.setMotorReference(Printer::zMotion.motorSteps() + stopTail);
        PrintLine::direct.task = TASK_NO_TASK;
    }
    Printer::pause.serviceMotion();
}

void Printer::offsetRelativeStepsCoordinates(long x, long y, long z, long e, int task) {
    // A short jog toward the print point travels one mm before key release.
    if (task == TASK_MOVE_FROM_BUTTON)
        z = std::max(z, -static_cast<long>(zStepsPerMm));
    if (x || y || z || e) {
        if (pollDuringMoves)
            poll();
        if (clipReturn[X_AXIS] && x < 0) x = 0;
        if (clipReturn[Y_AXIS] && y < 0) y = 0;
        if (clipReturn[Z_AXIS] && z < 0) z = 0;
        if ((clipReturn[E_AXIS] && e > 0) || g_nEmergencyESkip) e = 0;
        directCurrentSteps[X_AXIS] += x;
        directCurrentSteps[Y_AXIS] += y;
        directCurrentSteps[Z_AXIS] += z;
        directCurrentSteps[E_AXIS] += e;
        zMotion.setMotorReference(zMotion.motorSteps() + z);
        for (long pulse = 0; pulse < std::labs(e); ++pulse)
            pause.recordDirectExtrusion(e > 0 ? 1 : -1);
        if (duringMove)
            duringMove(x, y, z, e);
    }
}

void parkAndResume() {
    sample(printingForce, 9);
    assert(Printer::zOffset.senseMicrometers() == initialClearance && g_nSensiblePressure1stMarke);
    const long firstTarget = Printer::zMotion.correctionTarget();
    const long originalY = Printer::directCurrentSteps[Y_AXIS];
    const long originalZ = Printer::directCurrentSteps[Z_AXIS];

    Printer::pause.request(PrintPause::Destination::Park);
    assert(Printer::pause.isPaused() && !Printer::pause.isSettled());
    sample(0);
    assert(Printer::zOffset.senseMicrometers() == initialClearance && !g_nSensiblePressure1stMarke);
    Printer::pause.serviceMotion();
    assert(Printer::pause.isSettled());
    assert(Printer::directCurrentSteps[Y_AXIS] == originalY + parkY);
    assert(Printer::directCurrentSteps[Z_AXIS] == originalZ + parkZ);

    sample(0, 40);
    const long zeroForceTarget = Printer::zMotion.correctionTarget();
    const long printMatrix = getZMatrixDepth(Printer::currentXYSteps[X_AXIS], Printer::currentXYSteps[Y_AXIS]);
    assert(zeroForceTarget == printMatrix + Printer::zOffset.effectiveSteps());

    sample(printingForce, 40);
    assert(Printer::zMotion.correctionTarget() > zeroForceTarget);
    assert(Printer::zOffset.senseMicrometers() == initialClearance);
    const long parkedZ = Printer::directCurrentSteps[Z_AXIS];
    moveZAction(-1);
    assert(Printer::directCurrentSteps[Z_AXIS] == parkedZ - static_cast<long>(zStepsPerMm));

    sample(EMERGENCY_STOP_DIGITS_MAX + 1, 2 * EMERGENCY_STOP_CHECKS);
    assert(g_nEmergencyESkip);
    sample(printingForce, 50);
    assert(!g_nEmergencyESkip && Printer::pause.isWarmPause());
    extrudeFromPanel();
    const long manualExtruderPosition = -parkE + g_nManualSteps[E_AXIS];
    assert(Printer::directCurrentSteps[E_AXIS] == manualExtruderPosition);

    const auto& offset = Printer::zOffset;
    const long firstManual = offset.manualMicrometers();
    Printer::zOffset.adjustWithBedButton(Z_OFFSET_BUTTON_STEPS, 10000, zStepsPerMm);
    Printer::zOffset.adjustWithBedButton(-Z_OFFSET_BUTTON_STEPS, 10000, zStepsPerMm);
    assert(offset.manualMicrometers() == firstManual + Z_OFFSET_BUTTON_STEPS);
    assert(offset.senseMicrometers() == initialClearance - Z_OFFSET_BUTTON_STEPS);

    clockMs += EXTRUDER_CURRENT_PAUSE_DELAY + 1;
    Printer::pause.serviceStandby(clockMs);
    assert(Printer::pause.isSettled() && !Printer::pause.isWarmPause());
    uid.pressPlay();
    assert(Printer::pause.isWarmPause());
    uid.pressPlay();
    assert(Printer::pause.allowsQueueAdvance());
    assert(Printer::directCurrentSteps[Y_AXIS] == originalY && Printer::directCurrentSteps[Z_AXIS] == originalZ);
    assert(Printer::directCurrentSteps[E_AXIS] == manualExtruderPosition);
    assert(Printer::zMotion.correctionTarget() == firstTarget);

    sample(printingForce, 9);
    assert(offset.senseMicrometers() == initialClearance - Z_OFFSET_BUTTON_STEPS);
    sample(printingForce);
    assert(offset.senseMicrometers() > initialClearance - Z_OFFSET_BUTTON_STEPS);
}

void pauseBetweenPolls() {
    sample(printingForce, 9);
    pollDuringMoves = false;
    Printer::pause.request(PrintPause::Destination::InPlace);
    Printer::pause.serviceMotion();
    assert(Printer::pause.resume() && Printer::pause.allowsQueueAdvance());

    sample(printingForce, 9);
    assert(Printer::zOffset.senseMicrometers() == initialClearance);
    sample(printingForce);
    assert(Printer::zOffset.senseMicrometers() > initialClearance);
}

void interruptDuringLearning() {
    sample(printingForce, 9);
    // Let the poll capture its pause state and the nominal Z. Enter a queued
    // pause just before the lock that would publish the SenseOffset update.
    pauseBeforeLock = 3;
    sample(printingForce);
    assert(!pauseBeforeLock && Printer::pause.isPaused() && !Printer::pause.isSettled());
    assert(Printer::zOffset.senseMicrometers() == initialClearance);
}

void panelSequence() {
    const long y = Printer::directCurrentSteps[Y_AXIS], z = Printer::directCurrentSteps[Z_AXIS];
    duringMove = [](long, long, long, long) {
        assert(Printer::pause.isPaused() && !Printer::pause.isSettled());
        assert(!Printer::pause.resume());
        Printer::pause.pressPause();
        Printer::pause.serviceMotion();
    };

    Printer::pause.pressPause();
    assert(Printer::pause.isWarmPause() && Printer::menuPaused);
    assert(Printer::directCurrentSteps[Y_AXIS] == y && Printer::directCurrentSteps[Z_AXIS] == z);
    assert(Printer::directCurrentSteps[E_AXIS] == -parkE);
    Printer::pause.pressPause();
    duringMove = nullptr;
    assert(Printer::pause.isWarmPause() && Printer::directCurrentSteps[E_AXIS] == -parkE);
    assert(Printer::directCurrentSteps[Y_AXIS] == y + parkY && Printer::directCurrentSteps[Z_AXIS] == z + parkZ);

    // Force standby even if uptime is shorter than the automatic wait.
    assert(clockMs < EXTRUDER_CURRENT_PAUSE_DELAY);
    Printer::pause.pressPause();
    Printer::pause.serviceStandby(++clockMs);
    assert(Printer::pause.isSettled() && !Printer::pause.isWarmPause());
    assert(Extruder::current->tempControl.targetTemperatureC == 100.0f);
    uid.pressPlay();
    assert(Printer::pause.isWarmPause() && Extruder::current->tempControl.targetTemperatureC == 200.0f);
    assert(Printer::directCurrentSteps[Y_AXIS] == y + parkY);
    Printer::pause.serviceStandby(clockMs + EXTRUDER_CURRENT_PAUSE_DELAY + 1);
    assert(Printer::pause.isWarmPause());
    uid.pressPlay();
    assert(Printer::pause.allowsQueueAdvance() && !Printer::menuPaused);
    assert(Printer::directCurrentSteps[Y_AXIS] == y && Printer::directCurrentSteps[Z_AXIS] == z);
    assert(Printer::directCurrentSteps[E_AXIS] == 0);

    pollDuringMoves = false;
    clockMs = UINT32_MAX - 500;
    Printer::pause.pressPause();
    Printer::pause.serviceStandby(clockMs + EXTRUDER_CURRENT_PAUSE_DELAY);
    assert(Printer::pause.isWarmPause());
    Printer::pause.serviceStandby(clockMs + EXTRUDER_CURRENT_PAUSE_DELAY + 1);
    assert(Printer::pause.isSettled() && !Printer::pause.isWarmPause());
}

void clippedReturn() {
    const long y = Printer::directCurrentSteps[Y_AXIS], z = Printer::directCurrentSteps[Z_AXIS];
    pollDuringMoves = false;
    Printer::pause.request(PrintPause::Destination::Park);
    Printer::pause.serviceMotion();
    assert(Printer::pause.limitZJog(-2 * parkZ) == -parkZ);

    clipReturn[Y_AXIS] = true;
    assert(Printer::pause.resume());
    assert(Printer::pause.isSettled() && incompleteReturns == 1);
    assert(Printer::directCurrentSteps[Y_AXIS] == y + parkY);
    assert(Printer::directCurrentSteps[Z_AXIS] == z + parkZ && Printer::directCurrentSteps[E_AXIS] == -parkE);

    clipReturn[Y_AXIS] = false;
    clipReturn[Z_AXIS] = true;
    assert(Printer::pause.resume());
    assert(Printer::pause.isSettled() && incompleteReturns == 2);
    assert(Printer::directCurrentSteps[Y_AXIS] == y);
    assert(Printer::directCurrentSteps[Z_AXIS] == z + parkZ && Printer::directCurrentSteps[E_AXIS] == -parkE);

    clipReturn[Z_AXIS] = false;
    clipReturn[E_AXIS] = true;
    assert(Printer::pause.resume());
    assert(Printer::pause.isSettled() && incompleteReturns == 3);
    assert(Printer::directCurrentSteps[Z_AXIS] == z && Printer::directCurrentSteps[E_AXIS] == -parkE);
    assert(Printer::pause.limitZJog(-32) == 0);
    clipReturn[E_AXIS] = false;
    assert(Printer::pause.resume() && Printer::pause.allowsQueueAdvance());
    assert(Printer::directCurrentSteps[E_AXIS] == 0);

    Printer::pause.request(PrintPause::Destination::Park);
    Printer::pause.serviceMotion();
    duringMove = [](long, long yMove, long, long) {
        if (yMove < 0)
            Printer::stopPrint();
    };
    assert(Printer::pause.resume() && Printer::printStop.commandsBlocked());
    duringMove = nullptr;
    assert(Printer::directCurrentSteps[Y_AXIS] == y);
    assert(Printer::directCurrentSteps[Z_AXIS] == z + parkZ && Printer::directCurrentSteps[E_AXIS] == -parkE);
    assert(incompleteReturns == 3);
}

void overlappingJog() {
    const long z = Printer::directCurrentSteps[Z_AXIS];
    Printer::pause.request(PrintPause::Destination::Park);
    Printer::pause.serviceMotion();
    PrintLine::direct.task = DIRECT_RUNNING;
    assert(!Printer::pause.resume() && !Printer::pause.allowsManualMove());

    PrintLine::direct.task = DIRECT_RUNNING_STOPPABLE;
    uid.jogButtonAction = UI_ACTION_RF_HEAT_BED_DOWN;
    uid.nextKey = UI_ACTION_RF_HEAT_BED_DOWN;
    uid.fastAction();
    assert(!PrintLine::stopRequests);
    uid.nextKey = 0;
    uid.fastAction();
    assert(PrintLine::stopRequests == 1 && PrintLine::direct.task);

    // The return uses the position after the deceleration tail has completed.
    assert(Printer::pause.resume());
    assert(PrintLine::stopRequests == 2 && !PrintLine::direct.task);
    assert(Printer::pause.allowsQueueAdvance() && Printer::directCurrentSteps[Z_AXIS] == z);
}

void cancelAndCleanup() {
    const long y = Printer::directCurrentSteps[Y_AXIS];
    pollDuringMoves = false;
    clockMs = UINT32_MAX - 500;
    duringMove = [](long, long, long z, long) {
        if (z > 0) {
            PrintLine::direct.task = DIRECT_RUNNING;
            Printer::stopPrint();
        }
    };
    Printer::pause.request(PrintPause::Destination::Park);
    Printer::pause.serviceMotion();
    duringMove = nullptr;
    assert(Printer::directCurrentSteps[Y_AXIS] == y);
    assert(PrintLine::stopRequests == 1 && !PrintLine::hasLines());
    assert(Printer::printStop.commandsBlocked() && !Printer::menuPaused);
    assert(Printer::pause.allowsQueueAdvance());
    Printer::pause.queueRequest(PrintPause::Destination::InPlace);
    Printer::pause.request(PrintPause::Destination::Park);
    assert(Printer::pause.allowsQueueAdvance() && !PrintLine::hasLines());

    Printer::printStop.service(clockMs + 2001);
    assert(Printer::isPrinting() && incomingReads == 1 && !objectOutputs);
    PrintLine::direct.task = TASK_NO_TASK;
    g_nHeatBedScanStatus = g_nZOSScanStatus = 1;
    Printer::printStop.service(clockMs + 4001);
    assert(Printer::isPrinting() && !objectOutputs);
    Printer::printStop.service(clockMs + 4002);
    assert(!Printer::isPrinting() && !g_nHeatBedScanStatus && !g_nZOSScanStatus);
    assert(Extruder::bedTemperature == 0.0f && Extruder::current->tempControl.targetTemperatureC == 0.0f);
    assert(Printer::printStop.commandsBlocked());
    Printer::printStop.service(clockMs + 5002);
    assert(!objectOutputs);
    Printer::printStop.service(clockMs + 5003);
    Printer::printStop.service(clockMs + 6000);
    assert(objectOutputs == 1 && !Printer::printStop.commandsBlocked());

    // A USB stop during a heater wait must also discard the pending return.
    clockMs = 0;
    Printer::setPrinting(true);
    Extruder::setTemperatureForExtruder(200.0f, 0);
    Printer::pause.request(PrintPause::Destination::Park);
    Printer::pause.serviceMotion();
    clockMs += EXTRUDER_CURRENT_PAUSE_DELAY + 1;
    Printer::pause.serviceStandby(clockMs);
    duringHeatWait = Printer::stopPrint;
    assert(Printer::pause.resume() && Printer::printStop.commandsBlocked());
    assert(Printer::pause.allowsQueueAdvance() && Printer::directCurrentSteps[Y_AXIS] == y + parkY);
}

void queuedPause() {
    Printer::pause.queueRequest(PrintPause::Destination::Park);
    assert(Printer::pause.isSettled() && !PrintLine::hasLines());
    Printer::pause.queueRequest(PrintPause::Destination::InPlace);
    assert(!PrintLine::hasLines() && Printer::pause.resume());
    assert(Printer::pause.allowsQueueAdvance());

    PrintLine::linesCount = MOVE_CACHE_SIZE;
    pauseDuringQueueWait = true;
    Printer::pause.queueRequest(PrintPause::Destination::Park);
    assert(Printer::pause.isSettled() && PrintLine::linesCount == MOVE_CACHE_SIZE);
    assert(PrintLine::queuedTask == TASK_NO_TASK);
}

void planMove(long z, bool hasZMove, bool extrudes = true, bool hasXYMove = true) {
    PrintLine move;
    move.moveZ = hasZMove;
    move.positiveE = extrudes;
    move.moveXY = hasXYMove;
    Printer::queuedPositionSteps[Z_AXIS] = z;
    trackPlannedLayer(&move);
}

void homeBetweenPrints() {
    auto& layers = Printer::layers;
    const long oldLayer = 512; // 0.40 mm
    const long newLayer = 256; // 0.20 mm
    const long primeZ = 448; // 0.35 mm
    const long defaultMin = 422;
    const long fadeSpan = (g_offsetZCompensationSteps - g_ZCompensationMax) * 20;
    PrintLine::linesCount = 0;
    layers.resetTracking();
    planMove(oldLayer, true);
    assert(layers.fadeStartSteps() == oldLayer && layers.fadeEndSteps() == oldLayer + fadeSpan);
    sample(printingForce, 9);
    assert(Printer::zOffset.senseMicrometers() == initialClearance);

    const long manualZ = Printer::zOffset.manualMicrometers();
    planMove(oldLayer + 64, true);
    planMove(oldLayer + 128, true, false, false);
    Printer::compensatedPositionOverPercE = 12.5f;
    Printer::compensatedPositionCollectTinyE = 0.75f;

    Printer::homeZAxis();
    assert(!Printer::doHeatBedZCompensation && Printer::zMotion.isHomed());
    assert(!Printer::zMotion.nominalSteps() && !Printer::zMotion.motorSteps());
    assert(Printer::zOffset.manualMicrometers() == manualZ && Printer::zOffset.senseMicrometers() == 0);
    assert(Printer::zMotion.correctionTarget() == 0);
    assert(Printer::compensatedPositionOverPercE == 0.0f && Printer::compensatedPositionCollectTinyE == 0.0f);
    assert(g_nSensiblePressureDigits == 0 && !g_nSensiblePressure1stMarke);
    assert(layers.fadeStartSteps() == defaultMin && layers.fadeEndSteps() == 12800);
    planMove(0, false, true, false); // An E-only move must not confirm the pre-home candidate.
    assert(!layers.currentLayerSteps() && !layers.previousLayerSteps());

    Printer::doHeatBedZCompensation = true;
    g_startLineActive = true;
    planMove(primeZ, true);
    assert(layers.fadeStartSteps() == defaultMin && !layers.currentLayerSteps());
    g_startLineActive = false;
    planMove(newLayer, true);
    assert(layers.fadeStartSteps() == newLayer && layers.fadeEndSteps() == newLayer + fadeSpan);
    assert(layers.currentLayerSteps() == newLayer && !layers.previousLayerSteps());

    // Re-enable SenseOffset before the first DMS poll of the new job.
    Printer::zMotion.setNominalSteps(newLayer);
    g_nSensiblePressureDigits = senseThreshold;
    sample(printingForce, 9);
    assert(!Printer::zOffset.senseMicrometers());
    sample(printingForce);
    assert(Printer::zOffset.senseMicrometers() > 0);

    // A print without M3912 can learn the model layer even with no separate Z move.
    Printer::homeZAxis();
    assert(layers.fadeStartSteps() == defaultMin);
    const long layerWithoutPrime = 451; // 0.352 mm rounded to steps
    planMove(layerWithoutPrime, false);
    assert(layers.fadeStartSteps() == layerWithoutPrime);
    assert(layers.currentLayerSteps() == layerWithoutPrime && !layers.previousLayerSteps());
}

void expectCompensation(long nominalZ, long correctionTarget, float extraExtrusion, unsigned expectedReads) {
    Printer::zMotion.setNominalSteps(nominalZ);
    const unsigned readsBefore = matrixDepthReads;
    recalculateZCompensation();
    assert(Printer::zMotion.correctionTarget() == correctionTarget);
    assert(std::abs(Printer::compensatedPositionOverPercE - extraExtrusion) < 0.000001f);
    assert(matrixDepthReads - readsBefore == expectedReads);
}

void layerFadeAndZHop() {
    auto& layers = Printer::layers;
    const long primeZ = 448; // 0.35 mm
    const long fadeEnd = primeZ + 1000; // 50 steps of matrix relief.
    layers.resetTracking();
    Printer::currentXYSteps[X_AXIS] = 5280; // Interpolated bed depth of 202 steps.
    Printer::currentXYSteps[Y_AXIS] = 0;
    Printer::digitCompensation.setDigitZEnabled(false);
    const long offset = Printer::zOffset.effectiveSteps();

    g_startLineActive = true;
    planMove(primeZ, true);
    g_startLineActive = false;
    assert(!layers.currentLayerSteps());

    planMove(primeZ, false, true, false); // E-only unretract.
    assert(!layers.currentLayerSteps());
    planMove(primeZ, false);
    assert(layers.currentLayerSteps() == primeZ && !layers.previousLayerSteps());
    assert(layers.fadeStartSteps() == primeZ && layers.fadeEndSteps() == fadeEnd);
    expectCompensation(primeZ, 202 + offset, 0.0f, 1);

    // Travel and return to the same layer before XY extrusion resumes.
    planMove(primeZ + 128, true, false, false);
    planMove(primeZ, true, false, false);
    planMove(primeZ, false);
    assert(layers.currentLayerSteps() == primeZ && !layers.previousLayerSteps());
    assert(layers.fadeStartSteps() == primeZ && layers.fadeEndSteps() == fadeEnd);

    planMove(fadeEnd - 50, true);
    // The executor is still within the fade even though the planner is farther ahead.
    expectCompensation(primeZ + 500, 226 + offset, 0.048f, 1);
    planMove(fadeEnd + 50, true);
    expectCompensation(fadeEnd, 250 + offset, 0.024f, 1);
    planMove(fadeEnd + 100, true);
    expectCompensation(fadeEnd + 50, 250 + offset, 0.0f, 0);
}

void compensationBoundaries() {
    auto& layers = Printer::layers;
    Printer::currentXYSteps[X_AXIS] = 5280; // Interpolated bed depth of 202 steps.
    Printer::currentXYSteps[Y_AXIS] = 0;
    Printer::digitCompensation.setDigitZEnabled(false);
    const long offset = Printer::zOffset.effectiveSteps();
    const long fadeStart = layers.fadeStartSteps();
    const long fadeEnd = layers.fadeEndSteps();

    Printer::compensatedPositionOverPercE = 0.048f;
    locksUntilCompensationPublish = 2; // One read of nominal Z, then publication.
    expectCompensation(-1, 263 + offset, 0.0f, 0);
    assert(!locksUntilCompensationPublish && extrusionBeforePublish == 0.048f);
    expectCompensation(0, 215 + offset, 0.0f, 1);
    expectCompensation(fadeStart, 202 + offset, 0.0f, 1);
    expectCompensation(fadeStart + 1, 203 + offset, 0.048f, 1);
    expectCompensation(fadeEnd - 1, 250 + offset, 0.048f, 1);
    expectCompensation(fadeEnd, 250 + offset, 0.0f, 0);

    // Reloading the defaults puts the fade start at 422, inside the planned layer from 256 to 512.
    planMove(512, true);
    layers.resetFadeBounds(zStepsPerMm);
    expectCompensation(448, 203 + offset, 4320.0f / (12378.0f * 256.0f), 1);

    auto settleDigitZ = [](short digits) {
        forceDigits = digits;
        for (int i = 0; i < 40; ++i)
            sampleStrainGauge();
    };
    Printer::digitCompensation.setDigitZEnabled(true);
    settleDigitZ(-5500);
    expectCompensation(256, 202 + offset + 53, 0.0f, 1);
    settleDigitZ(5500);
    expectCompensation(256, 202 + offset + 53, 0.0f, 1);

    Printer::axisStepsPerMM[Z_AXIS] = 2560.0f;
    layers.resetFadeBounds(Printer::axisStepsPerMM[Z_AXIS]);
    Printer::zOffset.refreshSteps(Printer::axisStepsPerMM[Z_AXIS]);
    const long scaledOffset = Printer::zOffset.effectiveSteps();
    settleDigitZ(32767);
    expectCompensation(0, 215 + scaledOffset + 600, 0.0f, 1);

    Printer::doHeatBedZCompensation = false;
    expectCompensation(0, 0, 0.0f, 0);

    // A flat matrix gives a zero-width fade at the first layer.
    Printer::doHeatBedZCompensation = true;
    Printer::digitCompensation.setDigitZEnabled(false);
    g_ZCompensationMax = g_offsetZCompensationSteps;
    Printer::currentXYSteps[X_AXIS] = 6816; // Interpolated bed depth of 250 steps.
    layers.resetTracking();
    planMove(256, true);
    assert(layers.fadeStartSteps() == 256 && layers.fadeEndSteps() == 256);
    expectCompensation(256, 250 + scaledOffset, 0.0f, 1);
}

void layerScaleTransitions() {
    auto& layers = Printer::layers;
    planMove(253, true); // A near-bed descent starts a new model layer sequence.
    assert(layers.currentLayerSteps() == 253 && !layers.previousLayerSteps());
    assert(layers.fadeStartSteps() == 253 && layers.fadeEndSteps() == 1253);
    planMove(511, true);
    planMove(601, true, false, false);

    Printer::axisStepsPerMM[Z_AXIS] = 640.0f;
    layers.rescaleFadeBounds(0.5f);
    assert(layers.fadeStartSteps() == 126 && layers.fadeEndSteps() == 626);
    assert(layers.currentLayerSteps() == 511 && layers.previousLayerSteps() == 253);

    layers.resetFadeBounds(Printer::axisStepsPerMM[Z_AXIS]);
    assert(layers.fadeStartSteps() == 211 && layers.fadeEndSteps() == 6400);
    assert(layers.currentLayerSteps() == 511 && layers.previousLayerSteps() == 253);

    // Motor disable clears the confirmed layers and retains the candidate.
    layers.clearConfirmedLayers();
    assert(!layers.currentLayerSteps() && !layers.previousLayerSteps());
    planMove(601, false, true, false);
    assert(layers.currentLayerSteps() == 601 && !layers.previousLayerSteps());
}

void digitFlowSamples() {
    auto& compensation = Printer::digitCompensation;
    compensation.setFlowProfile({1500, 4500, -75, -90});
    planMove(256, true);
    Printer::zMotion.setNominalSteps(256);

    auto readAt = [](millis_t time, short digits, unsigned expectedReads) {
        clockMs = time;
        forceDigits = digits;
        const unsigned before = strainGaugeReads;
        handleStrainGaugeFeatures(clockMs);
        if (strainGaugeReads - before != expectedReads)
            std::fprintf(stderr, "Unexpected DMS sample count at %lu ms\n", static_cast<unsigned long>(time));
        assert(strainGaugeReads - before == expectedReads);
        recalculateZCompensation();
    };
    auto check = [&](short filtered, float extrusion, float speed, unsigned interval, long target) {
        assert(compensation.filteredDigits() == filtered);
        assert(std::abs(compensation.extrusionFactor() - extrusion) < 0.000001f);
        assert(std::abs(compensation.feedrateFactor() - speed) < 0.000001f);
        assert(compensation.intervalScale() == interval);
        assert(Printer::zMotion.correctionTarget() == target);
    };
    readAt(10, 4500, 0);
    check(0, 1, 1, 0, 37);
    for (millis_t time : {11U, 22U, 33U, 44U}) {
        readAt(time, 4500, 1);
        check(4500, 1, 1, 0, 80);
    }
    readAt(50, 5500, 0);
    assert(compensation.latestDigits() == 4500);

    // SenseOffset's 50 ms deadline also feeds the shared Digit-Z filter.
    Printer::zMotion.setNominalSteps(257);
    readAt(51, 4500, 1);
    check(4500, 0.25f, 0.99f, 1034, 81);
    readAt(54, 7000, 0);
    check(4500, 0.25f, 0.99f, 1034, 81);
    readAt(62, 1500, 1);
    check(3750, 0.4375f, 0.98f, 1044, 74);

    Printer::zMotion.setNominalSteps(256);
    g_startLineActive = true;
    readAt(73, -3000, 1);
    check(2062, 0.8595f, 0.97f, 1055, 57);

    g_startLineActive = false;
    Printer::doHeatBedZCompensation = false;
    readAt(84, 5000, 1);
    assert(compensation.latestDigits() == 5000);
    check(2062, 0.8595f, 0.97f, 0, 0);

    Printer::doHeatBedZCompensation = true;
    readAt(95, 4500, 1);
    check(2671, 1, 0.98f, 1044, 63);
    readAt(101, 4500, 1);
    check(3128, 1, 0.99f, 1034, 67);

    Printer::g_pressure_offset_active = true;
    Printer::g_pressure_offset = 1000;
    readAt(112, 5500, 1);
    assert(compensation.latestDigits() == 4500);
    check(3471, 1, 1, 0, 70);

    // The step interrupt can process M3000 during the foreground calculation.
    compensation.observe(4500, Printer::doHeatBedZCompensation, [] {
        Printer::doHeatBedZCompensation = false;
        return true;
    });
    assert(compensation.feedrateFactor() < 1.0f && compensation.intervalScale() == 0);
}

void primeFlowCleanup(unsigned skipAt, unsigned abortAt) {
    auto& compensation = Printer::digitCompensation;
    compensation.setFlowProfile({2000, 6100, 21, -30});
    // The prime lies below this learned boundary and still enables Digit Flow.
    Printer::layers.resetTracking();
    planMove(512, true);
    Printer::setEPositionMM(10.0f, 0.00001f);
    Printer::extrudeMultiplyErrorSteps = 0.125f;
    Printer::compensatedPositionCollectTinyE = 0.75f;
    Printer::relativeExtruderCoordinateMode = true;
    Printer::feedrateMultiply = 85;
    Printer::menuExtrusionFactor = 0.93f;
    skipPrimeAt = skipAt;
    abortPrimeAt = abortAt;
    duringPeriodicActions = [] {
        assert(g_startLineActive);
        ++primePolls;
        if (primePolls == abortPrimeAt) {
            Printer::stopPrint();
            return;
        }
        sample(printingForce);
        const auto& compensation = Printer::digitCompensation;
        const auto profile = compensation.flowProfile();
        assert(profile.minDigits == 1500 && profile.maxDigits == 4500);
        observedPrimeBrake |= compensation.extrusionFactor() < 0.5f
            && compensation.feedrateFactor() < 1.0f && compensation.intervalScale() > 1024;
    };
    GCode command;
    runStartLine(&command);
    duringPeriodicActions = nullptr;

    assert(observedPrimeBrake && primeWaits == 2);
    assert((skipAt || abortAt) ? primeMoves == (skipAt ? skipAt : abortAt) : primeMoves > 300);
    const auto profile = compensation.flowProfile();
    assert(profile.minDigits == 2000 && profile.maxDigits == 6100);
    assert(profile.extrusionPercent == 21 && profile.speedPercent == -30);
    assert(compensation.extrusionFactor() == 1 && compensation.feedrateFactor() == 1 && !compensation.intervalScale());
    assert(!g_startLineActive && Printer::relativeExtruderCoordinateMode);
    assert(Printer::feedrateMultiply == 85 && Printer::menuExtrusionFactor == 0.93f);
    if (abortAt) {
        assert(Printer::printStop.commandsBlocked() && Printer::destinationMM[E_AXIS] == 0.0f);
    } else {
        assert(Printer::destinationMM[E_AXIS] == 10.0f && Printer::destinationRemainderMM[E_AXIS] == 0.00001f);
        assert(Printer::extrudeMultiplyErrorSteps == 0.125f && Printer::compensatedPositionCollectTinyE == 0.75f);
    }
}

void initializeOffsets() {
    planMove(256, true);
    Printer::zMotion.setNominalSteps(256);
    Printer::zMotion.setMotorReference(256 + Printer::directCurrentSteps[Z_AXIS]);
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
    const bool digitFlowReplay = !std::strcmp(argv[1], "digit-flow");
    if (!digitFlowReplay)
        initializeOffsets();
    Printer::pause.restoreDefaultParking();
    Printer::pause.setParkSteps(E_AXIS, parkE);

    if (!std::strcmp(argv[1], "park"))
        parkAndResume();
    else if (!std::strcmp(argv[1], "short-pause"))
        pauseBetweenPolls();
    else if (!std::strcmp(argv[1], "interrupt"))
        interruptDuringLearning();
    else if (!std::strcmp(argv[1], "panel"))
        panelSequence();
    else if (!std::strcmp(argv[1], "return"))
        clippedReturn();
    else if (!std::strcmp(argv[1], "jog"))
        overlappingJog();
    else if (!std::strcmp(argv[1], "cancel"))
        cancelAndCleanup();
    else if (!std::strcmp(argv[1], "queue"))
        queuedPause();
    else if (!std::strcmp(argv[1], "home"))
        homeBetweenPrints();
    else if (!std::strcmp(argv[1], "layer-fade"))
        layerFadeAndZHop();
    else if (!std::strcmp(argv[1], "layer-scale"))
        layerScaleTransitions();
    else if (!std::strcmp(argv[1], "compensation-bounds"))
        compensationBoundaries();
    else if (digitFlowReplay)
        digitFlowSamples();
    else if (!std::strcmp(argv[1], "prime-complete"))
        primeFlowCleanup(0, 0);
    else if (!std::strcmp(argv[1], "prime-skip"))
        primeFlowCleanup(8, 0);
    else if (!std::strcmp(argv[1], "prime-abort"))
        primeFlowCleanup(0, 8);
    else
        return 1;
    std::printf("Print lifecycle replay passed (%s)\n", argv[1]);
}

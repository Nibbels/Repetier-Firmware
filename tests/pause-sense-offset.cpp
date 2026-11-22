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
#include "PrintPause.h"
#include "PrintStop.h"

// The replay compiles the live firmware routines. These shims supply DMS bytes,
// a bed profile, a clock and the motor displacements completed during each wait.
using millis_t = uint32_t;
constexpr int NUM_EXTRUDER = 1;
constexpr float zStepsPerMm = 1280.0f;
constexpr int16_t initialClearance = 20;
constexpr int16_t printingForce = 6000;
constexpr int16_t senseThreshold = 3000;
constexpr int MOVE_CACHE_SIZE = 17;
constexpr float DEFAULT_PAUSE_MM_Z_PRINT = 30.0f;
constexpr long parkY = 200 * 160L, parkZ = 30 * static_cast<long>(zStepsPerMm), parkE = 280;

millis_t clockMs = 0, g_uStartOfIdle = 0, previousMillisCmd = 0;
volatile uint8_t g_startLineActive = 0;
char g_nHeatBedScanStatus = 0, g_nZOSScanStatus = 0;
unsigned short g_nManualSteps[4] = { 1, 1, 1, 28 };
long g_minZCompensationSteps = 256, g_maxZCompensationSteps = 12800;
long g_offsetZCompensationSteps = 250;
long g_ZCompensationMax = 200;
short forceDigits = 0;
bool pollDuringMoves = true;
bool clipReturn[4] = {};
int incompleteReturns = 0;
int incomingReads = 0, objectOutputs = 0;
unsigned startWaitPolls = 0;
void (*duringMove)(long, long, long, long) = nullptr;
void (*duringHeatWait)() = nullptr;
void (*duringPeriodicActions)() = nullptr;

void requestInterruptPause();
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
            requestInterruptPause();
        interruptsEnabled = false;
    }
    ~InterruptProtectedBlock() { unprotect(); }
    void unprotect() { interruptsEnabled = previousState; }
};

enum { Z_MIN_PIN, Z_MAX_PIN, Z_DIR_PIN, Z_STEP_PIN };
constexpr bool INVERT_Z_DIR = false, LOW = false, HIGH = true;
constexpr float Z_ENDSTOP_DRIVE_OVER = 0.7f;
#define INLINE inline
bool READ(int pin);
void WRITE(int pin, bool high);
#include "ZMotion.h"

#define PSTR(text) (text)
#define UI_STATUS_UPD(text) ((void)0)
#define UI_TEXT_MANUAL ""
#define UI_TEXT_LCD_CIRCUIT ""
#define UI_TEXT_RESET ""
const char ui_text_pause[] = "", ui_text_operation_denied[] = "", ui_text_home_unknown[] = "";
const char ui_text_z_axis[] = "", ui_text_max_reached[] = "", ui_text_min_reached[] = "";
const char ui_text_emergency_pause[] = "";

struct Com {
    static int errorCount;
    template<class... Args> static void printF(Args...) {}
    template<class... Args> static void printFLN(Args...) {}
    template<class... Args> static void printErrorFLN(Args...) { ++errorCount; }
};
int Com::errorCount = 0;

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
long currentXYSteps[2] = { 6400, 12800 };
long directCurrentSteps[4] = { 0, 16, 8, 0 };
struct DirectDisplacement {
    long displacement;
    long steps() const { return displacement; }
};
DirectDisplacement directPosition(uint8_t axis) { return {directCurrentSteps[axis]}; }
long queuePositionZLayerGuessNew = 0, queuePositionZLayerLast = 0, queuePositionZLayerCurrent = 256;
long queuedPositionSteps[3] = {};
float axisStepsPerMM[3] = { 160.0f, 160.0f, zStepsPerMm };
float axisMMPerSteps[3] = { 1.0f / 160, 1.0f / 160, 1.0f / zStepsPerMm };
float compensatedPositionOverPercE = 0.0f;
float compensatedPositionCollectTinyE = 0.0f;
float dynamicFeedrateFactor = 1.0f, dynamicExtrusionFactor = 1.0f;
unsigned short interval_mod = 0;
bool blockAll = false, doHeatBedZCompensation = true, g_pressure_offset_active = false;
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
bool canBeginPrintJob();
bool beginPrintJob();
void resetLayerTracking();
void resetPrintZState();
void setXAxisSteps(long) {}
void setYAxisSteps(long) {}
void setZAxisSteps(long steps) { zMotion.setNominalSteps(steps); }
void setEPositionMM(float value) { assert(value == 0); }
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
        void waitForTargetTemperature(int, bool resumingPause) {
            assert(resumingPause && Printer::pause.isReheating());
            if (duringHeatWait)
                duringHeatWait();
            poll();
        }
    } tempControl;
    int8_t stepperDirection = 0;

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
void changeFeedrateMultiply(int) { assert(false); }
void printCurrentPosition() {}
}

struct { bool sdmode = false; } sd;
struct GCode {
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
    pauseBeforeLock = 4;
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
    sd.sdmode = true;
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
    assert(PrintLine::stopRequests == 1 && !sd.sdmode && !PrintLine::hasLines());
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

void planLayer(long z, bool hasZMove) {
    PrintLine move;
    move.moveZ = hasZMove;
    move.positiveE = true;
    move.moveXY = true;
    Printer::queuedPositionSteps[Z_AXIS] = z;
    trackPlannedLayer(&move);
}

void serialPrintStart() {
    const long oldLayer = 512; // 0.40 mm
    const long newLayer = 256; // 0.20 mm
    const long primeZ = 448; // 0.35 mm
    const long defaultMin = long(heatBedZCompensationMinMm * zStepsPerMm);
    const long fadeSpan = (g_offsetZCompensationSteps - g_ZCompensationMax) * 20;
    Printer::resetLayerTracking();
    planLayer(oldLayer, true);
    assert(g_minZCompensationSteps == oldLayer && g_maxZCompensationSteps == oldLayer + fadeSpan);
    sample(printingForce, 9);
    assert(Printer::zOffset.senseMicrometers() == initialClearance);

    const long parkNominalZ = 20 * long(zStepsPerMm);
    const long parkMotorZ = parkNominalZ + Printer::directCurrentSteps[Z_AXIS];
    const long manualZ = Printer::zOffset.manualMicrometers();
    Printer::zMotion.setNominalSteps(parkNominalZ);
    Printer::zMotion.setMotorReference(parkMotorZ);
    Printer::compensatedPositionCollectTinyE = 0.75f;
    Printer::feedrateMultiply = 93;
    g_nDigitZCompensationDigits_active = false;

    PrintLine::linesCount = 3;
    PrintLine::direct.task = DIRECT_RUNNING;
    duringPeriodicActions = [] {
        assert(g_minZCompensationSteps == 512 && Printer::zOffset.senseMicrometers() == initialClearance);
        if (++startWaitPolls == 5)
            PrintLine::direct.task = TASK_NO_TASK;
    };
    dispatchPrintStart(3921);
    duringPeriodicActions = nullptr;
    assert(startWaitPolls == 5 && !PrintLine::hasLines() && !PrintLine::direct.task);
    assert(!Com::errorCount && Printer::doHeatBedZCompensation && Printer::feedrateMultiply == 93);
    assert(Printer::zMotion.nominalSteps() == parkNominalZ && Printer::zMotion.motorSteps() == parkMotorZ);
    assert(Printer::zOffset.manualMicrometers() == manualZ && Printer::zOffset.senseMicrometers() == 0);
    assert(Printer::zMotion.correctionTarget() == g_offsetZCompensationSteps + Printer::zOffset.manualSteps(zStepsPerMm));
    assert(Printer::compensatedPositionCollectTinyE == 0.0f);
    assert(g_nSensiblePressureDigits == 0 && !g_nSensiblePressure1stMarke);
    assert(g_minZCompensationSteps == defaultMin && g_maxZCompensationSteps == heatBedZCompensationMaxMm * zStepsPerMm);
    assert(!Printer::queuePositionZLayerGuessNew && !Printer::queuePositionZLayerCurrent && !Printer::queuePositionZLayerLast);

    g_startLineActive = true;
    planLayer(primeZ, true);
    assert(g_minZCompensationSteps == defaultMin && !Printer::queuePositionZLayerCurrent);
    g_startLineActive = false;
    planLayer(newLayer, true);
    assert(g_minZCompensationSteps == newLayer && g_maxZCompensationSteps == newLayer + fadeSpan);
    assert(Printer::queuePositionZLayerCurrent == newLayer && !Printer::queuePositionZLayerLast);

    // Re-enable SenseOffset before the first DMS poll of the new job.
    Printer::zMotion.setNominalSteps(newLayer);
    g_nSensiblePressureDigits = senseThreshold;
    sample(printingForce, 9);
    assert(!Printer::zOffset.senseMicrometers());
    sample(printingForce);
    assert(Printer::zOffset.senseMicrometers() > 0);

    // A print without M3912 can learn the model layer even with no separate Z move.
    dispatchPrintStart(3921);
    assert(Com::errorCount == 0 && g_minZCompensationSteps == defaultMin);
    const long layerWithoutPrime = 451; // 0.352 mm rounded to steps
    planLayer(layerWithoutPrime, false);
    assert(g_minZCompensationSteps == layerWithoutPrime);
    assert(Printer::queuePositionZLayerCurrent == layerWithoutPrime && !Printer::queuePositionZLayerLast);
}

void serialPrintStartBlocked() {
    const long oldMin = g_minZCompensationSteps;
    const long oldLayer = Printer::queuePositionZLayerCurrent;
    auto checkRejected = [&] {
        const int errors = Com::errorCount;
        dispatchPrintStart(3921);
        assert(Com::errorCount == errors + 1);
        assert(g_minZCompensationSteps == oldMin && Printer::queuePositionZLayerCurrent == oldLayer);
        assert(Printer::zOffset.senseMicrometers() == initialClearance);
    };

    g_startLineActive = true;
    checkRejected();
    g_startLineActive = false;
    g_nZOSScanStatus = 1;
    checkRejected();
    g_nZOSScanStatus = 0;
    Printer::blockAll = true;
    checkRejected();
    Printer::blockAll = false;

    const int lines = PrintLine::linesCount;
    Printer::pause.request(PrintPause::Destination::InPlace);
    checkRejected();
    assert(PrintLine::linesCount == lines);
    Printer::pause.cancel();

    // A DMS pause can freeze queued motion while the marker is waiting.
    pollDuringMoves = false;
    pauseDuringQueueWait = true;
    checkRejected();
    assert(Printer::pause.isSettled() && PrintLine::linesCount == lines);
    Printer::pause.cancel();

    Printer::printStop.begin(clockMs);
    checkRejected();
    assert(PrintLine::linesCount == lines);
}

void initializeOffsets() {
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
    else if (!std::strcmp(argv[1], "serial-start"))
        serialPrintStart();
    else if (!std::strcmp(argv[1], "serial-start-blocked"))
        serialPrintStartBlocked();
    else
        return 1;
    std::printf("Print lifecycle replay passed (%s)\n", argv[1]);
}

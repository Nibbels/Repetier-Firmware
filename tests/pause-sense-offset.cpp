#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>

#include "Constants.h"
#undef uint
#define MOTHERBOARD DEVICE_TYPE_RF2000
#define NUM_EXTRUDER 1
#include "Configuration.h"
#include "RF2000.h"
#include "ZOffset.h"
#include "ExtrusionCompensation.h"
#include "PressureAdvance.h"
#include "PrintLayers.h"
#include "BedCompensation.h"
#include "PrintPause.h"
#include "PrintStop.h"

// The replay compiles the live firmware routines. These shims supply DMS bytes,
// a bed profile, a clock and the motor displacements completed during each wait.
using millis_t = uint32_t;
using speed_t = uint16_t;
constexpr float zStepsPerMm = 1280.0f;
constexpr int16_t initialClearance = 20;
constexpr int16_t printingForce = 6000;
constexpr int16_t senseThreshold = 3000;
constexpr long parkY = 200 * 160L, parkZ = 30 * static_cast<long>(zStepsPerMm), parkE = 280;

millis_t clockMs = 0, g_uStartOfIdle = 0, previousMillisCmd = 0;
char g_nHeatBedScanStatus = 0, g_nZOSScanStatus = 0;
unsigned short g_nManualSteps[4] = { 1, 1, 1, 28 };
int32_t g_offsetZCompensationSteps = 250;
short g_ZCompensationMax = 200;
short forceDigits = 0;
char g_nScanPressureReads = 4;
unsigned short g_nScanPressureReadDelay = 3;
short g_nScanPressureTolerance = 10;
bool pollDuringMoves = true;
bool clipReturn[4] = {};
int incompleteReturns = 0;
int incomingReads = 0, objectOutputs = 0;
unsigned matrixDepthReads = 0;
unsigned strainGaugeReads = 0;
void (*duringMove)(long, long, long, long) = nullptr;
void (*duringHeatWait)() = nullptr;
std::function<void()> duringPeriodicActions;
unsigned primePolls = 0, primeMoves = 0, primeWaits = 0, skipPrimeAt = 0, abortPrimeAt = 0;
unsigned primeSpacingMoves = 0;
struct PrimeMove {
    float x, y, z, e, speed;
};
PrimeMove primeApproach{}, firstPrimeSegment{};
bool observedPrimeBrake = false;

void poll();

namespace HAL {
millis_t timeInMilliseconds() { return clockMs; }
speed_t ComputeV(uint32_t ticks, uint32_t acceleration) {
    return ((uint64_t(ticks >> 8) * acceleration) >> 10);
}
void delayMilliseconds(unsigned delay) { clockMs += delay; }
int16_t eepromWords[2048] = {};
int32_t eepromDWords[2048] = {};
int16_t eprGetInt16(unsigned address) { return eepromWords[address]; }
void eprSetInt16(unsigned address, int16_t value) { eepromWords[address] = value; }
void eprSetInt32(unsigned address, int32_t value) { eepromDWords[address] = value; }
}

namespace EEPROM {
void updateChecksum() {}
}

bool interruptsEnabled = true;
unsigned interruptBeforeLock = 0;
void (*onInterrupt)() = nullptr;
unsigned locksUntilCompensationPublish = 0;
float extrusionBeforePublish = 0.0f;

namespace HAL {
void allowInterrupts() { interruptsEnabled = true; }
void forbidInterrupts() { interruptsEnabled = false; }
unsigned stepHighHolds = 0;
void holdStepHigh() {
    assert(!interruptsEnabled);
    ++stepHighHolds;
}
}

namespace Printer {
extern ExtrusionCompensation extrusionCompensation;
}

class InterruptProtectedBlock {
    const bool previousState = interruptsEnabled;

public:
    InterruptProtectedBlock() {
        if (interruptsEnabled && interruptBeforeLock && --interruptBeforeLock == 0)
            onInterrupt();
        if (interruptsEnabled && locksUntilCompensationPublish && --locksUntilCompensationPublish == 0)
            extrusionBeforePublish = Printer::extrusionCompensation.ratio();
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
#include "SenseOffsetControl.h"
#include "PressureProtection.h"
#include "StrainGauge.h"
#include "StartLine.h"
#include "ExtrusionPlanner.h"
#include "MoveTarget.h"
#include "WobbleCorrection.h"
#include "CartesianPlanner.h"
#include "XYMotion.h"

#define PSTR(text) (text)
#define UI_STATUS_UPD(text) ((void)0)
#define UI_TEXT_MANUAL ""
#define UI_TEXT_LCD_CIRCUIT ""
#define UI_TEXT_RESET ""
const char ui_text_pause[] = "", ui_text_operation_denied[] = "", ui_text_home_unknown[] = "";
const char ui_text_z_axis[] = "", ui_text_max_reached[] = "", ui_text_min_reached[] = "";
const char ui_text_emergency_pause[] = "";

struct Com {
    static constexpr const char* tSlash = "/";
    static const char* lastLine;
    static void printFLN(const char* line) { lastLine = line; }
    template<class... Args> static void printF(Args...) {}
    template<class... Args> static void printFLN(Args...) {}
};

const char* Com::lastLine = nullptr;

template<class... Args> void showWarning(Args...) { ++incompleteReturns; }
template<class... Args> void showError(Args...) { assert(false); }
template<class... Args> void showInformation(Args...) { assert(false); }
unsigned pressureStops = 0;
bool expectPressureStop = false;
void doEmergencyStop(int reason) {
    assert(expectPressureStop && reason == STOP_BECAUSE_OF_Z_BLOCK);
    ++pressureStops;
    expectPressureStop = false;
}
bool isAnyScanRunning();
void setExtruderCurrent(int, int) {}

struct WireInput {
    uint8_t byteIndex = 0;
    unsigned configurations = 0;
    void beginTransmission(uint8_t address) const { assert(address == I2C_ADDRESS_STRAIN_GAUGE); }
    void write(uint8_t value) {
        assert(value == 0x8c);
        byteIndex = 3;
        ++configurations;
    }
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
extern StartLine startLine;
ZOffset zOffset;
SenseOffsetControl senseOffsetControl{SENSIBLE_PRESSURE_MAX_OFFSET};
PressureProtection pressureProtection;
StrainGauge strainGauge;
ExtrusionPlanner extrusionPlanner;
ZMotion zMotion(zStepsPerMm);
PrintLayers layers{zStepsPerMm};
DigitCompensation digitCompensation{
    short(std::abs(EMERGENCY_PAUSE_DIGITS_MAX) * 0.7), short(std::abs(EMERGENCY_PAUSE_DIGITS_MAX))
};
XYMotion xyMotion;
bool xyMinHit[2] = {}, xyMaxHit[2] = {};
int8_t stoppedDirectAxis = -1;
WobbleCorrection wobbleCorrection;
long directCurrentSteps[4] = { 0, 16, 8, 0 };
struct DirectDisplacement {
    long displacement;
    long steps() const { return displacement; }
};
DirectDisplacement directPosition(uint8_t axis) { return {directCurrentSteps[axis]}; }
CartesianPlanner cartesianPlanner;
float axisStepsPerMM[4] = { 160.0f, 160.0f, zStepsPerMm, 280.0f };
float axisMMPerSteps[3] = { 1.0f / 160, 1.0f / 160, 1.0f / zStepsPerMm };
float homingFeedrate[3] = { 60.0f, 60.0f, 5.0f };
float axisLengthMM[3] = { 170.0f, 245.0f, 200.0f };
long maxSoftEndstopSteps[2] = { 170 * 160L, 245 * 160L };
MoveTarget moveTarget;
float feedrate = 10.0f;
bool unitIsInches = false, relativeCoordinateMode = false;
float menuExtrusionFactor = 1.0f;
bool relativeExtruderCoordinateMode = false;
ExtrusionCompensation extrusionCompensation;
bool blockAll = false;
PressureAdvance pressureAdvance;
bool advanceActive = false;
volatile char doHeatBedZCompensation = true;
bool printing = true, menuPaused = false;
bool axesHomed = true;
int feedrateMultiply = 100, motorCurrent[4] = {};
char moveMode[3] = { MOVE_MODE_SINGLE_STEPS, MOVE_MODE_SINGLE_STEPS, MOVE_MODE_SINGLE_STEPS };

bool debugErrors() { return false; }
bool infoLogging = false;
bool debugInfo() { return infoLogging; }
bool isAdvanceActivated() { return advanceActive; }
bool debugDryrun() { return false; }
float convertToMM(float value) { return unitIsInches ? value * 25.4 : value; }
bool isPrinting() { return printing; }
void setPrinting(bool active) { printing = active; }
bool areAxisHomed() { return axesHomed; }
bool isAxisHomed(int) { return true; }
bool isMinEndstopHit(uint8_t axis) { return xyMinHit[axis]; }
bool isMaxEndstopHit(uint8_t axis) { return xyMaxHit[axis]; }
void stopDirectAxis(uint8_t axis) { stoppedDirectAxis = axis; }
bool isZMaxEndstopHit() { return zMotion.maxEndstopHit(); }
bool isZMinEndstopHit() { return ZMotion::minSwitchHit(); }
bool needsCMPwait() { return false; }
void setMenuMode(int mode, bool active) {
    assert(mode == MENU_MODE_PAUSED);
    menuPaused = active;
}
void stopPrint();
void homeZAxis();
void homeDigits();
void enableSenseOffsetnow();
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
void setEPositionMM(float value, float remainder = 0.0f);
void setFeedrate(float value);
bool queueGCodeCoordinates(GCode*);
void updateWobbleFixOffsets();
int32_t prepareDestinationSteps(uint8_t axis);
float getDirectMM(uint8_t axis) { return directCurrentSteps[axis] * axisMMPerSteps[axis]; }
int32_t softwareEndstopSteps(uint8_t axis) {
    return axis == Z_AXIS ? zMotion.maxSteps() : maxSoftEndstopSteps[axis];
}
void queueFloatCoordinates(float x, float y, float z, float e, float speed) {
    assert(startLine.active() && !printStop.commandsBlocked());
    ++primeMoves;
    if (primeMoves == 1)
        primeApproach = {x, y, z, e, speed};
    if (primeMoves == 2)
        firstPrimeSegment = {x, y, z, e, speed};
    if (x == IGNORE_COORDINATE && y != IGNORE_COORDINATE && e == IGNORE_COORDINATE)
        ++primeSpacingMoves;
    if (z != IGNORE_COORDINATE)
        zMotion.setNominalSteps(std::lround(z * axisStepsPerMM[Z_AXIS]));
    if (e != IGNORE_COORDINATE) {
        setEPositionMM(e);
        extrusionPlanner.restoreStepRemainder(0.375f);
        extrusionCompensation.restoreRemainder(0.25f);
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
float max(float a, float b) { return std::max(a, b); }
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
    static unsigned stepPulses, trailingEdges;
    static void setDirection(bool forward) { current->stepperDirection = forward ? 1 : -1; }
    static void step() { assert(!interruptsEnabled); ++stepPulses; }
    static void unstep() { assert(!interruptsEnabled); ++trailingEdges; }
    static void setTemperatureForExtruder(float temperature, int tool);
    static void setHeatedBedTemperature(float temperature) { bedTemperature = temperature; }
    static void setTemperatureForAllExtruders(float temperature) { setTemperatureForExtruder(temperature, 0); }
    static float bedTemperature;
};

Extruder extruder[NUM_EXTRUDER];
Extruder* Extruder::current = extruder;
unsigned Extruder::stepPulses = 0, Extruder::trailingEdges = 0;
float Extruder::bedTemperature = 60.0f;

void Extruder::setTemperatureForExtruder(float temperature, int tool) {
    extruder[tool].tempControl.targetTemperatureC = temperature;
    extruder[tool].tempControl.paused = 0;
}

class MoveQueue;
struct PrintLine {
#include "step-clock-firmware.inc"

    bool moveZ = false;
    bool positiveE = false;
    bool moveXY = false;
    bool moveE = false;
    uint8_t xyDirection = 0;
    bool abortAtEndstops = false;
    bool isAbortAtEndstops() const { return abortAtEndstops; }
    bool isPositiveMoveOfAxis(uint8_t axis) const {
        uint8_t mask = 17 << axis;
        return (xyDirection & mask) == mask;
    }
    bool isNegativeMoveOfAxis(uint8_t axis) const {
        return (xyDirection & (17 << axis)) == (16 << axis);
    }
    void setMoveOfAxisFinished(uint8_t axis) { xyDirection &= ~(16 << axis); }
    void setMoveOfAxis(uint8_t axis) { xyDirection |= 16 << axis; }
    void setPositiveDirectionForAxis(uint8_t axis) { xyDirection |= 1 << axis; }
    int32_t error[4] = {}, delta[4] = {};
    int32_t stepsRemaining = 0;
    uint32_t accelSteps = 0, decelSteps = 0, fAcceleration = 0;
    speed_t vStart = 0, vMax = 0, vEnd = 0;
    speed_t nextStepSpeed(StepClock& clock, uint8_t pulses, uint32_t interval);
    float distance = 0;
    uint8_t primaryAxis = 0;
    char task = TASK_NO_TASK;
    uint8_t flags = 0, joinFlags = 0, dir = 0;
    uint8_t warmupDepth = 0;
    long warmupTicks = 0;
    void setWaitForXLinesFilled(uint8_t depth) { warmupDepth = depth; }
    void setWaitTicks(long ticks) { warmupTicks = ticks; }
    void setAxisSteps(uint8_t axis, int32_t steps);
    void setSegmentGeometry(const float axisDistanceMM[]);
    bool isEMove() const { return moveE; }
    bool isZMove() const { return moveZ || (xyDirection & (16 << Z_AXIS)); }
    bool isXYZMove() const { return xyDirection & 112; }
    bool isEPositiveMove() const { return positiveE; }
    bool isXOrYMove() const { return moveXY; }

    static struct Direct {
        int task = TASK_NO_TASK;
        int32_t stepsRemaining = 0;
        bool moveZ = false;
        bool isZMove() const { return moveZ; }
    } direct;
    static MoveQueue queue;
    static uint8_t insertWaitMovesIfNeeded(uint8_t pathOptimize, uint8_t waitExtraLines);
    static int linesCount;
    static int queuedTask, stopRequests;
    static bool hasLines() { return linesCount; }
    static bool directMoveActive() { return direct.task != TASK_NO_TASK; }
    static bool hasActiveZMove();
    static int queuedMoveCount() { return linesCount; }
    static bool hasStoppableDirectMove();
    static bool hasQueuedOvertravel() { return false; }
    static unsigned plannedMoves;
    static bool updatesWobble;
    static void prepareQueueMove(bool, bool, float, bool wobble) {
        ++plannedMoves;
        updatesWobble = wobble;
    }
    static void stopDirectMove() { ++stopRequests; }
    static void discardQueuedMoves() { linesCount = 0; queuedTask = TASK_NO_TASK; }
    static void queueTask(int task) {
        assert(!interruptsEnabled && linesCount < MOVE_CACHE_SIZE);
        queuedTask = task;
        ++linesCount;
    }
};

PrintLine::Direct PrintLine::direct;
int PrintLine::linesCount = 3;
int PrintLine::queuedTask = TASK_NO_TASK;
int PrintLine::stopRequests = 0;
unsigned PrintLine::plannedMoves = 0;
bool PrintLine::updatesWobble = false;
const int32_t queueError = 1, directError = 1;

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
    void senseOffsetUpAction();
    void senseOffsetDownAction();
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
    float E = 0, F = 0, I = 1, X = 0, Y = 0, Z = 0;
    bool explicitP = false, explicitS = false;
    bool hasS() const { return explicitS || S != 0; }
    bool hasP() const { return explicitP || P != 0; }
    bool hasE() const { return E != 0; }
    bool hasF() const { return F != 0; }
    bool hasI() const { return I != 0; }
    bool hasX() const { return X != 0; }
    bool hasY() const { return Y != 0; }
    bool hasZ() const { return Z != 0; }
    bool hasNoXYZ() const { return !hasX() && !hasY() && !hasZ(); }

    static void readFromSerial() {
        assert(Printer::printStop.commandsBlocked());
        ++incomingReads;
    }
};

unsigned queuedSenseOffsetEnables = 0;
void queueTask(char task) {
    assert(task == TASK_ENABLE_SENSE_OFFSET);
    ++queuedSenseOffsetEnables;
}

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

void withStartLine(const std::function<void()>& action) {
    assert(!duringPeriodicActions);
    skipPrimeAt = primePolls + 1;
    duringPeriodicActions = [&] {
        ++primePolls;
        action();
    };
    GCode command;
    Printer::startLine.run(command);
    duringPeriodicActions = nullptr;
    skipPrimeAt = 0;
}

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

void digitZero() {
    auto& gauge = Printer::strainGauge;
    gauge.begin();
    assert(Wire.configurations == 1);
    assert(gauge.zeroingEnabled());
    forceDigits = 1000;
    Printer::homeDigits();
    assert(gauge.zeroOffset() == 1000);
    assert(strainGaugeReads == 4 && clockMs == 12);
    assert(Printer::digitCompensation.latestDigits() == 1000);
    assert(sampleStrainGauge() == 0);

    forceDigits = -1200;
    Printer::homeDigits();
    assert(gauge.zeroOffset() == -1200); // Fresh reference, even with a prior zero.
    assert(sampleStrainGauge() == 0);
    forceDigits = -27767;
    assert(sampleStrainGauge() == -26567);
    forceDigits = -27768;
    assert(sampleStrainGauge() == -27768);
    forceDigits = 27766;
    assert(sampleStrainGauge() == 28966);
    forceDigits = 27767;
    assert(sampleStrainGauge() == 27767);

    gauge.setZeroingEnabled(false);
    forceDigits = 2400;
    const unsigned readsBeforeZero = strainGaugeReads;
    Printer::homeDigits();
    assert(strainGaugeReads == readsBeforeZero && gauge.zeroOffset() == -1200);
    assert(gauge.effectiveZeroOffset() == 0);
    assert(sampleStrainGauge() == 2400);
    gauge.setZeroingEnabled(true);
    assert(gauge.effectiveZeroOffset() == -1200);
    assert(sampleStrainGauge() == 3600);

    forceDigits = 5000;
    Printer::homeDigits();
    assert(!gauge.zeroOffset() && sampleStrainGauge() == 5000);
}

void pressurePause() {
    GCode command;
    command.S = -99999;
    command.P = 99999;
    configurePressure(3075, &command);
    assert(HAL::eepromDWords[EPR_RF_EMERGENCYPAUSEDIGITSMIN] == EMERGENCY_PAUSE_DIGITS_MIN);
    assert(HAL::eepromDWords[EPR_RF_EMERGENCYPAUSEDIGITSMAX] == EMERGENCY_PAUSE_DIGITS_MAX);
    sample(EMERGENCY_PAUSE_DIGITS_MAX, EMERGENCY_PAUSE_CHECKS);
    assert(!Printer::pause.isPaused());

    command.explicitS = command.explicitP = true;
    command.S = command.P = 0;
    configurePressure(3075, &command);
    assert(!HAL::eepromDWords[EPR_RF_EMERGENCYPAUSEDIGITSMIN] && !HAL::eepromDWords[EPR_RF_EMERGENCYPAUSEDIGITSMAX]);
    sample(EMERGENCY_PAUSE_DIGITS_MAX + 1, EMERGENCY_PAUSE_CHECKS);
    assert(!Printer::pause.isPaused());

    command.S = -7000;
    command.P = 7000;
    configurePressure(3075, &command);
    sample(-7500, EMERGENCY_PAUSE_CHECKS - 1);
    assert(!Printer::pause.isPaused());
    Printer::axesHomed = false;
    sample(-7500);
    Printer::axesHomed = true;
    sample(-7500, EMERGENCY_PAUSE_CHECKS - 1);
    assert(!Printer::pause.isPaused());
    sample(-7500);
    assert(Printer::pause.isPaused() && !Printer::pause.isSettled());
}

void pressureStop() {
    GCode command;
    command.S = -12000;
    command.P = 11500;
    configurePressure(3076, &command);
    assert(HAL::eprGetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMIN) == -12000);
    assert(HAL::eprGetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMAX) == 11500);

    PrintLine::direct.task = DIRECT_RUNNING;
    PrintLine::direct.stepsRemaining = 1;
    PrintLine::direct.moveZ = true;
    Extruder::current->stepperDirection = 1;
    forceDigits = 11501;
    auto advance = [](millis_t elapsed) {
        clockMs += elapsed;
        handleStrainGaugeFeatures(clockMs);
    };
    advance(EMERGENCY_STOP_INTERVAL);
    assert(strainGaugeReads == 0);
    advance(1);
    advance(EMERGENCY_STOP_INTERVAL);
    assert(strainGaugeReads == 1);
    advance(1);
    assert(!Printer::pressureProtection.extrusionBlocked());
    advance(EMERGENCY_STOP_INTERVAL + 1);
    assert(Printer::pressureProtection.extrusionBlocked() && !pressureStops);

    sample(11500, EMERGENCY_STOP_CHECKS);
    assert(!Printer::pressureProtection.extrusionBlocked());
    Extruder::current->stepperDirection = 0;
    sample(-12001, EMERGENCY_STOP_CHECKS - 1);
    assert(!Printer::pressureProtection.extrusionBlocked());
    expectPressureStop = true;
    sample(-12001);
    assert(Printer::pressureProtection.extrusionBlocked() && pressureStops == 1);

    PrintLine::direct.task = TASK_NO_TASK;
    PrintLine::direct.stepsRemaining = 0;
    command.explicitS = command.explicitP = true;
    command.S = command.P = 0;
    Printer::infoLogging = true;
    configurePressure(3076, &command);
    Printer::infoLogging = false;
    assert(Com::lastLine && !std::strcmp(Com::lastLine, "M3076: temporary stop band [0, 0] digits"));
    assert(HAL::eprGetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMIN) == -12000);
    assert(HAL::eprGetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMAX) == 11500);
    sample(0, EMERGENCY_STOP_CHECKS);
    assert(!Printer::pressureProtection.extrusionBlocked());
    // The temporary zero band still rejects any nonzero average.
    sample(1, EMERGENCY_STOP_CHECKS);
    assert(Printer::pressureProtection.extrusionBlocked());
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
        if ((clipReturn[E_AXIS] && e > 0) || Printer::pressureProtection.extrusionBlocked()) e = 0;
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
    assert(Printer::zOffset.senseMicrometers() == initialClearance && Printer::senseOffsetControl.inLearningRange());
    const long firstTarget = Printer::zMotion.correctionTarget();
    const long originalY = Printer::directCurrentSteps[Y_AXIS];
    const long originalZ = Printer::directCurrentSteps[Z_AXIS];

    Printer::pause.request(PrintPause::Destination::Park);
    assert(Printer::pause.isPaused() && !Printer::pause.isSettled());
    sample(0);
    assert(Printer::zOffset.senseMicrometers() == initialClearance && !Printer::senseOffsetControl.inLearningRange());
    Printer::pause.serviceMotion();
    assert(Printer::pause.isSettled());
    assert(Printer::directCurrentSteps[Y_AXIS] == originalY + parkY);
    assert(Printer::directCurrentSteps[Z_AXIS] == originalZ + parkZ);

    sample(0, 40);
    const long zeroForceTarget = Printer::zMotion.correctionTarget();
    const long printMatrix = getZMatrixDepth(Printer::xyMotion.nominalSteps(X_AXIS), Printer::xyMotion.nominalSteps(Y_AXIS));
    assert(zeroForceTarget == printMatrix + Printer::zOffset.effectiveSteps());

    sample(printingForce, 40);
    assert(Printer::zMotion.correctionTarget() > zeroForceTarget);
    assert(Printer::zOffset.senseMicrometers() == initialClearance);
    const long parkedZ = Printer::directCurrentSteps[Z_AXIS];
    moveZAction(-1);
    assert(Printer::directCurrentSteps[Z_AXIS] == parkedZ - static_cast<long>(zStepsPerMm));

    sample(EMERGENCY_STOP_DIGITS_MAX + 1, 2 * EMERGENCY_STOP_CHECKS);
    assert(Printer::pressureProtection.extrusionBlocked());
    sample(printingForce, 50);
    assert(!Printer::pressureProtection.extrusionBlocked() && Printer::pause.isWarmPause());
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
    onInterrupt = requestInterruptPause;
    interruptBeforeLock = 5;
    sample(printingForce);
    assert(!interruptBeforeLock && Printer::pause.isPaused() && !Printer::pause.isSettled());
    assert(Printer::zOffset.senseMicrometers() == initialClearance);
}

void senseOffsetWindow() {
    const auto& offset = Printer::zOffset;
    const auto& controller = Printer::senseOffsetControl;
    forceDigits = printingForce;
    clockMs += SENSIBLE_PRESSURE_INTERVAL;
    handleStrainGaugeFeatures(clockMs);
    assert(!controller.inLearningRange());

    sample(printingForce, 9);
    assert(offset.senseMicrometers() == 20 && controller.inLearningRange());
    sample(printingForce);
    assert(offset.senseMicrometers() == 33);
    sample(printingForce, 4);
    assert(offset.senseMicrometers() == 33);
    sample(printingForce);
    assert(offset.senseMicrometers() == 46);
    sample(printingForce, 5);
    assert(offset.senseMicrometers() == 56); // Taper once above a quarter of the cap.
    sample(3300, 5);
    assert(offset.senseMicrometers() == 57); // A falling average adds one micrometre.

    Printer::zMotion.setNominalSteps(257);
    sample(printingForce);
    assert(offset.senseMicrometers() == 57 && !controller.inLearningRange());
    Printer::zMotion.setNominalSteps(256);
    sample(printingForce, 9);
    assert(offset.senseMicrometers() == 57);
    sample(printingForce);
    assert(offset.senseMicrometers() == 67);

    withStartLine([&] {
        Printer::zMotion.setNominalSteps(256);
        sample(printingForce, 12);
        assert(offset.senseMicrometers() == 67 && !controller.inLearningRange());
    });
    sample(-3150, 10);
    assert(offset.senseMicrometers() == 67 && controller.inLearningRange());
    sample(-9000, 5);
    assert(offset.senseMicrometers() == 76);

    // The legacy window clears positive sums above the first layer.
    Printer::zMotion.setNominalSteps(257);
    sample(-9000);
    assert(offset.senseMicrometers() == 76 && !controller.inLearningRange());
    Printer::zMotion.setNominalSteps(256);
    sample(-9000, 4);
    assert(offset.senseMicrometers() == 76);
    sample(-9000);
    assert(offset.senseMicrometers() == 88);
}

void senseOffsetSettings() {
    const auto& controller = Printer::senseOffsetControl;
    GCode command;
    command.P = 3200;
    command.S = 55;
    configureSenseOffset(&command);
    assert(queuedSenseOffsetEnables == 1 && controller.limits().thresholdDigits == senseThreshold);
    assert(controller.limits().maxMicrometers == 55);
    assert(HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS) == 3200);
    assert(HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX) == 55);
    Printer::enableSenseOffsetnow();
    assert(controller.limits().thresholdDigits == 3200);

    // The queue accepts only stored thresholds strictly inside its bounds.
    HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS, EMERGENCY_PAUSE_DIGITS_MAX);
    HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX, 300);
    Printer::enableSenseOffsetnow();
    const auto limits = controller.limits();
    assert(limits.thresholdDigits == 3200 && limits.maxMicrometers == 55);
    HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS, 3200);
    HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX, 55);

    command.explicitP = true;
    command.P = 0;
    command.S = 0;
    configureSenseOffset(&command);
    assert(queuedSenseOffsetEnables == 1 && !controller.limits().thresholdDigits);
    assert(HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS) == 3200);
    sample(printingForce, 10);
    assert(Printer::zOffset.senseMicrometers() == initialClearance);

    Printer::doHeatBedZCompensation = false;
    sample(printingForce);
    assert(!Printer::zOffset.senseMicrometers() && Printer::zOffset.manualMicrometers() == 50);

    // The Mod Menu wraps through its endpoints without overwriting the saved threshold.
    uid.senseOffsetDownAction();
    assert(controller.limits().thresholdDigits == EMERGENCY_PAUSE_DIGITS_MAX * 0.8);
    uid.senseOffsetUpAction();
    assert(controller.limits().thresholdDigits == 0);
    assert(HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS) == 3200);
    for (int i = 0; i < 10; ++i)
        uid.senseOffsetUpAction();
    assert(controller.limits().thresholdDigits == 500);
    assert(HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS) == 500);
    uid.senseOffsetDownAction();
    assert(controller.limits().thresholdDigits == 400);
    assert(HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS) == 500);
}

void senseOffsetLimitInterrupt() {
    sample(printingForce, 9);
    // Activate a tighter cap from the step queue just before the increment write.
    HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS, 4500);
    HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX, 21);
    onInterrupt = Printer::enableSenseOffsetnow;
    interruptBeforeLock = 5;
    sample(printingForce);
    assert(!interruptBeforeLock && Printer::senseOffsetControl.limits().maxMicrometers == 21);
    assert(Printer::zOffset.senseMicrometers() == initialClearance);
    sample(printingForce, 5);
    assert(Printer::zOffset.senseMicrometers() == 21);
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
    Printer::cartesianPlanner.setAxisSteps(Z_AXIS, z);
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
    Printer::extrusionCompensation.publishRatio(0.5f);
    Printer::extrusionCompensation.restoreRemainder(0.75f);

    Printer::homeZAxis();
    assert(!Printer::doHeatBedZCompensation && Printer::zMotion.isHomed());
    assert(!Printer::zMotion.nominalSteps() && !Printer::zMotion.motorSteps());
    assert(Printer::zOffset.manualMicrometers() == manualZ && Printer::zOffset.senseMicrometers() == 0);
    assert(Printer::zMotion.correctionTarget() == 0);
    assert(Printer::extrusionCompensation.ratio() == 0.0f && Printer::extrusionCompensation.remainder() == 0.0f);
    assert(Printer::senseOffsetControl.limits().thresholdDigits == 0 && !Printer::senseOffsetControl.inLearningRange());
    assert(layers.fadeStartSteps() == defaultMin && layers.fadeEndSteps() == 12800);
    planMove(0, false, true, false); // An E-only move must not confirm the pre-home candidate.
    assert(!layers.currentLayerSteps() && !layers.previousLayerSteps());

    Printer::doHeatBedZCompensation = true;
    withStartLine([&] {
        planMove(primeZ, true);
        assert(layers.fadeStartSteps() == defaultMin && !layers.currentLayerSteps());
    });
    planMove(newLayer, true);
    assert(layers.fadeStartSteps() == newLayer && layers.fadeEndSteps() == newLayer + fadeSpan);
    assert(layers.currentLayerSteps() == newLayer && !layers.previousLayerSteps());

    // Re-enable SenseOffset before the first DMS poll of the new job.
    Printer::zMotion.setNominalSteps(newLayer);
    Printer::senseOffsetControl.setThresholdDigits(senseThreshold);
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
    assert(std::abs(Printer::extrusionCompensation.ratio() - extraExtrusion) < 0.000001f);
    assert(matrixDepthReads - readsBefore == expectedReads);
}

void layerFadeAndZHop() {
    auto& layers = Printer::layers;
    const long primeZ = 448; // 0.35 mm
    const long fadeEnd = primeZ + 1000; // 50 steps of matrix relief.
    layers.resetTracking();
    Printer::xyMotion.setNominalSteps(X_AXIS, 5280); // Interpolated bed depth of 202 steps.
    Printer::xyMotion.setNominalSteps(Y_AXIS, 0);
    Printer::digitCompensation.setDigitZEnabled(false);
    const long offset = Printer::zOffset.effectiveSteps();

    withStartLine([&] {
        planMove(primeZ, true);
        assert(!layers.currentLayerSteps());
    });

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
    Printer::xyMotion.setNominalSteps(X_AXIS, 5280); // Interpolated bed depth of 202 steps.
    Printer::xyMotion.setNominalSteps(Y_AXIS, 0);
    Printer::digitCompensation.setDigitZEnabled(false);
    const long offset = Printer::zOffset.effectiveSteps();
    const long fadeStart = layers.fadeStartSteps();
    const long fadeEnd = layers.fadeEndSteps();

    const float previousRatio = ExtrusionCompensation::prepareRatio(0.048f);
    Printer::extrusionCompensation.publishRatio(previousRatio);
    locksUntilCompensationPublish = 2; // One read of nominal Z, then publication.
    expectCompensation(-1, 263 + offset, 0.0f, 0);
    assert(!locksUntilCompensationPublish && extrusionBeforePublish == previousRatio);
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

    // 5,530 digits sits just above the transition from 53 to 54 microsteps.
    for (short sign : {1, -1}) {
        for (short startingMagnitude : {5000, 6000}) {
            settleDigitZ(sign * startingMagnitude);
            settleDigitZ(sign * 5530);
            assert(Printer::digitCompensation.filteredDigits() == sign * 5530);
            expectCompensation(256, 202 + offset + 54, 0.0f, 1);
        }
    }

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
    Printer::xyMotion.setNominalSteps(X_AXIS, 6816); // Interpolated bed depth of 250 steps.
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

    readAt(73, -3000, 1);
    check(2062, 0.8595f, 0.97f, 1055, 58);

    Printer::zMotion.setNominalSteps(256);
    Printer::doHeatBedZCompensation = false;
    readAt(84, 5000, 1);
    assert(compensation.latestDigits() == 5000);
    check(2062, 0.8595f, 0.97f, 0, 0);

    Printer::doHeatBedZCompensation = true;
    readAt(95, 4500, 1);
    check(2672, 1, 0.98f, 1044, 63);
    readAt(101, 4500, 1);
    check(3129, 1, 0.99f, 1034, 67);

    assert(Printer::strainGauge.setZeroFromAverage(1000));
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
    Printer::extrusionPlanner.restoreStepRemainder(0.125f);
    Printer::extrusionCompensation.restoreRemainder(0.75f);
    Printer::relativeExtruderCoordinateMode = true;
    Printer::feedrateMultiply = 85;
    Printer::menuExtrusionFactor = 0.93f;
    skipPrimeAt = skipAt;
    abortPrimeAt = abortAt;
    duringPeriodicActions = [] {
        assert(Printer::startLine.active());
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
    Printer::startLine.run(command);
    duringPeriodicActions = nullptr;

    assert(observedPrimeBrake && primeWaits == 2);
    assert((skipAt || abortAt) ? primeMoves == (skipAt ? skipAt : abortAt) : primeMoves > 300);
    const auto profile = compensation.flowProfile();
    assert(profile.minDigits == 2000 && profile.maxDigits == 6100);
    assert(profile.extrusionPercent == 21 && profile.speedPercent == -30);
    assert(compensation.extrusionFactor() == 1 && compensation.feedrateFactor() == 1 && !compensation.intervalScale());
    assert(!Printer::startLine.active() && Printer::relativeExtruderCoordinateMode);
    assert(Printer::feedrateMultiply == 85 && Printer::menuExtrusionFactor == 0.93f);
    if (abortAt) {
        assert(Printer::printStop.commandsBlocked() && Printer::moveTarget.machineMM(E_AXIS) == 0.0f);
    } else {
        assert(Printer::moveTarget.machineMM(E_AXIS) == 10.0f && Printer::moveTarget.remainderMM(E_AXIS) == 0.00001f);
        assert(Printer::extrusionPlanner.stepRemainder() == 0.125f && Printer::extrusionCompensation.remainder() == 0.75f);
    }
}

void primeParameters() {
    GCode command;
    command.S = 6000;
    command.P = 2000;
    command.E = 1;
    command.F = 100;
    command.I = 9;
    command.Y = 500;

    Printer::axesHomed = false;
    Printer::startLine.run(command);
    assert(!primeMoves && primeWaits == 1);
    Printer::axesHomed = true;
    Printer::doHeatBedZCompensation = false;
    Printer::startLine.run(command);
    assert(!primeMoves && primeWaits == 2);
    Printer::doHeatBedZCompensation = true;

    duringPeriodicActions = [] {
        const auto profile = Printer::digitCompensation.flowProfile();
        assert(profile.minDigits == 2000 && profile.maxDigits == 6000);
    };
    Printer::startLine.run(command);
    duringPeriodicActions = nullptr;
    assert(primeWaits == 4 && primeSpacingMoves == 5);
    assert(primeApproach.x == 15.0f && primeApproach.y == 122.5f && primeApproach.z == 0.35f);
    assert(primeApproach.e == IGNORE_COORDINATE && primeApproach.speed == 60.0f);
    assert(firstPrimeSegment.speed == 50.0f && firstPrimeSegment.y == 122.5f);
    assert(std::abs(firstPrimeSegment.x - 15.35f) < 0.00001f);
    assert(std::abs(firstPrimeSegment.e - 0.0175f) < 0.00001f);
}

void extrusionPlanning() {
    Printer::axisStepsPerMM[E_AXIS] = 1.0f;
    Printer::setEPositionMM(0);
    Printer::moveTarget.addMM(E_AXIS, 0.5f);
    assert(planExtrusionMove() == 1 && Printer::extrusionPlanner.stepRemainder() == -0.5f);

    // G92 rebases the logical position while retaining the motor-step carry.
    Printer::setEPositionMM(10.0f, 0.00001f);
    assert(planExtrusionMove() == 0 && Printer::extrusionPlanner.stepRemainder() == -0.5f);
    Printer::moveTarget.addMM(E_AXIS, 0.5f);
    assert(planExtrusionMove() == 0 && Printer::extrusionPlanner.stepRemainder() == 0.0f);

    Printer::menuExtrusionFactor = 0.5f;
    Printer::moveTarget.addMM(E_AXIS, 1.0f);
    assert(planExtrusionMove() == 1);
    Printer::moveTarget.addMM(E_AXIS, -1.0f);
    assert(planExtrusionMove() == -1 && Printer::extrusionPlanner.stepRemainder() == 0.0f);

    Printer::setEPositionMM(0);
    Printer::menuExtrusionFactor = 1.0f;
    Printer::axisStepsPerMM[E_AXIS] = 280.0f;
    int32_t steps = 0;
    for (int i = 0; i < 100000; ++i) {
        Printer::moveTarget.addMM(E_AXIS, 0.008f);
        steps += planExtrusionMove();
    }
    assert(steps == 224000);
}

void gcodeTarget() {
    Printer::moveTarget.setWorkOffsetMM(X_AXIS, 11);
    Printer::moveTarget.setWorkOffsetMM(Y_AXIS, -3);
    GCode command;
    command.X = 21;
    command.Y = 37;
    command.E = 2;
    command.F = 1200;
    assert(Printer::queueGCodeCoordinates(&command));
    assert(Printer::moveTarget.machineMM(X_AXIS) == 10 && Printer::moveTarget.machineMM(Y_AXIS) == 40);
    assert(Printer::moveTarget.machineMM(E_AXIS) == 2 && PrintLine::updatesWobble);
    assert(std::abs(Printer::feedrate - 20) < 0.00001f);

    Printer::relativeCoordinateMode = true;
    command.X = 0;
    command.Y = 0.25f;
    command.E = 1;
    assert(Printer::queueGCodeCoordinates(&command));
    assert(Printer::moveTarget.machineMM(X_AXIS) == 10 && Printer::moveTarget.machineMM(Y_AXIS) == 40.25f);
    assert(Printer::moveTarget.machineMM(E_AXIS) == 3);

    Printer::relativeCoordinateMode = false;
    Printer::relativeExtruderCoordinateMode = true;
    Printer::unitIsInches = true;
    command.X = 0.5f;
    command.Y = command.E = command.F = 0;
    assert(Printer::queueGCodeCoordinates(&command));
    assert(std::abs(Printer::moveTarget.machineMM(X_AXIS) - 1.7f) < 0.00001f);
    assert(Printer::moveTarget.machineMM(Y_AXIS) == 40.25f);

    Printer::unitIsInches = false;
    command.X = 0;
    command.E = -0.5f;
    assert(Printer::queueGCodeCoordinates(&command));
    assert(!PrintLine::updatesWobble && Printer::moveTarget.machineMM(E_AXIS) == 2.5f);
    command.E = 0;
    assert(!Printer::queueGCodeCoordinates(&command));
    assert(PrintLine::plannedMoves == 4);
}

void wobbleTarget() {
    auto& target = Printer::moveTarget;
    Printer::zMotion.setMotorReference(0);
    Printer::wobbleCorrection.setProfile({0, 100, 0, 0});
    target.setMachineMM(X_AXIS, 10.0f);
    for (unsigned i = 0; i < 50; ++i) {
        target.setMachineMM(Y_AXIS, float(i));
        Printer::updateWobbleFixOffsets();
        assert(Printer::prepareDestinationSteps(X_AXIS) == 1616);
        assert(target.machineMM(X_AXIS) == 10.0f);
    }

    Printer::wobbleCorrection.setProfile({0, 100, 100, 300});
    Printer::xyMotion.setMotorSteps(X_AXIS, Printer::maxSoftEndstopSteps[X_AXIS] / 2);
    Printer::zMotion.setMotorReference(1600); // A quarter turn of the 5 mm lead screw.
    Printer::updateWobbleFixOffsets();
    assert(std::abs(Printer::wobbleCorrection.offsetMM(Y_AXIS) - 0.2f) < 0.00001f);

    Printer::zMotion.setMotorReference(0);
    Printer::wobbleCorrection.setProfile({0, 200, 100, 300});
    Printer::updateWobbleFixOffsets();
    target.setMachineMM(X_AXIS, 169.98f);
    assert(Printer::prepareDestinationSteps(X_AXIS) == 27216);
    assert(target.machineMM(X_AXIS) == 169.98f);
    assert(std::abs(Printer::wobbleCorrection.offsetMM(X_AXIS) - 0.12f) < 0.00002f);
    assert(Printer::prepareDestinationSteps(X_AXIS) == 27216);

    target.setMachineMM(X_AXIS, 175.0f, 0.00001f);
    assert(Printer::prepareDestinationSteps(X_AXIS) == 27216);
    assert(target.machineMM(X_AXIS) == 170.1f && target.remainderMM(X_AXIS) == 0.0f);

    Printer::wobbleCorrection.setProfile({});
    Printer::updateWobbleFixOffsets();
    assert(!Printer::wobbleCorrection.offsetMM(X_AXIS) && !Printer::wobbleCorrection.offsetMM(Y_AXIS));
}

void cartesianPlanning() {
    int32_t steps = 0;
    for (int i = 0; i < 100000; ++i) {
        Printer::moveTarget.addMM(X_AXIS, 0.0005f);
        steps += planCartesianAxis(X_AXIS);
    }
    assert(steps == 8000);

    Printer::axisStepsPerMM[X_AXIS] = 320.0f;
    rescalePlannedAxis(X_AXIS);
    assert(planCartesianAxis(X_AXIS) == 0);
    Printer::moveTarget.setMachineMM(X_AXIS, 0.0f);
    assert(planCartesianAxis(X_AXIS) == -16000);
}

void ecmpStepping() {
    PrintLine move;
    move.moveE = move.positiveE = true;
    move.delta[E_AXIS] = 1;
    Printer::extrusionCompensation.publishRatio(0.25f);

    for (unsigned i = 0; i < 3; ++i)
        executeExtrusionStep(&move, true);
    assert(Extruder::stepPulses == 3 && Printer::extrusionCompensation.remainder() == 0.75f);
    executeExtrusionStep(&move, false);
    assert(Extruder::stepPulses == 4 && Printer::extrusionCompensation.remainder() == 0.75f);
    executeExtrusionStep(&move, true);
    assert(Extruder::stepPulses == 6 && Extruder::trailingEdges == 1);
    assert(Printer::extrusionCompensation.remainder() == 0.0f);

    auto replayReversal = [&](bool advance, float ratio, float remainder, unsigned count, int expectedSteps) {
        Printer::advanceActive = advance;
        Printer::pressureAdvance.clearSteps();
        Printer::extrusionCompensation.publishRatio(ExtrusionCompensation::prepareRatio(ratio));
        Printer::extrusionCompensation.restoreRemainder(remainder);
        const unsigned before = Extruder::stepPulses;

        move.positiveE = false;
        for (unsigned i = 0; i < count; ++i)
            executeExtrusionStep(&move, true);
        const unsigned afterRetract = Extruder::stepPulses;
        const int retracted = advance ? -Printer::pressureAdvance.pendingSteps() : int(afterRetract - before);

        move.positiveE = true;
        for (unsigned i = 0; i < count; ++i)
            executeExtrusionStep(&move, true);
        const int returned = advance ? Printer::pressureAdvance.pendingSteps() + retracted
                                      : int(Extruder::stepPulses - afterRetract);
        assert(retracted == expectedSteps && returned == retracted);
        assert(Printer::extrusionCompensation.remainder() == remainder);
        assert(Printer::pressureAdvance.pendingSteps() == 0);
        if (advance)
            assert(Extruder::stepPulses == before);
    };

    for (bool advance : {false, true}) {
        replayReversal(advance, 0.25f, 0.75f, 1, 1); // Original regression.
        replayReversal(advance, 0.25f, 0.125f, 1, 2); // Crossing the whole-step boundary.
        replayReversal(advance, 0.048f, 0.0f, 100, 105); // Non-binary ECMP rate.
    }
}

void advanceTimer() {
    PressureAdvance advance;
    const unsigned basePulses = Extruder::stepPulses;
    InterruptProtectedBlock lock;
    advance.setStepInterval(12);
    advance.addSteps(1);
    assert(advance.nextTimerCompare(0) == 12 && advance.pendingSteps() == 1);
    assert(Extruder::stepPulses == basePulses && HAL::stepHighHolds == 0);
    advance.addSteps(1);
    assert(advance.nextTimerCompare(12) == 64 && Extruder::current->stepperDirection == 1);
    assert(advance.nextTimerCompare(64) == 76 && advance.pendingSteps() == 1);
    assert(advance.nextTimerCompare(76) == 88 && advance.pendingSteps() == 0);
    assert(Extruder::stepPulses == basePulses + 2);

    advance.setLeadSteps(3);
    advance.setLeadSteps(5);
    advance.postponeRelaxation();
    for (int tick = 0; tick < 70; ++tick) {
        assert(!advance.relaxDelayElapsed());
        advance.onIdleTick();
        assert(advance.pendingSteps() == 5);
    }
    assert(advance.relaxDelayElapsed());
    advance.onIdleTick();
    assert(advance.pendingSteps() == 0);

    advance.addSteps(-2);
    assert(advance.nextTimerCompare(240) == 36 && Extruder::current->stepperDirection == -1);
    assert(advance.nextTimerCompare(36) == 48 && advance.pendingSteps() == -1);
    assert(advance.nextTimerCompare(48) == 60 && advance.pendingSteps() == 0);
    assert(Extruder::stepPulses == basePulses + 4 && HAL::stepHighHolds == 4);
}

void stepClock() {
    PrintLine move;
    PrintLine::StepClock queued, direct;
    queued.start(100);
    direct.start(125);
    move.accelSteps = 2;
    move.decelSteps = 3;
    move.stepsRemaining = 90;
    move.fAcceleration = 1024;
    move.vStart = 100;
    move.vMax = 300;
    move.vEnd = 75;
    constexpr uint32_t interval = 150 * 256;
    assert(move.nextStepSpeed(queued, 1, interval) == 250);
    assert(move.nextStepSpeed(queued, 1, interval) == 300);
    assert(move.nextStepSpeed(queued, 1, interval) == 300);
    assert(direct.steps == 0 && direct.ticks == 0 && direct.peak == 125);

    move.stepsRemaining = 3;
    assert(move.nextStepSpeed(queued, 1, interval) == 300);
    assert((move.flags & FLAG_DECELERATING) && queued.ticks == 0);
    assert(move.nextStepSpeed(queued, 1, interval) == 150);
    assert(move.nextStepSpeed(queued, 1, interval) == 75);
    queued.start(80);
    assert(queued.steps == 0 && queued.ticks == 0 && queued.peak == 80);
}

void segmentGeometry() {
    PrintLine move;
    move.setAxisSteps(X_AXIS, -10);
    move.setAxisSteps(Y_AXIS, 10);
    const float distances[] = {3, 4, 12, 2};
    move.setSegmentGeometry(distances);
    assert(move.isNegativeMoveOfAxis(X_AXIS) && move.isPositiveMoveOfAxis(Y_AXIS));
    assert(move.primaryAxis == X_AXIS && move.stepsRemaining == 10 && move.distance == 5);

    move.setAxisSteps(Z_AXIS, 10);
    move.setSegmentGeometry(distances);
    assert(move.primaryAxis == Z_AXIS && move.stepsRemaining == 10 && move.distance == 13);

    move.setAxisSteps(E_AXIS, -10);
    move.setSegmentGeometry(distances);
    assert(move.primaryAxis == E_AXIS && move.stepsRemaining == 10 && move.distance == 13);

    PrintLine retract;
    retract.setAxisSteps(E_AXIS, -1);
    retract.setSegmentGeometry(distances);
    assert(retract.isNegativeMoveOfAxis(E_AXIS));
    assert(retract.primaryAxis == E_AXIS && retract.stepsRemaining == 1 && retract.distance == 2);
}

void moveQueue() {
    auto& queue = PrintLine::queue;
    g_uStartOfIdle = 500;
    for (int count = 1; count < MOVE_CACHE_SIZE; ++count) {
        queue.writeSlot().task = TASK_DISABLE_Z_COMPENSATION;
        queue.publish(false);
        assert(queue.count() == count && queue.active() == nullptr);
    }
    assert(g_uStartOfIdle == 500);
    queue.activateFront()->moveZ = true;
    assert(PrintLine::hasActiveZMove() && interruptsEnabled);
    queue.clear();
    assert(!PrintLine::hasActiveZMove() && interruptsEnabled);
    assert(queue.count() == 0 && queue.active() == nullptr);
    assert(queue.readIndex() == MOVE_CACHE_SIZE - 1 && queue.writeIndex() == queue.readIndex());

    assert(PrintLine::insertWaitMovesIfNeeded(1, 0) == 1);
    assert(queue.count() == 4 && g_uStartOfIdle == 0);
    for (int depth = 3; depth >= 0; --depth) {
        auto* move = queue.activateFront();
        assert(move->task == TASK_NO_TASK && move->flags == FLAG_WARMUP);
        assert(move->warmupDepth == depth && move->warmupTicks == 100000);
        queue.releaseFront();
        assert(queue.count() == depth + 1 && queue.active() == nullptr);
        assert(queue.activateFront() == move);
        queue.retireFront();
        assert(!interruptsEnabled && queue.count() == depth);
        HAL::allowInterrupts();
    }
    assert(queue.count() == 0 && queue.active() == nullptr);
    assert(queue.readIndex() == 3 && queue.writeIndex() == 3);
}

void xyLimits() {
    for (uint8_t axis = X_AXIS; axis <= Y_AXIS; ++axis) {
        const uint8_t positive = 17 << axis, negative = 16 << axis;
        PrintLine move;
        move.xyDirection = positive;
        Printer::xyMaxHit[axis] = true;
        assert(!canStepXY(axis, &move, true));
        assert(!canStepXY(axis, &move, true));
        assert(Printer::xyMotion.queuedOvertravel(axis) == 2);

        move.xyDirection = negative;
        assert(!canStepXY(axis, &move, true));
        Printer::xyMaxHit[axis] = false;
        assert(!canStepXY(axis, &move, true));
        assert(canStepXY(axis, &move, true) && !Printer::xyMotion.queuedOvertravel(axis));

        Printer::xyMinHit[axis] = true;
        assert(!canStepXY(axis, &move, true));
        assert(Printer::xyMotion.queuedOvertravel(axis) == -1);
        move.xyDirection = positive;
        move.abortAtEndstops = true;
        assert(canStepXY(axis, &move, false)); // A direct retreat leaves the queue's debt alone.
        assert(Printer::xyMotion.queuedOvertravel(axis) == -1);
        assert(!canStepXY(axis, &move, true));
        assert(canStepXY(axis, &move, true) && !Printer::xyMotion.queuedOvertravel(axis));

        move.xyDirection = negative;
        move.abortAtEndstops = false;
        assert(!canStepXY(axis, &move, true));
        move.abortAtEndstops = true;
        assert(!canStepXY(axis, &move, true));
        assert(!move.isNegativeMoveOfAxis(axis) && !Printer::xyMotion.queuedOvertravel(axis));
        assert(Printer::stoppedDirectAxis != axis);

        move.xyDirection = negative;
        assert(!canStepXY(axis, &move, false));
        assert(!move.isNegativeMoveOfAxis(axis) && Printer::stoppedDirectAxis == axis);
        Printer::xyMinHit[axis] = false;

        auto& xy = Printer::xyMotion;
        xy.setNominalSteps(axis, 320);
        xy.setMotorSteps(axis, 319);
        xy.requestDirect(axis, -4);
        xy.recordMotorStep(axis, -1);
        xy.advanceDirect(axis, -1);
        assert(xy.nominalSteps(axis) == 320 && xy.motorSteps(axis) == 318);
        assert(xy.directPosition(axis).remaining() == -3);
        xy.stopDirect(axis);
        assert(xy.directPosition(axis).steps() == -1 && xy.directPosition(axis).remaining() == 0);
    }
}

void initializeOffsets() {
    planMove(256, true);
    Printer::zMotion.setNominalSteps(256);
    Printer::zMotion.setMotorReference(256 + Printer::directCurrentSteps[Z_AXIS]);
    Printer::zOffset.setManualMicrometers(50, zStepsPerMm);
    Printer::senseOffsetControl.setThresholdDigits(senseThreshold);
    Printer::senseOffsetControl.setMaxMicrometers(initialClearance);
    sample(3 * senseThreshold, 10);
    assert(Printer::zOffset.senseMicrometers() == initialClearance);
    Printer::senseOffsetControl.setThresholdDigits(0);
    sample(printingForce, 40);
    Printer::senseOffsetControl.setMaxMicrometers(SENSIBLE_PRESSURE_MAX_OFFSET);
    Printer::senseOffsetControl.setThresholdDigits(senseThreshold);
}

int main(int argc, char** argv) {
    assert(argc == 2);
    Printer::xyMotion.setNominalSteps(X_AXIS, 6400);
    Printer::xyMotion.setNominalSteps(Y_AXIS, 12800);
    const bool digitFlowReplay = !std::strcmp(argv[1], "digit-flow");
    if (!digitFlowReplay && std::strcmp(argv[1], "digit-zero") && std::strncmp(argv[1], "pressure-", 9))
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
    else if (!std::strcmp(argv[1], "prime-parameters"))
        primeParameters();
    else if (!std::strcmp(argv[1], "sense-window"))
        senseOffsetWindow();
    else if (!std::strcmp(argv[1], "sense-settings"))
        senseOffsetSettings();
    else if (!std::strcmp(argv[1], "sense-limit-interrupt"))
        senseOffsetLimitInterrupt();
    else if (!std::strcmp(argv[1], "pressure-pause"))
        pressurePause();
    else if (!std::strcmp(argv[1], "pressure-stop"))
        pressureStop();
    else if (!std::strcmp(argv[1], "digit-zero"))
        digitZero();
    else if (!std::strcmp(argv[1], "extrusion-planning"))
        extrusionPlanning();
    else if (!std::strcmp(argv[1], "gcode-target"))
        gcodeTarget();
    else if (!std::strcmp(argv[1], "wobble-target"))
        wobbleTarget();
    else if (!std::strcmp(argv[1], "cartesian-planning"))
        cartesianPlanning();
    else if (!std::strcmp(argv[1], "segment-geometry"))
        segmentGeometry();
    else if (!std::strcmp(argv[1], "move-queue"))
        moveQueue();
    else if (!std::strcmp(argv[1], "ecmp-stepping"))
        ecmpStepping();
    else if (!std::strcmp(argv[1], "advance-timer"))
        advanceTimer();
    else if (!std::strcmp(argv[1], "step-clock"))
        stepClock();
    else if (!std::strcmp(argv[1], "xy-limits"))
        xyLimits();
    else
        return 1;
    std::printf("Print lifecycle replay passed (%s)\n", argv[1]);
}

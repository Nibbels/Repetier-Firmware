#include "ScanSettings.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>

constexpr uint8_t EXTRUDER_OCIE = 1;
constexpr uint8_t otherTimer0Interrupts = 0x05; // Compare B and overflow.
uint8_t EXTRUDER_TIMSK = otherTimer0Interrupts;
#define _BV(bit) (1U << (bit))
#define INLINE inline

#include "Constants.h"
#include "QueueTask.h"
#include "stop-reason-firmware.inc"
#define MOTHERBOARD DEVICE_TYPE_RF2000
#ifndef NUM_EXTRUDER
#define NUM_EXTRUDER 1
#endif
#include "Configuration.h"
#include "RF2000.h"
#include "ZOffset.h"
#include "ExtrusionCompensation.h"
#include "ExtrusionOutput.h"
#include "MachineInactivity.h"
#include "IdleStatus.h"
#include "PrintLayers.h"
#include "BedCompensation.h"
#include "PrintPause.h"
#include "PrintJob.h"
#include "MotionPlanner.h"

// The replay compiles the live firmware routines. These shims supply DMS bytes,
// a bed profile, a clock and the motor displacements completed during each wait.
using millis_t = uint32_t;
using speed_t = uint16_t;
using ticks_t = uint32_t;
using fast8_t = int8_t;
constexpr uint32_t F_CPU = 16000000UL;
constexpr int32_t LIMIT_INTERVAL = (F_CPU / 40000) + 1;
constexpr bool FEEDRATE_GCODE = true;
constexpr float zStepsPerMm = 1280.0f;
constexpr int16_t initialClearance = 20;
constexpr int16_t printingForce = 6000;
constexpr int16_t senseThreshold = 3000;
constexpr long parkY = 200 * 160L, parkZ = 30 * static_cast<long>(zStepsPerMm), parkE = 280;

millis_t clockMs = 0;
char g_nHeatBedScanStatus = 0, g_nZOSScanStatus = 0;
int32_t g_offsetZCompensationSteps = 250;
short g_ZCompensationMax = 200;
long bedMatrixHighestZSteps() { return g_offsetZCompensationSteps; }
short bedMatrixLowestZSteps() { return g_ZCompensationMax; }
short forceDigits = 0;
ScanSettings scanSettings = [] {
    ScanSettings settings(1.0f);
    settings.pressureReadCount = 4;
    settings.pressureReadDelayMs = 3;
    settings.pressureTolerance = 10;
    return settings;
}();
bool pollDuringMoves = true;
bool clipReturn[4] = {};
int incompleteReturns = 0;
int incomingReads = 0, objectOutputs = 0;
unsigned matrixDepthReads = 0;
unsigned strainGaugeReads = 0;
void (*duringMove)(long, long, long, long) = nullptr;
void (*duringHeatWait)() = nullptr;
std::function<void()> duringPeriodicActions;
std::function<void()> duringOutputScript;
unsigned motorShutdowns = 0;
unsigned primePolls = 0, primeMoves = 0, skipPrimeAt = 0, abortPrimeAt = 0;
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
uint32_t U16SquaredToU32(uint16_t value) { return uint32_t(value) * value; }
void delayMilliseconds(unsigned delay) { clockMs += delay; }
int16_t eepromWords[4096] = {};
int32_t eepromDWords[4096] = {};
unsigned eepromReads = 0;
int16_t eprGetInt16(unsigned address) { ++eepromReads; return eepromWords[address]; }
void eprSetInt16(unsigned address, int16_t value) { eepromWords[address] = value; }
int32_t eprGetInt32(unsigned address) { ++eepromReads; return eepromDWords[address]; }
void eprSetInt32(unsigned address, int32_t value) { eepromDWords[address] = value; }
}

#include "EepromZonePatch.h"
constexpr uint16_t EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX = EepromZones::address(EepromZones::Field::F_RF_MOD_SENSEOFFSET_OFFSET_MAX);
constexpr uint16_t EPR_RF_MOD_SENSEOFFSET_DIGITS = EepromZones::address(EepromZones::Field::F_RF_MOD_SENSEOFFSET_DIGITS);
constexpr uint16_t EPR_RF_EMERGENCYPAUSEDIGITSMIN = EepromZones::address(EepromZones::Field::F_RF_EMERGENCYPAUSEDIGITSMIN);
constexpr uint16_t EPR_RF_EMERGENCYPAUSEDIGITSMAX = EepromZones::address(EepromZones::Field::F_RF_EMERGENCYPAUSEDIGITSMAX);
constexpr uint16_t EPR_RF_EMERGENCYZSTOPDIGITSMIN = EepromZones::address(EepromZones::Field::F_RF_EMERGENCYZSTOPDIGITSMIN);
constexpr uint16_t EPR_RF_EMERGENCYZSTOPDIGITSMAX = EepromZones::address(EepromZones::Field::F_RF_EMERGENCYZSTOPDIGITSMAX);

namespace EEPROM {
unsigned persistenceCommits = 0;
void publishSenseOffsetSettings();
void storeSenseOffsetSettings(int16_t, int16_t, bool, bool);
bool storePressureLimits(bool,int32_t,int32_t,bool,bool);
}

bool interruptsEnabled = true;
unsigned interruptBeforeLock = 0;
void (*onInterrupt)() = nullptr;
unsigned locksUntilCompensationPublish = 0;
float extrusionBeforePublish = 0.0f;

namespace HAL {
void allowInterrupts() { interruptsEnabled = true; }
void forbidInterrupts() { interruptsEnabled = false; }
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
bool READ(int pin);
void WRITE(int pin, bool high);
#include "ZMotion.h"
#include "DigitCompensation.h"
#include "SenseOffsetControl.h"
#include "PressureProtection.h"
#include "CalibrationScan.h"
#include "ManualMotion.h"
#include "GCodeModes.h"
#include "PressureMeasurement.h"
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
const char ui_text_output_object[] = "";

struct Com {
    static constexpr const char* tSlash = "/";
    static constexpr const char* tLinearLColon = "Linear L:";
    static const char* lastLine;
    static void printFLN(const char* line) { lastLine = line; }
    static void printErrorFLN(const char* line) { lastLine = line; }
    template<class... Args> static void printF(Args...) {}
    template<class... Args> static void printFLN(Args...) {}
    static void println() {}
};

const char* Com::lastLine = nullptr;

template<class... Args> void showWarning(Args...) { ++incompleteReturns; }
template<class... Args> void showError(Args...) { assert(false); }
template<class... Args> void showInformation(Args...) { assert(false); }
unsigned pressureStops = 0;
bool expectPressureStop = false;
void doEmergencyStop(StopReason reason) {
    assert(expectPressureStop && reason == StopReason::ZBlock);
    ++pressureStops;
    expectPressureStop = false;
}
bool isAnyScanRunning();
bool heatBedScanRunning() { return g_nHeatBedScanStatus != 0; }
bool zOffsetScanRunning() { return g_nZOSScanStatus != 0; }
void discardScans() { g_nHeatBedScanStatus = g_nZOSScanStatus = 0; }
void setExtruderCurrent(int, int) {}
namespace Drv8711 { uint8_t configuredCurrent(uint8_t) { return 0; } }

struct DmsInput {
    bool disconnected = false;
    bool wrongConfiguration = false;
    bool timed = false;
    uint32_t startedAt = 0;
    StrainGauge::Sample published{};

    void tick() {
        if (++clockMs - startedAt < 125)
            return;
        published = {startedAt, clockMs, static_cast<uint16_t>(published.sequence + 1), forceDigits};
        startedAt = clockMs;
    }
} dmsInput;

bool StrainGauge::history(uint8_t age, Sample& sample) const {
    if (age || (dmsInput.timed && !dmsInput.published.sequence))
        return false;
    sample = dmsInput.timed ? dmsInput.published : Sample{clockMs, clockMs, 1, forceDigits};
    return true;
}

StrainGauge::Availability StrainGauge::latest(uint32_t, Sample& sample) const {
    ++strainGaugeReads;
    if (dmsInput.disconnected || dmsInput.wrongConfiguration)
        return Availability::Fault;
    return history(0, sample) ? Availability::Ready : Availability::Waiting;
}

namespace Printer {
extern PrintPause pause;
extern PrintJob printJob;
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
float axisStepsPerMM_[4] = { 160.0f, 160.0f, zStepsPerMm, 280.0f };
float axisMMPerSteps_[4] = { 1.0f / 160, 1.0f / 160, 1.0f / zStepsPerMm, 1.0f / 280 };
float maxFeedrate[4] = { 200, 200, 9, 25 };
uint32_t maxPrintAccelerationStepsPerSquareSecond[4] = { 100000, 100000, 100000, 100000 };
uint32_t maxTravelAccelerationStepsPerSquareSecond[4] = { 100000, 100000, 100000, 100000 };
float homingFeedrate_[3] = { 60.0f, 60.0f, 5.0f };
float axisLengthMM[3] = { 170.0f, 245.0f, 200.0f };
long maxSoftEndstopSteps_[2] = { 170 * 160L, 245 * 160L };
MoveTarget moveTarget;
float feedrate = 10.0f;
GCodeModes gcodeModes;
float menuExtrusionFactor = 1.0f;
ExtrusionCompensation extrusionCompensation;
bool blockAll = false;
ExtrusionOutput extrusionOutput;
MachineInactivity inactivity(STEPPER_INACTIVE_TIME * 1000UL, MAX_INACTIVE_TIME * 1000UL);
IdleStatus idleStatus;
constexpr float maxXYJerk = 8.0f, maxZJerk = 0.5f;
float stepsPerMm(uint8_t axis) { return axisStepsPerMM_[axis]; }
void setStepsPerMm(uint8_t axis, float value) { axisStepsPerMM_[axis] = value; }
float mmPerStep(uint8_t axis) { return axisMMPerSteps_[axis]; }
float axisMaxFeedrate(uint8_t axis) { return maxFeedrate[axis]; }
float axisHomingFeedrate(uint8_t axis) { return homingFeedrate_[axis]; }
float axisLengthMm(uint8_t axis) { return axisLengthMM[axis]; }
long xyMaxSteps(uint8_t axis) { return maxSoftEndstopSteps_[axis]; }
float xyJerkLimit() { return maxXYJerk; }
float zJerkLimit() { return maxZJerk; }
const uint32_t* accelerationSteps(bool printing) { return printing ? maxPrintAccelerationStepsPerSquareSecond : maxTravelAccelerationStepsPerSquareSecond; }
volatile char doHeatBedZCompensation = true;
bool menuPaused = false;
bool axesHomed = true;
int feedrateMultiply = 100;
float requestedFeedrate() { return feedrate; }
int feedrateMultiplier() { return feedrateMultiply; }
float flowMultiplier() { return menuExtrusionFactor; }
ManualMotion manualMotion(1, 1, 1, 28);

bool debugErrors() { return false; }
bool infoLogging = false;
bool debugInfo() { return infoLogging; }
bool outputSafetyLatched = false;
bool heatingInhibited() { return outputSafetyLatched; }
bool areAxisHomed() { return axesHomed; }
bool isAxisHomed(int) { return true; }
bool isMinEndstopHit(uint8_t axis) { return xyMinHit[axis]; }
bool isMaxEndstopHit(uint8_t axis) { return xyMaxHit[axis]; }
void stopDirectAxis(uint8_t axis) { stoppedDirectAxis = axis; }
bool isZMaxEndstopHit() { return zMotion.maxEndstopHit(); }
bool isZMinEndstopHit() { return ZMotion::minSwitchHit(); }
bool zStepperEnabled() { return true; }
bool needsCMPwait() { return false; }
void disableAllSteppersNow() {
    assert(extrusionOutput.pendingSteps() == 0);
    ++motorShutdowns;
}
void setMenuMode(int mode, bool active) {
    assert(mode == MENU_MODE_PAUSED);
    menuPaused = active;
}
void stopPrint();
void stopForThermalFault();
void applyAdvanceSettings();
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
bool queueRelativeStepsCoordinates(long, long, long z, long, float, bool, bool checkEndstop) {
    assert(z < 0 && checkEndstop);
    zMotion.setMotorReference(0);
    return true;
}
bool queueRelativeMMCoordinates(float, float, float z, float, float, bool, bool checkEndstop = false) {
    const int32_t steps = zMotion.motorSteps() + int32_t(z * axisStepsPerMM_[Z_AXIS]);
    zMotion.setMotorReference(checkEndstop ? std::max(int32_t(0), steps) : steps);
    return true;
}
void setXAxisSteps(long) {}
void setYAxisSteps(long) {}
void setZAxisSteps(long steps) { zMotion.setNominalSteps(steps); }
void setEPositionMM(float value, float remainder = 0.0f);
void setFeedrate(float value);
bool queueGCodeCoordinates(GCode*);
bool queueInternalAxisMove(uint8_t axis, float value, float feedrateMmMin = 0);
void updateWobbleFixOffsets();
int32_t prepareDestinationSteps(uint8_t axis);
float getDirectMM(uint8_t axis) { return directCurrentSteps[axis] * axisMMPerSteps_[axis]; }
int32_t softwareEndstopSteps(uint8_t axis) {
    return axis == Z_AXIS ? zMotion.maxSteps() : maxSoftEndstopSteps_[axis];
}
void queueFloatCoordinates(float x, float y, float z, float e, float speed);
bool checkPlayKey() { return skipPrimeAt && primePolls >= skipPrimeAt; }
bool checkAbortKeys() { return false; }
void offsetRelativeStepsCoordinates(long x, long y, long z, long e, DirectMoveMode task = DirectMoveMode::Configured);
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
float sqr(float value) { return value * value; }
float min(float a, float b) { return std::min(a, b); }
int32_t min(int32_t a, int32_t b) { return std::min(a, b); }
float max(float a, float b) { return std::max(a, b); }
int32_t max(int32_t a, int32_t b) { return std::max(a, b); }
}

struct Extruder {
    static bool allHeatersCalibrated() { return current->tempControl.pidCalibrationValid(); }
    struct {
        bool calibrationValid = true;
        bool pidCalibrationValid() const { return calibrationValid; }
        float targetTemperatureC = 200.0f;
        uint8_t paused = 0;
        void waitForTargetTemperature(int = 0, bool resumingPause = false) {
            assert(resumingPause && Printer::pause.isReheating());
            if (duringHeatWait)
                duringHeatWait();
            poll();
        }
    } tempControl;
    int8_t stepperDirection = 0;
    uint8_t id = 0;
    bool enabled = false;
    int deliveredSteps = 0;
    float offsetMM[2] = {};
    float maxEJerk = 6.0f;
    float advanceL = 0.0f;

    static Extruder* current;
    static unsigned stepPulses, trailingEdges;
    static void setDirection(bool forward) { current->stepperDirection = forward ? 1 : -1; }
    static void step() { assert(!interruptsEnabled); ++stepPulses; }
    static void unstep() { assert(!interruptsEnabled); ++trailingEdges; }
    static void timedStep(volatile int16_t& pending, int16_t remaining) {
        step();
        pending = remaining;
        unstep();
        current->deliveredSteps += current->stepperDirection;
    }
    static bool selectExtruderById(uint8_t id, bool waitThroughPause = false);
    static void applyCurrentSettings() {}
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
#include "direct-phase-firmware.inc"
struct PrintLine {
#include "direct-control-firmware.inc"

    bool moveZ = false;
    bool positiveE = false;
    bool moveXY = false;
    bool moveE = false;
    bool abortAtEndstops = false;
    bool isAbortAtEndstops() const { return abortAtEndstops; }
    bool isPositiveMoveOfAxis(uint8_t axis) const {
        uint8_t mask = 17 << axis;
        return (dir & mask) == mask;
    }
    bool isNegativeMoveOfAxis(uint8_t axis) const {
        return (dir & (17 << axis)) == (16 << axis);
    }
    void setMoveOfAxisFinished(uint8_t axis) { dir &= ~(16 << axis); }
    void setMoveOfAxis(uint8_t axis) { dir |= 16 << axis; }
    void setPositiveDirectionForAxis(uint8_t axis) { dir |= 1 << axis; }
    int32_t delta[4] = {};
    int32_t stepsRemaining = 0;
#include "step-profile-firmware.inc"
    LookaheadData lookahead{};
    StepProfile stepProfile{};
    void fitRampToRemainingSteps();
    void limitToStopTail(bool refitRamp);
    float fullSpeed = 0;
    int32_t timeInTicks = 0;
    uint8_t primaryAxis = 0;
    volatile char task = DIRECT_IDLE;
    uint8_t flags = 0, joinFlags = 0, dir = 0;
#include "lookahead-flags-firmware.inc"
    uint8_t warmupDepth = 0;
    long warmupTicks = 0;
    void setWaitForXLinesFilled(uint8_t depth) { warmupDepth = depth; }
    void setWaitTicks(long ticks) { warmupTicks = ticks; }
    void setAxisSteps(uint8_t axis, int32_t steps);
    void setSegmentGeometry(const float axisDistanceMM[]);
    bool isMoveOfAxis(uint8_t axis) const { return dir & (16 << axis); }
    bool isNoMove() const { return (dir & 240) == 0; }
    bool isXMove() const { return isMoveOfAxis(X_AXIS); }
    bool isYMove() const { return isMoveOfAxis(Y_AXIS); }
    bool isXNegativeMove() const { return isNegativeMoveOfAxis(X_AXIS); }
    bool isYNegativeMove() const { return isNegativeMoveOfAxis(Y_AXIS); }
    bool isZNegativeMove() const { return isNegativeMoveOfAxis(Z_AXIS); }
    bool isENegativeMove() const { return isNegativeMoveOfAxis(E_AXIS); }
    bool isEMove() const { return moveE || (dir & 128); }
    bool isEOnlyMove() const { return (dir & 240) == 128; }
    bool isZMove() const { return moveZ || isMoveOfAxis(Z_AXIS); }
    bool isXYZMove() const { return dir & 112; }
    bool isEPositiveMove() const { return positiveE || isPositiveMoveOfAxis(E_AXIS); }
    bool isXOrYMove() const { return moveXY || (dir & 48); }

    static PrintLine direct;
    static MoveQueue queue;
    static uint8_t insertWaitMovesIfNeeded(uint8_t pathOptimize, uint8_t waitExtraLines);
    static int linesCount;
    static QueueTask queuedTask;
    static int stopRequests;
    static bool hasLines() { return linesCount; }
    static bool directMoveActive() { return direct.task != DIRECT_IDLE; }
#include "active-move-firmware.inc"
    static int queuedMoveCount() { return linesCount; }
    static bool hasStoppableDirectMove();
    static unsigned plannedMoves;
    static bool updatesWobble;
    static bool publishesMove;
    static bool prepareQueueMove(bool, bool, float, bool wobble) {
        ++plannedMoves;
        updatesWobble = wobble;
        return publishesMove;
    }
    static void abortDirectMoveForEmergency() { direct.stepsRemaining=0; direct.task=DIRECT_IDLE; }
    static void applyDirectStop();
    static void stopDirectMove() { ++stopRequests; applyDirectStop(); }
    static void discardQueuedMoves() { linesCount = 0; queuedTask = QueueTask::None; }
    static void queueTask(QueueTask task) {
        assert(!interruptsEnabled && linesCount < MOVE_CACHE_SIZE);
        queuedTask = task;
        ++linesCount;
    }
};

PrintLine PrintLine::direct;
bool PrintLine::publishesMove = true;

void Printer::queueFloatCoordinates(float x, float y, float z, float e, float speed) {
    assert(startLine.active() && !printJob.commandsBlocked());
    ++primeMoves;
    ++PrintLine::linesCount;
    if (primeMoves == 1)
        primeApproach = {x, y, z, e, speed};
    if (primeMoves == 2)
        firstPrimeSegment = {x, y, z, e, speed};
    if (x == IGNORE_COORDINATE && y != IGNORE_COORDINATE && e == IGNORE_COORDINATE)
        ++primeSpacingMoves;
    if (z != IGNORE_COORDINATE)
        zMotion.setNominalSteps(std::lround(z * axisStepsPerMM_[Z_AXIS]));
    if (e != IGNORE_COORDINATE) {
        setEPositionMM(e);
        extrusionPlanner.restoreStepRemainder(0.375f);
        extrusionCompensation.restoreRemainder(0.25f);
    }
}
int PrintLine::linesCount = 3;
QueueTask PrintLine::queuedTask = QueueTask::None;
int PrintLine::stopRequests = 0;
unsigned PrintLine::plannedMoves = 0;
bool PrintLine::updatesWobble = false;
const int32_t queuedPrimarySteps = 1, directPrimarySteps = 1;
bool hasQueuedOvertravel() { return false; }

struct StepExecutor {
#include "step-clock-firmware.inc"
    static StepErrors stepErrors[2];
    static speed_t nextStepSpeed(PrintLine& move, StepClock& clock, uint8_t pulses, uint32_t interval);
    static void updateAdvanceSteps(const PrintLine& move, uint16_t speed);
};
StepExecutor::StepErrors StepExecutor::stepErrors[2] = {};

struct UIDisplay {
    int activeButton() const { return lastButtonAction; }
    int lastButtonAction = 0, menuLevel = 1, menuPos[1] = {};
    int flags = 0, jogButtonAction = 0;
    int16_t nextKey = 0;
    millis_t lastButtonStart = 0;
    void exitmenu() {}
    void initializeLCD() {}
    void ui_check_keys(int16_t& action) { action = nextKey; }
    void setStatusP(const char*) {}
    void refreshPage() {}
    void pressPlay();
    void fastAction();
    void senseOffsetUpAction();
    void senseOffsetDownAction();
} uid;

enum FirmwareState { Paused, Processing, WaitHeater };
namespace Commands {
void checkForPeriodicalActions(FirmwareState);
enum class MotionWait { QueueAndZ, AllOutputs };
enum class PauseWait { Resume, Cancel };
bool waitForMotion(MotionWait motion, PauseWait pause);
void changeFeedrateMultiply(int) { assert(false); }
void printCurrentPosition() {}
void setFanSpeed(uint8_t) {}
}

struct GCode {
    bool internalCommand = false;
    long S = 0, P = 0;
    float E = 0, F = 0, I = 1, X = 0, Y = 0, Z = 0;
    bool explicitP = false, explicitS = false, explicitY = false;
    bool hasS() const { return explicitS || S != 0; }
    bool hasP() const { return explicitP || P != 0; }
    bool hasE() const { return E != 0; }
    bool hasF() const { return F != 0; }
    bool hasI() const { return I != 0; }
    bool hasX() const { return X != 0; }
    bool hasY() const { return explicitY || Y != 0; }
    bool hasZ() const { return Z != 0; }
    bool hasNoXYZ() const { return !hasX() && !hasY() && !hasZ(); }

};

struct GCodeInput {
    static void readFromSerial() {
        assert(Printer::printJob.commandsBlocked());
        ++incomingReads;
    }
    static bool flashScriptActive() { return false; }
    static bool executeOutputProgram() {
        assert(!PrintLine::hasLines());
        assert(!Printer::printJob.commandsBlocked());
        ++objectOutputs;
        if (duringOutputScript)
            duringOutputScript();
        return !Printer::printJob.commandsBlocked();
    }
};

unsigned queuedSenseOffsetEnables = 0;
void queueTask(QueueTask task) {
    assert(task == QueueTask::EnableSenseOffset);
    ++queuedSenseOffsetEnables;
}

void outputObject(bool showerrors);
void showIdle() {}
#define CASE_FAN_PIN -1
#define MINIMAL_IDLE_TIME 500
namespace Printer {
void switchEverythingOff() { assert(false); }
void setAllSwitchedOff(bool) {}
bool areAllSteppersDisabled() { return false; }
}

long getZMatrixDepth(long x, long y) {
    ++matrixDepthReads;
    return 37 + x / 32 - y / 64;
}

constexpr uint8_t OCF0A = 1;
uint8_t EXTRUDER_OCR = 0, TCNT0 = 0;
struct Timer0Flags {
    uint8_t pending = 0;
    void operator=(uint8_t clear) { pending &= ~clear; }
} TIFR0;
#define ISR(vector, ...) void vector()
#define EXTRUDER_TIMER_VECTOR advanceCompareInterrupt

// Model only the durable transport boundary. Production settings publication,
// band validation and typed patch construction are replayed below.
namespace EEPROM {
using namespace EepromZones;
bool loaded = true;
bool lastOk = false;
bool allowCommit = true;
enum class Result { Ok, Failed };
bool accept(Result result) { return lastOk = result == Result::Ok; }
bool persist(const Patch* edits, uint8_t count) {
    assert(interruptsEnabled);
    assert(patchMask(edits, count));
    if (!allowCommit) return false;
    bool changed = false;
    for (uint8_t i = 0; i < count; ++i) {
        const auto& p = edits[i];
        if (p.width == 2) {
            changed |= HAL::eepromWords[p.pos] != int16_t(p.bits);
            HAL::eepromWords[p.pos] = int16_t(p.bits);
        } else {
            assert(p.width == 4);
            changed |= HAL::eepromDWords[p.pos] != int32_t(p.bits);
            HAL::eepromDWords[p.pos] = int32_t(p.bits);
        }
    }
    if (changed) ++persistenceCommits;
    return true;
}
struct Session {
    Result read(uint8_t id, uint8_t* data, uint8_t n, uint32_t& alerts) {
        assert(interruptsEnabled && id == Pressure);
        BlockEncoder enc(id, data, n);
        enc.u32(Field::F_RF_EMERGENCYPAUSEDIGITSMIN, HAL::eprGetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMIN));
        enc.u32(Field::F_RF_EMERGENCYPAUSEDIGITSMAX, HAL::eprGetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMAX));
        enc.u16(Field::F_RF_EMERGENCYZSTOPDIGITSMIN, HAL::eprGetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMIN));
        enc.u16(Field::F_RF_EMERGENCYZSTOPDIGITSMAX, HAL::eprGetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMAX));
        assert(enc.complete());
        alerts = 0;
        return Result::Ok;
    }
    Result patch(const Patch* edits, uint8_t count) {
        return persist(edits, count) ? Result::Ok : Result::Failed;
    }
};
Session session() { return {}; }
void publishSenseOffsetSettings() {
    // Fixture for reload; actual LiveSettings decoder covered by EEPROM replay.
    Printer::senseOffsetControl.publishStoredLimits(
        HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS),
        HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX));
}
}

#include "pause-sense-offset-firmware.inc"

bool advanceTimerArmed() {
    assert((EXTRUDER_TIMSK & ~_BV(EXTRUDER_OCIE)) == otherTimer0Interrupts);
    return EXTRUDER_TIMSK & _BV(EXTRUDER_OCIE);
}

bool fireAdvanceCompare(uint8_t latency = 0) {
    if (!advanceTimerArmed())
        return false;
    InterruptProtectedBlock lock;
    TCNT0 = EXTRUDER_OCR + latency;
    TIFR0.pending &= ~_BV(OCF0A); // The AVR clears this flag on vector entry.
    advanceCompareInterrupt();
    return true;
}

void withStartLine(const std::function<void()>& action) {
    assert(!duringPeriodicActions);
    PrintLine::linesCount = 0;
    skipPrimeAt = primePolls + 1;
    duringPeriodicActions = [&] {
        if (++primePolls == skipPrimeAt)
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
    const auto readCurrentDigits = [] {
        int16_t digits = 0;
        assert(Printer::strainGauge.readDigits(clockMs, digits));
        return digits;
    };
    duringPeriodicActions = [] { dmsInput.tick(); };
    auto& gauge = Printer::strainGauge;
    const auto homeWithConversions = [] {
        dmsInput.timed = true;
        Printer::homeDigits();
        dmsInput.timed = false;
    };
    assert(gauge.zeroingEnabled());
    forceDigits = 1000;
    homeWithConversions();
    assert(gauge.zeroOffset() == 1000);
    assert(dmsInput.published.sequence == 5 && clockMs == 625);
    assert(readCurrentDigits() == 0);

    forceDigits = -1200;
    homeWithConversions();
    assert(gauge.zeroOffset() == -1200); // Fresh reference, even with a prior zero.
    assert(readCurrentDigits() == 0);
    forceDigits = -27767;
    assert(readCurrentDigits() == -26567);
    forceDigits = -27768;
    assert(readCurrentDigits() == -27768);
    forceDigits = 27766;
    assert(readCurrentDigits() == 28966);
    forceDigits = 27767;
    assert(readCurrentDigits() == 27767);

    gauge.setZeroingEnabled(false);
    forceDigits = 2400;
    const unsigned readsBeforeZero = strainGaugeReads;
    Printer::homeDigits();
    assert(strainGaugeReads == readsBeforeZero && gauge.zeroOffset() == -1200);
    assert(gauge.effectiveZeroOffset() == 0);
    assert(readCurrentDigits() == 2400);
    gauge.setZeroingEnabled(true);
    assert(gauge.effectiveZeroOffset() == -1200);
    assert(readCurrentDigits() == 3600);

    forceDigits = 5000;
    homeWithConversions();
    assert(!gauge.zeroOffset() && readCurrentDigits() == 5000);

    dmsInput.disconnected = true;
    int16_t digits = 5;
    assert(!gauge.readDigits(clockMs, digits) && digits == 5);
    short average = 5;
    assert(readAveragePressure(&average) && average == 0);
    dmsInput.disconnected = false;
    dmsInput.wrongConfiguration = true;
    assert(!gauge.readDigits(clockMs, digits) && digits == 5);
    duringPeriodicActions = nullptr;
}


void pressureMeasurement() {
    using Result = PressureMeasurement::Result;
    PressureMeasurement measurement;
    measurement.begin({3, 15, 15}, UINT32_MAX - 10, 5);
    assert(!measurement.record(8, 133, -300));
    assert(measurement.record(9, 134, -101));
    assert(!measurement.record(9, 135, -300) && !measurement.acceptsSample(23));
    measurement.record(135, 260, -98);
    measurement.record(261, 386, -100);
    assert(measurement.result() == Result::Stable && measurement.average() == -99);

    measurement.begin({2, 10, 15}, 0);
    uint32_t time = 10;
    for (unsigned attempt = 0; attempt < 5; ++attempt) {
        measurement.record(time, time + 125, INT16_MIN);
        measurement.record(time + 126, time + 251, INT16_MAX);
        if (attempt < 4) {
            assert(measurement.result() == Result::Pending && !measurement.acceptsSample(time + 360));
            time += 361;
            assert(measurement.acceptsSample(time));
        }
    }
    assert(measurement.result() == Result::Unstable && !measurement.acceptsSample(time + 1000));
    measurement.begin({2, 15, 15}, 0);
    measurement.readFailed();
    assert(measurement.result() == Result::ReadError && !measurement.acceptsSample(15));
}

void pressurePause() {
    GCode command;
    command.S = -99999;
    command.P = 99999;
    HAL::eepromDWords[EPR_RF_EMERGENCYPAUSEDIGITSMIN] = -6000;
    HAL::eepromDWords[EPR_RF_EMERGENCYPAUSEDIGITSMAX] = 6000;
    const unsigned saves=EEPROM::persistenceCommits;
    configurePressure(3075, &command);
    assert(Printer::pressureProtection.pauseLimits().minimum==EMERGENCY_PAUSE_DIGITS_MIN);
    assert(Printer::pressureProtection.pauseLimits().maximum==EMERGENCY_PAUSE_DIGITS_MAX);
    assert(HAL::eepromDWords[EPR_RF_EMERGENCYPAUSEDIGITSMIN] == -6000);
    assert(HAL::eepromDWords[EPR_RF_EMERGENCYPAUSEDIGITSMAX] == 6000);
    assert(EEPROM::persistenceCommits==saves);
    configurePressure(3075,&command);
    assert(EEPROM::persistenceCommits==saves);
    sample(EMERGENCY_PAUSE_DIGITS_MAX, EMERGENCY_PAUSE_CHECKS);
    assert(!Printer::pause.isPaused());

    command.explicitS = command.explicitP = true;
    command.S = command.P = 0;
    configurePressure(3075, &command);
    assert(HAL::eepromDWords[EPR_RF_EMERGENCYPAUSEDIGITSMIN]==-6000 && HAL::eepromDWords[EPR_RF_EMERGENCYPAUSEDIGITSMAX]==6000);
    assert(EEPROM::persistenceCommits==saves);
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
    // M3076 is gone. Exercise the same pressure owner and persistent
    // menu settings, not a removed serial interface.
    Printer::pressureProtection.setStopLimits({-12000,11500});
    EEPROM::storePressureLimits(true,-12000,11500,true,true);
    assert(HAL::eprGetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMIN) == -12000);
    assert(HAL::eprGetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMAX) == 11500);
    const unsigned saves=EEPROM::persistenceCommits;
    EEPROM::storePressureLimits(true,-12000,11500,true,true);
    assert(EEPROM::persistenceCommits==saves);
    // A one-sided menu save does not capture an unrelated temporary limit.
    EEPROM::storePressureLimits(true,-11900,0,true,false);
    assert(HAL::eprGetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMAX)==11500 && EEPROM::persistenceCommits==saves+1);
    EEPROM::storePressureLimits(true,-12000,0,true,false);

    PrintLine::direct.task = DIRECT_RUNNING;
    PrintLine::direct.stepsRemaining = 1;
    PrintLine::direct.setAxisSteps(Z_AXIS, 1);
    PrintLine::direct.setAxisSteps(E_AXIS, 1);
    auto& output = Printer::extrusionOutput;
    output.setAdvanceEnabled(true);
    output.setLeadSteps(2);
    output.addSteps(3);
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
    assert(!output.blocked() && output.pendingSteps() == 5);
    advance(EMERGENCY_STOP_INTERVAL + 1);
    assert(output.blocked() && !output.pendingSteps() && !pressureStops);

    PrintLine extrusion;
    extrusion.moveE = true;
    extrusion.positiveE = true;
    extrusion.delta[E_AXIS] = 1;
    extrusion.stepProfile.advanceL = 65536;
    executeExtrusionStep(&extrusion, true);
    executeExtrusionStep(&extrusion, false);
    StepExecutor::updateAdvanceSteps(extrusion, 4);
    GCode testStep;
    testStep.S = 5;
    injectAdvanceTestSteps(&testStep);
    assert(output.pendingSteps() == 0 && Extruder::stepPulses == 0);

    sample(11500, EMERGENCY_STOP_CHECKS);
    assert(!output.blocked() && !output.pendingSteps());
    {
        InterruptProtectedBlock lock;
        output.onIdleTick();
        assert(!fireAdvanceCompare());
    }
    assert(output.pendingSteps() == 0 && Extruder::stepPulses == 0);
    PrintLine::direct.setMoveOfAxisFinished(E_AXIS);
    Extruder::current->stepperDirection = 1; // A cached direction cannot hide a Z move.
    sample(-12001, EMERGENCY_STOP_CHECKS - 1);
    assert(!Printer::extrusionOutput.blocked());
    expectPressureStop = true;
    sample(-12001);
    assert(Printer::extrusionOutput.blocked() && pressureStops == 1);

    PrintLine::direct.task = DIRECT_IDLE;
    PrintLine::direct.stepsRemaining = 0;
    Printer::pressureProtection.setStopLimits({0,0});
    assert(HAL::eprGetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMIN) == -12000);
    assert(HAL::eprGetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMAX) == 11500);
    sample(0, EMERGENCY_STOP_CHECKS);
    assert(!Printer::extrusionOutput.blocked());
    // The temporary zero band still rejects any nonzero average.
    sample(1, EMERGENCY_STOP_CHECKS);
    assert(Printer::extrusionOutput.blocked());
}

void outputSafetyLock() {
    auto& output = Printer::extrusionOutput;
    Printer::gcodeModes.setRelativeEOverride(true);
    GCode command;
    command.E = 1;
    assert(Printer::queueGCodeCoordinates(&command));
    const float saved = Printer::moveTarget.machineMM(E_AXIS);
    output.setAdvanceEnabled(true);
    output.setLeadSteps(2);
    output.addSteps(3);
    assert(advanceTimerArmed() && output.pendingSteps() == 5);
    {
        InterruptProtectedBlock lock;
        Printer::outputSafetyLatched = true;
        output.inhibit(ExtrusionOutput::Inhibit::SafetyLock);
    }
    assert(output.blocked() && output.pendingSteps() == 0 && !advanceTimerArmed());
    assert(!Printer::queueGCodeCoordinates(&command));
    assert(Printer::moveTarget.machineMM(E_AXIS) == saved);
    command.X = 1;
    assert(Printer::queueGCodeCoordinates(&command));
    assert(Printer::moveTarget.machineMM(E_AXIS) == saved);
    const unsigned oldPulses = Extruder::stepPulses;
    for (bool advance : {true, false}) {
        output.setAdvanceEnabled(advance);
        // Clearing transient reasons must never clear the sticky safety reason.
        output.allow(ExtrusionOutput::Inhibit::PrintStop);
        output.allow(ExtrusionOutput::Inhibit::MotorsDisabled);
        output.allow(ExtrusionOutput::Inhibit::Pressure);
        assert(output.blocked());
        PrintLine extrusion;
        extrusion.moveE = true;
        extrusion.positiveE = true;
        extrusion.delta[E_AXIS] = 1;
        extrusion.stepProfile.advanceL = 65536;
        executeExtrusionStep(&extrusion, true);
        executeExtrusionStep(&extrusion, false);
        StepExecutor::updateAdvanceSteps(extrusion, 4);
        GCode testStep;
        testStep.S = 5;
        injectAdvanceTestSteps(&testStep);
        assert(output.pendingSteps() == 0 && !fireAdvanceCompare());
        assert(Extruder::stepPulses == oldPulses);
    }
}

void pressureWithFrozenZ() {
    pollDuringMoves = false;
    auto& queue = PrintLine::queue;
    queue.writeSlot().setAxisSteps(Z_AXIS, 80);
    queue.publish(true);
    queue.activateFront()->stepsRemaining = 80;
    assert(PrintLine::activeMove().source == PrintLine::ActiveMove::Source::Queued);
    Printer::pause.request(PrintPause::Destination::InPlace);
    Printer::pause.serviceMotion();
    assert(Printer::pause.isSettled());
    assert(PrintLine::activeMove().source == PrintLine::ActiveMove::Source::None);

    PrintLine::direct.task = DIRECT_RUNNING;
    PrintLine::direct.setAxisSteps(E_AXIS, 10);
    PrintLine::direct.stepsRemaining = 10;
    const auto move = PrintLine::activeMove();
    assert(move.source == PrintLine::ActiveMove::Source::Direct);
    assert(move.usesAxis(E_AXIS) && !move.usesAxis(Z_AXIS));
    sample(EMERGENCY_STOP_DIGITS_MAX + 1, EMERGENCY_STOP_CHECKS);
    assert(!pressureStops && Printer::extrusionOutput.blocked());
}

bool pauseDuringQueueWait = false;
void Commands::checkForPeriodicalActions(FirmwareState state) {
    if (duringPeriodicActions)
        duringPeriodicActions();
    if (state == Processing && Printer::pause.allowsQueueAdvance() && PrintLine::hasLines()) {
        if (pauseDuringQueueWait) {
            pauseDuringQueueWait = false;
            Printer::pause.request(PrintPause::Destination::InPlace);
        } else if (--PrintLine::linesCount == 0 && PrintLine::queuedTask != QueueTask::None) {
            const auto destination = PrintLine::queuedTask == QueueTask::PauseAndPark
                ? PrintPause::Destination::Park : PrintPause::Destination::InPlace;
            PrintLine::queuedTask = QueueTask::None;
            Printer::pause.request(destination);
        }
    }
    if (PrintLine::direct.task && PrintLine::stopRequests) {
        const long stopTail = -32;
        Printer::directCurrentSteps[Z_AXIS] += stopTail;
        Printer::zMotion.setMotorReference(Printer::zMotion.motorSteps() + stopTail);
        PrintLine::direct.task = DIRECT_IDLE;
    }
    Printer::pause.serviceMotion();
}

void Printer::offsetRelativeStepsCoordinates(long x, long y, long z, long e, DirectMoveMode task) {
    // A short jog toward the print point travels one mm before key release.
    if (task == DirectMoveMode::Button)
        z = std::max(z, -static_cast<long>(zStepsPerMm));
    if (x || y || z || e) {
        if (pollDuringMoves)
            poll();
        if (clipReturn[X_AXIS] && x < 0) x = 0;
        if (clipReturn[Y_AXIS] && y < 0) y = 0;
        if (clipReturn[Z_AXIS] && z < 0) z = 0;
        if ((clipReturn[E_AXIS] && e > 0) || Printer::extrusionOutput.blocked()) e = 0;
        directCurrentSteps[X_AXIS] += x;
        directCurrentSteps[Y_AXIS] += y;
        directCurrentSteps[Z_AXIS] += z;
        directCurrentSteps[E_AXIS] += e;
        zMotion.setMotorReference(zMotion.motorSteps() + z);
        if (extrusionOutput.advanceEnabled()) {
            PrintLine move;
            move.moveE = true;
            move.positiveE = e > 0;
            move.delta[E_AXIS] = 1;
            for (long pulse = 0; pulse < std::labs(e); ++pulse)
                executeExtrusionStep(&move, false);
        } else {
            for (long pulse = 0; pulse < std::labs(e); ++pulse)
                pause.recordDirectExtrusion(e > 0 ? 1 : -1);
        }
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
    assert(Printer::extrusionOutput.blocked());
    sample(printingForce, 50);
    assert(!Printer::extrusionOutput.blocked() && Printer::pause.isWarmPause());
    extrudeFromPanel();
    const long manualExtruderPosition = -parkE + Printer::manualMotion.steps(E_AXIS);
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
    // A failed durable transaction must not publish the proposed pair.
    const auto old = controller.storedLimits();
    EEPROM::allowCommit = false;
    EEPROM::storeSenseOffsetSettings(3200, 55, true, true);
    assert(controller.storedLimits().thresholdDigits == old.thresholdDigits);
    assert(controller.storedLimits().maxMicrometers == old.maxMicrometers);
    EEPROM::allowCommit = true;
    GCode command;
    command.P = 3200;
    command.S = 55;
    const unsigned beforeCommit = EEPROM::persistenceCommits;
    configureSenseOffset(&command);
    assert(EEPROM::persistenceCommits == beforeCommit + 1);
    assert(queuedSenseOffsetEnables == 1 && controller.limits().thresholdDigits == senseThreshold);
    assert(controller.limits().maxMicrometers == 55);
    assert(HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS) == 3200);
    assert(HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX) == 55);
    const unsigned beforeQueue = HAL::eepromReads;
    Printer::enableSenseOffsetnow();
    assert(HAL::eepromReads == beforeQueue);
    assert(controller.limits().thresholdDigits == 3200);

    // The queue accepts only stored thresholds strictly inside its bounds.
    HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS, EMERGENCY_PAUSE_DIGITS_MAX);
    HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX, 300);
    EEPROM::publishSenseOffsetSettings();
    const unsigned beforeInvalidQueue = HAL::eepromReads;
    Printer::enableSenseOffsetnow();
    assert(HAL::eepromReads == beforeInvalidQueue);
    const auto limits = controller.limits();
    assert(limits.thresholdDigits == 3200 && limits.maxMicrometers == 55);
    HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS, 3200);
    HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX, 55);
    EEPROM::publishSenseOffsetSettings();

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
    EEPROM::publishSenseOffsetSettings();
    const unsigned beforeInterrupt = HAL::eepromReads;
    onInterrupt = Printer::enableSenseOffsetnow;
    interruptBeforeLock = 5;
    sample(printingForce);
    assert(!interruptBeforeLock && Printer::senseOffsetControl.limits().maxMicrometers == 21);
    assert(HAL::eepromReads == beforeInterrupt);
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
    assert(Printer::pause.resume() && Printer::printJob.commandsBlocked());
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
    assert(Printer::printJob.commandsBlocked() && !Printer::menuPaused);
    assert(Printer::pause.allowsQueueAdvance());
    Printer::pause.queueRequest(PrintPause::Destination::InPlace);
    Printer::pause.request(PrintPause::Destination::Park);
    assert(Printer::pause.allowsQueueAdvance() && !PrintLine::hasLines());

    Printer::printJob.service(clockMs + 2001);
    assert(Printer::printJob.isBusy() && incomingReads == 1 && !objectOutputs);
    PrintLine::direct.task = DIRECT_IDLE;
    g_nHeatBedScanStatus = g_nZOSScanStatus = 1;
    Printer::printJob.service(clockMs + 4001);
    assert(Printer::printJob.isBusy() && !objectOutputs);
    Printer::printJob.service(clockMs + 4002);
    assert(!Printer::printJob.isBusy() && !g_nHeatBedScanStatus && !g_nZOSScanStatus);
    assert(Extruder::bedTemperature == 0.0f && Extruder::current->tempControl.targetTemperatureC == 0.0f);
    assert(Printer::printJob.commandsBlocked());
    Printer::printJob.service(clockMs + 5002);
    assert(!objectOutputs);
    Printer::printJob.service(clockMs + 5003);
    Printer::printJob.service(clockMs + 6000);
    assert(!objectOutputs && Printer::printJob.commandsBlocked());
    Printer::printJob.servicePendingOutput(clockMs + 5002);
    assert(!objectOutputs && Printer::printJob.commandsBlocked());
    Printer::printJob.servicePendingOutput(clockMs + 5003);
    Printer::printJob.servicePendingOutput(clockMs + 6000);
    assert(objectOutputs == 1 && !Printer::printJob.commandsBlocked());

    // A USB stop during a heater wait must also discard the pending return.
    clockMs = 0;
    Printer::printJob.admitPrintMotion();
    Extruder::setTemperatureForExtruder(200.0f, 0);
    Printer::pause.request(PrintPause::Destination::Park);
    Printer::pause.serviceMotion();
    clockMs += EXTRUDER_CURRENT_PAUSE_DELAY + 1;
    Printer::pause.serviceStandby(clockMs);
    duringHeatWait = Printer::stopPrint;
    assert(Printer::pause.resume() && Printer::printJob.commandsBlocked());
    assert(Printer::pause.allowsQueueAdvance() && Printer::directCurrentSteps[Y_AXIS] == y + parkY);
}

void queuedPause() {
    const auto initialLines = PrintLine::linesCount;
    for (char* scan : {&g_nHeatBedScanStatus, &g_nZOSScanStatus}) {
        *scan = 1;
        Printer::pause.request(PrintPause::Destination::InPlace);
        Printer::pause.queueRequest(PrintPause::Destination::Park);
        Printer::pause.pressPause();
        assert(!Printer::pause.isPaused() && PrintLine::linesCount == initialLines);
        assert(PrintLine::queuedTask == QueueTask::None);
        *scan = 0;
    }
    Printer::pause.queueRequest(PrintPause::Destination::Park);
    assert(Printer::pause.isSettled() && !PrintLine::hasLines());
    Printer::pause.queueRequest(PrintPause::Destination::InPlace);
    assert(!PrintLine::hasLines() && Printer::pause.resume());
    assert(Printer::pause.allowsQueueAdvance());

    PrintLine::linesCount = MOVE_CACHE_SIZE;
    pauseDuringQueueWait = true;
    Printer::pause.queueRequest(PrintPause::Destination::Park);
    assert(Printer::pause.isSettled() && PrintLine::linesCount == MOVE_CACHE_SIZE);
    assert(PrintLine::queuedTask == QueueTask::None);
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
        for (int i = 0; i < 40; ++i)
            updateDigitCompensation(digits);
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

    Printer::setStepsPerMm(Z_AXIS, 2560.0f);
    layers.resetFadeBounds(Printer::stepsPerMm(Z_AXIS));
    Printer::zOffset.refreshSteps(Printer::stepsPerMm(Z_AXIS));
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

    Printer::setStepsPerMm(Z_AXIS, 640.0f);
    layers.rescaleFadeBounds(0.5f);
    assert(layers.fadeStartSteps() == 126 && layers.fadeEndSteps() == 626);
    assert(layers.currentLayerSteps() == 511 && layers.previousLayerSteps() == 253);

    layers.resetFadeBounds(Printer::stepsPerMm(Z_AXIS));
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
    check(2062, 0.8595f, 0.97f, 0, 0);

    Printer::doHeatBedZCompensation = true;
    readAt(95, 4500, 1);
    check(2672, 1, 0.98f, 1044, 63);
    readAt(101, 4500, 1);
    check(3129, 1, 0.99f, 1034, 67);

    assert(Printer::strainGauge.setZeroFromAverage(1000));
    readAt(112, 5500, 1);
    assert(Printer::strainGauge.lastKnownDigits() == 4500);
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
    Printer::gcodeModes.setRelativeEOverride(true);
    Printer::feedrateMultiply = 85;
    Printer::menuExtrusionFactor = 0.93f;
    PrintLine::linesCount = 0;
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

    assert(observedPrimeBrake && !PrintLine::hasLines());
    assert((skipAt || abortAt) ? primeMoves == (skipAt ? skipAt : abortAt) : primeMoves > 300);
    const auto profile = compensation.flowProfile();
    assert(profile.minDigits == 2000 && profile.maxDigits == 6100);
    assert(profile.extrusionPercent == 21 && profile.speedPercent == -30);
    assert(compensation.extrusionFactor() == 1 && compensation.feedrateFactor() == 1 && !compensation.intervalScale());
    assert(!Printer::startLine.active() && Printer::gcodeModes.relativeEOverride());
    assert(Printer::feedrateMultiply == 85 && Printer::menuExtrusionFactor == 0.93f);
    if (abortAt) {
        assert(Printer::printJob.commandsBlocked() && Printer::moveTarget.machineMM(E_AXIS) == 0.0f);
    } else {
        assert(Printer::moveTarget.machineMM(E_AXIS) == 10.0f && Printer::moveTarget.remainderMM(E_AXIS) == 0.00001f);
        assert(Printer::extrusionPlanner.stepRemainder() == 0.125f && Printer::extrusionCompensation.remainder() == 0.75f);
    }
}

void primeParameters() {
    PrintLine::linesCount = 0;
    GCode command;
    command.S = 6000;
    command.P = 2000;
    command.E = 1;
    command.F = 100;
    command.I = 9;
    command.Y = 500;

    // Neither scan mode may enter the priming state/snapshot frame.
    for (char* scan : {&g_nHeatBedScanStatus, &g_nZOSScanStatus}) {
        *scan = 1;
        Printer::startLine.run(command);
        assert(!primeMoves && !Printer::startLine.active());
        *scan = 0;
    }
    // A scan can start while an idle job is waiting for internal motion.
    Printer::printJob.finishPrintMotion();
    PrintLine::linesCount = 1;
    duringPeriodicActions = [] { g_nZOSScanStatus = 1; };
    Printer::startLine.run(command);
    duringPeriodicActions = nullptr;
    assert(!primeMoves && !Printer::startLine.active() && Printer::printJob.isIdle());
    assert(!PrintLine::hasLines());
    g_nZOSScanStatus = 0;

    // An existing filament owner is not a printing phase.
    assert(Printer::printJob.beginFilamentService());
    Printer::startLine.run(command);
    assert(!primeMoves && !Printer::startLine.active());
    Printer::printJob.finishFilamentService();
    Printer::printJob.admitPrintMotion();

    Printer::axesHomed = false;
    Printer::startLine.run(command);
    assert(!primeMoves);
    Printer::axesHomed = true;
    Printer::doHeatBedZCompensation = false;
    Printer::startLine.run(command);
    assert(!primeMoves);
    Printer::doHeatBedZCompensation = true;

    duringPeriodicActions = [] {
        const auto profile = Printer::digitCompensation.flowProfile();
        assert(profile.minDigits == 2000 && profile.maxDigits == 6000);
    };
    Printer::startLine.run(command);
    duringPeriodicActions = nullptr;
    assert(!PrintLine::hasLines() && primeSpacingMoves == 5);
    assert(primeApproach.x == 15.0f && primeApproach.y == 122.5f && primeApproach.z == 0.35f);
    assert(primeApproach.e == IGNORE_COORDINATE && primeApproach.speed == 60.0f);
    assert(firstPrimeSegment.speed == 50.0f && firstPrimeSegment.y == 122.5f);
    assert(std::abs(firstPrimeSegment.x - 15.35f) < 0.00001f);
    assert(std::abs(firstPrimeSegment.e - 0.0175f) < 0.00001f);
}

void extrusionPlanning() {
    Printer::setStepsPerMm(E_AXIS, 1.0f);
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
    Printer::setStepsPerMm(E_AXIS, 280.0f);
    int32_t steps = 0;
    for (int i = 0; i < 100000; ++i) {
        Printer::moveTarget.addMM(E_AXIS, 0.008f);
        steps += planExtrusionMove();
    }
    assert(steps == 224000);
}

void printJob() {
    auto& job = Printer::printJob;
    assert(!job.isBusy() && !job.commandsBlocked());
    assert(PrintLine::hasLines()); // Artificial queue entries alone do not start a print.

    Extruder::current->tempControl.calibrationValid = false;
    GCode untuned;
    untuned.X = 10;
    untuned.E = 1;
    const float beforeX = Printer::moveTarget.machineMM(X_AXIS);
    const float beforeE = Printer::moveTarget.machineMM(E_AXIS);
    assert(!Printer::queueGCodeCoordinates(&untuned) && !job.isPrinting());
    assert(Printer::moveTarget.machineMM(X_AXIS) == beforeX && Printer::moveTarget.machineMM(E_AXIS) == beforeE);
    Extruder::current->tempControl.calibrationValid = true;

    GCode command;
    command.X = 10;
    command.internalCommand = true;
    assert(Printer::queueGCodeCoordinates(&command) && !job.isPrinting());

    command.internalCommand = false;
    PrintLine::publishesMove = false;
    assert(!Printer::queueGCodeCoordinates(&command) && !job.isPrinting());

    PrintLine::publishesMove = true;
    PrintLine::linesCount = 1;
    assert(Printer::queueGCodeCoordinates(&command) && job.isPrinting());
    Printer::pause.restoreDefaultParking();
    Printer::pause.setParkSteps(E_AXIS, 0);
    Printer::pause.request(PrintPause::Destination::InPlace);
    Printer::pause.serviceMotion();
    PrintLine::linesCount = 0;
    Printer::idleStatus.schedule(clockMs);
    clockMs += MINIMAL_IDLE_TIME + 1;
    handleStartStandby(clockMs);
    assert(job.isPrinting() && Printer::pause.isPaused());

    assert(Printer::pause.resume());
    clockMs += MINIMAL_IDLE_TIME + 1;
    handleStartStandby(clockMs);
    assert(!job.isBusy() && !Printer::pause.isPaused());

    job.beginFilamentService();
    assert(job.isBusy() && !job.isPrinting());
    Printer::stopPrint();
    assert(job.commandsBlocked() && !job.isPrinting());
    job.finishFilamentService();
    assert(job.commandsBlocked());
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

    Printer::gcodeModes.setRelativeXYZ(true);
    command.X = 0;
    command.Y = 0.25f;
    command.E = 1;
    assert(Printer::queueGCodeCoordinates(&command));
    assert(Printer::moveTarget.machineMM(X_AXIS) == 10 && Printer::moveTarget.machineMM(Y_AXIS) == 40.25f);
    assert(Printer::moveTarget.machineMM(E_AXIS) == 3);

    Printer::gcodeModes.setRelativeXYZ(false);
    Printer::gcodeModes.setRelativeEOverride(true);
    command.X = 12.7f;
    command.Y = command.E = command.F = 0;
    assert(Printer::queueGCodeCoordinates(&command));
    assert(std::abs(Printer::moveTarget.machineMM(X_AXIS) - 1.7f) < 0.00001f);
    assert(Printer::moveTarget.machineMM(Y_AXIS) == 40.25f);

    command.X = 0;
    command.E = -0.5f;
    assert(Printer::queueGCodeCoordinates(&command));
    assert(!PrintLine::updatesWobble && Printer::moveTarget.machineMM(E_AXIS) == 2.5f);
    command.E = 0;
    assert(!Printer::queueGCodeCoordinates(&command));
    assert(PrintLine::plannedMoves == 4);

    Printer::printJob.finishPrintMotion();
    const float priorFeedrate = Printer::feedrate;
    assert(Printer::queueInternalAxisMove(E_AXIS, -0.5f));
    assert(Printer::moveTarget.machineMM(E_AXIS) == 2.0f && !PrintLine::updatesWobble);
    assert(Printer::feedrate == priorFeedrate && !Printer::printJob.isPrinting());
    assert(Printer::queueInternalAxisMove(Y_AXIS, 37, 4800));
    assert(Printer::moveTarget.machineMM(Y_AXIS) == 40 && PrintLine::updatesWobble);
    assert(std::abs(Printer::feedrate - 80) < 0.0001f && !Printer::printJob.isPrinting());
    Extruder::current->tempControl.calibrationValid = false;
    assert(!Printer::queueInternalAxisMove(E_AXIS, 1, 100));
    assert(Printer::moveTarget.machineMM(E_AXIS) == 2.0f);
    assert(std::abs(Printer::feedrate - 80) < 0.0001f);
    assert(!Printer::queueInternalAxisMove(E_AXIS + 1, 1, 100));
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
    Printer::xyMotion.setMotorSteps(X_AXIS, Printer::xyMaxSteps(X_AXIS) / 2);
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

    Printer::setStepsPerMm(X_AXIS, 320.0f);
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
        Printer::extrusionOutput.setAdvanceEnabled(advance);
        Printer::extrusionOutput.clearSteps();
        Printer::extrusionCompensation.publishRatio(ExtrusionCompensation::prepareRatio(ratio));
        Printer::extrusionCompensation.restoreRemainder(remainder);
        const unsigned before = Extruder::stepPulses;

        move.positiveE = false;
        for (unsigned i = 0; i < count; ++i)
            executeExtrusionStep(&move, true);
        const unsigned afterRetract = Extruder::stepPulses;
        const int retracted = advance ? -Printer::extrusionOutput.pendingSteps() : int(afterRetract - before);

        move.positiveE = true;
        for (unsigned i = 0; i < count; ++i)
            executeExtrusionStep(&move, true);
        const int returned = advance ? Printer::extrusionOutput.pendingSteps() + retracted
                                      : int(Extruder::stepPulses - afterRetract);
        assert(retracted == expectedSteps && returned == retracted);
        assert(Printer::extrusionCompensation.remainder() == remainder);
        assert(Printer::extrusionOutput.pendingSteps() == 0);
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
    auto& advance = Printer::extrusionOutput;
    const unsigned basePulses = Extruder::stepPulses;
    const unsigned baseTrailingEdges = Extruder::trailingEdges;
    InterruptProtectedBlock lock;
    advance.setStepInterval(12);
    assert(!advanceTimerArmed());
    advance.setAdvanceEnabled(true);
    assert(!advanceTimerArmed());

    advance.addSteps(1);
    assert(!fireAdvanceCompare() && advance.pendingSteps() == 1);
    assert(Extruder::stepPulses == basePulses && Extruder::trailingEdges == baseTrailingEdges);
    TCNT0 = 250;
    TIFR0.pending = 0x07; // Stale A and pending B and overflow flags.
    advance.addSteps(1);
    assert(advanceTimerArmed() && EXTRUDER_OCR == 6 && TIFR0.pending == 0x05);
    assert(fireAdvanceCompare() && EXTRUDER_OCR == 58 && Extruder::current->stepperDirection == 1);
    assert(fireAdvanceCompare() && EXTRUDER_OCR == 70 && advance.pendingSteps() == 1);
    assert(fireAdvanceCompare() && advance.pendingSteps() == 0 && !advanceTimerArmed());
    assert(Extruder::stepPulses == basePulses + 2);

    advance.setLeadSteps(3);
    advance.setLeadSteps(5);
    for (int i = 0; i < 5; ++i)
        assert(fireAdvanceCompare());
    assert(advance.pendingSteps() == 0 && !advanceTimerArmed());
    advance.postponeRelaxation();
    for (int tick = 0; tick < 70; ++tick) {
        assert(!advance.relaxDelayElapsed());
        advance.onIdleTick();
        assert(!advanceTimerArmed());
    }
    assert(advance.relaxDelayElapsed());
    TCNT0 = 228;
    advance.onIdleTick();
    assert(advance.pendingSteps() == -5 && advanceTimerArmed() && EXTRUDER_OCR == 240);
    assert(fireAdvanceCompare() && EXTRUDER_OCR == 36 && Extruder::current->stepperDirection == -1);
    for (int i = 0; i < 5; ++i)
        assert(fireAdvanceCompare());
    assert(advance.pendingSteps() == 0 && !advanceTimerArmed());

    // A single reversal wakes for a drain. Canceling the drain lets the
    // pending compare retire without a pulse.
    advance.addSteps(1);
    assert(!advanceTimerArmed() && !advance.drain() && advanceTimerArmed());
    advance.cancelDrain();
    assert(fireAdvanceCompare() && !advanceTimerArmed() && advance.pendingSteps() == 1);
    assert(!advance.drain());
    assert(fireAdvanceCompare() && Extruder::current->stepperDirection == 1);
    assert(fireAdvanceCompare() && advance.drain() && !advanceTimerArmed());

    TCNT0 = 229;
    advance.addSteps(2);
    assert(EXTRUDER_OCR == 241);
    assert(fireAdvanceCompare(17) && EXTRUDER_OCR == 14);
    assert(fireAdvanceCompare() && !advanceTimerArmed());
    assert(Extruder::stepPulses == basePulses + 15 && Extruder::trailingEdges == baseTrailingEdges + 15);

    advance.addSteps(2);
    assert(advanceTimerArmed());
    const auto reason = ExtrusionOutput::Inhibit::Pressure;
    assert(advance.inhibit(reason) == 2 && !advanceTimerArmed());
    advance.allow(reason);
    assert(!fireAdvanceCompare() && !advance.pendingSteps());
    advance.addSteps(1);
    assert(advanceTimerArmed());
    advance.setAdvanceEnabled(false);
    assert(!fireAdvanceCompare());
    advance.clearSteps();

    advance.setStepInterval(1);
    advance.setAdvanceEnabled(true);
    TCNT0 = 255;
    advance.addSteps(2);
    assert(EXTRUDER_OCR == 1);
    assert(fireAdvanceCompare() && EXTRUDER_OCR == 42);
    assert(fireAdvanceCompare() && EXTRUDER_OCR == 44 && TIFR0.pending == otherTimer0Interrupts);
    assert(fireAdvanceCompare() && advance.pendingSteps() == 0 && !advanceTimerArmed());
}

#if NUM_EXTRUDER > 1
void advanceToolChange() {
    auto& advance = Printer::extrusionOutput;
    advance.setAdvanceEnabled(true);
    advance.setStepInterval(12);
    extruder[1].id = 1;
    extruder[1].offsetMM[X_AXIS] = 2.0f;
    extruder[0].enabled = true;

    const auto tick = [] { fireAdvanceCompare(); };
    advance.addSteps(2);
    tick();
    tick();
    tick();
    advance.setLeadSteps(1);
    tick();
    assert(extruder[0].deliveredSteps == 3 && advance.pendingSteps() == 0);

    PrintLine::linesCount = 2;
    unsigned waits = 0;
    duringPeriodicActions = [&] {
        assert(Extruder::current == &extruder[0]);
        if (PrintLine::hasLines())
            assert(advance.pendingSteps() == 0);
        // A nested panel selection cannot take over the motor during the drain.
        assert(!Extruder::selectExtruderById(1));
        ++waits;
        tick();
    };
    duringMove = [](long x, long y, long z, long e) {
        assert(Extruder::current == &extruder[1]);
        assert(x == -320 && !y && !z && !e);
    };
    assert(Extruder::selectExtruderById(1) && waits >= 4);
    duringMove = nullptr;
    duringPeriodicActions = nullptr;
    assert(!extruder[0].enabled && extruder[0].deliveredSteps == 2);
    assert(Extruder::current == &extruder[1] && extruder[1].deliveredSteps == 0);

    advance.addSteps(-2);
    tick();
    tick();
    tick();
    assert(extruder[1].deliveredSteps == -2);
    advance.addSteps(1);
    tick();
    assert(advance.pendingSteps() == 1);
    duringPeriodicActions = tick;
    assert(Extruder::selectExtruderById(0));
    duringPeriodicActions = nullptr;
    assert(advance.pendingSteps() == 0);
    assert(extruder[0].deliveredSteps == 2 && extruder[1].deliveredSteps == -1);

    Printer::pause.request(PrintPause::Destination::InPlace);
    assert(!Extruder::selectExtruderById(1));
    assert(Extruder::current == &extruder[0]);

    pollDuringMoves = false;
    duringPeriodicActions = [&] {
        assert(Extruder::current == &extruder[0]);
        Printer::pause.cancel();
    };
    assert(Extruder::selectExtruderById(1, true));
    duringPeriodicActions = nullptr;
    assert(Extruder::current == &extruder[1]);
}
#endif

void advanceModeChange() {
    auto& advance = Printer::extrusionOutput;
    advance.setStepInterval(15);

    GCode testStep;
    testStep.S = 20;
    injectAdvanceTestSteps(&testStep);
    assert(advance.pendingSteps() == 0);

    PrintLine::linesCount = 2;
    GCode setting;
    setting.Y = 20;
    unsigned calls = 0;
    duringPeriodicActions = [&] {
        ++calls;
        assert(!advance.advanceEnabled() && Extruder::current->advanceL == 0);
    };
    runAdvanceCommand(233, &setting);
    duringPeriodicActions = nullptr;
    assert(calls == 2 && advance.advanceEnabled() && Extruder::current->advanceL == 20);
    assert(advance.pendingSteps() == 0);

    testStep.S = 2;
    injectAdvanceTestSteps(&testStep);
    advance.setLeadSteps(1);
    const auto tick = [] { fireAdvanceCompare(); };
    for (int i = 0; i < 4; ++i)
        tick();
    assert(Extruder::current->deliveredSteps == 3 && advance.pendingSteps() == 0);

    setting.explicitY = true;
    setting.Y = 0;
    duringPeriodicActions = [&] {
        assert(advance.advanceEnabled() && Extruder::current->advanceL == 20);
        tick();
    };
    runAdvanceCommand(233, &setting);
    duringPeriodicActions = nullptr;
    assert(!advance.advanceEnabled() && Extruder::current->advanceL == 0);
    assert(Extruder::current->deliveredSteps == 2 && advance.pendingSteps() == 0);

    injectAdvanceTestSteps(&testStep);
    setting.Y = 30;
    runAdvanceCommand(233, &setting);
    assert(advance.advanceEnabled() && advance.pendingSteps() == 0);
}

void motionWait() {
    using namespace Commands;
    PrintLine::direct.task = DIRECT_RUNNING;
    PrintLine::linesCount = 2;
    unsigned polls = 0;
    duringPeriodicActions = [&] {
        ++polls;
        assert(PrintLine::directMoveActive());
    };
    assert(waitForMotion(MotionWait::QueueAndZ, PauseWait::Resume));
    assert(polls == 2 && PrintLine::directMoveActive());

    Printer::pause.request(PrintPause::Destination::InPlace);
    PrintLine::linesCount = 1;
    assert(!waitForMotion(MotionWait::QueueAndZ, PauseWait::Cancel));
    assert(!waitForMotion(MotionWait::AllOutputs, PauseWait::Cancel));
    assert(polls == 2 && PrintLine::linesCount == 1 && PrintLine::directMoveActive());

    duringPeriodicActions = [&] {
        ++polls;
        Printer::pause.cancel();
        PrintLine::direct.task = DIRECT_IDLE;
    };
    assert(waitForMotion(MotionWait::AllOutputs, PauseWait::Resume));
    assert(polls == 4 && !PrintLine::hasLines());

    PrintLine::linesCount = 1;
    duringPeriodicActions = [] { Printer::stopPrint(); };
    assert(!waitForMotion(MotionWait::QueueAndZ, PauseWait::Resume));
    assert(Printer::printJob.commandsBlocked());
    duringPeriodicActions = nullptr;
}

void advanceCancel() {
    auto& output = Printer::extrusionOutput;
    output.setAdvanceEnabled(true);
    output.setStepInterval(15);
    output.setLeadSteps(3);
    output.addSteps(2);
    Printer::stopPrint();
    assert(output.blocked() && output.pendingSteps() == 0);
    assert(Printer::printJob.commandsBlocked() && !PrintLine::hasLines());
    assert(!Commands::waitForMotion(Commands::MotionWait::AllOutputs, Commands::PauseWait::Resume));

    PrintLine move;
    move.moveE = true;
    move.positiveE = true;
    move.delta[E_AXIS] = 1;
    move.stepProfile.advanceL = 65536;

    executeExtrusionStep(&move, false); // A paused jog can still have an XYZ tail.
    StepExecutor::updateAdvanceSteps(move, 4);
    GCode testStep;
    testStep.S = 5;
    injectAdvanceTestSteps(&testStep);
    {
        InterruptProtectedBlock lock;
        output.onIdleTick();
        assert(!fireAdvanceCompare());
    }
    assert(!output.pendingSteps() && !Extruder::stepPulses);

    // Cleanup releases its own inhibit, preserving any motor shutdown.
    output.inhibit(ExtrusionOutput::Inhibit::MotorsDisabled);
    Printer::printJob.service(clockMs + 2001);
    Printer::printJob.service(clockMs + 3002);
    assert(!objectOutputs && Printer::printJob.commandsBlocked() && output.blocked());
    Printer::printJob.servicePendingOutput(clockMs + 3002);
    assert(objectOutputs == 1 && !Printer::printJob.commandsBlocked() && output.blocked());
    output.allow(ExtrusionOutput::Inhibit::MotorsDisabled);
    assert(!output.blocked());
    {
        InterruptProtectedBlock lock;
        output.onIdleTick();
        assert(!fireAdvanceCompare());
    }
    assert(!output.pendingSteps() && !Extruder::stepPulses);
}

void tickAdvanceInterrupt() {
    fireAdvanceCompare();
}

void manualExtrusionDuringPause(bool advance) {
    auto& output = Printer::extrusionOutput;
    output.setAdvanceEnabled(advance);
    output.setStepInterval(15);
    if (advance)
        duringPeriodicActions = tickAdvanceInterrupt;

    Printer::setEPositionMM(42.5f, 0.000125f);
    Printer::extrusionPlanner.restoreStepRemainder(0.25f);
    Printer::extrusionCompensation.restoreRemainder(0.375f);
    auto untouchedPlanner = Printer::extrusionPlanner;
    const auto motor = [&] {
        return advance ? long(Extruder::current->deliveredSteps) : Printer::directCurrentSteps[E_AXIS];
    };

    Printer::pause.request(PrintPause::Destination::InPlace);
    Printer::pause.serviceMotion();
    assert(Printer::pause.isSettled() && motor() == -parkE);

    // Play can arrive once Timer1 releases the slot but the foreground is
    // still completing the manual move.
    Printer::manualMotion.setSteps(E_AXIS, 2 * parkE);
    duringMove = [](long, long, long, long e) {
        if (e == Printer::manualMotion.steps(E_AXIS))
            assert(!Printer::pause.resume());
    };
    extrudeFromPanel();
    duringMove = nullptr;
    Printer::manualMotion.setSteps(E_AXIS, parkE / 4);
    extrudeFromPanel(UI_ACTION_RF_EXTRUDER_RETRACT);
    assert(motor() == 3 * parkE / 4 && output.pendingSteps() == 0);

    const long preparedFilament = motor();
    assert(Printer::pause.resume() && Printer::pause.allowsQueueAdvance());
    assert(motor() == preparedFilament);
    assert(Printer::moveTarget.machineMM(E_AXIS) == 42.5f);
    assert(Printer::moveTarget.remainderMM(E_AXIS) == 0.000125f);
    assert(Printer::extrusionPlanner.stepRemainder() == 0.25f);
    assert(Printer::extrusionCompensation.remainder() == 0.375f);

    // A completely blocked attempt leaves the automatic retract to recover.
    Printer::pause.request(PrintPause::Destination::InPlace);
    Printer::pause.serviceMotion();
    assert(motor() == preparedFilament - parkE);
    if (advance) {
        duringMove = [](long, long, long, long e) {
            if (e == Printer::manualMotion.steps(E_AXIS)) {
                // Timer1 admitted the jog. DMS trips before COMPA emits it.
                sample(EMERGENCY_STOP_DIGITS_MAX + 1, 2 * EMERGENCY_STOP_CHECKS);
            }
        };
    } else {
        sample(EMERGENCY_STOP_DIGITS_MAX + 1, 2 * EMERGENCY_STOP_CHECKS);
    }
    extrudeFromPanel();
    duringMove = nullptr;
    assert(output.blocked() && !output.pendingSteps() && motor() == preparedFilament - parkE);
    sample(0, 2 * EMERGENCY_STOP_CHECKS);
    assert(Printer::pause.resume() && Printer::pause.allowsQueueAdvance());
    assert(motor() == preparedFilament);

    GCode next;
    next.E = 43.0f;
    assert(Printer::queueGCodeCoordinates(&next));
    const float flow = Printer::menuExtrusionFactor * Printer::digitCompensation.extrusionFactor();
    assert(planExtrusionMove() == untouchedPlanner.planSteps({next.E, 0}, flow, Printer::stepsPerMm(E_AXIS)));
}

void filamentProgramDuringPause() {
    Printer::setEPositionMM(42.5f, 0.000125f);
    Printer::extrusionPlanner.restoreStepRemainder(0.25f);
    Printer::gcodeModes.setRelativeEOverride(true);
    const float targetTemperature = Extruder::current->tempControl.targetTemperatureC;
    const int queued = PrintLine::linesCount;

    GCode command;
    // A live print is rejected before any wait or E-coordinate change.
    assert(Printer::printJob.isPrinting());
    runFilamentCommand(3913, &command);
    assert(!std::strcmp(Com::lastLine, "Filament program requires idle print job and no scan"));
    assert(Printer::moveTarget.machineMM(E_AXIS) == 42.5f);
    assert(Printer::printJob.isPrinting() && PrintLine::linesCount == queued);

    // Idle ownership with internal queued motion still requires a settled
    // unpaused queue. A pause can occur during this actual motion wait.
    Printer::printJob.finishPrintMotion();
    duringPeriodicActions = [] { Printer::pause.request(PrintPause::Destination::InPlace); };
    runFilamentCommand(3913, &command);
    duringPeriodicActions = nullptr;
    assert(Printer::pause.isSettled() && PrintLine::linesCount == queued);
    assert(!std::strcmp(Com::lastLine, "Filament program requires settled, unpaused motion"));

    for (const int commandNumber : {3913, 3914}) {
        runFilamentCommand(commandNumber, &command);
        assert(!std::strcmp(Com::lastLine, "Filament program requires settled, unpaused motion"));
        assert(Printer::pause.isSettled() && PrintLine::linesCount == queued);
        assert(Printer::moveTarget.machineMM(E_AXIS) == 42.5f);
        assert(Printer::moveTarget.remainderMM(E_AXIS) == 0.000125f);
        assert(Printer::extrusionPlanner.stepRemainder() == 0.25f);
        assert(Printer::gcodeModes.relativeEOverride());
        assert(Extruder::current->tempControl.targetTemperatureC == targetTemperature);
    }
}

void finishAdvanceBeforeAndAfterOutput() {
    PrintLine::linesCount=0;
    for (char* scan : {&g_nHeatBedScanStatus, &g_nZOSScanStatus}) {
        *scan=1;
        outputObject(false);
        assert(!objectOutputs && !motorShutdowns);
        *scan=0;
    }
    Printer::printJob.finishPrintMotion();
    assert(Printer::printJob.beginFilamentService());
    outputObject(false);
    assert(!objectOutputs && !motorShutdowns);
    Printer::printJob.finishFilamentService();
    Printer::printJob.admitPrintMotion();

    auto& output = Printer::extrusionOutput;
    output.setAdvanceEnabled(true);
    output.setStepInterval(15);
    output.addSteps(2);
    output.setLeadSteps(1);
    for (int i = 0; i < 4; ++i)
        tickAdvanceInterrupt();
    assert(Extruder::current->deliveredSteps == 3);

    PrintLine::linesCount = 0;
    duringPeriodicActions = tickAdvanceInterrupt;
    duringOutputScript = [&] {
        assert(Extruder::current->deliveredSteps == 2 && !output.pendingSteps());
        assert(motorShutdowns == 0);
        output.addSteps(-1);
    };
    outputObject(true);
    assert(objectOutputs == 1 && motorShutdowns == 1);
    assert(Extruder::current->deliveredSteps == 1 && !output.pendingSteps());
}

void advancePauseWithPressureLimit() {
    auto& output = Printer::extrusionOutput;
    output.setAdvanceEnabled(true);
    output.setStepInterval(15);
    output.addSteps(2);
    output.setLeadSteps(1);
    for (int i = 0; i < 4; ++i)
        tickAdvanceInterrupt();
    assert(Extruder::current->deliveredSteps == 3);

    pollDuringMoves = false;
    duringPeriodicActions = tickAdvanceInterrupt;
    duringMove = [](long, long, long, long e) {
        assert(e == -parkE && Extruder::current->deliveredSteps == 2);
        for (int pulse = 0; pulse < 3; ++pulse)
            tickAdvanceInterrupt();
        assert(Extruder::current->deliveredSteps == -1);
        sample(EMERGENCY_STOP_DIGITS_MAX + 1, EMERGENCY_STOP_CHECKS);
        assert(Printer::extrusionOutput.blocked() && !Printer::extrusionOutput.pendingSteps());
    };
    Printer::pause.request(PrintPause::Destination::InPlace);
    Printer::pause.serviceMotion();
    assert(Printer::pause.isSettled() && Extruder::current->deliveredSteps == -1);

    sample(0, EMERGENCY_STOP_CHECKS);
    assert(!output.blocked());
    duringMove = [](long, long, long, long e) {
        assert(e == 3); // Only three physical retract steps remain to return.
        tickAdvanceInterrupt(); // DIR changes first.
        tickAdvanceInterrupt();
        assert(Extruder::current->deliveredSteps == 0);
        sample(EMERGENCY_STOP_DIGITS_MAX + 1, EMERGENCY_STOP_CHECKS);
    };
    assert(Printer::pause.resume() && Printer::pause.isSettled() && incompleteReturns == 1);

    sample(0, EMERGENCY_STOP_CHECKS);
    duringMove = [](long, long, long, long e) { assert(e == 2); };
    assert(Printer::pause.resume() && Printer::pause.allowsQueueAdvance());
    duringMove = nullptr;
    duringPeriodicActions = nullptr;
    assert(Extruder::current->deliveredSteps == 2 && !output.pendingSteps());
}

void stepClock() {
    PrintLine move;
    StepExecutor::StepClock queued, direct;
    queued.start(100);
    direct.start(125);
    move.stepProfile.accelSteps = 2;
    move.stepProfile.decelSteps = 3;
    move.stepsRemaining = 90;
    move.stepProfile.fAcceleration = 1024;
    move.stepProfile.vStart = 100;
    move.stepProfile.vMax = 300;
    move.stepProfile.vEnd = 75;
    constexpr uint32_t interval = 150 * 256;
    assert(StepExecutor::nextStepSpeed(move, queued, 1, interval) == 250);
    assert(StepExecutor::nextStepSpeed(move, queued, 1, interval) == 300);
    assert(StepExecutor::nextStepSpeed(move, queued, 1, interval) == 300);
    assert(direct.steps == 0 && direct.ticks == 0 && direct.peak == 125);

    move.stepsRemaining = 3;
    assert(StepExecutor::nextStepSpeed(move, queued, 1, interval) == 300);
    assert((move.flags & FLAG_DECELERATING) && queued.ticks == 0);
    assert(StepExecutor::nextStepSpeed(move, queued, 1, interval) == 150);
    assert(StepExecutor::nextStepSpeed(move, queued, 1, interval) == 75);
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
    assert(move.primaryAxis == X_AXIS && move.stepsRemaining == 10 && move.lookahead.distance == 5);

    move.setAxisSteps(Z_AXIS, 10);
    move.setSegmentGeometry(distances);
    assert(move.primaryAxis == Z_AXIS && move.stepsRemaining == 10 && move.lookahead.distance == 13);

    move.setAxisSteps(E_AXIS, -10);
    move.setSegmentGeometry(distances);
    assert(move.primaryAxis == E_AXIS && move.stepsRemaining == 10 && move.lookahead.distance == 13);

    PrintLine retract;
    retract.setAxisSteps(E_AXIS, -1);
    retract.setSegmentGeometry(distances);
    assert(retract.isNegativeMoveOfAxis(E_AXIS));
    assert(retract.primaryAxis == E_AXIS && retract.stepsRemaining == 1 && retract.lookahead.distance == 2);
}

void moveQueue() {
    auto& queue = PrintLine::queue;
    Printer::idleStatus.schedule(500);
    for (int count = 1; count < MOVE_CACHE_SIZE; ++count) {
        queue.writeSlot().setQueueTaskId(QueueTask::DisableZCompensation);
        queue.publish(false);
        assert(queue.count() == count && queue.active() == nullptr);
    }
    assert(Printer::idleStatus.poll(0, 500) == IdleStatus::Transition::Deferred);
    auto* move = queue.activateFront();
    assert(PrintLine::activeMove().source == PrintLine::ActiveMove::Source::None);
    move->setQueueTaskId(QueueTask::None);
    move->setAxisSteps(Z_AXIS, 1);
    move->stepsRemaining = 1;
    assert(PrintLine::activeMove().usesAxis(Z_AXIS) && interruptsEnabled);
    PrintLine::reserveDirectMove(false);
    assert(PrintLine::activeMove().source == PrintLine::ActiveMove::Source::None);
    PrintLine::direct.task = DIRECT_IDLE;
    queue.clear();
    assert(PrintLine::activeMove().source == PrintLine::ActiveMove::Source::None && interruptsEnabled);
    assert(queue.count() == 0 && queue.active() == nullptr);
    assert(queue.readIndex() == MOVE_CACHE_SIZE - 1 && queue.writeIndex() == queue.readIndex());

    assert(PrintLine::insertWaitMovesIfNeeded(1, 0) == 1);
    assert(queue.count() == 4 && Printer::idleStatus.poll(0, 500) == IdleStatus::Transition::Waiting);
    for (int depth = 3; depth >= 0; --depth) {
        auto* move = queue.activateFront();
        assert(move->queueTaskId() == QueueTask::None && move->flags == FLAG_WARMUP);
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

void moveReserve() {
    MoveQueue queue;
    const auto check = [&](float xyz, float filament, float seconds) {
        const auto reserve = queue.remainingMotion();
        assert(std::abs(reserve.xyzMm - xyz) < 0.001f);
        assert(std::abs(reserve.filamentMm - filament) < 0.001f);
        assert(std::abs(reserve.seconds - seconds) < 0.0001f);
        assert(interruptsEnabled);
    };
    check(0, 0, 0);
    for (int i = 0; i < MOVE_CACHE_SIZE - 1; ++i) {
        queue.publishTask(QueueTask::EnableZCompensation);
        queue.activateFront();
        queue.retireFront();
        HAL::allowInterrupts();
    }

    PrintLine& travel = queue.writeSlot();
    travel = PrintLine{};
    travel.setAxisSteps(X_AXIS, 16000);
    const float travelDistances[] = {100, 0, 0, 0};
    travel.setSegmentGeometry(travelDistances);
    travel.lookahead.speedX = 100;
    travel.lookahead.invFullSpeed = 0.01f;
    travel.lookahead.accelerationPrim = 100000;
    travel.stepProfile.vStart = 4000;
    travel.stepProfile.vEnd = 8000;
    travel.stepProfile.vMax = 16000;
    travel.stepProfile.accelSteps = 1200;
    travel.stepProfile.decelSteps = 960;
    queue.publish(true);
    assert(queue.writeIndex() == 0);

    // Reused task and warmup slots can contain geometry from an earlier move.
    queue.writeSlot() = travel;
    queue.publishTask(QueueTask::EnableZCompensation);
    queue.writeSlot() = travel;
    queue.writeSlot().flags = FLAG_WARMUP;
    queue.publish(true);

    PrintLine& retract = queue.writeSlot();
    retract = PrintLine{};
    retract.setAxisSteps(E_AXIS, -2800);
    const float retractDistances[] = {0, 0, 0, 10};
    retract.setSegmentGeometry(retractDistances);
    retract.lookahead.speedE = -25;
    retract.lookahead.invFullSpeed = 0.04f;
    retract.stepProfile.vStart = retract.stepProfile.vEnd = retract.stepProfile.vMax = 7000;
    queue.publish(true);

    check(100, 10, 1.465f);
    assert(queue.activateFront() == &travel);
    travel.stepsRemaining = 8000;
    check(50, 10, 0.920f);
    queue.retireFront();
    HAL::allowInterrupts();
    check(0, 10, 0.4f);
    queue.clear();
    check(0, 0, 0);

    PrintLine& mixed = queue.writeSlot();
    mixed = PrintLine{};
    mixed.setAxisSteps(X_AXIS, 480);
    mixed.setAxisSteps(Y_AXIS, 640);
    mixed.setAxisSteps(E_AXIS, 2800);
    const float mixedDistances[] = {3, 4, 0, 10};
    mixed.setSegmentGeometry(mixedDistances);
    MotionPlanner::prepareMove(mixed, mixedDistances, 100, queue);
    MotionPlanner::prepareStepProfile(mixed);
    queue.publish(true);
    const auto reserve = queue.remainingMotion();
    assert(std::abs(reserve.xyzMm - 5) < 0.001f);
    assert(std::abs(reserve.filamentMm - 10) < 0.001f);
    assert(reserve.seconds >= 0.399f);
}

void idleStatusTransitions() {
    IdleStatus status;
    using Transition = IdleStatus::Transition;
    constexpr uint32_t delay = 500;

    status.schedule(1000);
    assert(status.poll(999, delay) == Transition::Deferred);
    assert(status.poll(1500, delay) == Transition::Waiting);
    assert(status.poll(1501, delay) == Transition::Ready);
    assert(status.poll(1502, delay) == Transition::Waiting);

    status.schedule(UINT32_MAX - 200);
    assert(status.poll(300, delay) == Transition::Ready);
    status.schedule(0);
    assert(status.poll(UINT32_MAX, delay) == Transition::Deferred);
    assert(status.poll(501, delay) == Transition::Ready);

    status.schedule(1000);
    status.cancel();
    assert(status.poll(2000, delay) == Transition::Waiting && interruptsEnabled);
}

void advanceProfileReport() {
    PrintLine move;
    move.setAxisSteps(X_AXIS, 16000);
    move.setAxisSteps(E_AXIS, 5600);
    const float axisDistances[] = {100, 0, 0, 20};
    move.setSegmentGeometry(axisDistances);
    Extruder::current->advanceL = 100.0f;
    MotionPlanner::prepareMove(move, axisDistances, 25, PrintLine::queue);

    // Verify the actual planned pressure advance, not removed telemetry.
    assert(move.stepProfile.vMax > 0);
    const uint32_t expected = uint32_t(65536.0f * 140.0f / move.stepProfile.vMax);
    assert(move.stepProfile.advanceL == expected);
}

void lookaheadMoves() {
    MoveQueue& queue = PrintLine::queue;
    for (int index = 0; index < MOVE_CACHE_SIZE - 2; ++index) {
        queue.publish(false);
        queue.activateFront();
        queue.retireFront();
        HAL::allowInterrupts();
    }

    constexpr uint8_t printX = 16 | 128, printY = 32 | 128, travelX = 16, retract = 128;
    const auto prepare = [&](uint8_t axes) -> PrintLine& {
        PrintLine& move = queue.writeSlot();
        move = PrintLine{};
        move.dir = axes;
        move.lookahead.speedX = (axes & 16) ? 32.0f : 0.0f;
        move.lookahead.speedY = (axes & 32) ? 32.0f : 0.0f;
        move.lookahead.speedE = axes == retract ? -32.0f : (axes & 128) ? 2.0f : 0.0f;
        move.lookahead.distance = 0.3f;
        move.timeInTicks = 50000;
        move.fullSpeed = 32.0f;
        move.lookahead.invFullSpeed = 1.0f / 32.0f;
        move.lookahead.minSpeed = move.lookahead.startSpeed = move.lookahead.endSpeed = 4.0f;
        move.lookahead.accelerationDistance2 = 128.0f;
        move.stepsRemaining = 48;
        move.lookahead.accelerationPrim = 40000;
        move.stepProfile.fAcceleration = 655;
        move.stepProfile.vMax = 4000;
        return move;
    };
    uint32_t trace = 0;
    const auto run = [&](uint8_t axes, bool advance, float junctionSpeed) {
        Printer::extrusionOutput.setAdvanceEnabled(advance);
        queue.clear();
        PrintLine& front = prepare(printX);
        MotionPlanner::updateLookahead(queue);
        queue.publish(true);
        PrintLine& reserved = prepare(printX);
        MotionPlanner::updateLookahead(queue);
        queue.publish(true);
        assert(queue.activateFront() == &front);
        const uint8_t frontFlags = front.joinFlags, reservedFlags = reserved.joinFlags;
        PrintLine& previous = prepare(printX);
        MotionPlanner::updateLookahead(queue);
        queue.publish(true);
        PrintLine& newest = prepare(axes);
        MotionPlanner::updateLookahead(queue);

        assert(interruptsEnabled && queue.count() == 3);
        assert(queue.active() == &front && front.joinFlags == frontFlags && reserved.joinFlags == reservedFlags);
        assert(previous.lookahead.endSpeed == junctionSpeed && newest.lookahead.startSpeed == junctionSpeed);
        for (auto* move : {&front, &reserved, &previous, &newest}) {
            assert(move->isStepProfileReady() && !move->isBlocked());
        }
        queue.publish(true);

        for (auto* profile : {&previous, &newest}) {
            PrintLine move = *profile;
            StepExecutor::StepClock clock;
            clock.start(move.stepProfile.vStart);
            uint32_t interval = 16000000UL / move.stepProfile.vStart;
            while (move.stepsRemaining > 0) {
                --move.stepsRemaining;
                const speed_t speed = StepExecutor::nextStepSpeed(move, clock, 1, interval);
                interval = 16000000UL / speed;
                trace = trace * 31 + interval;
            }
        }
    };
    run(printX, true, 12.0f);
    run(printY, true, 4.0f);
    run(travelX, false, 12.0f);
    run(travelX, true, 4.0f);
    run(retract, false, 4.0f);
    run(retract, true, 4.0f);
    assert(trace == 0x670f3ba6U); // Step intervals captured before extracting the planner.
}

void plannerAccelerationLimits() {
    MoveQueue queue;
    const auto plan = [&](int32_t x, int32_t y, int32_t e, float requestedTicks) {
        PrintLine move{};
        move.setAxisSteps(X_AXIS, x);
        move.setAxisSteps(Y_AXIS, y);
        move.setAxisSteps(E_AXIS, e);
        const float distances[] = {
            x * Printer::mmPerStep(X_AXIS), y * Printer::mmPerStep(Y_AXIS), 0, e * Printer::mmPerStep(E_AXIS)
        };
        move.setSegmentGeometry(distances);
        const float feedrate = F_CPU * move.lookahead.distance / requestedTicks;
        MotionPlanner::prepareMove(move, distances, feedrate, queue);
        assert(move.primaryAxis == X_AXIS);
        return move.lookahead.accelerationPrim;
    };

    Printer::maxTravelAccelerationStepsPerSquareSecond[X_AXIS] = 240000;
    Printer::maxTravelAccelerationStepsPerSquareSecond[Y_AXIS] = 40000;
    assert(plan(400, 200, 0, 410280) == 80000);

    // This axis allowance exceeds 32 bits before division.
    Printer::maxTravelAccelerationStepsPerSquareSecond[Y_AXIS] = 165000;
    assert(plan(38400, 32000, 0, 90000000) == 198000);

    // Printing uses a separate set of limits, including the active extruder.
    Printer::maxPrintAccelerationStepsPerSquareSecond[E_AXIS] = 5000;
    assert(plan(6400, 0, 3200, 3000000) == 10000);
}

void shortMovePacing() {
    constexpr unsigned prefill = 12, segmentCount = 300;
    constexpr uint32_t sendPeriod = F_CPU / 200;
    // Forty bytes per line with 8N1 framing at the configured UART rate.
    constexpr uint32_t wireTicks = (uint64_t(F_CPU) * 40 * 10 + firmwareSerialBaudrate - 1) / firmwareSerialBaudrate;
    constexpr float radius = 1.35f, feedrate = 100.0f;
    for (const float chord : {0.2f, 0.6f}) {
        const double angle = 2 * std::asin(chord / (2 * radius));
        double referenceDistance = 0;
        for (const unsigned groupSize : {1U, 32U, 96U}) {
            MoveQueue queue;
            unsigned admitted = 0, retired = 0, stretched = 0, limitedJoins = 0;
            unsigned emptyGaps = 0, deferred = 0, speedJumps = 0;
            uint32_t now = 0, endOfMove = 0, emptyTicks = 0, longestGap = 0;
            uint32_t interval = 10000, longestBatchInterval = 0, prefillTicks = 0;
            uint8_t batchSteps = 1;
            float lastFullSpeed = 0;
            double distance = 0;
            int32_t previous[4] = {static_cast<int32_t>(std::lround(radius * Printer::stepsPerMm(X_AXIS))), 0, 0, 0};

            const auto arrival = [&](unsigned segment) {
                const unsigned offset = segment - prefill;
                return offset / groupSize * groupSize * sendPeriod + (offset % groupSize + 1) * wireTicks;
            };
            const auto admitMove = [&] {
                const unsigned segment = admitted + 1;
                const int32_t target[4] = {
                    static_cast<int32_t>(std::lround(radius * std::cos(segment * angle) * Printer::stepsPerMm(X_AXIS))),
                    static_cast<int32_t>(std::lround(radius * std::sin(segment * angle) * Printer::stepsPerMm(Y_AXIS))),
                    0,
                    static_cast<int32_t>(std::lround(chord * segment * 0.05f * Printer::stepsPerMm(E_AXIS)))
                };

                PrintLine& move = queue.writeSlot();
                move = PrintLine{};
                float distances[4];
                for (uint8_t axis = 0; axis < 4; ++axis) {
                    const int32_t steps = target[axis] - previous[axis];
                    distances[axis] = std::abs(steps) * Printer::mmPerStep(axis);
                    move.setAxisSteps(axis, steps);
                    previous[axis] = target[axis];
                }
                move.setSegmentGeometry(distances);
                assert(move.isEPositiveMove() && move.isXOrYMove() && move.stepsRemaining);
                distance += move.lookahead.distance;
                MotionPlanner::prepareMove(move, distances, feedrate, queue);
                const float requestedTicks = float(F_CPU) * move.lookahead.distance / feedrate;
                stretched += move.timeInTicks > requestedTicks + 1;
                if (admitted < prefill)
                    prefillTicks += move.timeInTicks;
                speedJumps += admitted >= prefill && std::abs(move.fullSpeed - lastFullSpeed) > 20.0f;
                lastFullSpeed = move.fullSpeed;
                MotionPlanner::updateLookahead(queue);
                if (queue.count()) {
                    uint8_t tail = queue.writeIndex();
                    MoveQueue::previousIndex(tail);
                    const PrintLine& previousMove = queue.slot(tail);
                    const float join = previousMove.lookahead.maxJunctionSpeed;
                    limitedJoins += join > 0 && join + 0.001f < std::min(previousMove.fullSpeed, move.fullSpeed);
                }
                queue.publish(true);
                ++admitted;
            };
            const auto moveDuration = [&](PrintLine move) {
                StepExecutor::StepClock clock;
                clock.start(move.stepProfile.vStart);
                uint32_t duration = interval >> 1;
                while (move.stepsRemaining) {
                    const uint8_t steps = std::min<int32_t>(batchSteps, move.stepsRemaining);
                    move.stepsRemaining -= steps;
                    const speed_t speed = StepExecutor::nextStepSpeed(move, clock, steps, interval);
                    assert(speed);
                    interval = std::max<uint32_t>(512, F_CPU / speed);
                    batchSteps = (STEP_PACKING_MIN_INTERVAL + interval - 1) / interval;
                    const uint32_t packed = interval * batchSteps;
                    duration += move.stepsRemaining ? packed : packed >> 1;
                    if (move.stepsRemaining)
                        longestBatchInterval = std::max(longestBatchInterval, packed);
                }
                return duration;
            };

            // Start both schedules with the same already planned motion.
            while (admitted < prefill)
                admitMove();
            while (retired < segmentCount) {
                if (queue.active() && now == endOfMove) {
                    queue.retireFront();
                    HAL::allowInterrupts();
                    if (++retired == segmentCount)
                        break;
                }
                while (admitted < segmentCount && queue.count() < MOVE_CACHE_SIZE && arrival(admitted) <= now) {
                    deferred += arrival(admitted) < now;
                    admitMove();
                }
                if (!queue.active() && queue.count()) {
                    PrintLine& move = *queue.activateFront();
                    assert(move.isStepProfileReady() && !move.isBlocked());
                    move.fixStartAndEndSpeed();
                    endOfMove = now + moveDuration(move);
                }
                const uint32_t input = admitted < segmentCount && queue.count() < MOVE_CACHE_SIZE ? arrival(admitted) : UINT32_MAX;
                const uint32_t output = queue.active() ? endOfMove : UINT32_MAX;
                const uint32_t next = std::min(input, output);
                assert(next > now && next != UINT32_MAX);
                if (!queue.active()) {
                    const uint32_t gap = next - now;
                    ++emptyGaps;
                    emptyTicks += gap;
                    longestGap = std::max(longestGap, gap);
                }
                now = next;
            }
            assert(admitted == segmentCount && queue.count() == 0 && interruptsEnabled);
            if (groupSize == 1)
                referenceDistance = distance;
            else
                assert(distance == referenceDistance);
            std::printf("Short moves %.2f mm, group %u: %.2f mm/s, initial reserve %.2f ms, stretched %u/%u, limited joins %u, "
                        "speed jumps %u, empty gaps %u (%.2f ms total, %.2f ms max), "
                        "max batch interval %.2f ms, deferred %u\n",
                        chord, groupSize, distance * F_CPU / now, prefillTicks * 1000.0 / F_CPU,
                        stretched, segmentCount, limitedJoins, speedJumps,
                        emptyGaps, emptyTicks * 1000.0 / F_CPU, longestGap * 1000.0 / F_CPU,
                        longestBatchInterval * 1000.0 / F_CPU, deferred);
        }
    }
}

void directRamp() {
    PrintLine& move = PrintLine::direct;
    const int32_t steps[4] = {161, 0, 0, 0};
    const auto reserve = [&] {
        move = PrintLine{};
        InterruptProtectedBlock lock;
        PrintLine::reserveDirectMove(true);
    };

    reserve();
    PrintLine::prepareDirectMove(steps, true, false);
    assert(interruptsEnabled && move.task == DIRECT_PREPARED_STOPPABLE);
    assert(move.isStepProfileReady() && move.stepsRemaining == 161);
    assert(move.stepProfile.vStart == move.stepProfile.vEnd && move.stepProfile.accelSteps == move.stepProfile.decelSteps && move.stepProfile.accelSteps > 16);

    PrintLine legacyStop = move;
    legacyStop.stepsRemaining = 32;
    legacyStop.invalidateStepProfile();
    MotionPlanner::prepareStepProfile(legacyStop);
    {
        InterruptProtectedBlock lock;
        PrintLine::stopDirectMove();
    }
    assert(move.stepsRemaining == 32);
    assert(move.stepProfile.accelSteps == legacyStop.stepProfile.accelSteps && move.stepProfile.decelSteps == legacyStop.stepProfile.decelSteps);

    reserve();
    interruptBeforeLock = 1;
    onInterrupt = [] {
        assert(interruptsEnabled && PrintLine::hasStoppableDirectMove());
        InterruptProtectedBlock lock;
        PrintLine::stopDirectMove();
        assert(PrintLine::direct.task == DIRECT_PREPARING_STOPPED);
    };
    PrintLine::prepareDirectMove(steps, true, false);
    assert(interruptBeforeLock == 0 && interruptsEnabled);
    assert(move.task == DIRECT_PREPARED_STOPPABLE && move.stepsRemaining == 32);
    assert(move.stepProfile.accelSteps == legacyStop.stepProfile.accelSteps && move.stepProfile.decelSteps == legacyStop.stepProfile.decelSteps);

    reserve();
    PrintLine::prepareDirectMove(steps, true, false);
    const uint32_t initialRampSteps = move.stepProfile.accelSteps;
    {
        InterruptProtectedBlock lock;
        move.task = DIRECT_RUNNING_STOPPABLE;
        PrintLine::stopDirectMove();
    }
    assert(move.stepsRemaining == 32 && move.stepProfile.accelSteps == initialRampSteps);

    reserve();
    const int32_t noSteps[4] = {};
    PrintLine::prepareDirectMove(noSteps, true, false);
    assert(!PrintLine::directMoveActive() && move.stepsRemaining == 0);
}

// Dispatch the replay's runtime fixture to the same four specializations used
// by the firmware. Do not duplicate the limit decision or its state changes.
static bool replayCanStepXY(uint8_t axis, PrintLine* move, bool queued) {
    assert(axis == X_AXIS || axis == Y_AXIS);
    if (axis == X_AXIS)
        return queued ? canStepXY<X_AXIS, true>(move) : canStepXY<X_AXIS, false>(move);
    return queued ? canStepXY<Y_AXIS, true>(move) : canStepXY<Y_AXIS, false>(move);
}

void xyLimits() {
    for (uint8_t axis = X_AXIS; axis <= Y_AXIS; ++axis) {
        const uint8_t positive = 17 << axis, negative = 16 << axis;
        PrintLine move;
        move.dir = positive;
        Printer::xyMaxHit[axis] = true;
        assert(!replayCanStepXY(axis, &move, true));
        assert(!replayCanStepXY(axis, &move, true));
        assert(Printer::xyMotion.queuedOvertravel(axis) == 2);

        move.dir = negative;
        assert(!replayCanStepXY(axis, &move, true));
        Printer::xyMaxHit[axis] = false;
        assert(!replayCanStepXY(axis, &move, true));
        assert(replayCanStepXY(axis, &move, true) && !Printer::xyMotion.queuedOvertravel(axis));

        Printer::xyMinHit[axis] = true;
        assert(!replayCanStepXY(axis, &move, true));
        assert(Printer::xyMotion.queuedOvertravel(axis) == -1);
        move.dir = positive;
        move.abortAtEndstops = true;
        assert(replayCanStepXY(axis, &move, false)); // A direct retreat leaves the queue's debt alone.
        assert(Printer::xyMotion.queuedOvertravel(axis) == -1);
        assert(!replayCanStepXY(axis, &move, true));
        assert(replayCanStepXY(axis, &move, true) && !Printer::xyMotion.queuedOvertravel(axis));

        move.dir = negative;
        move.abortAtEndstops = false;
        assert(!replayCanStepXY(axis, &move, true));
        move.abortAtEndstops = true;
        assert(!replayCanStepXY(axis, &move, true));
        assert(!move.isNegativeMoveOfAxis(axis) && !Printer::xyMotion.queuedOvertravel(axis));
        assert(Printer::stoppedDirectAxis != axis);

        move.dir = negative;
        assert(!replayCanStepXY(axis, &move, false));
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
    for (uint8_t axis = X_AXIS; axis <= Z_AXIS; ++axis)
        Printer::manualMotion.setMode(axis, MOVE_MODE_SINGLE_STEPS);
    assert(argc == 2);
    Printer::xyMotion.setNominalSteps(X_AXIS, 6400);
    Printer::xyMotion.setNominalSteps(Y_AXIS, 12800);
    const bool digitFlowReplay = !std::strcmp(argv[1], "digit-flow");
    if (!digitFlowReplay && std::strcmp(argv[1], "digit-zero") && std::strncmp(argv[1], "pressure-", 9))
        initializeOffsets();
    Printer::pause.restoreDefaultParking();
    Printer::pause.setParkSteps(E_AXIS, parkE);
    if (std::strcmp(argv[1], "print-job"))
        Printer::printJob.admitPrintMotion();

    if (!std::strcmp(argv[1], "park"))
        parkAndResume();
    else if (!std::strcmp(argv[1], "pause-manual-e"))
        manualExtrusionDuringPause(false);
    else if (!std::strcmp(argv[1], "pause-manual-e-advance"))
        manualExtrusionDuringPause(true);
    else if (!std::strcmp(argv[1], "pause-filament-program"))
        filamentProgramDuringPause();
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
    else if (!std::strcmp(argv[1], "pressure-frozen-z"))
        pressureWithFrozenZ();
    else if (!std::strcmp(argv[1], "digit-zero"))
        digitZero();
    else if (!std::strcmp(argv[1], "pressure-measurement"))
        pressureMeasurement();
    else if (!std::strcmp(argv[1], "extrusion-planning"))
        extrusionPlanning();
    else if (!std::strcmp(argv[1], "output-safety-lock"))
        outputSafetyLock();
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
    else if (!std::strcmp(argv[1], "move-reserve"))
        moveReserve();
    else if (!std::strcmp(argv[1], "idle-status"))
        idleStatusTransitions();
    else if (!std::strcmp(argv[1], "print-job"))
        printJob();
    else if (!std::strcmp(argv[1], "advance-profile"))
        advanceProfileReport();
    else if (!std::strcmp(argv[1], "ecmp-stepping"))
        ecmpStepping();
    else if (!std::strcmp(argv[1], "advance-timer"))
        advanceTimer();
    else if (!std::strcmp(argv[1], "advance-mode"))
        advanceModeChange();
    else if (!std::strcmp(argv[1], "motion-wait"))
        motionWait();
    else if (!std::strcmp(argv[1], "advance-cancel"))
        advanceCancel();
    else if (!std::strcmp(argv[1], "pressure-advance-pause"))
        advancePauseWithPressureLimit();
    else if (!std::strcmp(argv[1], "advance-output"))
        finishAdvanceBeforeAndAfterOutput();
#if NUM_EXTRUDER > 1
    else if (!std::strcmp(argv[1], "advance-tool"))
        advanceToolChange();
#endif
    else if (!std::strcmp(argv[1], "step-clock"))
        stepClock();
    else if (!std::strcmp(argv[1], "lookahead"))
        lookaheadMoves();
    else if (!std::strcmp(argv[1], "acceleration-limits"))
        plannerAccelerationLimits();
    else if (!std::strcmp(argv[1], "short-move-pacing"))
        shortMovePacing();
    else if (!std::strcmp(argv[1], "direct-ramp"))
        directRamp();
    else if (!std::strcmp(argv[1], "xy-limits"))
        xyLimits();
    else
        return 1;
    std::printf("Print lifecycle replay passed (%s)\n", argv[1]);
}

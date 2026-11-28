#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <initializer_list>
#include "Constants.h"

#define INLINE inline
using flag8_t = uint8_t;
using fast8_t = uint8_t;
using speed_t = uint16_t;
enum { Z_MIN_PIN, Z_MAX_PIN, Z_DIR_PIN, Z_STEP_PIN };
constexpr bool INVERT_Z_DIR = false, LOW = false, HIGH = true;
constexpr float Z_ENDSTOP_DRIVE_OVER = 0.25f;
constexpr uint16_t XYZ_STEPPER_HIGH_DELAY = 100, XYZ_STEPPER_LOW_DELAY = 100;
#ifndef MAX_STEP_PACKING_MIN_INTERVAL
#define MAX_STEP_PACKING_MIN_INTERVAL 5000
#endif

bool interruptsEnabled = true, high[3] = {}, positive[3] = {};
bool closed[3] = {};
int32_t physical[3] = {};
unsigned pulses = 0;
constexpr int32_t upper = 1000;
class InterruptProtectedBlock {
    const bool previous_ = interruptsEnabled;
public:
    InterruptProtectedBlock() { interruptsEnabled = false; }
    ~InterruptProtectedBlock() { unprotect(); }
    void unprotect() { interruptsEnabled = previous_; }
};

bool minContact(uint8_t axis) {
    if (physical[axis] <= 0) closed[axis] = true;
    if (physical[axis] >= 24) closed[axis] = false;
    return closed[axis];
}
void pulse(uint8_t axis) {
    assert(!high[axis]);
    high[axis] = true;
    physical[axis] += positive[axis] ? 1 : -1;
    ++pulses;
}
bool READ(int pin) {
    if (pin == Z_DIR_PIN) return positive[Z_AXIS];
    if (pin == Z_MIN_PIN) return !minContact(Z_AXIS);
    assert(pin == Z_MAX_PIN);
    return physical[Z_AXIS] < upper;
}
void WRITE(int pin, bool value) {
    if (pin == Z_DIR_PIN) {
        assert(!high[Z_AXIS]);
        positive[Z_AXIS] = value;
    } else {
        assert(pin == Z_STEP_PIN);
        if (value) pulse(Z_AXIS);
        else high[Z_AXIS] = false;
    }
}
namespace HAL {
void allowInterrupts() { interruptsEnabled = true; }
void forbidInterrupts() { interruptsEnabled = false; }
void delayMicroseconds(int) { assert(false); }
uint32_t timeInMilliseconds() { return 0; }
uint32_t CPUDivU2(uint16_t v) { assert(v); return 16000000UL / v; }
uint16_t ComputeV(uint32_t, uint32_t) { return 0; }
}

#include "XYMotion.h"
#include "ZMotion.h"
#include "CalibrationScan.h"
namespace Calibration { ScanSession scanSession; }
#define private public
#include "motion.h"
#include "StepExecutor.h"
#undef private
class MoveQueue {
public:
    unsigned count() const { return 0; }
    void retireFront() { assert(false); }
};
MoveQueue PrintLine::queue;
PrintLine PrintLine::direct;
StepExecutor::StepClock StepExecutor::stepClocks[2];
StepExecutor::StepErrors StepExecutor::stepErrors[2];
StepExecutor::DirectionOwner StepExecutor::directionOwner = DirectionOwner::None;
volatile uint32_t StepExecutor::stepIntervalTicks = 10000;
volatile uint16_t StepExecutor::packingInterval_ = 5000;
uint8_t StepExecutor::stepsPerTimerCall = 1;
volatile int32_t StepExecutor::queuedPrimarySteps;
int32_t StepExecutor::directPrimarySteps;
volatile float StepExecutor::nominalFeedrate_ = 0;

namespace Printer {
XYMotion xyMotion;
ZMotion zMotion(100);
bool blockAll = false, homed = true;
struct { bool blocked = false; bool commandsBlocked() const { return blocked; } } printJob;
struct { void schedule(uint32_t) {} } idleStatus;
struct {
    bool blocked() const { return false; }
    bool advanceEnabled() const { return false; }
    void addSteps(int) { assert(false); }
} extrusionOutput;
struct {
    bool addBaseStep(bool) { assert(false); return false; }
    void consumeExtraStep(bool) { assert(false); }
} extrusionCompensation;
struct { void recordDirectExtrusion(int8_t) { assert(false); } } pause;
struct { uint16_t intervalScale() { return 0; } } digitCompensation;
bool isAxisHomed(uint8_t) { return homed; }
bool zStepperEnabled() { return true; }
bool isMinEndstopHit(uint8_t axis) { return minContact(axis); }
bool isMaxEndstopHit(uint8_t axis) { return physical[axis] >= upper; }
bool getXDirectionIsPos() { return positive[X_AXIS]; }
bool getYDirectionIsPos() { return positive[Y_AXIS]; }
void startXStep(int8_t d) { pulse(X_AXIS); xyMotion.recordMotorStep(X_AXIS,d); }
void startYStep(int8_t d) { pulse(Y_AXIS); xyMotion.recordMotorStep(Y_AXIS,d); }
void endXYZSteps() { for (auto& h : high) h = false; }
void stopDirectAxis(uint8_t axis) {
    if (axis < Z_AXIS) xyMotion.stopDirect(axis);
    else if (axis == Z_AXIS) zMotion.stopDirectMove();
}
}
struct Extruder {
    int stepperDirection = 0;
    static Extruder* current;
    static void step() { assert(false); }
    static void unstep() {}
};
Extruder extruder;
Extruder* Extruder::current = &extruder;
bool isAnyScanRunning() { return false; }
bool hasQueuedOvertravel() { return Printer::xyMotion.hasQueuedOvertravel() || Printer::zMotion.hasQueuedOvertravel(); }
void StepExecutor::enableSteppers(PrintLine&) {}
void StepExecutor::adjustDirections(PrintLine& m) {
    for (uint8_t axis = X_AXIS; axis <= Z_AXIS; ++axis)
        if (m.isMoveOfAxis(axis)) positive[axis] = m.isPositiveMoveOfAxis(axis);
}
void StepExecutor::updateAdvanceSteps(const PrintLine&, uint16_t) {}
void StepExecutor::stepSlowedZCompensation() { assert(!Printer::zMotion.correctionCanMove()); }

#include "guarded-firmware.inc"

int32_t motor(uint8_t a) {
    return a == Z_AXIS ? Printer::zMotion.motorSteps() : Printer::xyMotion.motorSteps(a);
}
int32_t nominal(uint8_t a) {
    return a == Z_AXIS ? Printer::zMotion.nominalSteps() : Printer::xyMotion.nominalSteps(a);
}
const DirectPosition& ledger(uint8_t a) {
    return a == Z_AXIS ? Printer::zMotion.directPosition() : Printer::xyMotion.directPosition(a);
}
void reset(uint8_t a, int32_t origin) {
    for (uint8_t i=0; i<3; ++i) {
        physical[i] = i == a ? origin : 100;
        closed[i] = physical[i] <= 0;
        high[i] = false;
    }
    for (uint8_t i=0; i<2; ++i) {
        Printer::xyMotion.setMotorSteps(i, physical[i]);
        Printer::xyMotion.setNominalSteps(i, physical[i]);
        Printer::xyMotion.resetDirect(i);
        Printer::xyMotion.clearQueuedOvertravel(i);
    }
    auto& z = Printer::zMotion;
    z.configureTravel(100, 10);
    z.setHomed(false);
    z.setHomed(true);
    z.setMotorReference(physical[Z_AXIS]);
    z.setNominalSteps(physical[Z_AXIS]);
    z.resetDirectPosition();
    z.clearScanAndCorrection();
    Printer::blockAll = Printer::printJob.blocked = false;
    Printer::homed = true;
    pulses = 0;
}
unsigned run(uint8_t axis, int32_t distance, GuardedStop guard, int cancelAt=-1, int reason=0) {
    auto& m = PrintLine::direct;
    m.flags = FLAG_ABORT_AT_ENDSTOPS | static_cast<uint8_t>(guard);
    m.dir = 0;
    m.setMoveOfAxis(axis);
    if (distance > 0) m.setPositiveDirectionForAxis(axis);
    for (auto& d : m.delta) d = 0;
    m.delta[axis] = m.stepsRemaining = std::abs(distance);
    m.primaryAxis = axis;
    m.stepProfile = {};
    m.stepProfile.vStart = m.stepProfile.vEnd = m.stepProfile.vMax = 5000;
    m.task = DIRECT_PREPARED;
    if (axis == Z_AXIS) Printer::zMotion.requestDirectMove(distance);
    else Printer::xyMotion.requestDirect(axis,distance);

    const unsigned before = pulses;
    const int32_t origin = motor(axis), oldLedger = ledger(axis).steps(), oldNominal = nominal(axis);
    StepExecutor::stepsPerTimerCall = 4;
    for (unsigned tick=0; PrintLine::directMoveActive(); ++tick) {
        assert(tick < 2000);
        if (cancelAt >= 0 && pulses-before == static_cast<unsigned>(cancelAt)) {
            if (reason == 0) Printer::blockAll = true;
            if (reason == 1) Printer::printJob.blocked = true;
            if (reason == 2) Printer::homed = false;
        }
        const unsigned previous = pulses;
        { InterruptProtectedBlock lock; StepExecutor::performDirectMove(); }
        assert(pulses-previous <= 1);
        assert(!high[0] && !high[1] && !high[2]);
        assert(motor(axis) == physical[axis]);
        assert(nominal(axis) == oldNominal);
        assert(ledger(axis).steps() - oldLedger == motor(axis) - origin);
        assert(StepExecutor::stepsPerInterrupt() == 1);
    }
    assert(ledger(axis).steps() == ledger(axis).target());
    return pulses-before;
}
bool matched() {
    return (PrintLine::direct.flags & FLAG_GUARDED_STOP_MASK) == static_cast<uint8_t>(GuardedStop::Distance);
}
int main() {
    for (uint8_t axis=0; axis<=Z_AXIS; ++axis) {
        reset(axis,80);
        assert(run(axis,-80,GuardedStop::Closed)==80 && matched() && motor(axis)==0);
        assert(run(axis,24,GuardedStop::Open)==24 && matched() && motor(axis)==24);
        assert(run(axis,-80,GuardedStop::Closed)==24 && matched() && motor(axis)==0);
        assert(run(axis,80,GuardedStop::Distance)==80 && motor(axis)==80 && ledger(axis).steps()==0);
        reset(axis,80);
        assert(run(axis,-79,GuardedStop::Closed)==79 && !matched());
        reset(axis,0);
        assert(run(axis,23,GuardedStop::Open)==23 && !matched());
        reset(axis,0);
        assert(run(axis,-10,GuardedStop::Closed)==0 && matched());
        reset(axis,80);
        assert(run(axis,10,GuardedStop::Open)==0 && matched());
        reset(axis,999);
        assert(run(axis,10,GuardedStop::Distance)==1 && motor(axis)==1000);
        for (int reason=0; reason<3; ++reason)
            for (int cut=0; cut<=80; ++cut) {
                reset(axis,80);
                assert(run(axis,-80,GuardedStop::Closed,cut,reason)==static_cast<unsigned>(cut));
                assert(!matched());
            }
    }
}

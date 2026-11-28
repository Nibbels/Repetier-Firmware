#include <cassert>
#include <cstdint>
#include <cstdlib>

#include "Constants.h"

enum { Z_MIN_PIN, Z_MAX_PIN, Z_DIR_PIN, Z_STEP_PIN };
constexpr bool INVERT_Z_DIR = false, LOW = false, HIGH = true;
constexpr float Z_ENDSTOP_DRIVE_OVER = 0.25f;
constexpr uint16_t XYZ_STEPPER_HIGH_DELAY = 100, XYZ_STEPPER_LOW_DELAY = 100;
#define INLINE inline
#define PSTR(text) (text)
#define UI_STATUS_UPD(text) ((void)(text))

namespace {
bool interruptsEnabled = true;
bool stepHigh = false, positive = true;
int32_t motorPosition = 0, minContact = 0, maxContact = 1000;
int pulseCount = 0, waitedMs = 0;
bool tickDuringScan = false;
int cancelAtPulses = 0;
int lastServicePulse = 0;

class InterruptProtectedBlock {
    const bool previous_ = interruptsEnabled;

public:
    InterruptProtectedBlock() { interruptsEnabled = false; }
    ~InterruptProtectedBlock() { unprotect(); }
    void unprotect() { interruptsEnabled = previous_; }
};

bool READ(int pin) {
    if (pin == Z_DIR_PIN)
        return positive;
    if (pin == Z_MIN_PIN)
        return motorPosition > minContact;
    assert(pin == Z_MAX_PIN);
    return motorPosition < maxContact;
}

void WRITE(int pin, bool high) {
    if (pin == Z_DIR_PIN) {
        assert(!stepHigh);
        positive = high;
        return;
    }
    assert(pin == Z_STEP_PIN);
    if (high) {
        assert(!stepHigh);
        motorPosition += positive ? 1 : -1;
        pulseCount++;
    }
    stepHigh = high;
}
}

namespace HAL {
void allowInterrupts() { interruptsEnabled = true; }
void forbidInterrupts() { interruptsEnabled = false; }
void delayMilliseconds(int milliseconds);
void delayMicroseconds(int microseconds);
}

#include "ZMotion.h"
#include "CalibrationScan.h"

namespace Printer {
ZMotion zMotion(100.0f);
bool blockAll = false, doHeatBedZCompensation = true;
struct { bool commandsBlocked() const { return false; } } printJob;
bool needsCMPwait();
void disableCMPnow(bool wait);
}

struct StepExecutor {
    static constexpr uint8_t stepsPerTimerCall = 1;
    static void stepSlowedZCompensation();
    static bool needCmpWait();
};

constexpr int Processing = 0, Calibrating = 1;
ScanSession scanSession;
struct Commands {
    static void checkForPeriodicalActions(int state) {
        if (state == Calibrating) {
            assert(!stepHigh && pulseCount - lastServicePulse <= 16);
            lastServicePulse = pulseCount;
            if (cancelAtPulses && pulseCount >= cancelAtPulses)
                scanSession.requestAbort(ScanAbort::Canceled);
        }
        HAL::delayMilliseconds(1);
    }
};
struct Com { static void printFLN(const char*) {} };

enum class GuardedMoveStatus : uint8_t { Denied, Cancelled, Limit, SwitchReached, PositionReached };
struct GuardedMoveResult { GuardedMoveStatus status; int32_t positionSteps; };
struct PrintLine {
    static GuardedMoveResult runScanMove(int32_t nSteps, bool emergencyRetreat);
};
static bool moveZ(int32_t nSteps, bool emergencyRetreat = false);

#include "z-motion-firmware.inc"

namespace {
using Printer::zMotion;
using Path = ZMotion::Path;

void tick() {
    InterruptProtectedBlock lock;
    StepExecutor::stepSlowedZCompensation();
    assert(!stepHigh && zMotion.motorSteps() == motorPosition);
}

void settleCorrection() {
    for (int i = 0; i < 600; ++i)
        tick();
}

void reset(int32_t position, bool homed = true) {
    zMotion.configureTravel(100.0f, 10.0f);
    zMotion.setMotorReference(position);
    zMotion.setNominalSteps(position);
    zMotion.resetDirectPosition();
    zMotion.clearScanAndCorrection();
    zMotion.setHomed(false);
    zMotion.setHomed(homed);
    motorPosition = position;
    minContact = 0;
    maxContact = 1000;
    pulseCount = waitedMs = lastServicePulse = cancelAtPulses = 0;
    Printer::blockAll = false;
    Printer::doHeatBedZCompensation = true;
    scanSession.reset();
    scanSession.armRetries(2);
    ZMotion::setDirection(true);
}

bool step(Path path, int8_t direction) {
    InterruptProtectedBlock lock;
    ZMotion::setDirection(direction > 0);
    const bool continuing = zMotion.stepPath(path);
    assert(!interruptsEnabled);
    ZMotion::finishPulse();
    assert(zMotion.motorSteps() == motorPosition);
    return continuing;
}

}
// Model scheduling here; actual Timer1 executor is replayed separately.
GuardedMoveResult PrintLine::runScanMove(int32_t nSteps, bool emergencyRetreat) {
    using namespace Printer;
    const auto interrupted = [emergencyRetreat] {
        return !emergencyRetreat && (scanSession.aborting() || blockAll || printJob.commandsBlocked());
    };
    if (interrupted())
        return {GuardedMoveStatus::Cancelled,zMotion.scanSteps()};
    while (needsCMPwait()) {
        Commands::checkForPeriodicalActions(Processing);
        if (interrupted()) return {GuardedMoveStatus::Cancelled,zMotion.scanSteps()};
    }
    zMotion.setScanInProgress(true);
    GuardedMoveStatus status=GuardedMoveStatus::PositionReached;
    for (int32_t i=0;i<abs(nSteps);++i) {
        if (interrupted()) {status=GuardedMoveStatus::Cancelled;break;}
        if (!step(Path::Scan,nSteps<0?-1:1)) {status=GuardedMoveStatus::Limit;break;}
        if (tickDuringScan) {
            InterruptProtectedBlock lock;
            const int before=pulseCount;
            StepExecutor::stepSlowedZCompensation();
            assert(!StepExecutor::needCmpWait() && pulseCount==before);
        }
        if ((i&15)==15) Commands::checkForPeriodicalActions(Calibrating);
    }
    zMotion.setScanInProgress(false);
    if(interrupted()) status=GuardedMoveStatus::Cancelled;
    return {status,zMotion.scanSteps()};
}
namespace {
void driveDirect(int32_t distance, Path path = Path::Direct) {
    zMotion.requestDirectMove(distance);
    const int8_t direction = distance > 0 ? 1 : -1;
    for (int32_t i = 0; i < abs(distance); ++i) {
        if (!step(path, direction))
            break;
    }
    zMotion.stopDirectMove();
}

void queuedAndDirectTravel() {
    reset(999);
    for (int i = 0; i < 4; ++i)
        assert(step(Path::Queued, 1));
    assert(motorPosition == 1000 && pulseCount == 1);
    assert(zMotion.nominalSteps() == 1003 && zMotion.hasQueuedOvertravel());

    driveDirect(-2);
    assert(motorPosition == 998 && zMotion.directPosition().steps() == -2);
    driveDirect(7);
    assert(motorPosition == 1000 && zMotion.directPosition().steps() == 0);
    assert(zMotion.directPosition().target() == 0 && zMotion.hasQueuedOvertravel());

    for (int i = 0; i < 3; ++i)
        assert(step(Path::Queued, -1));
    assert(motorPosition == 1000 && !zMotion.hasQueuedOvertravel());
    assert(zMotion.nominalSteps() == 1000 && pulseCount == 5);
    assert(step(Path::Queued, -1) && motorPosition == 999);

    reset(1000);
    assert(step(Path::Queued, 1) && zMotion.hasQueuedOvertravel());
    driveDirect(-1);
    assert(step(Path::Queued, 1) && motorPosition == 999);
    assert(!zMotion.hasQueuedOvertravel());
    assert(step(Path::Queued, 1) && motorPosition == 1000);
    assert(step(Path::Queued, 1) && zMotion.hasQueuedOvertravel());
    assert(!step(Path::QueuedUntilEndstop, 1));
    assert(!zMotion.hasQueuedOvertravel());
}

void minimumTravel() {
    reset(1, false);
    assert(step(Path::QueuedUntilEndstop, -1) && motorPosition == 0);
    assert(!step(Path::QueuedUntilEndstop, -1));
    assert(pulseCount == 1 && zMotion.nominalSteps() == -1);
    driveDirect(-1);
    assert(pulseCount == 1 && zMotion.directPosition().steps() == 0);

    reset(1, false);
    zMotion.requestCorrection(-4);
    settleCorrection();
    assert(motorPosition == 0 && pulseCount == 1);
    zMotion.setHomed(true);
    settleCorrection();
    assert(motorPosition == -3 && pulseCount == 4);

    reset(0);
    driveDirect(-4, Path::StoppableDirect);
    assert(pulseCount == 0);
    driveDirect(-24);
    assert(motorPosition == -24 && zMotion.directPosition().steps() == -24);
    assert(!zMotion.manualStepExceedsMinimum(-1) && zMotion.manualStepExceedsMinimum(-2));
    driveDirect(-8);
    assert(motorPosition == -25 && zMotion.directPosition().target() == -25);
    driveDirect(25);
    assert(motorPosition == 0 && zMotion.directPosition().steps() == 0);

    // Changing the microstep scale preserves the physical overtravel distance.
    zMotion.configureTravel(200.0f, 10.0f);
    assert(zMotion.minOvertravel() == 50 && zMotion.maxSteps() == 2000);
    driveDirect(-52);
    assert(motorPosition == -50);
}

void upperLimitsByModel() {
    reset(999);
    maxContact = 1003;
    driveDirect(6);
#if MOTHERBOARD == DEVICE_TYPE_RF1000
    assert(motorPosition == 1000);
#else
    assert(motorPosition == 1003);
#endif
    reset(999);
    maxContact = 1003;
    zMotion.requestCorrection(12);
    settleCorrection();
    assert(motorPosition == 1000 && pulseCount == 1);

    reset(995);
    maxContact = 997;
    zMotion.requestCorrection(12);
    settleCorrection();
#if MOTHERBOARD == DEVICE_TYPE_RF1000
    assert(motorPosition == 1000);
#else
    assert(motorPosition == 997);
#endif
    assert(!StepExecutor::needCmpWait());
}

void opposingCorrection() {
    reset(500);
    zMotion.requestCorrection(2);
    assert(step(Path::Queued, -1));
    driveDirect(-1);
    assert(zMotion.nominalSteps() == 499 && zMotion.directPosition().steps() == -1);
    assert(pulseCount == 0 && !zMotion.correctionCanMove());
    assert(step(Path::Queued, -1) && pulseCount == 1);

    reset(1000);
    zMotion.requestCorrection(4);
    assert(step(Path::Queued, -1) && motorPosition == 999);
    assert(step(Path::Queued, -1) && motorPosition == 999);
    assert(zMotion.nominalSteps() == 998 && pulseCount == 1);

    reset(-25);
    zMotion.requestCorrection(-5);
    driveDirect(1);
    assert(motorPosition == -24 && pulseCount == 1);
    driveDirect(1);
    assert(motorPosition == -24 && zMotion.directPosition().steps() == 2);
}

void correctionWaits() {
    // Timer1 queue task: cancellation must never enter a delay with I=0.
    reset(500);
    zMotion.requestCorrection(26);
    settleCorrection();
    assert(motorPosition == 526);
    {
        InterruptProtectedBlock lock;
        Printer::disableCMPnow();
        assert(!interruptsEnabled && waitedMs == 0);
        assert(!Printer::doHeatBedZCompensation && motorPosition == 526);
    }
    assert(interruptsEnabled);
    settleCorrection();
    assert(motorPosition == 500 && waitedMs == 0);

    reset(500);
    zMotion.requestCorrection(25);
    assert(!StepExecutor::needCmpWait() && Printer::needsCMPwait());
    zMotion.requestCorrection(26);
    assert(StepExecutor::needCmpWait());
    settleCorrection();
    assert(motorPosition == 526 && !StepExecutor::needCmpWait());
    Printer::disableCMPnow(true);
    assert(waitedMs > 0 && !Printer::doHeatBedZCompensation);
    assert(motorPosition == 500);

    reset(999);
    zMotion.requestCorrection(-5);
    settleCorrection();
    driveDirect(6);
    assert(motorPosition == 1000);
    Printer::disableCMPnow(true);
    assert(waitedMs == 0 && !StepExecutor::needCmpWait());
    driveDirect(-7);
    settleCorrection();
    assert(motorPosition == 998);
}

void scanTravel() {
    reset(-25);
    zMotion.requestCorrection(-4);
    ZMotion::setDirection(false);
    tickDuringScan = true;
    moveZ(4);
    assert(!scanSession.aborting() && !zMotion.scanInProgress());
    assert(motorPosition == -21 && zMotion.scanSteps() == 4 && pulseCount == 4);
    tickDuringScan = false;
    settleCorrection();
    assert(motorPosition == -25 && pulseCount == 8 && zMotion.scanSteps() == 4);

    reset(-24);
    moveZ(-2);
    assert(scanSession.aborting() && scanSession.retriesRemaining() == 0);
    assert(motorPosition == -25 && zMotion.scanSteps() == -1 && !zMotion.scanInProgress());
    moveZ(1);
    assert(pulseCount == 1 && !zMotion.scanInProgress());
    assert(scanSession.takeAbort() == ScanAbort::MaxCompensation);

    reset(100);
    cancelAtPulses = 32;
    assert(!moveZ(80));
    assert(scanSession.aborting() && pulseCount == 32 && waitedMs == 2 && zMotion.scanSteps() == 32);
    assert(!zMotion.scanInProgress());

    reset(100);
    Printer::blockAll = true;
    scanSession.requestAbort(ScanAbort::DmsOverload);
    moveZ(4);
    assert(!pulseCount);
    assert(moveZ(4, true));
    assert(pulseCount == 4 && motorPosition == 104);

    reset(999);
    assert(!moveZ(4));
    assert(motorPosition == 1000 && pulseCount == 1 && scanSession.aborting());

    reset(100, false);
    assert(!moveZ(-4) && pulseCount == 0 && scanSession.aborting());

    // The central Timer1 scan path preserves nominal and jog positions,
    // honors the same limits and does not balance a pending correction.
    reset(-24);
    zMotion.requestCorrection(4);
    assert(step(Path::Scan, -1) && motorPosition == -25);
    assert(!step(Path::Scan, -1));
    assert(zMotion.scanSteps() == -1 && zMotion.nominalSteps() == -24);
    assert(zMotion.directPosition().steps() == 0 && zMotion.correctionTarget() == 4);
    assert(step(Path::Scan, 1) && motorPosition == -24 && zMotion.scanSteps() == 0);
    reset(999);
    assert(step(Path::Scan, 1) && !step(Path::Scan, 1));
    assert(zMotion.scanSteps() == 1 && motorPosition == 1000);
    reset(100, false);
    assert(!step(Path::Scan, -1) && step(Path::Scan, 1));
    assert(zMotion.scanSteps() == 1 && motorPosition == 101);

    // M3028 no longer emits foreground pulses; its replacement is
    // covered by thermal-emergency-home.py and guarded-drive replay.
}
}

void HAL::delayMilliseconds(int milliseconds) {
    waitedMs += milliseconds;
    assert(waitedMs < 10000);
    tick();
}

void HAL::delayMicroseconds(int microseconds) {
    assert(microseconds > 0);
    if (tickDuringScan) {
        assert(zMotion.scanInProgress());
        const int previousPulses = pulseCount;
        InterruptProtectedBlock lock;
        StepExecutor::stepSlowedZCompensation();
        assert(!StepExecutor::needCmpWait() && pulseCount == previousPulses);
    }
}

int main() {
    queuedAndDirectTravel();
    minimumTravel();
    upperLimitsByModel();
    opposingCorrection();
    correctionWaits();
    scanTravel();
}

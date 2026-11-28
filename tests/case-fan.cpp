#include <cassert>
#include <cstdint>

using millis_t = uint32_t;
enum { CASE_FAN_PIN = 58, CASE_FAN_OFF_DELAY = 60000, CASE_FAN_ON_TEMPERATURE = 50, NUM_EXTRUDER = 2 };
bool pinConfigured = false, fanOn = false;
void SET_OUTPUT(int pin) { assert(pin == CASE_FAN_PIN); pinConfigured = true; }
void WRITE(int pin, bool on) { assert(pin == CASE_FAN_PIN && pinConfigured); fanOn = on; }
enum { HIGH = true, LOW = false };
#define PSTR(text) text

namespace HAL {
millis_t now = 0;
millis_t timeInMilliseconds() { return now; }
}
namespace Printer {
bool debugInfo() { return true; }
}
namespace Com {
uint32_t reportedSeconds = 0;
void printF(const char*, uint32_t seconds) { reportedSeconds = seconds; }
void printFLN(const char*) {}
}

struct TemperatureController { float targetTemperatureC; } tools[NUM_EXTRUDER]{};
TemperatureController* tempController[NUM_EXTRUDER] = {&tools[0], &tools[1]};
struct GCode {
    int32_t S;
    bool hasSeconds;
    bool inhibit;
    int M = 3121;
    bool hasS() const { return hasSeconds; }
    bool hasP() const { return inhibit; }
    GCode(int32_t seconds, bool hasS, bool hasP = false) : S(seconds), hasSeconds(hasS), inhibit(hasP) {}
};
unsigned invalid = 0;
void showInvalidSyntax(int m) { assert(m == 3121); ++invalid; }

namespace ThermalEmergency {
static struct { bool fault=false; bool active()const{return fault;} } ledger;
const decltype(ledger)& faultLedger(){return ledger;}
}
#include "case-fan-firmware.inc"

int main() {
    CaseFan::initialize();
    assert(!fanOn && CaseFan::offDelay() == 60000);

    turnOnCaseFan();
    GCode timed(1, true);
    scheduleCaseFanOff(&timed);
    assert(fanOn && CaseFan::offDelay() == 1000);
    HAL::now = 1000;
    CaseFan::service();
    assert(fanOn);
    HAL::now = 1001;
    CaseFan::service();
    assert(!fanOn);

    tools[1].targetTemperatureC = 70;
    CaseFan::updateFromHotendTargets();
    assert(fanOn);
    tools[1].targetTemperatureC = 0;
    HAL::now = UINT32_MAX - 99;
    CaseFan::updateFromHotendTargets();
    HAL::now = 50;
    CaseFan::service();
    assert(fanOn);
    HAL::now = 901;
    CaseFan::service();
    assert(!fanOn);

    turnOnCaseFan();
    GCode negative(-1, true), overflowing(INT32_MAX, true);
    scheduleCaseFanOff(&negative);
    scheduleCaseFanOff(&overflowing);
    assert(fanOn && invalid == 2 && CaseFan::offDelay() == 1000);

    GCode manualOff(0, true, true);
    scheduleCaseFanOff(&manualOff);
    assert(!fanOn);
    tools[0].targetTemperatureC = 70;
    CaseFan::updateFromHotendTargets();
    assert(!fanOn);
    turnOnCaseFan();
    assert(fanOn);
    tools[0].targetTemperatureC = 0;
    CaseFan::updateFromHotendTargets();
    assert(!fanOn);
    ThermalEmergency::ledger.fault=true;
    CaseFan::updateFromHotendTargets();
    assert(fanOn);
    HAL::now+=100000;
    CaseFan::service();
    assert(fanOn);
}

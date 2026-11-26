#pragma once
#include <stdint.h>

// Task0034: compact fault evidence shared by emergency LCD and serial reports.
// Not a heater controller or an actuator interlock. The caller must revoke
// physical heater power and latch admission in the same bounded operation.
// Foreground owner only: an ISR revokes hardware first and publishes its own
// minimal fault witness; it must never format or transmit diagnostic text.
// There is intentionally no clear()/acknowledge-to-resume operation.
namespace ThermalEmergency {
enum class Cause : uint8_t {
    None, SensorInvalid, StaleSample, NoRise, ExcessiveRise, HoldFailure,
    OverTemperature, HeatingWhileOff, LeaseExpired, WatchdogReset,
    CalibrationMismatch, InterruptedHeating, Count
};
enum class Severity : uint8_t { None, ThermalError, FireWarning };

constexpr Severity severity(Cause c) {
    return c==Cause::None || c>=Cause::Count ? Severity::None :
        c==Cause::OverTemperature || c==Cause::ExcessiveRise ||
        c==Cause::HeatingWhileOff ? Severity::FireWarning : Severity::ThermalError;
}
constexpr uint16_t bit(Cause c) {
    return c>Cause::None && c<Cause::Count ? uint16_t(1u << (uint8_t(c)-1)) : 0;
}
static_assert(uint8_t(Cause::Count)<=17, "Fault mask capacity exceeded");

// Evidence is copied, not recomputed after targets and duty are zeroed.
// Duty is the software request/admission, not measured electrical power.
// Optional fields are explicitly valid or unknown; raw ADC zero is not a
// missing-value sentinel. No float formatting or EEPROM writes at capture.
struct Evidence {
    uint32_t uptimeMs;
    uint32_t windowMs;
    float measuredC;
    float targetC;
    float deltaC;
    float threshold;
    uint16_t rawAdc;
    uint8_t requestedDuty;
    uint8_t admittedDuty;
    uint8_t sensorType;
    uint8_t powerLimit;
    uint8_t profile; // J0..J3, or unavailable
    uint16_t valid;
    enum : uint16_t {
        Measured=1, Target=2, Delta=4, Threshold=8, Raw=16,
        Requested=32, Admitted=64, Profile=128, Uptime=256,
        Window=512, SensorType=1024, PowerLimit=2048
    };
};
// Same ADC batch used by conversion/authorization. Only a fresh batch
// is marked raw-known; an unavailable/stale batch is not new evidence.
struct SensorObservation {
    uint16_t raw=0;
    bool rawKnown=false;
    Cause invalidCause=Cause::SensorInvalid;
    void observe(bool fresh,uint16_t value) {
        raw=value;rawKnown=fresh;
        invalidCause=fresh?Cause::SensorInvalid:Cause::StaleSample;
    }
    void apply(Evidence& e) const {
        if(rawKnown) {e.rawAdc=raw;e.valid|=Evidence::Raw;}
    }
};
struct Record {
    Evidence evidence;
    Cause cause;
    uint8_t channel;
};

template<uint8_t HeaterCount>
class Ledger {
    static_assert(HeaterCount>=1 && HeaterCount<=3, "RFx000 heater count");
    // One first snapshot and one first fire-warning snapshot for the machine;
    // subsequent per-channel reasons use bit masks instead of a log ring.
    Record first_{};
    Record fire_{};
    uint16_t masks_[HeaterCount+1]{}; // final slot: system/unknown heater
    // At most (HeaterCount+1)*(Count-1) unique observations, never wraps.
    uint8_t revision_=0;
public:
    static constexpr uint8_t System=HeaterCount;
    bool active() const { return first_.cause!=Cause::None; }
    uint8_t revision() const { return revision_; }
    Severity level() const {
        return fire_.cause!=Cause::None ? Severity::FireWarning :
            active() ? Severity::ThermalError : Severity::None;
    }
    const Record& first() const { return first_; }
    const Record& firstFire() const { return fire_; }
    uint16_t reasons(uint8_t channel) const {
        return channel<=System ? masks_[channel] : 0;
    }
    // False means invalid cause/channel or repeated observation; original
    // records are never overwritten by consequences of the shutdown.
    bool record(uint8_t channel, Cause cause, const Evidence& evidence) {
        const uint16_t mask=bit(cause);
        if(channel>System || !mask || (masks_[channel]&mask)) return false;
        masks_[channel] |= mask;
        ++revision_;
        if(!active()) first_={evidence,cause,channel};
        if(severity(cause)==Severity::FireWarning && fire_.cause==Cause::None)
            fire_={evidence,cause,channel};
        return true;
    }
};
}

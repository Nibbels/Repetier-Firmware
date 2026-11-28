#pragma once
#include "EventLog.h"

namespace EventLog {
// Stable wire reason IDs, independent of production enum order. 1..11
// reserved for thermal events; other families need an explicit assignment.
template<class Cause> uint8_t thermalReason(Cause c) {
    // Use an arithmetic identity only if every named wire assignment agrees.
    // Retain the explicit switch for other enum layouts; unknowns stay zero.
    constexpr bool identity =
        Cause::SensorInvalid == static_cast<Cause>(1) &&
        Cause::StaleSample == static_cast<Cause>(2) &&
        Cause::NoRise == static_cast<Cause>(3) &&
        Cause::ExcessiveRise == static_cast<Cause>(4) &&
        Cause::HoldFailure == static_cast<Cause>(5) &&
        Cause::OverTemperature == static_cast<Cause>(6) &&
        Cause::HeatingWhileOff == static_cast<Cause>(7) &&
        Cause::LeaseExpired == static_cast<Cause>(8) &&
        Cause::WatchdogReset == static_cast<Cause>(9) &&
        Cause::CalibrationMismatch == static_cast<Cause>(10) &&
        Cause::InterruptedHeating == static_cast<Cause>(11);
    if(identity)
        return c>=Cause::SensorInvalid && c<=Cause::InterruptedHeating ? uint8_t(c) : 0;
    switch(c) {
    case Cause::SensorInvalid:return 1;
    case Cause::StaleSample:return 2;
    case Cause::NoRise:return 3;
    case Cause::ExcessiveRise:return 4;
    case Cause::HoldFailure:return 5;
    case Cause::OverTemperature:return 6;
    case Cause::HeatingWhileOff:return 7;
    case Cause::LeaseExpired:return 8;
    case Cause::WatchdogReset:return 9;
    case Cause::CalibrationMismatch:return 10;
    case Cause::InterruptedHeating:return 11;
    default:return 0;
    }
}
inline bool deciC(float v,int16_t& out) {
    if(!(v>=-3276.8f&&v<=3276.7f)){out=0;return false;}
    const float scaled=v*10.f;
    const int32_t rounded=int32_t(scaled+(scaled<0?-0.5f:0.5f));
    if(rounded<-32768L||rounded>32767L){out=0;return false;}
    out=int16_t(rounded);return true;
}
// Adapter for the immutable ThermalEmergency::Record. No extra permanent
// payload. Conversion is solely for log display. Source remains alive and
// unchanged until Ring::step reports commit/failure. Does not read sensors.
template<class Record,uint8_t HeaterCount> class ThermalSource {
    static_assert(HeaterCount==2||HeaterCount==3,"RF heater count");
    const Record& r_;
    using E=decltype(r_.evidence);
    uint8_t id() const {
        if(r_.channel>HeaterCount)return 0;
        const auto channel=r_.channel==HeaterCount?Channel::System:
            r_.channel==HeaterCount-1?Channel::Bed:
            r_.channel==0?Channel::E0:Channel::E1;
        return eventId(channel,thermalReason(r_.cause));
    }
    bool value(uint8_t k,int16_t& out) const {
        const auto& e=r_.evidence;
        out=0;
        switch(k){
        case 0:return (e.valid&E::Measured)&&deciC(e.measuredC,out);
        case 1:return (e.valid&E::Target)&&deciC(e.targetC,out);
        case 2:return (e.valid&E::Delta)&&deciC(e.deltaC,out);
        case 3:return (e.valid&E::Threshold)&&deciC(e.threshold,out);
        default:return false;
        }
    }
    uint16_t validity() const {
        uint16_t mask=r_.evidence.valid&0x0fff;
        int16_t v;
        if(!value(0,v))mask&=uint16_t(~E::Measured);
        if(!value(1,v))mask&=uint16_t(~E::Target);
        if(!value(2,v))mask&=uint16_t(~E::Delta);
        if(!value(3,v))mask&=uint16_t(~E::Threshold);
        return mask;
    }
public:
    explicit ThermalSource(const Record& r):r_(r){}
    uint8_t byte(uint8_t offset) const {
        const auto& e=r_.evidence;
        if(offset<4)return e.valid&E::Uptime?byteOf(e.uptimeMs,offset):0;
        if(offset<8)return e.valid&E::Window?byteOf(e.windowMs,offset-4):0;
        if(offset<16){
            int16_t v=0;value((offset-8)/2,v);
            return byteOf(uint16_t(v),(offset-8)%2);
        }
        if(offset<18)return e.valid&E::Raw?byteOf(e.rawAdc,offset-16):0;
        switch(offset){
        case 18:return e.valid&E::Requested?e.requestedDuty:0;
        case 19:return e.valid&E::Admitted?e.admittedDuty:0;
        case 20:return e.valid&E::SensorType?e.sensorType:0;
        case 21:return e.valid&E::PowerLimit?e.powerLimit:0;
        case 22:return e.valid&E::Profile?e.profile:0;
        case 23:case 24:return byteOf(validity(),offset-23);
        case 25:return id();
        default:return 0;
        }
    }
};

// First and first-fire records are immutable in the existing ledger.
// Consume them in order even if escalation arrives during an EEPROM write.
// Reconstruct the tiny reference adapter on each visit; no 32-byte buffer.
// A fault present from the start as firstFire is written once, not twice.
enum class Capture : uint8_t { First, Fire, Done };
template<uint8_t HeaterCount> class ThermalCapture {
    Capture phase_=Capture::First;
public:
    template<uint8_t Slots,class Store,class Ledger>
    Progress service(Ring<Slots>& ring,Store& store,const Ledger& ledger) {
        if(!ledger.active()||phase_==Capture::Done)return Progress::Idle;
        if(ring.failed())return Progress::Failed;
        if(!ring.initialized())return ring.scanStep(store);
        const auto& first=ledger.first();
        const auto& fire=ledger.firstFire();
        // Deduce the unqualified record type from const references.
        return serviceRecord(ring,store,ledger,first,fire);
    }
private:
    template<uint8_t Slots,class Store,class Ledger,class Record>
    Progress serviceRecord(Ring<Slots>& ring,Store& store,const Ledger& ledger,
                           const Record& first,const Record& fire) {
        (void)ledger;
        if(phase_==Capture::Fire && thermalReason(fire.cause)==0)return Progress::Idle;
        ThermalSource<Record,HeaterCount> source(phase_==Capture::First?first:fire);
        if(!ring.pending()&&!ring.begin(source))return Progress::Failed;
        const auto result=ring.step(store,source);
        if(result==Progress::CommittedEntry){
            if(phase_==Capture::Fire ||
               (first.channel==fire.channel&&first.cause==fire.cause))
                phase_=Capture::Done;
            else phase_=Capture::Fire;
        }
        return result;
    }
};
}

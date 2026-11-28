#pragma once
#include "ThermalEmergency.h"
#ifdef __AVR__
#include <avr/pgmspace.h>
#define RF_THERMAL_TEXT(s) PSTR(s)
#else
#define RF_THERMAL_TEXT(s) (s)
#endif

namespace ThermalEmergency {

// Task0034: bounded foreground-only report. No serial wait, no printf float,
// heap, persistent text buffer or flash-to-SRAM string table. The caller must
// latch/revoke heat BEFORE scheduling this report. A sink must accept a whole
// span or none (UsbSerial::tryWriteSpan); no other producer may interleave
// within a line. Each service attempts at most one <=63-byte line.
// Host cancellation and command admission are separate emergency operations.
class ReportLine {
    char bytes_[63];
    uint8_t size_=0;
    bool ok_=true;
public:
    void character(char c) {
        if(size_>=sizeof(bytes_)) {ok_=false;return;}
        bytes_[size_++]=c;
    }
    void text(const char* s) {
        while(ok_) {
#ifdef __AVR__
            const char c=char(pgm_read_byte(s++));
#else
            const char c=*s++;
#endif
            if(!c)break;
            character(c);
        }
    }
    void number(uint32_t value) {
        char digits[10];
        uint8_t n=0;
        do {
            digits[n++]=char('0'+value%10);
            value/=10;
        }while(value);
        while(n)character(digits[--n]);
    }
    void unknown() {text(RF_THERMAL_TEXT("unknown"));}
    void optionalNumber(const Evidence& e,uint16_t flag,uint32_t value) {
        if(e.valid&flag)number(value);else unknown();
    }
    void optionalDecimal(const Evidence& e,uint16_t flag,float value) {
        // Comparisons reject NaN/Inf; bound before integer conversion.
        // Diagnostic presentation only, never used for safety decisions.
        if(!(e.valid&flag)||!(value>=-10000.f&&value<=10000.f)){unknown();return;}
        const uint32_t tenth=uint32_t((value<0?-value:value)*10.f+0.5f);
        if(value<0&&tenth)character('-');
        number(tenth/10);character('.');character(char('0'+tenth%10));
    }
    template<uint8_t N> void channel(uint8_t c) {
        if(c==N)text(RF_THERMAL_TEXT("SYSTEM"));
        else if(c==N-1)text(RF_THERMAL_TEXT("BED"));
        else {character('E');number(c);}
    }
    void cause(Cause c) {
        switch(c) {
        case Cause::SensorInvalid: text(RF_THERMAL_TEXT("SENSOR INVALID"));break;
        case Cause::StaleSample: text(RF_THERMAL_TEXT("STALE SAMPLE"));break;
        case Cause::NoRise: text(RF_THERMAL_TEXT("NO RISE"));break;
        case Cause::ExcessiveRise: text(RF_THERMAL_TEXT("EXCESSIVE RISE"));break;
        case Cause::HoldFailure: text(RF_THERMAL_TEXT("HOLD FAILURE"));break;
        case Cause::OverTemperature: text(RF_THERMAL_TEXT("OVER TEMP"));break;
        case Cause::HeatingWhileOff: text(RF_THERMAL_TEXT("HEATING WHILE OFF"));break;
        case Cause::LeaseExpired: text(RF_THERMAL_TEXT("LEASE EXPIRED"));break;
        case Cause::WatchdogReset: text(RF_THERMAL_TEXT("WATCHDOG RESET"));break;
        case Cause::CalibrationMismatch: text(RF_THERMAL_TEXT("CALIBRATION MISMATCH"));break;
        case Cause::InterruptedHeating: text(RF_THERMAL_TEXT("HEATING INTERRUPTED"));break;
        default: unknown();break;
        }
    }
    bool finish() {character('\r');character('\n');return ok_;}
    const char* data() const {return bytes_;}
    uint8_t size() const {return size_;}
};

enum class ReportProgress : uint8_t { Idle, Sent, Blocked, Skipped, Overflow };

// Host cancellation is independent of diagnostic replay/revision. Do not
// cancel a second time when the host requests the retained evidence again.
class CancellationCursor {
    uint8_t cursor_=0;
public:
    template<class Sink>
    ReportProgress service(bool active,Sink& sink) {
        if(!active||cursor_>=2)return ReportProgress::Idle;
        ReportLine line;
        if(!cursor_)line.text(RF_THERMAL_TEXT("RequestStop:"));
        else line.text(RF_THERMAL_TEXT("// action:cancel"));
        if(!line.finish())return ReportProgress::Overflow;
        if(!sink.tryWriteSpan(line.data(),line.size()))return ReportProgress::Blocked;
        ++cursor_;
        return ReportProgress::Sent;
    }
};

template<uint8_t N>
class ReportCursor {
    static constexpr uint8_t recordLines=13;
    static constexpr uint8_t causes=uint8_t(Cause::Count)-1;
    static constexpr uint8_t end=1+2*recordLines+(N+1)*causes;
    uint16_t masks_[N+1]{};
    uint8_t cursor_=0,startedRevision_=0,reportedRevision_=0;
    bool running_=false,fire_=false,replay_=false;

    static void recordLine(ReportLine& line,const Record& r,bool fire,uint8_t row) {
        line.text(fire?RF_THERMAL_TEXT("Thermal fire "):RF_THERMAL_TEXT("Thermal first "));
        const auto& e=r.evidence;
        switch(row) {
        case 0:
            line.channel<N>(r.channel);line.character(' ');line.cause(r.cause);break;
        case 1:
            line.text(RF_THERMAL_TEXT("uptime_ms: "));line.optionalNumber(e,Evidence::Uptime,e.uptimeMs);break;
        case 2:
            line.text(RF_THERMAL_TEXT("target_C: "));line.optionalDecimal(e,Evidence::Target,e.targetC);break;
        case 3:
            line.text(RF_THERMAL_TEXT("measured_C: "));line.optionalDecimal(e,Evidence::Measured,e.measuredC);break;
        case 4:
            line.text(RF_THERMAL_TEXT("delta_C: "));line.optionalDecimal(e,Evidence::Delta,e.deltaC);break;
        case 5:
            line.text(RF_THERMAL_TEXT("threshold: "));line.optionalDecimal(e,Evidence::Threshold,e.threshold);break;
        case 6:
            line.text(RF_THERMAL_TEXT("window_ms: "));line.optionalNumber(e,Evidence::Window,e.windowMs);break;
        case 7:
            line.text(RF_THERMAL_TEXT("raw_ADC: "));line.optionalNumber(e,Evidence::Raw,e.rawAdc);break;
        case 8:
            line.text(RF_THERMAL_TEXT("requested_PWM: "));line.optionalNumber(e,Evidence::Requested,e.requestedDuty);break;
        case 9:
            line.text(RF_THERMAL_TEXT("admitted_PWM: "));line.optionalNumber(e,Evidence::Admitted,e.admittedDuty);break;
        case 10:
            line.text(RF_THERMAL_TEXT("sensor_type: "));line.optionalNumber(e,Evidence::SensorType,e.sensorType);break;
        case 11:
            line.text(RF_THERMAL_TEXT("power_limit: "));line.optionalNumber(e,Evidence::PowerLimit,e.powerLimit);break;
        case 12:
            line.text(RF_THERMAL_TEXT("autotune_profile: "));line.optionalNumber(e,Evidence::Profile,e.profile);break;
        }
    }
    void advance() {
        if(++cursor_==end) {
            reportedRevision_=startedRevision_;
            running_=false;
        }
    }
public:
    // The caller must rate-limit read-only replay requests. A request
    // during a report coalesces into one repeat, never interrupts a line.
    void requestReplay() {replay_=true;}
    template<class Sink>
    ReportProgress service(const Ledger<N>& ledger,Sink& sink) {
        if(!ledger.active())return ReportProgress::Idle;
        if(!running_) {
            if(!replay_&&reportedRevision_==ledger.revision())return ReportProgress::Idle;
            startedRevision_=ledger.revision();
            fire_=ledger.level()==Severity::FireWarning;
            for(uint8_t c=0;c<=N;++c)masks_[c]=ledger.reasons(c);
            cursor_=0;running_=true;replay_=false;
        }
        // Snapshot mask/revision and whether fire evidence existed at start.
        // The ledger's first and first-fire records are immutable. Changes
        // during emission trigger a fresh complete report afterward.
        ReportLine line;
        if(cursor_==0) {
            if(fire_)line.text(RF_THERMAL_TEXT("Error: FIRE WARNING"));
            else if(ledger.first().cause==Cause::WatchdogReset)
                line.text(RF_THERMAL_TEXT("Error: WATCHDOG RESET"));
            else line.text(RF_THERMAL_TEXT("Error: THERMAL EMERGENCY"));
        } else if(cursor_<=recordLines) {
            recordLine(line,ledger.first(),false,cursor_-1);
        } else if(cursor_<=2*recordLines) {
            if(!fire_) {advance();return ReportProgress::Skipped;}
            recordLine(line,ledger.firstFire(),true,cursor_-1-recordLines);
        } else {
            const uint8_t offset=cursor_-1-2*recordLines;
            const uint8_t c=offset/causes;
            const Cause why=Cause(1+offset%causes);
            if(!(masks_[c]&causeBit(why))) {advance();return ReportProgress::Skipped;}
            line.text(RF_THERMAL_TEXT("Thermal affected "));
            line.channel<N>(c);line.character(' ');line.cause(why);
        }
        if(!line.finish())return ReportProgress::Overflow;
        if(!sink.tryWriteSpan(line.data(),line.size()))return ReportProgress::Blocked;
        advance();return ReportProgress::Sent;
    }
};
}
#undef RF_THERMAL_TEXT

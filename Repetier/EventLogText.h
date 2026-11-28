#pragma once
#include "EventLogRead.h"
#include "CompactEvent.h"
#ifdef __AVR__
#include <avr/pgmspace.h>
#define EVENT_LOG_TEXT(s) PSTR(s)
#else
#define EVENT_LOG_TEXT(s) (s)
#endif

namespace EventLog {
// Reuse the production ReportLine interface (63-byte atomic serial span).
// No second set of thermal cause texts, no float formatting and no stored
// strings in EEPROM. Stable wire reasons are explicitly mapped back to
// existing decoder labels rather than casting to a changing enum ordinal.
template<class Line,class Cause> void reasonText(Line& line,uint8_t reason) {
    switch(reason){
    case 1:line.cause(Cause::SensorInvalid);break;
    case 2:line.cause(Cause::StaleSample);break;
    case 3:line.cause(Cause::NoRise);break;
    case 4:line.cause(Cause::ExcessiveRise);break;
    case 5:line.cause(Cause::HoldFailure);break;
    case 6:line.cause(Cause::OverTemperature);break;
    case 7:line.cause(Cause::HeatingWhileOff);break;
    case 8:line.cause(Cause::LeaseExpired);break;
    case 9:line.cause(Cause::WatchdogReset);break;
    case 10:line.cause(Cause::CalibrationMismatch);break;
    case 11:line.cause(Cause::InterruptedHeating);break;
    default:line.text(EVENT_LOG_TEXT("REASON:"));line.number(reason);break;
    }
}
template<class Line> void channelText(Line& line,Channel channel) {
    switch(channel){
    case Channel::E0:line.text(EVENT_LOG_TEXT("E0"));break;
    case Channel::E1:line.text(EVENT_LOG_TEXT("E1"));break;
    case Channel::Bed:line.text(EVENT_LOG_TEXT("BED"));break;
    case Channel::System:line.text(EVENT_LOG_TEXT("SYSTEM"));break;
    }
}
template<class Line> void optionalNumber(Line& line,const EntryView& v,uint16_t bit,uint32_t number) {
    if(v.valid()&bit)line.number(number);else line.unknown();
}
template<class Line> void optionalTemp(Line& line,const EntryView& v,uint16_t bit,uint8_t field) {
    if(!(v.valid()&bit)){line.unknown();return;}
    const int32_t n=v.value(field);
    if(n<0)line.character('-');
    const uint32_t magnitude=uint32_t(n<0?-n:n);
    line.number(magnitude/10);line.character('.');
    line.character(char('0'+magnitude%10));
}
inline bool scanEntry(const EntryView& v){
    return v.channel()==Channel::System&&isScanReason(v.reason());
}
template<class Line> void scanReasonText(Line& line,uint8_t reason){
    switch(reason){
#define RF_SCAN_LOG(name, value, code, shortText, hint) \
    case code: \
        line.text(EVENT_LOG_TEXT(shortText)); \
        if(sizeof(hint)>1){line.character(' ');line.text(EVENT_LOG_TEXT(hint));} \
        break;
    RF_SCAN_FAULTS(RF_SCAN_LOG)
#undef RF_SCAN_LOG
    default:line.unknown();break;
    }
}
template<class Line> void scanLine(Line& line,const EntryView& v,uint8_t row){
    switch(row){
    case 0:
        line.text(EVENT_LOG_TEXT("LOG #"));line.number(v.sequence());
        if(v.context(0)==uint8_t(ScanContext::HeatBed))
            line.text(EVENT_LOG_TEXT(" HBS"));
        else if(v.context(0)==uint8_t(ScanContext::ZOffset))
            line.text(EVENT_LOG_TEXT(" ZOS"));
        else line.text(EVENT_LOG_TEXT(" SCAN"));
        line.text(EVENT_LOG_TEXT(" CODE:"));line.number(v.reason());break;
    case 1:scanReasonText(line,v.reason());break;
    case 2:
        line.text(EVENT_LOG_TEXT("uptime_ms:"));optionalNumber(line,v,256,v.uptime());break;
    }
}
inline uint8_t rows(const EntryView& v){return scanEntry(v)?3:v.reason()<=11?6:1;}
template<class Line,class Cause> void entryLine(Line& line,const EntryView& v,uint8_t row) {
    if(scanEntry(v)){scanLine(line,v,row);return;}
    switch(row){
    case 0:
        line.text(EVENT_LOG_TEXT("LOG #"));line.number(v.sequence());
        line.character(' ');channelText(line,v.channel());line.character(' ');
        reasonText<Line,Cause>(line,v.reason());break;
    case 1:
        line.text(EVENT_LOG_TEXT("uptime_ms:"));optionalNumber(line,v,256,v.uptime());
        line.text(EVENT_LOG_TEXT(" window_ms:"));optionalNumber(line,v,512,v.window());break;
    case 2:
        line.text(EVENT_LOG_TEXT("measured_C:"));optionalTemp(line,v,1,0);
        line.text(EVENT_LOG_TEXT(" target_C:"));optionalTemp(line,v,2,1);break;
    case 3:
        line.text(EVENT_LOG_TEXT("delta_C:"));optionalTemp(line,v,4,2);
        line.text(EVENT_LOG_TEXT(" threshold:"));optionalTemp(line,v,8,3);break;
    case 4:
        line.text(EVENT_LOG_TEXT("raw_ADC:"));optionalNumber(line,v,16,v.raw());
        line.text(EVENT_LOG_TEXT(" req_PWM:"));optionalNumber(line,v,32,v.context(0));
        line.text(EVENT_LOG_TEXT(" admitted_PWM:"));optionalNumber(line,v,64,v.context(1));break;
    case 5:
        line.text(EVENT_LOG_TEXT("sensor_type:"));optionalNumber(line,v,1024,v.context(2));
        line.text(EVENT_LOG_TEXT(" power_limit:"));optionalNumber(line,v,2048,v.context(3));
        line.text(EVENT_LOG_TEXT(" profile:"));optionalNumber(line,v,128,v.context(4));break;
    }
}
// Construct on an existing nonblocking USB sink; retain only the row
// cursor. Partial progress deliberately returns false to keep Reader on the
// same entry. One line per visit, including when a host is not reading.
// Reset row cursor when starting/restarting a Reader after Changed.
template<class Line,class Cause,class Sink> class TextSink {
    Sink& sink_;
    // Valid rows are 0..5. Keep terminal formatting failure in the cursor.
    static constexpr uint8_t Failed=255;
    uint8_t row_=0;
public:
    explicit TextSink(Sink& sink):sink_(sink){}
    Sink& sink() const { return sink_; }
    void reset(){row_=0;}
    bool failed()const{return row_==Failed;}
    bool tryEntry(const EntryView& v) {
        if(failed())return false;
        Line line;
        entryLine<Line,Cause>(line,v,row_);
        if(!line.finish()){row_=Failed;return false;}
        if(!sink_.tryWriteSpan(line.data(),line.size()))return false;
        if(++row_<rows(v))return false;
        row_=0;return true;
    }
};
}
#undef EVENT_LOG_TEXT

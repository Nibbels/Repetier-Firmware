#pragma once
#include "ThermalEmergency.h"
#ifdef __AVR__
#include <avr/pgmspace.h>
#define RF_PANEL_TEXT(s) PSTR(s)
#else
#define RF_PANEL_TEXT(s) (s)
#endif

namespace ThermalEmergency {

// Fixed 16/20-column output; no scrolling or heap. The cause/channel is
// always shown together and never truncated. Numeric temperature evidence
// is explicitly unknown when absent, not a fictional zero measurement.
template<uint8_t Cols> class PanelLine {
    char* out_;
    uint8_t used_=0;
public:
    explicit PanelLine(char (&out)[Cols+1]):out_(out){out_[0]=0;}
    void character(char c){
        if(used_<Cols){out_[used_++]=c;out_[used_]=0;}
    }
    void text(const char* p){
        while(used_<Cols){
#ifdef __AVR__
            char c=char(pgm_read_byte(p++));
#else
            char c=*p++;
#endif
            if(!c)break;
            character(c);
        }
    }
    void integer(uint16_t n){
        char b[5];uint8_t i=0;
        do{b[i++]=char('0'+n%10);n/=10;}while(n);
        while(i)character(b[--i]);
    }
    void temperature(bool valid,float value){
        if(!valid||!(value>=-9999.f&&value<=9999.f)){character('?');return;}
        if(value<0)character('-');
        integer(uint16_t((value<0?-value:value)+0.5f));
    }
    template<uint8_t N> void channel(uint8_t c){
        if(c==N)text(RF_PANEL_TEXT("SYS:"));
        else if(c==N-1)text(RF_PANEL_TEXT("BED:"));
        else {character('E');integer(c);character(':');}
    }
    void cause(Cause c){
        switch(c){
        case Cause::SensorInvalid:text(RF_PANEL_TEXT("SENSOR"));break;
        case Cause::StaleSample:text(RF_PANEL_TEXT("STALE ADC"));break;
        case Cause::NoRise:text(RF_PANEL_TEXT("NO RISE"));break;
        case Cause::ExcessiveRise:text(RF_PANEL_TEXT("FAST RISE"));break;
        case Cause::HoldFailure:text(RF_PANEL_TEXT("HOLD FAIL"));break;
        case Cause::OverTemperature:text(RF_PANEL_TEXT("OVER TEMP"));break;
        case Cause::HeatingWhileOff:text(RF_PANEL_TEXT("HEAT STUCK"));break;
        case Cause::LeaseExpired:text(RF_PANEL_TEXT("LEASE"));break;
        case Cause::WatchdogReset:text(RF_PANEL_TEXT("WDT RESET"));break;
        case Cause::CalibrationMismatch:text(RF_PANEL_TEXT("CALIBRATION"));break;
        case Cause::InterruptedHeating:text(RF_PANEL_TEXT("INTERRUPTED"));break;
        default:text(RF_PANEL_TEXT("LOCKED"));break;
        }
    }
};
// Primary live failure stays visible even when EEPROM or USB is unavailable.
// Full affected-reason history remains available via the serial report.
// No page timer, count, division or cycling through the ledger masks.
template<uint8_t N> const Record& primaryFault(const Ledger<N>& ledger){
    return ledger.level()==Severity::FireWarning?ledger.firstFire():ledger.first();
}

// Supported RFx000 panels: 16x4/20x4. Live cause and temperature are not
// dependent on the persistence service or the diagnostic log contents.
template<uint8_t Cols,uint8_t Rows,uint8_t N>
void panelRow(char (&out)[Cols+1],uint8_t row,const Ledger<N>& ledger,
              bool liveValid,float liveC){
    static_assert((Cols==16||Cols==20)&&Rows==4,"RF LCD dimensions");
    PanelLine<Cols> line(out);
    const auto& fault=primaryFault(ledger);
    const uint8_t channel=ledger.active()?fault.channel:N;
    if(row==0){
        if(ledger.level()==Severity::FireWarning)line.text(RF_PANEL_TEXT("FIRE WARNING"));
        else if(ledger.first().cause==Cause::WatchdogReset)line.text(RF_PANEL_TEXT("WATCHDOG RESET"));
        else line.text(RF_PANEL_TEXT("THERMAL ERROR"));
    }else if(row==1){
        line.template channel<N>(channel);line.cause(fault.cause);
    }else if(row==2){
        line.template channel<N>(channel);
        if(channel==N)line.text(RF_PANEL_TEXT("CHECK LOG"));
        else {
            line.temperature(liveValid,liveC);
            line.character('/');
            line.temperature(ledger.active()&&(fault.evidence.valid&Evidence::Target),fault.evidence.targetC);
            line.text(RF_PANEL_TEXT(" C"));
        }
    }else if(row==3){
        if(ledger.level()==Severity::FireWarning)line.text(RF_PANEL_TEXT("POWER OFF! CHECK"));
        else line.text(RF_PANEL_TEXT("CHECK SERIAL LOG"));
    }
}
}
#undef RF_PANEL_TEXT

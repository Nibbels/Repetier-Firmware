#pragma once
#include "ThermalEventSource.h"
#include "EventLogText.h"
#ifdef __AVR__
#include <avr/pgmspace.h>
#define EVENT_SERVICE_TEXT(s) PSTR(s)
#else
#define EVENT_SERVICE_TEXT(s) (s)
#endif

namespace EventLog {
// Foreground integration candidate. The same service handles normal
// read-only retrieval after restart and terminal thermal capture.
// Caller retains authoritative safety state and runs thermal/protocol
// work first. Store's owner gate controls all EEPROM access.
// A single thermal capture producer owns the writer in this version.
template<uint8_t Slots,uint8_t HeaterCount,class Line,class Cause,class Sink>
class Service {
    Ring<Slots> ring_;
    ThermalCapture<HeaterCount> capture_;
    Reader<Slots> reader_;
    Sink& sink_;
    TextSink<Line,Cause,Sink> text_;
    enum class Query : uint8_t { Idle, Begin, Read, End, Changed, Error };
    Query query_=Query::Idle;
    bool logFailed_=false;

    bool notice(const char* msg) {
        Line line;line.text(msg);
        return line.finish()&&sink_.tryWriteSpan(line.data(),line.size());
    }
public:
    explicit Service(Sink& sink):sink_(sink),text_(sink){}
    // Coalesce duplicate history requests; no command or heater admission.
    void requestHistory(){if(query_==Query::Idle)query_=Query::Begin;}
    bool historyPending()const{return query_!=Query::Idle;}
    bool logFailed()const{return logFailed_;}
    const Ring<Slots>& ring()const{return ring_;}

    template<class Store,class Ledger> void service(Store& store,const Ledger& ledger) {
        // No EEPROM read/write or format at all during normal idle/printing
        // without an explicit request; terminal faults get priority.
        if(ledger.active()&&!logFailed_){
            const auto p=capture_.service(ring_,store,ledger);
            if(p==Progress::Failed)logFailed_=true;
            else if(p!=Progress::Idle)return;
        }
        if(query_==Query::Idle)return;
        if(!ring_.initialized()){ring_.scanStep(store);return;}
        if(query_==Query::Begin){
            if(ring_.pending()||!store.ready())return;
            if(!notice(EVENT_SERVICE_TEXT("LOG BEGIN")))return;
            text_.reset();
            if(reader_.begin(ring_))query_=Query::Read;
            else query_=Query::Error;
            return;
        }
        if(query_==Query::Read){
            const auto p=reader_.step(ring_,store,text_);
            if(text_.failed())query_=Query::Error;
            else if(p==ReadProgress::Done)query_=Query::End;
            else if(p==ReadProgress::Changed)query_=Query::Changed;
            return;
        }
        const char* message=query_==Query::End?EVENT_SERVICE_TEXT("LOG END"):
            query_==Query::Changed?EVENT_SERVICE_TEXT("LOG CHANGED; RETRY"):
            EVENT_SERVICE_TEXT("LOG OUTPUT ERROR");
        if(notice(message))query_=Query::Idle;
    }
};
}
#undef EVENT_SERVICE_TEXT

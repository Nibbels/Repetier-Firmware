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
// A single writer arbitrates immutable thermal evidence and one compact
// operation failure. Complete an already-started record before switching
// sources; otherwise terminal thermal evidence gets priority.
template<uint8_t Slots,uint8_t HeaterCount,class Line,class Cause,class Sink>
class Service {
    Ring<Slots> ring_;
    ThermalCapture<HeaterCount> capture_;
    Reader<Slots> reader_;
    Sink& sink_;
    TextSink<Line,Cause,Sink> text_;
    enum class Query : uint8_t { Idle, Begin, Read, End, Changed, Error };
    Query query_=Query::Idle;
    CompactEvent compact_;
    bool compactStarted_=false,logFailed_=false;

    template<class Store> Progress serviceCompact(Store& store) {
        if(!compact_.pending())return Progress::Idle;
        if(!ring_.initialized())return ring_.scanStep(store);
        if(!compactStarted_){
            if(!ring_.begin(compact_))return Progress::Failed;
            compactStarted_=true;
        }
        const auto p=ring_.step(store,compact_);
        if(p==Progress::CommittedEntry||p==Progress::Failed){
            compact_.clear();compactStarted_=false;
        }
        return p;
    }

    bool notice(const char* msg) {
        Line line;line.text(msg);
        return line.finish()&&sink_.tryWriteSpan(line.data(),line.size());
    }
public:
    explicit Service(Sink& sink):sink_(sink),text_(sink){}
    // Coalesce duplicate history requests; no command or heater admission.
    void requestHistory(){if(query_==Query::Idle)query_=Query::Begin;}
    bool historyPending()const{return query_!=Query::Idle;}
    bool scanPending()const{return compact_.pending();}
    bool logFailed()const{return logFailed_;}
    // Admission copies six logical bytes; never accesses EEPROM. If the
    // single slot is occupied, caller retains existing live diagnosis.
    bool submitScan(ScanAbort reason,ScanContext operation,uint32_t now){
        return !logFailed_&&compact_.admitScan(reason,operation,now);
    }
    const Ring<Slots>& ring()const{return ring_;}

    template<class Store,class Ledger> void service(Store& store,const Ledger& ledger) {
        // No EEPROM read/write or format at all during normal idle/printing
        // without an explicit request; terminal faults get priority.
        if(!logFailed_){
            Progress p=Progress::Idle;
            if(compactStarted_)p=serviceCompact(store);
            else {
                if(ledger.active())p=capture_.service(ring_,store,ledger);
                if(p==Progress::Idle)p=serviceCompact(store);
            }
            if(p==Progress::Failed){
                logFailed_=true;compact_.clear();compactStarted_=false;
            }else if(p!=Progress::Idle)return;
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

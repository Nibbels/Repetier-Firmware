#pragma once
#include "EventLog.h"
#include "CalibrationScan.h"

namespace EventLog {
// No full Record/Payload staging for nonthermal failures. A scan abort
// needs only the finalized reason, operation context and uptime. The
// active record is immutable until the ring reports commit or failure.
// Channel System is used for operation-level events; no invented sensor
// values. The code is diagnostic and never drives scan cleanup.
enum class ScanContext : uint8_t { HeatBed=1, ZOffset=2 };
inline bool isScanReason(uint8_t reason) { return reason>=12&&reason<=29; }
struct CompactEvent {
    uint32_t uptime=0;
    uint8_t id=0,context=0;
    bool pending()const{return id!=0;}
    void clear(){id=0;}
    bool admitScan(ScanAbort reason,ScanContext operation,uint32_t now) {
        const uint8_t code=scanFaultCode(reason);
        if(pending()||!code||
           (operation!=ScanContext::HeatBed&&operation!=ScanContext::ZOffset))
            return false;
        uptime=now;context=uint8_t(operation);
        id=eventId(Channel::System,code);
        return true;
    }
    uint8_t byte(uint8_t offset)const{
        if(offset<4)return byteOf(uptime,offset);
        if(offset==18)return context;
        // Uptime validity bit 256; all measurement fields unknown.
        if(offset==24)return 1;
        if(offset==25)return id;
        return 0;
    }
};
}

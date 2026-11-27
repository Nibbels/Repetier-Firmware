// Bounded foreground EEPROM byte transport. The device starts exactly one
// programming operation; subsequent readback waits for stable hardware.
// Never use this from an ISR, with IRQs masked, or from recursive work.
#ifndef RF_EEPROM_ZONE_BYTE_IO_H
#define RF_EEPROM_ZONE_BYTE_IO_H
#include "EepromZoneLayout.h"

namespace EepromZones {
template<class Device> class ReadyByteIO {
    Device& device_;
    bool good_=true;
    bool wait(uint16_t pos) {
        if(!good_ || pos<Base || pos>=End || blockOf(pos)>Last || !device_.foreground()) {
            good_=false;return false;
        }
        if(device_.ready()) return true;
        const uint32_t start=device_.now();
        do {
            if(uint32_t(device_.now()-start)>=20 || !device_.foreground()) {
                good_=false;return false;
            }
            device_.service();
        } while(!device_.ready());
        if(!device_.foreground()) {good_=false;return false;}
        return true;
    }
public:
    explicit ReadyByteIO(Device& d):device_(d) {}
    bool good() const {return good_;}
    uint8_t read(uint16_t pos) {
        return wait(pos)?device_.readReady(pos):0xFF;
    }
    void update(uint16_t pos,uint8_t v) {
        if(wait(pos)) device_.updateReady(pos,v);
    }
};

// One foreground transaction owner, including its service callbacks.
// Failed recursive entry never releases the outer owner's guard.
class SessionGuard {
    bool& busy_;
    bool entered_;
public:
    explicit SessionGuard(bool& busy):busy_(busy),entered_(!busy) {
        if(entered_) busy_=true;
    }
    ~SessionGuard(){if(entered_)busy_=false;}
    SessionGuard(const SessionGuard&)=delete;
    SessionGuard& operator=(const SessionGuard&)=delete;
    bool entered() const {return entered_;}
};
}
#endif

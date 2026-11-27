// Physical AVR adapter for ReadyByteIO. Only the settings service owns
// the upper-bank address space; HAL remains a raw byte transport.
#ifndef RF_EEPROM_ZONE_AVR_H
#define RF_EEPROM_ZONE_AVR_H
#include "EepromZoneByteIO.h"
namespace EepromZones {
class AvrDevice {
public:
    // Single foreground gate for settings and the future event logger.
    static bool& settingsBusy();
    static bool logMayAccess();
    static void enableRuntimeServices();
    bool foreground() const;
    bool ready() const;
    uint32_t now() const;
    void service();
    uint8_t readReady(uint16_t pos);
    void updateReady(uint16_t pos,uint8_t value);
};
}
#endif

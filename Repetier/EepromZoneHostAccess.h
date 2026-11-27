// Public physical address/type whitelist for the new schema.
// Caller includes the resolved BoardConfiguration first. Labels are shared
// by M205 and M206, but only the exact field starts are accepted.
#ifndef RF_EEPROM_ZONE_HOST_ACCESS_H
#define RF_EEPROM_ZONE_HOST_ACCESS_H
#include "EepromZoneLayout.h"
namespace EepromZones {
enum class HostAccess : uint8_t { Unknown, ReadOnly, Writable };
inline HostAccess hostAccess(int32_t pos,uint8_t wire) {
    // Check the signed value before any narrowing.
    if(pos<address(1)+PayloadOffset || pos>=End || wire>3)
        return HostAccess::Unknown;
#define RF_ZONE_FIELD(type, start, label, digits, writable) \
    if(pos==(start) && wire==(type)) return (writable)?HostAccess::Writable:HostAccess::ReadOnly;
#include "EepromZoneHostFields.h"
#undef RF_ZONE_FIELD
    return HostAccess::Unknown;
}
}
#endif

// Public physical address/type whitelist for the new schema.
// Caller includes the resolved BoardConfiguration first. Labels are shared
// by M205 and M206, but only the exact field starts are accepted.
#ifndef RF_EEPROM_ZONE_HOST_ACCESS_H
#define RF_EEPROM_ZONE_HOST_ACCESS_H
#include "EepromZonePatch.h"
#include <math.h>
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
// Canonical wire decoding after syntax validation in GCodeInput. The
// signed full-size P/T/S values must be checked before narrowing.
inline bool decodeHostEdit(int32_t pos,int32_t wire,int32_t value,float real,Patch& patch,HostAccess& access) {
    if(wire<0 || wire>3) return false;
    access=hostAccess(pos,uint8_t(wire));
    if(access==HostAccess::Unknown) return false;
    if((wire==0 && (value<0 || value>255)) ||
       (wire==1 && (value<-32768L || value>32767L)) ||
       (wire==3 && !isfinite(real))) return false;
    const auto f=static_cast<Field>(pos);
    switch(wire) {
    case 0:patch=Patch::byte(f,uint8_t(value));break;
    case 1:patch=Patch::word(f,uint16_t(value));break;
    case 2:patch=Patch::dword(f,uint32_t(value));break;
    case 3:patch=Patch::real(f,real);break;
    default:return false;
    }
    return true;
}
// Only gains, sensor identity and the power cap revoke calibration;
// DMIN/DMAX and hotend cooler speed are separate adjustments.
inline bool calibrationRevocation(const Patch& edit,Patch& revoke) {
    const uint8_t id=blockOf(edit.pos);
    if(id!=BedPid && (id<Hotend0 || id>=Hotend0+NUM_EXTRUDER)) return false;
    const uint16_t start=address(id)+PayloadOffset;
    const uint8_t offset=edit.pos-start;
    if(offset!=0 && offset!=4 && offset!=8 && offset!=14 && offset!=15) return false;
    revoke=Patch::identity(static_cast<Field>(start+16),0,0,0);
    return true;
}
}
#endif

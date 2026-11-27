// Bounded internal typed field edits. A small edit copies only affected
// persisted block(s), not unrelated mutable live settings. No RAM mirror.
// Public M206 must additionally use the exact HostAccess whitelist.
#ifndef RF_EEPROM_ZONE_PATCH_H
#define RF_EEPROM_ZONE_PATCH_H
#include "EepromZoneCodec.h"
namespace EepromZones {
struct Patch {
    uint16_t pos;
    uint8_t width;
    uint32_t bits;
    static Patch byte(Field f,uint8_t v){return {address(f),1,v};}
    static Patch word(Field f,uint16_t v){return {address(f),2,v};}
    static Patch dword(Field f,uint32_t v){return {address(f),4,v};}
    static Patch real(Field f,float v){
        uint32_t bits;memcpy(&bits,&v,4);return dword(f,bits);
    }
    static Patch identity(Field f,uint8_t marker,uint8_t sensor,uint8_t power){
        return {address(f),3,uint32_t(marker)|(uint32_t(sensor)<<8)|(uint32_t(power)<<16)};
    }
};
inline uint32_t patchMask(const Patch* edits,uint8_t count) {
    if(!edits || !count) return 0;
    uint32_t mask=0;
    for(uint8_t i=0;i<count;++i) {
        const auto& p=edits[i];
        const uint8_t id=blockOf(p.pos);
        if(!id || id>Last || id==BedSafety || !p.width || p.width!=fieldWidth(p.pos)
            || p.pos<address(id)+PayloadOffset || p.pos+p.width>address(id)+PayloadOffset+length(id))
            return 0;
        for(uint8_t j=0;j<i;++j)
            if(p.pos<edits[j].pos+edits[j].width && edits[j].pos<p.pos+p.width) return 0;
        mask|=blockBit(id);
    }
    return mask;
}
template<class Store> struct PatchProvider {
    Store& store;
    const Patch* edits;
    uint8_t count;
    bool operator()(uint8_t id,uint8_t* data,uint8_t n) {
        if(!store.readBlock(id,data,n)) return false;
        for(uint8_t i=0;i<count;++i) {
            const auto& p=edits[i];
            if(blockOf(p.pos)!=id)continue;
            const uint8_t offset=p.pos-address(id)-PayloadOffset;
            for(uint8_t b=0;b<p.width;++b)data[offset+b]=uint8_t(p.bits>>(8*b));
        }
        return true;
    }
};
}
#endif

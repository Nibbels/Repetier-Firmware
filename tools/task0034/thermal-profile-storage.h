#pragma once
#include "../../Repetier/ThermalRelayProfile.h"

// A39 research format, no assigned physical EEPROM address or runtime
// authorization. The owner must supply a durable calibration generation
// and invalidate the marker BEFORE any bound setting can change.
namespace ThermalProfileStorage {
constexpr uint8_t bytes=44, marker=0xd7, version=1;
struct Identity {
    uint8_t slot,sensor,cap,method; // fixed E0/E1/bed slots, not loop index
    uint16_t targetDeciC;
    uint32_t generation;
};
using ThermalRelayProfile::Summary;
struct Record {uint8_t data[bytes];};
inline bool validIdentity(const Identity& id){
    return id.slot<3&&id.sensor&&id.cap>=40&&id.generation&&
        id.targetDeciC>400&&id.targetDeciC<=10000&&
        (id.slot==2?(id.method==2||id.method==3):(id.method<=1));
}
inline void put16(uint8_t* b,uint16_t v){b[0]=uint8_t(v);b[1]=uint8_t(v>>8);}
inline void put32(uint8_t* b,uint32_t v){for(uint8_t i=0;i<4;++i)b[i]=uint8_t(v>>(8*i));}
inline uint16_t get16(const uint8_t* b){return uint16_t(b[0])|(uint16_t(b[1])<<8);}
inline uint32_t get32(const uint8_t* b){uint32_t v=0;for(uint8_t i=0;i<4;++i)v|=uint32_t(b[i])<<(8*i);return v;}
inline uint32_t checksum(const uint8_t* b){
    uint32_t crc=0xffffffffUL;
    for(uint8_t i=1;i<40;++i){crc^=b[i];for(uint8_t j=0;j<8;++j)crc=(crc>>1)^((crc&1)?0xedb88320UL:0);}
    return ~crc;
}
inline bool quantize(float v,uint16_t& q){
    if(!(v>=0&&v<=1100.f))return false;
    q=uint16_t(v*10.f);
    if(float(q)*.1f<v)++q;
    return q<=11000;
}
inline bool shape(const Summary& s,const Identity& id){
    const uint32_t maximum=id.slot==2?600000UL:180000UL;
    return validIdentity(id)&&!(s.mask&0xc0)&&s.completed>=3&&s.completed<=20&&
        s.minOffMs>0&&s.minOffMs<=s.maxOffMs&&s.maxOffMs<=maximum&&s.peakDelayMs<=s.maxOffMs;
}
inline bool encode(const Summary& s,const Identity& id,Record& out){
    if(!shape(s,id))return false;
    Record r{};auto b=r.data;
    b[0]=marker;b[1]=version;b[2]=id.slot;b[3]=id.sensor;b[4]=id.cap;b[5]=id.method;
    b[6]=s.mask;b[7]=s.completed;put16(b+8,id.targetDeciC);
    for(uint8_t i=0;i<6;++i){uint16_t q=0;if((s.mask&(1u<<i))&&!quantize(s.rates[i],q))return false;put16(b+10+2*i,q);}
    uint16_t rebound;if(!quantize(s.rebound,rebound))return false;
    put16(b+22,rebound);put32(b+24,s.peakDelayMs);put32(b+28,s.minOffMs);put32(b+32,s.maxOffMs);
    put32(b+36,id.generation);put32(b+40,checksum(b));out=r;return true;
}
inline bool decode(const Record& r,const Identity& id,Summary& out){
    const auto b=r.data;
    if(!validIdentity(id)||b[0]!=marker||b[1]!=version||b[2]!=id.slot||b[3]!=id.sensor||b[4]!=id.cap||b[5]!=id.method||
       get16(b+8)!=id.targetDeciC||get32(b+36)!=id.generation||get32(b+40)!=checksum(b))return false;
    Summary s;s.mask=b[6];s.completed=b[7];
    s.peakDelayMs=get32(b+24);s.minOffMs=get32(b+28);s.maxOffMs=get32(b+32);
    if(!shape(s,id))return false;
    for(uint8_t i=0;i<6;++i){const uint16_t q=get16(b+10+2*i);if(q>11000||(!(s.mask&(1u<<i))&&q))return false;s.rates[i]=q/10.f;}
    const uint16_t rebound=get16(b+22);if(rebound>11000)return false;s.rebound=rebound/10.f;
    out=s;return true;
}
// Store is an explicit byte read/write adapter. put() must complete the
// byte write before get() returns. No assumption of multi-byte atomicity.
// A failed invalidation MUST prevent the caller's bound setting mutation.
template<class Store> bool invalidate(Store& s,uint16_t base){s.put(base,0);return s.get(base)==0;}
template<class Store> bool write(Store& s,uint16_t base,const Summary& p,const Identity& id){
    Record r;if(!encode(p,id,r))return false;
    if(!invalidate(s,base))return false;
    for(uint8_t i=1;i<bytes;++i)s.put(base+i,r.data[i]);
    for(uint8_t i=1;i<bytes;++i)if(s.get(base+i)!=r.data[i])return false;
    s.put(base,marker);return s.get(base)==marker;
}
template<class Store> bool read(Store& s,uint16_t base,const Identity& id,Summary& out){
    Record r;for(uint8_t i=0;i<bytes;++i)r.data[i]=s.get(base+i);return decode(r,id,out);
}
}

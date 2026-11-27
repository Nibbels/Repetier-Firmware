// Fixed-width, little-endian settings payload codec. One 29-byte caller
// buffer, no persistent mirror. Coverage checking catches omitted/overlapping
// fields in complete block providers; raw metadata is not addressable here.
#ifndef RF_EEPROM_ZONE_CODEC_H
#define RF_EEPROM_ZONE_CODEC_H
#include "EepromZoneLayout.h"
#include <string.h>

namespace EepromZones {
class BlockEncoder {
    uint8_t* data_;
    uint8_t id_, size_;
    uint32_t written_=0;
    bool ok_;
public:
    BlockEncoder(uint8_t id,uint8_t* data,uint8_t n)
        :data_(data),id_(id),size_(n),ok_(data && id && id<=Last && n==length(id)) {}
    void bytes(Field field,const uint8_t* src,uint8_t count) {
        const uint16_t pos=address(field);
        if(!ok_ || !src || !count || blockOf(pos)!=id_ || pos<address(id_)+PayloadOffset
           || pos+count>address(id_)+PayloadOffset+size_) {ok_=false;return;}
        const uint8_t offset=pos-address(id_)-PayloadOffset;
        const uint32_t mask=((uint32_t(1)<<count)-1)<<offset;
        if(written_&mask) {ok_=false;return;}
        for(uint8_t i=0;i<count;++i) data_[offset+i]=src[i];
        written_|=mask;
    }
    void u8(Field f,uint8_t v) {bytes(f,&v,1);}
    void u16(Field f,uint16_t v) {
        const uint8_t b[]={uint8_t(v),uint8_t(v>>8)};bytes(f,b,2);
    }
    void u32(Field f,uint32_t v) {
        const uint8_t b[]={uint8_t(v),uint8_t(v>>8),uint8_t(v>>16),uint8_t(v>>24)};bytes(f,b,4);
    }
    void f32(Field f,float v) {
        static_assert(sizeof(float)==4,"settings float is binary32");
        uint32_t bits;memcpy(&bits,&v,4);u32(f,bits);
    }
    bool complete() const {
        return ok_ && written_==((uint32_t(1)<<size_)-1);
    }
};

class BlockDecoder {
    const uint8_t* data_;
    uint8_t id_, size_;
    bool ok_;
    const uint8_t* checkedBytes(Field field,uint8_t count) {
        const uint16_t pos=address(field);
        if(!ok_ || blockOf(pos)!=id_ || pos<address(id_)+PayloadOffset
           || pos+count>address(id_)+PayloadOffset+size_) {ok_=false;return nullptr;}
        const uint8_t offset=pos-address(id_)-PayloadOffset;
        return data_+offset;
    }
public:
    BlockDecoder(uint8_t id,const uint8_t* data,uint8_t n)
        :data_(data),id_(id),size_(n),ok_(data && id && id<=Last && n==length(id)) {}
    uint8_t u8(Field f) {
        const uint8_t* p=checkedBytes(f,1);
        return p?p[0]:0;
    }
    uint16_t u16(Field f) {
        const uint8_t* p=checkedBytes(f,2);
        return p?uint16_t(uint16_t(p[0]) | (uint16_t(p[1])<<8)):0;
    }
    uint32_t u32(Field f) {
        const uint8_t* p=checkedBytes(f,4);
        return p?(uint32_t(p[0]) | (uint32_t(p[1])<<8)
                  | (uint32_t(p[2])<<16) | (uint32_t(p[3])<<24)):0;
    }
    float f32(Field f) {
        const uint32_t bits=u32(f);float value;
        memcpy(&value,&bits,4);return value;
    }
    bool good() const {return ok_;}
};
}
#endif

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
    uint32_t read(Field field,uint8_t count) {
        const uint16_t pos=address(field);
        if(!ok_ || blockOf(pos)!=id_ || pos<address(id_)+PayloadOffset
           || pos+count>address(id_)+PayloadOffset+size_) {ok_=false;return 0;}
        const uint8_t offset=pos-address(id_)-PayloadOffset;
        uint32_t value=0;
        for(uint8_t i=0;i<count;++i) value|=uint32_t(data_[offset+i])<<(8*i);
        return value;
    }
public:
    BlockDecoder(uint8_t id,const uint8_t* data,uint8_t n)
        :data_(data),id_(id),size_(n),ok_(data && id && id<=Last && n==length(id)) {}
    uint8_t u8(Field f) {return uint8_t(read(f,1));}
    uint16_t u16(Field f) {return uint16_t(read(f,2));}
    uint32_t u32(Field f) {return read(f,4);}
    float f32(Field f) {
        const uint32_t bits=u32(f);float value;
        memcpy(&value,&bits,4);return value;
    }
    bool good() const {return ok_;}
};
}
#endif

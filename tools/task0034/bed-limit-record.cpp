// A63: EEPROM bed-limit transaction research; no physical address assigned.
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>
namespace BedLimit {
constexpr uint8_t bytes=6, marker=0xb6, format=1, ceiling=150, factory=120, margin=10;
constexpr bool allowed(uint8_t v){return v==0||(v>=40&&v<=ceiling);}
struct Record {uint8_t b[bytes];};
struct State {
    bool valid;uint8_t value;
    // Invalid configuration is not permission to heat. Monitoring remains
    // independent of admission; do not invent a 0 C overtemperature fault.
    uint8_t cutoff()const{return valid&&value?value:factory;}
    uint8_t targetMax()const{return valid&&value?uint8_t(value-margin):0;}
};
uint16_t crc(const uint8_t* b){uint16_t c=0xffff;for(unsigned i=1;i<=3;++i){c^=uint16_t(b[i])<<8;for(unsigned j=0;j<8;++j)c=uint16_t((c<<1)^((c&0x8000)?0x1021:0));}return c;}
Record encode(uint8_t value){assert(allowed(value));Record r{{marker,format,value,uint8_t(~value),0,0}};const auto c=crc(r.b);r.b[4]=uint8_t(c);r.b[5]=uint8_t(c>>8);return r;}
State decode(const Record& r){const auto b=r.b;const bool ok=b[0]==marker&&b[1]==format&&allowed(b[2])&&uint8_t(~b[2])==b[3]&&crc(b)==(uint16_t(b[4])|(uint16_t(b[5])<<8));return {ok,ok?b[2]:uint8_t(0)};}
template<class Store> State read(Store& s){Record r;for(unsigned i=0;i<bytes;++i)r.b[i]=s.get(i);return decode(r);}
// Caller revokes output and admission before this operation and keeps them
// off until successful global settings integrity update and reload. An
// interruption may retain the OLD complete record, or the NEW complete
// record, or invalidate the record; never admit a partially written body.
template<class Store> bool write(Store& s,uint8_t value){if(!allowed(value))return false;const auto r=encode(value);s.put(0,0);if(s.get(0)!=0)return false;for(unsigned i=1;i<bytes;++i)s.put(i,r.b[i]);for(unsigned i=1;i<bytes;++i)if(s.get(i)!=r.b[i])return false;s.put(0,marker);const auto v=read(s);return v.valid&&v.value==value;}
}
struct PowerLoss{};
struct Store {uint8_t mem[BedLimit::bytes]{};int cut=-1,ignore=-1,n=0;uint8_t torn=0;uint8_t get(unsigned p)const{assert(p<sizeof mem);return mem[p];}void put(unsigned p,uint8_t v){assert(p<sizeof mem);const int i=n++;if(i==cut){mem[p]=torn;throw PowerLoss{};}if(i!=ignore)mem[p]=v;}};
int main(){using namespace BedLimit;unsigned shapes=0,corrupt=0,torn=0,ignored=0;
    for(unsigned value=0;value<=255;++value){
        Store s;const bool ok=write(s,uint8_t(value));assert(ok==allowed(uint8_t(value)));const auto d=read(s);assert(d.valid==ok);
        assert(d.cutoff()<=ceiling&&d.targetMax()<=ceiling-margin);
        if(!ok||value==0)assert(!d.targetMax());
        if(ok){assert(d.value==value);const auto r=encode(uint8_t(value));for(unsigned i=0;i<bytes;++i)for(unsigned bit=0;bit<8;++bit){auto bad=r;bad.b[i]^=uint8_t(1u<<bit);auto x=decode(bad);assert(!x.valid&&!x.targetMax());++corrupt;}}
        ++shapes;
    }
    for(uint8_t old:{uint8_t(0),uint8_t(40),uint8_t(80),uint8_t(120),uint8_t(150)})for(uint8_t next:{uint8_t(0),uint8_t(40),uint8_t(80),uint8_t(120),uint8_t(150)}){
        Store base;assert(write(base,old));
        for(int at=0;at<bytes+1;++at)for(unsigned v=0;v<256;++v){Store s=base;s.n=0;s.cut=at;s.torn=uint8_t(v);try{write(s,next);assert(false);}catch(const PowerLoss&){}
            const auto d=read(s);const bool oldComplete=at==0&&v==marker,newComplete=at==bytes&&v==marker;
            assert(d.valid==(oldComplete||newComplete));if(d.valid)assert(d.value==(oldComplete?old:next));else assert(d.targetMax()==0);++torn;}
        for(int at=0;at<bytes+1;++at){Store s=base;s.n=0;s.ignore=at;const bool ok=write(s,next);auto d=read(s);if(ok)assert(d.valid&&d.value==next);else if(d.valid)assert(at==0&&d.value==old);++ignored;}
    }
    std::printf("Bed cutoff record: %u value cases, %u bit corruptions, %u torn writes, %u ignored writes passed\n",shapes,corrupt,torn,ignored);
}

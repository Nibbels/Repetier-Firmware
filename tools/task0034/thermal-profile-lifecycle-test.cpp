// A43: logical lower-settings anchor + upper-profile transaction model.
// NOT a firmware EEPROM address allocation or an authorization mechanism.
#include "thermal-profile-storage.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <initializer_list>
using namespace ThermalProfileStorage;
namespace {
constexpr uint8_t expectedProtocol=10; // research proposal only; real firmware remains v9
constexpr uint8_t anchorMarker=0x6b,anchorSize=10;
constexpr uint16_t lower(uint8_t slot){return uint16_t(slot)*anchorSize;}
constexpr uint16_t upper(uint8_t slot){return 3*anchorSize+uint16_t(slot)*bytes;}
struct PowerLoss{};
struct Store {
    uint8_t mem[3*(anchorSize+bytes)]{};
    uint8_t protocol=expectedProtocol;
    int cut=-1,ignore=-1,n=0;
    uint8_t torn=0;
    bool settingsChanged=false;
    uint8_t get(unsigned pos)const{assert(pos<sizeof mem);return mem[pos];}
    void put(unsigned pos,uint8_t v){assert(pos<sizeof mem);int i=n++;
        if(i==cut){mem[pos]=torn;throw PowerLoss{};}
        if(i!=ignore)mem[pos]=v;
    }
};
// CRC covers slot and generation, including format, without relying on struct layout.
uint32_t anchorCrc(uint8_t slot,uint32_t generation){
    uint8_t data[40]{};data[1]=expectedProtocol;data[2]=slot;put32(data+3,generation);
    return checksum(data);
}
uint32_t readAnchor(Store& s,uint8_t slot){
    if(slot>=3||s.protocol!=expectedProtocol||s.get(lower(slot))!=anchorMarker||s.get(lower(slot)+1)!=expectedProtocol)return 0;
    uint8_t b[8];for(unsigned i=0;i<8;++i)b[i]=s.get(lower(slot)+2+i);
    const uint32_t generation=get32(b);
    return generation&&get32(b+4)==anchorCrc(slot,generation)?generation:0;
}
bool readProfile(Store& s,Identity id,Summary& p){
    if(!validIdentity(id)||readAnchor(s,id.slot)!=id.generation)return false;
    return read(s,upper(id.slot),id,p);
}
bool prepareMutation(Store& s,uint8_t slot){
    if(slot>=3)return false;
    // Reject a failed revoke before changing *any* bound setting. Clearing
    // both layers also allows generation restart when no old valid data exist.
    return invalidate(s,lower(slot))&&invalidate(s,upper(slot));
}
bool commitProfile(Store& s,const Identity& id,const Summary& p){
    if(!validIdentity(id)||s.protocol!=expectedProtocol||s.get(lower(id.slot))!=0||s.get(upper(id.slot))!=0)return false;
    if(!write(s,upper(id.slot),p,id))return false;
    uint8_t b[anchorSize]{};b[1]=expectedProtocol;put32(b+2,id.generation);put32(b+6,anchorCrc(id.slot,id.generation));
    for(uint8_t i=1;i<anchorSize;++i)s.put(lower(id.slot)+i,b[i]);
    for(uint8_t i=1;i<anchorSize;++i)if(s.get(lower(id.slot)+i)!=b[i])return false;
    s.put(lower(id.slot),anchorMarker);
    return readAnchor(s,id.slot)==id.generation;
}
bool changeAndRetune(Store& s,const Identity& id,const Summary& p){
    if(!prepareMutation(s,id.slot))return false;
    s.settingsChanged=true; // boundary: settings commit is external to this model
    return commitProfile(s,id,p);
}
Summary profile(float rate){Summary p;p.mask=Summary::First1;p.completed=5;p.rates[0]=rate;p.rebound=2;p.peakDelayMs=1000;p.minOffMs=2000;p.maxOffMs=3000;return p;}
}
int main(){unsigned torn=0,ignored=0,migration=0,anchors=0;
    for(uint8_t slot=0;slot<3;++slot){
        Identity old{slot,13,128,uint8_t(slot==2?3:1),2000,1};Identity next=old;next.generation=2;
        Store initial;assert(changeAndRetune(initial,old,profile(1)));initial.settingsChanged=false;
        Summary p;assert(readProfile(initial,old,p)&&p.rates[0]==1);
        Store complete=initial;complete.n=0;assert(changeAndRetune(complete,next,profile(2)));
        assert(!readProfile(complete,old,p)&&readProfile(complete,next,p)&&p.rates[0]==2);
        const int writes=complete.n;assert(writes==2+bytes+1+anchorSize);
        for(int at=0;at<writes;++at)for(unsigned value=0;value<256;++value){
            Store s=initial;s.n=0;s.cut=at;s.torn=uint8_t(value);
            try{changeAndRetune(s,next,profile(2));assert(false);}catch(const PowerLoss&){}
            const bool oldOK=readProfile(s,old,p);
            assert(oldOK==(at==0&&value==anchorMarker));
            if(oldOK)assert(!s.settingsChanged&&p.rates[0]==1);
            const bool newOK=readProfile(s,next,p);
            assert(newOK==(at==writes-1&&value==anchorMarker));
            if(newOK)assert(s.settingsChanged&&p.rates[0]==2);
            for(uint8_t other=0;other<3;++other)if(other!=slot){
                assert(!std::memcmp(s.mem+lower(other),initial.mem+lower(other),anchorSize));
                assert(!std::memcmp(s.mem+upper(other),initial.mem+upper(other),bytes));
            }
            ++torn;
        }
        for(int at=0;at<writes;++at){Store s=initial;s.n=0;s.ignore=at;const bool ok=changeAndRetune(s,next,profile(2));
            assert(ok==readProfile(s,next,p));if(ok)assert(s.settingsChanged&&p.rates[0]==2);
            if(readProfile(s,old,p))assert(!s.settingsChanged&&p.rates[0]==1);
            ++ignored;
        }
        // A protocol-9 boot or a reset of the lower settings region must
        // never admit a retained upper record. Migration revokes before bump.
        for(uint8_t protocol:{uint8_t(0),uint8_t(8),uint8_t(9),uint8_t(11),uint8_t(255)}){
            Store s=initial;s.protocol=protocol;assert(!readProfile(s,old,p));
            assert(prepareMutation(s,slot));s.protocol=expectedProtocol;
            assert(!readProfile(s,old,p));assert(commitProfile(s,next,profile(2))&&readProfile(s,next,p));++migration;
        }
        Store s=initial;std::memset(s.mem,0,3*anchorSize);assert(!readProfile(s,old,p));
        assert(prepareMutation(s,slot));assert(commitProfile(s,next,profile(2))&&readProfile(s,next,p));++migration;
        for(unsigned b=0;b<anchorSize;++b)for(unsigned bit=0;bit<8;++bit){s=initial;s.mem[lower(slot)+b]^=uint8_t(1u<<bit);assert(!readProfile(s,old,p));++anchors;}
        // Matching sensor/cap after editing away and back does not resurrect old data.
        s=initial;assert(prepareMutation(s,slot));s.settingsChanged=true;assert(!readProfile(s,old,p));
        assert(prepareMutation(s,slot));assert(!readProfile(s,old,p));
    }
    std::printf("Lifecycle model: %u torn bytes, %u ignored writes, %u migration/reset paths, %u anchor corruptions passed\n",torn,ignored,migration,anchors);
}

#include "thermal-profile-storage.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
using namespace ThermalProfileStorage;
struct PowerLoss{};
struct Store {
    uint8_t mem[3*bytes]{};
    int cut=-1,ignore=-1,n=0;
    uint8_t torn=0;
    uint8_t get(unsigned pos)const{assert(pos<sizeof mem);return mem[pos];}
    void put(unsigned pos,uint8_t v){assert(pos<sizeof mem);const int i=n++;
        if(i==cut){mem[pos]=torn;throw PowerLoss{};}
        if(i!=ignore)mem[pos]=v;
    }
};
static Summary profile(float rate){Summary s;s.mask=Summary::First1|Summary::Cool5;s.completed=8;s.rates[0]=rate;s.rates[5]=.47f;s.rebound=4.23f;s.peakDelayMs=2100;s.minOffMs=5000;s.maxOffMs=8000;return s;}
static void eq(float a,float b){assert(std::fabs(a-b)<.0001f);}
int main(){
    unsigned corrupt=0,torn=0,ignored=0,mismatch=0,invalid=0;
    for(uint8_t slot=0;slot<3;++slot){
        const uint16_t base=slot*bytes;
        const Identity oldId{slot,13,128,uint8_t(slot==2?3:1),uint16_t(slot==2?900:2000),1};
        Identity newId=oldId;newId.generation=2;
        const Summary old=profile(1.23f),next=profile(35.81f);
        Store initial;std::memset(initial.mem,0x55,sizeof initial.mem);
        assert(write(initial,base,old,oldId));assert(initial.n==bytes+1);
        Summary p;assert(read(initial,base,oldId,p));eq(p.rates[0],1.3f);eq(p.rates[5],.5f);eq(p.rebound,4.3f);assert(p.mask==old.mask&&p.peakDelayMs==2100);
        Record record;assert(encode(next,newId,record));
        for(unsigned b=0;b<bytes;++b)for(unsigned bit=0;bit<8;++bit){Record bad=record;bad.data[b]^=uint8_t(1u<<bit);assert(!decode(bad,newId,p));++corrupt;}
        for(unsigned field=0;field<6;++field){Identity bad=newId;
            if(field==0)bad.slot=(slot+1)%3;
            if(field==1)++bad.sensor;
            if(field==2)++bad.cap;
            if(field==3)bad.method^=1;
            if(field==4)++bad.targetDeciC;
            if(field==5)++bad.generation;
            assert(!decode(record,bad,p));++mismatch;
        }
        // Every possible partially written byte at every write boundary.
        for(int at=0;at<bytes+1;++at)for(unsigned v=0;v<256;++v){
            Store s=initial;s.n=0;s.cut=at;s.torn=uint8_t(v);
            try{write(s,base,next,newId);assert(false);}catch(const PowerLoss&){}
            const bool oldOK=read(s,base,oldId,p);assert(oldOK==(at==0&&v==marker));
            if(oldOK)eq(p.rates[0],1.3f);
            const bool newOK=read(s,base,newId,p);assert(newOK==(at==bytes&&v==marker));
            if(newOK)eq(p.rates[0],35.9f);
            for(unsigned pos=0;pos<sizeof s.mem;++pos)if(pos<base||pos>=unsigned(base)+bytes)assert(s.mem[pos]==0x55);
            ++torn;
        }
        // Ignored hardware write must never make a partially updated body valid.
        for(int at=0;at<bytes+1;++at){Store s=initial;s.n=0;s.ignore=at;const bool ok=write(s,base,next,newId);
            assert(ok==read(s,base,newId,p));if(ok)eq(p.rates[0],35.9f);++ignored;
        }
        // Marker revoked before settings change and remains revoked on rollback.
        Store s=initial;s.n=0;assert(invalidate(s,base));assert(!read(s,base,oldId,p));assert(!read(s,base,newId,p));
        for(int f=0;f<7;++f){Summary bad=next;if(f==0)bad.mask=0xff;if(f==1)bad.completed=2;if(f==2)bad.minOffMs=0;
            if(f==3)bad.peakDelayMs=bad.maxOffMs+1;
            if(f==4)bad.rebound=-1;
            if(f==5)bad.rates[0]=std::numeric_limits<float>::quiet_NaN();
            if(f==6)bad.maxOffMs=700000;
            assert(!encode(bad,newId,record));++invalid;
        }
        // Invalid input does not invalidate an unrelated old valid record.
        s=initial;Summary bad=next;bad.completed=0;assert(!write(s,base,bad,newId));assert(read(s,base,oldId,p));
        // Recompute CRC on malformed content: schema/semantic checks still reject.
        assert(encode(next,newId,record));record.data[6]=0;put32(record.data+40,checksum(record.data));assert(!decode(record,newId,p));
    }
    std::printf("Storage prototype: %u single-bit corruptions, %u torn-write states, %u ignored writes, %u identity mismatches, %u invalid profiles passed; record %u bytes\n",corrupt,torn,ignored,mismatch,invalid,unsigned(bytes));
}

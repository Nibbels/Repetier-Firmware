#include "z-net-demand.h"
#include <cassert>
#include <cstdio>
#include <cstdlib>
using ZNetResearch::arbitrate;
int main(){
    unsigned decisions=0;
    for(int8_t n=-1;n<=1;++n)for(int8_t c=-1;c<=1;++c)
        for(unsigned mask=0;mask<16;++mask){
            const bool na=mask&1,ca=mask&2,ba=mask&4,pa=mask&8;
            auto d=arbitrate(n,c,na,ca,ba,pa);
            assert(d.nominal==0||d.nominal==n);
            assert(d.correction==0||d.correction==c);
            assert(d.nominal+d.correction==d.pulse);
            assert(d.pulse>=-1&&d.pulse<=1);
            if(d.pulse)assert(pa);
            if(d.correction)assert(ca);
            if(n&&d.nominal&&d.correction)assert(ba&&n==-c&&!d.pulse);
            if(n&&(!c||n==c||!ba||!ca))assert(d.nominal==(na&&pa?n:0));
            if(n&&c&&n==-c&&ba&&ca)assert(d.nominal==n&&d.correction==c&&!d.pulse);
            ++decisions;
        }
    // Closed request cycles with temporary physical unavailability. Debt is
    // retained by the caller until consumed, never reset at a segment join.
    int64_t N=0,C=0,P=0;
    uint64_t cancellations=0,physical=0,deferrals=0;
    uint32_t rng=0x9e3779b9;
    auto rand=[&](){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;};
    auto sign=[](int64_t x)->int8_t{return x>0?1:x<0?-1:0;};
    for(unsigned cycle=0;cycle<10000;++cycle){
        int64_t nGoal=0,cGoal=0;
        for(unsigned i=0;i<512;++i){
            nGoal+=int(rand()%3)-1;cGoal+=int(rand()%5)-2;
            auto d=arbitrate(sign(nGoal-N),sign(cGoal-C),true,true,true,rand()%4!=0);
            N+=d.nominal;C+=d.correction;P+=d.pulse;
            assert(P==N+C);
            cancellations+=d.nominal&&d.correction;
            physical+=d.pulse!=0;
            deferrals+=N!=nGoal||C!=cGoal;
        }
        // Opposite requests can disappear without motor motion; any
        // remaining signed requests drain at explicitly legal opportunities.
        while(N||C){
            auto d=arbitrate(sign(-N),sign(-C),true,true,true,true);
            N+=d.nominal;C+=d.correction;P+=d.pulse;
            assert(P==N+C);
            cancellations+=d.nominal&&d.correction;
            physical+=d.pulse!=0;
        }
        assert(!P);
    }
    printf("Z net arbiter decision cases=%u closures=10000 cancellations=%llu physical=%llu retained_debt=%llu PASS\n",
        decisions,(unsigned long long)cancellations,(unsigned long long)physical,(unsigned long long)deferrals);
}

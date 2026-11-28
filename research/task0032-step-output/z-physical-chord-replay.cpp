#include "z-physical-chord.h"
#include <cassert>
#include <cstdio>
#include <cstdint>
#include <algorithm>
#include <vector>
using ZFeedforwardResearch::PhysicalChord;
static uint64_t checks=0;
static uint32_t rng=0x9e3779b9;
static uint32_t rand32(){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static int32_t oracle(int32_t a,int32_t b,uint32_t i,uint32_t n){
    const int64_t p=2*(int64_t(a)*(n-i)+int64_t(b)*i)+n,d=2*int64_t(n);
    return int32_t(p>=0?p/d:-((-p+d-1)/d));
}
static void check(int32_t a,int32_t b,uint32_t n){
    PhysicalChord f,r;
    if(!f.prepare(a,b,n)){
        assert(!f.advance());
        assert((a<b?int64_t(b)-a:int64_t(a)-b)/n>32767);
        return;
    }
    assert(r.prepare(b,a,n));
    int32_t prev=a;
    for(uint32_t i=0;i<=n;++i){
        assert(f.value()==r.value() || i!=0 || a==b || r.value()==b);
        assert(f.value()==oracle(a,b,i,n));
        assert(r.value()==oracle(a,b,n-i,n));
        assert(f.remaining()==n-i&&r.remaining()==n-i);
        assert(a<=b?f.value()>=prev:f.value()<=prev);
        prev=f.value();
        assert(f.advance()==(i<n)&&r.advance()==(i<n));checks+=2;
    }
}
int main(){
    for(int a=-8;a<=8;++a)for(int b=-8;b<=8;++b)
        for(unsigned n=1;n<=64;++n)check(a,b,n);
    for(unsigned i=0;i<10000;++i){
        int32_t a=int32_t(rand32()%1000000)-500000,b=int32_t(rand32()%1000000)-500000;
        check(a,b,1+rand32()%2048);
    }
    check(INT32_MIN,INT32_MIN+32767,1);
    check(INT32_MAX,INT32_MAX-32767,1);
    check(INT32_MIN,INT32_MAX,32767); // rejected: >32767 per sample
    check(-32768,32780,32767);
    check(123456,123456,32767);
    PhysicalChord c;
    assert(!c.prepare(1,2,32768)&&!c.advance());
    assert(!c.prepare(1,2,0)&&!c.advance());
    printf("Physical Z chord checks=%llu sizeof_host=%zu PASS\n",(unsigned long long)checks,sizeof(c));
}

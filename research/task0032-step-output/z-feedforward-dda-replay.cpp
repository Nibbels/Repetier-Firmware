#include "z-feedforward-dda.h"
#include <cassert>
#include <cstdio>
#include <algorithm>
#include <vector>
#include <limits.h>
using ZFeedforwardResearch::Chord;
static uint64_t checked=0;
static uint32_t rng=0x9e3779b9;
static uint32_t random32(){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static int32_t oracle(int32_t a,int32_t b,uint32_t i,uint32_t n){
    // floor(real+1/2), including negative endpoints. Wide oracle only.
    int64_t p=2*(int64_t(a)*(n-i)+int64_t(b)*i)+n;
    int64_t d=2*int64_t(n);
    return int32_t(p>=0?p/d:-((-p+d-1)/d));
}
static void check(int32_t a,int32_t b,uint32_t n){
    Chord f,r;
    assert(f.prepare(a,b,n)&&r.prepare(b,a,n));
    std::vector<int32_t> vals;vals.reserve(n+1);
    for(uint32_t i=0;i<=n;++i){
        assert(f.value()==oracle(a,b,i,n));
        assert(f.remaining()==n-i);
        vals.push_back(f.value());
        assert(f.advance()==(i<n));++checked;
    }
    for(uint32_t i=0;i<=n;++i){
        assert(r.value()==vals[n-i]);
        assert(r.advance()==(i<n));++checked;
    }
}
int main(){
    for(int32_t a=-8;a<=8;++a)for(int32_t b=-8;b<=8;++b)
        for(uint32_t n=1;n<=64;++n)check(a,b,n);
    for(unsigned i=0;i<10000;++i){
        const int32_t a=int32_t(random32()%65549)-32768;
        const int32_t b=int32_t(random32()%65549)-32768;
        check(a,b,1+random32()%2048);
    }
    for(auto n:{1U,2U,65535U,65536U,65537U})check(-32768,32780,n);
    Chord f;
    assert(f.prepare(-32768,32780,INT32_MAX));
    for(unsigned i=0;i<1024;++i){
        assert(f.value()==oracle(-32768,32780,i,INT32_MAX));assert(f.advance());
    }
    assert(!f.prepare(0,0,0)&&!f.advance());
    assert(!f.prepare(0,0,UINT32_MAX)&&!f.advance());
    assert(!f.prepare(INT32_MIN,0,1)&&!f.advance());
    assert(!f.prepare(0,INT32_MAX,1)&&!f.advance());
    printf("Z feedforward DDA checked=%llu exact directional round trips PASS, host sizeof=%zu\n",
           (unsigned long long)checked,sizeof(Chord));
}

#include "StopTailEnergy.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <algorithm>
#include <random>
using namespace StopTailEnergy;
static uint16_t floorRoot(uint32_t x){
    uint32_t r=uint32_t(std::sqrt(double(x)));
    assert(uint64_t(r)*r<=x && uint64_t(r+1)*(r+1)>x);
    return uint16_t(r);
}
static unsigned long cases,groups,infeasible;
void check(uint16_t rate,uint16_t exit,uint32_t a,uint32_t available,uint8_t pack){
    auto p=prepare(rate,exit,a,available);
    const uint64_t e0=uint64_t(rate)*rate;
    const uint64_t e1=uint64_t(exit)*exit;
    const uint64_t diff=e0>e1?e0-e1:0;
    const uint64_t want=diff&&a?(diff+2ull*a-1)/(2ull*a):0;
    assert(p.steps<=available);
    assert(p.feasible==(!diff || (a && want<=available)));
    if(!p.feasible)++infeasible;
    if(!a){assert(!next(p,rate,pack,floorRoot).steps);++cases;return;}
    assert(p.steps==std::min<uint64_t>(want,available));
    const auto count=p.steps;
    uint16_t root=rate;
    uint32_t delivered=0;
    // Long synthetic profiles: verify boundary groups and random inputs
    // separately, not billions of redundant per-step loops.
    if(count>2000){++cases;return;}
    while(p.steps){
        const auto g=next(p,root,pack,floorRoot);
        assert(g.steps>0 && g.steps<=pack && g.steps<=p.steps);
        assert(g.energy<=p.energy && g.energy>=p.floor);
        assert(uint64_t(p.energy-g.energy)<=2ull*a*g.steps);
        assert(g.root<=root);
        const double ideal=32000000.0*g.steps/(std::sqrt(double(p.energy))+std::sqrt(double(g.energy)));
        assert(g.ticks>=ideal);
        delivered+=g.steps;root=g.root;commit(p,g);++groups;
    }
    assert(delivered==count);
    if(p.feasible)assert(p.energy==p.floor);
    else if(diff)assert(p.energy>p.floor);
    ++cases;
}
int main(){
    // Stock primary step-domain accelerations (rounded conservatively).
    for(uint32_t a:{228570u,128000u,25600u})
      for(unsigned v=1;v<=31250;v+=37)
        for(uint8_t n:{uint8_t(1),uint8_t(2),uint8_t(4),uint8_t(10)})
          for(uint16_t exit:{uint16_t(0),uint16_t(192),uint16_t(990),uint16_t(v)}){
            auto p=prepare(v,exit,a,UINT32_MAX);
            check(v,exit,a,p.steps,n);
            if(p.steps)check(v,exit,a,p.steps-1,n);
            check(v,exit,a,p.steps+1,n);
          }
    std::mt19937 rnd(0x0032);
    for(unsigned n=0;n<100000;++n){
       uint16_t v=uint16_t(rnd()),exit=uint16_t(rnd());uint32_t a=rnd();
       check(v,exit,a,rnd()%2048,uint8_t(1+rnd()%10));
    }
    for(uint16_t v:{uint16_t(0),uint16_t(1),uint16_t(31250),uint16_t(65535)})
      for(uint16_t exit:{uint16_t(0),uint16_t(1),v})
        for(uint32_t a:{0u,1u,UINT32_MAX/2,UINT32_MAX})
          for(uint32_t available:{0u,1u,32u,UINT32_MAX})check(v,exit,a,available,10);
    auto xy=prepare(15238,990,228570,10000),z=prepare(11520,192,128000,10000);
    assert(xy.steps==506 && z.steps==519);
    printf("Stop-tail energy: %lu plans, %lu groups, %lu infeasible paths; stock XY %u Z %u steps PASS\n",cases,groups,infeasible,unsigned(xy.steps),unsigned(z.steps));
}

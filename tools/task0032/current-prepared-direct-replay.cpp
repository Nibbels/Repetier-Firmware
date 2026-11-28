#include "PreparedDirectProfile.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <initializer_list>
#include <random>
using namespace PreparedDirectProfile;
static uint16_t floorRoot(uint32_t x){return uint16_t(std::sqrt(double(x)));}
static unsigned long profiles,groups,releases,brakeGroups;
static void check(uint16_t maxRate,uint16_t start,uint16_t exit,uint32_t a,uint32_t length,uint8_t pack){
 const auto lim=prepare(maxRate,exit,a);assert(lim.valid);
 uint32_t e=uint32_t(start)*start,remain=length;
 if(e>cap(lim,remain))return;
 uint16_t root=start;
 while(remain){
  // Every completed boundary must permit braking within admitted distance.
  const uint32_t diff=e>lim.exitEnergy?e-lim.exitEnergy:0;
  const uint32_t want=diff/lim.drop+uint32_t(diff%lim.drop!=0);
  assert(want<=remain);++releases;
  // Compare exact energy (not rounded rate) for the requested tail.
  StopTailEnergy::Plan brake{e,lim.exitEnergy,lim.drop,want,true};
  if(want){
   auto first=StopTailEnergy::next(brake,root,pack,floorRoot);
   assert(first.steps&&first.energy<=e&&first.energy>=lim.exitEnergy);
   assert(uint64_t(e-first.energy)<=2ull*a*first.steps);++brakeGroups;
  }
  auto g=next(lim,e,root,remain,pack,floorRoot);
  assert(g.steps>0 && g.steps<=pack && g.steps<=remain);
  assert(g.energy<=lim.ceiling && g.energy<=cap(lim,remain-g.steps));
  uint32_t diff2=g.energy>e?g.energy-e:e-g.energy;
  assert(uint64_t(diff2)<=2ull*a*g.steps);
  if(e||g.energy){
   const double ideal=32000000.0*g.steps/(std::sqrt(double(e))+std::sqrt(double(g.energy)));
   assert(g.ticks>=ideal);
  }else assert(g.ticks>0);
  remain-=g.steps;e=g.energy;root=g.root;++groups;
 }
 assert(e<=lim.exitEnergy);++profiles;
}
int main(){
 for(uint32_t a:{228570u,128000u,25600u,1u,UINT32_MAX})
  for(uint16_t vmax:{uint16_t(192),uint16_t(3048),uint16_t(11520),uint16_t(15238),uint16_t(31250),uint16_t(65535)})
   for(uint16_t exit:{uint16_t(0),uint16_t(1),uint16_t(vmax/4),vmax})
    for(uint16_t start:{uint16_t(0),exit,vmax})
     for(uint32_t length:{1u,2u,4u,31u,32u,33u,256u,506u,519u,1000u,5000u})
      for(uint8_t n:{uint8_t(1),uint8_t(2),uint8_t(5),uint8_t(10)})check(vmax,start,exit,a,length,n);
 std::mt19937 rnd(0x0032);
 for(unsigned n=0;n<20000;++n){uint16_t max=uint16_t(1+rnd()%65535),exit=uint16_t(rnd()%(unsigned(max)+1)),start=uint16_t(rnd()%(unsigned(max)+1));check(max,start,exit,1+rnd(),rnd()%2000,uint8_t(1+rnd()%10));}
 printf("Prepared direct reference: %lu feasible profiles, %lu groups, %lu stop boundaries, %lu first brake groups PASS\n",profiles,groups,releases,brakeGroups);
}

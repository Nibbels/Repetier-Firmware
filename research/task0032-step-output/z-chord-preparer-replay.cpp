#include "z-chord-preparer.h"
#include "z-chord-handoff.h"
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include "BedCompensation.h"
enum {X_AXIS,Y_AXIS};
namespace Printer {float stepsPerMm(uint8_t){return 152.38f;}}
static short g_ZCompensationMatrix[5][5]={
    {0,0,20,40,60},{0,-80,-280,-90,-320},{20,-310,-30,-390,-40},
    {40,-60,-420,-20,-240},{60,-220,-50,-300,0}
};
static uint8_t g_uZMatrixMax[2]={4,4};
#include "BedMatrixInterpolation.h"
using namespace ZFeedforwardResearch;
static constexpr int32_t F=256,H=8656;
struct Reader {
    uint32_t probes=0,corrections=0;
    Region region(Point p){
        ++probes;
        const Zone zone=p.z<0?Zone::Negative:p.z==0?Zone::Zero:p.z<=F?Zone::Full:p.z>=H?Zone::Flat:Zone::Fade;
        const int32_t zl=zone==Zone::Negative?INT32_MIN:zone==Zone::Zero?0:zone==Zone::Full?1:zone==Zone::Fade?F+1:H;
        const int32_t zh=zone==Zone::Negative?-1:zone==Zone::Zero?0:zone==Zone::Full?F:zone==Zone::Fade?H-1:INT32_MAX;
        if(zone==Zone::Negative||zone==Zone::Flat)return {zone,0,0,0,0,0,0,0,0,0,0,{INT32_MIN,INT32_MIN,zl},{INT32_MAX,INT32_MAX,zh}};
        auto x=matrixSpan(X_AXIS,p.x),y=matrixSpan(Y_AXIS,p.y);
        const int32_t b00=g_ZCompensationMatrix[x.low][y.low],b10=g_ZCompensationMatrix[x.high][y.low],
            b01=g_ZCompensationMatrix[x.low][y.high],b11=g_ZCompensationMatrix[x.high][y.high];
        const uint32_t sx=std::max(distance(b00,b10),distance(b01,b11)),sy=std::max(distance(b00,b01),distance(b10,b11));
        auto bounds=[](MatrixSpan s,uint8_t axis,int32_t p,int32_t& lo,int32_t& hi){
            if(s.width){lo=p-s.distance+1;hi=p-s.distance+s.width;return;}
            const int32_t coordinate=int32_t((axis==X_AXIS?g_ZCompensationMatrix[s.low][0]:g_ZCompensationMatrix[0][s.low])*Printer::stepsPerMm(axis));
            if(p<=coordinate){lo=INT32_MIN;hi=coordinate;}else{lo=coordinate+1;hi=INT32_MAX;}
        };
        Point min{0,0,zl},max{0,0,zh};bounds(x,X_AXIS,p.x,min.x,max.x);bounds(y,Y_AXIS,p.y,min.y,max.y);
        return {zone,x.low,x.high,y.low,y.high,uint32_t(x.width),uint32_t(y.width),
                zone==Zone::Fade?uint32_t(H-F):0,sx,sy,distance(b00+b11,b01+b10),min,max};
    }
    int32_t correction(Point p){
        ++corrections;
        const BedCompensation::Geometry g{p.z,F,H,0,F,0};
        return BedCompensation::calculate(g,[&]{return int32_t(getZMatrixDepth(p.x,p.y));}).zCorrectionSteps;
    }
};
static int32_t roundChord(int32_t a,int32_t b,uint32_t i,uint32_t n){
    const int64_t v=2*(int64_t(a)*(n-i)+int64_t(b)*i)+n,d=2*int64_t(n);
    return int32_t(v>=0?v/d:-((-v+d-1)/d));
}
static void inverseReplay(){
    uint64_t checks=0;
    for(int32_t a=-8;a<=8;++a)for(int32_t b=-8;b<=8;++b)
      for(uint32_t n=std::max(1U,distance(a,b));n<=48;++n){
        Path p{{a,0,0},{b,0,0},n};
        for(int32_t lo=-9;lo<=9;++lo)for(int32_t hi=lo;hi<=9;++hi){
            uint32_t first=n+1,last=0;
            for(uint32_t i=0;i<=n;++i){const int32_t x=p.axis(a,b,i);if(x>=lo&&x<=hi){first=std::min(first,i);last=i;}}
            if(first<=n){assert(p.lastInside(a,b,lo,hi)==last);++checks;}
        }
      }
    Path big{{INT32_MIN,0,0},{-1,0,0},INT32_MAX};
    assert(big.valid()&&big.axis(INT32_MIN,-1,65536)==INT32_MIN+65536);
    assert(big.lastInside(INT32_MIN,-1,INT32_MIN,-65536)==uint32_t(INT32_MAX)-65535);
    printf("Z integer boundary inverse checked=%llu PASS\n",(unsigned long long)checks);
}
int main(){
    inverseReplay();
    const Point pts[]={{0,0,F-1},{3048,3048,F},{6095,3048,F+1},{3048,6095,H-1},
        {6095,6095,H},{9142,9142,H+1},{0,9142,0},{9142,0,1},{3048,3048,-1},{3048,3048,H+500}};
    uint64_t samples=0,chords=0,probes=0;uint32_t shortChords=0,zero=0,maxChords=0,maxError=0,maxProbes=0;
    unsigned paths=0;
    auto trial=[&](Point a,Point b,uint32_t n){
        if(!n)return;
        Path path{a,b,n};Preparer prep;Reader r;assert(prep.start(path));
        int32_t previous=a.z+r.correction(a);Prepared c{};uint32_t i=0,pc=0;
        PhysicalHandoff h;
        while(i<n){
            const uint32_t prior=r.probes;
            assert(prep.next(r,c));assert(c.first==i&&c.length&&c.motorStart==previous);
            const Tag tag{uint32_t(paths+1),i,1};
            assert(h.offer(tag,c.motorStart,c.motorEnd,c.length));
            assert(h.take(tag,previous)==PhysicalHandoff::Take::Accepted);
            maxProbes=std::max(maxProbes,r.probes-prior);
            shortChords+=c.length<=4;zero+=c.zeroTransition;
            for(uint32_t j=1;j<=c.length;++j){
                const auto p=path.at(i+j);
                assert(h.advance());
                const int32_t got=h.value();
                assert(got==roundChord(c.motorStart,c.motorEnd,j,c.length));
                const int32_t want=p.z+r.correction(p);
                maxError=std::max(maxError,distance(got,want));
                if(j==c.length)assert(got==want);
                previous=got;++samples;
            }
            assert(!h.remaining()&&!h.advance());
            i+=c.length;++pc;++chords;
        }
        assert(!prep.next(r,c));assert(previous==b.z+r.correction(b));
        probes+=r.probes;maxChords=std::max(maxChords,pc);++paths;
    };
    for(auto a:pts)for(auto b:pts)trial(a,b,std::max({distance(a.x,b.x),distance(a.y,b.y),distance(a.z,b.z)}));
    trial({0,0,-1},{9142,9142,1},9142);trial({9142,9142,1},{0,0,-1},9142);
    trial({3048,3048,F},{3048,3048,F},100000);
    trial({3048,3048,F},{3048,3048,256000},255744);
    trial({0,3048,200},{3048,3048,10},10000);
    printf("Z foreground preparer paths=%u samples=%llu chords=%llu max_chords=%u short_le4=%u zero_transitions=%u max_region_probes=%u total_region_probes=%llu max_error_steps=%u\n",
        paths,(unsigned long long)samples,(unsigned long long)chords,maxChords,shortChords,zero,maxProbes,(unsigned long long)probes,maxError);
    assert(maxError<=4);
}

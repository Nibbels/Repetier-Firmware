// Actual firmware geometry/layer code, synthetic scanned bed and paths.
// No timing/physical proof. Host long is 64-bit; keep all products in
// signed AVR32 range for the matrix and fade fixtures.
#include <cstdint>
#include <cstdio>
#include <cassert>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <vector>
#include "BedCompensation.h"
#include "PrintLayers.h"
enum {X_AXIS,Y_AXIS};
namespace Printer {float stepsPerMm(uint8_t){return 152.38f;}}
static short g_ZCompensationMatrix[5][5]={
    {0,0,20,40,60},
    {0,-80,-280,-90,-320},
    {20,-310,-30,-390,-40},
    {40,-60,-420,-20,-240},
    {60,-220,-50,-300,0}
};
static uint8_t g_uZMatrixMax[2]={4,4};
#include "BedMatrixInterpolation.h"

struct XYZ {int32_t x,y,z;};
static constexpr int32_t start=256,end=8656,top=0; // 0.2mm, relief420*20
static int32_t correction(XYZ p){
    const BedCompensation::Geometry g{p.z,start,end,0,start,top};
    return BedCompensation::calculate(g,[&]{return int32_t(getZMatrixDepth(p.x,p.y));}).zCorrectionSteps;
}
static bool same(XYZ a,XYZ b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
static XYZ point(XYZ a,XYZ b,int i,int n) {
    // Absolute integer endpoints; symmetric nearest rounding for the
    // mathematical sampled path, not a claim about exact Bresenham order.
    auto at=[&](int32_t x,int32_t y){
        const int64_t t=int64_t(x)*(n-i)+int64_t(y)*i;
        return int32_t(t>=0?(t+n/2)/n:-((-t+n/2)/n));
    };
    return {at(a.x,b.x),at(a.y,b.y),at(a.z,b.z)};
}
static int32_t chord(int32_t a,int32_t b,int i,int n){
    return point({a,0,0},{b,0,0},i,n).x;
}
static uint64_t comparisons=0,netOpposite=0,cancelled=0,closures=0;
static int maxChord=0,maxLag=0,maxMixed=0;
static int32_t physical(XYZ p){return p.z+correction(p);}
static void trajectory(XYZ a,XYZ b,int n){
    int32_t pa=physical(a),pc=correction(a);
    const int32_t origin=pa;
    XYZ prev=a;
    for(int i=1;i<=n;i++){
        XYZ p=point(a,b,i,n);
        assert(same(p,point(b,a,n-i,n)));
        const int32_t c=correction(p),dn=p.z-prev.z,dc=c-pc;
        const int32_t net=dn+dc;
        if(dn&&net&&((dn>0)!=(net>0)))++netOpposite;
        if(dn&&dc&&((dn>0)!=(dc>0)))cancelled+=std::min(std::abs(dn),std::abs(dc));
        pa+=net;assert(pa==physical(p));
        maxChord=std::max(maxChord,std::abs(c-chord(correction(a),correction(b),i,n)));
        // A 16ms age at 100mm/s: ~244 XY steps. Model a sample age
        // in path samples, not actual firmware ISR/publication latency.
        const XYZ old=point(a,b,std::max(0,i-244),n);
        maxLag=std::max(maxLag,std::abs(c-correction(old)));
        // Source currently reads X/Y/Z separately. Construct an allowed
        // mixed-time snapshot; not a claim this exact interleave occurs.
        maxMixed=std::max(maxMixed,std::abs(c-correction({old.x,p.y,old.z})));
        pc=c;prev=p;++comparisons;
    }
    for(int i=n-1;i>=0;i--){
        const XYZ p=point(a,b,i,n);
        const int32_t c=correction(p);
        pa+=(p.z-prev.z)+(c-pc);assert(pa==physical(p));
        prev=p;pc=c;++comparisons;
    }
    assert(pa==origin&&same(prev,a));++closures;
}
int main(){
    // Same exact coordinates and fixed matrix/layer context -> same target.
    // Pure Z through both zones; XY wipe; diagonal XYZ; reverse along
    // another path via rectangle (only endpoint closure required).
    const XYZ pts[]={{0,0,start-1},{3048,3048,start},{6095,3048,start+1},
        {3048,6095,end-1},{6095,6095,end},{9142,9142,end+1},
        {0,9142,0},{9142,0,1},{3048,3048,-1},{3048,3048,end+500}};
    for(auto a:pts)for(auto b:pts)trajectory(a,b,1024);
    // Construct a small negative nominal dz with a positive geometric
    // correction large enough to reverse the net physical displacement.
    trajectory({0,3048,200},{3048,3048,190},1024);
    // Actual PrintLayers semantics. Return before positive E = normal hop;
    // positive E at elevated Z can confirm hop height as a layer, even
    // if it is only an unretract. Keep this explicit as policy evidence.
    PrintLayers layers(1280);
    layers.recordPlannedMove({256,true,true,true},1280,0,-420);
    assert(layers.currentLayerSteps()==256);
    layers.recordPlannedMove({640,true,false,false},1280,0,-420);
    layers.recordPlannedMove({640,false,true,false},1280,0,-420);
    layers.recordPlannedMove({256,true,false,false},1280,0,-420);
    layers.recordPlannedMove({256,false,false,true},1280,0,-420);
    assert(layers.currentLayerSteps()==256);
    layers.recordPlannedMove({640,true,false,false},1280,0,-420);
    layers.recordPlannedMove({640,false,false,true},1280,0,-420);
    assert(layers.currentLayerSteps()==640); // real existing policy
    std::printf("Z geometry actual source closures=%llu samples=%llu opposite_net=%llu cancelled=%llu max_endpoint_chord_error=%d max_sample_age_error=%d max_mixed_snapshot_error=%d elevated_E_confirms_hop=1\n",
        (unsigned long long)closures,(unsigned long long)comparisons,(unsigned long long)netOpposite,
        (unsigned long long)cancelled,maxChord,maxLag,maxMixed);
    assert(netOpposite&&cancelled&&maxChord&&maxLag&&maxMixed);
}

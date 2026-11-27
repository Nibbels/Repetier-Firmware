#include "layer-retraction-debt.h"
#include "PrintLayers.h"
#include "ExtrusionPlanner.h"
#include "MoveTarget.h"
#include <cassert>
#include <cstdio>
#include <cmath>

struct Fixture {
    PrintLayers legacy{1280.0f},candidate{1280.0f};
    ExtrusionPlanner extrusion;
    MoveTarget target;
    LayerRetractionResearch::Debt debt;
    float flow=1;
    int32_t z=256;
    static constexpr uint8_t E=3;
    void move(int32_t nextZ,bool xy,float e,bool relative=true){
        const bool movesZ=nextZ!=z;
        const float old=target.machineMM(E),rest=target.remainderMM(E);
        target.setFromGCodeMM(E,e,relative);
        float delta=(target.machineMM(E)-old)+(target.remainderMM(E)-rest);
        const bool beyond=debt.observe(delta);
        const int32_t steps=extrusion.planSteps({target.machineMM(E),target.remainderMM(E)},flow,280);
        z=nextZ;
        legacy.recordPlannedMove({z,movesZ,xy,steps>0},1280,0,-420);
        // This is a conservative classification candidate, not an adopted
        // rule. Positive new extrusion without XY cannot establish a layer.
        candidate.recordPlannedMove({z,movesZ,xy,beyond&&xy&&steps>0},1280,0,-420);
    }
    void g92(float e){
        target.setMachineMM(E,e);
        extrusion.setLogicalPosition({e,0});
    }
};
int main(){
    Fixture f;
    f.move(256,true,1);
    assert(f.legacy.currentLayerSteps()==256&&f.candidate.currentLayerSteps()==256);
    f.move(256,false,-2);
    f.move(640,true,0);
    f.move(640,true,1); // partial unretract while wiping at hop height
    assert(f.legacy.currentLayerSteps()==640&&f.candidate.currentLayerSteps()==256);
    f.g92(100);
    f.flow=1.5f; // change physical base-step conversion, not logical debt
    f.move(640,false,101,false);
    assert(f.debt.owedMM()==0&&f.candidate.currentLayerSteps()==256);
    f.move(256,false,0);
    f.move(256,true,.125f);
    assert(f.candidate.currentLayerSteps()==256);
    f.move(512,true,.125f); // true rising deposition incl vase-like move
    assert(f.candidate.currentLayerSteps()==512);
    f.move(512,false,-1);
    f.move(900,true,0);
    f.move(900,true,1.25f); // recovery plus real new extrusion in one move
    assert(f.candidate.currentLayerSteps()==900);
    Fixture pure;
    pure.move(256,true,1);
    pure.move(640,false,0);
    pure.move(640,false,.25f); // not proof of printing
    assert(pure.legacy.currentLayerSteps()==640&&pure.candidate.currentLayerSteps()==256);
    pure.move(640,true,.125f);
    assert(pure.candidate.currentLayerSteps()==640);
    // Exact binary32 dyadic input reference, including split recovery,
    // retraction sequences and separate tools. No invented tolerance.
    LayerRetractionResearch::Debt d[2];
    int32_t ref[2]={0,0};
    uint32_t rng=0x9e3779b9;unsigned checked=0;
    for(unsigned i=0;i<200000;++i){
        rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;
        unsigned tool=rng&1;int32_t delta=int32_t((rng>>1)%513)-256;
        bool dep=delta>ref[tool];
        ref[tool]-=delta;if(ref[tool]<0)ref[tool]=0;
        assert(d[tool].observe(delta/1024.0f)==dep);
        assert(d[tool].owedMM()==ref[tool]/1024.0f);
        ++checked;
    }
    // A real planner can round a logical recovery differently after flow
    // change; base-step debt is not a correct proxy for slicer intent.
    ExtrusionPlanner p;
    const auto retract=p.planSteps({-2,0},1,280);
    const auto unretract=p.planSteps({0,0},1.5f,280);
    assert(retract==-560&&unretract==840);
    printf("Layer retract logical reference cases=%u, legacy premature hop confirmed, candidate preserved layer, base-step recovery delta=%ld PASS\n",
        checked,(long)(retract+unretract));
}

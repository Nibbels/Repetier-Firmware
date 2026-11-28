#pragma once
namespace LayerRetractionResearch {
// Research classification only, not a physical extrusion/Advance ledger.
// Observe logical E delta BEFORE flow conversion. G92 changes the E
// coordinate origin, not debt. Track separately for each fitted tool.
// Reset only on an explicit reference/print lifecycle decision.
class Debt {
    float owed_=0.0f,remainder_=0.0f;
public:
    bool observe(float logicalDeltaMM) {
        const float delta=-logicalDeltaMM;
        const float sum=owed_+delta;
        const float added=sum-owed_;
        const float error=(owed_-(sum-added))+(delta-added);
        const float rest=remainder_+error;
        const float corrected=sum+rest;
        const float low=rest-(corrected-sum);
        if(corrected<0.0f || (corrected==0.0f&&low<0.0f)) {
            owed_=remainder_=0.0f;return true;
        }
        owed_=corrected;remainder_=low;return false;
    }
    void reset(){owed_=remainder_=0.0f;}
    float owedMM() const {return owed_+remainder_;}
};
}

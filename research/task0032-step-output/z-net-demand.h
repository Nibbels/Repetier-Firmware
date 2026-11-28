#pragma once
#include <stdint.h>
namespace ZNetResearch {
// One normal referenced Z decision. The caller supplies authoritative
// eligibility (endstop, reference, wake, ownership) and a physically legal
// pulse opportunity (direction, speed, acceleration, timing). Never interpret
// logical path direction from GPIO. Stop/suppression/scan/reference policies
// belong to the existing ZMotion owner, not to this algebra.
struct Decision {
    int8_t nominal,correction,pulse;
};
inline Decision arbitrate(int8_t nominal,int8_t correction,
                          bool nominalAllowed,bool correctionAllowed,
                          bool cancellationAllowed,bool pulseAllowed) {
    // Cancel only a genuinely eligible opposing correction, and only in
    // normal referenced operation. No DIR write or ramp update for no pulse.
    if(nominal && correction && nominal==-correction &&
       cancellationAllowed && correctionAllowed)
        return {nominal,correction,0};
    // A due nominal request has priority. Same-sign correction stays debt
    // until a separate legal opportunity; never issue a double STEP.
    if(nominal)
        return nominalAllowed&&pulseAllowed?Decision{nominal,0,nominal}:Decision{0,0,0};
    if(correction && correctionAllowed && pulseAllowed)
        return {0,correction,correction};
    return {0,0,0};
}
}

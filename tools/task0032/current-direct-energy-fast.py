#!/usr/bin/env python3
"""Optimize the exact #228 ordinary-direct energy reference.

No additional static state or queue slot bytes. The formerly planner-only
union has room for a prepared exact cruise interval. One-step energy math
retains exact boundary speeds and a conservative ceil period. Anchor the
next compare after the closed logical pulse transaction, not its old compare.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

def once(s,old,new):
    assert s.count(old)==1,(old,s.count(old))
    return s.replace(old,new,1)

DIVISION=r'''
// Fixed 16-round restoring division of the constant 32000000 by a
// 16-bit denominator. For d>=489, initial remainder488<d and quotient
// fits in 16 bits. Carry out of the remainder forces subtraction.
static inline uint32_t ceilEnergyPeriod(uint16_t d) {
    if (d<489) return d ? (32000000UL+d-1)/d : 0;
    uint16_t q=18432,rem=488;
#ifdef __AVR__
    uint8_t rounds=16;
    asm volatile(
        "1: lsl %A[q]\n\t"
        "rol %B[q]\n\t"
        "rol %A[r]\n\t"
        "rol %B[r]\n\t"
        "brcs 2f\n\t"
        "cp %A[r],%A[d]\n\t"
        "cpc %B[r],%B[d]\n\t"
        "brlo 3f\n\t"
        "2: sub %A[r],%A[d]\n\t"
        "sbc %B[r],%B[d]\n\t"
        "ori %A[q],1\n\t"
        "3: dec %[n]\n\t"
        "brne 1b\n\t"
        : [q] "+&d"(q),[r] "+&r"(rem),[n] "+&r"(rounds)
        : [d] "r"(d)
        : "cc");
#else
    for(uint8_t n=0;n<16;++n) {
        uint32_t x=uint32_t(rem)*2+(q>>15);
        q=uint16_t(q<<1);
        if(x>=d){x-=d;q|=1;}
        rem=uint16_t(x);
    }
#endif
    return uint32_t(q)+uint32_t(rem!=0);
}
'''
ARM=r'''
uint32_t StepExecutor::armDirectStep(PrintLine& move) {
    auto& clock=stepClocks[FOR_DIRECT];
    const auto& limits=move.directProfile;
    if (!limits.valid || move.stepsRemaining<=0) return 0;
    const auto root=[](uint32_t energy) -> uint16_t {
        if (!energy) return 0;
        const uint16_t r=HAL::integerSqrt(static_cast<int32_t>(energy));
        return uint32_t(r)*r>energy ? uint16_t(r-1) : r;
    };
    const auto plan=[&](bool stopping) -> StopTailEnergy::Group {
        const uint32_t before=clock.ticks;
        uint32_t end=before;
        if (stopping) {
            if (!clock.steps || before<=limits.exitEnergy)
                return {before,0,clock.peak,0};
            const uint32_t diff=before-limits.exitEnergy;
            end=diff<=limits.drop?limits.exitEnergy:before-limits.drop;
        } else {
            // Derive both boundary checks from the next remaining-distance
            // cap: no duplicate multiplication of the braking horizon.
            const uint32_t cap=PreparedDirectProfile::cap(limits,uint32_t(move.stepsRemaining-1));
            if (before>limits.ceiling || (before>cap && before-cap>limits.drop))
                return {before,0,clock.peak,0};
            if(before>cap)end=cap;
            else {
                const uint32_t diff=cap-before;
                end=diff<=limits.drop?cap:before+limits.drop;
            }
        }
        // The current root is already exact. Endpoint roots are prepared
        // speed fields, so no expensive sqrt on plateaux or endpoints.
        const uint16_t nextRoot=end==before?clock.peak:
            end==limits.ceiling?move.stepProfile.vMax:
            end==limits.exitEnergy?move.stepProfile.vEnd:root(end);
        uint16_t denom=uint16_t(clock.peak+nextRoot);
        if(!denom) {
            // Same rest-to-rest triangular one-step case as the reference.
            denom=root(RMath::min(limits.ceiling,limits.drop/2));
            if(!denom)return {before,0,clock.peak,0};
        }
        const uint32_t ticks=before==limits.ceiling && end==before
            ? limits.plateauTicks : PreparedDirectProfile::ceilEnergyPeriod(denom);
        return {end,ticks,nextRoot,1};
    };
    const bool stopping=PrintLine::directStopping(move.task);
    auto group=plan(stopping);
    if (!stopping && PrintLine::directStopping(move.task)) group=plan(true);
    if (!group.steps || !group.ticks || group.ticks>0xffffffUL) return 0;
    clock.ticks=group.energy;
    clock.peak=group.root;
    return group.ticks<minimumStepIntervalTicks ? minimumStepIntervalTicks : group.ticks;
}
'''
def candidate(fw):
    profile=(fw/'PreparedDirectProfile.h').read_text()
    profile=once(profile,'namespace PreparedDirectProfile {','namespace PreparedDirectProfile {\n'+DIVISION)
    profile=once(profile,'    bool valid;','    bool valid;\n    uint32_t plateauTicks;')
    profile=once(profile,'return {ceiling,floor,0,0,false};','return {ceiling,floor,0,0,false,0};')
    profile=once(profile,'return {ceiling,floor,drop,diff/drop+uint32_t(diff%drop!=0),true};',
        'return {ceiling,floor,drop,diff/drop+uint32_t(diff%drop!=0),true,\n        (16000000UL+maximum-1)/maximum};')
    e=(fw/'StepExecutor.cpp').read_text()
    i=e.index('uint32_t StepExecutor::armDirectStep(')
    j=e.index('// Keep the ramp update',i)
    e=e[:i]+ARM+'\n'+e[j:]
    e=once(e,'''        directPrimarySteps = direct.delta[direct.primaryAxis];
        stepErrors[FOR_DIRECT].start(directPrimarySteps);
        stepClocks[FOR_DIRECT].start(direct.stepProfile.vStart);
        nominalFeedrate_ = direct.fullSpeed;''','''        // Publish the phase and the multibyte feedrate coherently; the
        // remaining clock/error initialization is exclusively Timer1-owned.
        nominalFeedrate_ = direct.fullSpeed;
        if (!guardedStartup) HAL::allowInterrupts();
        directPrimarySteps = direct.delta[direct.primaryAxis];
        stepErrors[FOR_DIRECT].start(directPrimarySteps);
        stepClocks[FOR_DIRECT].start(direct.stepProfile.vStart);''')
    e=once(e,'''    }                       // for loop
    HAL::allowInterrupts(); // Allow interrupts for other types, timer1 is still disabled

    if (energyDirect)''','''    }                       // for loop
    // TCNT1 uses AVR's shared temporary high-byte latch. Read it while
    // the pulse transaction is still closed, then do all ramp math open.
    // Requires the existing Timer1 service to finish before its temporary
    // 61000-tick compare wraps; global admission remains a separate gate.
    uint16_t directAnchor=0;
#ifdef __AVR__
    if (energyDirect) directAnchor=TCNT1;
#endif
    HAL::allowInterrupts(); // Allow interrupts for other types, timer1 is still disabled

    if (energyDirect)''')
    e=once(e,'''        return delay;
    }
    const speed_t v = nextStepSpeed''','''        // A late current pulse must never be repaid by a too-early next
        // physical pulse. The compare is relative to the old CTC epoch;
        // anchor after the last fall/ledger instead. Any next entry/edge
        // latency only increases the spacing. No second hardware clock.
        return delay+directAnchor;
    }
    const speed_t v = nextStepSpeed''')
    return {'PreparedDirectProfile.h':profile,'StepExecutor.cpp':e}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();fw=a.source.resolve();out=a.out.resolve()
    assert fw!=out and fw not in out.parents
    files=candidate(fw);out.mkdir(parents=True,exist_ok=True)
    for name,s in files.items():(out/name).write_text(s)
    for name in ('StopTailEnergy.h','StepExecutor.h','motion.h','motion.cpp'):
        (out/name).write_bytes((fw/name).read_bytes())
    (out/'provenance.json').write_text(json.dumps({
        'input_sha256':{name:hashlib.sha256((fw/name).read_bytes()).hexdigest() for name in files},
        'candidate_sha256':{name:hashlib.sha256(s.encode()).hexdigest() for name,s in files.items()},
        'scope':'exact one-step math and conservative physical completion anchor; not final packing or nested timing proof'},indent=2)+'\n')
    # Exhaust all legal 16-bit denominators; test the portable same-round
    # implementation and exact prepared plateau at every legal rate.
    cpp=out/'division-replay.cpp'
    cpp.write_text(r'''
#include "PreparedDirectProfile.h"
#include <cassert>
#include <cstdio>
int main(){
    using namespace PreparedDirectProfile;
    for(uint32_t d=1;d<=62500;++d){
        uint32_t p=ceilEnergyPeriod(uint16_t(d));
        assert(uint64_t(p)*d>=32000000ULL);
        assert(uint64_t(p-1)*d<32000000ULL);
    }
    for(uint16_t v=1;v<=31250;++v){
        auto lim=prepare(v,0,128000);
        assert(lim.valid && lim.plateauTicks==ceilEnergyPeriod(uint16_t(2*v)));
    }
    puts("62500 exact ceil divisions and 31250 prepared plateau intervals PASS");
}
''')
    with tempfile.TemporaryDirectory() as d:
        exe=Path(d)/'div'
        subprocess.run(['c++','-std=c++11','-O2','-Wall','-Wextra','-Werror','-fsanitize=undefined',
            '-fno-sanitize-recover=all','-I',str(out),str(cpp),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)

if __name__=='__main__':main()

#!/usr/bin/env python3
"""Phase-aware ordinary direct STEP scheduling on the qualified #235 tree.

Anchor at the first coherent sample after the primary-axis rising edge
(or after the logical transaction when no physical primary pulse occurs).
Subtract only the independently checked unconditional next-vector prefix.
The existing late-compare clamp may extend but never shorten that delay.
No persistent state and no change to the E/Advance scheduling path.
"""
import argparse
import hashlib
import json
from pathlib import Path

PREFIX=51  # unconditional instruction cycles in all four #235 vector prologues

def once(s,old,new):
    assert s.count(old)==1,(old,s.count(old))
    return s.replace(old,new,1)

def candidate(fw):
    e=(fw/'StepExecutor.cpp').read_text()
    e=once(e,'    fast8_t completed = 0;\n','    fast8_t completed = 0;\n    uint16_t primaryAnchor=0;\n')
    for axis in ('X','Y'):
        old=f'''                    Printer::start{axis}Step(dir);
                }}
                HAL::forbidInterrupts();

                if (forQueue)'''
        new=f'''                    Printer::start{axis}Step(dir);
                }}
                HAL::forbidInterrupts();
#ifdef __AVR__
                // Sample after the physical edge and its motor ledger.
                // A nested IRQ before this sample can only delay the next edge.
                if (energyDirect && move->primaryAxis=={axis}_AXIS) primaryAnchor=TCNT1;
#endif

                if (forQueue)'''
        e=once(e,old,new)
    e=once(e,'''                const bool continuing = Printer::zMotion.stepPath(path, move->isZPositiveMove() ? 1 : -1);
                if (!continuing)''','''                const bool continuing = Printer::zMotion.stepPath(path, move->isZPositiveMove() ? 1 : -1);
#ifdef __AVR__
                if (energyDirect && move->primaryAxis==Z_AXIS) primaryAnchor=TCNT1;
#endif
                if (!continuing)''')
    e=once(e,'''    if (energyDirect) directAnchor=TCNT1;''','''    if (energyDirect) directAnchor=primaryAnchor?primaryAnchor:TCNT1;''')
    e=once(e,'''    uint16_t primaryAnchor=0;''','''#ifdef __AVR__
    uint16_t primaryAnchor=0;
#endif''')
    e=once(e,'''        // A late current pulse must never be repaid by a too-early next
        // physical pulse. The compare is relative to the old CTC epoch;
        // anchor after the last fall/ledger instead. Any next entry/edge
        // latency only increases the spacing. No second hardware clock.
        return delay+directAnchor;''','''        // The next Timer1 rising edge cannot precede the unconditional
        // interrupt prologue. A linked check on all four final images must
        // establish at least this many cycles before any branch or GPIO.
        // Never repay lateness beyond that proven minimum. The existing
        // compare clamp adds time if ramp calculation has overrun.
        // E/Advance has a different physical owner, retain its old anchor.
#ifdef __AVR__
        constexpr uint16_t primaryEntryPrefix = 51;
        static_assert(minimumStepIntervalTicks > primaryEntryPrefix, "positive delay");
        if (move->primaryAxis<=Z_AXIS) return delay+directAnchor-primaryEntryPrefix;
#endif
        return delay+directAnchor;''')
    return e

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();fw=a.source.resolve();out=a.out.resolve()
    assert fw!=out and fw not in out.parents
    out.mkdir(parents=True,exist_ok=True)
    old=(fw/'StepExecutor.cpp').read_bytes();new=candidate(fw).encode()
    (out/'StepExecutor.cpp').write_bytes(new)
    (out/'provenance.json').write_text(json.dumps({
        'input_sha256':hashlib.sha256(old).hexdigest(),
        'candidate_sha256':hashlib.sha256(new).hexdigest(),
        'required_entry_prefix_cycles':PREFIX,
        'scope':'ordinary direct primary edge phase; E/Advance and global Z scheduler unchanged'},indent=2)+'\n')

if __name__=='__main__':main()

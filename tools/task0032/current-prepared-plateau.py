#!/usr/bin/env python3
"""Bypass generic direct-ramp planning only on an immutable cruise plateau.

No cached interval is added. The existing prepared profile supplies it.
The stop byte is read last: an IRQ after that read can leave at most the
already armed next pulse, as in the qualified generic path.
"""
import argparse
from pathlib import Path

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--root",type=Path,required=True)
    a=p.parse_args()
    f=a.root/"Repetier/StepExecutor.cpp"
    s=f.read_text()
    old="        const uint32_t delay=move->isNoMove() || move->stepsRemaining<=0 ? 0 : armDirectStep(*move);"
    new="""        uint32_t delay=0;
        if (!move->isNoMove() && move->stepsRemaining>0) {
            const auto& limits=move->directProfile;
            // Both the current and next distance-domain boundary are
            // exactly at the prepared ceiling. No root, divide, horizon
            // multiplication or generic planner call on this path.
            if (limits.valid && clock.ticks==limits.ceiling &&
                uint32_t(move->stepsRemaining)>limits.horizon &&
                limits.plateauTicks && limits.plateauTicks<=0xffffffUL &&
                !PrintLine::directStopping(move->task)) {
                delay=limits.plateauTicks<minimumStepIntervalTicks
                    ? minimumStepIntervalTicks : limits.plateauTicks;
            } else {
                delay=armDirectStep(*move);
            }
        }"""
    assert s.count(old)==1
    f.write_text(s.replace(old,new))
    print("prepared plateau execution bypass; original slow path and stop gate retained")

if __name__=="__main__":
    main()

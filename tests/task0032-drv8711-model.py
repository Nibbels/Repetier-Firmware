#!/usr/bin/env python3
"""Independent protocol/port-race model; no firmware or hardware execution.
Run: python3 tests/task0032-drv8711-model.py
Candidate instruction costs are assumed, not measured linked code.
"""
from math import log

def candidate(word):
    # SCS already high; first bit begins after two cycles.
    t, data, events = 2, 0, [(0, "cs", 1)]
    boundaries = [0, t]
    for i in range(15, -1, -1):
        bit = (word >> i) & 1
        # SBRS/CBI/SBRC/SBI: balanced five cycles;
        # actual write at cycle 3 for zero or 5 for one.
        events.append((t + (5 if bit else 3), "data", bit))
        boundaries.extend(t + d for d in ((1, 3, 5) if not bit else (2, 3, 5)))
        t += 5
        t += 2
        events.append((t, "clk", 1))
        boundaries.append(t)
        t += 2
        boundaries.extend((t-1,t))
        t += 2
        events.append((t, "clk", 0))
        boundaries.append(t)
        # LSL, ROL, DEC, BRNE (last not taken)
        boundaries.extend((t+1,t+2,t+3))
        t += 5 if i else 4
        boundaries.append(t)
    events.append((t+2,"cs",0))
    return events, boundaries, t-2

def validate(events, word):
    cs = clk = data = 0
    cs_on = last_data = last_rise = last_fall = -1000
    received = []
    for t, signal, value in events:
        if signal == "cs":
            assert not clk
            if value:
                cs_on = t
            else:
                assert t-last_fall >= 1
            cs = value
        elif signal == "data":
            assert not clk
            if last_rise >= cs_on:
                assert t-last_rise >= 1
            data, last_data = value, t
        else:
            assert cs
            if value:
                assert not clk
                assert t-cs_on >= 1 and t-last_data >= 1
                assert t-last_rise >= 4 and t-last_fall >= 1
                last_rise = t
                received.append(data)
            else:
                assert clk and t-last_rise >= 1
                last_fall = t
            clk = value
    assert cs == clk == 0
    assert len(received) == 16
    assert received == [(word >> i)&1 for i in range(15,-1,-1)]

def port_race():
    # Model actual extended I/O RMW: foreground CS update and ISR STEP
    # edge between snapshot and store. IRQs cannot change other bits
    # of the saved snapshot. With a guard, edge occurs before or after.
    cs, step = 1, 8
    for initial in range(256):
        for select in (False, True):
            update = lambda p: (p|cs) if select else (p&~cs)
            snapshot = initial
            expected = update(initial ^ step)
            stale = update(snapshot)
            assert stale ^ expected == step
            assert update(initial ^ step) == (update(initial) ^ step)

def rl_time_us(v, emf, r, l, i0, i1):
    limit = (v-emf)/r
    if limit <= i1:
        return None
    return 1e6*l/r*log((limit-i0)/(limit-i1))

def main():
    port_race()
    for word in range(65536):
        events, boundaries, cycles = candidate(word)
        assert cycles == 255
        validate(events, word)
    injected = 0
    for torque in range(256):
        word = 0x1100 | torque
        events, boundaries, cycles = candidate(word)
        for boundary in set(boundaries):
            for delay in (1,80,594,1928):
                # IRQ starts after the instruction completing at boundary.
                # No ISR writes CS/clock/data: this is a prerequisite.
                validate([(t+(delay if t>boundary else 0),s,v)
                          for t,s,v in events], word)
                injected += 1
    print("DRV8711 abstract protocol: 65536 words,",injected,"injected preemptions; RMW counterexample 512 states")
    for emf in (0,12,20,22):
        print("Ideal RL, 24 V, 2 ohm, 4 mH, 1.5 -> 2 A:",emf,rl_time_us(24,emf,2,.004,1.5,2))
    print("Scope: not AVR execution, ISR ownership/deadlines or physical validation")

if __name__ == "__main__":
    main()

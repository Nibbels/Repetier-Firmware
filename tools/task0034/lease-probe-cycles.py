#!/usr/bin/env python3
"""Conservative control-flow bounds for the pinned Task0034 standalone AVR probe.

Extract host/heater-lease-avr-probe.txt from artifact 15073781932.
This is not a general analyzer or a production-firmware WCET proof.
"""
import re
import functools
import sys
import hashlib
from pathlib import Path

data = Path(sys.argv[1]).read_bytes()
assert hashlib.sha256(data).hexdigest() == "bc8cd5a5edf8afe0827d1d4869579ebadc3cf4a35bccfc6d354106dba6926dbc"
ins = {}
for line in data.decode().splitlines():
    m = re.match(r"^\s*([\da-f]+):\s+((?:[\da-f]{2}\s+)+)\s*([a-z][a-z0-9]*)\s*(.*)", line)
    if m:
        ins[int(m[1], 16)] = (len(m[2].split()), m[3], m[4])

# Reviewed loop bounds: 0..2 channel, 3 iterations of array loops.
# Inner shift loops in fail() conservatively allow 6 taken branches each.
limits = {0x148: 2, 0x16a: 2, 0x198: 6, 0x1b0: 6, 0x1c0: 2,
          0x1f8: 2, 0x24e: 2, 0x452: 2, 0x47c: 2}
keys = tuple(limits)
empty = (0,) * len(keys)
one = set("adc add and cli com cpc cpi dec eor in ldi mov movw or out sbc sbci sub subi".split())
two = set("adiw ld ldd lds pop push st std sts".split())

def dest(arg):
    m = re.search(r";\s*0x([0-9a-f]+)", arg) or re.search(r"^0x([0-9a-f]+)", arg)
    assert m, arg
    return int(m[1], 16)

@functools.lru_cache(None)
def walk(pc, counts, stop=None, healthy=False):
    if pc == stop:
        return 0
    if healthy and pc == 0x176:  # fail()
        return -10**8
    n, op, arg = ins[pc]
    nxt = pc + n
    if op in ("ret", "reti"):
        assert stop is None
        return 5  # ATmega2560 three-byte PC
    if op == "call":
        return 5 + walk(dest(arg), empty, None, healthy) + walk(nxt, counts, stop, healthy)
    if op == "rjmp":
        return 2 + walk(dest(arg), counts, stop, healthy)
    if op.startswith("br"):
        target = dest(arg)
        opts = [1 + walk(nxt, counts, stop, healthy)]
        if pc in limits:
            i = keys.index(pc)
            if counts[i] < limits[pc]:
                new = list(counts)
                new[i] += 1
                opts.append(2 + walk(target, tuple(new), stop, healthy))
        else:
            opts.append(2 + walk(target, counts, stop, healthy))
        return max(opts)
    if op == "cpse":
        nn = ins[nxt][0]
        return max(1 + walk(nxt, counts, stop, healthy),
                   (2 if nn == 2 else 3) + walk(nxt + nn, counts, stop, healthy))
    assert op in one | two, (pc, op)
    return (1 if op in one else 2) + walk(nxt, counts, stop, healthy)

for name, a, b in [
    ("grant masked", 0x3c8, 0x3de), ("frame masked", 0x360, 0x372),
    ("expiry ISR body", 0x254, None), ("live", 0x1c4, None),
    ("checkAll", 0x20e, None), ("fail", 0x176, None),
]:
    cycles = walk(a, empty, b)
    normal = walk(a, empty, b, True)
    print(name, cycles, "cycles", cycles / 16, "us; no-fail",
          normal if normal >= 0 else None)

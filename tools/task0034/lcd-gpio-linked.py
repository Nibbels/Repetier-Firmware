#!/usr/bin/env python3
"""Audit direct shared LCD port RMW windows in archived standard + combined ELFs.

Usage: python3 tools/task0034/lcd-gpio-linked.py firmware-evidence.zip
Scope: direct LDS/STS port accesses in the named LCD routines. Reject
unrecognized instructions within the guards; do not infer global IRQ WCET
or physical pulse timing. Guard cycles include CLI through restored SREG.
"""
import json
import re
import sys
import zipfile

PORTS = {"0x0108", "0x010b"}  # PORTK, PORTL
COST = {"lds": 2, "sts": 2, "andi": 1, "ori": 1, "and": 1,
        "or": 1, "mov": 1, "tst": 1, "ldi": 1, "cp": 1,
        "cpi": 1, "nop": 1}
NAMES = ("lcdWriteByte(", "lcdWriteNibble(", "UIDisplay::initializeLCD(")

def groups(text):
    result = {}
    name = None
    for line in text.splitlines():
        m = re.match(r"^([0-9a-f]+) <(.*)>:$", line)
        if m:
            name = m[2]
            result[name] = []
        m = re.match(r"^\s*([0-9a-f]+):\s+((?:[0-9a-f]{2} )+)\s*(\w+)\s*([^;]*)(?:;(.*))?$", line)
        if m and name:
            result[name].append((int(m[1], 16), len(m[2].split()), m[3],
                                 m[4].strip().lower(), m[5] or ""))
    return result

def audit(name, ops):
    shared = {a for a, _, op, arg, _ in ops
              if op == "sts" and arg.split(",")[0] in PORTS}
    assert not any(op in ("st", "std", "xch", "lac", "las", "lat")
                   for _, _, op, _, _ in ops), (name, "indirect store requires review")
    covered, times = set(), []
    lookup = {a: (size, op, arg, comment) for a, size, op, arg, comment in ops}
    guards = 0

    for idx, (a, size, op, arg, _) in enumerate(ops):
        if op != "cli":
            continue
        guards += 1
        previous = ops[idx-1]
        assert previous[2] == "in" and previous[3].endswith(", 0x3f"), previous
        saved = previous[3].split(",")[0]

        def walk(pc, cycles, seen, load=None, store=None):
            assert pc not in seen, (name, hex(pc), "loop inside guard")
            seen = seen | {pc}
            width, mnemonic, operands, comment = lookup[pc]
            nxt = pc + width
            if mnemonic == "out":
                assert operands == "0x3f, " + saved, (name, hex(pc), operands)
                assert load and store and load == store[0], (name, hex(pc), load, store)
                times.append(cycles + 1)
                covered.add(store[1])
                return
            if mnemonic in ("rjmp", "jmp") or mnemonic.startswith("br"):
                target = int(re.search(r"0x([0-9a-f]+)", comment, re.I)[1], 16)
                if mnemonic.startswith("br"):
                    walk(nxt, cycles+1, seen, load, store)
                    walk(target, cycles+2, seen, load, store)
                else:
                    walk(target, cycles+(2 if mnemonic=="rjmp" else 3), seen, load, store)
                return
            if mnemonic in ("sbrs", "sbrc", "cpse"):
                skip = lookup[nxt][0]
                walk(nxt, cycles+1, seen, load, store)
                walk(nxt+skip, cycles+1+skip//2, seen, load, store)
                return
            if mnemonic == "lds":
                assert not load and operands.split(", ")[1] in PORTS, (name, operands)
                load = operands.split(", ")[1]
            if mnemonic == "sts":
                assert not store and pc in shared, (name, operands)
                store = (operands.split(", ")[0], pc)
            walk(nxt, cycles+COST[mnemonic], seen, load, store)

        walk(a+size, 1, set())

    assert covered == shared and shared and guards, (name, covered, shared)
    return guards, len(shared), len(times), min(times), max(times)

def main(path):
    with zipfile.ZipFile(path) as archive:
        summary = json.loads(archive.read("summary.json"))
        assert not summary["failures"], "candidate failed canonical CI"
        print("tested merge", summary["sha"])
        for prefix in ("", "host/experimental-avr/emergency/"):
            for model in ("RF1000", "RF2000", "RF2000v2", "RF2000-X44"):
                print(prefix+model)
                found = set()
                for name, ops in groups(archive.read(prefix+model+"/disassembly.txt").decode()).items():
                    if not name.startswith(NAMES):
                        continue
                    key = name.split("(")[0]
                    found.add(key)
                    n, stores, paths, lo, hi = audit(name, ops)
                    print(f"  {key}: {n} guards, {stores} stores, {paths} paths, {lo}..{hi} cycles")
                assert found == {x.split("(")[0] for x in NAMES}, found

if __name__ == "__main__":
    main(sys.argv[1])

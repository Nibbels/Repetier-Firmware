#!/usr/bin/env python3
"""A73 linked IRQ stack walk; pinned to explicitly reviewed CI artifacts.

All conditional branches are explored conservatively. AVR return addresses
are three bytes. Follow actual instructions, not symbol-name call graphs
(which confuse flash tables and library aliases with executable paths).
The digitalWrite switch table is resolved through direct or trampoline targets.
Unsupported SP writes, indirect transfers, recursive calls or incomplete paths
fail this audit. This does not model elapsed cycles, foreground frames, IRQ
preemption timing or prove register-value invariants. Requalify a new image.
"""
import hashlib
import re
import sys
import zipfile

PINS = {
    "b65565e9af997dfa6ef886c7e76e48f43caa2455aa77e05e8461a37b988c8d09": 15220305328,
    "c669fe33337522542e83190ccd6831baeec7bcc715f513dd9053589ae22b34a1": 15220648845,
    "234dea9336e8dcdd9f65212723236381a781c17bfc243f70d6b9ad96ff18ff52": 15222066305,
}
BOARDS = ("RF1000", "RF2000", "RF2000v2", "RF2000-X44")
VECTORS = (17, 21, 22, 23, 25, 26, 32)


def target(arg):
    m = re.search(r";\s*0x([0-9a-f]+)\s*<", arg) or re.match(r"0x([0-9a-f]+)", arg)
    if not m:
        raise ValueError(arg)
    return int(m[1], 16)


def audit(z, board):
    p = "host/experimental-avr/emergency/" + board + "/"
    s = z.read(p + "disassembly.txt").decode()
    labels = {m[2]: int(m[1], 16) for m in re.finditer(r"^([0-9a-f]+) <(.*)>:$", s, re.M)}
    ins, raw = {}, {}
    for line in s.splitlines():
        m = re.match(r"^\s*([0-9a-f]+):\t((?:[0-9a-f]{2} )+)\s*\t([.a-z][a-z0-9.]*)\s*(.*?)$", line)
        if m:
            a = int(m[1], 16)
            bs = bytes.fromhex(m[2])
            ins[a] = (len(bs), m[3], m[4])
            raw.update({a+i: b for i, b in enumerate(bs)})
    # Pinned Arduino digitalWrite -> turnOffPWM switch: CPI 18, BRCC
    # excludes invalid indices, table immediately follows the helper JMP.
    start = labels["digitalWrite"]
    jumps = [a for a in range(start, start+0x200, 2)
             if a in ins and ins[a][1] == "jmp" and "<__tablejump2__>" in ins[a][2]]
    assert len(jumps) == 1
    j = jumps[0]
    assert any(ins.get(k, (0, "", ""))[1] == "cpi" and
               "0x12" in ins[k][2] for k in range(j-24, j, 2))
    base = j + ins[j][0]
    targets = [2*(raw[base+i*2]+256*raw[base+i*2+1]) for i in range(18)]
    assert all(start <= t < start+0x200 or
               (ins[t][1] == "jmp" and start <= target(ins[t][2]) < start+0x200)
               for t in targets)
    table = labels["__tablejump2__"]
    endpoint = next(a for a in range(table, table+24, 2) if ins[a][1] == "eijmp")
    # Verify all strong vectors are covered (Timer3 aliases use same entry).
    syms = z.read(p+"symbols.txt").decode()
    strong = {int(m[1],16) for m in re.finditer(
        r"^([0-9a-f]+)\s+[0-9a-f]+\s+T __vector_\d+$", syms, re.M)}
    assert strong == {labels["__vector_"+str(v)] for v in VECTORS}
    maxima = {}
    for v in VECTORS:
        root = labels["__vector_"+str(v)]
        todo = [(root, 3, ())]
        seen, sei, sreg, timsk = set(), set(), set(), set()
        maxd = 0
        while todo:
            pc, d, ret = todo.pop()
            if (pc, d, ret) in seen:
                continue
            seen.add((pc, d, ret))
            assert len(seen) <= 300000 and 3 <= d <= 512, (v, pc, d)
            maxd = max(maxd, d)
            w, op, arg = ins[pc]
            assert not op.startswith("."), (v, pc, op)
            n = pc+w
            if op == "push":
                d += 1
            if op == "pop":
                d -= 1
            if op == "sei":
                sei.add(pc)
            if op == "out" and arg.startswith("0x3f,"):
                sreg.add(pc)
            if op == "sts" and arg.lower().startswith("0x006f,"):
                timsk.add(pc)
            assert not (op == "out" and re.match(r"0x3[de],", arg)), (v, pc)
            assert not (op == "sts" and re.match(r"0x00(?:5[dDeE]),", arg)), (v, pc)
            if op in ("icall", "eicall", "ijmp", "eijmp"):
                assert pc == endpoint and op == "eijmp", (v, pc, op)
                todo.extend((t, d, ret) for t in targets)
                continue
            if op in ("call", "rcall"):
                t = target(arg)
                if op == "rcall" and t == n:
                    todo.append((n, d+3, ret))
                else:
                    assert n not in ret, (v, pc, "recursive call")
                    todo.append((t, d+3, ret+(n,)))
                continue
            if op in ("jmp", "rjmp"):
                todo.append((target(arg), d, ret))
                continue
            if op in ("ret", "reti"):
                if ret:
                    assert op == "ret"
                    todo.append((ret[-1], d-3, ret[:-1]))
                else:
                    assert d == 3 and op == "reti", (v, pc, d, op)
                continue
            if op.startswith("br") and op != "break":
                todo.append((target(arg), d, ret))
            if op in ("sbrc", "sbrs", "sbic", "sbis", "cpse"):
                todo.append((n+ins[n][0], d, ret))
            todo.append((n, d, ret))
        assert v == 17 or not sei
        assert v == 17 or not timsk
        maxima[v] = maxd
        print(board, v, "states", len(seen), "stack", maxd,
              "SEI", sorted(map(hex, sei)), "SREG", sorted(map(hex, sreg)),
              "TIMSK1", sorted(map(hex, timsk)))
    assert maxima == {17:59, 21:14 if board in ("RF1000","RF2000-X44") else 16,
                      22:45, 23:12, 25:12, 26:11, 32:22}, maxima
    # Conservative sum conditional on separately checked interrupt mask
    # invariants: only Timer1 may open I, its own compare stays masked, and
    # all other ISRs restore their entry I=0. Not a global WCET/stack proof.
    print(board, "conditional nested stack envelope", maxima[17]+max(
          n for v, n in maxima.items() if v != 17))


if __name__ == "__main__":
    with open(sys.argv[1], "rb") as f:
        digest = hashlib.sha256(f.read()).hexdigest()
        assert digest in PINS, "unreviewed artifact"
        print("Pinned artifact", PINS[digest])
    with zipfile.ZipFile(sys.argv[1]) as z:
        for board in BOARDS:
            audit(z, board)

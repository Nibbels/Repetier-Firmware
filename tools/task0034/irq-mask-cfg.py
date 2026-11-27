#!/usr/bin/env python3
"""A73 interrupt-mask inventory for a SHA256-pinned canonical CI archive.

Requires networkx (analysis only). Tracks the I bit through actual instructions,
including saved SREG values on the register stack. Ordinary data branches are
conservatively unconstrained. Cyclic masked paths are deliberately reported,
not assigned an invented bound. This is not a global response-time proof.
"""
import argparse
import bisect
import hashlib
import json
import re
import zipfile

import networkx as nx

BOARDS = ("RF1000", "RF2000", "RF2000v2", "RF2000-X44")
VECTORS = (17, 21, 22, 23, 25, 26, 32)
WRITES = set("adc add and andi asr bld com dec eor inc in ld ldd ldi lds lsl lsr neg or ori ror sbc sbci ser sub subi swap clr rol".split())
NO_WRITE = set("cp cpc cpi cpse tst sbrc sbrs sbic sbis st std sts out sbi cbi nop push ret reti jmp rjmp call rcall ijmp eijmp icall eicall break cli sei wdr bset bclr clc sec clt set bst".split())
SKIP = ("sbrc", "sbrs", "sbic", "sbis", "cpse")


def target(arg):
    m = re.search(r";\s*0x([0-9a-f]+)\s*<", arg) or re.match(r"0x([0-9a-f]+)", arg)
    if not m:
        raise ValueError(arg)
    return int(m[1], 16)


def cost(op):
    """Upper cycles/instruction for the reached AVR instructions, 3-byte PC."""
    if op in ("call", "ret", "reti", "eicall"):
        return 5
    if op in ("rcall", "icall"):
        return 4
    if op in SKIP or op in ("jmp", "lpm", "elpm"):
        return 3
    if op.startswith("br") or op in ("rjmp", "ijmp", "eijmp", "ld", "ldd", "lds", "st", "std", "sts", "push", "pop", "sbi", "cbi", "adiw", "sbiw", "mul", "muls", "mulsu", "fmul", "fmuls", "fmulsu"):
        return 2
    return 1


def inventory(z, board):
    p = board + "/"
    assert json.loads(z.read(p + "verification.json"))["status"] == "verified"
    dump = z.read(p + "disassembly.txt").decode()
    labels = {m[2]: int(m[1], 16) for m in re.finditer(r"^([0-9a-f]+) <(.*)>:$", dump, re.M)}
    ins, raw = {}, {}
    for line in dump.splitlines():
        m = re.match(r"^\s*([0-9a-f]+):\t((?:[0-9a-f]{2} )+)\s*\t([.a-z][a-z0-9.]*)\s*(.*?)$", line)
        if m:
            a = int(m[1], 16)
            bs = bytes.fromhex(m[2])
            ins[a] = (len(bs), m[3], m[4])
            raw.update({a+i: b for i, b in enumerate(bs)})
    names = sorted((a, n) for n, a in labels.items())
    addresses = [a for a, _ in names]

    def name(a):
        x, n = names[bisect.bisect_right(addresses, a)-1]
        return n + "+" + hex(a-x)

    # Same reviewed Arduino digitalWrite -> turnOffPWM table as irq-stack-cfg.
    start = labels["digitalWrite"]
    jumps = [a for a in range(start, start+0x200, 2)
             if a in ins and ins[a][1] == "jmp" and "<__tablejump2__>" in ins[a][2]]
    assert len(jumps) == 1
    j = jumps[0]
    assert any(ins.get(k, (0, "", ""))[1] == "cpi" and "0x12" in ins[k][2]
               for k in range(j-24, j, 2))
    base = j + ins[j][0]
    targets = [2*(raw[base+i*2]+256*raw[base+i*2+1]) for i in range(18)]
    assert all(start <= t < start+0x200 or
               (ins[t][1] == "jmp" and start <= target(ins[t][2]) < start+0x200) for t in targets)
    table = labels["__tablejump2__"]
    endpoint = next(a for a in range(table, table+24, 2) if ins[a][1] == "eijmp")
    strong = {int(m[1],16) for m in re.finditer(r"^([0-9a-f]+)\s+[0-9a-f]+\s+T __vector_\d+$", z.read(p+"symbols.txt").decode(), re.M)}
    assert strong == {labels["__vector_"+str(v)] for v in VECTORS}
    delay = labels["HAL::delayMilliseconds(unsigned int)"]

    def getreg(reg, regs):
        return regs.get(int(reg[1:]))

    def setreg(reg, val, regs):
        n = int(reg[1:])
        regs.pop(n, None)
        if val is not None:
            regs[n] = val

    def kill(reg, regs):
        setreg(reg, None, regs)

    results = {}
    for v in VECTORS:
        # (PC, call returns, stacked SREG bits, register SREG bits, I)
        root = (labels["__vector_"+str(v)], (), (None,)*3, (), 0)
        g = nx.DiGraph()
        todo, seen = [root], set()
        maxd = 0
        opened, delay_entries, timsk = set(), set(), set()
        while todo:
            state = todo.pop()
            if state in seen:
                continue
            seen.add(state)
            pc, ret, stack, regtuple, I = state
            assert 3 <= len(stack) <= 512 and len(seen) < 500000, (v, pc, len(seen))
            maxd = max(maxd, len(stack))
            w, op, arg = ins[pc]
            assert not op.startswith("."), (board, v, pc)
            n = pc+w
            a = arg.split(";")[0].strip()
            toks = a.split(", ")
            regs = dict(regtuple)
            next_i = I
            if pc == delay and not I:
                delay_entries.add(ret)
            if op == "in" and toks[1] == "0x3f":
                setreg(toks[0], I, regs)
            elif op == "out" and toks[0] == "0x3f":
                next_i = getreg(toks[1], regs)
                assert next_i is not None, (board, v, hex(pc), "unknown SREG restore")
            elif op == "sei":
                next_i = 1
            elif op == "cli":
                next_i = 0
            elif op == "push":
                stack += (getreg(toks[0], regs),)
            elif op == "pop":
                setreg(toks[0], stack[-1], regs)
                stack = stack[:-1]
            elif op == "mov":
                setreg(toks[0], getreg(toks[1], regs), regs)
            elif op == "movw":
                d, s = (int(t[1:]) for t in toks)
                lo, hi = regs.get(s), regs.get(s+1)
                setreg("r"+str(d), lo, regs)
                setreg("r"+str(d+1), hi, regs)
            elif op in ("adiw", "sbiw"):
                d = int(toks[0][1:])
                kill(toks[0], regs)
                kill("r"+str(d+1), regs)
            elif op in ("mul", "muls", "mulsu", "fmul", "fmuls", "fmulsu"):
                kill("r0", regs)
                kill("r1", regs)
            elif op in ("lpm", "elpm"):
                kill(toks[0] if a else "r0", regs)
            elif op in WRITES:
                kill(toks[0], regs)
            elif op not in NO_WRITE and not op.startswith("br"):
                raise ValueError((op, arg, hex(pc)))
            if op in ("ld", "st", "lpm", "elpm"):
                for i, rs in enumerate(("X", "Y", "Z")):
                    if re.search(r"(?:^|, )(?:"+rs+r"\+|-"+rs+r")(?:,|$)", a):
                        kill("r"+str(26+2*i), regs)
                        kill("r"+str(27+2*i), regs)
            assert not (op in ("bset", "bclr") and a == "7")
            assert not (op == "out" and re.match(r"0x3[de],", a))
            assert not (op == "sts" and re.match(r"0x00(?:5[dDeEfF]),", a))
            if op == "sts" and a.lower().startswith("0x006f,"):
                timsk.add(pc)
            if op in ("icall", "eicall", "ijmp", "eijmp"):
                assert pc == endpoint and op == "eijmp"
                nexts = [(t, ret, stack) for t in targets]
            elif op in ("call", "rcall"):
                t = target(arg)
                if op == "rcall" and t == n:
                    nexts = [(n, ret, stack+(None,)*3)]
                else:
                    assert n not in ret, (v, pc, "recursive IRQ call")
                    nexts = [(t, ret+(n,), stack+(None,)*3)]
            elif op in ("jmp", "rjmp"):
                nexts = [(target(arg), ret, stack)]
            elif op in ("ret", "reti"):
                if ret:
                    assert op == "ret"
                    nexts = [(ret[-1], ret[:-1], stack[:-3])]
                else:
                    assert len(stack) == 3 and op == "reti"
                    nexts = []
            else:
                nexts = [(n, ret, stack)]
                if op.startswith("br") and op != "break":
                    nexts.append((target(arg), ret, stack))
                if op in SKIP:
                    nexts.append((n+ins[n][0], ret, stack))
            if next_i and not I:
                opened.add(pc)
            rt = tuple(sorted(regs.items()))
            nexts = [(q, ret2, stk, rt, next_i) for q, ret2, stk in nexts]
            g.add_node(state)
            g.add_edges_from((state, q) for q in nexts)
            todo.extend(nexts)
        assert v == 17 or (not opened and not timsk), (board, v)
        mask = g.subgraph(x for x in g if not x[-1])
        scc = [c for c in nx.strongly_connected_components(mask)
               if len(c)>1 or any(mask.has_edge(x,x) for x in c)]
        cycles = []
        for c in scc:
            pcs = sorted({x[0] for x in c})
            cycles.append({"states":len(c), "first":name(pcs[0]), "last":name(pcs[-1])})
        # Cyclic graph: deliberately no number until loop bounds are proved.
        acyclic_cycles = None
        if not scc:
            longest = {}
            for x in nx.topological_sort(mask):
                longest[x] = cost(ins[x[0]][1]) + max((longest[p] for p in mask.predecessors(x)), default=0)
            acyclic_cycles = max(longest.values(), default=0)
        results[v] = {"states":len(seen), "stackBytesIncludingPc":maxd,
                      "interruptOpenSites":len(opened), "maskedDelayReturnChains":[[name(a) for a in t] for t in sorted(delay_entries)],
                      "maskedCycles":cycles, "acyclicMaskedBodyCycles":acyclic_cycles}
    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive")
    parser.add_argument("sha256")
    parser.add_argument("--reject-masked-delay", action="store_true")
    args = parser.parse_args()
    with open(args.archive,"rb") as f:
        assert hashlib.sha256(f.read()).hexdigest() == args.sha256
    with zipfile.ZipFile(args.archive) as z:
        summary = json.loads(z.read("summary.json"))
        assert not summary["failures"]
        result = {board: inventory(z,board) for board in BOARDS}
    print(json.dumps({"testedMerge":summary["sha"],"sha256":args.sha256,"boards":result},indent=2))
    if args.reject_masked_delay and any(v["maskedDelayReturnChains"] for b in result.values() for v in b.values()):
        raise SystemExit(1)

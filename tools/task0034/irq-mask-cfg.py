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
from timer1_exclusion import timer1_exclusion
from avr_constant import transfer, branch as known_branch

BOARDS = ("RF1000", "RF2000", "RF2000v2", "RF2000-X44")
VECTORS = (17, 21, 22, 23, 25, 26, 32)
WRITES = set("adc add and andi asr bld com dec eor inc in ld ldd ldi lds lsl lsr neg or ori ror sbc sbci ser sub subi swap clr rol".split())
NO_WRITE = set("cp cpc cpi cpse tst sbrc sbrs sbic sbis st std sts out sbi cbi nop push ret reti jmp rjmp call rcall ijmp eijmp icall eicall break cli sei wdr bset bclr clc sec clt set bst".split())
SKIP = ("sbrc", "sbrs", "sbic", "sbis", "cpse")


from system_register_sites import register_site


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


def admission_blocked_states(graph, instructions):
    """Conservative instruction-boundary blackout, including delayed admission.

    Include the successor of EVERY SEI or SREG write enabling I, not
    merely transitions from I=0: consecutive SEI instructions must not
    create an artificial service opportunity. A saved SREG write is
    treated at least as conservatively as SEI. RETI's resumed instruction
    is outside this vector graph and must be charged by its caller.
    """
    delayed = set()
    for x, y in graph.edges:
        _, op, arg = instructions[x[0]]
        if y[-1] and (op == "sei" or
                      (op == "out" and arg.split(";")[0].strip().startswith("0x3f,"))):
            delayed.add(y)
    return {x for x in graph if not x[-1]} | delayed


def inventory(z, board, constant_pwm=False, timer3_wrap=False, pwm_owned_memory=False):
    p = board + "/"
    assert json.loads(z.read(p + "verification.json"))["status"] == "verified"
    if timer3_wrap:
        # Source ownership prerequisite: normal free-running 16-bit Timer3.
        # The unrestricted PWM body must also be shorter than even one wrap
        # at /1; the CLI checks this before invoking the refinement.
        hal = z.read(p + "build/sketch/HAL.cpp").decode()
        assert "TCCR3A=0;" in hal and "TCCR3B=_BV(CS32);" in hal
        assert "TCNT3=0;" in hal
    tracked = set()
    if pwm_owned_memory:
        # Diagnostic refinement only. These four private scalar objects
        # have no address escape in the reviewed HAL. The model assumes
        # valid C++ object accesses (no out-of-bounds/stack corruption);
        # it is NOT a machine-level indirect-store alias proof.
        names = ("pwm_count_heater", "heaterLeaseFailed", "heaterLeaseReady", "watchdogActive")
        for name in names:
            assert not re.search(r"(?<!&)&\s*"+name+r"\b", hal), (board,name,"address escape")
            for filename in z.namelist():
                if filename.startswith(p+"build/sketch/") and filename.endswith((".h",".cpp",".ino")) and not filename.endswith("/HAL.cpp"):
                    assert not re.search(r"\b"+name+r"\b",z.read(filename).decode()), (board,filename,name)
            rows = [int(m[1],16)-0x800000 for m in re.finditer(
                r"^([0-9a-f]+) 00000001 [bBdD] (.*)$",z.read(p+"symbols.txt").decode(),re.M)
                if name in m[2]]
            assert len(rows)==1 and rows[0]>=0x200,(board,name,rows)
            tracked.add(rows[0])
        assert len(tracked)==4
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
        track = constant_pwm and v == 22
        wrap_track = track and timer3_wrap
        memory_track = wrap_track and pwm_owned_memory
        root = (labels["__vector_"+str(v)], (), (None,)*3, (), (), (), (None,)*3, -1, 1, (), None, 0)
        g = nx.DiGraph()
        todo, seen = [root], set()
        system_sites = set()
        maxd = 0
        opened, delay_entries, timsk = set(), set(), set()
        while todo:
            state = todo.pop()
            if state in seen:
                continue
            seen.add(state)
            pc, ret, stack, regtuple, values, flags, vstack, tov, remaining, memory, provenance, I = state
            assert 3 <= len(stack) <= 512 and len(seen) < 500000, (v, pc, len(seen))
            maxd = max(maxd, len(stack))
            w, op, arg = ins[pc]
            assert not op.startswith("."), (board, v, pc)
            n = pc+w
            a = arg.split(";")[0].strip()
            toks = a.split(", ")
            site = register_site(pc, op, arg)
            if site:
                system_sites.add(site)
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
            taken = None
            previous_values = dict(values)
            memory = dict(memory)
            old_provenance = provenance
            provenance = None
            if track:
                taken = known_branch(op,a,values,flags)
                values,flags,vstack = transfer(op,a,values,flags,vstack)
            if memory_track and op=="lds" and int(toks[1],0) in tracked:
                address=int(toks[1],0)
                reg=int(toks[0][1:])
                provenance=(reg,address)
                if address in memory:
                    known=dict(values);known[reg]=memory[address]
                    values=tuple(sorted(known.items()))
            if memory_track and op=="sts" and int(toks[0],0) in tracked:
                address=int(toks[0],0)
                value=previous_values.get(int(toks[1][1:]))
                if value is None:memory.pop(address,None)
                else:memory[address]=value
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
                    if taken is not False: nexts.append((target(arg), ret, stack))
                    if taken is True: nexts=nexts[1:]
                if op in SKIP:
                    if taken is not False: nexts.append((n+ins[n][0], ret, stack))
                    if taken is True: nexts=nexts[1:]
            if next_i and not I:
                opened.add(pc)
            rt = tuple(sorted(regs.items()))
            # PC pushes/call returns add/remove unknown return bytes.
            # Ordinary pushes/pops of tracked data were handled by transfer.
            # Initially unknown sticky TOV3, plus at most one new wrap.
            # A wrap between reads is conservatively delayed to the next
            # read, even if a real intervening W1C would have cleared it.
            if wrap_track and op in ("sbic", "sbis") and a == "0x18, 0":
                choices = [(b, remaining) for b in (0,1)] if tov == -1 else [(tov, remaining)]
                if tov == 0 and remaining:
                    choices.append((1,0))
                transitions = [(n+ins[n][0] if b == (op=="sbis") else n,
                                ret, stack, b, left) for b,left in choices]
            else:
                if wrap_track:
                    assert not (op in ("sbi","cbi") and toks[0]=="0x18")
                    assert not (op=="sts" and int(toks[0],0) in (0x38,0x90,0x91,0x92,0x94,0x95))
                    if op == "out" and toks[0] == "0x18":
                        written = dict(values).get(int(toks[1][1:]))
                        assert written is not None, (board,hex(pc),"unknown TIFR3 write")
                        if written & 1:
                            tov = 0
                transitions = [(q,ret2,stk,tov,remaining) for q,ret2,stk in nexts]
            nexts = []
            for q,ret2,stk,b,left in transitions:
                mem=dict(memory);known=dict(values)
                # Refine only the witnessed immediate LDS -> CPSE equality.
                # The non-equal edge remains unconstrained. Direct unknown
                # stores invalidate facts; other valid object accesses do not alias.
                if memory_track and op=="cpse" and old_provenance is not None and q!=n:
                    operands=[int(x[1:]) for x in toks]
                    reg,address=old_provenance
                    if reg in operands:
                        other=operands[1] if operands[0]==reg else operands[0]
                        if other in previous_values:
                            known[reg]=mem[address]=previous_values[other]
                nexts.append((q,ret2,stk,rt,tuple(sorted(known.items())),flags,
                              vstack+(None,)*(len(stk)-len(vstack)) if len(stk)>=len(vstack) else vstack[:len(stk)],
                              b,left,tuple(sorted(mem.items())),provenance,next_i))
            g.add_node(state)
            g.add_edges_from((state, q) for q in nexts)
            todo.extend(nexts)
        assert v == 17 or (not opened and not timsk), (board, v)
        def masked_bound(mask):
            scc = [c for c in nx.strongly_connected_components(mask)
                   if len(c)>1 or any(mask.has_edge(x,x) for x in c)]
            cycles = []
            for c in scc:
                pcs = sorted({x[0] for x in c})
                cycles.append({"states":len(c), "first":name(pcs[0]), "last":name(pcs[-1])})
            # Recognize the fully witnessed constant-count pulse delay:
            # LDI r,N; DEC r; BRNE DEC. All external entries must be from
            # that LDI, and DEC/BRNE must be the sole cycle and sole exit.
            bounded_mask = mask.copy()
            extra = {}
            unresolved = []
            for c in scc:
                if len(c) != 2:
                    unresolved.append(c)
                    continue
                dec, branch = sorted(c, key=lambda x:x[0])
                dw, dop, da = ins[dec[0]]
                bw, bop, ba = ins[branch[0]]
                parents = set(mask.predecessors(dec)) - c
                if not (dop == "dec" and bop == "brne" and
                        branch[0] == dec[0]+dw and target(ba)==dec[0] and
                        set(mask.successors(dec)) == {branch} and
                        set(mask.predecessors(branch)) == {dec} and
                        len(set(mask.successors(branch))-c)==1 and
                        parents and set(g.predecessors(dec)) == set(mask.predecessors(dec))):
                    unresolved.append(c)
                    continue
                counts = []
                for parent in parents:
                    pw, pop, pa = ins[parent[0]]
                    m = re.fullmatch(r"(r\d+), 0x([0-9a-fA-F]+)", pa.split(";")[0].strip())
                    if pop != "ldi" or parent[0]+pw != dec[0] or not m or m[1]!=da.strip():
                        break
                    counts.append(int(m[2],16) or 256)
                if len(counts)!=len(parents):
                    unresolved.append(c)
                    continue
                bounded_mask.remove_edge(branch,dec)
                extra[dec]=(max(counts)-1)*(cost(dop)+cost(bop))
            acyclic_cycles = None
            boundaries = None
            if not unresolved:
                longest = {}
                entry = {}
                for x in nx.topological_sort(bounded_mask):
                    charge = cost(ins[x[0]][1]) + extra.get(x,0)
                    parents = list(bounded_mask.predecessors(x))
                    longest[x] = charge + max((longest[p] for p in parents), default=0)
                    incoming = [entry[p] for p in parents if p in entry]
                    if x == root:
                        incoming.append(0)
                    if incoming:
                        entry[x] = charge + max(incoming)
                acyclic_cycles = max(longest.values(), default=0)
                exits = [x for x in longest if ins[x[0]][1] == "reti" and not g.out_degree(x)]
                opens = [x for x in entry if any(y not in mask for y in g.successors(x))]
                def maximum(values):
                    return max(values, default=None)
                boundaries = {
                    "entryToFirstAdmissionCycles": maximum(entry[x] for x in opens),
                    "entryToReturnWithoutAdmissionCycles": maximum(entry[x] for x in exits if x in entry),
                    "lastMaskedSuffixToReturnCycles": maximum(longest[x] for x in exits),
                    "includesVectorEntry": False,
                    "includesResumedInstructionAfterReti": False,
                }
            return cycles, len(unresolved), acyclic_cycles, boundaries

        cycles, unresolved_count, acyclic_cycles, _ = masked_bound(
            g.subgraph(x for x in g if not x[-1]))
        # AVR SEI cannot admit a pending interrupt before the following
        # instruction has executed. Conservatively apply the same delay
        # to a saved-SREG restore. Include all such successor states, even
        # if a different incoming path would already permit admission.
        # This catches SEI; CLI as a continuous blackout instead of two
        # independent short intervals.
        admission_cycles, admission_unresolved, admission_bound, admission_boundaries = masked_bound(
            g.subgraph(admission_blocked_states(g, ins)))
        results[v] = {"states":len(seen), "explicitSystemRegisterSites": sorted(system_sites), "stackBytesIncludingPc":maxd,
                      "constantTracking":track, "timer3WrapTracking":wrap_track, "pwmOwnedMemoryDiagnostic":memory_track, "interruptOpenSites":len(opened), "maskedDelayReturnChains":[[name(a) for a in t] for t in sorted(delay_entries)],
                      "maskedCycles":cycles, "unresolvedMaskedCycles":unresolved_count, "acyclicMaskedBodyCycles":acyclic_cycles,
                      "admissionMaskedCycles":admission_cycles, "unresolvedAdmissionMaskedCycles":admission_unresolved,
                      "acyclicAdmissionMaskedBodyCycles":admission_bound,
                      "admissionBoundaries":admission_boundaries}
        if v == 17:
            results[v]["selfMask"] = timer1_exclusion(g, root, ins)
    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive")
    parser.add_argument("sha256")
    parser.add_argument("--reject-masked-delay", action="store_true")
    parser.add_argument("--constant-pwm", action="store_true",
                        help="also propagate byte/flag constants in non-nesting PWM vector 22")
    parser.add_argument("--timer3-wrap", action="store_true", help="refine PWM using reviewed Timer3 ownership and one possible new wrap")
    parser.add_argument("--pwm-owned-memory", action="store_true", help="diagnostic private-scalar correlations; requires reviewed source ownership")
    args = parser.parse_args()
    assert not args.timer3_wrap or args.constant_pwm
    assert not args.pwm_owned_memory or args.timer3_wrap
    with open(args.archive,"rb") as f:
        assert hashlib.sha256(f.read()).hexdigest() == args.sha256
    with zipfile.ZipFile(args.archive) as z:
        summary = json.loads(z.read("summary.json"))
        assert not summary["failures"]
        result = {board: inventory(z,board,args.constant_pwm) for board in BOARDS}
        if args.timer3_wrap:
            for board in BOARDS:
                upper = result[board][22]["acyclicMaskedBodyCycles"]
                assert upper is not None and upper < 65536
                refined = inventory(z,board,True,True)
                refined[22]["unrestrictedBodyCycles"] = upper
                result[board][22] = refined[22]
                if args.pwm_owned_memory:
                    owned=inventory(z,board,True,True,True)[22]
                    owned["unrestrictedBodyCycles"]=upper
                    owned["withoutOwnedMemoryBodyCycles"]=refined[22]["acyclicMaskedBodyCycles"]
                    result[board][22]=owned
    print(json.dumps({"testedMerge":summary["sha"],"sha256":args.sha256,"boards":result},indent=2))
    if args.reject_masked_delay and any(v["maskedDelayReturnChains"] for b in result.values() for v in b.values()):
        raise SystemExit(1)

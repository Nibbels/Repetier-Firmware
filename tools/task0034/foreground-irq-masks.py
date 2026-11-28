"""Instruction-level foreground IRQ-mask diagnostic on the linked images.

Reuse the verified stack walk's exact instruction graph, including compiler
switch-table entries, tail transfers and return-address/stack adjustments.
Discover reachable (function, entry I) contexts by a monotone return-summary
fixed point. Track saved SREG.I through registers and direct Y-frame spills.

This is conditional on valid AVR ABI and stack/device object ownership.
Unknown restores are explored both ways and retained as blockers. Ordinary
data branches are unconstrained. This deliberately does not claim a composed
IRQ response bound or discard non-returning paths.
"""
import bisect
import collections
import re
import networkx as nx



def io_address(token):
    try:
        return int(token, 0)
    except ValueError:
        return None


def step_bits(base, ins, I, regs, slots, writes, no_write):
    """Update just SREG.I provenance; return all possible next I values."""
    pc, depth, y, _ = base
    _, op, arg = ins[pc]
    a = arg.split(";")[0].strip()
    t = a.split(", ")
    regs, slots = dict(regs), dict(slots)

    def put(reg, val):
        if not re.fullmatch(r"r\d+", reg):
            raise ValueError((hex(pc), op, a, "invalid register"))
        n = int(reg[1:])
        if val is None:
            regs.pop(n, None)
        else:
            regs[n] = val

    def get(reg):
        return regs.get(int(reg[1:]))

    def slot(operand):
        m = re.fullmatch(r"Y(?:\+(\d+|0x[0-9a-fA-F]+))?", operand)
        return (-y - int(m[1] or "0", 0)) if m and y is not None else None

    # Only track constant values for the explicit I bit; arbitrary arithmetic
    # discards provenance. A direct compiler frame access cannot alias the
    # return address under the same valid-object premise as the stack walk.
    # Direct SREG LDS/STS uses the data-space address, not the IO address.
    unknown_restore = False
    enabling_write = False
    next_i = (I,)
    if (op == "in" and io_address(t[1]) == 0x3f) or (op == "lds" and io_address(t[1]) == 0x5f):
        put(t[0], I)
    elif ((op == "out" and io_address(t[0]) == 0x3f) or
          (op == "sts" and io_address(t[0]) == 0x5f)):
        bit = get(t[1])
        unknown_restore = bit is None
        next_i = (0, 1) if bit is None else (bit,)
        enabling_write = True
    elif op == "sei" or (op == "bset" and a == "7"):
        next_i = (1,)
        enabling_write = True
    elif op == "cli" or (op == "bclr" and a == "7"):
        next_i = (0,)
    elif op == "push":
        value = get(t[0])
        if value is not None:
            slots[depth] = value
        else:
            slots.pop(depth, None)
    elif op == "pop":
        put(t[0], slots.pop(depth-1, None))
    elif op == "mov":
        put(t[0], get(t[1]))
    elif op == "movw":
        dest, src = (int(x[1:]) for x in t)
        lo, hi = regs.get(src), regs.get(src+1)
        put("r"+str(dest), lo)
        put("r"+str(dest+1), hi)
    elif op in ("ldi", "ser"):
        put(t[0], 1 if op == "ser" else int(t[1],0) >> 7)
    elif op == "clr" or (op == "eor" and t[0] == t[1]):
        put(t[0], 0)
    elif op in ("andi", "ori"):
        bit = (int(t[1],0) >> 7) & 1
        put(t[0], (bit if (op == "andi" and not bit) or (op == "ori" and bit)
                   else get(t[0])))
    elif op in ("ld", "ldd"):
        key = slot(t[1])
        put(t[0], slots.get(key) if key is not None else None)
    elif op in ("st", "std"):
        key = slot(t[0])
        if key is not None:
            if get(t[1]) is None:
                slots.pop(key, None)
            else:
                slots[key] = get(t[1])
    elif op in ("adiw", "sbiw"):
        put(t[0], None)
        put("r"+str(int(t[0][1:])+1), None)
    elif op in ("mul", "muls", "mulsu", "fmul", "fmuls", "fmulsu"):
        put("r0", None)
        put("r1", None)
    elif op in ("lpm", "elpm"):
        put(t[0] if a else "r0", None)
    elif op in writes:
        put(t[0], None)
    elif op not in no_write and not op.startswith("br"):
        raise ValueError((hex(pc), op, a, "unknown register effects"))
    if op in ("ld", "st", "lpm", "elpm"):
        for i, pointer in enumerate(("X","Y","Z")):
            if re.search(r"(?:^|, )(?:"+pointer+r"\+|-"+pointer+r")(?:,|$)", a):
                regs.pop(26+2*i, None)
                regs.pop(27+2*i, None)
    return next_i, enabling_write, unknown_restore, regs, slots


def discover(ins, trace, main, writes, no_write, state_limit=300000):
    """Discover actual call contexts, not every function under I=0."""
    returns = collections.defaultdict(set)
    dependents = collections.defaultdict(set)
    contexts = {}
    pending = {(main,0)}
    while pending:
        ctx = pending.pop()
        root, entry_i = ctx
        cfg = trace[root]
        start = (cfg["entry"], entry_i, False)
        todo, queued = collections.deque([start]), {start}
        facts = {start: ({1:0}, {})}
        graph = nx.DiGraph()
        calls, unknown = {}, set()
        exits = set()

        def propagate(state, successor, regs, slots):
            graph.add_edge(state, successor)
            old = facts.get(successor)
            if old is not None:
                # Meet equal known bits; disagreement is unknown. Keeping
                # only one fact map per stack/I/delay state prevents an
                # exponential product of unrelated LDI register values.
                regs={r:b for r,b in old[0].items() if regs.get(r)==b}
                slots={k:b for k,b in old[1].items() if slots.get(k)==b}
                if old==(regs,slots):
                    return
            facts[successor]=(regs,slots)
            if successor not in queued:
                todo.append(successor)
                queued.add(successor)

        while todo:
            state = todo.popleft()
            queued.remove(state)
            if len(facts) > state_limit:
                raise ValueError((hex(root),entry_i,"foreground I-state limit"))
            graph.add_node(state)
            base,I,delayed=state
            pc=base[0]
            _,op,_=ins[pc]
            ni,enable,unresolved,regs,slots=step_bits(base,ins,I,*facts[state],writes,no_write)
            if unresolved:
                unknown.add(pc)
            if op=="ret":
                exits.add(I)
            if base in cfg["calls"]:
                callee=(cfg["calls"][base],I)
                calls[state]=callee
                if callee not in contexts and callee!=ctx:
                    pending.add(callee)
                dependents[callee].add(ctx)
                ni=tuple(returns[callee])
                # Ordinary AVR ABI: the caller's saved register spills and
                # r2-r17/r28-r29 survive a returning call; r1 is zero again.
                # Any value passed via argument registers is unknown in the
                # callee, so an argument-based SREG restore is not silently
                # assumed safe. ABI/assembly ownership remains a premise.
                regs={r:b for r,b in regs.items() if 2<=r<=17 or r in (28,29)}
                regs[1]=0
                enable=False
            for successor in cfg["edges"][base]:
                next_slots={k:b for k,b in slots.items() if 3<=k<successor[1]}
                for i in ni:
                    ns=(successor,i,bool(enable and i))
                    propagate(state,ns,regs,next_slots)
        contexts[ctx]={"graph":graph,"calls":calls,"unknown":unknown,"entry":start,"returns":exits}
        # Least fixed point over possible returning I states. All edges
        # through a call are enabled only by a witnessed abstract return.
        if not exits <= returns[ctx]:
            returns[ctx].update(exits)
            pending.update(dependents[ctx])
    return contexts


def blocked(state):
    return not state[1] or state[2]


def analyze(ins, labels, contexts, cost):
    addresses=sorted(labels)
    def name(pc):
        a=addresses[bisect.bisect_right(addresses,pc)-1]
        return labels[a]+"+"+hex(pc-a)
    def context_name(ctx):
        return {"function":name(ctx[0]),"entryI":ctx[1]}

    # First close full, never-opening masked callees in bottom-up order.
    # Recursion and any local CFG loop remain unresolved, not assigned a
    # guessed cost. Later iterations may resolve only proven child DAGs.
    closed={}
    candidates={k for k,v in contexts.items()
                if k[1]==0 and all(s[1]==0 for s in v["graph"]) and
                not v["unknown"] and nx.is_directed_acyclic_graph(v["graph"])}
    def longest(g, calls):
        depth={}
        for s in nx.topological_sort(g):
            child = closed[calls[s]] if s in calls and calls[s][1]==0 else 0
            depth[s]=cost(ins[s[0][0]][1])+child+max((depth[p] for p in g.predecessors(s)),default=0)
        return max(depth.values(),default=0)
    while candidates:
        ready=[k for k in candidates if all(c in closed for c in contexts[k]["calls"].values())]
        if not ready:
            break
        for k in ready:
            v=contexts[k]
            closed[k]=longest(v["graph"],v["calls"])
            candidates.remove(k)

    rows=[]
    for k,v in sorted(contexts.items()):
        g=v["graph"]
        mask=g.subgraph(s for s in g if blocked(s))
        cycles=[]
        for component in nx.strongly_connected_components(mask):
            if len(component)>1 or any(mask.has_edge(s,s) for s in component):
                pcs=sorted({s[0][0] for s in component})
                cycles.append({"states":len(component),"first":name(pcs[0]),"last":name(pcs[-1])})
        unknown_calls=sorted({(s[0][0],c) for s,c in v["calls"].items()
                              if s[1]==0 and c not in closed})
        masked_calls=sorted({(s[0][0],c) for s,c in v["calls"].items() if s[1]==0})
        bound=None if cycles or unknown_calls or v["unknown"] else longest(mask,v["calls"])
        if mask or v["unknown"] or any(i!=k[1] for i in v["returns"]):
            rows.append({**context_name(k),"states":len(g),"maskedStates":len(mask),
                         "returnI":sorted(v["returns"]),
                         "unknownRestores":[name(pc) for pc in sorted(v["unknown"])],
                         "maskedLocalCycles":cycles,
                         "maskedCalls":[{"site":name(pc),**context_name(c)} for pc,c in masked_calls],
                         "unclosedMaskedCalls":[{"site":name(pc),**context_name(c)} for pc,c in unknown_calls],
                         "localPlusClosedCalleeCycles":bound,
                         "fullyMaskedAcyclicCalleeCycles":closed.get(k)})
    unresolved=[r for r in rows if r["localPlusClosedCalleeCycles"] is None]
    changed=[r for r in rows if any(i!=r["entryI"] for i in r["returnI"])]
    return {"contexts":len(contexts),"states":sum(len(v["graph"]) for v in contexts.values()),
            "closedMaskedCallees":len(closed),"maskedContexts":len(rows),
            "unresolvedContexts":len(unresolved),"changedReturnIContexts":len(changed),
            "largestClosedLocalCycles":max((r["localPlusClosedCalleeCycles"] or 0 for r in rows),default=0),
            "rows":rows,"releaseProof":False,
            "limitations":["valid AVR ABI, compiler spill and device/SRAM ownership assumed",
                           "ordinary data branches overapproximated; loops not bounded by guess",
                           "non-returning startup/reset paths retained; heater-off classification separate",
                           "local bounds do not include call-crossing suffix of a callee returning with changed I",
                           "in-flight instruction, interrupt nesting, priority and restart not yet composed"]}


def inventory(z,board,fg,irq):
    trace={}
    frame=fg.inventory(z,board,True,trace)
    assert not frame["errors"],frame["errors"]
    ins,_,labels=fg.decode_linked(z.read(board+"/disassembly.txt").decode(),board)
    contexts=discover(ins,trace,int(frame["main"],16),irq["WRITES"],irq["NO_WRITE"])
    return analyze(ins,labels,contexts,irq["cost"])

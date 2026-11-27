#!/usr/bin/env python3
"""A73 diagnostic foreground CFG inventory, NOT a stack bound.

Usage: python foreground-cfg-inventory.py ARTIFACT.zip RF2000v2
Requires GNU readelf and networkx. Pinned to the actual qualified combined
image archive. Resolves inline table candidates and instruction reachability
rather than treating embedded flash bytes as instructions. CFI frame sizes
are metadata only; this deliberately does not claim outgoing-argument,
assembly, feasible guarded recursion, startup or IRQ timing coverage.
"""
import collections
import hashlib
import re
import subprocess
import sys
import tempfile
import zipfile

import networkx as nx

PIN = "234dea9336e8dcdd9f65212723236381a781c17bfc243f70d6b9ad96ff18ff52"
BOARDS = ("RF1000", "RF2000", "RF2000v2", "RF2000-X44")

def target(arg):
    m = re.search(r";\s*0x([0-9a-f]+)\s*<", arg) or re.match(r"0x([0-9a-f]+)", arg)
    if not m:
        raise ValueError(arg)
    return int(m[1], 16)

def inventory(z, board):
    p = "host/experimental-avr/emergency/" + board + "/"
    s = z.read(p+"disassembly.txt").decode()
    labels = {int(m[1],16):m[2] for m in re.finditer(r"^([0-9a-f]+) <(.*)>:$",s,re.M)}
    ins, raw = {}, {}
    for line in s.splitlines():
        m = re.match(r"^\s*([0-9a-f]+):\t((?:[0-9a-f]{2} )+)\s*\t([.a-z][a-z0-9.]*)\s*(.*?)$",line)
        if m:
            a = int(m[1],16)
            bs = bytes.fromhex(m[2])
            ins[a] = (len(bs),m[3],m[4])
            raw.update({a+i:v for i,v in enumerate(bs)})
    # Candidates from GCC's table dispatch. The nearby CPI is an inventory
    # heuristic, not a proof of the range check. Re-audit guards/targets for
    # a release-bound analysis. Do not silently skip an unknown table.
    tables, bad = {}, []
    for j,(w,op,arg) in ins.items():
        if op not in ("jmp","rjmp") or "<__tablejump2__>" not in arg:
            continue
        prev = [ins.get(j-k,(0,"","")) for k in (6,4,2)]
        m = [re.match(r"r(?:30|31|24), 0x([0-9A-Fa-f]+)",i[2]) for i in prev]
        if [i[1] for i in prev] != ["subi","sbci","sbci"] or not all(m):
            bad.append((hex(j),prev))
            continue
        base = ((-sum(int(x[1],16)<<(i*8) for i,x in enumerate(m)))&0xffffff)*2
        if base != j+w:
            bad.append((hex(j),"not inline",hex(base)))
            continue
        c = [(a,re.match(r"r\d+, 0x([0-9A-Fa-f]+)",ins[a][2]))
             for a in range(j-28,j-6,2) if a in ins and ins[a][1]=="cpi"]
        if not c or not c[-1][1]:
            bad.append((hex(j),"no count"))
            continue
        count = int(c[-1][1][1],16)
        ts = [2*(raw[base+i*2]+256*raw[base+i*2+1]) for i in range(count)]
        tables[j] = ts
        if not all(t in ins for t in ts):
            bad.append((hex(j),"unknown target"))
    print("tables",len(tables),"bad",bad)
    assert not bad
    with tempfile.NamedTemporaryFile() as f:
        f.write(z.read(p+"Repetier.ino.elf"))
        f.flush()
        cfi = subprocess.check_output(["readelf","--debug-dump=frames-interp",f.name],text=True)
    frames = {}
    for block in re.split(r"(?=^[0-9a-f]+ .* FDE .*pc=)",cfi,flags=re.M):
        m = re.search(r"FDE .*pc=([0-9a-f]+)\.\.([0-9a-f]+)",block)
        if m:
            a,e = map(lambda x:int(x,16),m.groups())
            frames[a] = (e,max([3]+[int(x) for x in re.findall(
                r"^\s*[0-9a-f]+\s+r(?:28|32)\+(\d+)",block,re.M)]))
    syms = {int(m[1],16):(int(m[2],16),m[3]) for m in re.finditer(
        r"^([0-9a-f]+) ([0-9a-f]+) [tTwW] (.+)$",z.read(p+"symbols.txt").decode(),re.M)}
    def name(a):
        return labels.get(a,syms.get(a,(0,hex(a)))[1])
    g = nx.DiGraph()
    unknown, seen = set(), set()
    pending = [next(a for a,n in labels.items() if n=="main")]
    callsites = collections.defaultdict(list)
    while pending:
        root = pending.pop()
        if root in seen:
            continue
        seen.add(root)
        g.add_node(root)
        todo, visited = [root], set()
        while todo:
            pc = todo.pop()
            if pc in visited:
                continue
            visited.add(pc)
            if pc not in ins:
                unknown.add((root,pc,"missing"))
                continue
            w,op,arg = ins[pc]
            n = pc+w
            if pc in tables:
                todo.extend(tables[pc])
                continue
            if op.startswith(".") or op in ("icall","eicall","ijmp","eijmp"):
                unknown.add((root,pc,op))
                continue
            if op in ("call","rcall"):
                t = target(arg)
                if op=="rcall" and t==n:
                    todo.append(n)
                    continue
                g.add_edge(root,t)
                callsites[root,t].append(pc)
                pending.append(t)
                todo.append(n)
                continue
            if op in ("jmp","rjmp"):
                t = target(arg)
                # Tentative tail-transfer classification from FDE ranges.
                if t!=root and t in frames and (root not in frames or not root<=t<frames[root][0]):
                    g.add_edge(root,t)
                    callsites[root,t].append(pc)
                    pending.append(t)
                else:
                    todo.append(t)
                continue
            if op in ("ret","reti","break"):
                continue
            if op.startswith("br"):
                todo.append(target(arg))
            if op in ("sbrc","sbrs","sbic","sbis","cpse"):
                if n not in ins:
                    unknown.add((root,pc,"skip missing"))
                    continue
                todo.append(n+ins[n][0])
            todo.append(n)
    print("nodes",len(g),"edges",len(g.edges),"unknown",
          [(name(a),hex(b),c) for a,b,c in sorted(unknown)])
    for scc in sorted(nx.strongly_connected_components(g),key=len,reverse=True):
        if len(scc)<2 and not g.has_edge(next(iter(scc)),next(iter(scc))):
            continue
        print("SCC",len(scc),"sum CFI",sum(frames.get(a,(0,3))[1] for a in scc))
        for a in sorted(scc):
            print(hex(a),frames.get(a,(0,None))[1],name(a),"=>",
                  [(hex(b),name(b)) for b in g.successors(a) if b in scc])
    print("no CFI",[(hex(a),name(a)) for a in sorted(seen) if a not in frames])
    for a in g:
        if name(a).startswith("Commands::checkForPeriodicalActions"):
            print("SCHEDULER EDGES")
            for b in sorted(g.successors(a),key=lambda b:min(callsites[a,b])):
                print(" ".join(hex(pc) for pc in sorted(callsites[a,b])),
                      frames.get(b,(0,None))[1],name(b))
    print("DIAGNOSTIC ONLY: no proven foreground stack or IRQ response bound")

if __name__ == "__main__":
    with open(sys.argv[1],"rb") as f:
        assert hashlib.sha256(f.read()).hexdigest()==PIN, "unreviewed artifact"
    assert sys.argv[2] in BOARDS
    with zipfile.ZipFile(sys.argv[1]) as z:
        inventory(z,sys.argv[2])

#!/usr/bin/env python3
"""A73 instruction-level foreground frame and callsite inventory.

Not a whole-program stack proof: conditional branches are overapproximated
and guarded foreground call recursion still needs a feasible-chain model.
Inputs are a CI artifact, its SHA256 and board; use --canonical for the
new root-level combined-image layout. No AVR toolchain needed.
"""
import argparse
import bisect
import collections
import hashlib
import json
import re
import zipfile
from weak_serial_guard import prove_weak_main

BOARDS=("RF1000","RF2000","RF2000v2","RF2000-X44")

from system_register_sites import register_site


def target(arg):
    m=re.search(r";\s*0x([0-9a-f]+)\s*<",arg) or re.match(r"0x([0-9a-f]+)",arg)
    if not m: raise ValueError(arg)
    return int(m[1],16)

def signed16(n):
    return n-65536 if n>=32768 else n

def decode_linked(s,board):
    labels={int(m[1],16):m[2] for m in re.finditer(r"^([0-9a-f]+) <(.*)>:$",s,re.M)}
    ins,raw={},{}
    for line in s.splitlines():
        m=re.match(r"^\s*([0-9a-f]+):\t((?:[0-9a-f]{2} )+)\s*\t([.a-z][a-z0-9.]*)\s*(.*?)$",line)
        if m:
            a=int(m[1],16);bs=bytes.fromhex(m[2])
            ins[a]=(len(bs),m[3],m[4])
            raw.update({a+i:v for i,v in enumerate(bs)})
    # Same compiler table candidate inventory as foreground-cfg-inventory.
    # The CPI/range control-flow invariants still need review for release.
    tables={}
    for j,(w,op,arg) in list(ins.items()):
        if op not in ("jmp","rjmp") or "<__tablejump2__>" not in arg:continue
        prev=[ins.get(j-k,(0,"","")) for k in (6,4,2)]
        m=[re.match(r"r(?:30|31|24), 0x([0-9A-Fa-f]+)",i[2]) for i in prev]
        assert [i[1] for i in prev]==["subi","sbci","sbci"] and all(m),(board,hex(j),prev)
        base=((-sum(int(x[1],16)<<(i*8) for i,x in enumerate(m)))&0xffffff)*2
        assert base==j+w,(board,hex(j),hex(base))
        c=[(a,re.match(r"r\d+, 0x([0-9A-Fa-f]+)",ins[a][2]))
           for a in range(j-28,j-6,2) if a in ins and ins[a][1]=="cpi"]
        assert c and c[-1][1],(board,hex(j),"count")
        count=int(c[-1][1][1],16)
        end=base+2*count
        if end not in ins:
            # Linear objdump may interpret the final table word as the
            # first word of a four-byte instruction and consume the real
            # first instruction of the following case. Recover only the
            # witnessed two-byte LDI encoding; reject any other case.
            assert end-2>=base and end-2 in ins and ins[end-2][0]==4
            assert end+2 in ins, (board,hex(end),"missing table boundary")
            word=raw[end]+256*raw[end+1]
            assert word & 0xf000 == 0xe000, (board,hex(end),hex(word))
            reg=16+((word>>4)&15)
            imm=((word>>4)&0xf0)|(word&15)
            ins[end]=(2,"ldi",f"r{reg}, 0x{imm:02X}")
        ts=[2*(raw[base+i*2]+256*raw[base+i*2+1]) for i in range(count)]
        assert all(t in ins for t in ts),(board,hex(j),"target")
        tables[j]=ts
    return ins,tables,labels

def inventory(z,board,canonical=False,trace=None):
    p=board+"/" if canonical else "host/experimental-avr/emergency/"+board+"/"
    s=z.read(p+"disassembly.txt").decode()
    ins,tables,labels=decode_linked(s,board)
    syms={int(m[1],16):m[3] for m in re.finditer(
        r"^([0-9a-f]+) ([0-9a-f]+) [tTwW] (.+)$",z.read(p+"symbols.txt").decode(),re.M)}
    def name(a):return labels.get(a,syms.get(a,hex(a)))
    label_addrs=sorted(labels)
    def physical_name(pc):
        i=bisect.bisect_right(label_addrs,pc)-1
        assert i>=0,(board,hex(pc),"missing physical label")
        return labels[label_addrs[i]]
    # Source inventory: loadCompensationMatrix's optional callback is the
    # startup boot-eye lambda or null. emergencyStop's indirect call is reset.
    boot=next(a for a,n in labels.items() if n.startswith("setupForPrinting()::{lambda()#1}::_FUN"))
    main=next(a for a,n in labels.items() if n=="main")
    # Re-evaluate the weak Arduino serialEventRun guard in each image.
    # An unexpected reachable CALL 0 is still an error, not a reset exception.
    weak_guard_errors=[]
    try:
        weak_zero=prove_weak_main(ins,tables,main,target)
    except (ValueError, KeyError) as exc:
        weak_zero=set()
        weak_guard_errors.append(str(exc))
    system_sites = set()
    pending=[main,boot];seen=set();local={};errors=set()
    edges=collections.defaultdict(dict)
    tail_transfers=set()
    while pending:
        root=pending.pop()
        if root in seen:continue
        seen.add(root)
        # depth includes three-byte return PC. Y is an affine offset from
        # entry SP, or unknown. ABI calls preserve Y; a compiler-generated
        # paired immediate adjustment is interpreted as a signed 16-bit delta.
        entry=(root,3,None,None)
        todo=[entry];visited=set();peak=3
        local_edges = collections.defaultdict(set) if trace is not None else None
        local_calls = {} if trace is not None else None
        def add(next_state):
            if local_edges is not None:
                local_edges[state].add(next_state)
            todo.append(next_state)
        while todo:
            state=todo.pop()
            pc,d,y,pair=state
            if state in visited:continue
            visited.add(state);peak=max(peak,d)
            if local_edges is not None:
                local_edges[state]  # terminal states also belong to the graph
            if len(visited)>200000 or not 0<=d<=2048:
                errors.add((name(root),hex(pc),"depth/state limit",d));break
            if pc not in ins:errors.add((name(root),hex(pc),"missing"));continue
            w,op,arg=ins[pc];n=pc+w;a=arg.split(";")[0].strip();toks=a.split(", ")
            site = register_site(pc, op, arg)
            if site:
                system_sites.add(site)
            if op.startswith("."):errors.add((name(root),hex(pc),"data"));continue
            if op=="push":d+=1
            if op=="pop":
                d-=1
                if a in ("r28","r29"):y=None;pair=None
            if op=="in" and a=="r28, 0x3d":y=None;pair=("sp",d,n)
            elif op=="in" and a=="r29, 0x3e":
                y=-d if pair==("sp",d,pc) else None;pair=None
            elif op=="subi" and toks[0]=="r28":
                pair=("sub",int(toks[1],0),n) if y is not None else None
            elif op in ("sbci","sbc") and toks[0]=="r29":
                if pair and pair[0]=="sub" and pair[2]==pc and y is not None and (op=="sbci" or toks[1]=="r1"):
                    y-=signed16(pair[1]+(256*int(toks[1],0) if op=="sbci" else 0))
                else:y=None
                pair=None
            elif op in ("adiw","sbiw") and toks[0]=="r28":
                if y is not None:y+=(1 if op=="adiw" else -1)*int(toks[1],0)
                pair=None
            elif op=="out" and a=="0x3d, r28":
                if y is None:errors.add((name(root),hex(pc),"unknown SP",d));continue
                d=-y
            elif op=="out" and a.startswith("0x3d,"):
                # Outgoing argument cleanup: exact contiguous compiler
                # SP snapshot, constant add, protected writeback sequence.
                reg=int(toks[1][1:]);lo=f"r{reg}";hi=f"r{reg+1}"
                seq=[ins.get(pc-2*k,(0,"","")) for k in range(8,0,-1)]
                ops=[i[1] for i in seq];args=[i[2].split(";")[0].strip() for i in seq]
                if (ops!=["in","in","subi","sbci","in","cli","out","out"] or
                    args[:2]!=[lo+", 0x3d",hi+", 0x3e"] or
                    not args[2].startswith(lo+", ") or not args[3].startswith(hi+", ") or
                    args[4:]!=["r0, 0x3f","", "0x3e, "+hi,"0x3f, r0"]):
                    errors.add((name(root),hex(pc),"unknown SP pair",a));continue
                d+=signed16(int(args[2].split(", ")[1],0)+256*int(args[3].split(", ")[1],0))
            elif op=="sts" and re.match(r"0x00(?:5[dDeE]),",a):
                errors.add((name(root),hex(pc),"direct SP store",a));continue
            elif toks and toks[0] in ("r28","r29") and op not in (
                "push","cp","cpc","cpse","tst","sbrc","sbrs","st","std","out","sts","pop"):
                y=None;pair=None
            if op in ("ld","st") and re.search(r"(?:^|, )(?:Y\+|-Y)(?:,|$)",a):
                y=None;pair=None
            if pc in tables:
                for t in tables[pc]:add((t,d,y,pair))
                continue
            if op in ("icall","eicall","ijmp","eijmp"):
                if name(root).startswith("Commands::emergencyStop()"):continue
                if name(root).startswith("loadCompensationMatrix(") and op in ("icall","eicall"):
                    t=boot
                else:errors.add((name(root),hex(pc),"indirect",op));continue
            elif op in ("call","rcall"):
                t=target(arg)
                if op=="rcall" and t==n:add((n,d+3,y,pair));continue
            else:t=None
            if t is not None:
                if t==0:
                    if root==main and pc in weak_zero:
                        continue
                    if not name(root).startswith(("Commands::emergencyStop","HAL::resetHardware")):
                        errors.add((name(root),hex(pc),"unexpected reset call"))
                    continue
                pending.append(t)
                edges[root,t][pc]=max(d,edges[root,t].get(pc,0))
                if local_calls is not None:
                    local_calls[state]=t
                add((n,d,y,None));continue
            if op in ("jmp","rjmp"):
                t=target(arg)
                if t==0:
                    if not name(root).startswith(("Commands::emergencyStop","HAL::resetHardware")):
                        errors.add((name(root),hex(pc),"unexpected reset jump"))
                    continue
                if t in labels and physical_name(pc)!=labels[t]:
                    tail_transfers.add((root,pc,t))
                # Tail jumps share the current frame (including aliases and
                # library helper entry labels), not another return address.
                add((t,d,y,pair));continue
            if op in ("ret","reti","break"):
                if d!=3 and op=="ret":errors.add((name(root),hex(pc),"return depth",d))
                continue
            if op.startswith("br"):add((target(arg),d,y,pair))
            if op in ("sbrc","sbrs","sbic","sbis","cpse"):
                if n not in ins:errors.add((name(root),hex(pc),"skip missing"));continue
                add((n+ins[n][0],d,y,pair))
            add((n,d,y,pair))
        if trace is not None:
            trace[root]={"entry":entry,"edges":local_edges,"calls":local_calls}
        local[root]=peak
    result={"board":board,"tables":len(tables),"functions":len(seen),"edges":len(edges),
            "errors":sorted(errors),"main":hex(main), "explicitSystemRegisterSites": sorted(system_sites),
            "provenUnreachableWeakCalls":[hex(pc) for pc in sorted(weak_zero)],
            "weakGuardDiagnostics":weak_guard_errors,
            "frames":[{"address":hex(a),"name":name(a),"localPeakBytes":d} for a,d in sorted(local.items())],
            "tailTransfers":[{"root":hex(a),"pc":hex(pc),"target":hex(t),
                               "physicalCaller":physical_name(pc),"physicalTarget":labels[t]}
                              for a,pc,t in sorted(tail_transfers)],
            "calls":[{"caller":hex(a),"callee":hex(b),
                      "sites":[{"pc":hex(pc),"callerBytes":d,"physicalCaller":physical_name(pc)} for pc,d in sorted(xs.items())]}
                     for (a,b),xs in sorted(edges.items())],
            "limitations":["candidate jump-table range checks, callback/reset source assumptions",
                           "ABI and compiler atomic SP-write sequences assumed",
                           "CRT before main, feasible foreground recursion and IRQ response not bounded"]}
    return result

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive")
    parser.add_argument("sha256")
    parser.add_argument("board",choices=BOARDS)
    parser.add_argument("--canonical",action="store_true")
    args=parser.parse_args()
    with open(args.archive,"rb") as f:assert hashlib.sha256(f.read()).hexdigest()==args.sha256
    with zipfile.ZipFile(args.archive) as z:result=inventory(z,args.board,args.canonical)
    print(json.dumps(result,indent=2))
    if result["errors"]:raise SystemExit(1)

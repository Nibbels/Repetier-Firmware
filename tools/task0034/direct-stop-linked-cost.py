#!/usr/bin/env python3
"""A73 direct-stop-tail linked body costs, diagnostic not release proof.

Exact call/return paths and taken/not-taken AVR branch costs for the
acyclic linked stop-tail helper. The restricted cost excludes entry to
the historical generic fitter; do not use it as WCET without proving
the direct prepared-profile invariant. No caller/interrupt entry cost.
"""
import argparse
import functools
import hashlib
import json
import re
import zipfile

BOARDS=("RF1000","RF2000","RF2000v2","RF2000-X44")
SKIP=("sbrc","sbrs","sbic","sbis","cpse")
def cost(op,branch=None,skip_width=0):
    if op in ("call","ret","reti","eicall"):return 5
    if op in ("rcall","icall"):return 4
    if op in SKIP:return 1+skip_width//2 if branch else 1
    if op.startswith("br") and op!="break":return 2 if branch else 1
    if op in ("jmp","lpm","elpm"):return 3
    if op in ("rjmp","ijmp","eijmp","ld","ldd","lds","st","std","sts","push","pop",
              "sbi","cbi","adiw","sbiw","mul","muls","mulsu","fmul","fmuls","fmulsu"):return 2
    if op not in ("adc","add","and","andi","asr","bld","bst","com","cp","cpc","cpi",
                  "dec","eor","in","inc","ldi","lsl","lsr","mov","movw","neg","nop",
                  "or","ori","out","rol","ror","sbc","sbci","sub","subi","swap","clr",
                  "ser","tst","clc","sec","clt","set","cli","sei","bclr","bset","wdr"):
        raise ValueError(("unreviewed opcode",op))
    return 1

def target(arg):
    m=re.search(r";\s*0x([0-9a-f]+)\s*<",arg) or re.match(r"0x([0-9a-f]+)",arg)
    if not m:raise ValueError(arg)
    return int(m[1],16)

def inventory(z,board):
    p=board+"/"
    assert json.loads(z.read(p+"verification.json"))["status"]=="verified"
    s=z.read(p+"disassembly.txt").decode()
    labels={m[2]:int(m[1],16) for m in re.finditer(r"^([0-9a-f]+) <(.*)>:$",s,re.M)}
    ins={int(m[1],16):(len(bytes.fromhex(m[2])),m[3],m[4]) for m in
         filter(None,(re.match(r"^\s*([0-9a-f]+):\t((?:[0-9a-f]{2} )+)\s*\t([.a-z][a-z0-9.]*)\s*(.*?)$",line) for line in s.splitlines()))}
    roots=[v for k,v in labels.items() if k.startswith("PrintLine::limitToStopTail(bool)")]
    general={v for k,v in labels.items() if k.startswith("PrintLine::fitRampToRemainingSteps()")}
    assert len(roots)==len(general)==1,(board,roots,general)
    root=roots[0]
    active=set()
    @functools.lru_cache(None)
    def visit(pc,ret,allow_general):
        state=(pc,ret,allow_general)
        if pc in general and not allow_general:return -1000000,()
        if state in active:raise ValueError(("cycle",hex(pc)))
        active.add(state)
        w,op,arg=ins[pc];n=pc+w
        if op in ("ret","reti"):
            assert op=="ret"
            choices=[(ret[-1],ret[:-1],cost(op))] if ret else []
        elif op in ("jmp","rjmp"):choices=[(target(arg),ret,cost(op))]
        elif op in ("call","rcall"):choices=[(target(arg),ret+(n,),cost(op))]
        elif op.startswith("br") and op!="break":
            choices=[(n,ret,cost(op,False)),(target(arg),ret,cost(op,True))]
        elif op in SKIP:
            choices=[(n,ret,cost(op,False)),(n+ins[n][0],ret,cost(op,True,ins[n][0]))]
        elif op.startswith(".") or op in ("icall","eicall","ijmp","eijmp","break"):
            raise ValueError((hex(pc),op))
        else:choices=[(n,ret,cost(op))]
        if not choices:result=(cost(op),(pc,))
        else:
            result=max(((v+c,(pc,)+path) for q,r,c in choices
                        for v,path in [visit(q,r,allow_general)]),key=lambda x:x[0])
        active.remove(state)
        return result
    full=visit(root,(),True);fast=visit(root,(),False)
    assert full[0]>0 and fast[0]>0
    return {"entry":hex(root),"unrestrictedBodyCycles":full[0],
            "withoutGenericFitterDiagnosticCycles":fast[0],
            "unrestrictedWitness":[hex(x) for x in full[1]],
            "restrictedWitness":[hex(x) for x in fast[1]]}

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive")
    parser.add_argument("sha256")
    args=parser.parse_args()
    with open(args.archive,"rb") as f:assert hashlib.sha256(f.read()).hexdigest()==args.sha256
    with zipfile.ZipFile(args.archive) as z:
        summary=json.loads(z.read("summary.json"))
        boards={b:inventory(z,b) for b in BOARDS}
    print(json.dumps({"testedMerge":summary["sha"],"sha256":args.sha256,
                      "ciFailures":summary["failures"],"boards":boards,
                      "releaseProof":False,
                      "limitations":["ordinary branches unconstrained",
                                     "restricted path assumes no generic fitter entry",
                                     "no caller or global interrupt response"]},indent=2))

#!/usr/bin/env python3
"""A73: narrow foreground callsites for the five linked flash scripts.

Verify actual script strings/callers first. Follow the linked G/M switch
selection with concrete command numbers, then conservatively traverse all
handler branches. No source custom-script prohibition or whole-stack proof.
Requires the reviewed AVR GCC layout/ABI and parser/admission contract.
An unrecognized selector or transfer fails closed.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import sys
import zipfile

sys.dont_write_bytecode=True
def module(name,filename):
    spec=importlib.util.spec_from_file_location(name,Path(__file__).with_name(filename))
    m=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m

fg=module("foreground_depth","foreground-stack-depth.py")
scripts=module("linked_scripts","linked-script-context.py")
target=fg.target

def args(ins,p):
    return ins[p][1],ins[p][2].split(';')[0].strip()

def walk(ins,tables,start,stops=()):
    seen=set();todo=[start]
    while todo:
        pc=todo.pop()
        if pc in seen:continue
        seen.add(pc)
        if pc in stops:continue
        w,op,arg=ins[pc];n=pc+w
        if pc in tables:todo.extend(tables[pc]);continue
        if op in ('ret','reti','break'):continue
        if op in ('jmp','rjmp'):
            t=target(arg)
            if t:todo.append(t)
            continue
        if op in ('icall','eicall','ijmp','eijmp') or op.startswith('.'):
            raise ValueError((hex(pc),op))
        if op in ('call','rcall') and target(arg)==0:
            raise ValueError(("unreviewed zero call in script dispatch",hex(pc)))
        if op.startswith('br'):todo.append(target(arg))
        if op in ('sbrc','sbrs','sbic','sbis','cpse'):todo.append(n+ins[n][0])
        todo.append(n)
    return seen

def switch_trace(ins,start,code):
    # Initial register values at the verified G/M load. Stop at first
    # instruction not modeled here; all later branches are overapproximated.
    r={1:0,24:code&255,25:code>>8};f={};seen=set();pc=start
    def val(x):return r.get(int(x[1:])) if x.startswith('r') else int(x,0)
    while pc not in seen and len(seen)<256:
        seen.add(pc)
        w,op,arg=ins[pc];a=arg.split(';')[0].strip();t=a.split(', ');n=pc+w
        if op=='ldi':r[int(t[0][1:])]=val(t[1])
        elif op in ('cpi','cp','cpc','subi','sbci','sub','sbc'):
            x,y=map(val,t)
            carry=f.get('C') if op in ('cpc','sbc','sbci') else 0
            if x is None or y is None or carry is None:break
            u=x-y-carry;v=u&255;z=v==0
            ov=not -128<=(x if x<128 else x-256)-(y if y<128 else y-256)-carry<=127
            if op in ('cpc','sbc','sbci'):
                if 'Z' not in f:break
                z=z and f['Z']
            f={'C':u<0,'Z':z,'N':bool(v&128),'V':ov,'S':bool(v&128)^ov}
            if op in ('subi','sbci','sub','sbc'):r[int(t[0][1:])]=v
        elif op in ('sbiw','adiw'):
            i=int(t[0][1:]);lo=r.get(i);hi=r.get(i+1)
            if lo is None or hi is None:break
            old=lo|(hi<<8);u=old+val(t[1])*(1 if op=='adiw' else -1);v=u&65535
            ov=(old<32768 and v>=32768) if op=='adiw' else (old>=32768 and v<32768)
            f={'C':u>65535 if op=='adiw' else u<0,'Z':v==0,'N':bool(v&32768),'V':ov,'S':bool(v&32768)^ov}
            r[i]=v&255;r[i+1]=v>>8
        elif op in ('jmp','rjmp'):
            n=target(arg)
            if not n:raise ValueError(("unexpected reset during dispatch",hex(pc)))
        elif op.startswith('br'):
            cond={'breq':('Z',True),'brne':('Z',False),'brcs':('C',True),
                  'brlo':('C',True),'brcc':('C',False),'brsh':('C',False),
                  'brge':('S',False),'brlt':('S',True),'brmi':('N',True),'brpl':('N',False)}
            if op not in cond:break
            k,v=cond[op]
            if k not in f:break
            if f[k]==v:n=target(arg)
        else:break
        pc=n
    else:raise ValueError(('switch did not converge',code,hex(pc)))
    return seen,pc

def context(dump,board,commands):
    ins,tables,labels=fg.decode_linked(dump,board)
    by_name={name:a for a,name in labels.items()}
    root=by_name["Commands::executeGCode(GCode*)"]
    machine=by_name["GCodeCommands::(anonymous namespace)::dispatchMachine(GCode*)"]
    # Exact selector and register-load sequence for the witnessed compiler.
    match=re.search(r'^    if \(com->hasG\(\)\) \{\n\s*([0-9a-f]+):',dump,re.M)
    assert match
    g=int(match[1],16)
    assert args(ins,g)==('sbrs','r16, 2') and ins[g+2][1]=='rjmp'
    m=target(ins[g+2][2])
    assert args(ins,m)==('sbrs','r16, 1') and ins[m+2][1]=='rjmp'
    gs=g+4
    assert args(ins,gs)==('ldd','r24, Y+8') and args(ins,gs+2)==('ldd','r25, Y+9')
    loads=[a for a in range(machine,machine+128,2) if a in ins and
           args(ins,a)==('movw','r26, r24') and args(ins,a+2)==('adiw','r26, 0x06') and
           args(ins,a+4)==('ld','r24, X+') and args(ins,a+6)==('ld','r25, X')]
    assert len(loads)==1
    ms=loads[0]
    pre=walk(ins,tables,root,{g});assert g in pre
    machinepre=walk(ins,tables,m+4,{ms});assert ms in machinepre
    contexts={}
    for c in sorted(commands):
        isg=c[0]=='G';start=gs+4 if isg else ms+8
        trace,body=switch_trace(ins,start,int(c[1:]))
        reach=walk(ins,tables,body)
        pcs=pre|trace|reach|({gs,gs+2} if isg else machinepre|{ms+2,ms+4,ms+6})
        contexts[c]={"dispatchBody":hex(body),"reachable":pcs}
    return root,contexts

def inventory(z,board):
    p=board+"/"
    s=scripts.inventory(z,board)
    assert not s["errors"],(board,s["errors"])
    source=z.read(p+"build/sketch/GCodeInput.cpp").decode()
    for anchor in ("command.params = 0;", "command.G = parseLongValue(pos) & 0xffff;",
                   "command.M = parseLongValue(pos) & 0xffff;", "command.params |= 4;",
                   "command.params |= 2;", "parseAscii(code, buf, false)",
                   "Commands::executeGCode(&code);"):
        assert anchor in source,(board,"parser contract changed",anchor)
    admission=z.read(p+"build/sketch/GCodeCommands.cpp").decode()
    assert "if (com->hasG()) {" in admission and "} else if (com->hasM())" in admission
    command_set={c for sc in s["scripts"].values() for c in sc["commands"]}
    root,contexts=context(z.read(p+"disassembly.txt").decode(),board,command_set)
    frame=fg.inventory(z,board,True)
    assert not frame["errors"],frame["errors"]
    all_sites=[(x["callee"],site) for x in frame["calls"] if int(x["caller"],16)==root for site in x["sites"]]
    names={f["address"]:f["name"] for f in frame["frames"]}
    for c,v in contexts.items():
        reachable=v.pop("reachable")
        v["callSites"]=[dict(site,callee=callee,name=names[callee]) for callee,site in all_sites if int(site["pc"],16) in reachable]
        v["reachableInstructions"]=len(reachable)
        # Regression sentinel: no supported script enters unrestricted
        # panel, output-object, StartLine, home-axis or scan dispatch.
        forbidden=("UIDisplay::executeAction(", "outputObject(", "StartLine::run(",
                   "Printer::homeAxis(", "Printer::homeYAxis(", "Printer::checkHome(",
                   "GCodeCommands::Calibration::configureZOffsetScan(")
        assert not any(x["name"].startswith(forbidden) for x in v["callSites"]),(board,c)
    per_script={}
    for name,sc in s["scripts"].items():
        per_script[name]=sorted({site["pc"] for c in sc["commands"] for site in contexts[c]["callSites"]},key=lambda p:int(p,16))
    return {"dispatcher":hex(root),"commands":contexts,"scriptCallSites":per_script,
            "allDispatcherCallSites":len(all_sites),
            "unionScriptCallSites":len({pc for sites in per_script.values() for pc in sites})}

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive")
    parser.add_argument("sha256")
    args=parser.parse_args()
    with open(args.archive,"rb") as f:assert hashlib.sha256(f.read()).hexdigest()==args.sha256
    with zipfile.ZipFile(args.archive) as z:
        summary=json.loads(z.read("summary.json"))
        assert not summary["failures"]
        boards={b:inventory(z,b) for b in fg.BOARDS}
    print(json.dumps({"testedMerge":summary["sha"],"sha256":args.sha256,
                      "boards":boards,
                      "limitations":["reviewed parser/admission field ownership and ABI assumed",
                                     "candidate jump tables from frame inventory",
                                     "all handler branches after initial switch retained",
                                     "not a whole-program stack proof"]},indent=2))

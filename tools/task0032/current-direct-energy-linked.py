#!/usr/bin/env python3
"""Measure isolated real AVR direct-energy Timer1 calls and STEP spacings.

Not a concurrent peripheral simulation or a mechanical/thermal release gate.
Unmet spacing or CPU budget is reported as evidence against selection.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import re
import subprocess
import tempfile

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("stop_linked",HERE/"current-stop-tail-linked.py")
sl=importlib.util.module_from_spec(spec);spec.loader.exec_module(sl)

FAST_PROFILE=False
COMPACT=False

def checked_layout(elf):
    # The old validator returns its own mutable member dictionary. Never
    # rename a key through that alias (it poisoned the next board in #229).
    # Independently validate the anonymous union's referenced DIE and both
    # alternatives; do not accept an unrelated type named Limits.
    import copy
    saved=sl.LAYOUT
    try:
        sl.LAYOUT=copy.deepcopy(saved)
        fields=sl.LAYOUT["PrintLine"][1]
        assert fields.pop("lookahead")==28
        fields[None]=28
        if COMPACT:
            for k in ("stepProfile","stepsRemaining","task"):
                fields[k]-=4
            sl.LAYOUT["PrintLine"]=(99,fields)
        layout=copy.deepcopy(original_layout(elf))
    finally:
        sl.LAYOUT=saved
    raw=subprocess.check_output(["readelf","--debug-dump=info",str(elf)],text=True)
    pattern=re.compile(r"^\s*<(\d+)><([0-9a-f]+)>: Abbrev Number: \d+(?: \((DW_TAG_\w+)\))?",re.M)
    blocks=list(pattern.finditer(raw))
    nodes={};stack=[]
    for i,b in enumerate(blocks):
        depth=int(b[1]);stack=stack[:depth]
        if b[3] is None:continue
        body=raw[b.end():blocks[i+1].start() if i+1<len(blocks) else len(raw)]
        attrs=dict(re.findall(r"DW_AT_(\w+)\s*:\s*([^\n]+)",body))
        node={"tag":b[3],"attrs":attrs,"children":[]}
        offset=int(b[2],16);nodes[offset]=node
        if stack:stack[-1]["children"].append(node)
        stack.append(node)
    def name(n):
        return n["attrs"].get("name","").rsplit(": ",1)[-1]
    def target(n):
        return nodes[int(re.fullmatch(r"<0x([0-9a-f]+)>",n["attrs"]["type"])[1],16)]
    def size(n):
        return int(n["attrs"]["byte_size"],0)
    def location(n):
        v=n["attrs"]["data_member_location"]
        m=re.search(r"DW_OP_plus_uconst: (\d+)",v)
        assert m or v.isdecimal(),v
        return int(m[1] if m else v)
    expected={"ceiling":0,"exitEnergy":4,"drop":8,"horizon":12,"valid":16}
    if FAST_PROFILE:expected["plateauTicks"]=17
    limit_size=21 if FAST_PROFILE else 17
    matches=0
    for n in nodes.values():
        if n["tag"]!="DW_TAG_structure_type" or name(n)!="PrintLine" or "byte_size" not in n["attrs"]:continue
        members=[c for c in n["children"] if c["tag"]=="DW_TAG_member" and "data_member_location" in c["attrs"]]
        if not members:continue
        assert size(n)==(99 if COMPACT else 103)
        anonymous=[c for c in members if not name(c)]
        assert len(anonymous)==1 and location(anonymous[0])==28
        union=target(anonymous[0])
        assert union["tag"]=="DW_TAG_union_type" and size(union)==(44 if COMPACT else 48)
        alternatives={name(c):c for c in union["children"] if c["tag"]=="DW_TAG_member"}
        assert set(alternatives)=={"lookahead","directProfile"}
        for c in alternatives.values():
            assert "data_member_location" not in c["attrs"] or location(c)==0
        lookahead=target(alternatives["lookahead"])
        limits=target(alternatives["directProfile"])
        assert name(lookahead)=="LookaheadData" and size(lookahead)==(44 if COMPACT else 48)
        if COMPACT:
            la={name(c):location(c) for c in lookahead["children"] if c["tag"]=="DW_TAG_member" and "data_member_location" in c["attrs"]}
            assert la=={k:4*i for i,k in enumerate(("speedX","speedY","speedZ","speedE","accelerationDistance2","maxJunctionSpeed","startSpeed","endSpeed","minSpeed","distance","accelerationPrim"))},la
        assert name(limits)=="Limits" and size(limits)==limit_size
        actual={name(c):location(c) for c in limits["children"]
                if c["tag"]=="DW_TAG_member" and "data_member_location" in c["attrs"]}
        assert actual==expected,actual
        matches+=1
    assert matches,("no linked PrintLine energy union",str(elf))
    layout["PreparedDirectProfile::Limits"]={"size":limit_size,"members":expected,"verified_references":matches}
    layout["PrintLine"]["members"]["anonymous_union"]=layout["PrintLine"]["members"].pop(None)
    assert sl.LAYOUT["PrintLine"][1]["lookahead"]==28
    return layout

original_layout=sl.dwarf_layout
sl.dwarf_layout=checked_layout

def measure(mod,img,delta,rate,start,acceleration,stop_after,negative=False):
    remaining=max(delta);primary="XYZ"[delta.index(remaining)]
    c,pc,addr,errors,o=mod.setup(img,owner=0,delta=delta,rate=rate,n=5,
        phase="cruise",advance=False,ratio=0,zcor=0,negative=negative,remaining=remaining)
    assert img.sizes["PrintLine::direct"]==(99 if COMPACT else 103) and img.sizes["PrintLine::queue"]==(2381 if COMPACT else 2477)
    assert o==(-4 if COMPACT else 0) and img.sizes["StepExecutor::stepClocks"]==20
    sign=-1 if negative else 1
    c.put(addr+102+o,5)
    c.symput("motorWake",7,1,4);c.symput("motorWake",7,1,5)
    for a in range(2):c.symput("Printer::xyMotion",delta[a]*sign,4,20+a*8)
    c.symput("Printer::zMotion",delta[2]*sign,4,24)
    floor=start*start;ceiling=rate*rate;drop=2*acceleration
    for offset,value in ((28,ceiling),(32,floor),(36,drop),
        (40,(ceiling-floor+drop-1)//drop)):c.put(addr+offset,value,4)
    c.put(addr+44,1)
    if FAST_PROFILE:c.put(addr+45,(16000000+rate-1)//rate,4)
    c.put(addr+88+o,rate,2);c.put(addr+90+o,start,2);c.put(addr+92+o,start,2)
    total={a:0 for a in "XYZ"};epoch=0;last_edge=None;previous_energy=floor
    armed_energy=floor;short_gaps=late_calls=0;worst_short=0
    max_cycles=max_mask=peak_energy=tail=0;request_energy=None;request_pulses=None
    shortest=None;max_overshoot=0;calls=0
    peak_stack=0;mask_witness=None;costs={};high_min={};low_min={};falls={}
    cruise_samples=0;cruise_elapsed=0;cruise_cost=0
    assert stop_after is None or stop_after<remaining
    while c.get(addr+102+o):
        assert calls<remaining*5+100,(img.board,calls,remaining,c.get(addr+102+o))
        if stop_after is not None and request_pulses is None and total[primary]>=stop_after and c.get(addr+102+o) in (6,7):
            request_energy=c.symget("StepExecutor::stepClocks",4,0)
            request_pulses=total[primary]
            c.put(addr+102+o,c.get(addr+102+o)+2)
        old_phase=c.get(addr+102+o);old_energy=armed_energy
        c.timeline=[];c.edges=[];c.calls=[];c.timer_writes=[]
        c.cy=5;c.timer_live=True;c.sreg=0;c.mem[0x6f]=2
        c.timer0_phase=0;c.timer0_reads=0;c.timer0_flag_cleared_at_wrap=0
        c.run(17*4);calls+=1
        assert c.sp==0x21ff and not c.return_stack
        mask=sl.lt.windows(c.timeline,c.cy)
        longest=max(mask,key=lambda x:x[1]-x[0],default=(0,0))
        if longest[1]-longest[0]>max_mask:
            max_mask=longest[1]-longest[0]
            boundaries=[(t,hex(pc),img.labels.get(pc,""),depth) for t,pc,i,ocie,sp,depth in c.timeline
                        if longest[0]<=t<=longest[1]]
            mask_witness={"call":calls,"phase":old_phase,"range":longest,
                          "start":boundaries[:3],"end":boundaries[-3:],"calls":c.calls}
        peak_stack=max(peak_stack,0x21ff-c.minsp)
        max_cycles=max(max_cycles,c.cy)
        period=c.get(0x88,2)+1
        if c.cy>=period:
            late_calls+=1
            assert not c.get(addr+102+o),("late active compare outside isolated timebase",img.board,c.cy,period)
        # The next event's actual edge phase can differ from this one.
        for axis in "XYZ":
            states=[s for p,s,t in c.edges if p==axis]
            assert states==[1,0]*(len(states)//2)
            total[axis]+=sum(states)
            rise=None
            for p,state,t in c.edges:
                if p!=axis:continue
                t+=epoch
                if state:
                    if axis in falls:low_min[axis]=min(low_min.get(axis,1<<32),t-falls[axis])
                    rise=t
                else:
                    assert rise is not None
                    high_min[axis]=min(high_min.get(axis,1<<32),t-rise)
                    falls[axis]=t
        times=[epoch+t for p,s,t in c.edges if p==primary and s]
        assert len(times)<=1
        if times:
            if last_edge is not None:
                gap=times[0]-last_edge
                shortest=gap if shortest is None else min(shortest,gap)
                ideal=math.ceil(32_000_000/(math.sqrt(previous_energy)+math.sqrt(armed_energy)))
                if gap<ideal:
                    short_gaps+=1;worst_short=max(worst_short,ideal-gap)
                max_overshoot=max(max_overshoot,gap-ideal)
            last_edge=times[0];previous_energy=armed_energy
        if c.get(addr+102+o):
            armed_energy=c.symget("StepExecutor::stepClocks",4,0)
            root=c.symget("StepExecutor::stepClocks",2,8)
            assert root==math.isqrt(armed_energy)
            assert floor<=armed_energy<=ceiling
            peak_energy=max(peak_energy,armed_energy)
        for i,axis in enumerate("XYZ"):
            if i<2:
                motor=c.symget("Printer::xyMotion",4,8+i*4,signed=True)
                direct=c.symget("Printer::xyMotion",4,16+i*8,signed=True)
                nominal=c.symget("Printer::xyMotion",4,i*4,signed=True)
            else:
                motor=c.symget("Printer::zMotion",4,0,signed=True)
                direct=c.symget("Printer::zMotion",4,20,signed=True)
                nominal=c.symget("Printer::zMotion",4,4,signed=True)
            assert motor==1000+sign*total[axis] and direct==sign*total[axis] and nominal==1000
        assert c.symget("StepExecutor::stepClocks",4,4)==total[primary]
        kind=("startup" if old_phase==5 else "retire" if not c.get(addr+102+o)
              else "wait" if not times else "cruise" if old_energy==armed_energy==ceiling
              else "accel" if armed_energy>old_energy else "brake" if armed_energy<old_energy else "flat")
        data=costs.setdefault(kind,{"calls":0,"cycles_max":0,"cycles_sum":0})
        data["calls"]+=1;data["cycles_max"]=max(data["cycles_max"],c.cy);data["cycles_sum"]+=c.cy
        if kind=="cruise":
            cruise_samples+=1;cruise_elapsed+=period;cruise_cost+=c.cy
        epoch+=period
    if request_pulses is None:assert total[primary]==remaining
    else:
        tail=total[primary]-request_pulses
        assert tail<=1+(request_energy-floor+drop-1)//drop
    assert c.get(addr+98+o,4,signed=True)==0
    return dict(delta=delta,rate=rate,start=start,acceleration=acceleration,stop_after=stop_after,
        negative=negative,calls=calls,pulses=total,tail=tail,max_cycles=max_cycles,max_mask=max_mask,
        short_gaps=short_gaps,worst_short_ticks=worst_short,shortest_gap_ticks=shortest,
        max_excess_ticks=max_overshoot,late_calls=late_calls,peak_energy=peak_energy,
        stack_bytes=peak_stack,mask_witness=mask_witness,costs=costs,high_min=high_min,low_min=low_min,
        cruise_samples=cruise_samples,cruise_elapsed=cruise_elapsed,cruise_cost=cruise_cost,
        elapsed_ticks=epoch,request_energy=request_energy)

def vector_prefix(evidence, board):
    # This is a lower bound, not a sampled shortest pulse path. The
    # unconditional contiguous prologue executes before any STEP GPIO.
    # Reject unexpected instructions rather than count arbitrary code.
    path=evidence/board/"disassembly.txt"
    data=path.read_text()
    start=data.index("<__vector_17>:")
    cycles=0
    for line in data[start:].splitlines()[1:]:
        m=re.match(r"\s*[0-9a-f]+:\s+(?:[0-9a-f]{2} )+\s*(\w+)\b",line)
        if not m:continue
        op=m[1]
        if op not in ("push","in","eor"):break
        cycles+=2 if op=="push" else 1
    assert cycles>=51,("primary edge entry-prefix contract",board,cycles)
    return cycles

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--evidence",type=Path,required=True)
    p.add_argument("--out",type=Path,required=True)
    p.add_argument("--optimized",action="store_true")
    p.add_argument("--extended",action="store_true")
    p.add_argument("--phase-anchor",action="store_true")
    p.add_argument("--compact",action="store_true")
    a=p.parse_args()
    global FAST_PROFILE,COMPACT
    COMPACT=a.compact
    FAST_PROFILE=a.optimized
    mod=sl.lt.load_interpreter()
    result={"scope":__doc__,"boards":{}}
    with tempfile.TemporaryDirectory() as d:
        archive=Path(d)/"energy.zip"
        result["candidate_sha"],meta=sl.evidence(a.evidence,archive)
        for board in sl.lt.BOARDS:
            img=mod.Image(archive,board)
            rows=[]
            for delta,rate,start,accel in (((600,0,0,0),15238,3000,228570),
                ((0,0,600,0),11520,1000,128000),((128,63,29,0),10000,3000,228570)):
                for cut in (None,max(delta)//2):
                    rows.append(measure(mod,img,delta,rate,start,accel,cut))
            if a.extended:
                # A stop at cruise, NOT a short triangle already braking.
                for delta,rate,start,accel in (((2400,0,0,0),15238,990,228570),
                    ((0,0,2400,0),11520,192,128000),((1600,970,344,0),15238,990,228570)):
                    row=measure(mod,img,delta,rate,start,accel,800)
                    assert row["request_energy"]==rate*rate and row["tail"]<max(delta)-800,row
                    rows.append(row)
                rows.append(measure(mod,img,(600,0,0,0),15238,990,228570,50,negative=True))
            if a.phase_anchor:meta[board]["unconditional_vector_prefix_cycles"]=vector_prefix(a.evidence,board)
            result["boards"][board]={"metadata":meta[board],"paths":rows}
            print(board,"calls",sum(r["calls"] for r in rows),
                "max_cycles",max(r["max_cycles"] for r in rows),
                "max_mask",max(r["max_mask"] for r in rows),
                "short_gaps",sum(r["short_gaps"] for r in rows),flush=True)
            a.out.write_text(json.dumps(result,indent=2)+"\n")
        if a.phase_anchor:
            assert all(v>=31 for b in result["boards"].values() for row in b["paths"] for k in ("high_min","low_min") for v in row[k].values()), "electrical STEP minimum"
        if a.optimized:
            # Measured minimum is required, not just recorded.
            assert all(not row["short_gaps"] for b in result["boards"].values() for row in b["paths"]), "short physical primary gap"

if __name__=="__main__":main()

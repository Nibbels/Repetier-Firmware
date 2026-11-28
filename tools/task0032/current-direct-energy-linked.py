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

def checked_layout(elf):
    # GCC describes the anonymous union with an unnamed member at offset 28.
    fields=sl.LAYOUT["PrintLine"][1]
    fields.pop("lookahead",None);fields[None]=28
    layout=original_layout(elf)
    text=subprocess.check_output(["readelf","--debug-dump=info",str(elf)],text=True)
    types=re.split(r"(?m)^\s*<\d+><[0-9a-f]+>: Abbrev Number: \d+ \(DW_TAG_structure_type\)",text)
    expected={"ceiling":0,"exitEnergy":4,"drop":8,"horizon":12,"valid":16}
    matches=0
    for part in types[1:]:
        head=part.split("DW_TAG_member",1)[0]
        if not re.search(r"DW_AT_name\s*:.*\bLimits\s*$",head,re.M):continue
        if not re.search(r"DW_AT_byte_size\s*:\s*17\s*$",head,re.M):continue
        members={}
        for m in re.finditer(r"DW_TAG_member\)(.*?)(?=^\s*<\d+><[0-9a-f]+>: Abbrev Number:|\Z)",part,re.M|re.S):
            name=re.search(r"DW_AT_name\s*:\s*([^\n]+)",m[1])
            off=re.search(r"DW_OP_plus_uconst: (\d+)",m[1])
            if name and off:members[name[1].strip().rsplit(": ",1)[-1]]=int(off[1])
        if all(members.get(k)==v for k,v in expected.items()):matches+=1
    assert matches,("no linked energy limit layout",str(elf))
    layout["PreparedDirectProfile::Limits"]={"size":17,"members":expected}
    layout["PrintLine"]["members"]["anonymous_union"]=28
    layout["PrintLine"]["members"].pop(None,None)
    return layout

original_layout=sl.dwarf_layout
sl.dwarf_layout=checked_layout

def measure(mod,img,delta,rate,start,acceleration,stop_after,negative=False):
    remaining=max(delta);primary="XYZ"[delta.index(remaining)]
    c,pc,addr,errors,o=mod.setup(img,owner=0,delta=delta,rate=rate,n=5,
        phase="cruise",advance=False,ratio=0,zcor=0,negative=negative,remaining=remaining)
    assert img.sizes["PrintLine::direct"]==103 and img.sizes["PrintLine::queue"]==2477
    assert o==0 and img.sizes["StepExecutor::stepClocks"]==20
    sign=-1 if negative else 1
    c.put(addr+102,5)
    c.symput("motorWake",7,1,4);c.symput("motorWake",7,1,5)
    for a in range(2):c.symput("Printer::xyMotion",delta[a]*sign,4,20+a*8)
    c.symput("Printer::zMotion",delta[2]*sign,4,24)
    floor=start*start;ceiling=rate*rate;drop=2*acceleration
    for offset,value in ((28,ceiling),(32,floor),(36,drop),
        (40,(ceiling-floor+drop-1)//drop)):c.put(addr+offset,value,4)
    c.put(addr+44,1)
    c.put(addr+88,rate,2);c.put(addr+90,start,2);c.put(addr+92,start,2)
    total={a:0 for a in "XYZ"};epoch=0;last_edge=None;previous_energy=floor
    armed_energy=floor;short_gaps=late_calls=0;worst_short=0
    max_cycles=max_mask=peak_energy=tail=0;request_energy=None;request_pulses=None
    shortest=None;max_overshoot=0;calls=0
    while c.get(addr+102):
        assert calls<remaining*5+100,(img.board,calls,remaining,c.get(addr+102))
        if stop_after is not None and request_pulses is None and total[primary]>=stop_after and c.get(addr+102) in (6,7):
            request_energy=c.symget("StepExecutor::stepClocks",4,0)
            request_pulses=total[primary]
            c.put(addr+102,c.get(addr+102)+2)
        c.timeline=[];c.edges=[];c.calls=[];c.timer_writes=[]
        c.cy=5;c.timer_live=True;c.sreg=0;c.mem[0x6f]=2
        c.timer0_phase=0;c.timer0_reads=0;c.timer0_flag_cleared_at_wrap=0
        c.run(17*4);calls+=1
        assert c.sp==0x21ff and not c.return_stack
        mask=sl.lt.windows(c.timeline,c.cy)
        max_mask=max(max_mask,max((b-a for a,b in mask),default=0))
        max_cycles=max(max_cycles,c.cy)
        period=c.get(0x88,2)+1
        if c.cy>=period:late_calls+=1
        # The next event's actual edge phase can differ from this one.
        for axis in "XYZ":
            states=[s for p,s,t in c.edges if p==axis]
            assert states==[1,0]*(len(states)//2)
            total[axis]+=sum(states)
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
        if c.get(addr+102):
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
        epoch+=period
    if request_pulses is None:assert total[primary]==remaining
    else:
        tail=total[primary]-request_pulses
        assert tail<=1+(request_energy-floor+drop-1)//drop
    assert c.get(addr+98,4,signed=True)==0
    return dict(delta=delta,rate=rate,start=start,acceleration=acceleration,stop_after=stop_after,
        negative=negative,calls=calls,pulses=total,tail=tail,max_cycles=max_cycles,max_mask=max_mask,
        short_gaps=short_gaps,worst_short_ticks=worst_short,shortest_gap_ticks=shortest,
        max_excess_ticks=max_overshoot,late_calls=late_calls,peak_energy=peak_energy)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--evidence",type=Path,required=True)
    p.add_argument("--out",type=Path,required=True)
    a=p.parse_args()
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
            result["boards"][board]={"metadata":meta[board],"paths":rows}
            print(board,"calls",sum(r["calls"] for r in rows),
                "max_cycles",max(r["max_cycles"] for r in rows),
                "max_mask",max(r["max_mask"] for r in rows),
                "short_gaps",sum(r["short_gaps"] for r in rows),flush=True)
            a.out.write_text(json.dumps(result,indent=2)+"\n")

if __name__=="__main__":main()

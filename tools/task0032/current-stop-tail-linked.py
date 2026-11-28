#!/usr/bin/env python3
"""Actual linked Timer1 stop-tail measurement; not a global IRQ proof.

Reuse the pinned archived instruction interpreter, validate every assumed
layout against the actual ELF DWARF, and inject only the published byte
effect of a PWM/foreground stop request. No nested ISR execution is
simulated. The finite observed path set does not establish a universal
WCET or mechanical braking limit.
"""
import argparse
import collections
import hashlib
import itertools
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import zipfile

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"research/task0032-step-output"))
import linked_timer1 as lt

# All offsets used by the archived setup and our ledger assertions.
LAYOUT={
    "PrintLine":(103,{"joinFlags":0,"flags":1,"primaryAxis":2,"dir":7,
        "delta":8,"fullSpeed":24,"lookahead":28,"stepProfile":76,
        "stepsRemaining":98,"task":102}),
    "StepProfile":(22,{"accelSteps":0,"decelSteps":4,"fAcceleration":8,
        "vMax":12,"vStart":14,"vEnd":16,"advanceL":18}),
    "StepClock":(10,{"ticks":0,"steps":4,"peak":8}),
    "DirectPosition":(8,{"steps_":0,"target_":4}),
    "XYMotion":(36,{"nominalSteps_":0,"motorSteps_":8,"direct_":16,"skippedSteps_":32}),
    "ZMotion":(42,{"motorSteps_":0,"nominalSteps_":4,"queuedOvertravel_":8,
        "correctionTarget_":12,"appliedCorrection_":16,"direct_":20,
        "scanSteps_":28,"maxSteps_":32,"minOvertravel_":36,
        "queueWaitSteps_":38,"scanInProgress_":40,"homed_":41}),
}

def dwarf_layout(elf):
    raw=subprocess.check_output(["readelf","--debug-dump=info",str(elf)],text=True)
    die=re.compile(r"^\s*<(\d+)><[0-9a-f]+>: Abbrev Number: \d+(?: \((DW_TAG_\w+)\))?",re.M)
    blocks=list(die.finditer(raw))
    seen=collections.defaultdict(list)
    stack=[]
    def attr(body,key):
        match=re.search(r"DW_AT_"+key+r"\s*:\s*([^\n]+)",body)
        return match[1].strip() if match else None
    def name(body):
        value=attr(body,"name")
        return value.rsplit(": ",1)[-1] if value else None
    for i,b in enumerate(blocks):
        depth=int(b[1]);tag=b[2]
        stack=stack[:depth]
        body=raw[b.end():blocks[i+1].start() if i+1<len(blocks) else len(raw)]
        obj=None
        if tag in ("DW_TAG_class_type","DW_TAG_structure_type"):
            label=name(body);size=attr(body,"byte_size")
            if label in LAYOUT and size:
                obj={"size":int(size,0),"members":{}}
                seen[label].append(obj)
        if tag=="DW_TAG_member" and stack and stack[-1] is not None:
            location=attr(body,"data_member_location")
            label=name(body)
            if location is not None:
                m=re.search(r"DW_OP_plus_uconst: (\d+)",location)
                assert m or location.isdecimal(),(label,location)
                stack[-1]["members"][label]=int(m[1] if m else location)
        stack.append(obj)
    for name,(size,members) in LAYOUT.items():
        assert seen[name],("no linked DWARF type",name)
        # Some translation units contain incomplete declarations; exclude
        # only those without a body. Every complete body must agree.
        full=[x for x in seen[name] if x["members"]]
        assert full,name
        for record in full:
            assert record["size"]==size,(name,record)
            assert all(record["members"].get(k)==v for k,v in members.items()),(name,record)
    return {k:{"size":v[0],"members":v[1]} for k,v in LAYOUT.items()}

def evidence(path,archive):
    summary=json.loads((path/"summary.json").read_text())
    assert not summary["failures"] and summary["build_profile"]=="standard-recovery"
    assert set(summary["targets"])==set(lt.BOARDS)
    metadata={}
    with zipfile.ZipFile(archive,"w",compression=zipfile.ZIP_DEFLATED) as z:
        for board in lt.BOARDS:
            folder=path/board
            elf=folder/"build/Repetier.ino.elf"
            digest=hashlib.sha256(elf.read_bytes()).hexdigest()
            assert digest==summary["targets"][board]["elf_sha256"]
            metadata[board]={"elf_sha256":digest,"layout":dwarf_layout(elf)}
            for name in ("symbols.txt","disassembly.txt","build/Repetier.ino.hex"):
                data=(folder/name).read_bytes()
                metadata[board][name+"_sha256"]=hashlib.sha256(data).hexdigest()
                z.writestr(board+"/"+name,data)
    return summary["sha"],metadata

def phase_values(path,candidate):
    h=(path/"RF1000/build/sketch/motion.h").read_text()
    m=re.search(r"enum DirectMovePhase\s*:\s*char\s*\{([^}]+)\}",h,re.S)
    assert m
    names=[x.strip() for x in re.sub(r"//[^\n]*","",m[1]).split(",") if x.strip()]
    expected=["DIRECT_IDLE","DIRECT_PREPARING","DIRECT_PREPARING_STOPPABLE","DIRECT_PREPARING_STOPPED",
        "DIRECT_PREPARED","DIRECT_PREPARED_STOPPABLE","DIRECT_RUNNING","DIRECT_RUNNING_STOPPABLE"]
    if candidate:expected+=["DIRECT_RUNNING_STOPPING","DIRECT_RUNNING_STOPPABLE_STOPPING"]
    assert names==expected,(names,expected)
    return {name:i for i,name in enumerate(names)}

def run_board(mod,img,phases,inject):
    assert img.sizes["PrintLine::direct"]==103 and img.sizes["PrintLine::queue"]==24*103+5
    assert img.sizes["StepExecutor::stepClocks"]==20 and img.sizes["StepExecutor::stepErrors"]==32
    assert img.sizes["motorWake"]==6 and img.sizes["Printer::zMotion"]==42
    assert img.sizes["Printer::xyMotion"]==36
    Base=mod.CPU
    addr=img.addr("PrintLine::direct")
    class Probe(Base):
        def __init__(self,img):
            super().__init__(img);self.target=None;self.injected=None;self.remaining_reads=[]
        def boundary(self,pc):
            super().boundary(pc)
            if self.target==self.cy:
                assert self.flag(7) and not (self.mem[0x6f]&2)
                phase=self.get(addr+102)
                assert phase in (6,7)
                old=self.get(addr+98,4,signed=True)
                self.put(addr+102,phase+2)
                assert self.get(addr+98,4,signed=True)==old
                self.injected={"cycle":self.cy,"pc":hex(pc),"remaining":old,
                    "primary_rises":sum(p==self.primary and s for p,s,t in self.edges)}
        def read(self,address):
            value=super().read(address)
            if addr+98<=address<addr+102:
                self.remaining_reads.append((self.cy,address-addr-98))
            return value
    mod.CPU=Probe
    def one(c):
        c.timeline=[];c.edges=[];c.calls=[];c.timer_writes=[];c.remaining_reads=[]
        c.cy=5;c.timer_live=True;c.sreg=0;c.mem[0x6f]=2
        c.timer0_phase=0;c.timer0_reads=0;c.timer0_flag_cleared_at_wrap=0
        c.run(17*4)
        assert c.sp==0x21ff and not c.return_stack
        mask=lt.windows(c.timeline,c.cy)
        # Candidate IRQ boundaries: skip instruction immediately following
        # a transition into I=1, including SREG restore, conservatively.
        eligible=[];previous=False
        for t,pc,i,ocie,sp,depth in c.timeline:
            if i and previous and not ocie and pc!=0x3fffe:eligible.append(t)
            previous=bool(i)
        rises={axis:sum(p==axis and s for p,s,t in c.edges) for axis in "XYZ"}
        for axis in "XYZ":
            states=[s for p,s,t in c.edges if p==axis]
            assert states==[1,0]*(len(states)//2)
        return {"cycles":c.cy,"max_mask":max((b-a for a,b in mask),default=0),
            "mask_intervals":mask,"eligible":eligible,"remaining_reads":c.remaining_reads,"rises":rises,
            "remaining":c.get(addr+98,4,signed=True),"task":c.get(addr+102),
            "stack_bytes":0x21ff-c.minsp,"ocr1a":c.get(0x88,2)}
    def setup(delta,n,negative,stoppable,remaining,rate):
        c,pc,move,errors,o=mod.setup(img,owner=0,delta=delta,rate=rate,n=n,
            phase="cruise",advance=False,ratio=0,zcor=0,negative=negative,remaining=remaining)
        assert move==addr and o==0
        c.primary="XYZ"[max(range(3),key=lambda i:delta[i])]
        c.put(addr+102,7 if stoppable else 6)
        c.symput("motorWake",7,1,4);c.symput("motorWake",7,1,5)
        # Direct targets and initial motor/nominal state.
        sign=-1 if negative else 1
        for a in range(2):c.symput("Printer::xyMotion",delta[a]*sign,4,20+a*8)
        c.symput("Printer::zMotion",delta[2]*sign,4,24)
        return c
    def ledger(c,total,negative):
        sign=-1 if negative else 1
        for i,axis in enumerate("XYZ"):
            if i<2:
                motor=c.symget("Printer::xyMotion",4,8+i*4,signed=True)
                nom=c.symget("Printer::xyMotion",4,i*4,signed=True)
                direct=c.symget("Printer::xyMotion",4,16+i*8,signed=True)
                target=c.symget("Printer::xyMotion",4,20+i*8,signed=True)
            else:
                motor=c.symget("Printer::zMotion",4,0,signed=True)
                nom=c.symget("Printer::zMotion",4,4,signed=True)
                direct=c.symget("Printer::zMotion",4,20,signed=True)
                target=c.symget("Printer::zMotion",4,24,signed=True)
            assert motor==1000+sign*total[axis] and nom==1000 and direct==sign*total[axis],(
                axis,motor,nom,direct,total)
            if c.get(addr+102)==0:assert target==direct
        assert c.symget("StepExecutor::stepClocks",4,4)==total[c.primary]
    def trial(delta,n,negative,stoppable,remaining,rate,target=None,drain=False):
        c=setup(delta,n,negative,stoppable,remaining,rate);c.target=target
        first=one(c);total=first["rises"].copy();ledger(c,total,negative)
        if target is None:
            assert first["remaining"]==remaining-n and total[c.primary]==n
            return first,None
        assert c.injected is not None
        before=c.injected["primary_rises"]
        assert first["task"] in (8,9) and first["remaining"]>=0
        # Subsequent complete ISR invocations, with the same actual
        # registers/ledgers and no nested IRQ timing. Timer1 timebase is
        # isolated anew at each compare; not wall-time emulation.
        c.target=None
        peak=first["max_mask"];cycles=first["cycles"]
        if drain:
            for _ in range(40):
                if not c.get(addr+102):break
                row=one(c)
                for axis in "XYZ":total[axis]+=row["rises"][axis]
                ledger(c,total,negative)
                peak=max(peak,row["max_mask"]);cycles=max(cycles,row["cycles"])
            else:raise AssertionError("running tail did not retire")
            assert c.get(addr+102)==0 and c.get(addr+98,4,signed=True)==0
            assert total[c.primary]-before<=33,(total,before)
        else:
            # A request after the last complete transaction is consumed
            # before the next group. No counter mutation may be torn.
            assert first["remaining"]<=32 or (first["remaining"]==remaining-total[c.primary]
                and before==total[c.primary])
        return first,{"max_mask":peak,"max_cycles":cycles,"post_request_primary":total[c.primary]-before}
    results=[]
    try:
        # Match the actual supported <=10 packed steps; no new
        # artificial packing product limit is selected here.
        for delta,n,negative,stoppable in itertools.product(
            ((600,0,0,0),(0,600,0,0),(0,0,600,0),(600,371,129,0)),
            (1,5,10),(False,True),(False,True)):
            rate=11520 if delta[2] else 15238
            initial=256
            baseline,_=trial(delta,n,negative,stoppable,initial,rate)
            times=baseline.pop("eligible")
            row={"delta":delta,"n":n,"negative":negative,"stoppable":stoppable,
                 "uninterrupted":baseline,"injections":0,"drained":0}
            if inject:
                # Cover all legal instruction boundaries of the first
                # sampled packed call. Complete tail for representative
                # first/middle/last boundaries; all cases verify the
                # first call's actual counter/ledger and pulse ordering.
                chosen={times[0],times[len(times)//2],times[-1]}
                targets=set(chosen)
                for a,b in zip(baseline["remaining_reads"],baseline["remaining_reads"][1:]):
                    if a[1]+1==b[1]:
                        targets.update(t for t in times if a[0]<t<=b[0])
                # All eligible boundaries for three representative linked
                # paths; targeted multibyte and entry/exit elsewhere.
                if n==1 and not negative and stoppable and delta in (
                    (600,0,0,0),(0,0,600,0),(600,371,129,0)):
                    targets.update(times)
                max_mask=baseline["max_mask"];max_cycles=baseline["cycles"];max_tail=0
                for target in sorted(targets):
                    first,after=trial(delta,n,negative,stoppable,initial,rate,
                        target,drain=target in chosen)
                    max_mask=max(max_mask,after["max_mask"])
                    max_cycles=max(max_cycles,after["max_cycles"])
                    if target in chosen:max_tail=max(max_tail,after["post_request_primary"])
                row.update(injections=len(targets),drained=len(chosen),
                    max_observed_mask=max_mask,max_observed_cycles=max_cycles,max_tail=max_tail)
            results.append(row)
    finally:mod.CPU=Base
    return results

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--evidence",type=Path,required=True)
    p.add_argument("--prior-evidence",type=Path,required=True)
    p.add_argument("--out",type=Path,required=True)
    a=p.parse_args()
    a.out.parent.mkdir(parents=True,exist_ok=True)
    mod=lt.load_interpreter()
    result={"scope":__doc__,"boards":{}}
    with tempfile.TemporaryDirectory() as d:
        old=Path(d)/"old.zip";new=Path(d)/"new.zip"
        result["prior_sha"],old_meta=evidence(a.prior_evidence,old)
        result["candidate_sha"],new_meta=evidence(a.evidence,new)
        prior_phases=phase_values(a.prior_evidence,False)
        candidate_phases=phase_values(a.evidence,True)
        for board in lt.BOARDS:
            prior=run_board(mod,mod.Image(old,board),prior_phases,False)
            current=run_board(mod,mod.Image(new,board),candidate_phases,True)
            assert len(prior)==len(current)
            result["boards"][board]={"prior":old_meta[board],"candidate":new_meta[board],
                "prior_paths":prior,"candidate_paths":current}
            print(board,"injected",sum(x["injections"] for x in current),
                "complete tails",sum(x["drained"] for x in current),
                "observed mask prior/current",max(x["uninterrupted"]["max_mask"] for x in prior),
                max(x["max_observed_mask"] for x in current),flush=True)
            a.out.write_text(json.dumps(result,indent=2)+"\n")

if __name__=="__main__":main()

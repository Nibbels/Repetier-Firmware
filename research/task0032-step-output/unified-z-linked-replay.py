#!/usr/bin/env python3
"""Linked successive Timer1 calls for the deliberately changed Z scheduler.

Clock advances by actual programmed OCR1A+1 (CTC), no nested ISR simulation.
Nominal event comparison is independent of correction-only events. This is
not the full planner/governor, physical acceleration or safety qualification.
"""
import argparse,json,math,pathlib,statistics,zipfile
import linked_timer1 as lt

F=16_000_000
U32=0xffffffff

def unique(img,part):
    names=[n for n in img.symbols if part in n and img.sizes.get(n,0)>0]
    assert len(names)==1,(part,names)
    return names[0]

def configure(mod,img,kind,rate,negative=False,correction=0,unified=True,start=0):
    # Long synthetic prepared paths, avoiding completion except in explicit
    # idle tests. No planner reachability claim for these profiles.
    n=math.ceil(3300/max(512,F//rate))
    delta={"xy":(100000,70000,0,0),"xye":(100000,70000,0,30000),
           "e":(0,0,0,100000),"z":(0,0,100000,0),"idle":(100000,0,0,0)}[kind]
    c,pc,move,errors,o=mod.setup(img,owner=1,delta=delta,rate=rate,n=n,
        phase="cruise",advance=False,ratio=0,zcor=correction,negative=negative,remaining=100000)
    assert img.sizes["Printer::zMotion"]==42
    c.symput("Printer::extrusionCompensation",0,8)
    fitted=0x1f if img.board in ("RF2000","RF2000v2") else 0x0f
    c.symput("motorWake",fitted,1,4);c.symput("motorWake",fitted,1,5)
    c.mem[0x6f]=2
    c.put(0x88,3000,2)
    if kind=="idle":c.put(img.addr("PrintLine::queue")+24*103,0,5)
    if unified:
        state=unique(img,"zCorrectionE.lto_priv.")
        A=256000;cap=11520*11520//A
        if img.sizes[state]==41:
            c.symput(state,A,4,7)
            c.symput(state,cap,4,11)
            c.symput(state,11520,2,15)
        elif img.sizes[state]==43:
            import importlib.util
            spec=importlib.util.spec_from_file_location("inverse_root",pathlib.Path(__file__).with_name("inverse-root-research.py"))
            ir=importlib.util.module_from_spec(spec);spec.loader.exec_module(ir)
            c.symput(state,A,4,5)
            c.symput(state,cap,4,9)
            c.symput(state,A*29//64,4,13)
            c.symput(state,ir.table_values(*ir.norm(A*cap))[1],4,17)
        else:raise AssertionError((state,img.sizes[state]))
        c.symput(unique(img,"motionEpoch"),(start-3001)&U32,4)
        c.symput(unique(img,"nominalDeadline"),start,4)
    else:c.symput("stepperWait",0,4)
    return c,move,errors,o

def invoke(c,epoch):
    c.cy=5;c.sreg=0;c.timeline=[];c.edges=[];c.calls=[];c.timer_writes=[]
    c.timer_live=True;c.timer0_reads=0;c.timer0_phase=0;c.timer0_flag_cleared_at_wrap=0
    c.minsp=c.sp;c.hit.clear()
    c.mem[0x6f]|=2
    clocks=[n for n in c.img.symbols if "timer0_millis" in n and c.img.sizes.get(n)==4]
    if clocks:c.symput(clocks[0],(epoch//16000)&U32,4)
    c.run(17*4)
    mask=lt.windows(c.timeline,c.cy)
    next_ticks=c.get(0x88,2)+1
    assert next_ticks>c.cy, (epoch,next_ticks,c.cy)
    edges=list(c.edges)
    for pin in {p for p,s,t in edges}:
        last=None
        for p,s,t in edges:
            if p!=pin:continue
            if last:
                assert s!=last[0]
                assert t-last[1]>=31,(pin,last,(s,t),c.img.board)
            last=(s,t)
        if last:assert last[0]==0 or pin.startswith("DIR")
    return dict(cycles=c.cy,max_mask=max((b-a for a,b in mask),default=0),
                next_ticks=next_ticks,stack=0x21ff-c.minsp,edges=edges)

def zfield(c,offset,signed=True):return c.symget("Printer::zMotion",4,offset,signed=signed)
def target(c,value):c.symput("Printer::zMotion",value,4,12)
def z_pulses(row):return [e for e in row["edges"] if e[0]=="Z" and e[1]]

def run_scenario(mod,img,kind,rate,scenario,start=0,merge_guard=None):
    c,move,errors,o=configure(mod,img,kind,rate,start=start)
    if merge_guard is not None:
        c.symput(unique(img,"correctionGapResearch"),merge_guard,2)
    deadline=unique(img,"nominalDeadline")
    release_late=[];nominal_calls=0;correction_only=0;merged=0
    max_corr_only_cycles=0;max_merged_cycles=0
    epoch=start;start_motor=zfield(c,0);start_nominal=zfield(c,4)
    start_applied=zfield(c,16)
    events=[];rises=[];count=0;limit_applied=None;reversal_at=None
    wanted=1600 if scenario=="plateau" else 320 if scenario=="step" else 64
    if scenario=="opposite":wanted=-20
    if scenario=="limit":c.symput("Printer::zMotion",start_motor+20,4,32)
    target(c,wanted)
    before_remaining=c.get(move+98,4,signed=True)
    reached_positive=False;switched=False
    while epoch-start<2*F:
        before=zfield(c,16);oldmotor=zfield(c,0)
        lateness=(epoch-c.symget(deadline,4))&U32
        nominal_due=lateness<0x80000000
        if nominal_due:
            release_late.append(lateness);nominal_calls+=1
        row=invoke(c,epoch)
        after=zfield(c,16);motor=zfield(c,0);nominal=zfield(c,4)
        pulses=z_pulses(row)
        remaining=c.get(move+98,4,signed=True)
        assert motor-start_motor==nominal-start_nominal+after-start_applied,(kind,scenario,epoch,motor,nominal,after)
        if kind!="z":
            assert after-before in (-1,0,1)
            assert len(pulses)==abs(after-before)
            if pulses:
                assert (after-before)*(wanted-before)>0,(scenario,epoch,before,after,wanted)
                assert abs(wanted-after)<abs(wanted-before)
                rises.append(epoch+pulses[0][2])
                if len(rises)>1:assert rises[-1]-rises[-2]>=64
        else:
            assert remaining<=before_remaining
            if remaining==before_remaining:assert not pulses
            if before!=after:assert after<before
        if remaining!=before_remaining:
            assert nominal_due,(epoch,remaining,before_remaining)
            count+=1
        if kind!="z" and pulses:
            if nominal_due:
                merged+=1;max_merged_cycles=max(max_merged_cycles,row["cycles"])
            else:
                correction_only+=1;max_corr_only_cycles=max(max_corr_only_cycles,row["cycles"])
        before_remaining=remaining
        events.append({k:row[k] for k in ("cycles","max_mask","next_ticks","stack")})
        if scenario=="limit" and after==20 and limit_applied is None:
            limit_applied=epoch
        if scenario=="limit" and limit_applied is not None and not switched and epoch-limit_applied>F//20:
            wanted=-64;target(c,wanted);switched=True
        elif scenario=="reverse" and after>=10 and not switched:
            reversal_at=epoch;wanted=-64;target(c,wanted);switched=True
        elif scenario in ("unhome","scan","abort") and after>=10 and not switched:
            switched=True
            if scenario=="unhome":c.symput("Printer::zMotion",0,1,41)
            elif scenario=="scan":c.symput("Printer::zMotion",1,1,40)
            else:c.symput("Printer::blockAll",1)
            reversal_at=epoch
        if scenario in ("unhome","scan","abort") and switched:
            if epoch>reversal_at:assert not pulses
            if epoch-reversal_at>F//10:break
        elif scenario in ("step","plateau") and after==wanted:
            if not reached_positive:
                reached_positive=True;wanted=0;target(c,0)
            else:break
        elif scenario in ("reverse","limit") and switched and after==wanted:break
        elif scenario=="opposite" and after==wanted and 100000-remaining>=40:break
        epoch+=row["next_ticks"]
    if scenario in ("step","plateau"):assert reached_positive and zfield(c,16)==0,(kind,rate,zfield(c,16))
    elif scenario in ("reverse","limit"):assert switched and zfield(c,16)==-64
    elif scenario in ("unhome","scan","abort"):assert switched
    elif scenario=="opposite":assert zfield(c,16)==-20
    # CPU load includes the last executed vector, whose start is the final
    # epoch. No nested ISR or foreground time is counted here.
    total=sum(e["cycles"] for e in events)
    return dict(kind=kind,rate=rate,scenario=scenario,merge_guard=merge_guard,
        calls=len(events),nominal_events=count,nominal_calls=nominal_calls,
        correction_only=correction_only,merged_correction=merged,
        max_corr_only_cycles=max_corr_only_cycles,max_merged_cycles=max_merged_cycles,
        nominal_late_count=sum(x>0 for x in release_late),nominal_late_sum=sum(release_late),
        nominal_late_max=max(release_late,default=0),
        exclusive_cpu_fraction=total/(epoch-start+events[-1]["cycles"]),
        physical_correction_rises=len(rises),end_applied=zfield(c,16),elapsed_ticks=epoch-start,
        max_mask=max(e["max_mask"] for e in events),max_cycles=max(e["cycles"] for e in events),
        mean_cycles=round(statistics.mean(e["cycles"] for e in events),3),
        total_cycles=sum(e["cycles"] for e in events),
        max_stack=max(e["stack"] for e in events),
        min_gap=min((b-a for a,b in zip(rises,rises[1:])),default=None))

def compare_nominal(mod,old,new,kind,rate):
    a,am,ae,ao=configure(mod,old,kind,rate,unified=False)
    b,bm,be,bo=configure(mod,new,kind,rate)
    at=bt=0;oldrows=[];newrows=[]
    for c,m,e,o,rows in ((a,am,ae,ao,oldrows),(b,bm,be,bo,newrows)):
        t=0;remaining=c.get(m+98,4,signed=True)
        while len(rows)<24:
            row=invoke(c,t)
            r=c.get(m+98,4,signed=True)
            if r!=remaining:
                state=lt.snapshot(c,m,e,o)
                rows.append(dict(t=t,state=state,edges=[x[:2] for x in row["edges"]],
                                 cycles=row["cycles"],mask=row["max_mask"]))
            remaining=r;t+=row["next_ticks"]
            assert t<F*20
    for x,y in zip(oldrows,newrows):
        assert x["state"]==y["state"] and x["edges"]==y["edges"],(kind,rate,x,y)
    return dict(kind=kind,rate=rate,events=len(oldrows),
        max_nominal_time_delta=max(abs(x["t"]-y["t"]) for x,y in zip(oldrows,newrows)),
        max_old_cycles=max(x["cycles"] for x in oldrows),max_new_cycles=max(x["cycles"] for x in newrows),
        max_old_mask=max(x["mask"] for x in oldrows),max_new_mask=max(x["mask"] for x in newrows))

def main():
    p=argparse.ArgumentParser()
    p.add_argument("baseline",type=pathlib.Path);p.add_argument("candidate",type=pathlib.Path)
    p.add_argument("out",type=pathlib.Path);a=p.parse_args()
    mod=lt.load_interpreter();results={}
    for board in lt.BOARDS:
        old=mod.Image(a.baseline,board);new=mod.Image(a.candidate,board)
        nominal=[compare_nominal(mod,old,new,k,r) for k,r in (
            ("xy",152),("xy",9143),("xye",15238),("e",14),("e",7000),("z",11520))]
        tests=[("idle",5333,"step"),("xy",152,"step"),("xy",9143,"step"),("e",14,"step"),
               ("xye",15238,"step"),("idle",5333,"reverse"),("idle",5333,"limit"),
               ("idle",5333,"unhome"),("idle",5333,"scan"),("idle",5333,"abort"),
               ("idle",5333,"plateau"),("z",11520,"opposite")]
        rows=[run_scenario(mod,new,k,r,s,start=0xffff0000 if s=="reverse" else 0) for k,r,s in tests]
        with zipfile.ZipFile(a.candidate) as z:
            size=z.read(board+"/size.txt").decode()
        results[board]=dict(nominal=nominal,correction=rows,linked_size=size)
        print("unified-Z",board,json.dumps(results[board],separators=(",",":")),flush=True)
    a.out.write_text(json.dumps(results,indent=2)+"\n")
if __name__=="__main__":main()

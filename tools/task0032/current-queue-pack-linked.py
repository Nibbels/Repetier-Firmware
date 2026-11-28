#!/usr/bin/env python3
"""Actual compact linked queued pack cost/edge matrix, not a governor.

Vary admitted pack size independently of legacy threshold. No assertion
that a CPU-feasible pack is mechanically or electrically suitable.
"""
import argparse
import importlib.util
import itertools
import json
import tempfile
from pathlib import Path

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--evidence",type=Path,required=True)
    p.add_argument("--out",type=Path,required=True)
    p.add_argument("--extended",action="store_true",help="Include ramp phases, reverse motion and compensation; isolated admitted packs, not a planner trace")
    a=p.parse_args()
    spec=importlib.util.spec_from_file_location("energy",Path(__file__).with_name("current-direct-energy-linked.py"))
    el=importlib.util.module_from_spec(spec);spec.loader.exec_module(el)
    el.FAST_PROFILE=el.COMPACT=True
    mod=el.sl.lt.load_interpreter()
    result={"scope":__doc__,"extended":a.extended,"boards":{}}
    with tempfile.TemporaryDirectory() as d:
        archive=Path(d)/"firmware.zip"
        result["candidate_sha"],meta=el.sl.evidence(a.evidence,archive)
        result["metadata"]=meta
        for board in el.sl.lt.BOARDS:
            img=mod.Image(archive,board)
            rows=[]
            for name,delta,rates in (
                ("X",(100,0,0,0),(5000,10000,15238)),
                ("XY",(100,61,0,0),(5000,10000,15238)),
                ("Z",(0,0,100,0),(5000,10000,11520)),
                ("XYZ",(100,61,29,0),(5000,10000,15238)),
                ("XYE",(100,61,0,23),(5000,10000,15238)),
                ("XYZE",(100,61,29,23),(5000,10000,15238)),
                ("E",(0,0,0,100),(5000,10000)),
                ("E-dominant",(41,27,0,100),(5000,10000))):
                primary=max(range(4),key=lambda i:delta[i])
                modes=[("cruise",False,0,0)]
                if a.extended:
                    modes += [("accel",False,0,0),("decel",False,0,0),
                              ("cruise",True,0,0)]
                    if delta[0] or delta[1] or delta[2]:
                        modes += [("cruise",False,0,4),("cruise",True,0,4)]
                    if delta[3]:
                        modes += [("cruise",False,0.25,4),("cruise",True,0.25,4)]
                for rate,n,advance,(phase,negative,ratio,zcor) in itertools.product(
                        rates,range(1,11),(False,True) if delta[3] else (False,),modes):
                    # One already admitted queue pack. No competing Timer0
                    # ISR; in Advance mode E debt is not a physical E pulse.
                    c,pc,addr,errors,o=mod.setup(img,owner=1,delta=delta,rate=rate,n=n,
                        phase=phase,advance=advance,ratio=ratio,zcor=zcor,negative=negative,remaining=100)
                    assert o==-4 and img.sizes["PrintLine::queue"]==2381
                    c.symput("motorWake",31,1,4);c.symput("motorWake",31,1,5)
                    c.mem[0x6f]=2;c.cy=5;c.timer_live=True;c.sreg=0
                    c.run(17*4)
                    assert c.sp==0x21ff and not c.return_stack
                    assert c.get(addr+98+o,4,signed=True)==100-n,(board,name,rate,n)
                    counts={};min_high={};min_low={};min_gap={};short_gap={}
                    nominal_expected=[]
                    for axis in range(4):
                        initial=delta[primary]//2 if axis<3 else 0
                        expected=max(0,(n*delta[axis]-initial+delta[primary]-1)//delta[primary])
                        nominal_expected.append(expected)
                        assert c.get(errors+4*axis,4,signed=True)==initial-n*delta[axis]+expected*delta[primary], (board,name,phase,n,axis)
                    sign=-1 if negative else 1
                    zmotor=c.symget("Printer::zMotion",4,0,signed=True)-1000
                    znominal=c.symget("Printer::zMotion",4,4,signed=True)-1000
                    zapplied=c.symget("Printer::zMotion",4,16,signed=True)
                    assert znominal==sign*nominal_expected[2], (board,name,phase,n,znominal)
                    assert zmotor==znominal+zapplied, (board,name,phase,n,zmotor,znominal,zapplied)
                    extra=(nominal_expected[3]//4 if not negative else (nominal_expected[3]+3)//4) if ratio else 0
                    for axis,pin in enumerate(("X","Y","Z","E0")):
                        edges=[(s,t) for pn,s,t in c.edges if pn==pin]
                        assert [s for s,t in edges]==[1,0]*(len(edges)//2)
                        count=len(edges)//2
                        counts[pin]=count
                        initial=delta[primary]//2 if axis<3 else 0
                        expected=max(0,(n*delta[axis]-initial+delta[primary]-1)//delta[primary])
                        if axis==2 and zcor:
                            expected=abs(zmotor)
                        if axis==3:
                            expected+=extra
                        assert count==(0 if axis==3 and advance else expected),(board,name,n,phase,negative,ratio,zcor,pin,count,expected)
                        if count:
                            min_high[pin]=min(edges[i+1][1]-edges[i][1] for i in range(0,len(edges),2))
                        if count>1:
                            min_low[pin]=min(edges[i+2][1]-edges[i+1][1] for i in range(0,len(edges)-2,2))
                            gaps=[edges[i+2][1]-edges[i][1] for i in range(0,len(edges)-2,2)]
                            min_gap[pin]=min(gaps)
                            # Primary-only comparison; secondary axes have
                            # their own Bresenham pattern, not this period.
                            if axis==primary:short_gap[pin]=sum(g*rate<16000000 for g in gaps)
                    if advance:
                        pending=c.symget("Printer::extrusionOutput",2,0,signed=True)
                        lead=c.symget("Printer::extrusionOutput",2,2,signed=True)
                        assert pending-lead==sign*(nominal_expected[3]+extra),(board,name,phase,n,pending,lead)
                    windows=el.sl.lt.windows(c.timeline,c.cy)
                    rows.append(dict(move=name,delta=delta,rate=rate,pack=n,advance=advance,
                        phase=phase,negative=negative,ecmp_ratio=ratio,zcor_target=zcor,
                        nominal_expected=nominal_expected,z_motor_delta=zmotor,z_nominal_delta=znominal,
                        z_applied_delta=zapplied,advance_pending=c.symget("Printer::extrusionOutput",2,0,signed=True),
                        advance_lead=c.symget("Printer::extrusionOutput",2,2,signed=True),
                        cycles=c.cy,stack=0x21ff-c.minsp,
                        mask=max((b-a for a,b in windows),default=0),
                        next_top=c.get(0x88,2),next_pack=c.symget("StepExecutor::stepsPerTimerCall"),
                        counts=counts,min_high=min_high,min_low=min_low,min_gap=min_gap,
                        primary_short_gaps=short_gap,
                        exclusive_fraction_at_requested_rate=c.cy*rate/(16000000*n),
                        exclusive_slack_at_requested_rate=(16000000*n)//rate-c.cy))
            result["boards"][board]=rows
            print(board,len(rows),"actual queue packs; one-slot max cycles",max(x["cycles"] for x in rows if x["pack"]==1),
                "short primary gaps",sum(sum(x["primary_short_gaps"].values()) for x in rows),
                "electrical low violations",sum(sum(v<31 for v in x["min_low"].values()) for x in rows),flush=True)
            a.out.write_text(json.dumps(result,indent=2)+"\n")
if __name__=="__main__":
    main()

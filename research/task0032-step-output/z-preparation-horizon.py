#!/usr/bin/env python3
"""Replay measured preparation costs under explicitly hypothetical service contracts.

A ring capacity includes the active chord. Before motion starts prefill K
chords. The producer may only start another chord when a slot is freed.
Deadlines are nominal release times; missing a deadline is failure, NOT
permission to execute without correction or abruptly stop. A counterfactual
late producer keeps running only to quantify the deficit. No actual
foreground service guarantee, acceleration qualification or selected capacity.
"""
import argparse,json,math,pathlib
from fractions import Fraction as Q
F=16_000_000

def replay(rows,capacity,rate,share,blackout):
    assert capacity>=2 and 0<share<=1 and rate>0 and blackout>=0
    release=[0]
    for r in rows:release.append(release[-1]+r["length"])
    # Integer ticks, conservative service charge for each preparation job:
    # assumed rate-latency service per job, not inferred from average load.
    times=[(x*F+rate-1)//rate for x in release]
    finish=0;misses=0;worst=0;first=None;min_margin=None
    for i,r in enumerate(rows):
        if i<capacity:continue
        freed=times[i-capacity+1]
        finish=max(freed,finish)+blackout+math.ceil(Q(r["cycles"])/share)
        margin=times[i]-finish
        min_margin=margin if min_margin is None else min(min_margin,margin)
        if margin<0:
            misses+=1;worst=max(worst,-margin)
            if first is None:first=dict(chord=i,release=freed,deadline=times[i],finish=finish,
                                       preceding_samples=release[i]-release[i-capacity+1])
    return dict(capacity=capacity,rate=rate,share=float(share),blackout_ticks=blackout,
                chords=len(rows),samples=release[-1],prefill_chords=min(capacity,len(rows)),
                prefill_cycles=sum(r["cycles"] for r in rows[:capacity]),
                preparation_cycles=sum(r["cycles"] for r in rows),
                misses=misses,max_late_ticks=worst,min_margin_ticks=min_margin,first_miss=first)

def stop_samples(rate,acceleration,service_ticks):
    # Ideal scalar path deceleration, not actual multi-axis or junction proof.
    assert rate>=0 and acceleration>0 and service_ticks>=0
    return (rate*rate+2*acceleration-1)//(2*acceleration)+(rate*service_ticks+F-1)//F

def selftest():
    rows=[dict(length=1,cycles=1000)]*12
    assert replay(rows,2,16000,Q(1),0)["misses"]==0
    assert replay(rows,2,16000,Q(1,2),0)["misses"]==10
    assert replay(rows,12,16000,Q(1,2),0)["misses"]==0
    assert stop_samples(100,50,0)==100
    assert stop_samples(100,50,F//10)==110

def main():
    p=argparse.ArgumentParser()
    p.add_argument("measurements",type=pathlib.Path);p.add_argument("out",type=pathlib.Path);a=p.parse_args()
    selftest()
    source=json.loads(a.measurements.read_text())
    fixtures=sorted({r["fixture"] for r in source["rows"]})
    workloads={}
    for f in fixtures:
        rows=[r for r in source["rows"] if r["fixture"]==f]
        assert [r["index"] for r in rows]==list(range(len(rows)))
        workloads["path"+str(f)]=rows
    # Dense short moves require preparation on every move, even if each
    # move fits one chord. Reuse measured path0 cost, no parser/planner cost.
    workloads["dense_short_moves"]=workloads["path0"]*128
    results=[]
    for name,rows in workloads.items():
        for rate in (3048,9143,15238):
            for share in (Q(1,10),Q(1,5),Q(3,10),Q(1,2),Q(1)):
                for blackout in (0,16000,80000):
                    for capacity in (2,4,8,12,16,24):
                        r=replay(rows,capacity,rate,share,blackout);r["workload"]=name
                        results.append(r)
    # Summarize selected illustrative 20%/30% cases, never treat these
    # hypothetical budgets as measured guaranteed service.
    selected=[]
    for name in workloads:
        for share in (.2,.3,1.0):
            r=[x for x in results if x["workload"]==name and x["rate"]==15238 and x["share"]==share and x["blackout_ticks"]==16000]
            selected.append(dict(workload=name,share=share,misses_by_capacity={str(x["capacity"]):x["misses"] for x in r},
                first_zero_capacity=next((x["capacity"] for x in r if not x["misses"]),None)))
    # Memory estimate for a ring of prepared PhysicalChord+Tag (25 bytes
    # each on AVR). Control and execution context excluded deliberately.
    memory={str(k):25*k for k in (2,4,8,12,16,24)}
    stopping=[dict(rate=v,acceleration=a,service_ticks=t,ideal_samples=stop_samples(v,a,t)) for v in (3048,9143,15238) for a in (15238,76190,128000) for t in (0,16000,80000)]
    out=dict(schema=1,classification="measured isolated preparation costs with hypothetical rate-latency service; no guaranteed foreground availability or production admission",
             memory_records_only=memory,selected=selected,stopping=stopping,rows=results)
    a.out.write_text(json.dumps(out,indent=2)+"\n")
    print("Z preparation horizon",json.dumps({k:v for k,v in out.items() if k not in ("rows","stopping")}),flush=True)
if __name__=="__main__":main()

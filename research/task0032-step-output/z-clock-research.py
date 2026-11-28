#!/usr/bin/env python3
"""Compare call-count ZCMP with two common Timer1 scheduling alternatives.

This is an integer-tick discrete-event model, NOT actual AVR execution or
governor. Nominal releases are immutable input; no fabricated lookahead.
Reference ramp uses one-step constant acceleration in the distance domain.
A target update can invalidate an armed pulse. No past-due catch-up burst.
"""
import argparse, itertools, json, math, statistics, pathlib
F=16_000_000
STEPS=1280
AMAX=100*STEPS
VMAX=9*STEPS
END=F*2
MIN_GAP=64
# Nominal dispatch cost samples, NOT WCET. 0.1mm, 150mm/s, E/XY/Z etc
# are modeled separately; cost is a sensitivity, not a budget assignment.
COSTS=(0,1200,2400)
def ceildiv(a,b):return (a+b-1)//b
class Ramp:
    """An energy-indexed reference, using floor sqrt and ceil time.
    At rest, level=0. One accelerating step changes v^2 by 2*a.
    Abrupt retarget/limit stop is explicitly counted as a safety override.
    """
    def __init__(self,a=AMAX,v=VMAX):
        self.a=a;self.v=v;self.cap=v*v//(2*a)
        assert self.cap>=1
        self.level=0;self.dir=0;self.overrides=0
    def stop(self):
        self.overrides+=bool(self.level)
        self.level=0;self.dir=0
    def plan(self,error):
        if not error:
            self.stop()
            return None
        sign=1 if error>0 else -1;d=abs(error)
        if self.dir!=sign or d<self.level:
            self.stop();self.dir=sign
        l=self.level
        if d==1 and l==0:
            # One-step rest-to-rest triangular move.
            return sign,0,ceildiv(2*F,math.isqrt(self.a))
        n=l-1 if d<=l else l+1 if d>=l+2 and l<self.cap else l
        den=math.isqrt(2*self.a*l)+math.isqrt(2*self.a*n)
        assert den and 0<=n<=self.cap and d-1>=n,(d,l,n)
        return sign,n,ceildiv(2*F,den)
    def commit(self,plan):self.dir,self.level,_=plan
class Legacy:
    def __init__(self):self.mass=15;self.wait=0;self.pos=0
    def visit(self,n,target,eligible=True):
        if not eligible:return 0
        if self.wait:self.wait-=1;return 0
        d=target-self.pos
        if d:
            self.pos+=1 if d>0 else -1
            if self.mass<15 and abs(d)<15:self.mass+=1
            elif self.mass:self.mass-=1
        elif self.mass<15:self.mass+=1
        self.wait=self.mass+n
        return (1 if d>0 else -1) if d else 0
def nominal(kind,speed,n):
    if kind=="idle":return 3000,0
    rate=round(speed*(152.38 if kind=="xy" else 280 if kind=="e" else STEPS))
    return ceildiv(F*n,rate),n
def target_at(t,pattern):
    # All updates are nominally foreground publications every 16ms.
    # Matrix slope examples are synthetic, not measured RF bed profiles.
    sec=(t//256000)*.016
    if pattern=="step":return 320 if sec<1 else 0
    if pattern=="live":return (64 if sec<.4 else 6 if sec<.8 else -64 if sec<1.2 else 0)
    if pattern=="slope":return round(80*math.sin(2*math.pi*sec/.8))
    if pattern=="limited":return 80 if sec<.8 else -64
    raise ValueError(pattern)
def permitted(t,pos,sign,pattern):
    # Artificial positive contact and later release; retreat is always legal.
    return not(pattern=="limited" and t<F//2 and pos>=20 and sign>0)
def simulate(kind,speed,n,pattern,policy,cost=0,a=AMAX,v=VMAX):
    period,count=nominal(kind,speed,n)
    # A pure nominal-Z move owns the motor: no additional pulse in this
    # prototype. Opposing cancellation deferred to next arbitration replay.
    owns_z=kind=="z"
    legacy=Legacy();ramp=Ramp(a,v)
    pos=0;target=target_at(0,pattern);next_nominal=0;next_publish=0
    planned=None;due=None;last_pulse=None;ready_at=0
    edges=[];errors=[];nominal_count=0;extra_dispatch=0;shared_dispatch=0
    unavailable=0;blocked=0;stale=0;max_level=0;late_nominal=0
    def arm(now):
        nonlocal planned,due,blocked,max_level
        d=target-pos
        if owns_z or not d:
            ramp.stop();planned=due=None;return
        sign=1 if d>0 else -1
        if not permitted(now,pos,sign,pattern):
            ramp.stop();planned=due=None;blocked+=1;return
        planned=ramp.plan(d)
        max_level=max(max_level,ramp.level)
        # Replanning at a target update never accelerates the already armed
        # physical step or resets the elapsed clock to gain a catch-up pulse.
        due=max(now, (last_pulse if last_pulse is not None else now)+planned[2])
    def emit(now):
        nonlocal pos,last_pulse,planned,due,unavailable,stale
        assert planned is not None and now>=due
        sign=planned[0];d=target-pos
        if not d or (d>0)!=(sign>0) or not permitted(now,pos,sign,pattern):
            stale+=1;ramp.stop();planned=due=None;return
        if last_pulse is not None:
            assert now-last_pulse>=MIN_GAP
        pos+=sign;ramp.commit(planned);edges.append((now,sign,pos))
        last_pulse=now
        arm(now)
    while True:
        t=min(next_nominal,next_publish,due if policy=="merge" and due is not None else END)
        if t>=END:break
        if t==next_publish:
            new=target_at(t,pattern)
            next_publish+=256000
            if new!=target or (planned is None and target!=pos):
                target=new
                if policy!="legacy":
                    old_due=due;old_sign=planned[0] if planned else 0
                    arm(t)
                    if due is not None and old_due is not None and old_sign==planned[0]:
                        due=max(due,old_due)
            errors.append(abs(target-pos))
        if t==next_nominal:
            nominal_count+=count;next_nominal+=period
            # Nominal event cost is not included in its scheduled period.
            # Report overloaded sensitivity rather than silently speed limit.
            if cost>=period:late_nominal+=1
            if policy=="legacy":
                visits=1 if kind=="idle" else n if kind=="xy" else 0
                for j in range(visits):
                    d=target-legacy.pos
                    if d and not permitted(t,legacy.pos,1 if d>0 else -1,pattern):d=0
                    sign=legacy.visit(n,legacy.pos+d)
                    if sign:edges.append((t+j*80,sign,legacy.pos))
                pos=legacy.pos
            elif due is not None and t>=due:
                # Coalesce due correction with nominal event, no extra vector.
                emit(t);shared_dispatch+=1
            elif planned is None and target!=pos:
                arm(t)
            ready_at=max(ready_at,t+cost)
        if policy=="merge" and due is not None and due<=t:
            # Never run correction during nominal event. If next nominal
            # release is near, defer to it, keeping its deadline unchanged.
            # 200 cycles is only a correction-only cost sensitivity.
            earliest=max(t,ready_at)
            if earliest+200>=next_nominal:
                due=next_nominal;unavailable+=1
            elif earliest>t:
                due=earliest
            else:
                emit(t);extra_dispatch+=1;ready_at=t+200
    intervals=[b[0]-a[0] for a,b in zip(edges,edges[1:])]
    return {"kind":kind,"speed":speed,"packing":n,"pattern":pattern,"policy":policy,"cost":cost,
            "a":a,"v":v,"end":pos,"nominal_steps":nominal_count,"correction_steps":len(edges),
            "max_sample_error_steps":max(errors,default=0),"mean_sample_error_steps":round(statistics.mean(errors),4),
            "min_gap_ticks":min(intervals,default=None),"extra_dispatch":extra_dispatch,
            "shared_dispatch":shared_dispatch,"safety_ramp_resets":ramp.overrides,
            "limit_blocks":blocked,"stale_pulses_rejected":stale,"deferrals":unavailable,
            "overloaded_nominal_events":late_nominal,"max_ramp_level":max_level,
            "edges":edges}
def selftest():
    for a,v in ((6400,11520),(128000,11520),(128000,2000),(256000,23040)):
        for distance in (1,2,3,6,64,320,1280,4096):
            r=Ramp(a,v);d=distance;t=0
            while d:
                p=r.plan(d);assert p[0]==1 and p[2]>=ceildiv(F,v)
                t+=p[2];r.commit(p);d-=1
            assert r.level==0 and r.overrides==0
    # Direction change invalidates any armed pulse; no stale target movement.
    r=Ramp();r.commit(r.plan(320));assert r.level==1
    assert r.plan(-1)[0]==-1 and r.overrides==1
def main():
    p=argparse.ArgumentParser();p.add_argument("output",type=pathlib.Path);args=p.parse_args()
    selftest();rows=[]
    modes=[("idle",0,n) for n in (1,2,5)]
    modes += [("xy",s,n) for s in (1,20,60,100,150) for n in sorted({1,ceildiv(3300,F//round(s*152.38))})]
    modes += [("e",s,1) for s in (.05,1,25)]
    modes += [("z",s,1) for s in (.1,9)]
    for (kind,s,n),pattern,policy,cost in itertools.product(modes,("step","live","slope","limited"),("legacy","coalesce","merge"),COSTS):
        if policy=="legacy" and cost:continue
        row=simulate(kind,s,n,pattern,policy,cost)
        assert row["nominal_steps"]==simulate(kind,s,n,pattern,"legacy",0)["nominal_steps"]
        if policy!="legacy":assert row["stale_pulses_rejected"]==0 and (row["min_gap_ticks"] is None or row["min_gap_ticks"]>=MIN_GAP)
        rows.append(row)
    args.output.write_text(json.dumps({"schema":1,"classification":"synthetic discrete event model, not firmware or WCET","rows":rows},separators=(",",":"))+"\n")
    for kind,s,n in modes:
        a=[r for r in rows if (r["kind"],r["speed"],r["packing"],r["pattern"],r["cost"])==(kind,s,n,"step",0)]
        print(kind,s,n,[(r["policy"],r["end"],r["correction_steps"],r["mean_sample_error_steps"],r["extra_dispatch"]) for r in a],flush=True)
    print("Z clock scenarios",len(rows),"passed",flush=True)
if __name__=="__main__":main()

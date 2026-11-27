#!/usr/bin/env python3
"""Continuous no-intermediate-settle net-Z workload comparison.

Optimistic rate-only motor model: one physical pulse opportunity per
ceil(F_CPU/maxZrate), no acceleration or CPU scheduling. No hidden wait at
nominal move joins. Reports debt, not a shipping timing policy. Nominal
time is prescribed; any motor debt is a tracking/admission failure, not
silently treated as executed physical motion.
"""
import importlib.util,json,math,pathlib,sys
HERE=pathlib.Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("lattice",HERE/"z-feedforward-lattice.py")
l=importlib.util.module_from_spec(spec);spec.loader.exec_module(l)
F=16_000_000;ZRATE=11520;PERIOD=(F+ZRATE-1)//ZRATE;CADENCE=F*16//1000

def trajectory(points,xy_speed,z_speed):
    t0=0
    for seg,(a,b) in enumerate(zip(points,points[1:])):
        n,chords=l.prepare(a,b)
        if not n:continue
        duration=max(math.hypot(b[0]-a[0],b[1]-a[1])/152.38/xy_speed,
                     abs(b[2]-a[2])/1280/z_speed)
        ticks=max(n,math.ceil(duration*F))
        for lo,hi,c0,c1 in chords:
            for i in range(lo+1,hi+1):
                p=l.point(a,b,i,n)
                yield t0+(ticks*i+n-1)//n,p,l.round_chord(c0,c1,i-lo,hi-lo),seg,i==n
        t0+=ticks

class Motor:
    def __init__(self,pos):
        self.pos=self.goal=pos;self.last=-PERIOD;self.pulses=0
        self.last_dir=0;self.reversals=0;self.min_gap=10**30
    def move(self,at):
        assert self.goal!=self.pos and at-self.last>=PERIOD
        sign=1 if self.goal>self.pos else -1
        if self.pulses:self.min_gap=min(self.min_gap,at-self.last)
        if self.last_dir and self.last_dir!=sign:self.reversals+=1
        self.pos+=sign;self.pulses+=1;self.last=at;self.last_dir=sign
    def before(self,t,previous):
        while self.goal!=self.pos:
            due=max(self.last+PERIOD,previous+1)
            if due>=t:break
            self.move(due)
    def release(self,t,goal):
        self.goal=goal
        if self.goal!=self.pos and t-self.last>=PERIOD:self.move(t)
    def drain(self,t):
        while self.goal!=self.pos:self.move(max(self.last+PERIOD,t+1))
        return max(0,self.last-t)

def run(points,xy,z,mode):
    a=points[0];initial=a[2]+l.target(a)
    m=Motor(initial);prev=0;nextSample=CADENCE;sample=l.target(a)
    maxerr=0;sumerr=0;maxdebt=0;count=0;joins=[]
    peakGeom=0
    for t,p,c,seg,join in trajectory(points,xy,z):
        m.before(t,prev)
        truth=l.target(p)
        if t>=nextSample:
            sample=truth;nextSample=(t//CADENCE+1)*CADENCE
        target=c if mode=="feedforward" else sample
        m.release(t,p[2]+target)
        error=abs(m.pos-(p[2]+truth))
        maxerr=max(maxerr,error);sumerr+=error;count+=1
        peakGeom=max(peakGeom,abs(target-truth))
        maxdebt=max(maxdebt,abs(m.goal-m.pos))
        if join:joins.append(dict(t=t,physical=m.pos,ideal=p[2]+truth,error=m.pos-p[2]-truth))
        prev=t
    assert points[-1]==a
    final_error=m.pos-initial
    m.goal=initial;settle=m.drain(prev)
    assert m.pos==initial and (m.pulses==0 or m.min_gap>=PERIOD)
    return dict(mode=mode,xy_mm_s=xy,z_mm_s=z,samples=count,elapsed_ticks=prev,
        max_error_steps=maxerr,mean_error_steps=sumerr/count,max_target_error=peakGeom,
        max_physical_debt=maxdebt,unsettled_closure_error=final_error,
        diagnostic_final_settle_ticks=settle,pulses=m.pulses,reversals=m.reversals,joins=joins)

def main():
    a=(1543,1881,256)
    small=[a,(1543,1881,704),(4950,7148,704),(3238,7675,256),a]
    high=[a,(1543,1881,9200),(4950,7148,9200),(3238,7675,256),a]
    zero=[(0,3048,1),(9142,3048,-1),(9142,6095,1),(0,3048,1)]
    rows=[]
    for name,points in (("normal_hop_wipe",small),("cross_both_fade_bounds",high),("zero_discontinuity_stress",zero)):
        for xy in (20,60,100):
            for z in (3,9):
                for mode in ("sample16ms","feedforward"):
                    row=run(points,xy,z,mode);row["path"]=name;rows.append(row)
                    print("Z stream",json.dumps({k:v for k,v in row.items() if k!="joins"}),flush=True)
    pathlib.Path(sys.argv[1]).write_text(json.dumps(dict(schema=1,classification="optimistic rate-only net-target model; no acceleration/CPU proof",rows=rows),indent=2)+"\n")
if __name__=="__main__":main()

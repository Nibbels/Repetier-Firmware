#!/usr/bin/env python3
"""Compare separate and joint rounding of geometric motor coordinates.

Neither mode is a physical pulse scheduler. Report reversals, clustered
demands and an optimistic rest-to-rest Z-time lower bound for each monotone
pulse run. Stops at sign changes are mechanical constraints; rounding
artefacts that create extra sign changes should not reach the motor.
"""
import importlib.util,itertools,json,math,pathlib,sys
HERE=pathlib.Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("lattice",HERE/"z-feedforward-lattice.py")
l=importlib.util.module_from_spec(spec);spec.loader.exec_module(l)
g=l.g
V=11520;A=128000
def lower_time(steps):
    if steps<=V*V/A:return 2*math.sqrt(steps/A)
    return 2*V/A+(steps-V*V/A)/V

def trial(a,b,joint):
    n,chords=l.prepare(a,b)
    prev=a[2]+l.target(a);initial=prev;lastdir=0
    runs=[];err=0;rev=0;multi=0;maxd=0;total=0
    for lo,hi,c0,c1 in chords:
        z0=l.point(a,b,lo,n)[2];z1=l.point(a,b,hi,n)[2]
        assert prev==z0+c0
        for i in range(lo+1,hi+1):
            p=l.point(a,b,i,n)
            c=l.round_chord(z0+c0,z1+c1,i-lo,hi-lo) if joint else p[2]+l.round_chord(c0,c1,i-lo,hi-lo)
            d=c-prev
            err=max(err,abs(c-p[2]-l.target(p)))
            if d:
                s=1 if d>0 else -1
                if s!=lastdir:
                    if lastdir:rev+=1
                    runs.append(0);lastdir=s
                runs[-1]+=abs(d);total+=abs(d)
                multi+=abs(d)>1;maxd=max(maxd,abs(d))
            prev=c
        assert prev==z1+c1
    assert prev==b[2]+l.target(b)
    assert total>=abs(prev-initial)
    return dict(max_tracking_error_steps=err,sign_reversals=rev,
                multi_demand_samples=multi,max_demand=maxd,physical_distance_steps=total,
                rest_to_rest_minimum_s=sum(map(lower_time,runs)))

def main():
    pts=((0,0,g.F-1),(3048,3048,g.F),(6095,3048,g.F+1),
        (3048,6095,g.H-1),(6095,6095,g.H),(9142,9142,g.H+1),
        (0,9142,0),(9142,0,1),(3048,3048,-1),(3048,3048,g.H+500))
    paths=[(a,b) for a,b in itertools.product(pts,repeat=2) if a!=b]
    # Near-canceling small nominal Z slope and large XY correction.
    paths.extend([((0,3048,200),(3048,3048,190)),((3048,3048,190),(0,3048,200)),
        ((0,3048,200),(3048,3048,10)),((3048,3048,10),(0,3048,200)),
        ((0,0,-1),(9142,9142,1)),((9142,9142,1),(0,0,-1))])
    rows=[]
    for a,b in paths:
        x=trial(a,b,False);y=trial(a,b,True)
        assert y["physical_distance_steps"]<=x["physical_distance_steps"]
        assert y["sign_reversals"]<=x["sign_reversals"]
        rows.append(dict(a=a,b=b,separate=x,joint=y))
    summary={}
    for mode in ("separate","joint"):
        summary[mode]={k:sum(r[mode][k] for r in rows) for k in ("sign_reversals","multi_demand_samples","physical_distance_steps","rest_to_rest_minimum_s")}
        summary[mode].update({k:max(r[mode][k] for r in rows) for k in ("max_tracking_error_steps","max_demand")})
    out=dict(schema=1,classification="offline integer targets, optimistic isolated rest-to-rest motor time lower bounds; not a pulse schedule",paths=len(rows),summary=summary,rows=rows)
    pathlib.Path(sys.argv[1]).write_text(json.dumps(out,indent=2)+"\n")
    print("Z joint physical path",json.dumps({k:v for k,v in out.items() if k!="rows"}),flush=True)
if __name__=="__main__":main()

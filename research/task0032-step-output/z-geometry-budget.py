#!/usr/bin/env python3
"""Continuous geometric Z velocity/acceleration envelope for central governor.

Exact rational derivatives of each smooth cell/fade interval. A continuous
envelope is not discrete pulse/ISR feasibility and not a production speed
policy. Derivative jumps and discontinuities are reported, never averaged.
"""
import importlib.util,itertools,json,math,pathlib,sys
from fractions import Fraction as Q
HERE=pathlib.Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("geometry",HERE/"z-feedforward.py")
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
ZSTEPS=1280;V=9*ZSTEPS;A=100*ZSTEPS

def envelope(a,b):
    dz=b[2]-a[2]
    vmax=Q(0);amax=Q(0);jump=Q(0);gap=Q(0);prev=None;pieces=[]
    cuts=g.cuts(a,b)
    for lo,hi in zip(cuts,cuts[1:]):
        c=g.cubic(a,b,lo,hi);width=hi-lo
        def v(u):return dz+(c[1]+2*c[2]*u+3*c[3]*u*u)/width
        candidates=[Q(0),Q(1)]
        if c[3]:
            u=-c[2]/(3*c[3])
            if 0<u<1:candidates.append(u)
        vv=max(abs(v(u)) for u in candidates)
        aa=max(abs(2*c[2]),abs(2*c[2]+6*c[3]))/(width*width)
        start=g.evaluate(c,Q(0))+a[2]+dz*lo
        end=g.evaluate(c,Q(1))+a[2]+dz*hi
        if prev is None:gap=max(gap,abs(start-(a[2]+g.geom(a))))
        else:
            jump=max(jump,abs(v(0)-prev[1]))
            gap=max(gap,abs(start-prev[0]))
            # Historical z=0 value can differ from both one-sided limits.
            exact=a[2]+dz*lo+g.geom(g.pos(a,b,lo))
            gap=max(gap,abs(exact-start),abs(exact-prev[0]))
        prev=(end,v(1))
        vmax=max(vmax,vv);amax=max(amax,aa)
        pieces.append(dict(lo=float(lo),hi=float(hi),max_abs_first=float(vv),max_abs_second=float(aa)))
    gap=max(gap,abs(prev[0]-(b[2]+g.geom(b))))
    return vmax,amax,jump,gap,pieces

def main():
    pts=((0,0,g.F-1),(3048,3048,g.F),(6095,3048,g.F+1),
        (3048,6095,g.H-1),(6095,6095,g.H),(9142,9142,g.H+1),
        (0,9142,0),(9142,0,1),(3048,3048,-1),(3048,3048,g.H+500))
    rows=[]
    for a,b in itertools.product(pts,repeat=2):
        if a==b:continue
        v,acc,jump,gap,pieces=envelope(a,b)
        requested=max(math.hypot(b[0]-a[0],b[1]-a[1])/152.38/100,abs(b[2]-a[2])/ZSTEPS/9)
        lower=max(float(v/V),math.sqrt(float(acc/A)))
        rows.append(dict(a=a,b=b,requested_duration_s=requested,
            smooth_min_duration_s=lower,ratio=lower/requested,
            peak_v_steps_s=float(v)/requested,peak_a_steps_s2=float(acc)/(requested**2),
            boundary_velocity_jump_steps_s=float(jump)/requested,
            boundary_position_gap_steps=float(gap),pieces=pieces))
    # Pure Z at deepest cell: slope 420/(420*20)=1/20.
    a=(6095,3047,g.F);b=(6095,3047,g.H)
    v,acc,jump,gap,_=envelope(a,b)
    assert v==Q(21,20)*(g.H-g.F) and acc==jump==gap==0
    pure=dict(nominal_mm_s=9,physical_mm_s=9*21/20,
              allowable_nominal_mm_s=9*20/21,fade_slope=0.05)
    out=dict(schema=1,classification="smooth continuous geometry envelope only; jumps require separate admission/junction policy",
        paths=len(rows),pure_z=pure,
        rate_or_curvature_over_budget=sum(x["ratio"]>1 for x in rows),
        discontinuous_paths=sum(x["boundary_position_gap_steps"]>0 for x in rows),
        derivative_jump_paths=sum(x["boundary_velocity_jump_steps_s"]>0 for x in rows),rows=rows)
    pathlib.Path(sys.argv[1]).write_text(json.dumps(out,indent=2)+"\n")
    print("Z geometry budget",json.dumps({k:v for k,v in out.items() if k!="rows"}),flush=True)
if __name__=="__main__":main()

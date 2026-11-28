#!/usr/bin/env python3
"""Exact rational continuous-region Z junction smoothing oracle.

Constant primary rate inside one isolated geometric move. No nominal
acceleration, physical pulse schedule, planner integration or hardware proof.
Add compact quadratic bumps whose derivative jumps cancel only the real
cell/fade derivative jumps, not artifacts of integer chord subdivision.
Position discontinuities (historical zero-height rule) are rejected.
"""
import importlib.util,itertools,json,math,pathlib,sys
from fractions import Fraction as Q
HERE=pathlib.Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('geometry',HERE/'z-feedforward.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
V=11520; A=128000; F_CPU=16000000

def curve(a,b):
    n=max(abs(y-x) for x,y in zip(a,b));assert n
    cuts=g.cuts(a,b);pieces=[]
    for lo,hi in zip(cuts,cuts[1:]):
        c=list(g.cubic(a,b,lo,hi))
        c[0]+=g.mix(a[2],b[2],lo);c[1]+=(b[2]-a[2])*(hi-lo)
        pieces.append((n*lo,n*hi,c))
    gap=max(abs(pieces[0][2][0]-a[2]-g.geom(a)),abs(g.evaluate(pieces[-1][2],Q(1))-b[2]-g.geom(b)),
        max((abs(g.evaluate(x[2],Q(1))-y[2][0]) for x,y in zip(pieces,pieces[1:])),default=Q(0)))
    return n,pieces,gap

def base(piece,s):
    lo,hi,c=piece;w=hi-lo;u=(s-lo)/w
    return g.evaluate(c,u),(c[1]+2*c[2]*u+3*c[3]*u*u)/w,(2*c[2]+6*c[3]*u)/(w*w)

def check(pieces,rate,budget,tolerance):
    n=pieces[-1][1];bumps=[];knots={p[0] for p in pieces}|{n}
    for left,right in zip(pieces,pieces[1:]):
        s=left[1];assert s==right[0]
        vl=base(left,s)[1];vr=base(right,s)[1];jump=vr-vl
        if not jump:continue
        h=abs(jump)*rate*rate/(2*budget)
        if s-h<0 or s+h>n:return dict(ok=False,reason='endpoint_horizon')
        bumps.append((s,h,jump));knots.update((s-h,s,s+h))
    def evaluate(piece,active,s):
        p,d,dd=base(piece,s);b=bd=bdd=Q(0)
        for center,h,jump,side in active:
            u=h+side*(s-center)
            b+=jump*u*u/(4*h);bd+=side*jump*u/(2*h);bdd+=jump/(2*h)
        return p+b,d+bd,dd+bdd,b,bd
    speed=accel=error=Q(0);prev=None;overlap=0;intervals=0
    cuts=sorted(knots);pi=0
    for lo,hi in zip(cuts,cuts[1:]):
        mid=(lo+hi)/2
        while mid>pieces[pi][1]:pi+=1
        piece=pieces[pi]
        active=[(s,h,j,1 if mid<s else -1) for s,h,j in bumps if abs(mid-s)<h]
        overlap=max(overlap,len(active));intervals+=1
        l=evaluate(piece,active,lo);r=evaluate(piece,active,hi)
        if prev is None:assert l[0]==base(piece,lo)[0] and l[3]==l[4]==0
        else:assert l[:2]==prev[:2] # C1 even at true/overlapping junctions
        prev=r
        speed=max(speed,abs(l[1])*rate,abs(r[1])*rate)
        accel=max(accel,abs(l[2])*rate*rate,abs(r[2])*rate*rate)
        error=max(error,abs(l[3]),abs(r[3]))
        if l[2]*r[2]<0: # derivative stationary point, exact root
            s=lo+(hi-lo)*(-l[2])/(r[2]-l[2]);speed=max(speed,abs(evaluate(piece,active,s)[1])*rate)
        if l[4]*r[4]<0: # bump error stationary point
            s=lo+(hi-lo)*(-l[4])/(r[4]-l[4]);error=max(error,abs(evaluate(piece,active,s)[3]))
    assert prev[0]==base(pieces[-1],n)[0] and prev[3]==prev[4]==0
    reason='speed' if speed>V else 'acceleration' if accel>A else 'tracking' if error>tolerance else 'none'
    return dict(ok=reason=='none',reason=reason,max_speed=float(speed),max_acceleration=float(accel),
        max_blend_error_steps=float(error),bumps=len(bumps),max_overlap=overlap,intervals=intervals)

def chord_cap(a,b,n,accel,tolerance):
    lengths=[];slopes=[]
    for lo,hi,c,k,bound in g.plan(a,b):
        length=n*(hi-lo)/k
        for i in range(k):
            v0=g.evaluate(c,Q(i,k));v1=g.evaluate(c,Q(i+1,k))
            slopes.append(Q(b[2]-a[2],n)+(v1-v0)/length);lengths.append(length)
    jumps=[Q(0)]+[abs(b-a) for a,b in zip(slopes,slopes[1:])]+[Q(0)]
    caps=[Q(V*V)/(s*s) for s in slopes if s]
    caps.extend(8*accel*tolerance/(d*d) for d in jumps if d)
    caps.extend(2*accel*l/(jumps[i]+jumps[i+1]) for i,l in enumerate(lengths) if jumps[i]+jumps[i+1])
    return min(caps) if caps else None

def overlap_test():
    # Opposing and reinforcing overlapping bumps, actual combined
    # acceleration checked rather than assigning the same budget twice.
    for last,expected in ((Q(0),64000),(Q(1,5),128000)):
        ps=[(Q(0),Q(100),[Q(0)]*4),(Q(100),Q(110),[Q(0),Q(1),Q(0),Q(0)]),
            (Q(110),Q(210),[Q(1),last*100,Q(0),Q(0)])]
        r=check(ps,Q(6000),Q(64000),Q(3))
        assert r['ok'] and r['max_overlap']==2 and r['max_acceleration']==expected

def main():
    overlap_test()
    pts=((0,0,g.F-1),(3048,3048,g.F),(6095,3048,g.F+1),
        (3048,6095,g.H-1),(6095,6095,g.H),(9142,9142,g.H+1),
        (0,9142,0),(9142,0,1),(3048,3048,-1),(3048,3048,g.H+500))
    paths=[(a,b) for a,b in itertools.product(pts,repeat=2) if a!=b]
    paths.extend([((0,0,256),(9142,9142,256)),((9142,9142,256),(0,0,256)),
                  ((3047,3047,256),(6095,6095,256)),((6095,6095,256),(3047,3047,256))])
    rows=[];reject=0;verified=0
    scales=[Q(1),Q(95,100),Q(9,10),Q(4,5),Q(7,10),Q(1,2),Q(1,4),Q(1,10),Q(1,20),Q(1,100)]
    for a,b in paths:
        n,pieces,gap=curve(a,b)
        if gap:reject+=1;rows.append(dict(a=a,b=b,rejected=True,position_gap_steps=float(gap)));continue
        rn,reverse,rgap=curve(b,a);assert rn==n and not rgap
        for xy in (20,60,100):
            ticks=max(n,math.ceil(F_CPU*max(math.hypot(b[0]-a[0],b[1]-a[1])/152.38/xy,abs(b[2]-a[2])/1280/9)))
            req=Q(n*F_CPU,ticks)
            for tol in (Q(1),Q(3)):
                budget=A*Q(1,2);cap=chord_cap(a,b,n,budget,tol)
                chord_ratio=min(1,math.sqrt(float(cap/(req*req)))) if cap is not None else 1
                first=check(pieces,req,budget,tol);result=first;selected=None
                for scale in scales:
                    result=check(pieces,req*scale,budget,tol)
                    rev=check(reverse,req*scale,budget,tol)
                    assert result==rev,(a,b,scale,result,rev)
                    if result['ok']:selected=scale;verified+=1;break
                rows.append(dict(a=a,b=b,rejected=False,xy_mm_s=xy,tolerance_steps=float(tol),
                    requested_primary_rate=float(req),chord_constant_rate_ratio=chord_ratio,
                    true_region_grid_rate_ratio=float(selected) if selected else None,at_requested=first,selected=result))
    selected=[r for r in rows if not r['rejected'] and r['xy_mm_s']==100 and r['tolerance_steps']==3]
    out=dict(schema=1,classification='exact rational continuous constant-rate interior; half Z accel per bump; tested speed grid, not optimal; no pulse or actual planner proof',paths=len(paths),position_discontinuity_rejects=reject,verified_profiles=verified,
        selected=dict(profiles=len(selected),chord_full_rate=sum(r['chord_constant_rate_ratio']==1 for r in selected),
            region_full_rate=sum(r['true_region_grid_rate_ratio']==1 for r in selected),
            region_improves_chord_ratio=sum((r['true_region_grid_rate_ratio'] or 0)>r['chord_constant_rate_ratio']+1e-6 for r in selected),
            region_unsolved=sum(r['true_region_grid_rate_ratio'] is None for r in selected),
            max_overlap=max(r['selected'].get('max_overlap',0) for r in selected),
            reasons_at_requested={k:sum(r['at_requested']['reason']==k for r in selected) for k in ('none','speed','acceleration','tracking','endpoint_horizon')}),rows=rows)
    pathlib.Path(sys.argv[1]).write_text(json.dumps(out,indent=2)+'\n')
    print('Z continuous region blend',json.dumps({k:v for k,v in out.items() if k!='rows'}),flush=True)
if __name__=='__main__':main()

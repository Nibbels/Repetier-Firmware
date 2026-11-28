#!/usr/bin/env python3
"""Generate compact integer-indexed geometric chords and validate all samples.

Uses the exact rational region/curvature prototype. Preparation cost and
storage in this Python oracle are NOT the AVR implementation cost.
The fixture follows actual XYZ Bresenham convention, not a continuous line
or symmetric intermediate positions (ties may differ on the reverse path).
"""
import importlib.util,json,math,pathlib,sys
from fractions import Fraction as Q
HERE=pathlib.Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("geometry",HERE/"z-feedforward.py")
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
def point(a,b,i,n):
    if not n:return a
    return tuple(x+(1 if y>=x else -1)*((abs(y-x)*i+(n-1)//2)//n) for x,y in zip(a,b))
def span(x):
    G=g.GRID
    if x<=G[0]:return 0,0,0,1
    for i in range(1,len(G)):
        if x<=G[i]:return i-1,i,x-G[i-1],G[i]-G[i-1]
    j=len(G)-1;return j,j,0,1
def trunc(n,d):return n//d if n>=0 else -((-n)//d)
def target(p):
    x,y,z=p
    if z<0:return g.O+13
    if z>=g.H:return g.O
    i,j,u,w=span(x);k,l,v,h=span(y)
    B=g.B
    f=B[i][k]+trunc((B[j][k]-B[i][k])*u,w)
    b=B[i][l]+trunc((B[j][l]-B[i][l])*u,w)
    d=f+trunc((b-f)*v,h)
    if z<=g.F:return d+(13 if z==0 else 0)
    return g.O+trunc((d-g.O)*(g.H-z),g.H-g.F)
def round_chord(a,b,i,n):return (2*(a*(n-i)+b*i)+n)//(2*n)
def prepare(a,b):
    n=max(abs(y-x) for x,y in zip(a,b))
    if not n:return n,[]
    idx={0,n}
    for lo,hi,c,k,bound in g.plan(a,b):
        for j in range(k+1):
            t=(lo+(hi-lo)*Q(j,k))*n
            idx.add(math.floor(t));idx.add(math.ceil(t))
    # Integer path may stay on z=0 for many primary samples. Do not
    # smooth the historical discontinuity by interpolating across it.
    if min(a[2],b[2])<=0<=max(a[2],b[2]) and a[2]!=b[2]:
        asc=b[2]>a[2]
        transitions=[]
        for threshold in (0,1):
            lo,hi=0,n+1
            while lo<hi:
                m=(lo+hi)//2
                z=point(a,b,m,n)[2]
                if (z>=threshold if asc else z<threshold):hi=m
                else:lo=m+1
            transitions.append(lo)
            for j in (lo-1,lo):
                if 0<=j<=n:idx.add(j)
        # Discrete Bresenham may retain z==0 over a broad XY interval.
        # Plan its own bed+13 polynomial, rather than approximating the
        # unrelated continuous negative/positive-Z branch.
        first,last=(transitions[0],transitions[1]-1) if asc else (transitions[1],transitions[0]-1)
        first=max(0,first);last=min(n,last)
        if first<last:
            pa,pb=point(a,b,first,n),point(a,b,last,n)
            assert pa[2]==pb[2]==0
            for lo,hi,c,k,bound in g.plan(pa,pb):
                for j in range(k+1):
                    t=first+(lo+(hi-lo)*Q(j,k))*(last-first)
                    idx.add(math.floor(t));idx.add(math.ceil(t))
    indices=sorted(idx)
    return n,[(i,j,target(point(a,b,i,n)),target(point(a,b,j,n))) for i,j in zip(indices,indices[1:])]

def main():
    pts=((0,0,g.F-1),(3048,3048,g.F),(6095,3048,g.F+1),
        (3048,6095,g.H-1),(6095,6095,g.H),(9142,9142,g.H+1),
        (0,9142,0),(9142,0,1),(3048,3048,-1),(3048,3048,g.H+500))
    paths=[(a,b) for a in pts for b in pts]
    paths.extend([((0,3048,200),(3048,3048,190)),((3048,3048,190),(0,3048,200)),
                  ((0,0,-1),(9142,9142,1)),((9142,9142,1),(0,0,-1))])
    rows=[];samples=0;worst=0;worst_zero=0;worst_positive=0;opposite=0;maxnet=0
    for a,b in paths:
        n,chords=prepare(a,b)
        if not n:continue
        previous=target(a);prevz=a[2];err=0
        for lo,hi,c0,c1 in chords:
            assert c0==previous
            for i in range(lo+1,hi+1):
                p=point(a,b,i,n)
                c=round_chord(c0,c1,i-lo,hi-lo)
                e=abs(c-target(p));err=max(err,e)
                if p[2]<=0:worst_zero=max(worst_zero,e)
                else:worst_positive=max(worst_positive,e)
                dz=p[2]-prevz;net=dz+c-previous
                if net and dz and (net>0)!=(dz>0):opposite+=1
                maxnet=max(maxnet,abs(net))
                prevz=p[2];previous=c;samples+=1
            assert previous==c1
        assert previous==target(b)
        worst=max(worst,err)
        rows.append(dict(a=a,b=b,n=n,chords=len(chords),max_error=err))
    out=dict(schema=1,classification="offline integer Bresenham lattice and geometry; sampled error not universal bound",
             paths=len(rows),samples=samples,max_error_steps=worst,max_error_zero_region=worst_zero,max_error_positive_region=worst_positive,
             max_net_steps_per_primary_sample=maxnet,opposite_net_samples=opposite,
             max_chords=max(x["chords"] for x in rows),rows=rows)
    pathlib.Path(sys.argv[1]).write_text(json.dumps(out,indent=2)+"\n")
    print("Z lattice feedforward",json.dumps({k:v for k,v in out.items() if k!="rows"}),flush=True)
if __name__=="__main__":main()

#!/usr/bin/env python3
"""Offline geometric feedforward candidate; exact rational cubic bounds.

Not firmware, not a planner or real-time scheduling proof. Matrix cell/fade
crossings are internal subintervals, NOT extra PrintLine queue slots.
"""
from fractions import Fraction as Q
import itertools,json,math,pathlib,sys

F=256;H=8656;O=0
GRID=(0,3047,6095,9142) # floor(binary32 mm * stepsPerMm), synthetic
B=((-80,-280,-90,-320),(-310,-30,-390,-40),
   (-60,-420,-20,-240),(-220,-50,-300,0))

def mix(a,b,t):return a+(b-a)*t
def pos(a,b,t):return tuple(mix(x,y,t) for x,y in zip(a,b))
def span(x):
    if x<=GRID[0]:return 0,0,Q(0)
    for i in range(1,len(GRID)):
        if x<=GRID[i]:return i-1,i,Q(x-GRID[i-1],GRID[i]-GRID[i-1])
    i=len(GRID)-1;return i,i,Q(0)
def bed(x,y):
    i,j,u=span(x);k,l,v=span(y)
    return mix(mix(B[i][k],B[j][k],u),mix(B[i][l],B[j][l],u),v)
def geom(p):
    x,y,z=p
    if z<0:return Q(O+13)
    if z==0:return bed(x,y)+13
    if z<=F:return bed(x,y)
    if z>=H:return Q(O)
    return O+(bed(x,y)-O)*Q(H-z,H-F)
def firmware_geom(p):
    # Reproduce staged signed truncation, not round the continuous polynomial.
    x,y,z=p
    if z<0:return O+13
    if z>=H:return O
    i,j,u=span(x);k,l,v=span(y)
    d0=B[i][k]+int((B[j][k]-B[i][k])*u)
    d1=B[i][l]+int((B[j][l]-B[i][l])*u)
    b=d0+int((d1-d0)*v)
    if z<=F:return b+(13 if z==0 else 0)
    return O+int((b-O)*Q(H-z,H-F))
def cuts(a,b):
    t={Q(0),Q(1)}
    for x,y,bounds in zip(a,b,(GRID,GRID,(0,F,H))):
        if x!=y:
            for c in bounds:
                u=Q(c-x,y-x)
                if 0<u<1:t.add(u)
    return sorted(t)

def cubic(a,b,t0,t1):
    # A bilinear cell composed with a linear XYZ path is quadratic in t.
    # Fade multiplies it by a linear factor: degree <=3. Use one-sided
    # interior polynomial at the historical z=0 discontinuity.
    middle=pos(a,b,(t0+t1)/2)
    if middle[2]<0:return (Q(O+13),Q(0),Q(0),Q(0))
    if middle[2]==0:
        fun=lambda t:bed(*pos(a,b,t)[:2])+13
    elif middle[2]<=F:
        fun=lambda t:bed(*pos(a,b,t)[:2])
    elif middle[2]>=H:
        return (Q(O),Q(0),Q(0),Q(0))
    else:
        def fun(t):
            x,y,z=pos(a,b,t)
            return O+(bed(x,y)-O)*Q(H-z,H-F)
    f0,f1,f2,f3=[fun(mix(t0,t1,Q(i,3))) for i in range(4)]
    d1=f1-f0;d2=f2-2*f1+f0;d3=f3-3*f2+3*f1-f0
    return f0,3*d1-Q(3,2)*d2+d3,Q(9,2)*(d2-d3),Q(9,2)*d3

def evaluate(c,u):
    a,b,d,e=c;return a+u*(b+u*(d+u*e))
def ceil_sqrt_fraction(x):
    n=math.isqrt(x.numerator//x.denominator)
    return n if n*n==x else n+1
def plan(a,b,tolerance=Q(1)):
    result=[]
    cts=cuts(a,b)
    for lo,hi in zip(cts,cts[1:]):
        c=cubic(a,b,lo,hi)
        # Uniform subdivisions within polynomial interval, |f-L| <=
        # max|f''|/(8*n²); f'' linear, endpoint maxima exact.
        curv=max(abs(2*c[2]),abs(2*c[2]+6*c[3]))
        n=max(1,ceil_sqrt_fraction(curv/(8*tolerance)))
        result.append((lo,hi,c,n,curv/Q(8*n*n)))
    return result

def main():
    pts=((0,0,F-1),(3048,3048,F),(6095,3048,F+1),
        (3048,6095,H-1),(6095,6095,H),(9142,9142,H+1),
        (0,9142,0),(9142,0,1),(3048,3048,-1),(3048,3048,H+500))
    rows=[];max_error=Q(0);max_quant_error=Q(0);nchecks=0
    for a,b in itertools.product(pts,repeat=2):
        pieces=plan(a,b);rev=plan(b,a)
        assert sum(p[3] for p in pieces)==sum(p[3] for p in rev)
        for lo,hi,c,n,bound in pieces:
            assert bound<=1
            for i in range(n):
                u0=Q(i,n);u1=Q(i+1,n)
                v0=evaluate(c,u0);v1=evaluate(c,u1)
                for j in range(9):
                    u=mix(u0,u1,Q(j,8));t=mix(lo,hi,u)
                    true=evaluate(c,u)
                    if 0<j<8:assert true==geom(pos(a,b,t))
                    err=abs(true-mix(v0,v1,Q(j,8)))
                    assert err<=bound;max_error=max(max_error,err)
                    # Compare rounded absolute coordinates to historical
                    # staged integer truncation. Zero discontinuity is
                    # intentionally not hidden by smoothing.
                    p=tuple(round(x) for x in pos(a,b,t))
                    if p[2]>0 and pos(a,b,t)[2]>0:
                        max_quant_error=max(max_quant_error,abs(firmware_geom(p)-mix(v0,v1,Q(j,8))))
                    nchecks+=1
        rows.append(dict(a=a,b=b,region_intervals=len(pieces),internal_chords=sum(p[3] for p in pieces),
                         zero_crossing=(min(a[2],b[2])<=0<=max(a[2],b[2]))))
    out=dict(schema=1,classification="offline exact rational synthetic geometry, no firmware timing or queue implementation",
        paths=len(rows),checks=nchecks,max_continuous_chord_error=float(max_error),
        max_sampled_legacy_quantization_difference=float(max_quant_error),
        max_internal_chords=max(x["internal_chords"] for x in rows),rows=rows)
    pathlib.Path(sys.argv[1]).write_text(json.dumps(out,indent=2)+"\n")
    print("Z feedforward",json.dumps({k:v for k,v in out.items() if k!="rows"}),flush=True)
if __name__=="__main__":main()

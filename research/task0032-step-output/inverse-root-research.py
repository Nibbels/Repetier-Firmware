#!/usr/bin/env python3
"""Quake-style reciprocal sqrt vs conservative AVR integer tables.
Research only. No production source changes. Real-valued speed bounds,
not full motor acceleration/interrupt or planner qualification.
"""
import argparse, hashlib, json, math, pathlib, re, statistics, subprocess
HERE=pathlib.Path(__file__).resolve().parent
F=16_000_000
W=1024
# Normalized high word m: [16384,65536]. sqrt table lower chord,
# reciprocal period Q6 upper chord (convex), both rounded safely.
M=list(range(16384,65537,W))
ROOT=[math.isqrt(m<<14) for m in M]
P=[]
for m in M:
    y=math.isqrt(4_000_000**2//m)
    P.append(y+(y*y*m<4_000_000**2))

def norm(x):
    k=0
    while x<0x40000000:x<<=2;k+=1
    return x>>16,k
def table_values(m,k):
    i=(m-16384)//W;f=m%W
    root=(ROOT[i]+((ROOT[i+1]-ROOT[i])*f>>10))*2>>k
    p=P[i]-((P[i]-P[i+1])*f>>10)
    period=p<<(k-6) if k>=6 else (p+(1<<(6-k))-1)>>(6-k)
    return root,period

def validate():
    cells=0;worst_p=(0,None);worst_r=(0,None);worst_r_large=(0,None);worst_p_z=(0,None)
    # Exhaust every representable (normalized high word, shift) bucket.
    # Function constant inside each bucket, true root/period monotone.
    # Endpoints therefore cover every positive uint32_t, not sampling.
    for k in range(16):
        scale=1<<(2*k)
        for m in range(16384,65536):
            lo=((m<<16)+scale-1)//scale
            hi=(((m+1)<<16)-1)//scale
            if lo>hi:continue
            r,p=table_values(m,k)
            assert r*r<=lo and p*p*lo>=F*F,(m,k,lo,r,p)
            err=p*math.sqrt(hi)/F-1
            if err>worst_p[0]:worst_p=(err,hi)
            if lo>=256000 and hi<=132710400 and err>worst_p_z[0]:worst_p_z=(err,hi)
            err=1-r/math.sqrt(hi)
            if err>worst_r[0]:worst_r=(err,hi)
            if hi>=262144 and err>worst_r_large[0]:worst_r_large=(err,hi)
            cells+=1
    # Adjacent boundary speeds: sqrt(l(l+1)) >= l+13/32 for l>=1.
    # Thus average speed squared >= A*(l+29/64). Use this lower bound
    # to get a conservative period WITHOUT calculating either root.
    profiles=0;max_over=(0,None);max_default=(0,None);shorter_than_floor_ref=0
    for A in (2,4,12800,256000,512000,2_000_000,1_000_000_000,4294836224):
        for l in range(0,min(10000,4294836225//A-1)+1):
            n=l+1
            exact=2*F/(math.sqrt(A*l)+math.sqrt(A*n))
            x=A if not l else A*l+(A//64)*29+((A%64)*29//64)
            p=table_values(*norm(x))[1]*(2 if not l else 1)
            assert p>=exact,(A,l,p,exact)
            floorref=math.ceil(2*F/(math.isqrt(A*l)+math.isqrt(A*n)))
            shorter_than_floor_ref+=p<floorref
            e=p/exact-1
            if e>max_over[0]:max_over=(e,(A,l))
            if A==256000 and l<518 and e>max_default[0]:max_default=(e,(A,l))
            profiles+=1
    return dict(bucket_count=cells,period_max_rel_excess=worst_p,
                period_z_domain_max_rel_excess=worst_p_z,
                root_max_rel_shortfall=worst_r,root_large_max_rel_shortfall=worst_r_large,
                transition_checks=profiles,transition_max_rel_excess=max_over,
                transition_default_max_rel_excess=max_default,
                transitions_shorter_than_integer_floor_reference=shorter_than_floor_ref,
                table_bytes=2*len(M))

def header():
    return """#pragma once
#include <stdint.h>
#include <string.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#define STORAGE PROGMEM
#define READ_ENTRY(p) pgm_read_word(p)
#else
#define STORAGE
#define READ_ENTRY(p) (*(p))
#endif
namespace InverseRootResearch {
static const uint16_t roots[] STORAGE={"""+",".join(map(str,ROOT))+"""};
static const uint16_t periods[] STORAGE={"""+",".join(map(str,P))+"""};
struct Normal {uint16_t m; uint8_t k;};
inline Normal normalize(uint32_t x) {
    // Requires x>0. Byte-sized shift groups reduce the normalization loop.
    uint8_t k=0;
    if(x<0x10000UL){x<<=16;k+=8;}
    if(x<0x1000000UL){x<<=8;k+=4;}
    if(x<0x10000000UL){x<<=4;k+=2;}
    if(x<0x40000000UL){x<<=2;++k;}
    return {uint16_t(x>>16),k};
}
inline uint16_t lookup(const uint16_t* table,uint16_t m,bool descending) {
    const uint8_t i=uint8_t((m-16384U)>>10);
    const uint16_t f=m&1023U;
    const uint16_t a=READ_ENTRY(table+i),b=READ_ENTRY(table+i+1);
    const uint16_t diff=descending?a-b:b-a;
    const uint16_t part=uint16_t((uint32_t(diff)*f)>>10);
    return descending?a-part:a+part;
}
inline uint16_t lowerRoot(uint32_t x) {
    if(!x)return 0;
    const auto n=normalize(x);
    return uint16_t((uint32_t(lookup(roots,n.m,false))<<1)>>n.k);
}
inline uint32_t upperPeriod(uint32_t x) {
    if(!x)return UINT32_MAX;
    const auto n=normalize(x);
    const uint16_t p=lookup(periods,n.m,true);
    if(n.k>=6)return uint32_t(p)<<(n.k-6);
    const uint8_t s=6-n.k;
    return (uint32_t(p)+(1U<<s)-1)>>s;
}
// Direct period for equal or adjacent energy levels. No two square roots,
// no division, no 64-bit multiplication. A=2*a, l=min(level,nextLevel).
// Configuration must keep A*maxLevel within uint32_t.
inline uint32_t transitionPeriod(uint32_t A,uint32_t l,bool adjacent) {
    if(!adjacent)return upperPeriod(A*l);
    if(!l)return 2*upperPeriod(A);
    return upperPeriod(A*l+(A>>6)*29+((A&63)*29>>6));
}
struct DirectLimits {
    uint32_t A, cap, fraction;
    static DirectLimits prepare(uint32_t A,uint32_t cap) {
        return {A,cap,(A>>6)*29+((A&63)*29>>6)};
    }
};
struct DirectPulse {uint32_t level,interval;int8_t sign;bool restart;};
class DirectRamp {
    uint32_t level_=0;
    int8_t direction_=0;
public:
    DirectPulse plan(int32_t error,const DirectLimits& lim) const {
        if(!error||!lim.cap)return {0,0,0,false};
        const uint32_t d=error<0?uint32_t(0)-uint32_t(error):uint32_t(error);
        const int8_t sign=error>0?1:-1;
        const bool restart=level_&&(direction_!=sign||d<level_||level_>lim.cap);
        const uint32_t l=restart?0:level_;
        if(d==1&&!l)return {0,2*upperPeriod(lim.A/2),sign,restart};
        const uint32_t n=d<=l?l-1:(d-l>=2&&l<lim.cap?l+1:l);
        const uint32_t low=l<n?l:n;
        const uint32_t interval=l==n?upperPeriod(lim.A*l):
            !low?2*upperPeriod(lim.A):upperPeriod(lim.A*low+lim.fraction);
        return {n,interval,sign,restart};
    }
    void commit(const DirectPulse& p) {level_=p.level;direction_=p.sign;}
};
inline float quake(float x) {
    uint32_t bits; memcpy(&bits,&x,4);
    bits=0x5f3759dfUL-(bits>>1);
    float y;memcpy(&y,&bits,4);
    return y*(1.5f-0.5f*x*y*y);
}
}
"""
FIXTURE=r'''
#include <avr/interrupt.h>
#include <math.h>
#include "inverse-root-generated.h"
#include "z-correction-clock.h"
using namespace InverseRootResearch;
class HAL { public: static uint16_t integerSqrt(int32_t); static int32_t CPUDivU2(unsigned int); };
#include "actual-hal-math.inc"
struct FastMath {
    static uint16_t floorRoot(uint32_t x) {
        if(x>INT32_MAX)return ZCorrectionResearch::root(x);
        if(!x)return 0;
        uint16_t r=HAL::integerSqrt(int32_t(x));
        return uint32_t(r)*r>x?r-1:r;
    }
    static uint32_t period(uint16_t v) {return HAL::CPUDivU2(v);}
};
extern "C" { volatile uint8_t phase=0,fixture_id=0,queue_depth=0; }
volatile uint32_t input,output,energy;
volatile float floatInput,floatOutput;
static void mark(uint8_t p) {asm volatile("" ::: "memory");phase=p;asm volatile("" ::: "memory");}
static void require(bool b){if(!b){mark(254);mark(255);while(true){}}}
int main(){
    cli();
    const uint32_t vals[]={1,4,9,25,100,384,10000,128000,256000,512000,1000000,16384000,132710400,1000000000,2147483647UL};
    for(uint8_t i=0;i<sizeof(vals)/sizeof(vals[0]);i++){
        fixture_id=0;queue_depth=i;input=vals[i];floatInput=float(input);
        uint32_t x=input;float f=floatInput;
        mark(1);asm volatile("" : "+r"(x) :: "memory");output=HAL::integerSqrt(int32_t(x));mark(129);
        x=input;mark(2);asm volatile("" : "+r"(x) :: "memory");output=lowerRoot(x);mark(130);
        require(output==lowerRoot(input)&&uint32_t(output)*output<=input);
        f=floatInput;mark(3);asm volatile("" : "+r"(f) :: "memory");floatOutput=sqrtf(f);mark(131);
        f=floatInput;mark(4);asm volatile("" : "+r"(f) :: "memory");floatOutput=quake(f);mark(132);
        f=floatInput;mark(5);asm volatile("" : "+r"(f) :: "memory");floatOutput=16000000.0f*quake(f);mark(133);
        x=input;mark(6);asm volatile("" : "+r"(x) :: "memory");output=upperPeriod(x);mark(134);
        require(uint64_t(output)*output*input>=UINT64_C(256000000000000));
        f=floatInput;mark(7);asm volatile("" : "+r"(f) :: "memory");floatOutput=16000000.0f/sqrtf(f);mark(135);
        x=input;mark(8);asm volatile("" : "+r"(x) :: "memory");
        uint16_t r=HAL::integerSqrt(int32_t(x));if(uint32_t(r)*r>x)--r;
        output=r<10?ZCorrectionResearch::ceilDiv(16000000UL,r):HAL::CPUDivU2(r)+1;mark(136);
    }
    const uint32_t levels[]={0,1,2,4,8,16,32,64,128,256,517};
    energy=256000;
    for(uint8_t i=0;i<sizeof(levels)/sizeof(levels[0]);i++){
        fixture_id=1;queue_depth=i;input=levels[i];
        uint32_t a=energy,l=input;
        mark(9);asm volatile("" : "+r"(a),"+r"(l) :: "memory");output=transitionPeriod(a,l,true);mark(137);
        require(output==transitionPeriod(energy,input,true));
    }
    for(uint8_t j=0;j<3;j++){
        const uint32_t a=j==0?6400:j==1?128000:256000;
        const auto lim=ZCorrectionResearch::Limits::prepare(a,11520);
        const auto dl=DirectLimits::prepare(lim.twiceAcceleration,lim.maxLevel);
        ZCorrectionResearch::FastRamp<FastMath> fast;
        DirectRamp direct;
        fixture_id=2+j;
        for(uint8_t i=0;i<32;i++){
            queue_depth=i;
            const int32_t error=i<16?320:-(32-i);
            input=uint32_t(error);
            int32_t e=int32_t(input);
            mark(10);asm volatile("" : "+r"(e) :: "memory");
            auto fp=fast.plan(e,lim);output=fp.interval+fp.nextLevel;mark(138);
            e=int32_t(input);mark(11);asm volatile("" : "+r"(e) :: "memory");
            auto dp=direct.plan(e,dl);output=dp.interval+dp.level;mark(139);
            require(fp.nextLevel==dp.level&&fp.sign==dp.sign&&fp.safetyRestart==dp.restart);
            fast.commit(fp);direct.commit(dp);
        }
    }
    mark(255);while(true){}
}
'''
def run(args):
    p=subprocess.run([str(x) for x in args],capture_output=True,text=True)
    if p.returncode:print(p.stdout[-3000:]);print(p.stderr);p.check_returncode()
    return p.stdout
def main():
    p=argparse.ArgumentParser()
    p.add_argument("--out",type=pathlib.Path,required=True)
    p.add_argument("--compiler",type=pathlib.Path)
    p.add_argument("--runner-source",type=pathlib.Path)
    a=p.parse_args();d=a.out.resolve();d.mkdir(parents=True,exist_ok=True)
    result=validate()
    print("Inverse root integer bounds",json.dumps(result),flush=True)
    (d/"inverse-root-generated.h").write_text(header())
    (d/"bounds.json").write_text(json.dumps(result,indent=2)+"\n")
    if not a.compiler:return
    cc=a.compiler.resolve();src=d/"fixture.cpp";src.write_text(FIXTURE)
    hal=(HERE.parents[1]/"Repetier/HAL.cpp").read_text()
    begin=hal.index("uint16_t HAL::integerSqrt(")
    end=hal.index("} // CPUDivU2",begin)+len("} // CPUDivU2")
    old=': "=r"(b)\n        : "r"(a)\n        : "r18", "r19", "r27", "r26");'
    new=': "=r"(b), "+r"(a)\n        :\n        : "r18", "r19", "r27", "r26", "cc");'
    assert hal[begin:end].count(old)==1
    (d/"actual-hal-math.inc").write_text(hal[begin:end].replace(old,new))
    elf=d/"inverse-root.elf"
    flags=["-mmcu=atmega2560","-DF_CPU=16000000UL","-std=gnu++11","-Os","-flto","-ffunction-sections","-fdata-sections","-fno-exceptions","-Wl,--gc-sections"]
    run([cc,*flags,"-I"+str(HERE),src,"-Wl,-Map="+str(d/"inverse-root.map"),"-o",elf])
    syms={}
    for line in run([cc.with_name("avr-nm"),"-S","-n",elf]).splitlines():
        m=re.match(r"^([0-9a-f]+)\s+[0-9a-f]+\s+\w\s+(phase|fixture_id|queue_depth)$",line)
        if m:syms[m[2]]=int(m[1],16)-0x800000
    assert len(syms)==3,syms
    runner=d/"runner";run(["cc","-O2",a.runner_source,"-lsimavr","-lelf","-o",runner])
    trace=run([runner,elf,*[hex(syms[k]) for k in ("phase","fixture_id","queue_depth")]])
    (d/"markers.csv").write_text(trace)
    (d/"disassembly.txt").write_text(run([cc.with_name("avr-objdump"),"-d","-C","-z",elf]))
    (d/"size.txt").write_text(run([cc.with_name("avr-size"),"-A",elf]))
    rows=[];pending=None;done=False
    for line in trace.splitlines():
        if not line.startswith("E,"):continue
        _,t,f,i,mark=line.split(",");t,f,i,mark=map(int,(t,f,i,mark))
        if mark==254:raise ValueError((f,i))
        if mark==255:done=True;break
        if mark<128:assert pending is None;pending=(t,f,i,mark)
        else:
            start,ff,ii,stage=pending
            assert (ff,ii,stage)==(f,i,mark-128)
            rows.append(dict(fixture=f,index=i,stage=stage,cycles=t-start));pending=None
    assert done and pending is None and len(rows)==15*8+11+3*32*2
    names=["HAL integer sqrt","lower table sqrt","sqrtf","Quake invsqrt","Quake period (float)","upper table period","float sqrt/div period","HAL floor root + reciprocal","direct adjacent ramp period","cached HAL full ramp","direct table full ramp"]
    summary={}
    for stage,name in enumerate(names,1):
        times=[r["cycles"] for r in rows if r["stage"]==stage]
        summary[name]=dict(n=len(times),min=min(times),median=statistics.median(times),max=max(times))
    (d/"results.json").write_text(json.dumps(dict(bounds=result,summary=summary,rows=rows,flags=flags,elf_sha256=hashlib.sha256(elf.read_bytes()).hexdigest()),indent=2)+"\n")
    print("Inverse root AVR",json.dumps(summary),flush=True)
if __name__=="__main__":main()

#!/usr/bin/env python3
"""Measure foreground chord construction including actual matrix reads.
No real-time admission or interrupt nesting; lengths recorded separately.
"""
import argparse,json,pathlib,re,statistics,subprocess
HERE=pathlib.Path(__file__).resolve().parent
FW=HERE.parents[1]/"Repetier"
MAIN=r'''
extern "C" { volatile uint8_t phase=0,fixture_id=0,queue_depth=0; }
volatile int32_t sink;
volatile uint32_t pathIndex;
static void mark(uint8_t p){asm volatile("" ::: "memory");phase=p;asm volatile("" ::: "memory");}
int main(){
    cli();
    const Path paths[]={
        {{1543,1881,256},{1547,1884,256},4},
        {{1543,1881,256},{9142,9142,256},7599},
        {{1543,1881,256},{1543,1881,704},448},
        {{4950,7148,9200},{3238,7675,256},8944},
        {{0,0,-1},{9142,9142,1},9142},
        {{0,0,10000},{30000,20000,10000},30000},
        {{3048,3048,256},{3048,3048,256},100000},
        {{3048,3048,256},{3048,3048,256000},255744}
    };
    for(uint8_t k=0;k<8;++k){
        pathIndex=k;Path p=paths[pathIndex];
        Preparer prep;Reader r;Prepared c{};
        if(!prep.start(p)){mark(254);mark(255);while(true){}}
        uint8_t i=0;
        while(prep.index()<p.primary){
            fixture_id=k;queue_depth=i;
            mark(1);const bool ok=prep.next(r,c);sink=c.motorEnd;mark(129);
            if(!ok||!c.length||i==255){mark(254);mark(255);while(true){}}
            queue_depth=uint8_t(c.length);mark(140);
            queue_depth=uint8_t(c.length>>8);mark(141);
            ++i;
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
    p.add_argument("--compiler",type=pathlib.Path,required=True)
    p.add_argument("--runner-source",type=pathlib.Path,required=True)
    a=p.parse_args();d=a.out.resolve();d.mkdir(parents=True,exist_ok=True)
    fixture=(HERE/"z-chord-preparer-replay.cpp").read_text()
    reader=fixture[fixture.index('#include "BedCompensation.h"'):fixture.index('static int32_t roundChord')]
    reader=reader.replace("struct Reader {","template<class T> T maximum(T a,T b){return a>b?a:b;}\nstruct Reader {").replace("std::max(","maximum(")
    src=d/"fixture.cpp";src.write_text('#include <avr/interrupt.h>\n#include "z-chord-preparer.h"\n'+reader+MAIN)
    cc=a.compiler.resolve();elf=d/"preparer.elf"
    flags=["-mmcu=atmega2560","-DF_CPU=16000000UL","-std=gnu++11","-Os","-flto","-ffunction-sections","-fdata-sections","-fno-exceptions","-Wl,--gc-sections"]
    run([cc,*flags,"-I"+str(HERE),"-I"+str(FW),src,"-o",elf])
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
        if mark==1:
            assert pending is None;pending=(t,f,i)
        elif mark==129:
            start,ff,ii=pending;assert (ff,ii)==(f,i)
            rows.append(dict(fixture=f,index=i,cycles=t-start));pending=None
        elif mark==140:rows[-1]["length"]=i
        elif mark==141:rows[-1]["length"]|=i<<8
        else:raise AssertionError(mark)
    assert done and pending is None
    summary={}
    for f in range(8):
        v=[r for r in rows if r["fixture"]==f];assert v and all(r["length"]>0 for r in v)
        costs=[r["cycles"] for r in v]
        # At 15,238 primary steps/s, next-chord prep would need this
        # fraction of current chord time even with *all* CPU foreground.
        ratios=[b["cycles"]/(a["length"]*16_000_000/15238) for a,b in zip(v,v[1:])]
        summary[str(f)]=dict(chords=len(v),samples=sum(r["length"] for r in v),min=min(costs),median=statistics.median(costs),max=max(costs),short_le4=sum(r["length"]<=4 for r in v),
                            successor_over_full_cpu_window=sum(x>1 for x in ratios),max_successor_full_cpu_ratio=max(ratios,default=0))
    assert [x["samples"] for x in summary.values()]==[4,7599,448,8944,9142,30000,100000,255744]
    (d/"results.json").write_text(json.dumps(dict(summary=summary,rows=rows,flags=flags),indent=2)+"\n")
    print("Z foreground preparer AVR",json.dumps(summary),flush=True)
if __name__=="__main__":main()

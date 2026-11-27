#!/usr/bin/env python3
"""Isolated actual AVR timing for the active integer chord, not full ISR."""
import argparse,pathlib,re,subprocess,statistics,json
HERE=pathlib.Path(__file__).resolve().parent
FIXTURE=r'''
#include <avr/interrupt.h>
#include "z-feedforward-dda.h"
extern "C" { volatile uint8_t phase=0,fixture_id=0,queue_depth=0; }
volatile int32_t first,last,out;
volatile uint32_t length;
static void mark(uint8_t p){asm volatile("" ::: "memory");phase=p;asm volatile("" ::: "memory");}
int main(){
    cli();
    const int32_t starts[]={-32768,32780,-420,-50,0,-13};
    const int32_t ends[]={32780,-32768,-20,-50,0,13};
    const uint32_t lengths[]={129,257,1000,65535,65536,2147483647UL};
    for(uint8_t j=0;j<6;j++){
        first=starts[j];last=ends[j];length=lengths[j];
        fixture_id=j;queue_depth=0;
        ZFeedforwardResearch::Chord c;
        mark(1);
        bool ok=c.prepare(first,last,length);out=c.value();
        mark(129);
        if(!ok){mark(254);mark(255);while(true){}}
        for(uint8_t i=0;i<128;i++){
            queue_depth=i;
            mark(2);ok=c.advance();out=c.value();mark(130);
            if(!ok){mark(254);mark(255);while(true){}}
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
    cc=a.compiler.resolve();src=d/"fixture.cpp";src.write_text(FIXTURE)
    elf=d/"chord.elf"
    flags=["-mmcu=atmega2560","-DF_CPU=16000000UL","-std=gnu++11","-Os","-flto","-ffunction-sections","-fdata-sections","-fno-exceptions","-Wl,--gc-sections"]
    run([cc,*flags,"-I"+str(HERE),src,"-o",elf])
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
    assert done and pending is None and len(rows)==6*129
    result={}
    for stage,name in ((1,"prepare"),(2,"advance")):
        v=[r["cycles"] for r in rows if r["stage"]==stage]
        result[name]=dict(n=len(v),min=min(v),median=statistics.median(v),max=max(v))
    (d/"results.json").write_text(json.dumps(dict(summary=result,rows=rows,flags=flags),indent=2)+"\n")
    print("Z chord AVR",json.dumps(result),flush=True)
if __name__=="__main__":main()

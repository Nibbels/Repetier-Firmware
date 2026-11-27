#!/usr/bin/env python3
"""Measure isolated AVR correction plan, not actual ISR or physical motion."""
import argparse, hashlib, json, pathlib, re, statistics, subprocess
HERE=pathlib.Path(__file__).resolve().parent
FIXTURE=r'''
#include <avr/interrupt.h>
#include "z-correction-clock.h"
using namespace ZCorrectionResearch;
extern "C" { volatile uint8_t phase=0, fixture_id=0, queue_depth=0; }
volatile int32_t errorInput;
volatile uint32_t accelerationInput;
volatile uint16_t speedInput;
volatile uint32_t sink;
static void mark(uint8_t p) {
    asm volatile("" ::: "memory");
    phase=p;
    asm volatile("" ::: "memory");
}
static void require(bool b) {
    if(!b){mark(254);mark(255);while(true){}}
}
int main() {
    cli();
    const uint32_t accelerations[]={6400,128000,256000};
    const uint16_t speeds[]={384,2000,11520};
    for(uint8_t a=0;a<3;++a)for(uint8_t v=0;v<3;++v) {
        accelerationInput=accelerations[a];speedInput=speeds[v];
        const auto limits=Limits::prepare(accelerationInput,speedInput);
        Ramp ramp;
        fixture_id=a*3+v;
        for(uint8_t i=0;i<32;++i) {
            queue_depth=i;
            errorInput=i<16?320:-(32-i);
            int32_t err=errorInput;
            mark(1);
            asm volatile("" : "+r"(err) :: "memory");
            const auto plan=ramp.plan(err,limits);
            sink=plan.interval+plan.nextLevel;
            mark(129);
            require(plan.sign==(err>0?1:-1)&&plan.interval>0);
            ramp.commit(plan);
        }
        queue_depth=32;
        errorInput=0;
        mark(1);const auto stop=ramp.plan(errorInput,limits);
        sink=stop.interval;mark(129);require(!stop.sign);
        Deadlines d{0xfffffff0UL,0x20UL,true};
        queue_depth=33;
        uint32_t now=0xffffffe0UL+uint32_t(errorInput);
        mark(2);asm volatile("" : "+r"(now) :: "memory");sink=d.nextDelay(now);mark(130);
        require(sink==16);
    }
    mark(255);while(true){}
}
'''
def run(args):
    p=subprocess.run([str(x) for x in args],capture_output=True,text=True)
    if p.returncode:
        print(p.stdout[-3000:]);print(p.stderr);p.check_returncode()
    return p.stdout
def main():
    p=argparse.ArgumentParser()
    p.add_argument("--compiler",type=pathlib.Path,required=True)
    p.add_argument("--runner-source",type=pathlib.Path,required=True)
    p.add_argument("--out",type=pathlib.Path,required=True)
    a=p.parse_args();d=a.out.resolve();d.mkdir(parents=True,exist_ok=True)
    cc=a.compiler.resolve();src=d/"fixture.cpp";src.write_text(FIXTURE)
    elf=d/"z-clock.elf"
    flags=["-mmcu=atmega2560","-std=gnu++11","-Os","-flto","-ffunction-sections","-fdata-sections","-fno-exceptions","-Wl,--gc-sections"]
    run([cc,*flags,"-I"+str(HERE),src,"-Wl,-Map="+str(d/"z-clock.map"),"-o",elf])
    syms={}
    for line in run([cc.with_name("avr-nm"),"-S","-n",elf]).splitlines():
        m=re.match(r"^([0-9a-f]+)\s+[0-9a-f]+\s+\w\s+(phase|fixture_id|queue_depth)$",line)
        if m:syms[m[2]]=int(m[1],16)-0x800000
    assert len(syms)==3,syms
    runner=d/"runner";run(["cc","-O2",a.runner_source,"-lsimavr","-lelf","-o",runner])
    trace=run([runner,elf,*[hex(syms[k]) for k in ("phase","fixture_id","queue_depth")]])
    (d/"markers.csv").write_text(trace)
    records=[];pending=None;done=False
    for line in trace.splitlines():
        if not line.startswith("E,"):continue
        _,t,f,i,mark=line.split(",");t,f,i,mark=map(int,(t,f,i,mark))
        if mark==254:raise ValueError((f,i))
        if mark==255:done=True;break
        if mark<128:
            assert pending is None;pending=(t,f,i,mark)
        else:
            start,ff,ii,stage=pending
            assert (ff,ii,stage)==(f,i,mark-128)
            records.append(dict(fixture=f,index=i,stage=stage,cycles=t-start));pending=None
    assert done and pending is None and len(records)==9*34
    (d/"disassembly.txt").write_text(run([cc.with_name("avr-objdump"),"-d","-C","-z",elf]))
    (d/"size.txt").write_text(run([cc.with_name("avr-size"),"-A",elf]))
    summary={}
    for label,sel in (("ramp",lambda r:r["stage"]==1 and r["index"]<32),
                      ("settled",lambda r:r["stage"]==1 and r["index"]==32),
                      ("mux",lambda r:r["stage"]==2)):
        costs=[r["cycles"] for r in records if sel(r)]
        summary[label]=dict(n=len(costs),min=min(costs),median=statistics.median(costs),max=max(costs))
    (d/"results.json").write_text(json.dumps(dict(schema=1,classification="isolated AVR marker fixture incl marker/sink overhead; no concurrent IRQs",header_sha256=hashlib.sha256((HERE/"z-correction-clock.h").read_bytes()).hexdigest(),elf_sha256=hashlib.sha256(elf.read_bytes()).hexdigest(),flags=flags,records=records,summary=summary),indent=2)+"\n")
    print("Z clock AVR",json.dumps(summary),flush=True)
if __name__=="__main__":main()

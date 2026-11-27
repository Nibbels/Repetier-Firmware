#!/usr/bin/env python3
"""Isolated actual-source AVR benchmark; not integrated firmware WCET.

Copies actual source/header files into a temporary directory, extracts exact HAL
and geometry functions. Printer/Extruder state is a fixture with real accessor
semantics, not actual full object layout. Compiler flags match the pinned tool
family; stage instrumentation changes allocation, so compare outer uninstrumented
timings separately from instrumented substage costs. No hardware run.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
from tempfile import TemporaryDirectory

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
FW=ROOT/"Repetier"
FLAGS=["-mmcu=atmega2560","-DF_CPU=16000000UL","-Os","-flto","-std=gnu++11",
       "-ffunction-sections","-fdata-sections","-fno-exceptions","-fno-threadsafe-statics"]
PRINTER=r'''
#pragma once
#include "ExtrusionOutput.h"
#include "IdleStatus.h"
class Printer {
public:
    static float maxFeedrate_[4],maxXYJerk_,maxZJerk_;
    static uint32_t maxPrintAccelerationStepsPerSquareSecond_[4],maxTravelAccelerationStepsPerSquareSecond_[4];
    static float axisMaxFeedrate(uint8_t axis){return maxFeedrate_[axis];}
    static float xyJerkLimit(){return maxXYJerk_;}
    static float zJerkLimit(){return maxZJerk_;}
    static const uint32_t* accelerationSteps(bool printing){
        return printing?maxPrintAccelerationStepsPerSquareSecond_:maxTravelAccelerationStepsPerSquareSecond_;
    }
    static IdleStatus idleStatus;
    static ExtrusionOutput extrusionOutput;
};
class Extruder {
public:
    float advanceL=0,maxEJerk=6;
    static Extruder* current;
};
'''
HEADER=r'''
#pragma once
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#define INLINE __attribute__((always_inline))
#define EXTRUDER_TIMSK TIMSK0
#define EXTRUDER_OCIE OCIE0A
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3
typedef uint16_t speed_t;
typedef uint32_t millis_t;
typedef uint8_t flag8_t;
typedef int8_t fast8_t;
'''
TRACE=r'''
extern "C" volatile uint8_t phase;
struct ResearchStage {
    uint8_t id;
    explicit ResearchStage(uint8_t v):id(v){asm volatile("" ::: "memory");phase=id;asm volatile("" ::: "memory");}
    ~ResearchStage(){asm volatile("" ::: "memory");phase=128+id;asm volatile("" ::: "memory");}
};
'''
def section(s,start,end):
    first=s.index(start)
    return s[first:s.index(end,first+len(start))]
def define(s,name):
    return re.search(r"^#define\s+"+name+r"\s+[^\n]+",s,re.M)[0]
def run(args):
    p=subprocess.run([str(x) for x in args],check=True,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    return p.stdout
def load(name):
    return (FW/name).read_text(encoding="utf-8")
def stage_instrument(s):
    for name,i in [("limitJunction",3),("planDeceleration",4),("planAcceleration",5),("prepareStepProfile",6)]:
        pattern=r"(void MotionPlanner::"+name+r"\([^\n]*\)\s*\{)"
        s,n=re.subn(pattern,r"\1\n    ResearchStage research_stage("+str(i)+");",s)
        assert n==1,name
    return s

def create_fixture(d,capacity,instrument):
    for name in ("motion.h","MoveQueue.h","MotionPlanner.h","QueueTask.h","StepExecutor.h","RMath.h","ExtrusionOutput.h","IdleStatus.h","MoveQueue.cpp"):
        shutil.copyfile(FW/name,d/name)
    hal=load("HAL.h")
    header=HEADER+"\n#define MOVE_CACHE_SIZE "+str(capacity)+"\n"
    for src,key in [(load("Configuration.h"),"MAX_JERK_DISTANCE"),(load("RF2000.h"),"MOVE_CACHE_LOW"),(load("RF2000.h"),"LOW_TICKS_PER_MOVE"),(hal,"LIMIT_INTERVAL")]:
        header+=define(src,key)+"\n"
    header+=section(hal,"class InterruptProtectedBlock {","\n#define EEPROM_OFFSET")
    header+="\nclass HAL { public: static inline void forbidInterrupts(){cli();}\n"
    header+=section(hal,"    static inline unsigned long U16SquaredToU32(","\n    static inline speed_t ComputeV(")+"\n};\n"
    header+='\n#define private public\n#include "RMath.h"\n#include "Printer.h"\n#include "motion.h"\n#include "MoveQueue.h"\n#undef private\n'
    header+=TRACE
    (d/"Repetier.h").write_text(header)
    (d/"Printer.h").write_text(PRINTER)
    planner=load("MotionPlanner.cpp")
    (d/"MotionPlanner.cpp").write_text(stage_instrument(planner) if instrument else planner)
    motion=load("motion.cpp")
    geometry='#include "Repetier.h"\n'+section(motion,"void PrintLine::setAxisSteps(","\n/** \\brief Put a move")+section(motion,"void PrintLine::fitRampToRemainingSteps()","\n/** \\brief Check if move is new.")
    (d/"geometry.cpp").write_text(geometry)
    shutil.copyfile(HERE/"avr-fixture.cpp",d/"avr-fixture.cpp")
def parse_trace(raw):
    masks=[];start_mask=None;stack=[];rows=[]
    for line in raw.splitlines():
        if line.startswith("I,"):
            _,t,i=line.split(",");t=int(t)
            if int(i)==0:start_mask=t
            elif start_mask is not None:masks.append((start_mask,t));start_mask=None
        elif line.startswith("E,"):
            _,t,f,d,p=line.split(",");t,f,d,p=map(int,(t,f,d,p))
            if p==255:
                assert not stack
                break
            if p<128:stack.append([p,t,f,d,0])
            else:
                stage,start,fixture,depth,children=stack.pop()
                assert p==stage+128 and f==fixture and d==depth,(p,stage,f,fixture,d,depth)
                inclusive=t-start
                if stack:stack[-1][4]+=inclusive
                rows.append(dict(stage=stage,fixture=fixture,depth=depth,start=start,end=t,
                                 inclusive_cycles=inclusive,exclusive_cycles=inclusive-children))
    else:raise ValueError("fixture never reached done marker")
    for r in rows:
        a,b=r["start"],r["end"]
        r["max_sampled_mask_cycles"]=max([0]+[max(0,min(b,y)-max(a,x)) for x,y in masks if x<b and y>a])
    return rows
def stats(rows):
    names={1:"prepareMove",2:"updateLookahead",3:"limitJunction",4:"planDeceleration",5:"planAcceleration",6:"prepareStepProfile",7:"remainingMotion",8:"exactTimeOnly",9:"lower",10:"pacedTicks"}
    out={}
    for s in sorted({r["stage"] for r in rows}):
        rr=[r for r in rows if r["stage"]==s]
        x=sorted(r["inclusive_cycles"] for r in rr)
        e=sorted(r["exclusive_cycles"] for r in rr)
        out[names[s]]=dict(n=len(rr),min=x[0],median=x[len(x)//2],max=x[-1],
                          exclusive_min=e[0],exclusive_median=e[len(e)//2],exclusive_max=e[-1],
                          max_sampled_mask_cycles=max(r["max_sampled_mask_cycles"] for r in rr),
                          by_depth={str(d):{"min":min(r["inclusive_cycles"] for r in rr if r["depth"]==d),
                                               "max":max(r["inclusive_cycles"] for r in rr if r["depth"]==d)}
                                    for d in sorted({r["depth"] for r in rr})})
    return out

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler",required=True,type=Path)
    parser.add_argument("--out",required=True,type=Path)
    parser.add_argument("--capacity",type=int,nargs="+",default=[24,28,32])
    args=parser.parse_args()
    cc=args.compiler.resolve();nm=cc.with_name("avr-nm");size=cc.with_name("avr-size");objdump=cc.with_name("avr-objdump")
    out=args.out.resolve();out.mkdir(parents=True,exist_ok=True)
    report=dict(schema=1,qualification="isolated compiled AVR actual-source fixture, simavr cycles including marker overhead; not final linked firmware or concurrent timing/WCET",
                compiler=run([cc,"--version"]).splitlines()[0],flags=FLAGS,variants={},
                source_sha256={n:hashlib.sha256((FW/n).read_bytes()).hexdigest() for n in ["MotionPlanner.cpp","MoveQueue.cpp","MoveQueue.h","motion.h","motion.cpp","HAL.h","RMath.h"]})
    with TemporaryDirectory(prefix="planner-avr-") as temp:
        t=Path(temp);runner=t/"simavr-trace"
        # libsimavr and libelf are installed by the task-private hosted workflow.
        run(["cc","-O2",HERE/"simavr-trace.c","-lsimavr","-lelf","-o",runner])
        for cap in args.capacity:
            for name,instrument,only in [("outer",False,0),("substage",True,0)]+([("diag-only",False,7),("time-only",False,8),("lower-only",False,9),("paced-only",False,10)] if cap==24 else []):
                d=t/f"{cap}-{name}";d.mkdir();create_fixture(d,cap,instrument)
                elf=out/f"{cap}-{name}.elf"
                cmd=[cc,*FLAGS,f"-DESTIMATOR_ONLY={only}",f"-I{d}",d/"MotionPlanner.cpp",d/"MoveQueue.cpp",d/"geometry.cpp",d/"avr-fixture.cpp","-Wl,--gc-sections","-Wl,-Map="+str(out/f"{cap}-{name}.map"),"-o",elf]
                # Preserve actual stderr on failure for reproducible CI diagnostics.
                subprocess.run([str(x) for x in cmd],check=True)
                symbols=run([nm,"-S","-n",elf])
                addresses={}
                for line in symbols.splitlines():
                    m=re.match(r"^([0-9a-f]+)\s+[0-9a-f]+\s+\w\s+(phase|fixture_id|queue_depth)$",line)
                    if m:addresses[m[2]]=int(m[1],16)-0x800000
                assert len(addresses)==3,addresses
                sz=run([size,elf]).strip()
                q=re.search(r"^([0-9a-f]+)\s+([0-9a-f]+)\s+\w\s+_ZN9PrintLine5queueE$",symbols,re.M)
                info=dict(size=sz,queue_bytes=int(q[2],16) if q else None,elf_sha256=hashlib.sha256(elf.read_bytes()).hexdigest())
                (out/f"{cap}-{name}.dis").write_text(run([objdump,"-d","-z",elf]))
                if only==0:
                    raw=run([runner,elf,*[hex(addresses[k]) for k in ("phase","fixture_id","queue_depth")]])
                    (out/f"{cap}-{name}-markers.csv").write_text(raw)
                    rows=parse_trace(raw)
                    assert len([r for r in rows if r["stage"]==1])==cap*8
                    assert len([r for r in rows if r["stage"]==2])==cap*8
                    info["stages"]=stats(rows)
                    (out/f"{cap}-{name}-cycles.json").write_text(json.dumps(rows,indent=2)+"\n")
                report["variants"][f"{cap}-{name}"]=info
    (out/"avr-summary.json").write_text(json.dumps(report,indent=2)+"\n")
    for k,v in report["variants"].items():
        print(k,v["size"].replace("\n"," | "),"queue_bytes",v["queue_bytes"])
        for n,r in v.get("stages",{}).items():
            print(n,{f:r[f] for f in ("n","min","median","max","max_sampled_mask_cycles")})

if __name__=="__main__":
    main()

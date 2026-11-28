#!/usr/bin/env python3
"""Check the small integerSqrt operand correction on the pinned P0 source.

Use the existing simavr runner. Exercise forced inlining specifically because
the old input-only constraint is invalid when the caller keeps the input live.
This is isolated arithmetic, not motion/IRQ qualification.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import runpy
import subprocess
from tempfile import TemporaryDirectory

HERE=Path(__file__).resolve().parent

FIXTURE=r'''
#include <stdint.h>
class HAL {
public:
    static uint16_t integerSqrt(int32_t a);
};
'''
TEST=r'''
extern "C" {
volatile uint8_t phase=0, fixture_id=0, queue_depth=0;
}
static volatile uint32_t original;
static void mark(uint8_t p) {
    asm volatile("" ::: "memory");
    phase=p;
    asm volatile("" ::: "memory");
}
static void check(uint32_t input,uint16_t expected) {
    original=input;
    uint16_t root=HAL::integerSqrt(static_cast<int32_t>(input));
    // Keep input live across the inline asm. The independent volatile
    // read prevents the optimizer from simply proving equality in C++.
    if(root!=expected || input!=original) {
        fixture_id=uint8_t(expected>>8);
        queue_depth=uint8_t(expected);
        mark(254);mark(255);
        while(true){}
    }
}
int main() {
    // At integer n, rounding sqrt(n) to nearest changes from k to k+1
    // at n=k*k+k+1. Check both sides of every such boundary in the
    // nonnegative signed-32 input domain, plus exact squares.
    mark(1);
    for(uint32_t k=0;k<=46340UL;++k) {
        const uint32_t square=k*k;
        const uint32_t transition=square+k+1;
        check(square,uint16_t(k));
        if(transition<=2147483647UL) {
            check(transition-1,uint16_t(k));
            check(transition,uint16_t(k+1));
        }
    }
    check(2147483647UL,46341);
    mark(129);mark(255);
    while(true){}
}
'''


def run(args):
    p=subprocess.run([str(x) for x in args],text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    if p.returncode:
        print(p.stdout[-3000:]);print(p.stderr);p.check_returncode()
    return p.stdout


def extract(path):
    s=path.read_text()
    i=s.index("uint16_t HAL::integerSqrt(")
    j=s.index("} // integerSqrt",i)+len("} // integerSqrt")
    return s[i:j]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source",required=True,type=Path)
    p.add_argument("--compiler",required=True,type=Path)
    p.add_argument("--runner-source",required=True,type=Path)
    p.add_argument("--out",required=True,type=Path)
    a=p.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=True)
    cc=a.compiler.resolve()
    preflight=runpy.run_path(str(HERE/"small-cleanup-preflight.py"))
    head=run(["git","-C",a.source,"rev-parse","HEAD"]).strip()
    assert head==preflight["P0"],head
    original=a.source/"Repetier/HAL.cpp"
    before=extract(original)
    results=[]
    with TemporaryDirectory(prefix="task0032-sqrt-") as directory:
        d=Path(directory);(d/"Repetier").mkdir()
        copy=d/"Repetier/HAL.cpp";copy.write_bytes(original.read_bytes())
        preflight["sqrt_cleanup"](d)
        after=extract(copy)
        runner=d/"runner"
        run(["cc","-O2",a.runner_source,"-lsimavr","-lelf","-o",runner])
        for opt in ("-Os","-O2"):
            for name,source in (("baseline",before),("corrected",after)):
                label=name+"-"+opt[1:]
                # Keep the actual assembly; force inline only in the fixture.
                fixture=FIXTURE+source.replace("uint16_t HAL::integerSqrt(",
                    "__attribute__((always_inline)) inline uint16_t HAL::integerSqrt(",1)+"\n"+TEST
                (d/"fixture.cpp").write_text(fixture)
                elf=out/(label+".elf")
                run([cc,"-mmcu=atmega2560","-DF_CPU=16000000UL",opt,"-flto","-std=gnu++11",
                    d/"fixture.cpp","-Wl,--gc-sections","-o",elf])
                nm=run([cc.with_name("avr-nm"),"-S","-n",elf])
                symbols={}
                for line in nm.splitlines():
                    m=re.match(r"^([0-9a-f]+)\s+[0-9a-f]+\s+\w\s+(phase|fixture_id|queue_depth)$",line)
                    if m:symbols[m[2]]=int(m[1],16)-0x800000
                assert len(symbols)==3,symbols
                trace=run([runner,elf,*[hex(symbols[k]) for k in ("phase","fixture_id","queue_depth")]])
                (out/(label+".csv")).write_text(trace)
                marks=[list(map(int,l.split(",")[1:])) for l in trace.splitlines() if l.startswith("E,")]
                assert marks and marks[-1][-1]==255,marks
                fail=[m for m in marks if m[-1]==254]
                complete=[m for m in marks if m[-1]==129]
                assert bool(fail)!=bool(complete),marks
                info={"variant":name,"optimization":opt,"completed":bool(complete),
                    "checks":139024 if complete else None,"failure_expected_root":(fail[0][1]*256+fail[0][2]) if fail else None,
                    "elapsed_cycles":marks[-1][0]-marks[0][0],
                    "elf_sha256":hashlib.sha256(elf.read_bytes()).hexdigest()}
                results.append(info);print(json.dumps(info),flush=True)
                (out/(label+".dis")).write_text(run([cc.with_name("avr-objdump"),"-d","-z",elf]))
                if name=="corrected":
                    assert complete and not fail,info
    (out/"sqrt-results.json").write_text(json.dumps({"schema":1,"p0":head,
        "source_sha256":hashlib.sha256(original.read_bytes()).hexdigest(),
        "qualification":"isolated forced-inline AVR arithmetic; nonnegative int32 input only; no motion or IRQ timing proof",
        "results":results},indent=2)+"\n")

if __name__=="__main__":
    main()

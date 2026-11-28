#!/usr/bin/env python3
"""Compare an isolated decimal parser with actual pinned parseLongValue.

Host replay (native long clamped to AVR range) and exact AVR-libc oracle.
Cycle measurements isolate the numeric token, not full parser throughput.
Temporary firmware tree for existing serial regression; no production edits.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

HERE=Path(__file__).resolve().parent
P0="9072d65121bc2086522ea6b837ca3804ec80faa5"

FIXTURE=r'''
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#else
#include <stdio.h>
#include <assert.h>
#define PROGMEM
#define memcpy_P memcpy
#endif
#include "decimal32-candidate.h"
#include "baseline.inc"

static __attribute__((noinline)) int32_t reference(char* text) {
    const long v=baselineLong(text);
    // AVR long is 32 bits. Host long may be wider.
#if __SIZEOF_LONG__ > 4
    if(v>INT32_MAX)return INT32_MAX;
    if(v<INT32_MIN)return INT32_MIN;
#endif
    return int32_t(v);
}
static __attribute__((noinline)) int32_t candidate(char* text) {
    return parseDecimal32(text);
}
extern "C" {
volatile uint8_t phase=0, fixture_id=0, queue_depth=0;
}
static volatile int32_t result;
static void mark(uint8_t p) {
    asm volatile("" ::: "memory");
    phase=p;
    asm volatile("" ::: "memory");
}
static void require(bool condition) {
#ifdef __AVR__
    if(!condition){mark(254);mark(255);while(true){}}
#else
    assert(condition);
#endif
}
static uint32_t checks=0;
static void check(char* text) {
    const int32_t a=reference(text),b=candidate(text);
    require(a==b);
    ++checks;
}
static uint32_t state=0x9e3779b9UL;
static uint32_t randomWord() {
    state^=state<<13;state^=state>>17;state^=state<<5;return state;
}

static const char samples[][96] PROGMEM={
    "", " ", "0", "1", "9", "10", "99", "100", "255", "256",
    "104", "206", "65535", "65536", "99999", "100000", "123456789",
    "214748364", "2147483640", "2147483647", "2147483648", "4294967295",
    "-1", "-10", "-65536", "-2147483647", "-2147483648", "-2147483649",
    "+0", "+2147483647", "+2147483648", "-0", "00000000000000000000000001",
    "0000000000000000000002147483648", "999999999999999999999999999999",
    "-999999999999999999999999999999", "+", "-", "++1", "+-1", "--1",
    "  \t\r\n\v\f-42", "\t+123", "- 123", "+\t123", "12X34", "0x123",
    "1e9", "12.5", ".5", "nan", "  -", "  X9", "2147483647P5",
    "-2147483648S9", "0002147483647", "-0002147483648",
    "2147483646", "-2147483646", "2147483650", "-2147483650"
};

int main() {
#ifdef __AVR__
    static_assert(sizeof(long)==4,"actual AVR long required");
    cli();
#endif
    char text[96];
    for(uint8_t i=0;i<sizeof(samples)/sizeof(samples[0]);++i) {
        memcpy_P(text,samples[i],sizeof(text));
        fixture_id=0;queue_depth=i;
        mark(1);result=reference(text);mark(129);
        const int32_t expected=result;
        mark(2);result=candidate(text);mark(130);
        require(expected==result);++checks;
    }
    // Every byte value, including NUL, high-bit data, all C whitespace,
    // before/after sign and digits. Compare return value only, as caller does.
    for(uint16_t byte=0;byte<256;++byte) {
        const char c=char(byte);
        char a[]={c,'1','2','3',0};check(a);
        char b[]={'-',c,'1','2',0};check(b);
        char d[]={'1','2',c,'3','4',0};check(d);
        char e[]={' ',c,'+','7',0};check(e);
        char f[]={c,0};check(f);
    }
    for(uint16_t i=0;i<4096;++i) {
        // Every magnitude 0..4095, both signs, leading zero, suffix.
        uint16_t v=i;char* end=text+12;*end=0;*--end='X';
        do{*--end=char('0'+v%10);v/=10;}while(v);
        *--end='0';*--end='+';check(end);*end='-';check(end);
    }
#ifdef __AVR__
    const uint32_t fuzz=8192;
#else
    const uint32_t fuzz=200000;
#endif
    for(uint32_t n=0;n<fuzz;++n) {
        const uint8_t length=uint8_t(randomWord()%96);
        // Mix full arbitrary bytes and mostly numeric strings. All bounded
        // NUL terminated; no invalid memory input to either implementation.
        for(uint8_t i=0;i<length;++i) {
            uint32_t v=randomWord();
            if(n&1)text[i]=char(v);
            else if(i==0 && (v&3)==0)text[i]=(v&4)?'+':'-';
            else text[i]=char('0'+v%10);
        }
        text[length]=0;check(text);
    }
    require(checks==sizeof(samples)/sizeof(samples[0])+1280UL+8192UL+fuzz);
#ifdef __AVR__
    mark(255);while(true){}
#else
    printf("decimal32 host: %lu value comparisons PASS\n",(unsigned long)checks);
#endif
}
'''

def run(args,**kw):
    return subprocess.run([str(x) for x in args],check=True,text=True,**kw)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source",type=Path,required=True)
    p.add_argument("--out",type=Path,required=True)
    p.add_argument("--compiler",type=Path,required=True)
    p.add_argument("--runner-source",type=Path,required=True)
    a=p.parse_args()
    source=a.source.resolve();out=a.out.resolve();out.mkdir(parents=True,exist_ok=True)
    assert run(["git","-C",source,"rev-parse","HEAD"],stdout=subprocess.PIPE).stdout.strip()==P0
    assert not run(["git","-C",source,"status","--porcelain"],stdout=subprocess.PIPE).stdout.strip()
    receiver=(source/"Repetier/GCodeInput.cpp").read_text()
    i=receiver.index("inline long parseLongValue(")
    j=receiver.index("\n}\n",i)+3
    original=receiver[i:j]
    assert "strtol(s, &endPtr, 10)" in original
    header=(HERE/"decimal32-candidate.h").read_text()
    # No application errno consumer may silently lose overflow side effects.
    errno_users=[]
    for f in (source/"Repetier").rglob("*"):
        if f.is_file() and f.suffix in (".h",".cpp",".c",".ino",".hpp"):
            if re.search(r"\berrno\b",f.read_text(encoding="utf-8")):
                errno_users.append(str(f.relative_to(source)))
    assert not errno_users,errno_users
    with tempfile.TemporaryDirectory(prefix="task0032-decimal32-") as directory:
        d=Path(directory)
        (d/"baseline.inc").write_text(original.replace("parseLongValue","baselineLong"))
        (d/"decimal32-candidate.h").write_text(header)
        (d/"fixture.cpp").write_text(FIXTURE)
        host=d/"host"
        run([os.environ.get("CXX","c++"),"-std=c++11","-O2","-Wall","-Wextra","-Werror",
             "-fsanitize=address,undefined","-fno-sanitize-recover=all","-fno-pie","-no-pie",d/"fixture.cpp","-o",host])
        host_log=run([host],stdout=subprocess.PIPE).stdout
        print(host_log,end="")
        cc=a.compiler.resolve()
        flags=["-mmcu=atmega2560","-DF_CPU=16000000UL","-Os","-flto","-std=gnu++11",
               "-ffunction-sections","-fdata-sections","-fno-exceptions","-Wall","-Wextra","-Werror"]
        elf=out/"decimal32.elf"
        run([cc,*flags,d/"fixture.cpp","-Wl,--gc-sections","-Wl,-Map="+str(out/"decimal32.map"),"-o",elf])
        nm=run([cc.with_name("avr-nm"),"-S","-n",elf],stdout=subprocess.PIPE).stdout
        addresses={}
        for line in nm.splitlines():
            m=re.fullmatch(r"([0-9a-f]+) [0-9a-f]+ \w (phase|fixture_id|queue_depth)",line)
            if m:addresses[m[2]]=int(m[1],16)-0x800000
        assert len(addresses)==3
        runner=d/"simavr-trace"
        run(["cc","-O2",a.runner_source,"-lsimavr","-lelf","-o",runner])
        trace=run([runner,elf,*[hex(addresses[k]) for k in ("phase","fixture_id","queue_depth")]],
                  stdout=subprocess.PIPE).stdout
        (out/"markers.csv").write_text(trace)
        records={};pending=None;done=False
        for line in trace.splitlines():
            if not line.startswith("E,"):continue
            _,cycle,f,index,mark=line.split(",")
            cycle,f,index,mark=map(int,(cycle,f,index,mark))
            if mark==254:raise AssertionError(("AVR libc value mismatch",f,index))
            if mark==255:done=True;break
            if mark<128:
                assert pending is None and f==0 and mark in (1,2)
                pending=(cycle,index,mark)
            else:
                start,idx,stage=pending
                assert (idx,stage)==(index,mark-128)
                assert (idx,stage) not in records
                records[idx,stage]=cycle-start;pending=None
        assert done and pending is None
        rows=[{"id":i,"baseline":records[i,1],"candidate":records[i,2],
               "saved":records[i,1]-records[i,2]} for i in range(len(records)//2)]
        assert len(rows)==61,len(rows)
        print("AVR libc:",61+1280+8192+8192,"value comparisons PASS")
        print("paired samples saved cycles min/median/max:",
              min(r["saved"] for r in rows),sorted(r["saved"] for r in rows)[len(rows)//2],
              max(r["saved"] for r in rows))
        (out/"decimal32.dis").write_text(run([cc.with_name("avr-objdump"),"-d","-z",elf],stdout=subprocess.PIPE).stdout)
        # Reuse actual pinned receiver and canonical serial tests. Do not
        # modify the live checkout or narrow the M206 spelling validator.
        root=d/"candidate"
        for folder in ("Repetier","tests"):shutil.copytree(source/folder,root/folder)
        new=header+"\ninline long parseLongValue(char* s) { return parseDecimal32(s); }\n"
        (root/"Repetier/GCodeInput.cpp").write_text(receiver[:i]+new+receiver[j:])
        run(["python3",root/"tests/serial-protocol.py"])
        result={"p0":P0,"receiver_sha256":hashlib.sha256(receiver.encode()).hexdigest(),
                "candidate_sha256":hashlib.sha256(header.encode()).hexdigest(),
                "avr_elf_sha256":hashlib.sha256(elf.read_bytes()).hexdigest(),
                "host":host_log.strip(),"avr_value_checks":61+1280+8192+8192,
                "errno_users":errno_users,"rows":rows,
                "qualification":"actual AVR-libc return value comparison; isolated timing; canonical serial host regression; no integrated linked resource/IRQ proof"}
        (out/"decimal32-results.json").write_text(json.dumps(result,indent=2)+"\n")

if __name__=="__main__":
    main()

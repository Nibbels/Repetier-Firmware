#!/usr/bin/env python3
"""Actual-source isolated AVR RX/parse/admission/ACK measurement.

Real GCodeInput, GCode, UsbSerial ring functions and Com decimal output,
pinned AVR compiler and existing 0032-P simavr marker runner. No actual UART
transmission, concurrent IRQs, P0 settings/safety or command execution.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import struct
import subprocess
from tempfile import TemporaryDirectory

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / "Repetier"
FLAGS = ["-mmcu=atmega2560", "-DF_CPU=16000000UL", "-Os", "-flto",
         "-std=gnu++11", "-ffunction-sections", "-fdata-sections", "-fno-exceptions"]

HAL = r'''
#pragma once
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include "UsbSerial.h"
using millis_t = uint32_t;
using fast8_t = int8_t;
#define FSTRINGPARAM(var) PGM_P var
#define FSTRINGVAR(var) static const char var[] PROGMEM;
#define FSTRINGVALUE(var,value) const char var[] PROGMEM = value;
class HAL {
public:
    static millis_t timeInMilliseconds() { return 0; }
    static bool serialByteAvailable() { return UsbSerial::available(); }
    static uint8_t serialReadByte() { return UsbSerial::read(); }
    static void serialWriteByte(char b) { UsbSerial::write(b); }
    static void serialFlush() { UsbSerial::flush(); }
    static char readFlashByte(PGM_P p) { return pgm_read_byte(p); }
};
'''
REPETIER = r'''
#pragma once
#include <stdlib.h>
#include <math.h>
#include "GCodeInput.h"
#include "HAL.h"
#include "Communication.h"
#include "GCodeCommands.h"
#define GCODE_BUFFER_SIZE 2
extern volatile uint8_t runtime_flags;
namespace Printer {
    inline bool debugErrors() { return runtime_flags & 1; }
    inline bool debugEcho() { return runtime_flags & 2; }
    struct Job { bool commandsBlocked() const { return runtime_flags & 4; } };
    static Job printJob;
}
namespace Commands {
    void emergencyStop();
    void executeGCode(GCode*);
    void checkForPeriodicalActions(FirmwareState);
}
namespace RMath { inline uint8_t min(uint8_t a,uint8_t b) {return a<b?a:b;} }
'''
DECIMAL_CANDIDATE = """void Com::printNumber(uint32_t n) {
    char buf[11];
    char* str = &buf[10];
    *str = '\\0';
    // Consume only the necessary low digits using 32-bit division.
    while (n > UINT16_MAX) {
        const uint32_t q = n / 10;
        *--str = '0' + (n - 10 * q);
        n = q;
    }
    // Binary line numbers and most ordinary values fit in 16 bits.
    uint16_t value = static_cast<uint16_t>(n);
    do {
        const uint16_t q = value / 10;
        *--str = '0' + (value - 10 * q);
        value = q;
    } while (value);
    print(str);
} // printNumber
"""

FIXTURE = r'''
#include <string.h>
#define private public
#include "UsbSerial.h"
#undef private
#include "Repetier.h"
#include "UsbSerial.cpp"
#include "GCodeInput.cpp"
#include "gcode.cpp"
#include "com-fixture.inc"
#include "samples.inc"

extern "C" {
volatile uint8_t phase=0, fixture_id=0, queue_depth=0;
}
volatile uint8_t runtime_flags=0;
static void mark(uint8_t p) {
    asm volatile("" ::: "memory");
    phase=p;
    asm volatile("" ::: "memory");
}
static void require(bool ok) {
    if(!ok){ mark(254); mark(255); while(true){} }
}
namespace GCodeCommands {
bool rejectLegacySimulation(GCode*) { return runtime_flags & 8; }
}
namespace Commands {
void emergencyStop() { require(false); }
void executeGCode(GCode*) { require(false); }
void checkForPeriodicalActions(FirmwareState) { require(false); }
}
static void reset(const Sample& s) {
    require(commandQueue.empty());
    frame=ReceiveFrame{};
    serialReceive=ReceiveSession{};
    serialReceive.lastLineNumber=s.n-1;
    UsbSerial::rx_.head=UsbSerial::rx_.tail=0;
    UsbSerial::tx_.head=UsbSerial::tx_.tail=0;
}
static void feed(const Sample& s,uint8_t a,uint8_t b) {
    require(!UsbSerial::available() && b>=a);
    for(uint8_t i=a;i<b;++i) {
        auto& rx=UsbSerial::rx_;
        rx.bytes[rx.head]=s.bytes[i];
        rx.head=rx.next(rx.head);
    }
}
static void verifyAck(const Sample& s) {
    auto& tx=UsbSerial::tx_;
    uint8_t i=0;
    for(;tx.tail!=tx.head;++i) {
        require(i<s.ackLength && tx.bytes[tx.tail]==s.ack[i]);
        tx.tail=tx.next(tx.tail);
    }
    require(i==s.ackLength);
}
static void verify(const Sample& s) {
    auto* c=GCodeInput::peekCurrentCommand();
    require(c && c->hasN() && c->N==s.n && c->hasG() && c->G==1);
    require(c->hasX() && c->hasY() && c->hasE() && c->hasF()==s.hasF);
    require(fabsf(c->X-s.x)<0.0001f && fabsf(c->Y-s.y)<0.0001f &&
            fabsf(c->E-s.e)<0.0001f);
    if(s.hasF)require(c->F==s.f);
    require(!UsbSerial::available());
    verifyAck(s);
    GCodeInput::popCurrentCommand();
    require(!GCodeInput::peekCurrentCommand());
}
int main() {
    // No peripheral interrupts or actual wire transmission in isolated fixture.
    cli();
    for(uint8_t i=0;i<sizeof(samples)/sizeof(samples[0]);++i) {
        Sample s;
        memcpy_P(&s,&samples[i],sizeof(s));
        fixture_id=s.format; queue_depth=s.id;
        reset(s); feed(s,0,s.length);
        mark(1); GCodeInput::readFromSerial(); mark(129);
        verify(s);
        mark(2); Com::printFLN(Com::tOkSpace,uint32_t(s.n)); mark(130);
        verifyAck(s);
        reset(s); feed(s,0,s.length-1);
        mark(3); GCodeInput::readFromSerial(); mark(131);
        require(!GCodeInput::peekCurrentCommand() && UsbSerial::tx_.head==0);
        feed(s,s.length-1,s.length);
        mark(4); GCodeInput::readFromSerial(); mark(132);
        verify(s);
    }
    // Generic 32-bit formatter correctness, including transitions across
    // the 16-bit boundary and decimal digit boundaries.
    static const uint32_t numbers[] = {
        0UL, 9UL, 10UL, 99UL, 100UL, 9999UL, 10000UL,
        65534UL, 65535UL, 65536UL, 99999UL, 100000UL,
        999999999UL, 1000000000UL, 2147483648UL, 4294967295UL
    };
    fixture_id=3;
    for(uint8_t i=0;i<sizeof(numbers)/sizeof(numbers[0]);++i) {
        queue_depth=i;
        char expected[11]; ultoa(numbers[i],expected,10);
        mark(5); Com::printNumber(numbers[i]); mark(133);
        auto& tx=UsbSerial::tx_;
        uint8_t j=0;
        while(tx.tail!=tx.head) {
            require(expected[j] && tx.bytes[tx.tail]==expected[j++]);
            tx.tail=tx.next(tx.tail);
        }
        require(!expected[j]);
    }
    mark(255); while(true){}
}
'''


def sample_data():
    values = [
        (1, "1.25", "2.5", "0.125", "3600"),
        (9, "100.1000", "95.4560", "0.00400", None),
        (10, "100.2000", "95.4560", "0.00800", None),
        (99, "0.025", "0.05", "0.00050", "1200"),
        (100, "123.4567", "99.8765", "123.45678", None),
        (999, "-12.125", "0.0", "-0.125", None),
        (1000, "0", "0", "0", "6000"),
        (12345, "150.0000", "95.4560", "2.00000", None),
        (65535, "199.9999", "0.0001", "99.99999", "6000"),
    ]
    rows = []
    for fmt in range(3):
        for i, (n, x, y, e, f) in enumerate(values):
            if fmt == 0:
                body = f"N{n} G1 X{x} Y{y} E{e}" + (f" F{f}" if f else "")
                checksum = 0
                for c in body.encode("ascii"):
                    checksum ^= c
                raw = f"{body}*{checksum}\n".encode("ascii")
            else:
                flags = 0x00dd | (256 if f else 0) | (4096 if fmt == 2 else 0)
                raw = struct.pack("<H", flags) + (b"\0\0" if fmt == 2 else b"")
                raw += struct.pack("<H", n) + (b"\1\0" if fmt == 2 else b"\1")
                raw += struct.pack("<fff", float(x), float(y), float(e))
                if f:
                    raw += struct.pack("<f", float(f))
                a = b = 0
                for c in raw:
                    a = (a+c)%255
                    b = (b+a)%255
                raw += bytes([a,b])
                assert len(raw) == (22 if fmt == 2 else 19) + (4 if f else 0)
            ack = f"ok {n}\r\n".encode("ascii")
            rows.append(dict(format=fmt,id=i,n=n,x=float(x),y=float(y),e=float(e),
                             f=float(f or 0),hasF=bool(f),raw=list(raw),ack=list(ack)))
    return rows


def sample_header(rows):
    out = ["struct Sample {uint8_t format,id,length,ackLength; uint16_t n; float x,y,e,f; bool hasF; uint8_t bytes[96],ack[15];};",
           "static const Sample samples[] PROGMEM={"]
    for s in rows:
        fl = ",".join(repr(s[k])+"f" for k in ("x","y","e","f"))
        raw = ",".join(map(str,s["raw"]))
        ack = ",".join(map(str,s["ack"]))
        out.append("{"+f'{s["format"]},{s["id"]},{len(s["raw"])},{len(s["ack"])},{s["n"]},{fl},{str(s["hasF"]).lower()},'+"{"+raw+"},{"+ack+"}},")
    return "\n".join(out+["};\n"])


def run(args):
    p=subprocess.run([str(x) for x in args],text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    if p.returncode:
        print(p.stdout[-4000:]);print(p.stderr);p.check_returncode()
    return p.stdout


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--compiler",required=True,type=Path)
    p.add_argument("--runner-source",required=True,type=Path)
    p.add_argument("--out",required=True,type=Path)
    p.add_argument("--decimal",choices=["baseline","u16-tail"],default="baseline")
    args=p.parse_args()
    out=args.out.resolve();out.mkdir(parents=True,exist_ok=True)
    cc=args.compiler.resolve(); rows=sample_data()
    files=["GCodeInput.cpp","GCodeInput.h","gcode.cpp","gcode.h","UsbSerial.cpp","UsbSerial.h","Communication.h","GCodeCommands.h"]
    with TemporaryDirectory(prefix="task0032-parser-avr-") as directory:
        d=Path(directory)
        for name in files:shutil.copyfile(FW/name,d/name)
        assert re.search(r"^#define GCODE_BUFFER_SIZE\s+2\b",(FW/"Configuration.h").read_text(),re.M)
        (d/"HAL.h").write_text(HAL);(d/"Repetier.h").write_text(REPETIER)
        com=(FW/"Communication.cpp").read_text()
        functions=com[com.index("void Com::printWarningF("):]
        if args.decimal=="u16-tail":
            begin=functions.index("void Com::printNumber(")
            end=functions.index("} // printNumber",begin)+len("} // printNumber")
            functions=functions[:begin]+DECIMAL_CANDIDATE+functions[end:]
        names=set(re.findall(r"\bCom::(t\w+)\b", "\n".join((d/n).read_text() for n in ("GCodeInput.cpp","gcode.cpp"))+functions))
        names.update(["tWarning","tInfo","tError","tNAN","tINF"])
        constants=[line for line in com.splitlines() if any(line.startswith("FSTRINGVALUE(Com::"+n+",") for n in names)]
        assert len(constants)==len(names),(names,constants)
        (d/"com-fixture.inc").write_text("\n".join(constants)+"\n"+functions)
        (d/"samples.inc").write_text(sample_header(rows))
        (d/"fixture.cpp").write_text(FIXTURE)
        elf=out/"parser.elf"
        subprocess.run([str(x) for x in [cc,*FLAGS,f"-I{d}",d/"fixture.cpp","-Wl,--gc-sections","-Wl,-Map="+str(out/"parser.map"),"-o",elf]],check=True)
        nm=run([cc.with_name("avr-nm"),"-S","-n",elf])
        symbols={}
        for line in nm.splitlines():
            m=re.match(r"^([0-9a-f]+)\s+[0-9a-f]+\s+\w\s+(phase|fixture_id|queue_depth)$",line)
            if m:symbols[m[2]]=int(m[1],16)-0x800000
        assert len(symbols)==3,symbols
        runner=d/"simavr-trace"
        run(["cc","-O2",args.runner_source,"-lsimavr","-lelf","-o",runner])
        trace=run([runner,elf,*[hex(symbols[k]) for k in ("phase","fixture_id","queue_depth")]])
        (out/"markers.csv").write_text(trace)
        records=[];pending=None;done=False
        for line in trace.splitlines():
            if not line.startswith("E,"):continue
            _,t,f,i,mark=line.split(",");t,f,i,mark=map(int,(t,f,i,mark))
            if mark==254:raise ValueError(f"decoded field/ACK fixture failed: {f}/{i}")
            if mark==255:done=True;break
            if mark<128:
                assert pending is None
                pending=(t,f,i,mark)
            else:
                start,ff,ii,stage=pending
                assert (ff,ii,stage)==(f,i,mark-128)
                records.append(dict(format=f,id=i,stage=stage,cycles=t-start))
                pending=None
        assert done and pending is None and len(records)==len(rows)*4+16
        (out/"parser.dis").write_text(run([cc.with_name("avr-objdump"),"-d","-z",elf]))
        results=dict(schema=1,qualification="isolated AVR simavr cycle samples incl markers; actual software rings, parser, decimal ACK enqueue; no concurrent interrupts or TX wait, stub clock/debug/legacy gate; not integrated WCET",
                     decimal=args.decimal,compiler=run([cc,"--version"]).splitlines()[0],flags=FLAGS,
                     source_sha256={n:hashlib.sha256((FW/n).read_bytes()).hexdigest() for n in files+["Communication.cpp","Configuration.h"]},
                     elf_sha256=hashlib.sha256(elf.read_bytes()).hexdigest(),samples=rows,records=records)
        (out/"parser-results.json").write_text(json.dumps(results,indent=2)+"\n")
        for fmt in range(4):
            for stage in (range(1,5) if fmt<3 else [5]):
                x=sorted(r["cycles"] for r in records if r["format"]==fmt and r["stage"]==stage)
                print("format",fmt,"stage",stage,"n",len(x),"min/median/max",x[0],x[len(x)//2],x[-1])


if __name__=="__main__":
    main()

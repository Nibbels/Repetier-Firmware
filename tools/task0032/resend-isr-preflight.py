#!/usr/bin/env python3
"""Isolated linked AVR ISR structural path costs, pinned P0 USART source.

Compare original, sticky-only, separate request fence and packed gate with
cached TX cursor. Reuse Task0033's fail-closed instruction path walker.
Not an integrated timing proof; all variants compiled with matching flags.
"""
import argparse
import hashlib
import json
from pathlib import Path
import runpy
import subprocess

ROOT=Path(__file__).resolve().parents[2]
ring=runpy.run_path(str(ROOT/"tools/task0032/resend-ring-preflight.py"))
prior=runpy.run_path(str(ROOT/"tools/task0033/isr-candidates.py"))
once=ring["once"]
run=prior["run"]
MAIN=r'''
#include "UsbSerial.h"
int main() {
    UsbSerial::begin(250000);
    for(;;) {
        const int c=UsbSerial::read();
        if(c==127) {
            const char msg[]="\r\nResend:42\r\nok\r\n";
            SEND_RESEND;
        } else if(c>=0) {
            UsbSerial::tryWrite(uint8_t(c));
        }
    }
}
'''

def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",type=Path,required=True)
    parser.add_argument("--compiler",type=Path,required=True)
    parser.add_argument("--out",type=Path,required=True)
    args=parser.parse_args()
    assert run(["git","-C",args.source,"rev-parse","HEAD"]).strip()==ring["P0"]
    out=args.out.resolve();out.mkdir(parents=True,exist_ok=True)
    cc=args.compiler.resolve()
    hdr=(args.source/"Repetier/UsbSerial.h").read_text()
    src=(args.source/"Repetier/UsbSerial.cpp").read_text()
    sh=once(hdr,"    static bool available();","    static bool available();\n    static uint8_t lossFlags(){return lossFlags_;}")
    sh=once(sh,"    static Ring<receiveBufferSize> rx_;","    static volatile uint8_t lossFlags_;\n    static Ring<receiveBufferSize> rx_;")
    gate=runpy.run_path(str(ROOT/"tools/task0032/loss-gate-replay.py"))
    ss=once(src,gate["BASE_RX"],gate["GATED_RX"])
    ss=once(ss,"void UsbSerial::begin(unsigned long baud) {","volatile uint8_t UsbSerial::lossFlags_=0;\nvoid UsbSerial::begin(unsigned long baud) {")
    fh,fs=ring["candidate_uart"](hdr,src)
    ph,ps=ring["packed_candidate"](fh,fs)
    report={"schema":1,"p0":ring["P0"],"compiler":run([cc,"--version"]).splitlines()[0],
            "qualification":"isolated linked ISR paths, excludes priority/interrupt masking/global service; independent branches overapproximate",
            "results":{}}
    for opt in ["Os","O2","O3"]:
        flags=["-mmcu=atmega2560","-DF_CPU=16000000UL","-"+opt,"-flto","-std=gnu++11",
               "-ffunction-sections","-fdata-sections","-fno-exceptions","-fno-threadsafe-statics"]
        for name,h,s,fenced in [("original",hdr,src,False),("sticky",sh,ss,False),
                                 ("separate",fh,fs,True),("packed_cached",ph,ps,True)]:
            work=out/(opt+"-"+name);work.mkdir(exist_ok=True)
            # Same inlining policy across alternatives; fail on unknown
            # linked ISR instructions rather than inventing their cost.
            for fn in ["receiveInterrupt","transmitInterrupt"]:
                s=once(s,"inline void UsbSerial::"+fn+"()",
                       "inline __attribute__((always_inline)) void UsbSerial::"+fn+"()")
            (work/"UsbSerial.h").write_text(h)
            (work/"UsbSerial.cpp").write_text(s)
            send=("UsbSerial::tryWriteResendSpan(msg,sizeof(msg)-1,sizeof(msg)-5)"
                  if fenced else "UsbSerial::tryWriteSpan(msg,sizeof(msg)-1)")
            (work/"main.cpp").write_text(MAIN.replace("SEND_RESEND",send))
            elf=work/"candidate.elf"
            run([cc,*flags,work/"UsbSerial.cpp",work/"main.cpp","-Wl,--gc-sections","-o",elf])
            dis=run([cc.with_name("avr-objdump"),"-d","-z",elf])
            (work/"disassembly.txt").write_text(dis)
            rx=prior["path_set"](dis,25);tx=prior["path_set"](dis,26)
            if name!="original":
                assert all(len(p["status_reads"])==1 and p["status_reads"][0]<p["read_complete"][0] for p in rx)
            (work/"paths.json").write_text(json.dumps({"rx":rx,"tx":tx},indent=2))
            row={"flags":flags,"rx":prior["summarize"](rx),"tx":prior["summarize"](tx),
                 "rx_max":max(p["cycles"] for p in rx),"tx_max":max(p["cycles"] for p in tx),
                 "udr_read":sorted({p["read_complete"][0] for p in rx}),
                 "stack_rx":max(p["stack_bytes_including_return"] for p in rx),
                 "stack_tx":max(p["stack_bytes_including_return"] for p in tx),
                 "size":run([cc.with_name("avr-size"),elf]).strip(),
                 "elf_sha256":digest(elf),"source_sha256":digest(work/"UsbSerial.cpp"),
                 "header_sha256":digest(work/"UsbSerial.h")}
            report["results"][opt+"-"+name]=row
            print("TXRX",opt,name,json.dumps({k:v for k,v in row.items() if k in
                ("rx_max","tx_max","udr_read","stack_rx","stack_tx","size")},separators=(",",":")))
    (out/"summary.json").write_text(json.dumps(report,indent=2)+"\n")

if __name__=="__main__":
    main()

#!/usr/bin/env python3
"""Actual-source bounded legacy TX write/flush escape candidate.

No nested scheduler pump, no interrupt polling of UDR, no new queue.
A failed ordinary output is terminal evidence, not a silently recoverable
partial message. Existing nonblocking terminal spans remain available.
Trial 65535 poll iterations: >=40320 cycles needed for 63 x 640-cycle
wire frames in an ideal 250k 8N1 full drain. Final active/wall cycle bound,
masked callsites and current P0 interrupt response still need qualification.
"""
import argparse
from pathlib import Path
import re
import runpy
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[2]
ring=runpy.run_path(str(Path(__file__).with_name("resend-ring-preflight.py")))
once=ring["once"]
P0=ring["P0"]

def candidate(header,source):
    header,source=ring["candidate_uart"](header,source)
    header,source=ring["packed_candidate"](header,source)
    header=once(header,"    static void flush();",
        "    static void flush();\n    static bool blockingTxFailed(){return blockingTxFailed_;}")
    header=once(header,"    static uint16_t txPublished_;",
        "    static uint16_t txPublished_;\n    static bool blockingTxFailed_;")
    source=once(source,"uint16_t UsbSerial::txPublished_ = 0;",
        "uint16_t UsbSerial::txPublished_ = 0;\nbool UsbSerial::blockingTxFailed_ = false;")
    check="""        if(!(SREG & 0x80) || !--budget) {
            blockingTxFailed_=true;
            return;
        }"""
    source=once(source,
        """void UsbSerial::write(uint8_t byte) {
    const uint8_t next = tx_.next(tx_.head);
    while (next == tx_.tail) {
        // The transmit interrupt makes room in the ring.
    }""",
        """void UsbSerial::write(uint8_t byte) {
    if(blockingTxFailed_)return;
    const uint8_t next = tx_.next(tx_.head);
    if(next==tx_.tail) {
        uint16_t budget=UINT16_MAX;
        while(next==tx_.tail) {
"""+check+"\n        }\n    }")
    source=once(source,
        """void UsbSerial::flush() {
    while (tx_.head != tx_.tail) {
        // Wait for the interrupt to hand the queued bytes to the UART.
    }
}""",
        """void UsbSerial::flush() {
    if(blockingTxFailed_)return;
    uint16_t budget=UINT16_MAX;
    while(tx_.head!=tx_.tail) {
"""+check+"\n    }\n}")
    return header,source

TEST=r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include "resend-policy.h"
#define F_CPU 16000000UL
#define _BV(bit) (1U << (bit))
#define ISR(vector) void vector()
enum { U2X0=1, TXEN0=3, RXEN0=4, UDRIE0=5, RXCIE0=7,FE0=4,DOR0=3,UPE0=2 };
volatile uint8_t UCSR0A,UCSR0B,UBRR0H,UBRR0L,UDR0,SREG=0x80;
void cli(){SREG&=0x7f;}
unsigned visits,progressEvery;
std::string retired;
void researchWaitStep();
#include "usb-serial-firmware.inc"
void researchWaitStep(){
    ++visits;
    if(progressEvery && visits%progressEvery==0 && UsbSerial::transmitFree()!=63){
        assert(SREG&0x80);
        USART0_UDRE_vect();retired+=char(UDR0);
    }
}
std::string drain(){
    std::string result;
    while(UsbSerial::transmitFree()!=63){
        USART0_UDRE_vect();result+=char(UDR0);
    }
    USART0_UDRE_vect();
    return result;
}
int main(int argc,char** argv){
    assert(argc==2);const std::string mode=argv[1];
    UsbSerial::begin(250000);
    const bool flush=mode.find("flush")!=std::string::npos;
    const bool masked=mode.find("masked")!=std::string::npos;
    const bool good=mode.find("good")!=std::string::npos;
    const bool partial=mode.find("partial")!=std::string::npos;
    // Masked but immediately available is not a failure.
    SREG=0;UsbSerial::write(0x55);assert(!UsbSerial::blockingTxFailed()&&!visits);
    assert(drain()==std::string(1,0x55));UsbSerial::flush();
    SREG=masked?0x15:0x95;
    for(unsigned i=0;i<63;++i)assert(UsbSerial::tryWrite(uint8_t(i)));
    const uint16_t before=UsbSerial::transmitRetired();
    if(good)progressEvery=flush?1000:65534;
    else if(partial)progressEvery=40000;
    if(flush)UsbSerial::flush();else UsbSerial::write(0x77);
    assert(SREG==(masked?0x15:0x95));
    if(good){
        assert(!UsbSerial::blockingTxFailed());
        assert(visits==(flush?63000u:65534u));
        std::string expected;for(unsigned i=0;i<63;++i)expected+=char(i);
        if(!flush)expected+=char(0x77);
        assert(retired+drain()==expected);
    }else{
        assert(UsbSerial::blockingTxFailed());
        assert(visits==(masked?0u:65534u));
        assert(UsbSerial::transmitRetired()==uint16_t(before+(partial?1:0)));
        const auto count=visits;
        UsbSerial::write(0x78);UsbSerial::flush();assert(visits==count);
        ResendPolicy policy;policy.failTx();
        assert(policy.phase()==ResendPolicy::Phase::Fatal && policy.cause()==ResendPolicy::Cause::TxStall);
        std::string expected;for(unsigned i=0;i<63;++i)expected+=char(i);
        assert(retired+drain()==expected);
        // Terminal notification still uses the separate nonblocking path.
        const char msg[]="!! COMMUNICATION LOST\r\n";
        assert(UsbSerial::tryWriteSpan(msg,sizeof(msg)-1));
        assert(drain()==msg);
        assert(UsbSerial::blockingTxFailed());
    }
    std::printf("Bounded TX %s: %u poll visits, FIFO/terminal evidence passed\n",argv[1],visits);
}
'''

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",type=Path,required=True)
    parser.add_argument("--out",type=Path,required=True)
    args=parser.parse_args()
    assert subprocess.check_output(["git","-C",str(args.source),"rev-parse","HEAD"],text=True).strip()==P0
    args.out.mkdir(parents=True,exist_ok=True)
    source=args.source/"Repetier"
    hdr,src=candidate((source/"UsbSerial.h").read_text(),(source/"UsbSerial.cpp").read_text())
    (args.out/"UsbSerial.h").write_text(hdr);(args.out/"UsbSerial.cpp").write_text(src)
    # Test-only ISR progress hook after each bounded poll. Shipping-shaped
    # candidate above contains no hook or external scheduler invocation.
    check="            blockingTxFailed_=true;\n            return;\n        }"
    assert src.count(check)==2
    fixture=src.replace(check,check+"\n        researchWaitStep();")
    (args.out/"usb-serial-firmware.inc").write_text(re.sub(r"^#include <avr/\w+\.h>\n","",fixture,flags=re.M))
    (args.out/"replay.cpp").write_text(TEST)
    for flags in (["-O2"],["-O1","-fsanitize=undefined","-fno-sanitize-recover=all"]):
        with tempfile.TemporaryDirectory() as d:
            exe=Path(d)/"replay"
            subprocess.run(["c++","-std=c++11","-Wall","-Wextra","-Werror",*flags,
                "-I",str(ROOT/"tools/task0032"),str(args.out/"replay.cpp"),"-o",str(exe)],check=True)
            for mode in ("write-good","flush-good","write-masked","flush-masked","write-stall","flush-stall","flush-partial"):
                print(subprocess.check_output([str(exe),mode],text=True),end="")
if __name__=="__main__":
    main()

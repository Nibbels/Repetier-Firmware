#!/usr/bin/env python3
"""Actual P0 USART ring + bounded resend control preflight.

Reuses the earlier sticky RX gate, tests its atomic discard/reopen candidate
and exercises the retry policy against the real tryWriteSpan/UDRE code.
Frame/boundary trust and actuator stop are explicitly external at this stage.
No production edits; no physical UART/interrupt timing simulation.
"""
import argparse
from pathlib import Path
import os
import re
import runpy
import subprocess
from tempfile import TemporaryDirectory

ROOT = Path(__file__).resolve().parents[2]
P0 = "4ca16f6e09f8c4d7a8691de34281bee95605c578"

def once(s, old, new):
    assert s.count(old) == 1, old[:100]
    return s.replace(old, new)

REARM = r"""
// Foreground only; this reopens the byte ring, not command admission.
// Any pending hardware byte/error remains for the real RX ISR to sample.
// The parser still has to discard stale tails and prove the expected frame.
uint8_t UsbSerial::rearmForResync() {
    const uint8_t previous = SREG;
    cli();
    const uint8_t observed = lossFlags_;
    rx_.tail = rx_.head;
    lossFlags_ = 0;
    asm volatile("" ::: "memory");
    SREG = previous;
    asm volatile("" ::: "memory");
    return observed;
}
"""

SCENARIOS = r'''
#include "resend-policy.h"
#include <string>

using Phase = ResendPolicy::Phase;
using Cause = ResendPolicy::Cause;

static void rx(uint8_t byte, uint8_t flags=0) {
    UCSR0A = flags; UDR0 = byte; USART0_RX_vect();
}
static std::string drain() {
    std::string bytes;
    while (UsbSerial::transmitFree() != 63) {
        USART0_UDRE_vect();
        bytes += char(UDR0);
    }
    USART0_UDRE_vect();
    return bytes;
}
static void fillTx(unsigned count) {
    for (unsigned i=0; i<count; ++i)
        assert(UsbSerial::tryWrite(uint8_t(i)));
}
static bool publish(uint32_t line) {
    return publishResend<UsbSerial>(line);
}

static void rearmCases() {
    unsigned cases=0;
    for (unsigned cursor=0; cursor<128; ++cursor) {
        for (uint8_t flag : {uint8_t(_BV(FE0)), uint8_t(_BV(DOR0)),
                             uint8_t(_BV(UPE0)), uint8_t(1)}) {
            for (uint8_t sreg : {uint8_t(0x15),uint8_t(0x95)}) {
                const unsigned count=flag==1?127:11;
                for (unsigned i=0;i<count;++i) rx(uint8_t(i));
                rx(0xef,flag==1?0:flag);
                assert(UsbSerial::lossFlags()==flag);
                rx(0xbe); // quarantined
                rx(0xad,_BV(FE0)|_BV(DOR0));
                SREG=sreg;
                assert(UsbSerial::rearmForResync()==(flag|_BV(FE0)|_BV(DOR0)));
                assert(SREG==sreg && !UsbSerial::available() && !UsbSerial::lossFlags());
                rx(0x12);rx(0x34);
                assert(UsbSerial::read()==0x12 && UsbSerial::read()==0x34);
                assert(UsbSerial::read()==-1);
                // New loss immediately after clearing must be sticky again.
                rx(0x88,_BV(UPE0));rx(0x99);
                assert(UsbSerial::lossFlags()==_BV(UPE0) && !UsbSerial::available());
                assert(UsbSerial::rearmForResync()==_BV(UPE0) && SREG==sreg);
                // Return to same cursor. A ring advances by count+2.
                for (unsigned i=(count+2)%128;i<128 && (count+2)%128!=0;++i) {
                    rx(0);assert(UsbSerial::read()==0);
                }
                ++cases;
            }
        }
        rx(0);assert(UsbSerial::read()==0);
    }
    assert(cases==1024);
    std::printf("RX rearm: %u cursor/cause/SREG cases\n",cases);
}

static void exactWire() {
    unsigned cases=0;
    const uint32_t lines[]={0,1,65535,65536,4294967295UL};
    for (unsigned cursor=0;cursor<64;++cursor) {
        for (unsigned occupancy=0;occupancy<64;++occupancy) {
            for (uint32_t line:lines) {
                fillTx(occupancy);
                const std::string expected="\r\nResend:"+std::to_string(line)+"\r\nok\r\n";
                const bool sent=publish(line);
                assert(sent==(occupancy+expected.size()<=63));
                const std::string bytes=drain();
                assert(bytes.size()==occupancy+(sent?expected.size():0));
                for(unsigned i=0;i<occupancy;++i)assert(uint8_t(bytes[i])==i);
                if(sent) assert(bytes.substr(occupancy)==expected);
                for(unsigned i=bytes.size();i<64;++i) {
                    assert(UsbSerial::tryWrite(0));USART0_UDRE_vect();
                }
                USART0_UDRE_vect();
                ++cases;
            }
        }
        assert(UsbSerial::tryWrite(0));drain();
    }
    assert(cases==20480);
    std::printf("Resend span: %u cursor/occupancy/sequence cases\n",cases);
}

static void retries() {
    const auto tick=[](ResendPolicy& p,uint32_t now,bool ready=true,bool ack=true){
        p.service(now,ready,ack,publish);
    };
    const std::string reply="\r\nResend:42\r\nok\r\n";
    for(unsigned failures=0;failures<3;++failures) {
        ResendPolicy p;
        uint32_t now=0xfffffff0;
        // Corrected episodes never accumulate a fatal retry budget.
        for(unsigned episode=0;episode<300;++episode) {
            p.fault(42,true,now);tick(p,now);
            assert(p.phase()==Phase::Waiting && p.requests()==1 && drain()==reply);
            for(unsigned i=0;i<failures;++i) {
                for(unsigned j=0;j<20;++j)p.fault(99,true,now);
                tick(p,now+1999);assert(drain().empty());
                now+=2000;tick(p,now);
                assert(p.phase()==Phase::Waiting && p.requests()==i+2);
                assert(p.expected()==42 && drain()==reply);
            }
            // Represents actual final admission fence, not zero sync alone.
            p.committedExpected();
            assert(p.phase()==Phase::Healthy && p.requests()==0);
            now+=17;
        }
    }
    {
        ResendPolicy p;p.fault(42,true,0);
        fillTx(63);
        tick(p,0);
        for(unsigned now=1;now<1999;++now) {
            p.fault(42,true,now);tick(p,now);
            assert(p.phase()==Phase::Pending && p.requests()==0);
        }
        assert(drain().size()==63);
        tick(p,1999);assert(p.requests()==1 && drain()==reply);
        tick(p,3998);assert(drain().empty());
        tick(p,3999);assert(p.requests()==2 && drain()==reply);
    }
    {
        ResendPolicy p;p.fault(42,true,0);
        tick(p,0,false);tick(p,90000,false);
        assert(p.requests()==0 && drain().empty());
        tick(p,90000,true,false);
        tick(p,91999,true,false);assert(p.phase()==Phase::Pending);
        // ACK ordering may not hold indefinitely while storage is ready.
        tick(p,92000,true,false);
        assert(p.phase()==Phase::Fatal && p.cause()==Cause::TxStall);
    }
    {
        ResendPolicy p;p.fault(42,true,0);tick(p,0);
        assert(drain()==reply);
        tick(p,1000,false);tick(p,100000,false);
        tick(p,100000);tick(p,100999);assert(drain().empty());
        tick(p,101000);assert(p.requests()==2 && drain()==reply);
        tick(p,103000);assert(p.requests()==3 && drain()==reply);
        tick(p,104999);assert(p.phase()==Phase::Waiting);
        tick(p,105000);assert(p.phase()==Phase::Fatal && p.cause()==Cause::Exhausted);
        p.fault(43,true,105001);p.committedExpected();tick(p,110000);
        assert(p.phase()==Phase::Fatal && p.requests()==3 && drain().empty());
    }
    {
        ResendPolicy p;p.fault(42,false,0);tick(p,0);
        assert(p.phase()==Phase::Fatal && p.cause()==Cause::Unreplayable);
        assert(drain().empty());
    }
    {
        ResendPolicy p;p.fault(42,true,0);fillTx(63);tick(p,0);
        tick(p,2000);assert(p.phase()==Phase::Fatal && p.cause()==Cause::TxStall);
        assert(p.requests()==0 && drain().size()==63);
    }
    std::puts("Resend budget: 900 corrected episodes, timeout, TX stall, ACK order, wrap, backpressure and terminal latch passed");
}
'''

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True,exist_ok=True)
    assert subprocess.check_output(["git","-C",str(args.source),"rev-parse","HEAD"],text=True).strip()==P0
    gate=runpy.run_path(str(ROOT/"tools/task0032/loss-gate-replay.py"))
    header=(args.source/"Repetier/UsbSerial.h").read_text()
    source=(args.source/"Repetier/UsbSerial.cpp").read_text()
    header=once(header,"    static bool available();",
        "    static uint8_t lossFlags() { return lossFlags_; }\n"
        "    static uint8_t rearmForResync();\n    static bool available();")
    header=once(header,"    static Ring<receiveBufferSize> rx_;",
        "    static volatile uint8_t lossFlags_;\n    static Ring<receiveBufferSize> rx_;")
    source=once(source,gate["BASE_RX"],gate["GATED_RX"])
    source=once(source,"void UsbSerial::begin(unsigned long baud) {",
        "volatile uint8_t UsbSerial::lossFlags_ = 0;\n"+REARM+
        "\nvoid UsbSerial::begin(unsigned long baud) {")
    test=(args.source/"tests/usb-serial.cpp").read_text()
    test=once(test,"enum { U2X0 = 1,","enum { FE0=4, DOR0=3, UPE0=2, U2X0 = 1,")
    test=once(test,'#include "usb-serial-firmware.inc"',
        'volatile uint8_t SREG;\nvoid cli(){SREG &= 0x7f;}\n#include "usb-serial-firmware.inc"\n'+SCENARIOS)
    # Legacy test intentionally overflows silently; the dedicated rearm
    # cases now test that this overflow closes the new gate.
    test=once(test,"byte <= UsbSerial::receiveCapacity","byte < UsbSerial::receiveCapacity")
    test=once(test,"    nonblockingSpan();","    nonblockingSpan();\n    rearmCases();\n    exactWire();\n    retries();")
    # Original wrap test left status from the baud test, which is 0 for
    # errors; our additional injections explicitly set status each time.
    (args.out/"UsbSerial.h").write_text(header)
    (args.out/"UsbSerial.cpp").write_text(source)
    (args.out/"usb-serial-firmware.inc").write_text(re.sub(r"^#include <avr/\w+\.h>\n","",source,flags=re.M))
    (args.out/"replay.cpp").write_text(test)
    for mode,flags in [("optimized",["-O2"]),("ubsan",["-O1","-fsanitize=undefined","-fno-sanitize-recover=all"])]:
        with TemporaryDirectory() as d:
            exe=Path(d)/"replay"
            subprocess.run([os.environ.get("CXX","c++"),"-std=c++11","-Wall","-Wextra","-Werror",
                *flags,"-I",str(ROOT/"tools/task0032"),str(args.out/"replay.cpp"),"-o",str(exe)],check=True)
            result=subprocess.check_output([str(exe)],text=True,timeout=30)
            (args.out/(mode+".log")).write_text(result)
            print(mode+":\n"+result,end="")

if __name__=="__main__":
    main()

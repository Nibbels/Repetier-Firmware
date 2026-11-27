#!/usr/bin/env python3
"""Exercise ordinary ACK staging using the actual pinned P0 TX span API.

Temporary host-only test; does not change parser, thermal emergency, command
admission or shipping code. Tests a possible small staging primitive, not a
decision to defer all input behind ACK or a guarantee of TX service.
"""
from pathlib import Path
from tempfile import TemporaryDirectory
import argparse
import re
import shutil
import subprocess
import os

ROOT = Path(__file__).resolve().parents[2]

CANDIDATE = r'''
// One foreground writer. An ACK is an intent, not a second transmit queue.
// A caller may stage once after trusted admission, then retry without
// regenerating command effects. Emergency policy is deliberately outside.
struct PendingAck {
    uint32_t line = 0;
    bool pending = false;
    bool stage(uint32_t n) {
        if(pending)return false;
        line=n;pending=true;return true;
    }
    bool service() {
        if(!pending)return true;
        char reply[15]; // "ok " + 10 decimal digits + CRLF, no NUL
        char* end=reply+sizeof(reply);
        *--end='\n';*--end='\r';
        uint32_t n=line;
        while(n>UINT16_MAX) {
            uint32_t q=n/10;
            *--end='0'+(n-10*q);n=q;
        }
        uint16_t v=static_cast<uint16_t>(n);
        do {
            uint16_t q=v/10;
            *--end='0'+(v-10*q);v=q;
        }while(v);
        *--end=' ';*--end='k';*--end='o';
        const uint8_t count=reply+sizeof(reply)-end;
        if(!UsbSerial::tryWriteSpan(end,count))return false;
        pending=false;return true;
    }
};
void stagedAck() {
    const uint32_t lines[]={0,9,10,99,100,12345,65535,65536,4294967295UL};
    unsigned tests=0;
    for(unsigned phase=0;phase<64;++phase) {
        for(unsigned used=0;used<64;++used) {
            for(uint32_t line:lines) {
                char expected[16];
                int n=std::snprintf(expected,sizeof(expected),"ok %lu\r\n",static_cast<unsigned long>(line));
                assert(n>0 && n<=15);
                for(unsigned j=0;j<used;++j)assert(UsbSerial::tryWrite(uint8_t(j)));
                PendingAck ack;
                assert(ack.stage(line) && !ack.stage(line+1));
                bool sent=ack.service();
                assert(sent==(used+unsigned(n)<=63));
                unsigned consumed=0;
                if(!sent) {
                    for(int i=0;i<3;++i)assert(!ack.service());
                    while(UsbSerial::transmitFree()<n) {
                        USART0_UDRE_vect();assert(UDR0==consumed++);
                    }
                    assert(ack.service());
                }
                assert(ack.service() && !ack.pending);
                while(consumed<used) {
                    USART0_UDRE_vect();assert(UDR0==consumed++);
                }
                for(int j=0;j<n;++j) {
                    USART0_UDRE_vect();assert(UDR0==uint8_t(expected[j]));
                }
                USART0_UDRE_vect();
                assert(!(UCSR0B&_BV(UDRIE0)));
                // Return to same ring cursor for next fixture.
                for(unsigned j=used+n;j%64!=0;++j) {
                    assert(UsbSerial::tryWrite(0));USART0_UDRE_vect();
                }
                USART0_UDRE_vect();++tests;
            }
        }
        assert(UsbSerial::tryWrite(0));USART0_UDRE_vect();USART0_UDRE_vect();
    }
    assert(tests==64*64*9);
    std::printf("ACK intent: %u cursor/occupancy/line cases, exact FIFO bytes\n",tests);
}
'''


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--p0",required=True,type=Path,help="directory with exact pinned P0 UsbSerial.cpp/.h and usb-serial.cpp")
    args=p.parse_args()
    with TemporaryDirectory(prefix="task0032-ack-") as directory:
        d=Path(directory)
        shutil.copyfile(args.p0/"UsbSerial.h",d/"UsbSerial.h")
        source=(args.p0/"UsbSerial.cpp").read_text()
        (d/"usb-serial-firmware.inc").write_text(re.sub(r"^#include <avr/\w+\.h>\n","",source,flags=re.M))
        s=(args.p0/"usb-serial.cpp").read_text()
        anchor="} // namespace\n\nint main()"
        assert s.count(anchor)==1
        s=s.replace(anchor,CANDIDATE+"\n"+anchor)
        anchor="    nonblockingSpan();"
        assert s.count(anchor)==1
        s=s.replace(anchor,anchor+"\n    stagedAck();")
        (d/"test.cpp").write_text(s)
        exe=d/"replay"
        subprocess.run([os.environ.get("CXX","c++"),"-std=c++11","-O2","-Wall","-Wextra","-Werror",str(d/"test.cpp"),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True,timeout=10)


if __name__=="__main__":
    main()

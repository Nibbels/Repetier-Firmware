#!/usr/bin/env python3
"""Port the existing canonical serial scenarios to the actual stage2 UART.

Use the actual generated focused fixture, no parser substitutions. The
canonical host deque represents not-yet-transmitted bytes; feed one byte
through the real RX ISR on each demand, preserving old sustained-input
and visit-budget assertions without simulating unpaced hardware overflow.
Merge real ACK spans and mocked Com diagnostics into the same ordered wire.
The old unbounded resends of unidentified input are intentionally replaced
by terminal assertions in fresh processes. No shipping source changes.
"""
from pathlib import Path
import argparse
import re
import runpy
import shutil
import subprocess
import sys
import tempfile

HERE=Path(__file__).resolve().parent
once=runpy.run_path(str(HERE/"resend-ring-preflight.py"))["once"]

MODES=("decimal32-wire","wire-ascii","wire-binary-v1","wire-binary-v2","wire-mixed",
       "wire-unsupported-11","wire-unsupported-12","wire-unsupported-13",
       "wire-unsupported-14","wire-unsupported-15","numbered","queue","display","idle",
       "binary","overflow","frame-limit","binary-text","binary-text-clipped-80",
       "binary-text-clipped-255","sustained","bounded-receive","emergency","stopping",
       "internal-program-serial","soft-filament","hard-filament","output-program",
       "post-stop-output","legacy-ascii","legacy-binary","legacy-during-stop",
       "legacy-corrupt","inch-ascii","inch-binary","thermal-emergency","m206-syntax")

def canonical_cpp(s):
    # This fixture is a separate process from the focused real-ring
    # injection tests. Source-level injection seams are not installed.
    s=once(s,"bool serialByteAvailable() { return UsbSerial::available(); }",
        """bool serialByteAvailable() {
    if(!UsbSerial::available() && !incoming.empty()){
        UCSR0A=0;UDR0=incoming.front();incoming.pop_front();USART0_RX_vect();
    }
    return UsbSerial::available();
}""")
    # Replace call sites in the host fixture only, NOT extracted firmware.
    s=s.replace("GCodeInput::readFromSerial();","canonicalRead();")
    s=s.replace("Commands::commandLoop();","canonicalLoop();")
    s=once(s,"namespace Com {",
        "namespace { void drainCanonicalTx(); void canonicalRead(); void canonicalLoop(); }\nnamespace Com {")
    s=once(s,"    replies += value;","    drainCanonicalTx();replies += value;")
    s=once(s,"    replies += text;","    drainCanonicalTx();replies += text;")
    s=once(s,"void print(Number value) { replies += std::to_string(value); }",
        "void print(Number value) { drainCanonicalTx();replies += std::to_string(value); }")
    s=once(s,'void println() { replies += "\\r\\n"; }',
        'void println() { drainCanonicalTx();replies += "\\r\\n"; }')
    s=once(s,"void assertQuiet() {",
        """void drainCanonicalTx(){replies+=txDrain();}
void canonicalRead(){GCodeInput::readFromSerial();drainCanonicalTx();}
void canonicalLoop(){Commands::commandLoop();drainCanonicalTx();}
void assertQuiet() {""")

    # Preserve the selected original parser and numeric coverage. Invalid
    # extension fields now terminate (one independent process per bit).
    s=once(s,"void unsupportedExtensionBaseline() {","void unsupportedExtensionBaseline(unsigned bit) {")
    s=once(s,"    for (unsigned bit=11; bit<=15; ++bit) {",
        "    assert(bit>=11 && bit<=15);\n    {")
    s=once(s,r'''        readReply("\r\nResend:1\r\nok\r\n");
        assert(!GCodeInput::peekCurrentCommand() && dispatchedCommands==0);
    }
    assert(flushes==5);
    std::puts("TASK0032_UNSUPPORTED_REJECT five extension flags rejected, zero admission");''',
        '''        readReply("");
        assert(GCodeInput::transportFailed() && !GCodeInput::peekCurrentCommand() && !dispatchedCommands);
    }
    canonicalLoop();
    assert(flushes==0 && transportStops==1);
    std::puts("TASK0032_UNSUPPORTED_REJECT extension rejected, zero admission and terminal");''')
    s=once(s,'feed(numbered("N"+token+" M110"));',
        'feed("N"+token+" M110\\n"); // numeric parsing, not checked-envelope policy')
    s=once(s,"    // Accepted checksums keep the old uint8_t narrowing semantics.",
        "    // Checked numbered checksum aliases must no longer narrow modulo 256.")
    s=once(s,r'''    readReply("ok 1\r\n");popM(105);
    assert(incoming.empty()&&!GCodeInput::peekCurrentCommand());
    puts("decimal32 wire: 84 actual field/reset cases plus checksum narrowing PASS");''',
        '''    readReply("");
    assert(GCodeInput::transportFailed() && incoming.empty() && !GCodeInput::peekCurrentCommand());
    canonicalLoop();assert(transportStops==1);
    puts("decimal32 wire: 84 actual field/reset cases plus strict checksum rejection PASS");''')

    s=once(s,r'''    readReply("Error:expected line 42 got 43\r\n\r\nResend:42\r\nok\r\n");
    assert(flushes == 1);''',
        r'''    readReply("\r\nResend:42\r\nok\r\n");
    assert(flushes == 0);''')
    s=once(s,r'''    readReply("Error:Wrong checksum\r\nCorrupted: N43 M105*0\r\n\r\nResend:43\r\nok\r\n");''',
        r'''    readReply("\r\nResend:43\r\nok\r\n");''')
    s=once(s,r'''    readReply("\r\nResend:1\r\nok\r\n");
    assert(!GCodeInput::peekCurrentCommand());
    assert(flushes == 1);
}

void binaryFrames()''',
        '''    readReply("");
    assert(!GCodeInput::peekCurrentCommand() && GCodeInput::transportFailed());
    canonicalLoop();assert(transportStops==1 && flushes==0);
}

void binaryFrames()''')
    # A full frame without delimiter has no checked line identity. Retain
    # the 96th-byte terminator positive control in a separate clean process.
    s=once(s,r'''    readReply("\r\nResend:1\r\nok\r\n");
    assert(flushes == 1 && !GCodeInput::peekCurrentCommand());
    feed(numbered("N1 M105"));
    readReply("ok 1\r\n");
    popM(105);

    // A terminator in byte 96 is still a complete ASCII frame.
    feed("M105" + std::string(91, ' ') + "\n");
    readReply("ok 1\r\n");''',
        r'''    readReply("");
    assert(GCodeInput::transportFailed() && !GCodeInput::peekCurrentCommand());
    canonicalLoop();assert(transportStops==1 && flushes==0);
}
void frameLimit() {
    // A terminator in byte 96 is still a complete ASCII frame.
    feed("M105" + std::string(91, ' ') + "\n");
    readReply("ok 0\r\n");''')
    s=once(s,r'''    // A clipped packet with a correct Fletcher checksum must not make the
    // declared length escape the received payload or overwrite receiver state.
    for (uint8_t badLength : {80, 255}) {
        feed(panelPacket(badLength, 79));
        readReply("\r\nResend:1\r\nok\r\n");
        assert(!GCodeInput::peekCurrentCommand());
        feed(std::string(31, '\0') + next);
        readReply("ok 0\r\n");
        popM(105);
    }
    assert(flushes == 2);

''',"")
    s=once(s,"// Long discarded input must yield to foreground services even though it",
        """void binaryTextClipped(uint8_t badLength) {
    // Valid Fletcher over clipped bytes cannot legitimize an unnumbered
    // packet with an out-of-range declared payload.
    feed(panelPacket(badLength,79));
    readReply("");
    assert(GCodeInput::transportFailed() && !GCodeInput::peekCurrentCommand());
    canonicalLoop();assert(transportStops==1 && flushes==0);
}

// Long discarded input must yield to foreground services even though it""")
    s=once(s,r'''    auto packet = binary({0x82, 0x00, 105});
    // Establish binary mode before exercising the binary resend handshake.
    feed(packet);
    readReply("ok 0\r\n");
    popM(105);
    auto bad = packet;
    bad.back() ^= 1;
    feed(bad);
    readReply("Error:Wrong checksum\r\n\r\nResend:1\r\nok\r\n");''',
        r'''    // Establish an actual checked numbered binary stream. Unnumbered
    // traffic cannot prove which packet would need to be replayed.
    feed(binary({0x83,0,1,0,105}));
    readReply("ok 1\r\n");popM(105);
    auto packet=binary({0x83,0,2,0,105});
    auto bad=packet;bad.back()^=1;feed(bad);
    readReply("Error:Wrong checksum\r\n\r\nResend:2\r\nok\r\n");''')
    s=once(s,r'''    readReply("ok 0\r\n");
    popM(105);
    assert(incoming.empty());
}

void sustainedAdmission()''',
        r'''    readReply("ok 2\r\n");
    popM(105);
    assert(incoming.empty());
}

void sustainedAdmission()''')
    s=once(s,r'''        feed("N1 M111 S8*0\n");
        canonicalLoop();
        assert(!legacyRejected && stopRequests == 0);
        replies.clear();
        feed(numbered("N1 M111 S" + std::to_string(legacyMask)));''',
        '''        feed(numbered("N1 M111 S" + std::to_string(legacyMask)));''')
    s=once(s,"#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\nvoid strictEepromSyntax()",
        r'''void legacyCorrupt() {
    feed("N1 M111 S8*0\n");
    canonicalLoop();
    assert(!legacyRejected && !stopRequests && transportStops==1 &&
           !GCodeInput::peekCurrentCommand() && !dispatchedCommands);
    assert(replies.find("ok")==std::string::npos);
}

#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
void strictEepromSyntax()''')
    s=once(s,r'''    txFree=0;
    for(unsigned n=0;n<1000;++n)canonicalLoop();
    assert(!incoming.empty()&&replies.empty()&&liveRequests==0&&periodicVisits==1001);
    txFree=63;''',
        '''    for(unsigned n=0;n<63;++n)assert(UsbSerial::tryWrite(0x55));
    for(unsigned n=0;n<1000;++n)Commands::commandLoop(); // deliberately no TX progress
    assert(!incoming.empty()&&replies.empty()&&liveRequests==0&&periodicVisits==1001);
    assert(txDrain()==std::string(63,'U'));''')
    # Keep the original thermal diagnostic wire and all typed script tests.
    s=once(s,'    else if (scenario == "wire-unsupported") unsupportedExtensionBaseline();',
        '    else if (scenario.rfind("wire-unsupported-",0)==0) unsupportedExtensionBaseline(unsigned(std::stoi(scenario.substr(17))));')
    s=once(s,'    else if (scenario == "overflow") overflowAndRecovery();',
        '''    else if (scenario == "overflow") overflowAndRecovery();
    else if (scenario == "frame-limit") frameLimit();
    else if (scenario == "binary-text-clipped-80") binaryTextClipped(80);
    else if (scenario == "binary-text-clipped-255") binaryTextClipped(255);
    else if (scenario == "legacy-corrupt") legacyCorrupt();''')
    return s

def run(source,fixture,out):
    out.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="task0032-canonical-serial-") as d:
        root=Path(d);fw=root/"Repetier";tests=root/"tests"
        shutil.copytree(source/"Repetier",fw);tests.mkdir()
        cpp=canonical_cpp((fixture/"receiver-fixture.cpp").read_text())
        py=(fixture/"receiver-fixture.py").read_text()
        py,n=re.subn(r"        for scenario in \([^\n]+\):",
            "        for scenario in "+repr(MODES)+":",py)
        assert n==1
        (tests/"serial-protocol.cpp").write_text(cpp)
        (tests/"serial-protocol.py").write_text(py)
        (out/"serial-protocol.cpp").write_text(cpp)
        (out/"serial-protocol.py").write_text(py)
        subprocess.run([sys.executable,str(tests/"serial-protocol.py")],check=True)
        print("Canonical serial coverage migrated:",len(MODES),"scenarios PASS",flush=True)

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source",type=Path,required=True)
    p.add_argument("--fixture",type=Path,required=True)
    p.add_argument("--out",type=Path,required=True)
    a=p.parse_args()
    run(a.source,a.fixture,a.out)

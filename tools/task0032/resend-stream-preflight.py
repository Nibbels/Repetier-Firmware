#!/usr/bin/env python3
"""Compose the earlier actual UART, ACK and parser candidates on pinned P0.

The host is a deterministic numbered-history emulator (not Repetier binaries).
Its known replayability and negotiated synchronization mode are explicit test
inputs. The test driver calls recovery service; production pump integration,
exact linked costs and physical terminal actuator shutdown are NOT exercised.
All changes are confined to a temporary source tree.
"""
import argparse
from pathlib import Path
import runpy
import shutil
import subprocess
import sys
from tempfile import TemporaryDirectory

ROOT=Path(__file__).resolve().parents[2]
ring=runpy.run_path(str(ROOT/"tools/task0032/resend-ring-preflight.py"))
frames=runpy.run_path(str(ROOT/"tools/task0032/resend-frame-preflight.py"))
ack=runpy.run_path(str(ROOT/"tools/task0032/tx-ack-replay.py"))
prior=runpy.run_path(str(ROOT/"tools/task0033/loss-replay.py"))
once=ring["once"]
P0=ring["P0"]

STREAM_HELPERS=r'''
bool researchReplayable=true; // supplied by the emulator, not inferred
bool researchBinarySession=false;
uint8_t researchZeros=0;
static void researchFault() {
    researchBoundary=false;
    researchRecovery.fault(serialReceive.lastLineNumber+1,
                           researchReplayable,HAL::timeInMilliseconds());
}
'''

SCENARIOS=r'''
unsigned researchStops=0;
void uartByte(uint8_t b,uint8_t status=0) {
    UCSR0A=status;UDR0=b;USART0_RX_vect();
}
void uartFeed(const std::string& data) {
    for(unsigned char b:data)uartByte(b);
}
std::string drainTx() {
    std::string result;
    while(UsbSerial::transmitFree()!=63) {
        USART0_UDRE_vect();result+=char(UDR0);
    }
    USART0_UDRE_vect();
    return result;
}
bool receiveReady() {
    return !commandQueue.full() && !(frame.borrowedText && !commandQueue.empty());
}
void streamVisit(uint32_t now) {
    clockMs=now;
    using Phase=ResendPolicy::Phase;
    if(researchRecovery.phase()==Phase::Fatal)return;
    if(UsbSerial::lossFlags())researchFault();
    researchAck.service();
    // Give an already buffered candidate a bounded parse visit before
    // evaluating its deadline. Unrelated tails cannot defer the timer.
    if(!researchAck.pending && !UsbSerial::lossFlags() && receiveReady()
       && UsbSerial::available()
       && (researchRecovery.phase()==Phase::Healthy || researchRecovery.phase()==Phase::Waiting)) {
        GCodeInput::readFromSerial();
    }
    if(UsbSerial::lossFlags())researchFault();
    researchAck.service();
    researchRecovery.service(now,receiveReady(),!researchAck.pending,[](uint32_t expected){
        // Reserve conservatively for the whole maximum-length span before
        // discarding RX. No other foreground TX producer in this visit.
        if(UsbSerial::transmitFree()<25)return false;
        UsbSerial::rearmForResync();
        frame.length=frame.expectedLength=0;
        frame.comment=frame.borrowedText=false;
        serialReceive.waitingForResend=-1;
        researchZeros=0;
        researchBoundary=!researchBinarySession;
        assert(publishResend<UsbSerial>(expected));
        return true;
    });
    if(researchRecovery.phase()==Phase::Fatal && !stopped) {
        ++researchStops;stopped=true; // test seam, not actuator cleanup
    }
}
std::string streamPacket(unsigned n) {
    return researchBinarySession
        ? (n==2 ? binary({0x85,0x10,0,0,uint8_t(n),uint8_t(n>>8),1,0})
                 : binary({0x85,0,uint8_t(n),uint8_t(n>>8),1}))
        : numbered("N"+std::to_string(n)+" G1 X"+std::to_string(n));
}
void corruptStream(const std::string& packet,unsigned pos,uint8_t flag) {
    assert(pos<packet.size());
    for(unsigned i=0;i<packet.size();++i)
        uartByte(uint8_t(packet[i]),i==pos?flag:0);
}
void expectCommand(unsigned n) {
    auto* c=GCodeInput::peekCurrentCommand();
    assert(c && c->hasN() && c->N==n);
    GCodeInput::popCurrentCommand();
}
void startStream(bool binaryMode) {
    researchStreamActive=true;researchBinarySession=binaryMode;
    uartFeed(streamPacket(1));streamVisit(0);
    assert(drainTx()=="ok 1\r\n");expectCommand(1);
}
const std::string resend2="\r\nResend:2\r\nok\r\n";

void streamRecovery(bool binaryMode,unsigned failures) {
    startStream(binaryMode);
    const auto p2=streamPacket(2),p3=streamPacket(3),p4=streamPacket(4);
    corruptStream(p2,0,_BV(FE0)); // corrupt first-byte protocol choice
    uartFeed(p3+p4); // already in flight, gate must discard
    uint32_t now=1;streamVisit(now);
    assert(drainTx()==resend2 && !GCodeInput::peekCurrentCommand());
    assert(researchRecovery.requests()==1);
    for(unsigned attempt=0;attempt<failures;++attempt) {
        if(binaryMode)uartFeed(std::string(32,'\0'));
        corruptStream(p2,p2.size()-1,attempt?_BV(UPE0):_BV(DOR0));
        uartFeed(p3+p4);
        streamVisit(now+1);
        assert(drainTx().empty() && !GCodeInput::peekCurrentCommand());
        streamVisit(now+1999);assert(drainTx().empty());
        now+=2000;streamVisit(now);
        assert(drainTx()==resend2 && researchRecovery.requests()==attempt+2);
    }
    // A future packet left in flight after rearm cannot count as success.
    if(binaryMode) {
        uartFeed(std::string(31,'\0')+p4);
    } else {
        uartFeed("broken-tail\n"+p4);
    }
    for(unsigned i=0;i<3;++i)streamVisit(now+2+i);
    assert(drainTx().empty() && !GCodeInput::peekCurrentCommand());
    assert(researchRecovery.phase()==ResendPolicy::Phase::Waiting);
    // Historical binary host sends 32 zero bytes, then retransmits history.
    // Ping-pong case sends one packet; streaming case sends two in one burst.
    if(binaryMode)uartFeed(std::string(32,'\0'));
    uartFeed(p2+(failures? p3:std::string()));
    for(unsigned i=0;i<40;++i)streamVisit(now+10+i);
    assert(researchRecovery.phase()==ResendPolicy::Phase::Healthy);
    assert(!researchStops);
    if(failures) {
        assert(drainTx()=="ok 2\r\nok 3\r\n");expectCommand(2);expectCommand(3);
    } else {
        assert(drainTx()=="ok 2\r\n");expectCommand(2);
        uartFeed(p3);streamVisit(now+60);assert(drainTx()=="ok 3\r\n");expectCommand(3);
    }
    assert(!GCodeInput::peekCurrentCommand() && serialReceive.lastLineNumber==3);
    std::printf("Composed history replay %s failures=%u passed\n",binaryMode?"binary":"ASCII",failures);
}
void streamAckOrder() {
    researchStreamActive=true;
    for(unsigned i=0;i<63;++i)assert(UsbSerial::tryWrite(uint8_t(i)));
    uartFeed(streamPacket(1));streamVisit(0);
    assert(researchAck.pending && GCodeInput::peekCurrentCommand());
    corruptStream(streamPacket(2),2,_BV(FE0));streamVisit(1);
    assert(researchRecovery.phase()==ResendPolicy::Phase::Pending && !researchRecovery.requests());
    assert(drainTx().size()==63);
    streamVisit(2);
    assert(drainTx()=="ok 1\r\n"+resend2 && researchRecovery.requests()==1);
    expectCommand(1);uartFeed(streamPacket(2));streamVisit(3);
    assert(drainTx()=="ok 2\r\n");expectCommand(2);
    assert(researchRecovery.phase()==ResendPolicy::Phase::Healthy);
}
void streamBorrowed() {
    researchStreamActive=true;
    uartFeed(numbered("N1 M117 retained message"));streamVisit(0);
    assert(drainTx()=="ok 1\r\n");
    const auto* msg=GCodeInput::peekCurrentCommand();
    assert(msg && msg->text==std::string("retained message"));
    corruptStream(streamPacket(2),2,_BV(DOR0));streamVisit(1);
    streamVisit(90000);
    assert(msg->text==std::string("retained message") && !researchStops);
    assert(drainTx().empty() && researchRecovery.phase()==ResendPolicy::Phase::Pending);
    expectCommand(1);streamVisit(90001);assert(drainTx()==resend2);
    uartFeed(streamPacket(2));streamVisit(90002);
    assert(drainTx()=="ok 2\r\n");expectCommand(2);
}
void streamFatal(bool replayable) {
    startStream(false);researchReplayable=replayable;
    corruptStream(streamPacket(2),0,_BV(FE0));streamVisit(1);
    if(replayable) {
        assert(drainTx()==resend2);
        streamVisit(2001);assert(drainTx()==resend2);
        streamVisit(4001);assert(drainTx()==resend2);
        streamVisit(6001);
        assert(researchRecovery.cause()==ResendPolicy::Cause::Exhausted);
    } else {
        assert(researchRecovery.cause()==ResendPolicy::Cause::Unreplayable);
    }
    assert(researchStops==1 && stopped && drainTx().empty());
    uartFeed(streamPacket(2)+numbered("N3 M110")+numbered("N4 M112"));
    streamVisit(7000);
    GCodeInput::readFromSerial(); // direct receiver invocation also fenced
    assert(researchStops==1 && !GCodeInput::peekCurrentCommand() && drainTx().empty());
}
'''

def patch_receiver(s):
    s=once(s,'#include "UsbSerial.h"','#include "UsbSerial.h"\n#include "resend-policy.h"')
    helper=frames["HELPERS"].replace("return researchBoundary && !Printer",
        "return researchBoundary && (!researchStreamActive || !UsbSerial::lossFlags()) && !Printer")
    pending=ack["CANDIDATE"].split("void stagedAck()")[0]
    s=once(s,"} // namespace\n",helper+pending+"\nPendingAck researchAck;\n"+STREAM_HELPERS+"\n} // namespace\n")
    for binary,anchor in [(True,"                if (parseBinary(command, frame.bytes, frame.expectedLength))"),
                          (False,"                if (parseAscii(command, (char*)frame.bytes, true))")]:
        branch=frames["recovery_branch"](binary)
        branch=once(branch,"                        researchRecovery.committedExpected();",
            "                        if(!researchStreamActive || researchAck.pending) researchRecovery.committedExpected();\n"
            "                        else { serialReceive.currentLineNumber=oldLine; serialReceive.formatErrors=oldErrors; frame.borrowedText=false; }")
        s=once(s,anchor,branch+anchor)
    s=once(s,"void GCodeInput::requestResend() {","void GCodeInput::requestResend() {\n"
        "    if(researchStreamActive){researchFault();frame.length=frame.expectedLength=0;\n"
        "        frame.comment=frame.borrowedText=false;return;}")
    s=once(s,"void GCodeInput::checkAndPushCommand(GCode& command) {",
        "void GCodeInput::checkAndPushCommand(GCode& command) {\n"
        "    if(researchStreamActive && UsbSerial::lossFlags()){researchFault();return;}")
    s=once(s,"    Com::printFLN(Com::tOkSpace, serialReceive.currentLineNumber);",
        "    if(researchStreamActive) {assert(researchAck.stage(serialReceive.currentLineNumber));}\n"
        "    else Com::printFLN(Com::tOkSpace, serialReceive.currentLineNumber);")
    s=once(s,"void GCodeInput::readFromSerial() {","void GCodeInput::readFromSerial() {\n"
        "    if(researchStreamActive && (UsbSerial::lossFlags() || researchAck.pending ||\n"
        "       researchRecovery.phase()==ResendPolicy::Phase::Pending || researchRecovery.phase()==ResendPolicy::Phase::Fatal))return;")
    s=once(s,"        if (frame.length == 1 && !frame.comment) {",
        "        if (frame.length == 1 && !frame.comment) {\n"
        "            if(researchStreamActive && researchRecovery.phase()==ResendPolicy::Phase::Waiting &&\n"
        "               researchBinarySession && !researchBoundary){\n"
        "                if(frame.bytes[0])researchZeros=0;else if(++researchZeros==31)researchBoundary=true;\n"
        "                frame.length=0;continue;\n"
        "            }")
    s=once(s,"            frame.binary = (frame.bytes[0] & 128) != 0;",
        "            frame.binary = !(researchStreamActive && researchRecovery.phase()==ResendPolicy::Phase::Waiting &&\n"
        "                !researchBinarySession) && (frame.bytes[0] & 128) != 0;")
    return s

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",type=Path,required=True)
    args=parser.parse_args()
    assert subprocess.check_output(["git","-C",str(args.source),"rev-parse","HEAD"],text=True).strip()==P0
    with TemporaryDirectory(prefix="task0032-resend-stream-") as d:
        root=Path(d);fw=root/"Repetier";tests=root/"tests"
        shutil.copytree(args.source/"Repetier",fw);tests.mkdir()
        shutil.copyfile(ROOT/"tools/task0032/resend-policy.h",fw/"resend-policy.h")
        for name in ("serial-protocol.cpp","serial-protocol.py","usb-serial.cpp"):
            shutil.copyfile(args.source/"tests"/name,tests/name)
        header,source=ring["candidate_uart"]((fw/"UsbSerial.h").read_text(),(fw/"UsbSerial.cpp").read_text())
        (fw/"UsbSerial.h").write_text(header);(fw/"UsbSerial.cpp").write_text(source)
        path=fw/"GCodeInput.cpp";path.write_text(patch_receiver(path.read_text()))
        path=tests/"serial-protocol.cpp";s=path.read_text()
        s=once(s,'#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\n#include "UsbSerial.h"\n#endif','#include "UsbSerial.h"')
        s=once(s,"std::deque<uint8_t> incoming;","std::deque<uint8_t> incoming;\nbool researchStreamActive=false;")
        s=once(s,"bool serialByteAvailable() { return !incoming.empty(); }",
            "bool serialByteAvailable() { return researchStreamActive?UsbSerial::available():!incoming.empty(); }")
        s=once(s,"uint8_t serialReadByte() {","uint8_t serialReadByte() {\n    if(researchStreamActive){int b=UsbSerial::read();assert(b>=0);return uint8_t(b);}")
        regs=prior["REGISTERS"].replace('#include "usb-serial-firmware.inc"',
            'volatile uint8_t SREG;\nvoid cli(){SREG &= 0x7f;}\n#include "usb-serial-firmware.inc"')
        s=once(s,'#include "gcode-receiver.inc"','#include "gcode-receiver.inc"\n'+regs)
        s=once(s,"void numberedLines() {",SCENARIOS+"\nvoid numberedLines() {")
        s=once(s,'if (scenario == "numbered") numberedLines();',
            'if (scenario == "stream-a0") streamRecovery(false,0);\n'
            '    else if (scenario == "stream-a1") streamRecovery(false,1);\n'
            '    else if (scenario == "stream-a2") streamRecovery(false,2);\n'
            '    else if (scenario == "stream-b0") streamRecovery(true,0);\n'
            '    else if (scenario == "stream-b1") streamRecovery(true,1);\n'
            '    else if (scenario == "stream-b2") streamRecovery(true,2);\n'
            '    else if (scenario == "stream-ack") streamAckOrder();\n'
            '    else if (scenario == "stream-text") streamBorrowed();\n'
            '    else if (scenario == "stream-fatal") streamFatal(true);\n'
            '    else if (scenario == "stream-ambiguous") streamFatal(false);\n'
            '    else if (scenario == "numbered") numberedLines();')
        path.write_text(s)
        # This composition uses the actual USART instead of the emergency
        # replay's mock TX. Keep that independent two-mode suite in the
        # frame preflight; run all ordinary serial cases plus stream here.
        path=tests/"serial-protocol.py";s=path.read_text()
        names='"stream-a0","stream-a1","stream-a2","stream-b0","stream-b1","stream-b2","stream-ack","stream-text","stream-fatal","stream-ambiguous",'
        s=once(s,'for scenario in ("numbered",','for scenario in ('+names+'"numbered",')
        start=s.index('        subprocess.run(compiler + ["-DRF_THERMAL_EMERGENCY_EXPERIMENT')
        end=s.index('        uart_replay =',start)
        # Independent actual-UART exhaustive fixture is run separately.
        s=s[:start]+s[s.index('\n\nif __name__',end):]
        path.write_text(s)
        subprocess.run([sys.executable,str(path)],check=True)

if __name__=="__main__":
    main()

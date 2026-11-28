#!/usr/bin/env python3
"""Replay the actual stage-1 receiver, UART and outer dispatch together.

Real byte ring, parser, queue and extracted commandLoop; fake USART registers,
normal Com formatting and actuator/thermal transitions. The injection hooks
are test-only and confined to a temporary source copy. Not a resend or hardware
qualification; the post-admission ACK is intentionally allowed for a valid
frame whose loss occurred after publication.
"""
from pathlib import Path
import re
import runpy
import shutil
import subprocess
import sys
import tempfile

HERE=Path(__file__).resolve().parent
once=runpy.run_path(str(HERE/"resend-ring-preflight.py"))["once"]
prior=runpy.run_path(str(HERE.parents[1]/"tools/task0033/loss-replay.py"))

SCENARIOS=r'''
void rxByte(uint8_t byte,uint8_t status=0) {
    UCSR0A=status;UDR0=byte;USART0_RX_vect();
}
void rxFeed(const std::string& data) {
    for(unsigned char byte:data)rxByte(byte);
}
std::string txDrain() {
    std::string wire;
    while(UsbSerial::transmitFree()!=63){
        USART0_UDRE_vect();wire+=char(UDR0);
    }
    USART0_UDRE_vect();
    return wire;
}
void assertQuiet() {
    assert(replies.empty() && !GCodeInput::peekCurrentCommand());
    assert(!dispatchedCommands && !heaterRequests && !extrusionRequests);
    assert(txDrain().empty());
}
void promotedQuarantine() {
    // Model actual PrintJob's one-way transport->thermal promotion.
    assert(transportLocked && transportStops==1 && UsbSerial::transportFailed());
    transportLocked=false;thermalLocked=true;
    const auto live=liveRequests,replay=replayRequests;
    rxFeed(numbered("N3 M105")+numbered("N4 M115")+numbered("N5 M110")+"M112\n");
    for(unsigned n=0;n<8;++n){
        GCodeInput::readFromSerial();
        Commands::commandLoop();
    }
    assertQuiet();
    assert(transportStops==1 && liveRequests==live && replayRequests==replay);
}
void stageLoss(const std::string& mode) {
    if(mode=="stage-ring"){
        rxFeed(std::string(127,'q'));rxByte('9');
        assert(UsbSerial::lossFlags()&1);
    }else{
        rxFeed("G1 X");GCodeInput::readFromSerial();
        assert(!GCodeInput::peekCurrentCommand() && replies.empty());
        const uint8_t flag=mode=="stage-fe"?_BV(FE0):mode=="stage-dor"?_BV(DOR0):_BV(UPE0);
        rxByte('9',flag);rxByte('\n');
        assert(UsbSerial::lossFlags()==flag);
    }
    Commands::commandLoop();
    assert(transportLocked && transportStops==1);
    assertQuiet();promotedQuarantine();
}
void stageBoundary(const std::string& mode) {
    const bool after=mode=="stage-after";
    rxFeed(numbered(mode=="stage-m110"?"N1 M110":mode=="stage-m112"?"N1 M112":"N1 G1 X1"));
    injectBefore=!after;injectAfter=after;
    // Direct receiver call exercises the real final admission boundary;
    // outer dispatch must then retire, not execute, the accepted frame.
    GCodeInput::readFromSerial();
    assert(UsbSerial::lossFlags()==_BV(DOR0));
    if(after){
        assert(GCodeInput::peekCurrentCommand() && serialReceive.lastLineNumber==1);
        assert(replies=="ok 1\r\n");replies.clear();
    }else{
        assert(!GCodeInput::peekCurrentCommand() && serialReceive.lastLineNumber==0);
        assert(replies.empty());
    }
    Commands::commandLoop();
    assert(transportLocked && transportStops==1);
    assertQuiet();promotedQuarantine();
}
void stageBorrowed(bool full) {
    rxFeed("M117 retained message\n");GCodeInput::readFromSerial();
    auto* msg=GCodeInput::peekCurrentCommand();
    assert(msg && msg->text==std::string("retained message"));
    assert(replies=="ok 0\r\n");replies.clear();
    if(full){
        // Use two ordinary commands in a fresh scenario instead of
        // corrupting the borrowed frame: borrowed text prevents filling.
        GCodeInput::popCurrentCommand();
        rxFeed(numbered("N1 G1 X1"));GCodeInput::readFromSerial();
        rxFeed(numbered("N2 G1 X2"));GCodeInput::readFromSerial();
        assert(commandQueue.full());replies.clear();
    }
    rxByte('x',_BV(FE0));rxFeed("M112\n");
    GCodeInput::readFromSerial();
    assert(replies.empty());
    if(!full)assert(msg->text==std::string("retained message"));
    Commands::commandLoop();Commands::commandLoop();
    assert(transportLocked && transportStops==1);
    assertQuiet();promotedQuarantine();
}
void stageTx() {
    for(unsigned n=0;n<63;++n)assert(UsbSerial::tryWrite(uint8_t(n)));
    SREG=0x15;
    UsbSerial::write(0x99); // real full-ring masked failure
    assert(SREG==0x15 && UsbSerial::blockingTxFailed());
    SREG=0x95;
    rxFeed(numbered("N1 G1 X1")+"M112\n");
    Commands::commandLoop();
    assert(transportLocked && transportStops==1);
    const auto wire=txDrain();assert(wire.size()==63);
    for(unsigned n=0;n<63;++n)assert(uint8_t(wire[n])==n);
    assertQuiet();promotedQuarantine();
}
void stageThermalFirst() {
    thermalLocked=true;
    rxFeed(numbered("N1 M105"));GCodeInput::readFromSerial();
    assert(liveRequests==1 && txDrain()=="ok 1\r\n");
    rxByte('x',_BV(FE0));rxFeed(numbered("N2 M105")+numbered("N3 M115"));
    for(unsigned n=0;n<4;++n)Commands::commandLoop();
    assertQuiet();
    assert(thermalLocked && !transportStops && liveRequests==1 && !replayRequests);
}
'''

MODES=("stage-ring","stage-fe","stage-dor","stage-upe","stage-before",
       "stage-after","stage-m110","stage-m112","stage-text","stage-full",
       "stage-tx","stage-thermal-first")

def run(source,out):
    with tempfile.TemporaryDirectory(prefix="task0032-terminal-receiver-") as d:
        root=Path(d);fw=root/"Repetier";tests=root/"tests"
        shutil.copytree(source/"Repetier",fw)
        tests.mkdir()
        for name in ("serial-protocol.cpp","serial-protocol.py"):
            shutil.copyfile(source/"tests"/name,tests/name)
        # Add test hooks only, not a second implementation of admission.
        p=fw/"GCodeInput.cpp";s=p.read_text()
        s=once(s,"void GCodeInput::checkAndPushCommand(GCode& command) {",
            "void GCodeInput::checkAndPushCommand(GCode& command) {\n    researchBeforeCommit();")
        s=once(s,"        commandQueue.publish();","        commandQueue.publish();\n        researchAfterCommit();")
        p.write_text(s)
        p=tests/"serial-protocol.cpp";s=p.read_text()
        s=once(s,"bool thermalLocked=false;","bool thermalLocked=false,transportLocked=false;\nunsigned transportStops=0;")
        s=once(s,"uint8_t UsbSerial::transmitFree(){return txFree;}\nbool UsbSerial::tryWriteSpan(const char* bytes,uint8_t count) {\n    assert(count<=txFree);\n    replies.append(bytes,count);\n    txFree-=count;\n    return true;\n}\n","")
        regs=prior["REGISTERS"]
        regs=once(regs,"TXEN0 = 3,","UCSZ00=1, UCSZ01=2, TXEN0 = 3,")
        regs=once(regs,"UCSR0A, UCSR0B,","UCSR0A, UCSR0B, UCSR0C,")
        regs=once(regs,'#include "usb-serial-firmware.inc"',
            'volatile uint8_t SREG=0x95;\nvoid cli(){SREG&=0x7f;}\n#include "usb-serial-firmware.inc"')
        s=once(s,"namespace HAL {",regs+r'''
bool injectBefore=false,injectAfter=false;
void researchBeforeCommit(){
    if(injectBefore){injectBefore=false;UCSR0A=_BV(DOR0);UDR0=0;USART0_RX_vect();}
}
void researchAfterCommit(){
    if(injectAfter){injectAfter=false;UCSR0A=_BV(DOR0);UDR0=0;USART0_RX_vect();}
}
namespace HAL {''')
        s=once(s,"bool serialByteAvailable() { return !incoming.empty(); }",
            "bool serialByteAvailable() { return UsbSerial::available(); }")
        s=once(s,"    const auto byte = incoming.front();\n    incoming.pop_front();\n    return byte;",
            "    const int byte=UsbSerial::read();assert(byte>=0);return uint8_t(byte);")
        s=once(s,"    ++flushes;","    ++flushes;UsbSerial::flush();")
        s=once(s,"void stopPrint() { ++stopRequests; }",
            "void stopPrint() { ++stopRequests; }\nvoid stopForTransportFault(){assert(!thermalLocked&&!transportLocked);transportLocked=true;++transportStops;}")
        s=once(s,"    || thermalLocked\n","    || thermalLocked || transportLocked\n")
        s=once(s,"        ||thermalLocked\n","        ||thermalLocked || transportLocked\n")
        s=once(s,"    bool emergencyLocked() const {return thermalLocked;}",
            "    bool emergencyLocked() const {return thermalLocked||transportLocked;}\n"
            "    bool transportEmergency() const {return transportLocked;}\n"
            "    bool thermalEmergency() const {return thermalLocked;}")
        s=once(s,"        assert(!thermalLocked);\n#endif\n        ++pendingOutputVisits;",
            "        if(thermalLocked||transportLocked)return;\n#endif\n        ++pendingOutputVisits;")
        s=once(s,"void numberedLines() {",SCENARIOS+"\nvoid numberedLines() {")
        s=once(s,'    const std::string scenario = argv[1];',
            '''    const std::string scenario = argv[1];
    UsbSerial::begin();
    if(scenario=="stage-ring"||scenario=="stage-fe"||scenario=="stage-dor"||scenario=="stage-upe")stageLoss(scenario);
    else if(scenario=="stage-before"||scenario=="stage-after"||scenario=="stage-m110"||scenario=="stage-m112")stageBoundary(scenario);
    else if(scenario=="stage-text"||scenario=="stage-full")stageBorrowed(scenario=="stage-full");
    else if(scenario=="stage-tx")stageTx();
    else if(scenario=="stage-thermal-first")stageThermalFirst();
    else goto legacy;
    std::printf("Stage1 receiver: %s PASS\\n",argv[1]);return 0;
legacy:''')
        p.write_text(s)
        p=tests/"serial-protocol.py";s=p.read_text()
        s=once(s,'command_loop.index("void Commands::commandLoop()")',
            'command_loop.index("#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\\nstatic bool serviceTransportFailure()")')
        # Compile only this thermal-enabled focused fixture; canonical healthy
        # protocol regression is a separate gate, not silently waived.
        start=s.index('        replay = build / "serial-replay"')
        end=s.index('\n\nif __name__',start)
        s=s[:start]+'''        replay = build / "serial-replay"
        subprocess.run(compiler + ["-DRF_THERMAL_EMERGENCY_EXPERIMENT=1",str(tests / "serial-protocol.cpp"), "-o", str(replay)], check=True)
        for scenario in '''+repr(MODES)+''':
            subprocess.run([str(replay), scenario], check=True, timeout=10)
'''+s[end:]
        p.write_text(s)
        # Preserve generated fixture and exact hook-bearing body for reproduction.
        out.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(tests/"serial-protocol.cpp",out/"receiver-fixture.cpp")
        shutil.copyfile(tests/"serial-protocol.py",out/"receiver-fixture.py")
        shutil.copyfile(fw/"GCodeInput.cpp",out/"receiver-hooked.cpp")
        subprocess.run([sys.executable,str(p)],check=True)

if __name__=="__main__":
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",type=Path,required=True)
    parser.add_argument("--out",type=Path,required=True)
    args=parser.parse_args()
    run(args.source,args.out)

#!/usr/bin/env python3
"""Actual-source bounded receiver replay, using the terminal fixture.

A deterministic numbered-history emulator injects bytes into the real ring.
No runtime host oracle, no test-only recovery clock pump: readFromSerial and
commandLoop own the policy. Actuator transitions are instrumented stubs;
normal Com diagnostics are captured separately from the real ACK/resend ring.
"""
from pathlib import Path
import argparse
import runpy

HERE=Path(__file__).resolve().parent
terminal=runpy.run_path(str(HERE/"current-transport-receiver-replay.py"))

SCENARIOS=r'''
const std::string resend2="\r\nResend:2\r\nok\r\n";
std::string checkedPacket(unsigned line,bool binaryMode) {
    if(binaryMode)
        return line==2 ? binary({0x85,0x10,0,0,uint8_t(line),uint8_t(line>>8),1,0}) :
                         binary({0x85,0,uint8_t(line),uint8_t(line>>8),1});
    return numbered("N"+std::to_string(line)+" G1 X"+std::to_string(line));
}
std::string damagedPacket(unsigned line,bool binaryMode) {
    auto packet=checkedPacket(line,binaryMode);
    if(binaryMode)packet.back()^=1;
    else packet[packet.find('X')+1]='9';
    return packet;
}
void expectN(unsigned line) {
    const auto* code=GCodeInput::peekCurrentCommand();
    assert(code && code->N==uint16_t(line));
    GCodeInput::popCurrentCommand();
}
void startChecked(bool binaryMode=false) {
    rxFeed(checkedPacket(1,binaryMode));
    GCodeInput::readFromSerial();
    assert(txDrain()=="ok 1\r\n" && replies.empty());
    expectN(1);
    assert(checkedStream==(binaryMode?ReplayMode::Binary:ReplayMode::Ascii));
}
void visitAt(uint32_t now) {
    clockMs=now;
    GCodeInput::readFromSerial();
}
void startResend(bool binaryMode=false,bool gap=false) {
    startChecked(binaryMode);
    rxFeed(gap?checkedPacket(3,binaryMode):damagedPacket(2,binaryMode));
    visitAt(1);
    assert(!GCodeInput::peekCurrentCommand() && !transportStops);
    assert(recovery.phase()==ReplayPhase::Sending && recovery.requests()==1);
    assert(txDrain()==resend2);
    assert(replies.find("ok")==std::string::npos);replies.clear();
    visitAt(1);
    assert(recovery.phase()==ReplayPhase::Waiting);
}
void rejectCandidate(const std::string& data) {
    const auto oldLine=serialReceive.currentLineNumber;
    const auto oldErrors=serialReceive.formatErrors;
    rxFeed(data);GCodeInput::readFromSerial();
    assert(!GCodeInput::peekCurrentCommand() && !frame.borrowedText);
    assert(serialReceive.lastLineNumber==1 && serialReceive.currentLineNumber==oldLine);
    assert(serialReceive.formatErrors==oldErrors);
    assert(recovery.phase()==ReplayPhase::Waiting && txDrain().empty() && replies.empty());
}
void replayAscii(bool gap) {
    startResend(false,gap);
    rejectCandidate(numbered("N3 G1 X3"));
    rejectCandidate(numbered("N65538 G1 X9"));
    rejectCandidate(numbered("N2 M110"));
    rejectCandidate(numbered("N2 M112"));
    rejectCandidate(numbered("N2 N9 G1"));
    rejectCandidate("N2 M117 unchecked text\n");
    rejectCandidate("N2 M105*256\n");
    rxFeed(numbered("N2 M117 retained message")+checkedPacket(3,false));
    GCodeInput::readFromSerial();
    assert(recovery.phase()==ReplayPhase::Healthy && !recovery.requests());
    const auto* msg=GCodeInput::peekCurrentCommand();
    assert(msg && msg->text==std::string("retained message"));
    assert(txDrain()=="ok 2\r\n" && replies.empty());
    GCodeInput::readFromSerial();
    assert(msg->text==std::string("retained message") && txDrain().empty());
    expectN(2);GCodeInput::readFromSerial();
    assert(txDrain()=="ok 3\r\n");expectN(3);
    assert(!GCodeInput::peekCurrentCommand() && !transportStops);
}
void replayBinary() {
    startResend(true);
    rxFeed(checkedPacket(2,true));GCodeInput::readFromSerial();
    assert(!GCodeInput::peekCurrentCommand() && txDrain().empty());
    rejectCandidate(std::string(31,'\0')+binary({0x83,0,2,0,110}));
    rejectCandidate(std::string(31,'\0')+binary({0x83,0,2,0,112}));
    rxFeed(std::string(31,'\0')+checkedPacket(2,true)+checkedPacket(3,true));
    GCodeInput::readFromSerial();GCodeInput::readFromSerial();
    assert(txDrain()=="ok 2\r\nok 3\r\n" && replies.empty());
    expectN(2);expectN(3);
    assert(recovery.phase()==ReplayPhase::Healthy && !transportStops);
}
void replayRetries(bool fatal) {
    startResend();
    uint32_t now=1;
    for(unsigned attempt=1;attempt<3;++attempt){
        rejectCandidate(damagedPacket(2,false));
        visitAt(now+1999);assert(txDrain().empty() && recovery.requests()==attempt);
        now+=2000;visitAt(now);
        assert(txDrain()==resend2 && recovery.requests()==attempt+1);
        visitAt(now);
    }
    if(fatal){
        visitAt(now+2000);
        assert(GCodeInput::transportFailed() && recovery.cause()==ResendPolicy::Cause::Exhausted);
        Commands::commandLoop();
        assert(transportLocked && transportStops==1 && txDrain().empty());
        promotedQuarantine();
    }else{
        rxFeed(checkedPacket(2,false));visitAt(now+1);
        assert(txDrain()=="ok 2\r\n" && recovery.phase()==ReplayPhase::Healthy);
        expectN(2);
        // A later independent episode has a fresh budget.
        rxFeed(damagedPacket(3,false));visitAt(now+2);
        assert(txDrain()=="\r\nResend:3\r\nok\r\n" && recovery.requests()==1);
    }
}
void replayUnidentified(const std::string& mode) {
    if(mode=="replay-unidentified")rxFeed("G1 X1*0\n");
    else if(mode=="replay-alias")rxFeed(numbered("N1 F100 N2"));
    else {
        startChecked();
        rxFeed("N2 G1 X");
        visitAt(1);visitAt(202);
        assert(GCodeInput::transportFailed() && !recovery.requests());
        Commands::commandLoop();
        assert(transportStops==1 && txDrain().empty());
        return;
    }
    GCodeInput::readFromSerial();
    assert(GCodeInput::transportFailed() && !recovery.requests());
    Commands::commandLoop();
    assert(transportStops==1 && !GCodeInput::peekCurrentCommand() && txDrain().empty());
    assert(replies.find("ok")==std::string::npos);
}
void replayAck(bool fatal) {
    for(unsigned n=0;n<63;++n)assert(UsbSerial::tryWrite(uint8_t(n)));
    rxFeed(checkedPacket(1,false));visitAt(0);
    assert(pendingAck.pending && GCodeInput::peekCurrentCommand());
    rxFeed(checkedPacket(2,false));visitAt(1999);
    assert(pendingAck.pending && serialReceive.lastLineNumber==1);
    if(fatal){
        visitAt(2000);assert(GCodeInput::transportFailed());
        Commands::commandLoop();assert(transportStops==1);
        assert(txDrain().size()==63);assertQuiet();
    }else{
        assert(txDrain().size()==63);visitAt(1999);
        assert(txDrain()=="ok 1\r\nok 2\r\n");
        expectN(1);expectN(2);
        assert(!pendingAck.pending && !transportStops);
    }
}
void replayMarkerLoss() {
    startChecked();
    rxFeed(damagedPacket(2,false));visitAt(1);
    assert(recovery.phase()==ReplayPhase::Sending && UsbSerial::resyncPending());
    rxByte('x',_BV(DOR0));
    assert(txDrain()==resend2);
    assert(UsbSerial::lossFlags()==_BV(DOR0) && !UsbSerial::resyncPending());
    Commands::commandLoop();
    assert(transportStops==1 && !GCodeInput::peekCurrentCommand());
    promotedQuarantine();
}
void replayDispatchHold(bool ackOnly) {
    if(ackOnly) {
        for(unsigned n=0;n<63;++n)assert(UsbSerial::tryWrite(uint8_t(n)));
        rxFeed(checkedPacket(1,false));
        Commands::commandLoop();
        assert(pendingAck.pending && GCodeInput::dispatchDeferred());
        assert(GCodeInput::peekCurrentCommand() && !dispatchedCommands);
        assert(txDrain().size()==63);
        Commands::commandLoop();
        assert(txDrain()=="ok 1\r\n" && dispatchedCommands==1);
        assert(!GCodeInput::peekCurrentCommand() && !GCodeInput::dispatchDeferred());
    }else{
        rxFeed(checkedPacket(1,false));GCodeInput::readFromSerial();
        assert(txDrain()=="ok 1\r\n" && GCodeInput::peekCurrentCommand());
        rxFeed(damagedPacket(2,false));clockMs=1;
        Commands::commandLoop();
        assert(recovery.phase()==ReplayPhase::Sending && GCodeInput::dispatchDeferred());
        assert(!dispatchedCommands && GCodeInput::peekCurrentCommand()->N==1);
        assert(txDrain()==resend2);
        // No new blocking machine command may start while the policy is
        // waiting for an identifiable verified expected frame.
        for(unsigned n=0;n<8;++n)Commands::commandLoop();
        assert(recovery.phase()==ReplayPhase::Waiting && !dispatchedCommands);
        rxFeed(checkedPacket(2,false));Commands::commandLoop();
        assert(txDrain()=="ok 2\r\n" && dispatchedCommands==1);
        assert(GCodeInput::peekCurrentCommand()->N==2);
        Commands::commandLoop();
        assert(dispatchedCommands==2 && !GCodeInput::peekCurrentCommand());
    }
    assert(!transportStops && replies.empty());
}
void replayThermal() {
    startResend();thermalLocked=true;
    rxFeed(numbered("N2 M105")+numbered("N3 M115"));
    for(unsigned n=0;n<5;++n)Commands::commandLoop();
    assert(!liveRequests && !replayRequests && !transportStops);
    assertQuiet();
}
'''

MODES=("replay-ascii","replay-gap","replay-binary","replay-retries","replay-exhausted",
       "replay-unidentified","replay-alias","replay-partial","replay-ack","replay-ack-stall",
       "replay-marker-loss","replay-thermal","replay-dispatch-hold","replay-ack-hold")

DISPATCH=r'''    else if(scenario=="replay-ascii"||scenario=="replay-gap")replayAscii(scenario=="replay-gap");
    else if(scenario=="replay-binary")replayBinary();
    else if(scenario=="replay-retries"||scenario=="replay-exhausted")replayRetries(scenario=="replay-exhausted");
    else if(scenario=="replay-unidentified"||scenario=="replay-alias"||scenario=="replay-partial")replayUnidentified(scenario);
    else if(scenario=="replay-ack"||scenario=="replay-ack-stall")replayAck(scenario=="replay-ack-stall");
    else if(scenario=="replay-marker-loss")replayMarkerLoss();
    else if(scenario=="replay-thermal")replayThermal();
    else if(scenario=="replay-dispatch-hold"||scenario=="replay-ack-hold")replayDispatchHold(scenario=="replay-ack-hold");'''

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",type=Path,required=True)
    parser.add_argument("--out",type=Path,required=True)
    args=parser.parse_args()
    terminal["run"](args.source,args.out,{"SCENARIOS":SCENARIOS,"MODES":MODES,"DISPATCH":DISPATCH})

if __name__=="__main__":
    main()

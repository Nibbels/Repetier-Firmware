#!/usr/bin/env python3
"""Stage 2: bounded resend and ordered admission on the terminal candidate.

Only an established checked/numbered stream and an identifiable expected
damaged frame or checked forward gap may request replay. Physical loss is
terminal regardless of mode. No host-history oracle; success requires the
actual receiver's verified expected frame and queue publication. Trial
deadlines are NOT shipping timing bounds. Detached worktree only.
"""
from pathlib import Path
import argparse
import hashlib
import json
import runpy
import subprocess

HERE=Path(__file__).resolve().parent
drive=runpy.run_path(str(HERE/"current-drive-combined.py"))
ring=runpy.run_path(str(HERE/"resend-ring-preflight.py"))
ack=runpy.run_path(str(HERE/"tx-ack-replay.py"))
wire=runpy.run_path(str(HERE/"current-resend-envelope.py"))
once=ring["once"]

STATE=r'''
using ReplayMode=ResendEnvelope::Mode;
using ReplayPhase=ResendPolicy::Phase;
ResendPolicy recovery;
ReplayMode checkedStream=ReplayMode::None;
uint8_t binarySyncZeros=0;
PendingAck pendingAck;
AckPublicationDeadline ackDeadline;
TxRetirementDeadline txDeadline;

static bool receiveReady() {
    return !commandQueue.full() && !(frame.borrowedText && !commandQueue.empty());
}
static void discardUnadmittedFrame() {
    frame.length=frame.expectedLength=0;
    frame.comment=frame.borrowedText=false;
}
static void stageAck(uint32_t line) {
    if(!pendingAck.stage(line)){recovery.failTx();return;}
    ackDeadline.staged(HAL::timeInMilliseconds());
}
struct ResyncTxSerial {
    static bool tryWriteSpan(const char* bytes,uint8_t count) {
        return UsbSerial::tryWriteResendSpan(bytes,count,uint8_t(count-4));
    }
};
static bool requestReplay(uint32_t expected) {
    if(UsbSerial::transmitFree()<25 || UsbSerial::transportFailed())return false;
    // The policy only invokes this when receive storage is ready.
    // The old borrowed frame, if any, has already retired.
    discardUnadmittedFrame();
    binarySyncZeros=0;
    return publishResend<ResyncTxSerial>(expected);
}
'''

RECOVERY_ADMISSION=r'''
    if(recovery.phase()==ReplayPhase::Waiting) {
        // The raw envelope was captured before parseAscii/parseBinary could
        // mutate the borrowed text or narrow a field. Special commands cannot
        // bypass recovery, even when their checksums and sequence match.
        if(!evidence.mayAdmit(checkedStream,recovery.expected()) ||
           !command.hasN() || command.hasFormatError() ||
           serialReceive.currentLineNumber!=evidence.line ||
           Printer::printJob.commandsBlocked() ||
           (command.hasM() && (command.M==GCodeCommands::ResetLineNumber ||
                              command.M==GCodeCommands::EmergencyReset)))return;
        if(GCodeCommands::rejectLegacySimulation(&command)){
            recovery.fault(recovery.expected(),false,HAL::timeInMilliseconds());
            return;
        }
        // Single-byte sticky loss observation linearizes trust in the
        // completed frame. No ISR publishes commands or mutates the session.
        // A later loss belongs to the next input, not this accepted frame.
        if(transportFailed())return;
        serialReceive.lastLineNumber=serialReceive.currentLineNumber;
        commandQueue.publish();
        serialReceive.lastCommandWasBinary=frame.binary;
        serialReceive.waitingForResend=-1;
        recovery.committedExpected();
        stageAck(serialReceive.currentLineNumber);
        keepAlive(NotBusy);
        return;
    }
'''

PROCESS=r'''
void GCodeInput::processFrame(GCode& command) {
    const bool waiting=recovery.phase()==ReplayPhase::Waiting;
    const uint32_t oldLine=serialReceive.currentLineNumber;
    const uint8_t oldErrors=serialReceive.formatErrors;
    ResendEnvelope evidence=frame.binary ?
        ResendEnvelope::binary(frame.bytes,frame.expectedLength) :
        ResendEnvelope::ascii(frame.bytes,uint8_t(frame.length-1));
    bool hasChecksumMarker=false;
    if(!frame.binary)
        for(uint8_t n=0;n+1<frame.length;++n)
            if(frame.bytes[n]=='*'){hasChecksumMarker=true;break;}
    if(waiting && (evidence.mode!=checkedStream || !evidence.exact(recovery.expected()))) {
        discardUnadmittedFrame();
        if(checkedStream==ReplayMode::Binary)binarySyncZeros=0;
        return;
    }
    // A strict ASCII envelope with bad checksum must not reach the
    // permissive text parser, which can overwrite the '*' before checking.
    if(!Printer::printJob.emergencyLocked() &&
       evidence.mode==ReplayMode::Ascii && !evidence.verified) {
        requestResend(evidence.mayRequest(checkedStream,serialReceive.lastLineNumber+1));
        return;
    }
    const bool parsed=frame.binary ?
        parseBinary(command,frame.bytes,frame.expectedLength) :
        parseAscii(command,(char*)frame.bytes);
    evidence.binaryParsed(parsed);
    if(parsed) {
        if(!Printer::printJob.emergencyLocked() && !frame.binary &&
           command.hasN() && hasChecksumMarker && evidence.mode==ReplayMode::None)
            requestResend(false);
        else checkAndPushCommand(command,evidence);
    }else if(!waiting) {
        requestResend(evidence.mayRequest(checkedStream,serialReceive.lastLineNumber+1));
    }
    if(waiting && recovery.phase()!=ReplayPhase::Healthy) {
        // Failed candidates cannot alter committed sequence or retain a
        // text pointer. This is safe because the storage guard ran first.
        serialReceive.currentLineNumber=oldLine;
        serialReceive.formatErrors=oldErrors;
        discardUnadmittedFrame();
        if(checkedStream==ReplayMode::Binary)binarySyncZeros=0;
    }else if(!parsed)frame.borrowedText=false;
}

bool GCodeInput::transportFailed() {
    return UsbSerial::transportFailed() || recovery.phase()==ReplayPhase::Fatal;
}

void GCodeInput::serviceTransportTx() {
    if(transportFailed() || Printer::printJob.emergencyLocked())return;
    const uint32_t now=HAL::timeInMilliseconds();
    pendingAck.service();
    if(ackDeadline.expired(pendingAck.pending,now) ||
       txDeadline.poll(UsbSerial::transmitRetired(),UsbSerial::transmitFree()!=63,now))
        recovery.failTx();
}

void GCodeInput::readFromSerial() {
    if(transportFailed())return;
    if(Printer::printJob.emergencyLocked()){
        readFrame(); // unchanged thermal-only diagnostics if transport healthy
        return;
    }
    serviceTransportTx();
    if(transportFailed())return;
    if(!UsbSerial::resyncPending())
        recovery.observeHandoff(HAL::timeInMilliseconds(),receiveReady());
    if(!pendingAck.pending &&
       (recovery.phase()==ReplayPhase::Healthy || recovery.phase()==ReplayPhase::Waiting))
        readFrame();
    if(transportFailed() || Printer::printJob.emergencyLocked())return;
    serviceTransportTx();
    if(transportFailed())return;
    recovery.service(HAL::timeInMilliseconds(),receiveReady(),!pendingAck.pending,
        requestReplay,true);
    serviceTransportTx();
}
'''

def candidate(fw):
    s=(fw/"GCodeInput.cpp").read_text()
    h=(fw/"GCodeInput.h").read_text()
    c=(fw/"Commands.cpp").read_text()
    h=once(h,'#include "gcode.h"','#include "gcode.h"\nstruct ResendEnvelope;')
    h=once(h,"    static void readFromSerial();",
        "    static void readFromSerial();\n    static bool transportFailed();\n    static void serviceTransportTx();")
    h=once(h,"    static void requestResend();","    static void requestResend(bool replayable=false);\n    static void readFrame();\n    static void processFrame(GCode& command);")
    h=once(h,"    static void checkAndPushCommand(GCode& command);",
        "    static void checkAndPushCommand(GCode& command,const ResendEnvelope& evidence);")
    s=once(s,'#include "UsbSerial.h"','#include "UsbSerial.h"\n#include "ResendPolicy.h"\n#include "ResendEnvelope.h"')
    pending=ack["CANDIDATE"].split("void stagedAck()")[0]
    s=once(s,"} // namespace\n",pending+STATE+"\n} // namespace\n")
    # Keep physical loss terminal in every receiver entry, including
    # thermal promotion. The protocol-fatal latch joins the same guard.
    s=s.replace("UsbSerial::transportFailed()","GCodeInput::transportFailed()")
    # requestReplay is an internal helper, so use the combined predicate too.
    s=once(s,"void GCodeInput::requestResend() {","void GCodeInput::requestResend(bool replayable) {")
    i=s.index("    HAL::serialFlush();",s.index("void GCodeInput::requestResend("))
    j=s.index("} // requestResend",i)
    s=s[:i]+'''    if(recovery.phase()==ReplayPhase::Healthy)
        recovery.fault(serialReceive.lastLineNumber+1,replayable,HAL::timeInMilliseconds());
    // While waiting, malformed/stale packets do not reset the retry age.
    // A physical error is independently terminal through transportFailed.
    discardUnadmittedFrame();
    if(checkedStream==ReplayMode::Binary)binarySyncZeros=0;
'''+s[j:]
    s=once(s,"void GCodeInput::checkAndPushCommand(GCode& command) {",
        "void GCodeInput::checkAndPushCommand(GCode& command,const ResendEnvelope& evidence) {")
    i=s.index("void GCodeInput::checkAndPushCommand(")
    j=s.index("} // checkAndPushCommand",i)
    part=s[i:j]
    part=once(part,"#endif\n    if (command.hasM()) {","#endif\n"+RECOVERY_ADMISSION+"    if (command.hasM()) {")
    part=once(part,"            serialReceive.lastLineNumber = serialReceive.currentLineNumber;",
        "            checkedStream=ReplayMode::None;\n            serialReceive.lastLineNumber = serialReceive.currentLineNumber;")
    i2=part.index("    if (command.hasN()) {")
    j2=part.index("    keepAlive(NotBusy);",i2)
    part=part[:i2]+'''    if(command.hasN()) {
        const uint32_t expected=serialReceive.lastLineNumber+1;
        const bool fullAscii=evidence.mode==ReplayMode::Ascii && evidence.verified;
        const bool exact=fullAscii ? evidence.exact(expected) :
            uint16_t(expected)==uint16_t(serialReceive.currentLineNumber);
        if(!exact) {
            const bool duplicate=fullAscii ?
                serialReceive.currentLineNumber<=serialReceive.lastLineNumber &&
                serialReceive.lastLineNumber-serialReceive.currentLineNumber<40 :
                uint16_t(serialReceive.lastLineNumber-serialReceive.currentLineNumber)<40;
            if(duplicate) {
                frame.length=0;frame.borrowedText=false;
                Com::printFLN(Com::tSkip,serialReceive.currentLineNumber);
                Com::printFLN(Com::tOk);
            }else requestResend(evidence.mayRequest(checkedStream,expected));
            return;
        }
    }
    const bool admitted=!GCodeCommands::rejectLegacySimulation(&command) &&
        !Printer::printJob.commandsBlocked();
    if(transportFailed())return;
    if(command.hasN())serialReceive.lastLineNumber=serialReceive.currentLineNumber;
    if(admitted)commandQueue.publish();
    // Healthy blocked/rejected commands retain the previous host-draining
    // semantics but cannot establish a trusted checked stream.
    checkedStream=admitted && command.hasN() && evidence.verified &&
        serialReceive.currentLineNumber==evidence.line ? evidence.mode : ReplayMode::None;
    stageAck(serialReceive.currentLineNumber);
    serialReceive.lastCommandWasBinary=frame.binary;
'''+part[j2:]
    s=s[:i]+part+s[j:]
    s=once(s,"void GCodeInput::readFromSerial() {","void GCodeInput::readFrame() {")
    s=once(s,"    if (!HAL::serialByteAvailable()) {",
        "    if (!HAL::serialByteAvailable() && recovery.phase()==ReplayPhase::Healthy) {")
    s=once(s,"        if (frame.length == 1 && !frame.comment) {",
        """        if (frame.length == 1 && !frame.comment) {
            if(recovery.phase()==ReplayPhase::Waiting && checkedStream==ReplayMode::Binary && binarySyncZeros<31){
                if(frame.bytes[0])binarySyncZeros=0;else ++binarySyncZeros;
                frame.length=0;continue;
            }""")
    s=once(s,"            frame.binary = (frame.bytes[0] & 128) != 0;",
        """            frame.binary = !(recovery.phase()==ReplayPhase::Waiting &&
                checkedStream==ReplayMode::Ascii) && (frame.bytes[0] & 128) != 0;
            frame.expectedLength=0;""")
    s=once(s,"""                if (parseBinary(command, frame.bytes, frame.expectedLength))
                    checkAndPushCommand(command);
                else
                    requestResend();""","                processFrame(command);")
    s=once(s,"""                if (parseAscii(command, (char*)frame.bytes))
                    checkAndPushCommand(command);
                else
                    requestResend();""","                processFrame(command);")
    # Suppress parser diagnostic output on rejected recovery candidates;
    # normal/thermal diagnostic policies otherwise stay in their owners.
    i=s.index("bool GCodeInput::parseBinary(")
    s=s[:i]+s[i:].replace("if(!Printer::printJob.emergencyLocked())",
        "if(!Printer::printJob.emergencyLocked() && recovery.phase()==ReplayPhase::Healthy)")
    s=once(s,"} // readFromSerial\n","} // readFromSerial\n"+PROCESS+"\n")
    # Periodic service observes TX deadlines without recursive input parsing.
    c=once(c,"    if(!Printer::printJob.emergencyLocked() && UsbSerial::transportFailed())",
        "    GCodeInput::serviceTransportTx();\n    if(!Printer::printJob.emergencyLocked() && GCodeInput::transportFailed())")
    return {"GCodeInput.cpp":s,"GCodeInput.h":h,"Commands.cpp":c,
        "ResendPolicy.h":(HERE/"resend-policy.h").read_text(),
        "ResendEnvelope.h":wire["HEADER"].lstrip()}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",type=Path,required=True)
    parser.add_argument("--combined",type=Path,required=True)
    parser.add_argument("--stage1",type=Path,required=True)
    parser.add_argument("--out",type=Path,required=True)
    a=parser.parse_args()
    source=a.source.resolve();combined=a.combined.resolve()
    stage1=a.stage1.resolve();out=a.out.resolve()
    assert drive["git"](source,"rev-parse","HEAD")==drive["P0"]
    assert subprocess.run(["git","-C",str(source),"symbolic-ref","-q","HEAD"],
        capture_output=True).returncode==1
    assert not drive["git"](source,"status","--porcelain")
    assert out!=source and source not in out.parents
    manifest=json.loads((stage1/"provenance.json").read_text())
    assert manifest["p0"]==drive["P0"]
    assert manifest["combined_sha256"]==hashlib.sha256(combined.read_bytes()).hexdigest()
    subprocess.run(["git","-C",str(source),"apply","--check",str(combined)],check=True)
    subprocess.run(["git","-C",str(source),"apply",str(combined)],check=True)
    fw=source/"Repetier"
    for name,digest in manifest["candidate_sha256"].items():
        assert Path(name).name==name
        data=(stage1/name).read_bytes()
        assert hashlib.sha256(data).hexdigest()==digest,name
        (fw/name).write_bytes(data)
    files=candidate(fw)
    out.mkdir(parents=True,exist_ok=True)
    for name,data in files.items():
        (fw/name).write_text(data)
        (out/name).write_text(data)
    subprocess.run(["git","-C",str(source),"diff","--check"],check=True)
    (out/"provenance.json").write_text(json.dumps({
        "p0":drive["P0"],"combined_sha256":manifest["combined_sha256"],
        "stage1_manifest_sha256":hashlib.sha256((stage1/"provenance.json").read_bytes()).hexdigest(),
        "candidate_sha256":{n:hashlib.sha256(data.encode()).hexdigest() for n,data in files.items()},
        "scope":"Stage 2 disposable source only; actual parser/host replay, canonical regression and linked timing required"
    },indent=2)+"\n")

if __name__=="__main__":
    main()

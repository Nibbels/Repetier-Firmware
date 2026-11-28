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
        if(!Printer::printJob.emergencyLocked() &&
           ((evidence.verified && (!command.hasN() ||
             serialReceive.currentLineNumber!=evidence.line)) ||
            (!frame.binary && command.hasN() && hasChecksumMarker &&
             evidence.mode==ReplayMode::None)))
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
    // A thermal fault during an unfinished replay cannot turn stale
    // input into the otherwise permitted read-only thermal diagnostics.
    return UsbSerial::transportFailed() || recovery.phase()==ReplayPhase::Fatal ||
        (Printer::printJob.emergencyLocked() && recovery.phase()!=ReplayPhase::Healthy);
}

bool GCodeInput::dispatchDeferred() {
    // Recovery can only start after receiveReady() admitted a parse visit,
    // and no new slot is published until the verified expected frame
    // returns the phase to Healthy. Existing queued work need not start a
    // blocking command while the host is resending. The already planned
    // motion and thermal/periodic service continue independently.
    const auto phase=recovery.phase();
    return pendingAck.pending || (phase!=ReplayPhase::Healthy && phase!=ReplayPhase::Fatal);
}

void GCodeInput::serviceTransportTx() {
    if(transportFailed() || Printer::printJob.emergencyLocked())return;
    // The periodic scheduler calls this at several admission boundaries.
    // Avoid a timebase snapshot (and its IRQ mask) on the empty idle path.
    if(!pendingAck.pending && UsbSerial::transmitFree()==63){
        txDeadline.poll(0,false,0);
        return;
    }
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
    if(recovery.phase()==ReplayPhase::Sending && !UsbSerial::resyncPending())
        recovery.observeHandoff(HAL::timeInMilliseconds(),receiveReady());
    if(!pendingAck.pending &&
       (recovery.phase()==ReplayPhase::Healthy || recovery.phase()==ReplayPhase::Waiting))
        readFrame();
    if(transportFailed() || Printer::printJob.emergencyLocked())return;
    serviceTransportTx();
    if(transportFailed())return;
    if(recovery.phase()!=ReplayPhase::Healthy){
        recovery.service(HAL::timeInMilliseconds(),receiveReady(),!pendingAck.pending,
            requestReplay,true);
        serviceTransportTx();
    }
}
'''

def candidate(fw):
    s=(fw/"GCodeInput.cpp").read_text()
    h=(fw/"GCodeInput.h").read_text()
    c=(fw/"Commands.cpp").read_text()
    h=once(h,'#include "gcode.h"','#include "gcode.h"\nstruct ResendEnvelope;')
    h=once(h,"    static void readFromSerial();",
        "    static void readFromSerial();\n    static bool transportFailed();\n    static bool dispatchDeferred();\n    static void serviceTransportTx();")
    h=once(h,"    static void requestResend();","    static void requestResend(bool replayable=false);\n    static void readFrame();\n    static void processFrame(GCode& command);")
    h=once(h,"    static void checkAndPushCommand(GCode& command);",
        "    static void checkAndPushCommand(GCode& command,const ResendEnvelope& evidence);")
    s=once(s,'#include "UsbSerial.h"','#include "UsbSerial.h"\n#include "ResendPolicy.h"\n#include "ResendEnvelope.h"')
    pending=ack["CANDIDATE"].split("void stagedAck()")[0]
    # GCodeInput also has a later anonymous namespace for typed scripts.
    # Insert only in the first receiver-state owner.
    assert s.count("} // namespace\n")==2
    i=s.index("} // namespace\n")
    assert s.index("} serialReceive;")<i<s.index("uint8_t GCodeInput::computeBinarySize")
    s=s[:i]+pending+STATE+"\n"+s[i:]
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
    else frame.borrowedText=false;
    // Healthy blocked/rejected commands retain the previous host-draining
    // semantics but cannot establish a trusted checked stream.
    checkedStream=admitted && command.hasN() && evidence.verified &&
        serialReceive.currentLineNumber==evidence.line ? evidence.mode : ReplayMode::None;
    stageAck(serialReceive.currentLineNumber);
    serialReceive.lastCommandWasBinary=frame.binary;
'''+part[j2:]
    s=s[:i]+part+s[j:]
    s=once(s,"void GCodeInput::keepAlive(enum FirmwareState state) {",
        "void GCodeInput::keepAlive(enum FirmwareState state) {\n"
        "    if(pendingAck.pending || recovery.phase()!=ReplayPhase::Healthy)return;")
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
    c=once(c,"    GCode* code = GCodeInput::peekCurrentCommand();",
        """    // Do not start a foreground command that may run for seconds or
    // minutes before returning to the serial receive pump. A canceled or
    // terminal queue must still retire without executing.
    if(!Printer::printJob.commandsBlocked() && GCodeInput::dispatchDeferred()) {
        Commands::checkForPeriodicalActions(Processing);
        return;
    }
    GCode* code = GCodeInput::peekCurrentCommand();""")
    return {"GCodeInput.cpp":s,"GCodeInput.h":h,"Commands.cpp":c,
        "ResendPolicy.h":(HERE/"resend-policy.h").read_text(),
        "ResendEnvelope.h":wire["HEADER"].lstrip()}

def adapt_canonical(source):
    """Adapt only the affected actual-source fixtures and IDE declarations.

    Do not restore legacy heater-comparison selectors: the canonical P0
    owner removed that probe in 0914b879. Its historical evidence survives
    in the old commits. The standard recovery builder still verifies
    the integrated lease/watchdog paths on each of the four boards.
    """
    fw=source/"Repetier"
    for filename in ("Repetier.vcxproj","Repetier.vcxproj.filters"):
        p=fw/filename;s=p.read_text()
        if filename.endswith(".filters"):
            anchor='    <ClInclude Include="PrintJob.h">\n      <Filter>Header Files</Filter>\n    </ClInclude>'
            new=anchor+"".join('\n    <ClInclude Include="'+name+'">\n      <Filter>Header Files</Filter>\n    </ClInclude>'
                for name in ("ResendPolicy.h","ResendEnvelope.h"))
        else:
            anchor='    <ClInclude Include="PrintJob.h" />'
            new=anchor+"\n    <ClInclude Include=\"ResendPolicy.h\" />\n    <ClInclude Include=\"ResendEnvelope.h\" />"
        p.write_text(once(s,anchor,new))
    p=source/"tests/temperature-control-replay.py";s=p.read_text()
    s=once(s,'    #include "printer-safety.inc"','    #include "printer-safety.inc"\n'
        '    void clearHeaterTargets();\n    void revokeHeaterOutputsLocked();')
    s=once(s,"        bool emergencyLocked() const { return isAnyTempsensorDefect(); }",
        "        bool emergencyLocked() const { return isAnyTempsensorDefect(); }\n"
        "        bool transportEmergency() const { return false; } // thermal-only fixture")
    # The actual heating-inhibit helper now consults printJob. Define
    # the fixture owner first, forward-declaring only its sensor query.
    s=once(s,'    #include "printer-safety.inc"\n'
        '    void clearHeaterTargets();\n    void revokeHeaterOutputsLocked();',
        '    static bool isAnyTempsensorDefect();')
    s=once(s,'    } printJob;\n    static unsigned flag2, debugLevel;',
        '    } printJob;\n    #include "printer-safety.inc"\n'
        '    void clearHeaterTargets();\n    void revokeHeaterOutputsLocked();\n'
        '    static void stopForTransportFault(){ assert(false); }\n'
        '    static unsigned flag2, debugLevel;')
    # Compile the actual new transport-service helper alongside the
    # actual scheduler. Only the transport input is healthy/stubbed in
    # this temperature-only harness; real fault injection is separate.
    s=once(s,'namespace GCodeInput {\n    bool scriptActive = false;',
        'namespace GCodeInput {\n    void serviceTransportTx() {}\n'
        '    bool transportFailed() { return false; }\n    bool scriptActive = false;')
    s=once(s,'"scheduler.inc": section(commands, "#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\\nstatic void serviceThermalEmergency()",',
        '"scheduler.inc": section(commands, "#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\\nstatic bool serviceTransportFailure()", "void Commands::commandLoop()")\n'
        '            + section(commands, "#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\\nstatic void serviceThermalEmergency()",')
    p.write_text(s)
    p=source/"tests/task0034-terminal-stop.py";s=p.read_text()
    # Extract the common motion cleanup and both actual terminal owners.
    # Preserve the original thermal cases; add separate transport and
    # promotion controls, without duplicating implementation in stubs.
    s=once(s,'src.index("void Printer::stopForThermalFault()")',
        'src.index("void Printer::stopTerminalMotionLocked()")')
    s=once(s,'body=src[','body="#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\\n"+src[')
    s=once(s,'static bool irq, entryI, heatersOff, motorsEnabled;',
        'static bool irq, entryI, heatersOff, motorsEnabled, targetsCleared;\n'
        'static unsigned revokes, clears;')
    s=once(s,'    void stopForThermalFault();',
        '''    void stopTerminalMotionLocked();
    void stopForThermalFault();
    void stopForTransportFault();
    void revokeHeaterOutputsLocked(){
        assert(!irq && printJob.transportEmergency() && !heatersOff && sequence==0);
        heatersOff=true;++revokes;
    }
    void clearHeaterTargets(){
        assert(irq==entryI && heatersOff && printJob.transportEmergency() && sequence==6);
        targetsCleared=true;++clears;
    }''')
    s=once(s,'    sequence=7;', '    if(Printer::printJob.transportEmergency())assert(targetsCleared);\n    sequence=7;')
    s=once(s,'        Printer::stopForThermalFault();\n        assert(sequence==7 && irq==enabled && motorsEnabled);\n    }\n}',
        '''        Printer::stopForThermalFault();
        assert(sequence==7 && irq==enabled && motorsEnabled);
        assert(Printer::printJob.thermalEmergency());
        Printer::stopForTransportFault();
        assert(sequence==7 && irq==enabled && !Printer::printJob.transportEmergency());

        Printer::printJob=PrintJob{};
        if(phase==1||phase==3)Printer::printJob.admitPrintMotion();
        if(phase==2)Printer::printJob.beginFilamentService();
        if(phase==3)assert(Printer::printJob.beginStop(1));
        Printer::blockAll=Printer::movementLocked=false;
        irq=entryI=enabled;heatersOff=targetsCleared=false;motorsEnabled=true;
        sequence=revokes=clears=0;
        Printer::stopForTransportFault();
        assert(sequence==7 && irq==enabled && motorsEnabled && heatersOff && targetsCleared);
        assert(revokes==1 && clears==1);
        assert(Printer::blockAll && Printer::movementLocked);
        assert(Printer::printJob.emergencyLocked() && Printer::printJob.commandsBlocked());
        assert(Printer::printJob.transportEmergency());
        Printer::stopForTransportFault();
        assert(sequence==7 && revokes==1 && clears==1 && irq==enabled);
        Printer::stopForThermalFault();
        assert(Printer::printJob.thermalEmergency() && !Printer::printJob.transportEmergency());
        assert(sequence==7 && revokes==1 && clears==1 && irq==enabled && motorsEnabled);
    }
}''')
    s=once(s,'sequence=0;\n        Printer::stopForThermalFault();',
        'sequence=revokes=clears=0;\n        Printer::stopForThermalFault();')
    s=once(s,'Task0034 terminal motion stop: four phases, both entry I states, repeat call PASS',
        'Task0034 common terminal motion stop: four phases, both I states, transport, thermal and promotion PASS')
    p.write_text(s)
    # P0 already retired the obsolete nonstandard builds; don't re-enable
    # them to make this disposable standard-recovery candidate compile.
    old=source/"tests/heater-lease-integrated-avr.py"
    assert "Build historical comparison candidates" in old.read_text()
    old.unlink()

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
    adapt_canonical(source)
    subprocess.run(["git","-C",str(source),"diff","--check"],check=True)
    (out/"provenance.json").write_text(json.dumps({
        "p0":drive["P0"],"combined_sha256":manifest["combined_sha256"],
        "stage1_manifest_sha256":hashlib.sha256((stage1/"provenance.json").read_bytes()).hexdigest(),
        "candidate_sha256":{n:hashlib.sha256(data.encode()).hexdigest() for n,data in files.items()},
        "scope":"Stage 2 disposable source only; actual parser/host replay, canonical regression and linked timing required"
    },indent=2)+"\n")

if __name__=="__main__":
    main()

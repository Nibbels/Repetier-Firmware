#!/usr/bin/env python3
"""Actual P0 parser replay of a strict recovery-only admission envelope.

The boundary/loss witness is deliberately injected, as in Task0033's
recovery-contract.js. This checks real parser mutation and special-command
ordering, not physical RX synchronization, host software, or terminal stop.
Do not apply these temporary test hooks to production.
"""
import argparse
from pathlib import Path
import shutil
import subprocess
import sys
from tempfile import TemporaryDirectory
import runpy

ROOT = Path(__file__).resolve().parents[2]
prior = runpy.run_path(str(ROOT/"tools/task0032/resend-ring-preflight.py"))
P0 = prior["P0"]
once = prior["once"]

HELPERS = r'''
// Test seam for the still-open synchronization/admission proof. Use the
// same executable retry policy as the actual-ring preflight.
ResendPolicy researchRecovery;
bool researchBoundary = false;
bool researchLoseAtCommit = false;

static bool recoverySpace(char c) { return c==' ' || c=='\t'; }
static bool recoveryDigit(char c) { return c>='0' && c<='9'; }

// Validate the wire envelope before parseAscii can overwrite the '*' in
// M117 text. Not a second G-code command grammar. In recovery do NOT use
// the legacy uint8_t checksum narrowing or 16-bit ASCII line aliasing.
static bool recoveryAsciiEnvelope(const char* line, uint32_t expected) {
    const char* p=line;
    while(recoverySpace(*p))++p;
    if(*p!='N' && *p!='n')return false;
    ++p;
    while(recoverySpace(*p))++p;
    if(!recoveryDigit(*p))return false;
    uint32_t n=0;
    do {
        const uint8_t d=uint8_t(*p++-'0');
        if(n>(uint32_t(INT32_MAX)-d)/10)return false;
        n=n*10+d;
    } while(recoveryDigit(*p));
    if(n!=expected)return false;
    if(!recoverySpace(*p) && *p!='G' && *p!='g' && *p!='M' && *p!='m'
        && *p!='T' && *p!='t' && *p!='F' && *p!='f' && *p!='*')return false;
    uint8_t checksum=0;
    const char* star=line;
    while(*star && *star!='*')checksum^=uint8_t(*star++);
    if(*star!='*')return false;
    p=star+1;
    while(recoverySpace(*p))++p;
    if(!recoveryDigit(*p))return false;
    uint16_t given=0;
    do {
        given=uint16_t(given*10+(*p++-'0'));
        if(given>255)return false;
    } while(recoveryDigit(*p));
    while(recoverySpace(*p))++p;
    return !*p && given==checksum;
}

static bool recoveryBinaryEnvelope(const uint8_t* bytes,uint8_t length) {
    if(length<6)return false;
    const uint16_t flags=uint16_t(bytes[0])|(uint16_t(bytes[1])<<8);
    if((flags&0x81)!=0x81 || (flags&0x6000))return false;
    if(flags&0x1000) {
        if(length<8)return false;
        const uint16_t ext=uint16_t(bytes[2])|(uint16_t(bytes[3])<<8);
        if(ext&0xf800)return false; // reuse #116 supported extension mask
        if((flags&0x8000) && bytes[4]>=maxBinaryTextSize)return false;
    }
    return true; // actual parseBinary performs Fletcher verification
}
static bool recoveredParsed(GCode& command) {
    // Model a loss between successful parse and final admission.
    if(researchLoseAtCommit) {
        researchLoseAtCommit=false;
        researchBoundary=false;
    }
    const uint32_t expected=researchRecovery.expected();
    return researchBoundary && !Printer::printJob.commandsBlocked()
        && command.hasN() && !command.hasFormatError()
        && serialReceive.currentLineNumber==(frame.binary?uint16_t(expected):expected)
        && !(command.hasM() && (command.M==GCodeCommands::ResetLineNumber
                             || command.M==GCodeCommands::EmergencyReset));
}
'''

def recovery_branch(binary):
    envelope=("recoveryBinaryEnvelope(frame.bytes,frame.expectedLength)" if binary
              else "recoveryAsciiEnvelope((char*)frame.bytes,researchRecovery.expected())")
    parse=("parseBinary(command,frame.bytes,frame.expectedLength)" if binary
           else "parseAscii(command,(char*)frame.bytes,true)")
    return '''
                if(researchRecovery.phase()==ResendPolicy::Phase::Waiting) {
                    const uint32_t oldLine=serialReceive.currentLineNumber;
                    const uint8_t oldErrors=serialReceive.formatErrors;
                    if(researchBoundary && %s && %s && recoveredParsed(command)) {
                        checkAndPushCommand(command);
                        researchRecovery.committedExpected();
                    } else {
                        // This visit cannot have an admitted borrowed text:
                        // the real readFromSerial backpressure guard precedes it.
                        serialReceive.currentLineNumber=oldLine;
                        serialReceive.formatErrors=oldErrors;
                        frame.borrowedText=false;
                    }
                    frame.length=0;
                    return;
                }
'''%(envelope,parse)

SCENARIOS = r'''
void beginRecovery() {
    researchRecovery.fault(serialReceive.lastLineNumber+1,true,clockMs);
    researchRecovery.service(clockMs,true,true,[](uint32_t){return true;});
    assert(researchRecovery.phase()==ResendPolicy::Phase::Waiting);
    researchBoundary=true; // external boundary oracle, not inferred from zeros
}
void rejectRecovery(const std::string& data) {
    assert(!GCodeInput::peekCurrentCommand());
    const uint32_t last=serialReceive.lastLineNumber;
    const uint32_t current=serialReceive.currentLineNumber;
    const auto errors=serialReceive.formatErrors;
    replies.clear();
    feed(data);
    GCodeInput::readFromSerial();
    assert(!GCodeInput::peekCurrentCommand() && !frame.borrowedText);
    assert(researchRecovery.phase()==ResendPolicy::Phase::Waiting);
    assert(serialReceive.lastLineNumber==last && serialReceive.currentLineNumber==current);
    assert(serialReceive.formatErrors==errors);
    assert(replies.find("ok")==std::string::npos && !stopped && !dispatchedCommands);
    replies.clear();
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    txFree=63;
#endif
}
void recoveryAscii() {
    feed(numbered("N1 G1 X1"));readReply("ok 1\r\n");
    GCodeInput::popCurrentCommand();beginRecovery();
    rejectRecovery(numbered("N3 G1 X3"));
    rejectRecovery(numbered("N1 G1 X1"));
    rejectRecovery(numbered("N65538 G1 X9"));
    rejectRecovery(numbered("N2 N65538 G1 X9"));
    rejectRecovery("N2 G1 X2\n");
    rejectRecovery(numbered("G1 X9"));
    rejectRecovery(numbered("N2 M110"));
    rejectRecovery(numbered("N2 M112"));
    rejectRecovery("N2 M117 bad text*0\n");
    const auto valid=numbered("N2 G1 X2");
    const auto star=valid.find('*');
    rejectRecovery(valid.substr(0,star+1)+std::to_string(std::stoi(valid.substr(star+1))+256)+"\n");
    rejectRecovery(valid.substr(0,valid.size()-1)+" M112\n");
    researchLoseAtCommit=true;
    rejectRecovery(numbered("N2 M117 stale text"));
    rejectRecovery(valid); // old proof no longer valid
    researchBoundary=true;
    feed(numbered("N2 M117 retained message")+numbered("N3 G1 X3"));
    readReply("ok 2\r\n");
    assert(researchRecovery.phase()==ResendPolicy::Phase::Healthy);
    auto* msg=GCodeInput::peekCurrentCommand();
    assert(msg && msg->hasM() && msg->M==117 && msg->text==std::string("retained message"));
    GCodeInput::readFromSerial();
    assert(msg->text==std::string("retained message") && replies.empty());
    GCodeInput::popCurrentCommand();
    readReply("ok 3\r\n");GCodeInput::popCurrentCommand();
    feed(numbered("N2 M117 retained message"));
    readReply("skip 2\r\nok\r\n");
    assert(!GCodeInput::peekCurrentCommand());
    std::puts("Recovery ASCII: strict envelope, stale/future/alias/reset/emergency rejection, borrowed text, commit/duplicate passed");
}
void recoveryBinary() {
    feed(binary({0x85,0,1,0,1}));readReply("ok 1\r\n");
    GCodeInput::popCurrentCommand();beginRecovery();
    rejectRecovery(binary({0x85,0,3,0,1}));
    rejectRecovery(binary({0x84,0,1})); // unnumbered
    rejectRecovery(binary({0x83,0,2,0,110}));
    rejectRecovery(binary({0x83,0,2,0,112}));
    rejectRecovery(binary({0x85,0x10,0x00,0x08,2,0,1,0,0,0,0,0}));
    auto corrupt=binary({0x85,0,2,0,1});corrupt.back()^=1;
    rejectRecovery(corrupt);
    researchBoundary=false;rejectRecovery(binary({0x85,0,2,0,1}));
    researchBoundary=true;researchLoseAtCommit=true;
    rejectRecovery(binary({0x85,0,2,0,1}));
    researchBoundary=true;
    // V2 expected packet and V1 next packet may coexist in one stream.
    feed(binary({0x85,0x10,0,0,2,0,1,0})+binary({0x85,0,3,0,1}));
    readReply("ok 2\r\n");
    assert(researchRecovery.phase()==ResendPolicy::Phase::Healthy);
    assert(GCodeInput::peekCurrentCommand()->isV2());
    GCodeInput::popCurrentCommand();
    readReply("ok 3\r\n");GCodeInput::popCurrentCommand();
    std::puts("Recovery binary: V1/V2, unsupported flags, Fletcher, wrong line, fresh proof and in-flight tail passed");
}
void recoveryWrap(bool binaryMode) {
    // Preserve the actual firmware's full decimal Resend:65536 while
    // accepting the binary 16-bit N=0; ASCII keeps the full number.
    feed(numbered("N65535 M110"));readReply("ok\r\n");beginRecovery();
    assert(researchRecovery.expected()==65536);
    if(binaryMode) {
        feed(binary({0x85,0,0,0,1}));readReply("ok 0\r\n");
        assert(serialReceive.lastLineNumber==0);
    } else {
        rejectRecovery(numbered("N0 G1 X1"));
        feed(numbered("N65536 G1 X1"));readReply("ok 65536\r\n");
        assert(serialReceive.lastLineNumber==65536);
    }
    assert(GCodeInput::peekCurrentCommand() && researchRecovery.phase()==ResendPolicy::Phase::Healthy);
    GCodeInput::popCurrentCommand();
}
'''

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",type=Path,required=True)
    args=parser.parse_args()
    assert subprocess.check_output(["git","-C",str(args.source),"rev-parse","HEAD"],text=True).strip()==P0
    with TemporaryDirectory(prefix="task0032-resend-frames-") as d:
        root=Path(d);fw=root/"Repetier";tests=root/"tests"
        shutil.copytree(args.source/"Repetier",fw);tests.mkdir()
        shutil.copyfile(ROOT/"tools/task0032/resend-policy.h",fw/"resend-policy.h")
        for name in ("serial-protocol.cpp","serial-protocol.py","usb-serial.cpp"):
            shutil.copyfile(args.source/"tests"/name,tests/name)
        path=fw/"GCodeInput.cpp";s=path.read_text()
        s=once(s,'#include "UsbSerial.h"','#include "UsbSerial.h"\n#include "resend-policy.h"')
        s=once(s,"} // namespace\n",HELPERS+"\n} // namespace\n")
        for binary,anchor in [(True,"                if (parseBinary(command, frame.bytes, frame.expectedLength))"),
                             (False,"                if (parseAscii(command, (char*)frame.bytes, true))")]:
            s=once(s,anchor,recovery_branch(binary)+anchor)
        path.write_text(s)
        path=tests/"serial-protocol.cpp";s=path.read_text()
        s=once(s,"void numberedLines() {",SCENARIOS+"\nvoid numberedLines() {")
        s=once(s,'if (scenario == "numbered") numberedLines();',
            'if (scenario == "recovery-ascii") recoveryAscii();\n'
            '    else if (scenario == "recovery-binary") recoveryBinary();\n'
            '    else if (scenario == "recovery-wrap-ascii") recoveryWrap(false);\n'
            '    else if (scenario == "recovery-wrap-binary") recoveryWrap(true);\n'
            '    else if (scenario == "numbered") numberedLines();')
        path.write_text(s)
        path=tests/"serial-protocol.py";s=path.read_text()
        new='"recovery-ascii", "recovery-binary", "recovery-wrap-ascii", "recovery-wrap-binary", '
        s=once(s,'for scenario in ("numbered",','for scenario in ('+new+'"numbered",')
        s=once(s,'for scenario in ("thermal-emergency",','for scenario in ('+new+'"thermal-emergency",')
        path.write_text(s)
        subprocess.run([sys.executable,str(path)],check=True)
if __name__=="__main__":
    main()

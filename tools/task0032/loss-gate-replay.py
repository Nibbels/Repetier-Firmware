#!/usr/bin/env python3
"""Actual-source sticky RX quarantine candidate, using Task0033 replay helpers.

No automatic recovery: terminal until reset. The stop hook is a TEST seam,
not a production stop implementation. Do not apply this generator to firmware.
No electrical FIFO or simultaneous interrupt simulation.
"""
from pathlib import Path
from tempfile import TemporaryDirectory
import argparse
import runpy
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]

BASE_RX = """    const uint8_t byte = UDR0;
    const uint8_t next = rx_.next(rx_.head);
    if (next != rx_.tail) {
        rx_.bytes[rx_.head] = byte;
        rx_.head = next;
    }"""
GATED_RX = """    const uint8_t status = UCSR0A;
    const uint8_t byte = UDR0;
    const uint8_t error = status & (_BV(FE0) | _BV(DOR0) | _BV(UPE0));
    if (error) {
        lossFlags_ |= error;
        return;
    }
    if (lossFlags_) return;
    const uint8_t head = rx_.head;
    const uint8_t next = rx_.next(head);
    if (next == rx_.tail) {
        lossFlags_ |= 1;
        return;
    }
    rx_.bytes[head] = byte;
    asm volatile("" ::: "memory");
    rx_.head = next;"""

STOP = """// Test-only adapter; production terminal stop must be coordinated with P0.
static bool serviceRxLoss() {
    if (!UsbSerial::lossFlags()) return false;
    static bool notified = false;
    if (!notified) {
        notified = true;
        researchStopOnRxLoss();
    }
    return true;
}
"""

SCENARIOS = r'''
void cleanUart(const std::string& data) {
    uartFeed(data);
    uartDrain();
}
void requireQuarantine(uint8_t flags) {
    GCodeInput::readFromSerial();
    assert(researchLossStops == 1 && stopped);
    assert((UsbSerial::lossFlags() & flags) == flags);
    assert(!GCodeInput::peekCurrentCommand());
    assert(replies.empty());
    // Zeros, valid checksums, M110 and M112 do not reopen this gate.
    uartFeed(std::string(64, '\0'));
    uartFeed(numbered("N1 M110"));
    uartFeed("M112\n");
    uartFeed(numbered("N1 G1 X99"));
    uartDrain();
    for (int i=0;i<5;++i) Commands::commandLoop();
    assert(researchLossStops == 1 && dispatchedCommands == 0 &&
           heaterRequests == 0 && extrusionRequests == 0 && replies.empty());
}
void gatedLoss(const std::string& scenario) {
    if (scenario == "gate-ring") {
        uartFeed(std::string(100,'q'));
        for (int i=0;i<100;++i) assert(UsbSerial::read()=='q');
        uartFeed(";" + std::string(119,'a') + "\nG1 X12");
        uartByte('3');
        uartDrain();
        requireQuarantine(1);
    } else {
        cleanUart("G1 X");
        GCodeInput::readFromSerial(); // real partial frame
        const uint8_t flag = scenario=="gate-fe" ? _BV(FE0) :
            scenario=="gate-dor" ? _BV(DOR0) : _BV(UPE0);
        uartByte('9',flag); uartByte('\n'); uartDrain();
        requireQuarantine(flag);
    }
}
void gateBoundary(const std::string& scenario) {
    const bool after = scenario=="gate-after";
    const bool reset = scenario=="gate-m110";
    const bool emergency = scenario=="gate-m112";
    cleanUart(numbered(reset?"N1 M110":emergency?"N1 M112":"N1 G1 X9"));
    injectGateBefore=!after;
    injectGateAfter=after;
    GCodeInput::readFromSerial();
    if(after) {
        // Frame validated before loss: preserve its ordered ACK and exactly
        // one admission. Do not falsely claim it was a damaged command.
        assert(replies=="ok 1\r\n");
        assert(GCodeInput::peekCurrentCommand() && serialReceive.lastLineNumber==1);
        GCodeInput::popCurrentCommand(); replies.clear();
    } else {
        assert(serialReceive.lastLineNumber==0);
        assert(!GCodeInput::peekCurrentCommand() && replies.empty());
    }
    requireQuarantine(_BV(DOR0));
}
void gateBorrowedText() {
    cleanUart("M117 retained message\n");
    GCodeInput::readFromSerial();
    auto* msg=GCodeInput::peekCurrentCommand();
    assert(msg && msg->text==std::string("retained message"));
    replies.clear();
    uartByte('x',_BV(FE0)); uartFeed("G1 X1\n"); uartDrain();
    GCodeInput::readFromSerial();
    assert(researchLossStops==1 && msg->text==std::string("retained message"));
    GCodeInput::popCurrentCommand();
    requireQuarantine(_BV(FE0));
}
void gateFletcher() {
    // Reproduce #103's flagged Fletcher collision. Preserve the ordinary
    // checksum limitation while rejecting a byte known bad by the UART.
    auto packet=binary({0x82,0x04,104,0,0,0,0});
    packet[3]=static_cast<char>(255);
    for(size_t i=0;i<packet.size();++i)
        uartByte(static_cast<uint8_t>(packet[i]),i==3?_BV(FE0):0);
    uartByte(0,_BV(DOR0)); uartByte(0,_BV(UPE0));
    uartDrain();
    requireQuarantine(_BV(FE0)|_BV(DOR0)|_BV(UPE0));
}
void gateUnflaggedCollision() {
    auto packet=numbered("N1 G1 X111");
    packet.erase(packet.find("111"),2);
    cleanUart(packet); GCodeInput::readFromSerial();
    assert(!UsbSerial::lossFlags());
    assertX(1.0f); // documented protocol limitation, not successful detection
}
'''


def once(s, old, new):
    assert s.count(old)==1, old[:100]
    return s.replace(old,new)


def patch(fw, combined=False):
    header=fw/"UsbSerial.h"
    header.write_text(once(header.read_text(),"    static bool available();",
        "    static uint8_t lossFlags() { return lossFlags_; }\n    static bool available();")
        .replace("    static Ring<receiveBufferSize> rx_;","    static volatile uint8_t lossFlags_;\n    static Ring<receiveBufferSize> rx_;"))
    uart=fw/"UsbSerial.cpp"
    gate=GATED_RX
    if combined:
        gate=once(gate,"    if (error) {\n        lossFlags_ |= error;\n        return;\n    }\n    if (lossFlags_) return;",
            "    const uint8_t loss = lossFlags_ | error;\n    if (loss) {\n        lossFlags_ = loss;\n        return;\n    }")
    uart.write_text(once(once(uart.read_text(),BASE_RX,gate),
        "void UsbSerial::begin(unsigned long baud) {",
        "volatile uint8_t UsbSerial::lossFlags_ = 0;\n\nvoid UsbSerial::begin(unsigned long baud) {"))
    parser=fw/"GCodeInput.cpp"
    s=parser.read_text()
    s=once(s,"} // namespace\n",STOP+"\n} // namespace\n")
    # Single-byte sticky gate read linearizes trust of this complete frame.
    # No ISR can mutate session/queue; no reopening until reset. Parsing may
    # stage currentLineNumber/formatErrors/text but does not publish work.
    s=once(s,"void GCodeInput::checkAndPushCommand(GCode& command) {",
        "void GCodeInput::checkAndPushCommand(GCode& command) {\n"
        "    researchBeforeCommit();\n"
        "    if (serviceRxLoss()) return;\n"
        "    researchAfterCommit();")
    s=once(s,"void GCodeInput::readFromSerial() {",
        "void GCodeInput::readFromSerial() {\n    if (serviceRxLoss()) return;")
    s=once(s,"void GCodeInput::requestResend() {",
        "void GCodeInput::requestResend() {\n    if (serviceRxLoss()) return;")
    parser.write_text(s)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--prior-replay",required=True,type=Path)
    p.add_argument("--combined",action="store_true")
    args=p.parse_args()
    prior=runpy.run_path(str(args.prior_replay))
    # Reuse actual-ring injection helpers from the immutable Task0033 replay.
    helpers=prior["SCENARIOS"].split("void lossRing() {")[0]
    with TemporaryDirectory(prefix="task0032-rx-gate-") as directory:
        root=Path(directory);fw=root/"Repetier";tests=root/"tests"
        shutil.copytree(ROOT/"Repetier",fw);tests.mkdir()
        patch(fw,args.combined)
        for name in ("serial-protocol.py","serial-protocol.cpp","usb-serial.cpp"):
            shutil.copyfile(ROOT/"tests"/name,tests/name)
        cpp=tests/"serial-protocol.cpp";s=cpp.read_text()
        # Declarations must precede the extracted firmware body.
        s=once(s,'#include "gcode-receiver.inc"',
            'static unsigned researchLossStops=0;\n'
            'static bool injectGateBefore=false,injectGateAfter=false;\n'
            'void researchStopOnRxLoss(){++researchLossStops;stopped=true;}\n'
            'void researchBeforeCommit(); void researchAfterCommit();\n'
            '#include "gcode-receiver.inc"\n'+prior["REGISTERS"]+
            '\nvoid researchBeforeCommit(){if(injectGateBefore){UCSR0A=_BV(DOR0);UDR0=0;USART0_RX_vect();}}\n'
            'void researchAfterCommit(){if(injectGateAfter){UCSR0A=_BV(DOR0);UDR0=0;USART0_RX_vect();}}\n')
        s=once(s,"void numberedLines() {",helpers+SCENARIOS+"\nvoid numberedLines() {")
        s=once(s,'if (scenario == "numbered") numberedLines();',
            'if (scenario=="gate-ring" || scenario=="gate-fe" || scenario=="gate-dor" || scenario=="gate-upe") gatedLoss(scenario);\n'
            '    else if (scenario=="gate-before" || scenario=="gate-after" || scenario=="gate-m110" || scenario=="gate-m112") gateBoundary(scenario);\n'
            '    else if (scenario=="gate-text") gateBorrowedText();\n'
            '    else if (scenario=="gate-fletcher") gateFletcher();\n    else if (scenario=="gate-collision") gateUnflaggedCollision();\n'
            '    else if (scenario == "numbered") numberedLines();')
        cpp.write_text(s)
        py=tests/"serial-protocol.py"
        py.write_text(once(py.read_text(),'for scenario in ("numbered",',
            'for scenario in ("gate-ring","gate-fe","gate-dor","gate-upe","gate-before","gate-after","gate-m110","gate-m112","gate-text","gate-fletcher","gate-collision","numbered",'))
        # Ordinary ring test intentionally overflowed and expected continued
        # operation; that expectation is superseded ONLY for the candidate.
        uart=tests/"usb-serial.cpp";s=uart.read_text()
        s=once(s,"enum { U2X0 = 1,","enum { FE0=4, DOR0=3, UPE0=2, U2X0 = 1,")
        s=once(s,"byte <= UsbSerial::receiveCapacity","byte < UsbSerial::receiveCapacity")
        uart.write_text(s)
        subprocess.run([sys.executable,str(py)],check=True)
        print(("Combined" if args.combined else "Sticky")+" RX quarantine: eleven actual-source loss/boundary fixtures passed (including documented unflagged collision)")


if __name__=="__main__":
    main()

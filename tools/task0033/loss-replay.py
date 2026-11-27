#!/usr/bin/env python3
"""Replay baseline loss failures using actual UsbSerial and GCodeInput.
Creates a temporary copy of the existing source-extraction harness.
These are characterization assertions, NOT acceptance tests for a fix.
Run: python3 tools/task0033/loss-replay.py
No hardware. No source or shared test changes.
"""
from pathlib import Path
from tempfile import TemporaryDirectory
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]

REGISTERS = r'''
#define F_CPU 16000000UL
#define _BV(bit) (1U << (bit))
#define ISR(vector) void vector()
enum { U2X0 = 1, UPE0 = 2, DOR0 = 3, FE0 = 4,
       TXEN0 = 3, RXEN0 = 4, UDRIE0 = 5, RXCIE0 = 7 };
volatile uint8_t UCSR0A, UCSR0B, UBRR0H, UBRR0L, UDR0;
#include "usb-serial-firmware.inc"
'''

SCENARIOS = r'''
// Fake USART registers model flags presented to the real ISR, not hardware
// FIFO behavior or flag read side effects. Feed the actual ring output to
// the existing parser harness without introducing extra byte loss.
void uartByte(uint8_t b, uint8_t status = 0) {
    UCSR0A = status;
    UDR0 = b;
    USART0_RX_vect();
}
void uartFeed(const std::string& data) {
    for (unsigned char b : data) uartByte(b);
}
void uartDrain() {
    while (UsbSerial::available()) {
        const int b = UsbSerial::read();
        assert(b >= 0);
        incoming.push_back(static_cast<uint8_t>(b));
    }
}
void assertX(float x) {
    auto* cmd = GCodeInput::peekCurrentCommand();
    assert(cmd && cmd->hasG() && cmd->G == 1 && cmd->hasX() && cmd->X == x);
    assert(replies.find("ok ") != std::string::npos);
    GCodeInput::popCurrentCommand();
}
void lossRing() {
    // The 128th byte is discarded; foreground drains before the terminator.
    // 121 comment bytes + 6 command bytes = 127 accepted bytes.
    const std::string prefix = ";" + std::string(119, 'a') + "\nG1 X12";
    assert(prefix.size() == UsbSerial::receiveCapacity);
    // Exercise wrap first without leaving residue.
    uartFeed(std::string(100, 'q'));
    for (int i = 0; i < 100; ++i) assert(UsbSerial::read() == 'q');
    uartFeed(prefix);
    uartByte('3');
    uartDrain();
    assert(incoming.size() == UsbSerial::receiveCapacity);
    GCodeInput::readFromSerial();
    assert(!GCodeInput::peekCurrentCommand());
    uartByte('\n');
    uartDrain();
    GCodeInput::readFromSerial();
    assertX(12.0f); // Intended X123, silently accepted X12.
}
void lossFraming() {
    uartFeed("G1 X");
    uartByte('9', _BV(FE0));
    uartByte('\n');
    uartDrain();
    GCodeInput::readFromSerial();
    assertX(9.0f); // Framing-error byte is admitted.
}
void lossParity() {
    uartFeed("G1 X");
    uartByte('9', _BV(UPE0));
    uartByte('\n');
    uartDrain();
    GCodeInput::readFromSerial();
    assertX(9.0f); // Synthetic flag; baseline uses 8N1.
}
void lossDor() {
    // Intended X123; only X13 arrives, DOR flagged with following byte.
    uartFeed("G1 X1");
    uartByte('3', _BV(DOR0));
    uartByte('\n');
    uartDrain();
    GCodeInput::readFromSerial();
    assertX(13.0f);
}
void lossXor() {
    auto packet = numbered("N1 G1 X111");
    packet.erase(packet.find("111"), 2); // XOR of two equal bytes is zero.
    uartFeed(packet);
    uartDrain();
    GCodeInput::readFromSerial();
    assertX(1.0f); // Correct line number and checksum despite deletion.
}
void lossFletcher() {
    auto packet = binary({0x82, 0x04, 104, 0, 0, 0, 0});
    packet[3] = static_cast<char>(255); // Fletcher arithmetic modulo 255.
    for (size_t i = 0; i < packet.size(); ++i)
        uartByte(static_cast<uint8_t>(packet[i]), i == 3 ? _BV(FE0) : 0);
    uartDrain();
    GCodeInput::readFromSerial();
    auto* cmd = GCodeInput::peekCurrentCommand();
    assert(cmd && cmd->hasM() && cmd->M == 104 && cmd->hasS() && cmd->S == 255);
    assert(replies.find("ok ") != std::string::npos);
}
'''
def replace_once(text, old, new):
    assert text.count(old) == 1, old
    return text.replace(old, new)

def main():
    with TemporaryDirectory(prefix="task0033-loss-") as directory:
        root = Path(directory)
        (root / "Repetier").symlink_to(ROOT / "Repetier", target_is_directory=True)
        tests = root / "tests"
        tests.mkdir()
        for name in ("serial-protocol.py", "serial-protocol.cpp", "usb-serial.cpp"):
            shutil.copyfile(ROOT / "tests" / name, tests / name)
        cpp = tests / "serial-protocol.cpp"
        text = cpp.read_text()
        text = replace_once(text, '#include "gcode-receiver.inc"', '#include "gcode-receiver.inc"\n' + REGISTERS)
        text = replace_once(text, "void numberedLines() {", SCENARIOS + "\nvoid numberedLines() {")
        text = replace_once(text, 'if (scenario == "numbered") numberedLines();',
            'if (scenario == "loss-ring") lossRing();\n'
            '    else if (scenario == "loss-fe") lossFraming();\n'
            '    else if (scenario == "loss-upe") lossParity();\n'
            '    else if (scenario == "loss-dor") lossDor();\n'
            '    else if (scenario == "loss-xor") lossXor();\n'
            '    else if (scenario == "loss-fletcher") lossFletcher();\n'
            '    else if (scenario == "numbered") numberedLines();')
        cpp.write_text(text)
        py = tests / "serial-protocol.py"
        text = replace_once(py.read_text(), 'for scenario in ("numbered",',
            'for scenario in ("loss-ring", "loss-fe", "loss-upe", "loss-dor", "loss-xor", "loss-fletcher", "numbered",')
        py.write_text(text)
        subprocess.run([sys.executable, str(py)], check=True)

if __name__ == "__main__":
    main()

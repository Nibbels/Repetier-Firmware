#!/usr/bin/env python3
"""Actual-source parser replay using existing host harness in a temporary tree.

Constructed equivalent ASCII and binary G1 streams, not observed Host/Server
traffic or AVR timing. Preserve shared harness and production files.
"""
from pathlib import Path
from tempfile import TemporaryDirectory
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]

SCENARIOS = r'''
void appendU16(std::string& packet, uint16_t value) {
    packet += static_cast<char>(value & 255);
    packet += static_cast<char>(value >> 8);
}
void appendFloat(std::string& packet, float value) {
    static_assert(sizeof(float) == 4, "binary wire uses float32");
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    for (unsigned shift = 0; shift < 32; shift += 8)
        packet += static_cast<char>((bits >> shift) & 255);
}
void wireMoves(const std::string& scenario) {
    const bool ascii = scenario == "wire-ascii";
    const bool v2 = scenario == "wire-binary-v2";
    const unsigned start = 12345, count = 600;
    unsigned wireBytes = 0, ackBytes = 0, minBytes = 1000, maxBytes = 0;
    unsigned fragmented = 0;
    feed("N12344 M110\n");
    readReply("ok\r\n");
    for (unsigned i = 0; i < count; ++i) {
        const unsigned n = start + i;
        char xText[24], yText[24], eText[24];
        std::snprintf(xText, sizeof(xText), "%.4f", 100.0 + .1 * (i + 1));
        std::snprintf(yText, sizeof(yText), "%.4f", 95.456);
        std::snprintf(eText, sizeof(eText), "%.5f", .004 * (i + 1));
        const float x = std::strtof(xText, nullptr), y = std::strtof(yText, nullptr);
        const float e = std::strtof(eText, nullptr);
        const bool hasF = i == 0;
        std::string frameBytes;
        if (ascii) {
            frameBytes = numbered("N" + std::to_string(n) + " G1 X" + xText + " Y" + yText + " E" + eText + (hasF ? " F3600" : ""));
            frameBytes.insert(frameBytes.size() - 1, 1, '\r');
        } else {
            const uint16_t flags = 0x00dd | (hasF ? 256 : 0) | (v2 ? 4096 : 0);
            appendU16(frameBytes, flags);
            if (v2) appendU16(frameBytes, 0);
            appendU16(frameBytes, static_cast<uint16_t>(n));
            frameBytes += '\1';
            if (v2) frameBytes += '\0';
            appendFloat(frameBytes, x);
            appendFloat(frameBytes, y);
            appendFloat(frameBytes, e);
            if (hasF) appendFloat(frameBytes, 3600.0f);
            frameBytes = withBinaryChecksum(frameBytes);
            assert(frameBytes.size() == (v2 ? 22u : 19u) + (hasF ? 4u : 0u));
        }
        wireBytes += frameBytes.size();
        minBytes = std::min(minBytes, static_cast<unsigned>(frameBytes.size()));
        maxBytes = std::max(maxBytes, static_cast<unsigned>(frameBytes.size()));
        if (i % 3 == 0) {
            const auto half = frameBytes.size() / 2;
            feed(frameBytes.substr(0, half));
            readReply("");
            assert(!GCodeInput::peekCurrentCommand() && dispatchedCommands == i);
            feed(frameBytes.substr(half));
            ++fragmented;
        } else {
            feed(frameBytes);
        }
        GCodeInput::readFromSerial();
        assert(replies == "ok " + std::to_string(n) + "\r\n");
        ackBytes += replies.size();
        replies.clear();
        GCode* cmd = GCodeInput::peekCurrentCommand();
        assert(cmd && cmd->hasN() && cmd->N == n && cmd->hasG() && cmd->G == 1);
        assert(cmd->hasX() && cmd->hasY() && cmd->hasE());
        assert(cmd->X == x && cmd->Y == y && cmd->E == e);
        assert(cmd->hasF() == hasF);
        if (hasF) assert(cmd->F == 3600.0f);
        // ACK already emitted before normal dispatch or retirement.
        assert(dispatchedCommands == i);
        Commands::executeGCode(cmd);
        GCodeInput::popCurrentCommand();
        assert(dispatchedCommands == i + 1);
    }
    readReply(""); // consume last CRLF's LF if present
    assert(incoming.empty() && !GCodeInput::peekCurrentCommand());
    assert(flushes == 0 && ackBytes == 10 * count);
    std::printf("TASK0032_WIRE {\"scenario\":\"%s\",\"moves\":%u,\"rxBytes\":%u,\"ackBytes\":%u,\"minFrame\":%u,\"maxFrame\":%u,\"fragmented\":%u}\n",
                scenario.c_str(), count, wireBytes, ackBytes, minBytes, maxBytes, fragmented);
}
'''

def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError("source anchor changed: " + old)
    return text.replace(old, new)


def main():
    with TemporaryDirectory(prefix="task0032-wire-") as directory:
        root = Path(directory)
        (root / "Repetier").symlink_to(ROOT / "Repetier", target_is_directory=True)
        tests = root / "tests"
        tests.mkdir()
        for name in ("serial-protocol.py", "serial-protocol.cpp", "usb-serial.cpp"):
            shutil.copyfile(ROOT / "tests" / name, tests / name)
        cpp = tests / "serial-protocol.cpp"
        text = replace_once(cpp.read_text(), "#include <cstdlib>", "#include <cstdlib>\n#include <cstring>")
        text = replace_once(text, "void numberedLines() {", SCENARIOS + "\nvoid numberedLines() {")
        text = replace_once(text, 'if (scenario == "numbered") numberedLines();',
            'if (scenario == "wire-ascii" || scenario == "wire-binary-v1" || scenario == "wire-binary-v2") wireMoves(scenario);\n'
            '    else if (scenario == "numbered") numberedLines();')
        cpp.write_text(text)
        py = tests / "serial-protocol.py"
        py.write_text(replace_once(py.read_text(), 'for scenario in ("numbered",',
            'for scenario in ("wire-ascii", "wire-binary-v1", "wire-binary-v2", "numbered",'))
        subprocess.run([sys.executable, str(py)], check=True)


if __name__ == "__main__":
    main()

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <initializer_list>
#include <string>

using millis_t = uint32_t;
using fast8_t = int8_t;
using PGM_P = const char*;
#define FSTRINGPARAM(name) const char* name
#define PSTR(text) text
#include "gcode-settings.inc"
#include "GCodeInput.h"

namespace {
std::deque<uint8_t> incoming;
std::string replies;
millis_t clockMs = 0;
bool echo = false;
bool stopped = false;
unsigned flushes = 0;
unsigned scriptCommands = 0;
GCode lastScriptCommand{};
}

namespace HAL {
millis_t timeInMilliseconds() { return clockMs; }
bool serialByteAvailable() { return !incoming.empty(); }
uint8_t serialReadByte() {
    const auto byte = incoming.front();
    incoming.pop_front();
    return byte;
}
void serialFlush() { ++flushes; }
char readFlashByte(const char* position) { return *position; }
}

namespace Printer {
bool debugErrors() { return false; }
bool debugEcho() { return echo; }
struct {
    bool commandsBlocked() const { return stopped; }
} printJob;
struct { bool isPaused() const { return false; } } pause;
}

namespace Commands {
void commandLoop();
void emergencyStop() { std::abort(); }
void executeGCode(GCode* code) {
    assert(code->internalCommand);
    ++scriptCommands;
    lastScriptCommand = *code;
}
void checkForPeriodicalActions(FirmwareState) {}
}

namespace RMath {
int min(int a, int b) { return std::min(a, b); }
}

namespace Com {
#include "gcode-replies.inc"
void print(char value) { replies += value; }
void print(const char* text) { replies += text; }
void print(char* text) { print(static_cast<const char*>(text)); }
template <class Number>
void print(Number value) { replies += std::to_string(value); }
void println() { replies += "\r\n"; }
void printF(const char* text) { print(text); }
template <class Number>
void printF(const char* prefix, Number value) { print(prefix); print(value); }
void printF(const char* prefix, float value, int decimals) {
    char field[64];
    std::snprintf(field, sizeof(field), "%.*f", decimals, static_cast<double>(value));
    printF(prefix, field);
}
void printFLN(const char* text) { print(text); println(); }
template <class Number>
void printFLN(const char* prefix, Number value) { printF(prefix, value); println(); }
void printErrorFLN(const char* text) { print(tError); printFLN(text); }
}

#include "gcode-receiver.inc"

namespace {

void feed(const std::string& bytes) {
    incoming.insert(incoming.end(), bytes.begin(), bytes.end());
}

std::string numbered(const std::string& line) {
    uint8_t checksum = 0;
    for (unsigned char byte : line)
        checksum ^= byte;
    return line + "*" + std::to_string(checksum) + "\n";
}

std::string binary(std::initializer_list<uint8_t> payload) {
    std::string bytes;
    unsigned first = 0, second = 0;
    for (uint8_t byte : payload) {
        bytes += static_cast<char>(byte);
        first = (first + byte) % 255;
        second = (second + first) % 255;
    }
    bytes += static_cast<char>(first);
    bytes += static_cast<char>(second);
    return bytes;
}

void readReply(const std::string& expected) {
    GCodeInput::readFromSerial();
    if (replies != expected)
        std::fprintf(stderr, "Expected [%s], received [%s]\n", expected.c_str(), replies.c_str());
    assert(replies == expected);
    replies.clear();
}

void popM(unsigned number) {
    GCode* code = GCodeInput::peekCurrentCommand();
    assert(code && code->hasM() && code->M == number);
    GCodeInput::popCurrentCommand();
}

void numberedLines() {
    feed("N40 M110\n");
    readReply("ok\r\n");
    assert(!GCodeInput::peekCurrentCommand());

    const std::string move = numbered("N41 G1 X1.25 E0.5");
    feed(move.substr(0, 7));
    readReply("");
    assert(!GCodeInput::peekCurrentCommand());
    feed(move.substr(7));
    readReply("ok 41\r\n");
    GCode* code = GCodeInput::peekCurrentCommand();
    assert(code && code->N == 41 && code->G == 1 && code->X == 1.25f && code->E == 0.5f);
    GCodeInput::popCurrentCommand();

    feed(move);
    readReply("skip 41\r\nok\r\n");
    assert(!GCodeInput::peekCurrentCommand());
    feed(numbered("N43 M105"));
    readReply("\r\nResend:42\r\nok\r\n");
    assert(flushes == 1);
    feed(numbered("N42 M105"));
    readReply("ok 42\r\n");
    popM(105);

    feed("N43 M105*0\n");
    readReply("Error:Wrong checksum\r\nCorrupted: N43 M105*0\r\n\r\nResend:43\r\nok\r\n");
    assert(!GCodeInput::peekCurrentCommand());
}

void queueBackpressure() {
    feed("M105\r\nM114\nM115\n");
    readReply("ok 0\r\n");
    readReply("ok 0\r\n");
    const auto waiting = incoming.size();
    assert(waiting > 0);
    clockMs = 2000;
    readReply("busy: processing\r\n");
    assert(incoming.size() == waiting);
    popM(105);
    readReply("ok 0\r\n");
    popM(114);
    popM(115);
    assert(!GCodeInput::peekCurrentCommand());
}

void displayText() {
    feed("M117 Cooling down\nM115\n");
    readReply("ok 0\r\n");
    readReply("");
    const auto* display = GCodeInput::peekCurrentCommand();
    assert(display && display->text == std::string("Cooling down"));
    popM(117);
    readReply("ok 0\r\n");
    echo = true;
    popM(115);
    assert(replies == "Echo:M115 \r\n");
}

void idleAndPartialLine() {
    clockMs = 1000;
    readReply("");
    clockMs = 1001;
    readReply("wait\r\n");
    feed("G1 X");
    readReply("");
    clockMs += 200;
    readReply("");
    ++clockMs;
    readReply("\r\nResend:1\r\nok\r\n");
    assert(!GCodeInput::peekCurrentCommand());
    assert(flushes == 1);
}

void binaryFrames() {
    feed(binary({0x85, 0x00, 0x01, 0x00, 0x01}));
    readReply("ok 1\r\n");
    GCode* command = GCodeInput::peekCurrentCommand();
    assert(command && command->N == 1 && command->hasG() && command->G == 1 && !command->isV2());
    GCodeInput::popCurrentCommand();

    auto packet = binary({0x83, 0x10, 0x00, 0x00, 0x02, 0x00, 0x2F, 0x0C});
    auto corrupted = packet;
    corrupted.back() ^= 1;
    feed(corrupted);
    readReply("\r\nResend:2\r\nok\r\n");
    feed(std::string(31, '\0') + packet);
    readReply("ok 2\r\n");
    assert(GCodeInput::peekCurrentCommand()->isV2());
    popM(3119);

    feed(numbered("N3 M105"));
    readReply("ok 3\r\n");
    popM(105);
}

void stopDrainsInput() {
    feed("N1 M104 S200\n");
    readReply("ok 1\r\n");
    assert(GCodeInput::peekCurrentCommand());
    stopped = true;
    Commands::commandLoop();
    assert(!GCodeInput::peekCurrentCommand() && scriptCommands == 0);
    feed("N2 M105\n");
    readReply("ok 2\r\n");
    assert(!GCodeInput::peekCurrentCommand());
    stopped = false;
    feed("N3 M105\n");
    readReply("ok 3\r\n");
    popM(105);
}

void flashScript() {
    GCodeInput::executeFlashScript("; comment\nG91\nG1 X0.5 ; jog\n");
    assert(scriptCommands == 2 && lastScriptCommand.G == 1 && lastScriptCommand.X == 0.5f);
    assert(replies.empty() && !GCodeInput::peekCurrentCommand());
    feed("N1 M105\n");
    readReply("ok 1\r\n");
    popM(105);
}

} // namespace

int main(int argc, char** argv) {
    assert(argc == 2);
    const std::string scenario = argv[1];
    if (scenario == "numbered") numberedLines();
    else if (scenario == "queue") queueBackpressure();
    else if (scenario == "display") displayText();
    else if (scenario == "idle") idleAndPartialLine();
    else if (scenario == "binary") binaryFrames();
    else if (scenario == "stopping") stopDrainsInput();
    else if (scenario == "script") flashScript();
    else return 1;
    std::printf("Serial protocol replay passed (%s)\n", argv[1]);
}

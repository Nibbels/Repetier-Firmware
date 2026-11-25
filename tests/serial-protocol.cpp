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
unsigned dispatchedCommands = 0, statusCommands = 0, stopRequests = 0;
bool legacyRejected = false;
unsigned heaterRequests = 0, extrusionRequests = 0;
unsigned periodicVisits = 0;
struct EmergencyReached {};
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
static uint8_t debugLevel;
#include "debug-mask.inc"
bool debugErrors() { return (debugLevel & 4) != 0; }
bool legacySimulationRejected() { return legacyRejected; }
void latchOutputInhibit(bool thermalFault) { assert(!thermalFault); legacyRejected = true; }
void stopPrint() { ++stopRequests; }
void setFeedrate(float) { ++dispatchedCommands; }
bool debugEcho() { return echo; }
struct {
    bool commandsBlocked() const { return stopped; }
} printJob;
struct { bool isPaused() const { return false; } } pause;
}

namespace Commands {
void commandLoop();
void emergencyStop() { throw EmergencyReached{}; }
void executeGCode(GCode* code);
void checkForPeriodicalActions(FirmwareState) { ++periodicVisits; }
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

// Real receiver, M111 handler and dispatch entry; downstream command effects
// are represented by counters, not physical hardware.
namespace GCodeCommands {
void setDebugLevel(GCode*);
void dispatchMotion(GCode* code) {
    ++dispatchedCommands;
    if (code->internalCommand) { ++scriptCommands; lastScriptCommand = *code; }
    if (code->hasE()) ++extrusionRequests;
}
void dispatchMachine(GCode* code) {
    if (code->M == 111) { setDebugLevel(code); return; }
    if (code->M == 105 || code->M == 114 || code->M == 115) { ++statusCommands; return; }
    ++dispatchedCommands;
    if (code->M == 104 || code->M == 109 || code->M == 140 || code->M == 190 || code->M == 303)
        ++heaterRequests;
    if (code->M == 223 || code->M == 3912 || code->M == 3913 || code->M == 3914)
        ++extrusionRequests;
}
}
namespace Extruder { void selectExtruderById(uint8_t, bool) { ++dispatchedCommands; } }
#include "legacy-dispatch.inc"
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

std::string withBinaryChecksum(std::string bytes) {
    unsigned first = 0, second = 0;
    for (unsigned char byte : bytes) {
        first = (first + byte) % 255;
        second = (second + first) % 255;
    }
    bytes += static_cast<char>(first);
    bytes += static_cast<char>(second);
    return bytes;
}

std::string binary(std::initializer_list<uint8_t> payload) {
    return withBinaryChecksum(std::string(payload.begin(), payload.end()));
}

std::string panelPacket(uint8_t declaredLength, unsigned transmittedLength) {
    std::string payload("\x82\x90\x00\x00", 4); // Binary V2, M and text.
    payload += static_cast<char>(declaredLength);
    payload += '\x75'; // M117, little-endian.
    payload += '\0';
    payload.append(transmittedLength, 'Q');
    return withBinaryChecksum(payload);
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

void overflowAndRecovery() {
    feed(std::string(96, 'X'));
    readReply("\r\nResend:1\r\nok\r\n");
    assert(flushes == 1 && !GCodeInput::peekCurrentCommand());
    feed(numbered("N1 M105"));
    readReply("ok 1\r\n");
    popM(105);

    // A terminator in byte 96 is still a complete ASCII frame.
    feed("M105" + std::string(91, ' ') + "\n");
    readReply("ok 1\r\n");
    popM(105);
}

void binaryText() {
    const auto packet = panelPacket(79, 79);
    const auto next = binary({0x82, 0x00, 105});
    feed(packet.substr(0, 5));
    readReply("");
    feed(packet.substr(5) + next);
    readReply("ok 0\r\n");
    const auto waiting = incoming.size();
    readReply("");
    assert(incoming.size() == waiting);
    const auto* display = GCodeInput::peekCurrentCommand();
    assert(display && display->text == std::string(79, 'Q'));
    popM(117);
    readReply("ok 0\r\n");
    popM(105);

    // A clipped packet with a correct Fletcher checksum must not make the
    // declared length escape the received payload or overwrite receiver state.
    for (uint8_t badLength : {80, 255}) {
        feed(panelPacket(badLength, 79));
        readReply("\r\nResend:1\r\nok\r\n");
        assert(!GCodeInput::peekCurrentCommand());
        feed(std::string(31, '\0') + next);
        readReply("ok 0\r\n");
        popM(105);
    }
    assert(flushes == 2);

    // A binary text frame without any text is also valid.
    feed(panelPacket(0, 0));
    readReply("ok 0\r\n");
    assert(GCodeInput::peekCurrentCommand()->text == std::string());
    popM(117);
}

// Long discarded input must yield to foreground services even though it
// never fills the command frame. Preserve comment and binary resync state
// across the visit boundary; no command is acknowledged prematurely.
void boundedReceive() {
    feed("M105;" + std::string(400, 'x') + "\nM114\n");
    for (unsigned i = 0; i < 3; ++i) {
        const auto before = incoming.size();
        Commands::commandLoop();
        assert(before - incoming.size() == 127);
        assert(periodicVisits == i + 1 && replies.empty());
        assert(!GCodeInput::peekCurrentCommand());
    }
    Commands::commandLoop();
    assert(periodicVisits == 4 && replies == "ok 0\r\n" && statusCommands == 1);
    replies.clear();
    Commands::commandLoop();
    assert(periodicVisits == 5 && replies == "ok 0\r\n" && statusCommands == 2);
    assert(incoming.empty());
    replies.clear();

    auto packet = binary({0x82, 0x00, 105});
    auto bad = packet;
    bad.back() ^= 1;
    feed(bad);
    readReply("\r\nResend:1\r\nok\r\n");
    feed(std::string(300, 'x') + std::string(31, '\0') + packet);
    for (unsigned i = 0; i < 2; ++i) {
        const auto before = incoming.size();
        readReply("");
        assert(before - incoming.size() == 127);
        assert(!GCodeInput::peekCurrentCommand());
    }
    readReply("ok 0\r\n");
    popM(105);
    assert(incoming.empty());
}

void sustainedAdmission() {
    feed("N65534 M110\n");
    readReply("ok\r\n");
    // Exercise line-number wrap, slot wrap and backpressure, without making
    // host wall-clock timings into claims about the AVR's throughput.
    for (unsigned line = 65535; line < 65535 + 256; line += 2) {
        feed(numbered("N" + std::to_string(line) + " M105") +
             numbered("N" + std::to_string(line + 1) + " M114"));
        readReply("ok " + std::to_string(line) + "\r\n");
        readReply("ok " + std::to_string(line + 1) + "\r\n");
        readReply("");
        assert(GCodeInput::peekCurrentCommand()->N == static_cast<uint16_t>(line));
        popM(105);
        assert(GCodeInput::peekCurrentCommand()->N == static_cast<uint16_t>(line + 1));
        popM(114);
    }
    assert(incoming.empty() && !GCodeInput::peekCurrentCommand() && flushes == 0);
}

void emergencyAdmission() {
    stopped = true;
    // M112 is handled before line-number validation and command blocking.
    feed("N99 M112\n");
    try {
        GCodeInput::readFromSerial();
        assert(false && "Emergency stop must not return to admission");
    } catch (const EmergencyReached&) {
        assert(replies.empty() && !GCodeInput::peekCurrentCommand() && flushes == 0);
    }
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

void legacyStream(const std::string& scenario) {
    // Positive control: normal commands reach their owner, normal masks work.
    feed("M111 S7\nM104 S100\nM140 S40\nG1 E1\n");
    while (!incoming.empty()) Commands::commandLoop();
    assert(!legacyRejected && Printer::debugMask() == 7 && heaterRequests == 2 && extrusionRequests == 1);
    Printer::toggleDebugFlag(8); // retired bit cannot be set via the debug API
    assert(Printer::debugMask() == 7);
    Printer::setDebugMask(0);
    replies.clear();
    const unsigned normalDispatches = dispatchedCommands;
    if (scenario == "legacy-during-stop") stopped = true;
    const unsigned legacyMask = scenario == "legacy-ascii" ? 8 : 15;
    if (scenario == "legacy-binary") {
        // Binary V1 M111 S15 (M and 32-bit S).
        feed(binary({0x82, 0x04, 111, 15, 0, 0, 0}));
    } else {
        feed("N1 M111 S8*0\n");
        Commands::commandLoop();
        assert(!legacyRejected && stopRequests == 0);
        replies.clear();
        feed(numbered("N1 M111 S" + std::to_string(legacyMask)));
    }
    Commands::commandLoop();
    assert(legacyRejected && stopRequests == 1 && Printer::debugMask() == (legacyMask & ~8u));
    assert(replies.find("M111 bit 8 unsupported") != std::string::npos);
    assert(replies.find("ok ") != std::string::npos);
    assert(!GCodeInput::peekCurrentCommand());
    stopped = false; // stop cleanup finished; legacy rejection must survive it
    // A legacy host clears debug, resets line numbers and blindly keeps streaming.
    replies.clear();
    if (scenario == "legacy-binary") {
        feed(binary({0x82, 0x04, 104, 200, 0, 0, 0}));
        Commands::commandLoop();
        assert(heaterRequests == 2 && dispatchedCommands == normalDispatches);
    }
    // Switching from binary to ASCII requires a checksummed frame.
    feed(numbered("M111 S0"));
    feed("M110\nM104 S200\nM109 S200\nM140 S60\nM190 S60\nM116\n"
         "G28\nG90\nM83\nG1 X1 E5\nG1 E2\nM303 S150\nM223 S10\n"
         "M3912\nM3913\nM3914\nT0\nF1200\nM3029 S1077\nM105\nM114\nM115\n");
    unsigned loops = 0;
    while (!incoming.empty() || GCodeInput::peekCurrentCommand()) {
        assert(++loops < 100);
        Commands::commandLoop();
    }
    assert(legacyRejected && Printer::debugMask() == 0 && stopRequests == 1);
    assert(dispatchedCommands == normalDispatches && heaterRequests == 2 && extrusionRequests == 1);
    assert(statusCommands == 3 && replies.find("Job rejected;") != std::string::npos);
    GCodeInput::executeFlashScript("G1 E1\nM104 S200\n");
    assert(dispatchedCommands == normalDispatches && heaterRequests == 2 && extrusionRequests == 1);
    feed("M111 S8\n");
    Commands::commandLoop();
    assert(legacyRejected && stopRequests == 1);
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
    else if (scenario == "overflow") overflowAndRecovery();
    else if (scenario == "binary-text") binaryText();
    else if (scenario == "sustained") sustainedAdmission();
    else if (scenario == "bounded-receive") boundedReceive();
    else if (scenario == "emergency") emergencyAdmission();
    else if (scenario == "stopping") stopDrainsInput();
    else if (scenario == "script") flashScript();
    else if (scenario == "legacy-ascii" || scenario == "legacy-binary" || scenario == "legacy-during-stop") legacyStream(scenario);
    else return 1;
    std::printf("Serial protocol replay passed (%s)\n", argv[1]);
}

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <initializer_list>
#include <string>
#include <vector>

using millis_t = uint32_t;
using fast8_t = int8_t;
using PGM_P = const char*;
#define FSTRINGPARAM(name) const char* name
#define PSTR(text) text
#include "gcode-settings.inc"
#include "GCodeInput.h"
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
#include "UsbSerial.h"
#endif

namespace {
std::deque<uint8_t> incoming;
std::string replies;
millis_t clockMs = 0;
bool stopped = false;
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
bool thermalLocked=false;
uint8_t txFree=63;
unsigned liveRequests=0,replayRequests=0;
#endif
unsigned flushes = 0;
unsigned scriptCommands = 0;
unsigned dispatchedCommands = 0, statusCommands = 0, stopRequests = 0;
bool legacyRejected = false, settingsBlocked = false;
unsigned softLoads = 0, softUnloads = 0;
std::vector<int> internalEvents;
unsigned stopAtPeriodicVisit=0;
bool rejectInternalMotion=false;
int16_t softPressure = 0;
uint8_t softSpeed = 0;
unsigned heaterRequests = 0, extrusionRequests = 0;
unsigned periodicVisits = 0, pendingOutputVisits = 0;
bool injectNestedScript = false, stopDuringScript = false;
struct EmergencyReached {};
GCode lastScriptCommand{};
}

#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
uint8_t UsbSerial::transmitFree(){return txFree;}
bool UsbSerial::tryWriteSpan(const char* bytes,uint8_t count) {
    assert(count<=txFree);
    replies.append(bytes,count);
    txFree-=count;
    return true;
}
namespace ThermalEmergency {
void requestLiveReport(){++liveRequests;}
void requestDiagnosticReplay(){++replayRequests;}
}
#endif
namespace HAL {
millis_t timeInMilliseconds() { return clockMs; }
bool serialByteAvailable() { return !incoming.empty(); }
uint8_t serialReadByte() {
    const auto byte = incoming.front();
    incoming.pop_front();
    return byte;
}
void serialFlush() {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    assert(!thermalLocked);
#endif
    ++flushes;
}
char readFlashByte(const char* position) { return *position; }
}

enum { X_AXIS, Y_AXIS, Z_AXIS, E_AXIS };
static bool isMovingAllowed(const char*) { return !rejectInternalMotion; }
namespace Printer {
struct { void recordActivity(millis_t){internalEvents.push_back(1);} } inactivity;
void setEPositionMM(float e){assert(e==0);internalEvents.push_back(2);}
bool queueInternalAxisMove(uint8_t axis,float value,float feedrate){
    if(axis==E_AXIS)
        internalEvents.push_back(int(value));
    else {
        assert(axis==Y_AXIS || axis==Z_AXIS);
        internalEvents.push_back(1000+axis);
        internalEvents.push_back(int(value));
    }
    internalEvents.push_back(int(feedrate));
    return true;
}
bool debugErrors() { return true; }
bool legacySimulationRejected() { return legacyRejected; }
bool heatingInhibited() { return settingsBlocked || legacyRejected
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    || thermalLocked
#endif
    ; }
void latchOutputInhibit(bool thermalFault) { assert(!thermalFault); legacyRejected = true; }
void stopPrint() { ++stopRequests; }
void setFeedrate(float) { ++dispatchedCommands; }
struct {
    void servicePendingOutput(millis_t) {
        assert(!GCodeInput::peekCurrentCommand());
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        assert(!thermalLocked);
#endif
        ++pendingOutputVisits;
    }
    bool commandsBlocked() const { return stopped
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        ||thermalLocked
#endif
        ; }
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    bool emergencyLocked() const {return thermalLocked;}
#endif
} printJob;
struct { bool isPaused() const { return false; } } pause;
}

namespace Commands {
void commandLoop();
void emergencyStop() { throw EmergencyReached{}; }
void executeGCode(GCode* code);
void checkForPeriodicalActions(FirmwareState) {
    ++periodicVisits;
    if (GCodeInput::flashScriptActive()) internalEvents.push_back(3);
    if (stopAtPeriodicVisit && periodicVisits==stopAtPeriodicVisit) stopped=true;
    if (injectNestedScript) {
        injectNestedScript = false;
        GCodeInput::executeSoftFilament(false);
        assert(GCodeInput::flashScriptActive());
        GCodeInput::executeFlashScript("G1 X99\n");
    }
    if (stopDuringScript) {
        stopDuringScript = false;
        stopped = true;
    }
}
}

namespace RMath {
int min(int a, int b) { return std::min(a, b); }
}

namespace Com {
#include "gcode-replies.inc"
void print(char value) {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    assert(!thermalLocked);
#endif
    replies += value;
}
void print(const char* text) {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    assert(!thermalLocked);
#endif
    replies += text;
}
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
void dwellMilliseconds(uint32_t duration){assert(GCodeInput::flashScriptActive());internalEvents.push_back(int(duration));}
void selectAbsolutePositioningWithSource(bool internal){assert(internal && GCodeInput::flashScriptActive());internalEvents.push_back(90);}
void waitForHotendWithParameters(bool targetPresent,int32_t target,bool toolPresent,uint8_t tool){
    assert(targetPresent && !toolPresent && !tool && GCodeInput::flashScriptActive());
    internalEvents.push_back(target);
    GCodeInput::executeHardFilament(true); // refuse nested program
}
void setHotendTemperatureWithParameters(bool targetPresent,int32_t target,bool waitRequested,bool toolPresent,uint8_t tool){
    assert(targetPresent && !target && !waitRequested && !toolPresent && !tool);
    assert(GCodeInput::flashScriptActive());
    internalEvents.push_back(0);
}
void loadFilamentWithParameters(int16_t p, uint8_t f) {
    assert(GCodeInput::flashScriptActive());
    ++softLoads; softPressure=p; softSpeed=f;
    GCodeInput::executeSoftFilament(true); // refuse nested typed entry
}
void unloadFilamentWithParameters(int16_t p) {
    assert(GCodeInput::flashScriptActive());
    ++softUnloads; softPressure=p;
    GCodeInput::executeFlashScript("G1 X77"); // refuse nested text entry
}
void dispatchMotion(GCode* code) {
    ++dispatchedCommands;
    if (code->internalCommand) { ++scriptCommands; lastScriptCommand = *code; }
    if (code->hasE()) ++extrusionRequests;
}
void dispatchMachine(GCode* code) {
    if (code->M == 111) return; // retired command, no state
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
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    txFree=63;
#endif
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
    readReply("Error:expected line 42 got 43\r\n\r\nResend:42\r\nok\r\n");
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
    popM(115);
    assert(replies.empty());
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
    readReply("Error:Wrong checksum\r\n\r\nResend:2\r\nok\r\n");
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
    // Establish binary mode before exercising the binary resend handshake.
    feed(packet);
    readReply("ok 0\r\n");
    popM(105);
    auto bad = packet;
    bad.back() ^= 1;
    feed(bad);
    readReply("Error:Wrong checksum\r\n\r\nResend:1\r\nok\r\n");
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
    // Positive control: retired M111 S7 is harmless; ordinary commands still run.
    feed("M111 S7\nM104 S100\nM140 S40\nG1 E1\n");
    while (!incoming.empty()) Commands::commandLoop();
    assert(!legacyRejected && heaterRequests == 2 && extrusionRequests == 1);
    replies.clear();
    const unsigned normalDispatches = dispatchedCommands;
    if (scenario == "legacy-during-stop") stopped = true;
    const bool inch = scenario == "inch-ascii" || scenario == "inch-binary";
    const unsigned legacyMask = inch ? 0 : scenario == "legacy-ascii" ? 8 : 15;
    if (scenario == "inch-ascii") {
        feed(numbered("N1 G20"));
    } else if (scenario == "inch-binary") {
        // Binary V1 G20 (G field, one byte).
        feed(binary({0x84, 0x00, 20}));
    } else if (scenario == "legacy-binary") {
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
    assert(legacyRejected && stopRequests == 1);
    assert(replies.find(inch ? "G20 unsupported" : "M111 bit 8 unsupported") != std::string::npos);
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
    assert(legacyRejected && stopRequests == 1);
    assert(dispatchedCommands == normalDispatches && heaterRequests == 2 && extrusionRequests == 1);
    assert(statusCommands == 3 && replies.find("Job rejected;") != std::string::npos);
    GCodeInput::executeFlashScript("G1 E1\nM104 S200\n");
    assert(dispatchedCommands == normalDispatches && heaterRequests == 2 && extrusionRequests == 1);
    feed("M111 S8\n");
    Commands::commandLoop();
    assert(legacyRejected && stopRequests == 1);
}

#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
void strictEepromSyntax() {
    // Exhaust both signed alias neighborhoods before the 8-bit/16-bit conversion.
    for(int32_t type=-65536;type<=65536;++type){
        const std::string line="M206 T"+std::to_string(type)+" P107 "+(type==3?"X1":"S80");
        assert(validEepromAscii(line.c_str())==(type>=0&&type<=3));
    }
    for(int32_t pos=-65536;pos<=65536;++pos){
        const std::string line="M206 T0 P"+std::to_string(pos)+" S80";
        assert(validEepromAscii(line.c_str())==(pos>=0&&pos<2048));
    }
    for(const char* valid:{"M206 T0 P107 S80","m206p107t0s80","N1 M206 T3 P108 X1.25*12",
                          "M206 T3 P108 X-1.25e+2","M206 T2 P79 S2147483647",
                          "M206 T2 P79 S-2147483648","T0 M206 P107 S+80",
                          "M206 T0 P107 S80 ; comment"})assert(validEepromAscii(valid));
    for(const char* invalid:{"M206 T256 P107 S80","M206 T-256 P107 S80","M206 T P107 S80",
        "M206 T0 P S80","M206 T0 P107 S","M206 T0 P107 S1.5","M206 T0 P107 S2147483648",
        "M206 T0 P107 S-2147483649","M65742 T0 P107 S80","M206 T0 P65643 S80",
        "M206 T3 P108 X","M206 T3 P108 X1e","M206 T3 P108 X1.2.3",
        "M206 T0 T0 P107 S80","M206 T0 P107 P108 S80","M206 T0 P107 S80 X1",
        "M206 T0 P107 S80*0 T1","M206 T3 P108 S1","M206 T0 P107 X1",
        "M206 P107 S80","M206 T0 P107","M206 T3 P108 Xnan"})
        assert(!validEepromAscii(invalid));
    // Ordinary unnumbered commands use the existing numbered-style ACK (current N=0).
    feed("M206 T0 P107 S80\n");readReply("ok 0\r\n");popM(206);
    feed("M206 T256 P107 S80\n");
    for(unsigned i=0;i<8;++i)GCodeInput::readFromSerial();
    assert(!GCodeInput::peekCurrentCommand());
    assert(replies.find("Invalid M206 syntax")!=std::string::npos);
}
void thermalEmergency() {
    feed("N1 M104 S200\n");
    readReply("ok 1\r\n");
    assert(GCodeInput::peekCurrentCommand());
    thermalLocked=true;
    Commands::commandLoop(); // old admitted command is retired without execution/echo
    assert(!GCodeInput::peekCurrentCommand()&&dispatchedCommands==0&&replies.empty());
    feed(numbered("N2 M105"));
    txFree=0;
    for(unsigned n=0;n<1000;++n)Commands::commandLoop();
    assert(!incoming.empty()&&replies.empty()&&liveRequests==0&&periodicVisits==1001);
    txFree=63;
    readReply("ok 2\r\n");
    assert(liveRequests==1&&!GCodeInput::peekCurrentCommand());
    feed(numbered("N2 M105"));readReply("skip 2\r\nok\r\n");
    assert(liveRequests==1);
    feed(numbered("N4 M105"));readReply("\r\nResend:3\r\nok\r\n");
    feed("N3 M105*0\n");readReply("\r\nResend:3\r\nok\r\n");
    feed(numbered("N3 M112"));readReply("Error: THERMAL EMERGENCY\r\nok 3\r\n");
    feed(numbered("N4 M999"));readReply("Error: THERMAL EMERGENCY\r\nok 4\r\n");
    feed(numbered("N5 M115"));readReply("ok 5\r\n");
    assert(replayRequests==1);
    feed(binary({0x83,0x00,0x06,0x00,105}));readReply("ok 6\r\n");
    assert(liveRequests==2);
    feed(binary({0x85,0x00,0x07,0x00,1}));readReply("Error: THERMAL EMERGENCY\r\nok 7\r\n");
    feed(numbered("N8 M110"));readReply("ok\r\n");
    GCodeInput::executeFlashScript("G1 X1\nM104 S200\n");
    assert(!GCodeInput::peekCurrentCommand()&&dispatchedCommands==0&&heaterRequests==0&&flushes==0);
}
#endif

void postStopOutputBoundary() {
    feed("G1 X1\nG1 X2\n");
    GCodeInput::readFromSerial();
    GCodeInput::readFromSerial();
    assert(GCodeInput::peekCurrentCommand());
    stopped = true;
    Commands::commandLoop();
    assert(GCodeInput::peekCurrentCommand() && !pendingOutputVisits);
    Commands::commandLoop();
    assert(!GCodeInput::peekCurrentCommand() && pendingOutputVisits == 1);
    assert(!dispatchedCommands);
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    thermalLocked = true;
    Commands::commandLoop();
    assert(pendingOutputVisits == 1);
#endif
}

void flashScript() {
    injectNestedScript = true;
    GCodeInput::executeFlashScript("; comment\nG91\nG1 X0.5 ; jog\n");
    assert(scriptCommands == 2 && lastScriptCommand.G == 1 && lastScriptCommand.X == 0.5f);
    assert(!GCodeInput::flashScriptActive());
    stopDuringScript = true;
    GCodeInput::executeFlashScript("G1 X2\nG1 X3\n");
    assert(scriptCommands == 3 && lastScriptCommand.X == 2 && !GCodeInput::flashScriptActive());
    GCodeInput::executeFlashScript("G1 X4\n");
    assert(scriptCommands == 3);
    stopped = false;
    GCodeInput::executeFlashScript("G1 X5\n");
    assert(scriptCommands == 4 && lastScriptCommand.X == 5 && !GCodeInput::flashScriptActive());
    assert(replies.empty() && !GCodeInput::peekCurrentCommand());
    feed("N1 M105\n");
    readReply("ok 1\r\n");
    popM(105);
}

} // namespace

void outputProgram() {
    const std::vector<int> expected={1,3,2,1,3,-SCRIPT_RETRACT_MM,0,1,3,100,1,3,2,1,3,90,1,3,
                                     1000+Z_AXIS,200,540,1,3,1000+Y_AXIS,245,4800,1,3};
    assert(GCodeInput::executeOutputProgram());
    assert(internalEvents==expected && periodicVisits==8 && !GCodeInput::flashScriptActive());
    assert(!dispatchedCommands);
    unsigned boundary=0;
    for(unsigned n=1;n<=8;++n){
        while(expected[boundary++]!=3){}
        internalEvents.clear();periodicVisits=0;stopAtPeriodicVisit=n;
        assert(!GCodeInput::executeOutputProgram());
        assert(stopped && periodicVisits==n && !GCodeInput::flashScriptActive());
        assert(internalEvents==std::vector<int>(expected.begin(),expected.begin()+boundary));
        stopped=false;stopAtPeriodicVisit=0;
    }
    internalEvents.clear();periodicVisits=0;
    legacyRejected=true;assert(!GCodeInput::executeOutputProgram());legacyRejected=false;
    settingsBlocked=true;assert(!GCodeInput::executeOutputProgram());settingsBlocked=false;
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    thermalLocked=true;assert(!GCodeInput::executeOutputProgram());thermalLocked=false;
#endif
    assert(internalEvents.empty() && !periodicVisits);
}

void hardFilament() {
    for(bool unload:{false,true}){
        internalEvents.clear();periodicVisits=0;
        GCodeInput::executeHardFilament(unload);
        const std::vector<int> expected={unload?PREHEAT_A_HOTEND_C:PREHEAT_B_HOTEND_C,3,
            1,3,2,1,3,unload?-90:90,unload?500:100,1,3,2,1,3,0,3};
        assert(internalEvents==expected && periodicVisits==6 && !GCodeInput::flashScriptActive());
        assert(!dispatchedCommands);
        // Stop immediately after each original command: no later effects.
        unsigned boundary=0;
        for(unsigned n=1;n<=6;++n){
            while(expected[boundary++]!=3){}
            internalEvents.clear();periodicVisits=0;stopAtPeriodicVisit=n;
            GCodeInput::executeHardFilament(unload);
            assert(stopped && periodicVisits==n && !GCodeInput::flashScriptActive());
            assert(internalEvents==std::vector<int>(expected.begin(),expected.begin()+boundary));
            stopped=false;stopAtPeriodicVisit=0;
        }
    }
    internalEvents.clear();periodicVisits=0;
    legacyRejected=true;GCodeInput::executeHardFilament(false);legacyRejected=false;
    settingsBlocked=true;GCodeInput::executeHardFilament(true);settingsBlocked=false;
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    thermalLocked=true;GCodeInput::executeHardFilament(false);thermalLocked=false;
#endif
    assert(internalEvents.empty() && !periodicVisits);
}

void softFilament() {
    injectNestedScript = true;
    GCodeInput::executeSoftFilament(false);
    assert(softLoads==1 && softUnloads==0 && softPressure==3500 && softSpeed==3);
    assert(periodicVisits==1 && dispatchedCommands==0 && !GCodeInput::flashScriptActive());
    GCodeInput::executeSoftFilament(true);
    assert(softLoads==1 && softUnloads==1 && softPressure==std::min(4500, EMERGENCY_PAUSE_DIGITS_MAX));
    assert(periodicVisits==2 && dispatchedCommands==0 && !GCodeInput::flashScriptActive());
    stopped=true; GCodeInput::executeSoftFilament(false); stopped=false;
    legacyRejected=true; GCodeInput::executeSoftFilament(true); legacyRejected=false;
    settingsBlocked=true; GCodeInput::executeSoftFilament(false); settingsBlocked=false;
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    thermalLocked=true; GCodeInput::executeSoftFilament(true); thermalLocked=false;
#endif
    assert(softLoads==1 && softUnloads==1 && periodicVisits==2);
    stopDuringScript=true;
    GCodeInput::executeSoftFilament(false);
    assert(stopped && softLoads==2 && periodicVisits==3 && !GCodeInput::flashScriptActive());
    stopped=false;
    GCodeInput::executeSoftFilament(true);
    assert(softUnloads==2 && periodicVisits==4 && dispatchedCommands==0);
}

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
    else if (scenario == "soft-filament") softFilament();
    else if (scenario == "hard-filament") hardFilament();
    else if (scenario == "output-program") outputProgram();
    else if (scenario == "post-stop-output") postStopOutputBoundary();
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    else if (scenario == "thermal-emergency") thermalEmergency();
    else if (scenario == "m206-syntax") strictEepromSyntax();
#endif
    else if (scenario == "legacy-ascii" || scenario == "legacy-binary" || scenario == "legacy-during-stop" ||
             scenario == "inch-ascii" || scenario == "inch-binary") legacyStream(scenario);
    else return 1;
    std::printf("Serial protocol replay passed (%s)\n", argv[1]);
}

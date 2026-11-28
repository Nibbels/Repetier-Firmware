/*
    This file is part of the Repetier-Firmware for RF devices from Conrad Electronic SE.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/

// Serial framing, command decoding, receive queue and acknowledgments; not machine command execution.

#include "Repetier.h"
#include "GCodeInput.h"
#include "GCodeCommands.h"
#include "UsbSerial.h"
#include "DiagnosticHistory.h"
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
#include "ThermalEmergencyReport.h"
#endif

namespace {
class CommandQueue {
public:
    bool empty() const { return count_ == 0; }
    bool full() const { return count_ >= GCODE_BUFFER_SIZE; }
    GCode& writeSlot() { return slots_[write_]; }
    GCode* front() { return empty() ? nullptr : &slots_[read_]; }

    void publish() {
        if (++write_ == GCODE_BUFFER_SIZE)
            write_ = 0;
        ++count_;
    }

    void retireFront() {
        if (++read_ == GCODE_BUFFER_SIZE)
            read_ = 0;
        --count_;
    }

private:
    GCode slots_[GCODE_BUFFER_SIZE];
    uint8_t read_ = 0;
    uint8_t write_ = 0;
    volatile uint8_t count_ = 0;
};

CommandQueue commandQueue;
bool flashScriptRunning = false; // foreground-only script owner

constexpr uint8_t maxCommandSize = 96;
constexpr uint8_t maxBinaryTextSize = 80; // V2 text length byte plus payload.

struct ReceiveFrame {
    uint8_t bytes[maxCommandSize];
    uint8_t length;
    uint8_t expectedLength;
    bool binary;
    bool comment;
    bool borrowedText;
} frame;

// Embedded flash commands use the same parser state as incoming serial commands.
struct ReceiveSession {
    uint32_t lastLineNumber = 0;
    uint32_t currentLineNumber = 0;
    millis_t timeOfLastDataPacket = 0;
    millis_t lastBusySignal = 0;
    int8_t waitingForResend = -1;
    bool lastCommandWasBinary = false;
    uint8_t formatErrors = 0;
} serialReceive;

#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
// Reuse the normal framing and checksum code, but all emergency replies
// are short atomic spans. The receive service reserves an EMPTY TX ring
// before reading, so the sole foreground producer cannot be blocked.
static void emergencyReply(ThermalEmergency::ReportLine& line) {
    if(line.finish())UsbSerial::tryWriteSpan(line.data(),line.size());
}
static void emergencyOk(bool numbered) {
    ThermalEmergency::ReportLine line;
    line.text(PSTR("ok"));
    if(numbered){line.character(' ');line.number(serialReceive.currentLineNumber);}
    emergencyReply(line);
}
#endif

inline float parseFloatValue(char* s) {
    char* endPtr;
    while (*s == 32)
        s++; // skip spaces
    float f = (strtod(s, &endPtr));
    if (s == endPtr)
        f = 0.0; // treat empty string "x " as "x0"
    return f;
}
inline long parseLongValue(char* s) {
    char* endPtr;
    while (*s == 32)
        s++; // skip spaces
    long l = (strtol(s, &endPtr, 10));
    if (s == endPtr)
        l = 0; // treat empty string argument "p " as "p0"
    return l;
}

#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
// M206 is a typed settings transaction, unlike ordinary permissive G-code.
// Recheck its original ASCII spelling before narrowed T/M or empty numeric
// tokens can alias a different setting. Binary T is already exactly one byte.
static bool eepromDigit(char c){return c>='0'&&c<='9';}
static void eepromSpace(const char*& p){while(*p==' '||*p=='\t')++p;}
static bool eepromInteger(const char*& p,int32_t& result) {
    eepromSpace(p);
    const bool negative=*p=='-';
    if(*p=='+'||*p=='-')++p;
    if(!eepromDigit(*p))return false;
    const uint32_t limit=negative?2147483648UL:2147483647UL;
    uint32_t value=0;
    do{
        const uint8_t digit=uint8_t(*p++-'0');
        if(value>(limit-digit)/10)return false;
        value=value*10+digit;
    }while(eepromDigit(*p));
    result=negative?(value==2147483648UL?INT32_MIN:-int32_t(value)):int32_t(value);
    return true;
}
static bool eepromFloatSpelling(const char*& p) {
    eepromSpace(p);
    if(*p=='+'||*p=='-')++p;
    bool digit=false;
    while(eepromDigit(*p)){digit=true;++p;}
    if(*p=='.'){++p;while(eepromDigit(*p)){digit=true;++p;}}
    if(!digit)return false;
    if(*p=='e'||*p=='E'){
        ++p;if(*p=='+'||*p=='-')++p;
        if(!eepromDigit(*p))return false;
        while(eepromDigit(*p))++p;
    }
    return true;
}
static bool validEepromAscii(const char* p) {
    enum : uint8_t {M=1,P=2,T=4,S=8,X=16,N=32};
    uint8_t seen=0,type=255;
    for(;;){
        eepromSpace(p);
        if(!*p||*p=='('||*p=='%'||*p==';')break;
        if(*p=='*'){
            ++p;int32_t checksum=0;
            if(!eepromInteger(p,checksum)||checksum<0||checksum>255)return false;
            eepromSpace(p);
            if(*p&&*p!='('&&*p!='%'&&*p!=';')return false;
            break;
        }
        char key=*p++;
        if(key>='a'&&key<='z')key=char(key-'a'+'A');
        uint8_t bit=0;
        if(key=='M')bit=M;else if(key=='P')bit=P;else if(key=='T')bit=T;
        else if(key=='S')bit=S;else if(key=='X')bit=X;else if(key=='N')bit=N;
        else return false;
        if(seen&bit)return false;
        if(bit==N&&(seen&M))return false;
        seen|=bit;
        if(bit==X){if(!eepromFloatSpelling(p))return false;continue;}
        int32_t value;
        if(!eepromInteger(p,value))return false;
        if(bit==M&&value!=206)return false;
#ifdef RF_EEPROM_ZONE_EXPERIMENT
        if(bit==P&&(value<2048||value>=3072))return false;
#else
        if(bit==P&&(value<0||value>=2048))return false;
#endif
        if(bit==T){if(value<0||value>3)return false;type=uint8_t(value);}
    }
    return (seen&(M|P|T))==(M|P|T)&&(seen&(S|X))==(type==3?X:S);
}
#endif

} // namespace

/** \page Repetier-protocol

\section Introduction

The repetier-protocol was developed, to overcome some shortcommings in the standard
RepRap communication method, while remaining backward compatible. To use the improved
features of this protocal, you need a host which speaks it. On Windows the recommended
host software is Repetier-Host. It is developed in parallel to this firmware and supports
all implemented features.

\subsection Improvements

- With higher speeds, the serial connection is more likely to produce communication failures.
  The standard method is to transfer a checksum at the end of the line. This checksum is the
  XORd value of all characters send. The value is limited to a range between 0 and 127. It can
  not detect two identical missing characters or a wrong order. Therefore the new protocol
  uses Fletchers checksum, which overcomes these shortcommings.
- The new protocol send data in binary format. This reduces the data size to less then 50% and
  it speeds up decoding the command. No slow conversion from string to floats are needed.

*/

/** \brief Computes size of binary data from bitfield.

In the repetier-protocol in binary mode, the first 2 uint8_ts define the
data. From this bitfield, this function computes the size of the command
including the 2 uint8_ts of the bitfield and the 2 uint8_ts for the checksum.

Gcode Letter to Bit and Datatype:

- N : Bit 0 : 16-Bit Integer
- M : Bit 1 :  8-Bit unsigned uint8_t
- G : Bit 2 :  8-Bit unsigned uint8_t
- X : Bit 3 :  32-Bit Float
- Y : Bit 4 :  32-Bit Float
- Z : Bit 5 :  32-Bit Float
- E : Bit 6 :  32-Bit Float
-  : Bit 7 :  always set to distinguish binary from ASCII line.
- F : Bit 8 :  32-Bit Float
- T : Bit 9 :  8 Bit Integer
- S : Bit 10 : 32 Bit Value
- P : Bit 11 : 32 Bit Integer
- V2 : Bit 12 : Version 2 command for additional commands/sizes
- Ext : Bit 13 : There are 2 more uint8_ts following with Bits, only for future versions
- Int : Bit 14 : Marks it as internal command,
- Text : Bit 15 : 16 Byte ASCII String terminated with 0
Second word if V2:
- I : Bit 0 : 32-Bit float
- J : Bit 1 : 32-Bit float
- R : Bit 2 : 32-Bit float
- D : Bit 3 : 32-Bit float
- C : Bit 4 : 32-Bit float
- H : Bit 5 : 32-Bit float
- A : Bit 6 : 32-Bit float
- B : Bit 7 : 32-Bit float
- K : Bit 8 : 32-Bit float
- L : Bit 9 : 32-Bit float
- O : Bit 0 : 32-Bit float
*/
uint8_t GCodeInput::computeBinarySize(char* ptr) // unsigned int bitfield) {
{
    uint8_t s = 4; // include checksum and bitfield
    uint16_t bitfield = *(uint16_t*)ptr;
    if (bitfield & 1)
        s += 2;
    if (bitfield & 8)
        s += 4;
    if (bitfield & 16)
        s += 4;
    if (bitfield & 32)
        s += 4;
    if (bitfield & 64)
        s += 4;
    //128 Bit 7 :  always set to distinguish binary from ASCII line.
    if (bitfield & 256)
        s += 4;
    if (bitfield & 512)
        s += 1;
    if (bitfield & 1024)
        s += 4;
    if (bitfield & 2048)
        s += 4;
    if (bitfield & 4096) // Version 2 or later
    {
        s += 2; // for bitfield 2
        uint16_t bitfield2 = *(uint16_t*)(ptr + 2);
        if (bitfield & 2)
            s += 2;
        if (bitfield & 4)
            s += 2;
        if (bitfield2 & 1)
            s += 4;
        if (bitfield2 & 2)
            s += 4;
        if (bitfield2 & 4)
            s += 4;
        // START ich rechne len nach dem aktuellsten Stand aus aber nutze diesen nicht.
        if (bitfield2 & 8)
            s += 4;
        if (bitfield2 & 16)
            s += 4;
        if (bitfield2 & 32)
            s += 4;
        if (bitfield2 & 64)
            s += 4;
        if (bitfield2 & 128)
            s += 4;
        if (bitfield2 & 256)
            s += 4;
        if (bitfield2 & 512)
            s += 4;
        if (bitfield2 & 1024)
            s += 4;
        if (bitfield2 & 2048)
            s += 4;
        if (bitfield2 & 4096)
            s += 4;
        if (bitfield2 & 8192)
            s += 4;
        if (bitfield2 & 16384)
            s += 4;
        if (bitfield2 & 32768)
            s += 4;
        // ENDE ich rechne len nach dem aktuellsten Stand aus aber nutze diesen nicht.
        if (bitfield & 32768)
            s += RMath::min(maxBinaryTextSize, (uint8_t)ptr[4] + 1);
    } else {
        if (bitfield & 2)
            s += 1;
        if (bitfield & 4)
            s += 1;
        if (bitfield & 32768)
            s += 16;
    }
    return s;
} // computeBinarySize

void GCodeInput::keepAlive(enum FirmwareState state) {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(Printer::printJob.emergencyLocked())return;
#endif
    millis_t now = HAL::timeInMilliseconds();

    if (state != NotBusy) {
        if ((now - serialReceive.lastBusySignal) < keepAliveIntervalMs) {
            return;
        }
        if (state == Paused) {
            Com::printFLN(PSTR("busy: paused for user interaction"));
        } else if (state == WaitHeater) {
            Com::printFLN(PSTR("busy: heating"));
        } else if (state == Calibrating) {
            Com::printFLN(PSTR("busy: calibration"));
        } else {
            // processing and uncaught cases
            Com::printFLN(PSTR("busy: processing"));
        }
    }
    serialReceive.lastBusySignal = now;

    return;
} // keepAlive

void GCodeInput::requestResend() {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(Printer::printJob.emergencyLocked()) {
        frame.length=0;
        serialReceive.waitingForResend=frame.binary?30:14;
        ThermalEmergency::ReportLine line;
        line.text(PSTR("\r\nResend:"));
        line.number(serialReceive.lastLineNumber+1);
        line.text(PSTR("\r\nok"));
        emergencyReply(line);
        return;
    }
#endif
    HAL::serialFlush();
    frame.length = 0;
    if (frame.binary)
        serialReceive.waitingForResend = 30;
    else
        serialReceive.waitingForResend = 14;
    Com::println();
    Com::printFLN(Com::tResend, serialReceive.lastLineNumber + 1);
    Com::printFLN(Com::tOk);
} // requestResend

/**
Check if result is plausible. If it is, an ok is send and the command is stored in queue.
If not, a resend and ok is send.
*/
void GCodeInput::checkAndPushCommand(GCode& command) {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(Printer::printJob.emergencyLocked()) {
        // No machine command ever reaches the queue from this state, even
        // M112/M999 or a script-like command. Preserve protocol sequencing.
        if(command.hasM() && command.M==GCodeCommands::ResetLineNumber) {
            serialReceive.lastLineNumber=serialReceive.currentLineNumber;
            serialReceive.waitingForResend=-1;
            emergencyOk(false);
            return;
        }
        if(command.hasN() && uint16_t(serialReceive.lastLineNumber+1)!=uint16_t(serialReceive.currentLineNumber)) {
            if(uint16_t(serialReceive.lastLineNumber-serialReceive.currentLineNumber)<40 ||
               serialReceive.waitingForResend>=0) {
                if(serialReceive.waitingForResend>=0)--serialReceive.waitingForResend;
                ThermalEmergency::ReportLine line;
                line.text(PSTR("skip "));
                line.number(serialReceive.currentLineNumber);
                line.text(PSTR("\r\nok"));
                emergencyReply(line);
            }else requestResend();
            return;
        }
        if(command.hasN())serialReceive.lastLineNumber=serialReceive.currentLineNumber;
        serialReceive.lastCommandWasBinary=frame.binary;
        serialReceive.waitingForResend=-1;
        if(!command.hasG() && command.hasM() && command.M==105) {
            ThermalEmergency::requestLiveReport();
            emergencyOk(true);
        }else if(!command.hasG() && command.hasM() && command.M==115) {
            // Request retained evidence on reconnect; not a normal
            // firmware capability reply in the terminal state.
            ThermalEmergency::requestDiagnosticReplay();
            emergencyOk(true);
#ifdef RF_EEPROM_ZONE_EXPERIMENT
        }else if(!command.hasG() && command.hasM() && command.M==DiagnosticHistory::ReadCommand) {
            // Read-only history must not enter the machine command queue.
            DiagnosticHistory::request();
            emergencyOk(true);
#endif
        }else {
            ThermalEmergency::ReportLine line;
            line.text(PSTR("Error: THERMAL EMERGENCY\r\nok "));
            line.number(serialReceive.currentLineNumber);
            emergencyReply(line);
        }
        return;
    }
#endif
    if (command.hasM()) {
        if (command.M == GCodeCommands::ResetLineNumber)
        {
            serialReceive.lastLineNumber = serialReceive.currentLineNumber;
            Com::printFLN(Com::tOk);
            serialReceive.waitingForResend = -1;
            return;
        }
        if (command.M == GCodeCommands::EmergencyReset)
        {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
            if(Printer::printJob.emergencyLocked()) {
                // Already stopped; keep the original diagnostics instead of
                // deliberately rebooting on a queued host cancellation.
                return;
            }
#endif
            Commands::emergencyStop();
        }
    }
    if (command.hasN()) {
        if ((((serialReceive.lastLineNumber + 1) & 0xffff) != (serialReceive.currentLineNumber & 0xffff))) {
            if (static_cast<uint16_t>(serialReceive.lastLineNumber - serialReceive.currentLineNumber) < 40) {
                // we have seen that line already. So we assume it is a repeated resend and we ignore it
                frame.length = 0;
                Com::printFLN(Com::tSkip, serialReceive.currentLineNumber);
                Com::printFLN(Com::tOk);
            } else if (serialReceive.waitingForResend < 0) // after a resend, we have to skip the garbage in buffers, no message for this
            {
                {
                    Com::printF(Com::tExpectedLine, serialReceive.lastLineNumber + 1);
                    Com::printFLN(Com::tGot, serialReceive.currentLineNumber);
                }
                requestResend(); // Line missing, force resend
            } else {
                --serialReceive.waitingForResend;
                frame.length = 0;
                Com::printFLN(Com::tSkip, serialReceive.currentLineNumber);
                Com::printFLN(Com::tOk);
            }
            return;
        }
        serialReceive.lastLineNumber = serialReceive.currentLineNumber;
    }
    if (!GCodeCommands::rejectLegacySimulation(&command) && !Printer::printJob.commandsBlocked()) {
        commandQueue.publish();
    }

    Com::printFLN(Com::tOkSpace, serialReceive.currentLineNumber);

    serialReceive.lastCommandWasBinary = frame.binary;
    keepAlive(NotBusy);
    serialReceive.waitingForResend = -1; // everything is ok.
} // checkAndPushCommand

/**
\brief Get the next buffered command. Returns 0 if no more commands are buffered. For each
returned command, the popCurrentCommand() function must be called.
*/
GCode* GCodeInput::peekCurrentCommand() {
    return commandQueue.front();
} // peekCurrentCommand

/** \brief Removes the last returned command from cache. */
void GCodeInput::popCurrentCommand() {
    GCode* command = commandQueue.front();
    if (!command)
        return;
    commandQueue.retireFront();
}

bool GCodeInput::flashScriptActive() {
    return flashScriptRunning;
}

// The menu's former one-line M3913/M3914 scripts need neither a GCode
// object nor the ASCII parser or general G/M dispatch. Keep the existing
// foreground script ownership through the final periodic service.
// heatingInhibited includes thermal/legacy rejection and EEPROM admission;
// the common typed filament entry revalidates it after waiting for motion.
__attribute__((noinline)) void GCodeInput::executeSoftFilament(bool unload) {
    if (flashScriptRunning || Printer::printJob.commandsBlocked() || Printer::heatingInhibited())
        return;
    flashScriptRunning = true;
    runSoftFilament(unload);
    flashScriptRunning = false;
}

__attribute__((noinline)) void GCodeInput::runSoftFilament(bool unload) {
    if (unload)
        GCodeCommands::unloadFilamentWithParameters(EMERGENCY_PAUSE_DIGITS_MAX < 4500 ? EMERGENCY_PAUSE_DIGITS_MAX : 4500);
    else
        GCodeCommands::loadFilamentWithParameters(3500, 3);
    Commands::checkForPeriodicalActions(Processing);
}

namespace {
// Recheck between every original command, including after any wait
// that pumps thermal service and receives new serial data.
bool internalProgramAdmitted() {
    return !Printer::printJob.commandsBlocked() && !Printer::heatingInhibited();
}

bool finishInternalStep(bool motionCommand) {
    if (motionCommand)
        Printer::inactivity.recordActivity(HAL::timeInMilliseconds());
    Commands::checkForPeriodicalActions(Processing);
    return internalProgramAdmitted();
}

void internalAxisMove(uint8_t axis, float value, float feedrateMmMin) {
    if (isMovingAllowed(PSTR("G0/1")))
        Printer::queueInternalAxisMove(axis, value, feedrateMmMin);
}
} // namespace

__attribute__((noinline)) void GCodeInput::executeHardFilament(bool unload) {
    if (flashScriptRunning || !internalProgramAdmitted())
        return;
    flashScriptRunning = true;
    runHardFilament(unload);
    flashScriptRunning = false;
}

// Same operations and service boundaries as the former hard scripts.
__attribute__((noinline)) void GCodeInput::runHardFilament(bool unload) {
    GCodeCommands::waitForHotendWithParameters(true,
        unload ? PREHEAT_A_HOTEND_C : PREHEAT_B_HOTEND_C, false, 0);
    if (!finishInternalStep(false)) return; // M109
    if (!finishInternalStep(true)) return;  // G21: mm is the only supported unit
    Printer::setEPositionMM(0);
    if (!finishInternalStep(true)) return;  // G92 E0
    internalAxisMove(E_AXIS, unload ? -90.0f : 90.0f, unload ? 500.0f : 100.0f);
    if (!finishInternalStep(true)) return;  // G1
    Printer::setEPositionMM(0);
    if (!finishInternalStep(true)) return;  // G92 E0
    GCodeCommands::setHotendTemperatureWithParameters(true, 0, false, false, 0);
    finishInternalStep(false);             // M104 S0 (shared queue-settle path)
}

__attribute__((noinline)) bool GCodeInput::executeOutputProgram() {
    if (flashScriptRunning || !internalProgramAdmitted())
        return false;
    flashScriptRunning = true;
    const bool finished = runOutputProgram();
    flashScriptRunning = false;
    return finished;
}

__attribute__((noinline)) bool GCodeInput::runOutputProgram() {
    if (!finishInternalStep(true)) return false; // G21
    Printer::setEPositionMM(0);
    if (!finishInternalStep(true)) return false; // G92 E0
    internalAxisMove(E_AXIS, -float(SCRIPT_RETRACT_MM), 0);
    if (!finishInternalStep(true)) return false; // G1 E-1, retain F
    GCodeCommands::dwellMilliseconds(100);
    if (!finishInternalStep(true)) return false; // G4 P100
    Printer::setEPositionMM(0);
    if (!finishInternalStep(true)) return false; // G92 E0
    GCodeCommands::selectAbsolutePositioningWithSource(true);
    if (!finishInternalStep(true)) return false; // G90
    internalAxisMove(Z_AXIS, 200, 540);
    if (!finishInternalStep(true)) return false; // G1 Z200 F540
    internalAxisMove(Y_AXIS, 245, 4800);
    return finishInternalStep(true);            // G1 Y245 F4800
}

// Receive one complete command from the USB serial port per call.
void GCodeInput::readFromSerial() {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    // A reply fits in 63 bytes. No other foreground sender can run
    // during this bounded visit; RX/TX ISRs do not publish TX head.
    if(Printer::printJob.emergencyLocked() && UsbSerial::transmitFree()!=63)return;
#endif
    if (commandQueue.full() || (frame.borrowedText && !commandQueue.empty())) {
        keepAlive(Processing);
        return;
    }
    frame.borrowedText = false;
    const millis_t now = HAL::timeInMilliseconds();

    if (!HAL::serialByteAvailable()) {
        const millis_t idleTime = now - serialReceive.timeOfLastDataPacket;
        if ((serialReceive.waitingForResend >= 0 || frame.length > 0) && idleTime > 200) {
            requestResend();
            serialReceive.timeOfLastDataPacket = now;
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
            if(Printer::printJob.emergencyLocked())return;
#endif
        } else if (commandQueue.empty() && idleTime > 1000) {
            // Unblock the host if an acknowledgement was lost.
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
            if(Printer::printJob.emergencyLocked()) {
                ThermalEmergency::ReportLine line;
                line.text(PSTR("wait"));
                emergencyReply(line);
                serialReceive.timeOfLastDataPacket=now;
                return;
            }
#endif
            Com::printFLN(Com::tWait);
            serialReceive.timeOfLastDataPacket = now;
        }
    }

    // Consume at most one RX ring's usable capacity per foreground visit.
    // Comments, blank lines and binary resynchronization can otherwise keep
    // frame.length small indefinitely while the UART continues receiving.
    if (frame.length >= maxCommandSize || !HAL::serialByteAvailable())
        return;
    serialReceive.timeOfLastDataPacket = now;
    uint8_t budget = UsbSerial::receiveCapacity;
    do {
        frame.bytes[frame.length++] = HAL::serialReadByte();
        if (frame.length == 1 && !frame.comment) {
            if (serialReceive.waitingForResend >= 0 && serialReceive.lastCommandWasBinary) {
                if (!frame.bytes[0])
                    --serialReceive.waitingForResend; // Skip 30 zeros to regain frame sync.
                else
                    serialReceive.waitingForResend = 30;
                frame.length = 0;
                continue;
            }
            if (!frame.bytes[0]) {
                frame.length = 0;
                return;
            }
            frame.binary = (frame.bytes[0] & 128) != 0;
        }

        if (frame.binary) {
            if (frame.length < 2)
                continue;
            if (frame.length == 4 || frame.length == 5)
                frame.expectedLength = computeBinarySize((char*)frame.bytes);
            if (frame.length == frame.expectedLength) {
                GCode& command = commandQueue.writeSlot();
                if (parseBinary(command, frame.bytes, frame.expectedLength))
                    checkAndPushCommand(command);
                else
                    requestResend();
                frame.length = 0;
                return;
            }
        } else {
            const char ch = frame.bytes[frame.length - 1];
            if (ch == 0 || ch == '\n' || ch == '\r') {
                frame.bytes[frame.length - 1] = 0;
                frame.comment = false;
                if (frame.length == 1) {
                    frame.length = 0;
                    continue;
                }
                GCode& command = commandQueue.writeSlot();
                if (parseAscii(command, (char*)frame.bytes))
                    checkAndPushCommand(command);
                else
                    requestResend();
                frame.length = 0;
                return;
            }
            if (ch == ';')
                frame.comment = true;
            if (frame.comment)
                --frame.length;
        }

        if (frame.length == maxCommandSize) {
            requestResend();
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
            if(Printer::printJob.emergencyLocked())return;
#endif
        }
    } while (--budget && frame.length < maxCommandSize && HAL::serialByteAvailable());
} // readFromSerial

/** \brief Converts a binary uint8_tfield containing one GCode line into a GCode structure.
    Returns true if checksum was correct. */

bool GCodeInput::parseBinary(GCode& command, uint8_t* buffer, int8_t length) {
    command.internalCommand = false;
    unsigned int sum1 = 0, sum2 = 0; // for fletcher-16 checksum
    // first do fletcher-16 checksum tests see
    // http://en.wikipedia.org/wiki/Fletcher's_checksum
    uint8_t* p = buffer;
    uint8_t len = length - 2;
    while (len) {
        uint8_t tlen = len > 21 ? 21 : len;
        len -= tlen;
        do {
            sum1 += *p++;
            if (sum1 >= 255)
                sum1 -= 255;
            sum2 += sum1;
            if (sum2 >= 255)
                sum2 -= 255;
        } while (--tlen);
    }
    sum1 -= *p++;
    sum2 -= *p;
    if (sum1 | sum2) {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        if(!Printer::printJob.emergencyLocked())
#endif
            Com::printErrorFLN(Com::tWrongChecksum);
        return false;
    }

    p = buffer;
    command.params = *(uint16_t*)p;
    p += 2;
    uint8_t textlen = 16;
    if (command.isV2()) {
        command.params2 = *(uint16_t*)p;
        p += 2;
        if (command.hasString()) {
            textlen = *p++;
            // The frame size is capped including the length byte. A clipped
            // frame may have a valid checksum but not the declared text.
            if (textlen >= maxBinaryTextSize)
                return false;
        }
    } else
        command.params2 = 0;

    if (command.params & 1) {
        serialReceive.currentLineNumber = command.N = *(uint16_t*)p;
        p += 2;
    }
    if (command.isV2()) // Read G,M as 16 bit value
    {
        if (command.hasM()) {
            command.M = *(uint16_t*)p;
            p += 2;
        }
        if (command.hasG()) {
            command.G = *(uint16_t*)p;
            p += 2;
        }
    } else {
        if (command.hasM()) {
            command.M = *p++;
        }
        if (command.hasG()) {
            command.G = *p++;
        }
    }

    // Special behaviour for text Gcodes for RFx000, they have no additional switches - just text.
    if (command.hasM() && GCodeCommands::takesPanelText(command.M)) {
        if (command.hasString()) // set text pointer to string
        {
            command.text = (char*)p;
            command.text[textlen] = 0;                    // Terminate string overwriting checksum
            frame.borrowedText = true; // Don't destroy string until executed
        }
        serialReceive.formatErrors = 0;
        return true;
    }

    if (command.hasX()) {
        command.X = *(float*)p;
        p += 4;
    }
    if (command.hasY()) {
        command.Y = *(float*)p;
        p += 4;
    }
    if (command.hasZ()) {
        command.Z = *(float*)p;
        p += 4;
    }
    if (command.hasE()) {
        command.E = *(float*)p;
        p += 4;
    }
    if (command.hasF()) {
        command.F = *(float*)p;
        p += 4;
    }
    if (command.hasT()) {
        command.T = *p++;
    }
    if (command.hasS()) {
        command.S = *(int32_t*)p;
        p += 4;
    }
    if (command.hasP()) {
        command.P = *(int32_t*)p;
        p += 4;
    }
    if (command.hasI()) {
        command.I = *(float*)p;
        p += 4;
    }
    if (command.hasJ()) {
        command.J = *(float*)p;
        p += 4;
    }
    if (command.hasR()) {
        command.R = *(float*)p;
        p += 4;
    }
    if (command.hasD()) {
        command.D = *(float*)p;
        p += 4;
    }
    if (command.hasC()) {
        command.C = *(float*)p;
        p += 4;
    }
    if (command.hasH()) {
        command.H = *(float*)p;
        p += 4;
    }
    if (command.hasA()) {
        command.A = *(float*)p;
        p += 4;
    }
    if (command.hasB()) {
        command.B = *(float*)p;
        p += 4;
    }
    if (command.hasK()) {
        command.K = *(float*)p;
        p += 4;
    }
    if (command.hasL()) {
        command.L = *(float*)p;
        p += 4;
    }
    if (command.hasO()) {
        command.O = *(float*)p;
        p += 4;
    }
    if (command.hasString()) // set text pointer to string
    {
        command.text = (char*)p;
        command.text[textlen] = 0;                    // Terminate string overwriting checksum
        frame.borrowedText = true; // Don't destroy string until executed
    }
    serialReceive.formatErrors = 0;
    return true;
} // parseBinary

/**
  Converts a ASCII GCode line into a GCode structure.
*/
#if defined(__AVR__) && defined(RF_HEATER_RESET_WDT_EXPERIMENT)
// GCC7's inline switch table contains 16-bit WORD destinations used
// with EIND=0. Keep the entire parser below byte 0x20000 in the recovery
// linker experiment. The linked audit must prove all table destinations.
__attribute__((noinline, section(".text.rf_near_parser")))
#endif
bool GCodeInput::parseAscii(GCode& command, char* line) {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    const char* originalLine=line;
#endif
    char* pos = line;
    command.params = 0;
    command.params2 = 0;
    command.internalCommand = false;
    bool hasChecksum = false;
    char c;
    while ((c = *(pos++))) {
        if (c == '(' || c == '%')
            break; // alternative comment or program block
        switch (c) {
        case 'N':
        case 'n': {
            if (command.hasG() || command.hasM())
                break;
            serialReceive.currentLineNumber = parseLongValue(pos);
            command.params |= 1;
            command.N = serialReceive.currentLineNumber;
            break;
        }
        case 'G':
        case 'g': {
            // A line contains one G or M command. Later letters may be arguments.
            if (command.hasG() || command.hasM())
                break;
            command.G = parseLongValue(pos) & 0xffff;
            command.params |= 4;
            break;
        }
        case 'M':
        case 'm': {
            if (command.hasG() || command.hasM())
                break;
            command.M = parseLongValue(pos) & 0xffff;
            command.params |= 2;
            // handle non standard text arguments that some M codes have
            if (GCodeCommands::takesPanelText(command.M)) {
                char digit;
                while ((digit = *pos)) {
                    if (digit < '0' || digit > '9')
                        break;
                    pos++;
                }
                while ((digit = *pos)) {
                    if (digit != ' ')
                        break;
                    pos++;
                    // skip leading white spaces (may be no white space)
                }
                command.text = pos;
                while (*pos && *pos != '*')
                    pos++;
                *pos = 0;
                frame.borrowedText = true; // don't risk string be deleted
                command.params |= 32768;
            }
            break;
        }
        case 'X':
        case 'x': {
            command.X = parseFloatValue(pos);
            command.params |= 8;
            break;
        }
        case 'Y':
        case 'y': {
            command.Y = parseFloatValue(pos);
            command.params |= 16;
            break;
        }
        case 'Z':
        case 'z': {
            command.Z = parseFloatValue(pos);
            command.params |= 32;
            break;
        }
        case 'E':
        case 'e': {
            command.E = parseFloatValue(pos);
            command.params |= 64;
            break;
        }
        case 'F':
        case 'f': {
            command.F = parseFloatValue(pos);
            command.params |= 256;
            break;
        }
        case 'T':
        case 't': {
            command.T = parseLongValue(pos) & 0xff;
            command.params |= 512;
            break;
        }
        case 'S':
        case 's': {
            command.S = parseLongValue(pos);
            command.params |= 1024;
            break;
        }
        case 'P':
        case 'p': {
            command.P = parseLongValue(pos);
            command.params |= 2048;
            break;
        }
        case 'I':
        case 'i': {
            command.I = parseFloatValue(pos);
            command.params2 |= 1;
            break;
        }
        case 'J':
        case 'j': {
            command.J = parseFloatValue(pos);
            command.params2 |= 2;
            break;
        }
        case 'R':
        case 'r': {
            command.R = parseFloatValue(pos);
            command.params2 |= 4;
            break;
        }
        case 'D':
        case 'd': {
            command.D = parseFloatValue(pos);
            command.params2 |= 8;
            break;
        }
        case 'C':
        case 'c': {
            command.C = parseFloatValue(pos);
            command.params2 |= 16;
            break;
        }
        case 'H':
        case 'h': {
            command.H = parseFloatValue(pos);
            command.params2 |= 32;
            break;
        }
        case 'A':
        case 'a': {
            command.A = parseFloatValue(pos);
            command.params2 |= 64;
            break;
        }
        case 'B':
        case 'b': {
            command.B = parseFloatValue(pos);
            command.params2 |= 128;
            break;
        }
        case 'K':
        case 'k': {
            command.K = parseFloatValue(pos);
            command.params2 |= 256;
            break;
        }
        case 'L':
        case 'l': {
            command.L = parseFloatValue(pos);
            command.params2 |= 512;
            break;
        }
        case 'O':
        case 'o': {
            command.O = parseFloatValue(pos);
            command.params2 |= 1024;
            break;
        }
        case '*': //checksum
        {
            uint8_t checksum_given = parseLongValue(pos);
            uint8_t checksum = 0;
            while (line != (pos - 1))
                checksum ^= *line++;
            if (checksum != checksum_given) {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
                if(!Printer::printJob.emergencyLocked()) {
#endif
                    Com::printErrorFLN(Com::tWrongChecksum);
                    reportCorruptCommand(command);
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
                }
#endif
                return false; // mismatch
            }
            hasChecksum = true;
            break;
        }
        default:
            break;
        } // end switch
    }     // end while
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    if(command.hasM()&&command.M==206&&!validEepromAscii(originalLine)) {
        if(!Printer::printJob.emergencyLocked())Com::printErrorFLN(PSTR("Invalid M206 syntax"));
        return false;
    }
#endif
    if (serialReceive.lastCommandWasBinary && !hasChecksum && !frame.borrowedText) {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        if(!Printer::printJob.emergencyLocked())
#endif
        Com::printErrorFLN(PSTR("Checksum required when switching back to ASCII protocol."));
        return false;
    }
    if (command.hasFormatError() /*|| (params & 518) == 0*/) // Must contain G, M or T command and parameter need to have variables!
    {
        serialReceive.formatErrors++;
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        if(!Printer::printJob.emergencyLocked())
#endif
            Com::printErrorFLN(Com::tFormatError);
        if (serialReceive.formatErrors < 3)
            return false;
    } else
        serialReceive.formatErrors = 0;
    return true;
}


void GCodeInput::reportCorruptCommand(GCode& command) {
    Com::printF(PSTR("Corrupted: "));
    if ((int)frame.binary) {
        command.printCommand();
    } else {
        Com::print((char*)frame.bytes);
        Com::println();
    }
} // outputGCommand

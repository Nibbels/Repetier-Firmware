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

#include "Repetier.h"

GCode GCode::commandsBuffered[GCODE_BUFFER_SIZE];  ///< Buffer for received commands.
uint8_t GCode::bufferReadIndex = 0;                ///< Read position in gcode_buffer.
uint8_t GCode::bufferWriteIndex = 0;               ///< Write position in gcode_buffer.
uint8_t GCode::commandReceiving[MAX_CMD_SIZE];     ///< Current received command.
uint8_t GCode::commandsReceivingWritePosition = 0; ///< Writing position in gcode_transbuffer.
uint8_t GCode::sendAsBinary;                       ///< Flags the command as binary input.
uint8_t GCode::commentDetected = false;            ///< Flags true if we are reading the comment part of a command.
uint8_t GCode::binaryCommandSize;                  ///< Expected size of the incoming binary command.
bool GCode::waitUntilAllCommandsAreParsed = false; ///< Don't read until all commands are parsed. Needed if gcode_buffer is misused as storage for strings.
uint32_t GCode::actLineNumber;                     ///< Line number of current command.
volatile uint8_t GCode::bufferLength = 0;          ///< Number of commands stored in gcode_buffer
uint8_t GCode::formatErrors = 0;
PGM_P GCode::fatalErrorMsg = NULL;  ///< message unset = no fatal error
millis_t GCode::lastBusySignal = 0; ///< When was the last busy signal
GCode::SerialReceiveState GCode::serialReceive = {0, 0, -1, false};

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
uint8_t GCode::computeBinarySize(char* ptr) // unsigned int bitfield) {
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
            s += RMath::min((uint8_t)80, (uint8_t)ptr[4] + 1);
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

void GCode::keepAlive(enum FirmwareState state) {
    millis_t now = HAL::timeInMilliseconds();

    if (state != NotBusy) {
        if ((now - lastBusySignal) < keepAliveIntervalMs) {
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
    lastBusySignal = now;

    return;
} // keepAlive

void GCode::requestResend() {
    HAL::serialFlush();
    commandsReceivingWritePosition = 0;
    if (sendAsBinary)
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
void GCode::checkAndPushCommand() {
    if (hasM()) {
        if (M == 110) // Reset line number
        {
            serialReceive.lastLineNumber = actLineNumber;
            Com::printFLN(Com::tOk);
            serialReceive.waitingForResend = -1;
            return;
        }
        if (M == 112) // Emergency kill - freeze/reset printer
        {
            Commands::emergencyStop();
        }
    }
    if (hasN()) {
        if ((((serialReceive.lastLineNumber + 1) & 0xffff) != (actLineNumber & 0xffff))) {
            if (static_cast<uint16_t>(serialReceive.lastLineNumber - actLineNumber) < 40) {
                // we have seen that line already. So we assume it is a repeated resend and we ignore it
                commandsReceivingWritePosition = 0;
                Com::printFLN(Com::tSkip, actLineNumber);
                Com::printFLN(Com::tOk);
            } else if (serialReceive.waitingForResend < 0) // after a resend, we have to skip the garbage in buffers, no message for this
            {
                if (Printer::debugErrors()) {
                    Com::printF(Com::tExpectedLine, serialReceive.lastLineNumber + 1);
                    Com::printFLN(Com::tGot, actLineNumber);
                }
                requestResend(); // Line missing, force resend
            } else {
                --serialReceive.waitingForResend;
                commandsReceivingWritePosition = 0;
                Com::printFLN(Com::tSkip, actLineNumber);
                Com::printFLN(Com::tOk);
            }
            return;
        }
        serialReceive.lastLineNumber = actLineNumber;
    }
    if (GCode::hasFatalError()) {
        GCode::reportFatalError();
    } else {
        pushCommand();
    }

    Com::printFLN(Com::tOkSpace, actLineNumber);

    serialReceive.lastCommandWasBinary = sendAsBinary;
    keepAlive(NotBusy);
    serialReceive.waitingForResend = -1; // everything is ok.
} // checkAndPushCommand

void GCode::pushCommand() {
    if (Printer::printStop.commandsBlocked())
        return; // Continue acknowledging the host while the stop sequence drains input.

    if (++bufferWriteIndex >= GCODE_BUFFER_SIZE)
        bufferWriteIndex = 0;
    bufferLength++;
} // pushCommand

/**
\brief Get the next buffered command. Returns 0 if no more commands are buffered. For each
returned command, the popCurrentCommand() function must be called.
*/
GCode* GCode::peekCurrentCommand() {
    if (bufferLength == 0)
        return NULL; // No more data
    return &commandsBuffered[bufferReadIndex];
} // peekCurrentCommand

/** \brief Removes the last returned command from cache. */
void GCode::popCurrentCommand() {
    if (!bufferLength)
        return; // Should not happen, but safety first
    echoCommand();
    if (++bufferReadIndex == GCODE_BUFFER_SIZE)
        bufferReadIndex = 0;
    bufferLength--;
} // popCurrentCommand

void GCode::echoCommand() {
    if (Printer::debugEcho()) {
        Com::printF(Com::tEcho);
        printCommand();
    }
} // echoCommand

/** \brief Execute commands in progmem stored string. Multiple commands are seperated by \n */
void GCode::executeFString(FSTRINGPARAM(cmd)) {
    char buf[80];
    uint8_t buflen;
    char c = 0;
    GCode code;
    do {
        // Wait for a free place in command buffer
        // Scan next command from string
        uint8_t comment = 0;
        buflen = 0;
        do {
            c = HAL::readFlashByte(cmd++);
            if (c == 0 || c == '\n')
                break;
            if (c == ';')
                comment = 1;
            if (comment)
                continue;
            buf[buflen++] = c;
        } while (buflen < 79);
        if (buflen == 0) // empty line ignore
        {
            if (!c)
                return; // Special case \n0
            continue;
        }
        buf[buflen] = 0;
        // Send command into command buffer
        if (code.parseAscii((char*)buf, false) && (code.params & 518)) // Success
        {
            Commands::executeGCode(&code);
            Commands::checkForPeriodicalActions(Processing);
        }
    } while (c);
} // executeFString

/** \brief Execute commands in stored string. Multiple commands are seperated by \n */
void GCode::executeString(char* cmd) {
    char buf[80];
    uint8_t buflen;
    char c = 0;
    GCode code;

    do {
        // Wait for a free place in command buffer
        // Scan next command from string
        uint8_t comment = 0;
        buflen = 0;
        do {
            c = *cmd;
            cmd++;
            if (c == 0 || c == '\n')
                break;
            if (c == ';')
                comment = 1;
            if (comment)
                continue;
            buf[buflen++] = c;

        } while (buflen < 79);

        if (buflen == 0) // empty line ignore
        {
            if (!c)
                return; // Special case \n0
            continue;
        }

        // Send command into command buffer
        buf[buflen] = 0;
        if (code.parseAscii((char*)buf, false) && (code.params & 518)) // Success
        {
            Commands::executeGCode(&code);
            Commands::checkForPeriodicalActions(Processing);
        }
    } while (c);
} // executeString

// Receive one complete command from the USB serial port per call.
void GCode::readFromSerial() {
    if (bufferLength >= GCODE_BUFFER_SIZE || (waitUntilAllCommandsAreParsed && bufferLength)) {
        keepAlive(Processing);
        return;
    }
    waitUntilAllCommandsAreParsed = false;
    const millis_t now = HAL::timeInMilliseconds();

    if (!HAL::serialByteAvailable()) {
        const millis_t idleTime = now - serialReceive.timeOfLastDataPacket;
        if ((serialReceive.waitingForResend >= 0 || commandsReceivingWritePosition > 0) && idleTime > 200) {
            requestResend();
            serialReceive.timeOfLastDataPacket = now;
        } else if (bufferLength == 0 && idleTime > 1000) {
            // Unblock the host if an acknowledgement was lost.
            Com::printFLN(Com::tWait);
            serialReceive.timeOfLastDataPacket = now;
        }
    }

    while (HAL::serialByteAvailable() && commandsReceivingWritePosition < MAX_CMD_SIZE) {
        serialReceive.timeOfLastDataPacket = now;
        commandReceiving[commandsReceivingWritePosition++] = HAL::serialReadByte();
        if (commandsReceivingWritePosition == 1 && !commentDetected) {
            if (serialReceive.waitingForResend >= 0 && serialReceive.lastCommandWasBinary) {
                if (!commandReceiving[0])
                    --serialReceive.waitingForResend; // Skip 30 zeros to regain frame sync.
                else
                    serialReceive.waitingForResend = 30;
                commandsReceivingWritePosition = 0;
                continue;
            }
            if (!commandReceiving[0]) {
                commandsReceivingWritePosition = 0;
                return;
            }
            sendAsBinary = (commandReceiving[0] & 128) != 0;
        }

        GCode& command = commandsBuffered[bufferWriteIndex];
        if (sendAsBinary) {
            if (commandsReceivingWritePosition < 2)
                continue;
            if (commandsReceivingWritePosition == 4 || commandsReceivingWritePosition == 5)
                binaryCommandSize = computeBinarySize((char*)commandReceiving);
            if (commandsReceivingWritePosition == binaryCommandSize) {
                if (command.parseBinary(commandReceiving, binaryCommandSize))
                    command.checkAndPushCommand();
                else
                    requestResend();
                commandsReceivingWritePosition = 0;
                return;
            }
        } else {
            const char ch = commandReceiving[commandsReceivingWritePosition - 1];
            if (ch == 0 || ch == '\n' || ch == '\r') {
                commandReceiving[commandsReceivingWritePosition - 1] = 0;
                commentDetected = false;
                if (commandsReceivingWritePosition == 1) {
                    commandsReceivingWritePosition = 0;
                    continue;
                }
                if (command.parseAscii((char*)commandReceiving, true))
                    command.checkAndPushCommand();
                else
                    requestResend();
                commandsReceivingWritePosition = 0;
                return;
            }
            if (ch == ';')
                commentDetected = true;
            if (commentDetected)
                --commandsReceivingWritePosition;
        }

        if (commandsReceivingWritePosition == MAX_CMD_SIZE)
            requestResend();
    }
} // readFromSerial

/** \brief Converts a binary uint8_tfield containing one GCode line into a GCode structure.
    Returns true if checksum was correct. */

bool GCode::parseBinary(uint8_t* buffer, fast8_t length) {
    internalCommand = false;
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
        if (Printer::debugErrors()) {
            Com::printErrorFLN(Com::tWrongChecksum);
        }
        return false;
    }

    p = buffer;
    params = *(uint16_t*)p;
    p += 2;
    uint8_t textlen = 16;
    if (isV2()) {
        params2 = *(uint16_t*)p;
        p += 2;
        if (hasString())
            textlen = *p++;
    } else
        params2 = 0;

    if (params & 1) {
        actLineNumber = N = *(uint16_t*)p;
        p += 2;
    }
    if (isV2()) // Read G,M as 16 bit value
    {
        if (hasM()) {
            M = *(uint16_t*)p;
            p += 2;
        }
        if (hasG()) {
            G = *(uint16_t*)p;
            p += 2;
        }
    } else {
        if (hasM()) {
            M = *p++;
        }
        if (hasG()) {
            G = *p++;
        }
    }

    // Special behaviour for text Gcodes for RFx000, they have no additional switches - just text.
    if (hasM() && (M == 117 || M == 3117)) {
        if (hasString()) // set text pointer to string
        {
            text = (char*)p;
            text[textlen] = 0;                    // Terminate string overwriting checksum
            waitUntilAllCommandsAreParsed = true; // Don't destroy string until executed
        }
        formatErrors = 0;
        return true;
    }

    if (hasX()) {
        X = *(float*)p;
        p += 4;
    }
    if (hasY()) {
        Y = *(float*)p;
        p += 4;
    }
    if (hasZ()) {
        Z = *(float*)p;
        p += 4;
    }
    if (hasE()) {
        E = *(float*)p;
        p += 4;
    }
    if (hasF()) {
        F = *(float*)p;
        p += 4;
    }
    if (hasT()) {
        T = *p++;
    }
    if (hasS()) {
        S = *(int32_t*)p;
        p += 4;
    }
    if (hasP()) {
        P = *(int32_t*)p;
        p += 4;
    }
    if (hasI()) {
        I = *(float*)p;
        p += 4;
    }
    if (hasJ()) {
        J = *(float*)p;
        p += 4;
    }
    if (hasR()) {
        R = *(float*)p;
        p += 4;
    }
    if (hasD()) {
        D = *(float*)p;
        p += 4;
    }
    if (hasC()) {
        C = *(float*)p;
        p += 4;
    }
    if (hasH()) {
        H = *(float*)p;
        p += 4;
    }
    if (hasA()) {
        A = *(float*)p;
        p += 4;
    }
    if (hasB()) {
        B = *(float*)p;
        p += 4;
    }
    if (hasK()) {
        K = *(float*)p;
        p += 4;
    }
    if (hasL()) {
        L = *(float*)p;
        p += 4;
    }
    if (hasO()) {
        O = *(float*)p;
        p += 4;
    }
    if (hasString()) // set text pointer to string
    {
        text = (char*)p;
        text[textlen] = 0;                    // Terminate string overwriting checksum
        waitUntilAllCommandsAreParsed = true; // Don't destroy string until executed
    }
    formatErrors = 0;
    return true;
} // parseBinary

/**
  Converts a ASCII GCode line into a GCode structure.
*/
bool GCode::parseAscii(char* line, bool fromSerial) {
    char* pos = line;
    params = 0;
    params2 = 0;
    internalCommand = !fromSerial;
    bool hasChecksum = false;
    char c;
    while ((c = *(pos++))) {
        if (c == '(' || c == '%')
            break; // alternative comment or program block
        switch (c) {
        case 'N':
        case 'n': {
            if (hasG() || hasM())
                break;
            actLineNumber = parseLongValue(pos);
            params |= 1;
            N = actLineNumber;
            break;
        }
        case 'G':
        case 'g': {
            // A line contains one G or M command. Later letters may be arguments.
            if (hasG() || hasM())
                break;
            G = parseLongValue(pos) & 0xffff;
            params |= 4;
            break;
        }
        case 'M':
        case 'm': {
            if (hasG() || hasM())
                break;
            M = parseLongValue(pos) & 0xffff;
            params |= 2;
            // handle non standard text arguments that some M codes have
            if (M == 117 || M == 3117) {
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
                text = pos;
                while (*pos && *pos != '*')
                    pos++;
                *pos = 0;
                waitUntilAllCommandsAreParsed = true; // don't risk string be deleted
                params |= 32768;
            }
            break;
        }
        case 'X':
        case 'x': {
            X = parseFloatValue(pos);
            params |= 8;
            break;
        }
        case 'Y':
        case 'y': {
            Y = parseFloatValue(pos);
            params |= 16;
            break;
        }
        case 'Z':
        case 'z': {
            Z = parseFloatValue(pos);
            params |= 32;
            break;
        }
        case 'E':
        case 'e': {
            E = parseFloatValue(pos);
            params |= 64;
            break;
        }
        case 'F':
        case 'f': {
            F = parseFloatValue(pos);
            params |= 256;
            break;
        }
        case 'T':
        case 't': {
            T = parseLongValue(pos) & 0xff;
            params |= 512;
            break;
        }
        case 'S':
        case 's': {
            S = parseLongValue(pos);
            params |= 1024;
            break;
        }
        case 'P':
        case 'p': {
            P = parseLongValue(pos);
            params |= 2048;
            break;
        }
        case 'I':
        case 'i': {
            I = parseFloatValue(pos);
            params2 |= 1;
            break;
        }
        case 'J':
        case 'j': {
            J = parseFloatValue(pos);
            params2 |= 2;
            break;
        }
        case 'R':
        case 'r': {
            R = parseFloatValue(pos);
            params2 |= 4;
            break;
        }
        case 'D':
        case 'd': {
            D = parseFloatValue(pos);
            params2 |= 8;
            break;
        }
        case 'C':
        case 'c': {
            C = parseFloatValue(pos);
            params2 |= 16;
            break;
        }
        case 'H':
        case 'h': {
            H = parseFloatValue(pos);
            params2 |= 32;
            break;
        }
        case 'A':
        case 'a': {
            A = parseFloatValue(pos);
            params2 |= 64;
            break;
        }
        case 'B':
        case 'b': {
            B = parseFloatValue(pos);
            params2 |= 128;
            break;
        }
        case 'K':
        case 'k': {
            K = parseFloatValue(pos);
            params2 |= 256;
            break;
        }
        case 'L':
        case 'l': {
            L = parseFloatValue(pos);
            params2 |= 512;
            break;
        }
        case 'O':
        case 'o': {
            O = parseFloatValue(pos);
            params2 |= 1024;
            break;
        }
        case '*': //checksum
        {
            uint8_t checksum_given = parseLongValue(pos);
            uint8_t checksum = 0;
            while (line != (pos - 1))
                checksum ^= *line++;
            if (checksum != checksum_given) {
                Com::printErrorFLN(Com::tWrongChecksum);
                GCode::outputGCommand();
                return false; // mismatch
            }
            hasChecksum = true;
            break;
        }
        default:
            break;
        } // end switch
    }     // end while
    if (serialReceive.lastCommandWasBinary && !hasChecksum && fromSerial && !waitUntilAllCommandsAreParsed) {
        Com::printErrorFLN(PSTR("Checksum required when switching back to ASCII protocol."));
        return false;
    }
    if (hasFormatError() /*|| (params & 518) == 0*/) // Must contain G, M or T command and parameter need to have variables!
    {
        formatErrors++;
        if (Printer::debugErrors())
            Com::printErrorFLN(Com::tFormatError);
        if (formatErrors < 3)
            return false;
    } else
        formatErrors = 0;
    return true;
}

/** \brief Print command on serial console */
void GCode::printCommand() {
    if (hasN()) {
        Com::print('N');
        Com::print(int32_t(N)); //65536 wäre max, nicht 32768 -> sonst sind N-xxx in der Log
        Com::print(' ');
    }
    if (hasM()) {
        Com::print('M');
        Com::print(int32_t(M));
        Com::print(' ');
    }
    if (hasG()) {
        Com::print('G');
        Com::print(int32_t(G));
        Com::print(' ');
    }
    if (hasT()) {
        Com::print('T');
        Com::print((int)T);
        Com::print(' ');
    }
    if (hasX()) {
        Com::printF(Com::tX, X, 3);
    }
    if (hasY()) {
        Com::printF(Com::tY, Y, 3);
    }
    if (hasZ()) {
        Com::printF(Com::tZ, Z, 3);
    }
    if (hasE()) {
        Com::printF(Com::tE, E, 4);
    }
    if (hasF()) {
        Com::printF(Com::tF, F, 3);
    }
    if (hasS()) {
        Com::printF(Com::tS, S);
    }
    if (hasP()) {
        Com::printF(Com::tP, P);
    }
    if (hasI()) {
        Com::printF(Com::tI, I, 3);
    }
    if (hasJ()) {
        Com::printF(Com::tJ, J, 3);
    }
    if (hasR()) {
        Com::printF(Com::tR, R, 3);
    }
    if (hasD()) {
        Com::printF(Com::tD, D, 3);
    }
    if (hasC()) {
        Com::printF(Com::tC, C, 3);
    }
    if (hasH()) {
        Com::printF(Com::tH, H, 3);
    }
    if (hasA()) {
        Com::printF(Com::tA, A, 3);
    }
    if (hasB()) {
        Com::printF(Com::tB, B, 3);
    }
    if (hasK()) {
        Com::printF(Com::tK, K, 3);
    }
    if (hasL()) {
        Com::printF(Com::tL, L, 3);
    }
    if (hasO()) {
        Com::printF(Com::tO, O, 3);
    }
    if (hasString()) {
        Com::print(text);
    }
    Com::println();

} // printCommand

void GCode::reportFatalError() {
    Com::printF(Com::tFatal);
    Com::printF(fatalErrorMsg);
    Com::printFLN(PSTR(" - Printer stopped and heaters disabled due to this error. Fix error and restart."));
    UI_ERROR_P(fatalErrorMsg)
}

void GCode::outputGCommand() {
    Com::printF(PSTR("Corrupted: "));
    if ((int)sendAsBinary) {
        GCode::printCommand();
    } else {
        Com::print((char*)commandReceiving);
        Com::println();
    }
} // outputGCommand

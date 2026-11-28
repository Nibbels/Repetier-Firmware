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

#ifndef USB_SERIAL_H
#define USB_SERIAL_H

#include <stdint.h>

// The USB bridge on the RFx000 boards is wired to USART0.
class UsbSerial {
    static constexpr uint8_t receiveBufferSize = 128;
    static constexpr uint8_t transmitBufferSize = 64;

public:
    static constexpr uint8_t receiveCapacity = receiveBufferSize - 1;

    static void begin(unsigned long baud);
    static bool available();
    static int read();
    static void write(uint8_t byte);
    // Single foreground producer, like write(). Never wait for TX space.
    // False means no byte was accepted; retry from the same log cursor.
    static bool tryWrite(uint8_t byte);
    // Whole short diagnostic record or nothing. No partially published line.
    static bool tryWriteSpan(const char* bytes, uint8_t count);
    static uint8_t transmitFree(); // snapshot; only ISR consumes
    static void flush();

    static void receiveInterrupt();
    static void transmitInterrupt();

private:
    template <uint8_t Size>
    struct Ring {
        uint8_t bytes[Size];
        volatile uint8_t head;
        volatile uint8_t tail;

        static uint8_t next(uint8_t index) {
            return (index + 1) & (Size - 1);
        }
    };

    static Ring<receiveBufferSize> rx_;
    static Ring<transmitBufferSize> tx_;
};

#endif

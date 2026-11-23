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

/*
  HardwareSerial.h - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 28 September 2010 by Mark Sproul

  Modified to use only 1 queue with fixed length by Repetier
*/

#include "UsbSerial.h"
#include <avr/io.h>
#include <avr/interrupt.h>

UsbSerial::Ring<UsbSerial::receiveBufferSize> UsbSerial::rx_;
UsbSerial::Ring<UsbSerial::transmitBufferSize> UsbSerial::tx_;

void UsbSerial::begin(unsigned long baud) {
    bool doubleSpeed = true;
#if F_CPU == 16000000UL
    // Preserve 57600 baud with the settings used by the USB bridge and bootloader.
    doubleSpeed = baud != 57600;
#endif

    uint16_t divisor;
    if (doubleSpeed) {
        divisor = (F_CPU / 4 / baud - 1) / 2;
        doubleSpeed = divisor <= 4095;
    }
    if (!doubleSpeed)
        divisor = (F_CPU / 8 / baud - 1) / 2;

    UCSR0A = doubleSpeed ? _BV(U2X0) : 0;
    UBRR0H = divisor >> 8;
    UBRR0L = divisor;
    UCSR0B |= _BV(RXEN0);
    UCSR0B |= _BV(TXEN0);
    UCSR0B |= _BV(RXCIE0);
    UCSR0B &= ~_BV(UDRIE0);
}

bool UsbSerial::available() {
    return rx_.head != rx_.tail;
}

int UsbSerial::read() {
    if (!available())
        return -1;
    const uint8_t byte = rx_.bytes[rx_.tail];
    rx_.tail = rx_.next(rx_.tail);
    return byte;
}

void UsbSerial::write(uint8_t byte) {
    const uint8_t next = tx_.next(tx_.head);
    while (next == tx_.tail) {
        // The transmit interrupt makes room in the ring.
    }
    tx_.bytes[tx_.head] = byte;
    tx_.head = next;
    UCSR0B |= _BV(UDRIE0);
}

void UsbSerial::flush() {
    while (tx_.head != tx_.tail) {
        // Wait for the interrupt to hand the queued bytes to the UART.
    }
}

inline void UsbSerial::receiveInterrupt() {
    const uint8_t byte = UDR0;
    const uint8_t next = rx_.next(rx_.head);
    if (next != rx_.tail) {
        rx_.bytes[rx_.head] = byte;
        rx_.head = next;
    }
}

inline void UsbSerial::transmitInterrupt() {
    if (tx_.head == tx_.tail) {
        UCSR0B &= ~_BV(UDRIE0);
        return;
    }
    const uint8_t byte = tx_.bytes[tx_.tail];
    tx_.tail = tx_.next(tx_.tail);
    UDR0 = byte;
}

ISR(USART0_RX_vect) {
    UsbSerial::receiveInterrupt();
}

ISR(USART0_UDRE_vect) {
    UsbSerial::transmitInterrupt();
}

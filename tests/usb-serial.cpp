#include <cassert>
#include <cstdint>
#include <cstdio>

#define F_CPU 16000000UL
#define _BV(bit) (1U << (bit))
#define ISR(vector) void vector()
enum { U2X0 = 1, TXEN0 = 3, RXEN0 = 4, UDRIE0 = 5, RXCIE0 = 7 };
volatile uint8_t UCSR0A, UCSR0B, UBRR0H, UBRR0L, UDR0;

#include "usb-serial-firmware.inc"

namespace {

void checkBaud(unsigned long baud, bool doubleSpeed, unsigned divisor) {
    UCSR0B = _BV(UDRIE0) | 1;
    UsbSerial::begin(baud);
    assert(UCSR0A == (doubleSpeed ? _BV(U2X0) : 0));
    assert((unsigned(UBRR0H) << 8 | UBRR0L) == divisor);
    assert(UCSR0B == (_BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0) | 1));
}

void receive() {
    assert(!UsbSerial::available() && UsbSerial::read() == -1);
    for (unsigned byte = 0; byte <= UsbSerial::receiveCapacity; ++byte) {
        UDR0 = byte;
        USART0_RX_vect();
    }
    for (int byte = 0; byte < 80; ++byte)
        assert(UsbSerial::available() && UsbSerial::read() == byte);

    for (int byte = 160; byte < 240; ++byte) {
        UDR0 = byte;
        USART0_RX_vect();
    }
    for (int byte = 80; byte < UsbSerial::receiveCapacity; ++byte)
        assert(UsbSerial::read() == byte);
    for (int byte = 160; byte < 240; ++byte)
        assert(UsbSerial::read() == byte);
    assert(!UsbSerial::available() && UsbSerial::read() == -1);
}

void transmit(uint8_t firstByte, int count) {
    assert(!(UCSR0B & _BV(UDRIE0)));
    for (int index = 0; index < count; ++index)
        UsbSerial::write(firstByte + index);
    assert(UCSR0B & _BV(UDRIE0));
    for (int index = 0; index < count; ++index) {
        USART0_UDRE_vect();
        assert(UDR0 == firstByte + index);
    }
    UsbSerial::flush();
    USART0_UDRE_vect();
    assert(!(UCSR0B & _BV(UDRIE0)) && (UCSR0B & _BV(TXEN0)));
}

} // namespace

int main() {
    checkBaud(57600, false, 16);
    checkBaud(300, false, 3332);
    checkBaud(115200, true, 16);
    receive();
    transmit(0, 63);
    transmit(100, 40);
    std::puts("USART0 baud and ring replay passed");
}

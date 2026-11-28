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

// Every ring start position, full-buffer rejection, partial drain and
// resumption. A rejected diagnostic byte must not overwrite queued traffic.
void nonblockingTransmit() {
    for (unsigned phase = 0; phase < 64; ++phase) {
        assert(!(UCSR0B & _BV(UDRIE0)));
        for (unsigned i = 0; i < 63; ++i)
            assert(UsbSerial::tryWrite(uint8_t(i)));
        assert(UCSR0B & _BV(UDRIE0));
        for (unsigned i = 0; i < 128; ++i)
            assert(!UsbSerial::tryWrite(0xfe));
        USART0_UDRE_vect();
        assert(UDR0 == 0);
        assert(UsbSerial::tryWrite(0xa5));
        assert(!UsbSerial::tryWrite(0xfe));
        for (unsigned i = 1; i < 63; ++i) {
            USART0_UDRE_vect();
            assert(UDR0 == i);
        }
        USART0_UDRE_vect();
        assert(UDR0 == 0xa5);
        USART0_UDRE_vect();
        assert(!(UCSR0B & _BV(UDRIE0)));
        // Move the empty-ring cursor by one between rounds.
        assert(UsbSerial::tryWrite(0x5a));
        USART0_UDRE_vect();
        assert(UDR0 == 0x5a);
        USART0_UDRE_vect();
    }
}

void nonblockingSpan() {
    char span[64];
    for (unsigned i = 0; i < sizeof(span); ++i) span[i] = char(0x80 + i);
    assert(UsbSerial::tryWriteSpan(nullptr, 0));
    assert(!UsbSerial::tryWriteSpan(nullptr, 1));
    assert(!UsbSerial::tryWriteSpan(span, 64));
    // Exercise every start cursor, every occupancy, every attempted size.
    for (unsigned phase = 0; phase < 64; ++phase) {
        for (unsigned occupancy = 0; occupancy < 64; ++occupancy) {
            for (unsigned size = 0; size < 64; ++size) {
                for (unsigned i = 0; i < occupancy; ++i)
                    assert(UsbSerial::tryWrite(uint8_t(i)));
                const bool fits = occupancy + size <= 63;
                assert(UsbSerial::tryWriteSpan(span, uint8_t(size)) == fits);
                for (unsigned i = 0; i < occupancy; ++i) {
                    USART0_UDRE_vect();
                    assert(UDR0 == i);
                }
                if (fits) for (unsigned i = 0; i < size; ++i) {
                    USART0_UDRE_vect();
                    assert(UDR0 == uint8_t(span[i]));
                }
                USART0_UDRE_vect();
                assert(!(UCSR0B & _BV(UDRIE0)));
                // Return to the same start cursor (empty ring).
                const unsigned consumed = occupancy + (fits ? size : 0);
                for (unsigned i = consumed; i < 64; ++i) {
                    assert(UsbSerial::tryWrite(0));
                    USART0_UDRE_vect();
                }
                USART0_UDRE_vect();
            }
        }
        assert(UsbSerial::tryWrite(0));
        USART0_UDRE_vect();
        USART0_UDRE_vect();
    }
}

} // namespace

int main() {
    checkBaud(57600, false, 16);
    checkBaud(300, false, 3332);
    checkBaud(115200, true, 16);
    receive();
    transmit(0, 63);
    transmit(100, 40);
    nonblockingTransmit();
    nonblockingSpan();
    std::puts("USART0 baud and ring replay passed");
}

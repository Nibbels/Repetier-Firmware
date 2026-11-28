#!/usr/bin/env python3
"""Task-private fixed application UART startup candidate; no production edits.

Uses actual UsbSerial source and existing ring replay in a temporary build.
Fake registers do not model hardware flags, in-flight bytes, IRQ arbitration
or bootloader. Candidate is boot-only, not a live reinitialization API.
"""
from pathlib import Path
from tempfile import TemporaryDirectory
import os
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]

CANDIDATE = """void UsbSerial::begin() {
    static_assert(F_CPU == 16000000UL, "RFx000 application UART clock");
    UCSR0B = 0; // USART0 exclusively owned; disable receiver/TX and IRQs
    UCSR0A = 0; // normal 16x sampling
    UBRR0H = 0;
    UBRR0L = 3; // fixed 250000 baud
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // async, no parity, 1 stop, 8 bits
    // Cold application start only, before any accepted command.
    rx_.head = rx_.tail = 0;
    tx_.head = tx_.tail = 0;
    asm volatile("" ::: "memory");
    UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);
}

"""

CHECK = """void checkFixedStartup() {
    // Dirty configuration from predecessor; not a live UART restart test.
    UCSR0A = UCSR0B = UCSR0C = UBRR0H = UBRR0L = 255;
    UsbSerial::begin();
    assert(UCSR0A == 0 && UBRR0H == 0 && UBRR0L == 3);
    assert(UCSR0C == (_BV(UCSZ01) | _BV(UCSZ00)));
    assert(UCSR0B == (_BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0)));
    assert(!UsbSerial::available() && UsbSerial::read() == -1);
    USART0_UDRE_vect();
    assert(!(UCSR0B & _BV(UDRIE0)));
}

"""

def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError("source anchor changed: " + old)
    return text.replace(old, new)


def main():
    with TemporaryDirectory(prefix="task0032-fixed-init-") as directory:
        build = Path(directory)
        header = (ROOT / "Repetier/UsbSerial.h").read_text()
        header = replace_once(header, "static void begin(unsigned long baud);", "static void begin();")
        (build / "UsbSerial.h").write_text(header)
        source = (ROOT / "Repetier/UsbSerial.cpp").read_text()
        start = source.index("void UsbSerial::begin(unsigned long baud)")
        end = source.index("bool UsbSerial::available()", start)
        source = source[:start] + CANDIDATE + source[end:]
        source = re.sub(r"^#include <avr/\w+\.h>\n", "", source, flags=re.M)
        (build / "usb-serial-firmware.inc").write_text(source)
        test = (ROOT / "tests/usb-serial.cpp").read_text()
        test = replace_once(test, "volatile uint8_t UCSR0A, UCSR0B,", "enum { UCSZ00 = 1, UCSZ01 = 2 };\nvolatile uint8_t UCSR0C, UCSR0A, UCSR0B,")
        start = test.index("void checkBaud(")
        end = test.index("void receive()", start)
        test = test[:start] + CHECK + test[end:]
        test = replace_once(test, "    checkBaud(57600, false, 16);\n    checkBaud(300, false, 3332);\n    checkBaud(115200, true, 16);", "    checkFixedStartup();")
        (build / "test.cpp").write_text(test)
        exe = build / "replay"
        subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror", str(build / "test.cpp"), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True, timeout=10)
        print("Fixed 250000 baud startup candidate and existing RX/TX ring replay passed")


if __name__ == "__main__":
    main()

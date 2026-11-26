#!/usr/bin/env python3
"""Optional pinned-AVR linked probe for research-only Timer3 lease adapter."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
cli = shutil.which("arduino-cli")
if not cli:
    print("AVR probe skipped: arduino-cli not installed; host-only environment")
    raise SystemExit(0)
props = subprocess.check_output([cli, "compile", "--fqbn", "arduino:avr:mega:cpu=atmega2560",
                                 "--show-properties=expanded", str(root / "Repetier")], text=True)
properties = dict(s.split("=", 1) for s in props.splitlines() if "=" in s)
bindir = Path(properties["compiler.path"])
suffix = ".exe" if os.name == "nt" else ""
cxx = bindir / ("avr-g++" + suffix)
objdump = bindir / ("avr-objdump" + suffix)
code = r"""
#include <avr/interrupt.h>
#include "HeaterLease.h"
#include "AvrTimer3Lease.h"
// Dummy GPIO, not production heater mapping; no executable hardware test.
static void off(uint8_t c) { PORTA &= uint8_t(~_BV(c)); }
using Clock = AvrTimer3Lease<off>;
static Clock clock;
static HeaterLease<Clock,3,12500,3> lease(clock);
volatile uint8_t channel, demand, result;
volatile uint8_t frameResult[3];
extern "C" __attribute__((noinline,used,externally_visible)) void probeGrant() {
    const uint8_t sreg = SREG; cli();
    result=lease.authorize(channel,demand);
    SREG=sreg;
}
extern "C" __attribute__((noinline,used,externally_visible)) void probeFrame() {
    const uint8_t sreg=SREG; cli();
    uint8_t duties[3];
    lease.frameDuties(duties);
    for (uint8_t c=0;c<3;++c) frameResult[c]=duties[c];
    SREG=sreg;
}
ISR(TIMER3_COMPA_vect) {lease.expired(0);}
ISR(TIMER3_COMPB_vect) {lease.expired(1);}
ISR(TIMER3_COMPC_vect) {lease.expired(2);}
int main() {
    cli(); Clock::begin(); sei();
    for (;;) {probeGrant();probeFrame();}
}
"""
with tempfile.TemporaryDirectory() as temp:
    cpp = Path(temp) / "probe.cpp"
    elf = Path(temp) / "probe.elf"
    cpp.write_text(code)
    subprocess.run([str(cxx), "-mmcu=atmega2560", "-DF_CPU=16000000UL", "-std=gnu++11",
                    "-Os", "-flto", "-Wall", "-Wextra", "-Werror",
                    "-I", str(root / "tools/task0034"), str(cpp), "-o", str(elf)], check=True)
    dump = subprocess.check_output([str(objdump), "-d", "-C", str(elf)], text=True)
    for symbol in ("<probeGrant>:", "<probeFrame>:","<__vector_32>:", "<__vector_33>:", "<__vector_34>:"):
        assert symbol in dump, symbol
    print(dump)
    print("AVR probe linked; no runtime/cycle bound asserted.")

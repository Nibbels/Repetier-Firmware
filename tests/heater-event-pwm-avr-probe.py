#!/usr/bin/env python3
"""Optional pinned-AVR linked probe for research-only combined Timer3/5 heater adapter."""
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
#include "EventHeaterPwm.h"
#include "AvrTimer5Pwm.h"
// Dummy GPIO only, never a hardware experiment.
static void off(uint8_t c) { PORTA &= uint8_t(~_BV(c)); }
static void on(uint8_t c) { PORTA |= uint8_t(_BV(c)); }
using Clock = AvrTimer3Lease<off>;
using Lease = HeaterLease<Clock,3,12500,3>;
using PwmClock = AvrTimer5Pwm<on,off>;
static Clock clock;
static Lease lease(clock);
static PwmClock pwmClock;
static EventHeaterPwm<PwmClock,Lease,3,64> pwm(pwmClock,lease);
volatile uint8_t channel,demand,result;
extern "C" __attribute__((noinline,used,externally_visible)) void probeGrant() {
    const uint8_t sreg=SREG;cli();
    if (demand) result=lease.authorize(channel,demand);
    else {pwm.revoke(channel);result=!pwm.failed();}
    SREG=sreg;
}
ISR(TIMER3_COMPA_vect) {pwm.inhibit();}
ISR(TIMER3_COMPB_vect) {pwm.inhibit();}
ISR(TIMER3_COMPC_vect) {pwm.inhibit();}
ISR(TIMER5_COMPA_vect) {pwm.compare(0);}
ISR(TIMER5_COMPB_vect) {pwm.compare(1);}
ISR(TIMER5_COMPC_vect) {pwm.compare(2);}
ISR(TIMER5_OVF_vect) {pwm.frame();}
int main(){
    cli(); Clock::begin();PwmClock::begin();sei();
    for(;;) probeGrant();
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
    for symbol in ("<probeGrant>:", "<__vector_32>:", "<__vector_33>:", "<__vector_34>:", "<__vector_47>:", "<__vector_48>:", "<__vector_49>:", "<__vector_50>:"):
        assert symbol in dump, symbol
    print(dump)
    print("AVR probe linked; no runtime/cycle bound asserted.")

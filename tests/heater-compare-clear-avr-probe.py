#!/usr/bin/env python3
"""Link the actual Timer2 compare-clear adapter for inspection, never flash."""
from pathlib import Path
import re
import json
import importlib.util
import sys
import shutil
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
cli=shutil.which("arduino-cli")
if not cli:
    print("Compare-clear AVR probe skipped: arduino-cli unavailable")
    raise SystemExit(0)
props=subprocess.check_output([cli,"compile","--fqbn","arduino:avr:mega:cpu=atmega2560",
                              "--show-properties=expanded",str(root/"Repetier")],text=True)
bindir=Path(dict(s.split("=",1) for s in props.splitlines() if "=" in s)["compiler.path"])
code=r"""
#include <avr/interrupt.h>
#include "CompareClearHotends.h"
#include "AvrTimer2CompareClear.h"
static AvrTimer2CompareClear timer;
static CompareClearHotends<AvrTimer2CompareClear,2> pwm(timer);
volatile uint8_t channel,demand,admitted;
extern "C" __attribute__((noinline,used,externally_visible)) void probePulse() {
    const uint8_t sreg=SREG;cli();
    admitted=pwm.pulse(channel,demand);
    SREG=sreg;
}
extern "C" __attribute__((noinline,used,externally_visible)) void probeRevoke() {
    const uint8_t sreg=SREG;cli();
    pwm.revokeAll();
    SREG=sreg;
}
int main(){
    cli();
    if(!timer.begin())for(;;){}
    probeRevoke();
    sei();
    for(;;) probePulse();
}
"""
with tempfile.TemporaryDirectory() as temp:
    cpp=Path(temp)/"probe.cpp";elf=Path(temp)/"probe.elf"
    cpp.write_text(code)
    subprocess.run([str(bindir/"avr-g++"),"-mmcu=atmega2560","-DF_CPU=16000000UL",
                    "-std=gnu++11","-Os","-flto","-Wall","-Wextra","-Werror",
                    "-I",str(root/"tools/task0034"),str(cpp),"-o",str(elf)],check=True)
    dump=subprocess.check_output([str(bindir/"avr-objdump"),"-d","-C",str(elf)],text=True)
    symbols=subprocess.check_output([str(bindir/"avr-nm"),str(elf)],text=True)
    vectors={int(m[2]):int(m[1],16) for m in re.finditer(
        r"^([0-9a-f]+)\s+[TtWw]\s+__vector_(\d+)$",symbols,re.M)}
    # Timer2 vectors 13/14/15 remain the default handler, not a falling-edge ISR.
    assert vectors[13]==vectors[14]==vectors[15]==vectors[2]
    for name in ("<probePulse>:","<probeRevoke>:"):
        assert name in dump,name
    sys.dont_write_bytecode=True
    spec=importlib.util.spec_from_file_location("arm_bound",root/"tools/task0034/compare-arm-bound.py")
    bound=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bound)
    print("Linked Timer2 arm bound",json.dumps(bound.audit(dump)),flush=True)
    print(dump)
    print("Timer2 compare-clear linked; no Timer2 interrupt handlers. No physical or WCET claim.")

#!/usr/bin/env python3
"""Compile/link the standalone diagnostic byte writer with the hosted AVR toolchain.
Not a firmware build or physical EEPROM timing test.
"""
from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
cli=shutil.which("arduino-cli")
if not cli:
    if os.environ.get("GITHUB_ACTIONS")=="true":
        raise RuntimeError("Hosted Arduino AVR toolchain must be installed before replays")
    print("Optional local AVR probe skipped; mandatory on GitHub Actions")
    raise SystemExit(0)

props=subprocess.check_output([cli,"compile","--fqbn","arduino:avr:mega:cpu=atmega2560",
                               "--show-properties=expanded",str(root/"Repetier")],text=True)
bin_dir=None
for line in props.splitlines():
    if line.startswith("compiler.path="):
        bin_dir=Path(line.split("=",1)[1])
        break
if bin_dir is None:
    raise RuntimeError("Missing expanded compiler.path")
def tool(name):
    return str(bin_dir/(name+(".exe" if os.name=="nt" else "")))

source=r"""
#include "AvrEventStore.h"
#include "EventLogRead.h"
using namespace EventLog;
struct Owner {
    bool logMayAccess()const{return (PINB&1)!=0;}
};
Ring<16> ring;
Payload payload;
Reader<16> reader;
Owner owner;
AvrStore<0x0E00,16,Owner> store(owner); // probe-only example, NOT ABI allocation
struct Sink {
    bool tryEntry(const EntryView& v){
        PORTB=uint8_t(v.sequence()^uint32_t(v.value(0))^v.reason());return true;
    }
};
int main(){
    payload.id=eventId(Channel::Bed,6);
    payload.uptime=PINB;payload.values[0]=1234;
    Sink sink;
    for(;;){
        if(!ring.initialized())ring.scanStep(store);
        else if(ring.pending())ring.step(store,payload);
        else if(PINB&2)ring.begin(payload);
        else if(PINB&4)reader.begin(ring);
        else reader.step(ring,store,sink);
    }
}
"""
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"event-log-avr.cpp"
    elf=Path(d)/"event-log-avr.elf"
    p.write_text(source)
    subprocess.run([tool("avr-g++"),"-mmcu=atmega2560","-std=gnu++11","-Os",
                    "-Wall","-Wextra","-Werror","-fstack-usage","-ffunction-sections",
                    "-fdata-sections","-Wl,--gc-sections","-I",str(root/"tools/task0034/event-log"),
                    str(p),"-o",str(elf)],check=True)
    asm=subprocess.check_output([tool("avr-objdump"),"-d","-C",str(elf)],text=True)
    instructions=[line for line in asm.splitlines() if re.match(r"^\s*[0-9a-f]+:",line)]
    launches=[]
    for i,line in enumerate(instructions[:-1]):
        if re.search(r"\bsbi\s+0x1f,\s*2\b",line,re.I):
            launches.append((line,instructions[i+1]))
            assert re.search(r"\bsbi\s+0x1f,\s*1\b",instructions[i+1],re.I),launches[-1]
    assert launches,"EEMPE->EEPE sequence absent from linked AVR probe"
    for pair in launches:print("\n".join(pair))
    print(subprocess.check_output([tool("avr-size"),str(elf)],text=True))
    print(subprocess.check_output([tool("avr-nm"),"-S","--size-sort","-C",str(elf)],text=True))
    for f in Path(d).glob("*.su"):
        print(f.read_text())
    print("Standalone event-log AVR compile/link and adjacent EEMPE/EEPE launch PASS")

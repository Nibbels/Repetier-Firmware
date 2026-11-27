#!/usr/bin/env python3
"""Find two linked PORTK RMW sequences and replay an interrupt between them."""
import argparse
import re
from pathlib import Path

SYMBOL = re.compile(r"^[0-9a-f]+ <(.*)>:$")
INSTRUCTION = re.compile(r"^\s*([0-9a-f]+):\s+(?:[0-9a-f]{2} )+\s*(\w+)\s*([^;]*)")


def instructions(path):
    name = ""
    groups = {}
    for line in path.read_text().splitlines():
        symbol = SYMBOL.match(line)
        if symbol:
            name = symbol[1]
            groups[name] = []
        op = INSTRUCTION.match(line)
        if op and name:
            groups[name].append((op[1], op[2], op[3].strip().lower()))
    return groups


def window(groups, name, mask):
    ops = next(v for k, v in groups.items() if k.startswith(name))
    for i in range(len(ops) - 2):
        read, modify, write = ops[i:i+3]
        reg = read[2].split(",")[0]
        if (read[1:] == ("lds", f"{reg}, 0x0108") and
            modify[1:] == ("ori", f"{reg}, 0x{mask:02x}") and
            write[1:] == ("sts", f"0x0108, {reg}")):
            if name.startswith("lcdWriteByte") and any(
                    entry[1] in ("cli", "call", "rcall", "icall", "eicall")
                    or (entry[1] == "out" and entry[2].startswith("0x3f,"))
                    for entry in ops[:i]):
                raise ValueError("interrupt context changed before LCD write; inspect control flow")
            return read[0], write[0], int(modify[2].split(",")[1], 16)
    raise ValueError(f"re-audit {name}: expected PORTK sequence missing")


def replay(path):
    groups = instructions(path)
    lcd = window(groups, "lcdWriteByte(", 8)
    step = window(groups, "StepExecutor::enableSteppers(", 1)
    # Call context: foreground LCD enable edge, Timer1 admits a new Z move.
    # Both decoded windows use a CPU register across the modify and store.
    # The ISR preserves that register across the interrupt by the AVR ABI.
    lost = 0
    for old in range(256):
        if old & step[2]:
            continue
        expected = old | lcd[2] | step[2]
        for injection in ("before load", "after load", "after ORI", "after store"):
            port = old
            if injection == "before load":
                port |= step[2]
            snapshot = port
            if injection == "after load":
                port |= step[2]
            snapshot |= lcd[2]
            if injection == "after ORI":
                port |= step[2]
            port = snapshot
            if injection == "after store":
                port |= step[2]
            if port != expected:
                assert port == expected & ~step[2]
                lost += 1
    assert lost == 256
    print(f"LCD {lcd[0]}..{lcd[1]}, Z-EN {step[0]}..{step[1]}: {lost}/512 schedules lose PK0")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("disassembly", type=Path, help="avr-objdump -d -C -z output")
    replay(parser.parse_args().disassembly)

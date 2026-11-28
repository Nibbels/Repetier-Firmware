#!/usr/bin/env python3
"""Check straight-line linked RGB RMW windows at both mask sites in Task0034."""
import argparse
import re
from pathlib import Path

arg = argparse.ArgumentParser(description=__doc__)
arg.add_argument("disassembly", type=Path)
args = arg.parse_args()
ops = []
for line in args.disassembly.read_text().splitlines():
    match = re.match(r"^\s*([0-9a-f]+):\s+(?:[0-9a-f]{2} )+\s*(\w+)\s*([^;]*)", line)
    if match:
        ops.append((match[1], match[2], match[3].strip().lower()))
counts = {"set": 0, "init": 0}
cycles = {"cli": 1, "lds": 2, "sts": 2, "andi": 1, "or": 1, "ori": 1,
          "out": 1, "ldi": 1, "ld": 2, "st": 2}
for i, (_, op, arg) in enumerate(ops):
    if op != "andi" or not arg.endswith(", 0xc7"):
        continue
    start = next((j for j in range(i-1, max(-1, i-8), -1) if ops[j][1] == "cli"), None)
    if start is None:
        continue
    # Normalize address loads chosen by LTO. Keep the operands to verify
    # both dataflow and the exact registers accessed inside the guard.
    constants = {}
    normalized = []
    end = start
    while end < len(ops) and end-start < 18:
        _, mnemonic, operands = ops[end]
        if mnemonic not in cycles:
            break
        fields = operands.split(", ")
        if mnemonic == "ldi":
            constants[fields[0]] = int(fields[1], 16)
        elif mnemonic in ("ld", "st"):
            pos = 1 if mnemonic == "ld" else 0
            pair = {"x": 26, "y": 28, "z": 30}.get(fields[pos])
            if pair is None or any(f"r{k}" not in constants for k in (pair,pair+1)):
                break
            addr = constants[f"r{pair}"] + 256*constants[f"r{pair+1}"]
            fields[pos] = f"0x{addr:04x}"
            normalized.append((mnemonic+"s", fields))
        else:
            normalized.append((mnemonic, fields))
        if mnemonic == "out":
            break
        end += 1
    if len(normalized) < 3 or normalized[1][0] != "lds" or normalized[1][1][1] != "0x0102":
        continue
    saved = ops[start-1][2].split(", ")
    assert ops[start-1][1] == "in" and saved[1] == "0x3f"
    reg = normalized[1][1][0]
    assert normalized[2] == ("andi", [reg, "0xc7"])
    assert normalized[-1] == ("out", ["0x3f", saved[0]])
    if normalized[3][0] == "or":
        kind = "set"
        dest, src = normalized[3][1]
        assert reg in (dest, src) and dest != src
        assert normalized[4:-1] == [("sts", ["0x0102",dest])]
    else:
        kind = "init"
        ddr = normalized[4][1][0]
        assert normalized[3:-1] == [("sts", ["0x0102",reg]), ("lds",[ddr,"0x0101"]),
                                    ("ori",[ddr,"0x38"]), ("sts",["0x0101",ddr])]
    counts[kind] += 1
    print(kind, ops[start][0], sum(cycles[m] for _,m,_ in ops[start:end+1]), "cycles CLI through OUT")
assert all(counts.values()), counts

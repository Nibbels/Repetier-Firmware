#!/usr/bin/env python3
"""Count the historical linked USART0 successful RX and nonempty UDRE paths.
Not an emulator: branch decisions are fixed to the reviewed data paths.
"""
import argparse
import hashlib
import json
import re
import zipfile
from pathlib import Path

BOARDS = ("RF1000", "RF2000", "RF2000v2", "RF2000-X44")
EXPECTED = "1ffbe5f295d08ff4c8306900080dbcda873fd839f1f9663cdd589cdadf05d151"


def count(dis, vec):
    ins = {}
    for line in dis.splitlines():
        m = re.match(r"^\s*([0-9a-f]+):\s+((?:[0-9a-f]{2}\s+)+)\s*([a-z]+)\s*(.*)$", line)
        if m:
            ins[int(m[1], 16)] = (len(m[2].split()), m[3], m[4])
    pc = int(re.search(r"^([0-9a-f]+) <__vector_" + str(vec) + r">:", dis, re.M)[1], 16)
    start = pc
    cycles = 8  # ATmega2560: five-cycle IRQ entry, three-cycle vector JMP
    read = None
    ops = []
    # These two reviewed paths are not taken/skipped at BREQ/CPSE respectively.
    for _ in range(100):
        size, op, arg = ins[pc]
        ops.append(op)
        pc += size
        if op in ("breq", "cpse"):
            cycles += 1
        elif op == "rjmp":
            cycles += 2
            pc += int(re.search(r"\.([+-]\d+)", arg)[1])
        elif op == "reti":
            cycles += 5
            break
        elif op in ("push", "pop", "lds", "sts", "ld", "st"):
            cycles += 2
        elif op in ("in", "out", "eor", "subi", "sbci", "andi", "ldi", "cp"):
            cycles += 1
        else:
            raise ValueError((vec, op, arg))
        if op == "lds" and "0x00C6" in arg:
            read = cycles
    else:
        raise ValueError("no RETI")
    assert ops.count("breq") == (vec == 25)
    assert ops.count("cpse") == (vec == 26)
    assert (read is not None) == (vec == 25)
    return {"address": hex(start), "cycles": cycles, "udr_read_complete_cycles": read,
            "instructions": len(ops)}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("final_zip", type=Path)
    args = p.parse_args()
    sha = hashlib.sha256(args.final_zip.read_bytes()).hexdigest()
    if sha != EXPECTED:
        raise ValueError("historical archive hash mismatch")
    result = {}
    with zipfile.ZipFile(args.final_zip) as z:
        for board in BOARDS:
            dis = z.read(board + "/disassembly.txt").decode()
            result[board] = {"rx": count(dis, 25), "tx": count(dis, 26)}
            assert result[board]["rx"]["cycles"] == 73
            assert result[board]["tx"]["cycles"] == 74
    print(json.dumps({"archive_sha256": sha, "boards": result}, indent=2))


if __name__ == "__main__":
    main()

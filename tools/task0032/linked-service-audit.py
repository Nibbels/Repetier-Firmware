#!/usr/bin/env python3
"""Inventory selected linked core/libc/UI symbols in one verified P0 artifact.

Counts are static direct-transfer sightings in sized code symbols, not call
frequency, reachable-path proof, cycle timing, or marginal library savings.
"""
import argparse
import bisect
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import zipfile

BOARDS = ("RF1000", "RF2000", "RF2000v2", "RF2000-X44")
ARCHIVE_SHA256 = "47c6cf810f4da82a8c25024745298c1bf6e4e844d1fa3c225d68ac63a775634f"
TESTED_MERGE = "7b2c180f0eb8c9f8f870dfdc706f213abc0e68c8"
NAMES = ("millis", "micros", "delay", "delayMicroseconds", "digitalWrite", "pinMode",
         "random", "strtod", "strtol", "sqrt", "memcpy_P", "strcpy_P", "strcat_P",
         "strncasecmp_P", "isspace", "__udivmodsi4", "__divmodsi4", "__udivmodhi4",
         "__divsf3", "__mulsf3", "__addsf3", "__subsf3", "malloc", "free", "calloc", "realloc")
UI_PREFIXES = ("UIDisplay::refreshPage(", "UIDisplay::printRow(", "UIDisplay::parse(",
               "UIDisplay::addInt(", "UIDisplay::addLong(", "UIDisplay::addFloat(",
               "UIDisplay::slowAction(", "UIDisplay::changeSetting(", "UIDisplay::finishAction(",
               "UIDisplay::executeAction(", "lcdWriteByte(", "lcdWriteNibble(")


def inventory(z, board, stats):
    def read(name):
        return z.read(board + "/" + name).decode()
    v = json.loads(read("verification.json"))
    assert v == {"status": "verified", "recovery": True}, (board, v)
    elf = z.read(board + "/build/Repetier.ino.elf")
    assert hashlib.sha256(elf).hexdigest() == stats["elf_sha256"], board
    raw = read("symbols.txt")
    code_start = int(re.search(r"^([0-9a-f]+) T __ctors_end$", raw, re.M)[1], 16)
    symbols = {}
    sized_code = {}
    for line in raw.splitlines():
        m = re.fullmatch(r"([0-9a-f]+)(?: ([0-9a-f]+))? ([A-Za-z]) (.+)", line)
        if not m:
            continue
        addr, size, kind, name = m.groups()
        addr, size = int(addr, 16), int(size, 16) if size else None
        if name in NAMES or name in ("uid", "errno") or name.startswith(UI_PREFIXES):
            symbols[name] = {"address": addr, "body_bytes": size, "type": kind}
        if kind in "TtWw" and size and code_start <= addr < 0x40000:
            sized_code.setdefault(addr, []).append((size, name))
    starts = sorted(sized_code)
    targets = {s["address"]: name for name, s in symbols.items() if name in NAMES and s["type"] in "TtWw"}
    transfers = {name: Counter() for name in NAMES}
    disassembly = read("disassembly.txt")
    for line in disassembly.splitlines():
        m = re.match(r"^\s*([0-9a-f]+):\s+(?:[0-9a-f]{2}\s+)+(call|rcall|jmp|rjmp)\s+([^;]+)(?:;\s*(.*))?$", line)
        if not m:
            continue
        addr, op, operand, annotation = m.groups()
        addr = int(addr, 16)
        dest = re.match(r"\s*0x([0-9a-f]+)\b", operand) or re.match(r"0x([0-9a-f]+)\b", annotation or "")
        if not dest or int(dest[1], 16) not in targets:
            continue
        name = targets[int(dest[1], 16)]
        i = bisect.bisect_right(starts, addr) - 1
        if i < 0:
            continue
        origin = starts[i]
        enclosing = sorted(n for size, n in sized_code[origin] if addr < origin + size)
        if not enclosing or origin == int(dest[1], 16):
            continue
        transfers[name][op + " " + " / ".join(enclosing)] += 1
    # The constant LCD delay is inlined: absence of a library CALL is not absence of work.
    start = symbols["lcdWriteByte(unsigned char, unsigned char)"]["address"]
    instructions = {}
    for line in disassembly.splitlines():
        m = re.match(r"^\s*([0-9a-f]+):\s+((?:[0-9a-f]{2}\s+)+)", line)
        if m and start <= int(m[1], 16) < start + 8:
            instructions[int(m[1], 16)] = bytes.fromhex(m[2])
    assert [instructions[start + i] for i in (0, 2, 4, 6)] == [
        bytes.fromhex(x) for x in ("eb ef", "f4 e0", "31 97", "f1 f7")], (board, instructions)
    # LDI r30,0xfb; LDI r31,0x04; SBIW r30,1; BRNE back.
    cycles = 2 + 2 * 1275 + 2 * 1274 + 1
    cols = 16 if board == "RF1000" else 20
    full_bytes = 4 * (cols + 1)
    members = sorted(set(re.findall(r"core/core\.a\(([^)]+)\)", read("build/firmware.map"))))
    return {"global_sram_bytes": stats["global_sram_bytes"], "elf_sha256": stats["elf_sha256"],
            "symbols": symbols, "core_archive_members": members,
            "lcd": {"columns": cols, "rows": 4, "inline_delay_cycles_per_byte": cycles,
                    "full_row_refresh_bytes": full_bytes, "full_refresh_delay_cycles": full_bytes * cycles,
                    "full_refresh_delay_us_nominal": full_bytes * cycles / 16,
                    "excludes_gpio_rendering_and_preemption": True},
            "direct_transfer_sightings": {n: dict(sorted(c.items())) for n, c in transfers.items() if c}}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("archive", type=Path)
    a = p.parse_args()
    with a.archive.open("rb") as f:
        sha = hashlib.file_digest(f, "sha256").hexdigest()
    assert sha == ARCHIVE_SHA256, sha
    with zipfile.ZipFile(a.archive) as z:
        report = json.loads(z.read("summary.json"))
        assert report["sha"] == TESTED_MERGE and report["failures"] == [], report["failures"]
        rows = {b: inventory(z, b, report["targets"][b]) for b in BOARDS}
    print(json.dumps({"schema": 1, "artifact_id": 15225136890, "archive_sha256": sha,
                      "tested_merge": TESTED_MERGE, "boards": rows}, indent=2))


if __name__ == "__main__":
    main()

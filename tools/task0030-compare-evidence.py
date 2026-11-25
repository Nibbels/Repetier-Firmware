#!/usr/bin/env python3
"""Verify and compare retained RFx000 firmware-evidence ZIPs in chronological order.

Usage: python3 tools/task0030-compare-evidence.py BASELINE.zip ... FINAL.zip
Offline analysis of actual build artifacts, not an AVR emulator or device test.
"""
import hashlib
import json
import re
import sys
from pathlib import Path
from zipfile import ZipFile

BOARDS = ("RF1000", "RF2000", "RF2000v2", "RF2000-X44")
IMAGES = ("Repetier.ino.eep", "Repetier.ino.hex",
          "Repetier.ino.with_bootloader.hex", "Repetier.ino.with_bootloader.bin")
ISR = (12, 17, 21, 22, 23, 25, 26)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def ihex(data):
    """Validate checksums and address records; return populated flash bytes."""
    result = {}
    upper = 0
    eof = False
    for line in data.decode("ascii").splitlines():
        assert line.startswith(":") and not eof, line
        raw = bytes.fromhex(line[1:])
        n, a, b, kind = raw[:4]
        assert len(raw) == n + 5 and sum(raw) % 256 == 0, line
        addr = a * 256 + b
        payload = raw[4:-1]
        if kind == 0:
            for offset, byte in enumerate(payload):
                pos = upper + addr + offset
                assert pos not in result or result[pos] == byte, pos
                result[pos] = byte
        elif kind == 1:
            assert n == 0
            eof = True
        elif kind in (2, 4):
            assert n == 2 and addr == 0
            upper = int.from_bytes(payload, "big") << (4 if kind == 2 else 16)
        elif kind in (3, 5):
            assert n == 4
        else:
            raise AssertionError(line)
    assert eof
    return result


def map_address(text, name):
    pattern = r"^\s*(0x[0-9a-f]+)\s+(?:PROVIDE \()?" + re.escape(name) + r"(?:\s|,)"
    match = re.search(pattern, text, re.M)
    assert match, name
    return int(match[1], 16)


def instructions(text, symbols, name):
    match = re.search(r"^([0-9a-f]+) ([0-9a-f]+) [A-Za-z] " + re.escape(name) + "$", symbols, re.M)
    assert match, name
    lo, size = (int(x, 16) for x in match.groups())
    rows = []
    for line in text.splitlines():
        m = re.match(r"^\s*([0-9a-f]+):\s+((?:[0-9a-f]{2}\s+)+)\s*([a-z]+)\s*(.*)$", line)
        if m and lo <= int(m[1], 16) < lo + size:
            rows.append((int(m[1], 16), len(m[2].split()), m[3], m[4].split(";")[0].strip()))
    assert sum(r[1] for r in rows) == size, name
    return rows


def pulse_cycles(rows):
    result = []
    for i, (_, _, op, args) in enumerate(rows):
        if op != "sbi" or args not in ("0x02, 4", "0x08, 1"):
            continue
        j = next(j for j in range(i + 1, len(rows))
                 if rows[j][2:] == ("cbi", args))
        seq = rows[i+1:j+1]
        assert [r[2] for r in seq[:5]] == ["std", "st", "ldi", "dec", "brne"], seq
        assert all(r[2] == "nop" for r in seq[5:-1]), seq
        loops = int(seq[2][3].split(",")[1], 0)
        assert loops > 0 and seq[3][3] == seq[2][3].split(",")[0].strip()
        # Rise complete to fall complete: two stores, LDI, DEC/BRNE loop,
        # NOP tail and CBI. No hardware entry or nested IRQ timing here.
        assert seq[4][3] == ".-4", seq[4]
        result.append((args, 4 + 1 + 3 * loops - 1 + len(seq[5:-1]) + 2))
    return result


def audit(path):
    report = {}
    with ZipFile(path) as z:
        s = json.loads(z.read("summary.json"))
        assert not s["failures"] and s["host_requested"] and s["avr_requested"]
        assert set(s["targets"]) == set(BOARDS)
        report["sha"] = s["sha"]
        report["archive_sha256"] = sha(Path(path).read_bytes())
        report["boards"] = {}
        report["replay_commands"] = z.read("host/commands.txt").decode().splitlines()
        report["short_moves"] = [l for l in z.read("host/pause-sense-offset.txt").decode().splitlines()
                                 if l.startswith("Short moves ")]
        assert len(report["short_moves"]) == 6
        for b in BOARDS:
            t = s["targets"][b]
            p = b + "/build/"
            assert set(t["image_sha256"]) == set(IMAGES)
            for name in IMAGES:
                assert sha(z.read(p + name)) == t["image_sha256"][name], (b, name)
            for key, name in (("elf_sha256", "Repetier.ino.elf"), ("map_sha256", "firmware.map")):
                if key in t:
                    assert sha(z.read(p + name)) == t[key], (b, key)
            app = ihex(z.read(p + "Repetier.ino.hex"))
            combined = ihex(z.read(p + "Repetier.ino.with_bootloader.hex"))
            assert all(combined.get(a) == v for a, v in app.items())
            boot = {a: v for a, v in combined.items() if a not in app}
            assert max(app) < 253952 and min(boot) >= 253952 and max(boot) < 262144
            binary = z.read(p + "Repetier.ino.with_bootloader.bin")
            # Objcopy binary starts at the lowest populated address (zero).
            assert min(combined) == 0 and len(binary) > max(combined)
            assert all(binary[a] == v for a, v in combined.items())
            maptext = z.read(p + "firmware.map").decode()
            flash, ram = t["flash_bytes"], t["global_sram_bytes"]
            assert map_address(maptext, "__data_load_end") == flash == len(app)
            assert map_address(maptext, "__bss_end") - 0x800200 == ram
            assert 8192 - ram == t["sram_remaining_before_stack_bytes"]
            near = map_address(maptext, "__ctors_end")
            assert map_address(maptext, "__trampolines_end") <= near < 65536
            symbols = z.read(b + "/symbols.txt").decode()
            dis = z.read(b + "/disassembly.txt").decode()
            vectors = {n: instructions(dis, symbols, "__vector_" + str(n)) for n in ISR}
            pulses = pulse_cycles(vectors[21])
            assert len(pulses) == (2 if b in ("RF2000", "RF2000v2") else 1)
            report["boards"][b] = {
                "flash": flash, "sram": ram, "reserve": 8192-ram, "near_margin": 65536-near,
                "app_margin": 253952-flash, "images": t["image_sha256"],
                "boot_sha256": sha(b"".join(a.to_bytes(4, "little") + bytes([v])
                                            for a, v in sorted(boot.items()))),
                "advance_pulses": pulses,
                "isr_instructions": {n: len(v) for n, v in vectors.items()},
                "allocator": bool(re.search(r"^[0-9a-f]+ [0-9a-f]+ [Tt] (?:malloc|free)$",
                                             symbols, re.M)),
                "largest_sram": t.get("largest_sram_symbols", [])[:10],
            }
    return report


def main(paths):
    assert len(paths) >= 2, __doc__
    previous = None
    base = None
    for path in paths:
        r = audit(path)
        if base is None:
            base = r
        for b in BOARDS:
            x, y = base["boards"][b], r["boards"][b]
            assert x["images"]["Repetier.ino.eep"] == y["images"]["Repetier.ino.eep"]
            assert x["boot_sha256"] == y["boot_sha256"]
            y["flash_delta"] = y["flash"] - x["flash"]
            y["sram_delta"] = y["sram"] - x["sram"]
            if previous is not None:
                y["step_flash_delta"] = y["flash"] - previous["boards"][b]["flash"]
                y["step_sram_delta"] = y["sram"] - previous["boards"][b]["sram"]
        print(json.dumps({"artifact": str(path), **r}, sort_keys=True))
        previous = r
    # The final native wave standard: 31-cycle Advance HIGH, no linked heap
    # allocator. Do not claim that all physical STEP pulses are 31 cycles.
    for b in BOARDS:
        assert all(c == 31 for _, c in previous["boards"][b]["advance_pulses"])
        assert not previous["boards"][b]["allocator"]
    assert previous["short_moves"] == base["short_moves"], "dense-move replay changed"


if __name__ == "__main__":
    main(sys.argv[1:])

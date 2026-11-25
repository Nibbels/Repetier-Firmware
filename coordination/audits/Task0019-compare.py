#!/usr/bin/env python3
"""Compare the exact Task0019 baseline and candidate AVR captures."""
import argparse
from collections import Counter
from difflib import SequenceMatcher
import hashlib
import json
from pathlib import Path
import re
import sys

TARGETS = ("RF1000", "RF2000", "RF2000v2", "RF2000-X44")
IMAGES = ("Repetier.ino.hex", "Repetier.ino.with_bootloader.hex", "Repetier.ino.eep")


def read_json(path):
    return json.loads(path.read_text())


def decode_hex(path):
    data = {}
    base = 0
    eof = False
    for line in path.read_text().splitlines():
        if not line:
            continue
        if not line.startswith(":") or eof:
            raise ValueError(f"Malformed Intel HEX: {path}")
        rec = bytes.fromhex(line[1:])
        if len(rec) != rec[0] + 5 or sum(rec) % 256:
            raise ValueError(f"Invalid HEX checksum/length: {path}")
        addr = int.from_bytes(rec[1:3], "big")
        typ = rec[3]
        payload = rec[4:-1]
        if typ == 0:
            for i, value in enumerate(payload):
                loc = base + addr + i
                if loc in data and data[loc] != value:
                    raise ValueError(f"Conflicting HEX byte: {path} at {loc:x}")
                data[loc] = value
        elif typ == 1:
            eof = True
        elif typ == 2:
            base = int.from_bytes(payload, "big") << 4
        elif typ == 4:
            base = int.from_bytes(payload, "big") << 16
        elif typ not in (3, 5):
            raise ValueError(f"Unsupported record {typ}: {path}")
    if not eof:
        raise ValueError(f"Missing EOF: {path}")
    return data


def image_compare(a, b):
    av, bv = decode_hex(a), decode_hex(b)
    diff = sorted(k for k in av.keys() | bv.keys() if av.get(k) != bv.get(k))
    return {
        "baseline_sha256": hashlib.sha256(a.read_bytes()).hexdigest(),
        "candidate_sha256": hashlib.sha256(b.read_bytes()).hexdigest(),
        "baseline_loaded": len(av), "candidate_loaded": len(bv),
        "changed_loaded_bytes": len(diff),
        "first_differences": [[hex(k), av.get(k), bv.get(k)] for k in diff[:24]],
    }


def symbols(path):
    result = {}
    for line in path.read_text(errors="replace").splitlines():
        m = re.match(r"^([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+(\S)\s+(.+)$", line)
        if m:
            result[m[4]] = {"address": int(m[1], 16), "bytes": int(m[2], 16)}
    return result


def function_key(name):
    # Only explicit type substitutions; do not discard arbitrary clone suffixes.
    if name.startswith("queueTask(") or name.startswith("PrintLine::queueTask("):
        name = name.replace("(QueueTask)", "(char)")
    if name.startswith("doEmergencyStop("):
        name = name.replace("(StopReason)", "(char)")
    if name.startswith("Printer::offsetRelativeStepsCoordinates("):
        name = name.replace("DirectMoveMode", "unsigned char")
    return name


def functions(path):
    result = {}
    current = None
    for line in path.read_text(errors="replace").splitlines():
        header = re.match(r"^([0-9a-fA-F]+) <(.+)>:$", line)
        if header:
            current = function_key(header[2])
            result.setdefault(current, [])
            continue
        m = re.match(r"^\s*([0-9a-fA-F]+):\s+((?:[0-9a-fA-F]{2}\s+)+)(\S.*)$", line)
        if current is not None and m:
            insn, _, comment = m[3].partition(";")
            insn = " ".join(insn.split())
            # Relocate only symbol-annotated absolute branches/calls/load/store.
            # Relative branch displacements and all other operands stay intact.
            op = insn.split(" ", 1)[0]
            label = re.search(r"<(.+)>", comment)
            if label and op in ("call", "jmp", "lds", "sts"):
                target = function_key(label[1])
                insn = re.sub(r"0x[0-9a-fA-F]+", lambda _: "<" + target + ">", insn, count=1)
            result[current].append(("".join(m[2].split()), insn))
    return result


def affected(name):
    return (name.startswith("__vector_") or "StepExecutor::" in name
            or "PrintLine::" in name or "MoveQueue::" in name
            or "offsetRelativeStepsCoordinates" in name or "queueTask(" in name
            or "doEmergencyStop(" in name)


def compare_functions(a, b):
    af, bf = functions(a), functions(b)
    result = {}
    for name in sorted(af.keys() | bf.keys()):
        if not affected(name):
            continue
        av, bv = af.get(name, []), bf.get(name, [])
        raw_a, raw_b = [x[0] for x in av], [x[0] for x in bv]
        norm_a, norm_b = [x[1] for x in av], [x[1] for x in bv]
        row = {"baseline_instructions": len(av), "candidate_instructions": len(bv),
               "baseline_bytes": sum(len(x)//2 for x in raw_a),
               "candidate_bytes": sum(len(x)//2 for x in raw_b),
               "raw_equal": bool(av) and raw_a == raw_b,
               "relocation_normalized_equal": bool(av) and norm_a == norm_b}
        if not row["relocation_normalized_equal"]:
            row["differences"] = [
                {"kind": op, "baseline": norm_a[i:j][:12], "candidate": norm_b[k:l][:12]}
                for op, i, j, k, l in SequenceMatcher(None, norm_a, norm_b, autojunk=False).get_opcodes()
                if op != "equal"][:12]
        result[name] = row
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    args = parser.parse_args()
    a, b = args.baseline, args.candidate
    sa, sb = read_json(a / "summary.json"), read_json(b / "summary.json")
    assert not sa["failures"] and not sb["failures"]
    assert sa["arduino_core_required"] == sb["arduino_core_required"]
    assert sa["fqbn"] == sb["fqbn"]
    assert set(sa["targets"]) == set(sb["targets"]) == set(TARGETS)
    report = {"baseline": sa["sha"], "candidate": sb["sha"], "targets": {}}
    problems = []
    for target in TARGETS:
        ad, bd = a / target, b / target
        for tool in ("avr-gcc", "avr-g++", "avr-size", "avr-nm", "avr-objdump"):
            assert (ad / (tool + "-version.txt")).read_bytes() == (bd / (tool + "-version.txt")).read_bytes(), (target, tool)
        ra, rb = sa["targets"][target], sb["targets"][target]
        syma, symb = symbols(ad / "symbols.txt"), symbols(bd / "symbols.txt")
        queue = {name: [syma[name]["bytes"], symb[name]["bytes"]]
                 for name in ("PrintLine::queue", "PrintLine::direct")}
        if any(x != y for x, y in queue.values()):
            problems.append(target + ": queue layout changed")
        images = {name: image_compare(ad / "build" / name, bd / "build" / name) for name in IMAGES}
        if images[IMAGES[2]]["changed_loaded_bytes"]:
            problems.append(target + ": EEPROM image changed")
        hot = compare_functions(ad / "disassembly.txt", bd / "disassembly.txt")
        if not any(name.startswith("__vector_") for name in hot):
            problems.append(target + ": no linked ISR evidence")
        report["targets"][target] = {
            "flash": [ra["flash_bytes"], rb["flash_bytes"]],
            "sram": [ra["global_sram_bytes"], rb["global_sram_bytes"]],
            "queue_and_direct_sizes": queue,
            "images": images, "affected_functions": hot}
    report["problems"] = problems
    (b / "task0019-comparison.json").write_text(json.dumps(report, indent=2) + "\n")
    print("Task0019 exact baseline:", sa["sha"], "candidate:", sb["sha"])
    for target, row in report["targets"].items():
        print(target, "flash", row["flash"], "SRAM", row["sram"], "queue/direct", row["queue_and_direct_sizes"])
        for name, image in row["images"].items():
            print(target, name, "changed loaded bytes", image["changed_loaded_bytes"],
                  "SHA256", image["candidate_sha256"], "first changes", image["first_differences"][:8])
        counts = Counter("raw-identical" if r["raw_equal"] else "relocation-normalized-identical"
                         if r["relocation_normalized_equal"] else "changed" for r in row["affected_functions"].values())
        print(target, "affected functions:", dict(counts))
        for name, r in row["affected_functions"].items():
            if name.startswith("__vector_") or not r["relocation_normalized_equal"]:
                print(target, name, r["baseline_bytes"], "->", r["candidate_bytes"],
                      "raw", r["raw_equal"], "normalized", r["relocation_normalized_equal"],
                      "diff", r.get("differences", [])[:2])
    print("Comparison gate:", problems or "PASS (review any instruction/image differences separately)")
    return bool(problems)


if __name__ == "__main__":
    sys.exit(main())

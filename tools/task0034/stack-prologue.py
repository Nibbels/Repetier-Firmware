#!/usr/bin/env python3
"""Task0034: inspect explicit AVR GCC prologue allocation, not peak stack.

Usage: python3 tools/task0034/stack-prologue.py firmware-evidence.zip
Reads actual symbol addresses; ignores PROGMEM data and unrecognized forms.
Reported bytes exclude return PC, callers, callees, dynamic pushes and IRQs.
"""
import argparse
import json
import re
import zipfile

NAMES = (
    "TemperatureController::autotunePID", "Extruder::manageTemperatures",
    "Commands::checkForPeriodicalActions", "HAL::temperatureAdcSample",
    "EEPROM::readDataFromEEPROM", "ThermalEmergency::checkAbsoluteCutoff",
)


def prologue(dump, address):
    start = re.search(r"^" + address + r" <[^\n]+>:$", dump, re.M)
    if not start:
        return None
    ops = [re.sub(r"\s+", " ", x.split(";", 1)[0].strip()) for x in re.findall(
        r"^\s*[0-9a-f]+:\s+(?:[0-9a-f]{2} )+[0-9a-f]{2}\s+([^\n]+)",
        dump[start.end():], re.M)[:40]]
    i = 0
    while i < len(ops) and re.fullmatch(r"push r(?:[0-9]|[12][0-9]|3[01])", ops[i]):
        i += 1
    saved = i
    if not i or len(ops) <= i:
        return None
    if ops[i:i+2] != ["in r28, 0x3d", "in r29, 0x3e"]:
        # A non-frame-pointer function can still push later or allocate
        # dynamically. Report the explicit initial saves only.
        return {"initial_saved_registers": saved, "explicit_frame": None,
                "initial_stack_bytes": saved, "form": "initial-saves-only"}
    i += 2
    single = re.fullmatch(r"sbiw r28, (0x[0-9a-fA-F]+)", ops[i])
    low = re.fullmatch(r"subi r28, (0x[0-9a-fA-F]+)", ops[i])
    if single:
        frame = int(single[1], 16)
        i += 1
    elif low:
        frame = int(low[1], 16)
        high = re.fullmatch(r"sbci r29, (0x[0-9a-fA-F]+)", ops[i+1])
        if high:
            frame += int(high[1], 16) << 8
        elif ops[i+1] != "sbc r29, r1":
            return None
        i += 2
    else:
        return None
    if ops[i:i+5] != ["in r0, 0x3f", "cli", "out 0x3e, r29", "out 0x3f, r0", "out 0x3d, r28"]:
        return None
    return {"initial_saved_registers": saved, "explicit_frame": frame,
            "initial_stack_bytes": saved+frame, "form": "frame-pointer"}


def audit_archive(path):
    result = {}
    with zipfile.ZipFile(path) as z:
        for p in sorted(z.namelist()):
            if not p.startswith("host/experimental-avr/emergency/") or not p.endswith("/stats.json"):
                continue
            directory = p.rsplit("/", 1)[0]
            stats = json.loads(z.read(p))
            symbols = z.read(directory+"/symbols.txt").decode()
            dump = z.read(directory+"/disassembly.txt").decode()
            rows = {}
            for name in NAMES:
                matches = re.findall(r"^([0-9a-f]+) ([0-9a-f]+) [tT] ("+re.escape(name)+r"\([^\n]*\))$", symbols, re.M)
                if len(matches) != 1:
                    rows[name] = {"status": "absent-or-ambiguous", "matches": len(matches)}
                    continue
                address, size, signature = matches[0]
                allocation = prologue(dump,address)
                rows[name] = {"signature": signature,"code_bytes": int(size,16),
                              "allocation": allocation}
            result[directory.rsplit("/",1)[-1]] = {
                "global_sram": stats["global_sram_bytes"],
                "remaining_before_stack": stats["sram_remaining_before_stack_bytes"],
                "functions": rows}
    return result


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive")
    args=parser.parse_args()
    print(json.dumps(audit_archive(args.archive),indent=2))

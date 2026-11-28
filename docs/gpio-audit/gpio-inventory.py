#!/usr/bin/env python3
"""Expand the actual sketch/core compile commands; retain GPIO register names."""
import argparse
import json
import re
import subprocess
import tempfile
from pathlib import Path

PORTS = "ABCDEFGHJKL"
REGISTERS = [kind + port for port in PORTS for kind in ("PORT", "DDR", "PIN")]
MARKER = re.compile(r"RF_AUDIT_(PORT|DDR|PIN)([A-L])\b")
LINE = re.compile(r'^# (\d+) "([^"]+)"')
CORE_API = re.compile(r"\b(?:digitalWrite|digitalRead|pinMode|portOutputRegister|portModeRegister|portInputRegister)\s*\(")


def inventory(database):
    sites = set()
    with tempfile.TemporaryDirectory() as tmp:
        header = Path(tmp) / "gpio.h"
        header.write_text("#include <avr/io.h>\n" + "".join(
            f"#undef {reg}\n#define {reg} RF_AUDIT_{reg}\n" for reg in REGISTERS) + "".join(
            f"#undef PIN{port}{bit}\n#define PIN{port}{bit} {bit}\n"
            for port in PORTS for bit in range(8)))
        for unit in json.loads(database.read_text()):
            args = unit["arguments"].copy()
            if args[0].endswith("gcc-ar"):
                continue
            output_index = args.index("-o")
            del args[output_index:output_index + 2]
            args = [arg for arg in args if arg not in ("-c", "-MMD")]
            result = subprocess.check_output([*args, "-E", "-include", str(header)],
                                             cwd=unit["directory"], text=True)
            line = 0
            filename = ""
            for row in result.splitlines():
                location = LINE.match(row)
                if location:
                    line = int(location[1])
                    filename = Path(location[2]).name
                    continue
                regs = sorted(set(kind + port for kind, port in MARKER.findall(row)))
                if regs or CORE_API.search(row):
                    sites.add((filename, line, ",".join(regs) or "dynamic/API",
                               " ".join(row.split()).replace("RF_AUDIT_", "")))
                line += 1
    if not sites:
        raise ValueError("no GPIO sites")
    return sorted(sites)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("database", type=Path, help="Arduino build/compile_commands.json")
    rows = inventory(parser.parse_args().database)
    print("file\tline\tregisters\texpansion")
    for row in rows:
        print(*row, sep="\t")

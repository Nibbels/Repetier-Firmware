#!/usr/bin/env python3
"""Build all four opt-in lease candidates, retain linked evidence in host log.
Never flash these experimental images. Standard builds remain unchanged.
"""
import importlib.util
import json
from pathlib import Path
import re
import shutil
import sys
import tempfile
root=Path(__file__).resolve().parents[1]
cli=shutil.which("arduino-cli")
if not cli:
    print("Integrated AVR lease probe skipped: arduino-cli unavailable")
    sys.exit(0)
spec=importlib.util.spec_from_file_location("baseline",root/"tools/task0003-baseline.py")
baseline=importlib.util.module_from_spec(spec)
# Keep CI checkout clean.
sys.dont_write_bytecode=True
spec.loader.exec_module(baseline)
with tempfile.TemporaryDirectory() as temp:
    out=Path(temp)
    for board,defs in baseline.TARGETS.items():
        stats=baseline.build_target(out,cli,board,[*defs,"RF_HEATER_LEASE_EXPERIMENT=1"])
        print(board,json.dumps(stats),flush=True)
        dump=(out/board/"disassembly.txt").read_text()
        symbols=(out/board/"symbols.txt").read_text()
        vectors={int(m[2]):int(m[1],16) for m in re.finditer(
            r"^([0-9a-f]+)\s+(?:[0-9a-f]+\s+)?[TtWw]\s+__vector_(\d+)$",symbols,re.M)}
        assert vectors[32]==vectors[33],board
        if board in ("RF2000","RF2000v2"):
            assert vectors[32]==vectors[34],board
        for line in dump.splitlines():
            m=re.search(r"\bcall\s+0x([0-9a-f]+)\b|\brcall\s+[^;]+;\s*0x([0-9a-f]+)\b",line)
            if m:
                assert int(m[1] or m[2],16) not in vectors.values(),(board,line)
        print("Linked vector aliases and direct-call audit passed",board)
        # Store actual linked ISR disassembly including context, not just sizes.
        print("BEGIN "+board+" DISASSEMBLY")
        print(dump)
        print("END "+board+" DISASSEMBLY",flush=True)

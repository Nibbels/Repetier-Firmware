#!/usr/bin/env python3
"""Build four lease and four reset-WDT candidates, retain linked evidence in host log.
Never flash these experimental images. Standard builds remain unchanged.
"""
import importlib.util
import json
from pathlib import Path
import re
import shutil
import sys
import subprocess
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
recovery_spec=importlib.util.spec_from_file_location("recovery_link",root/"tools/task0034/recovery-link.py")
recovery=importlib.util.module_from_spec(recovery_spec)
recovery_spec.loader.exec_module(recovery)
with tempfile.TemporaryDirectory() as temp:
    out=Path(temp)
    for mode,extra in (("lease",[]),("reset-wdt",["RF_HEATER_RESET_WDT_EXPERIMENT=1"])):
        mode_dir=out/mode
        mode_dir.mkdir()
        for board,defs in baseline.TARGETS.items():
            try:
                stats=baseline.build_target(mode_dir,cli,board,[*defs,"RF_HEATER_LEASE_EXPERIMENT=1",*extra],
                                            linker_script_factory=recovery.make_script if extra else None)
            except subprocess.CalledProcessError:
                log=mode_dir/board/"compile.txt"
                if log.exists():
                    print(log.read_text(errors="replace")[-24000:],flush=True)
                raise
            print(mode,board,json.dumps(stats),flush=True)
            dump=(mode_dir/board/"disassembly.txt").read_text()
            symbols=(mode_dir/board/"symbols.txt").read_text()
            vectors={int(m[2]):int(m[1],16) for m in re.finditer(
                r"^([0-9a-f]+)\s+(?:[0-9a-f]+\s+)?[TtWw]\s+__vector_(\d+)$",symbols,re.M)}
            assert vectors[32]==vectors[33],board
            if board in ("RF2000","RF2000v2"):
                assert vectors[32]==vectors[34],board
            for line in dump.splitlines():
                m=re.search(r"\bcall\s+0x([0-9a-f]+)\b|\brcall\s+[^;]+;\s*0x([0-9a-f]+)\b",line)
                if m:
                    assert int(m[1] or m[2],16) not in vectors.values(),(board,line)
            if extra:
                recovery.verify_image(mode_dir/board)
                assert vectors[12] == vectors[2], (board,"WDT vector unexpectedly active")
                assert "heaterWatchdogEarlyInit" in dump, board
                start=dump.index("<heaterWatchdogEarlyInit>:")
                end=re.search(r"^[0-9a-f]+ <[^\n]+>:",dump[start+1:],re.M)
                assert end,board
                early=dump[start:start+1+end.start()]
                assert re.search(r"\bwdr\b",early),early
                assert not re.search(r"\bret[i]?\b",early),early
                # Timed reset configuration and feed locations are retained for
                # linked inspection; this does not prove oscillator or boot behavior.
                print("WDT reset-only candidate startup linked",board)
            print("Linked vector aliases and direct-call audit passed",board)
            # Store actual linked ISR disassembly including context, not just sizes.
            print("BEGIN "+mode+" "+board+" DISASSEMBLY")
            print(dump)
            print("END "+mode+" "+board+" DISASSEMBLY",flush=True)
    
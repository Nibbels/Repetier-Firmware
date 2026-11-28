#!/usr/bin/env python3
"""Build historical comparison candidates and retain linked evidence.
The canonical combined-image builder now owns the emergency mode in full CI.
Never flash experimental or qualification images.
"""
import importlib.util
import os
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
timing_spec=importlib.util.spec_from_file_location("renew_bound",root/"tools/task0034/renew-bound.py")
timing=importlib.util.module_from_spec(timing_spec)
timing_spec.loader.exec_module(timing)
archive=os.environ.get("RF_EXPERIMENT_EVIDENCE_DIR")
if archive:
    archive=Path(archive)
    archive.mkdir(parents=True,exist_ok=False)
    (archive/"README.txt").write_text("RESEARCH ONLY. Never flash these experimental candidates.\n"
        "Only board directories with verification.json status verified passed all linked checks.\n"
        "Failed links and layout probes are diagnostics, not accepted images.\n")


build_spec=importlib.util.spec_from_file_location("verified_build",root/"tools/task0034/verified-build.py")
verified=importlib.util.module_from_spec(build_spec)
build_spec.loader.exec_module(verified)

with tempfile.TemporaryDirectory() as temp:
    out=Path(temp)
    for mode,extra in (("lease",[]),("reset-wdt",["RF_HEATER_RESET_WDT_EXPERIMENT=1"]),
                       ("emergency",["RF_HEATER_RESET_WDT_EXPERIMENT=1","RF_THERMAL_EMERGENCY_EXPERIMENT=1","RF_EEPROM_ZONE_EXPERIMENT=1"])):
        if mode=="emergency" and os.environ.get("RF_P0_CANONICAL_BUILD")=="1":
            print("Combined images are verified by the canonical build below",flush=True)
            continue
        mode_dir=out/mode
        mode_dir.mkdir()
        for board,defs in baseline.TARGETS.items():
            definitions=[*defs,"RF_INTERNAL_COMPARISON_BUILD=1","RF_HEATER_LEASE_EXPERIMENT=1",*extra]
            stats,board_dir=verified.build_verified(baseline,recovery,timing,cli,mode_dir,board,
                definitions,bool(extra),archive,mode)
            dump=(board_dir/"disassembly.txt").read_text()
            # Store actual linked ISR disassembly including context, not just sizes.
            print("BEGIN "+mode+" "+board+" DISASSEMBLY")
            print(dump)
            print("END "+mode+" "+board+" DISASSEMBLY",flush=True)
    
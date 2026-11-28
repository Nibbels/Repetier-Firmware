#!/usr/bin/env python3
"""Audit direct CALL/RCALL to linked AVR interrupt handlers in a CI ZIP.

Only direct targets are checked; this is not an indirect-call or interrupt
nesting proof. Aliases are handled by target addresses in the disassembly.
"""
import argparse
import re
import zipfile

p=argparse.ArgumentParser()
p.add_argument("artifact")
p.add_argument("--include-probe", action="store_true")
args=p.parse_args()
names=["RF1000","RF2000","RF2000v2","RF2000-X44"]
paths=[b+"/disassembly.txt" for b in names]
if args.include_probe:
    paths.append("host/heater-event-pwm-avr-probe.txt")
failures=0
with zipfile.ZipFile(args.artifact) as z:
    for path in paths:
        s=z.read(path).decode()
        vectors={int(m[1],16):m[2] for m in re.finditer(
            r"^([0-9a-f]+) <(__vector_\d+)>:",s,re.M)}
        assert vectors, path
        calls=[]
        for line in s.splitlines():
            m=re.search(r"\bcall\s+0x([0-9a-f]+)\b|\brcall\s+[^;]+;\s*0x([0-9a-f]+)\b",line)
            if m and int(m[1] or m[2],16) in vectors:
                calls.append(line.strip())
        print(path,"linked vector addresses",len(vectors),"direct ISR calls",len(calls))
        for line in calls: print("  "+line)
        failures+=len(calls)
raise SystemExit(bool(failures))

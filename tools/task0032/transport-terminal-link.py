#!/usr/bin/env python3
"""Materialize only a disposable terminal candidate and inventory guard sites.

Caller must use a detached worktree at the exact pinned P0 commit. No
production patch, no parser/resend integration. Output source-context rows
for every common/thermal guard and heater inhibit occurrence; lexical audit
is not a control-flow proof. Link with the canonical P0 verified builder.
"""
from pathlib import Path
import argparse
import hashlib
import json
import re
import runpy
import subprocess

dispatch=runpy.run_path(str(Path(__file__).with_name("transport-terminal-dispatch.py")))
P0=dispatch["P0"]

PATTERN=re.compile(r"\b(?:emergencyLocked|thermalEmergency|transportEmergency|"
                   r"beginThermalEmergency|beginTransportEmergency|stopForThermalFault|"
                   r"stopForTransportFault|latchOutputInhibit|heatingInhibited|"
                   r"blockingTxFailed|serviceTransportFailure)\s*\(")

def git(source,*args):
    return subprocess.check_output(["git","-C",str(source),*args],text=True).strip()

def inventory(fw,overrides):
    rows=[]
    for path in sorted(fw.iterdir()):
        if path.suffix not in (".h",".cpp",".ino"):continue
        lines=overrides.get(path.name,path.read_text()).splitlines()
        for i,line in enumerate(lines):
            if PATTERN.search(line):
                rows.append({"file":path.name,"line":i+1,"text":line.strip(),
                             "context":"\n".join(lines[max(0,i-2):i+4])})
    return rows

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",type=Path,required=True)
    parser.add_argument("--out",type=Path,required=True)
    parser.add_argument("--with-tx-trigger",action="store_true")
    args=parser.parse_args()
    source=args.source.resolve();out=args.out.resolve()
    assert source not in out.parents and source!=out
    assert git(source,"rev-parse","HEAD")==P0
    assert not git(source,"status","--porcelain")
    # Detached disposable worktree, refuse actual branch checkout.
    assert subprocess.run(["git","-C",str(source),"symbolic-ref","-q","HEAD"],
                          capture_output=True).returncode==1
    out.mkdir(parents=True,exist_ok=True)
    fw=source/"Repetier"
    generator=(runpy.run_path(str(Path(__file__).with_name("transport-terminal-trigger.py")))
               if args.with_tx_trigger else dispatch)
    files=generator["candidate"](fw)
    before=inventory(fw,{})
    after=inventory(fw,files)
    for name,s in files.items():(fw/name).write_text(s)
    changed=git(source,"diff","--name-only").splitlines()
    assert sorted(changed)==sorted("Repetier/"+n for n in files)
    patch=subprocess.check_output(["git","-C",str(source),"diff","--binary"])
    (out/"candidate.diff").write_bytes(patch)
    subprocess.run(["git","-C",str(source),"diff","--check"],check=True)
    (out/"guard-inventory.json").write_text(json.dumps({"before":before,"after":after},indent=2)+"\n")
    for row in after:
        print("GUARD",row["file"],row["line"],row["text"])
    summary={"p0":P0,"changed":changed,"patch_sha256":hashlib.sha256(patch).hexdigest(),
             "source_sha256":{n:hashlib.sha256((fw/n).read_bytes()).hexdigest() for n in files},
             "before_guard_rows":len(before),"after_guard_rows":len(after),
             "with_tx_trigger":args.with_tx_trigger,
             "qualification":"lexical inventory and disposable patch; bounded legacy TX trigger only, no RX resend integration"
                 if args.with_tx_trigger else "lexical inventory and disposable terminal-only patch; no UART/pump detection integration"}
    (out/"candidate.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps(summary,indent=2))

if __name__=="__main__":
    main()

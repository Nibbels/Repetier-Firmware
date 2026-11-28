#!/usr/bin/env python3
"""Forward-reconcile exact qualified Task0032 candidate onto a later P0.

Firmware-only diagnostic: old/candidate selectors retired by the target
are resolved before a three-way merge. Never reintroduce retired files.
Test changes are inventoried independently; no canonical qualification is
claimed until the new P0 fixtures are explicitly migrated and executed.
No production branch writes.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile

OLD="c31b4d391cc4d2b9cca9ab3f8e236bed805716d9"
SELECTORS={"RF_THERMAL_EMERGENCY_EXPERIMENT","RF_EEPROM_ZONE_EXPERIMENT",
           "RF_HEATER_LEASE_EXPERIMENT","RF_HEATER_RESET_WDT_EXPERIMENT"}
RETIRED={"Repetier.sln","Repetier/Repetier.vcxproj","Repetier/Repetier.vcxproj.filters",
         "tests/task0013-visual-micro.py","tests/heater-lease-integrated-avr.py",
         "Doxyfile",".github/workflows/task0032-planner-queue-research.yml",
         ".github/workflows/task0032-step-output-research.yml",
         ".github/workflows/task0033-research.yml"}

def git(root,*args,raw=False):
    data=subprocess.check_output(["git","-C",str(root),*args])
    return data if raw else data.decode().strip()

def digest(data):
    return hashlib.sha256(data).hexdigest()

def condition(op,expr,retired):
    if op in ("ifdef","ifndef"):
        if expr in retired:return op=="ifdef",None
        return None,(op,expr)
    terms=expr.split(" && ")
    if all(re.fullmatch(r"defined\(\w+\)",term) for term in terms):
        remaining=[term for term in terms if term[8:-1] not in retired]
        if len(remaining)<len(terms):
            return (True,None) if not remaining else (None,("if"," && ".join(remaining)))
    assert not any(macro in expr for macro in retired),(op,expr)
    return None,(op,expr)

def normalize(source,retired):
    """Resolve only exact known-true selector directives; fail on unknown grammar."""
    out=[];stack=[];active=True
    for line in source.splitlines(keepends=True):
        m=re.match(r"^(\s*)#\s*(if|ifdef|ifndef|elif|else|endif)\b(.*)",line)
        if not m:
            if active:out.append(line)
            continue
        ws,op,expr=m.groups();expr=expr.strip().split("//")[0].strip()
        if op in ("if","ifdef","ifndef"):
            truth,new=condition(op,expr,retired)
            stack.append({"parent":active,"known":truth is not None,
                          "taken":truth is True,"emitted":truth is None})
            if truth is None:
                if active:out.append(line if new==(op,expr) else ws+"#"+new[0]+" "+new[1]+"\n")
            else:active=active and truth
        elif op=="elif":
            f=stack[-1]
            if not f["known"]:
                assert not any(macro in expr for macro in retired),line
                active=f["parent"]
                if active:out.append(line)
            elif f["taken"]:active=False
            else:
                truth,new=condition("if",expr,retired)
                if truth is None:
                    f["known"]=False;f["emitted"]=True;active=f["parent"]
                    if active:out.append(ws+"#"+new[0]+" "+new[1]+"\n")
                else:
                    f["taken"]=truth;active=f["parent"] and truth
        elif op=="else":
            f=stack[-1]
            if f["known"]:active=f["parent"] and not f["taken"];f["taken"]=True
            else:
                active=f["parent"]
                if active:out.append(line)
        else:
            f=stack.pop();active=f["parent"]
            if f["emitted"] and active:out.append(line)
    assert not stack
    return "".join(out)

def merge(current,base,candidate):
    with tempfile.TemporaryDirectory() as d:
        folder=Path(d)
        paths=[folder/name for name in ("p0","old","task0032")]
        for p,data in zip(paths,(current,base,candidate)):p.write_bytes(data)
        proc=subprocess.run(["git","merge-file","-p",*map(str,paths)],capture_output=True)
        assert 0<=proc.returncode<=127,proc.stderr.decode()
        return proc.returncode,proc.stdout

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source",type=Path,required=True,help="clean detached later P0 checkout")
    p.add_argument("--old-candidate",type=Path,required=True,help="clean detached exact prior candidate tree")
    p.add_argument("--old-tree",required=True)
    p.add_argument("--p0",required=True)
    p.add_argument("--out",type=Path,required=True)
    a=p.parse_args()
    source=a.source.resolve();candidate=a.old_candidate.resolve();out=a.out.resolve()
    assert len({source,candidate,out})==3 and source not in out.parents and candidate not in out.parents
    for work in (source,candidate):
        assert not git(work,"status","--porcelain")
        assert subprocess.run(["git","-C",str(work),"symbolic-ref","-q","HEAD"],capture_output=True).returncode==1
    assert git(source,"rev-parse","HEAD")==a.p0
    assert git(candidate,"rev-parse","HEAD^{tree}")==a.old_tree
    # The exact prior commit can be a reconstructed commit, but its parent
    # and full content tree must be the already recorded old baseline/tree.
    assert git(candidate,"rev-parse","HEAD^")==OLD
    old_config=git(candidate,"show",OLD+":Repetier/Configuration.h")
    new_config=(source/"Repetier/Configuration.h").read_text()
    retired={m for m in SELECTORS if re.search(r"^#define\s+"+m+r"\b",old_config,re.M)
             and not re.search(r"^#define\s+"+m+r"\b",new_config,re.M)}
    assert {"RF_THERMAL_EMERGENCY_EXPERIMENT","RF_EEPROM_ZONE_EXPERIMENT"}<=retired
    assert not any((source/name).exists() for name in RETIRED), "target still carries a retired path"
    before_wobble=digest((source/"Repetier/WobbleCorrection.h").read_bytes())
    # Read exact changed paths; do not treat an omitted file as deleted.
    paths=git(candidate,"diff","--name-only","--no-renames",OLD,"HEAD",raw=True).decode().splitlines()
    out.mkdir(parents=True,exist_ok=True)
    result={"scope":"firmware-only diagnostic; no host or AVR qualification",
            "old_p0":OLD,"old_candidate_tree":a.old_tree,"new_p0":a.p0,
            "retired_selectors":sorted(retired),"files":{},"tests":{},"conflicts":[]}
    pending={}
    for name in paths:
        path=Path(name)
        assert not path.is_absolute() and ".." not in path.parts
        prior=candidate/path;later=source/path
        if name in RETIRED:
            assert not later.exists()
            result["files"][name]={"status":"retired in P0; not restored"}
            continue
        if not (name.startswith("Repetier/") and path.suffix in (".h",".cpp",".ino")):
            assert name.startswith("tests/"),("unclassified predecessor change",name)
            # Do not overwrite newer safety fixtures with old candidates.
            proc=subprocess.run(["git","-C",str(candidate),"show",OLD+":"+name],capture_output=True)
            data=proc.stdout if proc.returncode==0 else None
            result["tests"][name]={"old":digest(data) if data is not None else None,
                "candidate":digest(prior.read_bytes()) if prior.is_file() else None,
                "p0":digest(later.read_bytes()) if later.is_file() else None,
                "status":"explicit migration required"}
            continue
        assert prior.is_file(),("unexpected candidate source deletion",name)
        raw=prior.read_bytes()
        normalized=normalize(raw.decode(),retired).encode()
        proc=subprocess.run(["git","-C",str(candidate),"show",OLD+":"+name],capture_output=True)
        if proc.returncode:
            assert not later.exists(),("new source collides",name)
            merged=normalized;rc=0
        else:
            assert later.is_file(),("P0 removed candidate source",name)
            base=normalize(proc.stdout.decode(),retired).encode()
            rc,merged=merge(later.read_bytes(),base,normalized)
        if rc:
            result["conflicts"].append(name)
            conflict=out/"conflicts"/name;conflict.parent.mkdir(parents=True,exist_ok=True)
            conflict.write_bytes(merged)
            continue
        for macro in retired:
            assert not re.search(r"^\s*#.*\b"+macro+r"\b",merged.decode(),re.M),(name,macro)
        assert not re.search(r"\b(?:M3029|accessPanel|executeRemoteAction)\b",merged.decode()),name
        result["files"][name]={"status":"clean three-way merge",
            "candidate_sha256":digest(raw),"merged_sha256":digest(merged)}
        pending[name]=merged
    (out/"reconcile.json").write_text(json.dumps(result,indent=2)+"\n")
    assert not result["conflicts"],result["conflicts"]
    for name,data in pending.items():
        path=source/name
        path.write_bytes(data)
    assert digest((source/"Repetier/WobbleCorrection.h").read_bytes())==before_wobble
    subprocess.run(["git","-C",str(source),"diff","--check"],check=True)
    patch=subprocess.check_output(["git","-C",str(source),"diff","--binary"])
    (out/"firmware.diff").write_bytes(patch)
    result["firmware_diff_sha256"]=digest(patch)
    result["new_files"]=[name for name in pending if not subprocess.run(
        ["git","-C",str(source),"ls-files","--error-unmatch",name],capture_output=True).returncode==0]
    (out/"reconcile.json").write_text(json.dumps(result,indent=2)+"\n")
    print("P0 reconciliation:",len(pending),"firmware files;",len(result["tests"]),
          "test paths require explicit migration; no production authorization",flush=True)

if __name__=="__main__":main()

#!/usr/bin/env python3
"""Compose already-qualified small print-core changes on one pinned P0 image.

Reuse the exact existing transformation helpers. Keep historic preflight pins
intact, validate the prior numeric oracle and sqrt inputs byte-for-byte, and
fail on new source consumers. No changes to the live checkout.
"""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import re
import runpy
import shutil
import subprocess
import sys
import tempfile

HERE=Path(__file__).resolve().parent
P0="188eeeb0c9a272d93a7230247b794748318ece86"
DECIMAL_P0="9072d65121bc2086522ea6b837ca3804ec80faa5"
SQRT_P0="9345d9f52f9428be0f920e6777c1e50286c42d71"


def git(root,*args):
    return subprocess.check_output(["git","-C",str(root),*args],text=True)


def numeric_token(s):
    i=s.index("inline long parseLongValue(")
    j=s.index("\n}\n",i)+3
    return s[i:j]


def sqrt_body(s):
    i=s.index("uint16_t HAL::integerSqrt(")
    j=s.index("} // integerSqrt",i)+len("} // integerSqrt")
    return s[i:j]


def digest(data):
    return hashlib.sha256(data).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source",type=Path,required=True)
    p.add_argument("--out",type=Path,required=True)
    a=p.parse_args()
    source=a.source.resolve();out=a.out.resolve();out.mkdir(parents=True,exist_ok=True)
    assert git(source,"rev-parse","HEAD").strip()==P0
    assert not git(source,"status","--porcelain")
    small=runpy.run_path(str(HERE/"small-cleanup-preflight.py"))
    integer=runpy.run_path(str(HERE/"decimal32-preflight.py"))
    receiver=(source/"Repetier/GCodeInput.cpp").read_text()
    old=numeric_token(receiver)
    assert old==numeric_token(git(source,"show",DECIMAL_P0+":Repetier/GCodeInput.cpp"))
    asm=sqrt_body((source/"Repetier/HAL.cpp").read_text())
    assert asm==sqrt_body(git(source,"show",SQRT_P0+":Repetier/HAL.cpp"))
    errno_users=[]
    for f in (source/"Repetier").rglob("*"):
        if f.is_file() and f.suffix in (".h",".cpp",".c",".ino",".hpp"):
            if re.search(r"\berrno\b",f.read_text(encoding="utf-8")):
                errno_users.append(str(f.relative_to(source)))
    assert not errno_users,errno_users
    with tempfile.TemporaryDirectory(prefix="task0032-combined-core-") as directory:
        root=Path(directory)
        for folder in ("Repetier","tests"):
            shutil.copytree(source/folder,root/folder)
        small["fixed_uart"](root)
        small["fixed_queue"](root)
        small["parser_cleanup"](root)
        small["sqrt_cleanup"](root)
        # Reuse the exact qualified decimal parser and its wire fixture.
        # The existing small-cleanup test already prepends wire cases.
        new=(HERE/"decimal32-candidate.h").read_text()+"\ninline long parseLongValue(char* s) { return parseDecimal32(s); }\n"
        small["edit"](root/"Repetier/GCodeInput.cpp",lambda s:small["once"](s,old,new))
        def wire(s):
            s=small["once"](s,"void numberedLines() {",integer["WIRE"]+"\nvoid numberedLines() {")
            return small["once"](s,'    else if (scenario == "numbered") numberedLines();',
                '    else if (scenario == "decimal32-wire") integerWire();\n    else if (scenario == "numbered") numberedLines();')
        small["edit"](root/"tests/serial-protocol.cpp",wire)
        def py(s):
            s=small["once"](s,'for scenario in ("wire-ascii",','for scenario in ("decimal32-wire", "wire-ascii",')
            return small["once"](s,'for scenario in ("thermal-emergency",','for scenario in ("decimal32-wire", "thermal-emergency",')
        small["edit"](root/"tests/serial-protocol.py",py)
        small["check_fixed_uart_consumers"](root)
        # Make the selected scope explicit, including all generated test edits.
        files=[];patch=[]
        for f in sorted(root.rglob("*")):
            if not f.is_file():continue
            rel=f.relative_to(root);before=source/rel
            assert before.is_file(),rel
            a_bytes,b_bytes=before.read_bytes(),f.read_bytes()
            if a_bytes==b_bytes:continue
            files.append({"file":str(rel),"before_sha256":digest(a_bytes),"after_sha256":digest(b_bytes)})
            patch.extend(difflib.unified_diff(before.read_text().splitlines(True),f.read_text().splitlines(True),
                                             fromfile="a/"+str(rel),tofile="b/"+str(rel)))
        (out/"candidate.diff").write_text("".join(patch))
        result={"p0":P0,"reused_decimal_oracle":DECIMAL_P0,"reused_sqrt_oracle":SQRT_P0,
                "original_decimal_sha256":digest(old.encode()),"original_sqrt_sha256":digest(asm.encode()),
                "patch_sha256":digest((out/"candidate.diff").read_bytes()),"files":files,
                "qualification":"combined disposable source transformation; requires canonical P0 recovery build"}
        (out/"sources.json").write_text(json.dumps(result,indent=2)+"\n")
        small["decimal_replay"](root,out)
        for test in ("serial-protocol.py","pause-sense-offset.py"):
            subprocess.run([sys.executable,str(root/"tests"/test)],check=True)
        print("Combined core: numeric source identity, actual decimal formatter and serial/motion replays PASS",flush=True)


if __name__=="__main__":
    main()

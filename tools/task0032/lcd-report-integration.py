#!/usr/bin/env python3
"""Compare exact M3029 report code before/after fixed-width cache conversion.

Reuse the historical row-stream report fixture, without changing its archived
source or baseline assumptions. This is a logical snapshot compatibility test.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("row_stream",HERE/"lcd-row-stream-preflight.py")
row_stream=importlib.util.module_from_spec(spec)
spec.loader.exec_module(row_stream)


def trace(root,out,fixed):
    fw=root/"Repetier"
    source=(fw/"GCodeSystem.cpp").read_text()
    i=source.index('        Com::printF(PSTR("RequestMenu:"), UI_ROWS);')
    j=source.index("\n    }\n}",i)
    body="void report(GCode* pCommand) {\n"+source[i:j]+"\n}\n"
    language=(fw/"uilang.h").read_text()
    selector=re.findall(r"^#define CHAR_SELECTOR (.+)$",language,re.M)
    selected=re.findall(r"^#define CHAR_SELECTED (.+)$",language,re.M)
    assert len(selector)==len(selected)==2
    main=row_stream.REPORT_MAIN
    old="                uid.rows[row][end]=0;\n                uid.rows[row][UI_COLS]=0;"
    assert main.count(old)==1
    main=main.replace(old,
        "                if(end<CACHE_WIDTH) uid.rows[row][end]=0;\n"
        "                if(UI_COLS<CACHE_WIDTH) uid.rows[row][UI_COLS]=0;")
    with tempfile.TemporaryDirectory(prefix="task0032-fixed-report-") as directory:
        d=Path(directory)
        with out.open("wb") as log:
            for index,cols in enumerate((16,20)):
                prefix=("#define UI_COLS "+str(cols)+"\n#define CACHE_WIDTH "+
                        str(cols if fixed else 29)+"\n#define CHAR_SELECTOR "+selector[index]+
                        "\n#define CHAR_SELECTED "+selected[index]+"\n")
                cpp=d/"report.cpp";cpp.write_text(prefix+row_stream.REPORT_FIXTURE+body+main)
                exe=d/"report"
                subprocess.run([os.environ.get("CXX","c++"),"-std=c++11","-O1","-Wall","-Wextra","-Werror",
                                "-fsanitize=address,undefined","-fno-sanitize-recover=all","-fno-pie","-no-pie",
                                cpp,"-o",exe],check=True)
                subprocess.run([exe],stdout=log,check=True,env={**os.environ,"ASAN_OPTIONS":"detect_leaks=0"})
    raw=out.read_bytes()
    assert raw.count(b"RequestMenu:4,29:")==256*(17+21)*2
    return raw


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source",type=Path,required=True)
    p.add_argument("--out",type=Path,required=True)
    p.add_argument("--candidate",action="store_true")
    a=p.parse_args()
    a.out.mkdir(parents=True,exist_ok=True)
    if not a.candidate:
        trace(a.source,a.out/"m3029-baseline.bin",False)
        return
    candidate=trace(a.source,a.out/"m3029-fixed-width.bin",True)
    baseline=(a.out/"m3029-baseline.bin").read_bytes()
    assert candidate==baseline,"M3029 logical snapshot wire report changed"
    result={"reports":256*(17+21)*2,"sha256":hashlib.sha256(candidate).hexdigest(),
            "widths":[16,20],"wire_columns":29,"sanitizers":["address","undefined"]}
    print(json.dumps(result,indent=2))
    (a.out/"m3029-comparison.json").write_text(json.dumps(result,indent=2)+"\n")

if __name__=="__main__":
    main()

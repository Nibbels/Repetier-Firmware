#!/usr/bin/env python3
"""Disposable actual-source counterfactual: stream LCD rows synchronously.

Keep the existing GPIO, timing, glyph, emergency and menu behavior. Do NOT
combine with the asynchronous transport experiment. Never edit the pinned
source checkout or the shared firmware/replays in the actual PR.
"""
import argparse
import difflib
import hashlib
import json
import os
import re
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

P0 = "33f00438522b75885a35a710062ff0781aaa2a91"


def once(s, old, new):
    assert s.count(old) == 1, old
    return s.replace(old, new)


def transform(root):
    fw, tests = root/"Repetier", root/"tests"
    ui = fw/"ui.cpp"
    text = ui.read_text()
    i = text.index("void UIDisplay::refreshPage()")
    j = text.index("} // refreshPage", i)
    original = text[i:j]
    candidate = once(original, """    char cache[UI_ROWS][MAX_COLS + 1];
    adjustMenuPos();

    // Copy result into cache""", """    adjustMenuPos();
    const uint8_t scroll = shift > 0 ? shift : 0;
    const auto sendCurrentRow = [this, scroll](uint8_t row) {
        const uint8_t length = strlen(printCols);
        const uint8_t offset = length > UI_COLS ? RMath::min(length - UI_COLS, scroll) : 0;
        printRow(row, &printCols[offset], NULL, UI_COLS);
    };

    // Render one row into the existing scratch buffer, then transmit it.""")
    assert candidate.count("strcpy(cache[r], printCols);") == 2
    candidate = candidate.replace("strcpy(cache[r], printCols);", "sendCurrentRow(r);")
    candidate = once(candidate, """    while (r < UI_ROWS)
        strcpy(cache[r++], printCols);

    for (uint8_t row = 0; row < UI_ROWS; ++row)
        strcpy(displayCache_[row], cache[row]);

    const uint8_t scroll = shift > 0 ? shift : 0;
    for (uint8_t row = 0; row < UI_ROWS; ++row) {
        const uint8_t length = strlen(cache[row]);
        const uint8_t offset = length > UI_COLS ? RMath::min(length - UI_COLS, scroll) : 0;
        printRow(row, &cache[row][offset], NULL, UI_COLS);
    }
""", """    while (r < UI_ROWS)
        sendCurrentRow(r++);
""")
    assert "cache[" not in candidate
    ui.write_text(text[:i]+candidate+text[j:])

    header = fw/"ui.h"
    header.write_text(once(header.read_text(), "char displayCache_[UI_ROWS][MAX_COLS + 1];",
                          "char displayCache_[UI_ROWS][UI_COLS + 1];"))
    # The M3029 report retains its 4x29 wire format even with a narrower
    # physical shadow. Never read past a visible row to synthesize padding.
    system = fw/"GCodeSystem.cpp"
    system.write_text(once(system.read_text(), "const char character = line[col];",
                           "const char character = col < UI_COLS ? line[col] : 0;"))
    replay = tests/"ui-workflows.py"
    replay.write_text(once(replay.read_text(), "displayCache_[UI_ROWS][MAX_COLS+1]={};",
                          "displayCache_[UI_ROWS][UI_COLS+1]={};"))
    # Fail closed if there is another direct reader or writer of the cache.
    owners = []
    for p in sorted(fw.rglob("*")):
        if p.is_file() and p.suffix in (".h", ".cpp", ".ino"):
            if "displayCache_" in p.read_text():
                owners.append(p.relative_to(fw).as_posix())
    assert owners == ["ui.cpp", "ui.h"], owners
    readers = []
    for p in sorted(fw.rglob("*")):
        if p.is_file() and p.suffix in (".h", ".cpp", ".ino") and "cachedRow(" in p.read_text():
            readers.append(p.relative_to(fw).as_posix())
    assert readers == ["GCodeSystem.cpp", "ui.cpp", "ui.h"], readers


SCROLL = r'''
static UIMenuEntry longAction={"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ",3,1234,0,0};
static UIMenuEntry longSub={"abcdefghijklmnopqrstuvwxyz",2,
    reinterpret_cast<uintptr_t>(&ui_menu_main),0,0};
static UIMenuEntry* const longEntries[]={&longAction,&longSub};
static UIMenu longMenu={UI_MENU_TYPE_SUBMENU,0,2,longEntries};

void scrollReplay() {
    visibilityRendering=true;
    Printer::activeModes=0;
    for(int position=0;position<2;++position) {
        for(int editing=0;editing<2;++editing) {
            for(int offset=-2;offset<=MAX_COLS+1-UI_COLS;++offset) {
                uid.menuLevel=2; uid.menu[1]=&ui_menu_main; uid.menu[2]=&longMenu;
                uid.menuPos[2]=position; uid.menuTop[2]=0;
                uid.activeAction=editing?1234:0;
                uid.shift=offset;
                uid.refreshPage();
                for(unsigned row=0;row<UI_ROWS;++row) {
                    assert(uid.lcd[row].size()==UI_COLS);
                    assert(uid.lcd[row]==uid.displayCache_[row]);
                }
            }
        }
    }
    uid.activeAction=0; uid.menuLevel=0;
    for(unsigned page=0;page<panelPageCount;++page) {
        uid.menuPos[0]=page;uid.shift=-2;uid.refreshPage();
        for(unsigned row=0;row<UI_ROWS;++row) {
            assert(uid.lcd[row].size()==UI_COLS);
            assert(uid.lcd[row]==uid.displayCache_[row]);
        }
    }
    visibilityRendering=false;
}
int main() { preheat(); offset(); navigation(); visibility(); scrollReplay(); }
'''


def trace(root, out):
    tests = root/"tests"
    s = (tests/"ui-workflows.py").read_text()
    s = once(s, "#include <cstring>\n", "#include <cstring>\n#include <cstdio>\n")
    s = once(s, "        assert(mode==0); row=-1;",
             '        std::printf("A%02x\\n", unsigned(v));\n        assert(mode==0); row=-1;')
    s = once(s, "void lcdPutChar(char c) { assert(row>=0",
             'void lcdPutChar(char c) { std::printf("D%02x\\n", unsigned(uint8_t(c))); assert(row>=0')
    s = once(s, "int main() { preheat(); offset(); navigation(); visibility(); }", SCROLL)
    path = tests/"task0032-lcd-trace.py"
    try:
        path.write_text(s)
        with out.open("w") as log:
            subprocess.run([sys.executable,path], stdout=log, check=True)
    finally:
        path.unlink(missing_ok=True)
    raw = out.read_bytes()
    # Do not let a silently omitted replay appear equal.
    assert raw.count(b" PASS") == 4 and raw.count(b"\nA") > 100, out
    return raw


REPORT_FIXTURE = r'''
#include <cassert>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <string>
using std::isprint;
#define PSTR(x) x
#define MAX_COLS 28
#define UI_ROWS 4
struct UI {
    char rows[UI_ROWS][CACHE_WIDTH];
    const char* cachedRow(uint8_t row) const { assert(row<UI_ROWS); return rows[row]; }
} uid;
struct GCode {
    bool pretty;
    bool hasL() const { return pretty; }
};
namespace Com {
    std::string output;
    void print(char c) { output+=c; }
    void printF(const char* s) { output+=s; }
    void printF(const char* s,unsigned n) { output+=s; output+=std::to_string(n); }
    void println() { output+="\r\n"; }
}
'''

REPORT_MAIN = r'''
int main() {
    // All byte values including the board-dependent selector and CGRAM
    // characters. NUL at each visible position, nonzero garbage afterward.
    // The full-row terminator and legacy 4x29 output width are invariant.
    for(unsigned value=0;value<256;++value) {
        for(unsigned end=0;end<=UI_COLS;++end) {
            for(unsigned row=0;row<UI_ROWS;++row) {
                memset(uid.rows[row],'X',CACHE_WIDTH);
                for(unsigned col=0;col<UI_COLS;++col)
                    uid.rows[row][col]=char((value+row+col)&255);
                uid.rows[row][end]=0;
                uid.rows[row][UI_COLS]=0;
            }
            for(bool pretty:{false,true}) {
                GCode command{pretty};
                Com::output.clear();
                report(&command);
                const std::string prefix="RequestMenu:4,29:";
                assert(Com::output.compare(0,prefix.size(),prefix)==0);
                assert(Com::output.size()==prefix.size()+4*29+2+(pretty?12:0));
                std::fwrite(Com::output.data(),1,Com::output.size(),stdout);
            }
        }
    }
}
'''


def report_trace(root,out,candidate):
    fw = root/"Repetier"
    source = (fw/"GCodeSystem.cpp").read_text()
    i = source.index('        Com::printF(PSTR("RequestMenu:"), UI_ROWS);')
    j = source.index("\n    }\n}",i)
    body = "void report(GCode* pCommand) {\n"+source[i:j]+"\n}\n"
    language = (fw/"uilang.h").read_text()
    selector = re.findall(r"^#define CHAR_SELECTOR (.+)$",language,re.M)
    selected = re.findall(r"^#define CHAR_SELECTED (.+)$",language,re.M)
    assert len(selector)==len(selected)==2
    with tempfile.TemporaryDirectory(prefix="task0032-lcd-report-") as directory:
        d=Path(directory)
        with out.open("wb") as log:
            for index,cols in enumerate((16,20)):
                prefix = ("#define UI_COLS "+str(cols)+"\n#define CACHE_WIDTH "+
                          str(cols+1 if candidate else 29)+"\n#define CHAR_SELECTOR "+selector[index]+
                          "\n#define CHAR_SELECTED "+selected[index]+"\n")
                cpp=d/"report.cpp";cpp.write_text(prefix+REPORT_FIXTURE+body+REPORT_MAIN)
                exe=d/"report"
                subprocess.run([os.environ.get("CXX","c++"),"-std=c++11","-O1","-Wall","-Wextra","-Werror",
                                "-fsanitize=address,undefined","-fno-sanitize-recover=all","-fno-pie","-no-pie",
                                cpp,"-o",exe],check=True)
                subprocess.run([exe],stdout=log,check=True,env={**os.environ,"ASAN_OPTIONS":"detect_leaks=0"})
    raw=out.read_bytes()
    assert raw.count(b"RequestMenu:4,29:")==256*(17+21)*2
    return raw


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source",type=Path,required=True)
    p.add_argument("--out",type=Path,required=True)
    a = p.parse_args()
    source, out = a.source.resolve(), a.out.resolve()
    out.mkdir(parents=True,exist_ok=True)
    head = subprocess.check_output(["git","-C",str(source),"rev-parse","HEAD"],text=True).strip()
    assert head == P0, head
    assert not subprocess.check_output(["git","-C",str(source),"status","--porcelain"],text=True)
    with tempfile.TemporaryDirectory(prefix="task0032-lcd-row-") as directory:
        root = Path(directory)
        for folder in ("Repetier","tests"):
            shutil.copytree(source/folder,root/folder)
        baseline = trace(root,out/"baseline-trace.txt")
        baseline_report = report_trace(root,out/"baseline-report.bin",False)
        transform(root)
        candidate = trace(root,out/"candidate-trace.txt")
        candidate_report = report_trace(root,out/"candidate-report.bin",True)
        assert baseline == candidate, "LCD address/data replay differs"
        assert baseline_report == candidate_report, "M3029 report differs"
        changes = []
        diff = []
        for relative in ("Repetier/ui.cpp","Repetier/ui.h","Repetier/GCodeSystem.cpp","tests/ui-workflows.py"):
            old,new = source/relative,root/relative
            old_bytes,new_bytes = old.read_bytes(),new.read_bytes()
            assert old_bytes != new_bytes
            changes.append({"file":relative,"old_sha256":hashlib.sha256(old_bytes).hexdigest(),
                            "candidate_sha256":hashlib.sha256(new_bytes).hexdigest()})
            diff.extend(difflib.unified_diff(old.read_text().splitlines(True),new.read_text().splitlines(True),
                        fromfile="a/"+relative,tofile="b/"+relative))
        (out/"candidate.diff").write_text("".join(diff))
        report = {"p0":head,"changes":changes,"trace_sha256":hashlib.sha256(baseline).hexdigest(),
                  "address_commands":sum(l.startswith(b"A") for l in baseline.splitlines()),
                  "data_bytes":sum(l.startswith(b"D") for l in baseline.splitlines()),
                  "m3029_trace_sha256":hashlib.sha256(baseline_report).hexdigest(),
                  "m3029_reports":256*(17+21)*2,
                  "source_local_row_cache_removed_bytes":116,
                  "source_global_cache_reduction_bytes":[48,32,32,32],
                  "qualification":"actual-source host UI and M3029 replays, static snapshots and selected scroll cases; not asynchronous integration or dynamic snapshot timing proof"}
        (out/"row-stream-results.json").write_text(json.dumps(report,indent=2)+"\n")
        print(json.dumps(report,indent=2))


if __name__ == "__main__":
    main()

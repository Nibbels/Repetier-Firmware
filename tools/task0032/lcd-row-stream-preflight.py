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
        transform(root)
        candidate = trace(root,out/"candidate-trace.txt")
        assert baseline == candidate, "LCD address/data replay differs"
        changes = []
        diff = []
        for relative in ("Repetier/ui.cpp","Repetier/ui.h","tests/ui-workflows.py"):
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
                  "source_local_row_cache_removed_bytes":116,
                  "source_global_cache_reduction_bytes":[48,32,32,32],
                  "qualification":"actual-source host UI replay, static snapshots and selected scroll cases; not asynchronous integration or dynamic snapshot timing proof"}
        (out/"row-stream-results.json").write_text(json.dumps(report,indent=2)+"\n")
        print(json.dumps(report,indent=2))


if __name__ == "__main__":
    main()

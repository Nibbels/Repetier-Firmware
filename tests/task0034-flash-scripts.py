#!/usr/bin/env python3
"""Check real configured hard-filament strings before AVR linking."""
from pathlib import Path
import re
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
config=(root/"Repetier/Configuration.h").read_text()
def definition(source,name):
    matches=re.findall(r"^#define\s+"+name+r"[ \t]+[^\n]+",source,re.M)
    assert len(matches)==1,(name,matches)
    return matches[0]
defaults={name:definition(config,name) for name in
          ("PREHEAT_A_HOTEND_C","PREHEAT_B_HOTEND_C")}
for board in ("RF1000","RF2000"):
    source=(root/"Repetier"/(board+".h")).read_text()
    macros=[definition(source,name) for name in
            ("MOUNT_FILAMENT_SCRIPT_HARD","UNMOUNT_FILAMENT_SCRIPT_HARD")]
    program="\n".join(["#include <cassert>","#include <cstring>","#include <cstdio>",
                      "#define str(s) #s","#define xstr(s) str(s)",
                      *defaults.values(),*macros,r'''
int main(){
    char expected[80];
    snprintf(expected,sizeof(expected),"M109 S%d\nG21\nG92 E0\nG1 E90 F100\nG92 E0\nM104 S0",
             PREHEAT_B_HOTEND_C);
    assert(strcmp(MOUNT_FILAMENT_SCRIPT_HARD,expected)==0);
    snprintf(expected,sizeof(expected),"M109 S%d\nG21\nG92 E0\nG1 E-90 F500\nG92 E0\nM104 S0",
             PREHEAT_A_HOTEND_C);
    assert(strcmp(UNMOUNT_FILAMENT_SCRIPT_HARD,expected)==0);
}
'''])
    with tempfile.TemporaryDirectory() as d:
        path=Path(d)/"test.cpp"
        path.write_text(program)
        exe=Path(d)/"test"
        subprocess.run(["g++","-std=c++11","-Wall","-Wextra","-Werror",
                        str(path),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
    print(board+" hard-filament numeric macro expansion PASS")

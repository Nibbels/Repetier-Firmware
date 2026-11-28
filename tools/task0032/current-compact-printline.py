#!/usr/bin/env python3
"""Task0032: remove only the redundant reciprocal from the prepared move.

Keep binary32 rounding at the same intermediate as the archived planner.
No range, capacity, direction, execution-profile or geometry changes.
"""
import argparse
from pathlib import Path
import re

def replace_once(path,old,new):
    s=path.read_text()
    assert s.count(old)==1,(str(path),old,s.count(old))
    path.write_text(s.replace(old,new))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--root",type=Path,required=True)
    a=p.parse_args()
    fw=a.root/"Repetier"
    replace_once(fw/"motion.h","        float invFullSpeed;\n","")
    replace_once(fw/"MotionPlanner.cpp","    lookahead.invFullSpeed = 1.0 / move.fullSpeed;\n","")
    replace_once(fw/"MotionPlanner.cpp", """    float startFactor = lookahead.startSpeed * lookahead.invFullSpeed;
    float endFactor = lookahead.endSpeed * lookahead.invFullSpeed;""", """    // Keep the original binary32 reciprocal and multiply rounding.
    // Recompute only when a profile actually requires preparation.
    const float invFullSpeed = 1.0 / move.fullSpeed;
    float startFactor = lookahead.startSpeed * invFullSpeed;
    float endFactor = lookahead.endSpeed * invFullSpeed;""")
    replace_once(fw/"MoveQueue.cpp", "        const float scale = lookahead.distance * lookahead.invFullSpeed * fraction;",
                 "        const float invFullSpeed = 1.0 / move.fullSpeed;\n        const float scale = lookahead.distance * invFullSpeed * fraction;")
    fixture=a.root/"tests/pause-sense-offset.cpp"
    replace_once(fixture,"    travel.lookahead.invFullSpeed = 0.01f;","    travel.fullSpeed = 100.0f;")
    replace_once(fixture,"    retract.lookahead.invFullSpeed = 0.04f;","    retract.fullSpeed = 25.0f;")
    replace_once(fixture,"        move.lookahead.invFullSpeed = 1.0f / 32.0f;\n","")
    for f in fw.glob("*"):
        if f.suffix in (".h",".cpp",".ino"):
            assert not re.search(r"\.invFullSpeed\b",f.read_text()),str(f)
    print("PrintLine reciprocal storage removed; all member references eliminated")

if __name__=="__main__":
    main()

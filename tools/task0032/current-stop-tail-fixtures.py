#!/usr/bin/env python3
"""Migrate canonical fixture extraction to the Timer1 single-writer contract."""
import argparse
from pathlib import Path

def replace_once(path,old,new):
    s=path.read_text()
    assert s.count(old)==1,(str(path),old,s.count(old))
    path.write_text(s.replace(old,new,1))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source",type=Path,required=True)
    a=p.parse_args()
    t=a.source/"tests"
    replace_once(t/"pause-sense-offset.cpp",
        "    void limitToStopTail(bool refitRamp);",
        "    void limitToStopTail(bool refitRamp);\n    void applyRunningStopTail();")
    replace_once(t/"pause-sense-offset.py",
        '            + section(motion, "    static bool directMovePreparing(", "\\n\\n")',
        '            + section(motion, "    static bool directRunningStoppable(", "\\n\\n") + "\\n"\n            + section(motion, "    static bool directMovePreparing(", "\\n\\n")')
    replace_once(t/"pause-sense-offset.cpp",
        """    {
        InterruptProtectedBlock lock;
        move.task = DIRECT_RUNNING_STOPPABLE;
        PrintLine::stopDirectMove();
    }
    assert(move.stepsRemaining == 32 && move.stepProfile.accelSteps == initialRampSteps);""",
        """    const int32_t initialRemaining = move.stepsRemaining;
    {
        InterruptProtectedBlock lock;
        move.task = DIRECT_RUNNING_STOPPABLE;
        PrintLine::stopDirectMove();
        assert(move.task == DIRECT_RUNNING_STOPPABLE_STOPPING);
        assert(move.stepsRemaining == initialRemaining);
        assert(PrintLine::hasStoppableDirectMove());
    }
    {
        InterruptProtectedBlock lock;
        move.applyRunningStopTail();
    }
    assert(move.stepsRemaining == 32 && move.stepProfile.accelSteps == initialRampSteps);""")
    replace_once(t/"task0034-guarded-drive.cpp",
        '#include "Constants.h"',
        '#include "Constants.h"\n#include <algorithm>\nnamespace RMath { using std::min; }')
    replace_once(t/"task0034-guarded-drive.py",
        'motion[motion.index("GuardedMoveResult PrintLine::runScanMove("):motion.index("void PrintLine::prepareDirectMove(")],',
        'motion[motion.index("GuardedMoveResult PrintLine::runScanMove("):motion.index("void PrintLine::prepareDirectMove(")] + "\\n" +\n        motion[motion.index("void PrintLine::applyRunningStopTail("):motion.index("/** \\\\brief Check if move is new.")],')
    print("Canonical stop-tail fixtures migrated; preserved actual production extraction.")

if __name__=="__main__":main()

#!/usr/bin/env python3
"""Run the shared source-extraction replay with task-private instrumentation.

No repository files are rewritten. The shared extractor produces firmware
fragments in its temporary build directory. We modify ONLY those fragments
and the generated test translation unit, and invoke the compiler supplied by CI.
Counters are operation counts, NOT cycles; timestamps are simulated.
"""
import argparse
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import types

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]

def replace_once(text, old, new):
    if text.count(old) != 1:
        raise RuntimeError(f"source anchor drift: {old!r} ({text.count(old)})")
    return text.replace(old, new, 1)

def instrument(s):
    hooks = [
        ("void MotionPlanner::prepareMove(PrintLine& move, const float axisDistanceMM[4], float feedrate, const MoveQueue& queue) {", "++pc.prepare;"),
        ("void MotionPlanner::prepareStepProfile(PrintLine& move) {", "++pc.profile;"),
        ("inline void MotionPlanner::limitJunction(PrintLine& previous, const PrintLine& next) {", "++pc.junction;"),
        ("inline void MotionPlanner::planDeceleration(MoveQueue& queue, uint8_t start, uint8_t first) {", "++pc.decel;"),
        ("inline void MotionPlanner::planAcceleration(MoveQueue& queue, uint8_t first, uint8_t end) {", "++pc.accel;"),
        ("void MotionPlanner::updateLookahead(MoveQueue& queue) {", "++pc.update;"),
        ("while (start != first) {", "++pc.backVisits;"),
        ("while (first != end) {", "++pc.forwardVisits;"),
        ("while (reservedTicks < planningReserveTicks && limit != end) {", "++pc.reserveVisits;"),
        ("if (first == end) {\n        newest.setStartSpeedFixed(true);", "++pc.noWindow;"),
        ("|| (Printer::extrusionOutput.advanceEnabled() && previous.isEMove() != newest.isEMove())) {", "++pc.extrusionBoundary;"),
        ("if (uint64_t(primaryAcceleration) * axisSteps > allowance) {", "++pc.axisLimit; if (allowance > UINT32_MAX) ++pc.wideDivide;"),
        ("if (move.isStepProfileReady() || move.isWarmUp())\n        return;", "++pc.profileRebuild;"),
        ("lastJunctionSpeed = next->isNominalMove() ? next->fullSpeed", "if (!next->isNominalMove()) ++pc.backRoots;"),
        ("float reachableExit = move.isNominalMove() ? move.fullSpeed", "if (!move.isNominalMove()) ++pc.forwardRoots;"),
        ("from.endSpeed = sqrt(entrySpeed * entrySpeed + from.accelerationDistance2);", "++pc.forwardRoots;"),
        ("reachableExit = sqrt(entrySpeed * entrySpeed + from.accelerationDistance2);", "++pc.forwardRoots;"),
    ]
    for old, hook in hooks:
        if old.endswith("{") or old.endswith("true);") or old.endswith("return;"):
            new = old + "\n    " + hook
        else:
            new = hook + "\n        " + old
        s = replace_once(s, old, new)
    s = replace_once(s, "while (first != limit && !queue.slot(first).isEndSpeedFixed())\n        MoveQueue::previousIndex(first);",
                     "while (first != limit && !queue.slot(first).isEndSpeedFixed()) {\n        ++pc.fixedSearch; MoveQueue::previousIndex(first);\n    }")
    s = replace_once(s, "firstMove.block();\n    lock.unprotect();",
                     "firstMove.block();\n    pc.first = first; pc.limit = limit; pc.reserveTicks = reservedTicks;\n    lock.unprotect();")
    return s

def junction_variant(s, variant):
    # Only the temporary extracted source is changed; all safety-related
    # E/Z limits, acceleration, safeSpeed, queue flags and boundaries remain.
    anchor = "    if (from.distance < MAX_JERK_DISTANCE)\n        lengthFactor = static_cast<float>(MAX_JERK_DISTANCE * MAX_JERK_DISTANCE) / (from.distance * from.distance);"
    if variant == "baseline":
        return s
    if variant == "unweighted":
        return replace_once(s, anchor, "    // Research counterfactual: remove length weighting.")
    if variant == "mean-length":
        return replace_once(s, anchor,
            "    const float joinLength = (from.distance + to.distance) * 0.5f;\n"
            "    if (joinLength < MAX_JERK_DISTANCE)\n"
            "        lengthFactor = static_cast<float>(MAX_JERK_DISTANCE * MAX_JERK_DISTANCE) / (joinLength * joinLength);")
    raise ValueError(variant)

def run_capacity(capacity, output, scenario, junction):
    spec = importlib.util.spec_from_file_location("shared_replay", ROOT / "tests/pause-sense-offset.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    real_run = subprocess.run
    compiled = False

    def intercepted(args, **kwargs):
        nonlocal compiled
        if "-o" not in args:
            return None  # shared scenarios are exercised by standard CI, not repeated here
        if compiled:
            return None  # one-extruder fixture; no fake claim of two-extruder coverage
        executable = Path(args[args.index("-o") + 1])
        build = executable.parent
        source = ROOT / "tests/pause-sense-offset.cpp"
        s = source.read_text(encoding="utf-8")
        s = replace_once(s, '#include "RF2000.h"', '#include "RF2000.h"\n#undef MOVE_CACHE_SIZE\n#define MOVE_CACHE_SIZE ' + str(capacity))
        anchor = '#include "pause-sense-offset-firmware.inc"'
        s = replace_once(s, anchor, '''struct PathCounts {
    unsigned prepare=0, profile=0, profileRebuild=0, junction=0, decel=0, accel=0, update=0;
    unsigned backVisits=0, forwardVisits=0, reserveVisits=0, fixedSearch=0, noWindow=0;
    unsigned extrusionBoundary=0, axisLimit=0, wideDivide=0, backRoots=0, forwardRoots=0;
    unsigned first=0, limit=0, reserveTicks=0;
} pc;
''' + anchor)
        s = replace_once(s, "int main(int argc, char** argv) {", '#include "planner-trace.inc"\nint main(int argc, char** argv) {\n    if (argc == 2 && !std::strcmp(argv[1], "planner-research")) { plannerResearch(); return 0; }')
        generated = build / "planner-research.cpp"
        generated.write_text(s, encoding="utf-8")
        fragments = build / "pause-sense-offset-firmware.inc"
        fragments.write_text(instrument(junction_variant(fragments.read_text(encoding="utf-8"),junction)), encoding="utf-8")
        cmd = [str(generated) if str(a) == str(source) else a for a in args]
        cmd[1:1] = ["-I", str(HERE)]
        real_run(cmd, **kwargs)
        env = dict(os.environ, PLANNER_SCENARIO=scenario)
        real_run([str(executable), "planner-research"], check=True, timeout=180, stdout=output, env=env)
        compiled = True

    mod.subprocess = types.SimpleNamespace(run=intercepted)
    mod.main()
    if not compiled:
        raise RuntimeError("shared replay did not compile")

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capacity", type=int, nargs="+", default=[24, 28, 32])
    parser.add_argument("--scenario", choices=["all", "smoke", "stress"], default="all")
    parser.add_argument("--junction",choices=["baseline","unweighted","mean-length"],default="baseline")
    parser.add_argument("--output", type=Path, default=Path("planner-trace.jsonl"))
    args = parser.parse_args()
    if any(n < 10 or n > 64 for n in args.capacity):
        parser.error("capacities must be 10..64; not a production allocation")
    with args.output.open("w", encoding="utf-8") as output:
        for capacity in args.capacity:
            print(f"Actual-source replay: capacity={capacity}", file=sys.stderr)
            run_capacity(capacity, output, args.scenario,args.junction)

if __name__ == "__main__":
    main()

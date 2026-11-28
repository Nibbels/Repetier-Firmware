#!/usr/bin/env python3
"""A73 guarded foreground callsite diagnostic on an exact green CI image.

This is NOT a global stack proof. It retains infeasible data/phase paths,
uses the frame inventory's candidate switch tables and ABI assumptions,
and does not prove CRT startup, interrupt response or a SRAM safety margin.
The guards below are reviewed source contracts, not inferred from names.
Refuse unreviewed linked callers or changed source anchors.
"""
import argparse
from collections import defaultdict
from functools import lru_cache
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import zipfile

sys.dont_write_bytecode = True

def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(filename))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

fg = load("foreground_depth", "foreground-stack-depth.py")
PROGRAM_WORKERS = {
    "GCodeInput::runSoftFilament(": ("GCodeInput::executeSoftFilament(",),
    "GCodeInput::runHardFilament(": ("GCodeInput::executeHardFilament(",),
    "GCodeInput::runOutputProgram(": ("GCodeInput::executeOutputProgram(",),
}

GUARDS = (
    ("UIDisplay::executeAction(", ("UIDisplay::slowAction(", "Commands::executeGCode(")),
    ("dispatchScanWork(", ("Commands::checkForPeriodicalActions(", "handleScanWorkTasks(")),
    ("servicePrintFeatures(", ("loopFeatures(",)),
    ("PrintPause::dispatchPendingMotion(", ("loopFeatures(", "PrintPause::serviceMotion(")),
    (tuple(PROGRAM_WORKERS), tuple(a for v in PROGRAM_WORKERS.values() for a in v)),
)
# The added guards correspond to explicitly named, admitted workers.
# Phase exclusions remain an opt-in diagnostic hypothesis until the
# source lifetime/linked caller proofs are complete.
PHASE_GUARDS = (
    ("StartLine::runPrepared(", ("StartLine::run(", "Commands::executeGCode(")),
    ("GCodeCommands::loadFilamentPrepared(", ("GCodeCommands::loadFilament(", "GCodeCommands::loadFilamentWithParameters(", "GCodeInput::runSoftFilament(", "Commands::executeGCode(")),
    ("GCodeCommands::unloadFilamentPrepared(", ("GCodeCommands::unloadFilament(", "GCodeCommands::unloadFilamentWithParameters(", "GCodeInput::runSoftFilament(", "Commands::executeGCode(")),
    ("TemperatureController::autotunePIDPrepared(", ("TemperatureController::autotunePID(", "UIDisplay::finishAction(")),
)
# Guard indices: panel, scan, features, pause, script, startline,
# load, unload, autotune. Direction matters: a soft script may enter
# its filament worker, but a filament worker may not start a new script.
PHASE_BLOCKED = {
    1: (3, 4, 5, 6, 7),
    3: (1,),
    4: (1, 6, 7),
    5: (1, 6, 7),
    6: (1, 5, 6, 7),
    7: (1, 5, 6, 7),
    8: (1, 3, 4, 5, 6, 7),
}
def phase_compatible(active, entering):
    return not any(active & (1 << other) for other in PHASE_BLOCKED.get(entering, ()))

PHASE_ANCHORS = {
    "RF.cpp": ("if (!Printer::printJob.isIdle() || Printer::pause.blocksCalibration() || GCodeInput::flashScriptActive())",
               "!GCodeInput::flashScriptActive() && !isAnyScanRunning()",
               "(Printer::printJob.isIdle() || Printer::printJob.isPrinting())",
               "!admitted())"),
    "PrintPause.h": ("bool blocksCalibration() const { return isPaused() || servicingMotion_; }",),
    "PrintPause.cpp": ("servicingMotion_ || !isPaused() || isAnyScanRunning()",),
    "HeatBedScan.cpp": ("!Printer::printJob.isIdle() || Printer::pause.blocksCalibration() || GCodeInput::flashScriptActive()",),
    "ZOffsetScan.cpp": ("!Printer::printJob.isIdle() || Printer::pause.blocksCalibration() || GCodeInput::flashScriptActive()",),
    "StartLine.cpp": ("(!Printer::printJob.isPrinting() && !Printer::printJob.isIdle())",
                      "Printer::printJob.admitPrintMotion();", "runPrepared(command);"),
    "GCodeThermal.cpp": ("!Printer::printJob.isIdle() || isAnyScanRunning()",
                        "!Printer::pause.isPaused() && Printer::printJob.beginFilamentService()",
                        "if (prepareFilamentProgram())", "loadFilamentPrepared(maxP, eFeedrate);",
                        "unloadFilamentPrepared(maxForce);", "Printer::printJob.finishFilamentService();"),
    "UiActions.cpp": ("GCodeInput::flashScriptActive() || !Printer::printJob.isIdle() || isAnyScanRunning()",),
    "TemperatureControl.cpp": ("!Printer::printJob.isIdle() || Printer::pause.blocksCalibration()",
                               "isAnyScanRunning() || GCodeInput::flashScriptActive()",
                               "autotunePIDPrepared(temp, controllerId, maxCycles, storeValues, method);"),
}
ANCHORS = {
    "ui.cpp": ("if ((flags & UI_FLAG_SLOW_ACTION_RUNNING) == 0)", "flags |= UI_FLAG_SLOW_ACTION_RUNNING;",
               "flags &= ~UI_FLAG_SLOW_ACTION_RUNNING;"),
    "ui.h": ("void executeRemoteAction(int action)", "if (flags & UI_FLAG_SLOW_ACTION_RUNNING)",
             "flags |= UI_FLAG_SLOW_ACTION_RUNNING;", "flags &= ~UI_FLAG_SLOW_ACTION_RUNNING;"),
    "GCodeSystem.cpp": ("uid.executeRemoteAction(pCommand->P);",),
    "RF.cpp": ("Calibration::scanSession.dispatching()", "Calibration::scanSession.beginDispatch();",
               "dispatchScanWork(tickDue);", "Calibration::scanSession.finishDispatch()",
               "if (!nEntered)", "nEntered++;", "servicePrintFeatures();", "nEntered--;"),
    "PrintPause.cpp": ("if (servicingMotion_", "servicingMotion_ = true;",
                       "dispatchPendingMotion();", "servicingMotion_ = false;"),
    "GCodeInput.cpp": ("if (flashScriptRunning || Printer::printJob.commandsBlocked() || Printer::heatingInhibited())",
                       "if (flashScriptRunning || !internalProgramAdmitted())",
                       "flashScriptRunning = true;", "runSoftFilament(unload);",
                       "runHardFilament(unload);", "runOutputProgram();", "flashScriptRunning = false;"),
    "UiFormatting.cpp": ("!ram && allowMessageExpansion", "parse(statusMsg, true, false);"),
}

def diagnostic(frame, phase=False):
    guards_spec = GUARDS + (PHASE_GUARDS if phase else ())
    names = {f["address"]: f["name"] for f in frame["frames"]}
    local = {f["address"]: f["localPeakBytes"] for f in frame["frames"]}
    edges = defaultdict(list)
    guard_callers = defaultdict(set)
    worker_callers = defaultdict(set)
    for edge in frame["calls"]:
        a, b = edge["caller"], edge["callee"]
        for i, (prefix, allowed) in enumerate(guards_spec):
            if names[b].startswith(prefix):
                assert names[a].startswith(allowed), ("unreviewed guard caller", names[a], names[b])
                if i == 4:
                    worker = next(k for k in PROGRAM_WORKERS if names[b].startswith(k))
                    assert names[a].startswith(PROGRAM_WORKERS[worker]), (worker, names[a])
                    worker_callers[worker].add(names[a])
                guard_callers[i].add(names[a])
        for site in edge["sites"]:
            edges[a].append((b, site["pc"], site["callerBytes"]))
    assert all(guard_callers[i] for i in range(len(guards_spec))), "missing reviewed linked guard"
    assert set(worker_callers) == set(PROGRAM_WORKERS), "missing internal worker"
    active, path = set(), []
    pruned = defaultdict(int)

    def transition(state, b, site):
        a, guards, parse = state
        bn = names[b]
        next_guards, next_parse = guards, parse
        for i, (prefix, _) in enumerate(guards_spec):
            if bn.startswith(prefix):
                if guards & (1 << i):
                    pruned["internal program owner" if i == 4 else prefix] += 1
                    return None
                if phase and not phase_compatible(guards, i):
                    pruned["phase hypothesis: " + ("internal program owner" if i == 4 else prefix)] += 1
                    return None
                next_guards |= 1 << i
        # Page -> one flash message -> one RAM status; no further expansion.
        if bn.startswith("UIDisplay::parse("):
            if parse >= 3:
                pruned["LCD parse depth"] += 1
                return None
            next_parse += 1
        return b, next_guards, next_parse

    @lru_cache(None)
    def visit(state):
        if state in active:
            raise ValueError(("unresolved recursion", [names[s[0]] for s in path[path.index(state):] + [state]]))
        active.add(state)
        path.append(state)
        a = state[0]
        best = local[a], [(a, local[a], None)]
        for b, site, depth in edges[a]:
            nxt = transition(state, b, site)
            if nxt is None:
                continue
            child, witness = visit(nxt)
            if depth + child > best[0]:
                best = depth + child, [(a, depth, site)] + witness
        path.pop()
        active.remove(state)
        return best

    peak, witness = visit((frame["main"], 0, 0))
    return {"diagnosticBytes": peak, "phaseHypothesesApplied": phase,
            "contextStates": visit.cache_info().currsize,
            "prunedContextEdges": dict(pruned),
            "guardCallers": {("internal program owner" if i == 4 else guards_spec[i][0]): sorted(v) for i, v in guard_callers.items()},
            "programWorkerCallers": {k: sorted(v) for k,v in worker_callers.items()},
            "witness": [{"function": names[a], "bytes": depth, "site": site}
                        for a, depth, site in witness]}

def inventory(z, board, phase=False):
    p = board + "/"
    assert json.loads(z.read(p + "verification.json"))["status"] == "verified"
    for filename, anchors in (list(ANCHORS.items()) + (list(PHASE_ANCHORS.items()) if phase else [])):
        source = z.read(p + "build/sketch/" + filename).decode()
        for anchor in anchors:
            assert anchor in source, (board, filename, "review guard", anchor)
    symbols = z.read(p + "symbols.txt").decode()
    for retired in ("runFlashScript", "executeFlashScript", "tMountFilamentSoft",
                    "tUnmountFilamentSoft", "tMountFilamentHard", "tUnmountFilamentHard", "tOutputObjectPrint"):
        assert retired not in symbols, (board, "retired internal text symbol", retired)
    source = z.read(p + "build/sketch/GCodeInput.cpp").decode()
    assert "runFlashScript(" not in source and "executeFlashScript(" not in source
    frame = fg.inventory(z, board, True)
    assert not frame["errors"], frame["errors"]
    return diagnostic(frame, phase)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive")
    parser.add_argument("sha256")
    parser.add_argument("--phase-diagnostic", action="store_true", help="apply reviewed-but-not-yet-proven operation lifetime hypotheses")
    args = parser.parse_args()
    with open(args.archive, "rb") as f:
        assert hashlib.sha256(f.read()).hexdigest() == args.sha256
    with zipfile.ZipFile(args.archive) as z:
        summary = json.loads(z.read("summary.json"))
        assert not summary["failures"]
        boards = {b: inventory(z, b, args.phase_diagnostic) for b in fg.BOARDS}
    print(json.dumps({"testedMerge": summary["sha"], "sha256": args.sha256,
                      "releaseProof": False, "boards": boards,
                      "limitations": ["source guard assumptions require review, not proved by string anchors",
                                      "candidate tables and ABI/weak callback assumptions",
                                      "unresolved phase/admission correlations; opt-in phase hypothesis is not a release proof",
                                      "CRT and interrupt nesting/response excluded"]}, indent=2))

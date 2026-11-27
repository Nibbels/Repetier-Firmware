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

dispatch = load("script_dispatch", "script-dispatch-context.py")
fg = dispatch.fg

GUARDS = (
    ("UIDisplay::executeAction(", ("UIDisplay::slowAction(", "Commands::executeGCode(")),
    ("dispatchScanWork(", ("Commands::checkForPeriodicalActions(", "handleScanWorkTasks(")),
    ("servicePrintFeatures(", ("loopFeatures(",)),
    ("PrintPause::dispatchPendingMotion(", ("loopFeatures(", "PrintPause::serviceMotion(")),
    ("GCodeInput::runFlashScript(", ("GCodeInput::executeFlashScript(",)),
)
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
    "GCodeInput.cpp": ("if (flashScriptRunning || Printer::printJob.commandsBlocked())",
                       "flashScriptRunning = true;", "runFlashScript(cmd);", "flashScriptRunning = false;"),
    "UiFormatting.cpp": ("!ram && allowMessageExpansion", "parse(statusMsg, true, false);"),
}

def diagnostic(frame, script_sites):
    names = {f["address"]: f["name"] for f in frame["frames"]}
    local = {f["address"]: f["localPeakBytes"] for f in frame["frames"]}
    edges = defaultdict(list)
    guard_callers = defaultdict(set)
    for edge in frame["calls"]:
        a, b = edge["caller"], edge["callee"]
        for i, (prefix, allowed) in enumerate(GUARDS):
            if names[b].startswith(prefix):
                assert names[a].startswith(allowed), ("unreviewed guard caller", names[a], names[b])
                guard_callers[i].add(names[a])
        for site in edge["sites"]:
            edges[a].append((b, site["pc"], site["callerBytes"]))
    assert all(guard_callers[i] for i in range(len(GUARDS))), "missing reviewed linked guard"
    active, path = set(), []
    pruned = defaultdict(int)

    def transition(state, b, site):
        a, guards, parse, script = state
        an, bn = names[a], names[b]
        next_guards, next_parse, next_script = guards, parse, script
        for i, (prefix, _) in enumerate(GUARDS):
            if bn.startswith(prefix):
                if guards & (1 << i):
                    pruned[prefix] += 1
                    return None
                next_guards |= 1 << i
        # Page -> one flash message -> one RAM status; no further expansion.
        if bn.startswith("UIDisplay::parse("):
            if parse >= 3:
                pruned["LCD parse depth"] += 1
                return None
            next_parse += 1
        if bn.startswith("GCodeInput::executeFlashScript("):
            if guards & (1 << 4):
                # Include the rejected wrapper's own frame. Its worker is
                # excluded by the guard, not by deleting the wrapper call.
                next_script = ""
            elif an.startswith("outputObject("):
                next_script = "out"
            elif an.startswith("UIDisplay::executeCommandAction("):
                next_script = "filament"
            else:
                raise ValueError(("unreviewed script caller", an))
        if an.startswith("Commands::executeGCode("):
            if script and site not in script_sites[script]:
                pruned["linked script command selection"] += 1
                return None
            next_script = ""
        if bn.startswith("Commands::executeGCode(") and not an.startswith("GCodeInput::runFlashScript("):
            next_script = ""
        return b, next_guards, next_parse, next_script

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

    peak, witness = visit((frame["main"], 0, 0, ""))
    return {"diagnosticBytes": peak,
            "contextStates": visit.cache_info().currsize,
            "prunedContextEdges": dict(pruned),
            "guardCallers": {GUARDS[i][0]: sorted(v) for i, v in guard_callers.items()},
            "witness": [{"function": names[a], "bytes": depth, "site": site}
                        for a, depth, site in witness]}

def inventory(z, board):
    p = board + "/"
    assert json.loads(z.read(p + "verification.json"))["status"] == "verified"
    for filename, anchors in ANCHORS.items():
        source = z.read(p + "build/sketch/" + filename).decode()
        for anchor in anchors:
            assert anchor in source, (board, filename, "review guard", anchor)
    sc = dispatch.inventory(z, board)
    frame = fg.inventory(z, board, True)
    assert not frame["errors"], frame["errors"]
    sites = sc["scriptCallSites"]
    assert set(sites) == set(dispatch.scripts.EXPECTED)
    groups = {"out": set(sites["tOutputObjectPrint"]),
              "filament": {pc for key, values in sites.items() if key != "tOutputObjectPrint" for pc in values}}
    return diagnostic(frame, groups)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive")
    parser.add_argument("sha256")
    args = parser.parse_args()
    with open(args.archive, "rb") as f:
        assert hashlib.sha256(f.read()).hexdigest() == args.sha256
    with zipfile.ZipFile(args.archive) as z:
        summary = json.loads(z.read("summary.json"))
        assert not summary["failures"]
        boards = {b: inventory(z, b) for b in fg.BOARDS}
    print(json.dumps({"testedMerge": summary["sha"], "sha256": args.sha256,
                      "releaseProof": False, "boards": boards,
                      "limitations": ["source guard assumptions require review, not proved by string anchors",
                                      "candidate tables and ABI/weak callback assumptions",
                                      "unresolved phase/admission correlations",
                                      "CRT and interrupt nesting/response excluded"]}, indent=2))

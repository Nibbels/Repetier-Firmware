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
import re
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

HOST_MACHINE = "GCodeCommands::(anonymous namespace)::dispatchMachine("
HOMING_MASK = 1 << 9
HOMING_CALLERS = {
    "Printer::homeAxis(": ("Commands::executeGCode(", "GCodeCommands::homeAxes(",
        "UIDisplay::executeCommandAction(", "dispatchScanWork(",
        "(anonymous namespace)::ZOffsetScan::abort(", "(anonymous namespace)::HeatBedScan::"),
    "Printer::checkHome(": ("Commands::executeGCode(", HOST_MACHINE,
        "GCodeCommands::testHomingRepeatability("),
}

def homing_owner(name):
    return re.fullmatch(r"Printer::(?:homeAxis\(bool, bool, bool\)|checkHome\(signed char\))(?: \[clone [^\]]+\])*", name) is not None

GUARDS = (
    ("UIDisplay::executeAction(", ("UIDisplay::slowAction(", "Commands::executeGCode(", HOST_MACHINE)),
    ("dispatchScanWork(", ("Commands::checkForPeriodicalActions(", "handleScanWorkTasks(")),
    ("servicePrintFeatures(", ("loopFeatures(",)),
    ("PrintPause::dispatchPendingMotion(", ("loopFeatures(", "PrintPause::serviceMotion(")),
    (tuple(PROGRAM_WORKERS), tuple(a for v in PROGRAM_WORKERS.values() for a in v)),
)
# The added guards correspond to explicitly named, admitted workers.
# Phase exclusions remain an opt-in diagnostic hypothesis until the
# source lifetime/linked caller proofs are complete.
PHASE_GUARDS = (
    ("StartLine::runPrepared(", ("StartLine::run(", "Commands::executeGCode(", HOST_MACHINE)),
    ("GCodeCommands::loadFilamentPrepared(", ("GCodeCommands::loadFilament(", "GCodeCommands::loadFilamentWithParameters(", "GCodeInput::runSoftFilament(", "Commands::executeGCode(", HOST_MACHINE)),
    ("GCodeCommands::unloadFilamentPrepared(", ("GCodeCommands::unloadFilament(", "GCodeCommands::unloadFilamentWithParameters(", "GCodeInput::runSoftFilament(", "Commands::executeGCode(", HOST_MACHINE)),
    ("TemperatureController::autotunePIDPrepared(", ("TemperatureController::autotunePID(", "UIDisplay::finishAction(")),
)
# Guard indices: panel, scan, features, pause, script, startline,
# load, unload, autotune. Direction matters: a soft script may enter
# its filament worker, but a filament worker may not start a new script.
PHASE_BLOCKED = {
    1: (3, 4, 5, 6, 7, 8),  # autotune enters with no scan, owns panel until return
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
             "flags |= UI_FLAG_SLOW_ACTION_RUNNING;", "flags &= ~UI_FLAG_SLOW_ACTION_RUNNING;",
             "class ScopedActionExclusion", "owns_(!(display.flags & UI_FLAG_SLOW_ACTION_RUNNING))",
             "if (owns_) display_.flags |= UI_FLAG_SLOW_ACTION_RUNNING;",
             "if (owns_) display_.flags &= ~UI_FLAG_SLOW_ACTION_RUNNING;"),
    "GCodeSystem.cpp": ("uid.executeRemoteAction(pCommand->P);",),
    "PrintPause.cpp": ("if (servicingMotion_", "servicingMotion_ = true;",
                       "dispatchPendingMotion();", "servicingMotion_ = false;"),
    "GCodeInput.cpp": ("if (flashScriptRunning || Printer::printJob.commandsBlocked() || Printer::heatingInhibited())",
                       "if (flashScriptRunning || !internalProgramAdmitted())",
                       "flashScriptRunning = true;", "runSoftFilament(unload);",
                       "runHardFilament(unload);", "runOutputProgram();", "flashScriptRunning = false;"),
    "UiFormatting.cpp": ("!ram && allowMessageExpansion", "parse(statusMsg, true, false);"),
    "Homing.cpp": ("__attribute__((noinline)) int8_t Printer::checkHome(",
                   "__attribute__((noinline)) void Printer::homeAxis(",
                   "UIDisplay::ScopedActionExclusion panelExclusion(uid);"),
    "GCodeMotion.cpp": ("void homeAxes(GCode* com)", "if (!isHomingAllowed(com))",
                        "void testHomingRepeatability(GCode* pCommand)",
                        "if (!isHomingAllowed(pCommand))",
                        "Printer::doHeatBedZCompensation || !Printer::zMotion.correctionCleared()"),
    "GCodeCommands.cpp": ("__attribute__((noinline)) void Commands::executeGCode(",
                          "GCodeCommands::dispatchMachine(com);", "case 3028: return testHomingRepeatability(com);"),
    "ZMotion.h": ("return correctionTarget_ == 0 && appliedCorrection_ == 0;",),
    "UiActions.cpp": ("if (!isHomingAllowed(NULL, 1))",),
    "RF.cpp": ("Calibration::scanSession.dispatching()", "Calibration::scanSession.beginDispatch();",
               "dispatchScanWork(tickDue);", "Calibration::scanSession.finishDispatch()",
               "if (!nEntered)", "nEntered++;", "servicePrintFeatures();", "nEntered--;",
               "unsigned char isHomingAllowed(", "if (isAnyScanRunning())"),
}

def diagnostic(frame, phase=False, reviewed_owners=False):
    autotune_panel_mask = 1 << 10
    priming_mask = 1 << 11
    guards_spec = GUARDS + (PHASE_GUARDS if phase else ())
    names = {f["address"]: f["name"] for f in frame["frames"]}
    local = {f["address"]: f["localPeakBytes"] for f in frame["frames"]}
    edges = defaultdict(list)
    guard_callers = defaultdict(set)
    worker_callers = defaultdict(set)
    # A tail transfer shares the stack frame but may otherwise hide an
    # admission boundary. Refuse cross-function jumps into any guarded
    # worker or LCD recursion frame rather than silently dropping a guard.
    for jump in frame["tailTransfers"]:
        target = jump["physicalTarget"]
        assert not any(target.startswith(prefix) for prefix, _ in guards_spec), ("unreviewed tail guard entry", jump)
        assert not target.startswith("UIDisplay::parse("), ("unreviewed tail LCD entry", jump)
        assert not homing_owner(target), ("unreviewed tail homing entry", jump)
        if target.startswith(HOST_MACHINE):
            assert jump["physicalCaller"].startswith("Commands::executeGCode("), ("unreviewed host dispatch tail", jump)
    for edge in frame["calls"]:
        a, b = edge["caller"], edge["callee"]
        for site in edge["sites"]:
            # Stack bytes belong to the root frame; admission is reviewed
            # against the actual instruction's containing linked symbol.
            caller = site["physicalCaller"]
            if names[b].startswith(HOST_MACHINE):
                assert caller.startswith("Commands::executeGCode("), ("unreviewed host dispatch entry", caller)
            if caller.startswith(HOST_MACHINE):
                assert names[a].startswith(("Commands::executeGCode(", HOST_MACHINE)), ("unreviewed host dispatch owner", names[a], caller)
            if homing_owner(names[b]):
                key = next(k for k in HOMING_CALLERS if names[b].startswith(k))
                assert caller.startswith(HOMING_CALLERS[key]), ("unreviewed homing caller", caller, names[b])
            for i, (prefix, allowed) in enumerate(guards_spec):
                if names[b].startswith(prefix):
                    assert caller.startswith(allowed), ("unreviewed guard caller", names[a], caller, names[b], site["pc"])
                    if i == 4:
                        worker = next(k for k in PROGRAM_WORKERS if names[b].startswith(k))
                        assert caller.startswith(PROGRAM_WORKERS[worker]), (worker, caller)
                        worker_callers[worker].add(caller)
                    guard_callers[i].add(caller)
            edges[a].append((b, site["pc"], site["callerBytes"]))
    assert all(guard_callers[i] for i in range(len(guards_spec))), "missing reviewed linked guard"
    assert set(worker_callers) == set(PROGRAM_WORKERS), "missing internal worker"
    active, path = set(), []
    pruned = defaultdict(int)

    def transition(state, b, site):
        a, guards, parse = state
        bn = names[b]
        next_guards, next_parse = guards, parse
        # Both homing entries retain the panel dispatch bit. External
        # homing refuses a running scan; scan-internal homing already
        # owns the scan dispatcher, so neither admits a new scan worker.
        if guards & priming_mask and bn.startswith(("dispatchScanWork(", "StartLine::runPrepared(", "TemperatureController::autotunePIDPrepared(", "GCodeCommands::loadFilamentPrepared(", "GCodeCommands::unloadFilamentPrepared(")):
            pruned["admitted startline owns non-idle job"] += 1
            return None
        if reviewed_owners and bn.startswith("StartLine::runPrepared("):
            next_guards |= priming_mask
        if reviewed_owners and bn.startswith("TemperatureController::autotunePIDPrepared(") and guards & 1:
            next_guards |= autotune_panel_mask
        if guards & autotune_panel_mask and bn.startswith("dispatchScanWork("):
            pruned["panel-owned autotune entered with no scan"] += 1
            return None
        if homing_owner(bn):
            next_guards |= 1 | HOMING_MASK
        if guards & HOMING_MASK and bn.startswith("dispatchScanWork("):
            pruned["homing excludes scan dispatch"] += 1
            return None
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

def validate_owner_snapshot(z, prefix):
    source = prefix + "build/sketch/"
    fingerprint = hashlib.sha256()
    for path in sorted(z.namelist()):
        if path.startswith(source) and path.endswith((".cpp", ".c", ".ino", ".h", ".hpp", ".S")):
            fingerprint.update(path[len(source):].encode() + b"\0" + hashlib.sha256(z.read(path)).digest())
    if fingerprint.hexdigest() not in ['6f43d86c1074e1dacbc7d6ee4a7a7167ae0ca87a1dd410adfab7f84b18c51bc7']:
        raise ValueError("source ownership changed; review before applying lifetime exclusions")


def inventory(z, board, phase=False, reviewed_owners=False):
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
    homing = z.read(p + "build/sketch/Homing.cpp").decode()
    assert homing.count("UIDisplay::ScopedActionExclusion panelExclusion(uid);") == 2
    frame = fg.inventory(z, board, True)
    assert not frame["errors"], frame["errors"]
    assert sum(homing_owner(f["name"]) for f in frame["frames"]) == 2, "missing or cloned homing ownership boundary"
    if reviewed_owners:
        validate_owner_snapshot(z, p)
    return diagnostic(frame, phase, reviewed_owners)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive")
    parser.add_argument("sha256")
    parser.add_argument("--phase-diagnostic", action="store_true", help="apply reviewed-but-not-yet-proven operation lifetime hypotheses")
    parser.add_argument("--reviewed-owners", action="store_true",
                        help="use the reviewed startline and panel-owned autotune lifetimes")
    args = parser.parse_args()
    with open(args.archive, "rb") as f:
        assert hashlib.sha256(f.read()).hexdigest() == args.sha256
    with zipfile.ZipFile(args.archive) as z:
        summary = json.loads(z.read("summary.json"))
        assert not summary["failures"]
        boards = {b: inventory(z, b, args.phase_diagnostic, args.reviewed_owners) for b in fg.BOARDS}
    print(json.dumps({"testedMerge": summary["sha"], "sha256": args.sha256,
                      "reviewedOwners": args.reviewed_owners, "releaseProof": False, "boards": boards,
                      "limitations": ["source guard assumptions require review, not proved by string anchors",
                                      "candidate tables and ABI/weak callback assumptions",
                                      "unresolved phase/admission correlations; opt-in phase hypothesis is not a release proof",
                                      "CRT and interrupt nesting/response excluded"]}, indent=2))

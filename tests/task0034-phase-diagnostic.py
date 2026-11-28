#!/usr/bin/env python3
"""Exercise directional hypotheses without claiming a firmware proof."""
import importlib.util
from pathlib import Path
import sys
sys.dont_write_bytecode=True
path=Path(__file__).resolve().parents[1]/"tools/task0034/contextual-stack-diagnostic.py"
sys.path.insert(0,str(path.parent))
spec=importlib.util.spec_from_file_location("diagnostic",path)
d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d)
assert len(d.GUARDS+d.PHASE_GUARDS)==9
blocked={(1,3),(1,4),(1,5),(1,6),(1,7),(1,8),(3,1),(4,1),(4,6),(4,7),
         (5,1),(5,6),(5,7),(6,1),(6,5),(6,6),(6,7),(7,1),(7,5),(7,6),(7,7),
         (8,1),(8,3),(8,4),(8,5),(8,6),(8,7)}
for entering in range(9):
    for mask in range(1<<9):
        expected=not any((entering,i) in blocked and mask&(1<<i) for i in range(9))
        assert d.phase_compatible(mask,entering)==expected
assert d.phase_compatible(1<<4,6) # soft script -> admitted load
assert not d.phase_compatible(1<<6,4)
assert not d.phase_compatible(1<<8,1) # sole autotune entry owns panel, no scan at admission
# Distinguish the frame that owns the stack bytes from the actual
# instruction owner after a tail transfer. Never grant a new guard
# by accepting the frame name or skipping an unmodeled tail entry.
from copy import deepcopy
cases=[
    ("UIDisplay::executeAction(int)", "UIDisplay::slowAction()"),
    ("dispatchScanWork(bool)", "handleScanWorkTasks(bool)"),
    ("servicePrintFeatures()", "loopFeatures()"),
    ("PrintPause::dispatchPendingMotion()", "PrintPause::serviceMotion()"),
    ("GCodeInput::runSoftFilament(bool)", "GCodeInput::executeSoftFilament(bool)"),
    ("GCodeInput::runHardFilament(bool)", "GCodeInput::executeHardFilament(bool)"),
    ("GCodeInput::runOutputProgram()", "GCodeInput::executeOutputProgram()"),
]
frame={"main":"main", "frames":[{"address":"main","name":"main","localPeakBytes":8}],
       "calls":[], "tailTransfers":[]}
for index,(callee,caller) in enumerate(cases):
    address=str(index)
    frame["frames"].append({"address":address,"name":callee,"localPeakBytes":5})
    frame["calls"].append({"caller":"main","callee":address,
                           "sites":[{"pc":address,"callerBytes":8,"physicalCaller":caller}]})
assert d.diagnostic(frame)["diagnosticBytes"]==13
def reject(candidate,reason):
    try:
        d.diagnostic(candidate)
    except AssertionError as exc:
        assert reason in str(exc), exc
    else:
        raise AssertionError("unreviewed path accepted")
bad=deepcopy(frame);bad["calls"][1]["sites"][0]["physicalCaller"]="unreviewed()"
reject(bad,"unreviewed guard caller")
bad=deepcopy(frame);bad["calls"][5]["sites"][0]["physicalCaller"]="GCodeInput::executeSoftFilament(bool)"
reject(bad,"runHardFilament")
for target,reason in [("dispatchScanWork(bool)","unreviewed tail guard entry"),
                      ("UIDisplay::parse(char*, bool, bool)","unreviewed tail LCD entry")]:
    bad=deepcopy(frame);bad["tailTransfers"].append({"physicalTarget":target})
    reject(bad,reason)
home=deepcopy(frame)
home["frames"].append({"address":"home","name":"Printer::homeAxis(bool, bool, bool)","localPeakBytes":30})
home["calls"].append({"caller":"main","callee":"home","sites":[{"pc":"home","callerBytes":8,"physicalCaller":"Commands::executeGCode(GCode*)"}]})
for address in ("0","1"):
    home["calls"].append({"caller":"home","callee":address,"sites":[{"pc":"nested"+address,"callerBytes":25,"physicalCaller":cases[int(address)][1]}]})
r=d.diagnostic(home)
assert r["diagnosticBytes"]==38
assert r["prunedContextEdges"]["UIDisplay::executeAction("]==1
assert r["prunedContextEdges"]["homing excludes scan dispatch"]==1
bad=deepcopy(home);bad["calls"][-3]["sites"][0]["physicalCaller"]="unreviewed()"
reject(bad,"unreviewed homing caller")
bad=deepcopy(home);bad["tailTransfers"].append({"physicalTarget":"Printer::homeAxis(bool, bool, bool)"})
reject(bad,"unreviewed tail homing entry")
bad=deepcopy(frame);bad["calls"][0]["sites"][0]["physicalCaller"]=d.HOST_MACHINE+"GCode*)"
reject(bad,"unreviewed host dispatch owner")
print("Task0034 phase diagnostic: 4608 directional masks + 9 callsite/tail fixtures PASS")

# The two reviewed lifetime rules require exact prepared-worker callers,
# even when the broad experimental phase model is disabled.
life=deepcopy(frame)
for i,(prefix,allowed) in enumerate(d.REVIEWED_ENTRY_CALLERS.items()):
    address="life"+str(i)
    life["frames"].append({"address":address,"name":prefix+")","localPeakBytes":6})
    caller=allowed[0]+")"
    life["calls"].append({"caller":"main","callee":address,
        "sites":[{"pc":address,"callerBytes":8,"physicalCaller":caller}]})
assert d.diagnostic(life,reviewed_owners=True)["diagnosticBytes"]==14
bad=deepcopy(life)
bad["calls"][-1]["sites"][0]["physicalCaller"]="unreviewed()"
try:
    d.diagnostic(bad,reviewed_owners=True)
except AssertionError as exc:
    assert "unreviewed lifetime caller" in str(exc)
else:
    raise AssertionError("unreviewed lifetime entry accepted")
bad=deepcopy(life)
bad["tailTransfers"].append({"physicalTarget":"StartLine::runPrepared(GCode&)"})
try:
    d.diagnostic(bad,reviewed_owners=True)
except AssertionError as exc:
    assert "unreviewed lifetime tail entry" in str(exc)
else:
    raise AssertionError("unreviewed lifetime tail accepted")
# Exercise the exact conditional linked table checker without AVR tools.
sg_spec=importlib.util.spec_from_file_location('switch_guards',path.with_name('switch-table-guards.py'))
sg=importlib.util.module_from_spec(sg_spec);sg_spec.loader.exec_module(sg)
sg.self_test()

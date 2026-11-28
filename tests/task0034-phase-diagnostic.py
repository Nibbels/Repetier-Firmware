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
blocked={(1,3),(1,4),(1,5),(1,6),(1,7),(3,1),(4,1),(4,6),(4,7),
         (5,1),(5,6),(5,7),(6,1),(6,5),(6,6),(6,7),(7,1),(7,5),(7,6),(7,7),
         (8,1),(8,3),(8,4),(8,5),(8,6),(8,7)}
for entering in range(9):
    for mask in range(1<<9):
        expected=not any((entering,i) in blocked and mask&(1<<i) for i in range(9))
        assert d.phase_compatible(mask,entering)==expected
assert d.phase_compatible(1<<4,6) # soft script -> admitted load
assert not d.phase_compatible(1<<6,4)
assert d.phase_compatible(1<<8,1) # do not assume converse autotune lifetime
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
print("Task0034 phase diagnostic: 4608 directional masks + 5 callsite/tail fixtures PASS")

# Exercise the exact conditional linked table checker without AVR tools.
sg_spec=importlib.util.spec_from_file_location('switch_guards',path.with_name('switch-table-guards.py'))
sg=importlib.util.module_from_spec(sg_spec);sg_spec.loader.exec_module(sg)
sg.self_test()

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
print("Task0034 phase diagnostic: 4608 directional mask hypotheses PASS")

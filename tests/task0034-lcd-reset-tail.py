#!/usr/bin/env python3
"""Narrow regression for the LCD confirmation tail-jump reset ownership."""
from pathlib import Path
import importlib.util
import sys

# The canonical evidence capture requires an unchanged checkout.
sys.dont_write_bytecode = True

tool = Path(__file__).resolve().parents[1] / "tools" / "task0034"
sys.path.insert(0,str(tool))
spec=importlib.util.spec_from_file_location("fg",tool/"foreground-stack-depth.py")
fg=importlib.util.module_from_spec(spec)
spec.loader.exec_module(fg)
pc=0x14bf4
ins={
    pc-4:(2,"ldi","r31, 0x00\t; 0"),
    pc-2:(2,"ldi","r30, 0x00\t; 0"),
    pc:(2,"eicall",""),
    pc+2:(2,"rjmp",".-2\t; 0x14bf6 <Commands::emergencyStop()+0x42>"),
}
owner="Commands::emergencyStop()"
assert fg.terminal_reset_site(ins,pc,owner)
assert not fg.terminal_reset_site(ins,pc,"UIDisplay::finishAction(int)")
for addr,inst in ((pc-4,(2,"ldi","r31, 0x01")),
                  (pc-2,(2,"ldi","r29, 0x00")),
                  (pc,(2,"icall","")),
                  (pc+2,(2,"rjmp",".+2\t; 0x14bfa <other>")),
                  (pc+2,(2,"ret",""))):
    broken=dict(ins)
    broken[addr]=inst
    assert not fg.terminal_reset_site(broken,pc,owner)
print("LCD reset-tail physical ownership: 7 cases PASS")

#!/usr/bin/env python3
"""The optional serialEventRun guard must not suppress a reachable CALL 0."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"tools/task0034"))
from weak_serial_guard import prove_weak_main

def target(arg):return int(arg,0)
def check():
    # Normal call preserves the two callee-saved registers, then CP/CPC/BREQ.
    ins={2:(2,'ldi','r16, 0x00'),4:(2,'ldi','r17, 0x00'),
         6:(4,'call','0x40'),10:(2,'cp','r16, r1'),12:(2,'cpc','r17, r1'),
         14:(2,'breq','0x14'),16:(4,'call','0x0'),20:(2,'ret','')}
    assert prove_weak_main(ins,{},2,target)=={16}
    bad=dict(ins);bad[4]=(2,'ldi','r17, 0x01')
    assert not prove_weak_main(bad,{},2,target)
    bad=dict(ins);bad[4]=(2,'lds','r17, 0x01')
    assert not prove_weak_main(bad,{},2,target)
    bad=dict(ins);bad[6]=(4,'jmp','0x10')
    assert not prove_weak_main(bad,{},2,target)
    bad=dict(ins);bad[0]=(2,'brne','0x10')
    assert not prove_weak_main(bad,{},0,target)
    bad=dict(ins);bad[6]=(4,'mul','r2, r3')
    assert not prove_weak_main(bad,{},2,target)
    bad=dict(ins);bad[10]=(2,'cp','r16, r2')
    assert not prove_weak_main(bad,{},2,target)
check()
print("Task0034 weak callback guard: valid and six rejected CFGs PASS")

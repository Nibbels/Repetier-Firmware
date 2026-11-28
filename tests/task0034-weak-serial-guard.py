#!/usr/bin/env python3
"""The optional serialEventRun guard must not suppress a reachable CALL 0."""
from pathlib import Path
import sys
sys.dont_write_bytecode = True
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
    word={2:(2,'ldi','r28, 0x00'),4:(2,'ldi','r29, 0x00'),
          6:(4,'call','0x40'),10:(2,'sbiw','r28, 0x00'),
          12:(2,'breq','0x12'),14:(4,'call','0x0'),18:(2,'ret','')}
    assert prove_weak_main(word,{},2,target)=={14}
    for pc,insn in [(4,(2,'ldi','r29, 0x01')),(4,(2,'lds','r29, 0x01')),
                    (10,(2,'sbiw','r28, 0x01')),(6,(4,'jmp','0xe')),
                    (6,(4,'ld','r24, Y+')),(6,(4,'st','-Y, r24'))]:
        bad=dict(word);bad[pc]=insn
        assert not prove_weak_main(bad,{},2,target)
    # Linked after internal parser removal: LDI r24/r25, OR, BREQ.
    pair={2:(2,'ldi','r24, 0x00'),4:(2,'ldi','r25, 0x00'),
          6:(2,'or','r24, r25'),8:(2,'breq','0xe'),
          10:(4,'call','0x0'),14:(2,'ret','')}
    assert prove_weak_main(pair,{},2,target)=={10}
    for pc,insn in [(2,(2,'ldi','r24, 0x01')),(4,(2,'ldi','r25, 0x01')),
                    (4,(2,'lds','r25, 0x01')),(6,(2,'or','r24, r26')),
                    (6,(2,'call','0x40')),(6,(2,'jmp','0xa')),
                    (2,(2,'jmp','0x8'))]:
        bad=dict(pair);bad[pc]=insn
        assert not prove_weak_main(bad,{},2,target)
    bad=dict(pair);bad[0]=(2,'brne','0xa')
    assert not prove_weak_main(bad,{},0,target)
    clobber={2:(2,'ldi','r24, 0x00'),4:(2,'ldi','r25, 0x00'),
             6:(4,'call','0x40'),10:(2,'or','r24, r25'),
             12:(2,'breq','0x12'),14:(4,'call','0x0'),18:(2,'ret','')}
    assert not prove_weak_main(clobber,{},2,target)
check()
print("Task0034 weak callback guard: three valid and twenty-one rejected CFGs PASS")

"""Bound snapshot-to-drive path in linked Timer2 probe, not silicon WCET."""
import json
import re
import sys
from pathlib import Path
from functools import lru_cache

def audit(dump):
    blocks={}
    block=None
    for line in dump.splitlines():
        m=re.match(r'^([0-9a-f]+) <(.+)>:$',line)
        if m:
            block=blocks[m[2]]={}
        m=re.match(r'^\s*([0-9a-f]+):\s*((?:[0-9a-f]{2}\s+)+)\s*(\S+)\s*(.*)$',line)
        if m and block is not None:
            block[int(m[1],16)]=(len(bytes.fromhex(m[2])),m[3],m[4])
    probe=blocks['probePulse']
    drive=blocks['AvrTimer2CompareClear::drive(unsigned char)']
    assert any(op=='sbi' and arg.startswith('0x04, 4') for _,op,arg in drive.values())
    assert any(op=='sts' and arg.startswith('0x0101,') for _,op,arg in drive.values())
    assert all(op not in ('call','rcall','icall','eicall','ijmp','eijmp') for _,op,_ in drive.values())
    def target(arg):
        return int(re.search(r'(?:^|;\s*)0x([0-9a-f]+)\b',arg)[1],16)
    start=min(a for a,(_,op,arg) in probe.items() if op=='lds' and re.search(r', 0x00B2\b',arg))
    ends={a for a,(_,op,arg) in probe.items() if op=='call' and '::drive(unsigned char)' in arg}
    assert len(ends)==1
    edges={}
    for a,(size,op,arg) in probe.items():
        n=a+size
        if a in ends or op in ('ret','reti'): e=[]
        elif op in ('rjmp','jmp'): e=[target(arg)]
        elif op.startswith('br'): e=[n,target(arg)]
        elif op in ('cpse','sbrs','sbrc','sbic','sbis'): e=[n,n+probe[n][0]]
        else: e=[n]
        edges[a]=e
    reach=set(ends)
    while True:
        new=reach|{a for a,e in edges.items() if any(p in reach for p in e)}
        if new==reach: break
        reach=new
    assert start in reach
    active=set()
    @lru_cache(None)
    def walk(a):
        if a in ends: return 5+5*len(drive) # all instructions, including mutually exclusive ones
        assert a not in active, ('cycle',hex(a))
        active.add(a)
        _,op,arg=probe[a]
        assert op not in ('call','rcall','icall','eicall','ijmp','eijmp'), (op,arg)
        n=5+max(walk(p) for p in edges[a] if p in reach)
        active.remove(a)
        return n
    for a,(_,op,arg) in drive.items():
        if op.startswith('br') or op in ('jmp','rjmp'): assert target(arg)>a
    cycles=walk(start)
    assert cycles<1024, cycles
    # Datasheet: match action/flag may be on the following timer clock.
    # Allow one additional tick beyond the 254-tick programmed duration.
    return {'snapshot_to_drive_cycles_upper':cycles,'timer_tick_cycles':1024,
            'clock_hz':16000000,'max_nominal_high_ticks_with_compare_latency':255}

if __name__=='__main__':
    print(json.dumps(audit(Path(sys.argv[1]).read_text())))

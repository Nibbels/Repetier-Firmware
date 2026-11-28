#!/usr/bin/env python3
"""Conservative linked TCNT snapshot -> OCR commit bounds, not system WCET.
All instructions charged five cycles (ATmega2560 3-byte PC); the one
variable mask shift in leaseLive is bounded by validated channels 0..2.
"""
import json
import re
import sys
from pathlib import Path
from functools import lru_cache

def audit(dump):
    functions={}
    fn=None
    for line in dump.splitlines():
        m=re.match(r'^([0-9a-f]+) <(.+)>:$',line)
        if m:
            fn=m[2]
            functions[fn]={}
            continue
        m=re.match(r'^\s*([0-9a-f]+):\s*((?:[0-9a-f]{2}\s+)+)\s*(\S+)\s*(.*)$',line)
        if m and fn:
            functions[fn][int(m[1],16)]=(len(bytes.fromhex(m[2])),m[3],m[4])
    def find(prefix):
        matches=[v for k,v in functions.items() if k.startswith(prefix)]
        assert len(matches)==1,prefix
        return matches[0]
    def target(arg):
        m=re.search(r'(?:^|;\s*)0x([0-9a-f]+)\b',arg)
        assert m,arg
        return int(m[1],16)
    def graph(block,ends):
        g={}
        for a,(size,op,arg) in block.items():
            nxt=a+size
            if a in ends or op in ('ret','reti','ijmp','eijmp'):
                edges=[]
            elif op in ('rjmp','jmp'):
                edges=[target(arg)]
            elif op.startswith('br'):
                edges=[target(arg),nxt]
            elif op in ('cpse','sbrs','sbrc','sbis','sbic'):
                edges=[nxt,nxt+block[nxt][0]]
            else:
                edges=[nxt]
            g[a]=edges
        return g
    def relevant(g,ends):
        reach=set(ends)
        while True:
            new=reach|{a for a,edges in g.items() if any(p in reach for p in edges)}
            if new==reach:return reach
            reach=new
    def longest(block,g,start,ends,extra):
        reach=relevant(g,ends)
        assert start in reach
        active=set()
        @lru_cache(None)
        def walk(a):
            assert a not in active,('unbounded loop',hex(a))
            if a in ends:return 5
            active.add(a)
            op,arg=block[a][1:]
            x=extra(op,arg)
            out=5+x+max(walk(p) for p in g[a] if p in reach)
            active.remove(a)
            return out
        return walk(start)
    lease=find('(anonymous namespace)::leaseLive(')
    setter=find('HAL::setHeaterDuty(unsigned char, unsigned char, HAL::HeaterSample*)')
    feed=find('tryThermalWatchdogFeed(')
    calls=('call','rcall','icall','eicall')
    assert not any(op in calls+('ijmp','eijmp') for _,op,_ in lease.values())
    ret={a for a,(_,op,_) in lease.items() if op=='ret'}
    g=graph(lease,ret)
    # Only the compiler-generated channel mask shift ADD/DEC/BRPL is
    # bounded manually. Reject another loop in the linked commit paths.
    loops=[(a,target(arg)) for a,(_,op,arg) in lease.items() if op=='brpl' and target(arg)<a]
    assert len(loops)==1,loops
    a,b=loops[0]
    assert [lease[p][1] for p in sorted(lease) if b<=p<=a]==['add','dec','brpl']
    g[a].remove(b)
    def no_call(op,arg):
        assert op not in calls,(op,arg)
        return 0
    leaf=longest(lease,g,min(lease),ret,no_call)+2*3*5
    leaf=max(leaf,5*(len(lease)+6))
    def interval(block,read,write,extra):
        starts=[a for a,(_,op,t) in block.items() if op=='lds' and read in t]
        assert starts
        ends={a for a,(_,op,t) in block.items() if op=='sts' and re.search(write,t)}
        assert ends
        return longest(block,graph(block,ends),min(starts),ends,extra)
    def lease_call(op,arg):
        if op in calls:
            assert '::leaseLive(' in arg,(op,arg)
            return leaf
        return 0
    timer3=interval(setter,'0x0094',r'0x009[8AC]',lease_call)
    timer5=interval(feed,'0x0124',r'0x0128',no_call)
    # Four ticks = 1024 CPU cycles, accepted remaining count >4.
    assert max(timer3,timer5)<4*256,(timer3,timer5)
    return {'timer3_cycles':timer3,'timer5_cycles':timer5,'lease_leaf_cycles':leaf,
            'guard_cycles':1024,'clock_hz':16000000}

if __name__=='__main__':
    for path in sys.argv[1:]:
        print(path,json.dumps(audit(Path(path).read_text())))

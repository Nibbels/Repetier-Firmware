#!/usr/bin/env python3
"""Bound the archived linked LCD foreground service, excluding preemption.

Exact-image structural paths with source-reviewed loop trip caps. This is
not a global firmware WCET or an interrupt-response/timebase proof. No
machine-code rewriting, external tools or additional persistent state.
"""
import argparse
import functools
import hashlib
import json
from pathlib import Path
import re
import sys
import zipfile

ARCHIVE = '1482fc11e489d893936298d61eaa0553a0c43f399a7cf21ca3808a40e6583290'
CANDIDATE = '271567df72aa29acb99fb3817d6c8e9c885198d0'
BOARDS = ('RF1000', 'RF2000', 'RF2000v2', 'RF2000-X44')
# Offset of loop-closing instruction -> (target offset, max total taken,
# reviewed source meaning). Row-search shifts total at most 0+1+2+3;
# the row loop visits the same four rows in normal or emergency order.
CAP16 = {
    0x92: (0x8c, 6, 'row summary variable shift, summed over all rows'),
    0xc6: (0xb6, 7, 'first set bit of nonzero dirty byte'),
    0xe6: (0xe0, 5, 'row address shift by six'),
    0x14c: (0x74, 3, 'advance to next row'),
    0x168: (0x162, 3, 'row times sixteen'),
    0x1a2: (0x19e, 7, 'clear one dirty bit'),
    0x1c0: (0x1bc, 3, 'clear row summary bit'),
}
CAP20 = {
    0x8e: (0x88, 6, 'row summary variable shift, summed over all rows'),
    0x98: (0x70, 3, 'advance to next row'),
    0xf2: (0xe2, 7, 'first set bit of nonzero dirty byte'),
    0x112: (0x10c, 5, 'row address shift by six'),
    0x1bc: (0x1b6, 7, 'clear one dirty bit'),
    0x20c: (0x208, 3, 'clear row summary bit'),
}
ONE = set('adc add and andi asr cli com cp cpi dec eor in inc ldi lsr mov movw nop ori out ror sbc sbci sub subi swap'.split())
TWO = set('adiw cbi ld ldd lds mul pop push sbi st std sts'.split())
BRANCH = set('brcc brcs breq brne brpl'.split())
SKIP = set('sbrc sbrs sbic sbis cpse'.split())

def target(arg):
    m = re.search(r';\s*0x([0-9a-f]+)', arg) or re.match(r'0x([0-9a-f]+)', arg)
    assert m, arg
    return int(m[1], 16)


def analyze(z, board, stats):
    prefix = 'candidate-evidence/'+board+'/'
    assert json.loads(z.read(prefix+'verification.json')) == {'status':'verified','recovery':True}
    for filename, key in [('Repetier.ino.elf','elf_sha256'),('firmware.map','map_sha256')]:
        assert hashlib.sha256(z.read(prefix+'build/'+filename)).hexdigest()==stats[key]
    syms = {}
    for line in z.read(prefix+'symbols.txt').decode().splitlines():
        m = re.fullmatch(r'([0-9a-f]+) ([0-9a-f]+) [tT] (.+)',line)
        if m:
            name=m[3].split('(')[0]
            if name in ('UIDisplay::serviceDisplay','UIDisplay::serviceDisplayPending','lcdWriteByteNow','micros','millis'):
                assert name not in syms
                syms[name]=(int(m[1],16),int(m[2],16))
    assert len(syms)==5, syms
    ins = {}
    for line in z.read(prefix+'disassembly.txt').decode().splitlines():
        m=re.match(r'^\s*([0-9a-f]+):\s+((?:[0-9a-f]{2}\s+)+)([a-z]+)\s*(.*?)$',line)
        if m:
            addr=int(m[1],16)
            if any(a<=addr<a+n for a,n in syms.values()):
                ins[addr]=(len(bytes.fromhex(m[2])),m[3],m[4])
    caps = CAP16 if board=='RF1000' else CAP20
    bounds = {}
    p=syms['UIDisplay::serviceDisplayPending'][0]
    for off,(dest,cap,meaning) in caps.items():
        assert target(ins[p+off][2])==p+dest,(board,off)
        bounds[p+off,p+dest]=cap
    m=syms['micros'][0]
    assert ins[m+0x46][1]=='brne' and target(ins[m+0x46][2])==m+0x3c
    assert ins[m+0x3a][1]=='ldi' and ins[m+0x3a][2].startswith('r20, 0x02')
    bounds[m+0x46,m+0x3c]=1  # per micros call
    entry={a:name for name,(a,n) in syms.items()}
    sys.setrecursionlimit(10000)
    leaf={}
    state_counts={}
    def analyze_function(name):
        start,size=syms[name]
        code={pc:i for pc,i in ins.items() if start<=pc<start+size}
        local_bounds={e:(1 if name=='micros' else cap) for e,cap in bounds.items() if e[0] in code}
        keys=tuple(local_bounds)
        limits=tuple(local_bounds.values())
        edges={}
        terminal={}
        for pc,(w,op,arg) in code.items():
            nxt=pc+w
            if op=='ret': terminal[pc]={'no_byte':5};edges[pc]=[];continue
            if op in ONE: e=[(nxt,1,'no_byte')]
            elif op in TWO: e=[(nxt,2,'no_byte')]
            elif op in BRANCH:
                e=[(nxt,1,'no_byte'),(target(arg),2,'no_byte')]
                if name=='UIDisplay::serviceDisplayPending' and pc==p+0x56:
                    assert op==('brcs' if board=='RF1000' else 'brcc')
                    idx=0 if board=='RF1000' else 1
                    dest,cost,_=e[idx];e[idx]=(dest,cost,'deadline_wait')
            elif op in SKIP:
                width=code[nxt][0]; assert width in (2,4)
                e=[(nxt,1,'no_byte'),(nxt+width,1+width//2,'no_byte')]
            elif op in ('jmp','rjmp'):
                dest=target(arg);cost=3 if op=='jmp' else 2
                if dest not in code:
                    assert entry[dest]=='UIDisplay::serviceDisplayPending' and name=='UIDisplay::serviceDisplay'
                    terminal[pc]={k:cost+v for k,v in leaf[entry[dest]].items()};edges[pc]=[];continue
                e=[(dest,cost,'no_byte')]
            elif op in ('call','rcall'):
                dest=target(arg); callee=entry[dest];cost=(5 if op=='call' else 4)+leaf[callee]['no_byte']
                kind='no_byte'
                if callee=='lcdWriteByteNow':
                    rel=pc-p
                    assert rel in ((0xfe,0x176) if board=='RF1000' else (0x12a,0x18e)),(board,hex(rel))
                    kind='address' if rel in (0xfe,0x12a) else 'data'
                e=[(nxt,cost,kind)]
            else:raise AssertionError((board,hex(pc),op,arg))
            assert all(x[0] in code for x in e),(board,name,hex(pc),e)
            edges[pc]=e
        # Counters for a loop no longer reachable can be forgotten. This
        # removes products of independent sequential loop counters.
        reverse={pc:[] for pc in code}
        for pc,es in edges.items():
            for dest,_,_ in es:reverse[dest].append(pc)
        relevant={pc:[] for pc in code}
        for idx,(src,_) in enumerate(keys):
            seen=set();todo=[src]
            while todo:
                pc=todo.pop()
                if pc in seen:continue
                seen.add(pc);relevant[pc].append(idx);todo.extend(reverse[pc])
        active=set()
        def dispatch(pc,counts):
            indices=relevant[pc]
            return walk(pc,tuple(counts[i] if i in indices else 0 for i in range(len(keys))))
        @functools.lru_cache(None)
        def walk(pc,counts):
            state=pc,counts
            assert state not in active,('unbounded cycle',board,name,hex(pc))
            if pc in terminal:return terminal[pc]
            active.add(state);result={}
            for dest,cost,kind in edges[pc]:
                nextcounts=counts
                if (pc,dest) in local_bounds:
                    idx=keys.index((pc,dest))
                    if counts[idx]>=limits[idx]:continue
                    nextcounts=counts[:idx]+(counts[idx]+1,)+counts[idx+1:]
                for k,v in dispatch(dest,nextcounts).items():
                    assert kind=='no_byte' or k=='no_byte',(board,name,'multiple GPIO calls')
                    k=k if kind=='no_byte' else kind
                    result[k]=max(result.get(k,0),v+cost)
            active.remove(state)
            return result
        result=dispatch(start,(0,)*len(keys))
        state_counts[name]=walk.cache_info().currsize
        walk.cache_clear()
        return result
    for name in ('micros','millis','lcdWriteByteNow','UIDisplay::serviceDisplayPending','UIDisplay::serviceDisplay'):
        leaf[name]=analyze_function(name)
    result=leaf['UIDisplay::serviceDisplay']
    # Audit each explicit local critical section in the leaf functions.
    # The archived instruction sequence always saves SREG immediately before
    # CLI. Walk both paths up to the matching restore, including both edges.
    masked={}
    for name in ('micros','millis','lcdWriteByteNow'):
        start,size=syms[name];code={pc:v for pc,v in ins.items() if start<=pc<start+size}
        local=[]
        for pc,(_,op,_) in code.items():
            if op!='cli':continue
            _,save,args=code[pc-2]
            assert save=='in' and ', 0x3f' in args,(name,hex(pc),args)
            reg=args.split(',')[0]
            active=set()
            def to_restore(at):
                assert at in code and at not in active,(name,hex(at))
                active.add(at)
                w,op,arg=code[at];n=at+w;a=arg.split(';')[0].strip()
                if op=='out' and a=='0x3f, '+reg:
                    active.remove(at);return 1
                assert op not in ('cli','ret','call','rcall'),(name,hex(at),op)
                # Saved SREG register must not be modified by the body.
                assert a.split(',')[0]!=reg or op in ('cp','cpi','cpse','sbrc','sbrs'),(name,hex(at),a)
                if op in ONE:es=[(n,1)]
                elif op in TWO:es=[(n,2)]
                elif op in BRANCH:es=[(n,1),(target(arg),2)]
                elif op in SKIP:es=[(n,1),(n+code[n][0],1+code[n][0]//2)]
                elif op in ('jmp','rjmp'):es=[(target(arg),3 if op=='jmp' else 2)]
                else:raise AssertionError((name,hex(at),op))
                result=max(cost+to_restore(dest) for dest,cost in es)
                active.remove(at);return result
            local.append(1+to_restore(pc+2))
        masked[name]={'sections':len(local),'max_cycles_including_cli_and_restore':max(local)}
    # Full run includes wrapper, calls, and final return, but excludes its caller.
    return {'symbols':{k:{'address':hex(a),'bytes':n} for k,(a,n) in syms.items()},
            'max_cycles_by_visit':result,'max_us_at_16mhz':{k:v/16 for k,v in result.items()},
            'loop_caps':{hex(off):{'destination':hex(d),'max_taken':c,'meaning':why} for off,(d,c,why) in caps.items()},
            'helper_max_cycles':leaf, 'explicit_mask_windows':masked, 'explored_states':state_counts}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('archive',type=Path);a=p.parse_args()
    with a.archive.open('rb') as f:assert hashlib.file_digest(f,'sha256').hexdigest()==ARCHIVE
    with zipfile.ZipFile(a.archive) as z:
        report=json.loads(z.read('candidate-evidence/summary.json'))
        assert report['sha']==CANDIDATE and not report['failures']
        rows={b:analyze(z,b,report['targets'][b]) for b in BOARDS}
    print(json.dumps({'archive_sha256':ARCHIVE,'candidate':CANDIDATE,'boards':rows,
        'limitations':['source-reviewed finite loop bounds; independent branches can be infeasible',
                       'excludes caller CALL, preemption, rendering, foreground cadence, and timebase correctness',
                       'exact archived image only, re-evaluate after final P0 integration']},indent=2))
if __name__=='__main__':main()

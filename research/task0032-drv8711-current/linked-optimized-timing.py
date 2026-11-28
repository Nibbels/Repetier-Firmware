#!/usr/bin/env python3
"""Fail-closed timing audit of the canonical linked DRV8711 candidate.
Usage: python3 linked-optimized-timing.py firmware-evidence.zip
Counts ATmega2560 AVRe instructions; not concurrent WCET or hardware validation.
"""
import hashlib
import json
import re
import sys
import zipfile

TX_HASH = '7cc21dcf958451cf47ccc8f9244d01e381618cc370dd517b489b9c5b563edda0'
MODELS = ('RF1000','RF2000','RF2000v2','RF2000-X44')

def function(dis, pattern):
    m = re.search(r'^([0-9a-f]+) <[^\n]*' + pattern + r'[^\n]*>:\n', dis, re.M)
    assert m, pattern
    end = re.search(r'^[0-9a-f]+ <[^\n]+>:\n', dis[m.end():], re.M)
    block = dis[m.end():m.end()+end.start()]
    code = {}
    for line in block.splitlines():
        if re.match(r'^\s*[0-9a-f]+:\t', line):
            parts = line.split('\t')
            pc = int(parts[0].strip()[:-1], 16)
            code[pc] = (bytes.fromhex(parts[1]), parts[2], parts[3].strip() if len(parts)>3 else '')
    return int(m[1],16), code

def tx_cycles(word):
    # Pinned 32 bytes: MOVW/LDI (2); SBRS+SBI (4) or SBRS+RJMP+CBI+RJMP (7);
    # SBI/NOP/NOP/CBI (6); ADD/ADC/SUBI/BRNE (5, last BRNE 1); RET (5).
    return 241 + 3*(16-word.bit_count())

def wrapper(start, code, tx, channel, word, model):
    # Restricted CFG walker: channel comparisons precede register clobber;
    # every path must select exactly the requested physical GPIO, then clear it.
    pc, cycles, masked, flags = start, 0, None, None
    masks, edges = [], []
    expected = [(0x10b,1),(0x10b,4),(0x10b,16) if model=='RF1000' else (0x25,64),
                (0x10b,64),(0x105,128)][channel-1]
    loaded, modified = None, None
    call_count = 0
    fixed = {'push':2,'pop':2,'mov':1,'movw':1,'ldi':1,'eor':1,'swap':1,'or':1,
             'in':1,'out':1,'cli':1,'lds':2,'sts':2,'ori':1,'andi':1,'sbi':2,'cbi':2,'nop':1}
    for _ in range(120):
        raw, op, args = code[pc]
        following = pc + len(raw)
        a = [v.strip() for v in args.split(',')]
        if op == 'cpi':
            assert a[0] in ('r24','r28')
            flags = (channel == int(a[1],0), channel >= int(a[1],0))
            cycles += 1
        elif op in ('brne','breq','brcc','rjmp'):
            take = op=='rjmp' or {'brne':not flags[0],'breq':flags[0],'brcc':flags[1]}[op]
            # rjmp and branches are relative to the following byte address.
            if take:
                following += int(args[1:])
            cycles += 2 if take else 1
        elif op == 'call':
            assert int(args,0)==tx and masked is None and len(edges)==1
            call_count += 1
            cycles += 5 + tx_cycles(word) + 5
        elif op == 'ret':
            cycles += 5
            assert masked is None and call_count==1 and len(masks)==2
            assert [v for _,v in edges]==[1,0] and cycles-edges[-1][0]>=2
            return cycles, masks
        else:
            assert op in fixed, (pc,op,args)
            cycles += fixed[op]
            if op=='cli':
                assert masked is None
                masked=cycles
            elif op=='out' and int(a[0],0)==0x3f:
                assert masked is not None
                masks.append(cycles-masked); masked=None
            elif op=='lds':
                assert masked is not None
                loaded=int(a[1],0); modified=None
            elif op in ('ori','andi') and masked is not None:
                modified=(op=='ori', int(a[1],0))
            elif op=='sts':
                assert masked is not None and loaded==int(a[0],0)==expected[0]
                on, mask=modified
                assert mask==(expected[1] if on else 255^expected[1])
                edges.append((cycles,int(on)))
            elif op in ('sbi','cbi'):
                assert masked is not None and (int(a[0],0)+0x20,1<<int(a[1],0))==expected
                edges.append((cycles,int(op=='sbi')))
        pc=following
    raise AssertionError('unbounded wrapper path')

def main(path):
    rows={}
    with zipfile.ZipFile(path) as z:
        summary=json.loads(z.read('summary.json'))
        assert not summary['failures'] and set(summary['targets'])==set(MODELS)
        for model in MODELS:
            elf=z.read(model+'/build/Repetier.ino.elf')
            assert hashlib.sha256(elf).hexdigest()==summary['targets'][model]['elf_sha256']
            dis=z.read(model+'/disassembly.txt').decode()
            tx, code=function(dis,r'drv8711Transmit\(unsigned int\)')
            assert hashlib.sha256(b''.join(v[0] for v in code.values())).hexdigest()==TX_HASH
            start, code=function(dis,r'writeRegister<false>')
            cases=[wrapper(start,code,tx,p,0x1100|s,model) for p in range(1,6) for s in range(256)]
            masks=[n for _,ms in cases for n in ms]
            assert max(masks)<=12
            rows[model]={'tx_addr':hex(tx),'tx_cycles_excluding_ret':[min(tx_cycles(0x1100|s) for s in range(256)),max(tx_cycles(0x1100|s) for s in range(256))],
                         'wrapper_cycles_including_ret':[min(c for c,_ in cases),max(c for c,_ in cases)],
                         'max_mask_after_cli_cycles':max(masks),'cases':len(cases),
                         'flash':summary['targets'][model]['flash_bytes'],'sram':summary['targets'][model]['global_sram_bytes']}
    print(json.dumps({'tested_merge_sha':summary['sha'],'tx_sha256':TX_HASH,'targets':rows,
                      'sclk_high_cycles':4,'sclk_low_cycles':[11,14],'sdati_setup_cycles':[2,4]},indent=2))

if __name__=='__main__':
    main(sys.argv[1])

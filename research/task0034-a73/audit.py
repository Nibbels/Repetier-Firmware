#!/usr/bin/env python3
"""Read-only A73 evidence ledger. Python 3 + GNU readelf, no AVR compiler.
CFI is compiler frame metadata, not WCET/stack watermark; ISR extra saves
(SREG/RAMPZ) are counted separately from the initial machine prologue.
Only explicit calls are reported; no indirect/tail calls or reachability proof.
"""
import bisect, hashlib, json, re, subprocess, sys, tempfile, zipfile
from pathlib import Path

def audit(path):
    z=zipfile.ZipFile(path); names=set(z.namelist()); summary=json.loads(z.read('summary.json'))
    out=dict(archive=Path(path).name, sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(), tested_sha=summary['sha'], failures=summary['failures'], images=[])
    for p in sorted(n[:-len('disassembly.txt')] for n in names if n.endswith('/disassembly.txt')):
        if 'layout-probe' in p or '-failed/' in p: continue
        board=p.rstrip('/').split('/')[-1]; mode=p.split('/')[2] if p.startswith('host/experimental-avr/') else 'standard'
        e=p+('build/' if mode=='standard' else '')+'Repetier.ino.elf'; m=p+('build/' if mode=='standard' else '')+'firmware.map'
        if e not in names: continue
        stats=summary['targets'][board] if mode=='standard' else json.loads(z.read(p+'stats.json'))
        elf=z.read(e); assert hashlib.sha256(elf).hexdigest()==stats['elf_sha256']; assert hashlib.sha256(z.read(m)).hexdigest()==stats['map_sha256']
        with tempfile.NamedTemporaryFile() as f:
            f.write(elf);f.flush(); cfi=subprocess.check_output(['readelf','--debug-dump=frames-interp',f.name],text=True)
        frames={}
        for block in re.split(r'(?=^[0-9a-f]+ .* FDE .*pc=)',cfi,flags=re.M):
            match=re.search(r'FDE .*pc=([0-9a-f]+)\.\.([0-9a-f]+)',block)
            if match: frames[int(match[1],16)]=max([3]+[int(x) for x in re.findall(r'^\s*[0-9a-f]+\s+r(?:28|32)\+(\d+)',block,re.M)])
        syms={int(a,16):(int(s,16),n) for a,s,n in re.findall(r'^([0-9a-f]+) ([0-9a-f]+) [tT] (.+)$',z.read(p+'symbols.txt').decode(),re.M)}
        ins={int(a,16):(op,arg) for a,op,arg in re.findall(r'^\s*([0-9a-f]+):\s+(?:[0-9a-f]{2} )+\s+([a-z][a-z0-9.]*)\s*(.*?)$',z.read(p+'disassembly.txt').decode(),re.M)}
        pcs=sorted(ins)
        functions={}
        for a,(size,n) in syms.items():
            if a not in frames:continue
            body=[(i,*ins[i]) for i in pcs[bisect.bisect_left(pcs,a):bisect.bisect_left(pcs,a+size)]]
            # Skip PROGMEM objects without an FDE. For ISR prologue, CFI
            # omits SREG/RAMPZ pushes; count actual contiguous initial saves.
            saves=0
            if re.fullmatch(r'__vector_\d+',n):
                for _,op,arg in body:
                    if op=='push':saves+=1
                    elif op in ('in','eor','clr'):pass
                    else:break
            calls=[]
            for i,op,arg in body:
                if op in ('call','rcall'):
                    t=re.search(r';\s*0x([0-9a-f]+)\s*<',arg) or re.match(r'0x([0-9a-f]+)',arg)
                    if t and int(t[1],16) in syms and int(t[1],16)!=i+2:calls.append(syms[int(t[1],16)][1])
            functions[n]={'bytes':size,'cfi_including_pc':frames[a],'initial_isr_pushes':saves,'direct_calls':sorted(set(calls)),'sei_count':sum(op=='sei' for _,op,_ in body)}
        keep={n:v for n,v in functions.items() if re.search(r'^(?:__vector_\d+|TemperatureController::autotunePID\(|Extruder::manageTemperatures\(|Commands::checkForPeriodicalActions\(|serviceThermalEmergency\(|.*failHeaterLease\(|.*checkHeaterLeases\(|.*pingWatchdog\(|.*watchdogProgressLive\(|ThermalEmergency::confirm\(|.*reportRelayObservation\(|.*report.*Emergency|.*printFloat\(|.*refreshPage\(|.*executeGCode\()',n)}
        chains=[]
        for n,v in keep.items():
            if 'autotunePID(' not in n:continue
            for b in v['direct_calls']:
                if 'checkForPeriodicalActions' not in b or b not in functions:continue
                for c in functions[b]['direct_calls']:
                    if ('manageTemperatures' in c or 'serviceThermalEmergency' in c) and c in functions:
                        chains.append([n,b,c,sum(functions[k]['cfi_including_pc'] for k in (n,b,c))])
        maptext=z.read(m).decode(); pad=re.search(r'^\s*0x([0-9a-f]+)\s+__rf_watchdog_padding_start\s*=',maptext,re.M)
        out['images'].append(dict(board=board,mode=mode,flash=stats['flash_bytes'],sram=stats['global_sram_bytes'],flash_remaining=253952-stats['flash_bytes'],sram_remaining=8192-stats['global_sram_bytes'],recovery_gap=131072-int(pad[1],16) if pad else None,functions=keep,chains=chains))
    out['probes']={n: {k:v for k,v in json.loads(z.read(n)).items() if k in ('flash_bytes','global_sram_bytes')} for n in names if n.endswith('stats.json') and 'layout-probe' in n}
    return out
if __name__=='__main__':
    print(json.dumps([audit(p) for p in sys.argv[1:]],indent=2))

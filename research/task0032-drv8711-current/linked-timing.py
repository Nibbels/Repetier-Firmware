#!/usr/bin/env python3
"""Exact static path count for one known linked AVR routine; fail on new opcodes.
Input: canonical firmware-evidence ZIP (all four builds).
Output: JSON with SHA, per-board function address, cycle/edge and UART screens.
Not an autonomous AVR/peripheral emulator. No hardware experiments.
"""
import hashlib
import json
import re
import sys
import zipfile

EXPECTED = "17cd851afd546833f72c1e7db76b368300c868e79577d6a71d560c709aa6211f"
BOARDS = ("RF1000", "RF2000", "RF2000v2", "RF2000-X44")
F_CPU = 16_000_000

def sha(data):
    return hashlib.sha256(data).hexdigest()

def trace(word):
    # Boundaries: cycle 0 = CLI completed. GPIO edges at instruction completion.
    # Six LDI before first iteration. Delay(1) compiled away.
    t = 6
    edges = []
    for i in range(15,-1,-1):
        bit = (word >> i) & 1
        # MOVW, MOV, RJMP, DEC, BRPL + i (ADD, ADC, DEC, BRPL).
        # Initial/final BRPL: one untaken, i taken.
        t += 1+1+2+1+1+5*i
        t += 3  # AND, AND, OR
        t += 1 if bit else 2  # BREQ
        t += 2  # SBI or CBI data
        data_at = t
        if not bit:
            t += 2  # RJMP to SCLK high
        t += 2  # SBI clock
        rise = t
        t += 1 + 15*2 + 14*2+1  # MOVW + SBIW/BRNE delay loop
        t += 2  # CBI clock
        fall = t
        edges.append((bit,data_at,rise,fall))
        t += 1+1+(2 if i else 1)  # SUBI/SBC/BRCC
    t += 1  # first OUT SREG restores I (caller must originally have I=1)
    assert t == 1886 + 3*(16-word.bit_count())
    assert [b for b,_,_,_ in edges] == [(word>>i)&1 for i in range(15,-1,-1)]
    return dict(word=word, mask_cycles=t, high_cycles=[f-r for _,_,r,f in edges],
                setup_cycles=[r-d for _,d,r,_ in edges],
                low_cycles=[edges[i+1][2]-edges[i][3] for i in range(15)],
                data_hold_cycles=[edges[i+1][1]-edges[i][2] for i in range(15)],
                first_rise=edges[0][2], last_fall=edges[-1][3])

def envelope(rows,key):
    vals=[v for r in rows for v in (r[key] if isinstance(r[key],list) else [r[key]])]
    return [min(vals),max(vals)]

def uart(period,blackout,initial,entry=40):
    # Task0033 proposed entry-to-UDR budget, not exact AVR DOR.
    # Conservative completed-byte FIFO; no shift-register slack or other IRQs.
    loss=[max(0,initial+(0 if phase>blackout+entry else 1+(blackout+entry-phase)//period)-2)
          for phase in range(period)]
    return dict(frame_cycles=period,blackout_cycles=blackout,initial_fifo=initial,
                read_latency=entry,loss_phases=sum(x>0 for x in loss),
                phases=period,max_lost=max(loss))

def main():
    path=sys.argv[1]
    result=dict(archive_sha256=sha(open(path,"rb").read()),clock_hz=F_CPU,boards={})
    with zipfile.ZipFile(path) as z:
        summary=json.loads(z.read("summary.json"))
        result["tested_merge"]=summary["sha"]
        assert not summary["failures"]
        for board in BOARDS:
            s=z.read(board+"/disassembly.txt").decode()
            m=re.search(r"^([0-9a-f]+) <\(anonymous namespace\)::drv8711Transmit\(unsigned short\)>:\n(.*?)(?=^[0-9a-f]+ <)",s,re.M|re.S)
            assert m, board+" transmit symbol absent"
            instructions=re.findall(r"^\s*([0-9a-f]+):\s*((?:[0-9a-f]{2} )+[0-9a-f]{2})\s+\S+",m[2],re.M)
            raw=bytes.fromhex("".join(b for _,b in instructions))
            assert len(instructions)==38 and len(raw)==76 and sha(raw)==EXPECTED, (board,sha(raw))
            elf=z.read(board+"/build/Repetier.ino.elf")
            assert sha(elf)==summary["targets"][board]["elf_sha256"]
            result["boards"][board]=dict(address="0x"+m[1],opcode_sha256=sha(raw),elf_sha256=sha(elf))
    all_current=[trace(0x1100|level) for level in range(256)]
    allowed=[trace(0x1100|level) for level in range(32,151)]
    keys=("mask_cycles","high_cycles","setup_cycles","low_cycles","data_hold_cycles")
    result["current_all_levels"]={k:envelope(all_current,k) for k in keys}
    result["current_global_range_32_150"]={k:envelope(allowed,k) for k in keys}
    result["example_levels"]={str(v):trace(0x1100|v) for v in (0,32,90,105,110,126,150,255)}
    result["uart_screen"]=[uart(p,b,i) for p in (640,320)
                           for b in (0,5,32,80,400,1904,1928) for i in (0,1)]
    print(json.dumps(result,indent=2))

if __name__=="__main__":
    main()

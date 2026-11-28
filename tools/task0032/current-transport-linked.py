#!/usr/bin/env python3
"""Measure actual linked USART vectors and the bounded writer poll loop.

Reuse the Task0033 fail-closed structural walker, not isolated compilation.
Verify the candidate summary, ELF hashes and (when reading an artifact)
the caller-supplied ZIP SHA-256. All paths include IRQ entry/vector JMP
and RETI; they exclude preemption and pending-to-entry latency. The writer
bound counts active foreground cycles, not wall time or cumulative service
latency. An unexpected linked loop shape is an explicit audit blocker.
"""
from pathlib import Path
import argparse
import hashlib
import json
import re
import runpy
import zipfile

HERE=Path(__file__).resolve().parent
prior=runpy.run_path(str(HERE.parents[1]/"tools/task0033/isr-candidates.py"))
BOARDS=("RF1000","RF2000","RF2000v2","RF2000-X44")

def digest(data):
    return hashlib.sha256(data).hexdigest()

def linked_writer(dis):
    instructions=prior["insns"](dis)
    symbols=list(re.finditer(r"^([0-9a-f]+) <([^>]+)>:",dis,re.M))
    hits=[(i,m) for i,m in enumerate(symbols) if m[2]=="UsbSerial::write(unsigned char)"]
    assert len(hits)==1,"write inlined/renamed: inspect new linked layout"
    i,m=hits[0];start=int(m[1],16);end=int(symbols[i+1][1],16)
    pcs=[p for p in sorted(instructions) if start<=p<end]
    rows=[instructions[p] for p in pcs]
    ops=[x[1] for x in rows]
    loops=[j for j,(n,op,arg) in enumerate(rows) if op=="brne" and
           prior["target"](arg,pcs[j]+n)<pcs[j]]
    assert len(loops)==1
    j=loops[0];head=prior["target"](rows[j][2],pcs[j]+rows[j][0]);k=pcs.index(head)
    # Exact AVR GCC 7.3 linked stage1 pattern: one 16-bit poll counter,
    # no loop body calls, full-ring branch, SREG.I guard before decrement.
    assert ops[k:j+1]==["lds","cpse","rjmp","in","sbrs","rjmp","subi","sbc","brne"],ops[k:j+1]
    assert ops[:k]==["lds","cpse","rjmp","lds","subi","andi","lds","cpse","rjmp","ldi","ldi"],ops[:k]
    assert ops[j+1:j+4]==["ldi","sts","ret"],ops[j+1:j+4]
    assert "UsbSerial::blockingTxFailed_" in rows[0][2] and "UsbSerial::blockingTxFailed_" in rows[j+2][2]
    assert "UsbSerial::tx_" in rows[k][2] and "UsbSerial::tx_" in rows[6][2]
    assert rows[k+3][2].split(";")[0].strip()=="r0, 0x3f"
    assert rows[k+4][2].split(";")[0].strip()=="r0, 7"
    assert prior["target"](rows[k+5][2],pcs[k+5]+rows[k+5][0])==pcs[j+1]
    lo,hi=(re.match(r"(r\d+),\s*0x([0-9a-f]+)",rows[n][2],re.I) for n in (k-2,k-1))
    assert lo and hi
    assert rows[k+6][2].split(";")[0].strip()==lo[1]+", 0x01"
    assert rows[k+7][2].split(";")[0].strip()==hi[1]+", r1"
    assert rows[k+4][0]==2 and rows[k+5][0]==2
    assert rows[k+2][0]==2 and rows[k+1][0]==2
    budget=int(lo[2],16)+256*int(hi[2],16)
    assert budget>0
    # Taken full-ring path: 14 preloop; 11 cycles per repeated poll,
    # final BRNE not taken saves one; latch LDI/STS/RET=8. Masked
    # path skips no SREG branch, takes RJMP to the same latch.
    repeated=2+2+1+2+1+1+2
    pre=2+2+2+1+1+2+2+1+1
    exit_cost=1+2+5
    masked=pre+2+2+1+1+2+exit_cost
    active=pre+(budget-1)*repeated+(repeated-1)+exit_cost
    return {"address":hex(start),"loop":hex(head),"budget":budget,
            "repeated_poll_cycles":repeated,"no_progress_active_cycles":active,
            "masked_full_ring_entry_to_ret_cycles":masked,
            "scope":"function entry through RET; caller CALL and IRQs excluded; no wall-time proof",
            "loop_instructions":[[hex(p),*instructions[p]] for p in pcs[k:j+4]]}

def analyze(read):
    summary=json.loads(read("summary.json"))
    assert not summary["failures"] and summary["build_profile"]=="standard-recovery"
    assert set(summary["targets"])==set(BOARDS)
    result={"schema":1,"candidate_sha":summary["sha"],"clock_hz":16000000,
            "uart_baud":250000,"character_cycles":640,"boards":{}}
    for b in BOARDS:
        elf=read(b+"/build/Repetier.ino.elf")
        expected=summary["targets"][b]["elf_sha256"]
        assert digest(elf)==expected,(b,"ELF mismatch")
        raw=read(b+"/disassembly.txt");dis=raw.decode()
        rx=prior["path_set"](dis,25);tx=prior["path_set"](dis,26)
        assert all(len(p["status_reads"])==len(p["read_complete"])==1 and
                   p["status_reads"][0]<p["read_complete"][0] for p in rx)
        assert all(len(p["write_complete"])<=1 for p in tx)
        result["boards"][b]={"elf_sha256":expected,"disassembly_sha256":digest(raw),
            "rx":prior["summarize"](rx),"udre":prior["summarize"](tx),
            "rx_max_cycles":max(p["cycles"] for p in rx),
            "udre_max_cycles":max(p["cycles"] for p in tx),
            "writer":linked_writer(dis)}
    return result

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    source=parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--evidence",type=Path)
    source.add_argument("--archive",type=Path)
    parser.add_argument("--archive-sha256")
    parser.add_argument("--out",type=Path,required=True)
    args=parser.parse_args()
    if args.archive:
        sha=digest(args.archive.read_bytes())
        assert sha==args.archive_sha256,"ZIP identity not verified"
        with zipfile.ZipFile(args.archive) as z:
            assert z.testzip() is None
            result=analyze(lambda p:z.read("candidate-evidence/"+p))
        result["archive_sha256"]=sha
    else:
        result=analyze(lambda p:(args.evidence/p).read_bytes())
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(result,indent=2)+"\n")
    for b,data in result["boards"].items():
        print(b,"RX",data["rx_max_cycles"],"UDRE",data["udre_max_cycles"],
              "write active cycles",data["writer"]["no_progress_active_cycles"],
              "masked full-ring cycles",data["writer"]["masked_full_ring_entry_to_ret_cycles"])

if __name__=="__main__":
    main()

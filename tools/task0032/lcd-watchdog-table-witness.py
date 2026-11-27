#!/usr/bin/env python3
"""Reproduce the exact X44 watchdog false-positive witness from run 46078552647.

This diagnostic neither changes the P0 verifier nor authorizes a recovery image.
The failed build deliberately does not publish an ELF; use the archived linked
listing and symbols, verified by the immutable artifact digest.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import zipfile

ARCHIVE="c160da5d4a093bba6a92a7949eae2943551de1657c9eb25d525a0cfde3d4fd8a"
PREFIX="candidate-evidence/layout-evidence/combined/RF2000-X44-verification-failed/"
PROVIDER="EepromZones::CheckedProvider<EepromZones::LiveSettings>::operator()"
TABLE_START=0x12a9a
TABLE_END=0x12ac4
DISPATCH=0x12a84
HELPER=0x3b8c8

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("archive",type=Path)
    a=p.parse_args()
    with a.archive.open("rb") as f:
        assert hashlib.file_digest(f,"sha256").hexdigest()==ARCHIVE
    with zipfile.ZipFile(a.archive) as z:
        summary=json.loads(z.read("candidate-evidence/summary.json"))
        assert summary["sha"]=="57ec6d214ab906e9afd9b366772708ac123df257"
        assert len(summary["failures"])==1 and "RF2000-X44" in summary["failures"][0]
        listing=z.read(PREFIX+"disassembly.txt").decode()
        symbols=z.read(PREFIX+"symbols.txt").decode()
    match=next(re.fullmatch(r"([0-9a-f]+) ([0-9a-f]+) [Tt] (.+)",l)
               for l in symbols.splitlines() if PROVIDER in l)
    lo,size=int(match[1],16),int(match[2],16)
    hi=lo+size
    assert (lo,hi)==(0x129f8,0x13148)
    code={};ins={}
    for l in listing.splitlines():
        m=re.match(r"^\s*([0-9a-f]+):\s+((?:[0-9a-f]{2}\s+)+)([a-z]+)\s*(.*)$",l)
        if m:
            addr=int(m[1],16);raw=bytes.fromhex(m[2])
            ins[addr]=(len(raw),m[3],m[4])
            code.update({addr+i:v for i,v in enumerate(raw)})
    def span(start,end):
        return bytes(code[i] for i in range(start,end))
    # LDI r30,255; ADD r30,r7; CPI r30,21; BRCC default;
    # LDI r31,0; EOR r24,r24; SUBI/SBCI/SBCI load index+0x954d;
    # JMP __tablejump2__. No fall-through into the embedded data.
    assert span(DISPATCH,TABLE_START).hex()=="efefe70de53130f7f0e08827e35bfa468f4f0d9464dc"
    assert ins[TABLE_START-4][1]=="jmp" and "0x3b8c8" in ins[TABLE_START-4][2]
    # Helper doubles word address, sets RAMPZ, reads two flash bytes and EIJMP.
    assert span(HELPER,HELPER+16).hex()=="ee0fff1f881f8bbf0790f691e02d1994"
    raw=span(TABLE_START,TABLE_END)
    assert len(raw)==21*2
    targets=[2*int.from_bytes(raw[i:i+2],"little") for i in range(0,len(raw),2)]
    assert all(lo<=t<hi and not TABLE_START<=t<TABLE_END and t in ins for t in targets)
    assert raw[2:4]==bytes.fromhex("a895") and targets[1]==0x12b50
    inbound=[]
    for addr,(size,op,operand) in ins.items():
        if TABLE_START<=addr<TABLE_END:continue
        if op in ("call","rcall","jmp","rjmp") or op.startswith("br"):
            m=re.search(r";\s*0x([0-9a-f]+)",operand) or re.match(r"\s*0x([0-9a-f]+)",operand)
            if m and TABLE_START<=int(m[1],16)<TABLE_END:
                inbound.append((addr,op,operand))
    assert not inbound,inbound
    print(json.dumps({"archive_sha256":ARCHIVE,"candidate":summary["sha"],
                      "table":[hex(TABLE_START),hex(TABLE_END)],
                      "table_words":raw.hex(),"targets":[hex(t) for t in targets],
                      "false_positive_address":"0x12a9c","actual_target":"0x12b50",
                      "direct_branches_into_table":inbound,
                      "qualification":"exact listing switch-table witness only; no global indirect control-flow or heater safety certification"},indent=2))

if __name__=="__main__":
    main()

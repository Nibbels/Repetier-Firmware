#!/usr/bin/env python3
"""Compare exact pinned baseline and integrated counterfactual linked resources."""
import argparse
import hashlib
import json
from pathlib import Path
import re

P0="9072d65121bc2086522ea6b837ca3804ec80faa5"
BOARDS=("RF1000","RF2000","RF2000v2","RF2000-X44")

def sha(path):
    with path.open("rb") as f:
        return hashlib.file_digest(f,"sha256").hexdigest()

def verified(directory,expected):
    report=json.loads((directory/"summary.json").read_text())
    assert report["sha"]==expected and report["p0_candidate"] and not report["failures"], report
    assert tuple(report["targets"])==BOARDS
    rows={}
    for board in BOARDS:
        d=directory/board
        stats=report["targets"][board]
        assert json.loads((d/"verification.json").read_text())=={"status":"verified","recovery":True}
        assert sha(d/"build/Repetier.ino.elf")==stats["elf_sha256"]
        assert sha(d/"build/firmware.map")==stats["map_sha256"]
        padding=stats["recovery_verification"]["padding_bytes"]
        uid=next(x["bytes"] for x in stats["largest_sram_symbols"] if x["name"]=="uid")
        assert stats["flash_bytes"]<=253952
        symbols={}
        ram_symbols={}
        for line in (d/"symbols.txt").read_text().splitlines():
            ram=re.fullmatch(r"([0-9a-f]+) ([0-9a-f]+) ([BbDd]) (.+)",line)
            if ram:
                name=re.sub(r"\.lto_priv\.\d+","",ram[4])
                ram_symbols[name]=ram_symbols.get(name,0)+int(ram[2],16)
            m=re.fullmatch(r"([0-9a-f]+) ([0-9a-f]+) ([TtWw]) (.+)",line)
            if m and (m[4].startswith(("UIDisplay::refreshPage(","UIDisplay::renderNormalPage(",
                                      "UIDisplay::serviceDisplay(","UIDisplay::serviceDisplayPending(",
                                      "UIDisplay::localKeyReady(","lcdWriteByteNow(","lcdWriteByte("))):
                symbols[m[4]]=int(m[2],16)
        rows[board]={"flash":stats["flash_bytes"],"padding":padding,
                     "occupied_flash":stats["flash_bytes"]-padding,
                     "global_sram":stats["global_sram_bytes"],"uid":uid,
                     "sram_before_stack":stats["sram_remaining_before_stack_bytes"],
                     "sections":{k:stats["sections"].get(k,0) for k in (".data",".bss",".noinit")},
                     "symbols":symbols,"ram_symbols":ram_symbols,
                     "elf_sha256":stats["elf_sha256"]}
    return rows

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("out",type=Path)
    a=p.parse_args()
    base=verified(a.out/"baseline-evidence",P0)
    candidate_sha=(a.out/"local-candidate-sha.txt").read_text().strip()
    candidate=verified(a.out/"candidate-evidence",candidate_sha)
    result={"baseline":P0,"candidate":candidate_sha,"boards":{}}
    violations=[]
    for b in BOARDS:
        before,after=base[b],candidate[b]
        delta={k:after[k]-before[k] for k in ("flash","padding","occupied_flash","global_sram","uid","sram_before_stack")}
        # No allocation of new SRAM is authorized during P0.
        if delta["global_sram"]>0:
            violations.append({"board":b,"global_sram_delta":delta["global_sram"]})
        section_delta={k:after["sections"][k]-before["sections"][k] for k in before["sections"]}
        ram_delta={k:after["ram_symbols"].get(k,0)-before["ram_symbols"].get(k,0)
                   for k in sorted(before["ram_symbols"].keys()|after["ram_symbols"].keys())}
        result["boards"][b]={"before":{k:v for k,v in before.items() if k!="ram_symbols"},
                              "after":{k:v for k,v in after.items() if k!="ram_symbols"},
                              "delta":delta,"section_delta":section_delta,
                              "named_ram_delta":{k:v for k,v in ram_delta.items() if v}}
    result["violations"]=violations
    result["qualification"]="linked resource comparison only; not a whole-program stack or service latency proof"
    text=json.dumps(result,indent=2)+"\n"
    (a.out/"resource-delta.json").write_text(text)
    print(text)
    assert not violations,violations
if __name__=="__main__":
    main()

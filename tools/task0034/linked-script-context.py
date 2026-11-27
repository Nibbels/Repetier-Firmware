#!/usr/bin/env python3
"""A73: inventory actual flash-script command contexts in pinned AVR images.

The checked grammar is deliberately narrower than the general G-code parser:
one literal G/M command and numeric arguments per line. A new/custom script
outside this inventory requires renewed context analysis, not a firmware ban.
This proves neither foreground stack depth nor handler admission.
"""
import argparse
import hashlib
import json
import re
import struct
import zipfile

BOARDS=("RF1000","RF2000","RF2000v2","RF2000-X44")
EXPECTED={
    "tOutputObjectPrint": ["G21","G92","G1","G4","G92","G90","G1","G1"],
    "tMountFilamentSoft": ["M3913"],
    "tUnmountFilamentSoft": ["M3914"],
    "tMountFilamentHard": ["M109","G21","G92","G1","G92","M104"],
    "tUnmountFilamentHard": ["M109","G21","G92","G1","G92","M104"],
}
LINE=re.compile(r"([GM]\d+)(?: +[XYZEFSP][+-]?(?:\d+(?:\.\d*)?|\.\d+))*")

def flash_bytes(elf,address,size):
    assert elf[:6]==b"\x7fELF\x01\x01","not little-endian ELF32"
    h=struct.unpack_from("<16sHHIIIIIHHHHHH",elf)
    assert h[2]==83 and h[9]==32,"not AVR ELF32 program headers"
    segments=[struct.unpack_from("<8I",elf,h[5]+i*h[9]) for i in range(h[10])]
    candidates=[p for p in segments if p[0]==1 and p[2]<=address and address+size<=p[2]+p[4]]
    assert len(candidates)==1,(hex(address),size)
    p=candidates[0]
    offset=p[1]+address-p[2]
    assert offset+size<=len(elf)
    return elf[offset:offset+size]

def inventory(z,board):
    p=board+"/"
    assert json.loads(z.read(p+"verification.json"))["status"]=="verified"
    symbols=z.read(p+"symbols.txt").decode()
    elf=z.read(p+"build/Repetier.ino.elf")
    scripts={}
    errors=[]
    for name,expected in EXPECTED.items():
        rows=re.findall(r"^([0-9a-f]+) ([0-9a-f]+) [tTrR] Com::"+name+r"$",symbols,re.M)
        assert len(rows)==1,(board,name,rows)
        a,n=(int(x,16) for x in rows[0])
        assert a+n<=0x10000,(board,name,"near flash address")
        raw=flash_bytes(elf,a,n)
        assert raw.endswith(b"\0") and b"\0" not in raw[:-1],(board,name)
        lines=raw[:-1].decode("ascii").split("\n")
        commands=[]
        for line in lines:
            m=LINE.fullmatch(line)
            if not m or len(line)>79:
                errors.append([name,"not strict numeric script line",line])
            else:
                commands.append(m[1])
        if commands!=expected:
            errors.append([name,"command sequence mismatch",commands,expected])
        scripts[name]={"address":hex(a),"bytes":n,"lines":lines,"commands":commands}
    config=z.read(p+"build/sketch/Configuration.h").decode()
    for name,profile in (("tMountFilamentHard","B"),("tUnmountFilamentHard","A")):
        m=re.search(r"^#define\s+PREHEAT_"+profile+r"_HOTEND_C\s+(\d+)\b",config,re.M)
        assert m,(board,profile,"default temperature definition")
        if scripts[name]["lines"][0]!="M109 S"+m[1]:
            errors.append([name,"hard filament target differs from numeric default",m[1]])
    callers=[]
    for path in z.namelist():
        if not path.startswith(p+"build/sketch/") or not path.endswith((".cpp",".h")):
            continue
        source=z.read(path).decode()
        for m in re.finditer(r"GCodeInput::executeFlashScript\b",source):
            rest=source[m.end():]
            if re.match(r"\(\s*const char\*\s*cmd\s*\)\s*\{",rest):
                continue
            c=re.match(r"\(\s*Com::(t\w+)\s*\)",rest)
            if not c or c[1] not in EXPECTED:
                errors.append([path,"unreviewed script callsite",rest[:80]])
                continue
            callers.append([path.rsplit("/",1)[1],c[1]])
    wanted=[["RF.cpp","tOutputObjectPrint"],["UiActions.cpp","tUnmountFilamentSoft"],
            ["UiActions.cpp","tUnmountFilamentHard"],["UiActions.cpp","tMountFilamentSoft"],
            ["UiActions.cpp","tMountFilamentSoft"],["UiActions.cpp","tMountFilamentHard"]]
    if sorted(callers)!=sorted(wanted):
        errors.append(["unreviewed source callsite inventory",callers])
    return {"scripts":scripts,"sourceCallsites":callers,"errors":errors}

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive")
    parser.add_argument("sha256")
    args=parser.parse_args()
    with open(args.archive,"rb") as f:
        assert hashlib.sha256(f.read()).hexdigest()==args.sha256
    with zipfile.ZipFile(args.archive) as z:
        summary=json.loads(z.read("summary.json"))
        boards={b:inventory(z,b) for b in BOARDS}
    print(json.dumps({"testedMerge":summary["sha"],"sha256":args.sha256,
                      "ciFailures":summary["failures"],"boards":boards},indent=2))
    if any(b["errors"] for b in boards.values()):
        raise SystemExit(1)

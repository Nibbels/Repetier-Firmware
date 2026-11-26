#!/usr/bin/env python3
"""Count the straight-line linked expiry ISR of verified A12 artifact.
ATmega2560, 3-byte PC: entry 5 cycles, JMP 3, CALL/RET/RETI 5.
No dispatch latency, no arbitrary instruction timing or power-stage proof.
"""
import hashlib
import re
import sys
import zipfile

with zipfile.ZipFile(sys.argv[1]) as z:
    raw=z.read("host/heater-lease-integrated-avr.txt")
assert hashlib.sha256(raw).hexdigest()=="75084f66a83522dead43fdc0d9d58105e3281ec52e646c161de1e13aee4f1065"
s=raw.decode()
cycles={"push":2,"pop":2,"in":1,"out":1,"eor":1,"call":5,"ret":5,"reti":5,
        "ldi":1,"sts":2,"cbi":2,"ld":2,"andi":1,"st":2,"std":2}
for board in ("RF1000","RF2000","RF2000v2","RF2000-X44"):
    b=s.split("BEGIN "+board+" DISASSEMBLY")[1].split("END "+board+" DISASSEMBLY")[0]
    def function(pattern):
        m=re.search(pattern,b,re.M)
        nxt=re.search(r"^[0-9a-f]+ <[^\n]+>:",b[m.end():],re.M)
        body=b[m.end():m.end()+nxt.start()]
        return [(m[1],m[2]) for m in re.finditer(
            r"^\s*[0-9a-f]+:\s+(?:[0-9a-f]{2}\s+)+([a-z]+)\s*(.*)$",body,re.M)]
    vec=function(r"^[0-9a-f]+ <__vector_32>:")
    off=function(r"^[0-9a-f]+ <[^\n>]*failHeaterLease[^\n>]*>:")
    calls=[i for i,(op,arg) in enumerate(vec) if op=="call"]
    assert len(calls)==1 and "failHeaterLease" in vec[calls[0]][1]
    assert vec[-1][0]=="reti" and off[-1][0]=="ret"
    pre=sum(cycles[op] for op,arg in vec[:calls[0]+1])
    # Pinned disassembly reviewed: CBI PORTB4 and PORTE2, or LD/ANDI/ST
    # at Z=0x102 (PORTH). A later ST Z,r1 zeroes requestedDuty, not a pin.
    # Identify pin writes explicitly before the demand-clear block.
    pin_indices=[i for i,(op,arg) in enumerate(off)
                 if op=="cbi" or (op=="st" and re.match(r"Z,\s*r24\b",arg))]
    assert len(pin_indices)==(3 if board in ("RF2000","RF2000v2") else 2)
    all_low=8+pre+sum(cycles[op] for op,arg in off[:pin_indices[-1]+1])
    total=8+sum(cycles[op] for op,arg in vec+off)
    print(board, "all GPIO low",all_low,"cycles",all_low/16,"us;",
          "ISR total",total,"cycles",total/16,"us")

#!/usr/bin/env python3
"""Paired actual linked foreground preparation: 103 versus 99 B PrintLine.
No estimate of concurrent WCET. Compare every prepared byte, not floats at a
tolerance, and retain separate ready/not-ready path timing and stack usage.
"""
import argparse
import itertools
import json
import struct
import tempfile
from pathlib import Path
import importlib.util

def load():
    p=Path(__file__).with_name("current-direct-energy-linked.py")
    spec=importlib.util.spec_from_file_location("energy",p)
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    return mod

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--baseline",type=Path,required=True)
    p.add_argument("--compact",type=Path,required=True)
    p.add_argument("--out",type=Path,required=True)
    a=p.parse_args()
    el=load()
    el.FAST_PROFILE=True
    mod=el.sl.lt.load_interpreter()
    result={"scope":__doc__,"boards":{}}
    with tempfile.TemporaryDirectory() as d:
        archives=[Path(d)/"old.zip",Path(d)/"new.zip"]
        metadata=[]
        for compact,path,archive in zip((False,True),(a.baseline,a.compact),archives):
            el.COMPACT=compact
            sha,meta=el.sl.evidence(path,archive)
            metadata.append({"sha":sha,"boards":meta})
        result["sources"]=metadata
        for board in el.sl.lt.BOARDS:
            images=[mod.Image(x,board) for x in archives]
            def trial(img,compact,case,ready):
                full,start,end,rate,accel,steps=case
                size=99 if compact else 103
                assert img.sizes["PrintLine::direct"]==size
                c=mod.CPU(img)
                addr=img.addr("PrintLine::direct")
                c.mem[addr:addr+size]=bytes(size)
                c.put(addr,1 if ready else 0)
                c.put(addr+2,0)
                c.put(addr+7,17)
                c.put(addr+8,steps,4)
                def fl(off,x):
                    c.mem[addr+off:addr+off+4]=struct.pack("<f",x)
                fl(24,full)
                if not compact:
                    # Same binary32 reciprocal rounding as legacy planner.
                    fl(44,1.0/struct.unpack("<f",struct.pack("<f",full))[0])
                shift=-4 if compact else 0
                fl(56+shift,start)
                fl(60+shift,end)
                c.put(addr+72+shift,accel,4)
                c.put(addr+88+shift,rate,2)
                c.put(addr+98+shift,steps,4)
                c.setpair(24,addr)
                c.sreg=128
                c.run(img.find("MotionPlanner::prepareStepProfile(PrintLine&)"))
                assert c.sp==0x21ff and c.flag(7)
                assert not c.edges
                assert c.get(addr)&1
                return dict(profile=c.mem[addr+76+shift:addr+98+shift].hex(),
                    cycles=c.cy,stack=0x21ff-c.minsp,
                    mask=max((b-a for a,b in el.sl.lt.windows(c.timeline,c.cy)),default=0))
            rows=[]
            for full,start,end in ((100.,3.2,0.5),(9.,.1,.2),(34.5,12.25,4.7),(1.,1.,1.),(.125,.125,.0625)):
                for rate,accel,steps,ready in itertools.product((800,11520,15238),(128000,228570),(1,12,1200),(False,True)):
                    case=(full,start,end,rate,accel,steps)
                    old,new=[trial(img,i==1,case,ready) for i,img in enumerate(images)]
                    assert old["profile"]==new["profile"],(board,case,ready,old,new)
                    assert old["mask"]==new["mask"]==0,(board,case,ready,old,new)
                    rows.append(dict(case=case,ready=ready,old=old,new=new,delta=new["cycles"]-old["cycles"]))
            result["boards"][board]=rows
            print(board,len(rows),"bit-identical profiles; unprepared cycle delta range",
                min(x["delta"] for x in rows if not x["ready"]),max(x["delta"] for x in rows if not x["ready"]),flush=True)
            a.out.write_text(json.dumps(result,indent=2)+"\n")
if __name__=="__main__":
    main()

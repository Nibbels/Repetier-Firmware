#!/usr/bin/env python3
"""Inject the real PWM stop-tail writer effect at each legal Timer1 boundary.
Synthetic direct Z batch with cancellation. Not nested PWM runtime/WCET.
"""
import argparse,importlib.util,json,pathlib
HERE=pathlib.Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("unified",HERE/"unified-z-linked-replay.py")
u=importlib.util.module_from_spec(spec);spec.loader.exec_module(u)
def board(mod,img):
    Base=mod.CPU
    addr=img.addr("PrintLine::direct")+98
    class Probe(Base):
        def __init__(self,img):
            super().__init__(img);self.inject_at=None;self.injected=False
        def boundary(self,pc):
            super().boundary(pc)
            if self.cy==self.inject_at:
                assert self.flag(7)
                old=self.get(addr,4,signed=True)
                assert self.get(addr+4)==7 and old>32
                self.put(addr,32,4);self.injected=True
    mod.CPU=Probe
    def trial(at=None):
        c,move,errors,o=u.configure(mod,img,"z",11520,owner=0)
        c.put(move+102,7)
        c.symput("Printer::zMotion",100000,4,24)
        u.target(c,-100)
        c.inject_at=at
        row=u.invoke(c,0)
        assert not u.z_pulses(row)
        assert u.zfield(c,20)==3 and u.zfield(c,16)==-3
        assert c.get(addr+4)==7
        r=c.get(addr,4,signed=True)
        if at is None:assert r==99997
        else:assert c.injected and 29<=r<=32,(at,r)
        return c,row,r
    try:
        c,row,r=trial()
        possible=u.lt.irq_opportunities(c,True)
        for t in possible:trial(t)
        return dict(tested=len(possible),violations=0,mask=row["max_mask"],cycles=row["cycles"])
    finally:mod.CPU=Base
def main():
    p=argparse.ArgumentParser()
    p.add_argument("archive",type=pathlib.Path);p.add_argument("out",type=pathlib.Path);a=p.parse_args()
    mod=u.lt.load_interpreter();results={}
    for b in u.lt.BOARDS:
        results[b]=board(mod,mod.Image(a.archive,b))
        print("Z packed stop interleavings",b,json.dumps(results[b]),flush=True)
    a.out.write_text(json.dumps(results,indent=2)+"\n")
if __name__=="__main__":main()

#!/usr/bin/env python3
"""Inject only the PWM stopDirectMove RUNNING_STOPPABLE writer effect.

Execute the actual linked Timer1 instruction stream. At a legal interrupt
admission boundary between successive bytes of a remaining-step read, set
remaining to 32 if greater than 32. No claim about PWM execution time or
real interrupt phase probability. Counterexamples require an actual
eligible boundary, not a byte-store interleaving within a CPU instruction.
"""
import argparse,json,pathlib
import linked_timer1 as lt

def board_cases(mod,img):
    Base=mod.CPU
    addr=img.addr("PrintLine::direct")+98
    class Probe(Base):
        def __init__(self,img):
            super().__init__(img)
            self.target=None;self.injected=None;self.reads=[]
        def boundary(self,pc):
            super().boundary(pc)
            if self.cy==self.target:
                assert self.flag(7)
                old=self.get(addr,4,signed=True)
                assert self.get(addr+4)==7 and old>32
                self.put(addr,32,4)
                self.injected={"cycle":self.cy,"pc":hex(pc),"old":old}
        def read(self,a):
            v=super().read(a)
            if addr<=a<addr+4:self.reads.append((self.cy,a-addr,v))
            return v
    mod.CPU=Probe
    def run(remaining,n,target=None):
        c,pc,move,errors,o=mod.setup(img,owner=0,delta=(remaining+100,0,0,0),
            rate=9143,n=n,phase="cruise",advance=False,ratio=0,zcor=0,remaining=remaining)
        c.put(move+102,7)
        if "motorWake" in img.symbols:
            c.symput("motorWake",1,1,4);c.symput("motorWake",1,1,5)
        c.target=target;c.mem[0x6f]=2;c.cy=5;c.timer_live=True
        c.run(17*4)
        return c,{"remaining":c.get(addr,4,signed=True),"task":c.get(addr+4),
            "x_rises":sum(p=="X" and s for p,s,t in c.edges),
            "injected":c.injected}
    rows=[]
    try:
        for n in (2,5):
            for remaining in (256,256+n,65536,65536+n):
                baseline,result=run(remaining,n)
                assert result["x_rises"]==n and result["remaining"]==remaining-n and result["task"]==7,result
                possible=set(lt.irq_opportunities(baseline,True));targets=set()
                for a,b in zip(baseline.reads,baseline.reads[1:]):
                    if a[1]+1==b[1]:
                        targets.update(t for t in possible if a[0]<t<=b[0])
                # Also exercise a few coherent entry/exit boundaries even
                # when no vulnerable multi-byte window remains.
                targets.update(sorted(possible)[:1]);targets.update(sorted(possible)[-1:])
                for target in sorted(targets):
                    c,r=run(remaining,n,target)
                    assert r["injected"] is not None
                    bad=r["task"]!=7 or r["x_rises"]!=n or not 32-n<=r["remaining"]<=32
                    rows.append(dict(initial=remaining,n=n,violation=bad,**r))
    finally:mod.CPU=Base
    return rows

def main():
    p=argparse.ArgumentParser()
    p.add_argument("archive",type=pathlib.Path);p.add_argument("output",type=pathlib.Path)
    p.add_argument("--require-safe",action="store_true");args=p.parse_args()
    mod=lt.load_interpreter();results={}
    for board in lt.BOARDS:
        rows=board_cases(mod,mod.Image(args.archive,board))
        violations=[r for r in rows if r["violation"]]
        results[board]={"tested":len(rows),"violations":len(violations),"examples":violations[:8],"rows":rows}
        print(board,len(rows),"interleavings",len(violations),"violations",json.dumps(violations[:2]),flush=True)
    args.output.write_text(json.dumps(results,indent=2)+"\n")
    return int(args.require_safe and any(r["violations"] for r in results.values()))

if __name__=="__main__":raise SystemExit(main())

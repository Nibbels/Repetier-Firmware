#!/usr/bin/env python3
"""Chained linked Timer1 queue execution for the physical Z admission candidate.

Actual instruction and GPIO traces; isolated vector entries, no competing
interrupts or missed matches. Verify that every programmed TOP is reached
only after the modeled vector returns. Late/coalesced CTC events are tested
by the independent compare-phase clock replay, not hidden in this model.
"""
import argparse
import importlib.util
import itertools
import json
from pathlib import Path
import tempfile


def expected(delta, total):
    p = max(delta)
    return [max(0, (total*d - (p//2 if axis < 3 else 0) + p-1)//p)
            for axis, d in enumerate(delta)]


def chain(mod, el, img, name, delta, rate, pack, negative, ratio, correction, advance):
    remaining = max(delta)
    c, pc, move, errors, o = mod.setup(img, owner=1, delta=delta,
        rate=rate, n=pack, phase="cruise", advance=advance,
        ratio=ratio, zcor=correction, negative=negative, remaining=remaining)
    assert o == -4 and img.sizes["PrintLine::queue"] == 2381
    assert [s for n,s in img.sizes.items() if "ZMotion::physicalClock_" in n] == [5]
    c.symput("motorWake",31,1,4)
    c.symput("motorWake",31,1,5)
    c.put(0x88,3000,2)
    c.mem[0x36]=0
    img.step_pins[(0x10b,1)]="DIR_Z"
    sign = -1 if negative else 1
    epoch=0
    done=0
    calls=deferred=truncated=0
    max_cycles=max_mask=0
    last_dir=last_z=last_fall=None
    z_direction=1 if c.mem[0x10b]&2 else -1
    z_net=0
    counts={pin:0 for pin in ("X","Y","Z","E0")}
    z_gaps=[]
    dir_setup=[]
    dir_hold=[]
    high=[]
    low=[]
    while done < remaining:
        assert calls < remaining*4+32, (img.board,name,done,calls)
        before=el.sl.lt.snapshot(c,move,errors,o)
        old_remaining=before["remaining"]
        requested=c.symget("StepExecutor::stepsPerTimerCall")
        top=c.get(0x88,2)
        c.cy=5
        c.sreg=0
        c.mem[0x6f]=2
        c.timer_live=True
        c.timeline=[];c.edges=[];c.calls=[];c.timer_writes=[]
        c.timer0_phase=0;c.timer0_reads=0;c.timer0_flag_cleared_at_wrap=0
        c.run(17*4)
        assert c.sp==0x21ff and not c.return_stack
        calls+=1
        after=el.sl.lt.snapshot(c,move,errors,o)
        completed=old_remaining-after["remaining"]
        assert 0<=completed<=min(requested,old_remaining), (img.board,name,calls,completed,requested)
        done+=completed
        if not completed:
            deferred+=1
            assert not c.edges, (img.board,name,"zero-slot GPIO")
            assert before==after, (img.board,name,"zero-slot state mutation",before,after)
        elif completed<min(requested,old_remaining):
            truncated+=1
        wanted=expected(delta,done)
        for axis,n in enumerate(wanted):
            initial=max(delta)//2 if axis<3 else 0
            assert after["errors"][axis]==initial-done*delta[axis]+n*max(delta)
        for pin in counts:
            edges=[(s,t) for pn,s,t in c.edges if pn==pin]
            assert [s for s,t in edges]==[1,0]*(len(edges)//2)
            counts[pin]+=len(edges)//2
        for pin,state,t in c.edges:
            t+=epoch
            if pin=="DIR_Z":
                if last_z is not None:
                    dir_hold.append(t-last_z)
                    assert t-last_z>=4
                last_dir=t
                z_direction=1 if state else -1
            elif pin=="Z" and state:
                if last_z is not None:
                    z_gaps.append(t-last_z)
                    assert t-last_z>=512, (img.board,name,"physical Z floor",t-last_z)
                if last_fall is not None:
                    low.append(t-last_fall)
                    assert t-last_fall>=31
                if last_dir is not None:
                    dir_setup.append(t-last_dir)
                    assert t-last_dir>=4
                last_z=t
                z_net+=z_direction
            elif pin=="Z":
                assert last_z is not None
                high.append(t-last_z)
                assert t-last_z>=31
                last_fall=t
        for axis,pin in enumerate(("X","Y")):
            assert counts[pin]==wanted[axis]
            assert c.symget("Printer::xyMotion",4,axis*4,signed=True)==1000+sign*wanted[axis]
            assert c.symget("Printer::xyMotion",4,8+axis*4,signed=True)==1000+sign*wanted[axis]
        z_motor=c.symget("Printer::zMotion",4,0,signed=True)-1000
        z_nominal=c.symget("Printer::zMotion",4,4,signed=True)-1000
        z_applied=c.symget("Printer::zMotion",4,16,signed=True)
        assert z_motor==z_net==z_nominal+z_applied
        assert z_nominal==sign*wanted[2]
        extra=(wanted[3]//4 if not negative else (wanted[3]+3)//4) if ratio else 0
        if advance:
            assert not counts["E0"]
            pending=c.symget("Printer::extrusionOutput",2,0,signed=True)
            lead=c.symget("Printer::extrusionOutput",2,2,signed=True)
            assert pending-lead==sign*(wanted[3]+extra)
        else:
            assert counts["E0"]==wanted[3]+extra
        # The archived instruction interpreter has no actual compare flag.
        # Explicitly reject a trace if a CTC match could have occurred in
        # this invocation, including before the initial 61000 sentinel.
        high_byte=top>>8
        for t,addr,value in c.timer_writes:
            if addr==0x89:
                assert t<top+1, (img.board,name,"compare before TOP high write",top,t)
                high_byte=value
            elif addr==0x88:
                assert t<top+1, (img.board,name,"compare before TOP commit",top,t)
                top=(high_byte<<8)|value
                assert t<=top, (img.board,name,"TOP below current phase",top,t)
        assert top==c.get(0x88,2)
        assert c.cy<top+1, (img.board,name,"compare before vector return",c.cy,top)
        max_cycles=max(max_cycles,c.cy)
        max_mask=max(max_mask,max((b-a for a,b in el.sl.lt.windows(c.timeline,c.cy)),default=0))
        epoch+=top+1
    assert done==remaining
    return dict(name=name,delta=delta,rate=rate,initial_pack=pack,negative=negative,
        ratio=ratio,correction=correction,advance=advance,calls=calls,
        zero_slot_deferrals=deferred,truncated_packs=truncated,elapsed_ticks=epoch,
        max_cycles=max_cycles,max_raw_I_clear=max_mask,stack=0x21ff-c.minsp,
        counts=counts,z_net=z_net,z_min_gap=min(z_gaps,default=None),
        z_min_high=min(high,default=None),z_min_low=min(low,default=None),
        z_min_dir_setup=min(dir_setup,default=None),z_min_dir_hold=min(dir_hold,default=None))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--evidence",type=Path,required=True)
    p.add_argument("--out",type=Path,required=True)
    a=p.parse_args()
    spec=importlib.util.spec_from_file_location("energy",Path(__file__).with_name("current-direct-energy-linked.py"))
    el=importlib.util.module_from_spec(spec);spec.loader.exec_module(el)
    el.FAST_PROFILE=el.COMPACT=True
    mod=el.sl.lt.load_interpreter()
    result={"scope":__doc__,"boards":{}}
    with tempfile.TemporaryDirectory() as d:
        archive=Path(d)/"firmware.zip"
        result["candidate_sha"],meta=el.sl.evidence(a.evidence,archive)
        result["metadata"]=meta
        for board in el.sl.lt.BOARDS:
            img=mod.Image(archive,board)
            rows=[]
            for name,delta,rate,ratio,correction in (
                ("Z",(0,0,100,0),11520,0,0),
                ("Z-floor-stress",(0,0,100,0),31250,0,0),
                ("XYZ",(100,61,73,0),15238,0,0),
                ("XYZE",(100,61,73,23),15238,0.25,4),
                ("XYE-CMP",(100,61,0,23),15238,0.25,4)):
                for pack,negative,advance in itertools.product((1,2,5,10),(False,True),
                    (False,True) if delta[3] else (False,)):
                    rows.append(chain(mod,el,img,name,delta,rate,pack,negative,ratio,correction,advance))
            result["boards"][board]=rows
            print(board,len(rows),"chained profiles",sum(r["calls"] for r in rows),"vectors",
                  sum(r["truncated_packs"] for r in rows),"truncated packs",
                  sum(r["zero_slot_deferrals"] for r in rows),"zero-slot retries",
                  "minimum Z rise spacing",min(r["z_min_gap"] for r in rows if r["z_min_gap"] is not None),
                  flush=True)
            a.out.write_text(json.dumps(result,indent=2)+"\n")


if __name__=="__main__":
    main()

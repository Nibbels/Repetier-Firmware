#!/usr/bin/env python3
"""Actual linked wake admission across successive Timer1 calls.

Clock observations are injected in RAM at each invocation; not simulated
Timer0 overflow, UART nesting, physical winding current or a full scheduler.
Each invocation executes actual AVR instructions with real GPIO stores.
"""
import argparse,collections,json,pathlib
import linked_timer1 as lt

ENABLE_PINS={(0x2b,7):"ENA_X",(0x31,2):"ENA_Y",(0x108,0):"ENA_Z",
             (0x22,2):"ENA_E0",(0x28,7):"ENA_E1"}
MASK_BITS={"ENA_X":1,"ENA_Y":2,"ENA_Z":4,"ENA_E0":8,"ENA_E1":16}

def clock_symbol(img):
    names=[n for n in img.symbols if "timer0_millis" in n and img.sizes[n]==4]
    assert len(names)==1,names
    return names[0]

def configure(mod,img,owner,delta,tool,advance=False,zcor=0,ready=0,stoppable=False):
    kw=dict(owner=owner,delta=delta,rate=6400,n=2,phase="cruise",advance=advance,ratio=0,zcor=zcor,tool=tool,negative=False)
    c,pc,move,errors,o=mod.setup(img,**kw)
    assert img.sizes["motorWake"]==6
    c.symput("motorWake",ready,1,4);c.symput("motorWake",ready,1,5)
    c.symput("Printer::extrusionCompensation",0,8)
    c.symput("Printer::enabledStepper",0,img.sizes["Printer::enabledStepper"])
    c.symput("Printer::extrusionOutput",4,1,9) # MotorsDisabled
    c.mem[0x6f]=2
    for (addr,bit),name in ENABLE_PINS.items():
        c.mem[addr-1]|=1<<bit
        if ready&MASK_BITS[name]:c.mem[addr]|=1<<bit
        else:c.mem[addr]&=~(1<<bit)
    if owner:c.put(img.addr("PrintLine::queue")+24*103,0,2)
    else:c.put(move+102,5 if stoppable else 4)
    return c,move,kw

def tick(c,clock,ms):
    c.symput(clock,ms,4)
    c.cy=5;c.sreg=0;c.timeline=[];c.edges=[];c.calls=[];c.timer_writes=[]
    c.timer_live=True;c.timer0_reads=0;c.timer0_phase=0;c.timer0_flag_cleared_at_wrap=0
    c.minsp=c.sp;c.hit.clear()
    c.run(17*4)
    assert not any(n in ("micros","delay") or n.startswith("delay(") for n,_ in c.calls),c.calls
    mask=lt.windows(c.timeline,c.cy)
    return {"millis":ms,"cycles":c.cy,"max_mask":max((b-a for a,b in mask),default=0),
            "edges":c.edges[:],"awake":c.symget("motorWake",1,4),"ready":c.symget("motorWake",1,5),
            "stack_bytes":0x21ff-c.minsp}

def activated(c,move,owner):
    return c.get(c.img.addr("PrintLine::queue")+24*103,2)==move if owner else c.get(move+102) in (6,7)
def pulses(row):
    return [e for e in row["edges"] if e[1] and e[0] in ("X","Y","Z","E0","E1")]
def enabled(row):
    return sum(MASK_BITS[e[0]] for e in row["edges"] if e[1] and e[0] in MASK_BITS)
def requested(delta,tool):
    return sum(bit for d,bit in zip(delta,(1,2,4,8<<tool)) if d)
def no_steps(row):
    assert not pulses(row),row

def case(mod,img,owner,delta,tool,advance,initial=0,abort=False,wrap=False,stoppable=False):
    clock=clock_symbol(img)
    c,move,kw=configure(mod,img,owner,delta,tool,advance,ready=initial,stoppable=stoppable)
    need=requested(delta,tool);new=need&~initial
    assert new
    start=0xfffffff0 if wrap else 100
    rows=[]
    for offset in (0,1,25,26):
        row=tick(c,clock,(start+offset)&0xffffffff);rows.append(row)
        no_steps(row)
        assert not activated(c,move,owner)
        assert c.get(move+98,4)==80
        assert row["ready"]==initial and row["awake"]==(initial|new)
        assert enabled(row)==(new if offset==0 else 0)
        if delta[3]:assert c.symget("Printer::extrusionOutput",1,9)&4
        if abort and offset==0:c.symput("Printer::blockAll",1)
    row=tick(c,clock,(start+27)&0xffffffff);rows.append(row)
    no_steps(row)
    assert enabled(row)==0
    if abort:
        assert not activated(c,move,owner) and row["ready"]==initial
        assert c.get(move+98,4)==80
        row=tick(c,clock,(start+100)&0xffffffff);rows.append(row);no_steps(row)
        assert enabled(row)==0 and not activated(c,move,owner)
    else:
        assert activated(c,move,owner) and row["ready"]==(initial|new)
        assert c.get(move+98,4)==80
        if delta[3]:assert not (c.symget("Printer::extrusionOutput",1,9)&4)
        row=tick(c,clock,(start+28)&0xffffffff);rows.append(row)
        assert c.get(move+98,4)<80
        assert pulses(row) or (advance and delta[3] and not any(delta[:3]))
        assert enabled(row)==0
    return {"fixture":kw,"abort":abort,"wrap":wrap,"initial_ready":initial,"stoppable":stoppable,"ticks":rows}

def idle_correction(mod,img,homed):
    clock=clock_symbol(img)
    c,move,kw=configure(mod,img,1,(100,0,0,0),0,zcor=4)
    c.put(img.addr("PrintLine::queue")+24*103,0,5) # no queued move
    c.symput("Printer::zMotion",int(homed),1,41)
    rows=[]
    for off in (0,1,26,27):
        row=tick(c,clock,100+off);rows.append(row)
        if not homed or off<27:no_steps(row)
        assert enabled(row)==(4 if homed and off==0 else 0)
        if not homed:assert row["ready"]==0
    if homed:
        assert rows[-1]["ready"]==4 and any(e[0]=="Z" for e in pulses(rows[-1]))
    return {"idle_correction":True,"homed":homed,"ticks":rows}

def main():
    p=argparse.ArgumentParser()
    p.add_argument("archive",type=pathlib.Path);p.add_argument("out",type=pathlib.Path)
    args=p.parse_args()
    mod=lt.load_interpreter();results={}
    for board in lt.BOARDS:
        img=mod.Image(args.archive,board);img.step_pins.update(ENABLE_PINS)
        if board not in ("RF2000","RF2000v2"):img.step_pins.pop((0x28,7),None)
        rows=[]
        for owner in (0,1):
            for delta in ((100,0,0,0),(100,70,0,30),(0,0,100,0),(0,0,0,100),(100,70,50,30)):
                for tool in range(2 if delta[3] and board in ("RF2000","RF2000v2") else 1):
                    for advance in (False,True) if delta[3] else (False,):
                        rows.append(case(mod,img,owner,delta,tool,advance,wrap=bool(tool),stoppable=not owner))
            rows.append(case(mod,img,owner,(100,70,50,30),0,False,abort=True,stoppable=not owner))
            # New tool/channel joins an already ready X and E0; no resetting
            # or duplicate assertion of those physical channels.
            if board in ("RF2000","RF2000v2"):
                rows.append(case(mod,img,owner,(100,0,0,30),1,True,initial=9))
        rows.extend(idle_correction(mod,img,h) for h in (False,True))
        ticks=[t for r in rows for t in r["ticks"]]
        results[board]={"cases":len(rows),"ticks":len(ticks),
                        "max_mask":max(t["max_mask"] for t in ticks),
                        "max_cycles":max(t["cycles"] for t in ticks),
                        "max_cold_first_mask":max(r["ticks"][0]["max_mask"] for r in rows),
                        "max_cold_first_cycles":max(r["ticks"][0]["cycles"] for r in rows),
                        "rows":rows}
        print("wake",board,json.dumps({k:v for k,v in results[board].items() if k!="rows"}),flush=True)
    args.out.write_text(json.dumps(results,indent=2)+"\n")

if __name__=="__main__":main()

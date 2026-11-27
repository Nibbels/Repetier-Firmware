#!/usr/bin/env python3
"""Task0032-S isolated linked Timer1 paths; NOT a concurrent peripheral emulator."""
import argparse, collections, hashlib, itertools, json, math, pathlib, sys, types, zipfile

ROOT = pathlib.Path(__file__).resolve().parents[2]
CLOCK = 16_000_000
BOARDS = ("RF1000", "RF2000", "RF2000v2", "RF2000-X44")
ARCHIVE_BLOB = "6bcb6bd965b77ebc284bc88f16808817164dcf17"
CONTRACT_BLOB = "003d9cf4a256ef99b46441c3b017fa6bc29bae57"
VECTORS = {"X":(100,0,0,0), "XY":(100,70,0,0), "XYE":(100,70,0,30),
           "denseXYE":(100,70,0,100), "E":(0,0,0,100), "Z":(0,0,100,0),
           "XYZE":(100,70,100,30), "Eprimary":(40,25,0,100)}

def blob(data):
    return hashlib.sha1(("blob %d\0" % len(data)).encode()+data).hexdigest()

def load_interpreter():
    data = (ROOT/"tools/avr-motion-diagnostics/linked_motion.py").read_bytes()
    assert blob(data) == ARCHIVE_BLOB, "archived interpreter changed; re-audit"
    source = data.decode()
    def change(old, new):
        nonlocal source
        assert source.count(old) == 1, old
        source = source.replace(old,new)
    # ATmega2560 AVRe, 22-bit PC: CALL 5, RCALL/ICALL/EICALL 4, RET/RETI 5.
    change("self.cy+=4 if op=='call' else 3", "self.cy+=5 if op=='call' else 4")
    change("self.cy+=4;self.pc=self.popreturn()", "self.cy+=5;self.pc=self.popreturn()")
    # Capture instruction boundaries, including after final return. No elapsed
    # time injection here. Preserve original instruction semantics otherwise.
    change("pc=self.pc\n            if pc==", "pc=self.pc\n            self.boundary(pc)\n            if pc==")
    module = types.ModuleType("step_output_archived")
    exec(compile(source,"linked_motion.instrumented.py","exec"),module.__dict__)
    parent=module.CPU
    class TraceCPU(parent):
        def __init__(self,img):
            super().__init__(img)
            self.timeline=[]
            self.timer_writes=[]
            self.timer_live=False
            self.tcnt_latch=0
        def boundary(self,pc):
            # instruction start, I bit, OCIE1A, stack usage, call depth.
            self.timeline.append((self.cy,pc,self.flag(7),bool(self.mem[0x6f]&2),
                                  0x21ff-self.sp,len(self.return_stack)))
        def read(self,addr):
            # Single Timer1 CTC compare service: TCNT counts from zero at release.
            # No peripheral flags, matches during ISR or nested interrupts.
            # Low-byte access latches high. Reject rollover as unsupported.
            if self.timer_live and addr==0x84:
                assert self.cy < 61001, "outside one-sentinel-window model"
                self.tcnt_latch=self.cy
                return self.tcnt_latch&255
            if self.timer_live and addr==0x85:
                return self.tcnt_latch>>8
            return super().read(addr)
        def write(self,addr,value):
            super().write(addr,value)
            if addr in (0x88,0x89,0x6f):
                self.timer_writes.append((self.cy,addr,value&255))
    module.CPU=TraceCPU
    return module

def windows(timeline, end):
    # Raw instruction-boundary I-clear windows. Pending interrupt recognition
    # is not identical to I=1; provide eligible boundaries after SEI separately.
    rows=[]
    start=0 if not timeline[0][2] else None
    for t,pc,i,ocie,stack,depth in timeline:
        if not i and start is None: start=t
        elif i and start is not None:
            rows.append([start,t]); start=None
    if start is not None: rows.append([start,end])
    return rows

def irq_opportunities(cpu,vector):
    # Explicit SEI has one-instruction deferral. No IRQ dispatch here.
    # SREG restore and RETI handling remain separate audit items.
    out=[]
    prev_sei=False
    for t,pc,i,ocie,sp,depth in cpu.timeline:
        if i and not prev_sei:
            out.append(t)
        prev_sei=cpu.img.code.get(pc,(0,"",(),None))[1]=="sei"
    return out

def snapshot(cpu,move,errors,o):
    img=cpu.img
    return {
        "remaining":cpu.get(move+98+o,4,signed=True),
        "flags":cpu.get(move+1), "axis_flags":cpu.get(move+7),
        "errors":[cpu.get(errors+4*i,4,signed=True) for i in range(4)],
        "next_group":cpu.symget("StepExecutor::stepsPerTimerCall"),
        "base_interval":cpu.symget("StepExecutor::stepIntervalTicks",4),
        "e_output_bytes":cpu.mem[img.addr("Printer::extrusionOutput"):img.addr("Printer::extrusionOutput")+10].hex(),
        "ecmp_bytes":cpu.mem[img.addr("Printer::extrusionCompensation"):img.addr("Printer::extrusionCompensation")+8].hex(),
        "xy_bytes":cpu.mem[img.addr("Printer::xyMotion"):img.addr("Printer::xyMotion")+36].hex(),
        "z_bytes":cpu.mem[img.addr("Printer::zMotion"):img.addr("Printer::zMotion")+42].hex(),
        "step_clocks_bytes":cpu.mem[img.addr("StepExecutor::stepClocks"):img.addr("StepExecutor::stepClocks")+20].hex(),
    }

def execute(mod,img,kw,mode,case):
    c,pc,move,errors,o=mod.setup(img,**kw)
    assert img.new and img.sizes["PrintLine::queue"]==24*103+5
    assert img.sizes["PrintLine::direct"]==103
    c.mem[0x6f]=2
    if mode=="activation":
        if kw["owner"]: c.put(img.addr("PrintLine::queue")+24*103,0,2)
        else: c.put(move+102,4) # DIRECT_PREPARED
    elif mode=="blocked":
        assert kw["owner"]
        c.put(img.addr("PrintLine::queue")+24*103,0,2)
        c.put(move+1,c.get(move+1)|128)
    elif mode=="stoppable":
        if kw["owner"]:c.put(move+1,c.get(move+1)|16)
        else:c.put(move+102,7)
    elif mode=="idle":
        if kw["owner"]: c.put(img.addr("PrintLine::queue")+24*103,0,5)
        else:c.put(move+102,0)
    elif mode=="wait":
        c.symput("stepperWait",0x9abc,4)
    elif mode=="wait_chunk":
        c.symput("stepperWait",0x19abc,4)
    before=snapshot(c,move,errors,o)
    if mode=="inner":
        # Enter function in the same I-clear state as the caller, but omit wrapper.
        entry=pc
    else:
        entry=17*4
        assert img.code[entry][1] in ("jmp","rjmp")
        c.cy=5 # interrupt hardware entry incl. three-byte return PC
        c.timer_live=True
    val=c.run(entry)
    after=snapshot(c,move,errors,o)
    edges=c.edges
    counts=collections.Counter(pin for pin,state,t in edges if state and not pin.startswith("DIR"))
    if mode in ("inner","active","stoppable") and not kw["advance"] and not kw["ratio"] and not kw["zcor"]:
        n=min(kw["n"],kw["remaining"]);primary=max(kw["delta"])
        expected=[max(0,(n*d-(primary//2 if i<3 else 0)+primary-1)//primary) for i,d in enumerate(kw["delta"])]
        assert all(counts[p]==e for p,e in zip(("X","Y","Z","E"+str(kw["tool"])),expected)), (case,counts,expected)
    mask=windows(c.timeline,c.cy)
    # Record GPIO phase widths within this invocation only.
    last={}; widths=[]
    for pin,state,t in edges:
        if pin in last:
            old,pt=last[pin]
            assert old!=state
            widths.append([pin,"high" if old else "low",t-pt])
        last[pin]=(state,t)
    assert all(state==0 for pin,(state,t) in last.items() if not pin.startswith("DIR"))
    opportunity=irq_opportunities(c,mode!="inner")
    # End timestamp is the point of return to interrupted context, not a
    # guarantee that a pending UART can enter immediately (RETI deferral).
    return {"case":case,"mode":mode,"fixture":kw,"classification":"isolated_linked_instruction_model",
            "exclusive_cycles":c.cy,"elapsed_with_competing_irqs":None,
            "stack_bytes_incl_synthetic_return":0x21ff-c.minsp,
            "masked_intervals_cycles":mask,"max_masked_cycles":max((b-a for a,b in mask),default=0),
            "candidate_irq_opportunity_cycles":opportunity,
            "step_dir_edges":edges,"edge_phase_widths":widths,"physical_rising_edges":dict(counts),
            "timer_writes":c.timer_writes,"ocr1a_final":c.get(0x88,2) if mode!="inner" else None,
            "return_interval":val if mode=="inner" else None,
            "calls":c.calls,"before":before,"after":after,
            "instruction_counts":dict(c.hit)}

def cases():
    # Component measurements at integer primary rates. Not a planner or actual
    # requested speed execution. Short segments need actual-source planner traces.
    def k(owner,delta,rate,n,phase="cruise",advance=False,ratio=0,zcor=0,tool=0,negative=False,remaining=80):
        return dict(owner=owner,delta=delta,rate=rate,n=n,phase=phase,advance=advance,
                    ratio=ratio,zcor=zcor,tool=tool,negative=negative,remaining=remaining)
    for name,delta in VECTORS.items():
        for owner,rate,phase in itertools.product((0,1),(3048,9143,15238,22857),("cruise","accel","decel")):
            n_legacy=math.ceil(3300/max(512,CLOCK//rate))
            for n in sorted({1,n_legacy}):
                kw=k(owner,delta,rate,n,phase)
                for mode in ("inner","active"):
                    yield "%s-%d-%d-%s-%d"%(name,owner,rate,phase,n),mode,kw
    for name in ("XYE","denseXYE","Eprimary","XYZE"):
        for owner,rate,adv,neg in itertools.product((0,1),(9143,15238),(False,True),(False,True)):
            for n in range(1,math.ceil(3300/max(512,CLOCK//rate))+1):
                yield name+"-comp","active",k(owner,VECTORS[name],rate,n,advance=adv,ratio=.25 if owner else 0,zcor=4,negative=neg)
    for owner,name,mode in itertools.product((0,1),("X","XYE","E","Z"),("activation","stoppable","idle","wait","wait_chunk","blocked")):
        if mode=="blocked" and not owner: continue
        yield name+"-"+mode,mode,k(owner,VECTORS[name],9143,2)
    for owner,name,remaining in itertools.product((0,1),("X","XYE","E","Z"),(1,2,4)):
        yield name+"-tail","active",k(owner,VECTORS[name],15238,4,remaining=remaining)
    # Axis-specific rate controls, configured XY rapid and homing, Z max.
    # E calibration/max and physical homing sequence are deferred.
    for owner,name,rate in ((1,"X",22857),(1,"XY",16162),(0,"X",12190),(0,"Z",11520),(1,"Z",11520)):
        for mode in ("active","stoppable"):
            yield name+"-axis-control",mode,k(owner,VECTORS[name],rate,math.ceil(3300/max(512,CLOCK//rate)))

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("archive",type=pathlib.Path)
    parser.add_argument("contract",type=pathlib.Path)
    parser.add_argument("output",type=pathlib.Path)
    parser.add_argument("--board",choices=BOARDS,action="append")
    args=parser.parse_args()
    data=args.contract.read_bytes()
    assert blob(data)==CONTRACT_BLOB
    contract=json.loads(data)
    assert contract["clock_hz"]==CLOCK
    budgets=contract["budgets_cycles"]
    args.output.mkdir(parents=True,exist_ok=True)
    mod=load_interpreter()
    # Synthetic instruction timing sanity check on the instrumented interpreter.
    fake=types.SimpleNamespace(initial=bytearray(0x2200),code={0:(4,"call",("0x8",),""),4:(2,"ret",(),""),8:(2,"ret",(),"")},labels={},step_pins={})
    cpu=mod.CPU(fake);cpu.run(0);assert cpu.cy==15 and cpu.sp==0x21ff and 0x21ff-cpu.minsp==6
    with zipfile.ZipFile(args.archive) as z:
        provenance=json.loads(z.read("summary.json"))
        sha={n:hashlib.sha256(z.read(n)).hexdigest() for n in z.namelist() if n.endswith(".elf")}
    meta={"schema":1,"clock_hz":CLOCK,"contract_blob":CONTRACT_BLOB,"budgets_cycles":budgets,
          "archive_sha256":hashlib.sha256(args.archive.read_bytes()).hexdigest(),"elf_sha256":sha,
          "source_summary":provenance,"classification":"restricted_linked_model_not_WCET"}
    (args.output/"provenance.json").write_text(json.dumps(meta,indent=2))
    failures=0
    for board in args.board or BOARDS:
        img=mod.Image(args.archive,board)
        img.step_pins.update({(0x31,1):"DIR_X",(0x31,7):"DIR_Y",(0x10b,1):"DIR_Z",(0x22,6):"DIR_E0",(0x28,3):"DIR_E1"})
        if board=="RF1000":img.step_pins.pop((0x28,3),None)
        worst={};success=0
        with (args.output/(board+".jsonl")).open("w") as f:
            for case,mode,kw in cases():
                try:
                    row=execute(mod,img,kw,mode,case)
                    success+=1
                    m=worst.get(mode)
                    if m is None or row["max_masked_cycles"]>m["max_masked_cycles"]:
                        worst[mode]={k:row[k] for k in ("case","fixture","exclusive_cycles","max_masked_cycles","stack_bytes_incl_synthetic_return")}
                except Exception as e:
                    failures+=1
                    row={"case":case,"mode":mode,"fixture":kw,"classification":"unsupported_or_failed","error":repr(e)}
                f.write(json.dumps(row,separators=(",",":"))+"\n")
        print(json.dumps({"board":board,"success":success,"worst_mask":worst}),flush=True)
    print("unsupported_or_failed",failures,flush=True)
    return int(bool(failures))

if __name__=="__main__":
    sys.exit(main())

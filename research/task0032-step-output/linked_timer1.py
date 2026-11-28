#!/usr/bin/env python3
"""Task0032-S isolated linked Timer1 paths; NOT a concurrent peripheral emulator."""
import argparse, bisect, collections, hashlib, itertools, json, math, pathlib, sys, types, zipfile

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

class ColdWaitEntry(Exception):
    pass

def load_interpreter():
    data = (ROOT/"tools/avr-motion-diagnostics/linked_motion.py").read_bytes()
    assert blob(data) == ARCHIVE_BLOB, "archived interpreter changed; re-audit"
    source = data.decode()
    def change(old, new):
        nonlocal source
        assert source.count(old) == 1, old
        source = source.replace(old,new)
    # The archived interpreter already adds one base cycle per instruction:
    # CALL 1+4=5, RCALL/ICALL/EICALL 1+3=4, RET/RETI 1+4=5.
    # Preserve these 22-bit PC timings; tested by the synthetic CALL/RET fixture.
    # Capture instruction boundaries, including after final return. No elapsed
    # time injection here. Preserve original instruction semantics otherwise.
    change("pc=self.pc\n            if pc==", "pc=self.pc\n            self.boundary(pc)\n            if pc==")
    # GCC AVR reserves three frame bytes with RCALL .+0: the CPU pushes
    # the next word PC, then continues there. The function epilogue may
    # discard these bytes by adjusting SP rather than RET. Preserve the
    # real push and cycle cost, but do not register a logical call frame.
    change("self.pushreturn(self.pc);self.cy+=4 if op=='call' else 3",
           "self.pushreturn(self.pc);self.cy+=4 if op=='call' else 3\n"
           "                    if op=='rcall' and int(a[0][1:])==0:self.return_stack.pop()")
    # Accept only explicitly supported PrintLine layouts; caller independently
    # verifies DWARF field offsets before running the compact fixture.
    change("size=103 if new else 119", "size=img.sizes['PrintLine::direct'] if new else 119")
    change("o=0 if new else 16", "o=size-103 if new else 16")
    change("move+24+o,struct.pack", "move+24+(0 if new else 16),struct.pack") if "move+24+o,struct.pack" in source else change("move+24+o,struct.unpack", "move+24+(0 if new else 16),struct.unpack")
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
            # Isolated Timer0 starts at phase zero, /64 prescaler. No overflow ISR.
            self.timer0_phase=0
            self.timer0_reads=0
            self.stop_at_micros=False
            self.timer0_flag_cleared_at_wrap=0
        def boundary(self,pc):
            # instruction start, I bit, OCIE1A, stack usage, call depth.
            self.timeline.append((self.cy,pc,self.flag(7),bool(self.mem[0x6f]&2),
                                  0x21ff-self.sp,len(self.return_stack)))
            if self.stop_at_micros and self.img.labels.get(pc)=="micros":
                raise ColdWaitEntry()
        def read(self,addr):
            # Arduino micros() observes TCNT0 and TOV0. Advance time by actual
            # executed cycles, not a stubbed delay. Permit only one pending wrap;
            # a second wrap requires an overflow-service model, not fake time.
            if addr in (0x46,0x35):
                ticks=self.timer0_phase+self.cy//64
                assert ticks<512, "Timer0 second wrap outside isolated model"
                self.timer0_reads+=1
                if addr==0x46: return ticks&255
                return (super().read(addr)&~1)|int(ticks//256>self.timer0_flag_cleared_at_wrap)
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
            assert addr != 0x46, "TCNT0 write outside isolated model"
            if addr==0x35:
                # TIFR0 is write-one-to-clear. Advance arm clears OCF0A, not TOV0.
                if value&1:self.timer0_flag_cleared_at_wrap=(self.timer0_phase+self.cy//64)//256
                self.mem[addr]&=~value
                self.timer_writes.append((self.cy,addr,value&255))
                return
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
        if i and not prev_sei and pc!=0x3fffe:
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
    if getattr(img,"q23",False):
        c.symput("Printer::extrusionCompensation",round(kw["ratio"]*(1<<23)),4)
    if getattr(img,"wake",False):
        # Warm fixture for differential comparison of the existing motion
        # paths. The actual cold/pending wake lifecycle is tested separately.
        assert img.sizes["motorWake"]==6
        fitted=0x1f if img.board in ("RF2000","RF2000v2") else 0x0f
        c.symput("motorWake",fitted,1,4)
        c.symput("motorWake",fitted,1,5)
    if mode in ("activation","activation_cold"):
        # Distinguish warm driver enable from actual cold hardware settling.
        # Extruder::enabled is the final char in its AVR class (no tail padding).
        enabled=mode=="activation"
        assert img.sizes["Printer::enabledStepper"] in (3,4)
        c.symput("Printer::enabledStepper",int.from_bytes(bytes([int(enabled)])*img.sizes["Printer::enabledStepper"],"little"),img.sizes["Printer::enabledStepper"])
        tool_count=2 if img.board in ("RF2000","RF2000v2") else 1
        size=img.sizes["extruder"]//tool_count
        assert size*tool_count==img.sizes["extruder"]
        c.put(img.addr("extruder")+size*(kw["tool"]+1)-1,int(enabled))
        c.stop_at_micros=not enabled
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
    if case=="ecmp-extra":
        # Exact .75 on Q23 grid: force an extra positive pulse at ratio .25;
        # negative path starts at zero and immediately crosses below zero.
        import struct
        if not kw["negative"]:
            c.symput("Printer::extrusionCompensation",3*(1<<21) if getattr(img,"q23",False) else struct.unpack("<I",struct.pack("<f",.75))[0],4,4)
    before=snapshot(c,move,errors,o)
    if mode=="inner":
        # Enter function in the same I-clear state as the caller, but omit wrapper.
        entry=pc
    else:
        entry=17*4
        assert img.code[entry][1] in ("jmp","rjmp")
        c.cy=5 # interrupt hardware entry incl. three-byte return PC
        c.timer_live=True
    try:
        val=c.run(entry)
    except ColdWaitEntry:
        assert mode=="activation_cold"
        mask=windows(c.timeline,c.cy)
        return {"case":case,"mode":mode,"fixture":kw,"classification":"partial_cold_wait_entry",
                "cycles_to_micros_entry":c.cy,"i_at_micros_entry":c.flag(7),
                "max_masked_so_far_cycles":max((b-a for a,b in mask),default=0),
                "masked_intervals_cycles":mask,"calls":c.calls,
                "full_exclusive_cycles":None,"actual_delay_cycles":None,
                "nominal_requested_delay_ms":25,"notes":"Stops before Arduino micros; no Timer0 overflow service simulation."}
    assert mode!="activation_cold", "cold fixture did not enter micros; check enable layout"
    if mode=="activation":assert not any(name=="micros" for name,t in c.calls), "warm fixture still waited"
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
    # Attribute longest I-clear window to linked instructions, including library
    # routines. The label is nearest preceding linked symbol, not source line.
    a,b=max(mask,key=lambda x:x[1]-x[0],default=(0,0))
    label_addrs=sorted(img.labels)
    cost=collections.Counter(); instructions=[]
    if a==0 and c.timeline[0][0]>0:cost["hardware_entry"]+=c.timeline[0][0]
    for first,second in zip(c.timeline,c.timeline[1:]):
        t,pc,i,ocie,sp,depth=first; end=second[0]
        if t>=a and end<=b and end>t:
            index=bisect.bisect_right(label_addrs,pc)-1
            label=img.labels[label_addrs[index]] if index>=0 else "unknown"
            size,op,args,comment=img.code[pc]
            cost[label]+=end-t
            if b-a>400:instructions.append([t,end,hex(pc),op,args,label,depth])
    # End timestamp is the point of return to interrupted context, not a
    # guarantee that a pending UART can enter immediately (RETI deferral).
    return {"case":case,"mode":mode,"fixture":kw,"classification":"isolated_linked_instruction_model",
            "exclusive_cycles":c.cy,"elapsed_with_competing_irqs":None,
            "stack_bytes_incl_synthetic_return":0x21ff-c.minsp,
            "masked_intervals_cycles":mask,"max_masked_cycles":max((b-a for a,b in mask),default=0),
            "candidate_irq_opportunity_cycles":opportunity,
            "longest_mask_attribution_cycles":dict(cost),"longest_mask_instruction_trace":instructions,
            "step_dir_edges":edges,"edge_phase_widths":widths,"physical_rising_edges":dict(counts),
            "timer0_phase_at_start":c.timer0_phase,"timer0_reads":c.timer0_reads,
            "timer_writes":c.timer_writes,"ocr1a_final":c.get(0x88,2) if mode!="inner" else None,
            "return_interval":val if mode=="inner" else None,
            "calls":c.calls,"before":before,"after":after,
            "instruction_counts":dict(c.hit)}

def cases(board):
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
    for owner,name in itertools.product((0,1),("X","XYE","E","Z")):
        yield name+"-cold-enable","activation_cold",k(owner,VECTORS[name],9143,2)
    for owner,name,remaining in itertools.product((0,1),("X","XYE","E","Z"),(1,2,4)):
        yield name+"-tail","active",k(owner,VECTORS[name],15238,4,remaining=remaining)
    # Configured defaults: XY 152.38 steps/mm, Z 1280, E 280 or 256.
    # Factory XY travel 150, homing XY 80, Z 9, E 25 mm/s.
    # Each control is a synthetic prepared profile, not a homing sequence.
    emax=6400 if board=="RF2000v2" else 7000
    for adv,neg,n in itertools.product((False,True),(False,True),(1,2)):
        yield "Eprimary-config-max","active",k(1,VECTORS["Eprimary"],emax,n,advance=adv,ratio=.25,zcor=4,negative=neg)
    for owner,name,rate in ((1,"X",22857),(1,"XY",16162),(0,"X",12190),(0,"Z",11520),(1,"Z",11520),(0,"E",emax),(1,"E",emax)):
        for mode in ("active","stoppable"):
            yield name+"-axis-control",mode,k(owner,VECTORS[name],rate,math.ceil(3300/max(512,CLOCK//rate)))

    # Exact floor/ceil quantization cases, straight X with optional base E.
    # These are isolated at requested cruise rate, NOT achievable planner speeds.
    for speed,length,extrude in itertools.product((20,60,100),(.5,.2,.1,.05,.025),(False,True)):
        rate=round(speed*152.38)
        for steps in sorted({math.floor(length*152.38),math.ceil(length*152.38)}):
            delta=(steps,0,0,round(steps*.3) if extrude else 0)
            for n in sorted({1,math.ceil(3300/max(512,CLOCK//rate))}):
                kw=k(1,delta,rate,n,remaining=steps)
                yield "segment-%s-%s-%s"%(speed,length,extrude),"active",kw
                yield "segment-%s-%s-%s"%(speed,length,extrude),"activation",kw
    for tool in range(2 if board in ("RF2000","RF2000v2") else 1):
        for advance,negative,ratio in itertools.product((False,True),(False,True),(.25,1.0)):
            yield "ecmp-extra","active",k(1,VECTORS["E"],emax,2,advance=advance,ratio=ratio,tool=tool,negative=negative)

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
    cpu=mod.CPU(fake)
    for cycles,count,overflow in ((0,0,0),(63,0,0),(64,1,0),(16383,255,0),(16384,0,1),(16448,1,1)):
        cpu.cy=cycles
        assert cpu.read(0x46)==count and cpu.read(0x35)&1==overflow
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
        worst={};success=0;errors=collections.Counter();samples=[];cold=[];mask_by_case={};paired={}
        with (args.output/(board+".jsonl")).open("w") as f:
            for case,mode,kw in cases(board):
                try:
                    row=execute(mod,img,kw,mode,case)
                    if row["classification"]=="partial_cold_wait_entry":
                        cold.append({k:row[k] for k in ("case","fixture","cycles_to_micros_entry","i_at_micros_entry","max_masked_so_far_cycles")})
                    else:
                        success+=1
                        m=worst.get(mode)
                        if m is None or row["max_masked_cycles"]>m["max_masked_cycles"]:
                            worst[mode]={k:row[k] for k in ("case","fixture","exclusive_cycles","max_masked_cycles","stack_bytes_incl_synthetic_return","longest_mask_attribution_cycles")}
                        if case=="Eprimary-config-max":
                            mask_by_case[str((kw["advance"],kw["negative"],kw["n"]))]={k:row[k] for k in ("exclusive_cycles","max_masked_cycles","longest_mask_attribution_cycles")}
                        if kw["phase"]=="cruise" and kw["rate"]==9143 and kw["owner"]==1 and not any((kw["advance"],kw["ratio"],kw["zcor"],kw["negative"])) and kw["remaining"]==80 and mode in ("inner","active"):
                            paired[str((mode,tuple(kw["delta"]),kw["n"]))]=row["exclusive_cycles"]
                except Exception as e:
                    failures+=1
                    errors[repr(e)]+=1
                    if len(samples)<8:samples.append({"case":case,"mode":mode,"fixture":kw,"error":repr(e)})
                    row={"case":case,"mode":mode,"fixture":kw,"classification":"unsupported_or_failed","error":repr(e)}
                f.write(json.dumps(row,separators=(",",":"))+"\n")
        print(json.dumps({"board":board,"success":success,"worst_mask":worst,"failed":sum(errors.values()),"error_samples":samples,"top_errors":errors.most_common(5),"cold_wait_entries":cold,"configured_E_primary_masks":mask_by_case,"paired_cruise_cycles":paired}),flush=True)
    print("unsupported_or_failed",failures,flush=True)
    return int(bool(failures))

if __name__=="__main__":
    sys.exit(main())

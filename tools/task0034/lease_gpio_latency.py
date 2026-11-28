"""Count the linked, straight-line lease expiry prefix on the canonical images."""
import re


def gpio_latency(board, ins, labels, branch_target, require_priority_fallback=False):
    address = {name:pc for pc,name in labels.items()}
    vector = address["__vector_32"]
    leaf = address["(anonymous namespace)::failHeaterLease()"]
    channels = (32,33,34) if board in ("RF2000","RF2000v2") else (32,33)
    for v in channels:
        _,op,args = ins[4*v]
        if op != "jmp" or branch_target(args) != vector:
            raise ValueError((board,v,"unexpected vector slot"))
    pc = vector
    cycles = 0
    prologue = {"push":2, "in":1, "eor":1}
    while ins[pc][1] in prologue:
        width, op, _ = ins[pc]
        cycles += prologue[op]
        pc += width
    if ins[pc][1] != "call" or branch_target(ins[pc][2]) != leaf:
        raise ValueError((board,"unreviewed expiry entry"))
    cycles += 5
    pc = leaf
    leaf_entry_cycles = cycles

    def require(op, operand, count):
        nonlocal pc,cycles
        width,actual,args = ins[pc]
        if actual != op or args.split(';')[0].strip() != operand:
            raise ValueError((board, hex(pc), op, operand, actual, args))
        cycles += count
        pc += width

    require("ldi","r24, 0x01",1)
    for symbol, register in (("heaterLeaseFailed","r24"),("heaterGrantProgress","r1")):
        width, op, args = ins[pc]
        if op != "sts" or symbol not in args or args.split(";")[0].strip().split(", ")[1] != register:
            raise ValueError((board, hex(pc),symbol))
        pc += width
        cycles += 2
    require("cbi","0x05, 4",2)
    pins = {"PB4":cycles}
    if board != "RF2000-X44":
        require("ldi","r30, 0x02",1)
        require("ldi","r31, 0x01",1)
        require("ld","r24, Z",2)
        require("andi","r24, 0xDF" if board == "RF1000" else "r24, 0xBF",1)
        require("st","Z, r24",2)
        pins["PH5" if board == "RF1000" else "PH6"] = cycles
    if board != "RF1000":
        require("cbi","0x0e, 2",2)
        pins["PE2"] = cycles
    report = {"cyclesFromFirstHandlerInstruction":pins, "clockHz":16000000}
    if require_priority_fallback:
        leaf_pins = {pin:count-leaf_entry_cycles for pin,count in pins.items()}
        report["priorityFallback"] = {
            str(v): priority_fallback(board, v, ins, address, branch_target, leaf, leaf_pins)
            for v in (17,21,22)
        }
    return report


def priority_fallback(board, vector, ins, address, branch_target, leaf, leaf_pins):
    """Prove all motion/PWM expiry checks precede any other body work."""
    pc = address["__vector_"+str(vector)]
    _,op,arg = ins[4*vector]
    assert op=="jmp" and branch_target(arg)==pc, (board,vector,"vector slot")
    prologue_cycles=0
    while True:
        width,op,arg=ins[pc]
        a=arg.split(";")[0].strip()
        if op=="in" and a.endswith(", 0x18"):  # TIFR3
            break
        if op=="push" and re.fullmatch(r"r\d+",a):
            prologue_cycles+=2
        elif op=="in" and re.fullmatch(r"r\d+, 0x3[bfde]",a):
            prologue_cycles+=1  # SREG, RAMPZ or SP
        elif op=="eor" and a=="r1, r1":
            prologue_cycles+=1
        else:
            raise ValueError((board,vector,hex(pc),"unreviewed prefix",op,a))
        pc+=width
    flag_site=pc
    flag_reg=a.split(", ")[0]
    pc+=width
    width,op,arg=ins[pc]
    a=arg.split(";")[0].strip()
    assert op=="lds" and a.endswith(", 0x0071"), (board,vector,"TIMSK3",a)
    mask_reg=a.split(", ")[0]
    assert flag_reg!=mask_reg
    pc+=width

    def require(expected, operands):
        nonlocal pc
        width,op,arg=ins[pc]
        assert op==expected and arg.split(";")[0].strip()==operands, (board,vector,hex(pc),op,arg)
        pc+=width

    require("and",flag_reg+", "+mask_reg)
    mask=0x0e if board in ("RF2000","RF2000v2") else 0x06
    require("andi",flag_reg+", 0x"+format(mask,"02X"))
    width,op,arg=ins[pc]
    assert op=="breq", (board,vector,"zero-mask branch",arg)
    skip=branch_target(arg)
    pc+=width
    # The small Advance ISR inlines the cold branch to avoid saving all
    # call-clobbered registers. Verify the same exact pin and state stores.
    if vector == 21:
        cycles = prologue_cycles + 6
        require("ldi","r24, 0x01")
        cycles+=1
        for symbol, register in (("heaterLeaseFailed","r24"),("heaterGrantProgress","r1")):
            width,op,arg=ins[pc]
            assert op=="sts" and symbol in arg and arg.split(";")[0].strip().endswith(", "+register), (board,vector,hex(pc),symbol)
            pc+=width
            cycles+=2
        require("cbi","0x05, 4")
        cycles+=2
        pins={"PB4":cycles}
        if board!="RF2000-X44":
            require("lds","r24, 0x0102")
            require("andi","r24, 0xDF" if board=="RF1000" else "r24, 0xBF")
            require("sts","0x0102, r24")
            cycles+=5
            pins["PH5" if board=="RF1000" else "PH6"]=cycles
        if board!="RF1000":
            require("cbi","0x0e, 2")
            cycles+=2
            pins["PE2"]=cycles
        count=3 if mask==0x0e else 2
        base=None
        for i in range(count):
            width,op,arg=ins[pc]
            operand=arg.split(";")[0].strip().split(", ")
            assert op=="sts" and "requestedDuty" in arg and operand[1]=="r1", (board,vector,hex(pc),arg)
            adr=int(operand[0],0)
            if base is None:base=adr
            assert adr==base+i
            pc+=width
        require("sts","0x0071, r1")
        assert pc==skip, (board,vector,"unexpected inline expiry suffix")
    else:
        width,op,arg=ins[pc]
        assert op=="call" and branch_target(arg)==leaf and pc+width==skip, (board,vector,"leaf call",arg)
        pins={pin:prologue_cycles+11+c for pin,c in leaf_pins.items()}
    # No vector admission or in-flight instruction included.
    return {"flagReadSite":hex(flag_site),
            "cyclesToFlagRead":prologue_cycles+1,
            "normalCheckCycles":7,
            "cyclesFromFirstHandlerInstruction":pins}


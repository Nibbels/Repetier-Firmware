"""Count the linked, straight-line lease expiry prefix on the canonical images."""
def gpio_latency(board, ins, labels, branch_target):
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
    return {"cyclesFromFirstHandlerInstruction":pins, "clockHz":16000000}

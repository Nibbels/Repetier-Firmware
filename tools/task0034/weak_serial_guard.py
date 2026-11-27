"""Conservative zero-value tracking for optional weak Arduino serialEventRun.

Track only AVR ABI registers r1 (zero), r16/r17 (callee saved). Ordinary
branches remain unconstrained except an immediately witnessed CP/CPC/BREQ
on known zero operands. Unknown operations clear facts. Assumes ABI-
conforming callees/interrupts and intact CPU state, as the frame walk does.
"""
TRACK={1:1,16:2,17:4}

def zero_transfer(op,arg,mask,phase):
    a=arg.split(';')[0].strip();t=a.split(', ')
    def bit(s):
        return TRACK.get(int(s[1:]),0) if s.startswith('r') and s[1:].isdigit() else 0
    def known(s):return bool(mask&bit(s))
    def setbit(s,z):
        nonlocal mask
        b=bit(s);mask=(mask&~b)|(b if z else 0)
    if op=='cp' and t==['r16','r1'] and mask&3==3:return mask,1
    if op=='cpc' and t==['r17','r1'] and phase==1 and mask&5==5:return mask,2
    if op=='ldi':setbit(t[0],int(t[1],0)==0)
    elif op in ('clr','eor') and (op=='clr' or t[0]==t[1]):setbit(t[0],True)
    elif op=='mov':setbit(t[0],known(t[1]))
    elif op=='movw':
        d,s=(int(x[1:]) for x in t)
        lo,hi=known('r'+str(s)),known('r'+str(s+1))
        setbit('r'+str(d),lo);setbit('r'+str(d+1),hi)
    elif op in ('mul','muls','mulsu','fmul','fmuls','fmulsu'):mask&=~1
    elif op in ('adiw','sbiw'):
        d=int(t[0][1:]);setbit(t[0],False);setbit('r'+str(d+1),False)
    elif op in ('call','rcall','icall','eicall'):
        # Preserve only facts already known for AVR GCC ABI registers.
        pass
    elif op in ('adc','add','and','andi','asr','bld','com','dec','eor','inc','in','ld','ldd','lds','lsl','lsr','neg','or','ori','pop','ror','sbc','sbci','ser','sub','subi','swap','rol'):
        setbit(t[0],False)
    elif op in ('lpm','elpm'):
        if a:setbit(t[0],False)
    elif op not in ('cp','cpc','cpi','cpse','tst','sbrc','sbrs','sbic','sbis','st','std','sts','out','sbi','cbi','nop','push','ret','reti','jmp','rjmp','ijmp','eijmp','break','cli','sei','wdr','bset','bclr','clc','sec','clt','set','bst') and not op.startswith('br'):
        mask=0
    return mask,0

def prove_weak_main(ins,tables,main,target):
    pending=[(main,1,0)];seen=set();reachable_zero=set();pruned=set()
    while pending:
        pc,mask,phase=pending.pop()
        if (pc,mask,phase) in seen:continue
        seen.add((pc,mask,phase))
        if len(seen)>=200000 or pc not in ins:
            raise ValueError(('incomplete weak main graph',hex(pc)))
        w,op,arg=ins[pc];n=pc+w
        oldphase=phase;mask,phase=zero_transfer(op,arg,mask,phase)
        if pc in tables:nexts=tables[pc]
        elif op in ('call','rcall'):
            if target(arg)==0:reachable_zero.add(pc);continue
            nexts=[n]
        elif op in ('jmp','rjmp'):
            t=target(arg);nexts=[t] if t else []
        elif op in ('ret','reti','break'):continue
        elif op in ('icall','eicall'):nexts=[n]
        elif op in ('ijmp','eijmp') or op.startswith('.'):
            raise ValueError(('weak main unresolved transfer',hex(pc),op))
        elif op=='breq' and oldphase==2:
            t=target(arg);nexts=[t]
            if n in ins and ins[n][1] in ('call','rcall') and target(ins[n][2])==0:
                pruned.add(n)
        elif op.startswith('br'):nexts=[n,target(arg)]
        elif op in ('sbrc','sbrs','sbic','sbis','cpse'):nexts=[n,n+ins[n][0]]
        else:nexts=[n]
        pending.extend((p,mask,phase) for p in nexts)
    # A CALL 0 reached through any alternate incoming path is NOT proven.
    return pruned-reachable_zero

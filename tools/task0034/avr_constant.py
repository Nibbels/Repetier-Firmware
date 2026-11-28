"""Conservative AVR byte/flag constant propagation for A73.

Unknown memory loads stay unknown. Only known conditions remove edges.
Assumes intact compiler-generated call frames and saved register slots;
arbitrary memory corruption is outside this instruction-path diagnostic.
"""

def transfer(op, args, values, flags, stack):
    v = dict(values)
    f = dict(flags)
    t = args.split(', ')
    def r(s): return int(s[1:])
    def read(s): return v.get(r(s)) if s.startswith('r') else int(s,0)
    def write(i,x):
        if x is None: v.pop(i,None)
        else: v[i]=x&255
    def flag(k,x):
        if x is None: f.pop(k,None)
        else: f[k]=int(bool(x))
    def nzvs(x,overflow):
        flag('Z',None if x is None else x==0)
        flag('N',None if x is None else x&128)
        flag('V',overflow)
        flag('S',None if x is None or overflow is None else bool(x&128)^bool(overflow))
    def clear(keys):
        for k in keys: flag(k,None)
    def nz(x): nzvs(x,0)
    if op=='ldi': write(r(t[0]),read(t[1]))
    elif op=='mov': write(r(t[0]),read(t[1]))
    elif op=='movw':
        i,j=map(r,t);a,b=v.get(j),v.get(j+1);write(i,a);write(i+1,b)
    elif op=='push': stack+=(read(t[0]),)
    elif op=='pop': write(r(t[0]),stack[-1]);stack=stack[:-1]
    elif op in ('in','lds','ld','ldd','lpm','elpm'):
        write(r(t[0]) if args else 0,None)
    elif op in ('and','andi','or','ori','eor','tst','clr','ser'):
        a=read(t[0]);b=read(t[1]) if len(t)>1 else a
        if op=='ser': write(r(t[0]),255)
        else:
            if op=='clr' or (op=='eor' and t[0]==t[1]): x=0
            elif op=='tst': x=a
            elif op in ('and','andi') and (a==0 or b==0): x=0
            elif op in ('or','ori') and (a==255 or b==255): x=255
            elif a is None or b is None: x=None
            elif op in ('and','andi'): x=a&b
            elif op in ('or','ori'): x=a|b
            else: x=a^b
            if op!='tst': write(r(t[0]),x)
            nz(x)
    elif op in ('dec','inc','com','neg','swap'):
        a=read(t[0]);x=None
        if a is not None:
            x={'dec':(a-1)&255,'inc':(a+1)&255,'com':255-a,'neg':(-a)&255,'swap':((a<<4)|(a>>4))&255}[op]
        write(r(t[0]),x)
        if op=='dec': nzvs(x,None if x is None else x==127)
        elif op=='inc': nzvs(x,None if x is None else x==128)
        elif op=='com': nz(x);flag('C',1)
        elif op=='neg': nzvs(x,None if x is None else x==128);flag('C',None if x is None else x!=0)
    elif op in ('add','adc','sub','subi','sbc','sbci','cp','cpi','cpc','lsl','rol'):
        a=read(t[0]);b=read(t[1]) if len(t)>1 else a
        add=op in ('add','adc','lsl','rol');carry=f.get('C') if op in ('adc','sbc','sbci','cpc','rol') else 0
        oldz=f.get('Z');x=None;co=None;overflow=None
        if a is not None and b is not None and carry is not None:
            u=a+b+carry if add else a-b-carry
            x=u&255;co=u>255 if add else u<0
            sa=a if a<128 else a-256;sb=b if b<128 else b-256
            signed=sa+sb+carry if add else sa-sb-carry
            overflow=not -128<=signed<=127
        if op not in ('cp','cpi','cpc'): write(r(t[0]),x)
        nzvs(x,overflow);flag('C',co)
        if op in ('sbc','sbci','cpc'):
            flag('Z',False if x is not None and x!=0 or oldz==0 else (True if x==0 and oldz==1 else None))
        if op in ('adc','rol'): flag('Z',None) # do not depend on ADC zero-chain semantics
    elif op in ('adiw','sbiw'):
        i=r(t[0]);a=v.get(i);b=v.get(i+1);x=None;co=None;ov=None
        if a is not None and b is not None:
            old=a|(b<<8);n=read(t[1]);u=old+n if op=='adiw' else old-n;x=u&65535
            co=u>65535 if op=='adiw' else u<0
            ov=(old<32768 and x>=32768) if op=='adiw' else (old>=32768 and x<32768)
        write(i,None if x is None else x&255);write(i+1,None if x is None else x>>8)
        flag('Z',None if x is None else x==0);flag('N',None if x is None else x&32768);flag('C',co);flag('V',ov);flag('S',None if x is None else bool(x&32768)^ov)
    elif op in ('asr','lsr','ror'):
        a=read(t[0]);carry=f.get('C');x=None;co=None
        if a is not None:
            co=a&1
            if op=='asr': x=(a>>1)|(a&128)
            elif op=='lsr': x=a>>1
            elif carry is not None: x=(a>>1)|(carry<<7)
        write(r(t[0]),x);flag('C',co);flag('Z',None if x is None else x==0);flag('N',None if x is None else x&128);flag('V',None if x is None or co is None else bool(x&128)^bool(co));flag('S',co)
    elif op in ('mul','muls','mulsu','fmul','fmuls','fmulsu'):
        write(0,None);write(1,None);clear('ZC')
    elif op=='out' and t[0]=='0x3f': clear('ZCNVST')
    elif op=='bld': write(r(t[0]),None)
    elif op=='bst':
        a=read(t[0]);flag('T',None if a is None else a&(1<<int(t[1])))
    elif op=='clc': flag('C',0)
    elif op=='sec': flag('C',1)
    elif op=='clt': flag('T',0)
    elif op=='set': flag('T',1)
    elif op in ('bset','bclr'):
        clear('ZCNVST')
    elif op not in ('cpse','sbrc','sbrs','sbic','sbis','st','std','sts','out','sbi','cbi','nop','ret','reti','jmp','rjmp','call','rcall','ijmp','eijmp','icall','eicall','break','cli','sei','wdr') and not op.startswith('br'):
        raise ValueError((op,args))
    if op in ('ld','st','lpm','elpm'):
        import re
        for i,rs in enumerate(('X','Y','Z')):
            if re.search(r'(?:^|, )(?:'+rs+r'\+|-'+rs+r')(?:,|$)',args):
                write(26+2*i,None);write(27+2*i,None)
    return tuple(sorted(v.items())),tuple(sorted(f.items())),stack


def branch(op,args,values,flags):
    """True/false if the taken edge (or skip edge) is certain; else None."""
    v=dict(values);f=dict(flags)
    cond={'breq':('Z',1),'brne':('Z',0),'brcs':('C',1),'brlo':('C',1),'brcc':('C',0),'brsh':('C',0),'brpl':('N',0),'brmi':('N',1),'brge':('S',0),'brlt':('S',1),'brtc':('T',0),'brts':('T',1),'brvs':('V',1),'brvc':('V',0)}
    if op in cond:
        k,x=cond[op];return None if k not in f else f[k]==x
    t=args.split(', ')
    if op=='cpse':
        if t[0]==t[1]:return True
        a,b=[v.get(int(s[1:])) for s in t];return None if a is None or b is None else a==b
    if op in ('sbrc','sbrs'):
        a=v.get(int(t[0][1:]));return None if a is None else bool(a&(1<<int(t[1])))==(op=='sbrs')
    return None

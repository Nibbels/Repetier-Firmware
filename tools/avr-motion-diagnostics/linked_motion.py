#!/usr/bin/env python3
"""Restricted interpreter for AVR linked motion diagnostics. No peripheral time model."""
import collections, re, struct, sys, zipfile

class Image:
    def __init__(self,path,board):
        with zipfile.ZipFile(path) as z:
            sy=z.read(board+'/symbols.txt').decode()
            self.symbols={}; self.sizes={}
            for line in sy.splitlines():
                m=re.match(r'^([0-9a-f]+) (?:([0-9a-f]+) )?[a-zA-Z] (.+)$',line)
                if m:
                    a,s,n=m.groups(); self.symbols[n]=int(a,16); self.sizes[n]=int(s or '0',16)
            self.code={}; self.labels={}
            dis=z.read(board+'/disassembly.txt').decode()
            for line in dis.splitlines():
                m=re.match(r'^([0-9a-f]+) <(.+)>:$',line)
                if m: self.labels[int(m[1],16)]=m[2]; continue
                m=re.match(r'^\s*([0-9a-f]+):\s+((?:[0-9a-f]{2}\s+)+)\s*([a-z]+)\s*(.*)$',line)
                if m:
                    a,raw,op,tail=m.groups(); operands,_,comment=tail.partition(';')
                    args=tuple(x.strip() for x in operands.split(',')) if operands.strip() else ()
                    self.code[int(a,16)]=(len(raw.split()),op,args,comment.strip())
            self.flash=bytearray(262144)
            upper=0
            for line in z.read(board+'/build/Repetier.ino.hex').decode().splitlines():
                raw=bytes.fromhex(line[1:]); n,hi,lo,k=raw[:4]; assert sum(raw)%256==0
                if k==4:upper=int.from_bytes(raw[4:6],'big')<<16
                if k==2:upper=int.from_bytes(raw[4:6],'big')<<4
                if k==0:
                    addr=upper+hi*256+lo;self.flash[addr:addr+n]=raw[4:4+n]
        start=self.addr('__data_start');end=self.addr('__data_end');fl=self.symbols['__data_load_start']
        self.initial=bytearray(0x2200);self.initial[start:end]=self.flash[fl:fl+end-start]
        self.board=board; self.new='StepExecutor::stepErrors' in self.symbols
        self.step_pins={(0x31,0):'X',(0x31,6):'Y',(0x10b if board=='RF1000' else 0x31,3):'Z',(0x22,4):'E0',(0x28,1):'E1'}
    def addr(self,name): return self.symbols[name]&0xffff
    def find(self,substring):
        n=[n for n in self.symbols if substring in n and self.sizes[n] and self.symbols[n]<0x40000];assert len(n)==1,n;return self.symbols[n[0]]

class CPU:
    def __init__(self,img):
        self.img=img; self.mem=img.initial[:]; self.r=bytearray(32); self.sreg=0;self.sp=0x21ff;self.pc=0;self.cy=0;self.edges=[];self.calls=[];self.minsp=self.sp;self.irqstart=0;self.maxirq=0;self.hit=collections.Counter();self.return_stack=[]
    def get(self,addr,n=1,signed=False):return int.from_bytes(self.mem[addr:addr+n],'little',signed=signed)
    def put(self,addr,value,n=1):self.mem[addr:addr+n]=(value&((1<<(8*n))-1)).to_bytes(n,'little')
    def symput(self,name,value,n=1,offset=0):self.put(self.img.addr(name)+offset,value,n)
    def symget(self,name,n=1,offset=0,signed=False):return self.get(self.img.addr(name)+offset,n,signed)
    def flag(self,k):return (self.sreg>>k)&1
    def setflag(self,k,v):self.sreg=(self.sreg&~(1<<k))|(bool(v)<<k)
    def nzvs(self,v,ov,z=None,bits=8):
        v&=(1<<bits)-1;n=v>>(bits-1);self.setflag(1,v==0 if z is None else z);self.setflag(2,n);self.setflag(3,ov);self.setflag(4,n^bool(ov))
    def reg(self,a):return int(a[1:])
    def v(self,a):return self.r[self.reg(a)] if a.startswith('r') else int(a,0)
    def pair(self,r):return self.r[r]+256*self.r[r+1]
    def setpair(self,r,v):self.r[r]=v&255;self.r[r+1]=(v>>8)&255
    def ptr(self,a):
        idx={'X':26,'Y':28,'Z':30}[a.strip('-')[0]];x=self.pair(idx)
        if a.startswith('-'):x=(x-1)&65535;self.setpair(idx,x)
        if a.endswith('+'):self.setpair(idx,x+1)
        elif '+' in a:x+=int(a.split('+')[1],0)
        return x&65535
    def read(self,a):
        if a==0x5f:return self.sreg
        if a==0x5d:return self.sp&255
        if a==0x5e:return self.sp>>8
        if a<32:return self.r[a]
        if a in (0x20,0x26,0x2c,0x2f,0x100,0x109):
            d=self.mem[a+1];return (self.mem[a]&~d)|(self.mem[a+2]&d)
        return self.mem[a]
    def write(self,a,v):
        v&=255
        if a==0x5f:
            old=self.flag(7);self.sreg=v
            if old!=self.flag(7):self.irqchange(old)
            return
        if a==0x5d:self.sp=(self.sp&0xff00)|v;self.minsp=min(self.sp,self.minsp);return
        if a==0x5e:self.sp=(self.sp&0xff)|v<<8;self.minsp=min(self.sp,self.minsp);return
        if a<32:self.r[a]=v;return
        diff=self.mem[a]^v
        if diff:
            for bit in range(8):
                key=(a,bit)
                if key in self.img.step_pins and diff&(1<<bit):self.edges.append((self.img.step_pins[key],(v>>bit)&1,self.cy))
        self.mem[a]=v
    def irqchange(self,old):
        if old and not self.flag(7):self.irqstart=self.cy
        if not old and self.flag(7):self.maxirq=max(self.maxirq,self.cy-self.irqstart)
    def push(self,v):self.mem[self.sp]=v&255;self.sp-=1;self.minsp=min(self.minsp,self.sp)
    def pop(self):self.sp+=1;return self.mem[self.sp]
    def pushreturn(self,pc):
        # AVR return address is a word address on the actual 3-byte hardware stack.
        w=pc//2
        for shift in (0,8,16):self.push(w>>shift)
        self.return_stack.append(pc)
    def popreturn(self):
        word=(self.pop()<<16)|(self.pop()<<8)|self.pop();pc=word*2;assert pc==self.return_stack.pop(),(hex(pc),self.return_stack);return pc
    def run(self,pc,limit=100000):
        self.pc=pc;self.pushreturn(0x3fffe)
        for _ in range(limit):
            pc=self.pc
            if pc==0x3fffe:break
            try:n,op,a,comment=self.img.code[pc]
            except KeyError:raise RuntimeError(('missing',hex(pc),self.calls[-10:]))
            self.pc+=n;self.hit[op]+=1;self.cy+=1
            R=self.r; V=self.v; reg=self.reg
            if op in ('mov','ldi'):R[reg(a[0])]=V(a[1])
            elif op=='movw':R[reg(a[0]):reg(a[0])+2]=R[reg(a[1]):reg(a[1])+2]
            elif op in ('lds','ld','ldd','in'):
                addr=int(a[1],0)+(0x20 if op=='in' else 0) if op in ('lds','in') else self.ptr(a[1]);R[reg(a[0])]=self.read(addr);self.cy+=op!='in'
            elif op in ('sts','st','std','out'):
                addr=int(a[0],0)+(0x20 if op=='out' else 0) if op in ('sts','out') else self.ptr(a[0]);self.cy+=op!='out';self.write(addr,V(a[1]))
            elif op in ('lpm','elpm'):
                self.cy+=2;addr=self.ptr(a[1] if len(a)>1 else 'Z');addr+=self.mem[0x5b]*65536 if op=='elpm' else 0;R[reg(a[0]) if a else 0]=self.img.flash[addr]
            elif op in ('push','pop'):
                self.cy+=1
                if op=='push':self.push(V(a[0]))
                else:R[reg(a[0])]=self.pop()
            elif op in ('sbi','cbi'):
                self.cy+=1;addr=int(a[0],0)+32;mask=1<<V(a[1]);self.write(addr,(self.read(addr)|mask) if op=='sbi' else (self.read(addr)&~mask))
            elif op in ('sei','cli','sec','clc','set','clt','bset','bclr'):
                k={'sei':7,'cli':7,'sec':0,'clc':0,'set':6,'clt':6}.get(op,V(a[0]) if a else 0);old=self.flag(7);self.setflag(k,op in ('sei','sec','set','bset'))
                if k==7 and old!=self.flag(7):self.irqchange(old)
            elif op in ('cp','cpi','cpc','sub','subi','sbc','sbci','add','adc'):
                x,y=V(a[0]),V(a[1]);c=self.flag(0) if op in ('cpc','sbc','sbci','adc') else 0
                sub=op not in ('add','adc');full=x-y-c if sub else x+y+c;v=full&255
                self.setflag(0,full<0 if sub else full>255);self.setflag(5,(x&15)-(y&15)-c<0 if sub else (x&15)+(y&15)+c>15)
                ov=bool(((x^y)&(x^v)&128) if sub else (~(x^y)&(x^v)&128));z=(v==0) and (self.flag(1) if op in ('cpc','sbc','sbci') else True)
                self.nzvs(v,ov,z)
                if op not in ('cp','cpi','cpc'):R[reg(a[0])]=v
            elif op in ('and','andi','or','ori','eor','clr','tst'):
                x=V(a[0]);y=V(a[1]) if len(a)>1 else x;v={'and':x&y,'andi':x&y,'or':x|y,'ori':x|y,'eor':x^y,'clr':0,'tst':x}[op];self.nzvs(v,0);R[reg(a[0])]=v
            elif op in ('inc','dec','com','neg','swap','lsl','rol','lsr','asr','ror'):
                x=V(a[0]);carry=self.flag(0)
                if op=='inc':v=(x+1)&255;self.nzvs(v,v==128)
                elif op=='dec':v=(x-1)&255;self.nzvs(v,v==127)
                elif op=='com':v=255-x;self.setflag(0,1);self.nzvs(v,0)
                elif op=='neg':v=(-x)&255;self.setflag(0,v!=0);self.setflag(5,bool((v|x)&8));self.nzvs(v,v==128)
                elif op=='swap':v=((x&15)<<4)|(x>>4)
                elif op in ('lsl','rol'):
                    c=carry if op=='rol' else 0;v=(x*2+c)&255;self.setflag(0,x>>7);self.setflag(5,(x>>3)&1);self.nzvs(v,((x^v)>>7)&1)
                else:
                    v=(x>>1)|((carry<<7) if op=='ror' else ((x&128) if op=='asr' else 0));self.setflag(0,x&1);self.nzvs(v,(v>>7)^(x&1))
                R[reg(a[0])]=v
            elif op in ('adiw','sbiw'):
                self.cy+=1;idx=reg(a[0]);x=self.pair(idx);y=V(a[1]);v=(x+y if op=='adiw' else x-y)&65535;self.setpair(idx,v);self.setflag(0,x+y>65535 if op=='adiw' else x<y);self.nzvs(v,((not bool(x&32768)) and bool(v&32768)) if op=='adiw' else (bool(x&32768) and not bool(v&32768)),bits=16)
            elif op in ('mul','muls','mulsu','fmul','fmuls','fmulsu'):
                self.cy+=1;x,y=V(a[0]),V(a[1]);x=x-256 if op in ('muls','mulsu','fmuls','fmulsu') and x&128 else x;y=y-256 if op in ('muls','fmuls') and y&128 else y;p=(x*y)&65535;self.setflag(0,p&32768);p=(p<<1)&65535 if op.startswith('f') else p;self.setpair(0,p);self.setflag(1,p==0)
            elif op in ('bst','bld'):
                mask=1<<V(a[1]);i=reg(a[0]);self.setflag(6,R[i]&mask) if op=='bst' else None
                if op=='bld':R[i]=(R[i]&~mask)|(self.flag(6)<<V(a[1]))
            elif op in ('cpse','sbrs','sbrc','sbis','sbic'):
                take=V(a[0])==V(a[1]) if op=='cpse' else bool((self.read(V(a[0])+32) if op in ('sbis','sbic') else V(a[0])) & (1<<V(a[1])))==(op in ('sbrs','sbis'))
                if take:size=self.img.code[self.pc][0];self.pc+=size;self.cy+=size//2
            elif op.startswith('br'):
                k,truth={'breq':(1,1),'brne':(1,0),'brcs':(0,1),'brlo':(0,1),'brcc':(0,0),'brsh':(0,0),'brmi':(2,1),'brpl':(2,0),'brvs':(3,1),'brvc':(3,0),'brlt':(4,1),'brge':(4,0),'brhs':(5,1),'brhc':(5,0),'brts':(6,1),'brtc':(6,0),'brie':(7,1),'brid':(7,0)}[op] if op not in ('brbs','brbc') else (V(a[0]),op=='brbs')
                if self.flag(k)==truth:self.pc+=int(a[-1][1:]);self.cy+=1
            elif op in ('call','rcall','jmp','rjmp','icall','eicall','ijmp','eijmp'):
                if op in ('call','rcall','icall','eicall'):
                    self.pushreturn(self.pc);self.cy+=4 if op=='call' else 3
                else:self.cy+=2 if op=='jmp' else 1
                self.pc=(self.pc+int(a[0][1:])) if op in ('rcall','rjmp') else (int(a[0],0) if op in ('call','jmp') else (self.pair(30)+((self.mem[0x5c]<<16) if op.startswith('ei') else 0))*2)
                if op in ('call','rcall','icall','eicall'):self.calls.append((self.img.labels.get(self.pc,hex(self.pc)),self.cy))
            elif op in ('ret','reti'):
                self.cy+=4;self.pc=self.popreturn()
                if op=='reti':self.setflag(7,1)
            elif op=='nop':pass
            else:raise RuntimeError(('unsupported',hex(pc),op,a,comment,self.calls[-10:]))
        else:raise RuntimeError(('instruction limit',hex(self.pc),self.calls[-10:]))
        assert self.sp==0x21ff and not self.return_stack
        return int.from_bytes(self.r[22:26],'little')


def setup(img,owner,delta,rate,n,phase,advance,ratio,zcor,tool=0,negative=False,remaining=80,scale=0):
    c=CPU(img);b=img.board;new=img.new;primary=max(range(4),key=lambda i:delta[i]);psteps=delta[primary]
    # GPIO direction and released switches. PIN reads reflect output latch on driven pins.
    for a in (0x20,0x26,0x2c,0x2f,0x100,0x109):c.mem[a]=255
    c.mem[0x30]=0xcb;c.mem[0x31]=0x80 if not negative else 0x02
    c.mem[0x10a]=0x0a;c.mem[0x10b]=0x02 if not negative else 0
    c.mem[0x21]=0x50;c.mem[0x22]=0;c.mem[0x27]=0x0a;c.mem[0x28]=0
    # The nominal/motor coordinates are safely within travel bounds.
    for i in range(4):c.symput('Printer::xyMotion',1000,4,i*4)
    for i in range(2):c.symput('Printer::maxSoftEndstopSteps_',100000,4,4*i)
    for i,val in [(0,1000),(4,1000),(8,0),(12,zcor),(16,0),(32,100000)]:c.symput('Printer::zMotion',val,4,i)
    c.symput('Printer::zMotion',1,1,41)
    c.symput('Printer::blockAll',0)
    # Keep output ownership and phase stable, no nested interrupts.
    c.symput('Printer::extrusionOutput',int(advance),1,8);c.symput('Printer::extrusionOutput',15,1,5)
    c.symput('Printer::extrusionOutput',255 if negative else 1,1,6)
    c.symput('Printer::extrusionOutput',0,1,9)
    c.symput('Printer::extrusionCompensation',struct.unpack('<I',struct.pack('<f',ratio))[0],4)
    size=img.sizes['extruder']//(2 if b in ('RF2000','RF2000v2') else 1)
    c.symput('Extruder::current',img.addr('extruder')+size*tool,2)
    c.put(img.addr('extruder')+size*tool,tool)
    c.symput('StepExecutor::stepsPerTimerCall',n);c.symput('StepExecutor::stepIntervalTicks',16000000//rate,4)
    c.symput('StepExecutor::packingInterval_',3300,2)
    c.symput('StepExecutor::directPrimarySteps' if not owner else 'StepExecutor::queuedPrimarySteps',psteps,4)
    c.symput('StepExecutor::directionOwner',1 if owner else 255)
    c.symput('StepExecutor::stepClocks',0 if phase!='decel' else 20000,4,10*owner)
    c.symput('StepExecutor::stepClocks',20 if phase=='decel' else 0,4,10*owner+4)
    c.symput('StepExecutor::stepClocks',rate,2,10*owner+8)
    move=img.addr('PrintLine::queue' if owner else 'PrintLine::direct');size=103 if new else 119
    c.mem[move:move+size]=bytes(size);c.put(move,1);c.put(move+1,2 if phase=='cruise' else (4 if phase=='decel' else 0));c.put(move+2,primary);c.put(move+3,100000,4)
    mask=sum((16+(0 if negative else 1))<<i for i,d in enumerate(delta) if d);c.put(move+7,mask)
    for i,d in enumerate(delta):c.put(move+8+i*4,d,4)
    errors=img.addr('StepExecutor::stepErrors')+16*owner if new else move+24
    for i in range(4):c.put(errors+i*4,psteps//2 if i<3 else 0,4)
    o=0 if new else 16
    c.put(move+24+o,struct.unpack('<I',struct.pack('<f',30.0))[0],4)
    c.put(move+76+o,40 if phase=='accel' else 0,4);c.put(move+80+o,remaining+5 if phase=='decel' else 0,4);c.put(move+84+o,2500,4)
    c.put(move+88+o,rate,2);c.put(move+90+o,rate//2 if phase=='accel' else rate,2);c.put(move+92+o,rate//2,2);c.put(move+94+o,32 if advance else 0,4);c.put(move+98+o,remaining,4);c.put(move+102+o,0 if owner else 6)
    if owner:
        q=img.addr('PrintLine::queue');m=q+24*size;c.put(m,move,2);c.put(m+2,0);c.put(m+3,1);c.put(m+4,1)
    # Old generic function receives owner in r22. New direct clone is constant address.
    c.setpair(24,move);c.r[22]=owner
    if new:pc=img.find('StepExecutor::performMove<(unsigned char)'+str(owner)+'>')
    else:pc=img.symbols['StepExecutor::performMove(PrintLine*, unsigned char)']
    return c,pc,move,errors,o


def trace(img,**kw):
    c,pc,move,errors,o=setup(img,**kw);val=c.run(pc)
    # Compare logical execution state and ordered physical pin transitions (not their timestamps).
    result=(val,c.get(move+98+o,4),c.get(move+1),c.get(move+7),c.mem[errors:errors+16],
            bytes(c.mem[img.addr('Printer::xyMotion'):img.addr('Printer::xyMotion')+36]),
            bytes(c.mem[img.addr('Printer::zMotion'):img.addr('Printer::zMotion')+42]),
            bytes(c.mem[img.addr('Printer::extrusionCompensation'):img.addr('Printer::extrusionCompensation')+8]),
            bytes(c.mem[img.addr('Printer::extrusionOutput'):img.addr('Printer::extrusionOutput')+10]),
            c.symget('StepExecutor::stepsPerTimerCall'),c.symget('StepExecutor::stepIntervalTicks',4),
            bytes(c.mem[img.addr('StepExecutor::stepClocks'):img.addr('StepExecutor::stepClocks')+20]),
            [(p,s) for p,s,_ in c.edges])
    return result,c

if __name__=='__main__':
    b=sys.argv[3] if len(sys.argv)>3 else 'RF2000v2';old=Image(sys.argv[1],b);new=Image(sys.argv[2],b)
    for owner in (0,1):
        for delta in [(100,0,0,0),(100,70,0,0),(100,70,0,30),(0,0,0,100),(0,0,100,0),(100,70,20,30)]:
            kw=dict(owner=owner,delta=delta,rate=8000,n=2,phase='cruise',advance=False,ratio=0,zcor=0)
            x,c1=trace(old,**kw);y,c2=trace(new,**kw)
            print(b,owner,delta,c1.cy,c2.cy,'edges',x[-1]);assert x==y,[(i,a,b) for i,(a,b) in enumerate(zip(x,y)) if a!=b]

import itertools,json,sys,collections
from linked_motion import Image,CPU,trace,setup

def reference(d):
    if d<512:return 16000000//max(10,d)
    width=32 if d<8192 else 4096;base=d-d%width;a=16000000//base;b=16000000//(base+width)
    return a-(a-b)*(d-base)//width

def run(board,old_path,new_path):
    old=Image(old_path,board);new=Image(new_path,board)
    vectors={'X':(100,0,0,0),'XY':(100,70,0,0),'XYE':(100,70,0,30),'denseXYE':(100,70,0,100),'E':(0,0,0,100),'Z':(0,0,100,0),'XYZE':(100,70,100,30)}
    tools=range(2 if board in ('RF2000','RF2000v2') else 1)
    results={'board':board,'n':0,'ranges':{},'max_irq':[0,0],'max_stack':[0,0],'min_high':[{},{}],'min_low':[{},{}],'rates':{},'representative':{},'completions':0,'period_values':0}
    def summarize(name,kw):
        x,a=trace(old,**kw);y,b=trace(new,**kw)
        if x!=y:raise AssertionError((board,kw,[(i,aa,bb) for i,(aa,bb) in enumerate(zip(x,y)) if aa!=bb]))
        # Independent check of step count in uncorrected, unobstructed simple fixtures.
        if not kw['advance'] and kw['ratio']==0 and kw['zcor']==0:
            n=min(kw['n'],kw.get('remaining',80));primary=max(kw['delta'])
            expected=[max(0,(n*d-(primary//2 if i<3 else 0)+primary-1)//primary) for i,d in enumerate(kw['delta'])]
            pins=['X','Y','Z','E'+str(kw['tool'])];counts=collections.Counter(p for p,state in y[-1] if state==1)
            assert all(counts[p]==e for p,e in zip(pins,expected)),(board,kw,counts,expected)
        results['n']+=1
        r=results['ranges'].setdefault(name,{'saved_min':100000,'saved_max':-100000,'cycles_old_min':100000,'cycles_new_min':100000,'cycles_old_max':0,'cycles_new_max':0,'n':0})
        saved=a.cy-b.cy;r['saved_min']=min(r['saved_min'],saved);r['saved_max']=max(r['saved_max'],saved);r['n']+=1
        for i,cpu in enumerate((a,b)):
            key=('old','new')[i];r['cycles_'+key+'_min']=min(r['cycles_'+key+'_min'],cpu.cy);r['cycles_'+key+'_max']=max(r['cycles_'+key+'_max'],cpu.cy)
            results['max_irq'][i]=max(results['max_irq'][i],cpu.maxirq, cpu.cy-cpu.irqstart if not cpu.flag(7) else 0)
            results['max_stack'][i]=max(results['max_stack'][i],0x21ff-cpu.minsp)
            last={}
            for p,s,t in cpu.edges:
                if p in last:
                    prev,pt=last[p];assert prev!=s
                    k='min_high' if prev else 'min_low';results[k][i][p]=min(results[k][i].get(p,100000),t-pt)
                last[p]=(s,t)
            assert all(s==0 for s,t in last.values())
        if kw['phase']=='cruise' and kw['rate']==8000 and kw['n']==2 and not kw['advance'] and not kw['ratio'] and not kw['zcor'] and not kw['tool'] and not kw['negative'] and kw.get('remaining',80)==80:
            results['representative'][str(kw['owner'])+'-'+name]=[a.cy,b.cy]
        if kw.get('remaining',80)<kw['n']+1:results['completions']+=1
        if kw['phase']=='cruise' and kw.get('remaining',80)==80 and kw['n']==(3300+max(512,reference(kw['rate']))-1)//max(512,reference(kw['rate'])):
            period=max(512,reference(kw['rate']))*kw['n'];r=results['rates'].setdefault(str(kw['rate']),[0,0]);r[0]=max(r[0],a.cy/period);r[1]=max(r[1],b.cy/period)
    for name,delta in vectors.items():
        for owner,phase,tool,neg,adv,ratio,zcor in itertools.product((0,1),('cruise','accel','decel'),tools,(False,True),(False,True),(0,.25),(0,4)):
            if ratio and (not owner or not delta[3]):continue
            if adv and not delta[3]:continue
            if zcor and not (delta[0] or delta[1] or delta[2]):continue
            summarize(name,dict(owner=owner,delta=delta,rate=8000,n=2,phase=phase,advance=adv,ratio=ratio,zcor=zcor,tool=tool,negative=neg))
        for owner,rate,adv,ratio,zcor in itertools.product((0,1),(4000,8000,16000,24000,31250),(False,True),(0,.25),(0,4)):
            if ratio and (not owner or not delta[3]):continue
            if adv and not delta[3]:continue
            if zcor and not (delta[0] or delta[1] or delta[2]):continue
            interval=max(512,reference(rate));n=(3300+interval-1)//interval
            summarize(name,dict(owner=owner,delta=delta,rate=rate,n=n,phase='cruise',advance=adv,ratio=ratio,zcor=zcor,tool=0,negative=False))
    for name,delta in vectors.items():
        for owner,adv,negative in itertools.product((0,1),(False,True),(False,True)):
            if adv and not delta[3]:continue
            summarize(name,dict(owner=owner,delta=delta,rate=8000,n=2,phase='cruise',advance=adv,ratio=0,zcor=0,tool=0,negative=negative,remaining=2))
    # Read the actual linked flash tables, not source initializers. Recheck all 65536 inputs.
    def table(name,index,sz):
        a=new.symbols['(anonymous namespace)::'+name]+sz*index
        return int.from_bytes(new.flash[a:a+sz],'little')
    for d in range(65536):
        if d<512:v=table('lowPeriods',d,4)
        elif d<8192:v=table('midPeriods',d-512,2)
        else:
            k,w=('high4Periods',4) if d<12288 else ('high8Periods',8) if d<16384 else ('high16Periods',16)
            lo=8192 if w==4 else 12288 if w==8 else 16384
            p=table(k,(d-lo)//w,2);v=(p&4095)+int(d%w < p>>12)
        assert v==reference(d),(board,d,v,reference(d));results['period_values']+=1
    return results

if __name__=='__main__':print(json.dumps(run(*sys.argv[1:]),sort_keys=True))

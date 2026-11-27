// Run: node tools/task0033/priority-model.js > tools/task0033/priority-model-results.json
function priorityResearch(){
  const F=16000000,assert=(x,m)=>{if(!x)throw Error(m)};
  // Simplified fixed-priority, non-nested interrupt service with one pending
  // flag per timer. Times are integer CPU cycles. RX FIFO data queues, UDRE
  // synthetic offered byte work queues; these are distinct from timer flags.
  function simulate({baud,profile,phase,frames=400,txRatio=.25}){
    const p=F*10/baud;
    const sources=[
      {name:"WDT",priority:12,period:256000,cost:profile.wdt,phase:phase},
      {name:"T1masked",priority:17,period:profile.t1Period,cost:profile.t1,phase:phase},
      {name:"Advance",priority:21,period:profile.advancePeriod,cost:profile.advance,phase:(phase*3)%profile.advancePeriod},
      {name:"PWM_ADC",priority:22,period:4096,cost:profile.pwm,phase:(phase*5)%4096},
      {name:"Timer0OVF",priority:23,period:16384,cost:profile.ovf,phase:(phase*7)%16384}
    ].map(s=>({...s,next:s.phase,pending:null,releases:0,served:0,coalesced:0,maxStart:0,cycles:0,maxGap:0,lastStart:null}));
    const horizon=Math.ceil(frames*p),fifo=[];
    let now=0,end=0,readAt=Infinity,rxNext=Math.ceil(p),rxOffered=0,rxRead=0,hwLoss=0,maxRead=0,maxFifo=0,rxCycles=0,txCycles=0;
    let txNext=0,txOffered=0,txDone=0,txPending=0,maxTxPending=0,idleCycles=0;
    const txLimit=Math.floor(frames*txRatio);
    const txP=txRatio?p/txRatio:Infinity;
    while(now<=horizon+10000){
      if(now===rxNext){
        rxOffered++;
        if(fifo.length===2)hwLoss++;else fifo.push(now);
        maxFifo=Math.max(maxFifo,fifo.length);
        rxNext=rxOffered<frames?Math.ceil((rxOffered+1)*p):Infinity;
      }
      if(now===txNext){
        if(txOffered<txLimit){txOffered++;txPending++;maxTxPending=Math.max(maxTxPending,txPending)}
        txNext=txOffered<txLimit?Math.ceil(txOffered*txP):Infinity;
      }
      for(const s of sources)if(now===s.next){
        s.releases++;
        if(s.pending!==null)s.coalesced++;else s.pending=now;
        s.next=now+s.period;
        if(s.next>horizon)s.next=Infinity;
      }
      if(now===readAt){
        assert(fifo.length,"UDR without data");
        maxRead=Math.max(maxRead,now-fifo.shift());rxRead++;readAt=Infinity;
      }
      if(now>=end){
        const s=sources.find(s=>s.pending!==null);
        if(s){
          s.maxStart=Math.max(s.maxStart,now-s.pending);s.pending=null;
          s.served++;s.cycles+=s.cost;end=now+s.cost+1;
          if(s.lastStart!==null)s.maxGap=Math.max(s.maxGap,now-s.lastStart);
          s.lastStart=now;
        }else if(fifo.length){readAt=now+31;end=now+74;rxCycles+=73}
        else if(txPending){txPending--;txDone++;end=now+75;txCycles+=74}
      }
      if(now>=horizon&&rxOffered===frames&&fifo.length===0&&readAt===Infinity&&txPending===0&&sources.every(s=>s.pending===null))break;
      const next=Math.min(rxNext,txNext,readAt,end>now?end:Infinity,...sources.map(s=>s.next));
      if(!Number.isFinite(next))break;
      assert(next>now,"progress");
      if(now>=end)idleCycles+=next-now;
      now=next;
    }
    assert(rxOffered===rxRead+hwLoss+fifo.length,"RX accounting");
    assert(txOffered===txDone+txPending,"TX accounting");
    for(const s of sources)assert(s.releases===s.served+s.coalesced+(s.pending!==null?1:0),"timer accounting "+s.name);
    return {phase,rxOffered,rxRead,hwLoss,maxRead,maxFifo,txOffered,txDone,maxTxPending,txPending,rxCycles,txCycles,idleCycles,elapsedCycles:now,timers:sources.map(({name,releases,served,coalesced,maxStart,maxGap,cycles})=>({name,releases,served,coalesced,maxStart,maxGap,cycles}))};
  }
  const profiles=[
    {name:"light",t1Period:1600,t1:160,advancePeriod:2400,advance:100,pwm:180,ovf:100,wdt:80},
    {name:"medium",t1Period:800,t1:320,advancePeriod:1200,advance:120,pwm:280,ovf:120,wdt:100},
    {name:"heavy_masked",t1Period:800,t1:594,advancePeriod:1200,advance:120,pwm:300,ovf:120,wdt:100}
  ];
  const results=[];
  for(const baud of [115200,250000,500000,1000000,2000000])
    for(const profile of profiles)
      for(const txRatio of [0,.25,1]){
        const rows=[];
        for(let phase=0;phase<Math.ceil(F*10/baud);phase+=Math.max(1,Math.floor(F*10/baud/32))){
          rows.push(simulate({baud,profile,phase,txRatio}));
        }
        const worst=rows.reduce((a,b)=>b.hwLoss>a.hwLoss?b:a);
        results.push({baud,profile:profile.name,txRatio,phases:rows.length,lossPhases:rows.filter(r=>r.hwLoss).length,worstHwLoss:worst.hwLoss,worstPhase:worst.phase,maxRxReadCycles:Math.max(...rows.map(r=>r.maxRead)),maxTxPending:Math.max(...rows.map(r=>r.maxTxPending)),maxTimerCoalesced:Object.fromEntries(worst.timers.map(t=>[t.name,Math.max(...rows.map(r=>r.timers.find(x=>x.name===t.name).coalesced))])),maxTimerStartDelay:Object.fromEntries(worst.timers.map(t=>[t.name,Math.max(...rows.map(r=>r.timers.find(x=>x.name===t.name).maxStart))])),worstTrace:worst});
      }
  assert(results.some(x=>x.worstHwLoss>0),"stress detects loss");
  return {model:"task0033-priority-v1",assumptions:"Illustrative non-nested fixed vector priority, one pending flag per timer, finite 400 byte stream. Timer0 COMPB nominal period 4096 cycles, Timer0 OVF 16384 assuming Arduino core /64, WDT nominal 16ms; T1 and Advance periods and all non-UART costs synthetic sensitivity parameters, NOT linked path measurements. ADC polled inside PWM on audited baseline, not a separate vector. Does not emulate Timer1 safe nesting, actual compare reschedule, STEP edges, watchdog oscillator, UDR shift pipeline at sub-frame granularity, TX empty interrupt, stack or parser; no software ring in this model. RX FIFO two entries and full at next start conservative. TX offered work queue is synthetic and can grow without limit; not physical TX ring. Loss phases are deterministic sampled offsets, not probabilities. Timer costs include synthetic entry/return, add 1 cycle dispatch deferral. No safety claims.",profiles,results};
}
console.log(JSON.stringify(priorityResearch(), null, 2));

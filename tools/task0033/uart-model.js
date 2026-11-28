// Run: node tools/task0033/uart-model.js > tools/task0033/uart-model-results.json
// Research only; no firmware or hardware access.
function uartResearch(){
  const F=16000000,RX=73,READ=31,TX=74;
  const assert=(x,m)=>{if(!x)throw Error(m)};
  const round=x=>Math.round(x*10000)/10000;
  const rates=[115200,250000,500000,1000000,2000000].map(baud=>{
    const mode=k=>{const ubrr=Math.max(0,Math.round(F/(k*baud)-1));const actual=F/(k*(ubrr+1));return {ubrr,actual:round(actual),error_pct:round(100*(actual/baud-1))}};
    const p=F*10/baud;
    return {baud,normal:mode(16),u2x:mode(8),frame_cycles:round(p),frame_us:round(1e7/baud),rx_pct:round(100*RX/p),tx_pct:round(100*TX/p),full_duplex_pct:round(100*(RX+TX)/p),ring_empty_to_full_us:round(127*1e7/baud)};
  });
  // Cycle-grid, nonpreemptible IRQ windows. Frame completion ties precede UDR reads:
  // back-to-back sender, next start immediate. Full FIFO on completion loses
  // completed shift-register byte (conservative boundary); no attempt to predict DOR flag placement.
  // RX release at stop-bit/frame completion approximation, not an electrical model.
  function simulate({baud,phase=0,frames=80,blackouts=[],drain=0,tx=false,initialRing=0}){
    const p=F*10/baud, fifo=[],ring={used:initialRing};
    let sent=0,hw=0,sw=0,good=0,consumed=0,maxFifo=0,maxRing=initialRing,maxRead=0,rxIrqs=0,txIrqs=0;
    let end=0,readAt=Infinity,txDue=phase,drainDue=drain||Infinity,t=0;
    let arrival=phase+Math.ceil(p),i=0;
    const horizon=phase+Math.ceil(frames*p)+Math.max(10000,...blackouts.map(b=>b[1]));
    while(t<=horizon){
      // A completed interrupt cannot begin another IRQ in the same model cycle;
      // one instruction deferral included via end=start+cost+1.
      if(t===arrival){
        sent++;
        if(fifo.length===2)hw++;else fifo.push(t);
        maxFifo=Math.max(maxFifo,fifo.length);
        arrival=sent<frames?phase+Math.ceil((sent+1)*p):Infinity;
      }
      if(t===readAt){
        assert(fifo.length>0,"read empty FIFO");const released=fifo.shift();
        maxRead=Math.max(maxRead,t-released);readAt=Infinity;
        if(ring.used===127)sw++;else{ring.used++;good++;maxRing=Math.max(maxRing,ring.used)}
      }
      if(t===drainDue){
        // A foreground *opportunity*, not actual parser execution. Only if CPU idle.
        if(t>=end&&!blackouts.some(([a,b])=>a<=t&&t<b)){consumed+=ring.used;ring.used=0;}
        drainDue+=drain;
      }
      if(t>=end){
        while(i<blackouts.length&&blackouts[i][1]<=t)i++;
        const blocked=i<blackouts.length&&blackouts[i][0]<=t;
        if(!blocked){
          if(fifo.length){rxIrqs++;readAt=t+READ;end=t+RX+1;}
          else if(tx&&t>=txDue&&txIrqs<frames){txIrqs++;end=t+TX+1;txDue=phase+Math.ceil(txIrqs*p)}
        }
      }
      if(sent===frames&&!fifo.length&&readAt===Infinity&&(!tx||txIrqs===frames))break;
      // event jump; if blocked defer dispatch until window ends. Overlapping
      // windows must be merged by caller; modeled blackouts never preempt RX.
      const blocked=i<blackouts.length&&blackouts[i][0]<=t&&t<blackouts[i][1];
      let next=Math.min(arrival,readAt,drainDue,end>t?end:Infinity,blocked?blackouts[i][1]:Infinity,
        tx&&txIrqs<frames&&txDue>t?txDue:Infinity,i<blackouts.length&&blackouts[i][0]>t?blackouts[i][0]:Infinity);
      if(!Number.isFinite(next))break;
      assert(next>t,"time progress");t=next;
    }
    assert(sent===frames,"all frames offered");assert(sent===hw+sw+good+fifo.length,"byte accounting");
    return {sent,hw,sw,good,consumed,ring_remaining:ring.used,maxFifo,maxRing,maxRead,rxIrqs,txIrqs,pending:fifo.length};
  }
  assert(simulate({baud:500000,frames:128}).sw===1,"128th byte lost at ring full");
  assert(simulate({baud:500000,frames:128,drain:2000}).sw===0,"ring serviced");
  assert(simulate({baud:500000,frames:20,blackouts:[[0,1000]]}).hw>0,"FIFO loss");
  assert(simulate({baud:500000,frames:20}).hw===0,"clean RX");
  assert(simulate({baud:500000,frames:20,tx:true}).hw===0,"clean duplex");
  assert(simulate({baud:2000000,frames:80,tx:true}).hw===0,"RX priority with TX backlog");
  const sweep=[];
  for(const baud of rates.map(x=>x.baud)){
    const p=Math.ceil(F*10/baud);
    for(const duration of [160,320,480,594,640,800,960,1280]){
      let loss=0,worstHw=0,worstRead=0;
      // vary blackout start relative to continuous byte stream, prehistory:
      // t=4 frame periods + phase; model includes previous serviced arrivals.
      for(let offset=0;offset<p;offset++){
        const start=4*p+offset;
        const r=simulate({baud,frames:20,blackouts:[[start,start+duration]],tx:true});
        if(r.hw)loss++;worstHw=Math.max(worstHw,r.hw);worstRead=Math.max(worstRead,r.maxRead);
      }
      sweep.push({baud,blackout_cycles:duration,phases:p,loss_phases:loss,worst_hw_loss:worstHw,worst_read_cycles:worstRead});
    }
  }
  // Same workload, not line saturation; representative 6-byte ACK per command,
  // synthetic wire sizes, no parser, no extra traffic or empty UDRE cost.
  const fixed=[];
  for(const commands_per_s of [100,250,500,1000,1500]){
    for(const bytes of [19,40]){
      const rxB=commands_per_s*bytes,txB=commands_per_s*6;
      fixed.push({commands_per_s,rx_bytes_per_command:bytes,tx_bytes_per_command:6,rx_Bps:rxB,tx_Bps:txB,uart_cpu_pct:round(100*(rxB*RX+txB*TX)/F),feasible_rates:rates.filter(r=>rxB<=r.baud/10&&txB<=r.baud/10).map(r=>r.baud)});
    }
  }
  return {model:"task0033-uart-v1",baseline:"3676b93a28a7cb4dfad070f25676a60567b4a501",historical_cost_cycles:{rx_success:RX,udr_read:READ,tx_nonempty:TX,archive_sha256:"1ffbe5f295d08ff4c8306900080dbcda873fd839f1f9663cdd589cdadf05d151"},limits:"Synthetic cycle model; costs historical Task0032, not freshly linked. Blackouts are injected nonpreemptible CPU unavailability intervals, not ISR WCET; a window starting in an active UART ISR cannot preempt it. No nested ISR, instruction exact phase, hardware sampling, parser, TX empty or timer coalescing model. TX stress synthetic pending bytes, no TX loss metric.",rates,sweep,fixed};
}
console.log(JSON.stringify(uartResearch(), null, 2));

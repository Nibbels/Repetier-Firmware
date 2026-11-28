// Task0034 healthy scheduling sensitivity, not firmware WCET.
function study(){
 // Abstract timing in 16us Timer3 ticks, not integrated CPU scheduling.
 const wrap=65536,batch=512,period=6256,ageLimit=6250;
 const horizon=period*120;
 let results=[];
 for(const channels of [2,3,4]){
  for(const delay of [0,16,64,128,256]){
   let maxGap=0,skipped=0,attempts=0,affected=0,over200=0,over250=0,over300=0;
   for(let phase=0;phase<period;phase+=64){
    for(let c=0;c<channels-+(channels===4);c++){
     let last=null,local=0;
     const cycle=batch*channels;
     for(let t=phase+period;t<horizon;t+=period){
      const end=t-((t-(c+1)*batch)%cycle+cycle)%cycle;
      const start=end-batch,commit=t+delay;
      // Conservative wrap invalidation, publication after read supersedes token.
      const valid=start>=0&&Math.floor(start/wrap)===Math.floor(commit/wrap)
       &&commit-start<=ageLimit&&commit<end+cycle;
      attempts++;
      if(!valid){skipped++;continue;}
      if(last!==null){
       const gap=commit-last;maxGap=Math.max(maxGap,gap);local=Math.max(local,gap);
       if(gap>=12500-4)over200++;
       if(gap>=15625-4)over250++;
       if(gap>=18750-4)over300++;
      }
      last=commit;
     }
     if(local>=12500-4)affected++;
    }
   }
   results.push({channels,delay_us:delay*16,attempts,skipped,max_gap_ms:maxGap*.016,affected_phase_channels:affected,gaps_exceed_200ms:over200,gaps_exceed_250ms:over250,gaps_exceed_300ms:over300});
  }
 }
 return {results,assumptions:"Nominal 256us ADC service, 32 conversions per batch, 100.096ms regulator period, 16us clock ticks; sampled phase, fixed compute delay; no jitter, overload, UART, WDT, or physical cutoff. 4 ADC channels modeled with 3 heaters."};
}
console.log(JSON.stringify(study(),null,2));

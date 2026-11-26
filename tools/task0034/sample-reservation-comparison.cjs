// Task0034 phase comparison of strict, retained and hypothetical retry policies.
function compare(){
 const wrap=65536,batch=512,period=6256,ageLimit=6250,horizon=period*120;
 const out=[];
 for(const channels of [2,3,4])for(const delay of [0,16,64,128,256])for(const policy of ["strict","reserved","reserved-retry"]){
  let attempts=0,firstSkips=0,missed=0,retries=0,maxGap=0,over200=0,minGap=Infinity;
  for(let phase=0;phase<period;phase+=64)for(let c=0;c<Math.min(channels,3);c++){
   let last=null;
   function sampleAt(t){
    const cycle=batch*channels;
    const end=t-((t-(c+1)*batch)%cycle+cycle)%cycle,start=end-batch,commit=t+delay;
    if(start<0||Math.floor(start/wrap)!==Math.floor(commit/wrap)||commit-start>ageLimit)return null;
    if(policy==="strict"&&commit>=end+cycle)return null;
    // At most one authorization from each batch. The fixed 100ms
    // interval and one bounded retry ensure distinct captured batches.
    return {commit,start};
   }
   let lastBatch=null;
   for(let t=phase+period;t<horizon;t+=period){
    attempts++;let a=sampleAt(t);
    if(!a){firstSkips++;if(policy==="reserved-retry"){retries++;a=sampleAt(t+batch*channels+batch);}}
    if(!a||a.start===lastBatch){missed++;continue;}
    lastBatch=a.start;
    if(last!==null){const g=a.commit-last;maxGap=Math.max(maxGap,g);minGap=Math.min(minGap,g);if(g>=12500-4)over200++;}
    last=a.commit;
   }
  }
  out.push({channels,decisionUs:delay*16,policy,attempts,firstSkips,missed,retries,maxGapMs:maxGap*.016,minGapMs:minGap*.016,gapsAtOrBeyond200msGuard:over200});
 }
 return {out,assumptions:"Sampled deterministic nominal phases; reserved retains exact batch across newer plausible publications, but not wrap. Retry is hypothetical complete fresh thermal decision after one ADC scan plus one batch, not implemented. No CPU jitter, rail, fault, physical cutoff or WCET proof."};
}
console.log(JSON.stringify(compare(),null,2));

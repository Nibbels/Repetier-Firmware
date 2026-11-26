// Task0034 research: continuity witnessed by a live sticky compare lease.
function model(){
 const W=65536,D=12500,G=4,A=6250;
 let witnessCases=0,unsafe=0,proven=0,ambiguous=0,ageCases=0,ageErrors=0;
 // Actual deadline is armed g ticks after latest synchronization.
 // Sticky OCF is set one tick after equality; guard rejects equality.
 for(let phase=0;phase<W;phase++)for(const armGap of [0,1,4,64])for(const age of [0,1,D-G-1,D-G,D,D+1,W-1,W,W+1,2*W+1]){
   const grant=phase+armGap,deadline=grant+D,now=grant+age;
   const pending=now>deadline,remaining=(deadline-now)&65535;
   const live=!pending&&remaining>G&&remaining<=D;
   if(live){proven++;if(now-phase>=W)unsafe++;}else ambiguous++;
   witnessCases++;
 }
 // With a proven single wrap, a record tagged as previous epoch
 // must reject now>=stamp even if uint16 modular age is small.
 for(let stamp=0;stamp<W;stamp++)for(const age of [0,1,A-1,A,A+1,W-1,W,W+1,2*W-1,2*W,2*W+1]){
   const absolute=stamp+age,wraps=Math.floor(absolute/W),now=absolute%W;
   const accepted=wraps<2&&(wraps===0||now<stamp)&&((now-stamp)&65535)<=A;
   if(accepted!==(age<=A))ageErrors++;
   ageCases++;
 }
 // A synthetic healthy stream: all three heaters continuously authorized,
 // prior grant every nominal period, ADC batches 512 ticks, retained
 // reservations. Prior live lease witnesses continuity at ordinary wrap.
 const rows=[];
 for(const channels of [2,3,4])for(const delay of [0,16,64,128,256]){
   let missed=0,maxGap=0,attempts=0;
   for(let phase=0;phase<6256;phase+=64)for(let c=0;c<Math.min(channels,3);c++){
     let last=null;
     for(let t=phase+6256;t<6256*120;t+=6256){
       const cycle=512*channels,end=t-((t-(c+1)*512)%cycle+cycle)%cycle,start=end-512,commit=t+delay;
       const wrap=Math.floor(start/W)!==Math.floor(commit/W);
       const continuous=last!==null&&commit-last<D-G;
       const good=start>=0&&commit-start<=A&&(!wrap||continuous);
       attempts++;
       if(!good){missed++;continue;}
       if(last!==null)maxGap=Math.max(maxGap,commit-last);
       last=commit;
     }
   }
   rows.push({channels,decisionUs:delay*16,attempts,missed,maxGapMs:maxGap*.016});
 }
 if(unsafe||ageErrors)throw Error("counterexample");
 return {witnessCases,proven,ambiguous,unsafe,ageCases,ageErrors,rows,
 assumptions:"Abstract sticky compare model; grant occurs after synchronization within bounded armGap, compare not cleared while active, clock running. One-wrap age tags invalidated on second observed wrap; without witness invalidate all. Healthy model assumes continuously powered channel, omits cold admission near wrap, other fault checks, actual ADC/IRQ scheduling and WDT. Not linked firmware evidence."};
}
console.log(JSON.stringify(model(),null,2));

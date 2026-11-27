// Task0032 S2a: isolated RX blackout phase sensitivity, conservative FIFO screen.
// Continuous 8N1 frames, two completed-byte FIFO entries. Full at arrival means
// modeled loss (next start follows immediately); NOT exact AVR DOR flag semantics.
// No competing IRQs after blackout, no ring/consumer/FE/clock error model.
"use strict";
const CPU=16000000,RX=73,READ=31;
function simulate({baud,blackout,phase,initial=0,frames=12}){
  const period=CPU*10/baud;
  if(!(baud>0&&blackout>=0&&phase>=0&&phase<period&&initial>=0&&initial<=2&&Number.isInteger(initial)&&frames>=1&&Number.isInteger(frames)))throw Error("bad phase fixture");
  let t=0,arrivals=0,lost=0,read=0,maxLatency=0,maxOccupancy=initial;
  let fifo=Array(initial).fill(null),readAt=Infinity,finishAt=Infinity;
  function dispatch(){if(t>=blackout&&finishAt===Infinity&&fifo.length){readAt=t+READ;finishAt=t+RX;}}
  dispatch();
  while(arrivals<frames||fifo.length||finishAt!==Infinity){
    const arrival=arrivals<frames?phase+arrivals*period:Infinity;
    t=Math.min(arrival,readAt,finishAt,t<blackout?blackout:Infinity);
    if(!Number.isFinite(t))throw Error("deadlock");
    // Conservative tie: arrival before UDR read.
    if(arrival===t){arrivals++;if(fifo.length===2)lost++;else fifo.push(t);maxOccupancy=Math.max(maxOccupancy,fifo.length);}
    if(readAt===t){const a=fifo.shift();if(a!==null){read++;maxLatency=Math.max(maxLatency,t-a);}readAt=Infinity;}
    if(finishAt===t)finishAt=Infinity;
    dispatch();
  }
  if(read+lost!==frames)throw Error("byte accounting");
  return {lost,read,maxLatencyCycles:maxLatency,maxOccupancy};
}
function sweep(baud,blackout,initial){
  const period=CPU*10/baud;
  let phases=0,lossPhases=0,worstLoss=0,maxLatency=0,firstLossPhase=null;
  for(let phase=0;phase<period;phase++){
    const r=simulate({baud,blackout,phase,initial});phases++;if(r.lost){lossPhases++;if(firstLossPhase===null)firstLossPhase=phase;}
    worstLoss=Math.max(worstLoss,r.lost);maxLatency=Math.max(maxLatency,r.maxLatencyCycles);
  }
  return {baud,blackout,initial,phases,lossPhases,worstLoss,maxLatencyCycles:maxLatency,firstLossPhase};
}
function selfTest(){
  const check=(v,m)=>{if(!v)throw Error(m)};
  check(simulate({baud:500000,blackout:0,phase:0}).maxLatencyCycles===31,"read anchor");
  check(sweep(500000,594,0).lossPhases===0,"isolated empty FIFO 594 cycles");
  check(sweep(500000,594,1).lossPhases>0,"prior byte matters");
  check(sweep(1000000,594,0).lossPhases>0,"higher baud phase loss");
  check(simulate({baud:500000,blackout:609,phase:0}).lost===1,"tie arrival before read");
  return "5 groups of checks passed";
}
function matrix(){
  const rows=[];
  for(const baud of [115200,250000,500000,1000000,2000000])for(const blackout of [0,160,320,594,800,1200])for(const initial of [0,1,2])
    rows.push(sweep(baud,blackout,initial));
  return rows;
}
if(require.main===module){
  if(process.argv[2]==="--self-test"){console.log(selfTest());process.exit(0);}
  console.log(JSON.stringify({schema:1,model:"isolated-rx-blackout-screen",sourceSha256:require("node:crypto").createHash("sha256").update(require("node:fs").readFileSync(__filename)).digest("hex"),selfTest:selfTest(),rows:matrix()},null,2));
}
module.exports={simulate,sweep,matrix,selfTest};

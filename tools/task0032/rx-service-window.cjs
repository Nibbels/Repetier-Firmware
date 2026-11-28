// Task0032: repeated blocked/open CPU windows and burst RX sensitivity.
// Conservative two completed-byte FIFO; arrival before read at ties.
// Each RX service must fit wholly in open window; no other ISR competitors.
// Synthetic schedule, not exact ATmega DOR or a real firmware timing trace.
"use strict";
const CPU=16000000,RX=73,READ=31,EPS=1e-8;
function simulate({baud,blocked,open,phase,initial=0,frames=64}){
  const byte=CPU*10/baud,period=blocked+open;
  if(!(baud>0&&blocked>=0&&open>=RX&&phase>=0&&phase<byte&&Number.isInteger(initial)&&initial>=0&&initial<=2&&Number.isInteger(frames)&&frames>0))throw Error("bad parameters");
  let time=0,arrivals=0,read=0,lost=0,readAt=Infinity,finish=Infinity,start=Infinity,maxLatency=0,maxOccupancy=initial;
  const fifo=Array(initial).fill(null);
  function nextStart(t){
    if(blocked===0)return t;
    const pos=t%period;
    if(pos<blocked)return t+blocked-pos;
    if(pos+RX<=period)return t;
    return t+period-pos+blocked;
  }
  function dispatch(){if(finish===Infinity&&fifo.length)start=nextStart(time);}
  dispatch();
  while(arrivals<frames||fifo.length||finish!==Infinity){
    const arrival=arrivals<frames?phase+arrivals*byte:Infinity;
    time=Math.min(arrival,start,readAt,finish);
    if(!Number.isFinite(time))throw Error("deadlock");
    if(arrival===time){arrivals++;if(fifo.length===2)lost++;else fifo.push(time);maxOccupancy=Math.max(maxOccupancy,fifo.length);}
    if(readAt===time){const x=fifo.shift();if(x!==null){read++;maxLatency=Math.max(maxLatency,time-x);}readAt=Infinity;}
    if(finish===time)finish=Infinity;
    if(start===time){start=Infinity;readAt=time+READ;finish=time+RX;}
    dispatch();
  }
  if(read+lost!==frames)throw Error("accounting");
  return {lost,read,maxLatencyCycles:maxLatency,maxOccupancy};
}
function sweep(baud,blocked,open,initial,frames=64){
  const byte=CPU*10/baud;let phases=0,lossPhases=0,worstLoss=0,maxLatency=0;
  for(let phase=0;phase<byte;phase++){
    const r=simulate({baud,blocked,open,phase,initial,frames});phases++;lossPhases+=+(r.lost>0);worstLoss=Math.max(worstLoss,r.lost);maxLatency=Math.max(maxLatency,r.maxLatencyCycles);
  }
  return {baud,blocked,open,initial,frames,phases,lossPhases,worstLoss,maxLatencyCycles:maxLatency};
}
function selfTest(){
  const check=(v,m)=>{if(!v)throw Error(m)};
  check(sweep(500000,0,73,0).lossPhases===0,"unblocked");
  check(sweep(500000,594,80,0).lossPhases===320,"one read per long cycle insufficient");
  check(sweep(500000,160,320,0).lossPhases===0,"short windows");
  check(sweep(250000,594,160,0).lossPhases===0,"slower baud control");
  return "4 groups of checks passed";
}
function matrix(){
  const rows=[];
  for(const baud of [250000,500000])for(const blocked of [160,320,594])for(const open of [80,160,320,640])for(const initial of [0,1])for(const frames of [12,64])rows.push(sweep(baud,blocked,open,initial,frames));
  return rows;
}
if(require.main===module){
  if(process.argv[2]==="--self-test")console.log(selfTest());
  else console.log(JSON.stringify({schema:1,model:"repeated-rx-service-window",selfTest:selfTest(),rows:matrix()},null,2));
}
module.exports={simulate,sweep,matrix,selfTest};

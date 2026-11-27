// Task0032: scalar lookahead and admission-governor thought experiment.
// NOT the actual MotionPlanner, UART or CPU simulator. Synthetic service times,
// fixed per-admission caps, immutable executing profile and zero-speed visible tail.
"use strict";
const EPS=1e-8;
function plan(rows,entry,accel){
  if(!rows.length)return [];
  const n=rows.length,b=Array(n+1).fill(0);
  for(let i=n-1;i>=0;i--)
    b[i]=Math.min(rows[i].cap,Math.sqrt(b[i+1]*b[i+1]+2*accel*rows[i].length));
  if(entry>b[0]+EPS)throw Error("frozen entry cannot reach visible tail");
  b[0]=entry;
  for(let i=0;i<n;i++)b[i+1]=Math.min(b[i+1],rows[i].cap,Math.sqrt(b[i]*b[i]+2*accel*rows[i].length));
  return rows.map((r,i)=>{
    const u=b[i],w=b[i+1],v=Math.min(r.cap,Math.sqrt(accel*r.length+(u*u+w*w)/2));
    const a=Math.max(0,(v*v-u*u)/(2*accel)),d=Math.max(0,(v*v-w*w)/(2*accel));
    const c=Math.max(0,r.length-a-d);
    if(v+EPS<Math.max(u,w)||a+d>r.length+EPS)throw Error("infeasible ramp");
    return {...r,entry:u,exit:w,peak:v,seconds:(v-u+v-w)/accel+c/v};
  });
}
function fixture({chord=.1,baud=500000,stallMs=0,request=100,plannerUs=1000}={}){
  const lengths=[...Array(20).fill(5),...Array(500).fill(chord),...Array(20).fill(5)];
  return lengths.map((length,i)=>({length,request,wire:40*10/baud,planner:plannerUs/1e6,
    stall:i===170?stallMs/1000:0,kind:length===5?"long":"dense"}));
}
function simulate(rows,{slots=24,prefill=12,accel=800,policy="predictive",horizon=.1,margin=.9}={}){
  if(!["none","count","supply","predictive"].includes(policy)||slots<prefill||prefill<1||accel<=0||!rows.length)throw Error("bad options");
  let time=0,received=0,done=0,queue=[],active=null,entry=0,nextArrival=null,started=null,starvedAt=null;
  let gapCount=0,gapSeconds=0,maximumGap=0,covered=0,capChanges=0,lastCap=null,minCoverage=Infinity;
  let denseDistance=0,denseSeconds=0,peak=0,firstDenseEntry=null,lastLongExit=null,trace=[];
  const occupied=()=>queue.length+(active?1:0);
  function arm(){
    if(nextArrival===null&&received<rows.length&&occupied()<slots){
      const r=rows[received];nextArrival=time+Math.max(r.wire,r.planner)+r.stall;
    }
  }
  function dispatch(){
    if(active||!queue.length||(started===null&&queue.length<prefill&&received<rows.length))return;
    if(started===null)started=time;
    if(starvedAt!==null){const gap=time-starvedAt;if(gap>EPS){gapCount++;gapSeconds+=gap;maximumGap=Math.max(maximumGap,gap);}starvedAt=null;}
    const p=plan(queue,entry,accel)[0];
    queue.shift();active={...p,end:time+p.seconds};
  }
  arm();
  while(done<rows.length){
    const t=Math.min(nextArrival??Infinity,active?.end??Infinity);
    if(!Number.isFinite(t)||t<time-EPS)throw Error("deadlock");
    const dt=t-time;
    if(started!==null){
      const before=(active?active.end-time:0)+(queue.length?plan(queue,active?.exit??entry,accel).reduce((s,r)=>s+r.seconds,0):0);
      covered+=Math.max(0,before*dt-(active?dt*dt/2:0));
      if(received<rows.length)minCoverage=Math.min(minCoverage,Math.max(0,before-(active?dt:0)));
    }
    time=t;
    if(active&&active.end<=time+EPS){
      const a=active;done++;entry=a.exit;peak=Math.max(peak,a.peak);
      if(a.kind==="dense"){denseDistance+=a.length;denseSeconds+=a.seconds;if(firstDenseEntry===null)firstDenseEntry=a.entry;}
      else if(firstDenseEntry===null)lastLongExit=a.exit;
      trace.push({index:done-1,kind:a.kind,cap:a.cap,entry:a.entry,exit:a.exit,peak:a.peak,seconds:a.seconds,time});
      active=null;
      if(!queue.length&&done<rows.length){if(entry>EPS)throw Error("nonzero speed at starvation");starvedAt=time;}
    }
    if(nextArrival!==null&&nextArrival<=time+EPS){
      const r=rows[received++];nextArrival=null;
      let cap=r.request;
      if(policy==="count"&&occupied()<10){
        const t=r.length/r.request;if(t<.015625)cap=r.length/(t+(.015625-t)*3/(occupied()+1));
      }
      if(policy==="supply"||policy==="predictive"){
        // Oracle for nominal service cost, not a measured runtime estimator.
        // Backlog may finance temporary acceleration, but never above request.
        const sustainable=r.length/Math.max(r.wire,r.planner)*margin;
        const backlog=queue.reduce((s,j)=>s+j.length,0);
        cap=Math.min(r.request,sustainable+(policy==="predictive"?backlog/horizon:0));
      }
      if(lastCap!==null&&Math.abs(lastCap-cap)>20)capChanges++;
      lastCap=cap;queue.push({...r,cap});
      if(occupied()>slots)throw Error("queue overflow");
    }
    dispatch();arm();
  }
  if(Math.abs(entry)>EPS||received!==rows.length)throw Error("tail or accounting");
  const distance=rows.reduce((s,r)=>s+r.length,0),printSeconds=time-started;
  const denseStart=trace.findIndex(r=>r.kind==="dense"),denseEnd=trace.findLastIndex(r=>r.kind==="dense");
  const denseElapsed=denseStart<0?0:trace[denseEnd].time-(denseStart?trace[denseStart-1].time:started);
  return {policy,slots,segments:done,distance,printSeconds,achieved:distance/printSeconds,
    denseAchieved:denseElapsed?denseDistance/denseElapsed:null,denseMotionSpeed:denseSeconds?denseDistance/denseSeconds:null,
    gapCount,gapSeconds,maximumGapMs:maximumGap*1000,meanCoverageMs:covered/printSeconds*1000,
    minCoverageMs:Number.isFinite(minCoverage)?minCoverage*1000:null,capChanges,peak,firstDenseEntry,lastLongExit,trace};
}
function selfTest(){
  const p=plan([{length:10,cap:100},{length:1,cap:20}],0,100);
  if(p[0].exit>20+EPS||p[1].entry!==p[0].exit||p[1].exit!==0)throw Error("lookahead brakes before slow segment");
  const r=simulate(fixture({baud:250000,chord:.05,stallMs:50}),{policy:"predictive"});
  if(r.segments!==540||r.trace.some((x,i)=>i&&Math.abs(x.entry-r.trace[i-1].exit)>EPS)||r.trace.some(x=>x.peak>x.cap+EPS))throw Error("continuity or cap");
  const s=simulate(fixture({baud:500000,chord:.1}),{policy:"supply"});
  if(s.trace.at(-1).exit!==0||s.distance<=0)throw Error("tail stop");
  return "3 groups of checks passed";
}
function matrix(){
  const result=[];
  for(const chord of [.05,.1,.2])for(const baud of [250000,500000])for(const stallMs of [0,20,80])for(const slots of [24,32])
    for(const policy of ["none","count","supply","predictive"]){
      const spec={chord,baud,stallMs},opts={slots,policy};
      result.push({spec,opts,...simulate(fixture(spec),opts)});
    }
  return result;
}
if(require.main===module){
  if(process.argv[2]==="--self-test")console.log(selfTest());
  else console.log(JSON.stringify({schema:1,model:"scalar-lookahead-preview",selfTest:selfTest(),rows:matrix()},null,2));
}
module.exports={plan,fixture,simulate,matrix,selfTest};

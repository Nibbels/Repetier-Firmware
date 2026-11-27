// Task0032 S1c: finite queue / host byte-credit, fluid CPU simulation.
// No hardware FIFO, ISR preemption, pulse jitter, acceleration or service-deadline proof.
"use strict";
const {geometry,evaluate,groupFor,defaults}=require("./demand-model.cjs");
const CPU=16000000, EPS=1e-10;
function run(rows,speed,policy,p,opts){
  const {baud,credit=127,frameWindow=Infinity,slots=24,prefill=12,ackDelay=0.001}=opts;
  if(!(baud>0&&credit>0&&slots>=prefill&&prefill>=1&&frameWindow>=1&&ackDelay>=0&&rows.length&&rows.every(r=>r.bytes<=credit)))throw Error("invalid queue model");
  const jobs=rows.map(r=>({...r,...evaluate(r,speed,groupFor(r,speed,policy,p),p),group:groupFor(r,speed,policy,p)}));
  let t=0,send=0,received=0,planned=0,done=0,used=0,outstanding=0,wire=null,active=null,planner=null,acks=[],txEnd=0;
  let start=null,starveStart=null,starveCount=0,starveSeconds=0,maxGap=0,maxQueued=0,maxCredit=0,maxRawBytes=0,raw=[],queue=[];
  let minCoverage=Infinity,coverageArea=0,queueArea=0,motionImpossible=0,plannerBusy=0,plannerBlocked=0,idleCpu=0,receivedBytes=0,acked=0;
  let motionCycles=0,admissionCycles=0,totalPath=0,steps=[0,0,0],eventCount=0;
  const deviceBaud=baud===115200?16000000/(8*17):baud;
  const coverage=()=>Math.max(0,active?active.end-t:0)+queue.reduce((s,j)=>s+j.duration,0);
  const occ=()=>queue.length+(active?1:0)+(planner?1:0);
  function dispatch(){
    if(!active&&queue.length&&(start!==null||queue.length>=prefill||planned===jobs.length)){
      if(start===null)start=t;
      if(starveStart!==null){const gap=t-starveStart;if(gap>EPS){starveCount++;starveSeconds+=gap;maxGap=Math.max(maxGap,gap);}starveStart=null;}
      const job=queue.shift();active={job,end:t+job.duration};
      if(job.motion/job.duration>CPU*(1-p.reserve)+EPS)motionImpossible++;
    }
    if(!planner&&raw.length&&occ()<slots){
      const job=raw.shift();planner={job,remaining:job.fore+job.uart};
    }
    if(!wire&&send<jobs.length&&outstanding<frameWindow&&used+jobs[send].bytes<=credit){
      const job=jobs[send++];used+=job.bytes;outstanding++;maxCredit=Math.max(maxCredit,used);
      wire={job,end:t+job.bytes*10/baud};
    }
    maxQueued=Math.max(maxQueued,occ());
    maxRawBytes=Math.max(maxRawBytes,raw.reduce((s,j)=>s+j.bytes,0));
    if(start!==null&&planned<jobs.length)minCoverage=Math.min(minCoverage,coverage());
    if(occ()>slots||used>credit||used<0)throw Error("capacity invariant");
  }
  dispatch();
  while(done<jobs.length){
    if(++eventCount>jobs.length*12+100)throw Error("event bound");
    const rate=Math.max(0,CPU*(1-p.reserve)-(active?active.job.motion/active.job.duration:0));
    const next=Math.min(wire?wire.end:Infinity,active?active.end:Infinity,acks.length?acks[0].end:Infinity,planner&&rate>0?t+planner.remaining/rate:Infinity);
    if(!Number.isFinite(next)||next<t-EPS)throw Error("deadlock or clock reversal");
    const dt=Math.max(0,next-t);
    if(start!==null){coverageArea+=coverage()*dt-(active?dt*dt/2:0);queueArea+=(queue.length+(active?1:0))*dt;}
    if(planner){planner.remaining=Math.max(0,planner.remaining-rate*dt);plannerBusy+=dt;}else{idleCpu+=rate*dt;if(raw.length&&occ()>=slots)plannerBlocked+=dt;}
    t=next;
    // Simultaneous events: completion before new dispatch.
    if(active&&active.end<=t+EPS){
      const j=active.job;done++;totalPath+=j.length;motionCycles+=j.motion;j.steps.forEach((v,i)=>steps[i]+=v);active=null;
    }
    if(wire&&wire.end<=t+EPS){received++;receivedBytes+=wire.job.bytes;raw.push(wire.job);wire=null;}
    if(planner&&planner.remaining<=0.01){
      const j=planner.job;planned++;admissionCycles+=j.fore+j.uart;queue.push(j);planner=null;
      txEnd=Math.max(txEnd,t+ackDelay)+p.ackBytes*10/deviceBaud;acks.push({end:txEnd,bytes:j.bytes});
    }
    while(acks.length&&acks[0].end<=t+EPS){const a=acks.shift();used-=a.bytes;outstanding--;acked++;}
    dispatch();
    if(start!==null&&!active&&done<jobs.length&&starveStart===null)starveStart=t;
  }
  if(done!==planned||done!==received||done!==send||occ()!==0||raw.length!==0||wire!==null)throw Error("accounting invariant");
  const printSeconds=t-start,nominalSeconds=jobs.reduce((s,j)=>s+j.duration,0);
  if(Math.abs(printSeconds-nominalSeconds-starveSeconds)>1e-7*Math.max(1,printSeconds))throw Error("time accounting");
  const finalAckSeconds=Math.max(t,txEnd);
  return {segments:done,offeredFrames:send,receivedFrames:received,plannedFrames:planned,executedFrames:done,ackedAtMotionEnd:acked,finalAckSeconds,receivedBytes,steps,pathMm:totalPath,startSeconds:start,printSeconds,nominalSeconds,
    achievedMmPerSecond:totalPath/printSeconds,endToEndMmPerSecond:totalPath/t,starveCount,starveSeconds,maxGapMs:maxGap*1000,
    minCoverageBeforeLastPlanMs:Number.isFinite(minCoverage)?minCoverage*1000:null,meanCoverageMs:coverageArea/printSeconds*1000,meanQueued:queueArea/printSeconds,maxReservedSlots:maxQueued,maxCredit,maxRawBytes,
    plannerBusySeconds:plannerBusy,plannerBlockedSeconds:plannerBlocked,unusedAdmissionCycles:idleCpu,motionCycles,admissionCycles,motionImpossible,validFluidMotion:motionImpossible===0,eventCount};
}
function scenario(spec,policy,params={},opts={}){
  const p={...defaults,...params};
  return {spec,policy,params:p,opts:{baud:spec.baud,...opts},...run(geometry(spec,p),spec.speed,policy,p,{baud:spec.baud,...opts})};
}
function selfTest(){
  const check=(v,m)=>{if(!v)throw Error(m)};
  const p={...defaults,planner:0,rxByte:0,rxCall:0,ackBytes:0,reserve:0};
  const rows=Array.from({length:4},()=>({length:1,n:0,cls:"X",steps:[0,0,0],bytes:10}));
  const r=run(rows,10,"one",p,{baud:1000,credit:10,slots:2,prefill:1,ackDelay:0});
  check(r.executedFrames===4&&r.receivedBytes===40&&r.maxCredit===10&&r.maxReservedSlots<=2,"finite credit and slots");
  check(r.printSeconds>=.4&&r.starveSeconds<.001,"balanced wire and motion");
  const slow=run(rows,100,"one",p,{baud:1000,credit:10,slots:2,prefill:1,ackDelay:0});
  check(slow.starveCount===3&&slow.starveSeconds>.26,"wire starvation");
  const spec={shape:"straight",count:120,radius:1.35,chord:.2,speed:60,baud:500000,ratio:.04};
  const a=scenario(spec,"one"),b=scenario(spec,"adaptive");
  check(JSON.stringify(a.steps)===JSON.stringify(b.steps)&&a.executedFrames===120,"step invariance");
  check(a.maxCredit<=127&&b.maxReservedSlots<=24,"queue and credit bounds");
  check(JSON.stringify(a)===JSON.stringify(scenario(spec,"one")),"deterministic");
  const h=scenario({...spec,baud:115200},"adaptive");
  check(h.motionCycles===b.motionCycles,"wire starvation does not choose packing");
  return "7 groups of checks passed";
}
function matrix(){
  const out=[];
  for(const shape of ["straight","zigzag","circle"])for(const chord of [.05,.1,.2,.6])for(const speed of [30,60,100,150])for(const baud of [115200,250000,500000,1000000,2000000])for(const ratio of [.04,.3])for(const policy of ["one","3300","5000","adaptive"])for(const frameWindow of [1,32])
    out.push({spec:{shape,count:120,radius:1.35,chord,speed,baud,ratio},policy,params:{},opts:{frameWindow}});
  return out;
}
if(require.main===module){
  if(process.argv[2]==="--self-test"){console.log(selfTest());process.exit(0);}
  const fs=require("node:fs"),path=require("node:path"),crypto=require("node:crypto");
  const dir=process.argv[2];if(!dir)throw Error("usage: node queue-model.cjs --self-test | OUTPUT_DIR");
  fs.mkdirSync(dir,{recursive:true});
  const rows=matrix().map(x=>scenario(x.spec,x.policy,x.params,x.opts));
  fs.writeFileSync(path.join(dir,"queue.jsonl"),rows.map(x=>JSON.stringify(x)).join("\n")+"\n");
  const hashes=Object.fromEntries([__filename,require.resolve("./demand-model.cjs"),require.resolve("./link-model.cjs")].map(f=>[path.basename(f),crypto.createHash("sha256").update(fs.readFileSync(f)).digest("hex")]));
  fs.writeFileSync(path.join(dir,"queue-summary.json"),JSON.stringify({schema:1,model:"fluid-cpu-finite-queue",cases:rows.length,node:process.version,hashes,selfTest:selfTest()},null,2)+"\n");
}
module.exports={run,scenario,selfTest,matrix};

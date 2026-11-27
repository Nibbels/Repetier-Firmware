// Task0032 S1b: deterministic demand sensitivity model, NOT firmware WCET.
// Cycle anchors: Task0030 RF2000v2 isolated cruise, group 2, no corrections/Advance.
// Split into fixed/per-step terms is an assumption, not measured independently.
"use strict";
const {workload,frame}=require("./link-model.cjs");
const CPU=16000000, anchors={X:1009,XY:1183,XYE:1441,denseXYE:1551,E:960};
const defaults={xy:152.38,e:256,fixed:400,wrapper:100,planner:3000,rxByte:85,rxCall:99,reserve:0.20,ackBytes:4};
function pathWorkload(spec){
  if(!spec.shape||spec.shape==="circle")return workload(spec);
  if(!["straight","zigzag"].includes(spec.shape))throw Error("bad shape");
  const count=spec.count??120, start=[100,100],lines=[],lengths=[];
  let previous=start;
  for(let i=1;i<=count;i++){
    const dx=spec.shape==="straight"?spec.chord:spec.chord/Math.sqrt(2);
    const p=[+(100+i*dx).toFixed(4),+(100+(spec.shape==="zigzag"&&i%2?dx:0)).toFixed(4)];
    lengths.push(Math.hypot(p[0]-previous[0],p[1]-previous[1]));
    lines.push(frame(i,"G1 X"+p[0].toFixed(4)+" Y"+p[1].toFixed(4)+" E0.00000"+(i===1?" F"+spec.speed*60:"")));
    previous=p;
  }
  return {start,lines,lengths};
}
function geometry(spec,p=defaults){
  const w=pathWorkload(spec), rows=[]; let prior=[...w.start.map(x=>Math.round(x*p.xy)),0], e=0, distance=0;
  const ratio=spec.ratio??0.04;
  for(let i=0;i<w.lines.length;i++){
    const line=w.lines[i],x=Number(/ X([\d.]+)/.exec(line)[1]),y=Number(/ Y([\d.]+)/.exec(line)[1]);
    distance+=w.lengths[i];e=Number((distance*ratio).toFixed(5));
    const pos=[Math.round(x*p.xy),Math.round(y*p.xy),Math.round(e*p.e)];
    const d=pos.map((v,j)=>v-prior[j]);prior=pos;
    const steps=d.map(Math.abs),n=Math.max(...steps);
    const cls=steps[2]&&!steps[0]&&!steps[1]?"E":steps[2]?(steps[2]/Math.max(1,n)>=.5?"denseXYE":"XYE"):(steps[0]&&steps[1]?"XY":"X");
    const body=line.slice(line.indexOf(" ")+1,line.lastIndexOf("*")).replace(/ E[\d.]+/," E"+e.toFixed(5));
    rows.push({length:w.lengths[i],steps,d,n,cls,bytes:frame(i+1,body).length,numberedAckBytes:5+String(i+1).length});
  }
  return rows;
}
function evaluate(row,speed,group,p){
  const requested=row.length/speed, duration=Math.max(requested,row.n*512/CPU);
  const calls=row.n?Math.ceil(row.n/group):0;
  const stepCost=(anchors[row.cls]-p.fixed)/2;
  const motion=row.n*stepCost+calls*(p.fixed+p.wrapper);
  const ackBytes=p.ackBytes==="numbered"?row.numberedAckBytes:p.ackBytes;
  if(!Number.isInteger(ackBytes)||ackBytes<0)throw Error("bad ACK byte count");
  const uart=row.bytes*73+ackBytes*74;
  // Task0028: 20 B incomplete ASCII 1799 cycles; 40 B 3499.
  // Single fragment approximation; complete parse/plan is ADDITIONAL.
  const receive=row.bytes*p.rxByte+p.rxCall, fore=p.planner+receive;
  const budget=CPU*duration*(1-p.reserve);
  return {duration,calls,motion,uart,fore,receive,ackBytes,feasible:motion+uart+fore<=budget+1e-8,
    packedSteps:row.n>0&&group>1?row.n-(row.n%group===1?1:0):0};
}
function groupFor(row,speed,policy,p){
  if(!row.n)return 1;
  const t=Math.max(512,Math.floor(CPU*row.length/speed/row.n));
  if(policy==="one")return 1;
  if(policy==="3300"||policy==="5000")return Math.min(row.n,Math.ceil(Number(policy)/t));
  if(policy!=="adaptive"&&policy!=="adaptive-margin")throw Error("bad policy");
  const max=Math.min(row.n,Math.ceil(5000/512));
  // Selection-only headroom; actual simulated service reserve remains unchanged.
  if(policy==="adaptive-margin"){
    const margin=p.selectionMargin??.05;
    if(!(margin>=0&&p.reserve+margin<1))throw Error("bad selection margin");
    for(let g=1;g<=max;g++)if(evaluate(row,speed,g,{...p,reserve:p.reserve+margin}).feasible)return g;
  }
  for(let g=1;g<=max;g++)if(evaluate(row,speed,g,p).feasible)return g;
  // Research sentinel, not a production fallback: caller must report infeasibility.
  return max;
}
function scenario(spec,policy,params={}){
  const p={...defaults,...params};
  if(!(p.fixed>=0&&p.fixed<=Math.min(...Object.values(anchors))&&p.wrapper>=0&&p.planner>=0&&p.rxByte>=0&&p.rxCall>=0&&p.reserve>=0&&p.reserve<1&&p.xy>0&&p.e>0&&spec.speed>0&&spec.baud>0))throw Error("invalid parameters");
  const rows=geometry(spec,p),sums={segments:rows.length,pathMm:0,seconds:0,bytes:0,primary:0,axisSteps:[0,0,0],motionCycles:0,uartCycles:0,plannerCycles:0,receiveCycles:0,calls:0,packedSteps:0,overloaded:0,groupHistogram:{},maxSegmentDemand:0,minQueue24Ms:Infinity,maxQueue24Ms:0};
  let window=[];
  for(const row of rows){
    const g=groupFor(row,spec.speed,policy,p),v=evaluate(row,spec.speed,g,p);
    sums.pathMm+=row.length;sums.seconds+=v.duration;sums.bytes+=row.bytes;sums.primary+=row.n;
    row.steps.forEach((s,i)=>sums.axisSteps[i]+=s);
    sums.motionCycles+=v.motion;sums.uartCycles+=v.uart;sums.plannerCycles+=v.fore-v.receive;sums.receiveCycles+=v.receive;sums.calls+=v.calls;sums.packedSteps+=v.packedSteps;
    sums.overloaded+=+!v.feasible;sums.groupHistogram[g]=(sums.groupHistogram[g]??0)+1;
    sums.maxSegmentDemand=Math.max(sums.maxSegmentDemand,(v.motion+v.uart+v.fore)/(CPU*v.duration)+p.reserve);
    window.push(v.duration);if(window.length>24)window.shift();
    if(window.length===Math.min(rows.length,24)){
      const ms=window.reduce((a,b)=>a+b,0)*1000;sums.minQueue24Ms=Math.min(sums.minQueue24Ms,ms);sums.maxQueue24Ms=Math.max(sums.maxQueue24Ms,ms);
    }
  }
  const utilization=(sums.motionCycles+sums.uartCycles+sums.plannerCycles+sums.receiveCycles)/(CPU*sums.seconds)+p.reserve;
  const wireSeconds=sums.bytes*10/spec.baud;
  return {spec,policy,params:p,...sums,movesPerMm:rows.length/sums.pathMm,movesPerSecond:rows.length/sums.seconds,nominalPathSpeed:sums.pathMm/sums.seconds,
    wireCeilingMmPerSecond:sums.pathMm/wireSeconds,linkFeasible:wireSeconds<=sums.seconds,meanDemand:utilization,
    cpuFeasible:sums.overloaded===0,meanGroup:sums.calls?sums.primary/sums.calls:1,packedFraction:sums.primary?sums.packedSteps/sums.primary:0};
}
function selfTest(){
  const check=(v,m)=>{if(!v)throw Error(m)};
  const p={...defaults,planner:0,reserve:0,wrapper:0};
  const row={n:2,length:1,cls:"X",bytes:0};
  check(evaluate(row,1,2,p).motion===1009,"two-step cycle anchor");
  check(evaluate({...row,n:5},1,2,p).calls===3,"clipped tail call");
  check(evaluate({...row,n:5},1,2,p).packedSteps===4,"single tail not packed");
  const spec={radius:1.35,chord:.1,speed:60,baud:500000,ratio:.04};
  const rs=["one","3300","5000","adaptive"].map(x=>scenario(spec,x));
  check(rs.every(x=>JSON.stringify(x.axisSteps)===JSON.stringify(rs[0].axisSteps)&&x.primary===rs[0].primary&&x.seconds===rs[0].seconds),"step and duration invariance");
  const a=geometry(spec);check(a.reduce((s,x)=>s+x.d[0],0)===0&&a.reduce((s,x)=>s+x.d[1],0)===0,"closed circle signed steps");
  for(const x of a){const g=groupFor(x,spec.speed,"adaptive",defaults);for(let n=1;n<g;n++)check(!evaluate(x,spec.speed,n,defaults).feasible,"minimum group");}
  check(scenario({...spec,baud:115200},"adaptive").motionCycles===rs[3].motionCycles,"no packing change merely from link saturation");
  check(scenario(spec,"adaptive",{planner:1e9}).overloaded===rs[3].segments,"infeasible explicit");
  check(evaluate({...row,bytes:40},1,2,defaults).receive===3499,"partial receiver anchor");
  const zig=geometry({...spec,shape:"zigzag",count:120,ratio:.8});
  check(zig.length===120&&zig.some(x=>x.d[1]>0)&&zig.some(x=>x.d[1]<0)&&zig.some(x=>x.steps[2]===x.n),"zigzag and mixed E primary");
  check(scenario(spec,"adaptive-margin",{selectionMargin:0}).motionCycles===rs[3].motionCycles,"zero selection margin equals adaptive");
  for(const x of a)check(groupFor(x,spec.speed,"adaptive-margin",defaults)>=groupFor(x,spec.speed,"adaptive",defaults),"headroom never reduces group");
  check(a.every((r,i)=>r.numberedAckBytes===5+String(i+1).length),"numbered ACK byte count");
  check(evaluate({...row,numberedAckBytes:10},1,2,{...p,ackBytes:"numbered"}).uart===740,"numbered ACK IRQ charge");
  return "14 groups of checks passed";
}
function matrix(){
  const out=[];
  for(const chord of [.05,.1,.2,.6,1])for(const speed of [30,60,100,150])for(const ratio of [0,.04,.3])for(const baud of [115200,250000,500000,1000000,2000000])
    for(const fixed of [200,400,600])for(const planner of [1000,3000,6000])for(const policy of ["one","3300","5000","adaptive"])
      out.push({spec:{radius:1.35,chord,speed,ratio,baud},policy,params:{fixed,planner}});
  // Targeted shape controls at the same chord/speed/ratio and baud.
  for(const shape of ["straight","zigzag","circle"])for(const chord of [.05,.1,.2,.6,1])for(const speed of [30,60,100,150])for(const ratio of [0,.04,.3,.8])for(const baud of [115200,500000])for(const policy of ["one","3300","5000","adaptive"])
    out.push({spec:{shape,radius:1.35,chord,speed,ratio,baud},policy,params:{}});
  return out;
}
if(require.main===module){
  if(process.argv[2]==="--self-test"){console.log(selfTest());process.exit(0);}
  const fs=require("node:fs"),path=require("node:path"),crypto=require("node:crypto");
  const dir=process.argv[2];if(!dir)throw Error("usage: node demand-model.cjs --self-test | OUTPUT_DIR");
  fs.mkdirSync(dir,{recursive:true});
  const rows=matrix().map(x=>scenario(x.spec,x.policy,x.params));
  fs.writeFileSync(path.join(dir,"demand.jsonl"),rows.map(x=>JSON.stringify(x)).join("\n")+"\n");
  fs.writeFileSync(path.join(dir,"demand-summary.json"),JSON.stringify({schema:2,model:"segment-demand-sensitivity",node:process.version,cases:rows.length,sourceSha256:crypto.createHash("sha256").update(fs.readFileSync(__filename)).digest("hex"),linkSha256:crypto.createHash("sha256").update(fs.readFileSync(require.resolve("./link-model.cjs"))).digest("hex"),selfTest:selfTest()},null,2)+"\n");
}
module.exports={geometry,evaluate,groupFor,scenario,selfTest,matrix,defaults};

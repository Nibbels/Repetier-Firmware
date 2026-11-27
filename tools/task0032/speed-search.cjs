// Task0032 S1d: discrete speed search, not production admission limits.
"use strict";
const demand=require("./demand-model.cjs"),queue=require("./queue-model.cjs");
const policies=[["one",0],["3300",0],["5000",0],["adaptive",0],["adaptive-margin",.05],["adaptive-margin",.10]];
const speeds=Array.from({length:25},(_,i)=>30+5*i);
function search(spec,policy,margin,opts={},speedsToTest=speeds){
  const cases=speedsToTest.map(speed=>{
    const s={...spec,speed},p={selectionMargin:margin};
    const d=demand.scenario(s,policy,p),q=queue.scenario(s,policy,p,opts);
    const pass=d.cpuFeasible&&d.linkFeasible&&q.validFluidMotion&&q.starveCount===0;
    return {speed,pass,cpuLocal:d.cpuFeasible,link:d.linkFeasible,fluid:q.validFluidMotion,starveCount:q.starveCount,maxGapMs:q.maxGapMs,achieved:q.achievedMmPerSecond,meanCoverageMs:q.meanCoverageMs,minCoverageMs:q.minCoverageBeforeLastPlanMs,group:d.meanGroup,packedFraction:d.packedFraction};
  });
  const feasible=cases.filter(x=>x.pass);
  const best=feasible.reduce((a,b)=>!a||a.speed<b.speed?b:a,null);
  return {spec,policy,margin,opts,best,cases};
}
function selfTest(){
  const s={shape:"straight",count:120,radius:1.35,chord:.6,baud:500000,ratio:.04};
  const r=search(s,"adaptive",0,{frameWindow:32},[30,60,100]);
  if(!r.best||r.best.speed!==100||r.cases.length!==3)throw Error("speed selection");
  const a=search(s,"adaptive-margin",0,{frameWindow:32},[30,60,100]);
  if(JSON.stringify(a.cases)!==JSON.stringify(r.cases))throw Error("zero margin");
  const b=search({...s,chord:.05,baud:115200},"one",0,{frameWindow:1},[100,150]);
  if(b.best!==null)throw Error("explicit no candidate");
  return "3 groups of checks passed";
}
function matrix(){
  const out=[];
  for(const shape of ["straight","zigzag","circle"])for(const chord of [.05,.1,.2,.6])for(const baud of [115200,250000,500000])for(const [policy,margin] of policies)
    out.push({spec:{shape,count:120,radius:1.35,chord,baud,ratio:.04},policy,margin,opts:{frameWindow:32}});
  return out;
}
if(require.main===module){
  if(process.argv[2]==="--self-test"){console.log(selfTest());process.exit(0);}
  const fs=require("node:fs"),path=require("node:path"),crypto=require("node:crypto");
  const dir=process.argv[2];if(!dir)throw Error("usage: speed-search.cjs --self-test | OUTPUT_DIR");
  fs.mkdirSync(dir,{recursive:true});
  const rows=matrix().map(x=>search(x.spec,x.policy,x.margin,x.opts));
  fs.writeFileSync(path.join(dir,"speed-search.jsonl"),rows.map(x=>JSON.stringify(x)).join("\n")+"\n");
  const hashes=Object.fromEntries([__filename,...["demand","queue","link"].map(n=>require.resolve("./"+n+"-model.cjs"))].map(f=>[path.basename(f),crypto.createHash("sha256").update(fs.readFileSync(f)).digest("hex")]));
  fs.writeFileSync(path.join(dir,"speed-search-summary.json"),JSON.stringify({schema:1,model:"discrete-no-starvation-screen",rows:rows.length,cases:rows.reduce((s,x)=>s+x.cases.length,0),hashes,node:process.version,selfTest:selfTest()},null,2)+"\n");
}
module.exports={search,matrix,selfTest};

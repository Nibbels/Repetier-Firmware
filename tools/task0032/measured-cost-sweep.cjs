// Consume isolated parser measurements and the completed 0032-P report in
// the existing shared demand/queue model. This is sensitivity, not integrated
// qualification: planner fixture mix is not matched to the simulated moves.
"use strict";
const {scenario:demand}=require("./demand-model.cjs");
const {scenario:queue}=require("./queue-model.cjs");
const {parameters,provenance:transport}=require("./transport-handoff.cjs");
const planner={commit:"e1b976318fd34c1e556a2f16491aaec9108d0d71",
  path:"research/task0032-planner-queue/REPORT.md",blob:"09ecfceaa57fdca77fb585de8c3b0dde6c919d93"};
function plannerCosts(report){
  const m=/Paired prepare\+update \(192 matched shape\/depth samples\): baseline median\/max (\d+)\/(\d+);/.exec(report);
  if(!m)throw Error("pinned planner paired cost anchor not found");
  const median=Number(m[1]),sampleMax=Number(m[2]);
  if(!(median>0&&sampleMax>=median))throw Error("invalid planner samples");
  return {historical:3000,median,sampleMax};
}
function receiverCosts(result){
  if(result.schema!==1||!Array.isArray(result.samples)||!Array.isArray(result.records))throw Error("bad parser result");
  const formats=["ascii","binary-v1","binary-v2"],out={};
  for(let f=0;f<formats.length;f++){
    const a=result.records.filter(r=>r.format===f&&r.stage===1).map(r=>r.cycles).sort((a,b)=>a-b);
    const expected=result.samples.filter(s=>s.format===f).length;
    if(expected!==9||a.length!==expected||a.some(n=>!Number.isInteger(n)||n<=0))throw Error("incomplete parser measurements");
    out[formats[f]]={n:a.length,min:a[0],median:a[Math.floor(a.length/2)],max:a.at(-1)};
  }
  return out;
}
function sweep(contract,report,result){
  const pc=plannerCosts(report),rc=receiverCosts(result),rows=[];
  const lengths=[...new Set(contract.stress.map(r=>r.segment_mm))].sort((a,b)=>a-b);
  const speeds=[...new Set(contract.stress.map(r=>r.requested_mm_s))].sort((a,b)=>a-b);
  for(const shape of ["straight","circle"])for(const chord of lengths)for(const speed of speeds)
    for(const frameWindow of [1,32])for(const packetFormat of Object.keys(rc))for(const plannerCase of Object.keys(pc)){
      const spec={shape,count:120,radius:1.35,chord,speed,baud:contract.baud,ratio:.04,packetFormat};
      const params={...parameters(contract,"budget"),planner:pc[plannerCase],receiverCycles:rc[packetFormat].median};
      const d=demand(spec,"adaptive",params);
      const q=queue(spec,"adaptive",params,{slots:24,frameWindow,ackPoint:"admission",parseFraction:0});
      rows.push({spec,frameWindow,plannerCase,receiverCycles:params.receiverCycles,plannerCyclesPerMove:params.planner,
        requestedUtilization:d.meanDemand,localCpuFeasible:d.cpuFeasible,wireFeasible:d.linkFeasible,
        motionOnlyFeasible:q.validFluidMotion,achieved:q.achievedMmPerSecond,gaps:q.starveCount,
        coverageMs:q.meanCoverageMs,meanGroup:d.meanGroup,steps:q.steps});
    }
  if(rows.length!==540)throw Error("cost matrix coverage");
  for(let i=0;i<rows.length;i+=3){
    const same=rows.slice(i,i+3);
    if(same.some(r=>JSON.stringify(r.steps)!==JSON.stringify(same[0].steps)))throw Error("cost override changed geometry");
  }
  return {plannerCosts:pc,receiverCosts:rc,rows};
}
if(require.main===module){
  const fs=require("node:fs"),crypto=require("node:crypto");
  const readPinned=(path,blob)=>{const a=fs.readFileSync(path);const actual=crypto.createHash("sha1").update("blob "+a.length+"\0").update(a).digest("hex");if(actual!==blob)throw Error("pinned blob mismatch "+path);return a.toString();};
  const contract=JSON.parse(readPinned(process.argv[2],transport.blob));
  const report=readPinned(process.argv[3],planner.blob);
  const parserResult=JSON.parse(fs.readFileSync(process.argv[4],"utf8"));
  const result=sweep(contract,report,parserResult);
  console.log(JSON.stringify({schema:1,planner,transport,parserElfSha256:parserResult.elf_sha256,
    qualification:"Sensitivity of existing fluid model to isolated full receiver and paired planner sample costs. Historical 3000 is only a comparison, median/max not matched or WCET, omitted real G-code planning setup and maintenance paths, no TX wait/IRQ deadline or physical qualification. 24 slots and 250k fixed. Receiver complete cost replaces old incomplete per-byte estimate; ACK after receiver, planner separate; IRQ service charged separately. No production speed or packing decision.",
    ...result},null,2));
}
module.exports={plannerCosts,receiverCosts,sweep};

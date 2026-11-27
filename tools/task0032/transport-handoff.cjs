// Task0032: consume Task0033's pinned transport contract, not another transport model.
"use strict";
const {scenario:demand}=require("./demand-model.cjs");
const {scenario:queue}=require("./queue-model.cjs");
const provenance={
  commit:"4cdb22106e66c02f86dffeb3f49a6a15506c2038",
  path:"tools/task0033/transport-contract-results.json",
  blob:"003d9cf4a256ef99b46441c3b017fa6bc29bae57"
};
function parameters(contract,mode){
  const b=contract.budgets_cycles,c=contract.measured_isolated_candidate;
  if(contract.schema!==1||contract.clock_hz!==16000000||contract.baud!==250000||
     contract.status!=="owner_agreed_baud_and_proposed_engineering_budgets_not_qualified")
    throw Error("unexpected transport contract");
  if(!["budget","isolated"].includes(mode))throw Error("unknown mode");
  return {rxIsr:mode==="budget"?b.rx_normal:c.rx_normal_cycles,
    txIsr:mode==="budget"?b.tx_normal:c.tx_normal_cycles,
    txEmptyPerCommand:mode==="budget"?b.tx_empty:c.tx_empty_cycles,
    ackBytes:"numbered"};
}
function matrix(contract){
  const lengths=[...new Set(contract.stress.map(r=>r.segment_mm))].sort((a,b)=>a-b);
  const speeds=[...new Set(contract.stress.map(r=>r.requested_mm_s))].sort((a,b)=>a-b);
  const rows=[];
  for(const baud of [contract.reference_baud,contract.baud])
    for(const shape of ["straight","circle"])for(const chord of lengths)for(const speed of speeds)
      for(const policy of ["one","3300","adaptive"])for(const mode of ["budget","isolated"]){
        const spec={shape,count:120,radius:1.35,chord,speed,baud,ratio:.04};
        const params=parameters(contract,mode);
        const d=demand(spec,policy,params),q=queue(spec,policy,params,{frameWindow:32});
        const bytesPerSecond=speed*d.bytes/d.pathMm;
        rows.push({spec,policy,mode,params,bytesPerSecond,
          planningScenarioWithin:baud===contract.baud?bytesPerSecond<=contract.planning_rx_bytes_s:null,
          wireFeasible:d.linkFeasible,localCpuFeasible:d.cpuFeasible,validFluidMotion:q.validFluidMotion,
          starveCount:q.starveCount,meanCoverageMs:q.meanCoverageMs,achievedMmPerSecond:q.achievedMmPerSecond,
          meanGroup:d.meanGroup,packedFraction:d.packedFraction});
      }
  return rows;
}
function admissionSweep(contract){
  const rows=[];
  const lengths=[...new Set(contract.stress.map(r=>r.segment_mm))].sort((a,b)=>a-b);
  const speeds=[...new Set(contract.stress.map(r=>r.requested_mm_s))].sort((a,b)=>a-b);
  const params=parameters(contract,"budget");
  for(const chord of lengths)for(const speed of speeds)for(const frameWindow of [1,32]){
    const spec={shape:"straight",count:600,radius:1.35,chord,speed,baud:contract.baud,ratio:.04};
    const baseline=queue(spec,"adaptive",params,{frameWindow,ackPoint:"publication"});
    for(const parseFraction of [.25,.5,.75]){
      const current=queue(spec,"adaptive",params,{frameWindow,ackPoint:"admission",parseFraction});
      if(current.admissionCycles!==baseline.admissionCycles||JSON.stringify(current.steps)!==JSON.stringify(baseline.steps))
        throw Error("admission split changed total work or steps");
      rows.push({spec,frameWindow,parseFraction,publication:{achieved:baseline.achievedMmPerSecond,gaps:baseline.starveCount,coverageMs:baseline.meanCoverageMs},
        admission:{achieved:current.achievedMmPerSecond,gaps:current.starveCount,coverageMs:current.meanCoverageMs,acceptedWaitingSeconds:current.acceptedWaitingSeconds,maxAcceptedNotPlanned:current.maxAcceptedNotPlanned}});
    }
  }
  return rows;
}
function wireFormatSweep(contract){
  const rows=[];
  const lengths=[...new Set(contract.stress.map(r=>r.segment_mm))].sort((a,b)=>a-b);
  const speeds=[...new Set(contract.stress.map(r=>r.requested_mm_s))].sort((a,b)=>a-b);
  const params=parameters(contract,"budget");
  for(const shape of ["straight","circle"])for(const chord of lengths)for(const speed of speeds)
    for(const frameWindow of [1,32])for(const packetFormat of ["ascii","binary-v1","binary-v2","mixed"]){
      const spec={shape,count:120,radius:1.35,chord,speed,baud:contract.baud,ratio:.04,packetFormat};
      const d=demand(spec,"adaptive",params);
      const q=queue(spec,"adaptive",params,{frameWindow,ackPoint:"admission",parseFraction:.5});
      const offeredRxBytesS=d.bytes/d.pathMm*speed;
      rows.push({spec,frameWindow,offeredRxBytesS,wireBytes:d.bytes,
        limits:{aboveIdealWireAtRequest:offeredRxBytesS>contract.wire_bytes_s,
          abovePlanningScenario:offeredRxBytesS>contract.planning_rx_bytes_s,
          nominalStepFloor:d.nominalPathSpeed<speed-1e-8,
          localCpuBudgetExceeded:!d.cpuFeasible,
          motionOnlyOverBudget:!q.validFluidMotion,
          observedFluidStarvation:q.starveCount>0},
        cycles:{motion:d.motionCycles,uart:d.uartCycles,receive:d.receiveCycles,additionalParsePlan:d.plannerCycles},
        achieved:q.achievedMmPerSecond,gaps:q.starveCount,coverageMs:q.meanCoverageMs,meanGroup:d.meanGroup,
        steps:q.steps});
    }
  return rows;
}
function selfTest(contract){
  const b=parameters(contract,"budget"),i=parameters(contract,"isolated");
  if(b.rxIsr!==80||b.txIsr!==80||b.txEmptyPerCommand!==64||
     i.rxIsr!==77||i.txIsr!==71||i.txEmptyPerCommand!==61)throw Error("contract values");
  const rows=matrix(contract);
  if(rows.length!==360||rows.some(r=>r.spec.baud!==contract.baud&&r.planningScenarioWithin!==null))
    throw Error("matrix coverage");
  if(rows.some(r=>!Number.isFinite(r.achievedMmPerSecond)||r.spec.speed>100))throw Error("workload bounds");
  if(admissionSweep(contract).length!==90)throw Error("admission matrix coverage");
  const wire=wireFormatSweep(contract);
  if(wire.length!==240)throw Error("wire-format workload coverage");
  for(let i=0;i<wire.length;i+=4){
    const same=wire.slice(i,i+4);
    if(same.some(r=>JSON.stringify(r.steps)!==JSON.stringify(same[0].steps))||
       !(same[1].wireBytes<same[3].wireBytes&&same[3].wireBytes<same[2].wireBytes))
      throw Error("wire format must not change motion steps");
  }
  return "5 groups of checks passed";
}
if(require.main===module){
  const fs=require("node:fs"),crypto=require("node:crypto");
  const input=fs.readFileSync(process.argv[2]);
  const blob=crypto.createHash("sha1").update("blob "+input.length+"\0").update(input).digest("hex");
  if(blob!==provenance.blob)throw Error("pinned Task0033 contract blob mismatch");
  const contract=JSON.parse(input);
  console.log(JSON.stringify({schema:1,provenance,transportStatus:contract.status,
    caveats:"Existing fluid CPU and queue model only; historical motion/parser anchors and synthetic reserve, ACK after planner publication vs real parser admission, no real host/IRQ/safety/step timing. Numbered generated ASCII frames (LF), one illustrative empty TX IRQ per command. Planning allowance is a screen not a hard cap. Isolated costs are not qualified production costs. Travel is intentionally outside this extrusion matrix. Separate admissionSweep uses one live foreground command, ACK after synthetic parser stage, sensitivity split of unmeasured complete-parse/planner cost. Actual TX blocking, partial frame consumption and IRQ chronology still absent. wireFormatSweep uses actual-source validated format sizes but generated workload, historical ASCII partial receive cost applied to binary only as sensitivity, no measured binary parser cycle saving. Limit flags overlap, not definitive causal diagnosis or real CPU/IRQ proof.",selfTest:selfTest(contract),rows:matrix(contract),admissionSweep:admissionSweep(contract),wireFormatSweep:wireFormatSweep(contract)},null,2));
}
module.exports={provenance,parameters,matrix,admissionSweep,wireFormatSweep,selfTest};

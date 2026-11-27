// Integrate archived component measurements into the existing demand/queue model.
// No runtime policy, production changes, concurrent IRQ timing or WCET claim.
"use strict";
const {scenario:demand}=require("./demand-model.cjs");
const {scenario:queue}=require("./queue-model.cjs");
const {parameters,provenance:transport}=require("./transport-handoff.cjs");
const {plannerCosts}=require("./measured-cost-sweep.cjs");
const provenance={
  step:{path:"research/task0032-step-output/linked-baseline-summary.json",blob:"0f724a36bb5628b44b5fbfb9b63c223bc4bd34e4",head:"c1db668629ceebdb880399fdc8cf9e6f9583d910"},
  planner:{path:"research/task0032-planner-queue/REPORT.md",blob:"09ecfceaa57fdca77fb585de8c3b0dde6c919d93"},
  receiver:{run:45981098761,job:140444897048,head:"635aa34c877dfb3adb62beee95ff08d8c71a466c",
    // Stage 1 median, nine actual-source samples/format. Complete receiver
    // includes ACK enqueue. Do NOT subtract or add ACK stage 2 separately.
    baseline:{"ascii":13320,"binary-v1":4848,"binary-v2":5223},
    u16tail:{"ascii":12563,"binary-v1":3704,"binary-v2":4081}},
  transport
};
const vectors={X:"100, 0, 0, 0",XY:"100, 70, 0, 0",XYE:"100, 70, 0, 30",denseXYE:"100, 70, 0, 100",E:"0, 0, 0, 100"};
const boardE={"RF1000":280,"RF2000":280,"RF2000v2":256,"RF2000-X44":280};
function costPairs(input){
  if(input.schema!==1||input.head!==provenance.step.head||
     input.clock_hz!==16000000||input.classification!=="isolated_linked_instruction_model_not_concurrent_WCET")
    throw Error("wrong linked step evidence");
  const out={};
  for(const b of input.boards){
    if(!Object.hasOwn(boardE,b.board)||Object.hasOwn(out,b.board))throw Error("unexpected board");
    const cycles=b.paired_cruise_cycles, table={};
    for(const [cls,v] of Object.entries(vectors)){
      const pair=[1,2].map(n=>cycles["('active', ("+v+"), "+n+")"]);
      if(!pair.every(Number.isInteger)||!(pair[0]>0&&pair[1]>=pair[0]&&pair[1]<=2*pair[0]))throw Error("invalid linked pair");
      table[cls]=pair;
      // The sampled full vector includes the wrapper; validate it explicitly.
      for(const n of [1,2])if(pair[n-1]-cycles["('inner', ("+v+"), "+n+")"]!==238)throw Error("unexpected sampled wrapper");
    }
    out[b.board]=table;
  }
  if(Object.keys(out).length!==4)throw Error("four boards required");
  return out;
}
function sweep(contract,plannerReport,stepReport){
  const costs=costPairs(stepReport),pc=plannerCosts(plannerReport),rows=[];
  const lengths=[...new Set(contract.stress.map(r=>r.segment_mm))].sort((a,b)=>a-b);
  const speeds=[...new Set(contract.stress.map(r=>r.requested_mm_s))].sort((a,b)=>a-b);
  for(const board of Object.keys(boardE))for(const shape of ["straight","circle"])for(const chord of lengths)
    for(const speed of speeds)for(const packetFormat of Object.keys(provenance.receiver.baseline)){
      const spec={shape,count:120,radius:1.35,chord,speed,baud:contract.baud,ratio:.04,packetFormat};
      for(const receiver of ["baseline","u16tail"])for(const model of ["historical","paired"]){
        // Keep 20% aggregate maintenance reserve a sensitivity assumption.
        // Sticky ISR candidate: 83/71/61 measured structurally, not a deadline.
        const params={...parameters(contract,"budget"),rxIsr:83,txIsr:71,txEmptyPerCommand:61,
          planner:pc.median,receiverCycles:provenance.receiver[receiver][packetFormat],e:boardE[board],
          ...(model==="paired"?{activeCyclesByClass:costs[board]}:{})};
        const d=demand(spec,"adaptive",params);
        const q=queue(spec,"adaptive",params,{slots:24,frameWindow:32,ackPoint:"admission",parseFraction:0});
        const groupAbove2=Object.keys(d.groupHistogram).some(n=>Number(n)>2);
        // Distinguish no gaps after an already slowed profile from truly
        // reaching the requested nominal speed in this model.
        const requestMet=q.achievedMmPerSecond>=speed-1e-7;
        rows.push({board,spec,receiver,model,groupAbove2,
          demand:d.meanDemand,cpuFeasible:d.cpuFeasible,wireFeasible:d.linkFeasible,
          motionFeasible:q.validFluidMotion,requestMet,gaps:q.starveCount,achieved:q.achievedMmPerSecond,
          jointPass:d.cpuFeasible&&d.linkFeasible&&q.validFluidMotion&&requestMet&&q.starveCount===0,
          meanGroup:d.meanGroup,groupHistogram:d.groupHistogram,steps:q.steps});
      }
    }
  if(rows.length!==1440)throw Error("workload coverage");
  for(let i=0;i<rows.length;i+=4){
    const same=rows.slice(i,i+4);
    if(same.some(r=>JSON.stringify(r.steps)!==JSON.stringify(same[0].steps)))throw Error("costs changed step counts");
  }
  const summary=[];
  for(const board of Object.keys(boardE))for(const receiver of ["baseline","u16tail"])for(const model of ["historical","paired"]){
    const a=rows.filter(r=>r.board===board&&r.receiver===receiver&&r.model===model);
    summary.push({board,receiver,model,n:a.length,cpu:a.filter(r=>r.cpuFeasible).length,zeroGaps:a.filter(r=>!r.gaps).length,
      joint:a.filter(r=>r.jointPass).length,groupAbove2:a.filter(r=>r.groupAbove2).length,
      jointWithGroupAbove2:a.filter(r=>r.jointPass&&r.groupAbove2).length});
  }
  return {costs,plannerCycles:pc.median,summary,rows};
}
if(require.main===module){
  const fs=require("node:fs"),crypto=require("node:crypto");
  const pinned=(path,blob)=>{const a=fs.readFileSync(path);if(crypto.createHash("sha1").update("blob "+a.length+"\0").update(a).digest("hex")!==blob)throw Error("pinned evidence mismatch "+path);return a.toString();};
  const data=sweep(JSON.parse(pinned(transport.path,transport.blob)),pinned(provenance.planner.path,provenance.planner.blob),JSON.parse(pinned(provenance.step.path,provenance.step.blob)));
  console.log(JSON.stringify({schema:1,provenance,qualification:"Historical isolated samples, not matched workload timing. Full active group 1/2 for sampled vectors only. Other axis ratios, group>2 and tail costs are affine estimates. No Advance, Z, acceleration, IRQ chronology, physical pulse guarantee or final P0 qualification. 20% maintenance reserve hypothetical; model passes are not sustainable printer speeds.",...data},null,2));
}
module.exports={costPairs,sweep};

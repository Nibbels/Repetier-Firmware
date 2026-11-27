// Read actual-source JSONL, reuse pinned parent estimator module (not copied).
// node analyze.cjs trace.jsonl /path/to/queue-estimate.cjs > summary.json
"use strict";
const fs = require("node:fs");
const path = require("node:path");
const readline = require("node:readline");
const CPU = 16000000;
const methods = ["lower","corrected","cached","tangentBound","fastRoot"];
const expected = new Set(["publish","activate","retire","begin","end","blocked_front_successor","task_barrier","warmup_barrier","cleared"]);
function stats(){return {n:0,min:null,max:null,sum:0,over:0,within20:0};}
function add(s,error){
  if(!Number.isFinite(error))throw Error("non-finite sample");
  s.n++;s.min=s.min===null?error:Math.min(s.min,error);s.max=s.max===null?error:Math.max(s.max,error);
  s.sum+=error;s.over+=error>1e-6;s.within20+=Math.abs(error)<=.2;
}
function finish(s){return {n:s.n,min:s.min,max:s.max,mean:s.n?s.sum/s.n:null,over:s.over,within20:s.within20};}
const buckets={queue:{},active:{},queued:{},triangular:{}};
for(const b of Object.values(buckets))for(const m of methods)b[m]=stats();
function near(a,b){return Math.abs(a-b)<=Math.max(1e-7,Math.abs(b)*2e-5);}
function requireTrue(c,msg){if(!c)throw Error(msg);}
function candidateInput(m){
  return {total:m.total,vStart:m.vStart,vEnd:m.vEnd,vMax:m.vMax,acceleration:m.acceleration,accelSteps:m.accelSteps,decelSteps:m.decelSteps};
}
function prospective(row){
  // Next admission only, after newest reset. When full, cannot admit.
  if(row.count===row.capacity)return null;
  let limit=Math.min(1,row.count),ticks=0;
  while(limit<row.count&&ticks<45000){
    const s=row.slots[limit];
    // Warmup/task time semantics are not interchangeable with motion.
    if(s.special)return {status:"special_reserve_unknown"};
    ticks+=s.paced_ticks;limit++;
  }
  let first=row.count;
  // Actual implementation starts at hypothetical newest, then walks back.
  while(first!==limit&&(first===row.count||!row.slots[first].fixed_exit))first--;
  if(first!==row.count&&row.slots[first].fixed_exit)first++;
  const suffix=row.slots.slice(first);
  if(suffix.some(m=>m.special||m.blocked||!m.ready))return {status:"barrier"};
  return {status:"available",mutable_slots:suffix.length,
    mutable_mm:suffix.reduce((n,m)=>n+m.distance,0),
    incoming_speed_mm_s:suffix.length?suffix[0].entry:null,
    first_slot:suffix.length?suffix[0].slot:null,reserved_ticks:ticks};
}
const key=r=>[r.workload,r.capacity,r.chord,r.requested,r.baud,r.group,r.cap_mode,r.advance].join("/");
async function main(){
  const [file,modulePath]=process.argv.slice(2);
  if(!file||!modulePath)throw Error("usage: analyze.cjs trace.jsonl parent-estimator.cjs");
  const est=require(path.resolve(modulePath));
  est.selfTest();
  const cases={};let records=0,comparisons=0,zeroMutable=0,nonEmptyZeroMutable=0,maxReferenceDelta=0;
  let blockedCounterexample=null,largeNominalRatio=0;
  const lines=readline.createInterface({input:fs.createReadStream(file),crlfDelay:Infinity});
  for await(const line of lines){
    const r=JSON.parse(line);records++;
    requireTrue(expected.has(r.event),"unknown event "+r.event);
    const k=key(r);
    let c=cases[k];
    if(!c)c=cases[k]={workload:r.workload,capacity:r.capacity,chord:r.chord,requested:r.requested,baud:r.baud,group:r.group,cap_mode:r.cap_mode,advance:r.advance,publishes:0,activations:0,retires:0,max_count:0,wraps:0,zero_mutable:0,mutable_min_mm:null,mutable_max_mm:0,plan_totals:{},plan_max:{},entry_exit_mismatch:0,entry_exit_mismatch_non_boundary:0,entry_exit_mismatch_after_empty:0,entry_exit_examples:[],acceleration_distance_violation:0,max_full_speed:0,active_speed_jump_max:0,previous_activation:null};
    if(r.event==="begin"){c.fixture={step_per_mm:r.step_per_mm,max_feed:r.max_feed,xy_jerk:r.xy_jerk,z_jerk:r.z_jerk,e_jerk:r.e_jerk};continue;}
    if(r.event==="retire"){c.retires++;c.previous_retire_ticks=r.ticks;continue;}
    if(r.event==="end"){
      c.elapsed_ticks=r.ticks;c.idle_ticks=r.idle_ticks;c.xyz_mm=r.xyz_mm;c.mean_xyz_speed=r.xyz_mm*CPU/r.ticks;
      requireTrue(c.publishes===r.admitted&&c.retires===r.retired&&c.activations===r.retired,"event count mismatch "+k);
      continue;
    }
    requireTrue(r.count===r.slots.length,"slot count mismatch");
    requireTrue((r.read+r.count)%r.capacity===r.write,"ring mismatch");
    c.max_count=Math.max(c.max_count,r.count);
    let ref=0,contig=0,barrier=false,actualLower=0,fastContig=0;
    const values=Object.fromEntries(methods.map(m=>[m,0]));
    for(let i=0;i<r.slots.length;i++){
      const s=r.slots[i];
      requireTrue(s.slot===(r.read+i)%r.capacity,"slot order");
      barrier=barrier||s.special||s.blocked||!s.ready;
      if(s.special||s.left<=0||s.total<=0)continue;
      requireTrue(s.left<=s.total,"remaining > total");
      const m=candidateInput(s),v=est.reference(m,s.left);
      requireTrue(Number.isFinite(v)&&v>0,"invalid reference");
      ref+=v;
      for(const f of methods){
        const q=f==='fastRoot'?s.fast_s:est[f](m,s.left);
        values[f]+=q;
        add(buckets[s.active?"active":"queued"][f],(q-v)/v);
        if(m.accelSteps+m.decelSteps>=m.total&&m.accelSteps>0&&m.decelSteps>0)add(buckets.triangular[f],(q-v)/v);
      }
      actualLower+=est.lower(m,s.left);
      if(!barrier){contig+=est.lower(m,s.left);fastContig+=s.fast_s;}
    }
    requireTrue(near(fastContig,r.fast_contiguous_s),'fast contiguous mismatch');
    requireTrue(near(values.fastRoot,r.fast_s),'fast root slot sum mismatch');
    requireTrue(values.fastRoot<=r.reference_s*(1+2e-6)+1e-8,'fast root unexpectedly above reference');
    requireTrue(near(ref,r.reference_s),"parent JS reference != actual C++ queue "+k+" "+ref+" "+r.reference_s);
    requireTrue(near(actualLower,r.lower_s)&&near(contig,r.contiguous_lower_s),"proxy mismatch");
    maxReferenceDelta=Math.max(maxReferenceDelta,Math.abs(ref-r.reference_s));
    if(ref>0){comparisons++;for(const m of methods)add(buckets.queue[m],(values[m]-r.reference_s)/r.reference_s);}
    if(r.nominal_s>0)largeNominalRatio=Math.max(largeNominalRatio,r.reference_s/r.nominal_s);
    if(r.event==="blocked_front_successor"){
      requireTrue(r.contiguous_lower_s<r.lower_s,"blocked suffix counted as executable");
      blockedCounterexample={reference_s:r.reference_s,lower_s:r.lower_s,contiguous_lower_s:r.contiguous_lower_s};
    }
    const p=prospective(r);
    if(p?.status==="available"){
      if(!p.mutable_slots){zeroMutable++;if(r.count)nonEmptyZeroMutable++;c.zero_mutable++;}
      c.mutable_min_mm=c.mutable_min_mm===null?p.mutable_mm:Math.min(c.mutable_min_mm,p.mutable_mm);
      c.mutable_max_mm=Math.max(c.mutable_max_mm,p.mutable_mm);
    }
    if(r.event==="publish"){
      c.publishes++;
      if(r.write===0)c.wraps++;
      const plan=r.planning;
      for(const [f,v] of Object.entries(plan)){
        if(["first","limit"].includes(f))continue;
        c.plan_totals[f]=(c.plan_totals[f]||0)+v;
        c.plan_max[f]=Math.max(c.plan_max[f]||0,v);
      }
      requireTrue(plan.prepare===1&&plan.update===1,"unexpected call accounting");
      requireTrue(plan.back_visits===plan.forward_visits,"lookahead sweep mismatch");
      requireTrue(plan.back_visits<r.capacity,"lookahead beyond ring");
      const last=r.slots.at(-1);
      const fi=r.slots.findIndex(s=>s.slot===plan.first);
      requireTrue(fi>=0,"actual mutable start outside published ring");
      requireTrue(!r.slots.slice(fi).some(s=>s.active),"actual planner rewrote active slot");
      if(plan.no_window)requireTrue(fi===r.slots.length-1,"no-window contradicts first");
      requireTrue(last.ready&&!last.blocked,"published unready");
      const frozen=r.slots.slice(0,fi);
      c.max_frozen_mm=Math.max(c.max_frozen_mm||0,frozen.filter(s=>!s.special).reduce((a,s)=>a+s.distance*s.left/s.total,0));
    }
    if(r.event==="activate"){
      c.activations++;
      const s=r.slots[0],prev=c.previous_activation;
      requireTrue(s.active&&s.ready&&!s.blocked,"activation not executable");
      c.max_full_speed=Math.max(c.max_full_speed,s.full);
      if(Math.abs(s.exit*s.exit-s.entry*s.entry)>s.accel_distance2+0.02)c.acceleration_distance_violation++;
      if(prev){
        const mismatch=Math.abs(prev.exit-s.entry);
        const eOnly=m=>m.steps[3]!==0&&m.steps.slice(0,3).every(n=>n===0);
        const boundary=eOnly(prev)!==eOnly(s)||(r.advance&&Boolean(prev.steps[3])!==Boolean(s.steps[3]));
        if(mismatch>0.02){
          c.entry_exit_mismatch++;if(!boundary)c.entry_exit_mismatch_non_boundary++;
          const afterEmpty=r.ticks>c.previous_retire_ticks;
          if(afterEmpty)c.entry_exit_mismatch_after_empty++;
          if(!boundary&&c.entry_exit_examples.length<4)c.entry_exit_examples.push({id:s.id,previous_exit:prev.exit,entry:s.entry,previous_min:prev.min_speed,min:s.min_speed,previous_full:prev.full,full:s.full,after_empty:afterEmpty,ticks:r.ticks,previous_steps:prev.steps,steps:s.steps});
        }
        c.active_speed_jump_max=Math.max(c.active_speed_jump_max,Math.abs(prev.full-s.full));
      }
      c.previous_activation=s;
    }
  }
  requireTrue(comparisons>0&&blockedCounterexample,"missing estimator or barrier evidence");
  for(const c of Object.values(cases))delete c.previous_activation;
  const summary={schema:1,qualification:"actual extracted source with simulated arrivals/step clock; no AVR cycles, physical motion or governor",records,comparisons,maxReferenceDelta,zeroMutable,nonEmptyZeroMutable,blockedCounterexample,largeNominalRatio,
    relative_error:Object.fromEntries(Object.entries(buckets).map(([k,b])=>[k,Object.fromEntries(Object.entries(b).map(([m,s])=>[m,finish(s)]))])),
    cases:Object.values(cases)};
  console.log(JSON.stringify(summary,null,2));
}
main().catch(e=>{console.error(e);process.exitCode=1;});

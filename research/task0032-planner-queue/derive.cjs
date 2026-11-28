// node derive.cjs summary.json trace.jsonl > decision-input.json
"use strict";
const fs=require("node:fs");
const readline=require("node:readline");
const CPU=16000000;
function aggregate(rows,fn){return rows.reduce((a,r)=>a+fn(r),0);}
function range(rows,fn){return [Math.min(...rows.map(fn)),Math.max(...rows.map(fn))];}
function stat(){return {n:0,min:Infinity,max:-Infinity,sum:0,over:0,within20:0};}
function add(s,v){if(!Number.isFinite(v))throw Error("non-finite");s.n++;s.min=Math.min(s.min,v);s.max=Math.max(s.max,v);s.sum+=v;s.over+=v>1e-6;s.within20+=Math.abs(v)<=.2;}
function compact(s){return {n:s.n,min:s.min,max:s.max,mean:s.sum/s.n,over:s.over,within20:s.within20};}
function exactFull(m) {
  // Same analytic reference for a full slot; sqrt(vStart^2) is redundant.
  const a=Math.min(m.accelSteps,m.total),d=Math.min(m.decelSteps,m.total-a);
  let peak=Math.min(m.vMax,31250);
  if(a)peak=Math.min(peak,Math.sqrt(m.vStart*m.vStart+2*m.acceleration*a));
  peak=Math.max(1,peak);
  return (m.total-a-d)/peak+(a?2*a/(Math.min(peak,m.vStart)+peak):0)+(d?2*d/(peak+Math.min(peak,m.vEnd)):0);
}
function pick(c){
  return {capacity:c.capacity,chord:c.chord,requested:c.requested,baud:c.baud,group:c.group,cap_mode:c.cap_mode,advance:c.advance,
    mean_xyz_speed:c.mean_xyz_speed,idle_ms:c.idle_ticks*1000/CPU,max_count:c.max_count,zero_mutable:c.zero_mutable,
    mutable_max_mm:c.mutable_max_mm,max_frozen_mm:c.max_frozen_mm,
    max_full_speed:c.max_full_speed,full_speed_jump_max:c.active_speed_jump_max,entry_exit_mismatch:c.entry_exit_mismatch,
    non_boundary_mismatch:c.entry_exit_mismatch_non_boundary,acceleration_distance_violation:c.acceleration_distance_violation,
    replan_per_admission:c.plan_totals.profile_rebuild/c.publishes,back_visits_max:c.plan_max.back_visits,forward_visits_max:c.plan_max.forward_visits,reserve_visits_max:c.plan_max.reserve_visits,
    fixed_search_max:c.plan_max.fixed_search,no_window:c.plan_totals.no_window,extrusion_boundary:c.plan_totals.extrusion_boundary};
}
async function main(){
  const [summaryPath,tracePath]=process.argv.slice(2);
  const s=JSON.parse(fs.readFileSync(summaryPath));
  const all=s.cases.filter(c=>c.elapsed_ticks!==undefined);
  const caps=[...new Set(all.map(c=>c.capacity))];
  const capacity=caps.map(cap=>{
    const rows=all.filter(c=>c.capacity===cap),circles=rows.filter(c=>c.workload==="circle");
    const sums=f=>aggregate(rows,c=>c.plan_totals[f]||0);
    return {capacity:cap,cases:rows.length,publishes:aggregate(rows,c=>c.publishes),
      max_count:Math.max(...rows.map(c=>c.max_count)),wraps:aggregate(rows,c=>c.wraps),
      no_window:sums("no_window"),zero_mutable:aggregate(rows,c=>c.zero_mutable),
      profile_rebuild_per_admission:sums("profile_rebuild")/aggregate(rows,c=>c.publishes),
      back_visits_per_admission:sums("back_visits")/aggregate(rows,c=>c.publishes),
      back_visits_max:Math.max(...rows.map(c=>c.plan_max.back_visits)),
      forward_visits_max:Math.max(...rows.map(c=>c.plan_max.forward_visits)),
      reserve_visits_max:Math.max(...rows.map(c=>c.plan_max.reserve_visits)),
      fixed_search_max:Math.max(...rows.map(c=>c.plan_max.fixed_search)),
      circle_idle_ms_sum:aggregate(circles,c=>c.idle_ticks)*1000/CPU,
      circle_mean_speed_range:range(circles,c=>c.mean_xyz_speed),
      entry_exit_mismatch:aggregate(rows,c=>c.entry_exit_mismatch),
      non_boundary_mismatch:aggregate(rows,c=>c.entry_exit_mismatch_non_boundary),
      acceleration_distance_violation:aggregate(rows,c=>c.acceleration_distance_violation)};
  });
  const errors={paced:stat(),nominal:stat(),exactFullPlusReferenceActive:stat()};
  const transitions={};
  let activeFrozenMismatch=0;
  const lines=readline.createInterface({input:fs.createReadStream(tracePath),crlfDelay:Infinity});
  for await(const line of lines){
    const r=JSON.parse(line);
    if(!r.slots)continue;
    if(r.reference_s>0){
      add(errors.paced,(r.paced_proxy_s-r.reference_s)/r.reference_s);
      add(errors.nominal,(r.nominal_s-r.reference_s)/r.reference_s);
      // Exact full path + analytic active tail (only active has remaining steps).
      let estimate=0;
      for(const m of r.slots){
        if(m.special||m.total<=0||m.left<=0)continue;
        if(!m.active||m.left===m.total){estimate+=exactFull(m);continue;}
        const a=Math.min(m.accelSteps,m.total),d=Math.min(m.decelSteps,m.total-a),completed=m.total-m.left;
        const accelerating=Math.max(0,a-completed),braking=Math.min(m.left,d),cruise=m.left-accelerating-braking;
        let p=Math.min(m.vMax,31250);
        if(a)p=Math.min(p,Math.sqrt(m.vStart*m.vStart+2*m.acceleration*a));
        p=Math.max(1,p);
        const e=Math.min(p,m.vEnd),start=Math.min(p,Math.sqrt(m.vStart*m.vStart+2*m.acceleration*completed));
        estimate+=cruise/p+(accelerating?2*accelerating/(start+p):0)+(braking?2*braking/((braking===d?p:Math.sqrt(e*e+(p*p-e*e)*braking/d))+e):0);
      }
      add(errors.exactFullPlusReferenceActive,(estimate-r.reference_s)/r.reference_s);
    }
    if(r.workload!=="transition"||r.event!=="activate")continue;
    const m=r.slots[0];
    const k=[r.capacity,r.chord,r.group,r.cap_mode,r.advance].join("/");
    if(!transitions[k])transitions[k]={capacity:r.capacity,chord:r.chord,group:r.group,cap_mode:r.cap_mode,advance:r.advance,approach:[],first_curve:[],last_curve:[],exit:[]};
    const t=transitions[k];
    const point={id:m.id,ticks:r.ticks,entry:m.entry,exit:m.exit,full:m.full,cap:m.cap,remaining_steps:m.left,accel_distance2:m.accel_distance2,nominal_s:r.nominal_s,reference_s:r.reference_s};
    if(m.id>=3&&m.id<=6)t.approach.push(point);
    if(m.id>=7&&m.id<=10)t.first_curve.push(point);
    if(m.id>=99&&m.id<=102)t.last_curve.push(point);
    if(m.id>=103)t.exit.push(point);
    if(!m.fixed_exit)activeFrozenMismatch++;
  }
  if(activeFrozenMismatch)throw Error("active move not frozen");
  const result={schema:1,qualification:"HOST, scripted arrivals and actual source planner; no qualification of physical speed or AVR CPU",
    capacities:capacity,proxy_errors:Object.fromEntries(Object.entries(errors).map(([k,v])=>[k,compact(v)])),
    selected_cases:all.filter(c=>c.workload==="transition"||c.workload==="isolated_travel"||c.workload==="mixed"||(c.workload==="circle"&&c.requested===100&&c.chord<=.1)).map(pick),
    transitions:Object.values(transitions)};
  console.log(JSON.stringify(result,null,2));
}
main().catch(e=>{console.error(e);process.exitCode=1;});

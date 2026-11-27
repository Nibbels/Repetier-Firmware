// Compare research-only variants and starvation cases without a controller.
"use strict";
const fs=require("node:fs"),path=require("node:path");
const dir=process.argv[2];if(!dir)throw Error("usage: compare-variants.cjs artifact-directory");
const load=name=>JSON.parse(fs.readFileSync(path.join(dir,name+"summary.json"),"utf8"));
const baseline=load(""),stress=load("stress-");
const key=c=>[c.workload,c.chord,c.requested,c.baud,c.group,c.cap_mode,c.advance].join("/");
const rows=s=>s.cases.filter(c=>c.elapsed_ticks!==undefined);
const sum=(r,f)=>r.reduce((n,c)=>n+(c[f]||0),0);
const compact=c=>({workload:c.workload,chord:c.chord,requested:c.requested,baud:c.baud,group:c.group,cap_mode:c.cap_mode,advance:c.advance,mean_xyz_speed:c.mean_xyz_speed,idle_ms:c.idle_ticks/16000,max_count:c.max_count,zero_mutable:c.zero_mutable,mutable_max_mm:c.mutable_max_mm,entry_exit_mismatch:c.entry_exit_mismatch,non_boundary:c.entry_exit_mismatch_non_boundary,after_empty:c.entry_exit_mismatch_after_empty,accel_distance_violation:c.acceleration_distance_violation,max_full_speed:c.max_full_speed});
const base=new Map(rows(baseline).filter(c=>c.capacity===24).map(c=>[key(c),c]));
const variants={};
for(const name of ["unweighted","mean-length"]){
  const rr=rows(load(name+"-")),delta=[];
  for(const c of rr){
    const b=base.get(key(c));if(!b)throw Error("unmatched variant "+key(c));
    if(c.publishes!==b.publishes||c.xyz_mm!==b.xyz_mm)throw Error("different work");
    delta.push({case:key(c),baseline:compact(b),variant:compact(c),speed_ratio:c.mean_xyz_speed/b.mean_xyz_speed,profile_rebuild_delta:c.plan_totals.profile_rebuild-b.plan_totals.profile_rebuild});
  }
  if(delta.length!==base.size)throw Error("case count mismatch");
  const circles=delta.filter(c=>c.baseline.workload==="circle"),transitions=delta.filter(c=>c.baseline.workload==="transition");
  const range=a=>[Math.min(...a),Math.max(...a)];
  variants[name]={case_count:rr.length,circle_speed_ratio_range:range(circles.map(c=>c.speed_ratio)),transition_speed_ratio_range:range(transitions.map(c=>c.speed_ratio)),
    entry_exit_mismatch:sum(rr,"entry_exit_mismatch"),non_boundary_mismatch:sum(rr,"entry_exit_mismatch_non_boundary"),after_empty_mismatch:sum(rr,"entry_exit_mismatch_after_empty"),acceleration_distance_violation:sum(rr,"acceleration_distance_violation"),
    selected:delta.filter(c=>c.baseline.baud===250000&&c.baseline.group===1&&c.baseline.requested===100&&(c.baseline.workload==="circle"||c.baseline.workload==="transition")&&c.baseline.chord<.101).map(c=>({case:c.case,base:c.baseline.mean_xyz_speed,variant:c.variant.mean_xyz_speed,ratio:c.speed_ratio,base_horizon:c.baseline.mutable_max_mm,variant_horizon:c.variant.mutable_max_mm,base_mismatch:c.baseline.non_boundary,variant_mismatch:c.variant.non_boundary}))};
}
const capacities=[24,28,32].map(cap=>{
  const rr=rows(stress).filter(c=>c.capacity===cap),circle=rr.filter(c=>c.workload==="circle");
  return {capacity:cap,cases:rr.length,starved_cases:rr.filter(c=>c.idle_ticks>0).length,circle_idle_ms_sum:sum(circle,"idle_ticks")/16000,non_boundary_mismatch:sum(rr,"entry_exit_mismatch_non_boundary"),after_empty_mismatch:sum(rr,"entry_exit_mismatch_after_empty"),selected:circle.filter(c=>!c.advance).map(compact)};
});
console.log(JSON.stringify({schema:1,qualification:"HOST actual-source counterfactual only; not a physical junction safety or firmware change qualification",fastRoot:{queue:baseline.relative_error.queue.fastRoot,active:baseline.relative_error.active.fastRoot,queued:baseline.relative_error.queued.fastRoot,triangular:baseline.relative_error.triangular.fastRoot,stress:stress.relative_error.queue.fastRoot},baseline:{non_boundary_mismatch:sum([...base.values()],"entry_exit_mismatch_non_boundary"),after_empty_mismatch:sum([...base.values()],"entry_exit_mismatch_after_empty"),examples:[...base.values()].filter(c=>c.entry_exit_examples.length).slice(0,3).map(c=>({case:key(c),examples:c.entry_exit_examples}))},stress:capacities,variants},null,2));

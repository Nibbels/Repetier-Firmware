// Compare actual-source HOST replays. Disabled pacing is a counterfactual,
// not a proposed production speed policy; arrival schedule is modeled.
"use strict";
const fs=require("node:fs"),path=require("node:path"),readline=require("node:readline");
const dir=process.argv[2];if(!dir)throw Error("usage: ledger-results.cjs artifact-directory");
const load=name=>JSON.parse(fs.readFileSync(path.join(dir,name+"summary.json"),"utf8"));
const key=c=>[c.workload,c.capacity,c.chord,c.requested,c.baud,c.group,c.cap_mode,c.advance].join("/");
const rows=s=>s.cases.filter(c=>c.elapsed_ticks!==undefined);
const sum=(r,f)=>r.reduce((n,c)=>n+(c[f]||0),0);
function comparison(before,after){
    const map=new Map(rows(before).map(c=>[key(c),c]));
    const out=rows(after).map(c=>{
        const b=map.get(key(c));if(!b)throw Error("unmatched "+key(c));
        if(b.publishes!==c.publishes||b.xyz_mm!==c.xyz_mm)throw Error("changed geometry "+key(c));
        return {key:key(c),workload:c.workload,capacity:c.capacity,chord:c.chord,requested:c.requested,baud:c.baud,group:c.group,advance:c.advance,cap_mode:c.cap_mode,
            baseline_speed:b.mean_xyz_speed,no_pacing_speed:c.mean_xyz_speed,ratio:c.mean_xyz_speed/b.mean_xyz_speed,
            baseline_idle_ms:b.idle_ticks/16000,no_pacing_idle_ms:c.idle_ticks/16000,
            baseline_mismatch:b.entry_exit_mismatch_non_boundary,no_pacing_mismatch:c.entry_exit_mismatch_non_boundary,
            no_pacing_after_empty:c.entry_exit_mismatch_after_empty,no_pacing_accel_violation:c.acceleration_distance_violation,
            baseline_rebuilds:b.plan_totals.profile_rebuild,no_pacing_rebuilds:c.plan_totals.profile_rebuild};
    });
    if(out.length!==map.size)throw Error("incomplete comparison");
    return out;
}
async function ledger(file) {
    const lines=readline.createInterface({input:fs.createReadStream(path.join(dir,file)),crlfDelay:Infinity});
    const result={snapshots:0,active:0,max_abs_error_ms:0,max_abs_active_error_ms:0,mean_abs_error_ms:0,admissions:0,rebuilds:0,replans:0,empty:0};
    for await(const l of lines){
        const r=JSON.parse(l);
        if(r.event==="end"||r.event==="retire"||r.event==="begin")continue;
        if(!Number.isFinite(r.ledger_s))throw Error("missing ledger snapshot");
        const e=Math.abs(r.ledger_s-r.reference_s)*1000;
        result.snapshots++;result.mean_abs_error_ms+=e;
        result.max_abs_error_ms=Math.max(e,result.max_abs_error_ms);
        if(r.active){result.active++;result.max_abs_active_error_ms=Math.max(e,result.max_abs_active_error_ms);}
        if(r.count===0){result.empty++;if(r.ledger_full_units!==0)throw Error("nonzero empty ledger");}
        if(r.event==="publish"){result.admissions++;result.rebuilds+=r.planning.profile_rebuild;result.replans+=r.planning.profile_rebuild-1;}
    }
    result.mean_abs_error_ms/=result.snapshots;
    result.rebuilds_per_admission=result.rebuilds/result.admissions;
    return result;
}
async function main(){
    const base=await ledger("trace.jsonl"),stress=await ledger("stress.jsonl"),disabled=await ledger("no-pacing.jsonl"),disabledStress=await ledger("no-pacing-stress.jsonl");
    const normal= comparison(load(""),load("no-pacing-"));
    const gaps=comparison(load("stress-"),load("no-pacing-stress-"));
    function summary(rr){return [24,28,32].map(cap=>{
        const a=rr.filter(c=>c.capacity===cap),circle=a.filter(c=>c.workload==="circle");
        return {capacity:cap,cases:a.length,speed_ratio_min:Math.min(...a.map(c=>c.ratio)),speed_ratio_max:Math.max(...a.map(c=>c.ratio)),baseline_idle_cases:a.filter(c=>c.baseline_idle_ms>0).length,no_pacing_idle_cases:a.filter(c=>c.no_pacing_idle_ms>0).length,
            baseline_circle_idle_ms:sum(circle,"baseline_idle_ms"),no_pacing_circle_idle_ms:sum(circle,"no_pacing_idle_ms"),
            baseline_mismatch:sum(a,"baseline_mismatch"),no_pacing_mismatch:sum(a,"no_pacing_mismatch"),no_pacing_after_empty:sum(a,"no_pacing_after_empty"),no_pacing_accel_violation:sum(a,"no_pacing_accel_violation"),
            baseline_rebuilds:sum(a,"baseline_rebuilds"),no_pacing_rebuilds:sum(a,"no_pacing_rebuilds")};
    });}
    console.log(JSON.stringify({schema:1,qualification:"HOST actual-source planner hooks; modeled arrival/step clock; no concurrent IRQ timing or physical quality proof",ledger:{base,stress,disabled,disabledStress},pacing:{normal:summary(normal),stress:summary(gaps),normal_cases:normal,stress_cases:gaps}},null,2));
}
main().catch(e=>{console.error(e);process.exitCode=1;});

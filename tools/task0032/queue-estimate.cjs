// Task0032: cheap scheduled-motion estimate research, no production implementation.
// The reference is a JS translation of MoveQueue::estimateSeconds on pinned ade95d6.
// Synthetic coherent ramp profiles; not actual-source AVR replay or physical time.
"use strict";
const CPU=16000000, VMAX=31250;
function portions(m,left){
  const a=Math.min(m.accelSteps,m.total),d=Math.min(m.decelSteps,m.total-a);
  const completed=m.total-left, accelerating=Math.max(0,a-completed),braking=Math.min(left,d);
  return {a,d,completed,accelerating,braking,cruising:left-accelerating-braking};
}
function reference(m,left){
  const {a,d,completed,accelerating,braking,cruising}=portions(m,left);
  const s2=m.vStart*m.vStart, aa=2*m.acceleration;
  let peak=Math.min(m.vMax,VMAX);
  if(a)peak=Math.min(peak,Math.sqrt(s2+aa*a));
  peak=Math.max(1,peak);
  let t=cruising/peak;
  if(accelerating)t+=2*accelerating/(Math.min(peak,Math.sqrt(s2+aa*completed))+peak);
  if(braking){const end=Math.min(m.vEnd,peak),from=braking===d?peak:Math.sqrt(end*end+(peak*peak-end*end)*braking/d);t+=2*braking/(from+end);}
  return t;
}
// Uses existing vMax/remaining steps; only one division, no sqrt or added per-slot state.
// Floor at capped peak is a lower estimate relative to reference, not a physical deadline.
function lower(m,left){return left/Math.max(1,Math.min(m.vMax,VMAX));}
// Ramp correction using endpoint rate and linear interpolation of velocity in distance.
// For short triangular ramps vMax can exceed attained peak: explicit underestimation risk.
function corrected(m,left){
  const {a,d,completed,accelerating,braking,cruising}=portions(m,left);
  const peak=Math.max(1,Math.min(m.vMax,VMAX)),start=Math.min(m.vStart,peak),end=Math.min(m.vEnd,peak);
  // RMS >= arithmetic mean; integer-friendly affine estimate approximates the sqrt.
  let t=cruising/peak;
  if(accelerating)t+=2*accelerating/(start+(peak-start)*completed/a+peak);
  if(braking)t+=2*braking/(end+(peak-end)*braking/d+end);
  return t;
}
// Candidate cache: calculate full planned duration once after profile changes and
// scale active remaining by steps. Can overestimate or underestimate tail; never a safety bound.
function cached(m,left){return reference(m,m.total)*left/m.total;}
function profile(total,vStart,vEnd,vMax,acceleration){
  let a=Math.max(0,Math.ceil((vMax*vMax-vStart*vStart)/(2*acceleration)));
  let d=Math.max(0,Math.ceil((vMax*vMax-vEnd*vEnd)/(2*acceleration)));
  if(a+d>total){
    a=Math.round((2*acceleration*total+vEnd*vEnd-vStart*vStart)/(4*acceleration));
    a=Math.max(0,Math.min(total,a));d=total-a;
  }
  return {total,vStart,vEnd,vMax,acceleration,accelSteps:a,decelSteps:d};
}
function matrix(){
  const rows=[];
  for(const total of [5,15,30,90,300,1500,15000])for(const vMax of [2000,8000,16000,31250,40000])
    for(const acceleration of [20000,80000,250000])for(const sf of [.1,.5,1])for(const ef of [.1,.5,1]){
      const vStart=Math.round(vMax*sf),vEnd=Math.round(vMax*ef);
      // Reject impossible entry/exit pairs under the selected distance/acceleration.
      if(Math.abs(vStart*vStart-vEnd*vEnd)>2*acceleration*total)continue;
      const m=profile(total,vStart,vEnd,vMax,acceleration);
      for(const left of [...new Set([total,Math.max(1,Math.ceil(total*.75)),Math.max(1,Math.ceil(total*.5)),Math.max(1,Math.ceil(total*.25)),1])])
        rows.push({m,left,reference:reference(m,left),lower:lower(m,left),corrected:corrected(m,left),cached:cached(m,left)});
    }
  return rows;
}
function summarize(rows){
  const result={cases:rows.length};
  for(const k of ["lower","corrected","cached"]){
    const errors=rows.map(r=>({relative:(r[k]-r.reference)/r.reference,row:r})).sort((a,b)=>a.relative-b.relative);
    result[k]={min:errors[0],max:errors.at(-1),median:errors[Math.floor(errors.length/2)].relative,within20:errors.filter(x=>Math.abs(x.relative)<=.2).length,over:errors.filter(x=>x.relative>1e-10).length};
  }return result;
}
function selfTest(){
  const m=profile(100,1000,1000,1000,20000);
  if(reference(m,50)!==.05||lower(m,50)!==.05||corrected(m,50)!==.05)throw Error("cruise");
  const rows=matrix();
  if(rows.some(r=>!Number.isFinite(r.reference)||r.reference<=0||r.lower>r.reference+1e-12))throw Error("lower bound");
  if(rows.some(r=>Math.abs(cached(r.m,r.m.total)-reference(r.m,r.m.total))>1e-12))throw Error("cache full");
  return "3 groups of checks passed";
}
if(require.main===module){
  if(process.argv[2]==="--self-test")console.log(selfTest());
  else console.log(JSON.stringify({schema:1,baseline:"ade95d627df5d82bb585413b678ffebe6e8a946a",selfTest:selfTest(),...summarize(matrix())},null,2));
}
module.exports={profile,reference,lower,corrected,cached,matrix,summarize,selfTest};

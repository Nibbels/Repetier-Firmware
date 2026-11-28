// Task0034 A22: offline sensitivity study, NOT device calibration.
// Parameter grid is hypothetical, not a list of qualified heaters.
// DC/AC differences averaged over integration step; actual pulse/line
// phase and scheduling/fresh ADC safety are separate HAL tests.
// PID equations mirror TemperatureControl.cpp (double precision here).
"use strict";
const clamp=(v,l,h)=>Math.max(l,Math.min(h,v));
const hotendGains=[
  {name:"v1",kp:37.52,ki:10,kd:35.18,imax:130,imin:30},
  {name:"v2",kp:12.5,ki:3.2,kd:18,imax:120,imin:30},
  {name:"v3",kp:17.11,ki:2.05,kd:35.75,imax:120,imin:30}
];
const bedGains=[
  {name:"bed-53",kp:53.74,ki:7.48,kd:96.52,imax:80,imin:5},
  {name:"bed-34",kp:34,ki:.015,kd:110,imax:80,imin:5}
];
function pid(s,p,target,temp,dt,cutoff){
  // Actual regulation equations: derivative on measured temperature,
  // elapsed time, low-pass 0.5s, bounded integral and anti-windup.
  const history=s.valid&&dt>0&&dt<=1;
  if(history)s.slope+=dt/(.5+dt)*((temp-s.previous)/dt-s.slope);
  else s.slope=0;
  s.previous=temp;s.valid=true;
  if(target<40){s.i=0;return 0;} // here target is either 0 or >=90
  const error=target-temp, pd=p.kp*error-p.kd*s.slope;
  const candidate=clamp(s.i+p.ki*error*(history?dt:0),-Math.min(p.imin,p.limit),Math.min(p.imax,p.limit));
  if((pd+candidate<=p.limit||error<0)&&(pd+candidate>=0||error>0))s.i=candidate;
  return temp>cutoff?0:Math.trunc(clamp(pd+s.i,0,p.limit));
}
// Two energy-conserving thermal masses plus a lagged sensor measurement.
// Watts, J/K, W/K, seconds, C. Explicit Euler 25/50ms convergence check.
function simulate(p,fault="none",h=.05){
  const ambient=25, cutoff=p.kind==="hotend"?275:180;
  const target=p.kind==="hotend"?200:90;
  const off=p.kind==="hotend"?480:660, end=off+120, sampleSteps=Math.round(.1/h);
  const state={valid:false,slope:0,previous:ambient,i:0};
  let heater=ambient,body=ambient,sensor=ambient,request=0, measured=ambient;
  let maxSlope=-Infinity,firstRise=null,peak=ambient,holdError=0,offPeak=0,offBase=ambient;
  let energy=0,offEnergy=0,firstBand=null,oldTrip=null,cutoffTrip=null,oldMode=0,oldTime=0,oldTemp=ambient,errors=0;
  let frozen=ambient,prevSec=ambient,off10Rise=null;
  const faultAt=fault==="stuck-on"?off:30;
  for(let n=0;n<=Math.round(end/h);n++){
    const t=n*h,sp=t<off?target:0,failed=t>=faultAt;
    if(n%sampleSteps===0){
      if(fault==="frozen"&&failed){measured=frozen;}
      else if(fault==="detached"&&failed){measured=ambient+(frozen-ambient)*Math.exp(-(t-faultAt)/5);}
      else{measured=sensor;frozen=sensor;}
      // Readings quantized only to 0.1 C for this illustrative sensor.
      measured=Math.round(measured*10)/10;
      if(t===off){offBase=measured;offPeak=measured;oldMode=0;errors=0;state.valid=false;state.i=0;}
      if(n%Math.round(1/h)===0){
        if(t>0&&t<off)maxSlope=Math.max(maxSlope,measured-prevSec);
        prevSec=measured;
      }
      if(Math.abs(t-12)<h/2)firstRise=measured-ambient;
      if(t<off&&firstBand===null&&Math.abs(measured-target)<=1)firstBand=t;
      if(t>=off-30&&t<off)holdError=Math.max(holdError,Math.abs(measured-target));
      if(t>=off){offPeak=Math.max(offPeak,measured);if(Math.abs(t-off-10)<h/2)off10Rise=measured-offBase;}
      if(cutoffTrip===null&&measured>cutoff)cutoffTrip=t;
      // Observational single-channel replica of existing decouple checks.
      // No emergency feedback here: compare detectors on the SAME trace.
      let error=false;
      if(oldMode&&t-oldTime>12+1e-8){
        if(oldMode===1){
          error=measured-oldTemp<1;
          if(!error){oldTime=t;oldTemp=measured;}
        }else{
          error=Math.abs(measured-sp)>20;
          if(!error)oldTime=t-11;
        }
      }
      errors=error?errors+1:Math.max(0,errors-1);
      if(errors>10&&oldTrip===null)oldTrip=t;
      request=pid(state,p,sp,measured,.1,cutoff);
      if(sp===0)oldMode=0;
      else if(oldMode!==2&&sp-measured>20&&request>0){
        if(oldMode!==1){oldMode=1;oldTime=t;oldTemp=measured;}
      }else if(oldMode!==2&&Math.abs(sp-measured)<=19){
        oldMode=2;oldTime=t;oldTemp=sp;
      }
    }
    peak=Math.max(peak,body);
    if(n===Math.round(end/h))break;
    // Legacy mask/full-duty convention, average power; compare-clear
    // maximum may instead be 254/256, a separate actuator study.
    const duty=request>=254?1:(request&254)/256;
    const power=p.watts*(fault==="stuck-off"&&failed?0:fault==="stuck-on"&&failed?1:duty);
    // Fan/filament load step during hold, plus environmental heat losses.
    const cooling=t>=off-90&&t<off-30?p.disturbance:0;
    const transfer=p.coupling*(heater-body);
    const heaterLoss=p.loss*.15*(heater-ambient);
    const bodyLoss=p.loss*.85*(body-ambient)+cooling;
    heater+=h*(power-transfer-heaterLoss)/p.heaterC;
    body+=h*(transfer-bodyLoss)/p.bodyC;
    sensor+=h*(body-sensor)/p.sensorTau;
    energy+=h*power;if(t>=off)offEnergy+=h*power;
    if(!Number.isFinite(body)||!Number.isFinite(heater))throw Error("unstable numerical model");
  }
  return {firstRise,maxSlope,firstBand,holdError,offRebound:offPeak-offBase,off10Rise,peak,energy,offEnergy,oldTrip,cutoffTrip,
          settled:firstBand!==null&&holdError<=5};
}
function grid(){
  const cases=[];
  for(const kind of ["hotend","bed"]){
    const hot=kind==="hotend";
    for(const watts of hot?[15,25,40,60]:[300,450,560])
    for(const bodyC of hot?[6,12,24]:[300,600,1000])
    for(const coupling of hot?[2,6]:[8,20])
    for(const loss of hot?[.035,.07]:[2,4])
    for(const sensorTau of hot?[.25,1.5]:[1,6])
    for(const gain of hot?hotendGains:bedGains)
    for(const limit of [128,255])
      cases.push({kind,watts,bodyC,coupling,loss,sensorTau,heaterC:hot?1.5:40,
                  disturbance:hot?2:30,limit,...gain});
  }
  return cases;
}
const range=(xs,k)=>[Math.min(...xs.map(x=>x[k])),Math.max(...xs.map(x=>x[k]))].map(x=>+x.toFixed(3));
function main(){
  const params=grid(),rows=params.map(p=>({p,...simulate(p)}));
  const result={assumptions:"Illustrative lumped thermal model; no measured plant parameters, no actual hardware safety limits; average duty, no ADC or AC phase replay",cases:rows.length,groups:[],faults:[]};
  for(const kind of ["hotend","bed"]){
    const r=rows.filter(x=>x.p.kind===kind),s=r.filter(x=>x.settled);
    result.groups.push({kind,cases:r.length,settled:s.length,unsettled:r.length-s.length,
        legacyTrips:r.filter(x=>x.oldTrip!==null).length,settledLegacyTrips:s.filter(x=>x.oldTrip!==null).length,
        settledRise12:range(s,"firstRise"),settledMaxSlope:range(s,"maxSlope"),
        settledOffRebound:range(s,"offRebound"),settledFirstBand:range(s,"firstBand")});
    // Representative settled cases per installed power, slowest and fastest
    // observed rise. Disturbance faults are replayed on identical plants.
    for(const watts of [...new Set(s.map(x=>x.p.watts))]){
      const group=s.filter(x=>x.p.watts===watts).sort((a,b)=>a.firstRise-b.firstRise);
      for(const row of [group[0],group[group.length-1]]){
        const half=simulate(row.p,"none",.025);
        const convergence={rise12:Math.abs(half.firstRise-row.firstRise),peak:Math.abs(half.peak-row.peak)};
        const faults={};
        for(const fault of ["stuck-off","frozen","detached","stuck-on"]){
          const x=simulate(row.p,fault);
          faults[fault]={oldTrip:x.oldTrip,cutoffTrip:x.cutoffTrip,peak:+x.peak.toFixed(2),off10Rise:x.off10Rise,offEnergy:+x.offEnergy.toFixed(1)};
        }
        result.faults.push({p:row.p,normal:{rise12:row.firstRise,firstBand:row.firstBand,offRebound:row.offRebound,holdError:row.holdError},convergence,faults});
      }
    }
  }
  return result;
}
module.exports={simulate,pid,grid,main};
if(require.main===module)console.log(JSON.stringify(main(),null,2));

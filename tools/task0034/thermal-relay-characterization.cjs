// Task0034 A36: offline relay characterization, not physical calibration.
// Hypothetical plants, average PWM, quantized sensor, 100 ms sampled relay.
// No EEPROM writes, runtime thresholds or claim of qualified hardware.
'use strict';
const ambient=25, hysteresis=.375;
const rules=[[.45,1.6,.08],[1/2.2,2.2,1/6.3],[.65,1.6,.16],[.45,3,.08]];
function gains(ku,tu,kind){
  return Object.fromEntries((kind==='hotend'?[0,1]:[2,3]).map(j=>{
    const [p,i,d]=rules[j], kp=p*ku;
    return ['J'+j,{kp,ki:kp/(i*tu),kd:kp*d*tu}];
  }));
}
function simulate(p,h=.025,limitCycles=8){
  const hot=p.kind==='hotend', target=hot?200:90, cutoff=target+(hot?20:15);
  const sampleSteps=Math.round(.1/h), maxSteps=Math.round((hot?1200:3600)/h);
  if(Math.abs(sampleSteps*h-.1)>1e-10)throw Error('sample grid');
  let heater=ambient,body=ambient,sensor=ambient,measured=ambient;
  let heating=true,cycleStart=0,phaseStart=0,riseStart=0,riseTemp=ambient,low=ambient,high=ambient;
  let onTime=0,cycles=0,stable=0,kuSum=0,tuSum=0,prevKu=0,prevTu=0;
  let result='total-timeout',t=0,firstOff=null,activeOff=null,maxRebound=0,maxPeakDelay=0;
  let maxRise1=0,maxFall1=0,maxBody=ambient,maxHeater=ambient,energy=0,accepted=[];
  const history=Array(10).fill(ambient);
  function updateOff(){
    if(!activeOff)return;
    if(measured>activeOff.peak){activeOff.peak=measured;activeOff.peakAt=t;}
  }
  function finishOff(){
    if(!activeOff)return;
    const rebound=activeOff.peak-activeOff.base, delay=activeOff.peakAt-activeOff.start;
    maxRebound=Math.max(maxRebound,rebound);maxPeakDelay=Math.max(maxPeakDelay,delay);
    if(!firstOff)firstOff={start:activeOff.start,rebound,peakDelay:delay,completed:true};
    activeOff=null;
  }
  for(let n=0;n<=maxSteps;n++){
    t=n*h;
    if(n%sampleSteps===0){
      // Integer sample clock avoids float equality at transition boundaries.
      t=Math.round(n/sampleSteps)*.1;
      measured=Math.round(sensor*10)/10;
      const idx=(n/sampleSteps)%10,delta=measured-history[idx];
      if(n>=10*sampleSteps){maxRise1=Math.max(maxRise1,delta);maxFall1=Math.max(maxFall1,-delta);}
      history[idx]=measured;
      updateOff();
      if(measured>cutoff){result='relative-cutoff';break;}
      if(t-phaseStart>(hot?180:600)){result='phase-timeout';break;}
      low=Math.min(low,measured);high=Math.max(high,measured);
      if(heating&&measured>=target+hysteresis){
        heating=false;onTime=t-cycleStart;phaseStart=t;
        activeOff={start:t,base:measured,peak:measured,peakAt:t};
      }else if(heating&&t-riseStart>=(hot?20:60)-1e-8){
        if(measured-riseTemp<1){result='no-rise';break;}
        riseStart=t;riseTemp=measured;
      }else if(!heating&&measured<=target-hysteresis){
        finishOff();++cycles;
        const period=t-cycleStart,fraction=onTime/period,span=high-low;
        const ku=4*p.limit*Math.sin(Math.PI*fraction)/(Math.PI*span),tu=period;
        const usable=cycles>2&&period>=1&&span>=.75&&fraction>=.05&&fraction<=.95&&ku>0&&ku<=1000&&tu>0;
        if(usable){
          if(stable&&(Math.abs(ku-prevKu)>.15*prevKu||Math.abs(tu-prevTu)>.15*prevTu)){
            stable=0;kuSum=tuSum=0;accepted=[];
          }
          if(!stable){prevKu=ku;prevTu=tu;}
          ++stable;kuSum+=ku;tuSum+=tu;accepted.push({period,span,fraction});
        }else{stable=0;kuSum=tuSum=0;accepted=[];}
        if(stable>=3&&cycles>=limitCycles){result='converged';break;}
        if(cycles>=limitCycles){result='nonconvergent';break;}
        heating=true;phaseStart=cycleStart=riseStart=t;riseTemp=low=high=measured;
      }
    }
    if(n===maxSteps)break;
    const power=heating?p.watts*(p.limit>=254?1:(p.limit&254)/256):0;
    const transfer=p.coupling*(heater-body);
    heater+=h*(power-transfer-p.loss*.15*(heater-ambient))/p.heaterC;
    body+=h*(transfer-p.loss*.85*(body-ambient))/p.bodyC;
    sensor+=h*(body-sensor)/p.sensorTau;
    energy+=h*power;maxBody=Math.max(maxBody,body);maxHeater=Math.max(maxHeater,heater);
    if(!Number.isFinite(body)||!Number.isFinite(heater)||!Number.isFinite(sensor))throw Error('unstable plant');
  }
  // An interrupted off phase is a censored observation, not a completed
  // rebound envelope. Record separately; never count it as calibrated.
  const partialOff=activeOff?{start:activeOff.start,rebound:activeOff.peak-activeOff.base,peakDelay:activeOff.peakAt-activeOff.start,completed:false}:null;
  const tuned=result==='converged'?gains(kuSum/stable,tuSum/stable,p.kind):null;
  return {result,elapsed:t,cycles,stable,firstOff,partialOff,maxRebound,maxPeakDelay,maxRise1,maxFall1,maxBody,maxHeater,energy,accepted,tuned};
}
function grid(){
  const rows=[];
  for(const kind of ['hotend','bed']){
    const hot=kind==='hotend';
    for(const watts of hot?[30,60,100]:[300,450,600])
    for(const bodyC of hot?[2,6,18]:[300,800])
    for(const coupling of hot?[4,12]:[8,20])
    for(const loss of hot?[.04,.07]:[2,4])
    for(const sensorTau of hot?[.15,.75,1.5]:[1,5])
    for(const limit of [128,255])rows.push({kind,watts,bodyC,coupling,loss,sensorTau,limit,heaterC:hot?.75:40});
  }
  return rows;
}
const range=(xs,k)=>xs.length?[Math.min(...xs.map(x=>x[k])),Math.max(...xs.map(x=>x[k]))].map(x=>+x.toFixed(3)):null;
function main(){
  const rows=grid().map(p=>({p,...simulate(p)})),groups=[];
  for(const kind of ['hotend','bed'])for(const watts of [...new Set(rows.filter(r=>r.p.kind===kind).map(r=>r.p.watts))]){
    const r=rows.filter(x=>x.p.kind===kind&&x.p.watts===watts),s=r.filter(x=>x.result==='converged');
    const counts={};for(const x of r)counts[x.result]=(counts[x.result]||0)+1;
    for(const x of s){if(x.stable<3||!x.firstOff||!x.tuned||Object.values(x.tuned).some(g=>!(g.kp>0&&g.kp<=1000&&g.ki>0&&g.ki<=1000&&g.kd>=0&&g.kd<=100000)))throw Error('invalid accepted profile');}
    groups.push({kind,watts,cases:r.length,counts,convergedRise1:range(s,'maxRise1'),convergedFall1:range(s,'maxFall1'),convergedRebound:range(s,'maxRebound'),convergedPeakDelay:range(s,'maxPeakDelay')});
  }
  const representatives=[];
  for(const kind of ['hotend','bed']){
    const s=rows.filter(x=>x.p.kind===kind&&x.result==='converged').sort((a,b)=>a.maxRise1-b.maxRise1);
    const rejected=rows.filter(x=>x.p.kind===kind&&x.result==='relative-cutoff').sort((a,b)=>b.maxRise1-a.maxRise1);
    for(const row of [s[0],s[s.length-1],rejected[0]].filter(Boolean)){
      const half=simulate(row.p,.0125);
      representatives.push({p:row.p,result:row.result,elapsed:row.elapsed,firstOff:row.firstOff,partialOff:row.partialOff,maxRise1:row.maxRise1,maxRebound:row.maxRebound,maxPeakDelay:row.maxPeakDelay,tuned:row.tuned,halfStep:{result:half.result,maxRise1:half.maxRise1,maxRebound:half.maxRebound,maxBodyDelta:Math.abs(half.maxBody-row.maxBody)}});
    }
  }
  return {assumptions:'Hypothetical lumped plants, 0.1C quantized lagged sensor, average commanded PWM, eight cycles, no actual hardware or firmware timing qualification',cases:rows.length,groups,representatives};
}
module.exports={simulate,grid,gains,main};
if(require.main===module)console.log(JSON.stringify(main(),null,2));

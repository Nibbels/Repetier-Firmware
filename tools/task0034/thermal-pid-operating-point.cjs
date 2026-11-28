'use strict';
// A57 research only: nonlinear radiation sensitivity; no actual heater measurements or policy.
// Loss normalized at 200 C; 50% radiation is a hypothetical sensitivity case, not fitted emissivity.
function study(steps,source){
 if(steps!==4&&steps!==8)throw Error("steps");
 const f=Math.fround,clamp=(v,l,h)=>Math.max(l,Math.min(h,v)),frac=d=>d>=254?1:(d&254)/256;
 const quantize=sensorModel(source),temps=[100,200,250];
 const lossLaw=(t,r)=>{const a=298.15,k=t+273.15,ref=473.15;return (1-r)*(t-25)+r*175*((k/a)**4-1)/((ref/a)**4-1);};
 function plant(p,initial=25){
  let h=initial,b=initial,s=initial;
  return {advance(power,load,dt){for(let i=0;i<steps;i++){
   const tr=p.coupling*(h-b);
   h+=dt*(power-tr-p.loss*.15*lossLaw(h,p.rad))/.75;
   b+=dt*(tr-p.loss*.85*lossLaw(b,p.rad)-load)/p.mass;
   s+=dt*(b-s)/p.tau;
  }},sensor(){return s},body(){return b},heater(){return h}};
 }
 function train(p,target){
  const m=plant(p),cutoff=Math.min(275,target+20);
  let heat=true,start=0,phase=0,rise=0,riseT=25,lo=25,hi=25,on=0,cycles=0,stable=0,kuSum=0,tuSum=0,prevKu=0,prevTu=0;
  for(let n=1;n<=12000;n++){
   m.advance(heat?p.w*frac(p.cap):0,0,.1/steps);
   const t=Math.round(m.sensor()*10)/10;
   if(!Number.isFinite(t)||t>cutoff||n-phase>1800)return null;
   lo=Math.min(lo,t);hi=Math.max(hi,t);
   if(heat&&t>=target+.375){heat=false;on=n-start;phase=n;}
   else if(heat&&n-rise>=200){if(t-riseT<1)return null;rise=n;riseT=t;}
   else if(!heat&&t<=target-.375){
    cycles++;const period=n-start,ratio=on/period,span=hi-lo,ku=4*p.cap*Math.sin(Math.PI*ratio)/(Math.PI*span),tu=period*.1;
    if(cycles>2&&period>=10&&span>=.75&&ratio>=.05&&ratio<=.95&&ku>0&&ku<=1000){
     if(stable&&(Math.abs(ku-prevKu)>.15*prevKu||Math.abs(tu-prevTu)>.15*prevTu)){stable=0;kuSum=tuSum=0;}
     if(!stable){prevKu=ku;prevTu=tu;}
     stable++;kuSum+=ku;tuSum+=tu;
    }else{stable=0;kuSum=tuSum=0;}
    if(cycles===8)return stable>=3?{ku:kuSum/stable,tu:tuSum/stable}:null;
    heat=true;start=phase=rise=n;riseT=lo=hi=t;
   }
  }return null;
 }
 function replay(p,c,target,method){
  const m=plant(p),isP=method==="PessenLag",kp=f((isP?.7:1/2.2)*c.ku),ki=f(kp/((isP?.4:2.2)*c.tu)),kd=f(kp*(isP?.15:1/6.3)*c.tu);
  const maxPower=p.w*frac(p.cap);
  // Bisection for steady heater temperature at target body temperature.
  function powerFor(load){const transfer=p.loss*.85*lossLaw(target,p.rad)+load;return transfer+p.loss*.15*lossLaw(target+transfer/p.coupling,p.rad);}
  const feasible=powerFor(0)<maxPower;
  let low=0,high=maxPower;
  for(let i=0;i<32;i++){const mid=(low+high)/2;if(powerFor(mid)>.85*maxPower)high=mid;else low=mid;}
  const load=(low+high)/2;
  let I=0,slope=0,previous=25,reference=25,duty=0,trip=false,overshoot=0,release=0,hold=0,ia=0,basePeak=0,bodyPeak=25,matPeak=25;
  const stage=2400,end=stage*3,window=300;
  for(let n=1;n<=end;n++){
   const fan=n>stage&&n<=stage*2;
   m.advance(p.w*frac(duty),fan?load*Math.max(0,m.body()-25)/(target-25):0,.1/steps);
   const t=quantize(m.sensor(),8,1,n,fan);
   const dt=n===1?0:.1;
   if(t>275)trip=true;
   slope=dt>0?f(slope+f(f(dt/(.5+dt))*f(f((t-previous)/dt)-slope))):0;
   previous=t;reference=isP?f(reference+f(f(dt/(c.tu+dt))*f(target-reference))):target;
   const e=f(reference-t),pd=f(f(kp*e)-f(kd*slope)),candidate=f(clamp(f(I+f(f(ki*e)*dt)),-Math.min(20,p.cap),p.cap));
   if((f(pd+candidate)<=p.cap||e<0)&&(f(pd+candidate)>=0||e>0))I=candidate;
   duty=trip?0:Math.trunc(clamp(f(pd+I),0,p.cap));
   if(n<=stage)overshoot=Math.max(overshoot,t-target);
   if(n>2*stage)release=Math.max(release,t-target);
   if(n>2*stage-window&&n<=2*stage)hold+=Math.abs(target-t)/window;
   if(n<=stage)basePeak=Math.max(basePeak,m.sensor()-target);
   ia+=Math.abs(target-t)*.1;bodyPeak=Math.max(bodyPeak,m.body());matPeak=Math.max(matPeak,m.heater());
  }
  return {feasible,trip,overshoot,release,hold,ia,basePeak,bodyPeak,matPeak};
 }
 const cohorts={},rows={},gains={};let traces=0;
 function add(key,r){const a=rows[key]??={n:0,trips:0,within1:0,overMax:0,releaseMax:0,holdMean:0,iaeMean:0,bodyMax:0,heaterMax:0};a.n++;a.trips+=+r.trip;a.within1+=+(r.hold<=1);a.overMax=Math.max(a.overMax,r.overshoot);a.releaseMax=Math.max(a.releaseMax,r.release);a.holdMean+=r.hold;a.iaeMean+=r.ia;a.bodyMax=Math.max(a.bodyMax,r.bodyPeak);a.heaterMax=Math.max(a.heaterMax,r.matPeak);}
 for(const rad of [0,.5]){
  const cohort=cohorts[rad]={plants:0,converged:[0,0,0],matched:0};
  for(const w of [30,60,100])for(const mass of [2,6,18])for(const coupling of [4,12])for(const loss of [.04,.07])for(const tau of [.15,.75,1.5])for(const cap of [128,255]){
   const p={w,mass,coupling,loss,tau,cap,rad};cohort.plants++;
   const c=temps.map(t=>train(p,t));c.forEach((v,i)=>{if(v)cohort.converged[i]++});
   if(c.some(v=>!v))continue;
   cohort.matched++;
   for(let i=0;i<3;i++){const key=rad+"/"+temps[i],g=gains[key]??={n:0,kuRatio:[],tuRatio:[]};g.n++;g.kuRatio.push(c[i].ku/c[1].ku);g.tuRatio.push(c[i].tu/c[1].tu);}
   for(let i=0;i<3;i++)for(const target of temps)for(const method of ["PessenLag","TL"]){
    const r=replay(p,c[i],target,method);traces++;add([rad,temps[i],target,method,r.feasible?"feasible":"limited"].join("/"),r);
   }
  }
 }
 function describe(v){v.sort((a,b)=>a-b);return {min:v[0],median:v[Math.floor(v.length/2)],max:v[v.length-1]};}
 for(const g of Object.values(gains)){g.kuRatio=describe(g.kuRatio);g.tuRatio=describe(g.tuRatio);}
 for(const a of Object.values(rows)){a.holdMean/=a.n;a.iaeMean/=a.n;}
 return {steps,cohorts,traces,gains,rows};
}
function sensorModel(source){
 const clean=source.replace(/\/\*[\s\S]*?\*\//g,"").replace(/\/\/[^\n]*/g,"");
 const values=x=>{if(!/^\s*-?\d+(?:\s*\*\s*-?\d+)*\s*$/.test(x))throw Error(x);return x.split("*").reduce((n,y)=>n*Number(y),1)};
 const luts={};
 for(const m of clean.matchAll(/const short temptable_(\d+)\[NUMTEMPS_\d+\]\[2\] PROGMEM\s*=\s*\{([\s\S]*?)\};/g)){
  const id=+m[1];if(![3,4,8,13].includes(id))continue;
  const p=[...m[2].matchAll(/\{\s*([^,{}]+),\s*([^,{}]+)\}/g)].map(x=>[values(x[1]),values(x[2])/8]);
  for(const rf of [0,1]){
   const curve=new Float64Array(4096);
   for(let raw=0;raw<4096;raw++){let i=1;while(i<p.length&&p[i][0]<=raw)i++;let v=i===p.length?p[i-1][1]:p[i-1][1]+(raw-p[i-1][0])*(p[i][1]-p[i-1][1])/(p[i][0]-p[i-1][0]);
    if(rf){let lo=0,off=0;const points=[60,80,100,120,140,160],offs=[-2,-4,-6,-8,-10,-10];for(let j=0;j<points.length;j++){if(v<points[j]){off+=(offs[j]-off)/(points[j]-lo)*(v-lo);break}lo=points[j];off=offs[j]}v+=off;}
    curve[raw]=Math.fround(v);
    if(raw&&curve[raw]>curve[raw-1])throw Error("nonmonotone");
   }luts[id+"/"+rf]=curve;
  }
 }
 if(Object.keys(luts).length!==8)throw Error("missing tables");
 return function quantize(t,id,rf,n,noise){const curve=luts[id+"/"+rf];let lo=0,hi=4095;while(hi-lo>1){let mid=(lo+hi)>>1;if(curve[mid]>t)lo=mid;else hi=mid;}let raw=Math.abs(curve[lo]-t)<=Math.abs(curve[hi]-t)?lo:hi;if(noise)raw+=n%7===0?1:n%7===3?-1:0;return curve[Math.max(1,Math.min(4091,raw))];}
}

module.exports=study;
if(require.main===module)console.log(JSON.stringify(study(Number(process.argv[2]||4),require('fs').readFileSync(process.argv[3]||'Repetier/TemperatureSensors.cpp','utf8')),null,2));

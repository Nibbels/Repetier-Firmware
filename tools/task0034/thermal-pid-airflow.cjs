'use strict';
// A53: temperature-dependent airflow, inverse sensor lookup with ADC quantization; research only.
function study(steps,source) {
 const clamp=(v,l,h)=>Math.max(l,Math.min(h,v)), frac=d=>d>=254?1:(d&254)/256;
 const rows=[],grid=[], results={},quantize=sensorModel(source);
 for(const bed of [false,true])for(const w of bed?[300,450,600]:[30,60,100])for(const mass of bed?[300,800]:[2,6,18])for(const coupling of bed?[8,20]:[4,12])for(const loss of bed?[2,4]:[.04,.07])for(const tau of bed?[1,5]:[.15,.75,1.5])for(const cap of [128,255])grid.push({bed,w,mass,coupling,loss,tau,cap,hc:bed?40:.75});
 function plant(p,ambient,initial=ambient) {
  let h=initial,b=initial,s=initial;
  return {advance(power,load,dt){for(let i=0;i<steps;i++){const tr=p.coupling*(h-b);h+=dt*(power-tr-p.loss*.15*(h-ambient))/p.hc;b+=dt*(tr-p.loss*.85*(b-ambient)-load)/p.mass;s+=dt*(b-s)/p.tau;}},read(){return Math.round(s*10)/10},sensor(){return s},body(){return b},heater(){return h}};
 }
 function train(p){
  const m=plant(p,25),target=p.bed?90:200,cutoff=target+(p.bed?15:20),limit=p.bed?36000:12000,phaseLimit=p.bed?6000:1800,riseLimit=p.bed?600:200;
  let heat=true,start=0,phase=0,rise=0,riseT=25,lo=25,hi=25,on=0,cycles=0,stable=0,kuSum=0,tuSum=0,prevKu=0,prevTu=0;
  for(let n=1;n<=limit;n++){
   m.advance(heat?p.w*frac(p.cap):0,0,.1/steps); const t=m.read();
   if(t>cutoff||n-phase>phaseLimit)return null;
   lo=Math.min(lo,t);hi=Math.max(hi,t);
   if(heat&&t>=target+.375){heat=false;on=n-start;phase=n;}
   else if(heat&&n-rise>=riseLimit){if(t-riseT<1)return null;rise=n;riseT=t;}
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
 const rules={J0:[.45,1.6,.08],J1:[1/2.2,2.2,1/6.3],J2:[.65,1.6,.16],J3:[.45,3,.08],Pessen:[.7,.4,.15],TL:[1/2.2,2.2,1/6.3]};
 const scenarioNames=["nominal","fan","overload","high-jitter","warm"];
 function replay(p,c,method,legacy,scenario,id,rf){
  const [rule,mode="base"]=method.split(":");
  const f=Math.fround,[a,b,d]=rules[rule],kp=f(a*c.ku),ki=f(kp/(b*c.tu)),kd=f(kp*d*c.tu);
  const lag=mode==="lag"||mode==="both"?c.tu:0;
  const tracking=mode==="track"||mode==="both";
  const trackingTime=Math.max(.1,Math.sqrt(b*d)*c.tu);
  const target=p.bed?(scenario==="high-jitter"?105:90):(scenario==="high-jitter"?250:200),cutoff=p.bed?120:275;
  const maxI=legacy?Math.min(p.bed?80:120,p.cap):p.cap,minI=-Math.min(legacy?(p.bed?5:30):20,p.cap);
  const initial=scenario==="warm"?(p.bed?75:175):25;
  const m=plant(p,25,initial),maxPower=p.w*frac(p.cap),tr0=p.loss*.85*(target-25),basePower=tr0+p.loss*.15*(target-25+tr0/p.coupling);
  const load=(scenario==="fan"||scenario==="overload")?Math.max(0,((scenario==="fan"?.85:1.15)*maxPower-basePower)/(1+p.loss*.15/p.coupling)):0;
  const feasible=basePower+(1+p.loss*.15/p.coupling)*load<maxPower;
  const stage=p.bed?3000:1800,end=stage*3,window=p.bed?300:200;
  let reference=initial,I=0,slope=0,prev=0,valid=false,last=0,duty=0,trip=false,overshoot=0,release=0,iae=0,holdAbs=0,holdErr=0,finalAbs=0,finalErr=0,sat=0,mat=25,body=25,tail=0,entered=-1,lastOutside=0;
  for(let n=1;n<=end;n++){
   const cooling=n>stage&&n<=2*stage;
   m.advance(p.w*frac(duty),cooling?load*Math.max(0,m.body()-25)/(target-25):0,.1/steps);
   const t=quantize(m.sensor(),id,rf,n,scenario==="high-jitter"||scenario==="fan"||scenario==="overload");mat=Math.max(mat,m.heater());body=Math.max(body,m.body());
   if(t>cutoff)trip=true;
   if(scenario==="high-jitter"&&n%11===0&&!trip){}else{
    const dt=valid?f((n-last)*.1):0;
    slope=dt>0?f(slope+f(f(dt/f(.5+dt))*f(f((t-prev)/dt)-slope))):0;
    prev=t;last=n;valid=true;
    reference=lag?f(reference+f(f(dt/(lag+dt))*f(target-reference))):target;
    const e=f(reference-t),pd=f(f(kp*e)-f(kd*slope)),candidate=f(clamp(f(I+f(f(ki*e)*dt)),minI,maxI));
    if(tracking&&dt>0){
     const u=f(pd+I),satError=f(clamp(u,0,p.cap)-u);
     I=f(clamp(f(I+f(f(ki*e)*dt)+f(dt/(trackingTime+dt)*satError)),minI,maxI));
    }else if((f(pd+candidate)<=p.cap||e<0)&&(f(pd+candidate)>=0||e>0))I=candidate;
    duty=trip?0:Math.trunc(clamp(f(pd+I),0,p.cap));
   }
   overshoot=Math.max(overshoot,t-target);
   if(n>2*stage)release=Math.max(release,t-target);
   iae+=Math.abs(target-t)*.1;
   if(n>2*stage-window&&n<=2*stage){holdAbs+=Math.abs(target-t)/window;holdErr+=(target-t)/window;if(duty===p.cap)sat++;}
   if(n>end-window){finalAbs+=Math.abs(target-t)/window;finalErr+=(target-t)/window;}
   if(n<=stage){if(Math.abs(target-t)>1)lastOutside=n;else if(entered<0)entered=n;}
   if(n>2*stage&&Math.abs(target-t)>1)tail=n-2*stage;
  }
  return {feasible,trip,overshoot,release,iae,holdAbs,holdErr,finalAbs,finalErr,sat:sat/window,mat,body,settled:entered>=0&&lastOutside<stage-window,settle:lastOutside*.1,recover:tail*.1};
 }
 function add(key,r){
  const a=results[key]??={n:0,trips:0,settled:0,within1:0,mat150:0,overshoot:0,release:0,iae:0,holdAbs:0,holdErr:0,finalAbs:0,sat:0,mat:0,body:0,recover:0,settle:0,hold:[]};
  a.n++;a.trips+=+r.trip;a.settled+=+r.settled;a.within1+=+(r.holdAbs<=1);a.mat150+=+(r.mat>=150);a.hold.push(r.holdAbs);
  for(const k of ["iae","holdAbs","holdErr","finalAbs","sat"])a[k]+=r[k];
  for(const k of ["overshoot","release","mat","body","recover","settle"])a[k]=Math.max(a[k],r[k]);
 }
 let trained=[0,0],traces=0;
 for(const p of grid){const c=train(p);if(!c)continue;trained[+p.bed]++;
  for(const method of p.bed?["J2","J3","TL"]:["J0","J1","Pessen","Pessen:lag"])for(const legacy of [false])for(const scenario of scenarioNames)for(const id of p.bed?[3,4]:[3,8,13])for(const rf of [1]){const r=replay(p,c,method,legacy,scenario,id,rf);traces++;add([p.bed?"bed":"hotend",method,id,scenario,r.feasible?"feasible":"limited"].join("/"),r);}
 }
 for(const a of Object.values(results)){for(const k of ["iae","holdAbs","holdErr","finalAbs","sat"])a[k]/=a.n;a.hold.sort((x,y)=>x-y);a.p95=a.hold[Math.ceil(a.n*.95)-1];delete a.hold;}
 return {steps,trained,traces,results};
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

'use strict';
// A51: source table and interpolation audit, not a sensor calibration.
function audit(source){
 const cleaned=source.replace(/\/\*[\s\S]*?\*\//g,"").replace(/\/\/[^\n]*/g,"");
 function value(s){if(!/^\s*-?\d+(?:\s*\*\s*-?\d+)*\s*$/.test(s))throw Error("unsupported initializer "+s);return s.split("*").reduce((n,x)=>n*Number(x.trim()),1)}
 const tables=[];
 for(const m of cleaned.matchAll(/const short temptable_(\d+)\[NUMTEMPS_\d+\]\[2\] PROGMEM\s*=\s*\{([\s\S]*?)\};/g)){
  const id=Number(m[1]), points=[...m[2].matchAll(/\{\s*([^,{}]+),\s*([^,{}]+)\}/g)].map(x=>[value(x[1]),value(x[2])/8]);
  const count=Number(cleaned.match(new RegExp("#define NUMTEMPS_"+id+"\\s+(\\d+)"))[1]);
  if(count!==points.length)throw Error("length "+id);
  tables.push({id:id===15?53:id,points});
 }
 if(tables.length!==12)throw Error("expected 12 tables");
 tables.push({id:60,points:null},{id:100,points:null});
 const f=Math.fround;
 function correction(t,rf){
  if(!rf)return t;
  let lo=0,off=0;
  const pp=[60,80,100,120,140,160],oo=[-2,-4,-6,-8,-10,-10];
  for(let i=0;i<pp.length;i++){
   if(t<pp[i])return f(t+f(f(f((oo[i]-off)/(pp[i]-lo))*f(t-lo))+off));
   lo=pp[i];off=oo[i];
  }return f(t+off);
 }
 const result=[];
 for(const {id,points} of tables){
  const direction=id>=53?1:-1;
  let badRaw=0,badTemp=0,flatSegments=0;
  if(points)for(let i=1;i<points.length;i++){
   if(points[i][0]<=points[i-1][0])badRaw++;
   const dy=direction*(points[i][1]-points[i-1][1]);if(dy<0)badTemp++;if(dy===0)flatSegments++;
  }
  for(const rf of [false,true]){
   let reversal=0,flat=0,maxStep=0,maxStepAt=0,maxInBand=0,maxInBandAt=0,maxCurvature=0,prev=null,prevStep=null;
   const band=id>=53||id===1||id===3||id===8||id===13?[40,275]:[40,275];
   for(let raw=0;raw<4096;raw++){
    let t;
    if(!points)t=f(f(raw*(id===60?1000:500))/4096);
    else{
     let i=1;while(i<points.length&&points[i][0]<=raw)i++;
     if(i===points.length)t=points[i-1][1];
     else {const [x0,y0]=points[i-1],[x1,y1]=points[i];t=f(y0+f(f((raw-x0)*(y1-y0))/(x1-x0)));}
    }
    t=correction(t,rf);
    if(prev!==null){
     const delta=t-prev;
     if(direction*delta<-1e-5)reversal++;
     if(delta===0)flat++;
     if(Math.abs(delta)>maxStep){maxStep=Math.abs(delta);maxStepAt=raw;}
     if(t>=band[0]&&t<=band[1]&&prev>=band[0]&&prev<=band[1]){
      if(Math.abs(delta)>maxInBand){maxInBand=Math.abs(delta);maxInBandAt=raw;}
      if(prevStep!==null)maxCurvature=Math.max(maxCurvature,Math.abs(delta-prevStep));
     }
     prevStep=delta;
    }prev=t;
   }
   result.push({id,board:rf?"RF2000":"RF1000",points:points?.length||0,badRaw,badTemp,flatSegments,reversal,flat,maxStep,maxStepAt,maxInBand,maxInBandAt,maxCurvature});
  }
 }
 return result;
}
module.exports=audit;
if(require.main===module)console.log(JSON.stringify(audit(require('fs').readFileSync(process.argv[2]||'Repetier/TemperatureSensors.cpp','utf8')),null,2));

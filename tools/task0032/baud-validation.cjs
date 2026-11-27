// Task0032: audit settings validation, not a second UART throughput model.
// At 16MHz compare the legacy divider calculation with an explicit range check.
// Nominal error is not an electrical link or supported-rate qualification.
"use strict";
const CPU=16000000;
function legacy(baud){
  // Reproduce AVR unsigned long arithmetic and uint16 narrowing. Division by
  // zero is undefined; do not pretend a specific AVR helper result is valid.
  if(baud===0)return null;
  const u=baud>>>0;
  let doubleSpeed=u!==57600;
  const div=(k)=>((Math.floor(CPU/k/u)-1)>>>0)>>>1;
  let divisor=0;
  if(doubleSpeed){divisor=div(4)&65535;doubleSpeed=divisor<=4095;}
  if(!doubleSpeed)divisor=div(8)&65535;
  return {doubleSpeed,divisor,registerFits:divisor<=4095};
}
function options(baud){
  if(!Number.isInteger(baud)||baud<Math.ceil(CPU/(16*4096))||baud>CPU/8)return [];
  const result=[];
  for(const sample of [16,8]){
    const ideal=CPU/(sample*baud);
    for(const n of new Set([Math.floor(ideal),Math.ceil(ideal)])){
      if(n<1||n>4096)continue;
      const actual=CPU/(sample*n);
      result.push({doubleSpeed:sample===8,divisor:n-1,actual,errorPct:100*(actual/baud-1)});
    }
  }
  return result.sort((a,b)=>Math.abs(a.errorPct)-Math.abs(b.errorPct)||Number(a.doubleSpeed)-Number(b.doubleSpeed));
}
function inspect(baud){
  const validInput=Number.isInteger(baud)&&baud>=Math.ceil(CPU/(16*4096))&&baud<=CPU/8;
  const old=Number.isInteger(baud)&&baud>=-2147483648&&baud<=4294967295?legacy(baud):null;
  if(old&&old.registerFits&&baud>0){
    old.actual=CPU/((old.doubleSpeed?8:16)*(old.divisor+1));
    old.errorPct=100*(old.actual/baud-1);
  }
  return {baud,validNominalRange:validInput,legacy:old,bestNominal:options(baud)[0]??null};
}
function menu(text){
  const m=/baudrates\[\]\s+PROGMEM\s*=\s*\{([^}]+)\}/.exec(text);
  if(!m)throw Error("baudrates menu anchor changed");
  const values=m[1].split(",").map(s=>Number(s.trim()));
  if(values.at(-1)!==0||values.slice(0,-1).some(n=>!Number.isInteger(n)||n<=0))throw Error("menu format");
  return values.slice(0,-1);
}
function selfTest(){
  const check=(v,m)=>{if(!v)throw Error(m)};
  check([0,-1,-2147483648,1,244,2000001,2147483647,4294967295,1.5].every(n=>!options(n).length),"invalid range");
  const b=inspect(250000).bestNominal;
  check(!b.doubleSpeed&&b.divisor===3&&b.errorPct===0,"250k normal exact");
  const a=inspect(115200);
  check(a.bestNominal.doubleSpeed&&a.bestNominal.divisor===16&&a.legacy.divisor===16,"115200 compatibility");
  check(!legacy(57600).doubleSpeed&&legacy(57600).divisor===16,"legacy 57600 special case");
  check(inspect(1500000).bestNominal&&Math.abs(inspect(1500000).bestNominal.errorPct)>30,"representability not quality");
  check(menu("const long baudrates[] PROGMEM = {9600, 250000, 0};").length===2,"menu extraction");
  return "6 groups of checks passed";
}
if(require.main===module){
  if(process.argv[2]==="--self-test")console.log(selfTest());
  else {
    const fs=require("node:fs");
    const path=process.argv[2]||"Repetier/UiSettings.cpp";
    console.log(JSON.stringify({schema:1,clockHz:CPU,selfTest:selfTest(),
      caveats:"Nominal register arithmetic only. No link tolerance, live baud change, EEPROM integration or fallback policy. Best nominal is not automatically recommended. Preserve 57600 legacy special case pending compatibility decision. 500k and higher not acceptance targets.",
      menu:menu(fs.readFileSync(path,"utf8")).map(inspect),
      invalid:[0,-1,-2147483648,1,244,245,2000000,2000001,2147483647,4294967295].map(inspect)},null,2));
  }
}
module.exports={inspect,menu,selfTest};

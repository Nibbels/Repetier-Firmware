// Task0032 S1a: link-only model, NOT a firmware or CPU simulation.
"use strict";
function frame(n, body) {
  const prefix = `N${n} ${body}`;
  let checksum = 0;
  for (const ch of prefix) checksum ^= ch.charCodeAt(0);
  return `${prefix}*${checksum}\n`;
}

function workload({radius, chord, speed, profile="compact"}) {
  if (!(radius>0 && chord>0 && chord<=2*radius && speed>0)) throw Error("invalid geometry");
  const segments=Math.max(3,Math.ceil(Math.PI/Math.asin(chord/(2*radius))-1e-12));
  const point=i=>[Number((100+radius*Math.cos(2*Math.PI*(i%segments)/segments)).toFixed(4)),Number((100+radius*Math.sin(2*Math.PI*(i%segments)/segments)).toFixed(4))];
  const start=point(0), lines=[], lengths=[];
  let previous=start, distance=0;
  for(let i=1;i<=segments;i++){
    const p=point(i), len=Math.hypot(p[0]-previous[0],p[1]-previous[1]);
    if(!len)throw Error("rounded zero chord");
    distance+=len; lengths.push(len);
    const body=`G1 X${p[0].toFixed(4)} Y${p[1].toFixed(4)} E${(distance*0.04).toFixed(5)}${i===1||profile==="verbose"?` F${speed*60}`:""}`;
    lines.push(frame(i,body)); previous=p;
  }
  const bytes=lines.reduce((a,l)=>a+l.length,0);
  return {start,lines,lengths,distance,bytes,segments};
}

function transport(lines,{hostBaud=115200,deviceBaud=16000000/(8*17),window=1,ackBytes=4,ackDelayUs=0}={}){
  if(!Number.isFinite(hostBaud)||!Number.isFinite(deviceBaud)||hostBaud<=0||deviceBaud<=0||!Number.isInteger(window)||window<1||!Number.isInteger(ackBytes)||ackBytes<1||!Number.isFinite(ackDelayUs)||ackDelayUs<0)throw Error("bad transport");
  let rxEnd=0,txEnd=0; const ackEnds=[];
  for(let i=0;i<lines.length;i++){
    const start=Math.max(rxEnd,i>=window?ackEnds[i-window]:0);
    rxEnd=start+lines[i].length*10*1e6/hostBaud;
    txEnd=Math.max(txEnd,rxEnd+ackDelayUs)+ackBytes*10*1e6/deviceBaud;
    ackEnds.push(txEnd);
  }
  return {wireRxEndUs:rxEnd,wireLastAckEndUs:txEnd,idealReceivedFrames:lines.length,idealAckedFrames:lines.length};
}

function scenario(spec,link) {
  const w=workload(spec), t=transport(w.lines,link);
  return {spec,link,initialXY:w.start,segments:w.segments,pathMm:w.distance,offeredBytes:w.bytes,meanLineBytes:w.bytes/w.segments,movesPerMm:w.segments/w.distance,requestedMovesPerSecond:spec.speed*w.segments/w.distance,requestedDurationUs:1e6*w.distance/spec.speed,idealRxFramesPerSecond:1e6*w.segments/t.wireRxEndUs,idealRxPathMmPerSecond:1e6*w.distance/t.wireRxEndUs,...t};
}

function selfTest(){
  const check=(b,m)=>{if(!b)throw Error(m)};
  const close=(a,b)=>Math.abs(a-b)<1e-7*Math.max(1,Math.abs(b));
  check(frame(1,"G1 X1")==="N1 G1 X1*96\n","known ASCII XOR");
  const t=transport(["1234567890","1234567890"],{hostBaud:1000,deviceBaud:1000,window:1,ackBytes:2,ackDelayUs:1000});
  check(close(t.wireRxEndUs,221000)&&close(t.wireLastAckEndUs,242000),"stop-and-wait hand calculation");
  const b=transport(["1234567890","1234567890"],{hostBaud:1000,deviceBaud:1000,window:2,ackBytes:2,ackDelayUs:1000});
  check(close(b.wireRxEndUs,200000)&&close(b.wireLastAckEndUs,221000),"pipelined hand calculation");
  const w=workload({radius:1,chord:1,speed:60});
  check(w.segments===6&&close(w.distance,2+4*Math.hypot(.5,.8660)),"hexagon rounded perimeter");
  for(const l of w.lines){
    const [body,s]=l.trimEnd().split("*");
    check([...body].reduce((x,c)=>x^c.charCodeAt(0),0)===Number(s),"checksum");
  }
  check(w.lines.length===w.lengths.length&&w.lines.every(l=>l.endsWith("\n")),"line count");
  check(JSON.stringify(w)===JSON.stringify(workload({radius:1,chord:1,speed:60})),"determinism");
  return "7 groups of checks passed";
}

function matrix(){
  const out=[];
  for(const radius of [0.5,1.35,5,20])for(const chord of [0.05,0.1,0.2,0.3,0.6,1,5,10])if(chord<=2*radius)
    for(const speed of [30,40,60,80,100,120,150])for(const profile of ["compact","verbose"])
      for(const window of [1,32,96])for(const ackDelayUs of [0,1000])
        out.push({spec:{radius,chord,speed,profile},link:{hostBaud:115200,deviceBaud:16000000/(8*17),window,ackBytes:4,ackDelayUs}});
  return out;
}

if(require.main===module){
  const fs=require("node:fs"), crypto=require("node:crypto"), path=require("node:path");
  if(process.argv[2]==="--self-test"){console.log(selfTest());process.exit(0);}
  const dir=process.argv[2];if(!dir)throw Error("usage: node link-model.cjs --self-test | OUTPUT_DIR");
  fs.mkdirSync(dir,{recursive:true});
  const cases=matrix();let misses=0;
  const rows=cases.map(({spec,link})=>{
    const w=workload(spec),gcode=w.lines.join("");
    const sha=crypto.createHash("sha256").update(gcode,"ascii").digest("hex");
    fs.writeFileSync(path.join(dir,sha+".gcode"),gcode,"ascii");
    const r=scenario(spec,link);if(r.idealRxPathMmPerSecond<spec.speed)misses++;
    return {...r,gcodeSha256:sha};
  });
  fs.writeFileSync(path.join(dir,"cases.jsonl"),rows.map(r=>JSON.stringify(r)).join("\n")+"\n");
  fs.writeFileSync(path.join(dir,"summary.json"),JSON.stringify({schema:1,model:"ideal-link-only",sourceSha256:crypto.createHash("sha256").update(fs.readFileSync(__filename)).digest("hex"),node:process.version,cases:rows.length,requestedRateExceedsIdealRx:misses},null,2)+"\n");
}
module.exports={frame,workload,transport,scenario,selfTest,matrix};

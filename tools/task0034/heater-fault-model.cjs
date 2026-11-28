// Abstract fault model, not production firmware or AVR emulation.
// Illustrative lease/reset times only. Reset gate is hypothetical, not board evidence.
function runModel(){
  // Abstract time in microseconds; not an AVR emulator. Fixed illustrative parameters.
  const frame=32768,lease=120000,end=400000,crash=50000,step=256;
  const modes=["legacy","lease-isr","lease-compare","lease-isr-reset"];
  const faults=["healthy","foreground-stall","irq-masked","thermal-stall-heartbeat-alive","invalid-sample","zero-demand"];
  const rows=[];
  for(const mode of modes) for(const fault of faults) for(const duty of [0,64,128,254,255]){
    let grant=-Infinity,high=false,off=Infinity,energy=0,lastHigh=-1,legacyDuty=0,lastRenew=-Infinity,latched=false,peak=0,rise=0;
    for(let t=0;t<end;t+=step){
      const failed=t>=crash&&fault!=="healthy";
      const irq=!(failed&&fault==="irq-masked");
      const fresh=!failed;
      const revoke=failed&&(fault==="invalid-sample"||fault==="zero-demand");
      if(revoke){grant=-Infinity;legacyDuty=0;if(mode!=="legacy")high=false;}
      // A completed thermal check, not a generic heartbeat, renews permission.
      if(t%99840===0&&fresh){grant=t+lease;legacyDuty=duty;lastRenew=t;}
      // Hypothetical independent reset gate, NOT established on the RF boards.
      if(mode==="lease-isr-reset" && t-lastRenew>=lease+20000){latched=true;high=false;}
      if(mode==="lease-compare" && t>=off)high=false;
      if(irq&&mode!=="legacy"&&t>=grant){high=false;if(lastRenew!==-Infinity&&!revoke)latched=true;}
      if(irq&&t%frame===0){
        if(mode==="legacy"){if(legacyDuty>=2){if(!high)rise=t;high=true;}off=legacyDuty>=254?Infinity:t+(legacyDuty&254)*128;if(!legacyDuty)high=false;}
        else if(!latched&&t<grant&&duty>0){if(!high)rise=t;high=true;off=Math.min(t+Math.floor(duty*frame/255/step)*step,grant);if(off<=t)high=false;}
      }
      if(irq&&t>=off)high=false;
      if(high){energy+=step;lastHigh=t+step;peak=Math.max(peak,t+step-rise);}
    }
    rows.push({mode,fault,duty,energy_us:energy,last_high_end_us:lastHigh,longest_high_us:peak,high_after_lease: lastHigh>lease+step});
  }
  const find=(mode,fault,duty)=>rows.find(x=>x.mode===mode&&x.fault===fault&&x.duty===duty);
  const checks=[
    ["legacy stale full duty persists",find("legacy","foreground-stall",255).last_high_end_us===end/step*step || find("legacy","foreground-stall",255).last_high_end_us>=end],
    ["legacy stale duty persists despite live generic heartbeat",find("legacy","thermal-stall-heartbeat-alive",255).last_high_end_us>=end],
    ["ISR lease bounds foreground stall",find("lease-isr","foreground-stall",255).last_high_end_us<=lease+step],
    ["ISR lease cannot stop masked IRQ",find("lease-isr","irq-masked",255).last_high_end_us>=end],
    ["hardware compare ends masked pulse",find("lease-compare","irq-masked",255).last_high_end_us<=crash+frame+step],
    ["hypothetical independent reset ends masked output",find("lease-isr-reset","irq-masked",255).last_high_end_us<=lease+20000+step],
    ["zero request never heats",rows.filter(x=>x.duty===0).every(x=>x.energy_us===0)],
    ["explicit revoke ends new-mode output",rows.filter(x=>x.mode!=="legacy"&&["invalid-sample","zero-demand"].includes(x.fault)).every(x=>x.last_high_end_us<=crash+step)]
  ];
  if(checks.some(x=>!x[1]))throw Error(JSON.stringify(checks));
  return {parameters:{frame,lease,end,crash,step},checks,rows};
}
console.log(JSON.stringify(runModel(), null, 2));

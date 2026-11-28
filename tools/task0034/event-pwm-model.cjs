// Task0034 candidate Timer5 event PWM geometry and ideal AC halfwave comparison.
function eventPwmModel(){
  // Timer5 normal /8: 0.5us tick, exact existing 32.768ms wrap.
  // Abstract ideal GPIO edge model, no AVR ISR/register or reset proof.
  const frame=65536,guard=64,periods=256;
  function legacyWidth(duty){const d=duty&254;return d===254?frame:d*256;}
  function candidateWidth(duty){return Math.min(legacyWidth(duty),frame-guard);}
  let cases=0,maxLoss=0,maxHealthyLateLoss=0,maxEdges=0;
  for(let duty=0;duty<256;duty++){
    const width=candidateWidth(duty),old=legacyWidth(duty);
    if(width<0||width>frame-guard||width>old)throw Error("width");
    if(duty<254&&width!==old)throw Error("non-full changed");
    maxLoss=Math.max(maxLoss,old-width);
    for(const startDelay of [0,1,8,64,256,512,1024,32768,65535]){
      for(const offDelay of [0,1,8,64,256,1024,65536]){
        // Late frame never starts if deadline or guard already reached.
        // Pending compare remains a software IRQ, not physical cutoff.
        const admit=width>0&&startDelay+guard<width;
        const high=admit?Math.max(0,width+offDelay-startDelay):0;
        if(!admit&&high)throw Error("late admission");
        if(offDelay===0&&high>width)throw Error("extra energy");
        if(startDelay===0&&offDelay===0&&width>0&&high!==width)throw Error("healthy duty");
        if(offDelay===0)maxHealthyLateLoss=Math.max(maxHealthyLateLoss,width-high);
        cases++;
      }
    }
  }
  // For mains model, assume ideal zero-cross switch samples GPIO at each
  // halfwave boundary; no gate-window, propagation or commutation model.
  // Units 1/6 us => Timer5 tick=3 units, 50Hz halfwave=60000, 60Hz=50000.
  const mains=[];
  for(const hz of [50,60]){
    const half=hz===50?60000:50000, period=frame*3;
    let worstLegacy=0,worstCandidate=0,worstDifference=0,maxFullLoss=0;
    const perDuty=[];
    const n=hz*2*60;
    for(let duty=0;duty<256;duty+=2){
      let minOld=1,maxOld=0,minNew=1,maxNew=0;
      for(let k=0;k<32;k++){
        const phase=Math.floor(half*k/32);
        let oldOn=0,newOn=0;
        for(let i=0;i<n;i++){
          const p=(phase+i*half)%period;
          if(p<legacyWidth(duty)*3)oldOn++;
          if(p<candidateWidth(duty)*3)newOn++;
        }
        const a=oldOn/n,b=newOn/n;
        minOld=Math.min(minOld,a);maxOld=Math.max(maxOld,a);
        minNew=Math.min(minNew,b);maxNew=Math.max(maxNew,b);
        worstLegacy=Math.max(worstLegacy,Math.abs(a-legacyWidth(duty)/frame));
        worstCandidate=Math.max(worstCandidate,Math.abs(b-candidateWidth(duty)/frame));
        worstDifference=Math.max(worstDifference,a-b);
        if(duty===254)maxFullLoss=Math.max(maxFullLoss,1-b);
        if(newOn>oldOn||duty<254&&newOn!==oldOn)throw Error("mains comparison");
      }
      perDuty.push({duty,legacy:[minOld,maxOld],candidate:[minNew,maxNew]});
    }
    mains.push({hz,phaseSamples:32,seconds:60,worstLegacy,worstCandidate,worstDifference,maxFullLoss,examples:perDuty.filter(x=>[2,64,128,192,254].includes(x.duty))});
  }
  // Frame start + one compare per active heater. Equal compare deadlines
  // may still mean separate ISR vectors, but zero channels need no compare.
  for(let a=0;a<2;a++)for(let b=0;b<2;b++)for(let c=0;c<2;c++)maxEdges=Math.max(maxEdges,1+a+b+c);
  return {cases,frameTicks:frame,guardTicks:guard,tickUs:0.5,fullDutyLossFraction:maxLoss/frame,maxHealthyLateLossTicks:maxHealthyLateLoss,isolatedMaxInterruptsPerSecond:maxEdges*1e6/(frame*.5),mains,limits:"Ideal edge and mains sampling model only. A delayed OFF ISR can stretch HIGH, and pending flags must not be erased. Timer3 lease plus independent physical fail-off still required. Guard is exploratory, not selected. No hardware or thermal validation."};
}
console.log(JSON.stringify(eventPwmModel(),null,2));

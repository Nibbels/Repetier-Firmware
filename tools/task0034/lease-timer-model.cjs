// Task0034: exhaustive tick-domain renewal prototype. Not production or hardware qualification.
function leaseModel(){
  // Tick-domain model of a free-running 16-bit AVR timer, NOT AVR emulation.
  // OCF becomes sticky one timer tick after equality; ISR dispatch clears it.
  const MOD=65536, GUARD=3, LEASE=12500; // /256, 16 us; illustrative 200 ms
  const u16=x=>x&65535;
  class Timer {
    constructor(now,ocr,flag=false){this.now=u16(now);this.ocr=u16(ocr);this.flag=flag;this.mask=true;this.armed=true;this.fault=false;this.high=true;}
    tick(n){while(n--){if(this.now===this.ocr)this.flag=true;this.now=u16(this.now+1);}}
    fail(){this.fault=true;this.armed=false;this.mask=false;this.high=false;}
    dispatch(){if(this.mask&&this.flag){this.flag=false;this.fail();}}
    // Foreground must hold I=0 during read/write/publish. Never clear an active OCF.
    renew(gaps){
      if(this.fault||!this.armed)return false;
      const snapshot=this.now;this.tick(gaps[0]);
      const remaining=u16(this.ocr-snapshot);this.tick(gaps[1]);
      if(this.flag||remaining<=GUARD||remaining>LEASE){this.fail();return false;}
      // The total sample-to-commit bound must be <= GUARD ticks, proven separately.
      this.tick(gaps[2]);this.ocr=u16(snapshot+LEASE);
      this.tick(gaps[3]);
      if(this.flag){this.fail();return false;}
      return true;
    }
    // Output gate must also look at the sticky flag; lower-priority expiry
    // ISR may still be pending when a PWM frame wants to start.
    startFrame(){const remaining=u16(this.ocr-this.now);if(this.fault||!this.armed||this.flag||remaining<=GUARD||remaining>LEASE){this.fail();return false;}this.high=true;return true;}
  }
  let checked=0,accepted=0,rejected=0,expired=0,wrap=0;
  const gaps=[];
  for(let a=0;a<=GUARD;a++)for(let b=0;b<=GUARD-a;b++)for(let c=0;c<=GUARD-a-b;c++)for(let d=0;d<=GUARD-a-b-c;d++)gaps.push([a,b,c,d]);
  // Enumerate every start count for representative distances and every
  // bounded micro-operation gap pattern. Exhaustively covers the wrap phase.
  for(let now=0;now<MOD;now++){
    for(const rem of [0,1,2,3,4,5,LEASE-1,LEASE,LEASE+1,65535]){
      for(const g of gaps){
        const t=new Timer(now,now+rem,rem===65535);
        const result=t.renew(g);
        const expected=rem>GUARD&&rem<=LEASE;
        if(result!==expected)throw Error(JSON.stringify({now,rem,g,result,expected}));
        if(result){accepted++;if(t.flag||t.fault||t.ocr!==u16(now+LEASE))throw Error("bad extension");if(now+LEASE>=MOD)wrap++;}
        else{rejected++;if(t.high||!t.fault||t.mask||t.armed)throw Error("failed open");if(t.renew([0,0,0,0])||t.startFrame())throw Error("latched fault rearmed");}
        checked++;
      }
    }
  }
  // Fault phases across wrap, including delayed dispatch and flag coalescence.
  for(const now of [0,1,65530,65535])for(const gap of [0,1,LEASE-1,LEASE,LEASE+1,MOD,2*MOD+17]){
    const t=new Timer(now,now+LEASE);
    t.tick(gap);
    if(gap>LEASE){
      if(!t.flag||t.startFrame()||t.high||t.renew([0,0,0,0]))throw Error("late frame/rearm");
      expired++;
      const x=new Timer(now,now+LEASE);x.tick(gap);x.dispatch();
      if(x.high||!x.fault||x.armed)throw Error("expiry IRQ failed");
    }
  }
  // Counterexamples: flag not yet posted at exact equality, and clearing
  // OCF during renewal would erase evidence of an expired lease.
  const edge=new Timer(1234,1234);
  if(edge.flag||edge.renew([0,0,0,0])||edge.high)throw Error("equality guard");
  const edgeFrame=new Timer(1234,1234);
  if(edgeFrame.flag||edgeFrame.startFrame()||edgeFrame.high)throw Error("equality gate");
  const lost=new Timer(65530,65531);lost.tick(2);
  if(!lost.flag)throw Error("expected pending compare");
  lost.flag=false;lost.ocr=u16(lost.now+LEASE);
  if(!lost.startFrame())throw Error("expected unsafe naive clear counterexample");
  // Guard is a contract, not inherent hardware guarantee: exceed it and old
  // compare may occur after validation. New OCR before next tick can erase
  // equality evidence. No unbounded masked-region claim.
  return {checked,accepted,rejected,wrapped_extensions:wrap,gap_patterns:gaps.length,delayed_expiry_cases:expired,assumptions:{counter_bits:16,tick_us:16,illustrative_lease_ticks:LEASE,guard_ticks:GUARD,irq_masked_during_renewal:true,hardware_clock_running:true},limits:"Software gate/ISR only. No physical off while CPU permanently masked; independent reset/OC required. No actual AVR instruction timing or initial arm implementation."};
}
console.log(JSON.stringify(leaseModel(),null,2));

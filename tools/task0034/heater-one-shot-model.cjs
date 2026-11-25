// Task0034: abstract safe one-shot arming crash-prefix model; no hardware proof.
function oneShotModel(){
  // Normal-mode OC timer model. FOC acts on latch without OCF. Physical
  // HIGH iff DDR drives the OC latch; input MUST have verified safe bias.
  // No arbitrary register corruption or stopped clock.
  const clear=2,set=3,disabled=0;
  class Timer {
    constructor(bits,now){this.mod=2**bits;this.now=now;this.ocr=0;this.flag=false;this.mode=disabled;this.ddr=true;this.port=false;this.oc=false;this.highTicks=0;this.maxHigh=0;this.run=0;}
    get high(){return this.ddr&&(this.mode?this.oc:this.port);}
    compare(){if(this.mode===set)this.oc=true;else if(this.mode===clear)this.oc=false;}
    force(){this.compare();}
    tick(n){while(n--){if(this.now===this.ocr){this.flag=true;this.compare();}
      this.now=(this.now+1)%this.mod;
      if(this.high){this.highTicks++;this.run++;this.maxHigh=Math.max(this.maxHigh,this.run);}else this.run=0;}}
  }
  const guard=3;
  const ops=[
    (t)=>{t.port=false;}, // GPIO latch never requests HIGH/pull-up
    (t)=>{t.ddr=false;}, // external bias assumption; no active pulse
    (t)=>{t.mode=set;},
    (t)=>{t.force();},
    (t)=>{t.flag=false;}, // inactive output only
    (t,d)=>{t.ocr=(t.now+d)%t.mod;},
    (t)=>{t.mode=clear;},
    (t,d)=>{const rem=(t.ocr-t.now+t.mod)%t.mod;if(t.flag||rem<=guard||rem>d){t.mode=disabled;t.ddr=true;return false;}t.ddr=true;return true;}
  ];
  let cases=0,maxObserved=0,naiveUnsafe=0;
  // Sweep phase, pulse lengths and every possible crash boundary. Gaps
  // represent timer ticks between instructions, not CPU instruction counts.
  for(const bits of [8,16]){
    const mod=2**bits;
    const starts=bits===8?Array.from({length:mod},(_,i)=>i):[0,1,2,32767,65520,65530,65534,65535];
    const durations=bits===8?[4,5,8,32,64,127,128,253,254,255]:[4,5,100,4096,8192,65534,65535];
    for(const now of starts)for(const dur of durations)for(const gap of [0,1,3])for(let crash=0;crash<=ops.length;crash++){
      const t=new Timer(bits,now);
      for(let i=0;i<crash;i++){ops[i](t,dur);t.tick(gap);}
      // Counter runs for two complete periods after foreground stops.
      t.tick(2*mod);
      if(t.high||t.maxHigh>dur+1)throw Error(JSON.stringify({bits,now,dur,gap,crash,maxHigh:t.maxHigh,high:t.high}));
      if(crash<ops.length&&t.highTicks!==0)throw Error("output high before safe commit");
      maxObserved=Math.max(maxObserved,t.maxHigh);cases++;
    }
    // A tempting FOC sequence with DDR already output is not crash-safe.
    const unsafe=new Timer(bits,0);unsafe.mode=set;unsafe.force();unsafe.tick(2*mod);
    if(!unsafe.high)throw Error("expected set-mode crash counterexample");naiveUnsafe++;
  }
  return {cases,max_observed_high_ticks:maxObserved,naive_set_mode_counterexamples:naiveUnsafe,assumptions:["timer clock continues","normal non-PWM mode","OC compare latch drives only when DDR output","input externally biased OFF","no existing pulse when starting","all register writes serialized; shared COM bits preserved","abstract compare timing; verify actual AVR peripheral behavior and arming instruction bounds"],limits:"No actual AVR or board test; 16-bit starts sampled, 8-bit exhaustive phase for selected widths. RF1000 schematic lacks explicit MOSFET pulldown; RF2000/v2 bed PE2 has no OC. No production change."};
}
console.log(JSON.stringify(oneShotModel(),null,2));

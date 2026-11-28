// Task0034: deliberately restricted opcode replay, NOT a silicon emulator.
// Execute actual STK500v2 reset -> CRT -> WDRF branch -> linked recovery.
// Unsupported opcodes stop the test. No assumption about timing, RWW,
// peripherals, reset electrical bias, or bootloaders other than this path.
"use strict";
function replay(boot, app, cookie, cause, seed) {
    const regs = Uint8Array.from({length:32},(_,i)=>(seed+17*i)&255);
    const ram = new Uint8Array(0x2200).fill(seed);
    // Model reset SFR state, with sticky WDRF forcing WDE.
    ram.fill(0,0,0x200); ram[0x54]=cause; ram[0x60]=cause&8?8:0;
    let pc=0x3e000, z=false,c=false,interrupt=false,wdr=0,entry=false,steps=0;
    const read=a=>{if(a>=ram.length)throw Error("RAM address "+a);return ram[a];};
    const write=(a,v)=>{
        if(a>=ram.length)throw Error("RAM address "+a);
        if(a===0x60 && (ram[0x54]&8))v|=8;
        ram[a]=v&255;
        if(a===0x5f){z=!!(v&2);c=!!(v&1);interrupt=!!(v&128);}
    };
    const pair=r=>regs[r]|regs[r+1]<<8;
    const setPair=(r,v)=>{regs[r]=v&255;regs[r+1]=v>>>8&255;};
    const flash=a=>{const v=(a>=0x3e000?boot:app)[a];if(v===undefined)throw Error("Missing flash byte "+a.toString(16));return v;};
    const word=a=>flash(a)|flash(a+1)<<8;
    const sp=()=>read(0x5d)|read(0x5e)<<8;
    const push=v=>{const s=sp();if(s<0x200||s>=0x2200)throw Error("Stack outside SRAM");write(s,v);write(0x5d,(s-1)&255);write(0x5e,(s-1)>>>8);};
    const call=target=>{const ret=pc/2;push(ret&255);push(ret>>>8&255);push(ret>>>16&255);pc=target;};
    const rel=(w,bits)=>{const mask=(1<<bits)-1,v=w&mask;return v&(1<<(bits-1))?v-(1<<bits):v;};
    const flags=(v,priorZ=false)=>{c=v<0;z=(v&255)===0&&(!priorZ||z);};
    const sign=(v,bits)=>v&(1<<(bits-1))?v-(1<<bits):v;
    function check(ok,msg){if(!ok)throw Error(msg+" PC="+pc.toString(16));}
    for(;steps<2000;steps++){
        if(pc===0x20000){check(cause&8,"Non-WDRF entry");check(!entry,"Repeated entry");check(read(0x5c)===1,"EIND not 1");check(read(0x60)===0,"WDT not disabled");entry=true;}
        if(pc===0){
            check(entry,"Unexpected direct application entry");
            check(!interrupt,"Recovery failed to mask IRQ");
            check([0x52,0x6c,0x9a,0xd3].every((v,i)=>read(cookie+i)===v),"Recovery cookie mismatch");
            return {steps,entry:"0x20000",next:"0x0",wdr,stack:sp(),eind:read(0x5c)};
        }
        if(pc===0x3e724){
            check(!(cause&8),"WDRF skipped recovery");
            check(!entry,"Unexpected recovery");
            return {steps,entry:null,next:"0x3e724",wdr};
        }
        const at=pc,w=word(pc);pc+=2;
        const d=(w>>>4)&31,r=(w&15)|((w>>>5)&16),k=(w&15)|((w>>>4)&0xf0);
        if((w&0xfe0e)===0x940c||(w&0xfe0e)===0x940e){
            const addr=word(pc)|((w&1)<<16)|((w&0x1f0)<<13);pc+=2;
            if((w&0xfe0e)===0x940e)call(addr*2);else pc=addr*2;
        }else if((w&0xf000)===0xe000)regs[16+((w>>>4)&15)]=k; // LDI
        else if((w&0xfc00)===0x2400){regs[d]^=regs[r];z=regs[d]===0;} // EOR
        else if((w&0xf800)===0xb800){const io=(w&15)|((w>>>5)&0x30);write(io+0x20,regs[d]);}
        else if((w&0xf800)===0xb000){const io=(w&15)|((w>>>5)&0x30);regs[d]=read(io+0x20);}
        else if((w&0xf000)===0xc000)pc+=2*rel(w,12);
        else if((w&0xfc07)===0xf401){if(!z)pc+=2*sign((w>>>3)&127,7);} // BRNE
        else if((w&0xf000)===0x3000)flags(regs[16+((w>>>4)&15)]-k); // CPI
        else if((w&0xfc00)===0x0400)flags(regs[d]-regs[r]-Number(c),true); // CPC
        else if((w&0xfe0f)===0x9007){ // ELPM Rd,Z+
            const a=(read(0x5b)<<16)|pair(30);regs[d]=flash(a);
            const next=a+1;setPair(30,next);write(0x5b,next>>>16);
        }else if((w&0xfe0f)===0x920d){write(pair(26),regs[d]);setPair(26,pair(26)+1);} // ST X+
        else if((w&0xfe0f)===0x920f)push(regs[d]);
        else if((w&0xf000)===0x5000||(w&0xf000)===0x4000){ // SUBI/SBCI
            const rd=16+((w>>>4)&15),carry=(w&0xf000)===0x4000;
            const v=regs[rd]-k-(carry?Number(c):0);regs[rd]=v&255;flags(v,carry);
        }else if((w&0xf000)===0x6000)regs[16+((w>>>4)&15)]|=k; // ORI
        else if((w&0xf000)===0x7000)regs[16+((w>>>4)&15)]&=k; // ANDI
        else if((w&0xfe0f)===0x9000){regs[d]=read(word(pc));pc+=2;}
        else if((w&0xfe0f)===0x9200){write(word(pc),regs[d]);pc+=2;}
        else if((w&0xfe08)===0xfe00){ // SBRS (next instruction must be RJMP here)
            if(regs[d]&(1<<(w&7))){check((word(pc)&0xf000)===0xc000,"Unexpected SBRS width");pc+=2;}
        }else if(w===0x94f8){interrupt=false;ram[0x5f]&=0x7f;}
        else if(w===0x9478){interrupt=true;ram[0x5f]|=0x80;}
        else if(w===0x95a8)++wdr;
        else if(w===0x9519)call(((read(0x5c)<<16)|pair(30))*2);
        else throw Error("Unsupported opcode "+w.toString(16)+" at "+at.toString(16));
    }
    throw Error("Replay step budget exhausted");
}
module.exports={replay};
if(require.main===module){
    const fs=require("fs"), input=JSON.parse(fs.readFileSync(0,"utf8"));
    const results=[];
    for(const cause of [0,1,2,4,8,9,10,12,15])for(const seed of [0,0x55,0xaa,0xff]){
        results.push({cause,seed,...replay(input.boot,input.app,input.cookie,cause,seed)});
    }
    console.log(JSON.stringify({cases:results.length,results},null,2));
}

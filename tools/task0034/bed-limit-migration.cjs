'use strict';
// A68: full-settings checksum and record lifecycle model, not firmware or hardware proof.
function study(){
 const SIZE=2048,ADDRESS=1987,MARK=0xb6,FORMAT=1,OLD=9,NEW=10,MAGIC=44,INTEGRITY=104,VERSION=105;
 const allowed=v=>Number.isInteger(v)&&(v===0||(v>=40&&v<=150));
 function crc(a){let c=0xffff;for(let i=1;i<=3;i++){c^=a[i]<<8;for(let j=0;j<8;j++)c=((c<<1)^((c&0x8000)?0x1021:0))&65535;}return c}
 function encode(v){if(!allowed(v))throw Error("value");const a=[MARK,FORMAT,v,255-v,0,0],c=crc(a);a[4]=c&255;a[5]=c>>8;return a}
 function decode(a){return a[0]===MARK&&a[1]===FORMAT&&allowed(a[2])&&a[3]===255-a[2]&&crc(a)===(a[4]|a[5]<<8)?a[2]:null}
 function sum(a){let n=0;for(let i=0;i<SIZE;i++)if(i!==INTEGRITY)n+=a[i];return n&255}
 function state(a){const value=a[0]===MAGIC&&a[VERSION]===NEW&&a[INTEGRITY]===sum(a)?decode(a.slice(ADDRESS,ADDRESS+6)):null;return {value,maxTarget:value?value-10:0,monitor:value||120}}
 function fixture(version,value){const a=new Uint8Array(SIZE);a[0]=MAGIC;a[VERSION]=version;a[1978]=0xa9;a[200]=0x42;a[107]=80;if(value!==null)a.set(encode(value),ADDRESS);a[INTEGRITY]=sum(a);return a}
 const fail={};
 function transaction(image,operation,cut=-1,torn=0,ignore=-1){
  const a=image.slice();let writes=0,outputPermitted=false,completed=false;
  function put(p,v){if(outputPermitted)throw Error("write while authorized");const index=writes++;if(index===cut){a[p]=torn;throw fail}if(index!==ignore)a[p]=v;}
  function checked(p,v){put(p,v);return a[p]===v}
  function finish(){return checked(INTEGRITY,sum(a));}
  function record(value){
   if(!allowed(value)||a[0]!==MAGIC||a[VERSION]!==NEW||a[INTEGRITY]!==sum(a))return false;
   if(!checked(ADDRESS,0))return false;
   const body=encode(value);for(let i=1;i<6;i++)if(!checked(ADDRESS+i,body[i]))return false;
   if(!checked(ADDRESS,MARK))return false;
   if(!finish())return false;
   return state(a).value===value;
  }
  function migrate(){
   // Revoke BEFORE promoting the global version. A valid-looking record
   // left by a previous version is deliberately not reused on migration.
   if(!checked(ADDRESS,0))return false;
   if(!checked(VERSION,NEW))return false;
   if(!finish())return false;
   return state(a).value===null;
  }
  function restore(){
   // Model an ordinary M502 unrelated write; never touch cutoff record.
   if(!checked(107,80)||!checked(200,0x24))return false;
   return finish();
  }
  function clear(){
   // Invalidate safety authorization first, then model the existing reset
   // of the other setting bytes. Only relevant state bytes are changed
   // here; not a replay of the actual 2048-byte EEPROM reset.
   if(!checked(ADDRESS,0))return false;
   for(let i=1;i<6;i++)if(!checked(ADDRESS+i,0))return false;
   if(!checked(107,80)||!checked(200,0)||!checked(1978,0)||
      !checked(0,MAGIC)||!checked(VERSION,NEW))return false;
   return finish()&&state(a).value===null;
  }
  try{
   completed=operation.kind==="set"?record(operation.value):operation.kind==="migrate"?migrate():operation.kind==="restore"?restore():clear();
   // Only after the last readback and global integrity check.
   outputPermitted=completed&&state(a).maxTarget>0;
  }catch(e){if(e!==fail)throw e;}
  return {a,writes,completed,outputPermitted,state:state(a)};
 }
 let complete=0,torn=0,ignored=0,corrupt=0,rollback=0;
 function check(c,msg){if(!c)throw Error(msg)}
 function test(image,op,valid){
  const old=state(image),r=transaction(image,op);
  check(r.completed,"complete transaction");
  check(valid.includes(r.state.value),"complete value");
  complete++;
  for(let at=0;at<r.writes;at++){
   for(let v=0;v<256;v++){
    const q=transaction(image,op,at,v);
    check(!q.outputPermitted,"interrupted runtime authorization");
    check(valid.includes(q.state.value)||q.state.value===null||q.state.value===old.value,"unexpected torn record");
    if(op.kind==="migrate")check(q.state.value===null,"rollback record resurrected");
    if(op.kind==="clear"&&old.value===null)check(q.state.value===null,"corruption reset rearmed");
    check(q.state.maxTarget<=140,"absolute target");
    torn++;
   }
   const q=transaction(image,op,-1,0,at);
   check(!q.outputPermitted||q.completed,"ignored write authorized");
   check(valid.includes(q.state.value)||q.state.value===null||q.state.value===old.value,"ignored write value");
   if(op.kind==="migrate")check(q.state.value===null,"ignored migration resurrected");
   ignored++;
  }
 }
 const levels=[0,40,80,90,120,150];
 for(const old of levels){
  for(const next of levels)test(fixture(NEW,old),{kind:"set",value:next},[next]);
  test(fixture(NEW,old),{kind:"restore"},[old]);
  test(fixture(NEW,old),{kind:"clear"},[null]);
  const down=fixture(OLD,old);
  check(state(down).maxTarget===0,"old version admitted");rollback++;
  test(down,{kind:"migrate"},[null]);
  const migrated=transaction(down,{kind:"migrate"}).a;
  test(migrated,{kind:"set",value:120},[120]);
  for(let i=0;i<6;i++)for(let bit=0;bit<8;bit++){
   const a=fixture(NEW,old);a[ADDRESS+i]^=1<<bit;
   // Even if an ordinary settings checksum is subsequently recalculated,
   // record-local validation must independently reject the bit flip.
   a[INTEGRITY]=sum(a);check(state(a).value===null,"local corruption rearmed");
   corrupt++;
  }
  const bad=fixture(NEW,old);bad[200]^=1;
  check(state(bad).value===null,"global checksum invalid allowed");
  test(bad,{kind:"clear"},[null]);
 }
 for(const old of [null,0,80,120]){
  test(fixture(OLD,old),{kind:"migrate"},[null]);
 }
 return {complete,torn,ignored,corrupt,rollback,total:complete+torn+ignored+corrupt+rollback};
}
module.exports=study;
if(require.main===module)console.log(JSON.stringify(study(),null,2));

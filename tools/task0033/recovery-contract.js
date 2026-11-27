// Run: node tools/task0033/recovery-contract.js > tools/task0033/recovery-contract-results.json
function recoveryResearch(){
  // Abstract transactional admission model; not a firmware patch or wire protocol.
  const assert=(x,m)=>{if(!x)throw Error(m)};
  function session(){
    let last=0,gate="OPEN",pending=null,queue=[],out=[],counters={FE:0,DOR:0,UPE:0,RING:0},token=0;
    function observeLoss(kind){
      if(!(kind in counters))throw Error(kind);
      counters[kind]=Math.min(255,counters[kind]+1);
      token++; // Abstract unbounded epoch; not an AVR implementation proposal.
      if(gate==="OPEN")gate="FENCED";
    }
    function parse(frame){
      if(gate!=="OPEN")return false;
      assert(!pending,"one parser transaction");
      pending={...frame,token};return true;
    }
    // Abstract atomic commit; no logging or long calculation inside.
    function commit(){
      if(!pending)return false;
      const f=pending;pending=null;
      if(gate!=="OPEN"||f.token!==token||!f.integrity)return false;
      if(f.number!==null&&f.number!==last+1)return false;
      queue.push(f.id);if(f.number!==null)last=f.number;
      out.push({kind:"ADMISSION_ACK_INTENT",id:f.id,number:f.number});return true;
    }
    function recover({numberedSession}){
      if(gate!=="FENCED")return false;
      pending=null;
      // The boundary proof and actual host compatibility are EXTERNAL prerequisites.
      // Loss before unnumbered admission cannot establish an expected command.
      if(!numberedSession){gate="ABORT";out.push({kind:"STOP_REQUIRED"});return false}
      gate="WAIT_SYNC";out.push({kind:"RESEND_INTENT",number:last+1});
      return true;
    }
    function boundaryProof(verified){
      // A test oracle representing an external, already-verified protocol boundary.
      return {verified,token};
    }
    function sync(proof){
      if(gate!=="WAIT_SYNC"||proof?.verified!==true||proof.token!==token)return false;
      gate="OPEN";token++;return true;
    }
    function snapshot(){return {last,gate,pending:pending&&pending.id,queue:[...queue],out:out.map(x=>({...x})),counters:{...counters}}}
    return {observeLoss,parse,commit,recover,boundaryProof,sync,snapshot};
  }
  const clean=(id,number)=>({id,number,integrity:true});
  const traces=[];
  for(const kind of ["FE","DOR","UPE","RING"]){
    for(const injection of ["before_parse","between_parse_commit","after_commit"]){
      const s=session();
      assert(s.parse(clean("valid",1))&&s.commit(),"first valid");
      if(injection==="before_parse")s.observeLoss(kind);
      s.parse(clean("candidate",2));
      if(injection==="between_parse_commit")s.observeLoss(kind);
      const accepted=s.commit();
      if(injection==="after_commit")s.observeLoss(kind);
      assert(accepted===(injection==="after_commit"),"atomic admission decision");
      const expected=accepted?3:2;
      s.recover({numberedSession:true});
      assert(s.snapshot().gate==="WAIT_SYNC","not reopened on resend intent");
      assert(!s.parse(clean("tail",expected))&&!s.sync(s.boundaryProof(false)),"reject ambiguous tail");
      assert(s.sync(s.boundaryProof(true)),"external confirmed sync");
      assert(s.parse(clean("replay",expected))&&s.commit(),"expected replay");
      assert(s.snapshot().last===expected,"line commit monotonic");
      assert(s.snapshot().queue.join(",")===(accepted?"valid,candidate,replay":"valid,replay"),"validated order");
      traces.push({kind,injection,...s.snapshot()});
    }
  }
  {
    const s=session();s.parse(clean("unnumbered",null));s.observeLoss("RING");assert(!s.commit(),"reject unnumbered");
    s.recover({numberedSession:false});assert(s.snapshot().gate==="ABORT"&&!s.sync(s.boundaryProof(true)),"unreplayable frame requires stop");
    traces.push({case:"unnumbered",...s.snapshot()});
  }
  {
    const s=session();s.observeLoss("DOR");for(let i=0;i<300;i++)s.observeLoss("DOR");s.observeLoss("FE");
    assert(s.snapshot().counters.DOR===255&&s.snapshot().counters.FE===1,"saturating diagnostics");
    s.recover({numberedSession:true});assert(!s.sync(s.boundaryProof(false)),"zero bytes alone are not verified proof");
    traces.push({case:"repeated_loss",...s.snapshot()});
  }
  {
    const s=session();s.observeLoss("DOR");s.recover({numberedSession:true});
    const old=s.boundaryProof(true);
    s.observeLoss("FE");
    assert(!s.sync(old),"loss while syncing invalidates previously verified boundary");
    assert(s.sync(s.boundaryProof(true)),"fresh boundary after last loss");
    assert(!s.sync(old),"no stale proof reuse");
    traces.push({case:"loss_during_sync",...s.snapshot()});
  }
  return {model:"task0033-recovery-contract-v2",assumptions:"Abstract transaction and state-machine tests only. No actual recovery parser, transport, host, CRC, interrupt cycle cost, hardware fault or proof of a valid binary synchronization boundary. External sync proof deliberately not implemented. ACK/RESEND intents are not literal wire replies; actual Repetier compatibility and safe stop integration are unresolved. Queue shown for validation order, not finite capacity or policy on abort. No guarantee against unflagged checksum collisions. Epoch is abstract; use a sticky latch plus carefully bounded atomic handshake in AVR, not a wrapping per-byte generation counter.",traces};
}
console.log(JSON.stringify(recoveryResearch(), null, 2));

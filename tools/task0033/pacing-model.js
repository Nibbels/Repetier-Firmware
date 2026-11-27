// Run: node tools/task0033/pacing-model.js > tools/task0033/pacing-model-results.json
function pacingResearch(){
  const assert=(x,m)=>{if(!x)throw Error(m)};
  function simulate({baud,mode="credit",credit=127,commandBytes=40,ackBytes=6,hostDelayUs=1000,parserPeriodUs=200,executePeriodUs=500,ackExtraUs=0,commands=1000,burstExtraBytes=0}){
    const frameUs=10e6/baud;
    let sent=0,admitted=0,executed=0,acked=0,inflight=0,maxInflight=0,ring=0,maxRing=0,queue=0,maxQueue=0,dropped=0,parserBytes=0;
    let nextParse=parserPeriodUs,nextExecute=executePeriodUs,wireRx=0,wireTx=0,hostReady=0,now=0,starved=0,blocked=0,rxEvents=[],acks=[],heads=0,tail=0,rxBytes=0,txBytes=0,creditStalls=0;
    function canSend(){return sent<commands&&(mode==="pingpong"?sent===acked:inflight+commandBytes<=credit)}
    if(!canSend())return {status:"window_too_small",baud,mode,credit,commandBytes};
    while(acked<commands||executed<commands){
      if(canSend()&&hostReady<=now){
        sent++;inflight+=commandBytes;maxInflight=Math.max(maxInflight,inflight);
        wireRx=Math.max(wireRx,now);
        for(let i=0;i<commandBytes;i++){wireRx+=frameUs;rxEvents.push(wireRx)}
      }else if(sent<commands&&!canSend()){creditStalls++}
      // RX frame completions happen before parser service on equal timestamp.
      while(heads<rxEvents.length&&rxEvents[heads]<=now+1e-8){
        heads++;rxBytes++;if(ring===127)dropped++;else {ring++;maxRing=Math.max(maxRing,ring)}
      }
      while(tail<acks.length&&acks[tail]<=now+1e-8){
        tail++;acked++;inflight-=commandBytes;hostReady=Math.max(hostReady,now+hostDelayUs);
      }
      if(nextParse<=now+1e-8){
        if(queue<2){
          const n=Math.min(ring,commandBytes-parserBytes);ring-=n;parserBytes+=n;
          if(parserBytes===commandBytes){
            admitted++;queue++;maxQueue=Math.max(maxQueue,queue);parserBytes=0;
            wireTx=Math.max(wireTx,now+ackExtraUs)+ackBytes*frameUs;txBytes+=ackBytes;acks.push(wireTx);
          }
        }else blocked++;
        nextParse+=parserPeriodUs;
      }
      if(nextExecute<=now+1e-8){
        if(queue){executed++;queue--}else if(executed<commands)starved++;
        nextExecute+=executePeriodUs;
      }
      assert(inflight>=0&&ring<=127&&queue<=2&&acked<=admitted&&admitted<=sent,"accounting");
      if(dropped)return {status:"loss",baud,mode,credit,commandBytes,sent,admitted,acked,executed,ring,maxRing,dropped,rxBytes,txBytes,maxInflight};
      if(acked===commands&&executed===commands)break;
      // All eligible sends can enqueue at this instant; host delay models
      // eligibility after most recently delivered ACK (deliberately conservative).
      if(canSend()&&hostReady<=now)continue;
      let next=Math.min(nextParse,nextExecute,heads<rxEvents.length?rxEvents[heads]:Infinity,tail<acks.length?acks[tail]:Infinity,canSend()&&hostReady>now?hostReady:Infinity);
      assert(next>now,"progress");
      now=next;
      assert(now<60e6,"simulation horizon");
    }
    assert(rxBytes===commands*commandBytes&&txBytes===commands*ackBytes&&inflight===0&&ring===0&&queue===0,"final accounting");
    return {status:"ok",baud,mode,credit,commandBytes,ackBytes,hostDelayUs,parserPeriodUs,executePeriodUs,ackExtraUs,commands,elapsedUs:Math.round(now),commandsPerSecond:Math.round(commands*1e6/now*100)/100,maxRing,maxQueue,maxInflight,starvedExecutionSlots:starved,blockedParserVisits:blocked,rxBytes,txBytes,uartCpuPct:Math.round((rxBytes*73+txBytes*74)/(16*now)*10000)/100};
  }
  assert(simulate({baud:500000,commands:10,credit:10}).status==="window_too_small","oversized strict credit");
  assert(simulate({baud:500000,commands:20,credit:200,parserPeriodUs:10000}).status==="loss","oversized credit causes ring overflow");
  assert(simulate({baud:500000,commands:20,credit:127,parserPeriodUs:10000}).status==="ok","credit bounds receive ring under assumptions");
  const results=[];
  for(const baud of [115200,250000,500000,1000000,2000000])
    for(const commandBytes of [19,40])
      for(const [mode,credit] of [["pingpong",127],["credit",63],["credit",127]])
        for(const hostDelayUs of [0,1000,4000])
          results.push(simulate({baud,commandBytes,mode,credit,hostDelayUs}));
  const stress=[];
  for(const credit of [63,127,128,200,256])
    for(const parserPeriodUs of [200,2500,10000])
      stress.push(simulate({baud:500000,commandBytes:19,credit,parserPeriodUs,commands:100}));
  return {model:"task0033-pacing-v1",assumptions:"Lossless UART except RX ring capacity; complete commands of fixed wire length; strict byte credit including every framed byte; pingpong one command; ACK on parser admission, two parsed slots; fixed parser and execution service periods, synthetic ACK size, host delay applied after latest ACK; no real CPU scheduling, hardware FIFO, driver, ACK loss, resend, binary text borrowing, emergency, or motion queue. At higher baud results are optimistic regarding CPU; use UART cycle model separately.",results,stress};
}
console.log(JSON.stringify(pacingResearch(), null, 2));

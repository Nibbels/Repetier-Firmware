#!/usr/bin/env python3
"""Stage 1: sticky hardware loss + bounded TX + existing terminal transition.

Consume the exact qualified combined Drive/core patch in a disposable tree.
This is NOT the bounded resend receiver: legacy retry behaviour is still
present, and loss of an unidentified byte is deliberately terminal. No
hardware test, production merge, or latency qualification is implied.
"""
from pathlib import Path
import argparse
import hashlib
import json
import re
import runpy
import subprocess

HERE=Path(__file__).resolve().parent
drive=runpy.run_path(str(HERE/"current-drive-combined.py"))
trigger=runpy.run_path(str(HERE/"transport-terminal-trigger.py"))
ring=runpy.run_path(str(HERE/"resend-ring-preflight.py"))
writer=runpy.run_path(str(HERE/"tx-blocking-preflight.py"))
once=ring["once"]

def candidate(fw):
    # Reuse actual-source terminal transition and foreground service seams.
    files=trigger["candidate"](fw)
    h=(fw/"UsbSerial.h").read_text()
    s=(fw/"UsbSerial.cpp").read_text()
    # The historical ring generator anchors at the old variable-rate init.
    # Adapt ONLY the signature while generating the additions.
    s=once(s,"void UsbSerial::begin() {","void UsbSerial::begin(unsigned long baud) {")
    h,s=ring["candidate_uart"](h,s)
    h,s=ring["packed_candidate"](h,s)
    s=once(s,"void UsbSerial::begin(unsigned long baud) {","void UsbSerial::begin() {")
    h,s=writer["bounded_writer"](h,s)
    # A full write needs only one slot (one wire character in the
    # uninterrupted case), not an entire 63-byte drain. Trial bound:
    # 4096 polls, ~2.82 ms active CPU at the measured 11 cycles/poll.
    # This is deliberately not a claim about wall time or final IRQ
    # feasibility. Keep the historical full-drain budget for stage1's
    # legacy flush only; stage2 has no production flush caller.
    s=once(s,"""    if(next==tx_.tail) {
        uint16_t budget=UINT16_MAX;""",
        """    if(next==tx_.tail) {
        uint16_t budget=4096;""")
    # Foreground and USART ISR use SPSC rings. Volatile cursors alone
    # do not order nonvolatile payload memory across compiler scheduling.
    # No cli: payload ownership changes only at the one-byte cursor.
    s=once(s,"""    const uint8_t byte = rx_.bytes[rx_.tail];
    rx_.tail = rx_.next(rx_.tail);""",
        """    asm volatile("" ::: "memory");
    const uint8_t tail=rx_.tail;
    const uint8_t byte=rx_.bytes[tail];
    asm volatile("" ::: "memory");
    rx_.tail=rx_.next(tail);""")
    s=once(s,"    tx_.bytes[tx_.head] = byte;\n    tx_.head = next;",
        "    tx_.bytes[tx_.head] = byte;\n    asm volatile(\"\" ::: \"memory\");\n    tx_.head = next;")
    # Reopening an ambiguous physical loss is not a permitted operation.
    # Retain only the request-prefix fence, and never let it erase errors
    # arriving before, during or after its short atomic publication.
    h=once(h,"    static uint8_t rearmForResync();\n","")
    start=s.index("uint8_t UsbSerial::rearmForResync() {")
    end=s.index("\n}",start)+2
    s=s[:start]+s[end:]
    s=once(s,"    cli();\n    rx_.tail=rx_.head;",
        """    cli();
    if(lossFlags_ || blockingTxFailed_) {
        asm volatile("" ::: "memory");
        SREG=previous;
        asm volatile("" ::: "memory");
        return false;
    }
    rx_.tail=rx_.head;""")
    s=once(s,"""    const uint8_t state=lossFlags_;
    if(state&0x80)return;
    const uint8_t loss=state | (status & (_BV(FE0) | _BV(DOR0) | _BV(UPE0)));
    if(loss) { lossFlags_=loss; return; }""",
        """    const uint8_t state=lossFlags_;
    const uint8_t loss=state | (status & (_BV(FE0) | _BV(DOR0) | _BV(UPE0)));
    if(loss) {
        if(loss!=state)lossFlags_=loss;
        return;
    }""")
    s=once(s,"if((lossFlags_&0x80) && next==resyncTxMarker_)lossFlags_=0;",
        "if((lossFlags_&0x80) && next==resyncTxMarker_)lossFlags_ &= 0x7f;")
    h=once(h,"    static bool blockingTxFailed(){return blockingTxFailed_;}",
        "    static bool blockingTxFailed(){return blockingTxFailed_;}\n"
        "    static bool transportFailed(){return blockingTxFailed() || lossFlags();}")
    # Fixed init is cold start only. Account for all three state variables
    # before re-enabling USART IRQs, together with the existing ring reset.
    s=once(s,"    tx_.head = tx_.tail = 0;",
        "    tx_.head = tx_.tail = 0;\n"
        "    txPublished_ = 0;\n    lossFlags_ = 0;\n"
        "    blockingTxFailed_ = false;")
    files["UsbSerial.h"],files["UsbSerial.cpp"]=h,s

    # No RX gate reopening is called by this stage. In particular a thermal
    # promotion must not turn a quarantined stream into diagnostic input.
    g=files["GCodeInput.cpp"]
    old="(UsbSerial::blockingTxFailed() && !Printer::printJob.thermalEmergency())"
    assert g.count(old)==4
    g=g.replace(old,"(UsbSerial::transportFailed())")
    # Retain exact standalone and combined receiver guards.
    assert g.count("if(UsbSerial::transportFailed())return;")==1
    g=once(g,"Printer::printJob.emergencyLocked() || UsbSerial::blockingTxFailed()",
        "Printer::printJob.emergencyLocked() || UsbSerial::transportFailed()")
    g=once(g,"    HAL::serialFlush();\n    frame.length = 0;",
        "    HAL::serialFlush();\n#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\n"
        "    if(UsbSerial::transportFailed())return;\n#endif\n    frame.length = 0;")
    g=once(g,"        frame.bytes[frame.length++] = HAL::serialReadByte();",
        "#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\n"
        "        if(UsbSerial::transportFailed())return;\n#endif\n"
        "        frame.bytes[frame.length++] = HAL::serialReadByte();")
    files["GCodeInput.cpp"]=g
    files["Commands.cpp"]=once(files["Commands.cpp"],
        "!Printer::printJob.emergencyLocked() && UsbSerial::blockingTxFailed()",
        "!Printer::printJob.emergencyLocked() && UsbSerial::transportFailed()")
    return files

def replay_uart(out,files):
    src=files["UsbSerial.cpp"]
    check="            blockingTxFailed_=true;\n            return;\n        }"
    assert src.count(check)==2
    # Host test only: emulate UDRE progress at a polling boundary.
    fixture=src.replace(check,check+"\n        researchWaitStep();")
    (out/"usb-serial-firmware.inc").write_text(re.sub(r"^#include <avr/\w+\.h>\n","",fixture,flags=re.M))
    test=writer["TEST"]
    test=once(test,"if(good)progressEvery=flush?1000:65534;",
        "if(good)progressEvery=flush?1000:4095;")
    test=once(test,"assert(visits==(flush?63000u:65534u));",
        "assert(visits==(flush?63000u:4095u));")
    test=once(test,"assert(visits==(masked?0u:65534u));",
        "assert(visits==(masked?0u:flush?65534u:4095u));")
    test=once(test,"UPE0=2 };","UPE0=2, UCSZ01=2, UCSZ00=1 };")
    test=once(test,"UCSR0A,UCSR0B,UBRR0H","UCSR0A,UCSR0B,UCSR0C,UBRR0H")
    test=once(test,"UsbSerial::begin(250000);","UsbSerial::begin();")
    # Extra actual ISR checks on the fixed-init composed source.
    test=once(test,'    const bool flush=mode.find("flush")!=std::string::npos;',
        r'''    if(mode=="hardware-loss") {
        UCSR0A=0;UDR0='a';USART0_RX_vect();
        UCSR0A=_BV(FE0);UDR0='b';USART0_RX_vect();
        assert(UsbSerial::transportFailed() && UsbSerial::lossFlags()==_BV(FE0));
        UCSR0A=0;UDR0='c';USART0_RX_vect();
        UCSR0A=_BV(DOR0);UDR0='d';USART0_RX_vect();
        assert(UsbSerial::lossFlags()==(_BV(FE0)|_BV(DOR0)));
        assert(UsbSerial::read()=='a' && UsbSerial::read()==-1);
        assert(UsbSerial::transportFailed());
        return 0;
    }
    if(mode=="marker-good" || mode=="marker-loss") {
        const char request[]="\r\nResend:2\r\nok\r\n";
        const uint8_t count=sizeof(request)-1;
        const uint8_t prefix=count-4;
        UCSR0A=0;UDR0='x';USART0_RX_vect();
        assert(UsbSerial::tryWriteResendSpan(request,count,prefix));
        assert(UsbSerial::resyncPending() && !UsbSerial::available());
        UDR0='y';USART0_RX_vect();assert(!UsbSerial::available());
        if(mode=="marker-loss"){
            UCSR0A=_BV(DOR0);UDR0='z';USART0_RX_vect();
            assert(UsbSerial::transportFailed());
        }
        for(unsigned n=0;n<prefix;++n){
            USART0_UDRE_vect();assert(UDR0==uint8_t(request[n]));
        }
        assert(!UsbSerial::resyncPending());
        UCSR0A=0;UDR0='r';USART0_RX_vect();
        if(mode=="marker-loss"){
            assert(UsbSerial::lossFlags()==_BV(DOR0));
            assert(!UsbSerial::available());
            const uint8_t old=SREG;
            assert(!UsbSerial::tryWriteResendSpan(request,count,prefix));
            assert(SREG==old && UsbSerial::lossFlags()==_BV(DOR0));
        }else assert(UsbSerial::read()=='r' && !UsbSerial::transportFailed());
        assert(drain()=="ok\r\n");
        return 0;
    }
    const bool flush=mode.find("flush")!=std::string::npos;''')
    (out/"uart-replay.cpp").write_text(test)
    for flags in (["-O2"],["-O1","-fsanitize=undefined","-fno-sanitize-recover=all"]):
        exe=out/"uart-replay"
        subprocess.run(["c++","-std=c++11","-Wall","-Wextra","-Werror",*flags,
            "-I",str(HERE),str(out/"uart-replay.cpp"),"-o",str(exe)],check=True)
        for mode in ("hardware-loss","marker-good","marker-loss","write-good","flush-good","write-masked","flush-masked","write-stall","flush-stall","flush-partial"):
            subprocess.run([str(exe),mode],check=True,timeout=10)
        exe.unlink()

def replay_scheduler(out,files):
    prelude=once(trigger["PRELUDE"],"static bool blockingTxFailed(){return txFailed;}",
        "static bool transportFailed(){return txFailed;}")
    # Scheduler mock still injects an abstract transport-failure bit; actual
    # RX and TX sources of this bit are exercised separately above.
    code=trigger["region"](files["Commands.cpp"],
        "#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT\nstatic bool serviceTransportFailure()",
        "bool Commands::waitForMotion")
    (out/"scheduler.cpp").write_text(prelude+code+trigger["SCENARIOS"])
    exe=out/"scheduler"
    subprocess.run(["c++","-std=c++11","-O1","-Wall","-Wextra","-Werror",
        "-fsanitize=undefined","-fno-sanitize-recover=all",str(out/"scheduler.cpp"),"-o",str(exe)],check=True)
    for mode in ("before-read","read","execute","keepalive","comp-ui","comp","temperature","print-temperature","temp-ui","dms","features","scan"):
        subprocess.run([str(exe),mode],check=True,timeout=10)
    exe.unlink()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source",type=Path,required=True)
    p.add_argument("--combined",type=Path,required=True)
    p.add_argument("--out",type=Path,required=True)
    a=p.parse_args()
    source=a.source.resolve();out=a.out.resolve();combined=a.combined.resolve()
    assert drive["git"](source,"rev-parse","HEAD")==drive["P0"]
    assert subprocess.run(["git","-C",str(source),"symbolic-ref","-q","HEAD"],
        capture_output=True).returncode==1
    assert not drive["git"](source,"status","--porcelain")
    assert out!=source and source not in out.parents
    out.mkdir(parents=True,exist_ok=True)
    manifest=json.loads(combined.with_name("combined.json").read_text())
    assert manifest["p0"]==drive["P0"]
    assert manifest["combined_sha256"]==hashlib.sha256(combined.read_bytes()).hexdigest()
    subprocess.run(["git","-C",str(source),"apply","--check",str(combined)],check=True)
    subprocess.run(["git","-C",str(source),"apply",str(combined)],check=True)
    fw=source/"Repetier"
    files=candidate(fw)
    for name,s in files.items():(out/name).write_text(s)
    replay_uart(out,files)
    replay_scheduler(out,files)
    for name,s in files.items():(fw/name).write_text(s)
    receiver=runpy.run_path(str(HERE/"current-transport-receiver-replay.py"))
    receiver["run"](source,out/"receiver")
    subprocess.run(["git","-C",str(source),"diff","--check"],check=True)
    (out/"provenance.json").write_text(json.dumps({
        "p0":drive["P0"],
        "combined_sha256":hashlib.sha256(combined.read_bytes()).hexdigest(),
        "candidate_sha256":{n:hashlib.sha256(s.encode()).hexdigest() for n,s in sorted(files.items())},
        "scope":"Stage 1 only: actual-source UART, outer receiver and scheduler seams; bounded resend and full canonical replay outstanding"
    },indent=2)+"\n")

if __name__=="__main__":
    main()

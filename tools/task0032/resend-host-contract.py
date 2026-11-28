#!/usr/bin/env python3
"""Execute pinned upstream host wire methods with isolated serial/UI stubs.

Only selected reviewed functions are compiled from the Python AST, no imports
or application startup from the upstream projects. Historical C# Host is
source-inspected, NOT executed. No printer or proprietary Repetier binaries.
Record exact wire bytes and history membership, not claims about every host.
"""
import argparse
import ast
import functools
import hashlib
import json
import logging
from pathlib import Path
import queue
import threading
from types import SimpleNamespace
import typing
import urllib.request

SOURCES = {
    "repetier": ("InlineTwin/Repetier-Host","fe9ec6ed7a7286120c842b3060259a06b2a59b00",
                 "src/RepetierHost/model/PrinterConnection.cs","c2a7d7ee454781d565ad98ef4aeb1cd2ee8b5866"),
    "printrun": ("kliment/Printrun","ecd260f14adf7e659eff92c4326240f0bb282e25",
                 "printrun/printcore.py","4819738d11d3fa1e6d9fc37b4579deabbb7d09e0"),
    "cura": ("Ultimaker/Cura","1a221ec29a86960d16a412940becc1f63534c41f",
             "plugins/USBPrinting/USBPrinterOutputDevice.py","85b98e532b6d74e534a88af6105fbeec69e9b89a"),
    "octoprint": ("OctoPrint/OctoPrint","38fd87601464ec921eacab28315b2d7944f13f07",
                  "src/octoprint/plugins/serial_connector/serial_comm.py","4ab87eed45bb943f821128ab9fe9b03308b8b17e"),
}

def source(name, directory):
    repo,commit,path,blob=SOURCES[name]
    dest=directory/(name+Path(path).suffix)
    if not dest.exists():
        url=f"https://raw.githubusercontent.com/{repo}/{commit}/{path}"
        with urllib.request.urlopen(url,timeout=30) as response:
            data=response.read(2_000_001)
        assert len(data)<=2_000_000
        dest.write_bytes(data)
    data=dest.read_bytes()
    assert hashlib.sha1(f"blob {len(data)}\0".encode()+data).hexdigest()==blob,name
    return data.decode("utf-8-sig")

def methods(src, names, globals_):
    """Compile the exact selected AST methods, with no upstream module imports."""
    tree=ast.parse(src)
    classes=[c for c in ast.walk(tree) if isinstance(c,ast.ClassDef)
             and set(names)<={f.name for f in c.body if isinstance(f,ast.FunctionDef)}]
    assert len(classes)==1
    nodes=[f for f in classes[0].body if isinstance(f,ast.FunctionDef) and f.name in names]
    assert len(nodes)==len(names)
    for f in nodes:
        assert all(isinstance(d,ast.Name) and d.id=="property" for d in f.decorator_list)
    cls=ast.ClassDef(name="Probe",bases=[],keywords=[],body=nodes,decorator_list=[])
    mod=ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[]))
    env=dict(globals_)
    exec(compile(mod,"<pinned host methods>","exec"),env)
    return env["Probe"]()

def wire_xor(line):
    return functools.reduce(lambda a,b:a^b,line,0)

def numbered(n,cmd,space=True):
    p=f"N{n}".encode()+(b" " if space else b"")+cmd.encode()
    return p+b"*"+str(wire_xor(p)).encode()+b"\n"

def histor_host(src):
    # Source evidence, not execution: one number/history stream includes
    # injected/manual commands. The binary synchronization choice is the
    # host's negotiated binaryVersion, NOT the previous frame's first byte.
    manual=src[src.index("// then check for manual commands"):src.index("// do we have a printing job?")]
    job=src[src.index("// do we have a printing job?"):src.index("catch (InvalidOperationException ex)")]
    for block in (manual,job):
        assert "gc.N = ++lastline;" in block
        assert "gc.getAscii(true, true)" in block and "gc.getBinary(binaryVersion)" in block
        assert "historygc = gc;" in block
    assert "StoreHistory(historygc);" in src
    assert "history.Count > 40" in src and "new byte[32]" in src
    assert "ignoreNextOk = true;" in src and "if (!ignoreNextOk)" in src
    assert "nackLines.Clear(); // printer flushed all coming commands" in src
    assert src.index("nackLines.Clear(); // printer flushed all coming commands")<src.index("int send = receivedCount();")
    # At 250000 baud the C# integer sleep truncates 1.28 ms to 1 ms.
    # Streaming credit is cleared BEFORE receivedCount(), so it is not a
    # measured bound on bridge/OS bytes still in flight.
    return {"manual_and_job_numbered":True,"history":40,"binary_zero_bytes":32,
            "swallows_control_ok":True,"ascii_guard_ms_at_127_credit":127*10000//250000,
            "binary_zero_sleep_ms":320000//250000,"executed":False}

class Serial:
    has_flow_control=False
    def __init__(self):self.wire=[]
    def write(self,data):self.wire.append(bytes(data))

def printrun(src):
    p=methods(src,["_checksum","_send","_sendnext","_listen"],{"reduce":functools.reduce,"logging":logging})
    p.printer=Serial();p.sent=[];p.sentlines={};p.analyzer=SimpleNamespace(append=lambda *a,**kw:None)
    p._callback=lambda *a,**kw:None;p.loud=False;p.printing=p.online=p.clear=True
    p.tcp_streaming_mode=False;p.priqueue=queue.Queue();p.lineno=3;p.resendfrom=-1
    p._send("G1 X2",2,True)
    p.priqueue.put("M104 S190")
    p._sendnext()
    assert p.printer.wire==[numbered(2,"G1 X2"),b"M104 S190\n"]
    assert list(p.sentlines)==[2]
    p.greetings=[];p._logError=lambda line:None
    def listen(line):
        data=iter([line,None])
        p._readline=lambda:next(data)
        p._listen_can_continue=lambda:True
        p._listen()
    # Actual listener does NOT consume the control ok as a special token.
    # Model its arrival after the retransmission was sent, before actual ACK.
    listen("Resend:2")
    assert p.resendfrom==2 and p.clear
    p._sendnext()
    assert p.printer.wire[-1]==numbered(2,"G1 X2") and not p.clear
    listen("ok")
    assert p.clear
    p.priqueue.put("M105")
    p._sendnext()
    assert p.printer.wire[-1]==b"M105\n"
    return {"wire":[b.hex() for b in p.printer.wire],"history":list(p.sentlines),
            "control_ok_releases_clear":True,"executed":True}

def cura(src):
    state=SimpleNamespace(Connected=1)
    c=methods(src,["_sendCommand","_sendNextGcodeLine"],
              {"Union":typing.Union,"cast":typing.cast,"ConnectionState":state,
               "functools":functools,"time":lambda:1.0})
    c._serial=Serial();c._connection_state=1;c._command_received=threading.Event()
    job=SimpleNamespace(updateTimeElapsed=lambda n:None,updateTimeTotal=lambda n:None)
    c._printers=[SimpleNamespace(activePrintJob=job)]
    c._gcode=["G1 X1","G1 X2","G1 X3"]
    c._gcode_position=1;c._print_start_time=0;c._print_estimated_time=100
    c._sendNextGcodeLine()
    c._sendCommand("M104 S190")
    # _update's resend branch rewinds file position, not manual-command history.
    c._gcode_position=1
    c._sendNextGcodeLine()
    assert c._serial.wire==[numbered(1,"G1 X2",False),b"M104 S190\n",numbered(1,"G1 X2",False)]
    assert 'self._sendCommand(self._command_queue.get())' in src
    return {"wire":[b.hex() for b in c._serial.wire],"manual_history":False,"executed":True}

def octoprint(src):
    o=methods(src,["_needs_checksum","_checksum_enabled","_do_send","_do_increment_and_send_with_checksum",
                   "_do_send_with_checksum"],{})
    o._serial_encoding="ascii";o._checksum_requiring_commands=set()
    o._send_checksum_with_unknown_commands=False;o._send_checksum="always"
    o._currentFile=SimpleNamespace(checksum=True);o._manualStreaming=False
    o._firmware_info_received=True;o.isPrinting=lambda:True
    o._line_mutex=threading.Lock();o._current_line=2
    history=[];wire=[]
    o._addToLastLines=history.append
    o._do_send_without_checksum=lambda b:wire.append(b+b"\n")
    o._do_send("G1 X2","G1")
    o._do_send("M104 S190","M104")
    assert wire==[numbered(2,"G1 X2"),numbered(3,"M104 S190")]
    o._send_checksum="never"
    o._do_send("M104 S190","M104")
    assert wire[-1]==b"M104 S190\n" and len(history)==2
    o._send_checksum="always"
    o._do_send("custom unknown",None)
    assert wire[-1]==b"custom unknown\n" and len(history)==2
    return {"wire":[b.hex() for b in wire],"numbered_history":[b.decode() for b in history],
            "always_not_unconditional_for_unknown":True,"executed":True}

def ambiguity():
    # Identical observations at firmware: a missing unnumbered command is
    # invisible if its whole frame was lost. Resend of N2 cannot recover it.
    good=numbered(1,"G1 X1")
    replay=numbered(2,"G1 X2")
    observed=(good,"DOR",replay)
    histories=[{"sent":[good.hex(),replay.hex()],"missing_unnumbered":False},
               {"sent":[good.hex(),b"M104 S190\n".hex(),replay.hex()],"missing_unnumbered":True}]
    # ASCII checksum collision at a truncated text boundary: a valid M117
    # prefix can have XOR zero, leaving a suffix identical to a valid N2
    # frame. Neither expected number nor checksum proves external framing.
    suffix=numbered(2,"G1 X2")
    for ch in range(32,127):
        prefix=b"N3 M117 "+bytes([ch])+b" "
        if wire_xor(prefix)==0:break
    else:raise AssertionError("no XOR-neutral printable prefix")
    full=prefix+suffix
    assert wire_xor(full.split(b"*")[0])==wire_xor(suffix.split(b"*")[0])
    return {"identical_observation":[good.hex(),"DOR",replay.hex()],
            "histories":histories,"ascii_valid_text_frame":full.hex(),
            "ascii_valid_suffix":suffix.hex(),"conclusion":"boundary/history require host contract; checksums alone insufficient"}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out",type=Path,required=True)
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    src={n:source(n,args.out) for n in SOURCES}
    report={"sources":SOURCES,"historical_repetier":histor_host(src["repetier"]),
            "printrun":printrun(src["printrun"]),"cura":cura(src["cura"]),
            "octoprint":octoprint(src["octoprint"]),"ambiguity":ambiguity()}
    (args.out/"host-contract.json").write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report,indent=2))

if __name__=="__main__":
    main()

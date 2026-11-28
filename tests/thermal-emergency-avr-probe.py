#!/usr/bin/env python3
"""Link the diagnostic primitives with the real USART0 sender on AVR.
Research probe only; never flash. No integration/physical safety claim.
"""
from pathlib import Path
import json
import re
import shutil
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
cli=shutil.which("arduino-cli")
if not cli:
    print("Thermal emergency AVR probe skipped: arduino-cli unavailable")
    raise SystemExit(0)
props=subprocess.check_output([cli,"compile","--fqbn","arduino:avr:mega:cpu=atmega2560",
                              "--show-properties=expanded",str(root/"Repetier")],text=True)
bindir=Path(dict(s.split("=",1) for s in props.splitlines() if "=" in s)["compiler.path"])
code=r"""
#include <avr/interrupt.h>
#include "ThermalEmergencyReport.h"
#include "UsbSerial.h"
using namespace ThermalEmergency;
static Ledger<HEATER_COUNT> ledger;
static ReportCursor<HEATER_COUNT> report;
volatile uint8_t inputChannel,inputCause,requestReplay;
volatile float inputTemperature;
struct Sink {
    bool tryWriteSpan(const char* bytes,uint8_t count) {
        return UsbSerial::tryWriteSpan(bytes,count);
    }
};
static Sink sink;
static_assert(sizeof(ledger)<=96,"AVR emergency ledger grew");
static_assert(sizeof(report)<=20,"AVR report cursor grew");
extern "C" __attribute__((noinline,used,externally_visible)) void probeRecord() {
    Evidence e{};
    e.measuredC=inputTemperature;
    e.targetC=210;
    e.requestedDuty=255;
    e.valid=Evidence::Measured|Evidence::Target|Evidence::Requested;
    ledger.record(inputChannel,Cause(inputCause),e);
}
extern "C" __attribute__((noinline,used,externally_visible)) void probeReport() {
    if(requestReplay) report.requestReplay();
    report.service(ledger,sink);
}
int main() {
    UsbSerial::begin(115200);
    sei();
    for(;;) {probeRecord();probeReport();}
}
"""
with tempfile.TemporaryDirectory() as temp:
    cpp=Path(temp)/"probe.cpp";cpp.write_text(code)
    for count in (2,3):
        elf=Path(temp)/f"probe-{count}.elf"
        subprocess.run([str(bindir/"avr-g++"),"-mmcu=atmega2560","-DF_CPU=16000000UL",
                        f"-DHEATER_COUNT={count}","-std=gnu++11","-Os","-flto","-Wall","-Wextra","-Werror",
                        "-I",str(root/"Repetier"),str(cpp),str(root/"Repetier/UsbSerial.cpp"),"-o",str(elf)],check=True)
        nm=subprocess.check_output([str(bindir/"avr-nm"),"-S","-C",str(elf)],text=True)
        size=subprocess.check_output([str(bindir/"avr-size"),"-A",str(elf)],text=True)
        dump=subprocess.check_output([str(bindir/"avr-objdump"),"-d","-C",str(elf)],text=True)
        def symbol(name):
            m=re.search(r"^([0-9a-f]+)\s+([0-9a-f]+)\s+\w\s+"+re.escape(name)+r"$",nm,re.M)
            assert m,name
            return tuple(int(v,16) for v in m.groups())
        _,ledger_size=symbol("ledger")
        _,report_size=symbol("report")
        assert ledger_size<=96 and report_size<=20
        # The PSTR labels must not contribute to SRAM. Other real USART
        # and input variables are zero-initialized in this standalone image.
        data=re.search(r"^\.data\s+(\d+)\s+",size,re.M)
        assert data and int(data[1])==0,size
        assert "<probeRecord>:" in dump and "<probeReport>:" in dump
        print(json.dumps({"heater_count":count,"ledger_bytes":ledger_size,
                          "report_cursor_bytes":report_size,"sections":size}),flush=True)
        print(dump)

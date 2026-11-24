#!/usr/bin/env python3
"""Replay the actual boot-board check, including writes to the type EEPROM."""
import pathlib
import subprocess
import tempfile

src = (pathlib.Path(__file__).resolve().parents[1]/"Repetier"/"RF.cpp").read_text()
body = src[src.index("bool hardwareTypeMatches()"):src.index("} // hardwareTypeMatches")+len("} // hardwareTypeMatches")]
prefix = r"""
#include <cassert>
#include <vector>
#include <utility>
#define DEVICE_TYPE_RF1000 1
#define DEVICE_TYPE_RF2000 2
#define DEVICE_TYPE_RF2000v2 3
#define I2C_ADDRESS_TYPE_EEPROM 42
#define TYPE_EEPROM_OFFSET_HEADER_FORMAT 0
#define TYPE_EEPROM_OFFSET_BOARD_TYPE 2
#define TYPE_EEPROM_FORMAT 1234
using Event=std::pair<int,int>;
std::vector<Event> trace;
bool present;
unsigned short header, type;
namespace ExternalEeprom {
unsigned short readWord(int addr,int offset) {
    assert(addr==42); trace.emplace_back(offset,-1);
    return present ? (offset==0 ? header : type) : 0xffff;
}
void writeWord(int addr,int offset,unsigned short data) {
    assert(addr==42); trace.emplace_back(offset,data);
    if(present) (offset==0?header:type)=data;
}
}
"""
test = r"""
int main() {
    const bool rf1000 = MOTHERBOARD==DEVICE_TYPE_RF1000;
    present=false; header=0; type=0;
    assert(hardwareTypeMatches()==rf1000);
    assert((trace==std::vector<Event>{{0,-1},{0,1234},{2,MOTHERBOARD},{0,-1}}));
    trace.clear();
    present=true;
    assert(hardwareTypeMatches()==!rf1000);
    assert((trace==std::vector<Event>{{0,-1},{0,1234},{2,MOTHERBOARD},{0,-1}}));
    assert(header==1234 && type==MOTHERBOARD);
    trace.clear();
    assert(hardwareTypeMatches()==!rf1000);
    std::vector<Event> expected{{0,-1}};
    if(!rf1000) expected.emplace_back(2,-1);
    assert(trace==expected);
    trace.clear();
    type=99;
    assert(hardwareTypeMatches()==!rf1000);
    if(!rf1000) expected.emplace_back(2,MOTHERBOARD);
    assert(trace==expected);
    assert(type==(rf1000?99:MOTHERBOARD));
}
"""
with tempfile.TemporaryDirectory() as tmp:
    p=pathlib.Path(tmp)/"test.cpp"
    p.write_text(prefix+body+test)
    for model in ("RF1000","RF2000","RF2000v2"):
        exe=pathlib.Path(tmp)/model
        subprocess.run(["g++","-std=c++11","-DMOTHERBOARD=DEVICE_TYPE_"+model,str(p),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
        print(model+": board identity read/write trace passed")

#!/usr/bin/env python3
"""Compare the isolated runtime reset sequencer to actual pinned firmware init."""
import argparse
import hashlib
import json
import re
from pathlib import Path
import subprocess
import tempfile

HERE=Path(__file__).resolve().parent

FIXTURE=r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <vector>
#include <string>
#include <initializer_list>
#include "lcd-runtime-reset.h"
#define PROGMEM
using byte=uint8_t;
using millis_t=uint32_t;
#define pgm_read_byte(p) (*(p))
enum {UI_DISPLAY_D4_PIN,UI_DISPLAY_D5_PIN,UI_DISPLAY_D6_PIN,UI_DISPLAY_D7_PIN,
      UI_DISPLAY_RS_PIN,UI_DISPLAY_ENABLE_PIN};
enum Kind {Input,Output,Pin,Nibble,Command,Data};
struct Event {Kind kind;uint8_t value;uint32_t start,end;};
static std::vector<Event> events;
static uint32_t clockUs=0;
static bool candidate=false;
static void emit(Kind k,uint8_t v){
    uint32_t start=clockUs;
    if(candidate) clockUs+=1+(events.size()*7)%11;
    events.push_back({k,v,start,clockUs});
}
#define SET_INPUT(pin) emit(Input,pin)
#define SET_OUTPUT(pin) emit(Output,pin)
#define LOW false
#define LCD_WRITE(pin,value) emit(Pin,uint8_t((pin)*2+(value)))
namespace HAL {
void delayMicroseconds(unsigned us){clockUs+=us;}
void delayMilliseconds(unsigned ms){clockUs+=uint32_t(ms)*1000;}
millis_t timeInMilliseconds(){return clockUs/1000;}
}
static void lcdWriteNibble(uint8_t v){emit(Nibble,v);}
static void lcdWriteByte(uint8_t v,uint8_t rs){
    HAL::delayMicroseconds(320);
    emit(rs?Data:Command,v);
}
struct UIDisplay {
    millis_t lastSwitch=0,lastRefresh=0;
    bool normalCharacters_=false;
    void loadNormalCharacters();
    void initializeLCD(bool normal);
    void createChar(uint8_t location,const uint8_t charmap[]);
} uid;
'''

MAIN=r'''
struct Sink {
    uint32_t now()const{return clockUs&~uint32_t(3);}
    void inputs(){oldInputs();}
    void outputs(){oldOutputs();}
    void nibble(uint8_t v){emit(Nibble,v);}
    void write(uint8_t v,bool rs){emit(rs?Data:Command,v);}
    uint8_t glyph(uint8_t slot,uint8_t row){assert(slot>=1&&slot<=7&&row<8);return glyphs[slot-1][row];}
};

int main(){
    uid.initializeLCD(true);
    assert(uid.normalCharacters_);
    const auto original=events;
    assert(original.size()==6+9+4+4+63);
    unsigned long long visits=0;
    uint32_t fastest=0xffffffffu,slowest=0;
    for(uint32_t offset:{0u,1u,2u,3u,0xfffffff0u,0xfffffff1u,0xfffffff2u,0xfffffff3u}){
        for(uint32_t period:{4u,31u,100u,250u,1000u,5000u,10000u}){
            candidate=true;clockUs=offset;events.clear();
            LcdRuntimeReset reset;
            Sink sink;
            assert(reset.begin(sink));
            assert(!reset.begin(sink));
            const auto before=events.size();
            // A small backwards clock observation must not authorize output.
            const uint32_t start=clockUs;
            clockUs=(sink.now()-20);
            assert(!reset.service(sink)&&events.size()==before);
            clockUs=start;
            unsigned local=0;
            while(reset.active()){
                assert(++local<200000);
                clockUs+=period;
                const auto n=events.size();
                const bool changed=reset.service(sink);
                assert(events.size()==n || changed);
                if(events.size()>n) {
                    const Kind k=events[n].kind;
                    assert(events.size()==n+1 || k==Output);
                }
                // A concurrent request cannot restart the sequence.
                if(reset.active()) assert(!reset.begin(sink));
            }
            visits+=local;
            assert(events.size()==original.size());
            for(size_t i=0;i<events.size();++i){
                assert(events[i].kind==original[i].kind&&events[i].value==original[i].value);
                if(i){
                    const uint32_t minimum=original[i].start-original[i-1].end;
                    assert(uint32_t(events[i].start-events[i-1].end)>=minimum);
                }
            }
            assert(uint32_t(clockUs-events.back().end)>=320);
            const auto duration=uint32_t(clockUs-offset);
            if(duration<fastest)fastest=duration;
            if(duration>slowest)slowest=duration;
            assert(!reset.service(sink));
        }
    }
    std::printf("runtime reset: 56 exact traces, %zu events each, %llu service visits, simulated duration %u..%u us PASS\n",
                original.size(),visits,unsigned(fastest),unsigned(slowest));
}
'''

def section(s,first,last):
    i=s.index(first)
    return s[i:s.index(last,i+len(first))]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source",type=Path,required=True)
    p.add_argument("--out",type=Path,required=True)
    a=p.parse_args()
    source=(a.source/"Repetier/ui.cpp").read_text()
    init=section(source,"void UIDisplay::initializeLCD(bool normal)","} // initializeLCD")+"}\n"
    normal=section(source,"void UIDisplay::loadNormalCharacters()","void UIDisplay::initializeLCD(")
    create=section(source,"void UIDisplay::createChar(","} // createChar")+"}\n"
    pairs=re.findall(r"createChar\((\d+),\s*(\w+)\);",normal)
    assert [int(slot) for slot,_ in pairs]==list(range(1,8))
    arrays=[]
    for _,name in pairs:
        m=re.search(r"const (?:uint8_t|byte) "+re.escape(name)+r"\[8\] PROGMEM = \{[^}]+\};",source)
        assert m,name
        arrays.append(re.sub(r"\bB([01]{5})\b",lambda x:str(int(x[1],2)),m[0]))
    macros={}
    for line in source.splitlines():
        if line.startswith("#define LCD_") and not line.startswith("#define LCD_WRITE"):
            macros[line.split()[1]]=line.split("/**")[0].strip()
    definitions="\n".join(macros.values())+"\n#define lcdPutChar(value) lcdWriteByte(value,1)\n#define lcdCommand(value) lcdWriteByte(value,0)\n"
    inputs="\n".join(l for l in init.splitlines() if l.strip().startswith("SET_INPUT("))
    outputs="\n".join(l for l in init.splitlines() if l.strip().startswith(("SET_OUTPUT(","LCD_WRITE(")))
    assert inputs.count("SET_INPUT(")==6 and outputs.count("SET_OUTPUT(")==6 and outputs.count("LCD_WRITE(")==3
    helpers="void oldInputs(){\n"+inputs+"\n}\nvoid oldOutputs(){\n"+outputs+"\n}\n"
    helpers+="static const uint8_t* glyphs[]={"+",".join(name for _,name in pairs)+"};\n"
    # Exact original pin/nibble/command/glyph sequence, plus timing from
    # actual source. The candidate must not depend on fixture expectations.
    cpp=FIXTURE+definitions+"\n".join(arrays)+"\n"+normal+init+create+helpers+MAIN
    a.out.mkdir(parents=True,exist_ok=True)
    (a.out/"runtime-reset-fixture.cpp").write_text(cpp)
    with tempfile.TemporaryDirectory(prefix="task0032-runtime-reset-") as directory:
        d=Path(directory);exe=d/"reset"
        subprocess.run(["c++","-std=c++11","-O1","-Wall","-Wextra","-Werror",
                        "-fsanitize=undefined","-fno-sanitize-recover=all","-I",str(HERE),
                        a.out/"runtime-reset-fixture.cpp","-o",exe],check=True)
        result=subprocess.check_output([exe],text=True)
    print(result,end="")
    (a.out/"runtime-reset-results.json").write_text(json.dumps({
        "source_sha256":hashlib.sha256(source.encode()).hexdigest(),
        "sequencer_sha256":hashlib.sha256((HERE/"lcd-runtime-reset.h").read_bytes()).hexdigest(),
        "result":result.strip(),
        "qualification":"isolated actual-source logical/relative-delay replay; no integrated reset, resource or hardware claim"
    },indent=2)+"\n")

if __name__=="__main__":
    main()

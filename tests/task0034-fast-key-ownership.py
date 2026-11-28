#!/usr/bin/env python3
"""Replay actual fastAction after the PWM-only key ownership cleanup."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
src=(root/"Repetier/ui.cpp").read_text()
hal=(root/"Repetier/HAL.cpp").read_text()
callers=[]
for path in (root/"Repetier").iterdir():
    if path.suffix not in (".cpp",".h",".ino"):
        continue
    for line in path.read_text().splitlines():
        if ".fastAction(" in line or "->fastAction(" in line:
            callers.append((path.name,line.strip()))
assert callers==[("HAL.cpp","uid.fastAction();")],callers
assert "ISR(PWM_TIMER_VECTOR)" in hal
assert "UI_FLAG_KEY_TEST_RUNNING" not in src
body=src[src.index("void UIDisplay::fastAction() {"):src.index("} // fastAction")]+ "}\n"
harness=r'''
#include <cassert>
#include <cstdint>
#include <initializer_list>
static bool irq,stoppable;
static int16_t rawKey;
static unsigned stops,reads,clockReads;
struct InterruptProtectedBlock {
    bool saved=irq;
    InterruptProtectedBlock(){irq=false;}
    ~InterruptProtectedBlock(){irq=saved;}
};
namespace HAL {
    uint32_t timeInMilliseconds(){assert(!irq);++clockReads;return 0xabcdef01u;}
}
namespace PrintLine {
    bool hasStoppableDirectMove(){assert(!irq);return stoppable;}
    void stopDirectMove(){assert(!irq);++stops;}
}
struct UIDisplay {
    uint8_t flags;
    int16_t lastButtonAction,jogButtonAction;
    uint32_t lastButtonStart;
    void ui_check_keys(int16_t& action){assert(!irq);++reads;action=rawKey;}
    void fastAction();
};
''' + body + r'''
int main(){
    unsigned count=0;
    for(bool enabled:{false,true})
        for(bool moving:{false,true})
            for(unsigned flags=0;flags<256;++flags)
                for(int16_t old:{0,1,1001,1002})
                    for(int16_t jog:{0,1001,1002})
                        for(int16_t next:{0,1,1001,1002}){
                            irq=enabled;stoppable=moving;rawKey=next;
                            stops=reads=clockReads=0;
                            UIDisplay u{uint8_t(flags),old,jog,0x12345678u};
                            u.fastAction();
                            assert(irq==enabled && reads==1 && u.flags==flags);
                            assert(u.lastButtonAction==next);
                            assert(clockReads==unsigned(old!=next));
                            assert(u.lastButtonStart==(old==next?0x12345678u:0xabcdef01u));
                            assert(stops==unsigned(moving && !(next && next==jog)));
                            ++count;
                        }
    assert(count==49152);
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"fast.cpp";exe=Path(d)/"fast"
    p.write_text(harness)
    subprocess.run(["c++","-std=c++11","-O2","-Wall","-Wextra","-Werror",
                    str(p),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print("Task0034 fast key ownership: 49152 state combinations, both entry I states PASS")

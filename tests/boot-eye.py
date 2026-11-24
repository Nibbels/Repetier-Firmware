"""Replay the real boot-only LCD renderer and deadline at both panel widths."""
import os
from pathlib import Path
import re
import subprocess
from tempfile import TemporaryDirectory

HARNESS = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <algorithm>
#define PROGMEM
#define PSTR(x) x
#define UI_ROWS 4
#define LCD_CLEAR 1
#define LCD_SETCGRAMADDR 0x40
#define UI_PRINTER_NAME "RFx000"
using millis_t = uint32_t;
using byte = uint8_t;
static const char versionString[] = "V test";
static const uint8_t LCDLineOffsets[] = {0,0x40,UI_COLS,0x40+UI_COLS};
template<class T> uint8_t pgm_read_byte(const T* p) { return uint8_t(*p); }
namespace HAL {
millis_t now = 0;
millis_t timeInMilliseconds() { return now; }
void delayMilliseconds(unsigned n) { now += n; }
}
namespace Printer {
bool error = false;
bool isUIErrorMessage() { return error; }
}
uint8_t ddram[128], cgram[64];
uint8_t addr;
bool cg = false;
unsigned transfers = 0, clears = 0, refreshes = 0;
void lcdCommand(uint8_t command) {
    ++transfers;
    if (command == LCD_CLEAR) {
        memset(ddram, ' ', sizeof ddram); ++clears; addr=0; cg=false;
    } else if (command & 0x80) {
        addr=command & 0x7f; cg=false;
    } else {
        assert(command & 0x40); addr=command & 63; cg=true;
    }
}
void lcdPutChar(uint8_t value) {
    ++transfers;
    if (cg) { assert(value <= 31 && addr < 64); cgram[addr++]=value; }
    else { assert(addr < 128); ddram[addr++]=value; }
}
class UIDisplay {
public:
    bool normalCharacters_=false;
    uint8_t menuLevel=0;
    millis_t lastSwitch=0,lastRefresh=0;
    char displayCache_[4][UI_COLS+1]{};
    void beginBootEye(); void serviceBootEye(); void finishBootEye();
    void drawBootEye(uint16_t);
    void createChar(uint8_t,const uint8_t*);
    void loadNormalCharacters();
    void refreshPage() { ++refreshes; assert(normalCharacters_); }
    void printRowP(uint8_t row, const char* s) {
        lcdCommand(128 + LCDLineOffsets[row]);
        for (unsigned i=0;i<UI_COLS;++i) {
            char ch = *s ? *s++ : ' ';
            lcdPutChar(ch); displayCache_[row][i]=ch;
        }
        displayCache_[row][UI_COLS]=0;
    }
};
#include "firmware.inc"

uint8_t pixel(unsigned row,unsigned x) { return ddram[LCDLineOffsets[row]+x]; }
void expectPupil(unsigned x, bool blink=false) {
    if (blink) {
        assert(pixel(1,x)=='-' && pixel(1,x+1)=='-');
        assert(pixel(2,x)==' ' && pixel(2,x+1)==' ');
    } else {
        assert(pixel(1,x)==5 && pixel(1,x+1)==6);
        assert(pixel(2,x)==7 && pixel(2,x+1)==0);
    }
    for (unsigned row=0;row<4;++row)
        for(unsigned col=0;col<UI_COLS;++col) {
            const auto v=pixel(row,col);
            if (v<8) assert((row==0||row==3) ? (col==UI_COLS/2-5||col==UI_COLS/2+4) : (col==x||col==x+1));
        }
}
int main() {
    UIDisplay u;
    HAL::now=0xfffffff0u; // exercise unsigned wrap
    u.beginBootEye();
    const auto start=u.lastSwitch;
    assert(!u.normalCharacters_ && clears==1);
    for(unsigned i=0;i<8;++i)
        assert(memcmp(cgram+8*((i+1)&7),bootEyeGlyphs[i],8)==0);
    expectPupil(UI_COLS/2-1);
    auto n=transfers;
    HAL::now=start+49; u.serviceBootEye(); assert(transfers==n);
    HAL::now=start+100; u.serviceBootEye(); expectPupil(UI_COLS/2+1);
    assert(transfers-n<=16);
    n=transfers; u.serviceBootEye(); assert(transfers==n);
    HAL::now=start+350; u.serviceBootEye(); expectPupil(UI_COLS/2-3);
    HAL::now=start+2800; u.serviceBootEye(); expectPupil(UI_COLS/2-3,true);
    HAL::now=start+3000; u.serviceBootEye(); expectPupil(UI_COLS/2-1);
    HAL::now=start+3900; u.serviceBootEye(); assert(u.normalCharacters_);
    assert(memcmp(cgram+8,character_back,8)==0);
    assert(memcmp(cgram+40,character_temperature,8)==0);
    assert(memcmp(cgram+48,c8,8)==0);
    assert(memcmp(cgram+56,character_ready,8)==0);
    assert(pixel(3,0)=='V');
    n=transfers; u.serviceBootEye(); u.finishBootEye(); assert(transfers==n);
    UIDisplay fast; fast.beginBootEye(); fast.finishBootEye(); assert(fast.normalCharacters_);
    n=transfers; fast.serviceBootEye(); assert(transfers==n);
    UIDisplay fault; fault.beginBootEye(); Printer::error=true;
    fault.serviceBootEye(); assert(fault.normalCharacters_ && refreshes==1);
    n=transfers; fault.serviceBootEye(); fault.finishBootEye(); assert(transfers==n);
    UIDisplay preexisting; preexisting.beginBootEye(); assert(transfers==n);
    preexisting.finishBootEye(); assert(preexisting.normalCharacters_ && refreshes==2);
    std::printf("Boot eye %d columns PASS\n", UI_COLS);
}
'''

def main():
    source = (Path(__file__).resolve().parents[1] / "Repetier" / "ui.cpp").read_text()
    glyphs = source[source.index("const uint8_t character_back"):source.index("#define LCD_ENTRYMODE")]
    glyphs = re.sub(r"\bB([01]{5})\b", r"0b\1", glyphs)
    normal = source[source.index("void UIDisplay::loadNormalCharacters()"):source.index("void UIDisplay::initializeLCD")]
    boot = source[source.index("static const uint8_t bootEyeGlyphs"):source.index("void UIDisplay::printRowP")]
    with TemporaryDirectory(prefix="rfx-boot-eye-") as tmp:
        p = Path(tmp)
        (p / "firmware.inc").write_text(glyphs + normal + boot)
        (p / "test.cpp").write_text(HARNESS)
        for width in (16,20):
            exe = p / "test"
            subprocess.run([os.environ.get("CXX", "c++"), "-std=c++14", "-O2",
                            "-Wall", "-Wextra", "-Werror", f"-DUI_COLS={width}",
                            str(p / "test.cpp"), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True, timeout=10)

if __name__ == "__main__":
    main()

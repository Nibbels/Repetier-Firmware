#!/usr/bin/env python3
"""Replay actual lighting GPIO/PWM ownership, all four board variants and X19 conflict."""
from pathlib import Path
import os, subprocess, tempfile
fw=Path(__file__).resolve().parents[1]/"Repetier"
source=(fw/"Lighting.cpp").read_text().replace('#include "Repetier.h"','')
stub=r'''
#include <cassert>
#include <cstdint>
#define __AVR_ATmega2560__ 1
#include "BoardConfiguration.h"
#include "Lighting.h"
bool output[100]{}, level[100]{};
unsigned writes[100]{};
#define LOW false
#define WRITE(pin,val) do {level[pin]=bool(val);++writes[pin];} while(0)
#define SET_OUTPUT(pin) do {output[pin]=true;} while(0)
#ifdef RGB_LIGHT_R_PIN
enum { COM4A1=7, COM4B1=5, COM4C1=3, WGM41=1, WGM42=3, WGM43=4, CS41=1 };
uint8_t TCCR4A=0xff,TCCR4B=0xff;
uint16_t TCNT4,ICR4,OCR4A,OCR4B,OCR4C;
#endif
'''
checks=r'''
using namespace Lighting;
static void check(Color c) {
    set(c);
#if HOTEND_COOLER_USES_PIN(CASE_LIGHT_PIN)
    assert(!writes[CASE_LIGHT_PIN] && !output[CASE_LIGHT_PIN]);
#elif defined(RGB_LIGHT_R_PIN)
    assert(level[CASE_LIGHT_PIN]==bool(c.r && c.r==c.g && c.g==c.b));
#else
    assert(level[CASE_LIGHT_PIN]==bool(c.r||c.g||c.b));
#endif
#ifdef RGB_LIGHT_R_PIN
    const uint8_t channels[]={c.r,c.g,c.b};
    const uint8_t pins[]={RGB_LIGHT_R_PIN,RGB_LIGHT_G_PIN,RGB_LIGHT_B_PIN};
    const uint8_t bits[]={COM4A1,COM4B1,COM4C1};
    const uint16_t compare[]={OCR4A,OCR4B,OCR4C};
    for(unsigned i=0;i<3;++i) {
        assert(output[pins[i]] && level[pins[i]]==(channels[i]==255));
        assert(bool(TCCR4A&(1<<bits[i]))==bool(channels[i] && channels[i]!=255));
        assert(compare[i]==uint16_t(channels[i])*80);
    }
    assert((TCCR4A&3)==(1<<WGM41) && ICR4==20480);
    assert(TCCR4B==((1<<WGM42)|(1<<WGM43)|(1<<CS41)));
#endif
}
int main() {
    initialize();
#ifdef RGB_LIGHT_R_PIN
    assert(available());
#else
    assert(available()==!HOTEND_COOLER_USES_PIN(CASE_LIGHT_PIN));
#endif
    check(Black);check(White);check(Red);
    for(unsigned i=0;i<256;++i)check({uint8_t(i),uint8_t(255-i),uint8_t(i^0x55)});
    for(uint32_t now:{0u,511u,512u,1023u,1024u,0xffffffffu,0u}) {
        serviceAlarm(now);
#ifdef RGB_LIGHT_R_PIN
        assert(level[RGB_LIGHT_R_PIN]==bool(now&512u));
        assert(!level[RGB_LIGHT_G_PIN]&&!level[RGB_LIGHT_B_PIN]);
#elif !HOTEND_COOLER_USES_PIN(CASE_LIGHT_PIN)
        assert(level[CASE_LIGHT_PIN]==bool(now&512u));
#endif
    }
    initialize();check(Black);
}
'''
variants=[
["-DMOTHERBOARD=DEVICE_TYPE_RF1000"],
["-DMOTHERBOARD=DEVICE_TYPE_RF2000"],
["-DMOTHERBOARD=DEVICE_TYPE_RF2000v2"],
["-DMOTHERBOARD=DEVICE_TYPE_RF2000","-DNUM_EXTRUDER=1","-DEXT0_EXTRUDER_COOLER_PIN=X44_PIN"],
["-DMOTHERBOARD=DEVICE_TYPE_RF1000","-DEXT0_EXTRUDER_COOLER_PIN=X19_PIN"],
["-DMOTHERBOARD=DEVICE_TYPE_RF2000","-DEXT0_EXTRUDER_COOLER_PIN=X19_PIN"],
]
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"lighting.cpp";exe=Path(d)/"lighting"
    p.write_text("#include <initializer_list>\n"+stub+source+checks)
    for flags in variants:
        subprocess.run([os.environ.get("CXX","c++"),"-std=c++11","-O2","-Wall","-Wextra","-Werror","-I",str(fw),*flags,str(p),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
        print("Lighting GPIO/PWM, 256 intensities and alarm rollover PASS",flags)

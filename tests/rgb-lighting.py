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
enum { COM4A1=7, COM4B1=5, COM4C1=3, WGM41=1, WGM42=3, WGM43=4, CS41=1, PH3=3, PH4=4, PH5=5, PH6=6 };
bool irq=true;
struct InterruptProtectedBlock {
    bool old=irq;
    InterruptProtectedBlock(){irq=false;}
    ~InterruptProtectedBlock(){irq=old;}
};
// Require the real driver to access the shared port under exclusion.
struct Port {
    uint8_t bits;
    operator uint8_t() const {assert(!irq);return bits;}
    void operator=(uint8_t v){assert(!irq);bits=v;}
    void operator&=(uint8_t v){*this=uint8_t(*this)&v;}
    void operator|=(uint8_t v){*this=uint8_t(*this)|v;}
};
Port PORTH{0xC7},DDRH{0xC7};
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
    const uint8_t pins[]={PH3,PH4,PH5};
    const uint8_t bits[]={COM4A1,COM4B1,COM4C1};
    const uint16_t compare[]={OCR4A,OCR4B,OCR4C};
    for(unsigned i=0;i<3;++i) {
        assert((DDRH.bits&(1<<pins[i])) && bool(PORTH.bits&(1<<pins[i]))==(channels[i]==255));
        assert(bool(TCCR4A&(1<<bits[i]))==bool(channels[i] && channels[i]!=255));
        assert(compare[i]==uint16_t(channels[i])*80);
    }
    assert((PORTH.bits&0xC7)==0xC7 && (DDRH.bits&0xC7)==0xC7 && irq);
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
        assert(bool(PORTH.bits&(1<<PH3))==bool(now&512u));
        assert(!(PORTH.bits&((1<<PH4)|(1<<PH5))) && (PORTH.bits&0xC7)==0xC7);
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

# Compile the actual board configuration for both permitted retrofits and
# formerly accepted conflicting PWM writers. No runtime registry is involved.
# Keep the expected diagnostic specific: an unrelated compiler error must not
# make a negative configuration test pass.
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"pin-ownership.cpp"
    p.write_text('#define __AVR_ATmega2560__ 1\n#include "BoardConfiguration.h"\n')
    cxx=[os.environ.get("CXX","c++"),"-std=c++11","-I",str(fw),"-fsyntax-only",str(p)]
    cooler_error="A hotend cooler cannot share a dedicated GPIO"
    board_error="The stepper board fan cannot share a dedicated GPIO"
    for model in ("RF1000","RF2000","RF2000v2"):
        base=["-DMOTHERBOARD=DEVICE_TYPE_"+model]
        cases=[
            (["-DEXT0_EXTRUDER_COOLER_PIN=Z_ENABLE_PIN"],cooler_error),
            (["-DEXT0_EXTRUDER_COOLER_PIN=62"],cooler_error),
            (["-DFAN_BOARD_PIN=Z_DIR_PIN"],board_error),
            (["-DFAN_BOARD_PIN=HEATER_0_PIN"],board_error),
            (["-DFAN_BOARD_PIN=HEATER_2_PIN"],board_error),
            (["-DFAN_BOARD_PIN=UI_DISPLAY_ENABLE_PIN"],board_error),
            (["-DEXT0_EXTRUDER_COOLER_PIN=67"],cooler_error),
            (["-DEXT0_EXTRUDER_COOLER_PIN=20"],cooler_error),
            (["-DEXT0_EXTRUDER_COOLER_PIN=WATCHDOG_PIN"],cooler_error),
            (["-DRF_THERMAL_EMERGENCY_EXPERIMENT","-DFAN_BOARD_PIN=THERMAL_ALARM_PIN"],board_error),
            (["-DFAN_BOARD_PIN=CASE_LIGHT_PIN"],board_error),
            (["-DFAN_BOARD_PIN=RESERVE_DIGITAL_PIN_PD3"],None),
            (["-DEXT0_EXTRUDER_COOLER_PIN=X19_PIN"],None),
        ]
        if model!="RF1000":
            cases += [
                (["-DFAN_BOARD_PIN=9"],board_error),
                (["-DEXT1_EXTRUDER_COOLER_PIN=Z_ENABLE_PIN"],cooler_error),
                (["-DFAN_BOARD_PIN=X43_R_PIN"],board_error),
                (["-DEXT0_EXTRUDER_COOLER_PIN=X43_G_PIN"],cooler_error),
                (["-DFAN_BOARD_PIN=X45_PIN"],board_error),
                (["-DFAN_BOARD_PIN=X44_PIN"],board_error),
                (["-DEXT0_EXTRUDER_COOLER_PIN=X42_PIN"],None),
                (["-DNUM_EXTRUDER=1","-DEXT0_EXTRUDER_COOLER_PIN=X44_PIN"],None),
            ]
        for flags,expected in cases:
            result=subprocess.run(cxx+base+flags,capture_output=True,text=True)
            if expected is None:
                assert result.returncode==0,(model,flags,result.stderr)
            else:
                assert result.returncode!=0 and expected in result.stderr,(model,flags,result.stderr)
        print("pin-ownership configuration checks PASS",model,len(cases))

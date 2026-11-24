#!/usr/bin/env python3
"""Replay the real DRV8711 translation unit with a host GPIO/HAL trace stub.

Run: python3 tests/drv8711-trace.py
CXX may select the host compiler. This is not an AVR cycle/timing measurement.
"""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
STUB = r'''
#pragma once
#include <cstdint>
#include <vector>
#include <array>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
using Event = std::array<int, 3>;
using Trace = std::vector<Event>;
static Trace actual;
enum { X_SCS_PIN=1, Y_SCS_PIN, Z_SCS_PIN, O0_SCS_PIN, O1_SCS_PIN,
       DRV_RESET1, DRV_SCLK, DRV_SDATI, DRV_SDATO, DRV_FAULT,
       X_STALL_PIN, Y_STALL_PIN, Z_STALL_PIN, O0_STALL_PIN, O1_STALL_PIN };
enum { X_AXIS, Y_AXIS, Z_AXIS, E_AXIS };
#define DRV8711_NUM_CHANNELS 5
#define RF_MICRO_STEPS_XY 32
#define RF_MICRO_STEPS_Z 16
#define RF_MICRO_STEPS_E 64
#define MOTOR_CURRENT_NORMAL { 10, 20, 30, 40, 50 }
#define LOW 0
#define HIGH 1
#define WRITE(pin,value) actual.push_back({0, pin, !!(value)})
#define SET_OUTPUT(pin) actual.push_back({1, pin, 0})
#define PSTR(x) x
struct HAL { static void delayMicroseconds(int us) { actual.push_back({2, us, 0}); } };
struct InterruptProtectedBlock {
    bool active = true;
    InterruptProtectedBlock() { actual.push_back({3, 0, 0}); }
    void unprotect() { assert(active); active = false; actual.push_back({4, 0, 0}); }
    ~InterruptProtectedBlock() { if (active) unprotect(); }
};
struct Printer {
    static uint8_t motorMicroStepsModeValue[5], motorCurrent[5];
    static bool debugInfo() { return false; }
};
uint8_t Printer::motorMicroStepsModeValue[5], Printer::motorCurrent[5];
struct Com { template<typename T> static void printF(const char*, T) {} template<typename T> static void printFLN(const char*, T) {} };
'''
HARNESS = r'''
#include "Drv8711.cpp"
static Trace expected;
void writePin(int p, int v) { expected.push_back({0,p,v}); }
void output(int p) { expected.push_back({1,p,0}); }
void delay(int us) { expected.push_back({2,us,0}); }
void enable(int p) { writePin(p,0); output(p); writePin(p,1); }
void enableAll() { for (int p=5;p>=1;--p) enable(p); }
void disableAll() { for (int p=5;p>=1;--p) writePin(p,0); }
void tx(unsigned word) {
    expected.push_back({3,0,0});
    for (unsigned mask=0x8000; mask; mask>>=1) {
        writePin(DRV_SDATI, !!(word&mask)); delay(1);
        writePin(DRV_SCLK,1); delay(5); writePin(DRV_SCLK,0);
    }
    expected.push_back({4,0,0});
}
void adjust(int p, int mode) {
    enable(p); tx(0x0e01 + 8*std::min(mode,8)); writePin(p,0); delay(1);
}
void configureBus() {
    writePin(DRV_SCLK,0); output(DRV_SCLK);
    writePin(DRV_SDATI,0); output(DRV_SDATI);
}
void current(int p, int level) {
    configureBus();
    if (p>=1 && p<=5) enable(p);
    tx(0x1100+level);
    if (p>=1 && p<=5) writePin(p,0);
}
void check(const char* name) {
    if (actual != expected) {
        size_t i=0;
        while (i<actual.size() && i<expected.size() && actual[i]==expected[i]) ++i;
        std::fprintf(stderr,"%s: mismatch at event %zu (actual %zu, expected %zu)\n",name,i,actual.size(),expected.size());
        std::abort();
    }
    actual.clear(); expected.clear();
}
int main() {
    unsigned mode=0;
    for (unsigned n=0;n<=65535;++n) {
        if (n>=2 && (n&(n-1))==0) ++mode;
        assert(drv8711MicroSteps_2_ModeValue(n)==mode);
    }
    for (unsigned m=0;m<256;++m)
        assert(drv8711ModeValue_2_MicroSteps(m)==(1u<<std::min(m,8u)));
    const unsigned defaults[]={32,32,16,64,64};
    for (unsigned a=0;a<256;++a)
        assert(drv8711Axis_2_InitMicrosteps(a)==(a<5?defaults[a]:32));
    for (unsigned d=0;d<256;++d) {
        const auto channel = static_cast<Channel>(d);
        setChipSelect<true>(channel); if (d>=1 && d<=5) enable(d); check("enable");
        setChipSelect<false>(channel); if (d>=1 && d<=5) writePin(d,0); check("disable");
        if (d<1 || d>5) {
            setMotorCurrent(d,127); current(d,127); check("current without chip select");
        }
    }
    setChipSelect<true, true>(Channel::E1); enableAll(); check("enable all");
    setChipSelect<false, true>(Channel::E1); disableAll(); check("disable all");
    for (unsigned w: {0u,1u,0x8000u,0xa55au,0xffffu}) { drv8711Transmit(w); tx(w); check("tx"); }
    for (int d=1;d<=5;++d) {
        for (int m=0;m<256;++m) {
            Printer::motorMicroStepsModeValue[d-1]=m;
            drv8711adjustMicroSteps(d); adjust(d,m); check("adjust microsteps");
        }
        for (int level=0;level<256;++level) {
            setMotorCurrent(d,level); current(d,level); check("current");
        }
    }
    for (int n=0;n<256;++n) {
        setExtruderCurrent(n,127); current(4+std::min(n,NUM_EXTRUDER-1),127); check("extruder clamp");
    }
    drv8711Init();
    writePin(DRV_RESET1,0); output(DRV_RESET1); configureBus();
    for (int p=DRV_SDATO;p<=O1_STALL_PIN;++p) writePin(p,1);
    writePin(DRV_RESET1,1); delay(5000); writePin(DRV_RESET1,0); delay(5000);
    const int modes[]={5,5,4,6,6};
    for (int p=1;p<=5;++p) {
        assert(Printer::motorMicroStepsModeValue[p-1]==modes[p-1]);
        adjust(p,modes[p-1]);
    }
    const unsigned words[]={0x2020,0x4504,0x583c,0x60f0,0x7000};
    for (unsigned w:words) { enableAll(); tx(w); disableAll(); if (w!=0x7000) delay(1); }
    for (int p=1;p<=5;++p) {
        assert(Printer::motorCurrent[p-1]==10*p);
        if (p<=3+NUM_EXTRUDER) current(p,10*p);
    }
    check("init including BLANK omission");
    std::printf("DRV8711 trace passed: NUM_EXTRUDER=%d\n",NUM_EXTRUDER);
}
'''

with tempfile.TemporaryDirectory() as tmp:
    work = Path(tmp)
    (work / 'Repetier.h').write_text(STUB)
    (work / 'Drv8711.cpp').write_bytes((ROOT / 'Repetier/Drv8711.cpp').read_bytes())
    (work / 'replay.cpp').write_text(HARNESS)
    for n in (1, 2):
        exe = work / 'replay'
        subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', '-O2', '-fsigned-char',
                        f'-DNUM_EXTRUDER={n}', '-I', str(ROOT / 'Repetier'), str(work / 'replay.cpp'), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)

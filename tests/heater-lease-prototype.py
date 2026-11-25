#!/usr/bin/env python3
"""Compile and exercise the task-private C++ lease prototype; no AVR timing claim."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = r"""
#include "HeaterLease.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>
struct Timer {
    uint16_t count = 0, compare[3] = {};
    bool flags[3] = {}, masks[3] = {}, pins[3] = {};
    uint16_t now() const { return count; }
    uint16_t deadline(uint8_t c) const { return compare[c]; }
    bool pending(uint8_t c) const { return flags[c]; }
    void writeDeadline(uint8_t c, uint16_t x) { compare[c] = x; }
    void clearPending(uint8_t c) { flags[c] = false; }
    void enable(uint8_t c) { masks[c] = true; }
    void disable(uint8_t c) { masks[c] = false; }
    void off(uint8_t c) { pins[c] = false; }
    void advance(uint32_t ticks) {
        for (uint8_t c=0; c<3; ++c) {
            const uint32_t distance = uint16_t(compare[c] - count);
            if (ticks > distance) flags[c] = true;
        }
        count = uint16_t(count + ticks);
    }
};
constexpr uint16_t duration = 12500, guard = 3;
template<uint8_t N> void check() {
    unsigned long cases=0;
    for (uint32_t start=0; start<65536; ++start) {
        for (uint8_t c=0; c<N; ++c) {
            for (uint32_t elapsed : {0u, 1u, 12496u, 12497u, 12500u, 12501u, 65536u, 131073u}) {
                Timer t; t.count = uint16_t(start);
                HeaterLease<Timer,N,duration,guard> lease(t);
                // Initial arm clears stale inactive flags; does not raise a pin.
                t.flags[c]=true;
                assert(lease.authorize(c,255) && !t.pins[c] && t.masks[c] && !t.flags[c]);
                assert(lease.frameDuty(c)==255);
                t.pins[c]=true;
                t.advance(elapsed);
                bool expected=elapsed<duration-guard;
                assert(lease.authorize(c,128)==expected);
                assert(lease.failed()!=expected);
                if (expected) {
                    assert(lease.frameDuty(c)==128);
                    assert(lease.authorize(c,0) && !t.pins[c] && !t.masks[c]);
                    assert(lease.frameDuty(c)==0);
                    assert(lease.authorize(c,1));
                } else {
                    assert(!t.pins[c] && !t.masks[c]);
                    assert(!lease.authorize(c,255) && lease.frameDuty(c)==0);
                }
                ++cases;
            }
        }
    }
    // Vector entry already cleared its flag.
    Timer t;
    HeaterLease<Timer,N,duration,guard> lease(t);
    for(uint8_t c=0;c<N;++c) {assert(lease.authorize(c,255));t.pins[c]=true;}
    t.advance(duration+1);
    t.flags[0]=false;
    lease.expired(0);
    assert(lease.failed());
    for(uint8_t c=0;c<N;++c) assert(!t.pins[c]&&!t.masks[c]&&!lease.authorize(c,255));
    // Expiry of one channel blocks admission of a different, still-live one.
    if(N>1) {
        Timer u;
        HeaterLease<Timer,N,duration,guard> multi(u);
        assert(multi.authorize(0,255));
        u.advance(100);
        assert(multi.authorize(1,255));
        u.pins[0]=u.pins[1]=true;
        u.advance(duration-99);
        assert(multi.frameDuty(1)==0 && multi.failed());
        assert(!u.pins[0]&&!u.pins[1]);
    }
    // Explicit inhibit is reset-only; no further positive demand accepted.
    Timer v;
    HeaterLease<Timer,N,duration,guard> inhibited(v);
    assert(!inhibited.authorize(N,255));
    assert(inhibited.authorize(0,64));
    v.pins[0]=true;
    inhibited.inhibit();
    assert(!v.pins[0]&&!inhibited.authorize(0,64));
    std::printf("channels=%u renewal cases=%lu\n", N,cases);
}
int main(){check<2>();check<3>();}
"""
with tempfile.TemporaryDirectory() as temp:
    cpp = Path(temp) / "lease.cpp"
    exe = Path(temp) / "lease"
    cpp.write_text(source)
    subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-O2",
                    "-I", str(root / "tools/task0034"), str(cpp), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)

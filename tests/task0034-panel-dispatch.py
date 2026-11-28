#!/usr/bin/env python3
"""Exercise the actual local/remote panel dispatch exclusion, no AVR hardware."""
from pathlib import Path
import subprocess
import tempfile

SRC = Path(__file__).resolve().parents[1] / "Repetier"
header = (SRC / "ui.h").read_text()
ui = (SRC / "ui.cpp").read_text()
system = (SRC / "GCodeSystem.cpp").read_text()

def section(source, first, last):
    start = source.index(first)
    return source[start:source.index(last, start + len(first))]

scope = section(header, "    class ScopedActionExclusion {", "    // Remote panel actions")
remote = section(header, "    void executeRemoteAction(int action) {", "    void slowAction();")
slow = section(ui, "void UIDisplay::slowAction() {", "} // slowAction") + "}\n"
assert "uid.executeRemoteAction(pCommand->P);" in system
assert "uid.executeAction(pCommand->P);" not in system

source = r'''
#include <cassert>
#include <cstdint>
#include <initializer_list>
using millis_t = uint32_t;
constexpr uint8_t UI_FLAG_SLOW_ACTION_RUNNING = 4;
constexpr uint8_t OTHER_TEST_FLAG = 8;
constexpr unsigned UI_KEY_BOUNCETIME = 20;
constexpr unsigned UI_KEY_FIRST_REPEAT = 200;
constexpr unsigned UI_KEY_REDUCE_REPEAT = 20;
constexpr int UI_KEY_MIN_REPEAT = 20;
constexpr unsigned UI_PAGES_DURATION = 5000;
constexpr int UI_COLS=20, MAX_COLS=28;
static bool irq = true;
struct InterruptProtectedBlock {
    bool saved = irq;
    InterruptProtectedBlock() { protect(); }
    void protect() { irq=false; }
    void unprotect() { irq=saved; }
    ~InterruptProtectedBlock() { irq=saved; }
};
namespace HAL { millis_t timeInMilliseconds() { return 100; } }
namespace Printer {
    struct { bool locked=false; bool emergencyLocked() { return locked; } } printJob;
    void allowMovement() {}
}
static int ui_menu_message;
struct UIDisplay {
    volatile uint8_t flags=0x80;
    int lastButtonAction=1001,lastAction=0,repeatDuration=0,activeAction=0,shift=0;
    millis_t lastButtonStart=0,nextRepeat=0,autoReturnTime_=0,lastSwitch=0,lastRefresh=0;
    uint8_t menuLevel=0;
    void* menu[6]{};
    bool autoReturnMessage_=false;
    unsigned calls=0, refreshes=0;
    int lastDispatched=0;
    bool expectedIrq=true;
    void exitmenu() { menuLevel=0; }
    void refreshPage() { ++refreshes; }
    void refreshEmergencyPage() { ++refreshes; }
    void slowAction();
    void executeAction(int action) {
        assert(flags & UI_FLAG_SLOW_ACTION_RUNNING);
        assert(irq==expectedIrq);
        ++calls;
        lastDispatched=action;
        assert(calls==1);
        executeRemoteAction(777);
        slowAction();
        assert(flags & UI_FLAG_SLOW_ACTION_RUNNING);
        assert(calls==1);
        flags |= OTHER_TEST_FLAG;
        assert(irq==expectedIrq);
    }
''' + scope + remote + r'''
};
''' + slow + r'''
int main() {
    for (bool enabled : {false,true}) {
        irq=enabled;
        UIDisplay remote;
        remote.expectedIrq=enabled;
        remote.executeRemoteAction(42);
        assert(remote.calls==1 && remote.lastDispatched==42);
        assert(remote.flags==(0x80|OTHER_TEST_FLAG));
        assert(remote.lastAction==0 && irq==enabled);
        UIDisplay local;
        local.expectedIrq=enabled;
        local.slowAction();
        assert(local.calls==1 && local.lastDispatched==1001);
        assert(local.flags==(0x80|OTHER_TEST_FLAG));
        assert(local.lastAction==1001 && irq==enabled);
        // A host/scan homing scope excludes both local and remote
        // dispatch and restores only the bit that it acquired.
        for (unsigned f=0; f<256; ++f) {
            UIDisplay u;
            u.flags=uint8_t(f);
            {
                UIDisplay::ScopedActionExclusion outer(u);
                assert(u.flags==(f|UI_FLAG_SLOW_ACTION_RUNNING));
                u.executeRemoteAction(42);
                u.slowAction();
                assert(u.calls==0 && irq==enabled);
                {
                    UIDisplay::ScopedActionExclusion inner(u);
                    u.flags^=OTHER_TEST_FLAG;
                }
                assert(u.flags==((f^OTHER_TEST_FLAG)|UI_FLAG_SLOW_ACTION_RUNNING));
            }
            assert(u.flags==(f^OTHER_TEST_FLAG) && irq==enabled);
        }
        UIDisplay busy;
        busy.flags|=UI_FLAG_SLOW_ACTION_RUNNING;
        busy.executeRemoteAction(42);
        assert(busy.calls==0 && busy.flags==(0x80|UI_FLAG_SLOW_ACTION_RUNNING));
        assert(irq==enabled);
        Printer::printJob.locked=true;
        busy.slowAction();
        assert(busy.refreshes==1 && busy.calls==0);
        assert(busy.flags==(0x80|UI_FLAG_SLOW_ACTION_RUNNING));
        Printer::printJob.locked=false;
    }
}
'''
with tempfile.TemporaryDirectory() as tmp:
    cpp=Path(tmp)/"test.cpp"
    exe=Path(tmp)/"test"
    cpp.write_text(source)
    for defs in ([],):
        subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror", *defs,
                        str(cpp), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
print("Task0034 local/remote panel dispatch exclusion: PASS (both IRQ entry states and terminal admission)")

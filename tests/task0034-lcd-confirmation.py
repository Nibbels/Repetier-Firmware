#!/usr/bin/env python3
"""Replay actual UI confirmation dispatcher and isolated bed commissioning.

EEPROM transactions and temperature admission are separately exercised by
eeprom-storage; this test asserts the UI call order, exact argument passing,
confirmation gates, error/recovery/thermal exits and status refresh.
"""
from pathlib import Path
import os
import re
import subprocess
from tempfile import TemporaryDirectory

fw = Path(__file__).resolve().parents[1] / "Repetier"
src = (fw / "UiSettings.cpp").read_text()
hdr = (fw / "ui.h").read_text() + "\n" + (fw / "RF.h").read_text()
actions = ("RF_RESET_ACK", "STOP_ACK", "RESTORE_DEFAULTS", "BED_CUTOFF",
           "CHOOSE_PID_ROBUST", "CHOOSE_PID_DYNAMIC")
defs = "\n".join(re.search(r"^#define UI_ACTION_" + a + r"\s+\d+", hdr, re.M)[0]
                 for a in actions)
defs += "\n" + "\n".join(re.search(r"^constexpr int " + n + r"\s*=[^;]+;", hdr, re.M)[0]
                        for n in ("heatBedScanPlaBedC", "heatBedScanPlaNozzleC"))
start = src.index("__attribute__((noinline)) void UIDisplay::finishBedCutoff()")
end = src.index("} // finishAction", start) + len("} // finishAction")
body = src[start:end]
assert body.count("EEPROM::commissionBedCutoff(") == 1

harness = r'''
#include <cassert>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <vector>
#include <iostream>
#include "defs.inc"
#define PSTR(x) x
#define UI_TEXT_RESTORE_DEFAULTS "restore defaults"
constexpr unsigned NUM_TEMPERATURE_LOOPS = NUM_EXTRUDER + 1;
enum Event { Commission, Error, Status, Refresh, Wait, Restore, Success,
    RecoveryCheck, Report, ThermalCheck, Exit, Stop, ResetText, Delay, Reset, Tune };
std::vector<Event> events;
bool commissionOK, motionOK, restoreOK, recovery, locked;
uint8_t passedCutoff;
std::string status, error;
unsigned chosenHeater;
int chosenTemp, chosenCycles, chosenMethod;
bool chosenStore;
namespace EEPROM {
bool commissionBedCutoff(uint8_t v) { events.push_back(Commission); passedCutoff=v; return commissionOK; }
void restoreEEPROMSettingsFromConfiguration() { events.push_back(Restore); }
bool lastOperationSucceeded() { events.push_back(Success); return restoreOK; }
bool recoveryPending() { events.push_back(RecoveryCheck); return recovery; }
void reportRecovery() { events.push_back(Report); }
}
namespace Commands {
enum class MotionWait { AllOutputs };
enum class PauseWait { Cancel };
bool waitForMotion(MotionWait m, PauseWait p) {
    assert(m==MotionWait::AllOutputs && p==PauseWait::Cancel);
    events.push_back(Wait); return motionOK;
}
void emergencyStop() { events.push_back(Reset); }
}
namespace Com {
void printErrorFLN(const char* s) { events.push_back(Error); error=s; }
void printFLN(const char* s) { assert(std::string(s)=="Reset via Menu"); events.push_back(ResetText); }
}
namespace HAL { void delayMilliseconds(unsigned t) { assert(t==100); events.push_back(Delay); } }
namespace Printer {
struct { bool emergencyLocked() { events.push_back(ThermalCheck); return locked; } } printJob;
void stopPrint() { events.push_back(Stop); }
}
struct Controller {
    void autotunePID(int temp, unsigned heater, int cycles, bool store, int method) {
        events.push_back(Tune); chosenTemp=temp; chosenHeater=heater;
        chosenCycles=cycles; chosenStore=store; chosenMethod=method;
    }
};
Controller controllers[NUM_TEMPERATURE_LOOPS];
Controller* tempController[NUM_TEMPERATURE_LOOPS];
struct UIDisplay {
    char confirmation_=0;
    uint8_t bedCutoffCandidate_=0, menuLevel=5, menuPos[6]={};
    void finishBedCutoff();
    void finishAction(int);
    void exitmenu(){ events.push_back(Exit); menuLevel=0; }
    void setStatusP(const char* s) { events.push_back(Status); status=s; }
    void refreshPage() { events.push_back(Refresh); }
};
#define UI_STATUS(x) setStatusP(x);
#define UI_STATUS_UPD(x) { setStatusP(x); refreshPage(); }
#include "body.inc"

void expect(std::initializer_list<Event> expected) {
    assert(events==std::vector<Event>(expected)); events.clear();
}
int main() {
    UIDisplay ui;
    unsigned n=0;
    for (unsigned i=0;i<NUM_TEMPERATURE_LOOPS;++i) tempController[i]=&controllers[i];
    for (int action : { UI_ACTION_RF_RESET_ACK, UI_ACTION_STOP_ACK,
            UI_ACTION_RESTORE_DEFAULTS, UI_ACTION_CHOOSE_PID_ROBUST, UI_ACTION_CHOOSE_PID_DYNAMIC }) {
        ui.confirmation_=0; ui.finishAction(action); expect({}); ++n;
    }
    ui.confirmation_=1;
    ui.finishAction(UI_ACTION_RF_RESET_ACK); expect({ResetText,Delay,Reset}); ++n;
    ui.finishAction(UI_ACTION_STOP_ACK); expect({Exit,Stop}); ++n;
    for (unsigned v=0;v<=255;++v) {
        for (bool ok : {false,true}) {
            commissionOK=ok; ui.bedCutoffCandidate_=v;
            ui.finishAction(UI_ACTION_BED_CUTOFF);
            if (ok) { expect({Commission,Status,Refresh}); assert(status=="Cutoff saved"); }
            else {
                expect({Commission,Error,Status,Refresh});
                assert(status=="Cutoff rejected");
                assert(error=="Bed cutoff rejected; idle cold targets required");
            }
            assert(passedCutoff==v); ++n;
        }
    }
    for (unsigned bits=0;bits<16;++bits) {
        motionOK=bits&1; restoreOK=bits&2; recovery=bits&4; locked=bits&8;
        ui.menuLevel=5; ui.finishAction(UI_ACTION_RESTORE_DEFAULTS);
        if (!motionOK) expect({Wait});
        else if (!restoreOK) expect({Wait,Restore,Success});
        else if (recovery) expect({Wait,Restore,Success,RecoveryCheck,Exit,Report});
        else if (locked) expect({Wait,Restore,Success,RecoveryCheck,ThermalCheck});
        else {
            expect({Wait,Restore,Success,RecoveryCheck,ThermalCheck,Exit,Status});
            assert(status=="restore defaults");
        }
        ++n;
    }
    for (int action : {UI_ACTION_CHOOSE_PID_ROBUST, UI_ACTION_CHOOSE_PID_DYNAMIC}) {
        for (unsigned heater=0;heater<=NUM_TEMPERATURE_LOOPS;++heater) {
            ui.menuLevel=5; ui.menuPos[3]=heater;
            ui.finishAction(action);
            if (heater==NUM_TEMPERATURE_LOOPS) expect({});
            else {
                expect({Exit,Tune});
                const bool bed=heater==NUM_EXTRUDER;
                assert(chosenHeater==heater && chosenStore);
                assert(chosenTemp==(bed?heatBedScanPlaBedC:heatBedScanPlaNozzleC));
                assert(chosenCycles==(bed?16:10));
                assert(chosenMethod==(bed?2:0)+(action==UI_ACTION_CHOOSE_PID_ROBUST?1:0));
                assert(ui.menuPos[0]==3);
            }
            ++n;
        }
    }
    std::cout << "UI confirmation/commissioning: " << n << " cases PASS\n";
}
'''
with TemporaryDirectory(prefix="rfx-ui-finish-") as tmp:
    p=Path(tmp)
    (p/"defs.inc").write_text(defs)
    (p/"body.inc").write_text(body)
    (p/"test.cpp").write_text(harness)
    for board, extruders in (("RF1000",1),("RF2000",2),("RF2000v2",2),("RF2000-X44",1)):
        exe=p/"test"
        subprocess.run([os.environ.get("CXX","c++"), "-std=c++11","-O1","-Wall","-Wextra","-Werror",
                        "-fno-pie","-no-pie","-fsanitize=address,undefined","-fno-sanitize-recover=all",
                        f"-DNUM_EXTRUDER={extruders}",str(p/"test.cpp"),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True,timeout=10,
                       env={**os.environ,"ASAN_OPTIONS":"detect_leaks=0"})
        print(board,flush=True)

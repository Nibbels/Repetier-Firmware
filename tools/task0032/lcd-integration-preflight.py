#!/usr/bin/env python3
"""Actual-source integrated LCD candidate on a disposable pinned P0 checkout.

No actual production changes in the research PR. Never flash this candidate.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

P0 = "6693513f2ecacabf47c5e08c80fc3a9bb8417eab"
HERE = Path(__file__).resolve().parent


def once(s, old, new):
    assert s.count(old)==1, (old, s.count(old))
    return s.replace(old,new)


def edit(p, fn):
    old=p.read_text()
    new=fn(old)
    assert old!=new,p
    p.write_text(new)


RESET = r'''
void UIDisplay::resetDisplayAfterClear() {
    lcdTransport_.afterClear(displayCache_,HAL::timeInMicroseconds());
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    emergencyFrame_=false;
#endif
}
'''

SERVICE = r'''
void UIDisplay::serviceDisplay() {
    if(!normalCharacters_ || !lcdTransport_.pending()) return;
    bool emergency=false;
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    emergency=Printer::printJob.emergencyLocked();
    // Until the real emergency renderer has supplied all four rows, do
    // not send even one previously pending ordinary character.
    if(emergency && !emergencyFrame_) return;
#endif
    struct Sink {
        uint32_t now() const { return HAL::timeInMicroseconds(); }
        void write(uint8_t c,bool rs) { lcdWriteByteNow(c,rs); }
    } sink;
    lcdTransport_.service(displayCache_,emergency,sink);
}
'''

LOCAL_KEY = r'''
bool UIDisplay::localKeyReady(int action) const {
    // A physical key must never confirm/edit a selection not yet visible.
    // The logical remote panel report is current immediately; the existing
    // local-confirmation exclusion and emergency stop remain independent.
    // Dedicated pause/continue and jog/offset buttons are never delayed.
    // Back is safe inside a menu, but on an info page it can change SenseOffset
    // or toggle the case light, so it too requires the visible context.
    if(Printer::blockAll || !lcdTransport_.pending()) return true;
    if(action==UI_ACTION_BACK) return menuLevel!=0;
    return action!=UI_ACTION_OK && action!=UI_ACTION_RIGHT &&
           action!=UI_ACTION_NEXT && action!=UI_ACTION_PREVIOUS;
}
'''


def source_changes(root):
    fw=root/"Repetier"
    new=fw/"LcdPendingFrame.h"
    assert not new.exists()
    new.write_bytes((HERE/"lcd-pending-frame.h").read_bytes())
    def header(s):
        s=once(s,'#include "DisplayModes.h"','#include "DisplayModes.h"\n#include "LcdPendingFrame.h"')
        s=once(s,"    void refreshPage();","    void refreshPage();\n    void serviceDisplay();")
        s=once(s,"    char displayCache_[UI_ROWS][MAX_COLS + 1];",
               """    // Logical panel snapshot; pending bits belong to the transport.
    char displayCache_[UI_ROWS][UI_COLS + 1];
    LcdPendingFrame<UI_COLS> lcdTransport_;
    void resetDisplayAfterClear();
    void renderNormalPage();
    bool localKeyReady(int action) const;
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    bool emergencyFrame_ = false;
#endif""")
        return s
    edit(fw/"ui.h",header)

    def ui(s):
        # Keep every previously audited RMW/strobe instruction intact.
        s=once(s,"""void lcdWriteByte(uint8_t c, uint8_t rs) {
    HAL::delayMicroseconds(320); // The LCD R/W line is tied low.

""","""static void lcdWriteByteNow(uint8_t c, uint8_t rs) {
""")
        s=once(s,"""} // lcdWriteByte

void UIDisplay::loadStartupCharacters()""","""} // lcdWriteByteNow

void lcdWriteByte(uint8_t c, uint8_t rs) {
    HAL::delayMicroseconds(320); // Exclusive startup/special operation.
    lcdWriteByteNow(c,rs);
} // lcdWriteByte

void UIDisplay::loadStartupCharacters()""")
        # Clear/initialization and boot glyphs retain the old synchronous
        # transport, then re-establish a known logical frame and deadline.
        s=once(s,"""    if (normal) {
        loadNormalCharacters();
    }
} // initializeLCD""","""    if (normal) {
        loadNormalCharacters();
    }
    resetDisplayAfterClear();
} // initializeLCD""")
        s=once(s,"""    loadNormalCharacters();
    if (Printer::isUIErrorMessage() || menuLevel != 0)""","""    loadNormalCharacters();
    resetDisplayAfterClear();
    if (Printer::isUIErrorMessage() || menuLevel != 0)""")
        s=once(s,"""        printRowP(3, versionString);
    }
    lastSwitch = lastRefresh""","""        printRowP(3, versionString);
        lcdTransport_.afterSynchronous(HAL::timeInMicroseconds());
    }
    lastSwitch = lastRefresh""")
        s=once(s,"// ----------- end direct LCD driver","// ----------- end direct LCD driver\n"+RESET)
        # Pure normal renderer: one row scratch, stage logical output only.
        i=s.index("void UIDisplay::refreshPage()")
        j=s.index("} // refreshPage",i)+len("} // refreshPage")
        old=s[i:j]
        k=old.index("    uint16_t r;")
        normal=old[k:]
        normal=once(normal,"""    char cache[UI_ROWS][MAX_COLS + 1];
    adjustMenuPos();

    // Copy result into cache""","""    adjustMenuPos();
    const uint8_t scroll = shift > 0 ? shift : 0;
    const auto stageCurrentRow = [this, scroll](uint8_t row) {
        const uint8_t length = strlen(printCols);
        const uint8_t offset = length > UI_COLS ? RMath::min(length - UI_COLS, scroll) : 0;
        lcdTransport_.stage(row, &printCols[offset], displayCache_);
    };

    // Render one row into the existing scratch buffer.""")
        assert normal.count("strcpy(cache[r], printCols);")==2
        normal=normal.replace("strcpy(cache[r], printCols);","stageCurrentRow(r);")
        normal=once(normal,"""    while (r < UI_ROWS)
        strcpy(cache[r++], printCols);

    for (uint8_t row = 0; row < UI_ROWS; ++row)
        strcpy(displayCache_[row], cache[row]);

    const uint8_t scroll = shift > 0 ? shift : 0;
    for (uint8_t row = 0; row < UI_ROWS; ++row) {
        const uint8_t length = strlen(cache[row]);
        const uint8_t offset = length > UI_COLS ? RMath::min(length - UI_COLS, scroll) : 0;
        printRow(row, &cache[row][offset], NULL, UI_COLS);
    }
} // refreshPage""","""    while (r < UI_ROWS)
        stageCurrentRow(r++);
} // renderNormalPage""")
        refresh=old[:k]
        refresh=once(refresh,"""        loadNormalCharacters();
    }""","""        loadNormalCharacters();
        resetDisplayAfterClear();
    }""")
        refresh=once(refresh,"""            // Existing LCD transport fills remaining columns with spaces.
            printRow(row,line,nullptr,UI_COLS);
        }
        return;""","""            lcdTransport_.stage(row,line,displayCache_);
        }
        emergencyFrame_=true;
        return;""")
        refresh+="""#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    emergencyFrame_=false;
#endif
    renderNormalPage();
} // refreshPage"""
        s=s[:i]+"void UIDisplay::renderNormalPage() {\n"+normal+"\n\n"+refresh+s[j:]
        s=once(s,"} // refreshPage\n\nvoid UIDisplay::showMessage","} // refreshPage\n"+SERVICE+"\nvoid UIDisplay::showMessage")
        s=once(s,"void UIDisplay::slowAction() {",LOCAL_KEY+"\nvoid UIDisplay::slowAction() {")
        s=once(s,"                executeAction(lastAction);\n                nextRepeat",
               "                if(localKeyReady(lastAction)) executeAction(lastAction);\n                nextRepeat")
        s=once(s,"                executeAction(lastAction);\n                repeatDuration",
               "                if(localKeyReady(lastAction)) executeAction(lastAction);\n                repeatDuration")
        return s
    edit(fw/"ui.cpp",ui)
    edit(fw/"GCodeSystem.cpp",lambda s:once(s,"const char character = line[col];",
                                            "const char character = col < UI_COLS ? line[col] : 0;"))
    # Poll after the high-priority service, never in an ISR. Include the
    # already latched emergency and the thermal pass that first latches it.
    def commands(s):
        s=once(s,"""        uid.refreshPage();
    }
}
#endif

void Commands::checkForPeriodicalActions""","""        uid.refreshPage();
    }
    uid.serviceDisplay();
}
#endif

void Commands::checkForPeriodicalActions""")
        s=once(s,"if(Printer::printJob.emergencyLocked()){uid.refreshPage();return;}",
               "if(Printer::printJob.emergencyLocked()){uid.refreshPage();uid.serviceDisplay();return;}")
        return once(s,"    handleScanWorkTasks(scanTickDue);\n} // checkForPeriodicalActions",
                    "    handleScanWorkTasks(scanTickDue);\n    uid.serviceDisplay();\n} // checkForPeriodicalActions")
    edit(fw/"Commands.cpp",commands)
    owners=[];readers=[]
    for p in sorted(fw.rglob("*")):
        if p.is_file() and p.suffix in (".h",".cpp",".ino"):
            source=p.read_text()
            if "displayCache_" in source: owners.append(p.relative_to(fw).as_posix())
            if "cachedRow(" in source: readers.append(p.relative_to(fw).as_posix())
    assert owners==["ui.cpp","ui.h"],owners
    assert readers==["GCodeSystem.cpp","ui.cpp","ui.h"],readers


def test_changes(root):
    tests=root/"tests"
    # Preserve the old GPIO test and add the raw transport path, which must
    # emit exactly the same pin/strobe sequence without a synchronous delay.
    def gpio(s):
        return once(s,"        lcdWriteByte(v,rs);checkSequence(want,1);",
                    """        lcdWriteByte(v,rs);checkSequence(want,1);
        reset(true,0);
        lcdWriteByteNow(v,rs);checkSequence(want,0);""")
    edit(tests/"lcd-gpio.py",gpio)

    # Match the exact indentation inside the with block.
    def boot_fixed(s):
        # Construct the extraction without changing preexisting fixture tests.
        s=once(s,'#include "Constants.h"','#include "Constants.h"\n#include "LcdPendingFrame.h"')
        s=once(s,"millis_t timeInMilliseconds() { return now; }",
               "millis_t timeInMilliseconds() { return now; }\nuint32_t timeInMicroseconds() { return now*1000u; }")
        s=once(s,"    char displayCache_[4][UI_COLS+1]{};",
               """    char displayCache_[4][UI_COLS+1]{};
    LcdPendingFrame<UI_COLS> lcdTransport_;
    void resetDisplayAfterClear();""")
        return once(s,'        (p / "firmware.inc").write_text(glyphs + normal + boot)',
                    '''        reset=source[source.index("void UIDisplay::resetDisplayAfterClear()"):source.index("void UIDisplay::printRow(")]
        (p / "firmware.inc").write_text(glyphs + normal + reset + boot)''')
    edit(tests/"boot-eye.py",boot_fixed)

    def workflow(s):
        s=once(s,'section(ui, "void UIDisplay::refreshPage()", "} // refreshPage") + "}\\n"',
               'section(ui, "void UIDisplay::renderNormalPage()", "void UIDisplay::showMessage(")')
        s=once(s,'#include "PreheatProfiles.h"','#include "PreheatProfiles.h"\n#include "LcdPendingFrame.h"')
        s=once(s,"millis_t timeInMilliseconds() { return 5; }",
               "millis_t timeInMilliseconds() { return 5; }\nuint32_t lcdTime=0;\nuint32_t timeInMicroseconds() { return lcdTime; }")
        s=once(s,"bool visibilityRendering=false;","bool visibilityRendering=false;\nvoid lcdWriteByteNow(uint8_t,uint8_t);")
        s=once(s,"char printCols[MAX_COLS+1]={},displayCache_[UI_ROWS][MAX_COLS+1]={};",
               """char printCols[MAX_COLS+1]={},displayCache_[UI_ROWS][UI_COLS+1]={};
    LcdPendingFrame<UI_COLS> lcdTransport_;
    unsigned lcdWrites=0;
    int lcdCol=0;
    void resetDisplayAfterClear(){lcdTransport_.afterClear(displayCache_,HAL::timeInMicroseconds());}
    void renderNormalPage();
    void serviceDisplay();
    void drainDisplay(){
        for(unsigned n=0;lcdTransport_.pending();++n){
            assert(n<8*UI_COLS+1);
            HAL::lcdTime+=324;
            const auto before=lcdWrites;
            serviceDisplay();
            assert(lcdWrites==before+1);
        }
    }""")
        s=once(s,"    std::string lcd[UI_ROWS];","    std::string lcd[UI_ROWS]={std::string(UI_COLS,' '),std::string(UI_COLS,' '),std::string(UI_COLS,' '),std::string(UI_COLS,' ')};")
        s=once(s,"""        assert(mode==0); row=-1;
        for(int i=0;i<UI_ROWS;i++) if(v==128+LCDLineOffsets[i]) row=i;
        assert(row>=0); lcd[row].clear();
    }
    void lcdPutChar(char c) { assert(row>=0 && lcd[row].size()<UI_COLS); lcd[row]+=c; }""",
               """        assert(mode==0); row=-1;
        for(int i=0;i<UI_ROWS;i++) if(v>=128+LCDLineOffsets[i] && v<128+LCDLineOffsets[i]+UI_COLS) {
            row=i; lcdCol=v-128-LCDLineOffsets[i];
        }
        assert(row>=0); ++lcdWrites;
    }
    void lcdPutChar(char c) { assert(row>=0 && lcdCol<UI_COLS); lcd[row][lcdCol++]=c; ++lcdWrites; }""")
        s=once(s,'} uid;\n#include "firmware.inc"',
               '''} uid;
void lcdWriteByteNow(uint8_t value,uint8_t rs) {
    if(rs) uid.lcdPutChar(char(value)); else uid.lcdWriteByte(value,0);
}
#include "firmware.inc"''')
        # Original observations are made after explicit transport drain,
        # not after rewriting the renderer to send synchronously in tests.
        s=once(s,"void rowIs(unsigned r, char marker, std::string text) {",
               "void rowIs(unsigned r, char marker, std::string text) {\n    uid.drainDisplay();")
        s=once(s,"    uid.menuTop[1]=0; uid.activeAction=editing; uid.refreshPage();",
               """    uid.menuTop[1]=0; uid.activeAction=editing;
    const auto before=uid.lcdWrites;
    uid.refreshPage();
    assert(uid.lcdWrites==before);
    uid.drainDisplay();""")
        # cursorAt is the other physical LCD observation in the old replay.
        s=once(s,"void cursorAt(unsigned row) {","void cursorAt(unsigned row) {\n    uid.drainDisplay();")
        return s
    edit(tests/"ui-workflows.py",workflow)

    def panel(s):
        s=once(s,'slow = section(ui, "void UIDisplay::slowAction() {", "} // slowAction") + "}\\n"',
               'slow = section(ui, "bool UIDisplay::localKeyReady(", "} // slowAction") + "}\\n"')
        s=once(s,"constexpr uint8_t UI_FLAG_SLOW_ACTION_RUNNING = 4;",
               """constexpr int UI_ACTION_OK=1001, UI_ACTION_RIGHT=1129, UI_ACTION_NEXT=1, UI_ACTION_PREVIOUS=2, UI_ACTION_BACK=1000;
constexpr uint8_t UI_FLAG_SLOW_ACTION_RUNNING = 4;""")
        s=once(s,"    void allowMovement() {}","    bool blockAll=false;\n    void allowMovement() {}")
        s=once(s,"    unsigned calls=0, refreshes=0;",
               """    unsigned calls=0, refreshes=0;
    struct { bool dirty=false; bool pending() const {return dirty;} } lcdTransport_;
    bool localKeyReady(int action) const;""")
        s=once(s,"        UIDisplay busy;",
               """        UIDisplay pending;
        pending.expectedIrq=enabled;
        pending.menuLevel=1;
        pending.lcdTransport_.dirty=true;
        pending.slowAction();
        assert(pending.calls==0 && pending.lastAction==1001 && irq==enabled);
        pending.lcdTransport_.dirty=false;
        pending.slowAction();
        assert(pending.calls==0); // stale OK consumed; release/repress required
        pending.lastButtonAction=0;
        pending.slowAction();
        pending.lastButtonAction=1001;
        pending.slowAction();
        assert(pending.calls==1 && irq==enabled);
        assert(pending.localKeyReady(1));
        pending.lcdTransport_.dirty=true;
        assert(!pending.localKeyReady(1)&&!pending.localKeyReady(2)&&!pending.localKeyReady(1129));
        assert(pending.localKeyReady(1518)&&pending.localKeyReady(514)&&pending.localKeyReady(1000));
        pending.menuLevel=0;
        assert(!pending.localKeyReady(1129)&&!pending.localKeyReady(1000)&&!pending.localKeyReady(1001));
        assert(pending.localKeyReady(1518)&&pending.localKeyReady(514));
        Printer::blockAll=true;
        assert(pending.localKeyReady(1001)); // existing reset path
        Printer::blockAll=false;
        UIDisplay busy;""")
        return s
    edit(tests/"task0034-panel-dispatch.py",panel)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source",type=Path,required=True)
    p.add_argument("--out",type=Path,required=True)
    a=p.parse_args()
    root=a.source.resolve();out=a.out.resolve()
    out.mkdir(parents=True,exist_ok=True)
    def git(*args):
        return subprocess.check_output(["git","-C",str(root),*args],text=True)
    assert git("rev-parse","HEAD").strip()==P0
    assert not git("status","--porcelain")
    source_changes(root)
    test_changes(root)
    subprocess.run(["git","-C",str(root),"add","Repetier","tests"],check=True)
    diff=git("diff","--cached","--binary")
    (out/"candidate.diff").write_text(diff)
    subprocess.run(["git","-C",str(root),"diff","--cached","--check"],check=True)
    print(json.dumps({"p0":P0,"diff_sha256":hashlib.sha256(diff.encode()).hexdigest(),
                      "files":git("diff","--cached","--name-only").splitlines()},indent=2))


if __name__=="__main__":
    main()

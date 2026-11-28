#!/usr/bin/env python3
"""Task0009: actual panel preheat rendering, dispatch and offset persistence.

Extract focused production functions/branches, not a second implementation.
Native flash pointers and an in-memory EEPROM/LCD replace physical AVR devices.
The menu tree, filters and geometry come from the firmware. Preheat fields
are rendered exactly; unrelated dynamic fields are stubbed only in the
visibility replay. Actions are recorded at the navigation boundary.
"""
import os
from pathlib import Path
import re
import subprocess
from tempfile import TemporaryDirectory


def section(source, first, last):
    start = source.index(first)
    return source[start:source.index(last, start + len(first))]


def main():
    tests = Path(__file__).resolve().parent
    fw = tests.parent / "Repetier"
    def read(name):
        return (fw / name).read_text(encoding="utf-8")
    header, menu, ui = read("ui.h"), read("uimenu.h"), read("ui.cpp")
    fmt, act, settings, epr = (read(n) for n in
        ("UiFormatting.cpp", "UiActions.cpp", "UiSettings.cpp", "EepromZoneRuntime.cpp"))
    defs = "\n".join(read(n) for n in
        ("Configuration.h", "HAL.h", "RF.h", "ui.h", "uilang.h"))
    macros = "\n".join(re.findall(
        r"^#define (?:PREHEAT_\w+|EPR_RF_PREHEAT_\w+|EPR_RF_Z_OFFSET|EPR_RF_MOD_SENSEOFFSET_DIGITS|EPR_INTEGRITY_BYTE|EEPROM_OFFSET|PERIODICAL_ACTIONS_CALL_INTERVAL|RF_MICRO_STEPS_Z|Z_OFFSET_(?:BUTTON|MENU)_STEPS|UI_ACTION_PREHEAT_\w+|UI_ACTION_RF_HEAT_BED_(?:UP|DOWN)|UI_SET_(?:MIN|MAX)_\w+_TEMP|UI_ACTION_ZOFFSET|UI_ACTION_(?:XPOSITION|YPOSITION|ZPOSITION|EPOSITION|RF_RESET_ACK|STOP_ACK|RESTORE_DEFAULTS|CHOOSE_PID_ROBUST|CHOOSE_PID_DYNAMIC)|UI_MENU_TYPE_(?:SUBMENU|MODIFICATION_MENU)|XYZ_POSITION_BUTTON_DIRECTION|UI_SET_EXTRUDER_FEEDRATE|SPEED_(?:MIN_MILLIS|MAX_MILLIS|MAGNIFICATION)|MAX_COLS|UI_TEXT_PREHEAT_\w+)\s+[^\n]+",
        defs, re.M))
    # The full menu table needs all action IDs (the original focused replay
    # extracted only the IDs it invoked). Preserve the actual masks and labels.
    macros += "\n" + "\n".join(re.findall(r"^#define UI_ACTION_\w+\s+[^\n]+", header + "\n" + read("RF.h"), re.M))
    macros += "\n#include \"DisplayModes.h\"\n#include \"uilang.h\"\n#define MAX_MENU_LEVELS 6\n"
    geometry = section(header, "#define UI_ROWS 4", "//mtype usw.")
    # Both character ROMs (only the cursor glyphs are relevant).
    rom = re.search(r"#if MOTHERBOARD == DEVICE_TYPE_RF1000[^\n]*\n((?:#define CHAR_\w+[^\n]*\n)+)", read("uilang.h"))[1] + "#else\n" + re.search(r"#else // RF2000 family LCD character ROM\n((?:#define CHAR_\w+[^\n]*\n)+)", read("uilang.h"))[1]
    # Keep the actual source condition rather than assigning UI_COLS directly.
    macros += "\n" + re.search(r"^constexpr millis_t menuAutoReturnDelayMs[^\n]+", ui, re.M)[0] + "\n" + geometry + "\n#if MOTHERBOARD == DEVICE_TYPE_RF1000\n" + rom + "#endif\n"
    macros += "\n" + section(header, "#if UI_COLS == 16", "\n#include \"uilang.h\"") + "\n"
    macros += "\n" + re.search(r"^constexpr int32_t maxManualZOffsetMicrometers[^\n]+", read("RF.h"), re.M)[0] + "\n"
    # Exactly two tune actions, shared menu mapped to the selected heater in settings.
    pid_menu = next(line for line in menu.splitlines() if line.startswith("#define UI_MENU_PID_CHOOSE "))
    assert pid_menu.count('ui_menu_pid_choose_robust') == 1
    assert pid_menu.count('ui_menu_pid_choose_dynamic') == 1
    for obsolete in ('lesserintegral', 'classicpid', 'tyreus_lyben', 'someovershoot'):
        assert obsolete not in pid_menu.lower()
    assert '(bed ? 2 : 0) + (action == UI_ACTION_CHOOSE_PID_ROBUST ? 1 : 0)' in settings
    # Check the real parent menu links and Mod Menu page/key mapping (not a key electrical test).
    for name, entries in (
        ("UI_MENU_PREHEAT", ("&ui_menu_preheat_a_bed", "&ui_menu_preheat_a_hotend",
                            "&ui_menu_preheat_b_bed", "&ui_menu_preheat_b_hotend")),
        ("UI_MENU_CONFIGURATION", ("&ui_menu_conf_preheat",)),
        ("UI_MENU_EXTRUDER", ("&ui_menu_quick_preheat_a", "&ui_menu_quick_preheat_b")),
        ("UI_MENU_MAIN", ("UI_MENU_EXTRUDER_ENTRY", "UI_MENU_CONFIGURATION_ENTRY"))):
        match = re.search(r"^#define " + name + r" \{(?:\\\n|[^\n])+", menu, re.M)[0]
        for entry in entries:
            assert entry in match.replace("\\\n", ""), (name, entry)
    assert re.search(r"UI_MENU_SUBMENU\(ui_menu_conf_preheat,\s*UI_TEXT_PREHEAT_PROFILES,\s*ui_menu_preheat\)", menu)
    assert re.search(r"UI_MENU_SUBMENU\(ui_menu_main3,\s*UI_TEXT_EXTRUDER,\s*ui_menu_extruder\)", menu)
    assert re.search(r"UI_MENU_SUBMENU\(ui_menu_main5,\s*UI_TEXT_CONFIGURATION,\s*ui_menu_configuration\)", menu)
    assert "#define UI_PAGES {&ui_page1 UI_MOD_PAGES," in menu
    assert "#define UI_MOD_PAGES , &ui_page_mod" in menu
    for key, action in (("E1", "UP"), ("E2", "DOWN")):
        assert f"UI_KEYS_BUTTON_LOW(ENABLE_KEY_{key}, UI_ACTION_RF_HEAT_BED_{action})" in ui
    assert "case ZOffset:Printer::zOffset.setManualMicrometers(int32_t(d.u32(Field::F_RF_Z_OFFSET)),Printer::stepsPerMm(Z_AXIS));" in read("EepromZoneLive.cpp")

    parser = (section(fmt, "void UIDisplay::parse(", "// Format a complete placeholder")
              + section(fmt, "void UIDisplay::renderField(", "        case 'a':"))
    parser = parser.replace("    switch (c1) {", "    (void)ram; (void)allowMessageExpansion; (void)ivalue; (void)fvalue;\n    switch (c1) {")
    # AVR stores menu pointers in an unsigned int. The non-PIE host executable
    # has static menu addresses below INT_MAX; adapt only the pointer casts.
    ok = section(ui, "void UIDisplay::pushMenu(", "} // okAction") + "}\n"
    ok = ok.replace("menu[menuLevel] = men;", "menu[menuLevel] = static_cast<UIMenu*>(men);")
    ok = ok.replace("menu[1] = (void*)&ui_menu_main;", "menu[1] = &ui_menu_main;")
    ok = ok.replace("(void*)action", "(void*)(intptr_t)action")
    generated = (
        section(fmt, "void UIDisplay::addInt(", "void UIDisplay::addLong(")
        + parser + "\n        default: assert(visibilityRendering); if (col < MAX_COLS) printCols[col++]=\'?\'; break;\n    }\n}\n"
        + section(ui, "void UIDisplay::printRow(", "void UIDisplay::ui_init_keys(")
        + section(ui, "bool UIMenuEntry::showEntry()", "} // showEntry") + "}\n"
        + ok
        + section(ui, "void UIDisplay::adjustMenuPos()", "} // adjustMenuPos") + "}\n"
        + section(ui, "void UIDisplay::navigateMenuItems(", "} // nextPreviousAction") + "}\n"
        + section(ui, "void UIDisplay::refreshPage()", "} // refreshNormalPage") + "}\n"
        + section(act, "void UIDisplay::zOffsetPlusAction()", "void UIDisplay::pressPlay()")
        + "void UIDisplay::mainSwitchCase(int action) { switch (action) {\n"
        + section(act, "    case UI_ACTION_RF_HEAT_BED_UP:", "    case UI_ACTION_RF_EXTRUDER_OUTPUT:")
        + section(act, "    case UI_ACTION_PREHEAT_A:\n", "    case UI_ACTION_COOLDOWN:")
        + "default: assert(false); } }\n"
        + section(settings, "\n#define INCREMENT_MIN_MAX(", "\n#define INCREMENT_MAX(")
        + "\n" + section(settings, "void UIDisplay::changeSwitchCase(", "// Actual setting changes")
        + "void UIDisplay::changeSetting(int action, int8_t increment) { switch(action) {\n"
        + section(settings, "    case UI_ACTION_PREHEAT_A_BED:", "    case UI_ACTION_FANSPEED:")
        + "default: assert(false); } }\n"
        + section(epr, "void EEPROM::storePreheatProfiles()", "void EEPROM::storeSenseOffsetSettings(")
        + "\n#define RF_THERMAL_EMERGENCY_EXPERIMENT 1\n"
        + section(read("PreheatProfiles.cpp"), "namespace {", "#ifndef RF_EEPROM_ZONE_EXPERIMENT")
        + "\n#undef RF_THERMAL_EMERGENCY_EXPERIMENT\n"
    )
    # Compile the complete real tree, not synthetic copies of the filtered lists.
    menu_defs = menu.replace("(unsigned int)&entries", "(uintptr_t)&entries")
    menu_defs += "\n#define QUICK_WINDOW {&ui_menu_quick_preheat_a, &ui_menu_quick_preheat_b}\nUI_MENU(quick_window, QUICK_WINDOW)\n"
    with TemporaryDirectory(prefix="rfx-ui-workflows-") as d:
        build = Path(d)
        for name, text in (("defs.inc", macros), ("firmware.inc", generated), ("menus.inc", menu_defs)):
            (build / name).write_text(text, encoding="utf-8")
        (build / "test.cpp").write_text(HARNESS, encoding="utf-8")
        # Match stock extruder counts and the X44 retrofit from the AVR build.
        for board, columns, extruders, x44 in ((1, 16, 1, False), (2, 20, 2, False),
                                              (3, 20, 2, False), (2, 20, 1, True)):
            # Only these pin feature conditions affect the menu table.
            features = ["-DCASE_LIGHT_PIN=25", "-DFET1=" + ("-1" if board == 1 else "33"),
                        "-DFET2=" + ("-1" if board == 1 else "32"),
                        "-DEXT0_EXTRUDER_COOLER_PIN=" + ("32" if x44 else "-1"),
                        "-DEXT1_EXTRUDER_COOLER_PIN=-1",
                        "-DHOTEND_COOLER_USES_PIN(pin)=(EXT0_EXTRUDER_COOLER_PIN==(pin)||(NUM_EXTRUDER>1&&EXT1_EXTRUDER_COOLER_PIN==(pin)))"]
            if board != 1:
                features.append("-DRGB_LIGHT_R_PIN=1")
            exe = build / "test"
            subprocess.run([
                os.environ.get("CXX", "c++"), "-std=c++11", "-O1", "-Wall", "-Wextra", "-Werror",
                "-fsanitize=address,undefined", "-fno-sanitize-recover=all", "-fno-pie", "-no-pie",
                "-I", str(build), "-I", str(fw), f"-DMOTHERBOARD={board}", f"-DEXPECTED_COLS={columns}",
                f"-DNUM_EXTRUDER={extruders}", *features,
                str(build / "test.cpp"), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True, timeout=10, env={**os.environ, "ASAN_OPTIONS": "detect_leaks=0"})
            print(f"UI workflow replay: model {board}, {columns} columns, X44={x44} PASS", flush=True)


HARNESS = r'''
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <set>
#include <limits>
#define DEVICE_TYPE_RF1000 1
#define DEVICE_TYPE_RF2000 2
#define DEVICE_TYPE_RF2000v2 3
using millis_t = uint32_t;
#include "defs.inc"
#include "PreheatProfiles.h"
#include "ZOffset.h"
#define PROGMEM
#define PSTR(x) x
using prog_char = char;
constexpr int Z_AXIS = 2;
constexpr float scale = 80.0f * RF_MICRO_STEPS_Z;
static_assert(UI_COLS == EXPECTED_COLS && UI_ROWS == 4, "panel geometry");
static_assert(Z_OFFSET_BUTTON_STEPS == 5, "transfer increment");
#include "EepromZonePatch.h"
constexpr unsigned offsetPos=EepromZones::address(EepromZones::Field::F_RF_Z_OFFSET);
constexpr unsigned sensePos=EepromZones::address(EepromZones::Field::F_RF_MOD_SENSEOFFSET_DIGITS);
constexpr unsigned preheatPos=EepromZones::address(EepromZones::Field::F_RF_PREHEAT_A_BED);
constexpr int bedMax=140;
struct BedFixture {int maximumTargetC()const{return bedMax;}} heatedBedController;
template<class T> T pgm_read_word(const T* p) { return *p; }
template<class T> uint8_t pgm_read_byte(const T* p) { return uint8_t(*p); }
namespace RMath { template<class A, class B> int min(A a, B b) { return std::min(int(a), int(b)); } }
namespace HAL {
std::array<uint8_t,4096> eeprom;
std::vector<unsigned> writes;
uint8_t readFlashByte(const char* p) { return uint8_t(*p); }
millis_t timeInMilliseconds() { return 5; }
void delayMilliseconds(unsigned) { assert(false && "unexpected LCD clear in normal UI replay"); }
uint8_t eprGetByte(unsigned p) { return eeprom.at(p); }
void eprSetByte(unsigned p, uint8_t v) { writes.push_back(p); eeprom.at(p) = v; }
int16_t eprGetInt16(unsigned p) { return int16_t(uint16_t(eeprom.at(p)) | uint16_t(eeprom.at(p+1))<<8); }
int32_t eprGetInt32(unsigned p) {
    uint32_t v=0; for(unsigned i=0;i<4;i++) v |= uint32_t(eeprom.at(p+i))<<(8*i); return int32_t(v);
}
void eprSetInt16(unsigned p, int16_t v) {
    writes.push_back(p); for(unsigned i=0;i<2;i++) eeprom.at(p+i)=uint16_t(v)>>(8*i);
}
void eprSetInt32(unsigned p, int32_t v) {
    writes.push_back(p); for(unsigned i=0;i<4;i++) eeprom.at(p+i)=uint32_t(v)>>(8*i);
}
}
enum { Processing };
namespace Commands { void checkForPeriodicalActions(int) {} void printCurrentPosition() {} }
template<class T, class L, class H> T constrain(T v, L low, H high) { return v < low ? low : v > high ? high : v; }
namespace Printer {
PreheatProfiles preheatProfiles;
ZOffset zOffset;
float stepsPerMm(int axis) { assert(axis == Z_AXIS); return scale; }
uint8_t activeModes=0;
uint8_t activeMenuModes() { return activeModes; }
bool uiError=false;
bool isUIErrorMessage() { return uiError; }
void setUIErrorMessage(bool v) { uiError=v; }
float menuExtrusionFactor=1.0f, lastE=0;
float flowMultiplier() { return menuExtrusionFactor; }
void queueRelativeMMCoordinates(float x,float y,float z,float e,float speed,bool wait) {
    assert(x==0 && y==0 && z==0 && speed==UI_SET_EXTRUDER_FEEDRATE && wait); lastE=e;
}
struct { unsigned calls=0; void recordActivity(millis_t t) { assert(t==5); ++calls; } } inactivity;
struct { void schedule(millis_t t) { assert(t==10005); } } idleStatus;
}
namespace Extruder {
int bed=-1, hotend=-1;
void setTemperatureForAllExtruders(int v) { hotend=v; }
void setHeatedBedTemperature(int v) { bed=v; }
}
namespace EEPROM {
void storePreheatProfiles(); void storeManualZOffset();
}
using EepromZones::Patch;
using EepromZones::Field;
using EepromZones::Preheat;
using EepromZones::blockBit;
// Stub only the durable transport; actual UI/runtime owners are extracted.
// Checked storage, metadata and power cuts are separately exercised.
static bool persist(Patch p) {
    assert(EepromZones::patchMask(&p,1));
    if(p.width==2) {
        if(HAL::eprGetInt16(p.pos)!=int16_t(p.bits))HAL::eprSetInt16(p.pos,int16_t(p.bits));
    } else {
        assert(p.width==4);
        if(HAL::eprGetInt32(p.pos)!=int32_t(p.bits))HAL::eprSetInt32(p.pos,int32_t(p.bits));
    }
    return true;
}
static bool saveLive(uint32_t mask) {
    assert(mask==EepromZones::blockBit(EepromZones::Preheat));
    const auto a=Printer::preheatProfiles.get(PreheatProfiles::Profile::A);
    const auto b=Printer::preheatProfiles.get(PreheatProfiles::Profile::B);
    return persist(Patch::word(Field::F_RF_PREHEAT_A_BED,a.bed))
        && persist(Patch::word(Field::F_RF_PREHEAT_A_HOTEND,a.hotend))
        && persist(Patch::word(Field::F_RF_PREHEAT_B_BED,b.bed))
        && persist(Patch::word(Field::F_RF_PREHEAT_B_HOTEND,b.hotend));
}
static bool reloadProfiles() {
    const bool a=Printer::preheatProfiles.readStored(PreheatProfiles::Profile::A,{HAL::eprGetInt16(preheatPos),HAL::eprGetInt16(preheatPos+2)});
    const bool b=Printer::preheatProfiles.readStored(PreheatProfiles::Profile::B,{HAL::eprGetInt16(preheatPos+4),HAL::eprGetInt16(preheatPos+6)});
    return a||b;
}
std::string status;
#define UI_STATUS_UPD(x) status=x
typedef struct { const char* text; uint8_t menuType; uintptr_t action;
    uint8_t filter,nofilter; bool showEntry() const; } const UIMenuEntry;
typedef struct { uint8_t menuType; int id,numEntries; const UIMenuEntry* const* entries; } const UIMenu;
#include "menus.inc"
const UIMenu* const ui_pages[] = UI_PAGES;
constexpr uint8_t panelPageCount=sizeof(ui_pages)/sizeof(ui_pages[0]);
bool visibilityRendering=false;
const uint8_t LCDLineOffsets[] = UI_LINE_OFFSETS;
struct UIDisplay {
    uint8_t col=0,menuLevel=0,menuPos[6]={},menuTop[6]={};
    int shift=0,activeAction=0;
    millis_t lastSwitch=0,lastNextPrev=0,autoReturnTime_=0;
    float lastNextAccumul=0;
    char confirmation_=0;
    const UIMenu* menu[6]={};
    char printCols[MAX_COLS+1]={},displayCache_[UI_ROWS][MAX_COLS+1]={};
    std::string lcd[UI_ROWS];
    int row=-1, jog=0, jogCalls=0;
    void lcdWriteByte(int v,int mode) {
        assert(mode==0); row=-1;
        for(int i=0;i<UI_ROWS;i++) if(v==128+LCDLineOffsets[i]) row=i;
        assert(row>=0); lcd[row].clear();
    }
    void lcdPutChar(char c) { assert(row>=0 && lcd[row].size()<UI_COLS); lcd[row]+=c; }
    void moveZAction(int direction) { jog=direction; ++jogCalls; }
    void moveXAction(int direction) { jog=direction; ++jogCalls; }
    void moveYAction(int direction) { jog=direction; ++jogCalls; }
    void parse(char*,bool,bool allowMessageExpansion=true);
    void renderField(char,char,bool,bool,int&,float&);
    void navigateMenuItems(UIMenuEntry**,uint8_t,int8_t);
    void nextPreviousAction(int8_t);
    void changeSetting(int,int8_t);
    void addInt(int16_t,uint8_t,char=' ');
    void printRow(uint8_t,char*,char*,uint8_t);
    void adjustMenuPos();
    int executed=0, finished=0;
    void executeAction(int action) { assert(false && "direct menu must not enter key dispatch"); (void)action; }
    void executeMenuCommand(int action) { executed=action; }
    void finishAction(int action) { finished=action; }
    void pushMenu(void*,bool);
    void menuEsc(); void okAction();
    bool normalCharacters_=true;
    void loadNormalCharacters() { normalCharacters_=true; }
    static constexpr int LCD_CLEAR = 1;
    void lcdCommand(int) { assert(false && "unexpected boot clear during normal UI replay"); }
    void refreshPage();
    void refreshNormalPage();
    void zOffsetPlusAction(); void zOffsetMinusAction();
    void mainSwitchCase(int); void changeSwitchCase(int,int8_t);
} uid;
#include "firmware.inc"
void unchangedExcept(const std::array<uint8_t,4096>& before,unsigned start,unsigned end) {
    for(unsigned i=0;i<before.size();i++)
        if(i<start || i>=end) assert(before[i]==HAL::eeprom[i]);
}
void rowIs(unsigned r, char marker, std::string text) {
    std::string expected=marker+text;
    assert(expected.size()<=UI_COLS);
    expected.resize(UI_COLS,' ');
    assert(uid.lcd[r]==expected);
    assert(std::string(uid.displayCache_[r])==expected);
}
void show(UIMenu* menu, int selected, int editing=0) {
    uid.menuLevel=1; uid.menu[1]=menu; uid.menuPos[1]=selected;
    uid.menuTop[1]=0; uid.activeAction=editing; uid.refreshPage();
}
void preheat() {
    using P=PreheatProfiles::Profile;
    HAL::eeprom.fill(0xff);
    HAL::eprSetInt16(sensePos,700); HAL::eprSetInt32(offsetPos,0);
    const auto before=HAL::eeprom;
    Printer::preheatProfiles.restoreDefaults();
    EEPROM::storePreheatProfiles();
    unchangedExcept(before,preheatPos,preheatPos+8);
    assert(HAL::eprGetInt16(preheatPos)==60 && HAL::eprGetInt16(preheatPos+2)==180);
    assert(HAL::eprGetInt16(preheatPos+4)==100 && HAL::eprGetInt16(preheatPos+6)==210);
    const unsigned addresses[]={preheatPos,preheatPos+2,preheatPos+4,preheatPos+6};
    const int changed[]={65,195,115,230};
    const int increments[]={5,15,15,20};
    for(int i=0;i<4;i++) {
        auto old=HAL::eeprom;
        auto entry=ui_menu_preheat.entries[i];
        assert(entry->menuType==4 && entry->showEntry());
        HAL::writes.clear();
        uid.changeSwitchCase(entry->action,increments[i]);
        assert(HAL::eprGetInt16(addresses[i])==changed[i]);
        unchangedExcept(old,addresses[i],addresses[i]+2);
        assert(HAL::writes.size()==1 && HAL::writes.back()==addresses[i]);
    }
    Printer::preheatProfiles.restoreDefaults();
    HAL::writes.clear(); assert(!reloadProfiles() && HAL::writes.empty());
    show(&ui_menu_preheat,0);
    rowIs(0,char(CHAR_SELECTOR),"A Bed 65 C");
    rowIs(1,' ',"A Ext 195 C"); rowIs(2,' ',"B Bed 115 C"); rowIs(3,' ',"B Ext 230 C");
    show(&ui_menu_preheat,3,ui_menu_preheat_b_hotend.action);
    rowIs(0,' ',"A Bed 65 C"); rowIs(3,CHAR_SELECTED,"B Ext 230 C");
    show(&quick_window,0);
    rowIs(0,char(CHAR_SELECTOR),UI_COLS==16?"A B65 E195":"A Bed 65 Ext 195");
    rowIs(1,' ',UI_COLS==16?"B B115 E230":"B Bed 115 Ext 230");
    rowIs(2,' ',""); rowIs(3,' ',"");
    HAL::writes.clear();
    uid.mainSwitchCase(ui_menu_quick_preheat_a.action);
    assert(Extruder::bed==65 && Extruder::hotend==195 && status=="Preheat A");
    uid.mainSwitchCase(ui_menu_quick_preheat_b.action);
    assert(Extruder::bed==115 && Extruder::hotend==230 && status=="Preheat B" && HAL::writes.empty());
    // Upper bound and zero transitions for each actual edit entry.
    for(int i=0;i<4;i++) {
        int action=ui_menu_preheat.entries[i]->action;
        bool bed=i%2==0;
        int max=bed?bedMax:UI_SET_MAX_EXTRUDER_TEMP;
        int min=bed?UI_SET_MIN_HEATED_BED_TEMP:UI_SET_MIN_EXTRUDER_TEMP;
        uid.changeSwitchCase(action,127); assert(HAL::eprGetInt16(addresses[i])==max);
        HAL::writes.clear(); uid.changeSwitchCase(action,1); uid.changeSwitchCase(action,0); assert(HAL::writes.empty());
        uid.changeSwitchCase(action,-127); uid.changeSwitchCase(action,-127);
        assert(HAL::eprGetInt16(addresses[i])==0);
        HAL::writes.clear(); uid.changeSwitchCase(action,-1); assert(HAL::writes.empty());
        uid.changeSwitchCase(action,1); assert(HAL::eprGetInt16(addresses[i])==min);
        uid.changeSwitchCase(action,127);
        assert(HAL::eprGetInt16(addresses[i])==std::min(max,min+127));
        uid.changeSwitchCase(action,127); assert(HAL::eprGetInt16(addresses[i])==max);
    }
    show(&ui_menu_preheat,2); rowIs(0,' ',"A Bed 140 C"); rowIs(1,' ',"A Ext 270 C");
    rowIs(2,char(CHAR_SELECTOR),"B Bed 140 C"); rowIs(3,' ',"B Ext 270 C");
    show(&quick_window,1); rowIs(0,' ',UI_COLS==16?"A B140 E270":"A Bed 140 Ext 270");
    rowIs(1,char(CHAR_SELECTOR),UI_COLS==16?"B B140 E270":"B Bed 140 Ext 270");
    // All four values survived the panel changes independently, including boundaries.
    assert(Printer::preheatProfiles.get(P::A).hotend==270 && Printer::preheatProfiles.get(P::B).bed==140);
}
void offset() {
    auto& z=Printer::zOffset;
    auto set=[&](int32_t manual,uint8_t sense) {
        z.setManualMicrometers(manual,scale); z.clearSenseOffset(scale);
        z.applySenseIncrement(z.prepareSenseIncrement(sense,1000,scale));
        EEPROM::storeManualZOffset(); HAL::writes.clear();
    };
    auto press=[&](int action) { uid.mainSwitchCase(action); };
    uid.menuLevel=0; uid.menuPos[0]=1;
    set(0,200);
    const auto before=HAL::eeprom; const auto total=z.effectiveSteps();
    for(int i=1;i<=40;i++) {
        HAL::writes.clear(); press(UI_ACTION_RF_HEAT_BED_DOWN);
        assert(z.manualMicrometers()==5*i && z.senseMicrometers()==205-5*i);
        assert(z.effectiveSteps()==int32_t(205*scale/1000.0f));
        assert(HAL::eprGetInt32(offsetPos)==5*i && (HAL::writes==std::vector<unsigned>{offsetPos}));
        unchangedExcept(before,offsetPos,offsetPos+4);
        HAL::writes.clear(); press(UI_ACTION_RF_HEAT_BED_UP);
        assert(z.manualMicrometers()==5*i && z.senseMicrometers()==200-5*i);
        assert(z.effectiveSteps()==total && HAL::writes.empty());
    }
    assert(uid.jogCalls==0);
    // Simulated reboot/reload: only the manual part is durable.
    set(25,100); press(UI_ACTION_RF_HEAT_BED_DOWN); press(UI_ACTION_RF_HEAT_BED_UP);
    assert(z.manualMicrometers()==30 && z.senseMicrometers()==95);
    z.clearSenseOffset(scale); z.setManualMicrometers(HAL::eprGetInt32(offsetPos),scale);
    assert(z.manualMicrometers()==30 && z.senseMicrometers()==0);
    // Insufficient dynamic offset: the remaining portion of UP reduces manual.
    set(20,2); auto initial=z.effectiveSteps();
    press(UI_ACTION_RF_HEAT_BED_DOWN); HAL::writes.clear(); press(UI_ACTION_RF_HEAT_BED_UP);
    assert(z.manualMicrometers()==22 && z.senseMicrometers()==0 && z.effectiveSteps()==initial);
    assert(HAL::eprGetInt32(offsetPos)==22 && (HAL::writes==std::vector<unsigned>{offsetPos}));
    // Zero snapping and upper clamp are intentionally not an exact transfer.
    set(-2,20); press(UI_ACTION_RF_HEAT_BED_DOWN); assert(z.manualMicrometers()==0);
    press(UI_ACTION_RF_HEAT_BED_UP); assert(z.manualMicrometers()==0 && z.senseMicrometers()==15);
    set(maxManualZOffsetMicrometers,20); press(UI_ACTION_RF_HEAT_BED_DOWN);
    assert(HAL::writes.empty()); press(UI_ACTION_RF_HEAT_BED_UP);
    assert(z.manualMicrometers()==maxManualZOffsetMicrometers && z.senseMicrometers()==15);
    // Same physical actions elsewhere must jog, never edit EEPROM.
    set(50,25); const auto saved=HAL::eeprom;
    for(int page : {0,2}) {
        uid.menuLevel=0; uid.menuPos[0]=page;
        press(UI_ACTION_RF_HEAT_BED_DOWN); assert(uid.jog==1);
        press(UI_ACTION_RF_HEAT_BED_UP); assert(uid.jog==-1);
    }
    uid.menuLevel=1; uid.menuPos[0]=1;
    press(UI_ACTION_RF_HEAT_BED_DOWN); assert(uid.jog==1);
    press(UI_ACTION_RF_HEAT_BED_UP); assert(uid.jog==-1);
    assert(uid.jogCalls==6 && HAL::writes.empty() && HAL::eeprom==saved);
    assert(z.manualMicrometers()==50 && z.senseMicrometers()==25);
    assert(HAL::eprGetInt16(sensePos)==700);
}
void navigation() {
    // Real navigation path: wrap, hidden-entry skip, edit routing, page wrap.
    const UIMenuEntry a={"A",4,UI_ACTION_PREHEAT_A_BED,0,0};
    const UIMenuEntry hidden={"hidden",4,UI_ACTION_PREHEAT_B_BED,1,0};
    const UIMenuEntry c={"C",4,UI_ACTION_PREHEAT_A_HOTEND,0,0};
    const UIMenuEntry* const entries[]={&a,&hidden,&c};
    const UIMenu browse={UI_MENU_TYPE_SUBMENU,0,3,entries};
    uid.menuLevel=1; uid.menu[1]=&browse; uid.menuPos[1]=0; uid.menuTop[1]=0; uid.activeAction=0;
    uid.nextPreviousAction(1); assert(uid.menuPos[1]==2 && uid.shift==-2);
    uid.nextPreviousAction(1); assert(uid.menuPos[1]==0);
    uid.nextPreviousAction(-1); assert(uid.menuPos[1]==2);
    uid.nextPreviousAction(-1); assert(uid.menuPos[1]==0);
    Printer::activeModes=1; uid.nextPreviousAction(1); assert(uid.menuPos[1]==1);
    Printer::activeModes=0; uid.menuPos[1]=0;
    uid.activeAction=UI_ACTION_PREHEAT_A_BED;
    const int old=HAL::eprGetInt16(preheatPos);
    uid.nextPreviousAction(1); assert(uid.menuPos[1]==0 && HAL::eprGetInt16(preheatPos)==old-1);
    assert(uid.autoReturnTime_==5+menuAutoReturnDelayMs);
    Printer::uiError=true; uid.nextPreviousAction(1);
    assert(!Printer::uiError && HAL::eprGetInt16(preheatPos)==old-1);
    uid.activeAction=0; uid.menuLevel=0; uid.menuPos[0]=0;
    uid.nextPreviousAction(-1); assert(uid.menuPos[0]==panelPageCount-1);
    uid.nextPreviousAction(1); assert(uid.menuPos[0]==0);
    HAL::writes.clear(); uid.changeSwitchCase(UI_ACTION_STOP_ACK,1); assert(uid.confirmation_==1);
    uid.changeSwitchCase(UI_ACTION_STOP_ACK,-1); assert(uid.confirmation_==0);
    uid.changeSwitchCase(UI_ACTION_XPOSITION,1); assert(uid.jog==XYZ_POSITION_BUTTON_DIRECTION);
    uid.changeSwitchCase(UI_ACTION_YPOSITION,-1); assert(uid.jog==-XYZ_POSITION_BUTTON_DIRECTION);
    uid.changeSwitchCase(UI_ACTION_ZPOSITION,2); assert(uid.jog==2*XYZ_POSITION_BUTTON_DIRECTION);
    uid.changeSwitchCase(UI_ACTION_EPOSITION,-2); assert(Printer::lastE==-2);
    assert(HAL::writes.empty());
}
// Enumerate the structural tree and every combination of the four actual
// mode bits. An ancestor's current filter is not a lifetime guarantee.
void auditMenus(UIMenu* menu, std::set<UIMenu*>& seen) {
    if (!seen.insert(menu).second) return;
    if (menu->menuType != UI_MENU_TYPE_SUBMENU) return;
    assert(menu->numEntries>0 && menu->numEntries<=255);
    bool unconditional=false;
    for(int i=0;i<menu->numEntries;++i) {
        auto e=menu->entries[i];
        unconditional |= !e->filter && !e->nofilter;
        if(e->menuType==2) {
            assert(e->action <= uintptr_t(std::numeric_limits<int>::max()));
            auditMenus(reinterpret_cast<UIMenu*>(e->action),seen);
        }
    }
    const bool filament = menu==&ui_menu_extruder_mount || menu==&ui_menu_extruder_unmount;
    assert(unconditional != filament);
    for(unsigned bits=0;bits<16;++bits) {
        Printer::activeModes=(bits&1?MENU_MODE_PRINTING:0)|(bits&2?MENU_MODE_PAUSED:0)|
            (bits&4?MENU_MODE_FAN_RUNNING:0)|(bits&8?MENU_MODE_FAN_MODE_PDM:0);
        unsigned visible=0;
        for(int i=0;i<menu->numEntries;++i) visible+=menu->entries[i]->showEntry();
        const bool empty=filament && (bits&3);
        assert((visible==0)==empty);
    }
}
int indexOf(UIMenu* menu,UIMenuEntry* entry) {
    for(int i=0;i<menu->numEntries;++i) if(menu->entries[i]==entry) return i;
    assert(false); return -1;
}
void cursorAt(unsigned row) {
    assert(row<UI_ROWS);
    for(unsigned i=0;i<UI_ROWS;++i) {
        assert(uid.lcd[i].size()==UI_COLS);
        assert((uid.lcd[i][0]==char(CHAR_SELECTOR))==(i==row));
    }
}
void visibility() {
    std::set<UIMenu*> seen;
    auditMenus(&ui_menu_main,seen);
    assert(seen.count(&ui_menu_extruder_mount) && seen.count(&ui_menu_extruder_unmount));
    visibilityRendering=true;
    Printer::activeModes=0;
    uid.executed=uid.finished=0;
    // Main: Position disappears before OK, which must not enter Quick Settings.
    show(&ui_menu_main,1);
    cursorAt(1);
    Printer::activeModes=MENU_MODE_PRINTING;
    uid.okAction();
    assert(uid.menuLevel==1 && uid.menuPos[1]==0 && uid.menuTop[1]==0);
    assert(!uid.executed && !uid.finished);
    uid.refreshPage(); cursorAt(0);
    uid.okAction();
    assert(uid.menuLevel==2 && uid.menu[2]==&ui_menu_quick);
    // Main: Pause Position disappears on resume/idle.
    Printer::activeModes=MENU_MODE_PRINTING|MENU_MODE_PAUSED;
    show(&ui_menu_main,2); cursorAt(1);
    Printer::activeModes=0;
    uid.refreshPage(); assert(uid.menuPos[1]==1 && uid.menuTop[1]==0); cursorAt(1);
    uid.okAction(); assert(uid.menu[2]==&ui_menu_positions);
    // Entire real filament submenu disappears. Exercise refresh, OK and both
    // navigation directions before refresh, for load/unload and each busy bit.
    for(auto parentEntry : {&ui_submenu_extruder_mount,&ui_submenu_extruder_unmount}) {
        auto child=reinterpret_cast<UIMenu*>(parentEntry->action);
        for(uint8_t mode : {MENU_MODE_PRINTING,MENU_MODE_PAUSED,MENU_MODE_PRINTING|MENU_MODE_PAUSED}) {
            for(int event=0;event<4;++event) {
                Printer::activeModes=0; uid.executed=uid.finished=0;
                const int parent=indexOf(&ui_menu_extruder,parentEntry);
                show(&ui_menu_extruder,parent);
                assert(uid.menuTop[1]==parent-3); cursorAt(3);
                uid.okAction();
                assert(uid.menuLevel==2 && uid.menu[2]==child);
                uid.nextPreviousAction(1); uid.refreshPage(); cursorAt(1);
                assert(uid.menuPos[2]==1 && uid.menuTop[2]==0);
                Printer::activeModes=mode;
                if(event==0) uid.refreshPage();
                if(event==1) uid.okAction();
                if(event==2) uid.nextPreviousAction(1);
                if(event==3) uid.nextPreviousAction(-1);
                assert(uid.menuLevel==1 && uid.menu[1]==&ui_menu_extruder);
                assert(uid.menuPos[1]==indexOf(&ui_menu_extruder,&ui_menu_quick_cooldown));
                assert(uid.menuTop[1]==parent-3 && uid.menuPos[2]==0 && uid.menuTop[2]==0);
                assert(!uid.executed && !uid.finished && !uid.activeAction);
                uid.refreshPage(); cursorAt(uid.menuPos[1]-uid.menuTop[1]);
                uid.okAction(); assert(uid.executed==UI_ACTION_COOLDOWN);
            }
        }
    }
    // Filtered edit: PWM frequency is hidden when PDM is selected.
    for(int event=0;event<3;++event) {
        Printer::activeModes=0; uid.executed=uid.finished=0;
        show(&ui_menu_settings_fan,1);
        uid.okAction(); assert(uid.activeAction==UI_ACTION_FAN_HZ);
        Printer::activeModes=MENU_MODE_FAN_MODE_PDM;
        if(event==0) uid.refreshPage();
        if(event==1) uid.okAction();
        if(event==2) uid.nextPreviousAction(1);
        assert(!uid.activeAction && !uid.executed && !uid.finished);
        assert(uid.menuPos[1]==0 && uid.menuTop[1]==0);
        uid.refreshPage(); cursorAt(0);
        uid.okAction(); assert(uid.executed==UI_ACTION_FAN_MODE);
    }
    Printer::activeModes=0; visibilityRendering=false;
}
int main() { preheat(); offset(); navigation(); visibility(); }
'''


if __name__ == "__main__":
    main()

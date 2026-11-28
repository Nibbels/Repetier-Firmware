#include "preheat-settings.inc"
#include "PreheatProfiles.h"
#include <cassert>
#include <cstring>
#include <string>

#include "EepromZoneLayout.h"
using namespace EepromZones;
namespace HAL { uint32_t timeInMilliseconds() { return 5; } }

namespace Printer {
PreheatProfiles preheatProfiles;
struct {
    void schedule(uint32_t delay) { assert(delay == 10005); }
} idleStatus;
}

struct EEPROM { static void storePreheatProfiles(); };
using Profile=PreheatProfiles::Profile;
static PreheatProfiles::Temperatures saved[2];
static unsigned saves=0;
static bool saveLive(uint32_t mask) {
    assert(mask==blockBit(Preheat));
    saved[0]=Printer::preheatProfiles.get(Profile::A);
    saved[1]=Printer::preheatProfiles.get(Profile::B);
    ++saves;return true;
}
struct BedFixture {int maximum=110;int maximumTargetC()const{return maximum;}} heatedBedController;
#include "preheat-firmware.inc"
// The actual LiveSettings codec and durable checked session are exercised in
// eeprom-storage.py. This fixture simulates loading a committed preference.
static bool loadProfiles() {
    const bool a=Printer::preheatProfiles.readStored(Profile::A,saved[0]);
    const bool b=Printer::preheatProfiles.readStored(Profile::B,saved[1]);
    return a||b;
}

namespace Extruder {
int bed = 0, hotend = 0;
void setTemperatureForAllExtruders(int value) { hotend = value; }
void setHeatedBedTemperature(int value) { bed = value; }
}

std::string status, label;
#define UI_STATUS_UPD(text) status = text
void addInt(int value, uint8_t) { label += std::to_string(value); }

#include "preheat-panel.inc"

std::string render(const char* text) {
    label.clear();
    while (*text) {
        if (*text == '%') {
            assert(text[1] == 'V' && text[2]);
            renderField(text[2]);
            text += 3;
        } else {
            label += *text++;
        }
    }
    assert(label.size() + 1 <= UI_COLS); // The menu reserves one column for its cursor.
    return label;
}

int main() {
    Printer::preheatProfiles.restoreDefaults();
    EEPROM::storePreheatProfiles();
    assert(!loadProfiles());
    saves=0;
    activate(UI_ACTION_PREHEAT_A);
    assert(Extruder::bed==60 && Extruder::hotend==180 && status=="Preheat A");
    edit(UI_ACTION_PREHEAT_A_BED,-5);
    edit(UI_ACTION_PREHEAT_A_HOTEND,20);
    edit(UI_ACTION_PREHEAT_B_BED,-10);
    edit(UI_ACTION_PREHEAT_B_HOTEND,25);
    assert(saves==4);
    Printer::preheatProfiles.restoreDefaults();
    assert(!loadProfiles());
    activate(UI_ACTION_PREHEAT_B);
    assert(Extruder::bed==90 && Extruder::hotend==235 && status=="Preheat B");
    activate(UI_ACTION_PREHEAT_A);
    assert(Extruder::bed==55 && Extruder::hotend==200);
    assert(render(ui_menu_quick_preheat_a)==(UI_COLS==16?"A B55 E200":"A Bed 55 Ext 200"));
    assert(render(ui_menu_quick_preheat_b)==(UI_COLS==16?"B B90 E235":"B Bed 90 Ext 235"));
    saved[1].hotend=300;
    assert(loadProfiles());
    assert(Printer::preheatProfiles.get(Profile::B).hotend==210);
    assert(Printer::preheatProfiles.get(Profile::B).bed==90);
    edit(UI_ACTION_PREHEAT_A_BED,-10);assert(saved[0].bed==0);
    edit(UI_ACTION_PREHEAT_A_BED,1);assert(saved[0].bed==UI_SET_MIN_HEATED_BED_TEMP);
    edit(UI_ACTION_PREHEAT_A_HOTEND,127);assert(saved[0].hotend==UI_SET_MAX_EXTRUDER_TEMP);
    saved[1].bed=150;assert(loadProfiles());
    assert(Printer::preheatProfiles.get(Profile::B).bed==100);
    edit(UI_ACTION_PREHEAT_B_BED,100);assert(saved[1].bed==110);
    heatedBedController.maximum=45;
    assert(!loadProfiles());
    assert(saved[0].bed==UI_SET_MIN_HEATED_BED_TEMP && saved[1].bed==110);
    const auto writes=saves;
    edit(UI_ACTION_PREHEAT_A_BED,1);
    assert(saved[0].bed==UI_SET_MIN_HEATED_BED_TEMP && saves==writes);
    heatedBedController.maximum=0;
    assert(!loadProfiles() && saved[1].bed==110);
}

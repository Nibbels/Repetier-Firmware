#include "preheat-settings.inc"
#include "PreheatProfiles.h"
#include <cassert>
#include <cstring>
#include <string>

namespace HAL {
uint8_t eeprom[2048];
uint32_t timeInMilliseconds() { return 5; }
int16_t eprGetInt16(unsigned pos) {
    int16_t value;
    std::memcpy(&value, eeprom + pos, sizeof(value));
    return value;
}
int32_t eprGetInt32(unsigned pos) {
    int32_t value;
    std::memcpy(&value, eeprom + pos, sizeof(value));
    return value;
}
void eprSetInt16(unsigned pos, int16_t value) { std::memcpy(eeprom + pos, &value, sizeof(value)); }
void eprSetInt32(unsigned pos, int32_t value) { std::memcpy(eeprom + pos, &value, sizeof(value)); }
}

namespace Printer {
PreheatProfiles preheatProfiles;
struct {
    void schedule(uint32_t delay) { assert(delay == 10005); }
} idleStatus;
}

namespace EEPROM {
bool readPreheatProfiles();
void writePreheatProfiles();
void storePreheatProfiles();
unsigned checksumWrites = 0;
void updateChecksum() { ++checksumWrites; }
}

#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
struct BedFixture {int maximum=110;int maximumTargetC()const{return maximum;}} heatedBedController;
#endif
#include "preheat-firmware.inc"

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
    std::memset(HAL::eeprom, 0xff, sizeof(HAL::eeprom));
    HAL::eeprom[1959] = 127;
    assert(EEPROM::readPreheatProfiles());
    assert(!EEPROM::readPreheatProfiles());
    assert(HAL::eeprom[1959] == 127);
    activate(UI_ACTION_PREHEAT_A);
    assert(Extruder::bed == 60 && Extruder::hotend == 180 && status == "Preheat A");

    edit(UI_ACTION_PREHEAT_A_BED, -5);
    edit(UI_ACTION_PREHEAT_A_HOTEND, 20);
    edit(UI_ACTION_PREHEAT_B_BED, -10);
    edit(UI_ACTION_PREHEAT_B_HOTEND, 25);
    assert(EEPROM::checksumWrites == 4);
    Printer::preheatProfiles.restoreDefaults();
    assert(!EEPROM::readPreheatProfiles());
    activate(UI_ACTION_PREHEAT_B);
    assert(Extruder::bed == 90 && Extruder::hotend == 235 && status == "Preheat B");
    activate(UI_ACTION_PREHEAT_A);
    assert(Extruder::bed == 55 && Extruder::hotend == 200);
    assert(render(ui_menu_quick_preheat_a) == (UI_COLS == 16 ? "A B55 E200" : "A Bed 55 Ext 200"));
    assert(render(ui_menu_quick_preheat_b) == (UI_COLS == 16 ? "B B90 E235" : "B Bed 90 Ext 235"));

    HAL::eprSetInt16(EPR_RF_PREHEAT_B_HOTEND, 300);
    assert(EEPROM::readPreheatProfiles());
    assert(HAL::eprGetInt16(EPR_RF_PREHEAT_B_HOTEND) == 210);
    assert(HAL::eprGetInt16(EPR_RF_PREHEAT_B_BED) == 90);
    edit(UI_ACTION_PREHEAT_A_BED, -10);
    assert(HAL::eprGetInt16(EPR_RF_PREHEAT_A_BED) == 0);
    edit(UI_ACTION_PREHEAT_A_BED, 1);
    assert(HAL::eprGetInt16(EPR_RF_PREHEAT_A_BED) == UI_SET_MIN_HEATED_BED_TEMP);
    edit(UI_ACTION_PREHEAT_A_HOTEND, 127);
    assert(HAL::eprGetInt16(EPR_RF_PREHEAT_A_HOTEND) == UI_SET_MAX_EXTRUDER_TEMP);
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    HAL::eprSetInt16(EPR_RF_PREHEAT_B_BED,150);
    assert(EEPROM::readPreheatProfiles());
    assert(HAL::eprGetInt16(EPR_RF_PREHEAT_B_BED)==100);
    edit(UI_ACTION_PREHEAT_B_BED,100);
    assert(HAL::eprGetInt16(EPR_RF_PREHEAT_B_BED)==110);
    heatedBedController.maximum=45;
    assert(EEPROM::readPreheatProfiles());
    assert(!HAL::eprGetInt16(EPR_RF_PREHEAT_A_BED)&&!HAL::eprGetInt16(EPR_RF_PREHEAT_B_BED));
    edit(UI_ACTION_PREHEAT_A_BED,1);
    assert(!HAL::eprGetInt16(EPR_RF_PREHEAT_A_BED));
#endif
}

#include "Printer.h"
#include "PreheatProfiles.h"
#include "Eeprom.h"

namespace {
constexpr int32_t preheatProfileTag = 0x50524831L;

int16_t bedProfileMaximum() {
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    const int16_t cap=heatedBedController.maximumTargetC();
    return cap<UI_SET_MAX_HEATED_BED_TEMP?cap:UI_SET_MAX_HEATED_BED_TEMP;
#else
    return UI_SET_MAX_HEATED_BED_TEMP;
#endif
}
bool validTemperature(int16_t temperature, int16_t minimum, int16_t maximum) {
    return temperature == 0 || (temperature >= minimum && temperature <= maximum);
}
}

bool PreheatProfiles::readStored(Profile profile, Temperatures temperatures) {
    const auto factory = defaults(profile);
    auto& target = profiles_[static_cast<uint8_t>(profile)];
    const int16_t bedMax=bedProfileMaximum();
    const int16_t safeDefault=validTemperature(factory.bed,UI_SET_MIN_HEATED_BED_TEMP,bedMax)?factory.bed:0;
    target.bed = validTemperature(temperatures.bed, UI_SET_MIN_HEATED_BED_TEMP, bedMax)
        ? temperatures.bed : safeDefault;
    target.hotend = validTemperature(temperatures.hotend, UI_SET_MIN_EXTRUDER_TEMP, UI_SET_MAX_EXTRUDER_TEMP)
        ? temperatures.hotend : factory.hotend;
    return target.bed != temperatures.bed || target.hotend != temperatures.hotend;
}

bool PreheatProfiles::adjust(Profile profile, Heater heater, int8_t increment) {
    auto& settings = profiles_[static_cast<uint8_t>(profile)];
    const bool bed = heater == Heater::Bed;
    int16_t& current = bed ? settings.bed : settings.hotend;
    const int16_t minimum = bed ? UI_SET_MIN_HEATED_BED_TEMP : UI_SET_MIN_EXTRUDER_TEMP;
    const int16_t maximum = bed ? bedProfileMaximum() : UI_SET_MAX_EXTRUDER_TEMP;
    if(maximum<minimum) {
        const bool changed=current!=0;
        current=0;
        return changed;
    }
    if (!increment)
        return false;

    int16_t next = current + increment;
    if (next < minimum)
        next = increment > 0 ? minimum : 0;
    if (next > maximum)
        next = maximum;
    if (current == next)
        return false;
    current = next;
    return true;
}

void EEPROM::writePreheatProfiles() {
    using Profile = PreheatProfiles::Profile;
    const auto a = Printer::preheatProfiles.get(Profile::A);
    const auto b = Printer::preheatProfiles.get(Profile::B);
    HAL::eprSetInt16(EPR_RF_PREHEAT_A_BED, a.bed);
    HAL::eprSetInt16(EPR_RF_PREHEAT_A_HOTEND, a.hotend);
    HAL::eprSetInt16(EPR_RF_PREHEAT_B_BED, b.bed);
    HAL::eprSetInt16(EPR_RF_PREHEAT_B_HOTEND, b.hotend);
    HAL::eprSetInt32(EPR_RF_PREHEAT_PROFILE_TAG, preheatProfileTag);
}

bool EEPROM::readPreheatProfiles() {
    using Profile = PreheatProfiles::Profile;
    bool changed;
    if (HAL::eprGetInt32(EPR_RF_PREHEAT_PROFILE_TAG) == preheatProfileTag) {
        const bool a = Printer::preheatProfiles.readStored(Profile::A, {
            HAL::eprGetInt16(EPR_RF_PREHEAT_A_BED), HAL::eprGetInt16(EPR_RF_PREHEAT_A_HOTEND)});
        const bool b = Printer::preheatProfiles.readStored(Profile::B, {
            HAL::eprGetInt16(EPR_RF_PREHEAT_B_BED), HAL::eprGetInt16(EPR_RF_PREHEAT_B_HOTEND)});
        changed = a || b;
    } else {
        Printer::preheatProfiles.restoreDefaults();
        changed = true;
    }
    if (changed)
        writePreheatProfiles();
    return changed;
}

void EEPROM::storePreheatProfiles() {
    writePreheatProfiles();
    updateChecksum();
}

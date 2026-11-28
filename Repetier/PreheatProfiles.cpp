#include "Printer.h"
#include "PreheatProfiles.h"

namespace {
int16_t bedProfileMaximum() {
    const int16_t cap=heatedBedController.maximumTargetC();
    return cap<UI_SET_MAX_HEATED_BED_TEMP?cap:UI_SET_MAX_HEATED_BED_TEMP;
}
bool validTemperature(int16_t temperature, int16_t minimum, int16_t maximum) {
    return temperature == 0 || (temperature >= minimum && temperature <= maximum);
}
}

bool PreheatProfiles::readStored(Profile profile, Temperatures temperatures) {
    // Scalar defaults avoid materializing both immutable profile aggregates in SRAM.
    const bool profileA = profile == Profile::A;
    const int16_t factoryBed = profileA ? PREHEAT_A_BED_C : PREHEAT_B_BED_C;
    const int16_t factoryHotend = profileA ? PREHEAT_A_HOTEND_C : PREHEAT_B_HOTEND_C;
    auto& target = profiles_[static_cast<uint8_t>(profile)];
    // A lower/disabled live bed limit must not erase a stored preference.
    const int16_t bedMax=UI_SET_MAX_HEATED_BED_TEMP<140?UI_SET_MAX_HEATED_BED_TEMP:140;
    const int16_t safeDefault=validTemperature(factoryBed,UI_SET_MIN_HEATED_BED_TEMP,bedMax)?factoryBed:0;
    target.bed = validTemperature(temperatures.bed, UI_SET_MIN_HEATED_BED_TEMP, bedMax)
        ? temperatures.bed : safeDefault;
    target.hotend = validTemperature(temperatures.hotend, UI_SET_MIN_EXTRUDER_TEMP, UI_SET_MAX_EXTRUDER_TEMP)
        ? temperatures.hotend : factoryHotend;
    return target.bed != temperatures.bed || target.hotend != temperatures.hotend;
}

bool PreheatProfiles::adjust(Profile profile, Heater heater, int8_t increment) {
    auto& settings = profiles_[static_cast<uint8_t>(profile)];
    const bool bed = heater == Heater::Bed;
    int16_t& current = bed ? settings.bed : settings.hotend;
    const int16_t minimum = bed ? UI_SET_MIN_HEATED_BED_TEMP : UI_SET_MIN_EXTRUDER_TEMP;
    const int16_t maximum = bed ? bedProfileMaximum() : UI_SET_MAX_EXTRUDER_TEMP;
    if(maximum<minimum)
        return false; // disabled heater: retain preference, do not persist zero
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

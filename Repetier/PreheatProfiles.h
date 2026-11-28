#ifndef PREHEAT_PROFILES_H
#define PREHEAT_PROFILES_H

#include <stdint.h>

class PreheatProfiles {
public:
    enum class Profile : uint8_t { A, B };
    enum class Heater : uint8_t { Bed, Hotend };

    struct Temperatures {
        int16_t bed;
        int16_t hotend;
    };

    constexpr PreheatProfiles()
        : beds_{PREHEAT_A_BED_C, PREHEAT_B_BED_C},
          hotends_{PREHEAT_A_HOTEND_C, PREHEAT_B_HOTEND_C} {}

    Temperatures get(Profile profile) const {
        const uint8_t i=static_cast<uint8_t>(profile);
        return {beds_[i], hotends_[i]};
    }
    void restoreDefaults() {
        beds_[0]=PREHEAT_A_BED_C; beds_[1]=PREHEAT_B_BED_C;
        hotends_[0]=PREHEAT_A_HOTEND_C; hotends_[1]=PREHEAT_B_HOTEND_C;
    }
    bool readStored(Profile profile, Temperatures temperatures);
    bool adjust(Profile profile, Heater heater, int8_t increment);

private:
    // All stored bed values are zero or validated nonnegative temperatures;
    // readStored caps at 140 C, adjust at the configured/live bed limit.
    // Keep the public int16_t and EEPROM/protocol field widths unchanged.
    // Retain full-width storage for a custom configuration outside the
    // supported profiles' byte range instead of imposing a new limit.
#if PREHEAT_A_BED_C >= 0 && PREHEAT_A_BED_C <= 255 && \
    PREHEAT_B_BED_C >= 0 && PREHEAT_B_BED_C <= 255 && \
    UI_SET_MIN_HEATED_BED_TEMP >= 0 && UI_SET_MAX_HEATED_BED_TEMP <= 255
    using BedStorage = uint8_t;
#else
    using BedStorage = int16_t;
#endif
    BedStorage beds_[2];
    int16_t hotends_[2];
};

#endif

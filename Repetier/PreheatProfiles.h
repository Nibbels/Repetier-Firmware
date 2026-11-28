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
        : profiles_{defaults(Profile::A), defaults(Profile::B)} {}

    Temperatures get(Profile profile) const { return profiles_[static_cast<uint8_t>(profile)]; }
    void restoreDefaults() {
        profiles_[0] = defaults(Profile::A);
        profiles_[1] = defaults(Profile::B);
    }
    bool readStored(Profile profile, Temperatures temperatures);
    bool adjust(Profile profile, Heater heater, int8_t increment);

private:
    static constexpr Temperatures defaults(Profile profile) {
        return profile == Profile::A ? Temperatures{PREHEAT_A_BED_C, PREHEAT_A_HOTEND_C}
                                     : Temperatures{PREHEAT_B_BED_C, PREHEAT_B_HOTEND_C};
    }

    Temperatures profiles_[2];
};

#endif

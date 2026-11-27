// Independent measured-temperature ceiling and target admission for the bed.
// No EEPROM access here. Only the settings owner may publish a verified
// commissioned cutoff; invalid/unconfirmed settings disable target admission.
#ifndef RF_BED_TEMPERATURE_LIMIT_H
#define RF_BED_TEMPERATURE_LIMIT_H
#include <stdint.h>
class BedTemperatureLimit {
public:
    enum : uint8_t { AbsoluteC=150, DefaultC=120, MinimumC=40, MarginC=10,
                     MaximumStoredTargetC=AbsoluteC-MarginC };
    static constexpr bool validSetting(uint8_t value) {
        return !value || (value>=MinimumC && value<=AbsoluteC);
    }
    // Existing candidate uses DefaultC until the schema cutover. At boot
    // of the new schema the settings owner explicitly publishes 0 first.
    constexpr explicit BedTemperatureLimit(uint8_t initial=0)
        : commissioned_(validSetting(initial)?initial:0),
          monitor_(initial && validSetting(initial)?initial:uint8_t(DefaultC)) {}
    uint8_t commissioned() const {return commissioned_;}
    uint8_t monitor() const {return monitor_;}
    uint8_t maximumTarget() const {return commissioned_?commissioned_-MarginC:0;}
    bool publish(uint8_t value) {
        if(!validSetting(value)) {commissioned_=0;return false;}
        commissioned_=value;
        // Loss of authorization never increases a previously known lower
        // monitoring threshold. A cold boot with no record uses DefaultC
        // for monitoring ONLY, never for permitting heat.
        if(value) monitor_=value;
        return true;
    }
private:
    uint8_t commissioned_,monitor_;
};
#endif

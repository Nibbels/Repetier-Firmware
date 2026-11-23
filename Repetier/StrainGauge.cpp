#include "Repetier.h"
#include <Wire.h>

namespace {
constexpr uint8_t continuous16BitGain1 = 0x0c;
}

void StrainGauge::begin() const {
    Wire.setWireTimeout(3000, true);
    Wire.beginTransmission(I2C_ADDRESS_STRAIN_GAUGE);
    Wire.write(continuous16BitGain1);
    Wire.endTransmission();
}

bool StrainGauge::readDigits(int16_t& digits) const {
    if (Wire.requestFrom(uint8_t(I2C_ADDRESS_STRAIN_GAUGE), uint8_t(3)) != 3)
        return false;

    const uint16_t high = Wire.read();
    const uint16_t low = Wire.read();
    const uint8_t configuration = Wire.read();
    // ST/BSY reads high in continuous mode.
    if ((configuration & 0x7f) != continuous16BitGain1)
        return false;

    digits = static_cast<int16_t>((high << 8) | low);
    // Leave values near the ADC rails in raw digits.
    if (zeroingEnabled_ && -27768 < digits && digits < 27767)
        digits -= zeroOffset_;
    return true;
}

bool StrainGauge::setZeroFromAverage(int16_t digits) {
    if (digits <= -5000 || digits >= 5000)
        return false;
    zeroOffset_ = digits;
    return true;
}

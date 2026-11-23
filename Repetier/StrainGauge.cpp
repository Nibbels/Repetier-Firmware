#include "Repetier.h"
#include <Wire.h>

void StrainGauge::begin() const {
    Wire.beginTransmission(I2C_ADDRESS_STRAIN_GAUGE);
    Wire.write(0x8c); // Single conversion, 16 bits, gain 1.
    Wire.endTransmission();
}

int16_t StrainGauge::readDigits() const {
    Wire.beginTransmission(I2C_ADDRESS_STRAIN_GAUGE);
    Wire.requestFrom(uint8_t(I2C_ADDRESS_STRAIN_GAUGE), uint8_t(3));

    const uint16_t high = Wire.read();
    const uint16_t low = Wire.read();
    const uint16_t value = (high << 8) + low;
    Wire.read(); // ADC status byte.
    Wire.endTransmission();

    int16_t digits = static_cast<int16_t>(value);
    // Keep the raw reading near the ADC limits to avoid overflow.
    if (zeroingEnabled_ && -27768 < digits && digits < 27767)
        digits -= zeroOffset_;
    return digits;
}

bool StrainGauge::setZeroFromAverage(int16_t digits) {
    if (digits <= -5000 || digits >= 5000)
        return false;
    zeroOffset_ = digits;
    return true;
}

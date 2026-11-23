#include "Repetier.h"
#include "TwiTransfer.h"

namespace {
constexpr uint8_t continuous16BitGain1 = 0x0c;
}

void StrainGauge::begin() const {
    TwiTransfer transfer;
    if (transfer.write(I2C_ADDRESS_STRAIN_GAUGE, &continuous16BitGain1, 1, HAL::timeInMilliseconds()))
        transfer.wait();
}

bool StrainGauge::readDigits(int16_t& digits) const {
    TwiTransfer transfer;
    if (!transfer.read(I2C_ADDRESS_STRAIN_GAUGE, 3, HAL::timeInMilliseconds()) || !transfer.wait())
        return false;

    const uint16_t high = transfer.byteAt(0);
    const uint16_t low = transfer.byteAt(1);
    const uint8_t configuration = transfer.byteAt(2);
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

#ifndef STRAIN_GAUGE_H
#define STRAIN_GAUGE_H

#include <stdint.h>

class StrainGauge {
public:
    void begin() const;
    int16_t readDigits() const;

    bool zeroingEnabled() const { return zeroingEnabled_; }
    void setZeroingEnabled(bool enabled) { zeroingEnabled_ = enabled; }
    int16_t zeroOffset() const { return zeroOffset_; }
    int16_t effectiveZeroOffset() const { return zeroingEnabled_ ? zeroOffset_ : 0; }

    void clearZeroOffset() { zeroOffset_ = 0; }
    bool setZeroFromAverage(int16_t digits);

private:
    int16_t zeroOffset_ = 0;
    bool zeroingEnabled_ = true;
};

#endif

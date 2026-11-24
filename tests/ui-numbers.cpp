#include <cassert>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

constexpr uint8_t MAX_COLS = 28;
#define PROGMEM
float pgm_read_float(const float* value) { return *value; }

struct UIDisplay {
    uint8_t col = 0;
    char printCols[MAX_COLS + 1] = {};
    void addInt(int16_t value, uint8_t digits, char fillChar = ' ');
    void addLong(int32_t value, char digits);
    void addFloat(float number, char fixdigits, uint8_t digits);
};

#include "ui-numbers-firmware.inc"

int main() {
    for (int32_t value = INT16_MIN; value <= INT16_MAX; ++value) {
        UIDisplay display;
        display.addInt(value, 0);
        assert(display.printCols == std::to_string(value));
    }
    for (const int32_t value : {INT32_MIN, -123456789, -1, 0, 1, 123456789, INT32_MAX}) {
        UIDisplay display;
        display.addLong(value, 0);
        assert(display.printCols == std::to_string(value));
    }

    UIDisplay display;
    display.addInt(-7, 4, '0');
    display.addLong(-42, 5);
    assert(std::strcmp(display.printCols, "-007-  42") == 0);

    display = {};
    std::memset(display.printCols, 'x', MAX_COLS - 2);
    display.col = MAX_COLS - 2;
    display.addLong(INT32_MIN, 0);
    assert(display.col == MAX_COLS);
    assert(std::strcmp(display.printCols + MAX_COLS - 2, "-2") == 0);

    display = {};
    display.addFloat(-12.25f, 2, 2);
    assert(std::strcmp(display.printCols, "-12.25") == 0);
    for (const float value : {2147483648.0f, std::numeric_limits<float>::infinity(),
                              std::numeric_limits<float>::quiet_NaN()}) {
        display = {};
        display.addFloat(value, 0, 2);
        assert(std::strcmp(display.printCols, "?") == 0);
    }
}

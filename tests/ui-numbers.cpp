#include <cassert>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

constexpr uint8_t MAX_COLS = 28;
#define PROGMEM
float pgm_read_float(const float* value) { return *value; }

using PGM_P = const char*;
#define pgm_read_byte(address) (*(address))
enum { CHAR_SELECTED = 3, PRINTER_FLAG3_X_HOMED = 1, PRINTER_FLAG3_Y_HOMED = 2 };

namespace Printer {
bool uiError = false;
int flag3 = PRINTER_FLAG3_X_HOMED;
bool isUIErrorMessage() { return uiError; }
void setUIErrorMessage(bool error) { uiError = error; }
struct { bool isHomed() const { return false; } } zMotion;
}

struct UIDisplay {
    uint8_t col = 0;
    char printCols[MAX_COLS + 1] = {};
    char statusMsg[MAX_COLS + 1] = {};
    bool statusIsTemplate_ = false, locked = false;
    char confirmation_ = 0;
    const void* messageLine1 = nullptr;
    const void* messageLine2 = nullptr;
    const void* messageLine3 = nullptr;
    const void* messageLine4 = nullptr;
    void parse(char* text, bool ram);
    void setStatusP(PGM_P text, bool error = false);
    void setStatus(char* text, bool error = false, bool force = false);
    void addInt(int16_t value, uint8_t digits, char fillChar = ' ');
    void addLong(int32_t value, char digits);
    void addFloat(float number, char fixdigits, uint8_t digits);
};

#include "ui-numbers-firmware.inc"

std::string statusLine(UIDisplay& display) {
    char format[] = "%os";
    display.col = 0;
    display.parse(format, false);
    return display.printCols;
}

void testStatusText() {
    UIDisplay display;
    char hostText[] = "Ready 100%";
    display.setStatus(hostText);
    assert(statusLine(display) == "Ready 100%");
    char hostFields[] = "%os and %m1";
    display.setStatus(hostFields);
    assert(statusLine(display) == hostFields);
    display.setStatusP("Home X %hx");
    assert(statusLine(display) == "Home X *");
    display.setStatusP("Hold %");
    assert(statusLine(display) == "Hold %");
    display.setStatusP("Hold %x");
    assert(statusLine(display) == "Hold %x");

    std::string clipped(UI_COLS - 1, 'a');
    display.setStatusP((clipped + "%hx").c_str());
    assert(statusLine(display) == clipped + "%");

    char messagePage[] = "%m1";
    display.messageLine1 = "State %os";
    display.setStatusP("Home X %hx");
    display.col = 0;
    display.parse(messagePage, false);
    assert(std::string(display.printCols) == "State Home X *");

    char partial[] = "%";
    display.col = 0;
    display.parse(partial, false);
    assert(std::string(display.printCols) == "%");
}

int main() {
    testStatusText();
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
